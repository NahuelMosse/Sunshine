/**
 * @file src/adaptive_bitrate.cpp
 * @brief Definitions for the per-session adaptive bitrate controller.
 */
// standard includes
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

// local includes
#include "adaptive_bitrate.h"
#include "config.h"
#include "logging.h"
#include "platform/common.h"

// lib includes
#include <moonlight-common-c/src/Limelight-internal.h>

using namespace std::literals;

namespace adaptive_bitrate {

#ifdef LI_FF_ADAPTIVE_BITRATE
  static_assert(LI_FF_ADAPTIVE_BITRATE == host_feature_bit, "the host bit must match moonlight-common-c");
#endif
  namespace {
    // Both client reports are parsed by hand so this file does not depend on the struct
    // packing of the C headers. SS_FRAME_FEC_STATUS is big-endian and 21 bytes long; the loss
    // stats message is little-endian and 32 bytes long.
    constexpr std::size_t fec_status_size = 21;
    constexpr std::size_t loss_stats_size = 20;  // the fields read here; the message is longer
    // Frame Stats: the count of entries in the first byte, three reserved, and from byte 4 on
    // that many big-endian entries of 18 bytes —frame index, client time of its first and last
    // packet in microseconds, bytes and packets received—, in the 80 bytes the message reserves.
    constexpr std::size_t frame_stats_header = 4;
    constexpr std::size_t frame_stats_entry = 18;
    // With an encoder that keeps its bitrate, the lowest frame rate the budget may take it to: ten
    // frames a second at sixty, a sixth of the bitrate. Below that the image stays and the queue grows.
    constexpr double kMaxRateDivider = 6;
    // Data and parity shards of one Reed-Solomon block
    constexpr int kMaxShards = 255;
    // The lowest first bitrate the encoder is given, in kbit/s, when the starting budget allows it
    constexpr double kMinInitialKbps = 500;
    // Frames that can be in flight before fewer are sent: the encoder's references reach this far back
    constexpr int kFramesInFlight = 12;
    // Seconds over which the frames the encoder produces are counted
    constexpr double kRateWindowS = 0.5;
    // The shortest time in which a queue over the repair reach is drained, in seconds
    constexpr double kMinDrainS = 0.1;

    std::uint32_t be32(const unsigned char *p) {
      return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | std::uint32_t(p[3]);
    }

    std::uint16_t be16(const unsigned char *p) {
      return std::uint16_t((p[0] << 8) | p[1]);
    }

    std::uint32_t le32(const unsigned char *p) {
      return (std::uint32_t(p[3]) << 24) | (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[1]) << 8) | std::uint32_t(p[0]);
    }

    std::int64_t to_us(std::chrono::steady_clock::time_point t) {
      return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
    }

    // The controller's tuning table, with the facts of this session filled in. Everything
    // else keeps the values the test bench validated.
    abr::Params make_params(int bitrate_kbps, int framerate, int payload, int wire, double audio_bps, int min_parity) {
      abr::Params p;
      p.fec_min_packets = std::max(0, min_parity);
      p.bitrate_max = bitrate_kbps * 1000.0;
      p.bitrate_min = std::min(p.bitrate_min, p.bitrate_max);
      p.fps = framerate;
      p.mtu = payload;
      p.header_bytes = wire - payload;
      p.audio_bps = audio_bps;
      return p;
    }
  }  // namespace

  mode_e mode_from_string(std::string_view value) {
    if (value == "fec"sv) {
      return mode_e::fec;
    }
    if (value == "full"sv) {
      return mode_e::full;
    }
    return mode_e::disabled;
  }

  int frame_divider_for(double in_flight_s, double framerate, int frames_in_flight, int repair_reach) {
    if (frames_in_flight <= 0) {
      return 1;
    }
    const int limit = std::max(frames_in_flight, repair_reach);
    return std::clamp(static_cast<int>(std::ceil(in_flight_s * framerate / limit)), 1, 4);
  }

  double encoded_rate_for(double measured, double nominal) {
    return measured > 0 ? std::min(measured, nominal) : nominal;
  }

  double payload_rate(double wire_rate, int payload, int wire) {
    return wire > 0 ? wire_rate * payload / wire : wire_rate;
  }

  std::optional<std::uint32_t> client_clock_ms(std::string_view payload) {
    // A length of 4 and the client's monotonic clock in milliseconds, little-endian. Moonlight
    // writes the length in 32 bits, and in 16 bits in older moonlight-common-c.
    const auto *p = reinterpret_cast<const unsigned char *>(payload.data());
    const bool long_length = payload.size() >= 8 && le32(p) == 4;
    const std::uint32_t client_ms = le32(p + (long_length ? 4 : 2));
    // With the 16-bit length the 8-byte ping carries a zero clock in its first two bytes after the 16-bit
    // length and two bytes the client never initialized: read with the 32-bit length, a clock
    // whose low half is zero. A zero clock is what clients send to other hosts.
    if (client_ms == 0 || (long_length && payload.size() == 8 && (client_ms & 0xFFFF) == 0)) {
      return std::nullopt;
    }
    return client_ms;
  }

  int ceiling_kbps(int vqos_kbps, std::int64_t configured_kbps) {
    // x-nv-vqos carries 0.8 times what the user set, capped at 100 000 kbit/s
    return configured_kbps > 0 ? static_cast<int>(std::min<std::int64_t>(configured_kbps, std::numeric_limits<int>::max())) : vqos_kbps;
  }

  std::chrono::milliseconds control_wait(const session_t *session) {
    return session && session->active() ? std::chrono::milliseconds(5) : std::chrono::milliseconds(150);
  }

  bool announces_signals(int ml_feature_flags) {
    return (ml_feature_flags & ML_FF_ADAPTIVE_SIGNALS) && (ml_feature_flags & ML_FF_FEC_STATUS);
  }

  bool session_wanted(mode_e mode, int ml_feature_flags) {
    return mode != mode_e::disabled && announces_signals(ml_feature_flags);
  }

  std::uint32_t host_feature_flags(mode_e mode) {
    return mode != mode_e::disabled ? host_feature_bit : 0;
  }

  session_t::session_t(mode_e mode, int bitrate_kbps, int framerate, int packetsize, int wire_overhead, double audio_bps, int min_parity, bool announced):
      mode {mode},
      configured_kbps {bitrate_kbps},
      framerate {std::max(1, framerate)},
      payload_per_packet {std::max(1, packetsize)},
      wire_per_packet {std::max(1, packetsize) + std::max(0, wire_overhead)},
      params {make_params(bitrate_kbps, this->framerate, payload_per_packet, wire_per_packet, audio_bps, min_parity)},
      active_signals {announced && mode != mode_e::disabled},
      control {params},
      published {std::make_shared<const decision_t>()} {
    draft.block_params = std::make_shared<const abr::Params>(params);
    draft.repair_params = repair_params_of(params);
    control.reset(params.bitrate_max);
    meter.audio(audio_bps);
    meter.configure(params);

    const double start_bps = params.bitrate_max * params.startup_fraction;
    const double image_bps = start_bps * payload_per_packet / wire_per_packet - audio_bps;
    // A floor for the encoder's first bitrate, never above the starting budget itself
    const double floor_kbps = std::min(kMinInitialKbps, start_bps / 1000.0);
    initial_kbps = encoder_kbps(static_cast<int>(std::max(floor_kbps, image_bps / 1000.0)));
    if (active_signals) {
      activate();
    }
    publish();
  }

  std::shared_ptr<const abr::Params> session_t::repair_params_of(const abr::Params &base) {
    auto repair = std::make_shared<abr::Params>(base);
    repair->parity_block_failure = base.parity_repair_failure;
    repair->fec_max = std::max(base.fec_max, base.fec_max_repair);
    // The repair block keeps its cap whatever the frame rate: the cap that grows with it is the ordinary block's
    repair->fec_max_fps = 0;
    return repair;
  }

  int session_t::requested_kbps() const {
    return encoder_kbps(static_cast<int>(target_kbps * drain_keep / bitrate_keep));
  }

  void session_t::set_budget(double budget, double capacity, bool with_pacing) {
    draft.budget = budget;
    draft.pay_multiple = stream::pay_multiple(budget, params.bitrate_max, abr::tun::kPaceMax);
    if (with_pacing) {
      draft.pacing = stream::pace_multiplier(abr::tun::kPaceOverPipe, budget, capacity, abr::tun::kPaceMax, abr::tun::kPaceMax, params.bitrate_max) * budget;
    }
  }

  void session_t::publish() {
    // Only full mode moves the encoder's bitrate; in the other modes a reopened encoder opens at the client's
    const int requested = mode == mode_e::full ? requested_kbps() : 0;
    draft.open_kbps = mode != mode_e::full || fixed_kbps > 0 ? 0 : (requested > 0 ? requested : initial_kbps);
    if (draft == *published.load()) {
      return;
    }
    published.store(std::make_shared<const decision_t>(draft));
  }

  int session_t::encoder_kbps(int kbps) {
    return config::video.max_bitrate > 0 ? std::min(kbps, config::video.max_bitrate) : kbps;
  }

  void session_t::activate() {
    // From the first frame the stream runs on what the controller starts from, not on what
    // the client asked for: the encoder on the image part of the starting budget, and the
    // pacing on its multiple. Otherwise the first frames go out at the client's bitrate and at
    // line rate, and against a path someone else already fills that is a broken first second.
    const double start_bps = params.bitrate_max * params.startup_fraction;
    if (mode == mode_e::full) {
      target_kbps = initial_kbps;
    }
    set_budget(start_bps, 0.0, mode == mode_e::full);
    BOOST_LOG(info) << "Adaptive bitrate: the client announced its signals, the controller acts from the start"sv;
  }

  void session_t::frame_sent(std::uint32_t frame_index, std::chrono::steady_clock::time_point first, std::chrono::steady_clock::time_point last, int packets, int data_packets, double rtp_bytes, double wire_bytes) {
    abr::gamestream::SentFrame frame;
    frame.index = frame_index;
    frame.t_first_us = to_us(first);
    frame.t_last_us = to_us(last);
    frame.packets = packets;
    frame.data_packets = data_packets;
    frame.rtp_bytes = rtp_bytes;
    frame.wire_bytes = wire_bytes;

    std::lock_guard lg {lock};
    meter.sent(frame);
    last_frame_index = frame_index;
    // Frames per second actually produced, over windows of at least half a second
    if (measured.window_frames < 0) {
      measured.window_start = first;
      measured.window_frames = 0;
    } else {
      ++measured.window_frames;
      const double span_s = std::chrono::duration<double>(first - measured.window_start).count();
      if (span_s >= kRateWindowS) {
        measured.fps = measured.window_frames / span_s;
        measured.window_start = first;
        measured.window_frames = 0;
      }
    }
  }

  bool session_t::idr_requested(std::chrono::steady_clock::time_point now) {
    if ((mode != mode_e::fec && mode != mode_e::full) || !active()) {
      return true;
    }
    std::lock_guard lg {lock};
    // A client that cannot invalidate references asks for an IDR frame for every frame it finds
    // broken while it waits, about twenty a second. Each one answered is a frame several times the
    // size of the others, into the path that just narrowed: they overflowed it themselves and every
    // loss brought more requests. The one already sent is on its way; another is sent only once it
    // would have arrived, a round trip later. If it was lost, the client asks once more and then
    // drops every frame for two seconds: what arrives inside the window is held, not dropped.
    if (last_idr_at != std::chrono::steady_clock::time_point {} && now - last_idr_at < idr_window()) {
      idr_held = true;
      return false;
    }
    idr_answered(now);
    return true;
  }

  std::chrono::steady_clock::duration session_t::idr_window() const {
    return std::max<std::chrono::steady_clock::duration>(std::chrono::milliseconds(300), std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(2.0 * last_rtt_s)));
  }

  void session_t::idr_answered(std::chrono::steady_clock::time_point now) {
    last_idr_at = now;
    idr_held = false;
    meter.idr_request(last_frame_index + 1);
    // And at the plain frame rate: one frame in several, each paid for several times over, made
    // the IDR frame that many times larger
    if (divider > 1) {
      divider = 1;
      publish_frames();
    }
  }

  bool session_t::fec_status(std::string_view payload) {
    if (payload.size() < fec_status_size) {
      return false;
    }
    // frameIndex, highestReceivedSequenceNumber, nextContiguousSequenceNumber,
    // missingPacketsBeforeHighestReceived, totalDataPackets, totalParityPackets,
    // receivedDataPackets, receivedParityPackets, fecPercentage, multiFecBlockIndex,
    // multiFecBlockCount
    auto data = reinterpret_cast<const unsigned char *>(payload.data());
    // GameStream sends a frame in at most four FEC blocks, and the index names one of them
    if (data[19] >= data[20] || data[20] > 4) {
      return false;
    }
    abr::gamestream::FecStatus status;
    status.index = be32(data);
    status.missing = be16(data + 8);
    status.data_total = be16(data + 10);
    status.parity_total = be16(data + 12);
    status.data_received = be16(data + 14);
    status.parity_received = be16(data + 16);
    status.block = data[19];

    std::lock_guard lg {lock};
    meter.fec_status(status);
    return true;
  }

  bool session_t::frame_stats(std::chrono::steady_clock::time_point now, std::string_view payload) {
    if (payload.size() < frame_stats_header) {
      return false;
    }
    auto data = reinterpret_cast<const unsigned char *>(payload.data());
    const std::size_t count = data[0];
    if (payload.size() < frame_stats_header + count * frame_stats_entry) {
      return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
      const unsigned char *e = data + frame_stats_header + i * frame_stats_entry;
      abr::gamestream::ReceivedTrain arrival;
      arrival.index = be32(e);
      arrival.t_first_us = be32(e + 4);
      arrival.t_last_us = be32(e + 8);
      arrival.bytes = be32(e + 12);
      arrival.packets = be16(e + 16);
      double queue_seen = 0;
      double growth = 0;
      double sample_ms = 0;
      bool queued;
      {
        std::lock_guard lg {lock};
        queued = meter.train(arrival, queue_seen, growth, &sample_ms);
      }
      queue_sample(now, queued, queue_seen, growth, sample_ms);
    }
    return true;
  }

  bool session_t::loss_stats(std::chrono::steady_clock::time_point now, std::string_view payload) {
    if (payload.size() < loss_stats_size) {
      return false;
    }
    // lossCountSinceLastReport, reportInterval, a constant 1000, lastGoodFrame (64 bits)
    auto data = reinterpret_cast<const unsigned char *>(payload.data());
    abr::gamestream::LossSummary stats;
    stats.lost = le32(data);
    stats.last_good = le32(data + 12);

    std::lock_guard lg {lock};
    meter.summary(to_us(now), stats);
    frames_lost += stats.lost;
    return true;
  }

  bool session_t::client_ping(std::chrono::steady_clock::time_point now, std::string_view payload) {
    if (payload.size() < 6) {
      return false;
    }
    if (const auto client_ms = client_clock_ms(payload)) {
      std::lock_guard lg {lock};
      meter.client_ping(to_us(now), *client_ms);
    }
    return true;
  }

  void session_t::queue_sample(std::chrono::steady_clock::time_point now, bool queued, double queue_seen, double growth, double sample_ms) {
    if (mode == mode_e::disabled || !active()) {
      return;
    }
    react_to_queue(now, queued, queue_seen, growth, sample_ms);
    publish();
  }

  void session_t::react_to_queue(std::chrono::steady_clock::time_point now, bool queued, double queue_seen, double growth, double sample_ms) {
    double now_budget = 0;
    double accepted_ms = sample_ms;
    bool cut = false;
    bool undone = false;
    // The frames in flight, within what the encoder can still predict from (take_frame_divider).
    // From the sample the meter stored, as a round trip, and only if it took one: a frame it
    // discarded says nothing new about the video's path.
    if (accepted_ms > 0 && (mode == mode_e::fec || mode == mode_e::full)) {
      // The round trip and the time frames wait in the host before leaving (host_delay)
      double host_delay_s;
      {
        std::lock_guard lg {lock};
        host_delay_s = measured.host_delay_s;
      }
      frames_in_flight_limit(now, accepted_ms / 1000.0 + host_delay_s);
    }
    if (!queued) {
      return;
    }
    {
      std::lock_guard lg {lock};
      queue = queue_seen;
      // The sample right after an alarm may show the queue back at its floor: then the step was a
      // delayed acknowledgement, not a narrower path, and everything the alarm changed goes back
      if (have_measurement && (mode == mode_e::fec || mode == mode_e::full) && control.retract_alarm(to_us(now) / 1e6, queue_seen)) {
        undone = true;
        now_budget = control.budget();
        draft.min_parity = control.min_parity();
      } else if (have_measurement && (mode == mode_e::fec || mode == mode_e::full) && control.alarm(to_us(now) / 1e6, queue_seen, growth, last_measurement.received_rate)) {
        cut = true;
        now_budget = control.budget();
        draft.min_parity = control.min_parity();
      }
    }
    if (!cut && !undone) {
      return;
    }
    const double before = draft.budget;
    set_budget(now_budget, capacity, true);
    if (before > 0) {
      target_kbps = encoder_kbps(static_cast<int>(target_kbps * now_budget / before));
    }
    bitrate_keep = frames_kept();
    BOOST_LOG(debug) << (undone ? "Adaptive bitrate alarm undone: budget="sv : "Adaptive bitrate alarm: budget="sv) << static_cast<int>(now_budget / 1000.0)
                     << " before="sv << static_cast<int>(before / 1000.0)
                     << " queue="sv << std::lround(queue_seen * 1000.0)
                     << " growth="sv << std::lround(growth * 1000.0);
  }

  void session_t::frames_in_flight_limit(std::chrono::steady_clock::time_point now, double in_flight_s) {
    // The frames are counted at the rate the encoder produces them, and with long-term references
    // the encoder reaches further back than the short-term ones
    double measured_fps;
    {
      std::lock_guard lg {lock};
      measured_fps = measured.fps;
    }
    const double rate = encoded_rate_for(measured_fps, framerate);
    const int reach = repair_reach;
    const int wanted = frame_divider_for(in_flight_s, rate, kFramesInFlight, reach);
    int skip = 1;
    if (wanted <= 1) {
      draining = false;
      drain_keep = 1.0;
    } else if (mode != mode_e::full || fixed_kbps > 0) {
      // Nothing to lower: the frame rate is the only lever
      skip = wanted;
    } else {
      // The bitrate comes first: the encoder keeps the share of the pipe that drains what is over
      // the reach within a window. Frames are skipped only where that cannot be done: below the
      // floor, or once the window has run out with the frames still over the reach
      const double window = std::max(2.0 * last_rtt_s, kMinDrainS);
      const double excess = in_flight_s - std::max(kFramesInFlight, reach) / rate;
      const double want = 1.0 - excess / window;
      const double budget_now = draft.budget;
      const double floor = budget_now > params.bitrate_min ? params.bitrate_min / budget_now : 1.0;
      if (!draining) {
        draining = true;
        drain_since = now;
      }
      drain_keep = std::clamp(want, floor, 1.0);
      if (want < floor || std::chrono::duration<double>(now - drain_since).count() > window) {
        skip = wanted;
      }
    }
    if (skip != divider) {
      divider = skip;
      publish_frames();
    }
  }

  void session_t::invalidation_requested(std::chrono::steady_clock::time_point /* now */) {
    if ((mode != mode_e::fec && mode != mode_e::full) || !active()) {
      return;
    }
    {
      // Until the frame with the invalidated references arrives the client drops whole frames and
      // counts them lost, exactly as while it waits for an IDR frame: that is not loss in the path
      std::lock_guard lg {lock};
      meter.idr_request(last_frame_index + 1);
    }
  }

  bool session_t::enet_ping_due(std::chrono::steady_clock::time_point now) {
    if (mode == mode_e::disabled || !active() || now < next_enet_ping) {
      return false;
    }
    next_enet_ping = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(abr::session::kHostPingMs / 1000.0));
    return true;
  }

  void session_t::tick(std::chrono::steady_clock::time_point now, double rtt_s) {
    if (mode == mode_e::disabled || !active()) {
      return;
    }
    {
      std::lock_guard lg {lock};
      if (idr_held && now - last_idr_at >= idr_window()) {
        idr_answered(now);
        idr_wanted = true;
      }
    }
    if (now < next_step) {
      return;
    }
    const auto cycle = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(params.Tc));
    next_step = now + cycle;
    // The client's reports are handled on this same thread. When it stalls —a busy machine— what
    // was sent keeps being counted by the video thread while what arrived is not, and the reports
    // held up in the socket are then read all at once: what arrived looks first like a collapse of
    // the path and then like a burst. Neither is the network, so the controller waits until a
    // whole measurement window has passed since the stall.
    if (last_tick != std::chrono::steady_clock::time_point {} && now - last_tick > 3 * cycle) {
      hold_until = now + std::chrono::milliseconds(300);
    }
    last_tick = now;

    abr::Decision decision;
    std::uint64_t lost_so_far;
    {
      std::lock_guard lg {lock};
      lost_so_far = frames_lost;
      if (rtt_s > 0) {
        last_rtt_s = rtt_s;
      }
      meter.rtt(rtt_s);
      abr::Measurement measurement;
      if (meter.measure(to_us(now), measurement)) {
        last_measurement = measurement;
        have_measurement = true;
      }
      if (!have_measurement || now < hold_until) {
        return;
      }
      control.set_fps(framerate * frames_kept());
      decision = control.step(last_measurement);
    }

    // What the encoder is asked for: the image budget, scaled up by what the encoder has been
    // falling short of it
    target_kbps = encoder_kbps(static_cast<int>(decision.encoder_request / 1000.0));
    bitrate_keep = frames_kept();
    draft.target_fec = static_cast<int>(std::lround(decision.repair * 100.0));
    capacity = decision.C_est;
    set_budget(decision.budget, decision.C_est, true);
    // An encoder that keeps its bitrate gets the budget as the share of the frames it encodes, by
    // what it sends with every frame, learnt as an average: not by its nominal bitrate —encoders
    // fall short of it, libx264 by a third, and the image stayed at half the budget—, and not by
    // the most it sent lately either: any estimate above what it sends leaves it below seven
    // tenths of the budget, the controller reads that as an encoder with nothing to send and stops
    // raising the budget. Fewer frames at once; more, smoothly over a second
    if (fixed_kbps > 0 && mode == mode_e::full) {
      // data_rate counts what the packets cost on the wire; the image rate it is compared with is payload
      const double data_kbps = payload_rate(last_measurement.data_rate, payload_per_packet, wire_per_packet) / 1000.0;
      if (data_kbps > 0) {
        const double full = data_kbps / frames_kept();
        full_kbps = full_kbps > 0 ? full_kbps + 0.1 * (full - full_kbps) : full;
      }
      const double image_kbps = decision.video_rate / 1000.0;
      const double wanted = std::clamp(image_kbps / std::max(1.0, full_kbps > 0 ? full_kbps : fixed_kbps), 1.0 / kMaxRateDivider, 1.0);
      const double previous = rate_keep;
      rate_keep = wanted < rate_keep ? wanted : rate_keep + 0.1 * (wanted - rate_keep);
      if (std::abs(rate_keep - previous) > 0.01) {
        publish_frames();
      }
    }
    if (decision.parity_failure != block_failure) {
      block_failure = decision.parity_failure;
      auto block = std::make_shared<abr::Params>(params);
      if (block_failure > 0) {
        block->parity_block_failure = block_failure;
      }
      draft.block_params = std::move(block);
    }
    draft.stepped = true;
    draft.loss = decision.loss_est;
    draft.burst = decision.loss_burst;
    draft.min_parity = decision.min_parity;
    publish();

    // One line per controller step, in a fixed key=value layout so a session can be analysed
    // afterwards: rates in kbps, times in ms, fractions in percent.
    BOOST_LOG(verbose) << "Adaptive bitrate step: budget="sv << static_cast<int>(decision.budget / 1000.0)
                       << " image="sv << static_cast<int>(decision.video_rate / 1000.0)
                       << " encoder="sv << static_cast<int>(decision.encoder_request / 1000.0)
                       << " data="sv << static_cast<int>(last_measurement.data_rate / 1000.0)
                       << " fec="sv << std::lround(decision.repair * 100.0)
                       << " fec_sent="sv << std::lround(last_measurement.parity_sent * 100.0)
                       << " queue="sv << std::lround(decision.queue * 1000.0)
                       << " target_queue="sv << std::lround(decision.dref * 1000.0)
                       << " capacity="sv << static_cast<int>(decision.C_est / 1000.0)
                       << " train="sv << static_cast<int>(decision.train_bandwidth / 1000.0)
                       << " received="sv << static_cast<int>(last_measurement.received_rate / 1000.0)
                       << " sent="sv << static_cast<int>(last_measurement.sent_rate / 1000.0)
                       << " loss="sv << std::lround(last_measurement.loss * 1000.0) / 10.0
                       << " burst="sv << decision.loss_burst
                       << " rtt="sv << std::lround(last_measurement.rtt * 1000.0)
                       << " frames_lost="sv << lost_so_far
                       << " reason="sv << abr::name(decision.reason);

    if (now >= next_log) {
      next_log = now + 2s;
      const auto mode_name = mode == mode_e::fec ? "fec"sv :
                                                   "full"sv;
      BOOST_LOG(debug) << "Adaptive bitrate ["sv << mode_name << "]: budget "sv
                       << static_cast<int>(decision.budget / 1000.0) << " of "sv << configured_kbps
                       << " kbps, encoder "sv << static_cast<int>(decision.video_rate / 1000.0)
                       << " kbps, FEC "sv << std::lround(decision.repair * 100.0)
                       << "%, measured capacity "sv << static_cast<int>(decision.C_est / 1000.0)
                       << " kbps, queue "sv << static_cast<int>(decision.queue * 1000.0)
                       << " ms, loss "sv << std::lround(last_measurement.loss * 1000.0) / 10.0
                       << "%, received "sv << static_cast<int>(last_measurement.received_rate / 1000.0)
                       << " kbps ("sv << abr::name(decision.reason) << ')';
    }
  }

  bool session_t::malformed(message_e kind) {
    return !std::exchange(malformed_seen[static_cast<std::size_t>(kind)], true);
  }

  void session_t::encoder_bitrate_fixed(int kbps) {
    fixed_kbps = kbps;
    BOOST_LOG(info) << "Adaptive bitrate: the encoder keeps "sv << kbps << " kbps; the budget goes into the frame rate"sv;
    publish();
  }

  void session_t::encoder_reopened() {
    last_applied_kbps = 0;
  }

  int session_t::bitrate_to_apply(std::chrono::steady_clock::time_point now) {
    if (mode != mode_e::full || fixed_kbps > 0) {
      return 0;
    }
    // The encoder spreads its bitrate over the frame rate it was opened with: with a share of the
    // frames encoded, it has to be asked for that many times more to fill the same budget. The share
    // is the one at the last budget step, so the frames and the bitrate that makes up for them change
    // together with the budget and not ahead of it
    const int kbps = requested_kbps();
    if (kbps <= 0) {
      return 0;
    }
    // Reconfiguring an encoder is not free, and the controller moves a little every step.
    // Two percent is well below what the image quality can show.
    const bool moved = last_applied_kbps == 0 || std::abs(kbps - last_applied_kbps) * 50 > last_applied_kbps;
    const bool stale = now - last_applied_at > 1s;
    if (!moved && !stale) {
      return 0;
    }
    last_applied_kbps = kbps;
    last_applied_at = now;
    return kbps;
  }

  int session_t::fec_percentage(int configured) const {
    return fec_percentage(configured, *published.load());
  }

  int session_t::fec_percentage(int configured, const decision_t &decision) const {
    // FEC the user turned off stays off
    if ((mode != mode_e::fec && mode != mode_e::full) || !active() || configured <= 0) {
      return configured;
    }
    const int fec = decision.target_fec;
    return fec < 0 ? configured : std::clamp(fec, 1, 255);
  }

  int session_t::block_fec_percentage(int data_shards, bool repairs) {
    return block_fec_percentage(data_shards, repairs, *published.load());
  }

  int session_t::block_fec_percentage(int data_shards, bool repairs, const decision_t &decision) {
    if ((mode != mode_e::fec && mode != mode_e::full) || !active() || data_shards <= 0) {
      return -1;
    }
    // Before the first decision nothing has been measured, and the first frames go out on a
    // budget nobody verified: protect them as the controller does for its first seconds
    const double loss = decision.stepped ? decision.loss : abr::cal::kParityStartup;
    const double burst = decision.stepped ? decision.burst : 1.0;
    int min_parity = decision.stepped ? decision.min_parity : -1;
    // The frame that repairs a loss must not be lost: losing one that repairs an invalidation
    // invalidates again from it, and two or three in a row push the last good reference out of the
    // encoder's sixteen; losing an IDR frame makes the client ask for another, several times the
    // size of the others, into the same full queue. Its block gets the client's minimum at least and
    // a far stricter failure target, on the loss measured: making up for a client that does not
    // report loss, with a floor here and each request counted as loss, overflowed a path that had
    // just narrowed and fed the very cascade it was meant to stop.
    if (repairs) {
      min_parity = std::max(min_parity, params.fec_min_packets);
    }
    const abr::Params &block_params = repairs ? *decision.repair_params : *decision.block_params;
    int parity = abr::block_parity(data_shards, loss, burst, block_params, min_parity);
    if (data_shards >= kMaxShards) {
      return 0;
    }
    // A parity of 0 is the controller's answer for a block the path does not threaten: no FEC for it
    parity = std::clamp(parity, 0, kMaxShards - data_shards);
    if (parity == 0) {
      return 0;
    }
    // fec::encode rounds data_shards * percentage / 100 up, so this is the smallest percentage
    // that gives back `parity`: exactly that many while data_shards is at most 100, one more at
    // most above that. Rounding the percentage up as well would add a parity packet to most
    // blocks — two instead of one on a seven-packet frame. The percentage travels in 8 bits of
    // every packet's fecInfo, and the client derives the parity count from it: above 255 it
    // would read the block wrong.
    // A Reed-Solomon block holds kMaxShards shards at most, and fec::encode rounds up: the last
    // bound keeps data + parity within them.
    return std::min({255, (parity - 1) * 100 / data_shards + 1, (kMaxShards - data_shards) * 100 / data_shards});
  }

}  // namespace adaptive_bitrate
