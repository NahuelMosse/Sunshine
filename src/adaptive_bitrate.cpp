/**
 * @file src/adaptive_bitrate.cpp
 * @brief Definitions for the per-session adaptive bitrate controller.
 */
// standard includes
#include <algorithm>
#include <cmath>
#include <cstdlib>

// local includes
#include "adaptive_bitrate.h"
#include "logging.h"

using namespace std::literals;

namespace adaptive_bitrate {
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
      // The variant the bench validated with the GameStream signals. Without per-frame arrival
      // times there is no dispersion to measure, so what remains is the delay law with its
      // ceilings and the jumps on loss.
      p.variant = abr::Variant::train;
      p.bitrate_max = bitrate_kbps * 1000.0;
      p.fps = framerate;
      p.mtu = payload;
      p.header_bytes = wire - payload;
      p.audio_bps = audio_bps;
      return p;
    }
  }  // namespace

  mode_e mode_from_string(std::string_view value) {
    if (value == "shadow"sv) {
      return mode_e::shadow;
    }
    if (value == "fec"sv) {
      return mode_e::fec;
    }
    if (value == "full"sv) {
      return mode_e::full;
    }
    return mode_e::disabled;
  }

  session_t::session_t(mode_e mode, int bitrate_kbps, int framerate, int packetsize, int wire_overhead, double audio_bps, int min_parity):
      mode {mode},
      configured_kbps {bitrate_kbps},
      framerate {std::max(1, framerate)},
      payload_per_packet {std::max(1, packetsize)},
      wire_per_packet {std::max(1, packetsize) + std::max(0, wire_overhead)},
      params {make_params(bitrate_kbps, this->framerate, payload_per_packet, wire_per_packet, audio_bps, min_parity)},
      control {params} {
    control.reset(params.bitrate_max);
    meter.audio(audio_bps);
    meter.configure(params);

    const double start_bps = params.bitrate_max * params.startup_fraction;
    const double image_bps = start_bps * payload_per_packet / wire_per_packet - audio_bps;
    initial_kbps = std::max(500, static_cast<int>(image_bps / 1000.0));
  }

  void session_t::activate() {
    if (active_signals.exchange(true, std::memory_order_relaxed)) {
      return;
    }
    // Until the first step the stream runs on what the controller starts from, not on what
    // the client asked for: the encoder on the image part of the starting budget, and the
    // pacing on its multiple. Otherwise the first frames go out at the client's bitrate and at
    // line rate, and against a path someone else already fills that is a broken first second.
    const double start_bps = params.bitrate_max * params.startup_fraction;
    if (mode == mode_e::full) {
      target_kbps.store(initial_kbps, std::memory_order_relaxed);
      if (params.train_pacing > 0) {
        pacing.store(params.train_pacing * start_bps, std::memory_order_relaxed);
      }
    }
    budget.store(start_bps, std::memory_order_relaxed);
    BOOST_LOG(info) << "Adaptive bitrate: the client reports its signals, the controller takes over"sv;
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
    // would have arrived, a round trip later.
    const auto window = std::max<std::chrono::steady_clock::duration>(std::chrono::milliseconds(300), std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(2.0 * last_rtt_s)));
    if (last_idr_at != std::chrono::steady_clock::time_point {} && now - last_idr_at < window) {
      return false;
    }
    last_idr_at = now;
    meter.idr_request(last_frame_index + 1);
    // And at the plain frame rate: one frame in several, each paid for several times over, made
    // the IDR frame that many times larger
    if (divider > 1) {
      divider = 1;
      publish_frames();
    }
    return true;
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
    abr::gamestream::FecStatus status;
    status.index = be32(data);
    status.missing = be16(data + 8);
    status.data_total = be16(data + 10);
    status.parity_total = be16(data + 12);
    status.data_received = be16(data + 14);
    status.parity_received = be16(data + 16);

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
    {
      std::lock_guard lg {lock};
      activate();
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
    activate();
    return true;
  }

  bool session_t::client_ping(std::chrono::steady_clock::time_point now, std::string_view payload) {
    // A length of 4 and the client's monotonic clock in milliseconds, little-endian. Moonlight
    // writes the length in 32 bits, and in 16 bits before moonlight-common-c e2722dc.
    if (payload.size() < 6) {
      return false;
    }
    const auto *p = reinterpret_cast<const unsigned char *>(payload.data());
    const std::size_t at = payload.size() >= 8 && le32(p) == 4 ? 4 : 2;
    const std::uint32_t client_ms = le32(p + at);
    std::lock_guard lg {lock};
    meter.client_ping(to_us(now), client_ms);
    if (client_ms != 0) {
      activate();
    }
    return true;
  }

  void session_t::queue_sample(std::chrono::steady_clock::time_point now, bool queued, double queue_seen, double growth, double sample_ms) {
    if (mode == mode_e::disabled || !active()) {
      return;
    }
    double now_budget = 0;
    double accepted_ms = sample_ms;
    bool cut = false;
    bool undone = false;
    // The frames in flight, within what the encoder can still predict from (take_frame_divider).
    // Up at once, down only once the round trip has stayed short for half a second. From the
    // sample the meter stored, as a round trip, and only if it took one: a frame it discarded
    // says nothing new about the video's path.
    if (accepted_ms > 0 && (params.frames_in_flight > 0 || divider > 1) && (mode == mode_e::fec || mode == mode_e::full)) {
      // The round trip and the time frames wait in the host before leaving (host_delay)
      const double in_flight_s = accepted_ms / 1000.0 + host_delay_s.load(std::memory_order_relaxed);
      const int wanted = params.frames_in_flight > 0 ? std::clamp(static_cast<int>(std::ceil(in_flight_s * framerate / params.frames_in_flight)), 1, 4) : 1;
      if (wanted >= divider) {
        divider_raised_at = now;
      }
      if (wanted > divider || (wanted < divider && now - divider_raised_at > std::chrono::milliseconds(500))) {
        divider = wanted;
        publish_frames();
      }
    }
    if (!queued) {
      return;
    }
    {
      std::lock_guard lg {lock};
      queue.store(queue_seen, std::memory_order_relaxed);
      // The sample right after an alarm may show the queue back at its floor: then the step was a
      // delayed acknowledgement, not a narrower path, and everything the alarm changed goes back
      if (have_measurement && (mode == mode_e::fec || mode == mode_e::full) && control.retract_alarm(to_us(now) / 1e6, queue_seen)) {
        undone = true;
        now_budget = control.budget();
        min_parity_now.store(control.min_parity(), std::memory_order_relaxed);
        if (divider != divider_before_alarm) {
          divider = divider_before_alarm;
          publish_frames();
        }
      } else if (have_measurement && (mode == mode_e::fec || mode == mode_e::full) && control.alarm(to_us(now) / 1e6, queue_seen, growth, last_measurement.received_rate)) {
        cut = true;
        divider_before_alarm = divider;
        now_budget = control.budget();
        min_parity_now.store(control.min_parity(), std::memory_order_relaxed);
        // What the queue that overflowed holds will be missed a whole queue later, when the first
        // good frame behind it arrives: fewer frames until then keep that loss repairable
        if (params.alarm_frame_divisor > divider) {
          divider = params.alarm_frame_divisor;
          publish_frames();
        }
        if (params.alarm_frame_divisor > 1) {
          divider_raised_at = now + std::chrono::milliseconds(100);
        }
        if (control.alarm_requests_keyframe()) {
          meter.idr_request(last_frame_index + 1);
          idr_wanted.store(true, std::memory_order_relaxed);
        }
      }
    }
    if (!cut && !undone) {
      return;
    }
    const double before = budget.load(std::memory_order_relaxed);
    budget.store(now_budget, std::memory_order_relaxed);
    if (params.train_pacing > 0) {
      pacing.store(params.train_pacing * now_budget, std::memory_order_relaxed);
    }
    if (before > 0) {
      target_kbps.store(static_cast<int>(target_kbps.load(std::memory_order_relaxed) * now_budget / before), std::memory_order_relaxed);
    }
    bitrate_keep.store(frames_kept(), std::memory_order_relaxed);
    BOOST_LOG(debug) << (undone ? "Adaptive bitrate alarm undone: budget="sv : "Adaptive bitrate alarm: budget="sv) << static_cast<int>(now_budget / 1000.0)
                     << " before="sv << static_cast<int>(before / 1000.0)
                     << " queue="sv << std::lround(queue_seen * 1000.0)
                     << " growth="sv << std::lround(growth * 1000.0);
  }

  void session_t::invalidation_requested(std::chrono::steady_clock::time_point now) {
    if ((mode != mode_e::fec && mode != mode_e::full) || !active()) {
      return;
    }
    {
      // Until the frame with the invalidated references arrives the client drops whole frames and
      // counts them lost, exactly as while it waits for an IDR frame: that is not loss in the path
      std::lock_guard lg {lock};
      meter.idr_request(last_frame_index + 1);
    }
    // A frame lost on a path without a queue leaves nothing to drain, and frames four times larger
    // would build one: the divider is for a loss behind a queue
    if (params.alarm_frame_divisor <= 1 || queue.load(std::memory_order_relaxed) < params.alarm_queue) {
      return;
    }
    if (params.alarm_frame_divisor > divider) {
      divider = params.alarm_frame_divisor;
      publish_frames();
    }
    divider_raised_at = now + std::chrono::milliseconds(100);
  }

  bool session_t::enet_ping_due(std::chrono::steady_clock::time_point now) {
    if (mode == mode_e::disabled || !active() || now < next_enet_ping) {
      return false;
    }
    next_enet_ping = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(params.enet_ping_ms / 1000.0));
    return true;
  }

  void session_t::tick(std::chrono::steady_clock::time_point now, double rtt_s) {
    if (mode == mode_e::disabled || !active() || now < next_step) {
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
    target_kbps.store(static_cast<int>(decision.encoder_request / 1000.0), std::memory_order_relaxed);
    bitrate_keep.store(frames_kept(), std::memory_order_relaxed);
    target_fec.store(static_cast<int>(std::lround(decision.repair * 100.0)), std::memory_order_relaxed);
    if (params.train_pacing > 0) {
      pacing.store(params.train_pacing * decision.budget, std::memory_order_relaxed);
    }
    budget.store(decision.budget, std::memory_order_relaxed);
    // An encoder that keeps its bitrate gets the budget as the share of the frames it encodes, by
    // what it sends with every frame, learnt as an average: not by its nominal bitrate —encoders
    // fall short of it, libx264 by a third, and the image stayed at half the budget—, and not by
    // the most it sent lately either: any estimate above what it sends leaves it below seven
    // tenths of the budget, the controller reads that as an encoder with nothing to send and stops
    // raising the budget. Fewer frames at once; more, smoothly over a second
    if (fixed_kbps > 0 && mode == mode_e::full) {
      const double data_kbps = last_measurement.data_rate / 1000.0;
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
    {
      std::lock_guard lg {fec_lock};
      have_decision = true;
      loss_estimate = decision.loss_est;
      burst_estimate = decision.loss_burst;
    }
    min_parity_now.store(decision.min_parity, std::memory_order_relaxed);

    // One line per controller step, in a fixed key=value layout so a session can be analysed
    // afterwards: rates in kbps, times in ms, fractions in percent.
    BOOST_LOG(debug) << "Adaptive bitrate step: budget="sv << static_cast<int>(decision.budget / 1000.0)
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
                     << " late="sv << last_measurement.late_reports
                     << " frames_lost="sv << lost_so_far
                     << " reason="sv << abr::name(decision.reason);

    if (now >= next_log) {
      next_log = now + 2s;
      const auto mode_name = mode == mode_e::shadow ? "shadow"sv : mode == mode_e::fec ? "fec"sv :
                                                                                         "full"sv;
      BOOST_LOG(info) << "Adaptive bitrate ["sv << mode_name << "]: budget "sv
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

  void session_t::encoder_bitrate_fixed(int kbps) {
    fixed_kbps = kbps;
    BOOST_LOG(info) << "Adaptive bitrate: the encoder keeps "sv << kbps << " kbps; the budget goes into the frame rate"sv;
  }

  int session_t::bitrate_to_apply(std::chrono::steady_clock::time_point now) {
    if (mode != mode_e::full || fixed_kbps > 0) {
      return 0;
    }
    // The encoder spreads its bitrate over the frame rate it was opened with: with a share of the
    // frames encoded, it has to be asked for that many times more to fill the same budget. The share
    // is the one at the last budget step, so the frames and the bitrate that makes up for them change
    // together with the budget and not ahead of it
    const int kbps = static_cast<int>(target_kbps.load(std::memory_order_relaxed) / bitrate_keep.load(std::memory_order_relaxed));
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
    if ((mode != mode_e::fec && mode != mode_e::full) || !active()) {
      return configured;
    }
    const int fec = target_fec.load(std::memory_order_relaxed);
    return fec < 0 ? configured : std::clamp(fec, 1, 255);
  }

  int session_t::block_fec_percentage(int data_shards, bool repairs) {
    if ((mode != mode_e::fec && mode != mode_e::full) || !active() || data_shards <= 0) {
      return -1;
    }
    std::lock_guard lg {fec_lock};
    // Before the first decision nothing has been measured, and the first frames go out on a
    // budget nobody verified: protect them as the controller does for its first seconds
    const double loss = have_decision ? loss_estimate : params.parity_startup;
    const double burst = have_decision ? burst_estimate : 1.0;
    if (!have_decision && !params.parity_in_packets) {
      return -1;
    }
    int min_parity = have_decision ? min_parity_now.load(std::memory_order_relaxed) : -1;
    // The frame that repairs a loss must not be lost: losing one that repairs an invalidation
    // invalidates again from it, and two or three in a row push the last good reference out of the
    // encoder's sixteen; losing an IDR frame makes the client ask for another, several times the
    // size of the others, into the same full queue. Its block gets the client's minimum at least and
    // a far stricter failure target, on the loss measured: making up for a client that does not
    // report loss, with a floor here and each request counted as loss, overflowed a path that had
    // just narrowed and fed the very cascade it was meant to stop.
    abr::Params block_params = params;
    if (repairs) {
      block_params.parity_block_failure = params.parity_repair_failure;
      block_params.fec_max = std::max(params.fec_max, params.fec_max_repair);
      min_parity = std::max(min_parity, params.fec_min_packets);
    }
    int parity = params.parity_in_packets ?
                   abr::block_parity(data_shards, loss, burst, block_params, min_parity) :
                   static_cast<int>(std::ceil(data_shards * target_fec.load(std::memory_order_relaxed) / 100.0));
    parity = std::clamp(parity, 1, 255 - data_shards);
    // fec::encode rounds data_shards * percentage / 100 up, so this is the smallest percentage
    // that gives back `parity`: exactly that many while data_shards is at most 100, one more at
    // most above that. Rounding the percentage up as well would add a parity packet to most
    // blocks — two instead of one on a seven-packet frame. The percentage travels in 8 bits of
    // every packet's fecInfo, and the client derives the parity count from it: above 255 it
    // would read the block wrong.
    return std::min(255, (parity - 1) * 100 / data_shards + 1);
  }

}  // namespace adaptive_bitrate
