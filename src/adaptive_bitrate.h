/**
 * @file src/adaptive_bitrate.h
 * @brief Declarations for the per-session adaptive bitrate controller.
 *
 * The controller itself lives in src/abr. This file only adapts Sunshine to it: it records
 * what each frame looked like when it left the host, feeds in what the client already reports
 * over GameStream (the per-frame FEC status and the periodic loss stats) together with the
 * round trip ENet measures on the control stream, and runs one controller step every control
 * period. Nothing is added to the protocol.
 */
#pragma once

// standard includes
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

// local includes
#include "abr/control.h"
#include "abr/gamestream/signals.h"
#include "stream_pacing.h"

/**
 * @brief Bit of `x-ml-general.featureFlags`: the client sends Frame Stats (0x0204) with per-frame
 * arrival times, the loss summary, and its clock in the periodic ping.
 *
 * moonlight-common-c's other ML_FF_* bits live in its Limelight-internal.h; this one is defined here
 * until it lands there, and must keep the same value.
 */
#ifndef ML_FF_ADAPTIVE_SIGNALS
  #define ML_FF_ADAPTIVE_SIGNALS 0x04
#endif
static_assert(ML_FF_ADAPTIVE_SIGNALS == 0x04, "ML_FF_ADAPTIVE_SIGNALS must match moonlight-common-c");

namespace adaptive_bitrate {

  /**
   * @brief What the controller is allowed to change.
   *
   * In fec and full mode the host advertises host_feature_bit in
   * `x-ss-general.featureFlags`, and the controller acts on a session from its start only when its
   * client announces the signals with ML_FF_ADAPTIVE_SIGNALS (announces_signals, active()): Frame
   * Stats, loss stats and its clock in the periodic ping. A session whose client does not announce
   * them streams as with the feature disabled, whatever it sends: it gets no controller session, the encoder
   * opened as without the controller, the configured FEC, the broadcast sender with its pacing, the
   * control thread waiting up to 150 ms for events. The only
   * difference is host_feature_bit in the SDP, which official clients ignore.
   */
  enum class mode_e {
    disabled,  ///< The controller does not run: the SDP, the encoder, the broadcast sender, its pacing and the control thread's 150 ms wait are upstream's.
    fec,  ///< For a client that announces the signals the controller runs on them, logs its decisions, waits up to 5 ms for events to read each round trip and sends the ENet pings that measure them. It also sets the FEC parity of every block, and the encoder opens with the references it repairs from: with NVENC a DPB of min(the level's frames, 5) frames holding 2 long-term references (nvenc_ltr.h, repair_references). The frame rate may drop behind a long queue so a loss stays repairable.
    full,  ///< Like fec, and for a client that announces the signals it also changes the encoder bitrate (or the share of frames encoded, with an encoder that cannot change it), paces each frame at its budget and sends the session's video from a sender thread of its own.
  };

  class session_t;

  /**
   * @brief How long the control loop waits for the next event.
   *
   * 150 ms, as upstream, unless a session's controller acts (active()): then 5 ms, so that each round
   * trip ENet measures is read as it comes.
   *
   * @param session The session's controller, or nullptr without one.
   * @return The wait this session asks for.
   */
  std::chrono::milliseconds control_wait(const session_t *session);

  /**
   * @brief Whether a client announces the controller's signals in its `x-ml-general.featureFlags`.
   *
   * It does when it sets ML_FF_ADAPTIVE_SIGNALS and ML_FF_FEC_STATUS: the controller reads the
   * per-frame FEC status too.
   *
   * @param ml_feature_flags The client's feature flags.
   * @return True when the session can be the controller's from its start.
   */
  bool announces_signals(int ml_feature_flags);

  /**
   * @brief The host's bit in `x-ss-general.featureFlags` that asks the client for the signals.
   *
   * Official clients read only the bits they know (0x01 pen/touch and 0x02 controller touch in
   * moonlight-common-c's public LI_FF_*), so this one changes nothing for them. It cannot be
   * advertised to our client alone: the DESCRIBE answer goes out before the ANNOUNCE that carries the
   * client's own bit. The value is declared here once; where moonlight-common-c declares it as
   * LI_FF_ADAPTIVE_BITRATE the two must agree.
   */
  inline constexpr std::uint32_t host_feature_bit = 0x04;

  /**
   * @brief Whether a client's session gets the controller.
   * @param mode The configured mode.
   * @param ml_feature_flags The client's feature flags.
   * @return True when the controller runs on the session.
   */
  bool session_wanted(mode_e mode, int ml_feature_flags);

  /**
   * @brief The host's feature bits the controller adds to `x-ss-general.featureFlags`.
   *
   * host_feature_bit in fec and full mode, which tells the client to send the
   * signals; none otherwise.
   *
   * @param mode The configured mode.
   * @return The bits to advertise.
   */
  std::uint32_t host_feature_flags(mode_e mode);

  /**
   * @brief Parse the `adaptive_bitrate` configuration value.
   * @param value Configured value; anything unknown means disabled.
   * @return The mode.
   */
  mode_e mode_from_string(std::string_view value);

  /**
   * @brief The frame divider the frames in flight call for.
   * @param in_flight_s The round trip plus the time frames wait in the host before leaving.
   * @param framerate Frames per second the host encodes.
   * @param frames_in_flight Frames that can be in flight before fewer are sent (`frames_in_flight`), 0 for no limit.
   * @param repair_reach Frames the encoder's long-term references reach back; it takes the place
   * of `frames_in_flight` when it is larger.
   * @return The frames in flight over that limit, rounded up, from 1 to 4; 1 without a limit.
   */
  int frame_divider_for(double in_flight_s, double framerate, int frames_in_flight, int repair_reach);

  /**
   * @brief The frame rate the host actually encodes, for counting frames in flight.
   * @param measured Frames per second seen over the last window, 0 or less before it has data.
   * @param nominal The session's frame rate.
   * @return The measured rate, never above the nominal one; the nominal one without data.
   */
  double encoded_rate_for(double measured, double nominal);

  /**
   * @brief A wire rate in the unit of the video rate: payload bytes.
   * @param wire_rate Rate of the data packets on the wire, headers included.
   * @param payload Video payload bytes of a packet.
   * @param wire Bytes of a packet on the wire.
   * @return The rate of the payload alone.
   */
  double payload_rate(double wire_rate, int payload, int wire);

  /**
   * @brief The client's clock in a periodic ping.
   * @param payload The ping's payload, at least 6 bytes.
   * @return The client's monotonic clock in milliseconds, or nothing when the ping does not carry one.
   */
  std::optional<std::uint32_t> client_clock_ms(std::string_view payload);

  /**
   * @brief The bitrate the controller may reach, from what the client announced.
   * @param vqos_kbps `x-nv-vqos[0].bw.maximumBitrateKbps`.
   * @param configured_kbps `x-ml-video.configuredBitrateKbps` as the client sent it, 0 if it did not.
   * @return The ceiling, in kbit/s: the client's value up to the largest int, or `vqos_kbps` when it is zero or negative.
   */
  int ceiling_kbps(int vqos_kbps, std::int64_t configured_kbps);

  /**
   * @brief The client messages the controller parses, for the log of the malformed ones.
   */
  enum class message_e {
    loss_stats,  ///< IDX_LOSS_STATS.
    frame_stats,  ///< IDX_FRAME_STATS.
    fec_status,  ///< SS_FRAME_FEC_STATUS.
  };

  /**
   * @brief What the control thread decided, as the sender thread reads it.
   *
   * Built by the control thread and never changed once published (snapshot_t), so the sender
   * reads a consistent set of decisions without a lock held across a frame.
   */
  struct decision_t {
    bool stepped = false;  ///< A controller step has run.
    int target_fec = -1;  ///< Parity percentage of the last step, -1 before it.
    int min_parity = 2;  ///< Fewest parity packets per block the controller allows.
    double loss = 0;  ///< Loss the parity is sized for.
    double burst = 1;  ///< Mean length of the loss bursts.
    double budget = 0;  ///< The whole budget in bits per second, 0 before the controller acts.
    double pacing = 0;  ///< Pacing rate in bits per second, 0 for the stream's default.
    double pay_multiple = 1.1;  ///< Multiple of the budget a frame is paid at: 1.1, more at the application's ceiling.
    int open_kbps = 0;  ///< Encoder bitrate a new encoder opens at, 0 for an encoder that keeps the client's bitrate.
    std::shared_ptr<const abr::Params> block_params;  ///< Parameters of an ordinary block, with the last step's failure target.
    std::shared_ptr<const abr::Params> repair_params;  ///< Parameters of a block that repairs a loss.

    /**
     * @brief Compare every field.
     * @return True when all fields are equal.
     */
    bool operator==(const decision_t &) const = default;
  };

  /**
   * @brief One immutable value, replaced whole by a single writer and read by any number of readers.
   *
   * A mutex guards the swap of the pointer and nothing else: an atomic shared pointer is not
   * in every standard library the project builds with, and the free `std::atomic_load` overloads are
   * deprecated in C++20. A reader holds the mutex for a pointer copy.
   *
   * @tparam T The published type.
   */
  template<class T>
  class snapshot_t {
  public:
    /**
     * @param first The value readers see until the first publication.
     */
    explicit snapshot_t(std::shared_ptr<const T> first):
        current {std::move(first)} {
    }

    /**
     * @brief The last published value.
     * @return A pointer that stays valid, and unchanged, for as long as it is held.
     */
    std::shared_ptr<const T> load() const {
      std::lock_guard lg {lock};
      return current;
    }

    /**
     * @brief Replace the published value. Only the owner calls this.
     * @param next The new value.
     */
    void store(std::shared_ptr<const T> next) {
      std::lock_guard lg {lock};
      current = std::move(next);
    }

  private:
    mutable std::mutex lock;
    std::shared_ptr<const T> current;
  };

  /**
   * @brief Controller state for one streaming session.
   *
   * Two threads touch it. The control thread owns every decision —`fec_status()`, `loss_stats()`,
   * `frame_stats()`, `tick()`, `bitrate_to_apply()` and the other calls that feed or act on the
   * controller— and publishes them as a decision_t. The sender thread only reads that snapshot
   * (`fec_percentage()`, `min_parity()`, `block_fec_percentage()`, `budget_bps()`, `pacing_bps()`,
   * `frame_pacing_bps()`, `initial_encoder_kbps()`) and reports what it measured
   * (`frame_sent()`, `host_delay()`), which the control thread reads under `lock`.
   */
  class session_t {
  public:
    /**
     * @param mode What the controller may change.
     * @param bitrate_kbps Bitrate the client asked for; the controller never goes above it.
     * @param framerate Frames per second of the stream.
     * @param packetsize Video payload size negotiated with the client.
     * @param wire_overhead Bytes each video packet costs on the wire beyond what the client counts.
     * @param audio_bps Audio bitrate on the wire, which is paid from the same budget.
     * @param min_parity Parity packets the client requires in every FEC block.
     * @param announced Whether the client announced the signals (announces_signals): the controller
     * acts from the start, and otherwise never.
     */
    session_t(mode_e mode, int bitrate_kbps, int framerate, int packetsize, int wire_overhead, double audio_bps, int min_parity = 2, bool announced = true);

    /**
     * @brief Record a frame that just left the host.
     * @param frame_index Index of the frame.
     * @param first When its first packet left.
     * @param last When its last packet left.
     * @param packets Data and parity packets of the frame.
     * @param data_packets Of those, the data packets; the rest is the parity that went out.
     * @param rtp_bytes Bytes the client counts of its packets.
     * @param wire_bytes Bytes it costs on the wire, with IP, UDP and encryption.
     */
    void frame_sent(std::uint32_t frame_index, std::chrono::steady_clock::time_point first, std::chrono::steady_clock::time_point last, int packets, int data_packets, double rtp_bytes, double wire_bytes);

    /**
     * @brief Record how long a frame spent in the host, from capture until its first packet left.
     *
     * The frames the encoder makes after a loss until the client's repair request arrives are the
     * round trip and this, times the frame rate: frames held by the pacing count as much as the
     * path's queue toward the references the encoder repairs from (take_frame_divider).
     *
     * @param delay Capture to first packet out.
     */
    void host_delay(std::chrono::steady_clock::duration delay) {
      // The most of the last ones, forgotten in about half a second of frames
      const double s = std::chrono::duration<double>(delay).count();
      std::lock_guard lg {lock};
      measured.host_delay_s = std::max(s, measured.host_delay_s * 0.97);
    }

    /**
     * @brief Handle an SS_FRAME_FEC_STATUS message from the client.
     *
     * Moonlight sends one for every frame (or FEC block) that needed parity to recover or
     * could not be recovered: how many data and parity packets it had and how many arrived.
     *
     * @param payload Message payload as received on the control stream.
     * @return false if the payload is malformed.
     */
    bool fec_status(std::string_view payload);

    /**
     * @brief Note that the client asked for an IDR frame.
     *
     * Until it arrives the client drops the frames it receives, and reports them lost: those are
     * not network loss, and the meter leaves them out.
     *
     * @param now When the request arrived.
     * @return Whether to send one now: not if the last one went out less than two round trips
     * ago, since that one may still be on its way. Such a request is not dropped, in case that one
     * was lost: the window's end sends one, through take_idr_request(), for every request it held.
     */
    bool idr_requested(std::chrono::steady_clock::time_point now);

    /**
     * @brief Handle the client's periodic ping (0x0200).
     *
     * Moonlight fills its 4-byte timestamp with its monotonic clock when talking to Sunshine.
     * How its arrival drifts from that clock is the queue of the client to host direction, which
     * the loss stats travel through: the meter takes it out of how old the last good frame is.
     * A client that leaves the timestamp at zero is simply not used.
     *
     * @param now When the ping arrived.
     * @param payload Message payload as received on the control stream.
     * @return False if the payload is too short.
     */
    bool client_ping(std::chrono::steady_clock::time_point now, std::string_view payload);

    /**
     * @brief Whether the controller wants a key frame sent now, once.
     *
     * When the path narrows, what left in the last hundred milliseconds is being lost, and the
     * client only notices when the first whole frame reaches it a third of a second later; from
     * then on it drops everything until the key frame it asks for. Sending one right away makes
     * that first whole frame the key frame. Deciding to send a key frame is the host's call: the
     * protocol carries it as it carries any other.
     *
     * @return True once when a key frame should be sent, then false until the controller asks again.
     */
    bool take_idr_request() {
      return std::exchange(idr_wanted, false);
    }

    /**
     * @brief How many captured frames in a thousand to encode, when that changed since the last
     * call; 0 if it did not.
     *
     * The client learns that a frame was lost only when the frames behind it arrive, a round trip
     * later counting the queue. To repair the loss by invalidating references, the last frame it
     * has must still be among the encoder's references (the reach the encoder reports, repair_reach_frames), and
     * behind a router queue of a quarter of a second sixty frames per second push it out. So the
     * frames in flight, the round trip times the frame rate the encoder produces, are kept within
     * `frames_in_flight`: first the bitrate drops so the queue drains, and only when that cannot be
     * done in time are fewer frames encoded, until the frames in flight fit again. With an encoder
     * that keeps its bitrate (encoder_bitrate_fixed) the share of frames is what carries the budget.
     *
     * @return Frames to encode per thousand captured, or 0 if nothing changed.
     */
    int take_frame_divider() {
      return std::exchange(divider_changed, 0);
    }

    /**
     * @brief Note that the encoder cannot change its bitrate during the session.
     *
     * Only NVENC on Windows and libx264 change it without resetting the encoder or forcing an IDR
     * frame. For the rest the budget goes into the frame rate: the encoder spreads the bitrate it
     * was opened with over the frames it is handed, so with a share of them encoded it sends that
     * share of it. From here on no bitrate is asked of the encoder.
     *
     * @param kbps The bitrate the encoder keeps.
     */
    void encoder_bitrate_fixed(int kbps);

    /**
     * @brief Note how many frames back the encoder repairs a loss from its long-term references.
     *
     * With them, a loss reported after more frames than the short-term references hold is
     * still repaired with a P frame, so the frames in flight are allowed up to this reach before
     * any frame is skipped: the frame rate is kept, which the player feels more than the detail.
     *
     * @param frames Frames back, 0 without long-term references.
     */
    void encoder_repair_reach(int frames) {
      repair_reach = frames;
    }

    /**
     * @brief Note that a new encoder opened, so the next `bitrate_to_apply()` pushes the controller's
     * bitrate at once instead of waiting for it to move or for the refresh.
     */
    void encoder_reopened();

    /**
     * @brief Note that the client asked to invalidate references.
     *
     * It keeps asking, from the same first frame, while it finds more frames lost behind it; each
     * time the encoder must still hold the frame before that one. From the first request on, fewer
     * the frames in flight are counted as always (take_frame_divider); the request itself changes nothing.
     *
     * @param now When the request arrived.
     */
    void invalidation_requested(std::chrono::steady_clock::time_point now);

    /**
     * @brief What the control thread has decided, as of now.
     *
     * One consistent set: the sender reads it once for a frame and sees no step land half way.
     *
     * @return The last published decisions; the pointer stays valid and unchanged while held.
     */
    std::shared_ptr<const decision_t> decision() const {
      return published.load();
    }

    /**
     * @brief The video queue the controller currently estimates.
     *
     * @return Seconds.
     */
    double queue_s() const {
      return queue;
    }

    /**
     * @brief Handle a loss stats message from the client.
     *
     * Carries the frames lost since the previous one and the last frame fully received. Clients
     * that send it do so every 50 ms; the controller works without it, estimating what arrives
     * instead of measuring it.
     *
     * @param now When the message arrived.
     * @param payload Message payload as received on the control stream.
     * @return false if the payload is malformed.
     */
    bool loss_stats(std::chrono::steady_clock::time_point now, std::string_view payload);

    /**
     * @brief Handle a Frame Stats message: when the client received each frame.
     *
     * GameStream reserves this message and nothing ever filled it. From how much more spread out
     * a frame's packets arrived than they left, the controller reads the width of the path even
     * while the frames are small (a menu). From how much later each frame arrived, relative to
     * when it left, it reads the video's own queue as it grows: between two controller steps, a
     * step in that queue that only a narrower path explains cuts the budget right away
     * (abr::Control::alarm), since a path that narrows threefold overflows a quarter-second router
     * buffer in about 150 ms, sooner than the next step. Clients that do not send it leave the
     * controller measuring without it.
     *
     * @param now Current time.
     * @param payload Message payload as received on the control stream.
     * @return false if the payload is malformed.
     */
    bool frame_stats(std::chrono::steady_clock::time_point now, std::string_view payload);
    /**
     * @brief Whether the host should send an ENet ping now.
     *
     * ENet only updates a peer's round trip when an acknowledgement for one of its own reliable
     * commands comes back, and it only pings when the peer has been silent. The client's periodic
     * ping keeps the host from ever being silent, so without its own pings the host's round trip
     * would stay at whatever it was when the session started.
     *
     * @param now Current time.
     * @return True if a ping is due.
     */
    bool enet_ping_due(std::chrono::steady_clock::time_point now);

    /**
     * @brief Run a controller step if one is due, and ask for the IDR frame a request inside
     * the last one's window was held for, once that window ended (take_idr_request()).
     * @param now Current time.
     * @param rtt_s Round-trip time of the control stream, in seconds, as ENet smooths it.
     */
    void tick(std::chrono::steady_clock::time_point now, double rtt_s);

    /**
     * @brief FEC percentage to use for the next frame.
     *
     * Only sizes the frame's FEC blocks; the parity of each block comes from
     * `block_fec_percentage()`. A configured 0 means no FEC and stays 0.
     *
     * @param configured The percentage from the configuration.
     * @return The percentage to use.
     */
    int fec_percentage(int configured) const;

    /**
     * @brief FEC percentage that sizes a frame's blocks, from a decision the caller already holds.
     *
     * @param configured The percentage from the configuration.
     * @param decision The frame's decisions (`decision()`).
     * @return The percentage to use.
     */
    int fec_percentage(int configured, const decision_t &decision) const;

    /**
     * @brief FEC percentage for one block of the frame being sent.
     *
     * The controller decides parity in packets, per block, from the loss and the burst length
     * it measured: the fewest parity packets that keep the chance of losing the block below
     * its target. This returns the percentage that makes `fec::encode` produce that many.
     *
     * @param data_shards Data packets of this block.
     * @param repairs Whether the frame repairs a loss —the one after a reference invalidation, or an
     *                IDR frame—, which gets a far stricter target.
     * @return The percentage, 0 for a block that needs no parity, or -1 to keep the frame's percentage.
     */
    int block_fec_percentage(int data_shards, bool repairs = false);

    /**
     * @brief FEC percentage for one block, from a decision the caller already holds.
     *
     * @param data_shards Data packets of this block.
     * @param repairs Whether the frame repairs a loss.
     * @param decision The frame's decisions (`decision()`).
     * @return The percentage, 0 for a block that needs no parity, or -1 to keep the frame's percentage.
     */
    int block_fec_percentage(int data_shards, bool repairs, const decision_t &decision);

    /**
     * @brief Fewest parity packets `fec::encode` may put in a block.
     *
     * The client asks for at least two, but it reads the parity count of every block from the
     * percentage in its packets and repairs with whatever came. With frames of five packets, two
     * parity packets are 40% of the wire; far from any loss the controller settles for fewer
     * (abr::Control::min_parity), and goes back to what the client asked as soon as it
     * sees loss or a queue step.
     *
     * @param requested The minimum the client asked for.
     * @return Parity packets per block: `requested`, or fewer if the controller allows it.
     */
    int min_parity(int requested) const {
      return min_parity(requested, *published.load());
    }

    /**
     * @brief Fewest parity packets a block may have, from a decision the caller already holds.
     *
     * @param requested The minimum the client asked for.
     * @param decision The frame's decisions (`decision()`).
     * @return Parity packets per block: `requested`, or fewer if the controller allows it.
     */
    int min_parity(int requested, const decision_t &decision) const {
      if ((mode != mode_e::fec && mode != mode_e::full) || !active()) {
        return requested;
      }
      return std::min(requested, std::max(0, decision.min_parity));
    }

    /**
     * @brief Encoder bitrate the controller wants now, in kilobits per second.
     *
     * This is the image budget: what is left of the total budget after FEC and audio. It is
     * 0 until the first controller step.
     *
     * @return Kilobits per second.
     */
    int target_bitrate_kbps() const {
      return target_kbps;
    }

    /**
     * @brief Encoder bitrate a new encoder opens at, in kilobits per second.
     *
     * The image part of the starting budget until the controller has stepped, then what it asks
     * the running encoder for (`bitrate_to_apply()`), so an encoder reopened mid-session does not
     * come back at the client's full bitrate. 0 for an encoder that keeps its bitrate
     * (`encoder_bitrate_fixed()`): it opens at the client's. Only meaningful in full mode.
     *
     * @return Kilobits per second.
     */
    int initial_encoder_kbps() const {
      return published.load()->open_kbps;
    }

    /**
     * @brief Rate to pace each frame's packets at, in bits per second.
     *
     * A multiple of the whole budget, so each frame leaves spread over part of the frame interval
     * instead of as one burst at line rate. Against a queue that another flow keeps full, a burst
     * loses a run of consecutive packets that no parity can repair; spread out, the losses are
     * scattered and the block parity covers them. The multiple is `tun::kPaceOverPipe` times the capacity
     * estimate over the budget, between 1.1 and 2 (`tun::kPaceMax` until there is a capacity estimate), and
     * `tun::kPaceMax` while the budget sits at the application's ceiling. Only in
     * full mode, and 0 for a client that does not announce its signals, which means the stream's default pacing.
     *
     * @return Bits per second, or 0 for the stream's default pacing.
     */
    double pacing_bps() const {
      return mode == mode_e::full ? published.load()->pacing : 0.0;
    }

    /**
     * @brief Rate to pace the frame leaving now at: `pacing_bps()`, except one frame every
     * `cal::kPaceProbeS` paced at twice the budget, so the dispersion still sees above it.
     * Called from the session's video sender only.
     * @param now When the frame leaves.
     * @return Bits per second, or 0 for the stream's default pacing.
     */
    double frame_pacing_bps(std::chrono::steady_clock::time_point now) {
      const auto decision = published.load();
      const double rate = mode == mode_e::full ? decision->pacing : 0.0;
      const double whole = decision->budget;
      if (rate <= 0 || whole <= 0) {
        return rate;
      }
      return pace_probe.frame_multiplier(now, abr::cal::kPaceProbeS, rate / whole) * whole;
    }

    /**
     * @brief Whether the client sends what the controller needs to act on the stream.
     *
     * Every Moonlight sends the per-frame FEC status, but only one that also reports the loss
     * stats, the frame arrivals (Frame Stats) and its clock in the periodic ping gives the
     * controller the loss of whole frames and the queue of each direction. A client announces them
     * in its feature flags (announces_signals), so the controller acts from the session's start or
     * never: without them the stream runs as without the controller, the client's bitrate, the
     * configured FEC and the default pacing.
     *
     * @return True when the client announced the signals.
     */
    bool active() const {
      return active_signals;
    }

    /**
     * @brief Whether the controller may pace the stream's frames, which is only in full mode.
     *
     * @return True in full mode.
     */
    bool paces() const {
      return mode == mode_e::full;
    }

    /**
     * @brief Whether the session's video leaves from a sender thread of its own.
     *
     * Only in full mode and for a client that announced the signals (active()), from the first
     * frame; otherwise the session's packets leave from the broadcast sender with its pacing, as
     * without the controller.
     *
     * @return True when the session's own sender should run.
     */
    bool own_sender() const {
      return paces() && active();
    }

    /**
     * @brief The whole budget, in bits per second, for the stream to pay its frames at.
     *
     * Each frame is paced at `pacing_bps()`, but a frame may not start before the previous one
     * has been paid for at this rate. Frames of the size the budget allows are never held back;
     * frames larger than that —encoded for the budget before the last cut, or an encoder
     * overshooting at a scene change— wait on the host instead of flooding the path's queue.
     * Only in full mode, and 0 means no limit.
     *
     * @return Bits per second, or 0 for no limit.
     */
    double budget_bps() const {
      return budget_bps(*published.load());
    }

    /**
     * @brief The budget of a decision the caller already holds.
     *
     * @param decision The frame's decisions (`decision()`).
     * @return Bits per second, or 0 for no limit.
     */
    double budget_bps(const decision_t &decision) const {
      return mode == mode_e::full ? decision.budget : 0.0;
    }

    /**
     * @brief The multiple of the budget the stream pays its frames at (`stream::pay_multiple`).
     *
     * 1.1, and `tun::kPaceMax` while the budget sits at the application's ceiling.
     *
     * @return The multiple.
     */
    double pay_multiple() const {
      return published.load()->pay_multiple;
    }

    /**
     * @brief Encoder bitrate to push to the encoder now, if any.
     *
     * Only in full mode, and only when the target moved enough to matter or a while has passed
     * since the last push — an encoder restarted mid-session comes back at the configured
     * bitrate and needs to be told again.
     *
     * @param now Current time.
     * @return Bitrate in kbps, or 0 if there is nothing to push.
     */
    int bitrate_to_apply(std::chrono::steady_clock::time_point now);

    /**
     * @brief Note a malformed client message, and say whether it is worth a warning.
     *
     * @param kind The message.
     * @return True for the first malformed message of its kind in the session.
     */
    bool malformed(message_e kind);

  private:
    const mode_e mode;
    const int configured_kbps;
    const int framerate;
    const int payload_per_packet;
    const int wire_per_packet;
    const abr::Params params;  ///< Set at construction; what the control thread decides goes into `published`.
    const bool active_signals;  ///< See active().
    int initial_kbps = 0;

    /// What the sender thread measured, and the control thread reads; under `lock`.
    struct measured_t {
      double host_delay_s = 0;  ///< Capture to send, the most of the last frames (host_delay).
      double fps = 0;  ///< Frames per second the encoder produced over the last closed window, 0 before one.
      std::chrono::steady_clock::time_point window_start {};  ///< The first frame of the window being counted.
      int window_frames = -1;  ///< Frames after the first of that window; -1 before the first frame.
    };

    mutable std::mutex lock;  ///< Guards the meter, the controller and `measured`.
    abr::Control control;
    abr::gamestream::SignalMeter meter;
    measured_t measured;
    bool have_measurement = false;
    std::uint64_t frames_lost = 0;  ///< Frames the client reported lost, over the whole session.
    std::uint32_t last_frame_index = 0;  ///< The last frame handed to the meter.
    abr::Measurement last_measurement;

    /// The published decisions: written by the control thread through `publish()`, read by the sender.
    snapshot_t<decision_t> published;

    // Owned by the control thread.
    std::array<bool, 3> malformed_seen {};  ///< Per message_e: a malformed one was already warned about.
    std::chrono::steady_clock::time_point next_enet_ping {};
    std::chrono::steady_clock::time_point next_step {};
    std::chrono::steady_clock::time_point last_tick {};
    std::chrono::steady_clock::time_point hold_until {};
    std::chrono::steady_clock::time_point next_log {};
    std::chrono::steady_clock::time_point last_applied_at {};
    int last_applied_kbps = 0;
    decision_t draft;  ///< The decisions as the control thread builds them; published when they change.
    int target_kbps = 0;
    double capacity = 0.0;  ///< The controller's capacity estimate, bits/s.
    double block_failure = 0.0;  ///< The block failure target of the last step, 0 before it.
    double queue = 0.0;
    bool idr_wanted = false;
    int divider_changed = 0;
    int repair_reach = 0;  ///< Frames back the encoder's long-term references repair from (encoder_repair_reach).
    int divider = 1;  ///< The divider last handed to the video thread.
    double bitrate_keep = 1.0;  ///< The share of frames encoded at the last budget step: what bitrate_to_apply scales by.
    double drain_keep = 1.0;  ///< The share of its bitrate the encoder keeps while the queue over the repair reach drains.
    bool draining = false;  ///< A drain of the queue over the repair reach is under way.
    std::chrono::steady_clock::time_point drain_since {};  ///< When it began.
    int fixed_kbps = 0;  ///< The bitrate of an encoder that cannot change it, 0 otherwise.
    double rate_keep = 1.0;  ///< The share of frames that brings that encoder down to the budget.
    double full_kbps = 0;  ///< What that encoder sends encoding every frame, learnt.
    std::chrono::steady_clock::time_point last_idr_at {};  ///< The last IDR request answered.
    bool idr_held = false;  ///< A request arrived inside the window of last_idr_at: its end answers it.
    double last_rtt_s = 0;  ///< The last round trip ENet measured (smoothed), in seconds.

    // Used by the video sender only.
    stream::pace_probe_t pace_probe;

    /// The parameters of a block that repairs a loss: stricter failure target, more parity allowed.
    static std::shared_ptr<const abr::Params> repair_params_of(const abr::Params &base);

    /// Publish the decisions if they changed since the last publication.
    void publish();

    /// The budget the stream is paid at and the pacing it leaves at, for a capacity estimate; pacing only when `with_pacing`.
    void set_budget(double budget, double capacity, bool with_pacing);

    /// What the encoder is asked for now: the image budget scaled by the share of frames and the drain, within `max_bitrate`.
    int requested_kbps() const;

    /// The share of captured frames encoded: the fewer of the divider's and the budget's.
    double frames_kept() const {
      return std::min(1.0 / divider, rate_keep);
    }

    /**
     * @brief Keep the frames in flight within what the encoder can repair, quality before frames.
     *
     * First the encoder's bitrate drops so the queue over the reach drains (drain_keep); frames
     * are skipped only when that cannot be done in time, and all at once when the frames fit.
     *
     * @param now When the sample arrived.
     * @param in_flight_s The round trip plus the time frames wait in the host before leaving.
     */
    void frames_in_flight_limit(std::chrono::steady_clock::time_point now, double in_flight_s);

    /// What a frame's arrival showed of the queue: the alarm, and the frames in flight.
    void queue_sample(std::chrono::steady_clock::time_point now, bool queued, double queue_seen, double growth, double sample_ms);

    /// The part of queue_sample that decides; queue_sample publishes what it changed.
    void react_to_queue(std::chrono::steady_clock::time_point now, bool queued, double queue_seen, double growth, double sample_ms);

    /// The client announced the controller's signals: the controller acts from the start (active()).
    void activate();

    /**
     * @brief A bitrate to ask the encoder for, within the host's `max_bitrate`.
     * @param kbps The bitrate the controller wants.
     * @return That bitrate, at most `config::video.max_bitrate` when it is set.
     */
    static int encoder_kbps(int kbps);

    /// How long after an IDR frame another request is held: the longer of 300 ms and two round trips.
    std::chrono::steady_clock::duration idr_window() const;

    /// An IDR request is answered at `now`: the window starts over, at the plain frame rate. Under `lock`.
    void idr_answered(std::chrono::steady_clock::time_point now);

    /// Hand the share of frames to the video thread, in thousandths.
    void publish_frames() {
      divider_changed = std::max(1, static_cast<int>(std::lround(1000.0 * frames_kept())));
    }
  };

}  // namespace adaptive_bitrate
