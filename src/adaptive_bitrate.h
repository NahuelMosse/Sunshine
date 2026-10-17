/**
 * @file src/adaptive_bitrate.h
 * @brief Declarations for the per-session adaptive bitrate controller.
 *
 * The controller itself lives in src/estab and is shared verbatim with the test bench that
 * validates it. This file only adapts Sunshine to it: it records what each frame looked like
 * when it left the host, feeds in what the client already reports over GameStream (the
 * per-frame FEC status and the periodic loss stats) together with the round trip ENet
 * measures on the control stream, and runs one controller step every control period. Nothing
 * is added to the protocol.
 */
#pragma once

// standard includes
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>

// local includes
#include "estab/control.h"
#include "estab/gamestream/senales.h"

namespace adaptive_bitrate {

  /**
   * @brief What the controller is allowed to change.
   */
  enum class mode_e {
    disabled,  ///< The controller does not run.
    shadow,  ///< It runs on the real signals and logs its decisions, but changes nothing.
    fec,  ///< It also sets the FEC percentage of every frame.
    full,  ///< It also changes the encoder bitrate, where the encoder supports it.
  };

  /**
   * @brief Parse the `adaptive_bitrate` configuration value.
   * @param value Configured value; anything unknown means disabled.
   * @return The mode.
   */
  mode_e mode_from_string(std::string_view value);

  /**
   * @brief Controller state for one streaming session.
   *
   * `frame_sent()` and `fec_percentage()` are called from the video thread; `fec_status()`,
   * `loss_stats()` and `tick()` from the control thread.
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
     */
    session_t(mode_e mode, int bitrate_kbps, int framerate, int packetsize, int wire_overhead, double audio_bps, int min_parity = 2);

    /**
     * @brief Record a frame that just left the host.
     * @param packets Data and parity packets of the frame.
     * @param data_packets Of those, the data packets; the rest is the parity that went out.
     */
    void frame_sent(std::uint32_t frame_index, std::chrono::steady_clock::time_point first, std::chrono::steady_clock::time_point last, int packets, int data_packets, double rtp_bytes, double wire_bytes);

    /**
     * @brief Record how long a frame spent in the host, from capture until its first packet left.
     *
     * The frames the encoder makes after a loss until the client's repair request arrives are the
     * round trip and this, times the frame rate: frames held by the pacing count as much as the
     * path's queue toward the sixteen references (take_frame_divider).
     *
     * @param delay Capture to first packet out.
     */
    void host_delay(std::chrono::steady_clock::duration delay) {
      // The most of the last ones, forgotten in about half a second of frames
      const double s = std::chrono::duration<double>(delay).count();
      const double held = host_delay_s.load(std::memory_order_relaxed);
      host_delay_s.store(std::max(s, held * 0.97), std::memory_order_relaxed);
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
     * @return Whether to send one: not if the last one went out less than two round trips ago,
     * since that one is still on its way.
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
     * @return False if the payload is too short.
     */
    bool client_ping(std::chrono::steady_clock::time_point now, std::string_view payload);

    /**
     * @brief Feed the last round trip ENet measured, as soon as it is read.
     *
     * Between two controller steps, a step in the queue that only a narrower path explains cuts
     * the budget right away (estab::Control::alarma): a path that narrows threefold overflows a
     * quarter-second router buffer in about 150 ms, sooner than the next step.
     *
     * @param now Current time.
     * @param sample_ms ENet's last round-trip sample, in milliseconds; 0 if none yet.
     */
    void enet_sample(std::chrono::steady_clock::time_point now, double sample_ms);

    /**
     * @brief The queue the last ENet sample showed over its floor, in seconds.
     */
    /**
     * @brief Whether the controller wants a key frame sent now, once.
     *
     * When the path narrows, what left in the last hundred milliseconds is being lost, and the
     * client only notices when the first whole frame reaches it a third of a second later; from
     * then on it drops everything until the key frame it asks for. Sending one right away makes
     * that first whole frame the key frame. Deciding to send a key frame is the host's call: the
     * protocol carries it as it carries any other.
     */
    bool take_idr_request() {
      return idr_wanted.exchange(false, std::memory_order_relaxed);
    }

    /**
     * @brief How many captured frames in a thousand to encode, when that changed since the last
     * call; 0 if it did not.
     *
     * The client learns that a frame was lost only when the frames behind it arrive, a round trip
     * later counting the queue. To repair the loss by invalidating references, the last frame it
     * has must still be among the encoder's references —sixteen at most in H.264 and HEVC—, and
     * behind a router queue of a quarter of a second sixty frames per second push it out. So the
     * frames in flight, the round trip times the frame rate, are kept within `cuadros_en_vuelo`:
     * with a long queue fewer frames are encoded, and the client sees a lower frame rate, as it
     * does whenever the screen is still. And with an encoder that keeps its bitrate
     * (encoder_bitrate_fixed) the share of frames is what carries the budget.
     */
    int take_frame_divider() {
      return divider_changed.exchange(0, std::memory_order_relaxed);
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
     * @brief Note that the client asked to invalidate references.
     *
     * It keeps asking, from the same first frame, while it finds more frames lost behind it; each
     * time the encoder must still hold the frame before that one. From the first request on, fewer
     * frames are encoded for a while (`cuadros_alarma`), as after an alarm.
     */
    void invalidation_requested(std::chrono::steady_clock::time_point now);

    double queue_s() const {
      return queue.load(std::memory_order_relaxed);
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
     * while the frames are small (a menu). Clients that do not send it leave the controller
     * measuring without it.
     *
     * @param payload Message payload as received on the control stream.
     * @return false if the payload is malformed.
     */
    bool frame_stats(std::string_view payload);
    /**
     * @brief Whether the host should send an ENet ping now.
     *
     * ENet only updates a peer's round trip when an acknowledgement for one of its own reliable
     * commands comes back, and it only pings when the peer has been silent. The client's periodic
     * ping keeps the host from ever being silent, so without its own pings the host's round trip
     * would stay at whatever it was when the session started.
     *
     * @param now Current time.
     */
    bool enet_ping_due(std::chrono::steady_clock::time_point now);

    /**
     * @brief Run a controller step if one is due.
     * @param now Current time.
     * @param rtt_s Round-trip time of the control stream, in seconds, as ENet smooths it.
     * @param rtt_sample_s The last round trip ENet measured, before smoothing, in seconds; 0 if
     * none yet. ENet's smoothing takes about eight samples to follow a queue that fills in a
     * fifth of a second, which is how fast a path that narrows threefold overflows.
     */
    void tick(std::chrono::steady_clock::time_point now, double rtt_s, double rtt_sample_s = 0);

    /**
     * @brief FEC percentage to use for the next frame.
     *
     * Only sizes the frame's FEC blocks; the parity of each block comes from
     * `block_fec_percentage()`.
     *
     * @param configured The percentage from the configuration.
     */
    int fec_percentage(int configured) const;

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
     * @return The percentage, or -1 to keep the frame's percentage.
     */
    int block_fec_percentage(int data_shards, bool repairs = false);

    /**
     * @brief Fewest parity packets `fec::encode` may put in a block.
     *
     * The client asks for at least two, but it reads the parity count of every block from the
     * percentage in its packets and repairs with whatever came. With frames of five packets, two
     * parity packets are 40% of the wire; far from any loss the controller settles for fewer
     * (estab::Control::paridad_minima), and goes back to what the client asked as soon as it
     * sees loss or a queue step.
     *
     * @param requested The minimum the client asked for.
     */
    int min_parity(int requested) const {
      if (mode != mode_e::fec && mode != mode_e::full) {
        return requested;
      }
      return std::min(requested, std::max(0, min_parity_now.load(std::memory_order_relaxed)));
    }

    /**
     * @brief Encoder bitrate the controller wants now, in kilobits per second.
     *
     * This is the image budget: what is left of the total budget after FEC and audio. It is
     * 0 until the first controller step.
     */
    int target_bitrate_kbps() const {
      return target_kbps.load(std::memory_order_relaxed);
    }

    /**
     * @brief Bitrate to open the encoder with, in kilobits per second.
     *
     * The image part of the budget the controller starts from, so the first frames do not go
     * out at the client's full bitrate before the controller has measured anything. Only
     * meaningful in full mode.
     */
    int initial_encoder_kbps() const {
      return initial_kbps;
    }

    /**
     * @brief Rate to pace each frame's packets at, in bits per second.
     *
     * A multiple of the whole budget (`tren_pacing`), so each frame leaves spread over part of
     * the frame interval instead of as one burst at line rate. Against a queue that another flow
     * keeps full, a burst loses a run of consecutive packets that no parity can repair; spread
     * out, the losses are scattered and the block parity covers them. Only in full mode, and 0
     * until the first controller step, which means the stream's default pacing.
     */
    double pacing_bps() const {
      return mode == mode_e::full ? pacing.load(std::memory_order_relaxed) : 0.0;
    }

    /**
     * @brief Whether the controller paces the stream's frames, which is only in full mode.
     */
    bool paces() const {
      return mode == mode_e::full;
    }

    /**
     * @brief The whole budget, in bits per second, for the stream to pay its frames at.
     *
     * Each frame is paced at `pacing_bps()`, but a frame may not start before the previous one
     * has been paid for at this rate. Frames of the size the budget allows are never held back;
     * frames larger than that —encoded for the budget before the last cut, or an encoder
     * overshooting at a scene change— wait on the host instead of flooding the path's queue.
     * Only in full mode, and 0 means no limit.
     */
    double budget_bps() const {
      return mode == mode_e::full ? budget.load(std::memory_order_relaxed) : 0.0;
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

  private:
    const mode_e mode;
    const int configured_kbps;
    const int framerate;
    const int payload_per_packet;
    const int wire_per_packet;

    mutable std::mutex lock;
    estab::Parametros params;
    estab::Control control;
    estab::gamestream::MedidorSenales meter;
    std::chrono::steady_clock::time_point next_enet_ping {};
    bool have_measurement = false;
    std::uint64_t frames_lost = 0;  ///< Frames the client reported lost, over the whole session.
    std::uint32_t last_frame_index = 0;  ///< The last frame handed to the meter.
    estab::Medicion last_measurement;

    std::chrono::steady_clock::time_point next_step {};
    std::chrono::steady_clock::time_point last_tick {};
    std::chrono::steady_clock::time_point hold_until {};
    std::chrono::steady_clock::time_point next_log {};
    std::chrono::steady_clock::time_point last_applied_at {};
    int last_applied_kbps = 0;
    int initial_kbps = 0;

    std::atomic<int> target_kbps {0};
    std::atomic<double> pacing {0.0};
    std::atomic<double> budget {0.0};
    std::atomic<double> queue {0.0};
    std::atomic<bool> idr_wanted {false};
    std::atomic<int> divider_changed {0};
    std::atomic<double> host_delay_s {0.0};  ///< Capture to send, the most of the last frames (host_delay).
    int divider = 1;  ///< Control thread only: the divider last handed to the video thread.
    int divider_before_alarm = 1;  ///< Control thread only: the divider the last alarm replaced.
    int fixed_kbps = 0;  ///< Control thread only: the bitrate of an encoder that cannot change it, 0 otherwise.
    double rate_keep = 1.0;  ///< Control thread only: the share of frames that brings that encoder down to the budget.
    double full_kbps = 0;  ///< Control thread only: what that encoder sends encoding every frame, learnt.

    /// The share of captured frames encoded: the fewer of the divider's and the budget's.
    double frames_kept() const {
      return std::min(1.0 / divider, rate_keep);
    }

    /// Hand the share of frames to the video thread, in thousandths.
    void publish_frames() {
      divider_changed.store(std::max(1, static_cast<int>(std::lround(1000.0 * frames_kept()))), std::memory_order_relaxed);
    }
    std::chrono::steady_clock::time_point divider_raised_at {};
    std::chrono::steady_clock::time_point last_idr_at {};  ///< The last IDR request answered.
    double last_rtt_s = 0;  ///< The last round trip ENet measured, in seconds.
    std::atomic<int> target_fec {-1};
    std::atomic<int> min_parity_now {2};

    // What the video thread needs from the last step to size each block's parity.
    std::mutex fec_lock;
    bool have_decision = false;
    double loss_estimate = 0;
    double burst_estimate = 1;
  };

}  // namespace adaptive_bitrate
