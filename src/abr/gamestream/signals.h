// From the signals GameStream already brings into the controller's Measurement.
//
// GameStream does not say when each packet or each frame arrived. What it does bring, without adding
// anything to the protocol (specs/propuesta-solucion/10-senales-gamestream.md):
//
//   · the round trip that ENet measures on the control channel, which crosses the same bottleneck as
//     the video —as long as the host has it measured: with the client's ping every
//     100 ms the host is never idle and never sends its own—;
//   · the FEC status of each frame that needed repair or was lost: how many data
//     and parity packets it had and how many arrived;
//   · the loss summary, every 50 ms, with the last frame that arrived whole and how many were
//     lost since the previous one. Moonlight sends it to Sunshine since what was
//     missing was implemented; a stock client does not send it, and the meter works without it.
//
// With that everything is built: the queue comes from the round trip, the loss and its burst from the FEC
// status, and what arrives from the advance of the last good frame, with the sizes the host knows.
//
// Like control.h: no network, no threads, no dependencies. It is tested in the bench and copied
// to Sunshine unedited.
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "../control.h"

namespace abr {
namespace gamestream {

/**
 * @brief What the host put on the wire for a frame. Host clock times, in µs.
 */
struct SentFrame {
  uint32_t index = 0;  ///< frame index
  int64_t  t_first_us = 0;  ///< when the first packet went out
  int64_t  t_last_us = 0;  ///< and the last
  int      packets = 0;  ///< data and parity
  int      data_packets = 0;  ///< of those, the data ones: the rest is the parity that went out
  double   rtp_bytes = 0;  ///< what the client will count of those packets
  double   wire_bytes = 0;  ///< that plus IP, UDP and encryption: what the bottleneck charges for
  /**
   * @brief in wire bytes, the size it would have had using the whole budget (used only by the
   *   dispersion measurement)
   */
  double   full_bytes = 0;
};

/**
 * @brief The FEC status of a frame, as moonlight-common-c sends it (SS_FRAME_FEC_STATUS).
 *
 * It only arrives for frames that needed parity or could not be reassembled. If it was
 * reassembled, the client sends it as soon as it gathers the data packets it needs: what was
 * received is just that and what has not arrived yet is not lost, so what is lost are the gaps
 * behind the highest received (missingPacketsBeforeHighestReceived; −1 if unknown).
 */
struct FecStatus {
  uint32_t index = 0;  ///< frame index
  int data_total = 0;  ///< data packets the frame has
  int parity_total = 0;  ///< parity packets the frame has
  int data_received = 0;  ///< data packets that arrived
  int parity_received = 0;  ///< parity packets that arrived
  /**
   * @brief missingPacketsBeforeHighestReceived: the gaps behind the highest received, -1 if
   *   unknown
   */
  int missing = -1;
};

/**
 * @brief The arrival of a frame, as reported by GameStream's Frame Stats message.
 *
 * The message is reserved in all generations of the protocol and was never implemented; its
 * content is this one. CLIENT clock times, truncated to 32 bits: they are never compared with the
 * host's except by difference.
 */
struct ReceivedTrain {
  uint32_t index = 0;  ///< frame index
  uint32_t t_first_us = 0;  ///< client clock when the first packet arrived, in µs
  uint32_t t_last_us = 0;  ///< client clock when the last packet arrived, in µs
  uint32_t bytes = 0;  ///< bytes the frame had
  uint16_t packets = 0;  ///< packets the frame had
};

/**
 * @brief The periodic loss summary (IDX_LOSS_STATS): frames lost since the previous one and the
 *   last one that arrived whole.
 */
struct LossSummary {
  uint32_t lost = 0;  ///< frames lost since the previous summary
  uint32_t last_good = 0;  ///< index of the last frame that arrived whole
};

/**
 * @brief Builds the controller's Measurement from the signals GameStream already carries.
 */
class SignalMeter {
 public:
  SignalMeter() = default;

  /**
   * @brief Record what the host put on the wire for a frame.
   *
   * @param c The frame.
   */
  void sent(const SentFrame& c);

  /**
   * @brief Record the FEC status of a frame.
   *
   * @param e The status.
   */
  void fec_status(const FecStatus& e);

  /**
   * @brief Record a loss summary.
   *
   * @param arrival_us Host clock when it arrived, in µs.
   * @param r The summary.
   */
  void summary(int64_t arrival_us, const LossSummary& r);

  /**
   * @brief Record the arrival of a frame (Frame Stats).
   *
   * With it the control measures the dispersion: how far the packets of a frame spread out when
   * crossing the bottleneck, which gives the pipe's width even if the frame is small. A client
   * that does not send it leaves the measurement as if without it.
   *
   * It also gives the video's one-way delay: the client clock at the first packet minus the host
   * clock when it left. The clocks are not aligned, so only its rise over the ten-second minimum
   * means anything: the queue in front of the video, without the uplink's part. Each frame is a
   * sample placed where a ping sent along with it would have come back.
   *
   * @param r The arrival.
   * @param queue Receives the queue over the floor, in seconds.
   * @param growth Receives how much the queue grew since the sample 50 ms before, in seconds.
   * @param sample_ms If given, receives the sample as a round trip in ms, or 0 if it was not taken.
   * @return False if there is nothing to compare with yet.
   */
  bool train(const ReceivedTrain& r, double& queue, double& growth, double* sample_ms = nullptr);

  /**
   * @brief Record the arrival of a frame, without asking for the queue.
   *
   * @param r The arrival.
   */
  void train(const ReceivedTrain& r) { double queue, growth; train(r, queue, growth); }

  /**
   * @brief Record ENet's round trip for the control channel's peer, as the host reads it.
   *
   * One below half the floor is not a shorter path but a broken reading —on Windows a round trip
   * of 1 ms shows up now and then—, and as a floor it poisons ten seconds of queue: it is
   * discarded, unless it repeats, which is what a path that really got shorter would do.
   *
   * @param seconds The round trip, in seconds.
   */
  void rtt(double seconds);

  /**
   * @brief Set the audio rate that goes out with the video.
   *
   * @param bps Bits/s on the wire.
   */
  void audio(double bps) { audio_bps_ = bps; }

  /**
   * @brief Record that the client asked for a keyframe.
   *
   * Until it arrives, the client discards the whole frames it receives —it has nothing to decode
   * them from— and the summary counts them lost: those are not network loss, and do not enter the
   * count.
   *
   * @param next_frame Index of the next frame to go out.
   */
  void idr_request(uint32_t next_frame) { if (!idr_pending_) { idr_pending_ = true; idr_from_ = next_frame; } }

  /**
   * @brief Record the client's periodic ping.
   *
   * How far its arrival drifts from the client's clock is the uplink's queue, which the summaries
   * come back through.
   *
   * @param arrival_us Host clock when it arrived, in µs.
   * @param client_ms The client's monotonic clock, in milliseconds.
   */
  void client_ping(int64_t arrival_us, uint32_t client_ms);

  /**
   * @brief How much the queue rises and falls by itself (Measurement::noise).
   *
   * @return Seconds.
   */
  double noise() const { return noise_; }

  /**
   * @brief Choose which optional parts the meter uses: the queue from the age of the last good
   *   frame (`queue_age`) and what was delivered over the same frames (`delivery_full` > 0).
   *
   * @param p The parameters.
   */
  void configure(const Params& p) {
    use_age_ = p.queue_age != 0;
    age_recent_us_ = static_cast<int64_t>(p.age_recent * 1e6);
    cadence_max_us_ = p.age_cadence > 0 ? static_cast<int64_t>(p.age_cadence * 1e6 / p.fps) : 0;
    use_same_frames_ = p.delivery_full > 0;
    prior_queue_max_ = p.alarm_prior_queue;
    alarm_intervals_ = p.alarm_intervals;
    prior_us_ = static_cast<int64_t>(p.alarm_prior_s * 1e6);
  }

  /**
   * @brief Build the Measurement with what arrived since the previous call.
   *
   * @param now_us Host clock, in µs.
   * @param m Receives the measurement.
   * @return False if there is not yet anything to build it with: no round trip measured nor frames sent.
   */
  bool measure(int64_t now_us, Measurement& m);

  /**
   * @brief Whether at least one loss summary arrived.
   *
   * With a stock client none does, and what arrives is estimated instead of measured.
   *
   * @return True if a summary arrived.
   */
  bool has_summary() const { return had_summary_; }

 private:
  std::map<uint32_t, SentFrame> sent_;
  struct Interval { int64_t t_us; double span_s; double bits; };
  std::deque<Interval> window_rx_, window_tx_, window_data_;
  std::deque<Interval> window_packets_tx_, window_packets_lost_;
  double bits_sent_ = 0, bits_data_ = 0;            // since the previous call
  double packets_tx_ = 0, data_tx_ = 0;
  double packets_lost_ = 0;                          // from the FEC status and the lost frames
  double frames_with_loss_ = 0, lost_in_frames_ = 0;
  int      failed_with_status_ = 0;                  // frames with status that were not reassembled
  double   bits_arrived_ = 0;                        // from the advance of the last good frame
  int64_t  summary_arrival_us_ = 0;
  int64_t  resumed_us_ = INT64_MIN / 2;              // when the summaries came back after a silence
  uint32_t last_good_ = 0;
  bool     had_summary_ = false;
  bool     idr_pending_ = false;                     // waiting for the requested keyframe
  uint32_t idr_from_ = 0;                            // the first one that could have been that frame
  int64_t  last_call_us_ = 0;
  double   rtt_ = 0;
  double   audio_bps_ = 0;
  double   packets_per_frame_ = 0;                   // mean, for the frames that were lost whole

  // The delivery over the same frames: the recent summaries, and the last they gave
  struct Summary { int64_t arrival_us; uint32_t good; double lost; };
  double   lost_total_ = 0;
  std::deque<Summary> summaries_;
  double   same_frames_rate_ = 0;                    // bits/s delivered, draft-cheng
  double   delivery_ = 0;                            // output span over arrival span
  int64_t  t_same_frames_us_ = 0;

  // The age of the last good frame and the ping's drift, with their ten-second minima
  struct Sample { int64_t t_us; double v; };
  std::deque<Sample> ages_, ping_drifts_, rtts_, raw_;
  // The path noise: how much each ENet sample stands out over those around it (the earlier
  // and later ones), in the last fifteen seconds. See update_noise.
  std::deque<Sample> excursions_;
  int64_t  noise_center_us_ = 0;                     // the last sample already judged
  double   noise_ = 0;                               // seconds, the 90th percentile
  void     update_noise(int64_t t_us);
  // The draining of each queue: the ENet samples from its peak until it returns to the floor. See
  // track_drain.
  struct Descent { int64_t t_us; int64_t out_us; double ms; };
  std::vector<Descent> descent_;
  bool     in_queue_ = false;
  int64_t  queue_since_us_ = 0;
  double   drain_rate_ = 0;                          // bits/s, the last one not yet reported
  int64_t  drain_start_us_ = 0;
  int64_t  drain_peak_us_ = 0;                       // when the peak's ping went out
  double   queue_since_ms_ = 0, drain_height_ = 0;    // the round trip when the queue started, and how much it rose
  void     track_drain(int64_t t_us, double ms, double delay_floor);
  double   bits_between(int64_t from_us, int64_t to_us) const;
  bool     use_age_ = false;
  int64_t  age_recent_us_ = 150000;
  int64_t  cadence_max_us_ = 41667;                  // the gap between frames that is still cadence
  int      alarm_intervals_ = 2;
  int64_t  prior_us_ = 0;
  bool     use_same_frames_ = false;
  double   prior_queue_max_ = 0.025;                 // the queue an alarm step can start from
  int64_t  t_enet_floor_us_ = 0;                     // the last ENet sample that touched the floor
  bool     plausible(double ms, std::deque<Sample>& history, int& streak);
  double   upstream_ms(int64_t t_us) const;
  bool     queue_sample(int64_t t_us, double ms, double& queue, double& growth, double* sample_ms);
  std::deque<Sample> delays_;                        // the video's one-way delay per frame, ten seconds
  uint32_t delay_ref_ = 0;                           // the first frame's, for the clocks' offset
  double   delay_base_ms_ = 0;                       // the round trip when that first one arrived
  bool     has_delay_ref_ = false;
  double   stall_ms_ = 0;                            // the last rise of the round trip that the video did not show
  int64_t  stall_us_ = INT64_MIN / 2;
  std::vector<ReceivedTrain> trains_rx_;             // the arrivals since the previous measurement
  int      rtt_low_ = 0;            // consecutive readings below half the floor
};

}  // namespace gamestream
}  // namespace abr
