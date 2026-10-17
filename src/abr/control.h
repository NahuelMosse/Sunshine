// THE THEORY.
//
// A measurement comes in, a decision comes out. No network, no threads, no I/O, no
// dependencies beyond the standard library. That is not purism: it is what lets this file be
// copied to Sunshine/src/ unedited, compiled to WebAssembly for the presentation's
// simulator, and linked into the bench's UDP sender — all three running exactly the same code,
// which is the only way to keep them from drifting apart.
//
// Everything numeric lives in Params. There is not a single loose constant here.
#pragma once

#include <deque>
#include <vector>

#include "params.h"

namespace abr {

// All the controller knows about the world. All of it arrives LATE: the client's report
// also has to travel. That lateness is τ, and it sets the speed of the loop.
struct Measurement {
  double t = 0;              // host time, seconds
  double owd_min = 0;        // smallest one-way delay of the reported period
  double owd_mean = 0;       // its average
  double loss = 0;           // fraction of packets the client did not see
  double rtt = 0;            // round trip, measured with an echo
  double received_rate = 0;  // bits/s the client counted. The one that allows dropping to a
                             // measured value instead of an invented fraction
  double delivery = 0;       // for the same frames, how long they took to leave over how long they
                             // took to arrive: below 1 is a pipe that slows them down. 0: unknown
  int    samples = 0;        // how many packets back the above

  // What the host knows WITHOUT the client's help: how many bits it actually put on the wire.
  // If that is far below the budget, the encoder had nothing to encode and the measurement
  // just taken says nothing about the network.
  double sent_rate = 0;

  // The dispersion of the last frame's packet train, for the SQP variant:
  // how many bytes it had and how long it took to cross, from its first packet to its last.
  double train_bytes = 0;
  double train_span = 0;     // T_end − T_start, measured at the client
  double train_full_bytes = 0;  // the size that frame would have had without undershoot
  // All the trains of the period, if whoever builds the Measurement has them; otherwise the one
  // above stands. With only one per period you lose exactly the one that matters: the probe
  // frame is almost never the last.
  struct Train { double bytes = 0, span = 0; };
  std::vector<Train> trains;
  // The parity that actually went out, over the data, in the last cycle. The host knows it
  // without the client's help. With small frames it is not the requested one: the per-block minimum rules.
  double parity_sent = 0;
  // Lost packets per frame that lost any, in the period: the length of a burst.
  // Zero if there was no loss or it is unknown.
  double loss_burst = 0;
  // How many reports arrived after their frame was already given up for lost, since startup.
  // For the log only: if it grows, the blackout deadline is too short for this path.
  int late_reports = 0;
  // What the encoder delivered, in bits/s of picture data —without parity or network
  // headers—. Zero if unknown.
  double data_rate = 0;
  // How much the queue rises and falls by itself on a real path (Wi-Fi, internet), in seconds: it is not
  // our own queue. Zero if unknown.
  double noise = 0;
  // The pipe that emptied the last queue, in bits/s: what the host sent between two round trips that
  // found it full, over how long they took to come back. While there is a queue the bottleneck
  // never idles, so that is what it delivers. Zero if no queue has emptied since the previous measurement.
  double drain_rate = 0;
  double drain_start = 0;    // when that queue started to fill, host time in seconds
  double drain_rise = 0;     // how long it took to reach its peak, in seconds
  double drain_height = 0;   // and how far it rose by then, in seconds of queue
};

// Why it moved. Without this, debugging the loop is guesswork (ch. 9).
enum class Reason {
  startup,
  steady,           // on target, nothing new
  no_evidence,      // the encoder did not fill the budget: the sample says nothing
  queue_growing,    // the growth term took over
  queue_high,       // the level corrector
  loss_backoff,     // overflow: high loss with a full queue
  ramping,          // climbing along the S curve
  reanchored,       // lowering did not help: it was not a queue
  probing,          // deliberate cut to re-read the floor
  capped,
};

const char* name(Reason m);

// The decision, plus everything chapter 9 says to record. The audit fields are not
// optional: retuning is the first thing to break, and without seeing it you do not notice.
struct Decision {
  double budget = 0;         // bits/s on the wire, whole
  double video_rate = 0;     // of that, what is left for the picture
  // What to ask the encoder for so that it delivers `video_rate`. It is the picture rate divided
  // by what the encoder has been delivering of what it is asked (see
  // `encoder_compensation_max`); without compensation, equal to `video_rate`.
  double encoder_request = 0;
  double repair = 0;         // fraction of parity
  Reason reason = Reason::startup;
  bool   app_limited = false;   // the encoder is not using the budget
  double train_bandwidth = 0;   // the dispersion estimate, if the variant uses it

  double queue = 0;          // what the controller believes the queue to be
  double delay_floor = 0;    // the reference it computed it against
  double dref = 0;           // and the current target
  double growth = 0;         // the per-cycle derivative, already filtered
  double dead_zone = 0;      // the yardstick the level is measured with
  double C_est = 0;          // estimated capacity
  double tau_est = 0;        // estimated dead time
  double kg = 0, kp = 0;     // the gains that came out of the two above
  double loss_est = 0;
  double loss_burst = 1;       // packets each stumble takes away, for the parity
  int    min_parity = 2;       // the minimum parity per block (fec_min_clean)
  int    samples = 0;        // how much evidence the decision rested on
  bool   probing = false;
  bool   reanchored = false;
  bool   capped = false;
};

// How many parity packets a block of k data packets carries when the repair is decided in
// packets (`parity_in_packets`). It is the smallest m —with the minimum the client asks for— that keeps
// the probability of losing the block below `parity_block_failure`, if the loss is
// `loss` and arrives in stumbles of `burst` consecutive packets: stumbles start with
// probability loss/burst per packet, and the block withstands as many as fit in its parity.
//
// Capped by `fec_max`. Without the cap, the loss of an overflowing queue —congestion loss— is
// answered with more parity, and that adds load to the pipe that is failing. Telling the
// overflow apart by the queue is not enough: with a download in the pipe the delay floor
// is poisoned and the queue looks low.
//
// The host calls it for each block, with `loss_est`, `loss_burst` and `min_parity` of the
// last decision; a negative `minimum` means `fec_min_packets`.
int block_parity(int k, double loss, double burst, const Params& p, int minimum = -1);

class Control {
 public:
  explicit Control(const Params& p);

  // Leaves the loop as freshly started. `negotiated` is the ceiling the client asked for.
  void reset(double negotiated);

  // Called every Tc with the latest that arrived from the client.
  Decision step(const Measurement& m);

  // Between two steps, with each round-trip sample: that sample's queue over its floor and
  // how much it grew in the last 50 ms, in seconds, and the latest measured delivery. If the
  // queue jumped by a step only a narrow pipe explains, it cuts the budget right there —without
  // waiting for the step— and returns true; the new budget is the one from budget().
  bool alarm(double t, double queue, double growth, double received);
  // With the sample following an alarm: if the queue is already back at its floor, the step was a
  // delayed ack and not a narrowing pipe —the queue of a narrow pipe does not empty in one sample—, and
  // the loop returns to how it was before the alarm. Returns true if it undid it; the budget
  // goes back to being the one from budget().
  bool retract_alarm(double t, double queue);
  double budget() const { return R_ * p_.utilization; }
  // Whether the alarm also asks for a keyframe right away (keyframe_on_drop). With a narrow
  // pipe, what left in the last hundred milliseconds is being lost, and the client notices it
  // only when the first whole frame reaches it, a third of a second later; from then on it
  // discards everything up to the keyframe it asks for. Sending it now makes that first frame be it.
  bool alarm_requests_keyframe() const { return p_.keyframe_on_drop != 0; }
  // The current minimum parity per block; the alarm raises it again right away.
  int min_parity() const { return min_parity_; }
  // The frames per second the host actually encodes (frames_in_flight): the minimum parity is
  // a per-frame cost, and counting it at the nominal rate left the picture on the floor.
  void set_fps(double fps) { fps_ = fps > 0 ? fps : p_.fps; }

  // For the ultimate-gain experiment: multiplies k_g and k_p by the same factor.
  // The model predicts the loop starts to oscillate steadily at a
  // specific value — it is the prediction that can kill it.
  void scale_gains(double factor) { scale_ = factor; }

  // For the estimation-error experiment: forces Ĉ instead of deducing it.
  void force_capacity(double bps) { C_forced_ = bps; }

  // Naive mode: no growth term, with fixed gains. It is chapter 4's counterfactual
  // — two integrations without a derivative.
  void naive_mode(bool on) { naive_ = on; }

  const Params& params() const { return p_; }
  const Tuning&   tuning() const { return s_; }

 private:
  struct Sample { double t = 0; double v = 0; };

  static void   prune(std::deque<Sample>& v, double t, double window);
  static double percentile(const std::deque<Sample>& v, int q);
  // The window value closest to a past instant. Returns 0 if the window is
  // empty; if the requested instant falls before what is kept, returns the oldest.
  static double value_at(const std::deque<Sample>& v, double t);

  // Ch. 6: average while there are few samples, line fit from ~3 on.
  // The median is no good for slopes. Returns the slope and leaves the residual,
  // which is where the dead-zone width comes from.
  double fit_slope(double& residual_rms) const;
  double filter_time_const(double rtt) const;

  void   estimate(const Measurement& m, double queue, double dref);
  bool   check_coherence(double queue, double dref, double delta, double dR);
  void   probe_delay_floor(double t);
  void   ramp_up_step(double dref, double queue);
  void   anti_windup(double ceiling);

  Params p_;
  Tuning   s_;

  // the loop
  double R_ = 0;             // the budget, the only thing it decides
  double f_ = 0;             // the repair
  double R_healthy_ = 0;     // last budget with the queue truly empty
  double C_healthy_ = 0;     // and the capacity estimated at that moment
  // A queue episode: what there was before and how much of it the queue was someone else's (cross_s)
  struct Episode {
    bool   active = false;
    double t0 = 0, R = 0, C = 0;
    double cross = 0;        // seconds with the queue high, still, and not explained by our own traffic
  } episode_;
  bool   after_cross_ = false;  // the last queue belonged to someone else and ended: the calm is explored right away
  double queue_prev_ = 0;
  double raw_queue_prev_ = 0;    // the raw queue sample from the previous step
  double raw_queue_prev2_ = 0;   // and the one from two steps back
  bool   had_queue_ = false;
  bool   first_ = true;
  double t_ = 0;

  // the signal
  std::deque<Sample> delay_floor_;  // one-way delay floor window
  std::deque<Sample> trips_;     // round-trip window
  std::deque<Sample> rates_;     // received-rate window
  // What the host put on the wire, with its time. It has to be kept, not just the latest:
  // the client's report arrives a trip late, so the rate it reports corresponds to
  // what was sent a trip earlier. Comparing it against what is being sent NOW
  // makes every rate increase look like loss.
  std::deque<Sample> sent_rates_;
  std::deque<Sample> queues_;    // short window, for the line fit
  std::deque<Sample> minima_;    // the raw queue of the last noise_window s: its lower envelope
  std::deque<Sample> budgets_;        // R over the last seconds, to return to what it was before a queue
  double delay_floor_cur_ = 0;
  double delay_floor_fresh_ = 0;
  bool   competitor_ = false;  // the delay in the window has another flow's swing (competitor_threshold)
  double t_no_swing_ = 0;      // the last time the window did not have that swing
  double loss_smooth_ = 0;
  double loss_peak_ = 0;       // the parity one: the last peak, which is forgotten slowly
  double t_loss_ = -1e9;       // the last time there was loss, or an alarm
  int    min_parity_ = 2;
  double fps_ = 60;              // the frames per second encoded
  double t_burst_ = -1e9;      // when the last loss burst started
  bool   in_burst_ = false, recurring_ = false;
  double t_start_ = -1;        // the time of the first step
  double loss_burst_ = 0;      // length of the channel's bursts, not counting those from overflow
  double C_est_ = 0, tau_est_ = 0;
  double C_forced_ = 0;
  double scale_ = 1.0;
  bool   naive_ = false;

  // the ramp-up curve
  double anchor_ = 0, ramp_t_ = 0, ramp_R0_ = 0;
  double t_last_queue_ = 0;      // the last time there was a queue
  bool   in_ramp_ = false;

  // the delay floor probe
  bool   probing_ = false;
  double probe_until_ = 0;
  int    steady_ = 0;

  // the frame that does not fill the budget
  bool   app_limited_ = false;
  // More reaches the wire than comes out the other end: there is a queue or there are drops, and in
  // both cases the pipe is full. It is the only thing that stays true when the delay
  // floor has been poisoned.
  bool   pipe_full_ = false;
  double train_bandwidth_ = 0;
  double t_bandwidth_ = 0;        // when the bandwidth was last updated
  double sent_then_ = 0;          // what was sent a trip before the current report
  double sent_smooth_ = 0, received_smooth_ = 0;    // the two, smoothed to compare them
  double t_fast_backoff_ = -1e9;  // when the fast backoff last fired
  int shortfall_streak_ = 0;      // consecutive cycles in which noticeably less arrived than was sent
  double t_loss_no_queue_ = -1;     // since when there has been excess loss with no queue visible
  double blind_until_ = -1e9;       // until when the delay cannot be trusted as a queue signal
  bool startup_done_ = false;       // the first queue or loss of the session has already appeared
  double delivery_smooth_ = 0;      // the delivery of the same frames, smoothed to half a second
  double dref_last_ = 0;            // the queue target of the last step, for the alarm
  double t_alarm_ = -1e9;
  std::deque<Sample> delivered_;   // what arrived in the last second, to leave exploration
  double t_cruise_ = 0;            // since when the path has been cruising
  bool   exploring_ = false;       // multiplicative ramp-up while cruising          // when the in-between-steps alarm last fired
  double encoder_efficiency_ = 1;  // what the encoder delivers over what it is asked
  double request_prev_ = 0;       // what was asked of it in the previous step
  int    queue_over_target_ = 0;    // consecutive steps with the queue above the target
  // What the last alarm changed, to undo it if the next sample retracts it
  struct BeforeAlarm {
    double R = 0, anchor = 0, t_alarm = 0, t_loss = 0, t_fast_backoff = 0;
    bool in_ramp = false, startup_done = false;
    int min_parity = 0;
  };
  BeforeAlarm before_alarm_;
  bool alarm_open_ = false;
  // The trains of the window: how many bits each frame had and how long it took to cross. The
  // bandwidth comes from the slope between the two, not from each quotient separately.
  struct Train { double t = 0; double bits = 0; double span = 0; };
  std::deque<Train> trains_;
  // The dispersion bandwidth of the window; false if no frame managed to spread out.
  bool bandwidth_by_regression(double& bandwidth) const;

  // the coherence check
  struct Trial { bool active = false; double t = 0; double R = 0; double queue = 0; };
  Trial trial_;
  int    suspicion_ = 0;
};

}  // namespace abr
