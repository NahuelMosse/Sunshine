/**
 * @file control.h
 * @brief The bitrate controller: a measurement comes in, a decision comes out.
 *
 * No network, no threads, no I/O, no dependencies beyond the standard library, so the same
 * source builds in every host that embeds it. Every number it uses lives in params.h.
 */
#pragma once

#include "params.h"

#include <deque>
#include <vector>

namespace abr {

  /**
   * @brief All the controller knows about the world.
   *
   * All of it arrives LATE: the client's report also has to travel. That lateness is τ, and it sets
   * the speed of the loop.
   */
  struct Measurement {
    double t = 0;  ///< host time, seconds
    double owd_min = 0;  ///< smallest one-way delay of the reported period
    double owd_mean = 0;  ///< its average
    double loss = 0;  ///< fraction of packets the client did not see
    double rtt = 0;  ///< round trip, measured with an echo
    /**
     * @brief bits/s the client counted. The one that allows dropping to a measured value instead
     *   of an invented fraction
     */
    double received_rate = 0;
    /**
     * @brief for the same frames, how long they took to leave over how long they took to arrive:
     *   below 1 is a pipe that slows them down. 0: unknown
     */
    double delivery = 0;
    int samples = 0;  ///< how many packets back the above

    /**
     * @brief What the host knows WITHOUT the client's help: how many bits it actually put on the
     *   wire. If that is far below the budget, the encoder had nothing to encode and the
     *   measurement just taken says nothing about the network.
     */
    double sent_rate = 0;

    /**
     * @brief The dispersion of one frame's packet train.
     */
    struct Train {
      double bytes = 0;  ///< bytes the frame had
      double span = 0;  ///< how long it took to cross, in seconds
    };

    /**
     * @brief All the trains of the period. With only one per period you lose exactly the one that
     *   matters: the frame that shows a narrower pipe is almost never the last.
     */
    std::vector<Train> trains;
    /**
     * @brief The parity that actually went out, over the data, in the last cycle. The host knows
     *   it without the client's help. With small frames it is not the requested one: the per-block
     *   minimum rules.
     */
    double parity_sent = 0;
    /**
     * @brief Lost packets per frame that lost any, in the period: the length of a burst. Zero if
     *   there was no loss or it is unknown.
     */
    double loss_burst = 0;
    /**
     * @brief The rate a frame that crossed without spreading out proves the pipe carries at least,
     *   in bits/s on the wire; 0 if no frame proved it.
     */
    double proof_bps = 0;
    /// The same with frames of at least kQuietFrameMinPacketsBack packets, for the way back from a cut.
    double proof_back_bps = 0;
    /**
     * @brief What the encoder delivered, in bits/s of picture data —without parity or network
     *   headers—. Zero if unknown.
     */
    double data_rate = 0;
    /**
     * @brief How much the queue rises and falls by itself on a real path (Wi-Fi, internet), in
     *   seconds: it is not the loop's own queue. Zero if unknown.
     */
    double noise = 0;
  };

  /**
   * @brief Why the budget moved.
   *
   * Without this, debugging the loop is guesswork.
   */
  enum class Reason {
    startup,  ///< the first steps of the session
    steady,  ///< on target, nothing new
    no_evidence,  ///< the encoder did not fill the budget: the sample says nothing
    queue_growing,  ///< the growth term took over
    queue_high,  ///< the level corrector
    loss_backoff,  ///< overflow: high loss with a full queue
    ramping,  ///< climbing along the S curve
    reanchored,  ///< lowering did not help: it was not a queue
    capped,  ///< the budget ran into the bitrate bounds or the proven ceiling
  };

  /**
   * @brief Name of a reason.
   *
   * @param m The reason.
   * @return The name the reason goes by in logs.
   */
  const char *name(Reason m);

  /**
   * @brief The decision, plus everything needed to audit it.
   *
   * The audit fields are not optional: retuning is the first thing to break, and without seeing it
   * you do not notice.
   */
  struct Decision {
    double budget = 0;  ///< bits/s on the wire, whole
    double video_rate = 0;  ///< of that, what is left for the picture
    /**
     * @brief What to ask the encoder for so that it delivers `video_rate`. It is the picture rate
     *   divided by what the encoder has been delivering of what it is asked (see
     *   `encoder_compensation_max`); without compensation, equal to `video_rate`.
     */
    double encoder_request = 0;
    double repair = 0;  ///< fraction of parity
    Reason reason = Reason::startup;  ///< why the budget moved
    bool app_limited = false;  ///< the encoder is not using the budget
    double train_bandwidth = 0;  ///< the dispersion estimate, if the variant uses it

    double queue = 0;  ///< what the controller believes the queue to be
    double delay_floor = 0;  ///< the reference it computed it against
    double dref = 0;  ///< and the current target
    double growth = 0;  ///< the per-cycle derivative, already filtered
    double dead_zone = 0;  ///< the yardstick the level is measured with
    double C_est = 0;  ///< estimated capacity
    double parity_failure = 0;  ///< the block failure target the parity is sized for
    double tau_est = 0;  ///< estimated dead time
    double kg = 0;  ///< gain on the queue growth, derived from C_est and tau_est
    double kp = 0;  ///< gain on the queue level, derived from C_est and tau_est
    double loss_est = 0;  ///< the loss the repair is decided with
    double loss_burst = 1;  ///< packets each stumble takes away, for the parity
    int min_parity = 2;  ///< the minimum parity per block (fec_min_clean)
    int samples = 0;  ///< how much evidence the decision rested on
    bool reanchored = false;  ///< the ramp-up curve was moved to a new anchor in this step
    bool capped = false;  ///< the budget ran into the bitrate bounds or the proven ceiling
  };

  /**
   * @brief How many parity packets a block of k data packets carries when the repair is decided in
   *   packets (`parity_in_packets`).
   *
   * It is the smallest m —with the minimum the client asks for— that keeps the probability of
   * losing the block below `parity_block_failure`, if the loss is `loss` and arrives in stumbles of
   * `burst` consecutive packets: stumbles start with probability loss/burst per packet, and the
   * block withstands as many as fit in its parity.
   *
   * Capped by `fec_max`. Without the cap, the loss of an overflowing queue —congestion loss— is
   * answered with more parity, and that adds load to the pipe that is failing. Telling the overflow
   * apart by the queue is not enough: with a download in the pipe the delay floor is poisoned and
   * the queue looks low.
   *
   * The host calls it for each block, with `loss_est`, `loss_burst` and `min_parity` of the last
   * decision.
   *
   * @param k Data packets in the block.
   * @param loss Estimated loss fraction.
   * @param burst Consecutive packets each loss stumble takes away.
   * @param p The parameters.
   * @param minimum Minimum parity packets; a negative value means `fec_min_packets`.
   * @return Parity packets for the block.
   */
  int block_parity(int k, double loss, double burst, const Params &p, int minimum = -1);

  /**
   * @brief Parity packets for a whole frame of k data packets.
   *
   * The frame is split into blocks the way the host does it —each block with its own parity fits the
   * Galois field, at most four of them— and `block_parity` is summed over the blocks.
   *
   * @param k Data packets in the frame.
   * @param loss Estimated loss fraction.
   * @param burst Consecutive packets each loss stumble takes away.
   * @param p The parameters.
   * @param minimum Minimum parity packets per block; a negative value means `fec_min_packets`.
   * @param repair The parity fraction the host announces, which sets the largest block.
   * @return Parity packets for the frame.
   */
  int frame_parity(int k, double loss, double burst, const Params &p, int minimum, double repair);

  /**
   * @brief The block failure target the parity is sized for, given the round-trip floor.
   * @param p The parameters.
   * @param trip_base The round-trip floor, in seconds.
   * @return The probability of a failed block.
   */
  double block_failure_target(const Params &p, double trip_base);

  /**
   * @brief Whether a frame's proof shows room: the queue nearly empty for a while, and its goal
   *   above the budget by the margin.
   * @param p The parameters.
   * @param raw_queue The queue now, in seconds.
   * @param dref The queue target, in seconds.
   * @param idle_s Seconds since the last queue.
   * @param goal_bps The budget the proof allows, in bit/s.
   * @param budget_bps The budget now, in bit/s.
   * @param margin The multiple of the budget the goal has to clear.
   * @return Whether the proof counts.
   */
  bool quiet_frame_proof_room(const Params &p, double raw_queue, double dref, double idle_s, double goal_bps, double budget_bps, double margin);

  /**
   * @brief The bitrate controller: a measurement comes in, a decision comes out.
   */
  class Control {
  public:
    /**
     * @brief Create a controller.
     *
     * @param p The parameters.
     */
    explicit Control(const Params &p);

    /**
     * @brief Build a controller started for a given negotiated ceiling.
     *
     * @param p The parameters.
     * @param negotiated The ceiling the client asked for, in bits/s.
     */
    Control(const Params &p, double negotiated);

    /**
     * @brief Leave the loop as freshly started.
     *
     * @param negotiated The ceiling the client asked for, in bits/s.
     */
    void reset(double negotiated);

    /**
     * @brief Take one step of the loop. Called every Tc with the latest that arrived from the client.
     *
     * @param m The latest measurement.
     * @return The decision, with the audit fields.
     */
    Decision step(const Measurement &m);

    /**
     * @brief Check a round-trip sample between two steps.
     *
     * If the queue jumped by a step only a narrow pipe explains, the budget is cut right there
     * —without waiting for the step— and the new budget is the one from budget().
     *
     * @param t Host time, in seconds.
     * @param queue The sample's queue over its floor, in seconds.
     * @param growth How much the queue grew in the last 50 ms, in seconds.
     * @param received The latest measured delivery.
     * @return True if the alarm fired.
     */
    bool alarm(double t, double queue, double growth, double received);

    /**
     * @brief Check the sample following an alarm.
     *
     * If the queue is already back at its floor, the step was a delayed ack and not a narrowing
     * pipe —the queue of a narrow pipe does not empty in one sample—, and the loop returns to how it
     * was before the alarm. The budget goes back to being the one from budget().
     *
     * @param t Host time, in seconds.
     * @param queue The sample's queue over its floor, in seconds.
     * @return True if the alarm was undone.
     */
    bool retract_alarm(double t, double queue);

    /**
     * @brief The budget the loop currently allows.
     *
     * @return Bits/s on the wire, after the utilization.
     */
    double budget() const {
      return R_;
    }

    /**
     * @brief The current minimum parity per block; the alarm raises it again right away.
     *
     * @return Parity packets per block.
     */
    int min_parity() const {
      return min_parity_;
    }

    /**
     * @brief Set the frames per second the host actually encodes .
     *
     * The minimum parity is a per-frame cost, and counting it at the nominal rate left the picture
     * on the floor.
     *
     * @param fps Frames per second; a value that is not positive restores the nominal rate.
     */
    void set_fps(double fps) {
      fps_ = fps > 0 ? fps : p_.fps;
    }

    /**
     * @brief Tell the controller whether the session's encoder repairs a lost frame by invalidating
     *   references (Params::repairs_by_invalidation): only then may the parity minimum drop on a clean path.
     *
     * @param repairs True once the encoder reports a repair reach above 0.
     */
    void set_repairs_by_invalidation(bool repairs) {
      p_.repairs_by_invalidation = repairs ? 1 : 0;
    }

    /**
     * @brief The lowest budget: `bitrate_min`, or what the frames cost on the wire if that is more.
     *
     * A frame is at least one packet, and with parity on the client asks for `fec_min_packets` more per
     * block, all of them full size. That cost is per frame: 1.9 Mbit/s with the audio at 60 fps, 3.4 at
     * 120. A budget below it is not what goes out: the frames keep their size, the host's buffer
     * overflows and its drops read as loss, which keeps the parity at its minimum.
     *
     * @return Bits per second.
     */
    double bitrate_floor() const {
      return std::max(p_.bitrate_min, fps_ * (p_.mtu + p_.header_bytes) * 8.0 * (1 + p_.fec_min_packets) + p_.audio_bps);
    }

    /**
     * @brief The parameters the controller runs with.
     *
     * @return The parameters.
     */
    const Params &params() const {
      return p_;
    }

    /**
     * @brief The gains derived from the parameters.
     *
     * @return The gains.
     */
    const Tuning &tuning() const {
      return s_;
    }

  private:
    struct Sample {
      double t = 0;
      double v = 0;
    };

    static void prune(std::deque<Sample> &v, double t, double window);
    bool trip_follows_step(double now, double step) const;
    static double percentile(const std::deque<Sample> &v, int q);
    // The window value closest to a past instant. Returns 0 if the window is
    // empty; if the requested instant falls before what is kept, returns the oldest.
    static double value_at(const std::deque<Sample> &v, double t);
    // Same as percentile, over the samples no older than `window` before `t`.
    static double percentile_within(const std::deque<Sample> &v, int q, double t, double window);

    // Average while there are few samples, line fit from ~3 on.
    // The median is no good for slopes. Returns the slope and leaves the residual,
    // which is where the dead-zone width comes from.
    double fit_slope(double &residual_rms) const;
    double filter_time_const(double rtt) const;

    void start(double negotiated);
    void estimate(const Measurement &m, double queue, double dref);
    void ramp_up_step();
    void anti_windup(double ceiling);

    // What one step carries from each phase to the next.
    struct Cycle {
      double raw_queue = 0;  // the queue of this sample over the delay floor
      double queue = 0;  // the queue the law uses: the mean of the fitted window
      double delta = 0;  // its growth per cycle
      double residual = 0;  // the residual of the line fit
      double delay_floor_min = 0;  // the window's minimum, before any competitor correction
      double dref = 0;  // the queue target
      double tau = 0;  // the dead time the gains are derived with
      double kg = 0, kp = 0;  // the gains
      double dead_zone = 0;
      double dR = 0;  // the law's correction
      Reason reason = Reason::steady;
      bool startup = false;  // the first step: the law does not act
      double recent_bits = 0, recent_span = 0;  // the trains of this report
      int recent_trains = 0;
      double recent_pipe = 0;  // the pipe the last frames crossed, if it could be measured
      double before = 0;  // the budget before the law's correction
      bool has_queue = false;
      bool loss_evidence = false;  // the loss says the pipe is full
      double overhead = 0;  // payload over wire bytes
      double video_rate = 0;
      Decision d;
    };

    // The phases of step(), in the order it runs them
    void dispersion(const Measurement &m, Cycle &c);
    void delay_floor(const Measurement &m, Cycle &c);
    void loss(const Measurement &m);
    void queue_target(const Measurement &m, Cycle &c);
    void control_law(Cycle &c);
    void back_off_to(const Measurement &m, Cycle &c);
    void apply(const Measurement &m, Cycle &c);
    void ramp(const Measurement &m, Cycle &c);
    void cross_queue(const Measurement &m, Cycle &c);
    void ceilings(const Measurement &m, Cycle &c);
    void parity_budget(Cycle &c);
    void encoder_compensation(const Measurement &m, Cycle &c);
    void audit(const Measurement &m, Cycle &c);

    Params p_;
    Params parity_p_;  // p_ with the block failure target of the current round trip
    Tuning s_;

    // the loop
    double R_ = 0;  // the budget, the only thing it decides
    double R_healthy_ = 0;  // last budget with the queue truly empty
    double C_healthy_ = 0;  // and the capacity estimated at that moment

    // A queue episode: what there was before and how much of it the queue was someone else's (kCrossS)
    struct Episode {
      bool active = false;
      double t0 = 0, R = 0, C = 0;
      double cross = 0;  // seconds with the queue high, still, and not explained by the loop's own traffic
    } episode_;

    bool after_cross_ = false;  // the last queue belonged to someone else and ended: the calm is explored right away
    double raw_queue_prev_ = 0;  // the raw queue sample from the previous step
    double raw_queue_prev2_ = 0;  // and the one from two steps back
    bool had_queue_ = false;
    bool first_ = true;
    double t_ = 0;

    // the signal
    std::deque<Sample> delay_floor_;  // one-way delay floor window
    double t_rebase_ = -1e9;  // when the floor was last re-based on a route change
    std::deque<Sample> trip_hist_;  // the round trip over the delay-floor window, to see whether it followed the step
    std::deque<Sample> step_hist_;  // the mean delay of the last kStepRebaseS, to tell a step that holds from a queue
    std::deque<Sample> trips_;  // round-trip window
    std::deque<Sample> rates_;  // received-rate window
    // What the host put on the wire, with its time. It has to be kept, not just the latest:
    // the client's report arrives a trip late, so the rate it reports corresponds to
    // what was sent a trip earlier. Comparing it against what is being sent NOW
    // makes every rate increase look like loss.
    std::deque<Sample> sent_rates_;
    std::deque<Sample> queues_;  // short window, for the line fit
    std::deque<Sample> minima_;  // the raw queue of the last kNoiseWindow s: its lower envelope
    double delay_floor_cur_ = 0;
    bool competitor_ = false;  // the delay in the window has another flow's swing (kCompetitorThreshold)
    double t_no_swing_ = 0;  // the last time the window did not have that swing
    double loss_smooth_ = 0;
    double loss_peak_ = 0;  // the parity one: the last peak, which is forgotten slowly
    double t_loss_ = -1e9;  // the last time there was loss, or an alarm
    int min_parity_ = 2;
    double fps_ = 60;  // the frames per second encoded
    double t_burst_ = -1e9;  // when the last loss burst started
    double t_burst_end_ = -1e9;  // when the loss last fell below the burst threshold
    bool in_burst_ = false, recurring_ = false;
    double t_start_ = -1;  // the time of the first step
    double loss_burst_ = 0;  // length of the channel's bursts, not counting those from overflow
    double C_est_ = 0, tau_est_ = 0;

    // the ramp-up curve
    double anchor_ = 0, ramp_t_ = 0, ramp_R0_ = 0;
    double t_last_queue_ = 0;  // the last time there was a queue
    bool in_ramp_ = false;

    // the frame that does not fill the budget
    bool app_limited_ = false;
    // More reaches the wire than comes out the other end: there is a queue or there are drops, and in
    // both cases the pipe is full. It is the only thing that stays true when the delay
    // floor has been poisoned.
    bool pipe_full_ = false;
    double train_bandwidth_ = 0;
    double t_cut_ = -1e9;  // the last step that cut the budget by more than 15 %
    double budget_before_cut_ = 0;  // the budget before the cut episode that ended at t_cut_
    double budget_prev_ = 0;  // the budget of the previous step
    double t_bandwidth_ = 0;  // when the bandwidth was last updated
    double sent_then_ = 0;  // what was sent a trip before the current report
    double sent_smooth_ = 0, received_smooth_ = 0;  // the two, smoothed to compare them
    double t_fast_backoff_ = -1e9;  // when the fast backoff last fired
    int shortfall_streak_ = 0;  // consecutive cycles in which noticeably less arrived than was sent
    double t_loss_no_queue_ = -1;  // since when there has been excess loss with no queue visible
    double blind_until_ = -1e9;  // until when the delay cannot be trusted as a queue signal
    bool startup_done_ = false;  // the first queue or loss of the session has already appeared
    double delivery_smooth_ = 0;  // the delivery of the same frames, smoothed to half a second
    double dref_last_ = 0;  // the queue target of the last step, for the alarm
    double t_alarm_ = -1e9;
    double encoder_efficiency_ = 1;  // what the encoder delivers over what it is asked
    double request_prev_ = 0;  // what was asked of it in the previous step
    int queue_over_target_ = 0;  // consecutive steps with the queue above the target
    int narrower_streak_ = 0;  // consecutive cycles in which the last frames crossed a narrower pipe

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
    struct Train {
      double t = 0;
      double bits = 0;
      double span = 0;
    };

    std::deque<Train> trains_;
    // The dispersion bandwidth of the window; false if no frame managed to spread out.
    bool bandwidth_by_regression(double &bandwidth) const;
  };

}  // namespace abr
