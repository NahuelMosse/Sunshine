// The "All parameters" table of chapter 9, as code.
//
// Each field carries next to it where it comes from. If a value has no provenance, it does not go here:
// it goes in supuestos.toml marked as such, which is where the fidelity debt shows.
//
// It is loaded from parametros.toml. None of this is recompiled to tune.
#pragma once

#include <string>

namespace abr {

  /**
   * @brief The ways of treating a frame that does not fill the budget.
   *
   * They are chosen by parameter so that all of them run against the same scenario and the same
   * seed.
   */
  enum class Variant {
    baseline,  ///< as it was: the loop does not notice and fools itself
    patched,  ///< detect that the budget is not being used and stop probing (BBR, WebRTC)
    padding,  ///< fill with dummy packets so the loop sees a saturated flow (Vidaptive)
    train,  ///< measure the dispersion of each frame's packet train (SQP)
  };

  /**
   * @brief Name of a variant.
   *
   * @param v The variant.
   * @return The name the variant goes by in parameter files.
   */
  const char *name(Variant v);

  /**
   * @brief Find a variant by name.
   *
   * @param s The name, as returned by name().
   * @return The variant.
   * @throws std::invalid_argument If the name is not a variant.
   */
  Variant variant_by_name(const std::string &s);

  /**
   * @brief Every tunable of the controller, with the default taken from chapter 9's "All parameters"
   *   table.
   */
  struct Params {
    Variant variant = Variant::baseline;  ///< how a frame that does not fill the budget is treated

    // ─── the frame that does not fill the budget ───────────────────────────────
    /**
     * @brief below this fraction of the budget, the encoder is not using what it has: the sample
     *   is not evidence of anything (WebRTC uses 0.65)
     */
    double app_limited_threshold = 0.70;
    double train_pacing = 2.0;  ///< kappa: each frame is sent at 2x the estimate (SQP)
    /**
     * @brief eta: 90 % of what was measured is used. The 10 % left over drains the queues by
     *   itself, without probing and without a transient
     */
    double train_utilization = 0.90;
    /**
     * @brief Seconds of frames the dispersion bandwidth is fitted over. The bandwidth comes from
     *   the slope of the span against the size, so the window has to gather frames of different
     *   sizes: two seconds cover a keyframe and about a hundred P frames.
     */
    double train_window = 2.0;
    /**
     * @brief How many times the line is fitted again with the trains that stayed below it: the
     *   delay the path adds only lengthens the span, and the pipe is the lower edge. 0, the plain
     *   line.
     */
    int train_envelope_iters = 2;
    /**
     * @brief How much of the computed budget is actually used. Below 1 there is permanent slack,
     *   and with slack the queues drain by themselves — which is SQP's alternative to the
     *   periodic probing of the floor. The trade-off between the two is measured in its own
     *   experiment.
     */
    double utilization = 1.0;
    /**
     * @brief Below this a frame has no dispersion. Menu frames, of one or two packets, do not
     *   have it even completed with parity: the header allows at most three parity packets for a
     *   one-packet frame, and a four-packet train fits whole in the burst an idle router lets
     *   through (15 KB in the bench's): it arrives together and does not show the pipe.
     */
    double train_min_packets = 6;
    /**
     * @brief How much the measured bandwidth may rise per second, as a fraction of itself. It
     *   drops without limit. 0 does not limit.
     */
    double train_rise_max = 0.10;
    /**
     * @brief How long the measured bandwidth lasts without new samples before it stops being a
     *   ceiling, in seconds.
     */
    double train_expiry = 3.0;
    /**
     * @brief With the queue empty, the anchor of the ramp-up curve is at least the train's
     *   utilization target, up to this multiple of the budget. 0 does not use it.
     */
    double train_anchor = 1.5;
    double train_anchor_wait = 6.0;  ///< And only after this many seconds without a queue.

    // ─── the encoder that delivers less than it is asked for ─────────────
    /**
     * @brief How much more than the video rate the encoder may be asked for. 1 turns the compensation off. 1.6: with the train ceiling the budget
     *   no longer exceeds the pipe, and libx264 delivers about 70 % of what it is asked for; with
     *   1.3 the compensation ends up pinned at its cap.
     */
    double encoder_compensation_max = 1.6;
    double encoder_time_const = 2.0;  ///< seconds over which what the encoder lacks is learned
    /**
     * @brief How many consecutive steps with the queue above the target it takes to release it
     *   (above double it is released immediately). 1 releases it with the first.
     */
    int encoder_release_cycles = 3;
    // ─── the pace of the loop ───────────────────────────────────────────────────
    /**
     * @brief Loop period, in seconds. Ch. 9: 100 ms on a home network. At 300 ms the hold alone
     *   contributes 150 ms of dead time.
     */
    double Tc = 0.100;
    double fps = 60.0;  ///< nominal frames per second of the stream

    // ─── the growth filter ───────────────────────────────────────────────────
    /**
     * @brief Growth filter time constant, in seconds, with a short trip. Ch. 9: it is NOT fixed.
     *   The best value shifts to the left as the trip rises, and a fixed value is bad at both
     *   extremes.
     */
    double filter_slow = 0.250;
    double filter_fast = 0.150;  ///< growth filter time constant, in seconds, with a long trip
    double filter_rtt_cutoff = 0.100;  ///< round trip, in seconds, where the two filters cross over
    int line_fit_samples = 3;  ///< ch. 6: the fit only wins from ~3 on

    // ─── the queue target ─────────────────────────────────────────────────────
    /**
     * @brief Lowest queue target, in seconds. Ch. 3: with a fixed 10 ms and a 250 ms trip, the queue empties 54 % of the time and
     *   the loop goes blind. That is why the target is tied to the path, not to the loop.
     */
    double dref_floor = 0.010;
    double dref_fraction = 0.25;  ///< fraction of (trip + frame) the queue target is set to
    /**
     * @brief And never below the path noise (Measurement::noise) by this factor: on Wi-Fi or over
     *   the internet the queue rises and falls by itself tens of milliseconds, and a 10 ms target
     *   read it as queue.
     */
    double dref_noise = 1.5;
    double noise_window = 0.35;  ///< s: the lower envelope of the queue, with noise
    double noise_filter = 0.75;  ///< s: the window of the queue fit on a noisy path
    /**
     * @brief A queue that emptied in less than this, with a pipe that delivers what was being
     *   sent before it, was an air outage and not a narrower pipe: the budget goes back to what
     *   it was. In seconds. 0 turns it off, and it is off: the bench could not tell it apart from
     *   noise between runs.
     */
    double drain_episode = 0;
    /**
     * @brief How much the queue has to rise per second for the queue to be that of an air outage
     *   and not the loop's own, in seconds of queue per second.
     */
    double drain_ramp = 0.5;
    /**
     * @brief In how much time the queue of an outage like the last one must be able to empty:
     *   sets what part of that pipe is used. Outages on a distant Wi-Fi come every couple of
     *   seconds. In seconds.
     */
    double drain_deadline = 0.5;
    int trip_percentile_dref = 5;  ///< the percentile of the round trip the trip is measured with
    int tau_from_floor = 0;  ///< if τ̂ comes from the trip floor instead of the trip_percentile

    // ─── the estimation windows ───────────────────────────────────────────────
    double delay_floor_window = 10.0;  ///< ch. 6: same value as BBR, same reasons
    double capacity_window = 1.5;  ///< seconds of received-rate and round-trip samples kept to estimate the capacity
    int capacity_percentile = 50;  ///< the median: the delivery of the same frames is no longer biased
    /**
     * @brief Percentile of the round trip the trip is measured with. Ch. 5 and 3: the gains move
     *   much more slowly than the loop, or retuning destabilizes.
     */
    int trip_percentile = 80;
    double capacity_time_const = 0.30;  ///< smoothing of Ĉ

    /**
     * @brief How much can stay in the path before it is considered that NOT everything put in is
     *   coming out. It is the gate of the only signal that does not depend on the delay floor, so
     *   it can be neither so small that measurement noise opens it nor so large that a real loss
     *   covers it.
     */
    double egress_margin = 0.05;
    /**
     * @brief The cross queue: a queue episode in which, for at least cross_s, the queue stayed
     *   high and still while what arrived was the same as what was sent —neither filling up from
     *   our traffic nor emptying when lowering—. If it ends by itself before cross_max_s, the
     *   pipe did not change: the budget and the capacity go back to what they were, and the calm
     *   is explored without waiting for reexplore_s. 0 turns it off.
     */
    double cross_s = 0.5;
    double cross_max_s = 4.0;  ///< longest a cross queue may last, in seconds, to still count as one (see cross_s)
    double cross_tolerance = 0.15;  ///< how much what arrives and what was sent may differ
    /**
     * @brief With the delivery of the same frames (Measurement::delivery), the pipe is full if
     *   they arrive more slowly than this relative to how they left. 0 does not use it and
     *   compares the rates.
     */
    double delivery_full = 0.93;
    int ceiling_anti_windup = 1;  ///< Whether the proven ceiling unwinds the ramp-up curve when it clips.
    /**
     * @brief Whether the queue also comes from the age of the last good frame (the GameStream
     *   meter).
     */
    int queue_age = 1;
    /**
     * @brief How far the minimum of the floor window must sit above that of the whole session for
     *   it to be taken as the queue of another flow, which does not empty (0 turns it off), and
     *   which percentile of the window becomes the floor in that case.
     */
    double competitor_threshold = 0.05;
    int competitor_percentile = 25;  ///< percentile of the floor window used as the floor while another flow is detected
    double competitor_sustained = 8.0;  ///< seconds a swing must last to be taken as another flow and not as our own queue
    double competitor_dref = 0.0;  ///< multiple of the queue target the swing has to exceed to count as another flow; 0 uses only competitor_threshold
    double competitor_loss_s = 10.0;  ///< seconds back in which a loss means the swing belongs to a loss-guided flow
    int capacity_sustained = 0;  ///< samples the queue must stay up before the capacity learns from it; 0 learns from the peak
    double encoder_release_loss = 0.0;  ///< smoothed loss above which the encoder compensation counts the path as congested; 0 turns it off
    double age_cadence = 2.5;  ///< frame periods of gap between good frames that still count as a steady cadence
    /**
     * @brief How far back the minimum that counts as the current age goes, in seconds: the
     *   summary comes out every 50 ms and the last good frame may carry up to one frame waiting,
     *   so the smallest of several is the one that adds the least of that.
     */
    double age_recent = 0.15;
    /**
     * @brief Seconds over which what was sent and what was received are smoothed before comparing
     *   them. A real encoder delivers very uneven frame sizes, and in a window of a quarter of a
     *   second what arrives ends up 5 % below what left one in every fourteen steps with no queue
     *   or loss. A narrowing pipe sustains the difference; noise does not.
     */
    double egress_time_const = 0.5;

    /**
     * @brief How much may be authorized above the capacity that could be PROVEN. It is not an
     *   operating target —in steady state R and Ĉ end up stuck together— but the margin the loop
     *   has left to probe for a new pipe. If it were exactly 1.0, R and Ĉ would feed back and the
     *   loop could not discover that the pipe grew.
     */
    double ceiling_over_C = 1.1;

    // ─── the coherence check (ch. 5) ───────────────────────────────────────────
    double coh_min_drop = 0.005;  ///< per cycle. Below it the model asserts nothing
    double coh_flat = 0.002;  ///< ±2 ms per cycle: neither falling nor still growing
    double coh_margin = 0.015;  ///< +15 ms over the target. Without this it fires at the normal equilibrium point
    int coh_cycles = 3;  ///< acts first and undoes afterwards

    // ─── the mix and the margins (ch. 4) ─────────────────────────────────────
    double rho = 5.0;  ///< 5 to 1. Beyond that almost no lead is gained
    double phase_margin = 0.7853981634;  ///< 45°, in radians

    // ─── the loss backoff (ch. 5) ──────────────────────────────────────────
    double backoff_factor = 0.85;  ///< over the measured received rate. Taken from GCC
    double backoff_loss = 0.05;  ///< loss credible enough to trigger it
    double backoff_queue_margin = 0.020;  ///< and the queue well above the target.
    /**
     * @brief The fast backoff: if this fraction less arrives than what was sent a trip earlier,
     *   it jumps to the received rate without waiting for the queue. 0 turns it off.
     */
    double fast_backoff = 0.3;
    /**
     * @brief It also jumps if the frames of a report —at least this many— crossed a pipe that
     *   fraction narrower than the budget. 0 does not look at the frames.
     */
    int backoff_trains = 2;
    /**
     * @brief The law does not lower the budget below this fraction of what the pipe delivers. 0
     *   has no floor.
     */
    double delivered_floor = 0.7;
    double fast_backoff_queue = 2;  ///< And only while the queue is below this multiple of the target.
    int fast_backoff_cycles = 2;  ///< And only if what arrives fell short in this many consecutive cycles.
    /**
     * @brief Loss above this, sustained for this many seconds with no queue visible, is the queue
     *   of a path the delay does not see; and for this many seconds afterwards the ramp-up does
     *   not accelerate. 0 turns it off.
     */
    double congestion_loss = 0.06;
    double congestion_loss_s = 0.5;  ///< seconds the excess loss must last with no queue visible before it is taken as congestion
    double blind_s = 30.0;  ///< seconds the delay is distrusted as a queue signal after loss without a visible queue
    /**
     * @brief If the raw queue sample rises more than this in one step, and is already past double
     *   the target, it jumps to this fraction of what arrives. 0 turns it off.
     */
    double backoff_step = 0.02;
    double backoff_step_factor = 0.6;  ///< fraction of the pipe the budget jumps to on a queue step (see backoff_step)
    /**
     * @brief The alarm between steps (Control::alarm): the queue has to grow more than this in 50
     *   ms and pass this (or 1.3 times the target); the target is alarm_factor of the pipe minus
     *   what the queue drains in alarm_drain seconds. alarm_step 0 turns it off. 15 ms and not
     *   20: a drop from 12 to 4 Mbit/s with real Sunshine makes the queue grow about 20 ms per
     *   interval, right at the edge; if it does not fire, the queue reaches 220 ms and the
     *   invalidation asks for a frame from twenty back, outside the sixteen references: a
     *   keyframe.
     */
    double alarm_step = 0.015;
    double alarm_queue = 0.025;  ///< absolute queue, in seconds, the alarm needs before it fires (or 1.3 times the target if larger)
    /**
     * @brief The queue from before the step cannot exceed this: a download builds a queue of
     *   hundreds of milliseconds that grows and falls at its own pace, and that is not a
     *   narrowing pipe. And our own, with the pipe full, runs between 30 and 50 ms: with 60,
     *   "emptied recently" asks to drop below 30 and the alarm goes mute right in the drop from a
     *   full pipe, which is the usual one.
     */
    double alarm_prior_queue = 0.080;
    /**
     * @brief Over how many consecutive intervals between samples the queue has to grow: with two
     *   the noise of jumping acks is removed, with one a sample is gained —a round trip plus the
     *   queue—, and in a drop from 12 to 4 Mbit/s the router's queue overflows at a hundred
     *   milliseconds. With 0, the growth is that of a fixed 50 ms window, which with back-to-back
     *   pings gathers several samples.
     */
    int alarm_intervals = 2;
    /**
     * @brief How far back the queue from before the step comes from: its minimum. 0 is the one
     *   from two samples back.
     */
    double alarm_prior_s = 0.25;
    /**
     * @brief How long after the alarm a queue back at its floor retracts it (0 never). A delayed
     *   ack raises the ENet sample all at once and the next one goes back to the floor; the queue
     *   of a pipe that narrowed takes hundreds of milliseconds to empty.
     */
    double alarm_retract_s = 0.25;
    double alarm_factor = 0.7;  ///< fraction of the pipe the alarm lowers the budget to, before subtracting the queue drain
    double alarm_drain = 0.2;  ///< seconds in which the alarm wants the queue to empty
    /**
     * @brief Whether the alarm asks for a keyframe right away: what left in the last hundred
     *   milliseconds is being lost, and the first whole frame to arrive is already the keyframe.
     *   Off, because in a drop that the queue absorbs nothing is lost and the keyframe is
     *   superfluous.
     */
    int keyframe_on_drop = 0;
    /**
     * @brief How many frames can be in flight —the round trip times the frames per second— before
     *   the host encodes less. The client learns of what was lost a round trip later, and to
     *   repair it by invalidating references the last frame it has must still be among the
     *   encoder's sixteen. 0 turns it off. With twelve, behind the queue of a download —two
     *   hundred milliseconds, fifteen frames at 60 per second— the host encodes half and the
     *   repair arrives in time; with a free path it does not act.
     */
    int frames_in_flight = 12;
    /**
     * @brief And when the alarm fires, one frame in every this many for six tenths of a second:
     *   what overflowed the queue is noticed only when the first good frame behind it arrives,
     *   one whole queue later.
     */
    int alarm_frame_divisor = 4;

    // ─── the ramp-up curve (ch. 4) ─────────────────────────────────────────────
    double ramp_duration = 2.0;  ///< the timid S: slow, accelerates, brakes on arrival
    double ramp_exploration = 0.02;  ///< how much it keeps rising past the anchor
    double ramp_convex = 0.002;  ///< and how much that rise accelerates while no queue appears (1/s³)
    double fast_startup = 1.0;  ///< until the first queue, the rise per second over what is being sent
    /**
     * @brief Exploration at cruise (section 7a of the control): how much it multiplies per step,
     *   what raw queue counts as stuck to the floor, and how many seconds of cruise are needed. 0
     *   turns it off.
     */
    double cruise_gain = 0.0;
    double cruise_queue = 0.004;  ///< raw queue, in seconds, below which the path counts as stuck to the floor while cruising
    double cruise_wait = 2.0;  ///< seconds of cruise needed before the exploration starts
    /**
     * @brief The fast startup again after this many seconds without a queue: the pipe may have
     *   grown. With real Sunshine, from 4 to 12 Mbit/s, getting there drops from fourteen seconds
     *   to five. 0 does not.
     */
    double reexplore_s = 4.0;
    int queue_requires_full = 0;  ///< a queue of less than double the target only counts with the pipe full
    double anchor_collapse = 0.8;  ///< if the queue reappears well below the anchor,
    double anchor_blend = 0.5;  ///< the anchor drops to halfway

    // ─── the repair (ch. 7) ───────────────────────────────────────────────────
    double fec_min = 0.05;  ///< prior project specifications
    double fec_max = 0.50;  ///< highest parity fraction of the repair decision
    double fec_rise = 0.60;  ///< rise fast
    double fec_fall_tau = 5.0;  ///< fall slowly: ~5 s
    double fec_z = 2.0;  ///< f ≈ p + z·√(p/D)
    double expected_blocks = 30.0;  ///< typical D; the plant overrides it with the real one

    // ─── the floor probe (ch. 5) ───────────────────────────────────────────────
    double probe_interval = 8.0;  ///< seconds between probes of the delay floor
    double probe_duration = 0.35;  ///< seconds the budget stays cut during a floor probe
    double probe_cut = 0.15;  ///< costs 0.66 % of the throughput

    // ─── the caps ───────────────────────────────────────────────────────────
    double bitrate_min = 1e6;  ///< lowest budget, in bits/s
    double bitrate_max = 30e6;  ///< highest budget, in bits/s
    /**
     * @brief Ch. 5: starting low and rising is not symmetric. With the law alone, starting low
     *   left the loop stuck below the pipe —it rises probing and stops as soon as the queue
     *   touches the target—, and 0.4 cost ten points of permanent utilization. With the train as
     *   the anchor of the ramp-up curve (`train_anchor`) the rise measures the pipe instead of
     *   probing it, and starting low no longer costs utilization. Starting high does cost: with a
     *   download already in the pipe, the first second at 0.7 of the negotiated broke five
     *   consecutive keyframes.
     */
    double startup_fraction = 0.3;

    // ─── what the budget pays for before reaching the picture ──────────────────
    // Provenance of each one in supuestos.toml. Those marked "invented" there
    // are debt, not decisions.
    /**
     * @brief Payload bytes per packet.
     */
    int mtu = 1024;
    int header_bytes = 56;  ///< IP + UDP + own header
    double audio_bps = 294400.0;  ///< on the wire, with its headers
    int gf_shards = 255;  ///< GF(256): a block does not exceed 255 fragments
    int blocks_per_frame = 4;  ///< number of FEC blocks a frame is split into
    /**
     * @brief Moonlight asks the host for a minimum of TWO parity packets per block, and says so
     *   in so many words in its code: "if a frame has fewer data fragments than would generate 2
     *   parity, the percentage is raised for that frame above the requested value (up to 200 %)".
     *   With small frames that minimum, and not the percentage, is what rules.
     */
    int fec_min_packets = 2;
    /**
     * @brief The minimum while there was no loss in the last fec_clean_s seconds (−1: always the
     *   one above). The client builds each block with the parity the percentage of its packets
     *   says, whatever it is; at 4 Mbit/s a frame has five packets, and two parity are 40 % of
     *   the wire. Without loss no parity is sent: if loss appears, the lost frame is repaired by
     *   invalidating references, and the first loss or the alarm return the client's minimum.
     */
    int fec_min_clean = 0;
    double fec_clean_s = 10.0;  ///< seconds without loss after which the minimum parity is dropped (see fec_min_clean)

    /**
     * @brief How to go from the repair decision to parity packets. 0  percentage: ceil(k·f) with
     *   the minimum above, which is what Sunshine does. With small frames the knob does not
     *   respond —the minimum covers the percentage— and the picture is computed with the
     *   requested parity even if double comes out. 1  packets: per block, the smallest parity m
     *   (with the same minimum) that keeps the probability of losing the block below
     *   `parity_block_failure` with the estimated loss, and the picture is computed with the
     *   parity that actually went out.
     */
    int parity_in_packets = 1;
    /**
     * @brief One in ten thousand. On a Wi-Fi with burst loss, one in a thousand leaves about four
     *   repaired pictures per minute (a short stutter each); one in ten thousand cuts them in
     *   half for about three points of utilization. Stricter does not help: what is left are
     *   pictures that lose half of their packets at once, and no reasonable parity covers that.
     */
    double parity_block_failure = 0.0001;
    /**
     * @brief The frame that repairs an invalidation is the only one that cannot be lost: if it is
     *   lost, it is invalidated again from it, and with two or three in a row the last sound
     *   reference falls outside the encoder's sixteen and a keyframe goes out. It happens behind
     *   the queue of a download. Its block is protected with this target, and its parity may
     *   reach this multiple of the data.
     */
    double parity_repair_failure = 1e-5;
    double fec_max_repair = 1.0;  ///< highest parity, as a multiple of the data, of the block that repairs an invalidation
    /**
     * @brief Seconds over which the parity forgets a loss peak (it is decided with the larger of
     *   the mean loss and the peak, which decays with this constant). 0 decides it with the mean.
     */
    double parity_memory = 10.0;
    double parity_burst = 0.01;  ///< Mean loss above which a burst is counted, to know whether it repeats.
    /**
     * @brief The ENet round trip, on the host side: how many milliseconds between the host's
     *   pings.
     */
    double enet_ping_ms = 25;
    double parity_startup = 0.10;  ///< In the first seconds, the parity is decided with at least this loss.
    double parity_startup_s = 2.0;  ///< seconds from the start in which parity_startup applies

    /**
     * @brief Load the parameters from a flat TOML file.
     *
     * @param file Path of the file.
     * @return Empty if it went well, or the reason for the error.
     */
    std::string load(const std::string &file);

    /**
     * @brief Set a single parameter by name.
     *
     * @param key Name of the parameter.
     * @param value New value, as text.
     * @return False if the name does not exist.
     */
    bool set(const std::string &key, const std::string &value);

    /**
     * @brief Render every parameter as text.
     *
     * @return One `name = value` entry per parameter.
     */
    std::string dump() const;
  };

  /**
   * @brief The two gains, derived from the phase margin and the mix ρ.
   *
   * Ch. 4: they are not tuning constants, they are a consequence. They are recomputed every cycle.
   */
  struct Tuning {
    double kg_n = 0;  ///< k_g = kg_n · Ĉ / τ̂
    double kp_n = 0;  ///< k_p = kp_n · Ĉ / τ̂²
    double wc_tau = 0;  ///< ω_c·τ
    double w180_tau = 0;  ///< ω_180·τ
    double gain_margin = 0;  ///< PREDICTION: the loop oscillates here
    double period_tau = 0;  ///< and with this period, in units of τ
  };

  /**
   * @brief Derive the gains from the phase margin and the mix.
   *
   * @param p The parameters; `phase_margin` and `rho` are the ones used.
   * @return The normalized gains, with the predictions of the loop's stability.
   */
  Tuning derive(const Params &p);

}  // namespace abr
