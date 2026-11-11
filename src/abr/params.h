/**
 * @file params.h
 * @brief The constants of the bitrate controller, in one place.
 *
 * `session` holds the facts the protocol and the host fix, `tun` the eleven tunables the rest
 * derives from, `cal` the fixed constants of the control law and the signal meter, and `Params`
 * the few values that tests and experiments set. Each constant carries where its value comes from.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace abr {

  /**
   * @brief Facts of the session: what the protocol and the host fix, not what is tuned.
   */
  namespace session {
    constexpr double kReportPeriod = 0.050;  ///< T_r, s: the client's statistics loop (moonlight-common-c)
    constexpr double kHostPingMs = 25;  ///< ms between the host's ENet pings
    constexpr int kMtu = 1024;  ///< payload bytes per packet
    constexpr int kHeaderBytes = 56;  ///< IP + UDP + own header
    constexpr double kAudioBps = 294400.0;  ///< on the wire, with its headers
    constexpr int kGaloisShards = 255;  ///< GF(256): a block does not exceed 255 fragments
    constexpr int kFecMinPackets = 2;  ///< Moonlight's minimum parity packets per block
    constexpr int kMaxBlocksPerFrame = 4;  ///< the most FEC blocks the host splits a frame into
  }  // namespace session

  /**
   * @brief The eleven tunables of the controller. Everything else is a fact of the session
   *   or follows from these.
   */
  namespace tun {
    constexpr double kUtilization = 0.875;  ///< eta: share of the measured pipe the budget uses
    constexpr double kPaceOverPipe = 1.3;  ///< gamma: a frame is paced at this multiple of the pipe the loop believes in
    constexpr double kPaceMax = 2.0;  ///< kappa: the sender's cap on the pacing multiple; also the probe and the ceiling
    constexpr double kPhaseMargin = 0.7853981634;  ///< phi, radians: 45 degrees
    constexpr double kQueueTargetShare = 0.25;  ///< kappa_q: share of (trip + frame) the queue target is set to
    constexpr double kDrainTime = 0.2;  ///< T_d, s: in which the alarm wants the queue to empty
    constexpr double kRateNoise = 0.3;  ///< theta: relative change of a rate that is already not noise
    constexpr double kBlockFailure = 1e-4;  ///< p: probability of losing a parity block
    constexpr double kEstimationWindow = 2.0;  ///< W, s: the window the estimates are fitted over
    constexpr double kMemory = 10.0;  ///< H, s: how long a floor or a loss peak is remembered
    constexpr double kRiseRate = 1.0;  ///< g, 1/s: how fast the budget may rise over what is being sent

    constexpr double kLoopPeriod = 2 * session::kReportPeriod;  ///< T_c
    constexpr double kAlarmStep = kRateNoise * session::kReportPeriod;  ///< queue growth in one report that fires the alarm
    constexpr double kAlarmQueue = (1.0 - kUtilization) * kDrainTime;  ///< queue the alarm needs, in s

    /**
     * @brief Compare a derived constant with its literal where the formula does not round exactly.
     * @param a Value from the formula.
     * @param b Literal it stands for.
     * @return Whether they differ by less than 1e-12.
     */
    constexpr bool almost_equal(double a, double b) {
      return (a > b ? a - b : b - a) < 1e-12;
    }

    static_assert(kLoopPeriod == 0.100);
    static_assert(kAlarmStep == 0.015);
    static_assert(kAlarmQueue == 0.025);
    static_assert(1.0 - kRateNoise == 0.70);
    static_assert(kBlockFailure / 10 == 1e-5);
    static_assert(1.0 / kPaceMax == 0.5 && kPaceMax - 1.0 == 1.0 && kPaceMax * kPaceMax == 4.0);
    static_assert(2.5 * kLoopPeriod == 0.250 && 15 * session::kReportPeriod == 0.75);
    static_assert(1.5 * kEstimationWindow == 3.0 && kEstimationWindow / 2 == 1.0 && 3 * kMemory == 30.0);
    static_assert(almost_equal(kDrainTime + kLoopPeriod + session::kReportPeriod, 0.35));
  }  // namespace tun

  /**
   * @brief Constants of the control law and of the parity: fixed, documented where they are used, and not
   *   tunables. Each carries where its value comes from.
   */
  namespace cal {

    // ─── the frame that does not fill the budget ───────────────────────────────
    /**
     * @brief below this fraction of the budget, the encoder is not using what it has: the sample
     *   is not evidence of anything (WebRTC uses 0.65). Not higher: 0.9 took a few ms off the queue
     *   in the simulator, but when only the control channel stalled it read the stall as a queue and
     *   cut the budget
     */
    constexpr double kAppLimitedThreshold = 1.0 - tun::kRateNoise;
    constexpr double kPaceProbeS = tun::kEstimationWindow / 2;  ///< every so many seconds one frame goes out at 2x the budget, so the dispersion still sees above it; 0 never
    constexpr double kQuietFrameProof = tun::kPaceMax - 1.0;  ///< max budget step per cycle granted by a frame that crossed without spreading out, since it shows the pipe is at least its rate (0 off)
    constexpr double kQuietFrameMinPackets = 12;  ///< packets a frame needs to prove capacity
    constexpr double kQuietFrameMinPacketsBack = 6;  ///< on the way back from a deep cut, packets a frame needs: at the floor budget frames are smaller than kQuietFrameMinPackets and no proof could ever come (0: off)
    constexpr double kWayBackMargin = 1.15;  ///< on the way back the proof's goal only has to clear the budget by this multiple (0: the usual margin)
    constexpr double kQuietFrameProofQueue = 0.5;  ///< a proof counts only with the queue below this fraction of the target
    constexpr double kQuietFrameProofIdleS = 1.0;  ///< seconds without a queue before a proof counts
    /**
     * @brief How many times the line is fitted again with the trains that stayed below it: the
     *   delay the path adds only lengthens the span, and the pipe is the lower edge. 0, the plain
     *   line.
     */
    constexpr int kTrainEnvelopeIters = 2;
    /**
     * @brief Below this a frame has no dispersion. Menu frames, of one or two packets, do not
     *   have it even completed with parity: the header allows at most three parity packets for a
     *   one-packet frame, and a four-packet train fits whole in the burst an idle router lets
     *   through (15 KB in the simulated router): it arrives together and does not show the pipe.
     */
    constexpr double kTrainMinPackets = 6;
    /**
     * @brief How much the measured bandwidth may rise per second, as a fraction of itself. It
     *   drops without limit. 0 does not limit. 0.30 brings 8 → 20 Mbit/s in 5 s instead of 10,
     *   but on a real drop from 40 to 8 it breaks frames in 59 intervals instead of 37.
     */
    constexpr double kTrainRiseMax = (1.0 - tun::kUtilization) * tun::kRiseRate;
    /**
     * @brief How long the measured bandwidth lasts without new samples before it stops being a
     *   ceiling, in seconds.
     */
    constexpr double kTrainExpiry = 1.5 * tun::kEstimationWindow;
    /**
     * @brief With the queue empty, the anchor of the ramp-up curve is at least the train's
     *   utilization target, up to this multiple of the budget. 0 does not use it.
     */
    constexpr double kTrainAnchor = 1.5;
    constexpr double kTrainAnchorWait = 6.0;  ///< And only after this many seconds without a queue.

    // ─── the encoder that delivers less than it is asked for ─────────────
    /**
     * @brief How many consecutive steps with the queue above the target it takes to release it
     *   (above double it is released immediately). 1 releases it with the first.
     */
    constexpr int kEncoderReleaseCycles = 3;

    // ─── the growth filter ───────────────────────────────────────────────────
    /**
     * @brief Growth filter time constant, in seconds, with a short trip. It is NOT fixed:
     *   The best value shifts to the left as the trip rises, and a fixed value is bad at both
     *   extremes.
     */
    constexpr double kFilterSlow = 2.5 * tun::kLoopPeriod;
    constexpr double kFilterFast = 0.150;  ///< growth filter time constant, in seconds, with a long trip

    // ─── the queue target ─────────────────────────────────────────────────────
    constexpr double kNoiseWindow = 0.35;  ///< s: the lower envelope of the queue, with noise
    constexpr double kNoiseFilter = 15 * session::kReportPeriod;  ///< s: the window of the queue fit on a noisy path
    /**
     * @brief Seconds of path noise (Measurement::noise) above which the law takes the lower envelope
     *   of the queue and the longer fit. Below it, the noise the meter sees is the queue oscillation
     *   the loop itself creates, 15–20 ms on a clean link: treating it as path noise delays the
     *   believed queue by about 0.4 s, which lets the re-exploration overshoot and, after a drop,
     *   leaves a ghost queue that drags Ĉ down.
     */
    constexpr double kNoiseMin = 0.040;

    // ─── the estimation windows ───────────────────────────────────────────────
    constexpr double kStepRebaseS = 2.5;  ///< seconds a jump of the delay must stay flat, with no loss and no other flow, to be taken as a longer path and become the new floor (0: never; the floor then keeps the old level for tun::kMemory)
    constexpr double kStepTripShare = 0.5;  ///< fraction of the delay's step the round trip must have risen by for the step to be a longer route and not a queue the ping does not see
    constexpr double kStepFlat = 0.006;  ///< seconds: how much the delay may move within that time and still be flat
    constexpr int kCapacityPercentile = 50;  ///< the median: the delivery of the same frames is no longer biased
    /**
     * @brief Percentile of the round trip the trip is measured with. The gains move
     *   much more slowly than the loop, or retuning destabilizes.
     */
    constexpr int kTripPercentile = 80;
    constexpr double kCapacityTimeConst = 0.30;  ///< smoothing of Ĉ
    constexpr double kCrossMaxS = 2 * tun::kEstimationWindow;  ///< longest a cross queue may last, in seconds, to still count as one (kCrossS in control.cpp)
    /**
     * @brief How far the minimum of the floor window must sit above that of the whole session for
     *   it to be taken as the queue of another flow, which does not empty (0 turns it off), and
     *   which percentile of the window becomes the floor in that case.
     */
    constexpr double kCompetitorThreshold = 0.05;
    constexpr int kCompetitorPercentile = 25;  ///< percentile of the floor window used as the floor while another flow is detected
    constexpr double kCompetitorSustained = 8.0;  ///< seconds a swing must last to be taken as another flow and not as the loop's own queue
    /**
     * @brief How far back the minimum that counts as the current age goes, in seconds: the
     *   summary comes out every 50 ms and the last good frame may carry up to one frame waiting,
     *   so the smallest of several is the one that adds the least of that.
     */
    constexpr double kAgeRecent = 0.15;

    // ─── the loss backoff ──────────────────────────────────────────────────
    constexpr double kBackoffLoss = 1.0 - tun::kUtilization;  ///< loss credible enough to trigger it: below it the parity repairs it, and backing off at 5 % only slowed the recovery on lossy paths
    /**
     * @brief It also jumps if the frames of a report —at least this many— crossed a pipe that
     *   fraction narrower than the budget. 0 does not look at the frames.
     */
    constexpr int kBackoffTrains = 2;
    /**
     * @brief The law does not lower the budget below this fraction of what the pipe delivers. 0
     *   has no floor.
     */
    constexpr double kDeliveredFloor = 0.7;
    constexpr int kFastBackoffCycles = 2;  ///< And only if what arrives fell short in this many consecutive cycles.
    /**
     * @brief Loss above this, sustained for this many seconds with no queue visible, is the queue
     *   of a path the delay does not see; and for this many seconds afterwards the ramp-up does
     *   not accelerate. 0 turns it off.
     */
    constexpr double kCongestionLoss = (1.0 - tun::kUtilization) / 2;
    /**
     * @brief If the raw queue sample rises more than this in one step, and is already past double
     *   the target, it jumps to this fraction of what arrives. 0 turns it off.
     */
    constexpr double kBackoffStep = 0.02;
    constexpr double kBackoffStepFactor = 0.6;  ///< fraction of the pipe the budget jumps to on a queue step (see kBackoffStep)
    /**
     * @brief Over how many consecutive intervals between samples the queue has to grow: with two
     *   the noise of jumping acks is removed, with one a sample is gained —a round trip plus the
     *   queue—, and in a drop from 12 to 4 Mbit/s the router's queue overflows at a hundred
     *   milliseconds. One catches the router's cap within a single interval (a 30 to 6 Mbit/s drop
     *   otherwise never triggers it); the cost is that a path that lengthens can be taken for a
     *   narrowing pipe. With 0, the growth is that of a fixed 50 ms window, which with back-to-back
     *   pings gathers several samples.
     */
    constexpr int kAlarmIntervals = 1;
    /**
     * @brief How far back the queue from before the step comes from: its minimum. 0 is the one
     *   from two samples back.
     */
    constexpr double kAlarmPriorS = 0.25;
    /**
     * @brief How long after the alarm a queue back at its floor retracts it (0 never). A delayed
     *   ack raises the ENet sample all at once and the next one goes back to the floor; the queue
     *   of a pipe that narrowed takes hundreds of milliseconds to empty.
     */
    constexpr double kAlarmRetractS = tun::kDrainTime + tun::kLoopPeriod + session::kReportPeriod;

    // ─── the ramp-up curve ─────────────────────────────────────────────────────
    constexpr double kRampExploration = 0.02;  ///< how much it keeps rising past the anchor
    constexpr double kRampConvex = 0.002;  ///< and how much that rise accelerates while no queue appears (1/s³)
    /**
     * @brief The fast startup again after this many seconds without a queue: the pipe may have
     *   grown. From 4 to 12 Mbit/s, getting there drops from fourteen seconds
     *   to five. 0 does not.
     */
    constexpr double kReexploreS = 2 * tun::kEstimationWindow;
    constexpr double kAnchorBlend = 0.5;  ///< the anchor drops to halfway

    // ─── the repair ───────────────────────────────────────────────────────────
    /**
     * @brief Frame rate `fec_max` holds for. Above it a frame has fewer packets and the same fraction of
     *   parity covers a shorter loss burst, which is counted in packets: at 120 fps, 2 % loss in bursts
     *   broke 5.5 times more blocks per second than at 60 with the same share of parity. The cap grows
     *   in proportion, so a block may carry as many parity packets as at this rate (Params::fec_max_fps; 0 keeps it fixed).
     */
    constexpr double kFecMaxFps = 60.0;

    // ─── what the budget pays for before reaching the picture ──────────────────
    /**
     * @brief 1: the block failure target follows the cost of repairing a lost frame, which is a
     *   round trip: parity_block_failure · 60 ms / trip floor, between parity_block_failure and
     *   1e-2. On a short path a failed block costs little, and the parity it would take is image.
     */
    constexpr int kParityRttTarget = 1;
    constexpr double kBurstGapS = 3.0;  ///< loss that comes back sooner than this after it stopped is the same burst, not a repeat (an outage flaps); 0: every return counts
    constexpr double kParityBurst = 0.01;  ///< Mean loss above which a burst is counted, to know whether it repeats.
    constexpr double kParityStartup = 1.0 - tun::kUtilization;  ///< In the first seconds, the parity is decided with at least this loss.
    constexpr double kParityStartupS = 2.0;  ///< seconds from the start in which kParityStartup applies

    // ─── the rest of the loop ──────────────────────────────────────────────────
    constexpr int kNarrowerCycles = 1;  ///< consecutive cycles in which the last frames must have crossed a narrower pipe to cut the budget
    constexpr double kNarrowerFloor = 1.0 / tun::kPaceMax;  ///< the narrow pipe measured by the trains is never taken below this fraction of what arrived
    constexpr double kLossSmoothGain = 0.30;  ///< per-cycle gain of the loss smoothing
    constexpr double kDeliverySmoothS = 0.5;  ///< time constant of the smoothed delivery

    // ─── the control law ──────────────────────────────────────────────────────
    /// Samples from which the line fit of the delay beats the plain difference
    constexpr int kLineFitSamples = 3;
    /// Percentile of the round trip with which the trip of the queue target is measured
    constexpr int kTripPercentileDref = 5;
    /// Fraction that may stay in the path before it is taken that not everything put in comes out
    constexpr double kEgressMargin = 0.05;
    /// Seconds over which what was sent and what was received are smoothed before comparing them
    constexpr double kEgressTimeConst = tun::kEstimationWindow / 4;
    /// Seconds a queue must stay high and still, with what arrives equal to what is sent, to be a cross queue
    constexpr double kCrossS = 0.5;
    /// Relative difference between what arrives and what was sent that still counts as equal
    constexpr double kCrossTolerance = 0.15;
    /// Seconds over the queue target that count as the queue being well above it
    constexpr double kBackoffQueueMargin = 0.020;
    /// The fast back-off applies only while the queue is under this multiple of the target
    constexpr double kFastBackoffQueue = 2;
    /// Seconds the excess loss must last with no queue visible to be taken as congestion
    constexpr double kCongestionLossS = 0.5;
    /// Seconds the delay is distrusted as a queue signal after loss without a visible queue
    constexpr double kBlindS = 3 * tun::kMemory;
    /// Fraction of the pipe the alarm lowers the budget to, before subtracting the queue drain
    constexpr double kAlarmFactor = 1.0 - tun::kRateNoise;
    /// The alarm never lowers the budget below this fraction of the measured pipe
    constexpr double kAlarmFloor = 0.25;
    /// The anchor is dropped when the queue reappears under this fraction of it
    constexpr double kAnchorCollapse = 0.8;
    /// The way back lasts while the budget is under this fraction of what it was before the cut
    constexpr double kWayBackRatio = 0.9;
    /// Longest the way back lasts after the cut, in seconds
    constexpr double kWayBackS = 3 * tun::kMemory;
    /// Smallest jump over the delay floor, in seconds, that is re-based
    constexpr double kStepMin = 0.030;

    /// The largest parity the client accepts for a block, as a percentage of its data (Moonlight caps it at 255 %).
    constexpr int kMaxParityPercent = 255;
    /// Percent to fraction.
    constexpr int kPercent = 100;
    /// Bisection steps for ω_180: far more than a double needs.
    constexpr int kBisectionSteps = 200;
    /// A regression needs this many trains with different sizes.
    constexpr size_t kMinTrains = 4;
    /// A dispersion bandwidth above this multiple of the declared ceiling is measurement noise.
    constexpr double kMaxBandwidthOverCeiling = tun::kPaceMax * tun::kPaceMax;
    /// Below this spread of the sample times, the line fit has no slope.
    constexpr double kMinTimeSpread = 1e-12;
    /// Steepness of the ramp-up S curve over its duration: from 1 % to 99 % of the way.
    constexpr double kRampSteepness = 9.2;
    /// The alarm's queue has to pass this multiple of the target too.
    constexpr double kAlarmQueueOverTarget = 1.3;
    /// The window the alarm's and the step's growth is measured over, in seconds.
    constexpr double kGrowthWindowS = 0.05;
    /// The most of the budget the queue's growth may say the pipe lost.
    constexpr double kMaxGrowthShare = 0.9;
    /// The alarm only fires if it cuts the budget below this fraction of what it is.
    constexpr double kAlarmMinCut = 0.9;
    /// The floor window needs this many samples before another flow's swing is looked for.
    constexpr size_t kSwingMinSamples = 10;
    constexpr size_t kSwingPercentile = 90;  ///< percentile of the floor window at which another flow's swing is read
    /// The widest dead zone, in seconds of queue.
    constexpr double kMaxDeadZone = 0.020;
    /// Growth per cycle, in seconds, above which the reason is a growing queue.
    constexpr double kGrowingPerCycle = 0.001;
    /// A cross queue counts as still while the raw sample drops less than this, in seconds.
    constexpr double kStillQueueS = 0.010;
    /// A cross queue survived the cut once the budget is this close to the minimum.
    constexpr double kCrossFloorOverMin = 1.5;
    /// The least picture rate, in bits/s.
    constexpr double kMinVideoRate = 0.3e6;
    /// Per-step gain of the burst-length smoothing.
    constexpr double kBurstGain = 0.2;

    // ─── the signal meter ─────────────────────────────────────────────────────

    /**
     * @brief How long what was sent is kept, in microseconds: it has to cover with room to spare the
     *   round trip plus the cadence of the loss summary, which is when it crosses with what arrived.
     */
    constexpr int64_t kRetentionUs = 3000000;

    /**
     * @brief The window of the rates and the loss, in microseconds. The summary arrives every 50 ms
     *   and the FEC status only with loss: a quarter of a second gathers several of each.
     */
    constexpr int64_t kWindowUs = 250000;

    /**
     * @brief Over how long the delivery is measured, in microseconds: with the quantization of one
     *   frame at each end, 300 ms leaves an error of about 2 %.
     */
    constexpr int64_t kDeliverySpanUs = 300000;

    /**
     * @brief A summary this long after the previous one, in microseconds, was held up on its way, and
     *   whatever it carries piled up while the path was silent: it says how far the last good frame
     *   advanced, not how fast.
     */
    constexpr int64_t kSilenceUs = 250000;

    /**
     * @brief How close a client ping has to arrive to an ENet sample, in microseconds, to say how
     *   much of that round trip is the uplink's queue: a ping every 100 ms and an ack every 25 ms,
     *   plus the spread of one that was held.
     */
    constexpr int64_t kPingPairUs = 150000;

    /**
     * @brief What share of a round trip's rise the ping's drift has to account for to call it the
     *   uplink's queue.
     */
    constexpr double kUplinkShare = 0.75;

    /**
     * @brief The window of the minima of the age, of the ping drift and of the round trip, in
     *   microseconds, like the control's floor.
     */
    constexpr int64_t kFloorWindowUs = 10000000;

    /**
     * @brief How much longer a frame has to take to arrive than to leave, as a ratio, for its
     *   dispersion to be that of the bottleneck and not the sender's.
     */
    constexpr double kMinDispersion = 1.1;

    /**
     * @brief The least time between two samples of the video's delay, in microseconds: one per frame
     *   would be read as growth from intervals of 16 ms.
     */
    constexpr int64_t kSampleGapUs = 40000;

    constexpr int64_t kDelayFreshUs = 500000;  ///< how recent the last frame's delay has to be, in microseconds, to say anything about the queue now
    constexpr int64_t kDelayRecentUs = 100000;  ///< span of the minimum of the last frame's delay, in microseconds, so that one frame's burst does not count

    /**
     * @brief How far back the video's delay is looked at, in microseconds, to tell whether a rise of
     *   the round trip was a queue that has since emptied: ENet's smoothing keeps the round trip high
     *   for about this long after it.
     */
    constexpr int64_t kDelayPeakUs = 1500000;

    constexpr double kStallMinMs = 10;  ///< milliseconds a rise of the smoothed round trip that the video's delay does not show must exceed to count as path noise
    constexpr int64_t kStallHoldUs = 5000000;  ///< microseconds a rise over kStallMinMs counts as path noise

    /**
     * @brief Consecutive readings that agree with each other and disagree with everything before
     *   them that it takes to believe a clock or a delay really moved, instead of one corrupt report.
     */
    constexpr int kConfirmReadings = 3;

    constexpr size_t kMaxTrains = 4096;  ///< most frame trains kept, so that a flood of reports cannot grow memory
    constexpr size_t kMaxDelays = 8192;  ///< most video delay samples kept, so that a flood of reports cannot grow memory
    constexpr size_t kMaxSeen = 2048;  ///< most sent-frame records kept, so that a flood of reports cannot grow memory

    /**
     * @brief Frame periods of gap between good frames that still count as a steady cadence.
     */
    constexpr double kAgeCadence = 2.5;

    /**
     * @brief Largest queue from before the step, in seconds, that still lets the alarm read it as a
     *   narrowing pipe.
     */
    constexpr double kAlarmPriorQueue = 0.080;

    /**
     * @brief Per-frame gain of the smoothed packets per frame.
     */
    constexpr double kPacketsPerFrameGain = 0.05;

    constexpr size_t kMinExcursions = 20;  ///< judged samples the noise needs before it is read
    constexpr double kNoisePercentile = 0.9;  ///< fraction of the judged samples at which the noise is read

    /**
     * @brief Samples the growth is measured from: the last three, two intervals.
     */
    constexpr size_t kGrowthSamples = 3;

    /**
     * @brief The window the growth is brought to, in microseconds.
     */
    constexpr int64_t kGrowthSpanUs = 50000;

    constexpr int64_t kFloorTouchedUs = 5000000;  ///< microseconds back in which the queue must have touched its floor for the growth to count
    constexpr int64_t kFloorKnownUs = 3000000;  ///< microseconds from the start while the floor is not yet known and the growth does not count

    /**
     * @brief The span assumed for the first measurement, in seconds.
     */
    constexpr double kFirstSpanS = 0.1;

    /**
     * @brief The age when none has been seen: larger than any.
     */
    constexpr double kNoAge = 1e18;
  }  // namespace cal

  /**
   * @brief Every tunable of the controller, with its default.
   */
  struct Params {
    // ─── the frame that does not fill the budget ───────────────────────────────
    /**
     * @brief eta: 87.5 % of what was measured is used. What is left over drains the queues by
     *   itself, without cutting the send and without a transient. This, not the queue target, sets the
     *   steady queue: at 0.90 the keyframes and the rest of the bursts kept 50–60 ms of p95 queue on a
     *   20 Mbit/s pipe. 0.85 lowers it further, but after a 2.4 s outage the budget took more than
     *   10 s to come back
     */
    double train_utilization = tun::kUtilization;

    // ─── the encoder that delivers less than it is asked for ─────────────
    /**
     * @brief How much more than the video rate the encoder may be asked for. 1 turns the compensation off. 1.6: with the train ceiling the budget
     *   no longer exceeds the pipe, and libx264 delivers about 70 % of what it is asked for; with
     *   1.3 the compensation ends up pinned at its cap.
     */
    double encoder_compensation_max = 1.6;
    double encoder_time_const = tun::kEstimationWindow;  ///< seconds over which what the encoder lacks is learned
    // ─── the pace of the loop ───────────────────────────────────────────────────
    /**
     * @brief Loop period, in seconds: 100 ms on a home network. At 300 ms the hold alone
     *   contributes 150 ms of dead time.
     */
    double Tc = tun::kLoopPeriod;
    double fps = 60.0;  ///< nominal frames per second of the stream

    // ─── the queue target ─────────────────────────────────────────────────────
    /**
     * @brief And never below the path noise (Measurement::noise) by this factor: on Wi-Fi or over
     *   the internet the queue rises and falls by itself tens of milliseconds, and a 10 ms target
     *   read it as queue. 1 and not more: the noise includes each frame's own dispersion, so 1.5
     *   counted it twice and blunted every mechanism tied to the target; with 0 a Wi-Fi with air
     *   outages broke more frames.
     */
    double dref_noise = 1.0;

    // ─── the estimation windows ───────────────────────────────────────────────

    /**
     * @brief With the delivery of the same frames (Measurement::delivery), the pipe is full if
     *   they arrive more slowly than this relative to how they left. 0 does not use it and
     *   compares the rates.
     */
    double delivery_full = 0.93;
    /**
     * @brief Whether the queue also comes from the age of the last good frame (the GameStream
     *   meter).
     */
    int queue_age = 1;

    /**
     * @brief How much may be authorized above the capacity that could be PROVEN. It is not an
     *   operating target —in steady state R and Ĉ end up stuck together— but the margin the loop
     *   has left to probe for a new pipe. If it were exactly 1.0, R and Ĉ would feed back and the
     *   loop could not discover that the pipe grew.
     */
    double ceiling_over_C = 1.0 / tun::kUtilization;

    // ─── the mix and the margins ─────────────────────────────────────────────
    double rho = 5.0;  ///< 5 to 1. Beyond that almost no lead is gained
    double phase_margin = tun::kPhaseMargin;  ///< 45°, in radians

    // ─── the loss backoff ──────────────────────────────────────────────────
    /**
     * @brief The alarm between steps (Control::alarm): the queue has to grow more than this in 50
     *   ms and pass this (or 1.3 times the target); the target is kAlarmFactor of the pipe minus
     *   what the queue drains in alarm_drain seconds. alarm_step 0 turns it off. 15 ms and not
     *   20: a drop from 12 to 4 Mbit/s makes the queue grow about 20 ms per
     *   interval, right at the edge; if it does not fire, the queue reaches 220 ms and the
     *   invalidation asks for a frame from twenty back, outside the sixteen references: a
     *   keyframe.
     */
    double alarm_step = tun::kAlarmStep;
    double alarm_span_min = 0.5;  ///< A frame of 3+ packets that arrived in less than this fraction of its send span gives no queue sample: the receiver stamped it in a batch (0 off)

    // ─── the repair ───────────────────────────────────────────────────────────
    double fec_min = 0.05;  ///< prior project specifications
    double fec_max = 0.50;  ///< highest parity fraction of the repair decision, for frames at fec_max_fps
    double fec_max_fps = cal::kFecMaxFps;  ///< frame rate fec_max holds for; above it the cap grows with the rate; 0 keeps it fixed (the repair block's cap)

    // ─── the caps ───────────────────────────────────────────────────────────
    double bitrate_min = 1e6;  ///< lowest budget, in bits/s, and never below what a frame costs on the wire (Control::bitrate_floor)
    double bitrate_max = 30e6;  ///< highest budget, in bits/s
    /**
     * @brief Starting low and rising is not symmetric. With the law alone, starting low
     *   left the loop stuck below the pipe —it rises feeling its way and stops as soon as the queue
     *   touches the target—, and 0.4 cost ten points of permanent utilization. With the train as
     *   the anchor of the ramp-up curve (`train_anchor`) the rise measures the pipe instead of
     *   feeling for it, and starting low no longer costs utilization. Starting high does cost: with a
     *   download already in the pipe, the first second at 0.7 of the negotiated broke five
     *   consecutive keyframes. Half the negotiated reaches a stable pipe in 5 s instead of 8.
     */
    double startup_fraction = 1.0 / tun::kPaceMax;

    // ─── what the budget pays for before reaching the picture ──────────────────
    /**
     * @brief Payload bytes per packet.
     */
    int mtu = session::kMtu;
    int header_bytes = session::kHeaderBytes;  ///< IP + UDP + own header
    double audio_bps = session::kAudioBps;  ///< on the wire, with its headers
    int gf_shards = session::kGaloisShards;  ///< GF(256): a block does not exceed 255 fragments
    /**
     * @brief Moonlight asks the host for a minimum of TWO parity packets per block, and says so
     *   in so many words in its code: "if a frame has fewer data fragments than would generate 2
     *   parity, the percentage is raised for that frame above the requested value (up to 200 %)".
     *   With small frames that minimum, and not the percentage, is what rules.
     */
    int fec_min_packets = session::kFecMinPackets;
    /**
     * @brief The minimum while there was no loss in the last fec_clean_s seconds (−1: always the
     *   one above). The client builds each block with the parity the percentage of its packets
     *   says, whatever it is; at 4 Mbit/s a frame has five packets, and two parity are 40 % of
     *   the wire. Without loss no parity is sent: if loss appears, the lost frame is repaired by
     *   invalidating references, and the first loss or the alarm return the client's minimum.
     */
    int fec_min_clean = 0;
    /**
     * @brief 1: the session's encoder repairs a lost frame by invalidating references (NVENC with
     *   long-term references: the host reports a repair reach above 0); 0: it cannot (libx264,
     *   FFmpeg NVENC, VAAPI, QSV, AMF, or NVENC without long-term references), and the first
     *   loss ends in a keyframe. Without that repair the clean-path drop of fec_min_clean does
     *   not apply: the parity per block never goes below fec_min_packets, the minimum the client
     *   asks for. The safe value is the default; a session sets it (Control::set_repairs_by_invalidation)
     *   when its encoder reports a reach, and a test bench that models NVENC with repair sets it.
     */
    int repairs_by_invalidation = 0;

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
    double parity_block_failure = tun::kBlockFailure;
    /**
     * @brief The frame that repairs an invalidation is the only one that cannot be lost: if it is
     *   lost, it is invalidated again from it, and with two or three in a row the last sound
     *   reference falls outside the encoder's sixteen and a keyframe goes out. It happens behind
     *   the queue of a download. Its block is protected with this target, and its parity may
     *   reach this multiple of the data.
     */
    double parity_repair_failure = tun::kBlockFailure / 10;
    double fec_max_repair = 1.0;  ///< highest parity, as a multiple of the data, of the block that repairs an invalidation
  };

  /**
   * @brief The two gains, derived from the phase margin and the mix ρ.
   *
   * They are not tuning constants but a consequence of the phase margin and the mix. They are
   * computed once, when the controller is built.
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
