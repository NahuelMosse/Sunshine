/**
 * @file control.cpp
 * @brief The control law, the ramp-up, the alarm and the parity decision.
 */
#include "control.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

namespace abr {
  using namespace cal;

  int block_parity(int k, double loss, double burst, const Params &p, int minimum) {
    if (minimum < 0) {
      minimum = p.fec_min_packets;
    }
    const double fec_max = p.fec_max_fps > 0 && p.fps > p.fec_max_fps ? p.fec_max * p.fps / p.fec_max_fps : p.fec_max;
    const int ceiling = std::min({p.gf_shards - k, (k * kMaxParityPercent + kPercent - 1) / kPercent, std::max(minimum, static_cast<int>(std::ceil(k * fec_max)))});
    if (k <= 0) {
      return 0;
    }
    burst = std::max(1.0, burst);
    // Stumbles start at each packet with probability q, and each one takes away a geometric
    // length of mean `burst`: the dwell in the bad state of a Gilbert-Elliott channel.
    // The block is lost if what is lost adds up to more than m. A fixed length ignores the tail of long
    // bursts: with the measured burst at one and a bit, it leaves blocks uncovered against a stumble of three.
    const double q = std::min(0.5, std::max(1e-6, loss) / burst);
    const double r = 1.0 - 1.0 / burst;  // keep losing within a stumble
    const double ratio = q / (1.0 - q);
    // P(lost ≤ m) = Σ_j P(j stumbles) · P(their lengths add up ≤ m), with j binomial(n, q) and the
    // sum of j geometrics a negative binomial: P(sum = t) = C(t−1, j−1)·(1−r)^j·r^(t−j).
    // The negative binomial term is advanced in t by a recurrence, and its cumulative F_j is kept
    // across m, so the whole search is O(m²) with no lgamma.
    const int upper = std::max(0, ceiling);
    std::vector<double> term(upper + 2, 0.0), cum(upper + 2, 0.0);  // nb(j, t = m) and F_j(m)
    for (int m = 0; m < ceiling; ++m) {
      // advance every existing j from t = m−1 to t = m, then start j = m
      for (int j = 1; j < m; ++j) {
        term[j] *= r * (m - 1.0) / (m - j);
        cum[j] += term[j];
      }
      if (m >= 1) {
        term[m] = std::exp(m * std::log1p(-r));
        cum[m] = term[m];
      }
      if (m < minimum) {
        continue;
      }
      const int n = k + m;
      double w = std::exp(n * std::log1p(-q));  // P(no stumble)
      double acc = w;
      for (int j = 1; j <= std::min(n, m); ++j) {
        w *= static_cast<double>(n - j + 1) / j * ratio;
        acc += w * cum[j];
      }
      if (1.0 - acc < p.parity_block_failure) {
        return m;
      }
    }
    return upper;
  }

  int frame_parity(int k, double loss, double burst, const Params &p, int minimum, double repair) {
    if (k <= 0) {
      return 0;
    }
    // The host splits a frame into blocks that fit the Galois field with their own parity, at most
    // session::kMaxBlocksPerFrame of them, and the parity is paid per block.
    const int max_data = std::max(1, static_cast<int>(p.gf_shards / (1.0 + std::max(0.0, repair))));
    const int blocks = std::min(session::kMaxBlocksPerFrame, std::max(1, (k + max_data - 1) / max_data));
    const int base = (k + blocks - 1) / blocks;
    int left = k, total = 0;
    for (int b = 0; b < blocks && left > 0; ++b) {
      const int kb = std::min(base, left);
      left -= kb;
      total += block_parity(kb, loss, burst, p, minimum);
    }
    return total;
  }

  double block_failure_target(const Params &p, double trip_base) {
    if (!kParityRttTarget || trip_base <= 0) {
      return p.parity_block_failure;
    }
    // A failed block costs a repair, and a repair costs a round trip.
    return std::min(1e-2, std::max(p.parity_block_failure, p.parity_block_failure * 0.060 / trip_base));
  }

  bool quiet_frame_proof_room(const Params &p, double raw_queue, double dref, double idle_s, double goal_bps, double budget_bps, double margin) {
    return raw_queue < kQuietFrameProofQueue * dref && idle_s > kQuietFrameProofIdleS && (margin <= 0 || goal_bps > margin * budget_bps);
  }

  const char *name(Reason m) {
    switch (m) {
      case Reason::startup:
        return "startup";
      case Reason::steady:
        return "steady";
      case Reason::no_evidence:
        return "no_evidence";
      case Reason::queue_growing:
        return "queue_growing";
      case Reason::queue_high:
        return "queue_high";
      case Reason::loss_backoff:
        return "loss_backoff";
      case Reason::ramping:
        return "ramping";
      case Reason::reanchored:
        return "reanchored";
      case Reason::capped:
        return "capped";
    }
    return "?";
  }

  // ── the derived tuning ────────────────────────────────────────────────────
  // k_g and k_p are not chosen constants, they are a consequence of asking the loop for a
  // phase margin and a mix ρ. From the same two quantities come the predictions of where the
  // loop would start to oscillate.
  //
  // The open loop has TWO integrations, and that is the key to the whole design:
  //
  //   the queue integrates the bitrate excess ........ 1/(C·s)
  //   the law integrates its own correction ......... (k_g·s + k_p)/s
  //   and the report comes back late ................ e^{−sτ}
  //
  //   L(s) = (k_g·s + k_p)·e^{−sτ} / (C·s²)
  //
  // Two integrations are −180° from the start: without the growth term no margin is
  // left, and that is why the derivative is not an optional improvement but the condition for the
  // loop to exist. With u = ω·τ and z = k_p/k_g normalized:
  //
  //   |L(u)| = kg_n·√(u² + z²) / u²        phase(u) = −π + atan(u/z) − u
  constexpr double kPi = 3.14159265358979323846;  ///< Ratio of a circle's circumference to its diameter.

  Tuning derive(const Params &p) {
    Tuning s;
    const double phi = std::atan(p.rho);
    s.wc_tau = phi - p.phase_margin;  // ω_c·τ
    s.kg_n = s.wc_tau / std::sqrt(1.0 + 1.0 / (p.rho * p.rho));
    s.kp_n = s.kg_n * s.wc_tau / p.rho;

    const double z = s.kp_n / s.kg_n;  // the knee of the lead

    // ω_180: where the correction comes back half a turn late. Asking for phase = −π leaves
    // atan(u/z) = u, which has no closed form — bisection.
    auto excess = [&](double u) {
      return std::atan(u / z) - u;
    };
    double lo = s.wc_tau, hi = kPi / 2 + 1.0;  // excess(hi) < 0 always
    for (int i = 0; i < kBisectionSteps; ++i) {
      const double mid = 0.5 * (lo + hi);
      if (excess(mid) > 0) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    s.w180_tau = 0.5 * (lo + hi);

    // At ω_180 the correction comes back smaller, and that factor is how much the
    // gain can be raised before the loop sustains oscillation. It is a verifiable number.
    auto magnitude = [&](double u) {
      return s.kg_n * std::sqrt(u * u + z * z) / (u * u);
    };
    s.gain_margin = 1.0 / magnitude(s.w180_tau);
    s.period_tau = 2.0 * kPi / s.w180_tau;
    return s;
  }

  // ── window utilities ────────────────────────────────────────────────────
  void Control::prune(std::deque<Sample> &v, double t, double window) {
    while (!v.empty() && v.front().t < t - window) {
      v.pop_front();
    }
  }

  double Control::percentile(const std::deque<Sample> &v, int q) {
    if (v.empty()) {
      return 0;
    }
    std::vector<double> a;
    a.reserve(v.size());
    for (const auto &m : v) {
      a.push_back(m.v);
    }
    const size_t k = std::min(a.size() - 1, static_cast<size_t>(q / 100.0 * a.size()));
    std::nth_element(a.begin(), a.begin() + k, a.end());
    return a[k];
  }

  double Control::percentile_within(const std::deque<Sample> &v, int q, double t, double window) {
    std::deque<Sample> w;
    for (const auto &m : v) {
      if (m.t >= t - window) {
        w.push_back(m);
      }
    }
    return percentile(w, q);
  }

  bool Control::bandwidth_by_regression(double &bandwidth) const {
    if (trains_.size() < kMinTrains) {
      return false;
    }
    // The line of span against size, over the trains that remain. Returns false if the
    // frames did not have different sizes: with all equal, the slope is not determined.
    auto fit_line = [](const std::vector<const Train *> &v, double &slope_out, double &intercept) {
      if (v.size() < kMinTrains) {
        return false;
      }
      double sx = 0, sy = 0;
      for (const Train *x : v) {
        sx += x->bits;
        sy += x->span;
      }
      const double n = static_cast<double>(v.size());
      const double mx = sx / n, my = sy / n;
      double sxx = 0, sxy = 0;
      for (const Train *x : v) {
        sxx += (x->bits - mx) * (x->bits - mx);
        sxy += (x->bits - mx) * (x->span - my);
      }
      if (sxx <= 1e-9 * mx * mx * n) {
        return false;
      }
      slope_out = sxy / sxx;
      intercept = my - slope_out * mx;
      return true;
    };
    std::vector<const Train *> v;
    v.reserve(trains_.size());
    for (const Train &x : trains_) {
      v.push_back(&x);
    }
    double slope = 0, intercept = 0;  // seconds per bit, and the intercept
    if (!fit_line(v, slope, intercept)) {
      return false;
    }
    // What delays a frame on the path —the air bursts of a Wi-Fi, a thread that woke
    // up late— only adds span to it, never subtracts: the pipe is the lower edge of the cloud,
    // not its middle. It is fitted again with the trains that stayed below the line.
    for (int i = 0; i < kTrainEnvelopeIters; ++i) {
      std::vector<const Train *> below;
      for (const Train *x : v) {
        if (x->span <= intercept + slope * x->bits) {
          below.push_back(x);
        }
      }
      double p2 = 0, o2 = 0;
      if (!fit_line(below, p2, o2)) {
        break;
      }
      v.swap(below);
      slope = p2;
      intercept = o2;
    }
    if (slope <= 0) {
      return false;  // the span does not grow with size
    }
    const double c = 1.0 / slope;
    // Above the declared ceiling it is measurement noise, not a discovery.
    if (c > p_.bitrate_max * kMaxBandwidthOverCeiling) {
      return false;
    }
    bandwidth = std::min(c, p_.bitrate_max);
    return true;
  }

  double Control::value_at(const std::deque<Sample> &v, double t) {
    if (v.empty()) {
      return 0;
    }
    const Sample *best = &v.front();
    double dist = std::fabs(v.front().t - t);
    for (const auto &m : v) {
      const double d = std::fabs(m.t - t);
      if (d < dist) {
        dist = d;
        best = &m;
      }
    }
    return best->v;
  }

  // ── the growth filter ────────────────────────────────────────────────────
  // It is NOT fixed. The best value shifts to the left as the trip rises, and a
  // fixed value is bad at both extremes.
  double Control::filter_time_const(double rtt) const {
    return rtt > tun::kLoopPeriod ? kFilterFast : kFilterSlow;
  }

  // Average while there are few samples, line fit from ~3 on.
  // The fit's residual is literally the width of the meter's cloud, and from it comes
  // the dead zone — which thus narrows by itself when there is more evidence.
  double Control::fit_slope(double &residual_rms) const {
    residual_rms = 0;
    const size_t n = queues_.size();
    if (n < 2) {
      return 0;
    }

    if (n < static_cast<size_t>(kLineFitSamples)) {
      // without evidence for a line: simple difference over the span
      const double dt = queues_.back().t - queues_.front().t;
      return dt > 1e-9 ? (queues_.back().v - queues_.front().v) / dt : 0.0;
    }

    double st = 0, sv = 0;
    for (const auto &m : queues_) {
      st += m.t;
      sv += m.v;
    }
    const double mt = st / n, mv = sv / n;
    double num = 0, den = 0;
    for (const auto &m : queues_) {
      num += (m.t - mt) * (m.v - mv);
      den += (m.t - mt) * (m.t - mt);
    }
    const double b = den > kMinTimeSpread ? num / den : 0.0;
    const double a = mv - b * mt;

    double ss = 0;
    for (const auto &m : queues_) {
      const double e = m.v - (a + b * m.t);
      ss += e * e;
    }
    // n-2 degrees of freedom: two parameters were estimated
    residual_rms = n > 2 ? std::sqrt(ss / (n - 2)) : 0.0;
    return b;
  }

  // ── the estimates that retune the loop ───────────────────────────────────
  // They move much more slowly than the loop, or the retuning itself destabilizes it. That is
  // why percentiles over long windows and not fast averages.
  void Control::estimate(const Measurement &m, double queue, double dref) {
    // With a queue there is information: if there is no queue, what comes out is what is put in and says nothing
    // about the capacity.
    //
    // But there is a second gate, and it is the one that saves the loop when the first lies. If of
    // what is put on the wire noticeably less arrives, something stayed in the path: either there is a queue
    // or there are drops, and in both cases the pipe is full. Then the received rate IS a
    // capacity measurement — and this conclusion DOES NOT GO THROUGH THE DELAY FLOOR.
    //
    // It is exactly the case that used to break the loop. Against a flow that never lets the
    // queue drain, the floor is poisoned, the measured queue drops below `dref`, and with the old gate alone
    // not a single capacity sample was gathered. Without samples, the branch below pushed Ĉ toward
    // R: the ceiling kept rising by itself just when it needed not to move.
    // Two precautions, and each cost one run:
    //
    //   · It has to be really sending. If the encoder delivered a fraction of the
    //     budget, what arrives measures the encoder and not the pipe — and Ĉ would learn from that
    //     a pipe that is not there.
    //   · It has to be compared against what was sent ONE TRIP EARLIER. The report arrives late,
    //     so the rate it brings corresponds to the past. Against the current rate, every
    //     increase looks like loss — and in a still scene, which goes up and down all the time, that
    //     brought the budget down to a quarter of the pipe.
    sent_rates_.push_back({m.t, m.sent_rate});
    prune(sent_rates_, m.t, 2.0);
    const double sent_then = value_at(sent_rates_, m.t - m.rtt);
    sent_then_ = sent_then;

    const double a = std::min(1.0, p_.Tc / kEgressTimeConst);
    if (m.received_rate > 0) {
      received_smooth_ += a * (m.received_rate - received_smooth_);
      sent_smooth_ += a * (sent_then - sent_smooth_);
    }
    const bool really_sending = sent_then > kAppLimitedThreshold * R_;
    bool stays_in_path =
      really_sending &&
      received_smooth_ < sent_smooth_ * (1.0 - kEgressMargin);
    // If the measurement brings the delivery of the same frames, that is what says whether the pipe slows down:
    // it does not mix the encoder's variation with the pipe's.
    if (p_.delivery_full > 0 && m.delivery > 0) {
      delivery_smooth_ = delivery_smooth_ > 0 ? delivery_smooth_ + (p_.Tc / kDeliverySmoothS) * (m.delivery - delivery_smooth_) : m.delivery;
      stays_in_path = really_sending && delivery_smooth_ < p_.delivery_full;
    }
    pipe_full_ = stays_in_path;

    // What arrives only says how much the pipe gives if it was really sending: with a still picture, what
    // arrives is the audio, and learned as a pipe it took the capacity to the floor.
    const bool queue_teaches = queue > dref && really_sending;
    if (m.received_rate > 0 && (queue_teaches || stays_in_path)) {
      rates_.push_back({m.t, m.received_rate});
    }
    prune(rates_, m.t, tun::kEstimationWindow);

    const double measured = percentile(rates_, kCapacityPercentile);  // toward the safe side
    if (measured > 0) {
      C_est_ += (p_.Tc / kCapacityTimeConst) * (measured - C_est_);
    } else if (R_ > C_est_ && !app_limited_ && !stays_in_path) {
      // With no queue the capacity is at least what is being sent. With two
      // guards, because the two times this branch was wrong it was by believing that something went through the
      // pipe that did not:
      //   · if the encoder delivered a fraction of the budget, nothing went out and there is
      //     nothing to conclude;
      //   · if it went out but did not arrive, neither: that is not a free pipe, it is a full pipe.
      C_est_ += (p_.Tc / tun::kEstimationWindow) * (R_ - C_est_);
    }
    C_est_ = std::max(bitrate_floor(), C_est_);

    trips_.push_back({m.t, m.rtt});
    prune(trips_, m.t, std::max(tun::kEstimationWindow, tun::kEstimationWindow));
    const double trip = percentile_within(trips_, kTripPercentile, m.t, tun::kEstimationWindow);
    const double frame_time = 1.0 / p_.fps;
    tau_est_ = trip + frame_time + p_.Tc / 2.0 + filter_time_const(trip);
  }

  // ── ramping up is feeling the way ─────────────────────────────────────────
  // The law cannot rise by itself: without a queue there is no signal. The timid S is the one that leaves the least latency
  // peak over the whole recovery range, and in a game that is what you feel.
  // When a ceiling clips the budget, the ramp-up curve cannot keep advancing underneath
  // as if nothing happened: it went on toward its anchor —the value it started from, for example— and the
  // day the ceiling disappeared the budget jumped at once to the curve's value. With a 4 Mbit/s
  // pipe, from 3.3 to 9.3 in one cycle. The anchor drops to the ceiling and the curve starts over from there.
  void Control::anti_windup(double ceiling) {
    anchor_ = std::min(anchor_, ceiling);
    in_ramp_ = false;
  }

  void Control::ramp_up_step() {
    if (!in_ramp_) {
      in_ramp_ = true;
      ramp_t_ = 0;
      ramp_R0_ = R_;
      if (anchor_ < R_) {
        anchor_ = R_;
      }
    }
    ramp_t_ += p_.Tc;
    const double T = tun::kEstimationWindow, a = kRampSteepness / T;
    const double sg = 1.0 / (1.0 + std::exp(-a * (ramp_t_ - T / 2)));
    const double s0 = 1.0 / (1.0 + std::exp(a * T / 2));
    double target = ramp_R0_ + (anchor_ - ramp_R0_) * (sg - s0) / (1.0 - 2.0 * s0);
    // Past the curve it keeps exploring: the anchor is a memory, not a wall. First slowly
    // and then faster and faster, like the convex part of CUBIC: the more time passes without
    // a queue appearing, the less the anchor is worth as an estimate of the pipe. Without the train there is no other
    // way to know the pipe grew, and at a constant pace going from 4 to 12 Mbit/s took minutes.
    if (ramp_t_ > T) {
      const double x = ramp_t_ - T;
      // The acceleration counts from when the delay is trustworthy again: time spent blind does not accumulate,
      // and when it expires it does not jump at once to what it would have reached
      const double xc = std::min(x, std::max(0.0, t_ - blind_until_));
      target += anchor_ * (kRampExploration * x + kRampConvex * xc * xc * xc);
    }
    // Until the first queue of the session nothing says where the pipe is, and the anchor is only the value
    // it started from, which was deliberately chosen low. Meanwhile it rises in proportion to what is
    // already being sent, like BBR's startup: on a 12 Mbit/s pipe,
    // rising from the anchor the action took thirty-five seconds to reach the pipe.
    //
    // And the same when there has been no queue for a while: the pipe may have grown, and the anchor is the
    // memory of a narrower one. With the curve alone, from a 4 to a 12 Mbit/s pipe, getting there took fifteen seconds.
    // After a cross queue that ended by itself, the calm that follows is short —the movie comes back in
    // seconds— and the deadline cannot be waited out: it is explored right away.
    const bool no_queue = (kReexploreS > 0 && t_ - t_last_queue_ > kReexploreS) || after_cross_;
    if (tun::kRiseRate > 0 && (!startup_done_ || no_queue)) {
      const double fast = R_ * (1.0 + tun::kRiseRate * p_.Tc);
      target = std::max(target, fast);
    }
    R_ = std::max(R_, std::min(p_.bitrate_max, target));
  }

  // ── the alarm: the pipe drop, without waiting for the step ────────────────
  // Without loss, R ≤ C_new·(1 + B/T_d) is needed, with B the router's buffer and T_d how long the
  // loop takes to see and act. With the 100 ms step and the smoothed round trip, T_d came to 0.25 to
  // 0.3 s, and dropping from 12 to 4 Mbit/s overflowed a 280 ms buffer before the loop found out.
  // With each round-trip sample —the host pings every 25 ms— T_d drops to one round trip plus
  // one sample.
  //
  // The step is the queue growing more than anything else makes it grow: 15 ms in 50 ms is
  // sending more than 1.4 times the pipe. The queue a download builds grows one packet per trip.
  // And the target is not a fraction of the pipe but what also empties the queue in a given time,
  // like the drain of the near-zero-queuing cloud gaming controller (NSDI 2024): 0.85 of the pipe
  // minus the queue over the drain time.
  bool Control::alarm(double t, double queue, double growth, double received) {
    if (p_.alarm_step <= 0 || received <= 0) {
      return false;
    }
    if (growth < p_.alarm_step) {
      return false;
    }
    if (queue < std::max(tun::kAlarmQueue, kAlarmQueueOverTarget * dref_last_)) {
      return false;
    }
    // The new pipe: what arrives is still the old one —it measures frames that left earlier—, but the
    // queue's slope is already the new one, because it grows at (R−C)/R. From 12 to 4 Mbit/s, what arrived
    // said 9 and the slope said 5.5.
    const double growth_share = std::min(kMaxGrowthShare, std::max(0.0, growth / kGrowthWindowS));
    // With a still picture what arrives is the audio: it says nothing about the pipe
    const double pipe = std::min({C_est_, app_limited_ ? C_est_ : received, R_ * (1.0 - growth_share)});
    const double target = std::max({bitrate_floor(), kAlarmFloor * pipe, pipe * (kAlarmFactor - queue / tun::kDrainTime)});
    // One alarm every two seconds, like the backoffs; but it can fire even if the step's backoff
    // has just fired, if it goes lower: that one looks at what arrives, which at that moment is still the
    // old pipe, and with the common guard the alarm was blocked just when it was needed.
    if (t - t_alarm_ < tun::kEstimationWindow) {
      return false;
    }
    if (R_ <= target / kAlarmMinCut) {
      return false;
    }
    before_alarm_ = {R_, anchor_, t_alarm_, t_loss_, t_fast_backoff_, in_ramp_, startup_done_, min_parity_};
    alarm_open_ = true;
    t_alarm_ = t;
    t_loss_ = t;
    min_parity_ = p_.fec_min_packets;
    R_ = target;
    anti_windup(pipe);
    t_fast_backoff_ = t;
    startup_done_ = true;
    return true;
  }

  bool Control::retract_alarm(double t, double queue) {
    if (!alarm_open_) {
      return false;
    }
    // Only the immediately following sample can retract it: past that span, whatever happens is something else
    if (t - t_alarm_ > kAlarmRetractS) {
      alarm_open_ = false;
      return false;
    }
    if (queue >= 0.5 * dref_last_) {
      return false;
    }
    alarm_open_ = false;
    const BeforeAlarm &a = before_alarm_;
    R_ = a.R;
    anchor_ = std::min(a.anchor, std::max(anchor_, R_));
    t_alarm_ = a.t_alarm;
    t_loss_ = a.t_loss;
    t_fast_backoff_ = a.t_fast_backoff;
    in_ramp_ = a.in_ramp;
    startup_done_ = a.startup_done;
    min_parity_ = a.min_parity;
    return true;
  }

  // ── the loop ───────────────────────────────────────────────────────────
  Control::Control(const Params &p):
      Control(p, p.bitrate_max) {
  }

  Control::Control(const Params &p, double negotiated):
      p_(p),
      parity_p_(p),
      s_(derive(p)) {
    start(negotiated);
  }

  // Every member goes back to its initial value, by building a fresh controller: a member added later
  // cannot be forgotten here.
  void Control::reset(double negotiated) {
    Control fresh(p_, negotiated);
    *this = std::move(fresh);
  }

  void Control::start(double negotiated) {
    // Starting low and rising costs a few seconds of poorer picture; starting
    // high and dropping costs a BROKEN picture plus the risk of triggering the emergency backoff.
    // It is not symmetric.
    R_ = std::max(p_.bitrate_min, std::min(p_.bitrate_max, negotiated * p_.startup_fraction));
    R_healthy_ = R_;
    anchor_ = R_;
    C_est_ = negotiated;
    delay_floor_.clear();
    trips_.clear();
    trip_hist_.clear();
    step_hist_.clear();
    t_rebase_ = -1e9;
    rates_.clear();
    queues_.clear();
    minima_.clear();
    sent_rates_.clear();
    delay_floor_cur_ = 0;
    competitor_ = false;
    t_no_swing_ = -1e9;
    loss_smooth_ = 0;
    loss_peak_ = 0;
    loss_burst_ = 0;
    t_burst_ = -1e9;
    t_burst_end_ = -1e9;
    in_burst_ = false;
    recurring_ = false;
    t_start_ = -1;
    t_loss_ = -1e9;
    min_parity_ = p_.fec_min_packets;
    fps_ = p_.fps;
    raw_queue_prev_ = 0;
    raw_queue_prev2_ = 0;
    had_queue_ = false;
    first_ = true;
    in_ramp_ = false;
    C_healthy_ = negotiated;
    episode_ = {};
    after_cross_ = false;
    app_limited_ = false;
    pipe_full_ = false;
    train_bandwidth_ = 0;
    trains_.clear();
    t_bandwidth_ = 0;
    t_fast_backoff_ = -1e9;
    shortfall_streak_ = 0;
    delivery_smooth_ = 0;
    t_alarm_ = -1e9;
    startup_done_ = false;
    t_loss_no_queue_ = -1;
    blind_until_ = -1e9;
    encoder_efficiency_ = 1;
    queue_over_target_ = 0;
    request_prev_ = 0;
    sent_smooth_ = received_smooth_ = 0;
    t_last_queue_ = 0;
    narrower_streak_ = 0;
    t_ = 0;
  }

  Decision Control::step(const Measurement &m) {
    t_ = m.t;
    Cycle c;

    // ── 0. did the encoder use the budget? ─────────────────────────────────
    // It is the first thing to know, because it decides whether what follows is evidence or noise.
    // The host knows it without the client's help: they are the bits it itself put on the wire.
    app_limited_ = m.sent_rate > 0 && m.sent_rate < kAppLimitedThreshold * R_;
    dispersion(m, c);

    // ── 1. what the measurement says: the queue, the loss and the queue target ──
    delay_floor(m, c);
    loss(m);
    queue_target(m, c);

    // ── 2. the law, and the backoffs that override it ─────────────────────────
    control_law(c);
    if (!c.startup) {
      back_off_to(m, c);
    }
    apply(m, c);

    // ── 3. dropping is correcting, rising is feeling the way ─────────────────
    ramp(m, c);
    cross_queue(m, c);

    // ── 4. the ceilings, the parity and the encoder ───────────────────────────
    ceilings(m, c);
    parity_budget(c);
    encoder_compensation(m, c);
    audit(m, c);
    if (budget_prev_ > 0 && c.d.budget < 0.85 * budget_prev_) {
      // A cut that follows another within a few seconds is the same episode: the budget to come back to is the
      // one before the first.
      budget_before_cut_ = m.t - t_cut_ < 5.0 ? std::max(budget_before_cut_, budget_prev_) : budget_prev_;
      t_cut_ = m.t;
    }
    budget_prev_ = c.d.budget;
    return c.d;
  }

  // The sample by dispersion of the packet train (SQP). It does not depend on the average rate
  // but on how the packets of ONE frame spread out when crossing the bottleneck, so
  // it still works even if the frame is small.
  // Likewise, only with the frames of this report: the pipe the last frames crossed.
  // It is noisy, but it is the first thing that shows the pipe narrowed —before any
  // average of rates—, and with the frames spaced at κ·R it measures the new pipe as soon as it drops
  // below what is being sent.
  void Control::dispersion(const Measurement &m, Cycle &c) {
    if (!m.trains.empty()) {
      // A frame of two or three packets has no dispersion to measure: what is read is
      // the clock noise, not the bottleneck. SQP acknowledges it and it is its known
      // limit. With fewer packets than this floor, the sample is discarded.
      for (const auto &tr : m.trains) {
        const double packets = tr.bytes / static_cast<double>(p_.mtu + p_.header_bytes);
        if (tr.span > 1e-6 && packets >= kTrainMinPackets) {
          trains_.push_back({m.t, tr.bytes * 8.0, tr.span});
          // n packets open n − 1 gaps
          c.recent_bits += tr.bytes * 8.0 * (packets - 1.0) / packets;
          c.recent_span += tr.span;
          ++c.recent_trains;
        }
      }
      // With a single frame, "bits over span" would be the bandwidth. But if the bottleneck
      // lets B bytes through at once when idle —a router's token bucket, Wi-Fi
      // aggregation—, a train of S bytes takes not S/C but (S − B)/C, and a small frame
      // that fits whole in the burst takes no time: the quotient gives an infinite pipe.
      // It happens precisely in the still scene, where frames are small and the link is idle.
      //
      // Fitting the span against the size over several frames, the slope is 1/C and the
      // burst ends up in the intercept, so it does not contaminate the estimate. If all the frames
      // fit in the burst there is no slope to fit, and the bandwidth does not move: there is no
      // evidence, and without evidence it does not rise.
      while (!trains_.empty() && m.t - trains_.front().t > tun::kEstimationWindow) {
        trains_.pop_front();
      }
      // It drops at once and rises slowly. Dropping is what protects; the jumps upward are almost
      // always from the bottleneck: if it lets bursts through, every time the queue empties it
      // recovers the credit, the frames pass partly at once and the bandwidth comes out inflated for a couple
      // of seconds, just when the law sees the queue empty and rises.
      double bandwidth = 0;
      if (bandwidth_by_regression(bandwidth)) {
        // The pipe delivers at least what just arrived. A frame the client processed
        // late —with no CPU, packets are read in batches— has an enormous span, and the regression
        // with few samples gave a nearly null bandwidth: dropping at once, a single sample like that threw
        // the budget to the minimum with the pipe delivering four times more.
        if (m.received_rate > 0) {
          bandwidth = std::max(bandwidth, m.received_rate);
        }
        const double dt = std::max(0.0, m.t - t_bandwidth_);
        // On a clean path —no queue and no loss for a while— the rise can be faster: what inflates the
        // measure is the burst credit of a queue that keeps emptying, and what a fast rise costs shows
        // up as queue or loss, which put it back on the slow one
        const bool clean = m.t - t_loss_ > tun::kEstimationWindow && m.t - t_last_queue_ > tun::kEstimationWindow;
        const double rise = clean && tun::kRiseRate > kTrainRiseMax ? tun::kRiseRate : kTrainRiseMax;
        if (rise > 0 && train_bandwidth_ > 0 && bandwidth > train_bandwidth_) {
          bandwidth = std::min(bandwidth, train_bandwidth_ * (1.0 + rise * dt));
        }
        train_bandwidth_ = bandwidth;
        t_bandwidth_ = m.t;
      }
    }
    // An old measurement is not a ceiling. With the budget low and many frames per second, the
    // frames end up below the minimum of packets that have measurable dispersion and no new
    // samples come in; without expiry, the ceiling of the last measurement —the one from the drop, for
    // example— held the budget below a pipe that had already come back, and the low budget
    // kept the frames small. Once expired, the law rules with its proven ceiling.
    if (train_bandwidth_ > 0 && m.t - t_bandwidth_ > kTrainExpiry) {
      train_bandwidth_ = 0;
    }
  }

  // The delay floor, the reference for the whole measurement: the minimum over 10 s, the same
  // window as BBR's for the same reasons. The client's clock has an unknown offset, so the absolute
  // level means nothing; the only usable thing is how far it came away from the recent minimum.
  // A longer route lengthens the round trip as well: the ping goes over the same path, so its floor rises with the
  // one-way delay's. A one-way delay that rose with the round trip staying where it was is not the route, it is a
  // queue on the way of the video that the ping does not see. A missing round trip says nothing. Once the route
  // is known to have changed, the later corrections of the floor within three windows do not ask again.
  bool Control::trip_follows_step(double now, double step) const {
    double sum = 0;
    double floor = 0;
    int n = 0;
    for (const auto &x : trip_hist_) {
      if (x.v <= 0) {
        return true;
      }
      floor = floor > 0 ? std::min(floor, x.v) : x.v;
      if (now - x.t <= kStepRebaseS) {
        sum += x.v;
        ++n;
      }
    }
    return n > 0 && sum / n - floor >= kStepTripShare * step;
  }

  void Control::delay_floor(const Measurement &m, Cycle &c) {
    delay_floor_.push_back({m.t, m.owd_min});
    prune(delay_floor_, m.t, tun::kMemory);
    delay_floor_cur_ = delay_floor_.front().v;
    for (const auto &x : delay_floor_) {
      delay_floor_cur_ = std::min(delay_floor_cur_, x.v);
    }
    c.delay_floor_min = delay_floor_cur_;
    // A route that gets longer raises the delay by a step that stays flat: with the floor kept for ten seconds
    // it reads as a queue the loop cannot empty, because there is none, and holds the budget at its
    // minimum for the whole window. A delay that sat flat above the floor for kStepRebaseS with no loss
    // and no other flow's swing is the new floor. Another flow's standing queue moves —its sawtooth, its
    // bursts— and loss would have said so.
    step_hist_.push_back({m.t, m.owd_mean});
    prune(step_hist_, m.t, kStepRebaseS);
    trip_hist_.push_back({m.t, m.rtt});
    prune(trip_hist_, m.t, tun::kMemory);
    if (kStepRebaseS > 0 && !competitor_ && m.t - t_loss_ > kStepRebaseS && m.t - step_hist_.front().t >= 0.9 * kStepRebaseS && (m.delivery <= 0 || m.delivery >= 0.97)) {
      double lo = step_hist_.front().v, hi = lo;
      for (const auto &x : step_hist_) {
        lo = std::min(lo, x.v);
        hi = std::max(hi, x.v);
      }
      if (hi - lo <= kStepFlat && lo - delay_floor_cur_ >= kStepMin && (m.t - t_rebase_ < 3 * tun::kMemory || trip_follows_step(m.t, lo - delay_floor_cur_))) {
        t_rebase_ = m.t;
        delay_floor_.clear();
        for (const auto &x : step_hist_) {
          delay_floor_.push_back({x.t, x.v});
        }
        delay_floor_cur_ = lo;
        c.delay_floor_min = lo;
      }
    }
    // With another flow filling the pipe the queue never empties, and the ten-second minimum stays
    // wherever the other's deepest valley leaves it: a TCP that backs off a lot lowers the floor, the loop believes
    // the other's queue is its own and yields the pipe; one that backs off little leaves it high and the loop competes.
    // What gives the other away is not the level but the swing: its sawtooth moves the window's delay
    // considerably more than the loop's own queue, which the loop keeps at tens of milliseconds. With
    // that swing the floor becomes a low percentile of the window, which does not depend on how deep the valleys are.
    // A swing of the loop's own —a drop, a route jump— lasts as long as the loop takes to empty
    // its queue; the other's, as long as the other lasts: only one sustained for kCompetitorSustained counts.
    // Loss says it is another flow —a flow guided by loss fills the buffer until it overflows— in the last
    // tun::kMemory; or, if the loop already went so low that it does not even lose, the low percentile
    // lifted off the minimum by more than the target: the loop's own queue does not occupy three quarters of the window.
    competitor_ = false;
    if (kCompetitorThreshold > 0 && delay_floor_.size() >= kSwingMinSamples) {
      std::vector<double> v;
      v.reserve(delay_floor_.size());
      for (const auto &x : delay_floor_) {
        v.push_back(x.v);
      }
      const size_t j = v.size() * kSwingPercentile / kPercent;
      std::nth_element(v.begin(), v.begin() + j, v.end());
      if (v[j] - delay_floor_cur_ <= kCompetitorThreshold) {
        t_no_swing_ = m.t;
      } else if (m.t - t_no_swing_ >= kCompetitorSustained) {
        const size_t k = std::min(v.size() - 1, v.size() * static_cast<size_t>(kCompetitorPercentile) / kPercent);
        std::nth_element(v.begin(), v.begin() + k, v.end());
        const bool lost_recently = m.t - t_loss_ < tun::kMemory;
        if (lost_recently || v[k] - delay_floor_cur_ > dref_last_) {
          competitor_ = true;
          delay_floor_cur_ = v[k];
        }
      }
    } else {
      t_no_swing_ = m.t;
    }

    c.raw_queue = std::max(0.0, m.owd_mean - delay_floor_cur_);
    // On a noisy path, the law looks at the lower envelope: the minimum of the last
    // kNoiseWindow. An air outage builds a real queue —the pipe drops to almost nothing for a couple of
    // tenths of a second— but goes away by itself, and counted whole it lowered the budget at once every two seconds
    // without the ramp-up, which follows the timid curve, recovering it before the next outage. A queue
    // that is sustained shows up with that delay; the alarm and the backoff, which look at the raw sample,
    // do not change.
    double law_queue = c.raw_queue;
    minima_.push_back({m.t, c.raw_queue});
    prune(minima_, m.t, kNoiseWindow);
    if (m.noise > kNoiseMin) {
      double minimum = c.raw_queue;
      for (const auto &x : minima_) {
        minimum = std::min(minimum, x.v);
      }
      law_queue = minimum;
    }
    queues_.push_back({m.t, law_queue});
    // And with noise, the line is fitted over more time: a hundred-millisecond jump dominated a
    // window of two hundred fifty and was read as the queue growing.
    prune(queues_, m.t, m.noise > kNoiseMin ? std::max(filter_time_const(m.rtt), kNoiseFilter) : filter_time_const(m.rtt));

    const double queue_slope = fit_slope(c.residual);  // line fit
    c.delta = queue_slope * p_.Tc;  // growth per cycle
    // the queue the loop uses is the fit's, not the raw sample
    double st = 0;
    for (const auto &x : queues_) {
      st += x.v;
    }
    c.queue = st / queues_.size();
  }

  void Control::loss(const Measurement &m) {
    loss_smooth_ += kLossSmoothGain * (std::max(0.0, m.loss) - loss_smooth_);
    // The loss the parity is decided with remembers the last peak. The one that follows a
    // TCP download arrives in bursts —every ten seconds, a few hundred milliseconds— and the
    // mean goes back to zero between one and the next: the parity was at the minimum just when the next
    // burst arrived.
    //
    // Only if the loss repeats: a burst that arrives before double the memory has passed
    // since the previous one. The loss of a capacity drop is a single one, and remembering it left the
    // parity high —and the picture low— long after the loop had settled.
    const bool in_burst = loss_smooth_ > kParityBurst;
    if (in_burst && !in_burst_ && m.t - t_burst_end_ >= kBurstGapS) {
      recurring_ = m.t - t_burst_ < 2.0 * tun::kMemory;
      t_burst_ = m.t;
    }
    if (!in_burst && in_burst_) {
      t_burst_end_ = m.t;
    }
    in_burst_ = in_burst;
    if (recurring_ && m.t - t_burst_ < 2.0 * tun::kMemory) {
      loss_peak_ = std::max(loss_smooth_, loss_peak_ * std::exp(-p_.Tc / tun::kMemory));
    } else {
      loss_peak_ = loss_smooth_;
    }
    // The parity minimum drops only far from any loss and from startup: with a download in the
    // pipe the loss returns every few seconds, and a block with a single parity packet does not
    // withstand the burst of the overflowing queue.
    if (m.loss > 0) {
      t_loss_ = m.t;
    }
    const bool clean = p_.fec_min_clean >= 0 && t_start_ >= 0 &&
                       m.t - std::max(t_loss_, t_start_) > tun::kMemory;
    min_parity_ = clean ? std::min(p_.fec_min_clean, p_.fec_min_packets) : p_.fec_min_packets;
    // And at startup, before anything has been measured, it protects as if there were loss. The first
    // second goes out at a budget nobody verified, and if another flow already fills the pipe a quarter
    // of what is sent is lost just when the parity is at the minimum, because it has not
    // yet measured loss: each frame broken there is one more keyframe.
    if (t_start_ < 0) {
      t_start_ = m.t;
    }
    if (m.t - t_start_ < kParityStartupS) {
      loss_peak_ = std::max(loss_peak_, kParityStartup);
    }
  }

  // The queue target, tied to the PATH and not to the loop: with a fixed 10 ms and a 250 ms trip
  // the queue empties 54 % of the time and the loop goes blind. The target is not a constant.
  void Control::queue_target(const Measurement &m, Cycle &c) {
    // With the round-trip floor, not its high percentile: when the round trip is the only
    // delay measure —ENet's, in GameStream— it includes the queue, and a target tied to the
    // high percentile grew with the queue itself, which grew with the target.
    const double trip_base = trips_.empty() ? m.rtt : percentile_within(trips_, kTripPercentileDref, trips_.back().t, tun::kEstimationWindow);
    parity_p_.parity_block_failure = block_failure_target(p_, trip_base);
    c.d.parity_failure = parity_p_.parity_block_failure;
    //
    // And never below the path noise: what the queue rises and falls by itself is not the loop's own queue. Tied to the
    // noise and not to the queue, the target does not grow with the queue the loop itself builds.
    // Never below one frame time: a frame's own dispersion makes the queue move by about that much.
    const double frame_time = 1.0 / p_.fps;
    c.dref = std::max({frame_time, tun::kQueueTargetShare * (trip_base + frame_time), p_.dref_noise * m.noise});
    dref_last_ = c.dref;
    estimate(m, c.queue, c.dref);
  }

  // ΔR = −k_g·(d_k − d_{k−1}) − k_p·T_c·(d_k − d_ref), with the gains recomputed from the
  // estimates and a dead zone that is the width of the meter's cloud.
  void Control::control_law(Cycle &c) {
    const double C = C_est_;
    c.tau = tau_est_;
    c.kg = s_.kg_n * C / c.tau;
    c.kp = s_.kp_n * C / (c.tau * c.tau);

    // The dead zone is not a value to choose. It is the fit's own dispersion, divided by the root of
    // the evidence: it narrows by itself when there are more samples.
    const double n = static_cast<double>(std::max<size_t>(1, queues_.size()));
    c.dead_zone = std::min(kMaxDeadZone, 2.0 * c.residual / std::sqrt(n));

    if (first_) {
      first_ = false;
      c.startup = true;
      c.reason = Reason::startup;
      return;
    }
    const double level = std::fabs(c.queue - c.dref) > c.dead_zone ? (c.queue - c.dref) : 0.0;
    c.dR = -c.kg * c.delta - c.kp * p_.Tc * level;
    c.reason = std::fabs(c.delta) > kGrowingPerCycle ? Reason::queue_growing : (level != 0.0 ? Reason::queue_high : Reason::steady);
  }

  // The backoffs: each one cuts the budget to what some measurement says the pipe delivers.
  void Control::back_off_to(const Measurement &m, Cycle &c) {
    const double queue = c.queue, dref = c.dref;
    // ── the loss backoff ─────────────────────────────────────────────────────
    // Credible loss WITH a full queue is overflow: the bottleneck is already
    // dropping. With the queue empty, the same loss does not move a bit — that is the repair
    // loop's job.
    //
    // "Full", not "growing": the queue stops growing when the buffer fills, and by the
    // time the loss is credible the growth has already disappeared. Asking for both
    // together makes the backoff never fire.
    //
    // And a backoff erases what the ramp-up curve remembered. The anchor is the rate at which the last queue appeared;
    // after an overflow that rate is proven not to fit, and if the
    // curve stayed alive, as soon as the queue dropped it went back in one step to the old anchor. With a
    // download in the pipe, from 1.9 to 10.7 Mbit/s in one cycle, right before
    // going from the menu to the action.
    if (loss_smooth_ > kBackoffLoss && queue > dref + kBackoffQueueMargin) {
      c.dR = std::min(c.dR, -(R_ - tun::kUtilization * C_est_));
      c.reason = Reason::loss_backoff;
    }
    // The same backoff, without waiting for the queue. When the pipe narrows at once, the queue
    // the loop sees arrives a quarter of a second late —the report, the trip, the filter—, and by
    // then the buffer has already overflowed. But the narrow pipe shows up earlier somewhere else: noticeably
    // less arrives than what was sent a trip earlier, even though nothing is lost yet. And
    // what arrives at that moment is exactly the new pipe, while Ĉ still
    // remembers the old one.
    //
    // What has to be real is the sending, not its ratio to R: on leaving a menu
    // R is the unverified value it started from, and what is sent may fall just
    // below any fraction of it. It is enough that it was well above the floor.
    //
    // Only with the queue still below double the target, and not twice in a row. With
    // another flow in the pipe what arrives stays below what leaves all the time —the other
    // takes its share— and the backoff fired every cycle, each time on what was left:
    // a ratchet that gave the whole pipe away to a download. Anticipating the queue only makes
    // sense while the queue has not built up; once built, the backoff above rules. Looking at whether what arrives
    // collapsed does not tell them apart: TCP's sawtooth collapses it too.
    //
    // Double and not the target itself because the report arrives late: when the pipe narrows, the
    // queue that is seen has already passed the target by the time what arrives shows the drop, and with the guard
    // at the target the backoff did not fire right in the drop —on a drop from 12 to
    // 4 Mbit/s, the queue reached 264 ms before the loss backoff cut it—. Above
    // double the ratchet against the download returns.
    //
    // And the pipe the last frames crossed warns even earlier than what arrives, which is
    // an average over a quarter of a second. The same goes for the target: right after the drop
    // what arrives still drags the old pipe, and jumping there left the queue filling.
    // With a floor at half of what arrived, for the same reason as the train bandwidth: a
    // frame processed late is not a narrow pipe. Half and not all because right after
    // a drop what arrives still drags the old pipe.
    double recent_bandwidth =
      kBackoffTrains > 0 && c.recent_trains >= kBackoffTrains && c.recent_span > 0 ? c.recent_bits / c.recent_span : 0.0;
    if (recent_bandwidth > 0) {
      recent_bandwidth = std::max(recent_bandwidth, kNarrowerFloor * m.received_rate);
    }
    c.recent_pipe = recent_bandwidth;
    // One cycle alone may be the report that was delayed, not the pipe: it is required to repeat.
    const bool shortfall = p_.delivery_full > 0 && m.delivery > 0 ? m.delivery < 1.0 - tun::kRateNoise : m.received_rate > 0 && m.received_rate < (1.0 - tun::kRateNoise) * sent_then_;
    shortfall_streak_ = shortfall ? shortfall_streak_ + 1 : 0;
    const bool shortfall_confirmed = shortfall_streak_ >= std::max(1, kFastBackoffCycles);
    // And only with no queue yet. With another flow in the queue the packets of each frame go out
    // interleaved with its own, and the dispersion measures the part of the pipe that belongs to the video,
    // not the pipe: "crossed a narrow pipe" became true all the time, and each backoff
    // ceded more pipe to a download. A drop from steady state starts with the queue empty.
    const bool narrower_now = recent_bandwidth > 0 && queue < dref &&
                              recent_bandwidth < (1.0 - tun::kRateNoise) * R_;
    narrower_streak_ = narrower_now ? narrower_streak_ + 1 : 0;
    const bool crossed_narrower = narrower_streak_ >= std::max(1, kNarrowerCycles);
    if (tun::kRateNoise > 0 && m.received_rate > 0 && sent_then_ > 2.0 * bitrate_floor() && queue < kFastBackoffQueue * dref && m.t - t_fast_backoff_ > tun::kEstimationWindow && (shortfall_confirmed || crossed_narrower)) {
      double target = tun::kUtilization * std::min(C_est_, m.received_rate);
      if (recent_bandwidth > 0) {
        target = std::min(target, tun::kUtilization * recent_bandwidth);
      }
      if (R_ > target) {
        c.dR = std::min(c.dR, -(R_ - target));
        c.reason = Reason::loss_backoff;
        t_fast_backoff_ = m.t;
      }
    }

    // ── sustained loss without a queue ───────────────────────────────────────
    // The only delay GameStream gives is ENet's round trip, which travels on the
    // control channel and not on the video's. Behind a queue that shares per flow —fq_codel, CAKE: the
    // routers with queue management— the control has its own queue and does not see the video's, which
    // fills all the same: the believed queue stays at zero with two hundred milliseconds of real queue, and
    // the loss backoff, which needs to see the queue, never fires. The random loss of a
    // link does not last like that; if it is sustained above what the parity covers, it is the queue.
    //
    // And from there on the delay is not evidence that there is room: the ramp-up does not accelerate.
    if (kCongestionLoss > 0 && loss_smooth_ > kCongestionLoss && queue <= dref) {
      if (t_loss_no_queue_ < 0) {
        t_loss_no_queue_ = m.t;
      }
      if (m.t - t_loss_no_queue_ >= kCongestionLossS && m.received_rate > 0) {
        const double target = tun::kUtilization * std::min(C_est_, m.received_rate);
        if (R_ > target) {
          c.dR = std::min(c.dR, -(R_ - target));
          c.reason = Reason::loss_backoff;
        }
        t_loss_no_queue_ = -1;
        blind_until_ = m.t + kBlindS;
      }
    } else {
      t_loss_no_queue_ = -1;
    }

    // ── the backoff on the queue step ─────────────────────────────────────────
    // A pipe that narrows to a third overflows a router's buffer in a fifth of a second:
    // before the first loss arrives, and before the filtered queue passes the target by
    // much. What is seen in that same cycle is the raw queue sample hitting a step,
    // and what arrives, which is already the new pipe. It backs off there, and lower than in the other backoffs:
    // the queue that built up has to empty before the keyframe the client
    // is going to ask for goes out, or that is lost too and it asks again.
    //
    // A TCP download builds the queue one packet per round trip, without steps; and the
    // backoff does not repeat before two seconds.
    const double raw_queue = c.raw_queue;
    if (kBackoffStep > 0 && raw_queue - raw_queue_prev_ > kBackoffStep && raw_queue > 2.0 * dref && m.received_rate > 0 && m.t - t_fast_backoff_ > tun::kEstimationWindow) {
      // The new pipe: what arrives is still the old one, but the queue already grows at (R−C)/R. With the
      // smaller of the last two growths: a path that lengthens raises the queue by one step
      // and leaves it flat, and read as a slope it cut the budget to almost nothing.
      const double growth_share = std::min(kMaxGrowthShare, std::max(0.0, std::min(raw_queue - raw_queue_prev_, raw_queue_prev_ - raw_queue_prev2_) / p_.Tc));
      const double new_pipe = std::min({C_est_, app_limited_ ? C_est_ : m.received_rate, R_ * (1.0 - growth_share)});
      const double target = kBackoffStepFactor * new_pipe;
      if (R_ > target) {
        c.dR = std::min(c.dR, -(R_ - target));
        c.reason = Reason::loss_backoff;
        t_fast_backoff_ = m.t;
        // This one does not ask for a keyframe: it does not tell apart a download building its queue from a pipe that
        // narrows, and a keyframe inside a download's queue is lost, and the client
        // asks for one per frame. The alarm does tell them apart.
      }
    }
  }

  void Control::apply(const Measurement &m, Cycle &c) {
    // Without evidence it does not rise. With the encoder delivering a fraction of the budget, the
    // queue is empty because WE ARE NOT SENDING, not because there is room. The proportional term
    // cannot tell: it sees the queue far below the target and orders a rise, about 73 kbit/s per
    // cycle with the derived gains — eleven megabits in fifteen seconds of menu, against a pipe
    // that did not change.
    //
    // It is cut in one direction on purpose. Rising without evidence is inventing pipe;
    // dropping without evidence is prudent, and it is also what has to be done if while the
    // encoder is silent the network got worse.
    if (app_limited_ && c.dR > 0) {
      c.dR = 0;
      if (c.reason != Reason::loss_backoff) {
        c.reason = Reason::no_evidence;
      }
    }

    c.before = R_;
    R_ = R_ + c.dR;
    // Dropping empties the queue; dropping below what the pipe delivers only empties it faster, and
    // past a point all it adds is a worse picture. When the pipe narrows, the frames
    // already encoded for the old one keep going out for a couple of cycles, the queue
    // keeps growing, and the growth term kept cutting: on a drop from 12
    // to 4 Mbit/s, the budget ended at the minimum with the pipe delivering 4.25. And with the
    // budget at the minimum, those same frames took even longer to go out.
    //
    // The floor is a fraction of what arrives —of the pipe of the last frames, if smaller, because
    // right after a drop what arrives still drags the old pipe—, and does not apply in the step
    // of a backoff, which already chose its target with the same numbers.
    if (kDeliveredFloor > 0 && c.dR < 0 && c.reason != Reason::loss_backoff && !app_limited_ && m.received_rate > 0) {
      const double delivery = c.recent_pipe > 0 ? std::min(m.received_rate, c.recent_pipe) : m.received_rate;
      R_ = std::max(R_, std::min(c.before, kDeliveredFloor * delivery));
    }
    if (R_ < bitrate_floor()) {
      R_ = bitrate_floor();
      c.d.capped = true;
    }
    if (R_ > p_.bitrate_max) {
      R_ = p_.bitrate_max;
      c.d.capped = true;
    }
    if (c.queue <= c.dref) {
      R_healthy_ = R_;
    }
    if (c.reason == Reason::loss_backoff || loss_smooth_ > kParityBurst) {
      startup_done_ = true;
    }
    // A backoff decided here supersedes the alarm's pre-alarm state: retracting it would undo the backoff.
    if (c.reason == Reason::loss_backoff) {
      alarm_open_ = false;
    }
  }

  // Hysteresis: to declare the queue empty it has to drop well below what it took
  // to declare it full.
  void Control::ramp(const Measurement &m, Cycle &c) {
    c.has_queue = had_queue_ ? (c.queue > c.dref * tun::kRateNoise) : (c.queue > c.dref);
    if (c.has_queue) {
      if (!had_queue_) {
        // the anchor is the rate at the EDGE where the queue appeared. Touching it every cycle
        // collapses it; not letting it drop takes the queue peak to 62 ms.
        const double edge = R_;
        if (edge > anchor_) {
          anchor_ = edge;
        } else if (edge < anchor_ * kAnchorCollapse) {
          anchor_ = kAnchorBlend * (anchor_ + edge);
        }
      }
      in_ramp_ = false;
      t_last_queue_ = m.t;
      startup_done_ = true;
    } else if (!app_limited_ && c.reason != Reason::loss_backoff) {
      // Never in the same step as a backoff: the curve returned the budget to where it was before
      // the backoff, and the backoff did not lower anything.
      //
      // And if the train measures more pipe than the anchor, the anchor is the train. Without this, past the curve the
      // rise is exploration —a fixed percentage of the old anchor per second—, and
      // from a 4 to a 12 Mbit/s pipe, twenty-five seconds went by at 7. The train
      // knows how much pipe there is without having filled it; with the queue empty, which is the only way to
      // get here, there is no other flow to refute it.
      //
      // But only if the last queue was several seconds back. If the queue comes and goes, the anchor
      // that its edge left is the one that counts: behind a router that lets bursts through the train comes out
      // inflated every time the queue empties, and pointing the curve there made it crash again and again.
      if (kTrainAnchor > 0 && train_bandwidth_ > 0 && m.t - t_last_queue_ > kTrainAnchorWait) {
        anchor_ = std::max(anchor_, std::min(p_.train_utilization * train_bandwidth_, kTrainAnchor * R_));
      }
      // With the queue empty it feels its way upward. But if the queue is empty because WE ARE
      // SENDING NOTHING, rising is inventing: there is no evidence at all that there is room.
      ramp_up_step();
      if (c.reason == Reason::steady) {
        c.reason = Reason::ramping;
      }
    } else if (app_limited_) {
      in_ramp_ = false;  // when the scene returns, the curve starts from zero
      if (c.reason == Reason::steady) {
        c.reason = Reason::no_evidence;
      }
    }
  }

  // A movie in steady state downloads a chunk every few seconds at the pipe's full speed and
  // fills the router's queue while it lasts. For the loop it is the same as a drop: it cuts, and what
  // arrives meanwhile —the little it sends— enters Ĉ as if it were the pipe. When the movie
  // finishes the chunk, the queue disappears at once, but Ĉ and the anchor stayed at the minimum, and in
  // the two seconds of calm the rise gets nowhere: the video
  // stayed at 1 Mbit/s with 5 free.
  //
  // What tells another's queue apart is what it does after the cut. One's own, from a
  // drop, empties: more arrives than is sent. Another's stays high and still while
  // the same arrives as is sent —the video passes whole, behind the cross queue—. If that lasts and the
  // episode ends by itself and short, the pipe did not change and it goes back to what it was. A download
  // with no pause does not end, and there the law keeps sending.
  //
  // Ending is truly emptying, against the delay minimum and not against the competitor's
  // floor: with a continuous download that floor is a percentile of the window, and the queue measured against it
  // empties at times with two hundred milliseconds of queue. Read as the end of a chunk of movie,
  // the budget explored up to the maximum inside the queue of a game's download.
  void Control::cross_queue(const Measurement &m, Cycle &c) {
    const double true_queue = std::max(0.0, m.owd_mean - c.delay_floor_min);
    if (c.queue <= c.dref) {
      C_healthy_ = C_est_;
    }
    {
      if (c.has_queue && !had_queue_) {
        episode_ = {true, m.t, R_healthy_, C_healthy_, 0.0};
      }
      if (c.has_queue) {
        after_cross_ = false;
      }
      if (c.has_queue && episode_.active && !app_limited_ && m.received_rate > 0 && sent_then_ > 0) {
        const double r = m.received_rate / sent_then_;
        const bool same = std::fabs(r - 1.0) < kCrossTolerance;
        const bool still = true_queue > 2.0 * c.dref && c.raw_queue >= raw_queue_prev_ - kStillQueueS;
        // One's own queue in equilibrium also looks still and with everything arriving; the difference is that
        // one's own empties if lowered, and this one already survived dropping to half, or to the floor
        const bool lowering = R_ <= std::max(0.5 * episode_.R, kCrossFloorOverMin * bitrate_floor());
        if (same && still && lowering) {
          episode_.cross += p_.Tc;
        }
      }
      if (!c.has_queue && had_queue_ && episode_.active) {
        const bool cross = episode_.cross >= kCrossS && m.t - episode_.t0 <= kCrossMaxS &&
                           true_queue < c.dref;
        if (cross && episode_.R > R_) {
          R_ = std::min(p_.bitrate_max, episode_.R);
          C_est_ = std::max(C_est_, episode_.C);
          rates_.clear();
          // And the train starts over: the frames that crossed behind another's queue went out
          // interleaved with its packets, and their dispersion measures the part that fell to the video, not the
          // pipe. Nor is the earlier measurement any use: in a calm of a couple of seconds no new
          // samples come in, and the first one stayed as the ceiling forever.
          train_bandwidth_ = 0;
          trains_.clear();
          anchor_ = std::max(anchor_, R_);
          in_ramp_ = false;
          c.d.reanchored = true;
        }
        after_cross_ = cross;
        episode_.active = false;
      }
    }
    had_queue_ = c.has_queue;
    raw_queue_prev2_ = raw_queue_prev_;
    raw_queue_prev_ = c.raw_queue;
  }

  void Control::ceilings(const Measurement &m, Cycle &c) {
    // Loss counts as a sign of a full pipe.
    c.loss_evidence = loss_smooth_ > kBackoffLoss * 0.5;
    // A frame that crossed without spreading out proves the pipe is at least its rate. With the queue
    // nearly empty, not growing and no loss, that proof is room: the ceiling and the anchor move to it at
    // once and the budget climbs toward it by at most kQuietFrameProof per cycle. Only after a
    // while without a queue and when the proof shows room above the budget: near the pipe a proof
    // only adds queue.
    // On the way back from a deep cut the budget sits at the floor, where frames are too small for the usual proof
    // and none could ever come: frames of fewer packets count while the budget is far below what it was.
    const bool way_back = m.t - t_cut_ < kWayBackS && R_ < kWayBackRatio * budget_before_cut_;
    const double proof = m.proof_bps > 0 || !way_back ? m.proof_bps : m.proof_back_bps;
    const double margin = way_back && kWayBackMargin > 0 ? kWayBackMargin : tun::kPaceOverPipe;
    if (proof > 0 && !app_limited_ && startup_done_ && quiet_frame_proof_room(p_, c.raw_queue, c.dref, m.t - t_last_queue_, p_.train_utilization * proof, R_, margin) && c.delta <= 0 && loss_smooth_ < kParityBurst && c.reason != Reason::loss_backoff && m.t - t_fast_backoff_ > tun::kEstimationWindow && m.t - t_alarm_ > tun::kEstimationWindow) {
      if (proof > train_bandwidth_) {
        train_bandwidth_ = proof;
        t_bandwidth_ = m.t;
      }
      const double goal = p_.train_utilization * proof;
      if (goal > R_) {
        anchor_ = std::max(anchor_, goal);
        R_ = std::min({p_.bitrate_max, goal, R_ * (1.0 + kQuietFrameProof)});
      }
    }
    // ── the dispersion ceiling (SQP) ───────────────────────────────────────
    // Here the budget does not come from integrating corrections over the delay: it comes from what
    // was measured the pipe delivers, trimmed by the utilization multiplier. That trim leaves
    // permanent slack, and the slack drains the queues by itself.
    if (train_bandwidth_ > 0) {
      // The dispersion measures the LINK's capacity, not the part left free: with another
      // flow in the queue, the packets of a frame still go out spread at the pipe's
      // speed. If it replaced the law, the loop would ignore any competitor —against a
      // download it stayed with 16 of 20 Mbit/s and broke 6 % of the frames—. As a ceiling
      // over the law it contributes what the law lacks —knowing how much pipe there is without having sent
      // anything, which is the menu case— and leaves to the law what it does know: whether there is a queue.
      //
      // It always brakes the rise, but it lowers the budget only with evidence that the pipe is not
      // enough —queue, loss, what was sent and does not arrive— or with the unverified budget, which is
      // that of the silent encoder. A ceiling below what is being sent with no queue or
      // loss is a measurement the network itself refutes: with scarce CPU the train measured the
      // sender and not the pipe, each sample lowered the budget to what was already being sent, and the
      // budget stayed at half the pipe, rising and falling.
      const double ceiling = p_.train_utilization * train_bandwidth_;
      if (R_ > ceiling) {
        const bool not_enough = pipe_full_ || c.queue > c.dref || c.loss_evidence;
        R_ = (not_enough || app_limited_) ? std::max(bitrate_floor(), ceiling) : std::min(R_, std::max(ceiling, c.before));
        if (c.reason == Reason::steady || c.reason == Reason::ramping) {
          c.reason = Reason::capped;
        }
        anti_windup(R_);
      }
    }

    // ── the ceiling that does not go through the delay floor ───────────────
    // The whole law rests on a single measurement: how far the delay came away from its floor.
    // When that floor is poisoned —and against a flow that never lets the queue drain it is always
    // poisoned, at exactly ten seconds— the law does not degrade: it is left without input, and an
    // integrator without input goes wherever it is pushed.
    //
    // Ĉ is what really reached the client, measured at the low percentile. It does not go through the
    // floor or the delay. Saturating R against that does not replace the law: it lets it operate
    // normally while there is signal, and puts a backstop under it when there is not.
    //
    // It goes AFTER everything that touches R, ramp-up and dispersion included: a ceiling that some
    // branch can bypass is not a ceiling.
    //
    // And it applies only when there is EVIDENCE of a full pipe. The same argument, used the other way round: if
    // everything put on the wire arrives and nothing is lost, the pipe is not full —and that
    // does not go through the floor either—, so saturating there protects from nothing and brakes the loop just
    // when it can grow. Without this guard, recovering from 8 to 20 Mbit/s took 16.5 s instead
    // of 4.3, and the permanent-slack mechanism stayed at 55 % of the pipe.
    const double proven_ceiling = p_.ceiling_over_C * C_est_;
    const bool has_evidence = pipe_full_ || c.loss_evidence;
    if (has_evidence && R_ > proven_ceiling) {
      R_ = std::max(bitrate_floor(), proven_ceiling);
      // Without this the ramp-up curve went on toward its anchor under the ceiling, and the budget
      // jumped every cycle between the ceiling and the curve: on a drop from 12.9 to 8.2 Mbit/s
      // and back, ten times per second. The same anti-windup as the train ceiling.
      anti_windup(R_);
      c.d.capped = true;
      if (c.reason == Reason::ramping || c.reason == Reason::steady) {
        c.reason = Reason::capped;
      }
    }
  }

  void Control::parity_budget(Cycle &c) {
    // ── the single budget ──────────────────────────────────────────────────
    // What goes out on the wire is R, whole. The picture is what is left after paying for the
    // audio, the headers and the parity. That is why moving f does not add a single bit to the pipe.
    const double egress = R_;
    c.overhead = static_cast<double>(p_.mtu) / (p_.mtu + p_.header_bytes);
    if (p_.parity_in_packets) {
      // With the parity decided in packets, what is paid is what the host will put in a frame of the
      // budgeted size: the same per-block computation, over k packets. The minimum of two per
      // block is a fixed cost per frame, not a fraction.
      //
      // Not the one that went out: when the encoder delivers frames smaller than requested, the
      // parity proportion that comes out rises —two parity over three data is 67 %—, and
      // paying it shrank the picture, which shrank the frames, which raised the proportion. With
      // libx264 over an 8 Mbit/s pipe the picture went to half the budget.
      //
      // The largest picture whose parity, computed for its own size, fits in the budget. The
      // parity rises in steps with k, so iterating does not converge: the last pass paid for that of a
      // larger frame, and raising the budget 4 % ended up lowering the picture 12 %.
      const double usable = (egress - p_.audio_bps) * c.overhead;
      const double per_packet = p_.mtu * 8.0 * fps_;
      // The condition is k + parity(k) < usable/per_packet + 1, and parity(k) never decreases, so the
      // largest k that holds is found by bisection.
      const auto parity_of = [&](int k) {
        return frame_parity(k, loss_peak_, std::max(1.0, loss_burst_), parity_p_, min_parity_, p_.fec_min);
      };
      const auto fits_of = [&](int k, int parity) {
        return std::min(k * per_packet, usable - parity * per_packet);
      };
      c.video_rate = kMinVideoRate;
      int hi = std::max(1, static_cast<int>(std::ceil(usable / per_packet)));
      int lo = 0;  // largest k known to fit, 0 if none
      while (lo < hi) {
        const int mid = lo + (hi - lo + 1) / 2;
        if (fits_of(mid, parity_of(mid)) > (mid - 1) * per_packet) {
          lo = mid;
        } else {
          hi = mid - 1;
        }
      }
      if (lo >= 1) {
        c.video_rate = std::max(kMinVideoRate, fits_of(lo, parity_of(lo)));
      }
    } else {
      c.video_rate = std::max(kMinVideoRate, (egress - p_.audio_bps) * c.overhead / (1.0 + p_.fec_min));
    }

    c.d.budget = egress;
    c.d.video_rate = c.video_rate;
  }

  // A fixed-rate encoder with a one-frame buffer (libx264 as Sunshine configures it) delivers about
  // a quarter less than it is asked, so the pipe is left half used and the loop cannot do more: the
  // request is raised in the proportion by which the encoder falls short, so that what it delivers
  // is the budgeted picture.
  //
  // The proportion is learned only while the encoder uses the budget (a menu delivers little because
  // there is little to encode), with no queue and no loss, and under a ceiling: after a scene change
  // the excess over the budget is at most that ceiling, and the law corrects it like any excess.
  //
  // It is released at once on the first sign of congestion (queue above the target, pipe full or a
  // backoff), because asking for more while the pipe narrows multiplies the broken frames of a drop.
  // Loss alone does not release it: on Wi-Fi there is always some, and releasing on it leaves the
  // encoder delivering a sixth less. A queue barely above the target does not either when it is a
  // single step: Wi-Fi bursts move the queue around the target all the time and the compensation
  // would never be learned. Above double the target, or sustained, it releases; a drop passes double
  // in its first report.
  void Control::encoder_compensation(const Measurement &m, Cycle &c) {
    const double queue = c.queue, dref = c.dref;
    queue_over_target_ = queue > dref ? queue_over_target_ + 1 : 0;
    const bool queue_congestion = kEncoderReleaseCycles <= 1 ? queue > dref : queue > 2.0 * dref || queue_over_target_ >= kEncoderReleaseCycles;
    const bool congestion = queue_congestion || pipe_full_ || c.reason == Reason::loss_backoff;
    if (congestion) {
      encoder_efficiency_ = 1.0;
    } else if (p_.encoder_compensation_max > 1.0 && m.data_rate > 0 && request_prev_ > 0 && m.sent_rate >= kAppLimitedThreshold * R_) {
      // What the encoder delivered, in the request's units: data_rate counts the data packets on the wire, headers
      // and all, and the request is the picture's payload. Compared as they come, an encoder delivering 95 % of the
      // request read as delivering all of it.
      const double sample = std::min(1.0, m.data_rate * c.overhead / request_prev_);
      encoder_efficiency_ += p_.Tc / p_.encoder_time_const * (sample - encoder_efficiency_);
    }
    const double efficiency = std::max(1.0 / std::max(1.0, p_.encoder_compensation_max), std::min(1.0, encoder_efficiency_));
    // The compensation never asks for more than the video share of the largest budget.
    const double video_cap = std::max(c.video_rate, (p_.bitrate_max - p_.audio_bps) * c.overhead);
    c.d.encoder_request = std::min(c.video_rate / efficiency, video_cap);
    request_prev_ = c.d.encoder_request;
  }

  void Control::audit(const Measurement &m, Cycle &c) {
    Decision &d = c.d;
    d.repair = p_.fec_min;
    d.reason = c.reason;
    d.queue = c.queue;
    d.delay_floor = delay_floor_cur_;
    d.dref = c.dref;
    d.growth = c.delta;
    d.dead_zone = c.dead_zone;
    d.C_est = C_est_;
    d.tau_est = c.tau;
    d.kg = c.kg;
    d.kp = c.kp;
    d.loss_est = loss_peak_;
    d.min_parity = min_parity_;
    // The length of the channel's bursts. With the queue full the loss is the queue's own,
    // which drops everything that arrives while the overflow lasts: that says nothing about the channel, and if
    // it entered the average the parity would keep protecting against bursts that do not exist.
    const bool overflow = c.queue > c.dref + kBackoffQueueMargin;
    if (m.loss_burst > 0 && !overflow) {
      loss_burst_ = loss_burst_ > 0 ? loss_burst_ + kBurstGain * (m.loss_burst - loss_burst_) : m.loss_burst;
    } else if (tun::kMemory > 0 && m.loss_burst <= 0 && loss_burst_ > 1.0) {
      // A channel that stopped stumbling is forgotten, instead of protecting forever against the last burst.
      loss_burst_ = 1.0 + (loss_burst_ - 1.0) * std::exp(-p_.Tc / tun::kMemory);
    }
    d.loss_burst = std::max(1.0, loss_burst_);
    d.samples = m.samples;
    d.app_limited = app_limited_;
    d.train_bandwidth = train_bandwidth_;
  }

}  // namespace abr
