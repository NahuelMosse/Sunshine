#include "control.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace abr {

const char* name(Variant v) {
  switch (v) {
    case Variant::baseline: return "baseline";
    case Variant::patched:  return "patched";
    case Variant::padding:  return "padding";
    case Variant::train:    return "train";
  }
  return "?";
}

Variant variant_by_name(const std::string& s) {
  if (s == "baseline") return Variant::baseline;
  if (s == "patched")  return Variant::patched;
  if (s == "padding")  return Variant::padding;
  if (s == "train")    return Variant::train;
  throw std::invalid_argument("unknown variant '" + s + "' (baseline, patched, padding, train)");
}

int block_parity(int k, double loss, double burst, const Params& p, int minimum) {
  if (minimum < 0) minimum = p.fec_min_packets;
  const int ceiling = std::min({p.gf_shards - k, (k * 255 + 99) / 100,
                              std::max(minimum, static_cast<int>(std::ceil(k * p.fec_max)))});
  burst = std::max(1.0, burst);
  // Stumbles start at each packet with probability q, and each one takes away a geometric
  // length of mean `burst`: the dwell in the bad state of a Gilbert-Elliott channel.
  // The block is lost if what is lost adds up to more than m. A fixed length ignores the tail of long
  // bursts: with the measured burst at one and a bit, it leaves blocks uncovered against a stumble of three.
  const double q = std::min(0.5, std::max(1e-6, loss) / burst);
  const double r = 1.0 - 1.0 / burst;                    // keep losing within a stumble
  for (int m = minimum; m < ceiling; ++m) {
    const int n = k + m;
    // P(lost ≤ m) = Σ_j P(j stumbles) · P(their lengths add up ≤ m), with j binomial(n, q) and the
    // sum of j geometrics a negative binomial: P(sum = t) = C(t−1, j−1)·(1−r)^j·r^(t−j)
    double lj = n * std::log1p(-q);                      // log P(no stumble)
    double acc = std::exp(lj);
    for (int j = 1; j <= std::min(n, m); ++j) {
      lj += std::log(double(n - j + 1) / j) + std::log(q) - std::log1p(-q);
      double sum = 1.0;                                   // with length 1, j stumbles are j lost
      if (r > 0) {
        sum = 0;
        for (int t = j; t <= m; ++t)
          sum += std::exp(std::lgamma(t) - std::lgamma(j) - std::lgamma(t - j + 1) +
                           j * std::log1p(-r) + (t - j) * std::log(r));
      }
      acc += std::exp(lj) * sum;
    }
    if (1.0 - acc < p.parity_block_failure) return m;
  }
  return std::max(0, ceiling);
}

const char* name(Reason m) {
  switch (m) {
    case Reason::startup:      return "startup";
    case Reason::steady:        return "steady";
    case Reason::no_evidence: return "no_evidence";
    case Reason::queue_growing:    return "queue_growing";
    case Reason::queue_high:     return "queue_high";
    case Reason::loss_backoff: return "loss_backoff";
    case Reason::ramping:     return "ramping";
    case Reason::reanchored:     return "reanchored";
    case Reason::probing:     return "probing";
    case Reason::capped:       return "capped";
  }
  return "?";
}

// ── the derived tuning ────────────────────────────────────────────────────
// Ch. 4: k_g and k_p are not chosen constants, they are a consequence of asking the loop for a
// phase margin and a mix ρ. From the same two quantities come the predictions that
// chapter 10 uses to try to kill the model.
//
// The open loop has TWO integrations, and that is the key to all of chapter 4:
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
constexpr double kPi = 3.14159265358979323846;

Tuning derive(const Params& p) {
  Tuning s;
  const double phi = std::atan(p.rho);
  s.wc_tau = phi - p.phase_margin;                      // ω_c·τ
  s.kg_n = s.wc_tau / std::sqrt(1.0 + 1.0 / (p.rho * p.rho));
  s.kp_n = s.kg_n * s.wc_tau / p.rho;

  const double z = s.kp_n / s.kg_n;                    // the knee of the lead

  // ω_180: where the correction comes back half a turn late. Asking for phase = −π leaves
  // atan(u/z) = u, which has no closed form — bisection.
  auto excess = [&](double u) { return std::atan(u / z) - u; };
  double lo = s.wc_tau, hi = kPi / 2 + 1.0;            // excess(hi) < 0 always
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (excess(mid) > 0) lo = mid; else hi = mid;
  }
  s.w180_tau = 0.5 * (lo + hi);

  // At ω_180 the correction comes back smaller, and that factor is how much the
  // gain can be raised before the loop sustains oscillation. It is a verifiable number.
  auto magnitude = [&](double u) { return s.kg_n * std::sqrt(u * u + z * z) / (u * u); };
  s.gain_margin = 1.0 / magnitude(s.w180_tau);
  s.period_tau = 2.0 * kPi / s.w180_tau;
  return s;
}

// ── window utilities ────────────────────────────────────────────────────
void Control::prune(std::deque<Sample>& v, double t, double window) {
  while (!v.empty() && v.front().t < t - window) v.pop_front();
}

double Control::percentile(const std::deque<Sample>& v, int q) {
  if (v.empty()) return 0;
  std::vector<double> a;
  a.reserve(v.size());
  for (const auto& m : v) a.push_back(m.v);
  const size_t k = std::min(a.size() - 1,
                            static_cast<size_t>(q / 100.0 * a.size()));
  std::nth_element(a.begin(), a.begin() + k, a.end());
  return a[k];
}

bool Control::bandwidth_by_regression(double& bandwidth) const {
  if (trains_.size() < 4) return false;
  // The line of span against size, over the trains that remain. Returns false if the
  // frames did not have different sizes: with all equal, the slope is not determined.
  auto fit_line = [](const std::vector<const Train*>& v, double& fit_slope, double& intercept) {
    if (v.size() < 4) return false;
    double sx = 0, sy = 0;
    for (const Train* x : v) { sx += x->bits; sy += x->span; }
    const double n = static_cast<double>(v.size());
    const double mx = sx / n, my = sy / n;
    double sxx = 0, sxy = 0;
    for (const Train* x : v) {
      sxx += (x->bits - mx) * (x->bits - mx);
      sxy += (x->bits - mx) * (x->span - my);
    }
    if (sxx <= 1e-9 * mx * mx * n) return false;
    fit_slope = sxy / sxx;
    intercept = my - fit_slope * mx;
    return true;
  };
  std::vector<const Train*> v;
  v.reserve(trains_.size());
  for (const Train& x : trains_) v.push_back(&x);
  double fit_slope = 0, intercept = 0;              // seconds per bit, and the intercept
  if (!fit_line(v, fit_slope, intercept)) return false;
  // What delays a frame on the path —the air bursts of a Wi-Fi, a thread that woke
  // up late— only adds span to it, never subtracts: the pipe is the lower edge of the cloud,
  // not its middle. It is fitted again with the trains that stayed below the line.
  for (int i = 0; i < p_.train_envelope_iters; ++i) {
    std::vector<const Train*> below;
    for (const Train* x : v) if (x->span <= intercept + fit_slope * x->bits) below.push_back(x);
    double p2 = 0, o2 = 0;
    if (!fit_line(below, p2, o2)) break;
    v.swap(below);
    fit_slope = p2;
    intercept = o2;
  }
  if (fit_slope <= 0) return false;                 // the span does not grow with size
  const double c = 1.0 / fit_slope;
  // Above the declared ceiling it is measurement noise, not a discovery.
  if (c > p_.bitrate_max * 4) return false;
  bandwidth = std::min(c, p_.bitrate_max);
  return true;
}

double Control::value_at(const std::deque<Sample>& v, double t) {
  if (v.empty()) return 0;
  const Sample* best = &v.front();
  double dist = std::fabs(v.front().t - t);
  for (const auto& m : v) {
    const double d = std::fabs(m.t - t);
    if (d < dist) { dist = d; best = &m; }
  }
  return best->v;
}

// ── the growth filter ────────────────────────────────────────────────────
// Ch. 9: it is NOT fixed. The best value shifts to the left as the trip rises, and a
// fixed value is bad at both extremes.
double Control::filter_time_const(double rtt) const {
  return rtt > p_.filter_rtt_cutoff ? p_.filter_fast : p_.filter_slow;
}

// Ch. 6: average while there are few samples, line fit from ~3 on.
// The fit's residual is literally the width of the meter's cloud, and from it comes
// the dead zone — which thus narrows by itself when there is more evidence.
double Control::fit_slope(double& residual_rms) const {
  residual_rms = 0;
  const size_t n = queues_.size();
  if (n < 2) return 0;

  if (n < static_cast<size_t>(p_.line_fit_samples)) {
    // without evidence for a line: simple difference over the span
    const double dt = queues_.back().t - queues_.front().t;
    return dt > 1e-9 ? (queues_.back().v - queues_.front().v) / dt : 0.0;
  }

  double st = 0, sv = 0;
  for (const auto& m : queues_) { st += m.t; sv += m.v; }
  const double mt = st / n, mv = sv / n;
  double num = 0, den = 0;
  for (const auto& m : queues_) {
    num += (m.t - mt) * (m.v - mv);
    den += (m.t - mt) * (m.t - mt);
  }
  const double b = den > 1e-12 ? num / den : 0.0;
  const double a = mv - b * mt;

  double ss = 0;
  for (const auto& m : queues_) {
    const double e = m.v - (a + b * m.t);
    ss += e * e;
  }
  // n-2 degrees of freedom: two parameters were estimated
  residual_rms = n > 2 ? std::sqrt(ss / (n - 2)) : 0.0;
  return b;
}

// ── the estimates that retune the loop ───────────────────────────────────
// Ch. 5 and ch. 3: they move much more slowly than the loop, or the retuning itself
// destabilizes it. That is why percentiles over long windows and not fast averages.
void Control::estimate(const Measurement& m, double queue, double dref) {
  // With a queue there is information: if there is no queue, what comes out is what we put in and says nothing
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
  //     a pipe that is not there. The variant's indicator is not used because this is a
  //     backstop for all of them, including the one that does not have it.
  //   · It has to be compared against what was sent ONE TRIP EARLIER. The report arrives late,
  //     so the rate it brings corresponds to the past. Against the current rate, every
  //     increase looks like loss — and in a still scene, which goes up and down all the time, that
  //     brought the budget down to a quarter of the pipe.
  sent_rates_.push_back({m.t, m.sent_rate});
  prune(sent_rates_, m.t, 2.0);
  const double sent_then = value_at(sent_rates_, m.t - m.rtt);
  sent_then_ = sent_then;

  const double a = p_.egress_time_const > 0 ? std::min(1.0, p_.Tc / p_.egress_time_const) : 1.0;
  if (m.received_rate > 0) {
    received_smooth_ += a * (m.received_rate - received_smooth_);
    sent_smooth_ += a * (sent_then - sent_smooth_);
  }
  const bool really_sending = sent_then > p_.app_limited_threshold * R_;
  bool stays_in_path =
      really_sending &&
      received_smooth_ < sent_smooth_ * (1.0 - p_.egress_margin);
  // If the measurement brings the delivery of the same frames, that is what says whether the pipe slows down:
  // it does not mix the encoder's variation with the pipe's.
  if (p_.delivery_full > 0 && m.delivery > 0) {
    delivery_smooth_ = delivery_smooth_ > 0 ? delivery_smooth_ + (p_.Tc / 0.5) * (m.delivery - delivery_smooth_)
                                        : m.delivery;
    stays_in_path = really_sending && delivery_smooth_ < p_.delivery_full;
  }
  pipe_full_ = stays_in_path;

  // What arrives only says how much the pipe gives if it was really sending: with a still picture, what
  // arrives is the audio, and learned as a pipe it took the capacity to the floor.
  if (m.received_rate > 0 && ((queue > dref && really_sending) || stays_in_path)) {
    rates_.push_back({m.t, m.received_rate});
  }
  prune(rates_, m.t, p_.capacity_window);

  const double measured = percentile(rates_, p_.capacity_percentile);  // toward the safe side
  if (C_forced_ > 0) {
    C_est_ = C_forced_;
  } else if (measured > 0) {
    C_est_ += (p_.Tc / p_.capacity_time_const) * (measured - C_est_);
  } else if (R_ > C_est_ && !app_limited_ && !stays_in_path) {
    // With no queue we know the capacity is at least what we are sending. With two
    // guards, because the two times this branch was wrong it was by believing that something went through the
    // pipe that did not:
    //   · if the encoder delivered a fraction of the budget, nothing went out and there is
    //     nothing to conclude;
    //   · if it went out but did not arrive, neither: that is not a free pipe, it is a full pipe.
    C_est_ += (p_.Tc / 1.5) * (R_ - C_est_);
  }
  C_est_ = std::max(p_.bitrate_min, C_est_);

  trips_.push_back({m.t, m.rtt});
  prune(trips_, m.t, p_.capacity_window);
  const double trip = percentile(trips_, p_.tau_from_floor ? p_.trip_percentile_dref : p_.trip_percentile);
  const double frame_time = 1.0 / p_.fps;
  tau_est_ = trip + frame_time + p_.Tc / 2.0 + filter_time_const(trip);
}

// ── the coherence check (ch. 5) ─────────────────────────────────────────
// If lowering does not help, it was not a queue: the path changed, and the stored floor is rotten.
// The test is done for real — it is noted where the budget was when the
// descent began and compared a few τ later.
bool Control::check_coherence(double queue, double dref, double delta, double dR) {
  if (naive_) return false;

  // four conditions, and each one rules out a different case
  const bool well_above = queue > dref + p_.coh_margin;
  const bool flat = std::fabs(delta) < p_.coh_flat;
  const bool lowering = dR < 0;

  if (!trial_.active && well_above && lowering) {
    trial_ = {true, t_, R_ - dR, queue};
    return false;
  }
  if (trial_.active && queue <= dref) { trial_.active = false; suspicion_ = 0; return false; }
  if (!trial_.active) return false;

  if (t_ - trial_.t < std::max(1.2, 3.0 * tau_est_)) return false;

  const double dropped_frac   = (trial_.R - R_) / std::max(1.0, trial_.R);
  const double improved_frac = (trial_.queue - queue) / std::max(1e-9, trial_.queue);
  trial_.active = false;

  // we really lowered, the delay did not ease off, no loss explains it, and it is
  // stuck instead of still growing (which would be a real capacity drop)
  const bool is_stale = dropped_frac > 0.10 && improved_frac < 0.15 && loss_smooth_ < 0.02 &&
                     flat && well_above;
  if (!is_stale) { suspicion_ = 0; return false; }
  if (++suspicion_ < p_.coh_cycles) return false;

  suspicion_ = 0;
  delay_floor_.clear();
  trips_.clear();
  delay_floor_fresh_ = t_;
  R_ = R_healthy_;
  queues_.clear();
  return true;
}

// ── the floor probe (ch. 5) ───────────────────────────────────────────
// If it has been a while since any measurement touches the bottom, the queue never emptied and the window's
// minimum swallowed it. It is cut a little, allowed to drain, and re-read.
void Control::probe_delay_floor(double t) {
  if (naive_) return;
  // The dispersion variant does not need it: the target multiplier leaves permanent
  // slack and the queues drain by themselves. SQP points out precisely that halting the send
  // every ten seconds —BBR's ProbeRTT, where this mechanism came from— is
  // especially problematic for interactive streaming.
  if (p_.variant == Variant::train) return;
  if (probing_) {
    if (t > probe_until_) probing_ = false;
    return;
  }
  const int steady_cycles = static_cast<int>(2.5 / p_.Tc);
  if (t - delay_floor_fresh_ > p_.probe_interval && steady_ >= steady_cycles) {
    probing_ = true;
    probe_until_ = t + p_.probe_duration;
    steady_ = 0;
  }
}

// ── ramping up is feeling the way (ch. 4) ─────────────────────────────────
// The law cannot rise by itself: without a queue there is no signal. The timid S is the one that leaves the least latency
// peak over the whole recovery range, and in a game that is what you feel.
// When a ceiling clips the budget, the ramp-up curve cannot keep advancing underneath
// as if nothing happened: it went on toward its anchor —the value it started from, for example— and the
// day the ceiling disappeared the budget jumped at once to the curve's value. With a 4 Mbit/s
// pipe, from 3.3 to 9.3 in one cycle. The anchor drops to the ceiling and the curve starts over from there.
void Control::anti_windup(double ceiling) {
#ifdef ABR_BENCH_DEBUG
  if (std::getenv("ABR_DEBUG_ALARM")) std::fprintf(stderr, "anti_windup t=%.2f R=%.0f ceiling=%.0f C=%.0f\n", t_, R_, ceiling, C_est_);
#endif
  anchor_ = std::min(anchor_, ceiling);
  in_ramp_ = false;
}

void Control::ramp_up_step(double dref, double queue) {
  if (!in_ramp_) {
    in_ramp_ = true;
    ramp_t_ = 0;
    ramp_R0_ = R_;
    if (anchor_ < R_) anchor_ = R_;
  }
  ramp_t_ += p_.Tc;
  const double T = p_.ramp_duration, a = 9.2 / T;
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
    target += anchor_ * (p_.ramp_exploration * x + p_.ramp_convex * xc * xc * xc);
  }
  // Until the first queue of the session nothing says where the pipe is, and the anchor is only the value
  // it started from, which was deliberately chosen low. Meanwhile it rises in proportion to what is
  // already being sent, like BBR's startup: with real Sunshine and a 12 Mbit/s pipe,
  // rising from the anchor the action took thirty-five seconds to reach the pipe.
  //
  // And the same when there has been no queue for a while: the pipe may have grown, and the anchor is the
  // memory of a narrower one. It is the accelerated probing in the recovery that the 8/29 panel asked for.
  // With the curve alone, from a 4 to a 12 Mbit/s pipe, getting there took fifteen seconds.
  // After a cross queue that ended by itself, the calm that follows is short —the movie comes back in
  // seconds— and the deadline cannot be waited out: it is explored right away.
  const bool no_queue = (p_.reexplore_s > 0 && t_ - t_last_queue_ > p_.reexplore_s) || after_cross_;
  if (p_.fast_startup > 0 && (!startup_done_ || no_queue)) {
    target = std::max(target, R_ * (1.0 + p_.fast_startup * p_.Tc));
  }
  R_ = std::max(R_, std::min(p_.bitrate_max, target));
  (void)dref; (void)queue;
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
// like Pudica's drain (NSDI 2024): 0.85 of the pipe minus the queue over the drain time.
bool Control::alarm(double t, double queue, double growth, double received) {
  if (p_.alarm_step <= 0 || received <= 0 || naive_) return false;
  if (growth < p_.alarm_step) return false;
  if (queue < std::max(p_.alarm_queue, 1.3 * dref_last_)) return false;
  // The new pipe: what arrives is still the old one —it measures frames that left earlier—, but the
  // queue's slope is already the new one, because it grows at (R−C)/R. From 12 to 4 Mbit/s, what arrived
  // said 9 and the slope said 5.5.
  const double fit_slope = std::min(0.9, std::max(0.0, growth / 0.05));
  // With a still picture what arrives is the audio: it says nothing about the pipe
  const double pipe = std::min({C_est_, app_limited_ ? C_est_ : received, R_ * (1.0 - fit_slope)});
  const double target = std::max({p_.bitrate_min, 0.25 * pipe,
                                   pipe * (p_.alarm_factor - queue / p_.alarm_drain)});
  // One alarm every two seconds, like the backoffs; but it can fire even if the step's backoff
  // has just fired, if it goes lower: that one looks at what arrives, which at that moment is still the
  // old pipe, and with the common guard the alarm was blocked just when it was needed.
  if (t - t_alarm_ < 2.0) return false;
  if (R_ <= target / 0.9) return false;
  before_alarm_ = {R_, anchor_, t_alarm_, t_loss_, t_fast_backoff_, in_ramp_, startup_done_, min_parity_};
  alarm_open_ = true;
  t_alarm_ = t;
  t_loss_ = t;
  min_parity_ = p_.fec_min_packets;
#ifdef ABR_BENCH_DEBUG
  if (std::getenv("ABR_DEBUG_ALARM")) std::fprintf(stderr, "alarm t=%.2f queue=%.3f growth=%.3f R=%.0f target=%.0f\n", t, queue, growth, R_, target);
#endif
  R_ = target;
  anti_windup(pipe);
  t_fast_backoff_ = t;
  startup_done_ = true;
  return true;
}

bool Control::retract_alarm(double t, double queue) {
  if (!alarm_open_) return false;
  // Only the immediately following sample can retract it: past that span, whatever happens is something else
  if (t - t_alarm_ > p_.alarm_retract_s) { alarm_open_ = false; return false; }
  if (queue >= 0.5 * dref_last_) return false;
  alarm_open_ = false;
  const BeforeAlarm& a = before_alarm_;
  R_ = a.R; anchor_ = a.anchor; t_alarm_ = a.t_alarm; t_loss_ = a.t_loss;
  t_fast_backoff_ = a.t_fast_backoff; in_ramp_ = a.in_ramp; startup_done_ = a.startup_done;
  min_parity_ = a.min_parity;
  return true;
}

// ── the loop ───────────────────────────────────────────────────────────
Control::Control(const Params& p) : p_(p), s_(derive(p)) {
  reset(p.bitrate_max);
}

void Control::reset(double negotiated) {
  // Ch. 5: starting low and rising costs a few seconds of poorer picture; starting
  // high and dropping costs a BROKEN picture plus the risk of triggering the emergency backoff.
  // It is not symmetric.
  R_ = std::max(p_.bitrate_min, std::min(p_.bitrate_max, negotiated * p_.startup_fraction));
  R_healthy_ = R_;
  anchor_ = R_;
  f_ = p_.fec_min;
  C_est_ = negotiated;
  tau_est_ = 0.3;
  delay_floor_.clear(); trips_.clear(); rates_.clear(); queues_.clear(); minima_.clear(); sent_rates_.clear();
  budgets_.clear();
  delay_floor_cur_ = 0; delay_floor_fresh_ = 0; competitor_ = false; t_no_swing_ = -1e9; loss_smooth_ = 0; loss_peak_ = 0; loss_burst_ = 0;
  t_burst_ = -1e9; in_burst_ = false; recurring_ = false; t_start_ = -1;
  t_loss_ = -1e9; min_parity_ = p_.fec_min_packets; fps_ = p_.fps;
  queue_prev_ = 0; raw_queue_prev_ = 0; raw_queue_prev2_ = 0; had_queue_ = false; first_ = true;
  in_ramp_ = false; probing_ = false; steady_ = 0; suspicion_ = 0;
  trial_ = {};
  C_healthy_ = negotiated; episode_ = {}; after_cross_ = false;
  app_limited_ = false;
  pipe_full_ = false;
  train_bandwidth_ = 0;
  trains_.clear();
  t_bandwidth_ = 0;
  t_fast_backoff_ = -1e9;
  shortfall_streak_ = 0;
  delivery_smooth_ = 0;
  delivered_.clear(); t_cruise_ = 0; exploring_ = false;
  t_alarm_ = -1e9;
  startup_done_ = false;
  t_loss_no_queue_ = -1;
  blind_until_ = -1e9;
  encoder_efficiency_ = 1;
  queue_over_target_ = 0;
  request_prev_ = 0;
  sent_smooth_ = received_smooth_ = 0;
  t_last_queue_ = 0;
  t_ = 0;
}

Decision Control::step(const Measurement& m) {
  t_ = m.t;
  Decision d;
  budgets_.push_back({m.t, R_});
  prune(budgets_, m.t, 5.0);
  // Of the pipe that emptied the queue, what can be used: what lets a queue like the one just now empty in drain_deadline.
  // An outage of h seconds sending R leaves R·h in the queue, which empties at C − R:
  // R ≤ C·T/(h + T). Using the whole pipe leaves one outage's queues piling up on the next one's.
  const double usable_drain_rate = m.drain_rate > 0
      ? std::min(p_.train_utilization, p_.drain_deadline / (m.drain_rise + p_.drain_deadline)) * m.drain_rate
      : 0.0;
  // An air outage raises the queue almost as much as time passes —the pipe delivers nothing—; the queue
  // the loop itself builds, at the pace of what is sent in excess, a fraction of that. Only a queue
  // that rose like an outage, on a noisy path, says something the law does not know.
  const bool outage = p_.drain_episode > 0 && m.drain_rate > 0 && m.noise > 0.002 &&
                     m.drain_height >= p_.drain_ramp * std::max(1e-3, m.drain_rise);

  // ── 0. did the encoder use the budget? ─────────────────────────────────
  // It is the first thing to know, because it decides whether what follows is evidence or noise.
  // The host knows it without the client's help: they are the bits it itself put on the wire.
  if (p_.variant != Variant::baseline && p_.variant != Variant::padding) {
    app_limited_ = m.sent_rate > 0 && m.sent_rate < p_.app_limited_threshold * R_;
  } else {
    app_limited_ = false;   // in `padding` it never is: the pacer fills in with dummies
  }

  // The sample by dispersion of the packet train (SQP). It does not depend on the average rate
  // but on how the packets of ONE frame spread out when crossing the bottleneck, so
  // it still works even if the frame is small.
  // Likewise, only with the frames of this report: the pipe the last frames crossed.
  // It is noisy, but it is the first thing that shows the pipe narrowed —before any
  // average of rates—, and with the frames spaced at κ·R it measures the new pipe as soon as it drops
  // below what is being sent.
  double recent_bits = 0, recent_span = 0;
  int recent_trains = 0;
  if (p_.variant == Variant::train &&
      (!m.trains.empty() || (m.train_span > 1e-6 && m.train_bytes > 0))) {
    // A frame of two or three packets has no dispersion to measure: what is read is
    // the clock noise, not the bottleneck. SQP acknowledges it and it is its known
    // limit. With fewer packets than this floor, the sample is discarded.
    auto add_train = [&](double bytes, double span) {
      const double packets = bytes / static_cast<double>(p_.mtu + p_.header_bytes);
      if (span > 1e-6 && packets >= p_.train_min_packets) {
        trains_.push_back({m.t, bytes * 8.0, span});
        // n packets open n − 1 gaps
        recent_bits += bytes * 8.0 * (packets - 1.0) / packets;
        recent_span += span;
        ++recent_trains;
      }
    };
    if (!m.trains.empty()) {
      for (const auto& tr : m.trains) add_train(tr.bytes, tr.span);
    } else {
      add_train(m.train_bytes, m.train_span);
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
    while (!trains_.empty() && m.t - trains_.front().t > p_.train_window) trains_.pop_front();
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
      if (m.received_rate > 0) bandwidth = std::max(bandwidth, m.received_rate);
      const double dt = std::max(0.0, m.t - t_bandwidth_);
      if (p_.train_rise_max > 0 && train_bandwidth_ > 0 && bandwidth > train_bandwidth_) {
        bandwidth = std::min(bandwidth, train_bandwidth_ * (1.0 + p_.train_rise_max * dt));
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
  if (p_.variant == Variant::train && train_bandwidth_ > 0 && m.t - t_bandwidth_ > p_.train_expiry) {
    train_bandwidth_ = 0;
  }

  // ── 1. the delay floor: the reference for the whole measurement ────────────
  // Ch. 6: minimum over 10 s, same value as BBR and same reasons. The client's
  // clock has an unknown offset, so the absolute level means nothing;
  // the only usable thing is how far it came away from the recent minimum.
  delay_floor_.push_back({m.t, m.owd_min});
  prune(delay_floor_, m.t, p_.delay_floor_window);
  delay_floor_cur_ = delay_floor_.front().v;
  for (const auto& x : delay_floor_) delay_floor_cur_ = std::min(delay_floor_cur_, x.v);
  if (m.owd_min < delay_floor_cur_ + 0.003) delay_floor_fresh_ = m.t;
  const double delay_floor_min = delay_floor_cur_;
  // With another flow filling the pipe the queue never empties, and the ten-second minimum stays
  // wherever the other's deepest valley leaves it: a TCP that backs off a lot lowers the floor, the loop believes
  // the other's queue is its own and yields the pipe; one that backs off little leaves it high and the loop competes.
  // What gives the other away is not the level but the swing: its sawtooth moves the window's delay
  // considerably more than our own queue, which the loop keeps at tens of milliseconds. With
  // that swing the floor becomes a low percentile of the window, which does not depend on how deep the valleys are.
  // A swing of our own —a drop, a route jump— lasts as long as the loop takes to empty
  // its queue; the other's, as long as the other lasts: only one sustained for competitor_sustained counts.
  // Our own queue moves with its target, which grows with the trip: on a long path the threshold is
  // competitor_dref times that target, and the swing is our own unless something says otherwise. Loss
  // says it —a flow guided by loss fills the buffer until it overflows— in the last
  // competitor_loss_s; or, if the loop already went so low that it does not even lose, the low percentile
  // lifted off the minimum by more than the target: our own queue does not occupy three quarters of the window.
  competitor_ = false;
  if (p_.competitor_threshold > 0 && delay_floor_.size() >= 10) {
    std::vector<double> v;
    v.reserve(delay_floor_.size());
    for (const auto& x : delay_floor_) v.push_back(x.v);
    const size_t j = v.size() * 9 / 10;
    std::nth_element(v.begin(), v.begin() + j, v.end());
    const double threshold = std::max(p_.competitor_threshold, p_.competitor_dref * dref_last_);
    if (v[j] - delay_floor_cur_ <= threshold) {
      t_no_swing_ = m.t;
    } else if (m.t - t_no_swing_ >= p_.competitor_sustained) {
      const size_t k = std::min(v.size() - 1, v.size() * static_cast<size_t>(p_.competitor_percentile) / 100);
      std::nth_element(v.begin(), v.begin() + k, v.end());
      const bool lost_recently = p_.competitor_loss_s <= 0 || m.t - t_loss_ < p_.competitor_loss_s;
      if (lost_recently || v[k] - delay_floor_cur_ > dref_last_) {
        competitor_ = true;
        delay_floor_cur_ = v[k];
      }
    }
  } else {
    t_no_swing_ = m.t;
  }

  const double raw_queue = std::max(0.0, m.owd_mean - delay_floor_cur_);
  // On a noisy path, the law looks at the lower envelope: the minimum of the last
  // noise_window. An air outage builds a real queue —the pipe drops to almost nothing for a couple of
  // tenths of a second— but goes away by itself, and counted whole it lowered the budget at once every two seconds
  // without the ramp-up, which follows the timid curve, recovering it before the next outage. A queue
  // that is sustained shows up with that delay; the alarm and the backoff, which look at the raw sample,
  // do not change.
  double law_queue = raw_queue;
  minima_.push_back({m.t, raw_queue});
  prune(minima_, m.t, p_.noise_window);
  if (m.noise > 0.002) {
    double minimum = raw_queue;
    for (const auto& x : minima_) minimum = std::min(minimum, x.v);
    law_queue = minimum;
  }
  queues_.push_back({m.t, law_queue});
  // And with noise, the line is fitted over more time: a hundred-millisecond jump dominated a
  // window of two hundred fifty and was read as the queue growing.
  prune(queues_, m.t, m.noise > 0.002 ? std::max(filter_time_const(m.rtt), p_.noise_filter) : filter_time_const(m.rtt));

  double residual = 0;
  const double queue_slope = fit_slope(residual);  // line fit, ch. 6
  const double delta = queue_slope * p_.Tc;        // growth per cycle
  // the queue the loop uses is the fit's, not the raw sample
  double queue = 0;
  { double st = 0; for (const auto& x : queues_) st += x.v; queue = st / queues_.size(); }

  loss_smooth_ += 0.30 * (std::max(0.0, m.loss) - loss_smooth_);
  // The loss the parity is decided with remembers the last peak. The one that follows a
  // TCP download arrives in bursts —every ten seconds, a few hundred milliseconds— and the
  // mean goes back to zero between one and the next: the parity was at the minimum just when the next
  // burst arrived.
  //
  // Only if the loss repeats: a burst that arrives before double the memory has passed
  // since the previous one. The loss of a capacity drop is a single one, and remembering it left the
  // parity high —and the picture low— long after the loop had settled.
  const bool in_burst = loss_smooth_ > p_.parity_burst;
  if (in_burst && !in_burst_) {
    recurring_ = m.t - t_burst_ < 2.0 * p_.parity_memory;
    t_burst_ = m.t;
  }
  in_burst_ = in_burst;
  if (p_.parity_memory > 0 && recurring_ && m.t - t_burst_ < 2.0 * p_.parity_memory) {
    loss_peak_ = std::max(loss_smooth_, loss_peak_ * std::exp(-p_.Tc / p_.parity_memory));
  } else {
    loss_peak_ = loss_smooth_;
  }
  // The parity minimum drops only far from any loss and from startup: with a download in the
  // pipe the loss returns every few seconds, and a block with a single parity packet does not
  // withstand the burst of the overflowing queue.
  if (m.loss > 0) t_loss_ = m.t;
  const bool clean = p_.fec_min_clean >= 0 && t_start_ >= 0 &&
                      m.t - std::max(t_loss_, t_start_) > p_.fec_clean_s;
  min_parity_ = clean ? std::min(p_.fec_min_clean, p_.fec_min_packets) : p_.fec_min_packets;
  // And at startup, before anything has been measured, it protects as if there were loss. The first
  // second goes out at a budget nobody verified, and if another flow already fills the pipe a quarter
  // of what is sent is lost just when the parity is at the minimum, because it has not
  // yet measured loss: each frame broken there is one more keyframe.
  if (t_start_ < 0) t_start_ = m.t;
  if (m.t - t_start_ < p_.parity_startup_s) {
    loss_peak_ = std::max(loss_peak_, p_.parity_startup);
  }

  // ── 2. the queue target, tied to the PATH and not to the loop ──────────────
  // Ch. 3: with a fixed 10 ms and a 250 ms trip the queue empties 54 % of the time and the
  // loop goes blind. The target is not a constant.
  //
  // With the round-trip floor, not its high percentile: when the round trip is the only
  // delay measure —ENet's, in GameStream— it includes the queue, and a target tied to the
  // high percentile grew with the queue itself, which grew with the target.
  const double trip_base = trips_.empty() ? m.rtt : percentile(trips_, p_.trip_percentile_dref);
  //
  // And never below the path noise: what the queue rises and falls by itself is not our own queue. Tied to the
  // noise and not to the queue, the target does not grow with the queue the loop itself builds.
  const double dref = naive_
      ? p_.dref_floor
      : std::max({p_.dref_floor, p_.dref_fraction * (trip_base + 1.0 / p_.fps), p_.dref_noise * m.noise});

  dref_last_ = dref;
  // The capacity learns from the queue that is sustained over the last capacity_sustained samples, not
  // from a peak: a single jitter sample above the target took it at once to what the loop
  // was sending, and it never left there, because it measures with what the loop sends. A real queue does not empty
  // between two samples; jitter does.
  double sustained_queue = queue;
  {
    int n = 0;
    for (auto it = queues_.rbegin(); it != queues_.rend() && n < p_.capacity_sustained; ++it, ++n) {
      sustained_queue = std::min(sustained_queue, it->v);
    }
  }
  estimate(m, sustained_queue, dref);
  probe_delay_floor(m.t);

  // ── 3. the gains, recomputed ─────────────────────────────────────────────
  const double C = naive_ ? p_.bitrate_max : C_est_;
  const double tau = naive_ ? 0.2 : std::max(0.05, tau_est_);
  const double kg = scale_ * s_.kg_n * C / tau;
  const double kp = scale_ * s_.kp_n * C / (tau * tau);

  // ── 4. the dead zone: the width of the meter's cloud (ch. 6) ──────────────
  // It is not a value to choose. It is the fit's own dispersion, divided by the root of
  // the evidence: it narrows by itself when there are more samples.
  const double n = static_cast<double>(std::max<size_t>(1, queues_.size()));
  const double dead_zone = naive_ ? 0.0 : std::min(0.020, 2.0 * residual / std::sqrt(n));

  // ── 5. THE LAW (ch. 9) ───────────────────────────────────────────────────
  //   ΔR = −k_g·(d_k − d_{k−1}) − k_p·T_c·(d_k − d_ref)
  double dR = 0;
  double recent_pipe = 0;        // the pipe of the last frames, if it could be measured
  Reason reason = Reason::steady;
  if (first_) {
    first_ = false;
    reason = Reason::startup;
  } else if (naive_) {
    // chapter 4's counterfactual: it looks at the VALUE, not whether it grows. Two integrations
    // without a derivative are not stabilized by any gain.
    dR = -kg * 0.8 * (queue - dref);
    if (loss_smooth_ > 0.02) dR -= R_ * 0.22;
    reason = Reason::queue_high;
  } else {
    const double level = std::fabs(queue - dref) > dead_zone ? (queue - dref) : 0.0;
    dR = -kg * delta - kp * p_.Tc * level;
    reason = std::fabs(delta) > 0.001 ? Reason::queue_growing
           : (level != 0.0 ? Reason::queue_high : Reason::steady);

    // ── the loss backoff (ch. 5) ─────────────────────────────────────────────
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
    // download in the pipe and real Sunshine, from 1.9 to 10.7 Mbit/s in one cycle, right before
    // going from the menu to the action.
    if (loss_smooth_ > p_.backoff_loss && queue > dref + p_.backoff_queue_margin) {
      dR = std::min(dR, -(R_ - p_.backoff_factor * C_est_));
      reason = Reason::loss_backoff;
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
    // at the target the backoff did not fire right in the drop —with real Sunshine, from 12 to
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
        p_.backoff_trains > 0 && recent_trains >= p_.backoff_trains && recent_span > 0
            ? recent_bits / recent_span : 0.0;
    if (recent_bandwidth > 0) recent_bandwidth = std::max(recent_bandwidth, 0.5 * m.received_rate);
    recent_pipe = recent_bandwidth;
    // One cycle alone may be the report that was delayed, not the pipe: it is required to repeat.
    const bool shortfall = p_.delivery_full > 0 && m.delivery > 0
                           ? m.delivery < 1.0 - p_.fast_backoff
                           : m.received_rate > 0 && m.received_rate < (1.0 - p_.fast_backoff) * sent_then_;
    shortfall_streak_ = shortfall ? shortfall_streak_ + 1 : 0;
    const bool shortfall_confirmed = shortfall_streak_ >= std::max(1, p_.fast_backoff_cycles);
    // And only with no queue yet. With another flow in the queue the packets of each frame go out
    // interleaved with its own, and the dispersion measures the part of the pipe that belongs to the video,
    // not the pipe: "crossed a narrow pipe" became true all the time, and each backoff
    // ceded more pipe to a download. A drop from steady state starts with the queue empty.
    const bool crossed_narrower = recent_bandwidth > 0 && queue < dref &&
                             recent_bandwidth < (1.0 - p_.fast_backoff) * R_;
    if (p_.fast_backoff > 0 && m.received_rate > 0 &&
        sent_then_ > 2.0 * p_.bitrate_min && queue < p_.fast_backoff_queue * dref &&
        m.t - t_fast_backoff_ > 2.0 && (shortfall_confirmed || crossed_narrower)) {
      double target = p_.backoff_factor * std::min(C_est_, m.received_rate);
      if (recent_bandwidth > 0) target = std::min(target, p_.backoff_factor * recent_bandwidth);
      if (R_ > target) {
        dR = std::min(dR, -(R_ - target));
        reason = Reason::loss_backoff;
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
    if (p_.congestion_loss > 0 && loss_smooth_ > p_.congestion_loss && queue <= dref) {
      if (t_loss_no_queue_ < 0) t_loss_no_queue_ = m.t;
      if (m.t - t_loss_no_queue_ >= p_.congestion_loss_s && m.received_rate > 0) {
        const double target = p_.backoff_factor * std::min(C_est_, m.received_rate);
        if (R_ > target) {
          dR = std::min(dR, -(R_ - target));
          reason = Reason::loss_backoff;
        }
        t_loss_no_queue_ = -1;
        blind_until_ = m.t + p_.blind_s;
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
    if (p_.backoff_step > 0 && raw_queue - raw_queue_prev_ > p_.backoff_step &&
        raw_queue > 2.0 * dref && m.received_rate > 0 && m.t - t_fast_backoff_ > 2.0) {
      // The new pipe: what arrives is still the old one, but the queue already grows at (R−C)/R. With the
      // smaller of the last two growths: a path that lengthens raises the queue by one step
      // and leaves it flat, and read as a slope it cut the budget to almost nothing.
      const double fit_slope = std::min(0.9, std::max(0.0,
          std::min(raw_queue - raw_queue_prev_, raw_queue_prev_ - raw_queue_prev2_) / p_.Tc));
      const double new_pipe = std::min({C_est_, app_limited_ ? C_est_ : m.received_rate, R_ * (1.0 - fit_slope)});
      const double target = p_.backoff_step_factor * new_pipe;
      if (R_ > target) {
        dR = std::min(dR, -(R_ - target));
        reason = Reason::loss_backoff;
        t_fast_backoff_ = m.t;
        // This one does not ask for a keyframe: it does not tell apart a download building its queue from a pipe that
        // narrows, and a keyframe inside a download's queue is lost, and the client
        // asks for one per frame. The alarm does tell them apart.
      }
    }
  }

  // ── 5b. without evidence it does not rise ───────────────────────────────
  // With the encoder delivering a fraction of the budget, the queue is empty because
  // WE ARE NOT SENDING, not because there is room. The proportional term cannot tell: it sees the
  // queue far below the target and orders a rise, and with chapter 4's gains that
  // is about 73 kbit/s per cycle — eleven megabits in fifteen seconds of menu, against a pipe
  // that did not change.
  //
  // Blocking the probing was not enough: probing is the explicit rise mechanism, but the
  // law itself rises just the same, and it is the law that does almost all the damage.
  //
  // It is cut in one direction on purpose. Rising without evidence is inventing pipe;
  // dropping without evidence is prudent, and it is also what has to be done if while the
  // encoder is silent the network got worse.
  if (app_limited_ && dR > 0) {
    dR = 0;
    if (reason != Reason::loss_backoff) reason = Reason::no_evidence;
  }

  steady_ = std::fabs(delta) < 0.002 ? steady_ + 1 : 0;

  const double before = R_;
  R_ = R_ + dR;
  // Dropping empties the queue; dropping below what the pipe delivers only empties it faster, and
  // past a point all it adds is a worse picture. When the pipe narrows, the frames
  // already encoded for the old one keep going out for a couple of cycles, the queue
  // keeps growing, and the growth term kept cutting: with real Sunshine, from 12
  // to 4 Mbit/s, the budget ended at the minimum with the pipe delivering 4.25. And with the
  // budget at the minimum, those same frames took even longer to go out.
  //
  // The floor is a fraction of what arrives —of the pipe of the last frames, if smaller, because
  // right after a drop what arrives still drags the old pipe—, and does not apply in the step
  // of a backoff, which already chose its target with the same numbers.
  if (p_.delivered_floor > 0 && dR < 0 && reason != Reason::loss_backoff && !app_limited_ &&
      m.received_rate > 0) {
    const double delivery = recent_pipe > 0 ? std::min(m.received_rate, recent_pipe) : m.received_rate;
    R_ = std::max(R_, std::min(before, p_.delivered_floor * delivery));
  }
  if (R_ < p_.bitrate_min) { R_ = p_.bitrate_min; d.capped = true; }
  if (R_ > p_.bitrate_max) { R_ = p_.bitrate_max; d.capped = true; }
  if (queue <= dref) R_healthy_ = R_;
  if (reason == Reason::loss_backoff || loss_smooth_ > p_.parity_burst) startup_done_ = true;

  // ── 6. the defenses ──────────────────────────────────────────────────────
  if (check_coherence(queue, dref, delta, R_ - before)) {
    d.reanchored = true;
    reason = Reason::reanchored;
  }

  // ── 7. dropping is correcting, rising is feeling the way ─────────────────
  // Hysteresis: to declare the queue empty it has to drop well below what it took
  // to declare it full.
  //
  // And a small queue with the pipe not full is not the pipe: it is the bursts of each frame. If
  // what goes out keeps arriving at the same pace, that queue does not mark where the pipe is and the rise goes on.
  // Counted as a queue, every time it crossed the target the curve started over from the old
  // anchor, and from a 4 to a 12 Mbit/s pipe it took fifteen seconds to get there.
  const bool burst_queue = p_.queue_requires_full && !pipe_full_ && queue < 2.0 * dref &&
                               loss_smooth_ < p_.parity_burst;
  const bool has_queue = !burst_queue && (had_queue_ ? (queue > dref * 0.3) : (queue > dref));
  if (has_queue) {
    if (!had_queue_) {
      // the anchor is the rate at the EDGE where the queue appeared. Touching it every cycle
      // collapses it; not letting it drop takes the queue peak to 62 ms.
      if (R_ > anchor_) anchor_ = R_;
      else if (R_ < anchor_ * p_.anchor_collapse) anchor_ = p_.anchor_blend * (anchor_ + R_);
    }
    in_ramp_ = false;
    t_last_queue_ = m.t;
    startup_done_ = true;
  } else if (!naive_ && !app_limited_ && reason != Reason::loss_backoff) {
    // Never in the same step as a backoff: the curve returned the budget to where it was before
    // the backoff, and the backoff did not lower anything.
    //
    // And if the train measures more pipe than the anchor, the anchor is the train. Without this, past the curve the
    // rise is exploration —a fixed percentage of the old anchor per second—, and with real Sunshine,
    // from a 4 to a 12 Mbit/s pipe, twenty-five seconds went by at 7. The train
    // knows how much pipe there is without having filled it; with the queue empty, which is the only way to
    // get here, there is no other flow to refute it.
    //
    // But only if the last queue was several seconds back. If the queue comes and goes, the anchor
    // that its edge left is the one that counts: behind a router that lets bursts through the train comes out
    // inflated every time the queue empties, and pointing the curve there made it crash again and again.
    if (p_.train_anchor > 0 && p_.variant == Variant::train && train_bandwidth_ > 0 &&
        m.t - t_last_queue_ > p_.train_anchor_wait) {
      anchor_ = std::max(anchor_, std::min(p_.train_utilization * train_bandwidth_, p_.train_anchor * R_));
    }
    // With the queue empty it feels its way upward. But if the queue is empty because WE ARE
    // SENDING NOTHING, rising is inventing: there is no evidence at all that there is room.
    ramp_up_step(dref, queue);
    if (reason == Reason::steady) reason = Reason::ramping;
  } else if (app_limited_) {
    in_ramp_ = false;           // when the scene returns, the curve starts from zero
    if (reason == Reason::steady) reason = Reason::no_evidence;
  }

  // ── the cross queue ──────────────────────────────────────────────────────
  // A movie in steady state downloads a chunk every few seconds at the pipe's full speed and
  // fills the router's queue while it lasts. For the loop it is the same as a drop: it cuts, and what
  // arrives meanwhile —the little it sends— enters Ĉ as if it were the pipe. When the movie
  // finishes the chunk, the queue disappears at once, but Ĉ and the anchor stayed at the minimum, and in
  // the two seconds of calm the rise gets nowhere: with real Sunshine, the video
  // stayed at 1 Mbit/s with 5 free.
  //
  // What tells another's queue apart is what it does after the cut. One's own, from a
  // drop, empties: more arrives than is sent. Another's stays high and still while
  // the same arrives as is sent —the video passes whole, behind the cross queue—. If that lasts and the
  // episode ends by itself and short, the pipe did not change and it goes back to what it was. A download
  // with no pause does not end, and there the law keeps sending.
  //
  // Ending is truly emptying, against the delay minimum and not against the chapter 5 floor:
  // with a continuous download that floor is a percentile of the window, and the queue measured against it
  // empties at times with two hundred milliseconds of queue. Read as the end of a chunk of movie,
  // the budget explored up to the maximum inside the queue of a game's download.
  const double true_queue = std::max(0.0, m.owd_mean - delay_floor_min);
  if (queue <= dref) C_healthy_ = C_est_;
  if (p_.cross_s > 0 && !naive_) {
    if (has_queue && !had_queue_) episode_ = {true, m.t, R_healthy_, C_healthy_, 0.0};
    if (has_queue) after_cross_ = false;
    if (has_queue && episode_.active && !app_limited_ && m.received_rate > 0 && sent_then_ > 0) {
      const double r = m.received_rate / sent_then_;
      const bool same = std::fabs(r - 1.0) < p_.cross_tolerance;
      const bool still = true_queue > 2.0 * dref && raw_queue >= raw_queue_prev_ - 0.010;
      // One's own queue in equilibrium also looks still and with everything arriving; the difference is that
      // one's own empties if lowered, and this one already survived dropping to half, or to the floor
      const bool lowering = R_ <= std::max(0.5 * episode_.R, 1.5 * p_.bitrate_min);
      if (same && still && lowering) episode_.cross += p_.Tc;
    }
    if (!has_queue && had_queue_ && episode_.active) {
      const bool cross = episode_.cross >= p_.cross_s && m.t - episode_.t0 <= p_.cross_max_s &&
                         true_queue < dref;
      if (cross && episode_.R > R_) {
        R_ = std::min(p_.bitrate_max, episode_.R);
        C_est_ = std::max(C_est_, episode_.C);
        rates_.clear();
        // And the train starts over: the frames that crossed behind another's queue went out
        // interleaved with its packets, and their dispersion measures the part that fell to the video, not the
        // pipe. Nor is the earlier measurement any use: in a calm of a couple of seconds no new
        // samples come in, and with real Sunshine the first one stayed as the ceiling forever.
        train_bandwidth_ = 0;
        trains_.clear();
        anchor_ = std::max(anchor_, R_);
        in_ramp_ = false;
        d.reanchored = true;
      }
      after_cross_ = cross;
      episode_.active = false;
    }
  }
  had_queue_ = has_queue;

  // ── 7a. exploration at cruise (HyStart++, RFC 9406) ───────────────────────
  // Past the first queue, the curve rises like a cubic from the anchor, and from a 4 to a
  // 12 Mbit/s pipe that is ten seconds. If the path is cruising —the raw queue stuck to the floor,
  // what goes out arrives whole and at the same pace, with no loss— for a few seconds, there is no evidence
  // of being near the pipe, and it rises by multiplying, like the startup. It exits as soon as the raw
  // queue lifts off, just as HyStart++ exits when the round's minimum rises, and it does not go back to the
  // budget it reached —that is already past— but to the maximum that was really delivered, like BBR.
  if (p_.cruise_gain > 0 && !naive_) {
    delivered_.push_back({m.t, m.received_rate});
    prune(delivered_, m.t, 1.0);
    const bool clean = raw_queue < p_.cruise_queue && !app_limited_ && loss_smooth_ < p_.parity_burst &&
                        (m.delivery <= 0 || delivery_smooth_ >= 0.97) && reason != Reason::loss_backoff;
    if (!clean) t_cruise_ = m.t;
    if (exploring_) {
      if (raw_queue >= p_.cruise_queue || reason == Reason::loss_backoff || loss_smooth_ > p_.parity_burst) {
        double maximum = 0;
        for (const auto& x : delivered_) maximum = std::max(maximum, x.v);
        if (maximum > 0 && reason != Reason::loss_backoff) {
          R_ = std::max(p_.bitrate_min, std::min(R_, 0.95 * maximum));
          anchor_ = std::max(anchor_, maximum);
        }
        exploring_ = false;
        in_ramp_ = false;
        t_cruise_ = m.t;
      } else {
        R_ = std::min(p_.bitrate_max, R_ * (1.0 + p_.cruise_gain));
      }
    } else if (startup_done_ && m.t - t_cruise_ >= p_.cruise_wait) {
      exploring_ = true;
    }
  }
  queue_prev_ = queue;
  raw_queue_prev2_ = raw_queue_prev_;
  raw_queue_prev_ = raw_queue;

  // ── 7b. the dispersion variant (SQP) ───────────────────────────────────
  // Here the budget does not come from integrating corrections over the delay: it comes from what
  // was measured the pipe delivers, trimmed by the utilization multiplier. That trim
  // replaces the floor probe — the 10 % left over drains the queues by itself, without halting the
  // send every eight seconds.
  if (p_.variant == Variant::train && train_bandwidth_ > 0) {
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
      const bool not_enough = pipe_full_ || queue > dref || loss_smooth_ > p_.backoff_loss * 0.5;
      R_ = (not_enough || app_limited_) ? std::max(p_.bitrate_min, ceiling)
                                        : std::min(R_, std::max(ceiling, before));
      if (reason == Reason::steady || reason == Reason::ramping) reason = Reason::capped;
      anti_windup(R_);
    }
  }

  // ── 7b'. the queue that emptied by itself ─────────────────────────────
  // An air outage —a distant Wi-Fi, the access point serving another— builds in a quarter of a
  // second the same queue ramp as a pipe that narrows, and the alarm, the step and the law
  // cut it just the same: each outage lowered the budget to 1 Mbit/s and the timid curve took a while to
  // give it back. While it is happening there is no way to tell them apart; when the queue empties, there is: what
  // emptied it is the pipe that remained. If it delivers what was being sent before the queue, it goes back there, up to the
  // part of that pipe that lets the next outage's queue empty in time, and never lower. In the
  // bench, with 8 Mbit/s and an outage every two seconds, it was not separated from run-to-run noise (utilization of
  // 55-59 % with and without this): it stays off until tried on a real path.
  if (outage && !naive_ && m.t - m.drain_start <= p_.drain_episode) {
    double budget_before_queue = 0;
    for (const auto& x : budgets_) {
      if (x.t > m.drain_start) break;
      if (x.t >= m.drain_start - 0.3) budget_before_queue = std::max(budget_before_queue, x.v);
    }
    const double target = std::min({budget_before_queue, p_.bitrate_max, usable_drain_rate});
#ifdef ABR_BENCH_DEBUG
    if (std::getenv("ABR_DEBUG_ALARM")) std::fprintf(stderr, "drain t=%.2f from=%.2f pipe=%.0f before=%.0f R=%.0f\n", m.t, m.drain_start, m.drain_rate, budget_before_queue, R_);
#endif
    // Only if the pipe delivers at least what was being sent: if it delivers less, it narrowed, and the queue
    // the drop left is still going away behind the cut. Going back there filled it again: with
    // real Sunshine, from 12 to 4 Mbit/s, the router's drops went from 18 to 73.
    if (m.drain_rate >= budget_before_queue && target > R_) {
      if (alarm_open_ || t_alarm_ >= m.drain_start) {
        min_parity_ = before_alarm_.min_parity;
        t_loss_ = before_alarm_.t_loss;
        alarm_open_ = false;
      }
      R_ = target;
      anchor_ = std::max(anchor_, target);
      C_est_ = std::max(C_est_, target);
      in_ramp_ = false;
      reason = Reason::reanchored;
      d.reanchored = true;
    }
  }

  // ── 7c. the ceiling that does not go through the delay floor ───────────────
  // The whole law of chapters 3 and 4 rests on a single measurement: how far the
  // delay came away from its floor. When that floor is poisoned —and against a flow that never lets
  // the queue drain it is always poisoned, at exactly ten seconds— the law does not degrade:
  // it is left without input, and an integrator without input goes wherever it is pushed.
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
  const bool has_evidence = pipe_full_ || loss_smooth_ > p_.backoff_loss * 0.5;
  if (has_evidence && R_ > proven_ceiling) {
    R_ = std::max(p_.bitrate_min, proven_ceiling);
    // Without this the ramp-up curve went on toward its anchor under the ceiling, and the budget
    // jumped every cycle between the ceiling and the curve: with real Sunshine, from 12.9 to 8.2 Mbit/s
    // and back, ten times per second. The same anti-windup as the train ceiling.
    if (p_.ceiling_anti_windup) anti_windup(R_);
    d.capped = true;
    if (reason == Reason::ramping || reason == Reason::steady) reason = Reason::capped;
  }

  // ── 8. the repair (ch. 7) ───────────────────────────────────────────────
  // f ≈ p + z·√(p/D): the parity needed for a block of D fragments to
  // withstand the measured loss. Rise fast, fall slow.
  const double D = std::max(8.0, p_.expected_blocks);
  const double fec_target = std::min(p_.fec_max,
      std::max(p_.fec_min, loss_smooth_ + p_.fec_z * std::sqrt(std::max(0.0, loss_smooth_) / D)));
  const double alpha = fec_target > f_ ? p_.fec_rise : (p_.Tc / p_.fec_fall_tau);
  f_ += alpha * (fec_target - f_);

  // ── 9. the single budget (ch. 7) ────────────────────────────────────────
  // What goes out on the wire is R, whole. The picture is what is left after paying for the
  // audio, the headers and the parity. That is why moving f does not add a single bit to the pipe.
  double egress = R_ * p_.utilization;
  if (probing_) egress *= (1.0 - p_.probe_cut);
  if (probing_) reason = Reason::probing;
  const double overhead = static_cast<double>(p_.mtu) / (p_.mtu + p_.header_bytes);
  double video_rate = std::max(0.3e6, (egress - p_.audio_bps) * overhead / (1.0 + f_));
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
    const double usable = (egress - p_.audio_bps) * overhead;
    const double per_packet = p_.mtu * 8.0 * fps_;
    video_rate = 0.3e6;
    for (int k = std::max(1, static_cast<int>(std::ceil(usable / per_packet))); k >= 1; --k) {
      const int m_parity = block_parity(k, loss_peak_, std::max(1.0, loss_burst_), p_, min_parity_);
      const double fits = std::min(k * per_packet, usable - m_parity * per_packet);
      if (fits > (k - 1) * per_packet) {
        video_rate = std::max(0.3e6, fits);
        break;
      }
    }
  }

  d.budget = egress;
  d.video_rate = video_rate;

  // ── 10. the encoder, compensated ────────────────────────────────────────
  // A fixed-rate encoder with a one-frame buffer —libx264 as Sunshine configures it—
  // delivers a quarter less than it is asked even if the scene demands everything, and the pipe is left
  // half used with the loop unable to do anything: the budget is left over. It is asked for more in
  // the proportion it has been falling short, so that what it delivers is the budgeted picture.
  //
  // It is learned only with the encoder using the budget —in a menu it delivers little because it
  // has nothing to encode, and compensating that would be asking for too much right before the blow—, with no queue and no
  // loss. And with a ceiling: if on a scene change the encoder does reach what was asked, what
  // comes out exceeds the budget by at most that ceiling, and the law corrects it like any excess.
  //
  // And it is released at once on the first sign of congestion: queue above the target, the
  // pipe full or a backoff. Measured with real Sunshine and Moonlight, asking for more when the pipe
  // narrows multiplied a drop's broken frames by ten: the encoder's excess falls
  // in the worst moment. Released, it learns again slowly. Loss alone is not a signal: on
  // Wi-Fi there is always some, and releasing with it leaves the encoder delivering a sixth less. The
  // queue barely above the target is not one either, if it is a single step: on Wi-Fi the air bursts
  // move it around the target all the time and the compensation never gets learned. Above
  // double, or sustained, yes; a drop goes past double in the first report.
  queue_over_target_ = queue > dref ? queue_over_target_ + 1 : 0;
  const bool queue_congestion = p_.encoder_release_cycles <= 1
                                   ? queue > dref
                                   : queue > 2.0 * dref || queue_over_target_ >= p_.encoder_release_cycles;
  const bool congestion = queue_congestion || pipe_full_ || reason == Reason::loss_backoff ||
                          (p_.encoder_release_loss > 0 && loss_smooth_ > p_.encoder_release_loss);
  if (congestion) {
    encoder_efficiency_ = 1.0;
  } else if (p_.encoder_compensation_max > 1.0 && m.data_rate > 0 && request_prev_ > 0 &&
             m.sent_rate >= p_.app_limited_threshold * R_) {
    const double sample = std::min(1.0, m.data_rate / request_prev_);
    encoder_efficiency_ += p_.Tc / p_.encoder_time_const * (sample - encoder_efficiency_);
  }
  const double efficiency = std::max(1.0 / std::max(1.0, p_.encoder_compensation_max),
                                     std::min(1.0, encoder_efficiency_));
  d.encoder_request = video_rate / efficiency;
  request_prev_ = d.encoder_request;
  d.repair = f_;
  d.reason = reason;
  d.queue = queue;
  d.delay_floor = delay_floor_cur_;
  d.dref = dref;
  d.growth = delta;
  d.dead_zone = dead_zone;
  d.C_est = C_est_;
  d.tau_est = tau;
  d.kg = kg;
  d.kp = kp;
  d.loss_est = loss_peak_;
  d.min_parity = min_parity_;
  // The length of the channel's bursts. With the queue full the loss is the queue's own,
  // which drops everything that arrives while the overflow lasts: that says nothing about the channel, and if
  // it entered the average the parity would keep protecting against bursts that do not exist.
  const bool overflow = queue > dref + p_.backoff_queue_margin;
  if (m.loss_burst > 0 && !overflow) {
    loss_burst_ = loss_burst_ > 0 ? loss_burst_ + 0.2 * (m.loss_burst - loss_burst_) : m.loss_burst;
  }
  d.loss_burst = std::max(1.0, loss_burst_);
  d.samples = m.samples;
  d.probing = probing_;
  d.app_limited = app_limited_;
  d.train_bandwidth = train_bandwidth_;
  return d;
}

}  // namespace abr
