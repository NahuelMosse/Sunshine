/**
 * @file tests/unit/test_abr_one_cycle.cpp
 * @brief Test the two events that made the budget settle in more than one cycle after a change of the
 * pipe: an alarm retracted too late, and a train measure that expired and freed the ramp.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <algorithm>
#include <cmath>

// local includes
#include <src/abr/control.h>

namespace {
  /**
   * @brief A bottleneck with a drop-tail buffer, stepped once per controller cycle.
   */
  struct link_t {
    double capacity = 20e6;  ///< bits per second
    double base_owd = 0.020;  ///< one-way delay with an empty queue, seconds
    double buffer_s = 0.250;  ///< buffer depth at the current capacity, in seconds
    bool trains = false;  ///< report the per-frame dispersion samples

    double queue_bits = 0;
    double t = 0;
    int frame = 0;

    abr::Measurement step(const abr::Params &p, double send_bps) {
      const double dt = p.Tc;
      const double offered = send_bps * dt;
      const double before = queue_bits;
      queue_bits = std::max(0.0, queue_bits + offered - capacity * dt);
      const double limit = buffer_s * capacity;
      double lost = 0;
      if (queue_bits > limit) {
        lost = queue_bits - limit;
        queue_bits = limit;
      }
      const double delivered = std::max(0.0, offered - lost + before - queue_bits);
      t += dt;

      abr::Measurement m;
      m.t = t;
      const double q0 = std::min(before, queue_bits) / capacity;
      const double q1 = 0.5 * (before + queue_bits) / capacity;
      m.owd_min = base_owd + q0;
      m.owd_mean = base_owd + q1;
      m.loss = offered > 0 ? lost / offered : 0.0;
      m.rtt = 2 * base_owd + q1;
      m.received_rate = std::min(capacity, delivered / dt);
      m.delivery = offered > 0 ? std::min(1.0, delivered / offered) : 0.0;
      m.samples = 10;
      m.sent_rate = send_bps;
      m.data_rate = send_bps * 0.9;
      m.loss_burst = m.loss > 0 ? 2.0 : 0.0;
      if (trains) {
        // Frames of different sizes crossing the bottleneck: the span grows with the size
        for (int i = 0; i < 6; ++i) {
          const double bytes = (20 + 9 * ((frame++) % 7)) * (p.mtu + p.header_bytes);
          m.trains.push_back({bytes, bytes * 8.0 / capacity});
        }
      }
      return m;
    }
  };

  struct run_t {
    abr::Control control;
    abr::Params params;
    link_t link;
    abr::Decision last;

    explicit run_t(const abr::Params &p):
        control(p),
        params(p) {
      control.reset(p.bitrate_max);
      last.budget = control.budget();
    }

    void run(double seconds) {
      const int n = static_cast<int>(seconds / params.Tc);
      for (int i = 0; i < n; ++i) {
        last = control.step(link.step(params, last.budget));
      }
    }
  };
}  // namespace

TEST(AbrOneCycleTest, AnAlarmIsNotRetractedOnceAFrameSentAfterTheCutCouldShowTheQueue) {
  abr::Params p;
  run_t r(p);
  r.run(12);
  const double before = r.control.budget();
  ASSERT_TRUE(r.control.alarm(r.link.t, 0.2, 0.05, before));
  const double cut = r.control.budget();
  ASSERT_LT(cut, before);
  // A narrow pipe cut deep drains its queue within a round trip, a drain time and a report: an empty queue read
  // after that is the cut's own doing, not a delayed acknowledgement
  const double shows_queue_by = 2 * r.link.base_owd + 0.2 + abr::session::kReportPeriod;
  ASSERT_GT(abr::cal::kAlarmRetractS, shows_queue_by);
  EXPECT_FALSE(r.control.retract_alarm(r.link.t + 0.5 * (shows_queue_by + abr::cal::kAlarmRetractS), 0.0));
  EXPECT_DOUBLE_EQ(r.control.budget(), cut);
}

TEST(AbrOneCycleTest, AnExpiredTrainMeasureLeavesACeilingThatRisesLinearly) {
  abr::Params p;
  p.bitrate_max = 20e6;
  run_t r(p);
  r.link.capacity = 6e6;
  r.link.trains = true;
  r.run(20);
  const double held = r.last.budget;
  ASSERT_GT(held, 0.7 * 6e6);
  ASSERT_LT(held, 6e6);
  // The pipe opens and the frames stop spreading out: nothing measures it, and the measure expires
  r.link.capacity = 20e6;
  r.link.trains = false;
  r.run(abr::cal::kTrainExpiry + 0.5);
  constexpr double rise_per_second = 1.0;
  double lowest = 1e9;
  double highest = 0;
  const double from = r.link.t;
  while (r.last.budget < p.bitrate_max && r.link.t - from < 5.0) {
    r.run(p.Tc);
    const double ratio = r.last.budget / (held * (1.0 + rise_per_second * (r.link.t - from + 0.5)));
    lowest = std::min(lowest, ratio);
    highest = std::max(highest, ratio);
  }
  EXPECT_LE(highest, 1.05) << "the budget stays under the line that rises 1/s from where the measure stood";
  EXPECT_GE(lowest, 0.95) << "and follows it up";
}
