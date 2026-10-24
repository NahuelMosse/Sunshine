/**
 * @file tests/unit/test_abr_control.cpp
 * @brief Test src/abr/control.*
 */
#include "../tests_common.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <src/abr/control.h>

namespace {
  /**
   * @brief A bottleneck with a drop-tail buffer, stepped once per controller cycle.
   *
   * It plays the network for the controller: whatever budget it decided is put on the link, and
   * the measurement the client would report comes back.
   */
  struct link_t {
    double capacity = 20e6;  ///< bits per second
    double base_owd = 0.020;  ///< one-way delay with an empty queue, seconds
    double buffer_s = 0.250;  ///< buffer depth at the current capacity, in seconds
    double fill = 1.0;  ///< fraction of the budget the encoder actually produces
    double cross_queue = 0.0;  ///< seconds of queue somebody else's flow keeps in the buffer
    double noise = 0.0;
    bool trains = false;  ///< report the per-frame dispersion samples
    bool delivery = true;  ///< report the delivery ratio of the frames
    double extra_loss = 0.0;  ///< random loss that has nothing to do with the queue
    double train_capacity = 0.0;  ///< what the frames' dispersion shows, if it is not the link's capacity

    double queue_bits = 0;
    double t = 0;
    int frame = 0;

    abr::Measurement step(const abr::Params &p, double send_bps) {
      const double dt = p.Tc;
      const double sent = send_bps * fill;
      const double offered = sent * dt;
      const double before = queue_bits;
      queue_bits += offered - capacity * dt;
      queue_bits = std::max(0.0, queue_bits);
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
      const double q0 = std::min(before, queue_bits) / capacity + cross_queue;
      const double q1 = 0.5 * (before + queue_bits) / capacity + cross_queue;
      m.owd_min = base_owd + q0;
      m.owd_mean = base_owd + q1;
      m.loss = std::min(1.0, (offered > 0 ? lost / offered : 0.0) + extra_loss);
      m.rtt = 2 * base_owd + q1;
      m.received_rate = std::min(capacity, delivered / dt);
      m.delivery = delivery && offered > 0 ? std::min(1.0, delivered / offered) : 0.0;
      m.samples = 10;
      m.sent_rate = sent;
      m.data_rate = sent * 0.9;
      m.noise = noise;
      m.loss_burst = m.loss > 0 ? 2.0 : 0.0;
      if (trains) {
        // Frames of different sizes crossing the bottleneck: the span grows with the size
        for (int i = 0; i < 6; ++i) {
          const double bytes = (20 + 9 * ((frame++) % 7)) * (p.mtu + p.header_bytes);
          m.trains.push_back({bytes, bytes * 8.0 / (train_capacity > 0 ? train_capacity : capacity)});
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
    std::set<abr::Reason> reasons;
    double min_budget = 1e18;
    double max_budget = 0;
    std::function<void(abr::Measurement &)> mutate;  ///< bends what the link reported

    explicit run_t(const abr::Params &p, double negotiated = 20e6):
        control(p),
        params(p) {
      control.reset(negotiated);
      last.budget = control.budget();
    }

    void run(double seconds) {
      const int n = static_cast<int>(seconds / params.Tc);
      for (int i = 0; i < n; ++i) {
        auto m = link.step(params, last.budget);
        if (mutate) {
          mutate(m);
        }
        last = control.step(m);
        reasons.insert(last.reason);
        min_budget = std::min(min_budget, last.budget);
        max_budget = std::max(max_budget, last.budget);
      }
    }
  };

  abr::Params params_for(abr::Variant v) {
    abr::Params p;
    p.variant = v;
    p.bitrate_max = 20e6;
    return p;
  }
}  // namespace

TEST(AbrControlTest, DerivedGainsAreStable) {
  const auto s = abr::derive(abr::Params {});
  EXPECT_GT(s.kg_n, 0);
  EXPECT_GT(s.kp_n, 0);
  EXPECT_GT(s.w180_tau, s.wc_tau) << "the crossover must come before the half-turn lag";
  EXPECT_GT(s.gain_margin, 1.0);
  EXPECT_GT(s.period_tau, 0);
}

TEST(AbrControlTest, BlockParityGrowsWithLossAndRespectsLimits) {
  const abr::Params p;
  const int clean = abr::block_parity(40, 0.0, 1.0, p);
  const int lossy = abr::block_parity(40, 0.05, 1.0, p);
  const int bursty = abr::block_parity(40, 0.05, 6.0, p);
  EXPECT_GE(clean, p.fec_min_packets);
  EXPECT_GT(lossy, clean);
  EXPECT_GE(bursty, lossy);
  EXPECT_EQ(abr::block_parity(40, 0.0, 1.0, p, 4), 4) << "an explicit minimum replaces the default one";
  EXPECT_LE(abr::block_parity(200, 0.9, 1.0, p), p.gf_shards - 200) << "data plus parity must fit the field";
}

TEST(AbrControlTest, StartsLowThenClimbsOnAFreePath) {
  run_t r(params_for(abr::Variant::baseline));
  EXPECT_DOUBLE_EQ(r.control.budget(), 0.3 * 20e6);
  r.run(0.1);
  EXPECT_EQ(r.last.reason, abr::Reason::startup);
  r.run(20);
  EXPECT_GT(r.last.budget, 0.8 * 20e6);
  EXPECT_LE(r.max_budget, 20e6 * r.params.utilization);
}

TEST(AbrControlTest, CutsTheBudgetWhenThePipeNarrows) {
  run_t r(params_for(abr::Variant::train));
  r.link.trains = true;
  r.run(15);
  const double before = r.last.budget;
  EXPECT_GT(before, 10e6);
  r.link.capacity = 5e6;
  r.run(10);
  EXPECT_LT(r.last.budget, 6e6) << "the budget must follow the pipe down";
  EXPECT_TRUE(r.reasons.count(abr::Reason::loss_backoff) || r.reasons.count(abr::Reason::queue_growing));
  EXPECT_GE(r.min_budget, r.params.bitrate_min);
  // ...and back up when the pipe returns
  r.link.capacity = 20e6;
  r.run(40);
  EXPECT_GT(r.last.budget, 2 * 5e6);
}

TEST(AbrControlTest, EveryVariantStaysInsideTheBounds) {
  for (auto v : {abr::Variant::baseline, abr::Variant::patched, abr::Variant::padding, abr::Variant::train}) {
    run_t r(params_for(v));
    r.link.trains = true;
    r.run(8);
    r.link.capacity = 4e6;
    r.run(8);
    r.link.capacity = 15e6;
    r.run(8);
    EXPECT_GE(r.min_budget, r.params.bitrate_min * r.params.utilization * 0.84) << abr::name(v);
    EXPECT_LE(r.max_budget, r.params.bitrate_max * r.params.utilization) << abr::name(v);
  }
}

TEST(AbrControlTest, AnEncoderThatDoesNotFillTheBudgetIsNoEvidence) {
  run_t r(params_for(abr::Variant::train));
  r.run(6);
  r.link.fill = 0.3;
  r.run(6);
  EXPECT_TRUE(r.reasons.count(abr::Reason::no_evidence));
  EXPECT_TRUE(r.last.app_limited);
  // The budget is not raised on the strength of a queue that is empty because nothing is sent
  const double stalled = r.last.budget;
  r.run(5);
  EXPECT_LE(r.last.budget, stalled * 1.01);
}

TEST(AbrControlTest, ReportsAQueueThatKeepsGrowing) {
  run_t r(params_for(abr::Variant::baseline));
  r.run(8);
  r.link.capacity = 8e6;
  r.link.buffer_s = 1.0;
  double deepest = 0;
  r.mutate = [&](abr::Measurement &) {
    deepest = std::max(deepest, r.last.queue);
  };
  r.run(4);
  EXPECT_TRUE(r.reasons.count(abr::Reason::queue_growing));
  EXPECT_GT(deepest, 0.0);
  EXPECT_GT(r.last.kg, 0.0);
  EXPECT_GT(r.last.kp, 0.0);
}

TEST(AbrControlTest, ASteadyQueueAboveTheTargetIsCorrectedByLevel) {
  abr::Params p = params_for(abr::Variant::baseline);
  run_t r(p);
  r.run(10);
  r.link.cross_queue = 0.060;
  bool level = false;
  for (int i = 0; i < 80; ++i) {
    auto m = r.link.step(p, r.last.budget);
    // A queue that stays put: the part of it that is ours does not move
    m.owd_mean = m.owd_min = r.link.base_owd + 0.060 + 0.001 * (i % 2);
    r.last = r.control.step(m);
    level = level || r.last.reason == abr::Reason::queue_high;
  }
  EXPECT_TRUE(level);
}

TEST(AbrControlTest, LossWithAFullQueueBacksOffToTheCapacity) {
  run_t r(params_for(abr::Variant::baseline));
  r.run(10);
  r.link.capacity = 6e6;
  r.link.buffer_s = 0.050;
  r.run(6);
  EXPECT_TRUE(r.reasons.count(abr::Reason::loss_backoff));
  EXPECT_GT(r.last.loss_est, 0.0);
  EXPECT_GT(r.last.repair, r.params.fec_min);
}

TEST(AbrControlTest, SustainedLossWithoutAQueueIsCongestion) {
  run_t r(params_for(abr::Variant::baseline));
  r.link.delivery = false;
  r.run(10);
  // fq_codel-like path: the delay never shows a queue, but the video is dropped
  const double before = r.last.budget;
  r.link.extra_loss = 0.15;
  r.min_budget = 1e18;
  r.run(6);
  EXPECT_LT(r.min_budget, before);
  EXPECT_TRUE(r.reasons.count(abr::Reason::loss_backoff));
}

TEST(AbrControlTest, AlarmCutsBeforeTheStepAndCanBeRetracted) {
  run_t r(abr::Params {});
  r.run(12);
  const double before = r.control.budget();
  EXPECT_FALSE(r.control.alarm(r.link.t, 0.0, 0.0, 0.0)) << "no received rate, no alarm";
  EXPECT_FALSE(r.control.alarm(r.link.t, 0.5, 0.001, 5e6)) << "growth below the step";
  EXPECT_FALSE(r.control.alarm(r.link.t, 0.001, 0.05, 5e6)) << "queue below the minimum";
  EXPECT_TRUE(r.control.alarm(r.link.t, 0.2, 0.05, 5e6));
  EXPECT_LT(r.control.budget(), before);
  EXPECT_FALSE(r.control.alarm(r.link.t + 0.5, 0.2, 0.05, 5e6)) << "one alarm every two seconds";
  EXPECT_FALSE(r.control.retract_alarm(r.link.t + 0.1, 0.5)) << "the queue is still there";
  EXPECT_TRUE(r.control.retract_alarm(r.link.t + 0.1, 0.0));
  EXPECT_NEAR(r.control.budget(), before, 1.0);
  EXPECT_FALSE(r.control.retract_alarm(r.link.t + 0.1, 0.0)) << "nothing left to retract";

  EXPECT_TRUE(r.control.alarm(r.link.t + 5, 0.2, 0.05, 5e6));
  EXPECT_FALSE(r.control.retract_alarm(r.link.t + 6, 0.0)) << "too late to be the same event";
  EXPECT_FALSE(r.control.retract_alarm(r.link.t + 6, 0.0));
}

TEST(AbrControlTest, AccessorsAndOverrides) {
  abr::Params p;
  p.keyframe_on_drop = 1;
  abr::Control c(p);
  EXPECT_TRUE(c.alarm_requests_keyframe());
  EXPECT_EQ(c.min_parity(), p.fec_min_packets);
  c.set_fps(0);
  c.set_fps(30);
  c.scale_gains(0.5);
  c.force_capacity(7e6);
  EXPECT_DOUBLE_EQ(c.params().Tc, p.Tc);
  EXPECT_GT(c.tuning().kg_n, 0);
  const auto d = c.step({});
  EXPECT_EQ(d.reason, abr::Reason::startup);
  EXPECT_DOUBLE_EQ(d.C_est, 7e6);
}

TEST(AbrControlTest, NaiveModeLooksAtTheLevelOnly) {
  run_t r(params_for(abr::Variant::baseline));
  r.control.naive_mode(true);
  r.run(10);
  r.link.capacity = 5e6;
  r.run(10);
  EXPECT_TRUE(r.reasons.count(abr::Reason::queue_high));
  EXPECT_FALSE(r.control.alarm(r.link.t, 0.2, 0.05, 5e6)) << "the naive controller has no alarm";
  EXPECT_FALSE(r.control.retract_alarm(r.link.t, 0.0));
}

namespace {
  // The queue the controller aims at, for a path whose round trip is `rtt`
  double queue_target(const abr::Params &p, double rtt) {
    return std::max(p.dref_floor, p.dref_fraction * (rtt + 1.0 / p.fps));
  }
}  // namespace

TEST(AbrControlTest, ARightSizedQueueIsSteadyAndTheCurveKeepsClimbing) {
  abr::Params p = params_for(abr::Variant::baseline);
  run_t r(p);
  const double on_target = queue_target(p, 0.060);
  int i = 0;
  r.mutate = [&](abr::Measurement &m) {
    m.rtt = 0.060;
    m.owd_min = 0.020;
    m.owd_mean = 0.020 + on_target - 0.001 - 0.004 * (i++ % 2);
  };
  r.run(6);
  EXPECT_TRUE(r.reasons.count(abr::Reason::ramping));
}

TEST(AbrControlTest, ASilentEncoderOnTargetIsNoEvidence) {
  abr::Params p = params_for(abr::Variant::train);
  run_t r(p);
  const double on_target = queue_target(p, 0.060);
  int i = 0;
  r.link.fill = 0.3;
  r.mutate = [&](abr::Measurement &m) {
    m.rtt = 0.060;
    m.owd_min = 0.020;
    m.owd_mean = 0.020 + on_target - 0.001 - 0.004 * (i++ % 2);
  };
  r.run(6);
  EXPECT_TRUE(r.reasons.count(abr::Reason::no_evidence));
}

TEST(AbrControlTest, ProbesTheDelayFloorWhenItHasNotBeenTouchedForAWhile) {
  abr::Params p = params_for(abr::Variant::baseline);
  run_t r(p);
  int i = 0;
  r.mutate = [&](abr::Measurement &m) {
    // A floor seen once, and then a delay that never gets back down to it
    const bool early = (i++) < 10;
    m.owd_min = early ? 0.020 : 0.030;
    m.owd_mean = m.owd_min + 0.0005 * (i % 2);
  };
  bool probed = false;
  for (int k = 0; k < 140; ++k) {
    r.run(0.1);
    probed = probed || r.last.probing;
  }
  EXPECT_TRUE(probed);
  EXPECT_TRUE(r.reasons.count(abr::Reason::probing));
  EXPECT_FALSE(r.last.probing) << "the cut lasts probe_duration and no longer";
}

TEST(AbrControlTest, ADelayThatDoesNotEaseWhenLoweredIsAStaleFloor) {
  abr::Params p = params_for(abr::Variant::baseline);
  run_t r(p);
  r.control.scale_gains(4.0);
  r.run(8);
  const double floor_before = r.last.delay_floor;
  bool reanchored = false;
  int i = 0;
  // The path got longer: the delay is 60 ms over what the floor remembers, and stays there
  r.mutate = [&](abr::Measurement &m) {
    m.owd_min = floor_before + 0.060;
    m.owd_mean = m.owd_min + 0.0003 * (i++ % 2);
  };
  for (int k = 0; k < 90; ++k) {
    r.run(0.1);
    reanchored = reanchored || r.last.reason == abr::Reason::reanchored;
  }
  EXPECT_TRUE(reanchored);
}

TEST(AbrControlTest, NoisyPathsLookAtTheLowerEnvelopeOfTheQueue) {
  abr::Params p = params_for(abr::Variant::baseline);
  p.capacity_sustained = 3;
  p.queue_requires_full = 1;
  run_t r(p);
  r.link.noise = 0.010;
  int i = 0;
  r.mutate = [&](abr::Measurement &m) {
    // Air outages: every other sample is a spike the envelope must not follow
    m.owd_mean += (i++ % 2) ? 0.040 : 0.0;
  };
  r.run(10);
  EXPECT_GT(r.last.budget, 0.5 * 20e6);
}

TEST(AbrControlTest, DispersionFromASingleFrameAndItsExpiry) {
  abr::Params p = params_for(abr::Variant::train);
  run_t r(p);
  int i = 0;
  r.mutate = [&](abr::Measurement &m) {
    m.train_bytes = (20.0 + 9 * (i++ % 7)) * (p.mtu + p.header_bytes);
    m.train_span = m.train_bytes * 8.0 / 20e6;
  };
  r.run(3);
  EXPECT_GT(r.last.train_bandwidth, 0.0);
  r.mutate = nullptr;
  r.run(5);
  EXPECT_EQ(r.last.train_bandwidth, 0.0) << "an old measurement is not a ceiling";
}

TEST(AbrControlTest, DispersionNeedsFramesOfDifferentSizes) {
  abr::Params p = params_for(abr::Variant::train);
  {
    // Every frame the same size: the slope of span against size is not determined
    run_t r(p);
    r.mutate = [&](abr::Measurement &m) {
      const double bytes = 40.0 * (p.mtu + p.header_bytes);
      m.trains = {{bytes, bytes * 8 / 20e6}, {bytes, bytes * 8 / 20e6}};
    };
    r.run(4);
    EXPECT_EQ(r.last.train_bandwidth, 0.0);
  }
  {
    // A span that shrinks as the frame grows is not a pipe
    run_t r(p);
    int i = 0;
    r.mutate = [&](abr::Measurement &m) {
      m.trains.clear();
      for (int k = 0; k < 4; ++k) {
        const double bytes = (20 + 10 * ((i++) % 5)) * (p.mtu + p.header_bytes);
        m.trains.push_back({bytes, 0.05 - bytes * 8 / 1e9});
      }
    };
    r.run(4);
    EXPECT_EQ(r.last.train_bandwidth, 0.0);
  }
  {
    // Beyond four times the declared ceiling it is noise, not a discovery
    run_t r(p);
    r.link.train_capacity = 1e12;
    r.link.trains = true;
    r.run(4);
    EXPECT_EQ(r.last.train_bandwidth, 0.0);
  }
}

TEST(AbrControlTest, TheNarrowPipeShowsInTheFramesBeforeItShowsInTheQueue) {
  abr::Params p = params_for(abr::Variant::train);
  run_t r(p);
  r.link.trains = true;
  r.run(12);
  const double before = r.last.budget;
  // Only the dispersion of the frames has seen the new pipe so far
  r.link.train_capacity = 4e6;
  r.min_budget = 1e18;
  r.run(1.5);
  EXPECT_TRUE(r.reasons.count(abr::Reason::loss_backoff));
  EXPECT_LT(r.min_budget, 0.8 * before);
}

TEST(AbrControlTest, ADispersionCeilingBelowTheBudgetCapsIt) {
  abr::Params p = params_for(abr::Variant::train);
  run_t r(p);
  r.link.trains = true;
  r.run(10);
  // The frames cross a narrower pipe than the one the budget is aimed at, with the queue on target
  const double on_target = queue_target(p, 0.060);
  int i = 0;
  r.link.train_capacity = 8e6;
  r.mutate = [&](abr::Measurement &m) {
    m.rtt = 0.060;
    m.owd_min = 0.020;
    m.owd_mean = 0.020 + on_target - 0.006 - 0.004 * (i++ % 2);
    m.received_rate = 8e6;
  };
  r.run(8);
  EXPECT_TRUE(r.reasons.count(abr::Reason::capped));
  EXPECT_LT(r.last.train_bandwidth, 9e6);
}

TEST(AbrControlTest, ProvenCeilingCapsABudgetThePipeDoesNotSustain) {
  abr::Params p = params_for(abr::Variant::baseline);
  run_t r(p);
  r.run(10);
  // The queue is on target, but a fifth of what is sent never arrives: the pipe is full and
  // what does arrive is its capacity
  const double on_target = queue_target(p, 0.060);
  int i = 0;
  r.mutate = [&](abr::Measurement &m) {
    m.rtt = 0.060;
    m.owd_min = 0.020;
    m.owd_mean = 0.020 + on_target - 0.006 - 0.004 * (i++ % 2);
    m.received_rate = 8e6;
    m.delivery = 0.8;
  };
  r.run(6);
  EXPECT_TRUE(r.reasons.count(abr::Reason::capped));
  EXPECT_TRUE(r.last.capped);
  EXPECT_LT(r.last.budget, 10e6);
  EXPECT_GT(r.last.budget, 8e6);
}

TEST(AbrControlTest, AQueueSomeoneElseFillsEndsAndTheBudgetComesBack) {
  abr::Params p = params_for(abr::Variant::baseline);
  run_t r(p);
  r.control.scale_gains(6.0);
  r.run(10);
  const double before = r.last.budget;
  // A download keeps 200 ms of queue for three seconds while everything we send still arrives
  r.mutate = [&](abr::Measurement &m) {
    m.owd_min += 0.2;
    m.owd_mean += 0.2;
    m.received_rate = m.sent_rate;
  };
  r.run(3);
  EXPECT_LT(r.last.budget, 0.5 * before) << "the law cuts while the queue lasts";
  r.mutate = nullptr;
  bool reanchored = false;
  for (int i = 0; i < 20 && !reanchored; ++i) {
    r.run(0.1);
    reanchored = r.last.reason == abr::Reason::reanchored || r.last.budget > 0.9 * before;
  }
  EXPECT_TRUE(reanchored) << "the pipe did not change: the budget goes back to what it was";
}

TEST(AbrControlTest, ABudgetAtTheMinimumCannotBeCutFurtherByTheAlarm) {
  abr::Params p;
  p.bitrate_max = 1e6;
  abr::Control c(p);
  EXPECT_FALSE(c.alarm(1.0, 0.2, 0.05, 5e6));
}

TEST(AbrControlTest, AnOutageThatEmptiesItselfGivesTheBudgetBack) {
  abr::Params p = params_for(abr::Variant::baseline);
  p.drain_episode = 2.0;
  run_t r(p);
  r.run(10);
  const double before = r.last.budget;
  const double started = r.link.t;
  // The alarm cuts at the start of what looks like a drop...
  ASSERT_TRUE(r.control.alarm(started + 0.05, 0.2, 0.05, 10e6));
  const double cut = r.control.budget();
  ASSERT_LT(cut, before);
  // ...and the queue empties by itself: what it measured on the way out is the pipe that remained
  r.mutate = [&](abr::Measurement &m) {
    m.noise = 0.01;
    m.drain_rate = 40e6;
    m.drain_start = started;
    m.drain_rise = 0.3;
    m.drain_height = 0.3;
  };
  r.run(0.1);
  EXPECT_EQ(r.last.reason, abr::Reason::reanchored);
  EXPECT_GT(r.last.budget, cut);
  EXPECT_FALSE(r.control.retract_alarm(r.link.t, 0.0)) << "the outage already settled the alarm";
}

TEST(AbrControlTest, DispersionFromFewFramesKeepsTheFirstFit) {
  abr::Params p = params_for(abr::Variant::train);
  p.train_window = 0.45;
  run_t r(p);
  int i = 0;
  r.mutate = [&](abr::Measurement &m) {
    m.trains.clear();
    m.train_bytes = (20.0 + 9 * (i % 7)) * (p.mtu + p.header_bytes);
    m.train_span = m.train_bytes * 8.0 / 20e6 + 0.002 * (i++ % 2);
  };
  r.run(3);
  EXPECT_GT(r.last.train_bandwidth, 0.0);
}

TEST(AbrControlTest, ExploresByMultiplyingWhileThePathCruises) {
  abr::Params p = params_for(abr::Variant::baseline);
  p.cruise_gain = 0.05;
  run_t r(p);
  r.link.capacity = 6e6;
  r.run(10);
  // The pipe grows: the budget explores it multiplicatively until the queue lifts off
  r.link.capacity = 20e6;
  r.run(30);
  EXPECT_GT(r.max_budget, 12e6);
  EXPECT_LE(r.max_budget, 20e6);
  // ...and a loss while exploring gives the exploration up
  r.link.capacity = 5e6;
  r.link.buffer_s = 0.03;
  r.run(8);
  EXPECT_TRUE(r.reasons.count(abr::Reason::loss_backoff));
}
