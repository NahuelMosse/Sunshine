/**
 * @file tests/unit/test_abr_robustness.cpp
 * @brief Test the defenses of the sources under src/abr against costly parity, hostile client reports and corner cases.
 */
#include "../tests_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <random>
#include <src/abr/control.h>
#include <src/abr/gamestream/signals.h>

using abr::Control;
using abr::Decision;
using abr::gamestream::FecStatus;
using abr::gamestream::LossSummary;
using abr::gamestream::ReceivedTrain;
using abr::gamestream::SentFrame;
using abr::gamestream::SignalMeter;
using abr::Measurement;
using abr::Params;
using abr::Reason;

namespace {
  double now_ms() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  /**
   * @brief The block parity as it was computed before, in O(m^3) with lgamma: the reference for the results.
   */
  int reference_block_parity(int k, double loss, double burst, const Params &p, int minimum) {
    if (minimum < 0) {
      minimum = p.fec_min_packets;
    }
    const int ceiling = std::min({p.gf_shards - k, (k * 255 + 99) / 100, std::max(minimum, static_cast<int>(std::ceil(k * p.fec_max)))});
    burst = std::max(1.0, burst);
    const double q = std::min(0.5, std::max(1e-6, loss) / burst);
    const double r = 1.0 - 1.0 / burst;
    for (int m = minimum; m < ceiling; ++m) {
      const int n = k + m;
      double lj = n * std::log1p(-q);
      double acc = std::exp(lj);
      for (int j = 1; j <= std::min(n, m); ++j) {
        lj += std::log(double(n - j + 1) / j) + std::log(q) - std::log1p(-q);
        double sum = 1.0;
        if (r > 0) {
          sum = 0;
          for (int t = j; t <= m; ++t) {
            sum += std::exp(std::lgamma(t) - std::lgamma(j) - std::lgamma(t - j + 1) + j * std::log1p(-r) + (t - j) * std::log(r));
          }
        }
        acc += std::exp(lj) * sum;
      }
      if (1.0 - acc < p.parity_block_failure) {
        return m;
      }
    }
    return std::max(0, ceiling);
  }

  Params repair_params() {
    Params r;
    r.parity_block_failure = r.parity_repair_failure;
    r.fec_max = std::max(r.fec_max, r.fec_max_repair);
    return r;
  }

  Measurement sample(double t, double rate, double owd, double loss) {
    Measurement m;
    m.t = t;
    m.owd_min = 0.030;
    m.owd_mean = owd;
    m.rtt = 0.060;
    m.samples = 40;
    m.sent_rate = rate;
    m.received_rate = rate * (1 - loss);
    m.loss = loss;
    return m;
  }

  SentFrame frame(uint32_t i, int packets = 10, int data = 9) {
    SentFrame c;
    c.index = i;
    c.t_first_us = 100000000 + static_cast<int64_t>(i) * 16667;
    c.t_last_us = c.t_first_us + 1000;
    c.packets = packets;
    c.data_packets = data;
    c.rtp_bytes = packets * 1000.0;
    c.wire_bytes = packets * 1100.0;
    return c;
  }

  /**
   * @brief Frames leaving the host and their arrival reports, with a client clock offset.
   */
  struct train_rig_t {
    SignalMeter s;
    uint32_t idx = 0;
    const uint32_t offset = 123456789u;

    train_rig_t() {
      s.rtt(0.040);
    }

    // One frame leaving at t_us and its report; `extra_us` is added to the client's last-packet time
    bool frame(int64_t t_us, uint32_t extra_us, double *ms) {
      SentFrame f;
      f.index = ++idx;
      f.packets = f.data_packets = 10;
      f.rtp_bytes = 10240;
      f.wire_bytes = 10800;
      f.t_first_us = t_us;
      f.t_last_us = t_us + 1000;
      s.sent(f);
      ReceivedTrain r;
      r.index = idx;
      r.packets = 10;
      r.bytes = 10240;
      r.t_first_us = static_cast<uint32_t>(t_us + 20000) + offset;
      r.t_last_us = static_cast<uint32_t>(t_us + 20000 + 2000) + offset + extra_us;
      double q = 0;
      double g = 0;
      return s.train(r, q, g, ms, t_us + 30000);
    }
  };

  // A clean path, then 60 ms of queue from 5 s; the client's ping carries its own clock.
  double queue_seen(uint32_t clock_base, bool bogus) {
    SignalMeter s;
    const int64_t t0 = 100000000;
    uint32_t idx = 0;
    double seen = 0;
    for (int64_t now = t0; now <= t0 + 8000000; now += 10000) {
      if ((now - t0) % 20000 == 0) {
        SentFrame f;
        f.index = ++idx;
        f.packets = f.data_packets = 10;
        f.rtp_bytes = f.wire_bytes = 10000;
        f.t_first_us = now;
        f.t_last_us = now + 500;
        s.sent(f);
      }
      if ((now - t0) % 100000 == 0) {
        const uint32_t cms = static_cast<uint32_t>(now / 1000) + clock_base;
        s.client_ping(now, (bogus && now == t0 + 3000000) ? 0xFFFFFFF0u : cms);
        s.rtt(now < t0 + 5000000 ? 0.040 : 0.100);
        Measurement m;
        s.measure(now, m);
        if (now == t0 + 6000000) {
          seen = m.owd_mean - 0.040;
        }
      }
    }
    return seen;
  }
}  // namespace

// @ABR-01-S1
TEST(AbrRobustnessTest, BlockParityMatchesTheReference) {
  const Params normal;
  const Params repair = repair_params();
  int compared = 0;
  int differ = 0;
  int off_by_more = 0;
  auto check = [&](const Params &p, int k, double loss, double burst, int minimum) {
    const int a = abr::block_parity(k, loss, burst, p, minimum);
    const int b = reference_block_parity(k, loss, burst, p, minimum);
    ++compared;
    if (a != b) {
      ++differ;
    }
    if (std::abs(a - b) > 1) {
      ++off_by_more;
    }
    EXPECT_LE(std::abs(a - b), 1) << "k=" << k << " loss=" << loss << " burst=" << burst << " min=" << minimum;
  };
  for (int k : {1, 2, 3, 5, 8, 13, 20, 32, 48, 64, 96, 128}) {
    for (double loss : {0.0, 0.002, 0.01, 0.03, 0.08, 0.2, 0.5, 1.0}) {
      for (double burst : {1.0, 1.3, 2.0, 4.0, 10.0, 1000.0}) {
        for (int minimum : {-1, 0, 5}) {
          check(normal, k, loss, burst, minimum);
        }
      }
    }
  }
  for (int k : {8, 32, 127}) {
    for (double loss : {0.01, 0.1, 0.3}) {
      for (double burst : {1.0, 1.5, 3.0}) {
        check(repair, k, loss, burst, 2);
      }
    }
  }
  EXPECT_EQ(off_by_more, 0);
  EXPECT_LE(differ, compared / 100) << differ << " of " << compared << " differ";
}

// @ABR-01-S1
TEST(AbrRobustnessTest, BlockParityCostIsBounded) {
  // The reference is O(m^3) with lgamma; the cost is compared against it on the same machine
  const Params repair = repair_params();
  const int reps = 20;
  int m = 0;
  int r = 0;
  double t0 = now_ms();
  for (int i = 0; i < reps; ++i) {
    m = abr::block_parity(127, 0.25, 1.5, repair, 2);
  }
  const double fast = (now_ms() - t0) / reps;
  t0 = now_ms();
  for (int i = 0; i < reps; ++i) {
    r = reference_block_parity(127, 0.25, 1.5, repair, 2);
  }
  const double slow = (now_ms() - t0) / reps;
  EXPECT_LE(std::abs(m - r), 1);
  EXPECT_GT(m, 20);
  EXPECT_LT(fast, slow / 10.0) << "fast " << fast << " ms, reference " << slow << " ms";
  EXPECT_LT(fast, 5.0);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, StepCostIsBoundedAtHighRateWithBurstLoss) {
  Params p;
  p.bitrate_max = 100e6;
  p.startup_fraction = 1.0;
  Control c(p);
  c.reset(p.bitrate_max);
  double worst = 0;
  double t = 100;
  for (int i = 0; i < 60; ++i) {
    t += p.Tc;
    Measurement m = sample(t, c.budget(), 0.03, 0.3);
    m.loss_burst = 3.0;
    m.received_rate = c.budget() * 0.7;
    const double t0 = now_ms();
    c.step(m);
    worst = std::max(worst, now_ms() - t0);
  }
  EXPECT_LT(worst, 20.0) << "a step must not take a frame's worth of time";
}

// @ABR-01-S1
TEST(AbrRobustnessTest, FrameParityIsTheSumOverTheHostsBlocks) {
  const Params p;
  // k = 300 cannot be one block: the host splits it, and each block pays its own parity
  const int total = abr::frame_parity(300, 0.02, 1.3, p, 2, 0.02);
  EXPECT_GT(total, 2);
  EXPECT_EQ(total, 2 * abr::block_parity(150, 0.02, 1.3, p, 2));
  // Up to one block it is the block's parity
  EXPECT_EQ(abr::frame_parity(100, 0.02, 1.3, p, 2, 0.02), abr::block_parity(100, 0.02, 1.3, p, 2));
}

// @ABR-01-S1
TEST(AbrRobustnessTest, FrameSizeAndParityNeverShrinkTogether) {
  // The step searches the largest frame by bisection: that needs k + parity(k) not to decrease
  // with k, within one packet at block boundaries
  const Params p;
  for (double loss : {0.0, 0.02, 0.1, 0.4}) {
    for (double burst : {1.0, 1.5, 4.0}) {
      int previous = 0;
      for (int k = 1; k <= 600; ++k) {
        const int v = k + abr::frame_parity(k, loss, burst, p, 2, 0.02);
        EXPECT_GE(v + 1, previous) << "k=" << k << " loss=" << loss << " burst=" << burst;
        previous = v;
      }
    }
  }
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ParityIsReservedForFramesOver255Packets) {
  Params p;
  p.bitrate_max = 40e6;
  p.startup_fraction = 1.0;
  Control c(p);
  c.reset(40e6);
  c.set_fps(15);
  Decision d;
  for (double t = p.Tc; t <= 3.0; t += p.Tc) {
    Measurement m = sample(t, c.budget(), 0.030, 0.02);
    m.loss_burst = 1.2;
    d = c.step(m);
  }
  const double per_packet = p.mtu * 8.0 * 15;
  const double usable = (d.budget - p.audio_bps) * p.mtu / (p.mtu + p.header_bytes);
  const int k = static_cast<int>(std::ceil(d.video_rate / per_packet - 1e-9));
  ASSERT_GT(k, 255);
  const int parity = abr::frame_parity(k, d.loss_est, d.loss_burst, p, d.min_parity, d.repair);
  EXPECT_GT(parity, 0);
  EXPECT_GE((usable - d.video_rate) / per_packet, parity - 1.0);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, OneGarbageDelayDoesNotKillLaterSamples) {
  train_rig_t rig;
  const int64_t t0 = 100000000;
  int64_t t = t0;
  double ms = 0;
  for (; t < t0 + 20000000; t += 16667) {
    rig.frame(t, 0, &ms);
  }
  rig.frame(t, 1000000000u, &ms);  // +1000 s, once
  EXPECT_LT(ms, 1000.0);
  int accepted = 0;
  int offered = 0;
  for (t += 16667; t < t0 + 40000000; t += 16667) {
    ms = 0;
    rig.frame(t, 0, &ms);
    ++offered;
    if (ms > 0) {
      ++accepted;
    }
    EXPECT_LT(ms, 1000.0);
  }
  EXPECT_GT(accepted, offered / 3) << "samples taken after the corrupt one";
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ADelayThatReallyMovedIsAdoptedAfterConfirmation) {
  train_rig_t rig;
  const int64_t t0 = 100000000;
  int64_t t = t0;
  double ms = 0;
  for (; t < t0 + 10000000; t += 16667) {
    rig.frame(t, 0, &ms);
  }
  int accepted = 0;
  for (; t < t0 + 20000000; t += 16667) {
    ms = 0;
    rig.frame(t, 10000000u, &ms);  // the client's clock moved 10 s for good
    if (ms > 0) {
      ++accepted;
    }
  }
  EXPECT_GT(accepted, 100);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ARepeatedFrameIndexIsOneSample) {
  SignalMeter s;
  s.rtt(0.040);
  for (uint32_t i = 1; i <= 6; ++i) {
    s.sent(frame(i));
  }
  ReceivedTrain r;
  r.index = 3;
  r.packets = 10;
  r.bytes = 10000;
  r.t_first_us = 1000;
  r.t_last_us = 1000 + 3000;
  double q = 0;
  double g = 0;
  s.train(r, q, g);
  for (int i = 0; i < 100000; ++i) {
    s.train(r, q, g);
  }
  Measurement m;
  ASSERT_TRUE(s.measure(200000000, m));
  EXPECT_EQ(m.trains.size(), 1u);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ASummaryAheadOfTheLastSentFrameIsIgnored) {
  SignalMeter s;
  s.rtt(0.060);
  for (uint32_t i = 1; i <= 30; ++i) {
    s.sent(frame(i));
  }
  s.summary(50000, {0, 10});
  s.summary(100000, {0, 1000000000u});  // impossible
  s.summary(150000, {0, 20});
  Measurement m;
  ASSERT_TRUE(s.measure(150000, m));
  EXPECT_NEAR(m.received_rate, 10 * 11000 * 8 / 0.1, 1e3) << "frames 11 to 20, not up to the bogus one";
}

// @ABR-01-S1
TEST(AbrRobustnessTest, AFirstSummaryAheadOfTheLastSentFrameIsNotTheReference) {
  SignalMeter s;
  s.rtt(0.060);
  for (uint32_t i = 1; i <= 30; ++i) {
    s.sent(frame(i));
  }
  s.summary(50000, {0, 1000000000u});
  EXPECT_FALSE(s.has_summary());
  s.summary(100000, {0, 5});
  s.summary(150000, {0, 15});
  Measurement m;
  ASSERT_TRUE(s.measure(150000, m));
  EXPECT_GT(m.received_rate, 5 * 11000 * 8 / 0.15);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, LostFramesAreCappedByTheAdvance) {
  SignalMeter s;
  s.rtt(0.060);
  for (uint32_t i = 1; i <= 12; ++i) {
    s.sent(frame(i));
  }
  s.summary(50000, {0, 0});
  s.summary(100000, {1000000u, 6});  // six frames of advance cannot have lost a million
  Measurement m;
  ASSERT_TRUE(s.measure(100000, m));
  EXPECT_LE(m.loss, 0.5 + 1e-9);
}

// @ABR-01-S1 @ABR-10-S10
TEST(AbrRobustnessTest, FecStatusIsClampedToWhatTheHostSent) {
  SignalMeter s;
  s.rtt(0.060);
  for (uint32_t i = 1; i <= 6; ++i) {
    s.sent(frame(i));  // 10 packets, 9 of data
  }
  s.fec_status({.index = 3, .data_total = 20, .parity_total = 2, .data_received = 20, .parity_received = 2, .missing = 65535});  // everything arrived, 65535 "missing"
  Measurement m;
  ASSERT_TRUE(s.measure(100000, m));
  EXPECT_DOUBLE_EQ(m.loss, 0.0);
  EXPECT_LE(m.loss_burst, 10.0);
}

// @ABR-01-S1 @ABR-10-S10
TEST(AbrRobustnessTest, FecStatusTotalsAreClamped) {
  SignalMeter s;
  s.rtt(0.060);
  for (uint32_t i = 1; i <= 6; ++i) {
    s.sent(frame(i));
  }
  s.fec_status({.index = 3, .data_total = 1000, .parity_total = 1000, .data_received = 0, .parity_received = 0, .missing = -1});
  Measurement m;
  ASSERT_TRUE(s.measure(100000, m));
  EXPECT_LE(m.loss, 10.0 / 60.0 + 1e-9);
  EXPECT_LE(m.loss_burst, 10.0);
}

// @ABR-01-S1 @ABR-10-S10
TEST(AbrRobustnessTest, FecStatusUnknownOrRepeatedFramesAreIgnored) {
  SignalMeter s;
  s.rtt(0.060);
  for (uint32_t i = 1; i <= 6; ++i) {
    s.sent(frame(i));
  }
  s.fec_status({.index = 99, .data_total = 9, .parity_total = 1, .data_received = 8, .parity_received = 1});  // never sent
  for (int i = 0; i < 50; ++i) {
    s.fec_status({.index = 3, .data_total = 9, .parity_total = 1, .data_received = 8, .parity_received = 1});
  }
  Measurement m;
  ASSERT_TRUE(s.measure(100000, m));
  EXPECT_NEAR(m.loss, 1.0 / 60.0, 1e-9) << "once, not fifty times";
}

// @ABR-01-S1
TEST(AbrRobustnessTest, AClientPingJumpDoesNotHideAQueue) {
  EXPECT_NEAR(queue_seen(5000u, false), 0.060, 0.005);
  EXPECT_NEAR(queue_seen(5000u, true), 0.060, 0.005);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, TheClientClockWrapsCleanly) {
  EXPECT_NEAR(queue_seen(0xFFFFFFFFu - 103000u, false), 0.060, 0.005);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ABackoffAfterTheAlarmIsNotUndoneByTheRetraction) {
  const Params p;
  Control c(p);
  c.reset(20e6);
  double t = 0;
  for (t = p.Tc; t <= 8.0; t += p.Tc) {
    c.step(sample(t, 10e6, 0.030, 0));
  }
  ASSERT_TRUE(c.alarm(t, 0.060, 0.030, 10e6));
  Decision d;
  for (int i = 0; i < 2; ++i) {
    t += p.Tc;
    d = c.step(sample(t, 10e6, 0.200, 0.40));
  }
  ASSERT_EQ(d.reason, Reason::loss_backoff);
  const double after_backoff = c.budget();
  EXPECT_FALSE(c.retract_alarm(t + 0.02, 0.003));
  EXPECT_DOUBLE_EQ(c.budget(), after_backoff);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, EncoderRequestStaysUnderTheLargestBudgetsVideo) {
  Params p;
  p.bitrate_max = 8e6;
  p.startup_fraction = 1.0;
  p.encoder_compensation_max = 1.6;
  p.encoder_time_const = 0.5;
  Control c(p);
  c.reset(p.bitrate_max);
  const double overhead = static_cast<double>(p.mtu) / (p.mtu + p.header_bytes);
  const double cap = (p.bitrate_max - p.audio_bps) * overhead;
  double request = 0;
  double worst = 0;
  for (double t = p.Tc; t < 60.0; t += p.Tc) {
    Measurement m = sample(t, c.budget(), 0.030, 0);
    m.data_rate = 0.5 * request;  // the encoder delivers half of what is asked
    const Decision d = c.step(m);
    request = d.encoder_request;
    worst = std::max(worst, request);
  }
  EXPECT_LE(worst, cap + 1.0);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ResetLeavesTheControllerAsFreshAsANewOne) {
  const Params p;
  auto drive = [&](Control &c, std::deque<double> &out) {
    double t = 0;
    for (int i = 0; i < 200; ++i) {
      t += p.Tc;
      const Decision d = c.step(sample(t, c.budget(), i > 100 ? 0.120 : 0.030, i > 100 ? 0.05 : 0));
      out.push_back(d.budget);
      out.push_back(d.video_rate);
      out.push_back(d.encoder_request);
    }
  };
  Control used(p);
  used.reset(20e6);
  std::deque<double> scratch;
  drive(used, scratch);
  used.alarm(100.0, 0.080, 0.030, 5e6);  // may leave the alarm open
  used.reset(20e6);
  EXPECT_FALSE(used.retract_alarm(100.01, 0.0));
  Control fresh(p);
  fresh.reset(20e6);
  std::deque<double> a;
  std::deque<double> b;
  drive(used, a);
  drive(fresh, b);
  EXPECT_EQ(a, b);
}

// @ABR-01-S1
TEST(AbrRobustnessTest, ExtremeInputsKeepTheBudgetFiniteAndInRange) {
  std::mt19937_64 g(12345);
  auto uniform = [&](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(g);
  };
  for (int run = 0; run < 40; ++run) {
    // The budget has no deliberate cut below the minimum: its floor is bitrate_min.
    struct: Params {
      double probe_cut = 0;
    } p;

    p.bitrate_max = run % 3 == 0 ? 8e6 : 100e6;
    p.encoder_compensation_max = run % 4 == 0 ? 1.6 : 1.0;
    Control c(p);
    c.reset(p.bitrate_max);
    double t = 100;
    for (int i = 0; i < 1000; ++i) {
      t += p.Tc * uniform(0.5, 1.5);
      Measurement m;
      m.t = t;
      m.owd_min = uniform(-1, 5);
      m.owd_mean = m.owd_min + uniform(-0.5, 3);
      m.rtt = uniform(0, 5);
      m.loss = uniform(-0.5, 1.5);
      m.received_rate = uniform(-1e6, 1e9);
      m.sent_rate = uniform(-1e6, 1e9);
      m.delivery = uniform(-1, 5);
      m.noise = uniform(-1, 3);
      m.loss_burst = uniform(-5, 1e5);
      m.data_rate = uniform(-1, 1e9);
      for (int k = 0; k < 6; ++k) {
        m.trains.push_back({uniform(-1e3, 1e7), uniform(-1, 10)});
      }
      const Decision d = c.step(m);
      ASSERT_TRUE(std::isfinite(d.budget) && std::isfinite(d.video_rate) && std::isfinite(d.encoder_request)) << "run " << run << " step " << i;
      ASSERT_GE(c.budget(), p.bitrate_min - 1);
      ASSERT_LE(c.budget(), p.bitrate_max + 1);
      ASSERT_GE(d.budget, p.bitrate_min * (1 - p.probe_cut) - 1);
      ASSERT_LE(d.budget, p.bitrate_max + 1);
      if (i % 3 == 0) {
        c.alarm(t + 0.02, uniform(-1, 5), uniform(-1, 1), uniform(-1, 1e9));
      }
      if (i % 3 == 1) {
        c.retract_alarm(t + 0.03, uniform(-1, 1));
      }
      ASSERT_TRUE(std::isfinite(c.budget()));
      ASSERT_GE(c.budget(), p.bitrate_min - 1);
      ASSERT_LE(c.budget(), p.bitrate_max + 1);
      if (i % 250 == 249) {
        c.reset(p.bitrate_max);
      }
    }
  }
}
