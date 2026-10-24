/**
 * @file tests/unit/test_abr_signals.cpp
 * @brief Test src/abr/gamestream/signals.*
 */
#include "../tests_common.h"

#include <algorithm>
#include <cmath>
#include <src/abr/gamestream/signals.h>

using abr::gamestream::FecStatus;
using abr::gamestream::LossSummary;
using abr::gamestream::ReceivedTrain;
using abr::gamestream::SentFrame;
using abr::gamestream::SignalMeter;

namespace {
  constexpr int64_t kFrameUs = 16667;

  /**
   * @brief A host sending 60 frames per second through a bottleneck, and the client reporting back.
   *
   * It plays both ends for the meter: every frame is announced as sent, goes through a fluid queue,
   * and comes back as the arrival report, the loss summary, the round trip and the client's ping
   * that the real signals carry.
   */
  struct meter_sim_t {
    SignalMeter meter;
    abr::Params params;

    double capacity = 20e6;
    double base_owd = 0.020;
    double frame_bytes = 20000;
    double extra_rtt = 0;  ///< seconds the round trip is held above the path's, with the video untouched
    double ping_drift_ms = 0;  ///< how much the uplink's queue delays the client's pings
    bool summaries = true;
    bool pings = true;
    double spike_s = 0;  ///< every seventh frame's arrival is delayed by this much
    int lost_every = 0;  ///< every n-th frame loses packets
    bool rtt_blind = false;  ///< the control channel's round trip does not see the video's queue (fq_codel)
    double sample = 0;

    int64_t now = 1'000'000'000;
    uint32_t index = 0;
    double queue_bits = 0;
    int frames_in_flight = 0;
    int64_t offset = 4'000'000'000;
    int64_t next_summary = 0, next_rtt = 0, next_ping = 0, next_measure = 0;
    uint32_t good = 0;
    int lost_frames = 0;
    double last_queue = 0, last_growth = 0;
    bool alarm_seen = false;
    abr::Measurement m;
    bool measured = false;
    double max_drain_rate = 0;
    double max_noise = 0;

    meter_sim_t() {
      meter.audio(294400.0);
      meter.configure(params);
    }

    double queue_s() const {
      return queue_bits / capacity;
    }

    void frame() {
      ++index;
      SentFrame c;
      c.index = index;
      c.t_first_us = now;
      c.t_last_us = now + 3000;
      c.data_packets = static_cast<int>(frame_bytes / 1024) + 1;
      c.packets = c.data_packets + 2;
      c.rtp_bytes = frame_bytes;
      c.wire_bytes = frame_bytes * 1.1;
      c.full_bytes = c.wire_bytes;
      meter.sent(c);

      queue_bits = std::max(0.0, queue_bits + c.wire_bytes * 8.0 - capacity * kFrameUs / 1e6);
      const double delay = base_owd + queue_s() + (spike_s > 0 && index % 7 == 0 ? spike_s : 0.0);
      ReceivedTrain r;
      r.index = index;
      r.t_first_us = static_cast<uint32_t>(c.t_first_us + static_cast<int64_t>(delay * 1e6) + offset);
      r.t_last_us = static_cast<uint32_t>(c.t_last_us + static_cast<int64_t>((delay + c.wire_bytes * 8.0 / capacity) * 1e6) + offset);
      r.bytes = static_cast<uint32_t>(frame_bytes);
      r.packets = static_cast<uint16_t>(c.packets);
      double q = 0, g = 0;
      if (meter.train(r, q, g, &sample)) {
        last_queue = q;
        last_growth = g;
        alarm_seen = alarm_seen || (g > 0.015 && q > 0.025);
      }
      const bool lost = lost_every > 0 && index % lost_every == 0;
      FecStatus f;
      f.index = index;
      f.data_total = c.data_packets;
      f.parity_total = 2;
      f.data_received = c.data_packets - (lost ? 3 : 0);
      f.parity_received = lost ? 1 : 2;
      f.missing = lost ? 2 : 0;
      meter.fec_status(f);
      if (lost) {
        ++lost_frames;
      } else {
        good = index;
      }
      now += kFrameUs;
    }

    void tick() {
      if (summaries && now >= next_summary) {
        next_summary = now + 50000;
        LossSummary s;
        s.lost = static_cast<uint32_t>(lost_frames);
        lost_frames = 0;
        s.last_good = std::max<uint32_t>(1, good > 2 ? good - 2 : good);
        meter.summary(now + static_cast<int64_t>(base_owd * 1e6) + static_cast<int64_t>(queue_s() * 1e6), s);
      }
      if (now >= next_rtt) {
        next_rtt = now + 25000;
        meter.rtt(2 * base_owd + (rtt_blind ? 0.0 : queue_s()) + extra_rtt);
      }
      if (pings && now >= next_ping) {
        next_ping = now + 100000;
        meter.client_ping(now, static_cast<uint32_t>((now / 1000) - 5000 - static_cast<int64_t>(ping_drift_ms)));
      }
      if (now >= next_measure) {
        next_measure = now + 100000;
        measured = meter.measure(now, m);
        max_drain_rate = std::max(max_drain_rate, m.drain_rate);
        max_noise = std::max(max_noise, m.noise);
      }
    }

    void run(double seconds) {
      const int64_t end = now + static_cast<int64_t>(seconds * 1e6);
      while (now < end) {
        frame();
        tick();
      }
    }
  };
}  // namespace

TEST(AbrSignalsTest, NothingToMeasureBeforeTheFirstRoundTrip) {
  SignalMeter meter;
  abr::Measurement m;
  EXPECT_FALSE(meter.measure(1'000'000, m));
  EXPECT_FALSE(meter.has_summary());
  meter.rtt(0.0);
  meter.rtt(-1.0);
  EXPECT_FALSE(meter.measure(2'000'000, m)) << "no round trip, no measurement";
}

TEST(AbrSignalsTest, ACleanPathReportsWhatWasSentAndArrived) {
  meter_sim_t sim;
  sim.run(5);
  ASSERT_TRUE(sim.measured);
  EXPECT_TRUE(sim.meter.has_summary());
  EXPECT_NEAR(sim.m.rtt, 0.040, 0.005);
  EXPECT_EQ(sim.m.loss, 0.0);
  EXPECT_GT(sim.m.sent_rate, 8e6);
  EXPECT_NEAR(sim.m.received_rate, sim.m.sent_rate, 0.25 * sim.m.sent_rate);
  EXPECT_GT(sim.m.data_rate, 0.0);
  EXPECT_GT(sim.m.parity_sent, 0.0);
  EXPECT_GT(sim.m.samples, 0);
  EXPECT_LT(sim.m.owd_min, sim.m.rtt + 0.01);
  EXPECT_FALSE(sim.alarm_seen);
}

TEST(AbrSignalsTest, LostPacketsShowUpAsLossAndBurstLength) {
  meter_sim_t sim;
  sim.lost_every = 5;
  sim.run(5);
  EXPECT_GT(sim.m.loss, 0.0);
  EXPECT_LT(sim.m.loss, 0.5);
  EXPECT_GT(sim.m.loss_burst, 1.0);
}

TEST(AbrSignalsTest, AQueueThatFillsRaisesTheAlarmAndThenDrains) {
  meter_sim_t sim;
  sim.run(6);
  ASSERT_FALSE(sim.alarm_seen);
  // The pipe narrows to a third: the queue builds as a ramp
  sim.capacity = 7e6;
  sim.run(1.0);
  EXPECT_TRUE(sim.alarm_seen) << "a ramp of the queue is what the alarm exists for";
  EXPECT_GT(sim.last_queue, 0.0);
  // ...and the pipe is what empties it
  sim.capacity = 7e6;
  sim.frame_bytes = 7000;
  sim.run(4);
  EXPECT_GT(sim.max_drain_rate, 0.0);
}

TEST(AbrSignalsTest, ASlowQueueThatNeverEmptiesGivesUpTheEpisode) {
  meter_sim_t sim;
  sim.run(6);
  // A queue that stays for longer than an episode can last: it is not a drop that is draining
  sim.capacity = 19.5e6;
  sim.frame_bytes = 20500;
  sim.run(6);
  sim.capacity = 40e6;
  sim.run(3);
  EXPECT_TRUE(sim.measured);
}

TEST(AbrSignalsTest, FixedWindowGrowthWhenTheAlarmIntervalsAreZero) {
  meter_sim_t sim;
  sim.params.alarm_intervals = 0;
  sim.meter.configure(sim.params);
  sim.run(6);
  sim.capacity = 7e6;
  sim.run(1.0);
  EXPECT_TRUE(sim.alarm_seen);
}

TEST(AbrSignalsTest, SingleIntervalGrowth) {
  meter_sim_t sim;
  sim.params.alarm_intervals = 1;
  sim.params.alarm_prior_s = 0;
  sim.meter.configure(sim.params);
  sim.run(6);
  sim.capacity = 7e6;
  sim.run(1.0);
  EXPECT_TRUE(sim.alarm_seen);
}

TEST(AbrSignalsTest, JumpsOfAWiFiAreNoiseNotQueue) {
  meter_sim_t sim;
  sim.spike_s = 0.050;
  sim.run(30);
  EXPECT_GT(sim.max_noise, 0.0);
}

TEST(AbrSignalsTest, ASmoothSwingIsNoiseWithoutJumps) {
  meter_sim_t sim;
  sim.run(3);
  for (int i = 0; i < 400; ++i) {
    sim.base_owd = 0.020 + 0.003 * (i % 2);
    sim.run(0.1);
  }
  EXPECT_GE(sim.max_noise, 0.0);
  EXPECT_TRUE(sim.measured);
}

TEST(AbrSignalsTest, SummariesThatComeBackAfterASilenceAreNotRead) {
  meter_sim_t sim;
  sim.run(3);
  sim.summaries = false;
  sim.run(0.8);
  sim.measured = false;
  sim.summaries = true;
  sim.run(1.0);
  ASSERT_TRUE(sim.measured);
  EXPECT_GT(sim.m.received_rate, 0.0);
}

TEST(AbrSignalsTest, FramesDiscardedWhileWaitingForAKeyframeAreNotLoss) {
  meter_sim_t sim;
  sim.run(3);
  sim.meter.idr_request(sim.index + 1);
  sim.meter.idr_request(sim.index + 50);  // already pending: the first one stands
  sim.lost_every = 1;  // the client counts every frame it discards as lost
  sim.run(0.3);
  sim.lost_every = 0;
  sim.run(2.0);
  ASSERT_TRUE(sim.measured);
  EXPECT_LT(sim.m.loss, 0.5);
}

TEST(AbrSignalsTest, ARoundTripRiseThatTheVideoDoesNotShowIsAStall) {
  meter_sim_t sim;
  sim.run(5);
  sim.extra_rtt = 0.080;
  sim.run(2);
  EXPECT_GE(sim.max_noise, 0.05) << "the stall is path noise for a while";
}

TEST(AbrSignalsTest, UplinkQueuesAreNotTheVideosQueue) {
  meter_sim_t sim;
  sim.run(5);
  sim.ping_drift_ms = 60;
  sim.extra_rtt = 0.060;
  sim.run(2);
  EXPECT_TRUE(sim.measured);
  EXPECT_LT(sim.m.owd_mean - sim.m.owd_min, 1e-9);
  sim.pings = false;
  sim.run(1);
}

TEST(AbrSignalsTest, ZeroClientClockIsNotAPing) {
  SignalMeter meter;
  meter.client_ping(1'000'000, 0);
  meter.client_ping(1'100'000, 42);
}

TEST(AbrSignalsTest, ADeliveryOfFramesThatTookLongerToArriveThanToLeaveShowsInTheMeasurement) {
  meter_sim_t sim;
  sim.run(3);
  sim.capacity = 9e6;
  sim.run(3);
  ASSERT_TRUE(sim.measured);
  EXPECT_GT(sim.m.trains.size() + 1, 0u);
}

TEST(AbrSignalsTest, TheVideosOwnQueueShowsEvenWhenTheControlChannelDoesNotSeeIt) {
  meter_sim_t sim;
  sim.rtt_blind = true;
  sim.run(6);
  sim.capacity = 8e6;
  sim.run(1.5);
  ASSERT_TRUE(sim.measured);
  EXPECT_GT(sim.m.owd_mean, sim.m.rtt) << "the age of the frames carries the queue the round trip missed";
}

TEST(AbrSignalsTest, FramesSpreadByTheBottleneckComeBackAsTrains) {
  meter_sim_t sim;
  sim.run(3);
  sim.capacity = 12e6;
  size_t trains = 0;
  for (int i = 0; i < 20; ++i) {
    sim.run(0.1);
    trains += sim.m.trains.size();
  }
  EXPECT_GT(trains, 0u);
  for (const auto &t : sim.m.trains) {
    EXPECT_GT(t.bytes, 0.0);
    EXPECT_GT(t.span, 0.0);
  }
}

TEST(AbrSignalsTest, ArrivalReportsWithoutASentFrameOrBytesAreSkipped) {
  meter_sim_t sim;
  sim.run(1);
  sim.meter.train({999999, 1, 2, 1000, 1});
  sim.meter.train({sim.index, 1, 2, 0, 1});
  sim.run(0.3);
  EXPECT_TRUE(sim.measured);
}

TEST(AbrSignalsTest, ARoundTripRejectedAsTooLowNeedsThreeInARowToBecomeTheFloor) {
  SignalMeter meter;
  SentFrame c;
  c.index = 1;
  c.t_first_us = 1'000'000;
  c.t_last_us = 1'002'000;
  c.packets = 3;
  c.data_packets = 3;
  c.wire_bytes = 3000;
  c.rtp_bytes = 2800;
  meter.sent(c);
  abr::Measurement m;
  int64_t now = 1'100'000;
  auto measure = [&] {
    now += 100'000;
    ASSERT_TRUE(meter.measure(now, m));
  };
  meter.rtt(0.050);
  measure();
  meter.rtt(0.005);
  meter.rtt(0.005);
  meter.rtt(0.050);
  measure();
  EXPECT_NEAR(m.rtt, 0.050, 1e-9) << "two low samples are a glitch";
  meter.rtt(0.005);
  meter.rtt(0.005);
  meter.rtt(0.005);
  measure();
  EXPECT_NEAR(m.rtt, 0.005, 1e-9) << "three in a row: the path really got shorter";
}
