/**
 * @file tests/unit/test_abr_encoded_rate.cpp
 * @brief Test that image quality goes before frames: no divider after an alarm, frames in flight
 * counted at the encoded rate, the bitrate lowered before any frame is skipped, and the rate
 * keeper comparing payload with payload.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

// local includes
#include <src/adaptive_bitrate.h>

using namespace std::literals;

namespace {
  void put16be(std::vector<char> &out, std::uint16_t v) {
    out.push_back(char(v >> 8));
    out.push_back(char(v));
  }

  void put32be(std::vector<char> &out, std::uint32_t v) {
    out.push_back(char(v >> 24));
    out.push_back(char(v >> 16));
    out.push_back(char(v >> 8));
    out.push_back(char(v));
  }

  void put32le(std::vector<char> &out, std::uint32_t v) {
    out.push_back(char(v));
    out.push_back(char(v >> 8));
    out.push_back(char(v >> 16));
    out.push_back(char(v >> 24));
  }

  std::vector<char> loss_stats_message(std::uint32_t last_good_frame) {
    std::vector<char> out;
    put32le(out, 0);
    put32le(out, 50);
    put32le(out, 1000);
    put32le(out, last_good_frame);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0x14);
    return out;
  }

  // An SS_FRAME_STATS message with one entry per frame in [first, last]: the client's clock runs
  // 4000 s ahead of the host's, and each frame takes `delay_us(i)` to arrive.
  template<typename Delay>
  std::vector<char> frame_stats_message(std::chrono::steady_clock::time_point start, int first, int last, int packets, int interval_us, Delay delay_us) {
    std::vector<char> out;
    out.push_back(char(last - first + 1));
    out.push_back(0);
    put16be(out, 0);
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(static_cast<std::int64_t>(i) * interval_us);
      const auto at = std::uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(sent.time_since_epoch()).count() + 4000000000LL + delay_us(i));
      put32be(out, i + 1);
      put32be(out, at);
      put32be(out, at);
      put32be(out, packets * 1040);
      put16be(out, packets);
    }
    return out;
  }

  // Frames every `interval_us` from `start`, with the signals a client that announced them sends
  template<typename Delay>
  void drive_frames(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int first, int last, Delay delay_us, int packets = 12, int interval_us = 16667) {
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(static_cast<std::int64_t>(i) * interval_us);
      session.frame_sent(i + 1, sent, sent, packets, packets - 2, packets * 1040.0, packets * 1130.0);
      if (i % 5 == 4) {
        auto msg = frame_stats_message(start, i - 4, i, packets, interval_us, delay_us);
        ASSERT_TRUE(session.frame_stats(sent + 30ms, {msg.data(), msg.size()}));
      }
      if (i % 3 == 2) {
        auto msg = loss_stats_message(i + 1);
        ASSERT_TRUE(session.loss_stats(sent + 30ms, {msg.data(), msg.size()}));
      }
      if (i % 6 == 5) {
        session.tick(sent + 50ms, 0.060);
      }
    }
  }

  // A pipe whose delay is 30 ms for 300 frames, then `step_us` more
  struct Step {
    int step_us;

    int operator()(int i) const {
      return 30000 + (i >= 300 ? step_us : 0);
    }
  };

  // At 120 frames per second: 30 ms for 600 frames, then 2 ms more with every frame for `hold` frames, and
  // it stays there. Steeper than that the controller's alarm fires, and the alarm takes the bitrate to
  // its floor on its own.
  struct Ramp {
    int hold;

    int operator()(int i) const {
      return 30000 + std::min(std::max(i - 600, 0), hold) * 2000;
    }
  };

  constexpr int k120 = 8333;

  using adaptive_bitrate::mode_e;
  using adaptive_bitrate::session_t;
}  // namespace

// @ABR-14-S2 @ABR-14-ASM-03
TEST(AbrEncodedRateTest, TheEncodedRateIsTheMeasuredOneNeverAboveNominal) {
  EXPECT_DOUBLE_EQ(adaptive_bitrate::encoded_rate_for(60.0, 120.0), 60.0);
  EXPECT_DOUBLE_EQ(adaptive_bitrate::encoded_rate_for(200.0, 120.0), 120.0) << "never above nominal";
  EXPECT_DOUBLE_EQ(adaptive_bitrate::encoded_rate_for(0.0, 120.0), 120.0) << "no data: nominal";
  EXPECT_DOUBLE_EQ(adaptive_bitrate::encoded_rate_for(-1.0, 120.0), 120.0);
  EXPECT_DOUBLE_EQ(adaptive_bitrate::encoded_rate_for(120.0, 120.0), 120.0) << "at nominal rate nothing changes";
}

// @ABR-14-S2
TEST(AbrEncodedRateTest, AMeasuredRateBelowNominalDoesNotSkipFramesThatFit) {
  // 120 fps asked, 60 produced: 150 ms in flight is 9 frames, within the 12 allowed, where at the
  // nominal rate it would be 18
  session_t session {mode_e::full, 20000, 120, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, Step {0});
  (void) session.take_frame_divider();
  drive_frames(session, start, 300, 340, Step {90000});
  EXPECT_EQ(session.take_frame_divider(), 0) << "the frames in flight fit at the rate the encoder produces";
}

// @ABR-14-S1 @ABR-14-ASM-02
TEST(AbrEncodedRateTest, AnAlarmLowersTheBitrateAndKeepsEveryFrame) {
  session_t session {mode_e::full, 20000, 60, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, Step {0});
  ASSERT_GT(session.budget_bps(), 0.0);
  (void) session.take_frame_divider();
  const int before = session.bitrate_to_apply(start + 5s);
  ASSERT_GT(before, 0);

  // The delay of each frame grows by 10 ms per frame: a pipe that narrowed to a fraction
  const double budget = session.budget_bps();
  drive_frames(session, start, 300, 314, [](int i) {
    return 30000 + std::max(0, i - 300) * 10000;
  });
  ASSERT_LT(session.budget_bps(), 0.5 * budget) << "the alarm fired";
  EXPECT_EQ(session.take_frame_divider(), 0) << "the alarm acts on the bitrate: every captured frame is encoded";
  const int after = session.bitrate_to_apply(start + 7s);
  EXPECT_GT(after, 0);
  EXPECT_LT(after, before) << "and the bitrate drops";
}

// @ABR-14-S3 @ABR-14-ASM-05
TEST(AbrEncodedRateTest, InFlightNearTheReachLowersTheBitrateFirst) {
  // 120 frames per second: twelve frames in flight are 100 ms, and the round trip alone is 60
  session_t session {mode_e::full, 20000, 120, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 599, Ramp {40}, 12, k120);
  (void) session.take_frame_divider();
  // The queue grows past the reach by a few milliseconds
  drive_frames(session, start, 600, 629, Ramp {40}, 12, k120);
  EXPECT_EQ(session.take_frame_divider(), 0) << "no frame is skipped while the bitrate can still drain the queue";
  const int wanted = session.target_bitrate_kbps();
  ASSERT_GT(wanted, 0);
  const int applied = session.bitrate_to_apply(start + 9s);
  EXPECT_GT(applied, 0);
  EXPECT_LT(applied, 0.97 * wanted) << "the encoder is asked for less than the controller's request, to drain what is over the reach";
}

// @ABR-14-S4 @ABR-14-ASM-06
TEST(AbrEncodedRateTest, FramesAreSkippedOnlyAtTheFloorOrWhenTheDrainRunsOut) {
  {
    // The queue stays: the drain window runs out and frames are skipped
    session_t session {mode_e::full, 20000, 120, 1024, 90, 300000.0};
    const auto start = std::chrono::steady_clock::now() + 4s;
    drive_frames(session, start, 0, 599, Ramp {40}, 12, k120);
    (void) session.take_frame_divider();
    drive_frames(session, start, 600, 629, Ramp {40}, 12, k120);
    ASSERT_EQ(session.take_frame_divider(), 0);
    drive_frames(session, start, 630, 700, Ramp {40}, 12, k120);
    EXPECT_EQ(session.take_frame_divider(), 500) << "still over the reach after the drain window: one frame in two";
  }
  {
    // A queue no bitrate above the floor drains in time: frames are skipped at once
    session_t session {mode_e::full, 20000, 120, 1024, 90, 300000.0};
    const auto start = std::chrono::steady_clock::now() + 4s;
    drive_frames(session, start, 0, 599, Ramp {40}, 12, k120);
    (void) session.take_frame_divider();
    drive_frames(session, start, 600, 609, [](int i) {
      return 30000 + (i >= 600 ? 250000 : 0);
    },
                 12,
                 k120);
    EXPECT_EQ(session.take_frame_divider(), 250) << "the floor cannot drain 250 ms in time: one frame in four";
  }
}

// @ABR-14-S4
TEST(AbrEncodedRateTest, FramesComeBackWhenTheInFlightFramesFit) {
  session_t session {mode_e::full, 20000, 120, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 599, Ramp {40}, 12, k120);
  (void) session.take_frame_divider();
  drive_frames(session, start, 600, 700, Ramp {40}, 12, k120);
  ASSERT_EQ(session.take_frame_divider(), 500);

  // The queue is gone: all frames at once, not after a while
  drive_frames(session, start, 701, 820, [](int) {
    return 30000;
  },
               12,
               k120);
  EXPECT_EQ(session.take_frame_divider(), 1000);
}

// @ABR-14-S6 @ABR-14-ASM-09
TEST(AbrEncodedRateTest, TheRateKeeperComparesPayloadWithPayload) {
  // A fixed-rate encoder delivering exactly its request: 1024 bytes of payload in every packet of 1114
  const double payload = 8.0e6;
  const double wire = payload * 1114 / 1024;
  EXPECT_NEAR(adaptive_bitrate::payload_rate(wire, 1024, 1114) / payload, 1.0, 1e-9) << "reads as 100 %, not 109 %";
  EXPECT_NEAR(adaptive_bitrate::payload_rate(0.0, 1024, 1114), 0.0, 1e-9);
}

// @ABR-14-S5 @ABR-14-ASM-07
TEST(AbrEncodedRateTest, AFixedRateEncoderKeepsTheFrameRateMode) {
  session_t session {mode_e::full, 20000, 60, 1024, 90, 300000.0};
  session.encoder_bitrate_fixed(10000);
  const auto start = std::chrono::steady_clock::now() + 4s;
  // The encoder sends far more than the controller's starting budget: only the frame rate can bring it down
  drive_frames(session, start, 0, 299, Step {0}, 40);
  const int share = session.take_frame_divider();
  EXPECT_GT(share, 0);
  EXPECT_LT(share, 1000) << "a share of the frames carries the budget";
  EXPECT_EQ(session.bitrate_to_apply(start + 6s), 0) << "no bitrate is asked of an encoder that cannot change it";
}
