/**
 * @file tests/unit/test_abr_batched_stamps.cpp
 * @brief Test that a frame the receiver stamped in a batch gives no queue sample, and that an alarm
 * without repair reach encodes one frame in two.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <vector>

// local includes
#include <src/abr/gamestream/signals.h>
#include <src/adaptive_bitrate.h>

using namespace std::literals;
using abr::gamestream::SentFrame;
using abr::gamestream::SignalMeter;

namespace {
  /**
   * @brief Thirty frames of four packets sent over 4 ms, each stamped by the receiver `span_us` apart.
   * @return True if any of them gave a queue sample.
   */
  bool gives_a_sample(std::uint32_t span_us, double span_min) {
    abr::Params p;
    p.alarm_span_min = span_min;
    SignalMeter meter;
    meter.configure(p);
    meter.rtt(0.060);
    double queue = 0;
    double growth = 0;
    bool any = false;
    for (std::uint32_t k = 1; k <= 30; ++k) {
      const std::int64_t sent_us = static_cast<std::int64_t>(k) * 16667;
      SentFrame frame;
      frame.index = k;
      frame.t_first_us = sent_us;
      frame.t_last_us = sent_us + 4000;
      frame.packets = frame.data_packets = 4;
      frame.wire_bytes = frame.rtp_bytes = 4000;
      meter.sent(frame);
      const auto arrived = static_cast<std::uint32_t>(sent_us + 4000000000LL);
      double sample = 0;
      meter.train({k, arrived, arrived + span_us, 4000, 4}, queue, growth, &sample);
      any = any || sample > 0;
    }
    return any;
  }

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

  // A loss stats message, laid out as moonlight-common-c sends it: little-endian
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
  std::vector<char> frame_stats_message(std::chrono::steady_clock::time_point start, int first, int last, Delay delay_us) {
    std::vector<char> out;
    out.push_back(char(last - first + 1));
    out.push_back(0);
    put16be(out, 0);
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      const auto at = std::uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(sent.time_since_epoch()).count() + 4000000000LL + delay_us(i));
      put32be(out, i + 1);
      put32be(out, at);
      put32be(out, at);
      put32be(out, 12 * 1040);
      put16be(out, 12);
    }
    return out;
  }

  // Frames at 60 FPS from `start` with the signals a client that announced them sends
  template<typename Delay>
  void drive_frames(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int first, int last, Delay delay_us) {
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 5 == 4) {
        auto msg = frame_stats_message(start, i - 4, i, delay_us);
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
}  // namespace

// @ABR-13-S1 @ABR-13-S6 @ABR-13-ASM-02
TEST(AbrBatchedStampsTest, ACompressedTrainGivesNoQueueSample) {
  EXPECT_FALSE(gives_a_sample(0, 0.5)) << "a frame stamped all at once is the receiver, not the queue";
  EXPECT_FALSE(gives_a_sample(1500, 0.5)) << "under half of the 4 ms the host took to send it";
}

// @ABR-13-S7 @ABR-13-ASM-03
TEST(AbrBatchedStampsTest, ANormalOrStretchedTrainGivesASample) {
  EXPECT_TRUE(gives_a_sample(4000, 0.5)) << "a train that arrived as it was sent";
  EXPECT_TRUE(gives_a_sample(2000, 0.5)) << "exactly half the send span is not under it";
  EXPECT_TRUE(gives_a_sample(9000, 0.5)) << "a train the queue stretched";
}

// @ABR-13-S8
TEST(AbrBatchedStampsTest, TheFilterAtZeroKeepsEverySample) {
  EXPECT_TRUE(gives_a_sample(0, 0)) << "with the filter off every frame gives a sample";
  EXPECT_TRUE(gives_a_sample(1500, 0));
}

// @ABR-13-S3 @ABR-13-S9
TEST(AbrBatchedStampsTest, AnInvalidationBehindAQueueKeepsOneFrameInTwo) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::full, 20000, 60, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, [](int) {
    return 30000;
  });
  // On a path with nothing queued a lost frame is not worth skipping others
  session.invalidation_requested(start + 6s);
  EXPECT_EQ(session.take_frame_divider(), 0);
  // With a queue standing, the share is one frame in two, whether the queue's alarm or the invalidation set it
  drive_frames(session, start, 300, 359, [](int i) {
    return 30000 + std::min(i - 300, 8) * 20000;
  });
  session.invalidation_requested(start + 7s);
  EXPECT_EQ(session.take_frame_divider(), 500);
}
