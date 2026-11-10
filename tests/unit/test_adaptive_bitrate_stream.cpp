/**
 * @file tests/unit/test_adaptive_bitrate_stream.cpp
 * @brief Test the adaptive bitrate session as the stream drives it: log lines, malformed messages, the parity of a block, the client bitrate and the mode setting.
 */
// test includes
#include "../tests_common.h"
#include "../tests_log_checker.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

// local includes
#include <src/adaptive_bitrate.h>

using namespace std::literals;
using adaptive_bitrate::mode_e;

namespace {
  constexpr auto log_file = "test_sunshine.log";

  void put32le(std::vector<char> &out, std::uint32_t v) {
    out.push_back(char(v));
    out.push_back(char(v >> 8));
    out.push_back(char(v >> 16));
    out.push_back(char(v >> 24));
  }

  // A loss stats message, laid out as moonlight-common-c sends it: little-endian
  std::vector<char> loss_stats_message(std::uint32_t lost, std::uint32_t last_good_frame) {
    std::vector<char> out;
    put32le(out, lost);
    put32le(out, 50);
    put32le(out, 1000);
    put32le(out, last_good_frame);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0x14);
    return out;
  }

  std::vector<char> loss_stats(std::uint32_t lost, std::uint32_t last_good) {
    std::vector<char> out(32, 0);
    out[0] = char(lost);
    out[12] = char(last_good);
    return out;
  }

  // Frames through a clean path, as the control thread sees them; returns the time reached
  std::chrono::steady_clock::time_point feed(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int frames) {
    for (int i = 0; i < frames; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 3 == 2) {
        auto msg = loss_stats(0, i + 1);
        session.loss_stats(sent + 30ms, {msg.data(), msg.size()});
      }
      session.tick(sent + 50ms, 0.060);
    }
    return start + std::chrono::microseconds(frames * 16667) + 50ms;
  }

}  // namespace

// @ABR-05-S7 @ABR-05-ASM-04
TEST(AdaptiveBitrateStreamTest, TheStepLineIsVerboseAndTheSessionLineDebug) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 180; ++i) {
    const auto sent = start + std::chrono::microseconds(i * 16667);
    session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
    if (i % 3 == 2) {
      auto msg = loss_stats_message(0, i + 1);
      ASSERT_TRUE(session.loss_stats(sent + 30ms, {msg.data(), msg.size()}));
    }
    session.tick(sent + 50ms, 0.060);
  }

  EXPECT_TRUE(log_checker::line_contains(log_file, "Verbose: Adaptive bitrate step: "));
  EXPECT_FALSE(log_checker::line_contains(log_file, "Debug: Adaptive bitrate step: "));
  EXPECT_TRUE(log_checker::line_contains(log_file, "Debug: Adaptive bitrate [fec]: "));
  EXPECT_FALSE(log_checker::line_contains(log_file, "Info: Adaptive bitrate [fec]: "));
}

// @ABR-05-S8 @ABR-05-ASM-05
TEST(AdaptiveBitrateStreamTest, AMalformedMessageWarnsOncePerKind) {
  using adaptive_bitrate::message_e;
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  EXPECT_TRUE(session.malformed(message_e::fec_status));
  EXPECT_FALSE(session.malformed(message_e::fec_status));
  EXPECT_FALSE(session.malformed(message_e::fec_status));
  EXPECT_TRUE(session.malformed(message_e::loss_stats)) << "each kind warns once";
  EXPECT_TRUE(session.malformed(message_e::frame_stats));
  EXPECT_FALSE(session.malformed(message_e::loss_stats));
  EXPECT_FALSE(session.malformed(message_e::frame_stats));

  adaptive_bitrate::session_t next {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  EXPECT_TRUE(next.malformed(message_e::fec_status)) << "once per session";
}

// @ABR-22-S4
TEST(AdaptiveBitrateStreamTest, ABlockTheControllerSizesAtZeroParityGoesOutWithNoFec) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  feed(session, std::chrono::steady_clock::now(), 2400);
  ASSERT_EQ(session.min_parity(2), 0) << "40 s without a loss drop the minimum parity";

  int zero = 0;
  for (int data = 1; data <= 60; ++data) {
    const int percentage = session.block_fec_percentage(data);
    ASSERT_GE(percentage, 0) << data;
    if (percentage == 0) {
      ++zero;
    }
  }
  EXPECT_GT(zero, 0) << "a clean path needs no parity on a small block, and the block is sent with 0 %";
}

// @ABR-18-S5
TEST(AdaptiveBitrateStreamTest, TheClientsBitrateIsClampedToAnInt) {
  constexpr int vqos = 16000;
  constexpr int int_max = std::numeric_limits<int>::max();
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, 20000), 20000);
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, std::int64_t {int_max} + 1), int_max) << "no wrap to a negative or tiny ceiling";
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, std::int64_t {5'000'000'000}), int_max);
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, 0), vqos);
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, -7), vqos) << "zero or negative keeps the vqos value";
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, std::numeric_limits<std::int64_t>::min()), vqos);
}

// @ABR-18-S6
TEST(AdaptiveBitrateStreamTest, FecPercentageZeroFromTheUserStaysZero) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::full, 20000, 60, 1024, 90, 300000.0};
  feed(session, std::chrono::steady_clock::now(), 180);
  ASSERT_TRUE(session.decision()->stepped);
  EXPECT_EQ(session.fec_percentage(0), 0) << "the controller does not raise a percentage the user turned off to 1";
  EXPECT_GE(session.fec_percentage(20), 1);
}

// @ABR-16-S1
TEST(AdaptiveBitrateStreamTest, AnUnknownModeReadsAsDisabled) {
  for (const auto unknown : {"shadow", "bogus", "FEC", " fec"}) {
    EXPECT_EQ(adaptive_bitrate::mode_from_string(unknown), mode_e::disabled) << "a config that says '" << unknown << "' gets the default";
  }
  EXPECT_EQ(adaptive_bitrate::mode_from_string("disabled"), mode_e::disabled);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("fec"), mode_e::fec);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("full"), mode_e::full);
  EXPECT_EQ(adaptive_bitrate::mode_from_string(""), mode_e::disabled);
}
