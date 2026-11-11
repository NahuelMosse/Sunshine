/**
 * @file tests/unit/test_stream_pacing.cpp
 * @brief Test when the packets of a video frame are due, in src/stream_pacing.h.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ratio>
#include <utility>

// lib includes
#include <boost/asio.hpp>

// local includes
#include <src/adaptive_bitrate.h>
#include <src/nvenc/nvenc_ltr.h>
#include <src/platform/common.h>
#include <src/rtsp.h>
#include <src/stream.h>
#include <src/stream_internal.h>
#include <src/stream_pacing.h>
#include <src/video.h>

using namespace std::literals;

namespace {
  /**
   * @brief The packets sent in a millisecond, as stream.cpp computed it before the pacing moved here.
   */
  std::size_t reference_packets_in_1ms(std::size_t blocksize) {
    return std::giga::num * 80 / 100 / 1000 / blocksize / 8;
  }

  /**
   * @brief When the packets are due, as stream.cpp computed it before the pacing moved here.
   */
  auto reference_due(std::size_t blocksize, std::size_t packets_sent) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(1ms) * packets_sent / reference_packets_in_1ms(blocksize);
  }
}  // namespace

TEST(StreamPacingTest, DefaultsToEightyPercentOfOneGigabit) {
  EXPECT_EQ(stream::pacing(1392, 0).packets_in_1ms, 71U);
  EXPECT_EQ(stream::pacing(1024, 0).packets_in_1ms, 97U);
  EXPECT_FALSE(stream::pacing(1392, 0).adaptive);
}

TEST(StreamPacingTest, DueMatchesTheFormulaStreamUsedBefore) {
  for (const std::size_t blocksize : {64U, 512U, 1024U, 1392U, 1432U, 4096U}) {
    const auto pacing = stream::pacing(blocksize, 0);
    EXPECT_EQ(pacing.packets_in_1ms, reference_packets_in_1ms(blocksize));
    for (std::size_t packets : {0U, 1U, 7U, 64U, 71U, 72U, 500U, 1000U, 5000U}) {
      EXPECT_EQ(pacing.due(packets).count(), reference_due(blocksize, packets).count()) << blocksize << " " << packets;
    }
  }
}

TEST(StreamPacingTest, AMillisecondOfPacketsIsDueAfterAMillisecond) {
  const auto pacing = stream::pacing(1392, 0);
  EXPECT_EQ(pacing.due(0), 0ns);
  EXPECT_EQ(pacing.due(pacing.packets_in_1ms), 1ms);
  EXPECT_EQ(pacing.due(3 * pacing.packets_in_1ms), 3ms);
}

TEST(StreamPacingTest, ARateBelowTheDefaultPacesEachPacketAtThatRate) {
  const auto pacing = stream::pacing(1250, 10e6);  // 1250 bytes at 10 Mbit/s: a packet each millisecond

  EXPECT_TRUE(pacing.adaptive);
  EXPECT_EQ(pacing.packet_time, 1ms);
  EXPECT_EQ(pacing.packets_in_1ms, 1U);
  EXPECT_EQ(pacing.due(0), 0ns);
  EXPECT_EQ(pacing.due(10), 10ms);
}

TEST(StreamPacingTest, AFastRateSendsSeveralPacketsEveryMillisecond) {
  const auto pacing = stream::pacing(1250, 100e6);  // a packet every 100 us

  EXPECT_EQ(pacing.packet_time, 100us);
  EXPECT_EQ(pacing.packets_in_1ms, 10U);
}

TEST(StreamPacingTest, ASlowRateStillSendsAPacketEveryMillisecond) {
  const auto pacing = stream::pacing(1250, 1e6);  // a packet every 10 ms

  EXPECT_EQ(pacing.packet_time, 10ms);
  EXPECT_EQ(pacing.packets_in_1ms, 1U);
}

TEST(StreamPacingTest, ARateAtOrAboveTheDefaultChangesNothing) {
  const auto fastest = stream::pacing(1392, 0.8e9);
  EXPECT_FALSE(fastest.adaptive);
  EXPECT_EQ(fastest.packets_in_1ms, 71U);
  EXPECT_FALSE(stream::pacing(1392, 5e9).adaptive);
  EXPECT_FALSE(stream::pacing(1392, -1).adaptive);
}

// @ABR-06-S3 @ABR-06-S4 @ABR-06-ASM-01
TEST(StreamPacingTest, APacketIsDueAtThePacingRate) {
  // Without the controller's pacing: upstream's groups, 80 % of 1 Gbit/s a millisecond at a time
  const auto upstream = stream::pacing(1024, 0.0);
  EXPECT_FALSE(upstream.adaptive);
  EXPECT_EQ(upstream.packets_in_1ms, 97u) << "1e9 * 80 / 100 / 1000 / 1024 / 8";
  EXPECT_EQ(upstream.due(97), 1ms);
  EXPECT_EQ(upstream.due(194), 2ms);
  EXPECT_EQ(upstream.batch_size(64), 64u);

  // At or above that rate the controller's pacing is not used
  EXPECT_FALSE(stream::pacing(1024, 800e6).adaptive);
  EXPECT_FALSE(stream::pacing(1024, 900e6).adaptive);

  // 1024 bytes of 8 bits at 8.192 Mbit/s: a packet every millisecond, one per batch
  const auto slow = stream::pacing(1024, 8.192e6);
  EXPECT_TRUE(slow.adaptive);
  EXPECT_EQ(slow.packet_time, 1ms);
  EXPECT_EQ(slow.packets_in_1ms, 1u);
  EXPECT_EQ(slow.due(0), 0ms);
  EXPECT_EQ(slow.due(10), 10ms);
  EXPECT_EQ(slow.batch_size(64), 1u);

  // Ten times faster: ten packets a millisecond, batches of ten
  const auto fast = stream::pacing(1024, 81.92e6);
  EXPECT_TRUE(fast.adaptive);
  EXPECT_EQ(fast.packet_time, 100us);
  EXPECT_EQ(fast.packets_in_1ms, 10u);
  EXPECT_EQ(fast.due(25), 2500us);
  EXPECT_EQ(fast.batch_size(64), 10u);
  EXPECT_EQ(fast.batch_size(4), 4u) << "never larger than the batch upstream allows";

  // Slower than a packet a millisecond: still one per batch
  const auto slowest = stream::pacing(1024, 4.096e6);
  EXPECT_EQ(slowest.packet_time, 2ms);
  EXPECT_EQ(slowest.packets_in_1ms, 1u);
  EXPECT_EQ(slowest.due(3), 6ms);
}

// @ABR-06-S4 @ABR-06-ASM-01
TEST(StreamPacingTest, AFrameIsPaidForTenPercentOverTheBudget) {
  const auto start = std::chrono::steady_clock::now();

  // 11000 bytes on the wire at 10 Mbit/s plus 10 %: 88000 bits at 11 Mbit/s, 8 ms
  const auto paid = stream::paid_until(start, 11000, 10e6);
  const std::chrono::duration<double, std::micro> owed = paid - start;
  EXPECT_NEAR(owed.count(), 8000.0, 0.01);

  // A frame of the budget's size is paid for before the next one is due
  const auto at_budget = stream::paid_until(start, 10e6 / 8 / 60, 10e6);
  EXPECT_LT(at_budget - start, std::chrono::duration_cast<std::chrono::nanoseconds>(1s) / 60);

  // Without a budget there is nothing to pay
  EXPECT_EQ(stream::paid_until(start, 11000, 0.0), start);
}
