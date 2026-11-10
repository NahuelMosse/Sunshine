/**
 * @file tests/unit/test_stream_pacing.cpp
 * @brief Test when the packets of a video frame are due in src/stream_pacing.h.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstddef>
#include <ratio>

// local includes
#include <src/stream_pacing.h>

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
  EXPECT_EQ(stream::pacing(1392).packets_in_1ms, 71U);
  EXPECT_EQ(stream::pacing(1024).packets_in_1ms, 97U);
}

TEST(StreamPacingTest, DueMatchesTheFormulaStreamUsedBefore) {
  for (const std::size_t blocksize : {64U, 512U, 1024U, 1392U, 1432U, 4096U}) {
    const auto pacing = stream::pacing(blocksize);
    EXPECT_EQ(pacing.packets_in_1ms, reference_packets_in_1ms(blocksize));
    for (std::size_t packets : {0U, 1U, 7U, 64U, 71U, 72U, 500U, 1000U, 5000U}) {
      EXPECT_EQ(pacing.due(packets).count(), reference_due(blocksize, packets).count()) << blocksize << " " << packets;
    }
  }
}

TEST(StreamPacingTest, AMillisecondOfPacketsIsDueAfterAMillisecond) {
  const auto pacing = stream::pacing(1392);
  EXPECT_EQ(pacing.due(0), 0ns);
  EXPECT_EQ(pacing.due(pacing.packets_in_1ms), 1ms);
  EXPECT_EQ(pacing.due(3 * pacing.packets_in_1ms), 3ms);
}
