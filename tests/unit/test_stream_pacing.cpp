/**
 * @file tests/unit/test_stream_pacing.cpp
 * @brief Test when the packets of a video frame are due, in src/stream_pacing.h.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstddef>

// local includes
#include <src/stream_pacing.h>

using namespace std::literals;

TEST(StreamPacingTest, AMillisecondOfPacketsIsDueAfterAMillisecond) {
  const auto pacing = stream::pacing(1392);
  EXPECT_EQ(pacing.due(0), 0ns);
  EXPECT_EQ(pacing.due(pacing.packets_in_1ms), 1ms);
  EXPECT_EQ(pacing.due(3 * pacing.packets_in_1ms), 3ms);
}

TEST(StreamPacingTest, PacketsTooLargeForAMillisecondAreSentOnePerMillisecond) {
  const auto pacing = stream::pacing(std::size_t {1} << 20);
  EXPECT_EQ(pacing.packets_in_1ms, 1U);
  EXPECT_EQ(pacing.due(5), 5ms);
}

TEST(StreamPacingTest, DueNeverDividesByZero) {
  const stream::pacing_t pacing {0};
  EXPECT_EQ(pacing.due(2), 2ms);
}
