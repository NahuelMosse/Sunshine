/**
 * @file tests/unit/test_abr_pacing.cpp
 * @brief Test the pacing multiple of the adaptive bitrate controller's frames.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>

// local includes
#include <src/stream_pacing.h>

using namespace std::literals;

// @ABR-08-S23 @ABR-08-S27
TEST(AbrPacingTest, TheMultiplierFollowsTheUtilization) {
  constexpr double gamma = 1.1;
  constexpr double budget = 10e6;
  EXPECT_NEAR(stream::pace_multiplier(gamma, budget, 12e6, 2.0), 1.32, 1e-9) << "gamma / u, with u = budget / capacity";
  EXPECT_NEAR(stream::pace_multiplier(gamma, budget, 30e6, 2.0), 2.0, 1e-9) << "at most twice the budget";
  EXPECT_NEAR(stream::pace_multiplier(gamma, budget, 8e6, 2.0), 1.1, 1e-9) << "at least 1.1 times the budget";
  EXPECT_NEAR(stream::pace_multiplier(gamma, budget, 0.0, 2.0), 2.0, 1e-9) << "without a capacity estimate, the fixed multiple";
  EXPECT_NEAR(stream::pace_multiplier(0.0, budget, 12e6, 2.0), 2.0, 1e-9) << "gamma 0 keeps the fixed multiple";
}

// @ABR-08-S24 @ABR-08-S27
TEST(AbrPacingTest, OneProbeFramePerSecondAtTwice) {
  stream::pace_probe_t probe;
  const auto start = std::chrono::steady_clock::now();
  int probes = 0;
  auto last_probe = start - 1h;
  for (int i = 0; i < 180; ++i) {  // three seconds at 60 frames per second
    const auto now = start + i * 16667us;
    const double multiple = probe.frame_multiplier(now, 1.0, 1.3);
    if (multiple == 2.0) {
      ++probes;
      EXPECT_GE(now - last_probe, 1s) << "one probe frame per abr::cal::kPaceProbeS";
      last_probe = now;
    } else {
      EXPECT_DOUBLE_EQ(multiple, 1.3);
    }
  }
  EXPECT_EQ(probes, 3);

  stream::pace_probe_t never;
  EXPECT_DOUBLE_EQ(never.frame_multiplier(start, 0.0, 1.3), 1.3);
  EXPECT_DOUBLE_EQ(never.frame_multiplier(start + 5s, 0.0, 1.3), 1.3) << "a probe period of 0 sends no probe frame";
}
