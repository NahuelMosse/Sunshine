/**
 * @file tests/unit/test_adaptive_bitrate_start.cpp
 * @brief Test what an adaptive bitrate session starts from, in src/adaptive_bitrate.*
 */
#include "../tests_common.h"

#include <src/adaptive_bitrate.h>

// @ABR-01-S3 @ABR-01-ASM-03
TEST(AdaptiveBitrateTest, TheStartingEncoderBitrateFollowsTheBitrateTheSessionIsGiven) {
  // The stream hands the session the bitrate the client asked for: the budget covers the whole
  // wire, and the encoder starts on the image part of its starting fraction
  const adaptive_bitrate::session_t asked {adaptive_bitrate::mode_e::full, 30000, 60, 1024, 90, 300000.0};
  const adaptive_bitrate::session_t reduced {adaptive_bitrate::mode_e::full, 24000, 60, 1024, 90, 300000.0};
  const double start_bps = abr::Params {}.startup_fraction * 30e6;
  const int expected_kbps = static_cast<int>((start_bps * 1024 / (1024 + 90) - 300000.0) / 1000.0);
  EXPECT_EQ(asked.initial_encoder_kbps(), expected_kbps);
  EXPECT_GT(asked.initial_encoder_kbps(), reduced.initial_encoder_kbps());
}
