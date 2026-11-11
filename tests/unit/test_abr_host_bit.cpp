/**
 * @file tests/unit/test_abr_host_bit.cpp
 * @brief Test the host's feature bit that asks the client for the adaptive bitrate signals.
 */
// test includes
#include "../tests_common.h"

// local includes
#include <src/adaptive_bitrate.h>
#include <src/platform/common.h>

using adaptive_bitrate::mode_e;

// @ABR-15-S1
TEST(AbrHostBitTest, TheHostBitIsAdvertisedOnlyWithTheFeatureOnAndIsNotAPlatformCapability) {
  EXPECT_EQ(adaptive_bitrate::host_feature_flags(mode_e::disabled), 0u) << "with the feature off the exchange is upstream's";
  for (const auto mode : {mode_e::fec, mode_e::full}) {
    EXPECT_EQ(adaptive_bitrate::host_feature_flags(mode), adaptive_bitrate::host_feature_bit) << static_cast<int>(mode);
  }
  EXPECT_EQ(adaptive_bitrate::host_feature_bit, 0x04u);
  EXPECT_EQ(platf::get_capabilities() & adaptive_bitrate::host_feature_bit, 0u) << "the platform's capabilities never carry the bit";
}

// @ABR-12-S8 @ABR-12-S17
TEST(AbrHostBitTest, TheHostAdvertisesItsBitInFecAndFull) {
  EXPECT_EQ(adaptive_bitrate::host_feature_flags(mode_e::disabled), 0u) << "no controller, nothing to ask for";
  EXPECT_EQ(adaptive_bitrate::host_feature_flags(mode_e::fec), adaptive_bitrate::host_feature_bit);
  EXPECT_EQ(adaptive_bitrate::host_feature_flags(mode_e::full), adaptive_bitrate::host_feature_bit);
}
