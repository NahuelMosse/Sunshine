/**
 * @file tests/unit/test_abr_stock_parity_session.cpp
 * @brief Test that only a client that announces the controller's signals gets a controller session.
 */
// test includes
#include "../tests_common.h"

// local includes
#include <moonlight-common-c/src/Limelight-internal.h>
#include <src/adaptive_bitrate.h>

using adaptive_bitrate::mode_e;

// @ABR-15-S4 @ABR-15-ASM-02
TEST(AbrStockParitySessionTest, AClientThatAnnouncesNothingGetsNoControllerInAnyMode) {
  const int official[] = {
    0,
    ML_FF_FEC_STATUS,
    ML_FF_SESSION_ID_V1,
    ML_FF_FEC_STATUS | ML_FF_SESSION_ID_V1,
    ML_FF_ADAPTIVE_SIGNALS,
  };
  for (const auto mode : {mode_e::disabled, mode_e::fec, mode_e::full}) {
    for (const int flags : official) {
      EXPECT_FALSE(adaptive_bitrate::session_wanted(mode, flags)) << "mode " << static_cast<int>(mode) << ", flags " << flags;
    }
  }
}

// @ABR-15-S4
TEST(AbrStockParitySessionTest, AnAnnouncingClientGetsTheControllerInTheActingModes) {
  const int announcing = ML_FF_FEC_STATUS | ML_FF_ADAPTIVE_SIGNALS;
  EXPECT_FALSE(adaptive_bitrate::session_wanted(mode_e::disabled, announcing));
  for (const auto mode : {mode_e::fec, mode_e::full}) {
    EXPECT_TRUE(adaptive_bitrate::session_wanted(mode, announcing)) << static_cast<int>(mode);
    EXPECT_TRUE(adaptive_bitrate::session_wanted(mode, announcing | ML_FF_SESSION_ID_V1)) << static_cast<int>(mode);
  }
}
