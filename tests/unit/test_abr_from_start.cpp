/**
 * @file tests/unit/test_abr_from_start.cpp
 * @brief Test that the adaptive bitrate controller has a session from its start when the client
 * announces the signals, and that a ping never hands it a session whose client did not.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <vector>

// local includes
#include <moonlight-common-c/src/Limelight-internal.h>
#include <src/adaptive_bitrate.h>

using namespace std::literals;
using adaptive_bitrate::mode_e;

namespace {
  adaptive_bitrate::session_t make_session(mode_e mode, bool announced) {
    return {mode, 20000, 60, 1024, 90, 300000.0, 2, announced};
  }
}  // namespace

// @ABR-12-S9 @ABR-12-S10 @ABR-12-S11 @ABR-12-S18 @ABR-12-ASM-09
TEST(AbrFromStartTest, AnAnnouncedSessionIsTheControllersFromTheStart) {
  const int announced = ML_FF_FEC_STATUS | ML_FF_SESSION_ID_V1 | ML_FF_ADAPTIVE_SIGNALS;
  for (const auto mode : {mode_e::fec, mode_e::full}) {
    EXPECT_TRUE(adaptive_bitrate::session_wanted(mode, announced)) << "the first encoder has the repair references";
    EXPECT_FALSE(adaptive_bitrate::session_wanted(mode, ML_FF_FEC_STATUS | ML_FF_SESSION_ID_V1));
    auto session = make_session(mode, true);
    EXPECT_TRUE(session.active()) << "before any signal arrives";
    EXPECT_EQ(adaptive_bitrate::control_wait(&session), 5ms);
    // The budget the stream is paced at is full mode's alone
    EXPECT_DOUBLE_EQ(session.budget_bps(), mode == mode_e::full ? abr::Params {}.startup_fraction * 20e6 : 0.0);
    EXPECT_EQ(session.own_sender(), mode == mode_e::full) << "full mode sends from the first frame";
    EXPECT_EQ(session.pacing_bps() > 0.0, mode == mode_e::full);
  }
  EXPECT_FALSE(adaptive_bitrate::session_wanted(mode_e::disabled, announced));
}

// @ABR-08-S4
TEST(AbrFromStartTest, APingNeverActivatesTheControllerOfAClientWithoutTheBit) {
  auto session = make_session(mode_e::full, false);
  std::vector<char> ping;
  for (const std::uint32_t v : {4u, 123456u}) {
    for (int shift = 0; shift < 32; shift += 8) {
      ping.push_back(char(v >> shift));
    }
  }
  EXPECT_TRUE(session.client_ping(std::chrono::steady_clock::now(), {ping.data(), ping.size()}));
  EXPECT_FALSE(session.active()) << "only the client's feature flags hand the stream to the controller";
}
