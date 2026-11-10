/**
 * @file tests/unit/test_abr_feature_flag.cpp
 * @brief Test the feature bits that announce the adaptive bitrate signals: the client's in
 * `x-ml-general.featureFlags`, the host's in `x-ss-general.featureFlags`, and the session each makes.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <string_view>
#include <vector>

// local includes
#include <moonlight-common-c/src/Limelight-internal.h>
#include <src/adaptive_bitrate.h>
#include <src/platform/common.h>
#include <src/utility.h>

using namespace std::literals;
using adaptive_bitrate::mode_e;

namespace {
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

  std::string_view view(const std::vector<char> &bytes) {
    return {bytes.data(), bytes.size()};
  }

  /// The client's feature flags as rtsp.cpp reads them from the ANNOUNCE.
  int parsed(std::string_view field) {
    return static_cast<int>(util::from_view(field));
  }

  adaptive_bitrate::session_t make_session(mode_e mode, bool announced) {
    return {mode, 20000, 60, 1024, 90, 300000.0, 2, announced};
  }
}  // namespace

// @ABR-12-S2 @ABR-12-S6 @ABR-12-S7 @ABR-12-ASM-02
TEST(AbrFeatureFlagTest, TheBitsAreTheAgreedValues) {
  EXPECT_EQ(ML_FF_ADAPTIVE_SIGNALS, 0x04);
  EXPECT_EQ(ML_FF_ADAPTIVE_SIGNALS & (ML_FF_FEC_STATUS | ML_FF_SESSION_ID_V1), 0) << "a free client bit";
  EXPECT_EQ(adaptive_bitrate::host_feature_bit, 0x04u);
  EXPECT_EQ(adaptive_bitrate::host_feature_bit & (platf::platform_caps::pen_touch | platf::platform_caps::controller_touch), 0u) << "the next free host bit";
}

// @ABR-12-S1 @ABR-12-S16 @ABR-12-ASM-03 @ABR-12-ASM-08
TEST(AbrFeatureFlagTest, TheClientBitIsReadFromItsFeatureFlags) {
  EXPECT_TRUE(adaptive_bitrate::announces_signals(parsed("5"))) << "the bit and FEC status";
  EXPECT_TRUE(adaptive_bitrate::announces_signals(parsed("7"))) << "with the session id too";
  EXPECT_FALSE(adaptive_bitrate::announces_signals(parsed("3"))) << "today's clients";
  EXPECT_FALSE(adaptive_bitrate::announces_signals(parsed("4"))) << "without FEC status the controller is blind";
  EXPECT_FALSE(adaptive_bitrate::announces_signals(parsed("0"))) << "the field's default";
}

// @ABR-12-S12 @ABR-12-S13 @ABR-12-S19 @ABR-12-ASM-05
TEST(AbrFeatureFlagTest, WithoutTheBitTheSignalsNeverTakeTheSessionOver) {
  for (const auto mode : {mode_e::fec, mode_e::full}) {
    auto session = make_session(mode, false);
    const auto now = std::chrono::steady_clock::now();
    session.frame_sent(1, now, now, 12, 10, 12 * 1040.0, 12 * 1130.0);

    std::vector<char> loss;
    for (const std::uint32_t v : {0u, 50u, 1000u, 1u, 0u, 0u, 0u, 0x14u}) {
      put32le(loss, v);
    }
    EXPECT_TRUE(session.loss_stats(now + 30ms, view(loss)));

    std::vector<char> stats {1, 0, 0, 0};
    put32be(stats, 1);
    put32be(stats, 1000);
    put32be(stats, 2000);
    put32be(stats, 12 * 1040);
    put16be(stats, 12);
    EXPECT_TRUE(session.frame_stats(now + 30ms, view(stats)));

    std::vector<char> ping;
    put32le(ping, 4);
    put32le(ping, 123456);
    EXPECT_TRUE(session.client_ping(now + 40ms, view(ping)));

    session.tick(now + 50ms, 0.060);
    EXPECT_FALSE(session.active()) << "neither loss stats, Frame Stats nor a ping";
    EXPECT_EQ(adaptive_bitrate::control_wait(&session), 150ms);
    EXPECT_FALSE(session.own_sender());
    EXPECT_EQ(session.pacing_bps(), 0.0);
    EXPECT_EQ(session.bitrate_to_apply(now + 60ms), 0);
    EXPECT_TRUE(session.idr_requested(now + 60ms));
  }
}
