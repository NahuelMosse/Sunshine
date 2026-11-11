/**
 * @file tests/unit/test_abr_stock_parity_flag.cpp
 * @brief Test that a session is the adaptive bitrate controller's from its start when its client
 * announces the signals, in the modes that act on it, and stays upstream's when it does not.
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

namespace {
  /**
   * @brief The periodic ping of a client that fills in its clock.
   */
  std::vector<char> ping_with_clock(std::uint32_t client_ms) {
    std::vector<char> out;
    for (std::uint32_t v : {std::uint32_t {4}, client_ms}) {
      for (int shift = 0; shift < 32; shift += 8) {
        out.push_back(char(v >> shift));
      }
    }
    return out;
  }

  /**
   * @brief A loss stats message as moonlight-common-c lays it out, little-endian.
   */
  std::vector<char> loss_stats() {
    std::vector<char> out;
    for (std::uint32_t v : {0u, 50u, 1000u, 1u, 0u, 0u, 0u, 0x14u}) {
      for (int shift = 0; shift < 32; shift += 8) {
        out.push_back(char(v >> shift));
      }
    }
    return out;
  }

  /**
   * @brief The per-frame FEC status every Moonlight sends.
   */
  std::vector<char> fec_status(std::uint32_t frame) {
    std::vector<char> out(21, 0);
    out[0] = char(frame >> 24);
    out[1] = char(frame >> 16);
    out[2] = char(frame >> 8);
    out[3] = char(frame);
    out[11] = 40;  // data packets
    out[13] = 2;  // parity packets
    out[15] = 40;  // data packets received
    out[17] = 2;  // parity packets received
    out[18] = 20;  // FEC percentage
    out[20] = 1;  // FEC blocks
    return out;
  }

  adaptive_bitrate::session_t make_session(adaptive_bitrate::mode_e mode, bool announced) {
    return {mode, 20000, 60, 1024, 90, 300000.0, 2, announced};
  }

  constexpr int announcing = ML_FF_FEC_STATUS | ML_FF_SESSION_ID_V1 | ML_FF_ADAPTIVE_SIGNALS;
}  // namespace

// @ABR-02-S7 @ABR-02-S4 @ABR-02-ASM-03
TEST(AbrStockParityFlagTest, ASessionWithoutTheBitKeepsTheBroadcastPathAndThe150MsLoop) {
  EXPECT_EQ(adaptive_bitrate::control_wait(nullptr), 150ms) << "with the feature off there is no controller";

  for (auto mode : {adaptive_bitrate::mode_e::fec, adaptive_bitrate::mode_e::full}) {
    EXPECT_FALSE(adaptive_bitrate::session_wanted(mode, ML_FF_FEC_STATUS | ML_FF_SESSION_ID_V1)) << "its encoder opens as upstream's";
    auto session = make_session(mode, false);
    // A client without the bit, whatever it sends: the FEC status, pings with its clock, loss stats
    const auto start = std::chrono::steady_clock::now();
    for (std::uint32_t i = 1; i <= 120; ++i) {
      auto fec = fec_status(i);
      ASSERT_TRUE(session.fec_status({fec.data(), fec.size()}));
      auto ping = ping_with_clock(1000 + i);
      session.client_ping(start + i * 16ms, {ping.data(), ping.size()});
      auto loss = loss_stats();
      session.loss_stats(start + i * 16ms, {loss.data(), loss.size()});
      session.tick(start + i * 16ms, 0.060);
    }
    EXPECT_FALSE(session.own_sender()) << "its packets leave from the broadcast sender";
    EXPECT_EQ(adaptive_bitrate::control_wait(&session), 150ms) << "the control loop keeps upstream's period";
    EXPECT_EQ(session.pacing_bps(), 0.0) << "upstream's pacing";
    EXPECT_EQ(session.budget_bps(), 0.0) << "no frame is held";
  }
}

// @ABR-02-S3 @ABR-02-ASM-04 @ABR-02-ASM-02
TEST(AbrStockParityFlagTest, TheControllerHasTheSessionFromTheStartWithTheBit) {
  for (auto mode : {adaptive_bitrate::mode_e::fec, adaptive_bitrate::mode_e::full}) {
    EXPECT_TRUE(adaptive_bitrate::session_wanted(mode, announcing)) << "the encoder opens with the references the controller repairs from";
    auto session = make_session(mode, true);
    ASSERT_TRUE(session.active());
    EXPECT_EQ(session.own_sender(), mode == adaptive_bitrate::mode_e::full) << "only full mode paces from a sender of its own";
    EXPECT_EQ(adaptive_bitrate::control_wait(&session), 5ms);
  }
}
