/**
 * @file tests/unit/test_abr_fec_blocks.cpp
 * @brief Test the FEC status of a frame sent in several Reed-Solomon blocks: the layout of the
 * report, the blocks GameStream can carry and what the blocks of one frame may claim together.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <cstdint>
#include <string_view>
#include <vector>

// local includes
#include <src/abr/control.h>
#include <src/abr/gamestream/signals.h>
#include <src/adaptive_bitrate.h>

using abr::gamestream::FecStatus;
using abr::gamestream::SentFrame;
using abr::gamestream::SignalMeter;
using abr::Measurement;

namespace {
  SentFrame sent_frame(uint32_t i) {
    SentFrame c;
    c.index = i;
    c.t_first_us = 100000000 + static_cast<int64_t>(i) * 16667;
    c.t_last_us = c.t_first_us + 1000;
    c.packets = 10;
    c.data_packets = 9;
    c.rtp_bytes = 10000.0;
    c.wire_bytes = 11000.0;
    return c;
  }

  // Six frames of ten packets, nine of data and one of parity
  void send_six(SignalMeter &s) {
    s.rtt(0.060);
    for (uint32_t i = 1; i <= 6; ++i) {
      s.sent(sent_frame(i));
    }
  }

  // A block that arrived empty: every packet it claims is lost
  FecStatus empty_block(uint32_t frame, int block) {
    FecStatus status;
    status.index = frame;
    status.data_total = 9;
    status.parity_total = 1;
    status.block = block;
    return status;
  }

  std::vector<char> fec_status_message(int block, int blocks) {
    std::vector<char> out(21, 0);
    out[3] = 1;  // frameIndex 1
    out[11] = 9;  // totalDataPackets
    out[13] = 1;  // totalParityPackets
    out[15] = 9;  // receivedDataPackets
    out[17] = 1;  // receivedParityPackets
    out[18] = 10;  // fecPercentage
    out[19] = static_cast<char>(block);  // multiFecBlockIndex
    out[20] = static_cast<char>(blocks);  // multiFecBlockCount
    return out;
  }
}  // namespace

// @ABR-10-S5 @ABR-10-S6
TEST(AbrFecBlockTest, BlockIsTheLastMember) {
  const FecStatus status {1, 2, 3, 4, 5, 6};
  EXPECT_EQ(status.index, 1u);
  EXPECT_EQ(status.data_total, 2);
  EXPECT_EQ(status.missing, 6) << "the six fields the protocol always had keep their places";
  EXPECT_EQ(status.block, 0);
}

// @ABR-10-S8
TEST(AbrFecBlockTest, ABlockIndexOfFourOrMoreIsIgnored) {
  SignalMeter s;
  send_six(s);
  s.fec_status(empty_block(3, 4));
  s.fec_status(empty_block(4, 7));
  Measurement m;
  ASSERT_TRUE(s.measure(100000, m));
  EXPECT_DOUBLE_EQ(m.loss, 0.0) << "GameStream carries the block in two bits";

  SignalMeter control;
  send_six(control);
  control.fec_status(empty_block(3, 3));
  ASSERT_TRUE(control.measure(100000, m));
  EXPECT_GT(m.loss, 0.0) << "block 3 is the last one a frame can have";
}

// @ABR-10-S2 @ABR-10-S9
TEST(AbrFecBlockTest, BlocksNeverClaimMoreThanTheFrameHad) {
  SignalMeter s;
  send_six(s);
  for (int block = 0; block < 4; ++block) {
    s.fec_status(empty_block(3, block));
  }
  Measurement m;
  ASSERT_TRUE(s.measure(100000, m));
  EXPECT_NEAR(m.loss, 10.0 / 60.0, 1e-9) << "four blocks of one frame lose at most its ten packets";
}

// @ABR-10-S11 @ABR-10-ASM-02
TEST(AbrFecBlockTest, TheParserDropsABlockOutsideItsCount) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  auto ok = fec_status_message(3, 4);
  EXPECT_TRUE(session.fec_status({ok.data(), ok.size()}));
  auto first = fec_status_message(0, 1);
  EXPECT_TRUE(session.fec_status({first.data(), first.size()}));
  auto past_count = fec_status_message(1, 1);
  EXPECT_FALSE(session.fec_status({past_count.data(), past_count.size()})) << "data[19] < data[20]";
  auto too_many = fec_status_message(4, 5);
  EXPECT_FALSE(session.fec_status({too_many.data(), too_many.size()})) << "data[20] <= 4";
  auto no_blocks = fec_status_message(0, 0);
  EXPECT_FALSE(session.fec_status({no_blocks.data(), no_blocks.size()}));
}
