/**
 * @file tests/unit/test_adaptive_bitrate.cpp
 * @brief Test src/adaptive_bitrate.*
 */
#include "../tests_common.h"

#include <src/adaptive_bitrate.h>

#include <cstring>
#include <vector>

using namespace std::literals;

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

  // An SS_FRAME_FEC_STATUS message, laid out as moonlight-common-c sends it: big-endian
  std::vector<char> fec_status_message(std::uint32_t frame, int data, int parity, int data_received, int parity_received) {
    std::vector<char> out;
    put32be(out, frame);
    put16be(out, 0);  // highestReceivedSequenceNumber
    put16be(out, 0);  // nextContiguousSequenceNumber
    put16be(out, std::uint16_t(data + parity - data_received - parity_received));  // missing
    put16be(out, std::uint16_t(data));
    put16be(out, std::uint16_t(parity));
    put16be(out, std::uint16_t(data_received));
    put16be(out, std::uint16_t(parity_received));
    out.push_back(20);  // fecPercentage
    out.push_back(0);  // multiFecBlockIndex
    out.push_back(1);  // multiFecBlockCount
    return out;
  }

  // A loss stats message, laid out as moonlight-common-c sends it: little-endian
  std::vector<char> loss_stats_message(std::uint32_t lost, std::uint32_t last_good_frame) {
    std::vector<char> out;
    put32le(out, lost);
    put32le(out, 50);
    put32le(out, 1000);
    put32le(out, last_good_frame);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0x14);
    return out;
  }

  adaptive_bitrate::session_t make_session(adaptive_bitrate::mode_e mode) {
    return {mode, 20000, 60, 1024, 90, 300000.0};
  }

  // Seconds of frames at 60 FPS through a path that is never congested: every frame arrives,
  // and the client says so every 50 ms with its loss stats.
  void feed_clean_seconds(adaptive_bitrate::session_t &session, int seconds) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 60 * seconds; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 3 == 2) {
        auto msg = loss_stats_message(0, i + 1);
        ASSERT_TRUE(session.loss_stats(sent + 30ms, {msg.data(), msg.size()}));
      }
      session.tick(sent + 50ms, 0.060);
    }
  }
}  // namespace

TEST(AdaptiveBitrateTest, ParsesMode) {
  EXPECT_EQ(adaptive_bitrate::mode_from_string("shadow"sv), adaptive_bitrate::mode_e::shadow);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("fec"sv), adaptive_bitrate::mode_e::fec);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("disabled"sv), adaptive_bitrate::mode_e::disabled);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("anything"sv), adaptive_bitrate::mode_e::disabled);
}

TEST(AdaptiveBitrateTest, RejectsMalformedReports) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  EXPECT_FALSE(session.fec_status(""sv));
  EXPECT_FALSE(session.loss_stats(std::chrono::steady_clock::now(), ""sv));

  auto fec = fec_status_message(1, 10, 2, 9, 2);
  EXPECT_FALSE(session.fec_status({fec.data(), fec.size() - 1}));
  EXPECT_TRUE(session.fec_status({fec.data(), fec.size()}));

  auto stats = loss_stats_message(0, 1);
  EXPECT_FALSE(session.loss_stats(std::chrono::steady_clock::now(), {stats.data(), 8}));
  EXPECT_TRUE(session.loss_stats(std::chrono::steady_clock::now(), {stats.data(), stats.size()}));
}

TEST(AdaptiveBitrateTest, DecidesFromReportedFrames) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  EXPECT_EQ(session.target_bitrate_kbps(), 0);
  EXPECT_EQ(session.fec_percentage(20), 20) << "before any decision the configured FEC must be used";

  // Past the first seconds, which are protected as if there were loss
  feed_clean_seconds(session, 3);

  EXPECT_GT(session.target_bitrate_kbps(), 0);
  EXPECT_LE(session.target_bitrate_kbps(), 20000) << "the controller must never exceed what the client asked for";
  const int fec = session.fec_percentage(20);
  EXPECT_GE(fec, 1);
  EXPECT_LE(fec, 255);

  // Without loss each block carries the client's minimum of two parity packets, and the
  // percentage has to make fec::encode produce exactly that.
  for (int k = 1; k <= 100; ++k) {
    const int pct = session.block_fec_percentage(k);
    EXPECT_EQ((k * pct + 99) / 100, 2) << k << " data shards";
  }
  // Tiny blocks: the percentage travels in 8 bits.
  EXPECT_LE(session.block_fec_percentage(1), 255);
  EXPECT_EQ(session.pacing_bps(), 0.0) << "only full mode paces the stream";
}

TEST(AdaptiveBitrateTest, LossReportedInFecStatusRaisesParity) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  feed_clean_seconds(session, 3);
  const int clean = session.block_fec_percentage(40);

  // A second in which every frame lost four of its packets: a tenth of the path. Two parity
  // packets per block, the client's minimum, cannot repair that. It starts after the clean
  // seconds, whose steps already scheduled the next one that far ahead.
  const auto start = std::chrono::steady_clock::now() + 4s;
  for (int i = 0; i < 60; ++i) {
    const auto sent = start + std::chrono::microseconds(i * 16667);
    session.frame_sent(1000 + i, sent, sent, 42, 40, 42 * 1040.0, 42 * 1130.0);
    auto fec = fec_status_message(1000 + i, 40, 2, 36, 2);
    ASSERT_TRUE(session.fec_status({fec.data(), fec.size()}));
    session.tick(sent + 50ms, 0.060);
  }
  EXPECT_GT(session.block_fec_percentage(40), clean);
}

TEST(AdaptiveBitrateTest, FullModePacesAtTwiceTheBudget) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  EXPECT_EQ(session.pacing_bps(), 0.0) << "until the client sends its signals the default pacing must be used";

  // From the first report on, and before any decision, the stream runs on the controller's
  // starting budget, not on what the client asked for
  auto stats = loss_stats_message(0, 1);
  ASSERT_TRUE(session.loss_stats(std::chrono::steady_clock::now(), {stats.data(), stats.size()}));
  EXPECT_TRUE(session.active());
  EXPECT_GT(session.pacing_bps(), 0.0);
  EXPECT_LT(session.pacing_bps(), 2 * 20000 * 1000.0);
  EXPECT_GT(session.initial_encoder_kbps(), 0);
  EXPECT_LT(session.initial_encoder_kbps(), 20000);
  EXPECT_EQ(session.target_bitrate_kbps(), session.initial_encoder_kbps());

  feed_clean_seconds(session, 1);

  // Twice the whole budget, which is above the image the encoder is asked for
  const double pacing = session.pacing_bps();
  EXPECT_GT(pacing, 2 * 1000.0 * session.target_bitrate_kbps());
  EXPECT_LE(pacing, 2 * 20000 * 1000.0);
}

TEST(AdaptiveBitrateTest, PingsEnetOnItsOwnSchedule) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(session.enet_ping_due(start)) << "no extra traffic for a client that does not send the signals";
  auto stats = loss_stats_message(0, 1);
  ASSERT_TRUE(session.loss_stats(start, {stats.data(), stats.size()}));
  EXPECT_TRUE(session.enet_ping_due(start));
  EXPECT_FALSE(session.enet_ping_due(start + 1ms));
  EXPECT_TRUE(session.enet_ping_due(start + 100ms));

  auto disabled = make_session(adaptive_bitrate::mode_e::disabled);
  EXPECT_FALSE(disabled.enet_ping_due(start));
}

TEST(AdaptiveBitrateTest, ParsesClientPing) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  // A 32-bit length of 4 and a timestamp left at zero, as every client fills it for other hosts
  std::vector<char> blank;
  put32le(blank, 4);
  put32le(blank, 0);
  EXPECT_TRUE(session.client_ping(std::chrono::steady_clock::now(), {blank.data(), blank.size()}));
  EXPECT_FALSE(session.active()) << "a ping without the client's clock is not one of its signals";

  // The 16-bit length older clients wrote, and the client's clock in milliseconds
  std::vector<char> ping {4, 0};
  put32le(ping, 123456);
  EXPECT_TRUE(session.client_ping(std::chrono::steady_clock::now(), {ping.data(), ping.size()}));
  EXPECT_FALSE(session.client_ping(std::chrono::steady_clock::now(), {ping.data(), 4}));
  EXPECT_TRUE(session.active());
}

TEST(AdaptiveBitrateTest, ClientWithoutSignalsKeepsTheStreamAsItWas) {
  // A client that only sends the per-frame FEC status, like every Moonlight does
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 180; ++i) {
    const auto sent = start + std::chrono::microseconds(i * 16667);
    session.frame_sent(i + 1, sent, sent, 42, 40, 42 * 1040.0, 42 * 1130.0);
    auto fec = fec_status_message(i + 1, 40, 2, 36, 2);
    ASSERT_TRUE(session.fec_status({fec.data(), fec.size()}));
    session.enet_sample(sent + 20ms, 60 + (i % 40) * 10);
    session.tick(sent + 50ms, 0.060);
    EXPECT_TRUE(session.idr_requested(sent + 50ms)) << "every IDR request must be answered";
  }
  EXPECT_FALSE(session.active());
  EXPECT_EQ(session.target_bitrate_kbps(), 0);
  EXPECT_EQ(session.bitrate_to_apply(start + 4s), 0);
  EXPECT_EQ(session.fec_percentage(20), 20);
  EXPECT_EQ(session.block_fec_percentage(40), -1);
  EXPECT_EQ(session.min_parity(2), 2);
  EXPECT_EQ(session.pacing_bps(), 0.0);
  EXPECT_EQ(session.budget_bps(), 0.0);
  EXPECT_EQ(session.take_frame_divider(), 0);
  EXPECT_FALSE(session.take_idr_request());
}

TEST(AdaptiveBitrateTest, SteadyRoundTripRaisesNoAlarm) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  feed_clean_seconds(session, 3);
  const double budget = session.budget_bps();
  // Four seconds of round-trip samples every 25 ms at the floor: the queue seen stays at zero
  const auto start = std::chrono::steady_clock::now() + 4s;
  for (int i = 0; i < 160; ++i) {
    session.enet_sample(start + std::chrono::milliseconds(25 * i), 60 + (i % 2));
  }
  EXPECT_LT(session.queue_s(), 0.005);
  EXPECT_EQ(session.budget_bps(), budget);
}

namespace {
  std::vector<char> client_ping_message(std::uint32_t client_ms) {
    std::vector<char> out;
    put32le(out, 4);
    put32le(out, client_ms);
    return out;
  }

  // Four seconds in, a path that is steady at 60 ms with the client's ping arriving on time, then a
  // sample of `late_ms` that arrives together with a ping held up `held_ms`
  void steady_then_late_sample(adaptive_bitrate::session_t &session, double late_ms, int held_ms) {
    feed_clean_seconds(session, 3);
    (void) session.take_frame_divider();
    const auto start = std::chrono::steady_clock::now() + 4s;
    for (int i = 0; i < 160; ++i) {
      const auto at = start + std::chrono::milliseconds(25 * i);
      if (i % 4 == 0) {
        auto ping = client_ping_message(std::uint32_t(100000 + 25 * i));
        ASSERT_TRUE(session.client_ping(at, {ping.data(), ping.size()}));
      }
      session.enet_sample(at, 60 + (i % 2) * 0.4);
    }
    const auto at = start + 4s;
    auto ping = client_ping_message(std::uint32_t(100000 + 4000 - held_ms));
    ASSERT_TRUE(session.client_ping(at, {ping.data(), ping.size()}));
    session.enet_sample(at + 1ms, late_ms);
  }
}  // namespace

// The round trip that rose together with the client's ping is the uplink's: it does not make the
// host skip frames, nor cut the budget
TEST(AdaptiveBitrateTest, ASampleHeldUpWithItsPingChangesNeitherFramesNorBudget) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  steady_then_late_sample(session, 560, 500);
  const double budget = session.budget_bps();
  EXPECT_EQ(session.take_frame_divider(), 0);
  EXPECT_LT(session.queue_s(), 0.005);
  EXPECT_GT(budget, 0.0);
}

// The same sample with the ping on time is the video's queue, and the frames in flight go down
TEST(AdaptiveBitrateTest, TheSameSampleWithThePingOnTimeSkipsFrames) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  steady_then_late_sample(session, 560, 0);
  EXPECT_NE(session.take_frame_divider(), 0);
}

TEST(AdaptiveBitrateTest, ShadowModeNeverTouchesFec) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  EXPECT_EQ(session.fec_percentage(20), 20);
  EXPECT_EQ(session.block_fec_percentage(10), -1);
}
