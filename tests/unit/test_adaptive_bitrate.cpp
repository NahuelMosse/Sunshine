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

namespace {
  // An SS_FRAME_STATS message with one entry per frame in [first, last]: the client's clock runs
  // `offset_us` ahead of the host's, and each frame takes `delay_us(i)` to arrive.
  template<typename Delay>
  std::vector<char> frame_stats_message(std::chrono::steady_clock::time_point start, int first, int last, std::int64_t offset_us, Delay delay_us) {
    std::vector<char> out;
    out.push_back(char(last - first + 1));
    out.push_back(0);
    put16be(out, 0);
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      const auto at = std::uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(sent.time_since_epoch()).count() + offset_us + delay_us(i));
      put32be(out, i + 1);
      put32be(out, at);
      put32be(out, at);
      put32be(out, 12 * 1040);
      put16be(out, 12);
    }
    return out;
  }

  // Frames sent at 60 FPS from `start` whose arrival is reported every fifth frame, after the
  // frames' own delay, with the host's round trip at 60 ms
  template<typename Delay>
  void frames_with_delay(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int first, int last, Delay delay_us) {
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 5 == 4) {
        auto msg = frame_stats_message(start, i - 4, i, 4000000000LL, delay_us);
        ASSERT_TRUE(session.frame_stats(sent + 30ms, {msg.data(), msg.size()}));
      }
    }
  }
}  // namespace

TEST(AdaptiveBitrateTest, RejectsMalformedFrameStats) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  EXPECT_FALSE(session.frame_stats(std::chrono::steady_clock::now(), ""sv));
  std::vector<char> two {2, 0, 0, 0};
  put32be(two, 1);
  EXPECT_FALSE(session.frame_stats(std::chrono::steady_clock::now(), {two.data(), two.size()}));
}

TEST(AdaptiveBitrateTest, SteadyFrameDelayRaisesNoAlarm) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  feed_clean_seconds(session, 3);
  const double budget = session.budget_bps();
  // Four seconds of frames whose delay stays at the floor, under a client clock with its own offset
  const auto start = std::chrono::steady_clock::now() + 4s;
  frames_with_delay(session, start, 0, 239, [](int i) { return 30000 + (i % 2) * 400; });
  EXPECT_LT(session.queue_s(), 0.005);
  EXPECT_EQ(session.budget_bps(), budget);
}

// A queue the frames have to cross, built over a few frames, is the video's: the frames in flight go
// down at once
TEST(AdaptiveBitrateTest, FramesDelayedByAQueueSkipFrames) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  feed_clean_seconds(session, 3);
  (void) session.take_frame_divider();
  const auto start = std::chrono::steady_clock::now() + 4s;
  frames_with_delay(session, start, 0, 239, [](int i) { return 30000 + (i % 2) * 400; });
  frames_with_delay(session, start, 240, 299, [](int) { return 30000 + 500000; });
  EXPECT_NE(session.take_frame_divider(), 0);
}
TEST(AdaptiveBitrateTest, ShadowModeNeverTouchesFec) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  EXPECT_EQ(session.fec_percentage(20), 20);
  EXPECT_EQ(session.block_fec_percentage(10), -1);
}
