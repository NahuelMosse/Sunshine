/**
 * @file tests/unit/test_adaptive_bitrate.cpp
 * @brief Test src/adaptive_bitrate.*
 */
#include "../tests_common.h"

#include <cstdio>
#include <cstring>
#include <src/adaptive_bitrate.h>
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
  frames_with_delay(session, start, 0, 239, [](int i) {
    return 30000 + (i % 2) * 400;
  });
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
  frames_with_delay(session, start, 0, 239, [](int i) {
    return 30000 + (i % 2) * 400;
  });
  frames_with_delay(session, start, 240, 299, [](int) {
    return 30000 + 500000;
  });
  EXPECT_NE(session.take_frame_divider(), 0);
}

TEST(AdaptiveBitrateTest, ShadowModeNeverTouchesFec) {
  auto session = make_session(adaptive_bitrate::mode_e::shadow);
  EXPECT_EQ(session.fec_percentage(20), 20);
  EXPECT_EQ(session.block_fec_percentage(10), -1);
}

namespace {
  // The frames of `frames_with_delay`, with what a client that sends every signal also reports: the loss
  // stats each third frame, and the host's controller step every sixth.
  template<typename Delay>
  void drive_frames(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int first, int last, Delay delay_us) {
    for (int i = first; i <= last; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 5 == 4) {
        auto msg = frame_stats_message(start, i - 4, i, 4000000000LL, delay_us);
        ASSERT_TRUE(session.frame_stats(sent + 30ms, {msg.data(), msg.size()}));
      }
      if (i % 3 == 2) {
        auto msg = loss_stats_message(0, i + 1);
        ASSERT_TRUE(session.loss_stats(sent + 30ms, {msg.data(), msg.size()}));
      }
      if (i % 6 == 5) {
        session.tick(sent + 50ms, 0.060);
      }
    }
  }
}  // namespace

TEST(AdaptiveBitrateTest, ParsesTheFullMode) {
  EXPECT_EQ(adaptive_bitrate::mode_from_string("full"sv), adaptive_bitrate::mode_e::full);
}

TEST(AdaptiveBitrateTest, AQueueRampCutsTheBudgetAtOnce) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, [](int) {
    return 30000;
  });
  const double budget = session.budget_bps();
  const int target = session.target_bitrate_kbps();
  ASSERT_GT(budget, 0.0);
  (void) session.take_frame_divider();

  // The delay of each frame grows by 10 ms per frame: a pipe that narrowed to a fraction
  drive_frames(session, start, 300, 314, [](int i) {
    return 30000 + std::max(0, i - 300) * 10000;
  });
  EXPECT_LT(session.budget_bps(), 0.5 * budget) << "the alarm cuts before the step would";
  EXPECT_LT(session.target_bitrate_kbps(), target);
  EXPECT_LT(session.pacing_bps(), 2 * budget);
  EXPECT_NE(session.take_frame_divider(), 0) << "fewer frames until the queue's loss is repaired";
  EXPECT_GT(session.queue_s(), 0.0);

  // The encoder is asked for the new budget, with the share of frames that went with the cut
  const auto now = start + 6s;
  EXPECT_GT(session.bitrate_to_apply(now), 0);
  EXPECT_EQ(session.bitrate_to_apply(now), 0) << "reconfiguring an encoder is not free: nothing moved";
  EXPECT_GT(session.bitrate_to_apply(now + 2s), 0) << "but it is refreshed once a second at least";
}

TEST(AdaptiveBitrateTest, AnAlarmThatTheNextSampleContradictsIsUndone) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, [](int) {
    return 30000;
  });
  const double budget = session.budget_bps();
  (void) session.take_frame_divider();
  // The delay climbs until the alarm cuts...
  int i = 300;
  for (; i < 340 && session.budget_bps() >= budget; ++i) {
    drive_frames(session, start, i, i, [](int k) {
      return 30000 + (k - 300) * 10000;
    });
  }
  ASSERT_LT(session.budget_bps(), budget);
  EXPECT_NE(session.take_frame_divider(), 0);
  // ...and the very next report shows it back at its floor: a delayed acknowledgement, not a narrower path
  drive_frames(session, start, i, i + 9, [](int) {
    return 30000;
  });
  EXPECT_DOUBLE_EQ(session.budget_bps(), budget) << "everything the alarm changed goes back";
  EXPECT_NE(session.take_frame_divider(), 0) << "the frames' share goes back too, and is published";
}

TEST(AdaptiveBitrateTest, TheFrameDividerFollowsTheFramesInFlightAndFallsSlowly) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 239, [](int) {
    return 30000;
  });
  (void) session.take_frame_divider();
  // A host that holds its frames before sending them counts as much as the path's queue
  session.host_delay(200ms);
  drive_frames(session, start, 240, 299, [](int i) {
    return 30000 + (i % 2) * 400;
  });
  EXPECT_NE(session.take_frame_divider(), 0);
  // Down only after the round trip has stayed short: not at once
  EXPECT_EQ(session.take_frame_divider(), 0);
}

TEST(AdaptiveBitrateTest, KeyframeRequestsAreAnsweredOncePerRoundTrip) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now() + 4s;
  EXPECT_TRUE(session.idr_requested(start)) << "before any signal the host answers every request";
  drive_frames(session, start, 0, 119, [](int) {
    return 30000;
  });
  const auto now = start + 3s;
  EXPECT_TRUE(session.idr_requested(now));
  EXPECT_FALSE(session.idr_requested(now + 100ms)) << "the one already sent is on its way";
  EXPECT_TRUE(session.idr_requested(now + 500ms));
}

TEST(AdaptiveBitrateTest, AKeyframeBringsTheFramesBackToThePlainRate) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, [](int) {
    return 30000;
  });
  // A queue standing makes the invalidation skip frames...
  drive_frames(session, start, 300, 359, [](int i) {
    return 30000 + std::min(i - 300, 8) * 20000;
  });
  session.invalidation_requested(start + 7s);
  ASSERT_NE(session.take_frame_divider(), 0);
  session.invalidation_requested(start + 7s + 100ms);
  // ...but a keyframe is paid for several times over if one frame in several is encoded
  EXPECT_TRUE(session.idr_requested(start + 9s));
  EXPECT_NE(session.take_frame_divider(), 0);
}

TEST(AdaptiveBitrateTest, AnInvalidationBehindAQueueSkipsFrames) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, [](int) {
    return 30000;
  });
  // On a path with nothing queued a lost frame is not worth skipping others
  session.invalidation_requested(start + 6s);
  EXPECT_EQ(session.take_frame_divider(), 0);
  // With a queue standing, it is
  drive_frames(session, start, 300, 359, [](int i) {
    return 30000 + std::min(i - 300, 8) * 20000;
  });
  (void) session.take_frame_divider();
  session.invalidation_requested(start + 7s);
  EXPECT_NE(session.take_frame_divider(), 0);

  auto shadow = make_session(adaptive_bitrate::mode_e::shadow);
  shadow.invalidation_requested(start);
  EXPECT_TRUE(shadow.idr_requested(start)) << "shadow mode answers everything";
}

TEST(AdaptiveBitrateTest, AnEncoderThatKeepsItsBitrateGetsTheBudgetAsFrames) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  session.encoder_bitrate_fixed(20000);
  EXPECT_EQ(session.bitrate_to_apply(std::chrono::steady_clock::now()), 0) << "its bitrate is not the knob";
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 479, [](int) {
    return 30000;
  });
  EXPECT_NE(session.take_frame_divider(), 0) << "the share of frames the budget allows is published";
}

TEST(AdaptiveBitrateTest, OnlyFullModeChangesTheBitrateAndPaces) {
  auto fec = make_session(adaptive_bitrate::mode_e::fec);
  EXPECT_FALSE(fec.paces());
  EXPECT_EQ(fec.bitrate_to_apply(std::chrono::steady_clock::now()), 0);
  auto full = make_session(adaptive_bitrate::mode_e::full);
  EXPECT_TRUE(full.paces());
  EXPECT_EQ(full.min_parity(2), 2) << "until the signals arrive the client's minimum stands";
}

TEST(AdaptiveBitrateTest, RepairFramesGetAStricterBlock) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  feed_clean_seconds(session, 3);
  for (int k : {1, 7, 40, 100}) {
    EXPECT_GE(session.block_fec_percentage(k, true), session.block_fec_percentage(k, false)) << k;
    EXPECT_LE(session.block_fec_percentage(k, true), 255);
  }
  EXPECT_LE(session.min_parity(8), 8);
  EXPECT_GE(session.min_parity(8), 0);
}

TEST(AdaptiveBitrateTest, KeyframeRequestsFromTheAlarmCanBeEnabledAndAreDrainedOnce) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  EXPECT_FALSE(session.take_idr_request());
  const auto start = std::chrono::steady_clock::now() + 4s;
  drive_frames(session, start, 0, 299, [](int) {
    return 30000;
  });
  EXPECT_FALSE(session.take_idr_request()) << "the alarm does not ask for keyframes by default";
}
