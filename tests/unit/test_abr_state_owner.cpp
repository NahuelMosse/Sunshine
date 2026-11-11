/**
 * @file tests/unit/test_abr_state_owner.cpp
 * @brief Test that the control thread alone writes the controller's decisions and that the sender
 * thread reads them as an immutable snapshot.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

// local includes
#include <src/adaptive_bitrate.h>

using namespace std::literals;

namespace {
  adaptive_bitrate::session_t make_session(adaptive_bitrate::mode_e mode) {
    return {mode, 20000, 60, 1024, 90, 300000.0};
  }

  std::vector<char> loss_stats(std::uint32_t lost, std::uint32_t last_good) {
    std::vector<char> out(32, 0);
    out[0] = char(lost);
    out[12] = char(last_good);
    return out;
  }

  // Frames through a clean path, as the control thread sees them; returns the time reached
  std::chrono::steady_clock::time_point feed(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int frames, int first = 0) {
    for (int i = first; i < first + frames; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 3 == 2) {
        auto msg = loss_stats(0, i + 1);
        session.loss_stats(sent + 30ms, {msg.data(), msg.size()});
      }
      session.tick(sent + 50ms, 0.060);
    }
    return start + std::chrono::microseconds((first + frames) * 16667);
  }
}  // namespace

// @ABR-17-S1
TEST(AbrStateOwnerTest, APublishedValueNeverChangesUnderAReader) {
  adaptive_bitrate::snapshot_t<int> cell {std::make_shared<const int>(1)};
  const auto held = cell.load();
  EXPECT_EQ(cell.load(), held) << "the same pointer until the owner publishes";
  cell.store(std::make_shared<const int>(2));
  EXPECT_EQ(*held, 1) << "what a reader holds does not change";
  EXPECT_EQ(*cell.load(), 2);
}

// @ABR-17-S1 @ABR-17-S2
TEST(AbrStateOwnerTest, TheSenderReadsOneSnapshotThatOnlyAStepReplaces) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  const auto before = session.decision();
  EXPECT_FALSE(before->stepped);
  EXPECT_EQ(before->target_fec, -1);
  EXPECT_EQ(session.decision(), before) << "reading does not publish";
  EXPECT_EQ(session.fec_percentage(20), 20);

  const auto start = std::chrono::steady_clock::now();
  feed(session, start, 180);
  const auto after = session.decision();
  EXPECT_NE(after, before);
  EXPECT_TRUE(after->stepped);
  EXPECT_FALSE(before->stepped) << "the snapshot a sender holds is not rewritten by the step";
  EXPECT_EQ(before->target_fec, -1);
  EXPECT_EQ(session.fec_percentage(20), std::clamp(after->target_fec, 1, 255));
  EXPECT_EQ(session.budget_bps(), after->budget);
  EXPECT_EQ(session.pacing_bps(), after->pacing);
  EXPECT_EQ(session.min_parity(8), std::min(8, std::max(0, after->min_parity)));
}

// @ABR-17-S3
TEST(AbrStateOwnerTest, ABlockDoesNotCopyTheParameters) {
  auto session = make_session(adaptive_bitrate::mode_e::fec);
  const auto start = std::chrono::steady_clock::now();
  const auto reached = feed(session, start, 180);
  const auto first = session.decision();
  ASSERT_TRUE(first->block_params);
  ASSERT_TRUE(first->repair_params);
  for (int k = 1; k <= 60; ++k) {
    session.block_fec_percentage(k);
    session.block_fec_percentage(k, true);
  }
  EXPECT_EQ(session.decision(), first) << "sizing blocks publishes and builds nothing";

  // A later step with the same failure target keeps the very same parameters
  feed(session, reached, 60, 180);
  const auto later = session.decision();
  if (later->block_params->parity_block_failure == first->block_params->parity_block_failure) {
    EXPECT_EQ(later->block_params, first->block_params);
  }
  EXPECT_EQ(later->repair_params, first->repair_params) << "the repair parameters are set once";
  EXPECT_DOUBLE_EQ(first->repair_params->parity_block_failure, first->repair_params->parity_repair_failure);
  EXPECT_GE(first->repair_params->fec_max, first->block_params->fec_max);
}

// @ABR-17-S1 @ABR-17-S4
TEST(AbrStateOwnerTest, TheSenderReadsWhileTheControlThreadSteps) {
  auto session = make_session(adaptive_bitrate::mode_e::full);
  std::atomic<bool> done {false};
  std::atomic<int> reads {0};
  std::atomic<int> incoherent {0};

  std::thread sender {[&]() {
    while (!done.load()) {
      const auto decision = session.decision();
      // Every field of one snapshot belongs to the same step
      if (decision->stepped != (decision->target_fec >= 0)) {
        ++incoherent;
      }
      if (decision->stepped && !decision->block_params) {
        ++incoherent;
      }
      session.fec_percentage(20);
      session.block_fec_percentage(40);
      session.frame_pacing_bps(std::chrono::steady_clock::now());
      session.initial_encoder_kbps();
      ++reads;
    }
  }};

  const auto start = std::chrono::steady_clock::now();
  feed(session, start, 300);
  done = true;
  sender.join();

  EXPECT_GT(reads.load(), 0);
  EXPECT_EQ(incoherent.load(), 0);
}
