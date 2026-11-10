/**
 * @file tests/unit/test_abr_quiet_proof.cpp
 * @brief Test when a frame that crossed without spreading out may raise the budget: in steady
 * state, after a second without a queue, and only when it shows room above the budget.
 */
// test includes
#include "../tests_common.h"

// local includes
#include <src/abr/control.h>
#include <src/abr/params.h>

// @ABR-10-S12 @ABR-10-ASM-03
TEST(AbrQuietProofTest, TheD2Parameters) {
  const abr::Params p;
  EXPECT_DOUBLE_EQ(abr::cal::kQuietFrameProof, 1.0);
  EXPECT_DOUBLE_EQ(abr::cal::kQuietFrameMinPackets, 12);
  EXPECT_DOUBLE_EQ(abr::cal::kQuietFrameProofQueue, 0.5);
  EXPECT_DOUBLE_EQ(abr::cal::kQuietFrameProofIdleS, 1.0) << "a second without a queue";
  EXPECT_DOUBLE_EQ(abr::tun::kPaceOverPipe, 1.3) << "30 % above the budget";
}

// @ABR-10-S12 @ABR-10-ASM-03
TEST(AbrQuietProofTest, AProofCountsOnlyQuietAndAboveTheMargin) {
  const abr::Params p;
  const double dref = 0.020;
  const double budget = 10e6;
  EXPECT_TRUE(abr::quiet_frame_proof_room(p, 0.4 * dref, dref, 1.1, 1.31 * budget, budget, abr::tun::kPaceOverPipe));
  EXPECT_FALSE(abr::quiet_frame_proof_room(p, 0.6 * dref, dref, 1.1, 1.31 * budget, budget, abr::tun::kPaceOverPipe)) << "a queue over half the target";
  EXPECT_FALSE(abr::quiet_frame_proof_room(p, 0.4 * dref, dref, 0.9, 1.31 * budget, budget, abr::tun::kPaceOverPipe)) << "less than a second since the last queue";
  EXPECT_FALSE(abr::quiet_frame_proof_room(p, 0.4 * dref, dref, 1.1, 1.29 * budget, budget, abr::tun::kPaceOverPipe)) << "the budget is already near the pipe";
  EXPECT_TRUE(abr::quiet_frame_proof_room(p, 0.4 * dref, dref, 1.1, 1.01 * budget, budget, 0));
}
