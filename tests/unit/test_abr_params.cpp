/**
 * @file tests/unit/test_abr_params.cpp
 * @brief Test src/abr/params.*
 */
#include "../tests_common.h"

#include <src/abr/control.h>
#include <src/abr/params.h>

TEST(AbrParamsTest, EveryReasonHasAName) {
  for (auto r : {abr::Reason::startup, abr::Reason::steady, abr::Reason::no_evidence, abr::Reason::queue_growing, abr::Reason::queue_high, abr::Reason::loss_backoff, abr::Reason::ramping, abr::Reason::reanchored, abr::Reason::capped}) {
    EXPECT_STRNE(abr::name(r), "?");
  }
  EXPECT_STREQ(abr::name(static_cast<abr::Reason>(99)), "?");
}

// @ABR-01-ASM-02
TEST(AbrParamsTest, DefaultsAreTheSourceOfTruths) {
  const abr::Params p;
  EXPECT_DOUBLE_EQ(p.train_utilization, 0.875);
  EXPECT_DOUBLE_EQ(p.startup_fraction, 0.5);
  EXPECT_DOUBLE_EQ(p.dref_noise, 1.0);
  EXPECT_DOUBLE_EQ(abr::cal::kBackoffLoss, 0.125);
  EXPECT_EQ(abr::cal::kNarrowerCycles, 1);
}
