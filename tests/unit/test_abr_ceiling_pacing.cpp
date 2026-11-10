/**
 * @file tests/unit/test_abr_ceiling_pacing.cpp
 * @brief Test the sender at the application's ceiling: a frame is paced and paid at twice the
 * budget, below it nothing changes.
 */
// test includes
#include "../tests_common.h"

// local includes
#include <src/stream_pacing.h>

// @ABR-21-S2
TEST(AbrCeilingPacingTest, AtTheCeilingTheFrameIsPacedAtTwiceTheBudget) {
  EXPECT_TRUE(stream::at_ceiling(19.8e6, 20e6));
  EXPECT_FALSE(stream::at_ceiling(19.7e6, 20e6));
  EXPECT_FALSE(stream::at_ceiling(20e6, 0)) << "no ceiling, no ceiling pacing";
  // gamma 1.3 and a capacity of 1.2 times the budget give 1.56; at the ceiling the multiple is 2
  EXPECT_NEAR(stream::pace_multiplier(1.3, 10e6, 12e6, 2.0, 2.0, 20e6), 1.56, 1e-9);
  EXPECT_DOUBLE_EQ(stream::pace_multiplier(1.3, 20e6, 20e6, 2.0, 2.0, 20e6), 2.0);
  EXPECT_NEAR(stream::pace_multiplier(1.3, 20e6, 20e6, 2.0, 0, 20e6), 1.3, 1e-9) << "without a ceiling multiple the usual one";
}

// @ABR-21-S2
TEST(AbrCeilingPacingTest, AtTheCeilingTheFrameIsPaidAtTwiceTheBudget) {
  EXPECT_DOUBLE_EQ(stream::pay_multiple(10e6, 20e6, 2.0), 1.1);
  EXPECT_DOUBLE_EQ(stream::pay_multiple(20e6, 20e6, 2.0), 2.0);
}
