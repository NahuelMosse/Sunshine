/**
 * @file tests/unit/test_stream_padding.cpp
 * @brief Test the extra FEC parity of a padded probe frame, in src/stream_padding.h.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <cstddef>
#include <numeric>
#include <vector>

// local includes
#include <src/stream_padding.h>

namespace pad = stream::padding;

TEST(StreamPaddingTest, ThePercentageForAParityCountReachesItAndNoLess) {
  for (std::size_t k = 1; k <= 200; ++k) {
    for (std::size_t parity = 1; parity <= pad::parity_limit(k); ++parity) {
      const auto pct = pad::percentage_for(k, parity);
      ASSERT_LE(pct, pad::MAX_PERCENTAGE);
      EXPECT_GE(pad::parity_of(k, pct), parity) << k << " data shards, " << parity << " parity";
      EXPECT_LT(pad::parity_of(k, pct - 1), parity) << k << " data shards, " << parity << " parity";
    }
  }
}

TEST(StreamPaddingTest, ABlockCarriesAtMost255ShardsAndAPercentageOf255) {
  EXPECT_EQ(pad::parity_limit(8), 21U);
  EXPECT_EQ(pad::parity_limit(100), 155U);
  EXPECT_EQ(pad::parity_limit(255), 0U);
}

TEST(StreamPaddingTest, ExtraParityGoesToOneBlockUpToWhatItCanCarry) {
  const auto plan = pad::plan({30}, 20, 2, 100, 60);
  ASSERT_TRUE(plan.padded);
  EXPECT_EQ(plan.extra_parity, 100U - 36U);
  EXPECT_EQ(pad::parity_of(30, plan.percentages[0]), 70U);
}

TEST(StreamPaddingTest, AFrameThatCannotReachTheMinimumGoesOutAsItIs) {
  // 8 data shards: the 8-bit percentage caps the frame at 29 packets, 3.6 times its data
  const auto plan = pad::plan({8}, 20, 2, 60, 40);
  EXPECT_FALSE(plan.padded);
  EXPECT_EQ(plan.extra_parity, 0U);
  EXPECT_EQ(plan.percentages, (std::vector<std::size_t> {20}));
  const auto reachable = pad::plan({8}, 20, 2, 60, 29);
  ASSERT_TRUE(reachable.padded);
  EXPECT_EQ(pad::parity_of(8, reachable.percentages[0]), 21U);
}

TEST(StreamPaddingTest, ParitySpreadsOverTheBlocksOfAFrame) {
  const auto plan = pad::plan({100, 100, 60}, 20, 2, 360, 300);
  ASSERT_TRUE(plan.padded);
  std::size_t total = 260;
  const std::vector<std::size_t> data {100, 100, 60};
  for (std::size_t b = 0; b < data.size(); ++b) {
    const auto parity = pad::parity_of(data[b], plan.percentages[b]);
    EXPECT_LE(parity, pad::parity_limit(data[b]));
    EXPECT_LE(plan.percentages[b], pad::MAX_PERCENTAGE);
    total += parity;
  }
  EXPECT_GE(total, 360U);
  EXPECT_LE(total, 365U) << "and the percentage rounding adds only a shard or two per block";
}

TEST(StreamPaddingTest, NoFecMeansNoPadding) {
  EXPECT_FALSE(pad::plan({20}, 0, 2, 100, 10).padded);
  EXPECT_FALSE(pad::plan({}, 20, 2, 100, 10).padded);
  EXPECT_FALSE(pad::plan({20}, 20, 2, 20, 10).padded) << "already as big as the target";
}

TEST(StreamPaddingTest, PaddedProbesAddAShareOfTheBudgetOnAverage) {
  pad::gate_t gate;
  EXPECT_TRUE(gate.due(0.0, 10e6, 0.1, 0.2)) << "the first one";
  gate.sent(0.0, 480000);  // 0.48 Mbit at 20 % of 10 Mbit/s: 0.24 s
  EXPECT_FALSE(gate.due(0.2, 10e6, 0.1, 0.2));
  EXPECT_TRUE(gate.due(0.25, 10e6, 0.1, 0.2));
  EXPECT_FALSE(gate.due(0.05, 1e9, 0.1, 0.2)) << "the gap between the probes of a chain";
  EXPECT_TRUE(gate.due(0.05, 1e9, 0.0, 0.0)) << "no gap and no duty";
}
