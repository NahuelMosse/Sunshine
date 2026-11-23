/**
 * @file tests/unit/test_stream_fec.cpp
 * @brief Test the Reed-Solomon video FEC in src/stream_fec.h.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

// local includes
#include <src/stream_fec.h>

namespace {
  constexpr size_t block_size = 16;

  /**
   * @brief Build a payload of `shards` blocks of distinct, non-zero bytes.
   */
  std::string make_payload(size_t shards) {
    std::string payload(shards * block_size, '\0');
    for (size_t i = 0; i < payload.size(); ++i) {
      payload[i] = static_cast<char>(1 + (i * 7 + i / block_size) % 251);
    }
    return payload;
  }

  /**
   * @brief Parity of the payload computed straight from the Reed-Solomon library.
   */
  std::vector<std::vector<uint8_t>> reference_parity(const std::string &payload, size_t data_shards, size_t parity_shards) {
    std::vector<std::vector<uint8_t>> shards(data_shards + parity_shards, std::vector<uint8_t>(block_size));
    std::vector<uint8_t *> pointers;
    for (size_t i = 0; i < shards.size(); ++i) {
      if (i < data_shards) {
        std::copy_n(payload.begin() + i * block_size, block_size, shards[i].begin());
      }
      pointers.push_back(shards[i].data());
    }
    stream::fec::rs_t rs {reed_solomon_new(static_cast<int>(data_shards), static_cast<int>(parity_shards))};
    EXPECT_EQ(reed_solomon_encode(rs.get(), pointers.data(), static_cast<int>(shards.size()), static_cast<int>(block_size)), 0);
    return {shards.begin() + data_shards, shards.end()};
  }

  /**
   * @brief Check an encoded block: the parity count a client derives from the percentage in the
   * packet header is the one sent, and the parity shards are the library's.
   */
  void expect_parity(stream::fec::fec_t &fec, const std::string &payload, size_t data_shards, size_t parity_shards) {
    ASSERT_EQ(fec.data_shards, data_shards);
    ASSERT_EQ(fec.nr_shards, data_shards + parity_shards);
    EXPECT_LE(fec.percentage, 255U) << "the header field is 8 bits wide";
    EXPECT_EQ(fec.nr_shards - fec.data_shards, (fec.data_shards * fec.percentage + 99) / 100) << "the client derives the parity count from the percentage";
    const auto parity = reference_parity(payload, data_shards, parity_shards);
    for (size_t i = 0; i < parity_shards; ++i) {
      const auto *shard = reinterpret_cast<const uint8_t *>(fec.data(data_shards + i));
      EXPECT_TRUE(std::equal(parity[i].begin(), parity[i].end(), shard)) << "parity shard " << i;
    }
  }

  /**
   * @brief Check a block that goes out without parity.
   */
  void expect_no_parity(const stream::fec::fec_t &fec, size_t data_shards) {
    EXPECT_EQ(fec.data_shards, data_shards);
    EXPECT_EQ(fec.nr_shards, data_shards);
    EXPECT_EQ(fec.percentage, 0U);
  }
}  // namespace

TEST(StreamFecTest, ComputesParityForThePercentageRequested) {
  const auto payload = make_payload(10);
  auto fec = stream::fec::encode(payload, block_size, 20, 0, 0);

  EXPECT_EQ(fec.percentage, 20U);
  expect_parity(fec, payload, 10, 2);
}

TEST(StreamFecTest, RoundsTheParityCountUp) {
  const auto payload = make_payload(7);
  auto fec = stream::fec::encode(payload, block_size, 20, 0, 0);

  EXPECT_EQ(fec.percentage, 20U);
  expect_parity(fec, payload, 7, 2);
}

TEST(StreamFecTest, PadsTheLastDataShard) {
  auto payload = make_payload(10);
  payload.resize(payload.size() - block_size / 2);
  auto padded = payload;
  padded.resize(10 * block_size, '\0');

  auto fec = stream::fec::encode(payload, block_size, 20, 0, 4);

  EXPECT_NE(fec.prefix(0), nullptr);
  EXPECT_EQ(fec.size(), 12U);
  expect_parity(fec, padded, 10, 2);
}

TEST(StreamFecTest, RaisesTheParityToTheMinimum) {
  const auto payload = make_payload(10);
  auto fec = stream::fec::encode(payload, block_size, 10, 4, 0);

  EXPECT_EQ(fec.percentage, 40U);
  expect_parity(fec, payload, 10, 4);
}

TEST(StreamFecTest, RaisesThePercentageSoTheClientCountsTheMinimum) {
  // 51 parity shards for 200 data shards is 25.5 %: a client reading 25 % would count 50
  const auto payload = make_payload(200);
  auto fec = stream::fec::encode(payload, block_size, 10, 51, 0);

  EXPECT_EQ(fec.percentage, 26U);
  expect_parity(fec, payload, 200, 52);
}

TEST(StreamFecTest, CapsTheMinimumParityToTheBlockLimit) {
  // The 55 parity shards that fit next to 200 data shards are 27.5 %: at 28 % a client would count 56
  const auto payload = make_payload(200);
  auto fec = stream::fec::encode(payload, block_size, 10, 100, 0);

  EXPECT_LE(fec.nr_shards, RS8_DATA_SHARDS_MAX);
  EXPECT_EQ(fec.percentage, 27U);
  expect_parity(fec, payload, 200, 54);
}

TEST(StreamFecTest, CapsThePercentageToTheBlockLimit) {
  const auto payload = make_payload(250);
  auto fec = stream::fec::encode(payload, block_size, 20, 0, 0);

  EXPECT_EQ(fec.nr_shards, RS8_DATA_SHARDS_MAX);
  EXPECT_EQ(fec.percentage, 2U);
  expect_parity(fec, payload, 250, 5);
}

TEST(StreamFecTest, CapsThePercentageToTheHeaderField) {
  const auto payload = make_payload(2);
  auto fec = stream::fec::encode(payload, block_size, 1000, 0, 0);

  EXPECT_EQ(fec.percentage, 255U);
  expect_parity(fec, payload, 2, 6);
}

TEST(StreamFecTest, CapsAMinimumLargerThanTheBlock) {
  const auto payload = make_payload(10);
  auto fec = stream::fec::encode(payload, block_size, 10, std::numeric_limits<size_t>::max(), 0);

  EXPECT_EQ(fec.percentage, 255U);
  expect_parity(fec, payload, 10, 26);
}

TEST(StreamFecTest, SendsNoParityWhenTheDataFillsTheBlock) {
  const auto payload = make_payload(255);
  auto fec = stream::fec::encode(payload, block_size, 5, 20, 0);

  expect_no_parity(fec, 255);
}

TEST(StreamFecTest, SendsNoParityThatCouldNotBeComputed) {
  const auto payload = make_payload(300);
  auto fec = stream::fec::encode(payload, block_size, 10, 0, 0);

  expect_no_parity(fec, 300);
}

TEST(StreamFecTest, SendsNoParityWhenTheRoomLeftRoundsToZeroPercent) {
  // 253 data shards leave room for 2 parity shards, 0 % of the block: the client would read none
  const auto payload = make_payload(253);
  auto fec = stream::fec::encode(payload, block_size, 1, 20, 0);

  expect_no_parity(fec, 253);
  for (size_t i = 0; i < fec.size(); ++i) {
    EXPECT_NE(fec.data(i), nullptr) << "shard " << i;
  }
}

TEST(StreamFecTest, WithoutFecLeavesTheDataAlone) {
  const auto payload = make_payload(10);
  auto fec = stream::fec::encode(payload, block_size, 0, 8, 0);

  expect_no_parity(fec, 10);
}

TEST(StreamFecPlanTest, SplitsAFrameInTheBlocksThatFitTheShardLimit) {
  constexpr std::size_t blocksize = 1400;

  auto plan = stream::fec::plan_blocks(100 * blocksize, blocksize, 20);
  EXPECT_EQ(plan.blocks, 1U);
  EXPECT_EQ(plan.percentage, 20);

  plan = stream::fec::plan_blocks(600 * blocksize, blocksize, 20);
  EXPECT_EQ(plan.blocks, 3U) << "212 data shards fit a block at 20 %";
  EXPECT_EQ(plan.blocks_needed, 3U);
  EXPECT_EQ(plan.percentage, 20);
}

TEST(StreamFecPlanTest, SendsAFrameOverTheBlockLimitWithoutParity) {
  constexpr std::size_t blocksize = 1400;

  auto plan = stream::fec::plan_blocks(900 * blocksize, blocksize, 20);
  EXPECT_EQ(plan.blocks, static_cast<std::size_t>(stream::fec::MAX_FEC_BLOCKS));
  EXPECT_EQ(plan.blocks_needed, 5U);
  EXPECT_EQ(plan.percentage, 0);

  plan = stream::fec::plan_blocks(900 * blocksize, blocksize, 0);
  EXPECT_EQ(plan.blocks, static_cast<std::size_t>(stream::fec::MAX_FEC_BLOCKS));
  EXPECT_EQ(plan.percentage, 0);
}
