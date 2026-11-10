/**
 * @file tests/unit/test_stream_fec.cpp
 * @brief Test the Reed-Solomon video FEC in src/stream_fec.h.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <algorithm>
#include <cstdint>
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
    EXPECT_EQ(reed_solomon_encode(rs.get(), pointers.data(), static_cast<int>(shards.size()), block_size), 0);
    return {shards.begin() + data_shards, shards.end()};
  }

  /**
   * @brief Check the parity shards of an encoded block against the reference.
   */
  void expect_parity(stream::fec::fec_t &fec, const std::string &payload, size_t data_shards, size_t parity_shards) {
    ASSERT_EQ(fec.nr_shards, data_shards + parity_shards);
    const auto parity = reference_parity(payload, data_shards, parity_shards);
    for (size_t i = 0; i < parity_shards; ++i) {
      const auto *shard = reinterpret_cast<const uint8_t *>(fec.data(data_shards + i));
      EXPECT_TRUE(std::equal(parity[i].begin(), parity[i].end(), shard)) << "parity shard " << i;
    }
  }
}  // namespace

TEST(StreamFecTest, ComputesParityForThePercentageRequested) {
  const auto payload = make_payload(10);
  auto fec = stream::fec::encode(payload, block_size, 20, 0, 0);

  EXPECT_EQ(fec.data_shards, 10U);
  EXPECT_EQ(fec.percentage, 20U);
  expect_parity(fec, payload, 10, 2);
}

TEST(StreamFecTest, PadsTheLastDataShard) {
  auto payload = make_payload(10);
  payload.resize(payload.size() - block_size / 2);
  auto padded = payload;
  padded.resize(10 * block_size, '\0');

  auto fec = stream::fec::encode(payload, block_size, 20, 0, 4);

  EXPECT_EQ(fec.data_shards, 10U);
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

TEST(StreamFecTest, CapsTheMinimumParityToTheBlockLimit) {
  const auto payload = make_payload(200);
  auto fec = stream::fec::encode(payload, block_size, 10, 100, 0);

  EXPECT_EQ(fec.nr_shards, stream::fec::kMaxShards);
  EXPECT_EQ(fec.percentage, 27U);
  expect_parity(fec, payload, 200, 55);
}

TEST(StreamFecTest, SendsNoParityWhenTheDataFillsTheBlock) {
  const auto payload = make_payload(255);
  auto fec = stream::fec::encode(payload, block_size, 5, 20, 0);

  EXPECT_EQ(fec.nr_shards, 255U);
  EXPECT_EQ(fec.percentage, 0U);
}

TEST(StreamFecTest, SendsNoParityThatCouldNotBeComputed) {
  const auto payload = make_payload(300);
  auto fec = stream::fec::encode(payload, block_size, 10, 0, 0);

  EXPECT_EQ(fec.data_shards, 300U);
  EXPECT_EQ(fec.nr_shards, 300U);
  EXPECT_EQ(fec.percentage, 0U);
}

TEST(StreamFecTest, SendsNoParityWhenTheRequestedParityOverflowsTheBlock) {
  const auto payload = make_payload(250);
  auto fec = stream::fec::encode(payload, block_size, 20, 0, 0);

  EXPECT_EQ(fec.nr_shards, 250U);
  EXPECT_EQ(fec.percentage, 0U);
}

TEST(StreamFecTest, WithoutFecLeavesTheDataAlone) {
  const auto payload = make_payload(10);
  auto fec = stream::fec::encode(payload, block_size, 0, 8, 0);

  EXPECT_EQ(fec.nr_shards, 10U);
  EXPECT_EQ(fec.percentage, 0U);
}
