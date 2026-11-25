/**
 * @file src/stream_fec.h
 * @brief Declarations for the Reed-Solomon forward error correction of video frames.
 */
#pragma once

// standard includes
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

// lib includes
#include <rs.h>

// local includes
#include "platform/common.h"
#include "utility.h"

namespace stream::fec {
  /**
   * @brief Owning pointer for a Reed-Solomon encoder instance.
   */
  using rs_t = util::safe_ptr<reed_solomon, [](reed_solomon *rs) {
    reed_solomon_release(rs);
  }>;

  /**
   * @brief Reed-Solomon FEC encoder state for video packets.
   */
  struct fec_t {
    size_t data_shards;  ///< Number of original packet shards in each FEC block.
    size_t nr_shards;  ///< Total data and recovery shards generated for each FEC block.
    size_t percentage;  ///< Recovery-shard percentage requested for the stream.

    size_t blocksize;  ///< Bytes reserved for the payload portion of each shard.
    size_t prefixsize;  ///< Bytes reserved before each shard payload for protocol headers.
    util::buffer_t<char> shards;  ///< Contiguous backing storage for all encoded FEC shards.
    util::buffer_t<char> headers;  ///< Backing storage for the RTP/FEC headers attached to shards.
    util::buffer_t<uint8_t *> shards_p;  ///< Pointer table passed to the Reed-Solomon encoder.

    std::vector<platf::buffer_descriptor_t> payload_buffers;  ///< Platform send descriptors for FEC payload buffers.

    /**
     * @brief Return the FEC shard data pointer for a packet-group element.
     *
     * @param el Packet-group element index.
     * @return Pointer to the shard bytes for the requested element.
     */
    char *data(size_t el) {
      return (char *) shards_p[el];
    }

    /**
     * @brief Return the FEC prefix bytes for the current packet group.
     *
     * @param el Packet-group element index.
     * @return Pointer to the element's prefix bytes, or nullptr when no prefix is used.
     */
    char *prefix(size_t el) {
      return prefixsize ? &headers[el * prefixsize] : nullptr;
    }

    /**
     * @brief Return the serialized size of the current object.
     *
     * @return Number of elements currently stored.
     */
    size_t size() const {
      return nr_shards;
    }
  };

  /**
   * @brief Split a frame payload into data shards and compute its parity shards.
   *
   * The percentage is capped at 255, the most its 8-bit header field holds, and so that the data and
   * parity shards together do not exceed RS8_DATA_SHARDS_MAX. The parity shard count is always
   * `(data_shards * percentage + 99) / 100`, the formula the client uses to derive it, whether the
   * percentage was requested, raised to meet `minparityshards` or capped. A block whose parity cannot
   * be computed is returned without parity, with `percentage` and the shard count reflecting that.
   *
   * @param payload The frame data to protect.
   * @param blocksize Bytes in each shard.
   * @param fecpercentage Parity shards as a percentage of the data shards; 0 disables FEC.
   * @param minparityshards Fewest parity shards to produce when FEC is enabled.
   * @param prefixsize Bytes reserved before each shard for protocol headers.
   * @return The shards of the block and the percentage actually applied.
   */
  fec_t encode(const std::string_view &payload, size_t blocksize, size_t fecpercentage, size_t minparityshards, size_t prefixsize);

  /// Most FEC blocks a frame can be split in: the packet header has 2 bits for the block count.
  constexpr int MAX_FEC_BLOCKS = 4;

  /**
   * @brief Limit data shards so data and parity fit in an 8-bit Reed-Solomon block.
   *
   * @param percentage Recovery-shard percentage between 0 and 100.
   * @return Maximum data shards with room for the requested parity shards.
   */
  int max_data_shards(int percentage);

  /**
   * @brief How a frame is split in FEC blocks.
   */
  struct block_plan_t {
    std::size_t blocks_needed;  ///< Blocks the frame needs at the requested percentage.
    std::size_t blocks;  ///< Blocks the frame is split in, at most MAX_FEC_BLOCKS.
    int percentage;  ///< Parity percentage of each block: 0 when the frame needs more than MAX_FEC_BLOCKS.
  };

  /**
   * @brief Split a frame in the FEC blocks it needs.
   *
   * Each block holds at most max_data_shards() packets. A frame that needs more than MAX_FEC_BLOCKS
   * blocks goes in MAX_FEC_BLOCKS blocks without parity.
   *
   * @param payload_bytes Size of the frame's payload.
   * @param blocksize Bytes in each packet.
   * @param percentage The requested parity percentage.
   * @return The blocks and the percentage they carry.
   */
  inline block_plan_t plan_blocks(std::size_t payload_bytes, std::size_t blocksize, int percentage) {
    const auto max_data_bytes = static_cast<std::size_t>(std::max(max_data_shards(percentage), 1)) * blocksize;
    const auto blocks_needed = (payload_bytes + max_data_bytes - 1) / max_data_bytes;
    if (blocks_needed > MAX_FEC_BLOCKS) {
      return {blocks_needed, MAX_FEC_BLOCKS, 0};
    }
    return {blocks_needed, blocks_needed, percentage};
  }
}  // namespace stream::fec
