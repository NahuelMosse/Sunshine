/**
 * @file src/stream_fec.h
 * @brief Declarations for the Reed-Solomon forward error correction of video frames.
 */
#pragma once

// standard includes
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

  /// Most data and parity shards one Reed-Solomon block can hold together.
  constexpr size_t kMaxShards = DATA_SHARDS_MAX;

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
   * A block never holds more than kMaxShards shards: the parity count requested through
   * `minparityshards` is capped to what is left after the data shards. A block whose parity
   * cannot be computed is returned without parity, with `percentage` and the shard count
   * reflecting that, so no parity that was not computed is ever sent.
   *
   * @param payload The frame data to protect.
   * @param blocksize Bytes in each shard.
   * @param fecpercentage Parity shards as a percentage of the data shards; 0 disables FEC.
   * @param minparityshards Fewest parity shards to produce when FEC is enabled.
   * @param prefixsize Bytes reserved before each shard for protocol headers.
   * @return The shards of the block and the percentage actually applied.
   */
  fec_t encode(const std::string_view &payload, size_t blocksize, size_t fecpercentage, size_t minparityshards, size_t prefixsize);
}  // namespace stream::fec
