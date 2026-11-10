/**
 * @file src/stream_pacing.h
 * @brief Declarations for when a frame's video packets are due and how a frame is split in FEC
 * blocks.
 */
#pragma once

// standard includes
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ratio>

namespace stream {

  /**
   * @brief How the video sender spreads a frame's packets over time.
   */
  struct pacing_t {
    std::size_t packets_in_1ms = 1;  ///< Packets sent in a millisecond, at least one.

    /**
     * @brief How long after the start of the frame the given number of its packets is due.
     * @param packets_sent Packets of the frame already sent.
     * @return The offset from the frame's start.
     */
    [[nodiscard]] std::chrono::nanoseconds due(std::size_t packets_sent) const {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(1)) * packets_sent / packets_in_1ms;
    }
  };

  /**
   * @brief How a frame is split in FEC blocks.
   */
  struct fec_plan_t {
    int blocks = 1;  ///< FEC blocks of the frame.
    int percentage = 0;  ///< Parity percentage of each block.
  };

  /**
   * @brief The FEC blocks of a frame and the parity percentage they carry.
   *
   * A block holds at most 255 shards, data and parity, and the protocol counts at most four blocks.
   * A frame that needs more at the wanted percentage goes in four, with no parity.
   *
   * @param payload_bytes Size of the frame's payload, a whole number of packets.
   * @param blocksize Bytes of a packet.
   * @param percentage The wanted parity percentage.
   * @return The blocks and the percentage.
   */
  inline fec_plan_t fec_plan(std::size_t payload_bytes, std::size_t blocksize, int percentage) {
    constexpr std::size_t max_shards = 255;
    constexpr int max_blocks = 4;
    const std::size_t max_data = max_shards * 100 / (100 + static_cast<std::size_t>(std::max(percentage, 0)));
    const std::size_t blocks_needed = (payload_bytes + max_data * blocksize - 1) / (max_data * blocksize);
    if (blocks_needed <= max_blocks) {
      return {static_cast<int>(std::max<std::size_t>(blocks_needed, 1)), percentage};
    }
    return {max_blocks, 0};
  }

  /**
   * @brief The pacing of a frame's packets.
   * @param blocksize Bytes of a packet's payload.
   * @return Upstream's 80 % of 1 Gbit/s.
   */
  inline pacing_t pacing(std::size_t blocksize) {
    pacing_t result;
    // Around 80 % of 1 Gbit/s: 1 Gbit/s, percent, ms, packet, byte
    result.packets_in_1ms = std::giga::num * 80 / 100 / 1000 / blocksize / 8;
    return result;
  }

}  // namespace stream
