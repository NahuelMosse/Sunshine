/**
 * @file src/stream_pacing.h
 * @brief Declarations for when the packets of a video frame are due.
 */
#pragma once

// standard includes
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <ratio>

namespace stream {

  /**
   * @brief How the video sender spreads the packets of a frame over time.
   */
  struct pacing_t {
    std::size_t packets_in_1ms = 1;  ///< Packets sent in a millisecond, at least one.

    /**
     * @brief How long after the start of the frame the given number of its packets is due.
     *
     * @param packets_sent Packets of the frame already sent.
     * @return The offset from the start of the frame.
     */
    [[nodiscard]] std::chrono::nanoseconds due(std::size_t packets_sent) const {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(1)) * packets_sent / std::max<std::size_t>(packets_in_1ms, 1);
    }
  };

  /**
   * @brief The pacing of the packets of a frame: around 80 % of 1 Gbit/s.
   *
   * @param blocksize Bytes in each packet.
   * @return The pacing for packets of that size.
   */
  inline pacing_t pacing(std::size_t blocksize) {
    pacing_t result;
    // 1 Gbit/s, percent, ms, packet, byte; a packet larger than the rate allows in a millisecond is paced one per millisecond
    result.packets_in_1ms = std::max<std::size_t>(std::giga::num * 80 / 100 / 1000 / blocksize / 8, 1);
    return result;
  }

}  // namespace stream
