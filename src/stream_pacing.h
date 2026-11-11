/**
 * @file src/stream_pacing.h
 * @brief Declarations for when a frame's video packets are due, at the adaptive bitrate
 * controller's pacing and budget.
 */
#pragma once

// standard includes
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ratio>

// lib includes
#include <boost/asio/ip/address.hpp>

namespace stream {

  namespace fec {
    /**
     * @brief Limit data shards so data and parity fit in an 8-bit Reed-Solomon block.
     *
     * @param percentage Recovery-shard percentage between 0 and 100.
     * @return Maximum data shards with room for the requested parity shards.
     */
    int max_data_shards(int percentage);
  }  // namespace fec

  /**
   * @brief How the video sender spreads a frame's packets over time.
   */
  struct pacing_t {
    bool adaptive = false;  ///< Paced at the adaptive controller's rate instead of upstream's 80 % of 1 Gbit/s.
    std::chrono::nanoseconds packet_time {};  ///< Time a packet takes at the adaptive pacing rate.
    std::size_t packets_in_1ms = 1;  ///< Packets sent in a millisecond, at least one.

    /**
     * @brief How long after the start of the frame the given number of its packets is due.
     * @param packets_sent Packets of the frame already sent.
     * @return The offset from the frame's start.
     */
    [[nodiscard]] std::chrono::nanoseconds due(std::size_t packets_sent) const {
      if (adaptive) {
        return packet_time * packets_sent;
      }
      return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(1)) * packets_sent / packets_in_1ms;
    }

    /**
     * @brief The packets sent in one batch: no more than a millisecond's worth at the adaptive
     * pacing rate, or the pacing is lost inside the batch.
     * @param batch_size The batch upstream allows.
     * @return The batch size to use.
     */
    [[nodiscard]] std::size_t batch_size(std::size_t batch_size) const {
      return adaptive ? std::min(batch_size, packets_in_1ms) : batch_size;
    }
  };

  /**
   * @brief Bytes of IP and UDP headers on each packet to a client.
   * @param address The client's address.
   * @return 28 for IPv4 (an IPv4-mapped IPv6 address is IPv4), 48 for IPv6.
   */
  inline int ip_udp_header_bytes(const boost::asio::ip::address &address) {
    return address.is_v6() && !address.to_v6().is_v4_mapped() ? 48 : 28;
  }

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
   * A frame that needs more at the wanted percentage goes in four. Upstream's stream sends them with no
   * parity; with `keep_parity` the percentage is lowered to what fits next to the data of the largest
   * block instead, and parity is dropped only when the data alone fills it.
   *
   * @param payload_bytes Size of the frame's payload, a whole number of packets.
   * @param blocksize Bytes of a packet.
   * @param percentage The wanted parity percentage.
   * @param keep_parity Whether a frame over four blocks keeps the parity that fits, as the adaptive controller sends it.
   * @return The blocks and the percentage.
   */
  inline fec_plan_t fec_plan(std::size_t payload_bytes, std::size_t blocksize, int percentage, bool keep_parity) {
    constexpr std::size_t max_shards = 255;
    constexpr int max_blocks = 4;
    const auto max_data = static_cast<std::size_t>(fec::max_data_shards(std::max(percentage, 0)));
    const std::size_t blocks_needed = (payload_bytes + max_data * blocksize - 1) / (max_data * blocksize);
    if (blocks_needed <= max_blocks) {
      return {static_cast<int>(std::max<std::size_t>(blocks_needed, 1)), percentage};
    }
    if (!keep_parity) {
      return {max_blocks, 0};
    }
    const std::size_t aligned = ((payload_bytes / max_blocks + blocksize - 1) / blocksize) * blocksize;
    const std::size_t data = aligned / blocksize;
    const int fits = data >= max_shards ? 0 : static_cast<int>((max_shards - data) * 100 / data);
    return {max_blocks, std::min(percentage, fits)};
  }

  /**
   * @brief The pacing of a frame's packets.
   * @param blocksize Bytes of a packet's payload.
   * @param pacing_bps The adaptive controller's pacing rate, 0 without one.
   * @return Upstream's 80 % of 1 Gbit/s, or the controller's rate when it is lower.
   */
  inline pacing_t pacing(std::size_t blocksize, double pacing_bps) {
    pacing_t result;
    // Around 80 % of 1 Gbit/s: 1 Gbit/s, percent, ms, packet, byte
    result.packets_in_1ms = std::giga::num * 80 / 100 / 1000 / blocksize / 8;
    if (pacing_bps > 0 && pacing_bps < std::giga::num * 0.8) {
      result.adaptive = true;
      result.packet_time = std::chrono::nanoseconds(static_cast<std::int64_t>(static_cast<double>(blocksize) * 8 * 1e9 / pacing_bps));
      result.packets_in_1ms = std::max<std::int64_t>(1, std::chrono::nanoseconds(std::chrono::milliseconds(1)) / result.packet_time);
    }
    return result;
  }

  /**
   * @brief Whether the budget sits at the application's ceiling, where the loop is not limited by the pipe.
   * @param budget_bps The controller's budget.
   * @param bitrate_max_bps The stream's bitrate ceiling, 0 when there is none.
   * @return True from 0.99 of the ceiling up.
   */
  inline bool at_ceiling(double budget_bps, double bitrate_max_bps) {
    return bitrate_max_bps > 0 && budget_bps >= 0.99 * bitrate_max_bps;
  }

  /**
   * @brief The multiple of the budget a frame is paid at.
   *
   * 10 % over the budget, so frames of the budget's size never wait on the ones before. At the
   * ceiling a frame is paid at `ceiling_multiple` when that is more: a payment clock slower than
   * the link only adds wait there.
   * @param budget_bps The controller's budget.
   * @param bitrate_max_bps The stream's bitrate ceiling.
   * @param ceiling_multiple The multiple at the ceiling.
   * @return The multiple.
   */
  inline double pay_multiple(double budget_bps, double bitrate_max_bps, double ceiling_multiple) {
    return at_ceiling(budget_bps, bitrate_max_bps) ? std::max(1.1, ceiling_multiple) : 1.1;
  }

  /**
   * @brief When a frame is paid for at the adaptive controller's budget.
   * @param start When the frame's packets started to leave.
   * @param wire_bytes Bytes the frame took on the wire, headers and parity included.
   * @param budget_bps The controller's budget, 0 without one.
   * @param multiple The multiple of the budget the frame is paid at (pay_multiple).
   * @return The time the next frame waits for (frame_send_timing), or `start` without a budget.
   */
  inline std::chrono::steady_clock::time_point paid_until(std::chrono::steady_clock::time_point start, double wire_bytes, double budget_bps, double multiple = 1.1) {
    if (budget_bps <= 0) {
      return start;
    }
    return start + std::chrono::nanoseconds(static_cast<std::int64_t>(wire_bytes * 8 * 1e9 / (multiple * budget_bps)));
  }

  /**
   * @brief The multiple of the budget a frame's packets are paced at.
   * @param gamma The pacer's multiple of the capacity estimate, 0 for the fixed multiple.
   * @param budget_bps The controller's budget.
   * @param capacity_bps The controller's capacity estimate, 0 before it has one.
   * @param fixed The multiple used without a capacity estimate.
   * @param ceiling_multiple The multiple while the budget sits at the ceiling, 0 for none (the sender caps it at 2).
   * @param bitrate_max_bps The stream's bitrate ceiling, 0 when there is none.
   * @return The multiple.
   */
  inline double pace_multiplier(double gamma, double budget_bps, double capacity_bps, double fixed, double ceiling_multiple = 0, double bitrate_max_bps = 0) {
    double multiple = fixed;
    if (gamma > 0 && budget_bps > 0 && capacity_bps > 0) {
      // rho = gamma / u, with u = budget / capacity: a frame never reaches the bottleneck much
      // faster than the pipe the controller believes in
      multiple = std::clamp(gamma * capacity_bps / budget_bps, 1.1, 2.0);
    }
    if (ceiling_multiple > 0 && at_ceiling(budget_bps, bitrate_max_bps)) {
      multiple = std::max(multiple, std::min(ceiling_multiple, 2.0));
    }
    return multiple;
  }

  /**
   * @brief Picks the frames paced at twice the budget, so the dispersion still sees above it.
   */
  struct pace_probe_t {
    /**
     * @brief The multiple a frame leaving now is paced at.
     * @param now When the frame leaves.
     * @param probe_s Seconds between probe frames, 0 for none.
     * @param multiple The multiple of the other frames.
     * @return 2 for a probe frame, else `multiple`.
     */
    double frame_multiplier(std::chrono::steady_clock::time_point now, double probe_s, double multiple) {
      if (probe_s <= 0) {
        return multiple;
      }
      if (started && now - last < std::chrono::duration<double>(probe_s)) {
        return multiple;
      }
      started = true;
      last = now;
      return 2.0;
    }

    std::chrono::steady_clock::time_point last {};  ///< When the last probe frame left.
    bool started = false;  ///< Whether a frame left already.
  };

}  // namespace stream
