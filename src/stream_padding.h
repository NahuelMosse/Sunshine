/**
 * @file src/stream_padding.h
 * @brief Declarations for padding a probe frame with extra FEC parity.
 *
 * A padded probe is an ordinary video frame that carries more parity than its blocks announce by default. The
 * client derives the parity count of a block from the FEC percentage in each packet header, so any per-block
 * percentage is accepted and the surplus parity is dropped without decoding: the stream protocol is unchanged.
 */
#pragma once

// standard includes
#include <algorithm>
#include <cstddef>
#include <vector>

namespace stream::padding {

  /// Data and parity shards together in one Reed-Solomon block.
  constexpr std::size_t MAX_BLOCK_SHARDS = 255;
  /// Largest parity percentage the 8-bit field of the packet header holds.
  constexpr std::size_t MAX_PERCENTAGE = 255;

  /**
   * @brief Parity shards of a block of `data_shards` shards at the given percentage: the formula the client uses.
   */
  inline std::size_t parity_of(std::size_t data_shards, std::size_t percentage) {
    return (data_shards * percentage + 99) / 100;
  }

  /**
   * @brief Smallest percentage whose parity reaches `parity` shards for a block of `data_shards` shards.
   */
  inline std::size_t percentage_for(std::size_t data_shards, std::size_t parity) {
    if (data_shards == 0 || parity == 0) {
      return 0;
    }
    return (100 * parity + data_shards - 100) / data_shards;
  }

  /**
   * @brief Most parity shards a block of `data_shards` shards can carry: the room left in the Reed-Solomon block and
   *   the 8-bit percentage.
   */
  inline std::size_t parity_limit(std::size_t data_shards) {
    const std::size_t room = MAX_BLOCK_SHARDS > data_shards ? MAX_BLOCK_SHARDS - data_shards : 0;
    return std::min(room, parity_of(data_shards, MAX_PERCENTAGE));
  }

  /**
   * @brief What a padded probe frame is made of.
   */
  struct plan_t {
    std::vector<std::size_t> percentages;  ///< Parity percentage to announce in each block.
    std::size_t extra_parity = 0;  ///< Parity shards added over the ones the unpadded blocks carry.
    bool padded = false;  ///< False if the frame goes out as it is.
  };

  /**
   * @brief Spread extra parity over the blocks of a frame so that it reaches `target_packets` packets in all.
   *
   * The parity goes round-robin to the blocks up to what each can carry, so the frame's size is bounded by the
   * blocks it already has: a block holds at most 255 shards and its percentage at most 255, which is a ratio
   * and does not grow with more blocks (a frame carries up to 4). The frame is not padded unless it can reach
   * `min_packets` packets, since a probe that stays small crosses a router's burst bucket whatever its pace.
   *
   * @param data_shards Data shards in each block of the frame.
   * @param percentage The percentage the blocks carry without padding.
   * @param min_parity Fewest parity shards a block with FEC carries (the encoder raises the percentage to meet it).
   * @param target_packets Packets the padded frame should have, data and parity.
   * @param min_packets Fewest packets the padded frame must be able to reach.
   * @return The percentage of each block, and whether the frame was padded.
   */
  inline plan_t plan(const std::vector<std::size_t> &data_shards, std::size_t percentage, std::size_t min_parity, std::size_t target_packets, std::size_t min_packets) {
    plan_t result;
    std::vector<std::size_t> base;
    std::vector<std::size_t> room;
    std::size_t total = 0;
    std::size_t reachable = 0;
    for (const auto k : data_shards) {
      const auto limit = parity_limit(k);
      auto parity = parity_of(k, std::min(percentage, MAX_PERCENTAGE));
      if (parity < min_parity && percentage != 0) {
        parity = parity_of(k, percentage_for(k, std::min(min_parity, limit)));
      }
      parity = std::min(parity, limit);
      base.push_back(parity);
      room.push_back(limit - parity);
      total += k + parity;
      reachable += k + limit;
      result.percentages.push_back(percentage);
    }
    if (data_shards.empty() || percentage == 0 || target_packets <= total || std::min(reachable, target_packets) < min_packets) {
      return result;
    }
    auto wanted = target_packets - total;
    std::vector<std::size_t> added(data_shards.size(), 0);
    for (bool moved = true; wanted > 0 && moved;) {
      moved = false;
      for (std::size_t b = 0; b < added.size() && wanted > 0; ++b) {
        if (added[b] < room[b]) {
          ++added[b];
          --wanted;
          moved = true;
        }
      }
    }
    for (std::size_t b = 0; b < added.size(); ++b) {
      if (added[b] > 0) {
        result.percentages[b] = percentage_for(data_shards[b], base[b] + added[b]);
        result.extra_parity += added[b];
      }
    }
    result.padded = result.extra_parity > 0;
    return result;
  }

  /**
   * @brief When the next padded probe may go out.
   *
   * Padded probes add load of their own, so they are limited to a share of the budget on average: the next one
   * waits until the bits of the last would have been paid for at that share. A chain of rounds also leaves a
   * minimum gap between its probes.
   */
  struct gate_t {
    double last_s = -1e9;  ///< When the last padded probe left, in seconds.
    double last_bits = 0;  ///< Extra bits it carried on the wire.

    /**
     * @param now_s The current time, in seconds.
     * @param budget_bps The rate budget, in bits/s.
     * @param gap_s Least time since the last padded probe; 0 for none.
     * @param duty Share of the budget the padded probes may add on average; 0 for no limit.
     */
    [[nodiscard]] bool due(double now_s, double budget_bps, double gap_s, double duty) const {
      const double since = now_s - last_s;
      return since >= gap_s && (duty <= 0 || since >= last_bits / (duty * std::max(1.0, budget_bps)));
    }

    void sent(double now_s, double extra_bits) {
      last_s = now_s;
      last_bits = extra_bits;
    }
  };

}  // namespace stream::padding
