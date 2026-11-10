/**
 * @file src/nvenc/nvenc_ltr.h
 * @brief Declarations for the long-term reference slots NVENC keeps for repairs from a
 * reference the client acknowledges.
 */
#pragma once

// standard includes
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace nvenc {

  /**
   * @brief Frames between long-term reference marks: n slots in turn reach back n − 1 to n
   * periods, so that a repair covering most of them still finds a slot marked before the loss.
   */
  inline constexpr std::uint64_t ltr_period = 30;

  /**
   * @brief The most long-term reference slots an encoder keeps.
   */
  inline constexpr int ltr_slots_kept = 4;

  /**
   * @brief How many long-term reference slots a session keeps.
   * @param repairs Whether the client acknowledges long-term references.
   * @param rfi Whether the client invalidates reference frames.
   * @param ref_frames_in_dpb Reference frames in the decoded picture buffer.
   * @param gpu_max The GPU's NV_ENC_CAPS_NUM_MAX_LTR_FRAMES.
   * @return At most four and a quarter of the DPB (four of H.264's and HEVC's sixteen, two of
   * AV1's eight); 0 without repairs or RFI, with a DPB under 8 or when fewer than two fit.
   */
  constexpr int ltr_slots(bool repairs, bool rfi, std::uint32_t ref_frames_in_dpb, int gpu_max) {
    const int slots = std::min({ltr_slots_kept, gpu_max, static_cast<int>(ref_frames_in_dpb / 4)});
    if (!repairs || !rfi || ref_frames_in_dpb < 8 || slots < 2) {
      return 0;
    }
    return slots;
  }

  /**
   * @brief The slot a frame is marked into as a long-term reference.
   * @param frame_index The frame's index.
   * @param slots The session's slots (ltr_slots).
   * @param in_use The slot the frame predicts from, -1 for none: it keeps its mark.
   * @return The slot, or -1 when the frame is not marked: one mark every ltr_period frames, the
   * slots in turn.
   */
  constexpr int ltr_mark_slot(std::uint64_t frame_index, std::uint32_t slots, int in_use) {
    if (slots == 0 || frame_index % ltr_period != 0) {
      return -1;
    }
    const auto slot = static_cast<int>((frame_index / ltr_period) % slots);
    return slot == in_use ? -1 : slot;
  }

  /**
   * @brief How far back a repair still finds a long-term reference marked before the loss.
   * @param slots The session's slots (ltr_slots).
   * @return Frames covered by every slot but the newest mark, 0 with fewer than two slots.
   */
  constexpr std::uint32_t ltr_repair_reach(std::uint32_t slots) {
    return slots > 1 ? static_cast<std::uint32_t>((slots - 1) * ltr_period) : 0;
  }

  /**
   * @brief The extraFlags bit of a video packet whose frame is a long-term reference
   * (NV_VIDEO_PACKET_EXTRA_FLAG_LTR_FRAME, moonlight-common-c #122).
   */
  inline constexpr std::uint8_t ltr_extra_flag = 0x1;

  /**
   * @brief Bytes of the client's long-term reference acknowledgement (SS_LTR_FRAME_ACK: the frame
   * index and a reserved word, little-endian).
   */
  inline constexpr std::size_t ltr_ack_size = 8;

  /**
   * @brief Long-term references the encoder keeps when the client acknowledges them.
   */
  inline constexpr int acked_ltr_slots = 2;

  /**
   * @brief The most short-term references beside the acknowledged long-term ones.
   */
  inline constexpr std::uint32_t acked_short_term_refs = 3;

  /**
   * @brief A long-term reference slot.
   */
  struct ltr_slot_t {
    std::uint64_t frame_index = 0;  ///< Frame marked into the slot.
    bool valid = false;  ///< The encoder can still predict from it: no repair or IDR dropped it.
  };

  /**
   * @brief Where a repair predicts its next frame from.
   */
  enum class repair_source_e {
    acked_ltr,  ///< The long-term reference the client acknowledged.
    short_term,  ///< The last short-term reference before the loss.
    blind_ltr,  ///< The newest long-term reference marked before the loss, never acknowledged.
    keyframe,  ///< Nothing left to predict from.
  };

  /**
   * @brief A repair's source and the slot it predicts from.
   */
  struct repair_t {
    repair_source_e source;  ///< Where the next frame predicts from.
    int slot;  ///< The long-term reference slot, -1 for the short-term reference or a keyframe.
  };

  /**
   * @brief The references an encoder opens with.
   */
  struct reference_layout_t {
    std::uint32_t ref_frames_in_dpb;  ///< Reference frames in the decoded picture buffer.
    int ltr_slots;  ///< Long-term reference slots among them, 0 for none.
  };

  /**
   * @brief The extraFlags of a frame's video packets.
   * @param ltr_marked Whether the encoder marked the frame as a long-term reference.
   * @return ltr_extra_flag for a marked frame, 0 for any other.
   */
  constexpr std::uint8_t video_extra_flags(bool ltr_marked) {
    return ltr_marked ? ltr_extra_flag : 0;
  }

  /**
   * @brief Reads the client's long-term reference acknowledgement (0x0350).
   * @param payload The message's payload.
   * @return The acknowledged frame's 32-bit index, or nothing for a payload shorter than
   * ltr_ack_size.
   */
  constexpr std::optional<std::uint32_t> parse_ltr_ack(std::string_view payload) {
    if (payload.size() < ltr_ack_size) {
      return std::nullopt;
    }
    std::uint32_t frame = 0;
    for (std::size_t i = 0; i < 4; ++i) {
      frame |= static_cast<std::uint32_t>(static_cast<unsigned char>(payload[i])) << (8 * i);
    }
    return frame;
  }

  /**
   * @brief The acknowledgement a session keeps after the client acknowledges a frame.
   * @param slots The encoder's long-term reference slots.
   * @param kept The frame index kept so far, -1 for none.
   * @param acked The acknowledged frame's 32-bit index.
   * @return The acknowledged frame when a valid slot holds it (same low 32 bits) and it is newer
   * than the one kept; the one kept otherwise (out of order or stale).
   */
  constexpr std::int64_t kept_ltr_ack(std::span<const ltr_slot_t> slots, std::int64_t kept, std::uint32_t acked) {
    for (const auto &slot : slots) {
      if (slot.valid && static_cast<std::uint32_t>(slot.frame_index) == acked) {
        const auto frame = static_cast<std::int64_t>(slot.frame_index);
        return frame > kept ? frame : kept;
      }
    }
    return kept;
  }

  /**
   * @brief Where a repair of the lost frames predicts from.
   * @param slots The encoder's long-term reference slots.
   * @param kept The acknowledged frame index kept, -1 for none.
   * @param first_frame The first lost frame.
   * @param last_frame The last frame encoded.
   * @param short_term Short-term references in the DPB.
   * @return The acknowledged long-term reference while a valid slot holds it before the loss;
   * otherwise the short-term reference when the loss fits in them, else the newest valid long-term
   * reference before the loss, else a keyframe.
   */
  constexpr repair_t repair_source(std::span<const ltr_slot_t> slots, std::int64_t kept, std::uint64_t first_frame, std::uint64_t last_frame, std::uint64_t short_term) {
    int newest = -1;
    for (int i = 0; i < static_cast<int>(slots.size()); ++i) {
      const auto &slot = slots[i];
      if (!slot.valid || slot.frame_index >= first_frame) {
        continue;
      }
      if (kept >= 0 && slot.frame_index == static_cast<std::uint64_t>(kept)) {
        return {repair_source_e::acked_ltr, i};
      }
      if (newest < 0 || slot.frame_index > slots[newest].frame_index) {
        newest = i;
      }
    }
    if (last_frame - first_frame + 1 < short_term) {
      return {repair_source_e::short_term, -1};
    }
    if (newest >= 0) {
      return {repair_source_e::blind_ltr, newest};
    }
    return {repair_source_e::keyframe, -1};
  }

  /**
   * @brief The H.264 DPB frames the lowest level a stream needs allows (Table A-1).
   * @param width Frame width.
   * @param height Frame height.
   * @param fps Frames per second.
   * @return MaxDpbMbs over the frame's macroblocks at the level MaxFS and MaxMBPS pick, at most 16.
   */
  constexpr std::uint32_t h264_level_dpb_frames(std::uint32_t width, std::uint32_t height, std::uint32_t fps) {
    struct level_t {
      std::uint64_t max_mbps;
      std::uint64_t max_fs;
      std::uint64_t max_dpb_mbs;
    };

    // Levels 3.1 to 6.2; 4 and 4.1 share their limits
    constexpr std::array<level_t, 10> levels {{
      {108000, 3600, 18000},
      {216000, 5120, 20480},
      {245760, 8192, 32768},
      {522240, 8704, 34816},
      {589824, 22080, 110400},
      {983040, 36864, 184320},
      {2073600, 36864, 184320},
      {4177920, 139264, 696320},
      {8355840, 139264, 696320},
      {16711680, 139264, 696320},
    }};
    const std::uint64_t frame_mbs = std::uint64_t {(width + 15) / 16} * ((height + 15) / 16);
    if (frame_mbs == 0) {
      return 16;
    }
    for (const auto &level : levels) {
      if (frame_mbs <= level.max_fs && frame_mbs * fps <= level.max_mbps) {
        return static_cast<std::uint32_t>(std::min<std::uint64_t>(16, level.max_dpb_mbs / frame_mbs));
      }
    }
    return static_cast<std::uint32_t>(std::min<std::uint64_t>(16, levels.back().max_dpb_mbs / frame_mbs));
  }

  /**
   * @brief The HEVC DPB frames the lowest level a stream needs allows (Table A.8).
   * @param width Frame width.
   * @param height Frame height.
   * @param fps Frames per second.
   * @return maxDpbSize at the level MaxLumaPs and MaxLumaSr pick, at most 16.
   */
  constexpr std::uint32_t hevc_level_dpb_frames(std::uint32_t width, std::uint32_t height, std::uint32_t fps) {
    struct level_t {
      std::uint64_t max_luma_ps;
      std::uint64_t max_luma_sr;
    };

    // Levels 3.1 to 6.2
    constexpr std::array<level_t, 9> levels {{
      {983040, 33177600},
      {2228224, 66846720},
      {2228224, 133693440},
      {8912896, 267386880},
      {8912896, 534773760},
      {8912896, 1069547520},
      {35651584, 1069547520},
      {35651584, 2139095040},
      {35651584, 4278190080},
    }};
    constexpr std::uint64_t max_dpb_pic_buf = 6;
    const std::uint64_t picture = std::uint64_t {width} * height;
    auto level = levels.back();
    for (const auto &candidate : levels) {
      if (picture <= candidate.max_luma_ps && picture * fps <= candidate.max_luma_sr) {
        level = candidate;
        break;
      }
    }
    std::uint64_t frames = max_dpb_pic_buf;
    if (picture <= level.max_luma_ps >> 2) {
      frames = max_dpb_pic_buf * 4;
    } else if (picture <= level.max_luma_ps >> 1) {
      frames = max_dpb_pic_buf * 2;
    } else if (picture <= (3 * level.max_luma_ps) >> 2) {
      frames = max_dpb_pic_buf * 4 / 3;
    }
    return static_cast<std::uint32_t>(std::min<std::uint64_t>(16, frames));
  }

  /**
   * @brief The references an NVENC session opens with.
   * @param repairs Whether the client acknowledges the long-term references it decodes.
   * @param rfi Whether the client invalidates reference frames.
   * @param video_format 0 H.264, 1 HEVC, 2 AV1.
   * @param width Frame width.
   * @param height Frame height.
   * @param fps Frames per second.
   * @param default_count The codec's DPB without repairs.
   * @param gpu_max The GPU's NV_ENC_CAPS_NUM_MAX_LTR_FRAMES.
   * @return Without repairs, default_count and no long-term references; with them, acked_ltr_slots
   * plus up to acked_short_term_refs within the level's DPB, or the level's DPB and none when RFI,
   * the GPU or the level leave no room for them.
   */
  constexpr reference_layout_t repair_references(bool repairs, bool rfi, int video_format, std::uint32_t width, std::uint32_t height, std::uint32_t fps, std::uint32_t default_count, int gpu_max) {
    if (!repairs) {
      return {default_count, 0};
    }
    std::uint32_t level_frames = default_count;
    if (video_format == 0) {
      level_frames = h264_level_dpb_frames(width, height, fps);
    } else if (video_format == 1) {
      level_frames = hevc_level_dpb_frames(width, height, fps);
    }
    if (!rfi || gpu_max < acked_ltr_slots || level_frames < static_cast<std::uint32_t>(acked_ltr_slots) + 1) {
      return {level_frames, 0};
    }
    return {std::min(level_frames, static_cast<std::uint32_t>(acked_ltr_slots) + acked_short_term_refs), acked_ltr_slots};
  }

}  // namespace nvenc
