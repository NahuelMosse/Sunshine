/**
 * @file tests/unit/test_nvenc_ltr_slots.cpp
 * @brief Test the long-term reference slots and the client acknowledgement of a repair reference.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

// lib includes
#include <boost/asio.hpp>

// local includes
#include <src/adaptive_bitrate.h>
#include <src/nvenc/nvenc_ltr.h>
#include <src/platform/common.h>
#include <src/rtsp.h>
#include <src/stream.h>
#include <src/stream_pacing.h>
#include <src/video.h>

using nvenc::ltr_slot_t;
using nvenc::repair_source_e;
using namespace std::literals;

namespace {
  std::string ack_bytes(std::uint32_t frame, std::size_t size) {
    std::string bytes(size, '\0');
    for (std::size_t i = 0; i < 4 && i < size; ++i) {
      bytes[i] = static_cast<char>((frame >> (8 * i)) & 0xFF);
    }
    return bytes;
  }

  // Slots 0 and 1 hold frames 300 and 330, both valid
  constexpr std::array<ltr_slot_t, 2> held {{{300, true}, {330, true}}};
}  // namespace

// @ABR-11-S6 @ABR-11-S7 @ABR-11-S17 @ABR-11-ASM-07
TEST(NvencLtrAckTest, TheLtrFlagIsSetOnMarkedFramesOnly) {
  EXPECT_EQ(nvenc::ltr_extra_flag, 0x1);
  int marked = 0;
  for (std::uint64_t frame = 0; frame < 300; ++frame) {
    const bool ltr = nvenc::ltr_mark_slot(frame, nvenc::acked_ltr_slots, -1) >= 0;
    marked += ltr;
    EXPECT_EQ(nvenc::video_extra_flags(ltr), ltr ? 0x1 : 0x0) << "frame " << frame;
  }
  // one mark every 30 frames, as before the acknowledgements
  EXPECT_EQ(marked, 10);
}

// @ABR-11-S8 @ABR-11-S18 @ABR-11-ASM-04
TEST(NvencLtrAckTest, TheAckPayloadIsParsed) {
  EXPECT_EQ(nvenc::ltr_ack_size, 8u);
  // valid: little-endian frame index, then the reserved word
  EXPECT_EQ(nvenc::parse_ltr_ack(ack_bytes(300, 8)), std::optional<std::uint32_t>(300));
  EXPECT_EQ(nvenc::parse_ltr_ack(ack_bytes(0x01020304, 8)), std::optional<std::uint32_t>(0x01020304));
  // the index is 32 bits wide
  EXPECT_EQ(nvenc::parse_ltr_ack(ack_bytes(0xFFFFFFFFu, 8)), std::optional<std::uint32_t>(0xFFFFFFFFu));
  // a longer payload is read from its first 4 bytes
  EXPECT_EQ(nvenc::parse_ltr_ack(ack_bytes(330, 12)), std::optional<std::uint32_t>(330));
  // malformed: shorter than SS_LTR_FRAME_ACK
  EXPECT_EQ(nvenc::parse_ltr_ack(ack_bytes(300, 7)), std::nullopt);
  EXPECT_EQ(nvenc::parse_ltr_ack(ack_bytes(300, 4)), std::nullopt);
  EXPECT_EQ(nvenc::parse_ltr_ack(std::string_view {}), std::nullopt);
}

// @ABR-11-S9 @ABR-11-S18 @ABR-11-ASM-05
TEST(NvencLtrAckTest, TheMostRecentHeldAckIsKept) {
  // most recent: each newer held frame replaces the one kept
  EXPECT_EQ(nvenc::kept_ltr_ack(held, -1, 300), 300);
  EXPECT_EQ(nvenc::kept_ltr_ack(held, 300, 330), 330);
  // out of order: an older acknowledgement arriving late changes nothing
  EXPECT_EQ(nvenc::kept_ltr_ack(held, 330, 300), 330);
  // stale: a frame no slot holds anymore, or a slot a repair or an IDR dropped
  EXPECT_EQ(nvenc::kept_ltr_ack(held, 330, 270), 330);
  EXPECT_EQ(nvenc::kept_ltr_ack(held, -1, 270), -1);
  constexpr std::array<ltr_slot_t, 2> dropped {{{300, false}, {330, true}}};
  EXPECT_EQ(nvenc::kept_ltr_ack(dropped, -1, 300), -1);
  // the 32-bit index names the slot with the same low 32 bits
  constexpr std::array<ltr_slot_t, 2> wrapped {{{0x100000005ULL, true}, {0x0FFFFFFF0ULL, true}}};
  EXPECT_EQ(nvenc::kept_ltr_ack(wrapped, 0x0FFFFFFF0LL, 5), 0x100000005LL);
}

// @ABR-11-S10 @ABR-11-S19 @ABR-11-ASM-06
TEST(NvencLtrAckTest, ARepairPredictsFromTheAckedLtrWithinReach) {
  // the acknowledged frame before the loss, whatever the loss's length
  auto repair = nvenc::repair_source(held, 300, 340, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::acked_ltr);
  EXPECT_EQ(repair.slot, 0);
  repair = nvenc::repair_source(held, 330, 349, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::acked_ltr);
  EXPECT_EQ(repair.slot, 1);
  repair = nvenc::repair_source(held, 330, 400, 480, 3);
  EXPECT_EQ(repair.source, repair_source_e::acked_ltr);
  EXPECT_EQ(repair.slot, 1);
}

// @ABR-11-S11 @ABR-11-S19
TEST(NvencLtrAckTest, WithoutAcksTheRepairFollowsTodaysChain) {
  // none acknowledged, a loss the short-term references cover
  auto repair = nvenc::repair_source(held, -1, 349, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::short_term);
  EXPECT_EQ(repair.slot, -1);
  // a longer loss: the newest long-term reference before it, blind
  repair = nvenc::repair_source(held, -1, 340, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::blind_ltr);
  EXPECT_EQ(repair.slot, 1);
  repair = nvenc::repair_source(held, -1, 320, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::blind_ltr);
  EXPECT_EQ(repair.slot, 0);
  // nothing marked before the loss: a keyframe
  repair = nvenc::repair_source(held, -1, 300, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::keyframe);
  EXPECT_EQ(repair.slot, -1);
}

// @ABR-11-S19 @ABR-11-ASM-06
TEST(NvencLtrAckTest, AnAckedLtrTooOldFallsBack) {
  // too old: its slot holds a later mark now
  auto repair = nvenc::repair_source(held, 270, 340, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::blind_ltr);
  EXPECT_EQ(repair.slot, 1);
  // dropped by a repair or an IDR
  constexpr std::array<ltr_slot_t, 2> dropped {{{300, true}, {330, false}}};
  repair = nvenc::repair_source(dropped, 330, 340, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::blind_ltr);
  EXPECT_EQ(repair.slot, 0);
  // not before the loss
  repair = nvenc::repair_source(held, 330, 320, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::blind_ltr);
  EXPECT_EQ(repair.slot, 0);
  repair = nvenc::repair_source(held, 330, 349, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::acked_ltr);
  repair = nvenc::repair_source(dropped, 330, 349, 350, 3);
  EXPECT_EQ(repair.source, repair_source_e::short_term);
}

// @ABR-11-S12 @ABR-11-S13 @ABR-11-S20 @ABR-11-ASM-03
TEST(NvencLtrAckTest, TheDpbFitsTheLevelPerCodecAndResolution) {
  // H.264, Table A-1: 1080p30 level 4 and 1080p60 level 4.2 allow 4 frames, 1080p120 level 5.1
  // 16; 720p60 level 3.2 5, 1440p60 level 5.1 12, 4K60 level 5.2 5
  EXPECT_EQ(nvenc::h264_level_dpb_frames(1920, 1080, 30), 4u);
  EXPECT_EQ(nvenc::h264_level_dpb_frames(1920, 1080, 60), 4u);
  EXPECT_EQ(nvenc::h264_level_dpb_frames(1920, 1080, 120), 16u);
  EXPECT_EQ(nvenc::h264_level_dpb_frames(1280, 720, 60), 5u);
  EXPECT_EQ(nvenc::h264_level_dpb_frames(2560, 1440, 60), 12u);
  EXPECT_EQ(nvenc::h264_level_dpb_frames(3840, 2160, 60), 5u);
  // HEVC, Table A.8: 1080p60 level 4.1 6, 720p60 level 4 12, 1440p60 level 5 12, 4K60 level 5.1 6
  EXPECT_EQ(nvenc::hevc_level_dpb_frames(1920, 1080, 60), 6u);
  EXPECT_EQ(nvenc::hevc_level_dpb_frames(1280, 720, 60), 12u);
  EXPECT_EQ(nvenc::hevc_level_dpb_frames(2560, 1440, 60), 12u);
  EXPECT_EQ(nvenc::hevc_level_dpb_frames(3840, 2160, 60), 6u);
  // 2 long-term references and up to 3 short-term ones within the level, per codec
  auto layout = nvenc::repair_references(true, true, 0, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 4u);
  EXPECT_EQ(layout.ltr_slots, 2);
  layout = nvenc::repair_references(true, true, 0, 3840, 2160, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  layout = nvenc::repair_references(true, true, 0, 2560, 1440, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  layout = nvenc::repair_references(true, true, 1, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  EXPECT_EQ(layout.ltr_slots, 2);
  layout = nvenc::repair_references(true, true, 1, 3840, 2160, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  layout = nvenc::repair_references(true, true, 2, 3840, 2160, 60, 8, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  EXPECT_EQ(layout.ltr_slots, 2);
  // a GPU with fewer than 2 long-term references, or no RFI: the level's DPB and none
  layout = nvenc::repair_references(true, true, 0, 1920, 1080, 60, 5, 1);
  EXPECT_EQ(layout.ref_frames_in_dpb, 4u);
  EXPECT_EQ(layout.ltr_slots, 0);
  layout = nvenc::repair_references(true, false, 1, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 6u);
  EXPECT_EQ(layout.ltr_slots, 0);
}

// @ABR-11-S15 @ABR-11-ASM-02
TEST(NvencLtrAckTest, TheAckedLayoutComesWithTheFeatureBit) {
  // with ABR-12's bit: 2 long-term references and 3 short-term ones, never 16
  auto layout = nvenc::repair_references(true, true, 1, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ltr_slots, nvenc::acked_ltr_slots);
  EXPECT_EQ(layout.ref_frames_in_dpb, nvenc::acked_ltr_slots + nvenc::acked_short_term_refs);
  EXPECT_EQ(nvenc::acked_ltr_slots, 2);
  EXPECT_EQ(nvenc::acked_short_term_refs, 3u);
  // without it the session is upstream's, acknowledged or not: the codec's DPB, no long-term ones
  layout = nvenc::repair_references(false, true, 0, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  EXPECT_EQ(layout.ltr_slots, 0);
  layout = nvenc::repair_references(false, true, 2, 1920, 1080, 60, 8, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 8u);
  EXPECT_EQ(layout.ltr_slots, 0);
}

// @ABR-06-S7 @ABR-06-ASM-04
TEST(NvencLtrSlotsTest, LongTermMarksRotateEveryThirtyFramesOverTheSlots) {
  using nvenc::ltr_mark_slot;
  EXPECT_EQ(nvenc::ltr_period, 30u);

  // Over 4 slots, one mark every 30 frames, in turn
  EXPECT_EQ(ltr_mark_slot(0, 4, -1), 0);
  EXPECT_EQ(ltr_mark_slot(30, 4, -1), 1);
  EXPECT_EQ(ltr_mark_slot(60, 4, -1), 2);
  EXPECT_EQ(ltr_mark_slot(90, 4, -1), 3);
  EXPECT_EQ(ltr_mark_slot(120, 4, -1), 0);
  EXPECT_EQ(ltr_mark_slot(150, 4, -1), 1);

  // Frames between marks are not marked
  EXPECT_EQ(ltr_mark_slot(1, 4, -1), -1);
  EXPECT_EQ(ltr_mark_slot(29, 4, -1), -1);
  EXPECT_EQ(ltr_mark_slot(31, 4, -1), -1);

  // The slot the frame predicts from keeps its mark
  EXPECT_EQ(ltr_mark_slot(60, 4, 2), -1);
  EXPECT_EQ(ltr_mark_slot(60, 4, 1), 2);

  // Over 2 slots
  EXPECT_EQ(ltr_mark_slot(30, 2, -1), 1);
  EXPECT_EQ(ltr_mark_slot(60, 2, -1), 0);

  // Without slots nothing is marked
  EXPECT_EQ(ltr_mark_slot(0, 0, -1), -1);

  // A frame lost that far back still has an older mark kept: every slot but the newest mark
  EXPECT_EQ(nvenc::ltr_repair_reach(4), 90u);
  EXPECT_EQ(nvenc::ltr_repair_reach(2), 30u);
  EXPECT_EQ(nvenc::ltr_repair_reach(1), 0u);
  EXPECT_EQ(nvenc::ltr_repair_reach(0), 0u);
}

// @ABR-06-S7 @ABR-06-ASM-04
TEST(NvencLtrSlotsTest, LongTermSlotsAreAtMostFourAndAQuarterOfTheDpb) {
  using nvenc::ltr_slots;
  // A quarter of the DPB: four of H.264's and HEVC's sixteen, two of AV1's eight
  EXPECT_EQ(ltr_slots(true, true, 16, 8), 4);
  EXPECT_EQ(ltr_slots(true, true, 8, 8), 2);
  EXPECT_EQ(ltr_slots(true, true, 12, 8), 3);

  // Never more than the 4 kept, nor than the GPU has
  EXPECT_EQ(ltr_slots(true, true, 32, 8), 4);
  EXPECT_EQ(ltr_slots(true, true, 16, 3), 3);

  // None under 2 slots or with a DPB under 8
  EXPECT_EQ(ltr_slots(true, true, 16, 1), 0);
  EXPECT_EQ(ltr_slots(true, true, 16, 0), 0);
  EXPECT_EQ(ltr_slots(true, true, 7, 8), 0);

  // None without the controller's repairs or without reference frame invalidation
  EXPECT_EQ(ltr_slots(false, true, 16, 8), 0);
  EXPECT_EQ(ltr_slots(true, false, 16, 8), 0);
}
