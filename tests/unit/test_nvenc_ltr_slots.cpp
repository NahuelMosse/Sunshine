/**
 * @file tests/unit/test_nvenc_ltr_slots.cpp
 * @brief Test the long-term reference slots and the client acknowledgement of a repair reference.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <array>
#include <cstdint>

// local includes
#include <src/nvenc/nvenc_ltr.h>

using nvenc::ltr_slot_t;
using nvenc::repair_source_e;

namespace {
  // Slots 0 and 1 hold frames 300 and 330, both valid
  constexpr std::array<ltr_slot_t, 2> held {{{300, true}, {330, true}}};
}  // namespace

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

TEST(NvencLtrAckTest, WithoutAnAcknowledgementTheRepairUsesTheShortTermReferences) {
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
  // HEVC pictures well under the level's MaxLumaPs hold more frames: up to a quarter of it 24 (at
  // most 16), up to three quarters 8 (Table A.8 at level 3.1)
  EXPECT_EQ(nvenc::hevc_level_dpb_frames(640, 360, 30), 16u);
  EXPECT_EQ(nvenc::hevc_level_dpb_frames(1024, 640, 30), 8u);
  // H.264 with no picture holds the most a DPB can; one beyond every level gets what the largest
  // level's MaxDpbMbs leaves it
  EXPECT_EQ(nvenc::h264_level_dpb_frames(0, 720, 60), 16u);
  EXPECT_EQ(nvenc::h264_level_dpb_frames(8192, 8192, 30), 2u);
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

TEST(NvencLtrAckTest, TheLayoutKeepsTwoLongTermAndUpToThreeShortTermReferences) {
  // HEVC at 1080p60 would allow 6 frames: 2 long-term references and 3 short-term ones are kept
  auto layout = nvenc::repair_references(true, true, 1, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ltr_slots, 2);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
}

TEST(NvencLtrAckTest, WithoutAcknowledgementsTheCodecsDpbIsKept) {
  auto layout = nvenc::repair_references(false, true, 0, 1920, 1080, 60, 5, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 5u);
  EXPECT_EQ(layout.ltr_slots, 0);
  layout = nvenc::repair_references(false, true, 2, 1920, 1080, 60, 8, 8);
  EXPECT_EQ(layout.ref_frames_in_dpb, 8u);
  EXPECT_EQ(layout.ltr_slots, 0);
}

TEST(NvencLtrSlotsTest, LongTermMarksRotateEveryThirtyFramesOverTheSlots) {
  using nvenc::ltr_mark_slot;
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

  // None without acknowledgements or without reference frame invalidation
  EXPECT_EQ(ltr_slots(false, true, 16, 8), 0);
  EXPECT_EQ(ltr_slots(true, false, 16, 8), 0);
}
