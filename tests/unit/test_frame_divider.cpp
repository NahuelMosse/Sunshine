/**
 * @file tests/unit/test_frame_divider.cpp
 * @brief Test that the frame divider encodes the last skipped image when the capture times out and survives a reopened encoder.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

// local includes
#include <src/config.h>
#include <src/frame_divider.h>

using namespace std::literals;

namespace {
  std::shared_ptr<platf::img_t> make_image() {
    return std::make_shared<platf::img_t>();
  }
}  // namespace

// @ABR-04-S1 @ABR-04-S2 @ABR-04-S3 @ABR-04-S4
TEST(FrameDividerTest, TheSkippedImageIsEncodedWhenTheCaptureTimesOut) {
  // A divider of 2: one captured image in two is encoded
  video::frame_divider_t divider;
  divider.set_frames_kept(0.5);

  const auto first = make_image();
  const auto on_screen = make_image();
  EXPECT_EQ(divider.captured(first, false), nullptr);
  EXPECT_EQ(divider.captured(on_screen, false), on_screen);

  // A static desktop: a keystroke produces a single captured image, and the divider skips it
  const auto keystroke = make_image();
  EXPECT_EQ(divider.captured(keystroke, false), nullptr);

  // Nothing else is captured: the capture times out and the skipped image is the one encoded
  EXPECT_EQ(divider.timed_out(), keystroke);
  // It was sent: the next timeout re-encodes it, as with any image already on screen
  EXPECT_EQ(divider.timed_out(), nullptr);
}

// @ABR-04-ASM-01
TEST(FrameDividerTest, OnlyTheNewestSkippedImageIsHeld) {
  // A divider of 3: two captured images in a row are skipped
  video::frame_divider_t divider;
  divider.set_frames_kept(1.0 / 3);

  const auto older = make_image();
  const auto newer = make_image();
  EXPECT_EQ(divider.captured(older, false), nullptr);
  EXPECT_EQ(divider.captured(newer, false), nullptr);

  EXPECT_EQ(divider.timed_out(), newer);
  EXPECT_EQ(divider.timed_out(), nullptr);
}

// @ABR-04-ASM-01
TEST(FrameDividerTest, AnEncodedImageDropsTheHeldOne) {
  video::frame_divider_t divider;
  divider.set_frames_kept(0.5);

  // A newer image is encoded: the skipped one is older than what the client shows
  const auto skipped = make_image();
  const auto kept = make_image();
  EXPECT_EQ(divider.captured(skipped, false), nullptr);
  EXPECT_EQ(divider.captured(kept, false), kept);
  EXPECT_EQ(divider.timed_out(), nullptr);

  // An IDR frame is never skipped, and it too leaves nothing older to send
  const auto skipped_before_idr = make_image();
  const auto idr = make_image();
  EXPECT_EQ(divider.captured(skipped_before_idr, false), nullptr);
  EXPECT_EQ(divider.captured(idr, true), idr);
  EXPECT_EQ(divider.timed_out(), nullptr);
}

// @ABR-04-ASM-02
TEST(FrameDividerTest, TheHeldImageDoesNotCountAsAKeptFrame) {
  video::frame_divider_t divider;
  divider.set_frames_kept(0.5);

  const auto skipped = make_image();
  EXPECT_EQ(divider.captured(skipped, false), nullptr);
  EXPECT_EQ(divider.timed_out(), skipped);

  // Half a frame was owed before the timeout and half is added now: this image is encoded
  const auto next = make_image();
  EXPECT_EQ(divider.captured(next, false), next);
}

// @ABR-08-S12 @ABR-08-S13 @ABR-08-S14 @ABR-08-ASM-05
TEST(FrameDividerTest, AReopenedEncoderKeepsTheShareAndDropsTheHeldImage) {
  video::frame_divider_t divider;
  divider.set_frames_kept(0.5);
  const auto held = std::make_shared<platf::img_t>();
  EXPECT_EQ(divider.captured(held, false), nullptr);

  divider.reopened();
  EXPECT_EQ(divider.timed_out(), nullptr) << "the image held for the old encoder is not encoded by the new one";

  // One captured image in two, as before the restart
  const auto first = std::make_shared<platf::img_t>();
  const auto second = std::make_shared<platf::img_t>();
  EXPECT_EQ(divider.captured(first, false), nullptr);
  EXPECT_EQ(divider.captured(second, false), second);
}
