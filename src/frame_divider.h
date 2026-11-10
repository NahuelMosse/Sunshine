/**
 * @file src/frame_divider.h
 * @brief Declarations for the frame divider's choice of which captured images to encode.
 */
#pragma once

// standard includes
#include <algorithm>
#include <memory>
#include <utility>

// local includes
#include "platform/common.h"

namespace video {

  /**
   * @brief Chooses which captured images the encode loop encodes when a share of them is
   * requested.
   *
   * Used by the encode thread only.
   */
  class frame_divider_t {
  public:
    /**
     * @brief Set the share of captured images to encode.
     * @param share The share, from the frame_divider mail; clamped to [0.001, 1].
     */
    void set_frames_kept(double share) {
      frames_kept = std::clamp(share, 0.001, 1.0);
    }

    /**
     * @brief Decide on an image the capture delivered.
     * @param img The captured image.
     * @param idr Whether an IDR frame was requested for this pass; its image is never skipped.
     * @return The image to encode now, or null when the divider skips it; a skipped image is held
     * until the capture times out, so a static screen does not keep the image before it.
     */
    std::shared_ptr<platf::img_t> captured(std::shared_ptr<platf::img_t> img, bool idr) {
      if (!idr && frames_kept < 1.0) {
        frames_owed += frames_kept;
        if (frames_owed < 1.0) {
          skipped = std::move(img);
          return nullptr;
        }
        frames_owed -= 1.0;
      }
      skipped.reset();
      return img;
    }

    /**
     * @brief Decide what to encode when the capture times out.
     *
     * The held image does not count against the share of captured images to encode.
     * @return The last image skipped since the last one encoded, or null to encode the image
     * already converted again.
     */
    std::shared_ptr<platf::img_t> timed_out() {
      return std::exchange(skipped, nullptr);
    }

    /**
     * @brief A new encoder took over from the one that stopped.
     *
     * The share of images to encode stays; the image held for the old encoder is dropped, and the
     * count starts over.
     */
    void reopened() {
      frames_owed = 0.0;
      skipped.reset();
    }

  private:
    double frames_kept = 1.0;  ///< The share of captured images to encode.
    double frames_owed = 0.0;  ///< What has accumulated of that share since the last image encoded.
    /// The last image skipped since the last one encoded; it stays out of the capture's pool until then.
    std::shared_ptr<platf::img_t> skipped;
  };

}  // namespace video
