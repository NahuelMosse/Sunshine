/**
 * @file src/nvenc/nvenc_encoder.h
 * @brief Declarations for the SDK-neutral NVENC encoder interface.
 */
#pragma once

// standard includes
#include <cstdint>

// local includes
#include "nvenc_config.h"
#include "nvenc_encoded_frame.h"

namespace platf {
  enum class pix_fmt_e;
}

namespace video {
  struct config_t;
  struct sunshine_colorspace_t;
}  // namespace video

namespace nvenc {

  /**
   * @brief SDK-neutral standalone NVENC encoder interface.
   */
  class nvenc_encoder {
  public:
    /**
     * @brief Destroy the SDK-neutral encoder interface.
     */
    virtual ~nvenc_encoder() = default;

    /**
     * @brief Create the encoder.
     *
     * @param config NVENC encoder configuration.
     * @param client_config Stream configuration requested by the client.
     * @param colorspace Sunshine colorspace metadata.
     * @param buffer_format Platform-agnostic input surface format.
     * @return `true` on success, `false` on error.
     */
    virtual bool create_encoder(
      const nvenc_config &config,
      const video::config_t &client_config,
      const video::sunshine_colorspace_t &colorspace,
      platf::pix_fmt_e buffer_format
    ) = 0;

    /**
     * @brief Destroy the encoder.
     */
    virtual void destroy_encoder() = 0;

    /**
     * @brief Encode the next frame using the platform-specific input surface.
     *
     * @param frame_index Frame index that uniquely identifies the frame.
     * @param force_idr Whether to encode the frame as a forced IDR.
     * @return Encoded frame.
     */
    virtual nvenc_encoded_frame encode_frame(std::uint64_t frame_index, bool force_idr) = 0;

    /**
     * @brief Invalidate reference frames in the requested range.
     *
     * @param first_frame First frame index of the invalidation range.
     * @param last_frame Last frame index of the invalidation range.
     * @return `true` on success, `false` on error.
     */
    virtual bool invalidate_ref_frames(std::uint64_t first_frame, std::uint64_t last_frame) = 0;

    /**
     * @brief Take the client's acknowledgement of a long-term reference frame (0x0350); an
     * encoder without long-term references ignores it.
     *
     * @param frame_index The acknowledged frame's 32-bit index.
     */
    virtual void acknowledge_ltr([[maybe_unused]] std::uint32_t frame_index) {
    }

    /**
     * @brief Change the target bitrate without recreating the encoder.
     *
     * @param kbps New target bitrate in kilobits per second.
     * @return `true` if the encoder accepted it, `false` if it cannot change bitrate on the fly.
     */
    virtual bool set_bitrate(std::uint32_t kbps) {
      return false;
    }

    /**
     * @brief How many frames back a loss can be repaired from a long-term reference.
     *
     * @return Frames, or 0 without long-term references.
     */
    virtual std::uint32_t repair_reach_frames() const {
      return 0;
    }

    /**
     * @brief The target bitrate the encoder runs at.
     *
     * @return Kilobits per second: the one it was opened with until a change succeeds, or 0 without an encoder.
     */
    virtual std::uint32_t bitrate_kbps() const {
      return 0;
    }
  };

}  // namespace nvenc
