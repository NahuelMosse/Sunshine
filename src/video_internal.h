/**
 * @file src/video_internal.h
 * @brief Functions of src/video.cpp that are not part of the video interface, declared so the unit
 * tests can reach them.
 */
#pragma once

// standard includes
#include <memory>

// local includes
#include "platform/common.h"
#include "video.h"

namespace video {

  /**
   * @brief Create the encode device for a display and an encoder.
   */
  std::unique_ptr<platf::encode_device_t> make_encode_device(platf::display_t &disp, const encoder_t &encoder, const config_t &config);

  /**
   * @brief Create an encode session on a device.
   */
  std::unique_ptr<encode_session_t> make_encode_session(platf::display_t *disp, const encoder_t &encoder, const config_t &config, int width, int height, std::unique_ptr<platf::encode_device_t> encode_device);

}  // namespace video
