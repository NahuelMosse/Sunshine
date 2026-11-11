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
#include "sync.h"
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

  /**
   * @brief Apply the bitrate the adaptive bitrate controller asked for, and say once when the encoder cannot change it.
   */
  void apply_requested_bitrate(encode_session_t &session, safe::mail_raw_t::event_t<int> &events, safe::mail_raw_t::event_t<int> &fixed_events, bool &fixed_reported);

}  // namespace video
