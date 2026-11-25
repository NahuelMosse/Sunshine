/**
 * @file src/video_internal.h
 * @brief Functions of src/video.cpp that are not part of the video interface, declared so the unit
 * tests can reach them.
 */
#pragma once

// standard includes
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

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

  /**
   * @brief Encode one frame and queue its packet.
   *
   * @param frame_nr Frame index.
   * @param session The encode session.
   * @param packets Queue the packet goes to.
   * @param channel_data Stream session the packet is for.
   * @param frame_timestamp Capture time of the frame.
   * @return 0 when the frame is encoded and queued; nonzero on encoder failure.
   */
  int encode(int64_t frame_nr, encode_session_t &session, safe::mail_raw_t::queue_t<packet_t> &packets, stream::session_t *channel_data, std::optional<std::chrono::steady_clock::time_point> frame_timestamp);

}  // namespace video
