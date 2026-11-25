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
#include "frame_divider.h"
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

  /**
   * @brief Run the encode loop of one session until it stops or its encoder has to be reopened.
   *
   * @param frame_nr Index of the next frame; advanced as frames are encoded.
   * @param mail Mailbox of the session.
   * @param images Captured images.
   * @param config Stream configuration.
   * @param disp Display the images come from.
   * @param encode_device Device that converts and encodes the images.
   * @param reinit_event Signal raised while the encoder or display is reinitializing.
   * @param encoder The encoder in use.
   * @param channel_data Stream session each packet is for.
   * @param divider The share of captured images to encode.
   */
  void encode_run(int &frame_nr, safe::mail_t mail, img_event_t images, config_t config, std::shared_ptr<platf::display_t> disp, std::unique_ptr<platf::encode_device_t> encode_device, safe::signal_t &reinit_event, const encoder_t &encoder, stream::session_t *channel_data, frame_divider_t &divider);

}  // namespace video
