/**
 * @file src/stream_internal.h
 * @brief Functions of src/stream.cpp that are not part of the stream's interface, declared so the
 * unit tests can reach them.
 */
#pragma once

// lib includes
#include <boost/asio.hpp>

// local includes
#include "audio.h"
#include "stream.h"

namespace stream {

  /**
   * @brief Audio bitrate on the wire assumed by the adaptive bitrate controller.
   */
  double estimated_audio_bps(const audio::config_t &config, int ip_header_bytes);

  /**
   * @brief Bytes each video packet costs on the wire beyond what the client counts.
   */
  int video_wire_overhead(const config_t &config);

  /**
   * @brief Set where a session's video goes and from which local address.
   *
   * @param session The session.
   * @param peer The client's video endpoint.
   * @param local The local address the packets leave from.
   */
  void set_video_route(session_t &session, const boost::asio::ip::udp::endpoint &peer, const boost::asio::ip::address &local);

  /**
   * @brief The broadcast video sender, which sends the packets of every session.
   *
   * @param sock Socket the packets are written to.
   */
  void videoBroadcastThread(boost::asio::ip::udp::socket &sock);

}  // namespace stream
