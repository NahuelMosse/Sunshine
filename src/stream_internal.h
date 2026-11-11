/**
 * @file src/stream_internal.h
 * @brief Functions of src/stream.cpp that are not part of the stream's interface, declared so the
 * unit tests can reach them.
 */
#pragma once

// standard includes
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

// lib includes
#include <boost/asio.hpp>

// local includes
#include "audio.h"
#include "platform/common.h"
#include "stream.h"
#include "video.h"

namespace stream {

  /**
   * @brief Audio bitrate on the wire assumed by the adaptive bitrate controller.
   */
  double estimated_audio_bps(const audio::config_t &config, int ip_header_bytes);

  /**
   * @brief When a frame starts to leave and the processing latency its header reports.
   *
   * @param now Current time.
   * @param next_frame_start When the previous frame's last pacing group ends.
   * @param captured When the frame was captured, if known.
   * @param budget_bps The controller's budget, 0 when it does not hold the session's frames.
   * @param paid_until When the frames sent so far are paid for at the budget.
   * @return The start, and the latency in 1/10 ms units when the capture time is known.
   */
  std::pair<std::chrono::steady_clock::time_point, std::optional<std::uint16_t>> frame_send_timing(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point next_frame_start,
    std::optional<std::chrono::steady_clock::time_point> captured,
    double budget_bps,
    std::chrono::steady_clock::time_point paid_until
  );

  /**
   * @brief The session's own video sender queue, starting its thread on first use.
   *
   * @param session The session.
   * @param sock Socket the sender writes to.
   * @param video_epoch Origin of the RTP timestamps.
   * @param make_timer Creates the timer that paces the sender.
   * @return The queue, or nullptr when the timer could not be created.
   */
  std::shared_ptr<safe::queue_t<video::packet_t>> own_sender_queue(session_t &session, boost::asio::ip::udp::socket &sock, std::chrono::steady_clock::time_point video_epoch, const std::function<std::unique_ptr<platf::high_precision_timer>()> &make_timer);

  /**
   * @brief Hand a frame to the session's own sender, when the controller paces the session on one.
   *
   * @param session The session the frame belongs to.
   * @param packet The frame; moved to the session's queue only when it is handed over.
   * @param sock Socket the session's own sender writes to.
   * @param video_epoch Origin of the RTP timestamps.
   * @param make_timer Creates the timer that paces the own sender.
   * @return True when the session's own sender has the frame, false when the calling thread sends it.
   */
  bool hand_off_to_own_sender(session_t &session, video::packet_t &packet, boost::asio::ip::udp::socket &sock, std::chrono::steady_clock::time_point video_epoch, const std::function<std::unique_ptr<platf::high_precision_timer>()> &make_timer);

  /**
   * @brief Stop the session's own sender and wait for its thread.
   */
  void close_own_sender(session_t &session);

  /**
   * @brief Whether the adaptive bitrate controller holds this session's frames to its budget and pacing.
   */
  bool controller_holds_frames(session_t &session);

  /**
   * @brief The rate at which the controller paces a session's frames.
   *
   * @param session The session.
   * @param holds Whether the controller holds the session's frames (controller_holds_frames).
   * @param now Current time.
   * @return Bits per second, or 0 for the stream's default pacing.
   */
  double controller_pacing_bps(session_t &session, bool holds, std::chrono::steady_clock::time_point now);

  /**
   * @brief Record what a sent frame owes the session's budget and report it to the controller.
   *
   * @param session The session the frame belongs to.
   * @param packet The frame.
   * @param frame_start When the frame's packets started to leave, by the pacing.
   * @param budget_bps The controller's budget, 0 without a holding controller.
   * @param first_send When the first packet actually left, if any did.
   * @param last_send When the last batch left.
   * @param packets Packets sent, parity included.
   * @param data_packets Data packets sent.
   * @param rtp_bytes RTP bytes sent.
   * @param wire_bytes Bytes on the wire, headers included.
   */
  void report_frame(session_t &session, video::packet_raw_t &packet, std::chrono::steady_clock::time_point frame_start, double budget_bps, std::optional<std::chrono::steady_clock::time_point> first_send, std::chrono::steady_clock::time_point last_send, int packets, int data_packets, double rtp_bytes, double wire_bytes);

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
   * @brief When the session's frames sent so far are paid for at the controller's budget.
   *
   * @param session The session.
   * @return The instant, as report_frame recorded it.
   */
  std::chrono::steady_clock::time_point video_paid_until(const session_t &session);

  /**
   * @brief The video sender: the broadcast thread when `own_packets` is null, a session's own sender otherwise.
   *
   * @param sock Socket the packets are written to.
   * @param video_epoch Origin of the RTP timestamps.
   * @param own_packets The session's own queue, or nullptr for the broadcast queue.
   * @param timer The own sender's timer; the broadcast thread makes its own.
   */
  void videoSendThread(boost::asio::ip::udp::socket &sock, std::chrono::steady_clock::time_point video_epoch, std::shared_ptr<safe::queue_t<video::packet_t>> own_packets, std::unique_ptr<platf::high_precision_timer> timer);

}  // namespace stream
