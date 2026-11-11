/**
 * @file tests/unit/test_stream_sender.cpp
 * @brief Test the video sender of a session: hand-off of frames to its own sender, pacing, hold, closing and reopening.
 */
// test includes
#include "../tests_common.h"
#include "../tests_log_checker.h"

// standard includes
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// lib includes
#include <boost/asio.hpp>

// local includes
#include <moonlight-common-c/src/Limelight-internal.h>
#include <src/adaptive_bitrate.h>
#include <src/audio.h>
#include <src/config.h>
#include <src/display_device.h>
#include <src/globals.h>
#include <src/nvenc/nvenc_config.h>
#include <src/nvenc/nvenc_encoder.h>
#include <src/nvenc/nvenc_ltr.h>
#include <src/platform/common.h>
#include <src/rtsp.h>
#include <src/stream.h>
#include <src/stream_internal.h>
#include <src/stream_pacing.h>
#include <src/thread_safe.h>
#include <src/video.h>
#include <src/video_internal.h>

using namespace std::literals;

namespace {
  constexpr int packet_size = 1024;

  /**
   * @brief Sets the adaptive bitrate mode for as long as it lives.
   */
  struct scoped_mode_t {
    explicit scoped_mode_t(std::string mode):
        saved {config::stream.adaptive_bitrate} {
      config::stream.adaptive_bitrate = std::move(mode);
    }

    ~scoped_mode_t() {
      config::stream.adaptive_bitrate = saved;
    }

    scoped_mode_t(const scoped_mode_t &) = delete;
    scoped_mode_t &operator=(const scoped_mode_t &) = delete;

    std::string saved;
  };

  /**
   * @brief An encoded frame of a given size, for a session.
   */
  struct test_packet_t: video::packet_raw_t {
    test_packet_t(stream::session_t *session, int64_t index, std::size_t size, bool idr):
        bytes(size, 0x5a),
        index {index},
        idr {idr} {
      channel_data = session;
    }

    bool is_idr() override {
      return idr;
    }

    int64_t frame_index() override {
      return index;
    }

    uint8_t *data() override {
      return bytes.data();
    }

    size_t data_size() override {
      return bytes.size();
    }

    std::vector<uint8_t> bytes;
    int64_t index;
    bool idr;
  };

  std::shared_ptr<stream::session_t> make_session(int ml_feature_flags) {
    stream::config_t config {};
    config.packetsize = packet_size;
    config.minRequiredFecPackets = 2;
    config.mlFeatureFlags = ml_feature_flags;
    config.requestedBitrateKbps = 20000;
    config.monitor.bitrate = 20000;
    config.monitor.framerate = 60;
    rtsp_stream::launch_session_t launch_session {};
    launch_session.gcm_key.resize(16);
    launch_session.iv.resize(16);
    return stream::session::alloc(config, launch_session);
  }

  const int signals = ML_FF_ADAPTIVE_SIGNALS | ML_FF_FEC_STATUS;

  /**
   * @brief A client's UDP socket on the loopback address and a sender's socket next to it.
   */
  struct loopback_t {
    loopback_t():
        client {io, boost::asio::ip::udp::endpoint {boost::asio::ip::make_address("127.0.0.1"), 0}},
        sender {io, boost::asio::ip::udp::endpoint {boost::asio::ip::address_v4::any(), 0}} {
      client.non_blocking(true);
    }

    /**
     * @brief Collect the datagrams that arrive within `wait`, or until `count` have.
     */
    std::vector<std::vector<uint8_t>> receive(std::size_t count, std::chrono::milliseconds wait) {
      std::vector<std::vector<uint8_t>> got;
      const auto until = std::chrono::steady_clock::now() + wait;
      while (got.size() < count && std::chrono::steady_clock::now() < until) {
        std::vector<uint8_t> buffer(4096);
        boost::system::error_code ec;
        const auto n = client.receive(boost::asio::buffer(buffer), 0, ec);
        if (ec) {
          std::this_thread::sleep_for(1ms);
          continue;
        }
        buffer.resize(n);
        got.push_back(std::move(buffer));
      }
      return got;
    }

    boost::asio::io_context io;
    boost::asio::ip::udp::socket client;
    boost::asio::ip::udp::socket sender;
  };

  const std::function<std::unique_ptr<platf::high_precision_timer>()> real_timer = []() {
    return platf::create_high_precision_timer();
  };

  /**
   * @brief A timer that says when it is destroyed, which is when its sender thread has ended.
   */
  struct flag_timer_t: platf::high_precision_timer {
    explicit flag_timer_t(std::atomic<bool> &destroyed):
        destroyed {destroyed} {
    }

    ~flag_timer_t() override {
      destroyed = true;
    }

    void sleep_for(const std::chrono::nanoseconds &duration) override {
      std::this_thread::sleep_for(duration);
    }

    explicit operator bool() override {
      return true;
    }

    std::atomic<bool> &destroyed;
  };

  std::shared_ptr<stream::session_t> make_stream_session() {
    stream::config_t stream_config {};
    rtsp_stream::launch_session_t launch_session {};
    launch_session.gcm_key.resize(16);
    launch_session.iv.resize(16);
    return stream::session::alloc(stream_config, launch_session);
  }

  std::vector<char> loss_stats(std::uint32_t lost, std::uint32_t last_good) {
    std::vector<char> out(32, 0);
    out[0] = char(lost);
    out[12] = char(last_good);
    return out;
  }

  // Frames through a clean path, as the control thread sees them; returns the time reached
  std::chrono::steady_clock::time_point feed(adaptive_bitrate::session_t &session, std::chrono::steady_clock::time_point start, int frames) {
    for (int i = 0; i < frames; ++i) {
      const auto sent = start + std::chrono::microseconds(i * 16667);
      session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
      if (i % 3 == 2) {
        auto msg = loss_stats(0, i + 1);
        session.loss_stats(sent + 30ms, {msg.data(), msg.size()});
      }
      session.tick(sent + 50ms, 0.060);
    }
    return start + std::chrono::microseconds(frames * 16667) + 50ms;
  }

  /**
   * @brief An NVENC encoder that cannot change its bitrate during the session, as on a GPU
   *        without NV_ENC_CAPS_SUPPORT_DYN_BITRATE_CHANGE.
   */
  struct fixed_bitrate_nvenc_t: nvenc::nvenc_encoder {
    bool create_encoder(const nvenc::nvenc_config &, const video::config_t &, const video::sunshine_colorspace_t &, platf::pix_fmt_e) override {
      return true;
    }

    void destroy_encoder() override {
    }

    nvenc::nvenc_encoded_frame encode_frame(std::uint64_t, bool) override {
      return {};
    }

    bool invalidate_ref_frames(std::uint64_t, std::uint64_t) override {
      return false;
    }

    bool set_bitrate(std::uint32_t) override {
      return false;
    }

    std::uint32_t bitrate_kbps() const override {
      return 8000;
    }
  };

  /**
   * @brief The platform side of an NVENC session, around a given encoder.
   */
  struct nvenc_device_t: platf::nvenc_encode_device_t {
    explicit nvenc_device_t(nvenc::nvenc_encoder *encoder) {
      nvenc = encoder;
    }

    bool init_encoder(const video::config_t &, const video::sunshine_colorspace_t &) override {
      return true;
    }

    int convert(platf::img_t &) override {
      return 0;
    }
  };

  template<class T>
  void drain(safe::mail_raw_t::event_t<T> &events) {
    while (events->peek()) {
      events->pop(0ms);
    }
  }

  /**
   * @brief Sets a configuration value for as long as it lives.
   */
  struct scoped_config_t {
    scoped_config_t(std::string mode, int fec_percentage):
        saved_mode {config::stream.adaptive_bitrate},
        saved_fec {config::stream.fec_percentage} {
      config::stream.adaptive_bitrate = std::move(mode);
      config::stream.fec_percentage = fec_percentage;
    }

    ~scoped_config_t() {
      config::stream.adaptive_bitrate = saved_mode;
      config::stream.fec_percentage = saved_fec;
    }

    scoped_config_t(const scoped_config_t &) = delete;
    scoped_config_t &operator=(const scoped_config_t &) = delete;

    std::string saved_mode;
    int saved_fec;
  };
}  // namespace

// @ABR-19-S2
TEST(StreamSenderTest, AFrameOfAControlledSessionLeavesFromItsOwnSenderToTheClient) {
  scoped_mode_t mode {"full"};
  auto session = make_session(signals);
  ASSERT_NE(session.get(), nullptr);
  loopback_t net;
  stream::set_video_route(*session, net.client.local_endpoint(), boost::asio::ip::make_address("127.0.0.1"));

  const auto epoch = std::chrono::steady_clock::now();
  video::packet_t frame = std::make_unique<test_packet_t>(session.get(), 7, 6000, true);
  frame->frame_timestamp = epoch;
  ASSERT_TRUE(stream::hand_off_to_own_sender(*session, frame, net.sender, epoch, real_timer));
  EXPECT_FALSE(frame) << "the frame now belongs to the session's queue";

  const auto datagrams = net.receive(6000 / (packet_size - 8) + 1, 3s);
  stream::close_own_sender(*session);
  ASSERT_GE(datagrams.size(), 6U) << "the data packets of the frame and its parity reached the client";
  for (const auto &datagram : datagrams) {
    EXPECT_EQ(datagram.size(), datagrams.front().size()) << "every packet of a frame has the block size";
    EXPECT_EQ(datagram[0], 0x90) << "an RTP header with the extension bit";
  }
  EXPECT_GT(stream::video_paid_until(*session), std::chrono::steady_clock::time_point {}) << "the frame was paid at the controller's budget";
}

// @ABR-19-S2
TEST(StreamSenderTest, ASessionWithoutAControllerOrWithoutATimerKeepsItsFrames) {
  loopback_t net;
  {
    scoped_mode_t mode {"disabled"};
    auto session = make_session(signals);
    ASSERT_NE(session.get(), nullptr);
    video::packet_t frame = std::make_unique<test_packet_t>(session.get(), 1, 100, false);
    EXPECT_FALSE(stream::hand_off_to_own_sender(*session, frame, net.sender, std::chrono::steady_clock::now(), real_timer)) << "the broadcast sender sends it";
    EXPECT_TRUE(frame);
  }
  {
    scoped_mode_t mode {"full"};
    auto session = make_session(0);
    ASSERT_NE(session.get(), nullptr);
    video::packet_t frame = std::make_unique<test_packet_t>(session.get(), 1, 100, false);
    EXPECT_FALSE(stream::hand_off_to_own_sender(*session, frame, net.sender, std::chrono::steady_clock::now(), real_timer)) << "a client that does not announce its signals is not controlled";
    EXPECT_TRUE(frame);
  }
  {
    scoped_mode_t mode {"full"};
    auto session = make_session(signals);
    ASSERT_NE(session.get(), nullptr);
    video::packet_t frame = std::make_unique<test_packet_t>(session.get(), 1, 100, false);
    const std::function<std::unique_ptr<platf::high_precision_timer>()> no_timer = []() {
      return std::unique_ptr<platf::high_precision_timer> {};
    };
    EXPECT_FALSE(stream::hand_off_to_own_sender(*session, frame, net.sender, std::chrono::steady_clock::now(), no_timer));
    EXPECT_TRUE(frame) << "a failed timer leaves the frame to the broadcast sender";
    stream::close_own_sender(*session);
  }
}

// @ABR-19-S2
TEST(StreamSenderTest, TheControllerPacesAHeldSessionAndTheDefaultPacesTheRest) {
  scoped_mode_t mode {"full"};
  auto session = make_session(signals);
  ASSERT_NE(session.get(), nullptr);
  const auto now = std::chrono::steady_clock::now();
  EXPECT_GT(stream::controller_pacing_bps(*session, true, now), 0.0) << "the controller's rate";
  EXPECT_EQ(stream::controller_pacing_bps(*session, false, now), 0.0) << "0 is the stream's default pacing";
}

// @ABR-19-S2
TEST(StreamSenderTest, AFrameOwesItsWireBytesAtTheBudgetAndIsReportedToTheController) {
  scoped_mode_t mode {"full"};
  auto session = make_session(signals);
  ASSERT_NE(session.get(), nullptr);
  const auto start = std::chrono::steady_clock::now();
  test_packet_t frame {session.get(), 3, 100, false};
  EXPECT_EQ(stream::video_paid_until(*session), std::chrono::steady_clock::time_point {});

  // No budget: nothing is owed, and a frame that sent no packet reports nothing
  stream::report_frame(*session, frame, start, 0.0, std::nullopt, start, 0, 0, 0.0, 0.0);
  EXPECT_EQ(stream::video_paid_until(*session), std::chrono::steady_clock::time_point {});

  // 1 Mbit on the wire at 10 Mbit/s is paid for at 10 % over the budget: 1/11 s after the frame starts
  frame.frame_timestamp = start;
  stream::report_frame(*session, frame, start, 10e6, start + 2ms, start + 3ms, 12, 10, 12 * 1000.0, 125000.0);
  EXPECT_NEAR(std::chrono::duration<double>(stream::video_paid_until(*session) - start).count(), 1.0 / 11.0, 1e-3);

  // A frame without a capture time is reported without its host delay
  test_packet_t duplicate {session.get(), 4, 100, false};
  stream::report_frame(*session, duplicate, start, 10e6, start + 20ms, start + 21ms, 12, 10, 12 * 1000.0, 125000.0);
  stream::close_own_sender(*session);
}

// @ABR-18-S1 @ABR-18-S3
TEST(StreamSenderTest, ClosingJoinsTheOwnSenderAndLeavesItsStoppedQueueForTheSession) {
  auto session = make_stream_session();
  ASSERT_NE(session.get(), nullptr);
  boost::asio::io_context io;
  boost::asio::ip::udp::socket sock {io};
  std::atomic<bool> destroyed {false};
  int timers_made = 0;
  const std::function<std::unique_ptr<platf::high_precision_timer>()> make_timer = [&]() {
    ++timers_made;
    return std::make_unique<flag_timer_t>(destroyed);
  };

  auto own = stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), make_timer);
  ASSERT_NE(own, nullptr);
  EXPECT_FALSE(destroyed) << "the sender runs until the session closes";

  stream::close_own_sender(*session);
  EXPECT_TRUE(destroyed) << "closing returns once the sender thread has ended, not while it can still be in a frame";

  // What the broadcast thread meets after the close: the stopped queue of the session's own sender,
  // not nullptr, so it never sends the session's packets itself
  auto after = stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), make_timer);
  ASSERT_NE(after, nullptr) << "a session that had an own sender never goes back to the broadcast sender";
  EXPECT_FALSE(after->raise(video::packet_t {})) << "what is queued for a closed session is dropped";
  EXPECT_EQ(timers_made, 1) << "no second sender";

  stream::close_own_sender(*session);
}

// @ABR-18-S1
TEST(StreamSenderTest, ASessionThatNeverHadAnOwnSenderStaysOnTheBroadcastSender) {
  auto session = make_stream_session();
  ASSERT_NE(session.get(), nullptr);
  boost::asio::io_context io;
  boost::asio::ip::udp::socket sock {io};
  int timers_made = 0;
  const std::function<std::unique_ptr<platf::high_precision_timer>()> make_timer = [&]() {
    ++timers_made;
    return std::unique_ptr<platf::high_precision_timer> {};
  };

  stream::close_own_sender(*session);
  EXPECT_EQ(stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), make_timer), nullptr) << "the broadcast thread is the session's only sender";
  EXPECT_EQ(timers_made, 0) << "a closed session starts no sender";
}

// @ABR-18-S2 @ABR-18-S3
TEST(StreamSenderTest, AFailedTimerTurnsTheControllersHoldOffWithTheOwnSender) {
  auto session = make_stream_session();
  ASSERT_NE(session.get(), nullptr);
  boost::asio::io_context io;
  boost::asio::ip::udp::socket sock {io};
  const std::function<std::unique_ptr<platf::high_precision_timer>()> failing = []() {
    return std::unique_ptr<platf::high_precision_timer> {};
  };

  EXPECT_TRUE(stream::controller_holds_frames(*session)) << "the own sender may still start";
  EXPECT_EQ(stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), failing), nullptr);
  EXPECT_FALSE(stream::controller_holds_frames(*session)) << "the broadcast thread sends this session's packets at the stream's default pacing, never holding a frame ahead of other sessions";
  stream::close_own_sender(*session);
  EXPECT_FALSE(stream::controller_holds_frames(*session));
}

// @ABR-18-S8
TEST(StreamSenderTest, AReopenedEncoderOpensAtTheControllersBitrate) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::full, 20000, 60, 1024, 90, 300000.0};
  const auto now = feed(session, std::chrono::steady_clock::now(), 180);
  ASSERT_GT(session.target_bitrate_kbps(), 0);
  const int wanted = session.bitrate_to_apply(now);
  ASSERT_GT(wanted, 0);
  EXPECT_EQ(session.initial_encoder_kbps(), wanted) << "what the controller asks the running encoder for is what a new one opens at";

  session.encoder_bitrate_fixed(8000);
  EXPECT_EQ(session.initial_encoder_kbps(), 0) << "an encoder that keeps its bitrate keeps the client's";
}

// @ABR-18-S8
TEST(StreamSenderTest, TheControllerSendsItsBitrateRightAfterTheReopen) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::full, 20000, 60, 1024, 90, 300000.0};
  const auto now = feed(session, std::chrono::steady_clock::now(), 180);
  const int first = session.bitrate_to_apply(now);
  ASSERT_GT(first, 0);
  EXPECT_EQ(session.bitrate_to_apply(now + 10ms), 0) << "nothing moved, and the last push is recent";

  session.encoder_reopened();
  EXPECT_EQ(session.bitrate_to_apply(now + 20ms), first) << "the new encoder is told at once, not after the 2 % change or the 1 s refresh";
  EXPECT_EQ(session.bitrate_to_apply(now + 30ms), 0);
}

// @ABR-06-S5 @ABR-06-ASM-02
TEST(StreamSenderTest, TheHoldWaitsForThePreviousFramesAndNeverPassesAQuarterSecond) {
  const auto now = std::chrono::steady_clock::now();
  const auto captured = now - 10ms;

  // The previous frames already paid for: no hold
  EXPECT_EQ(stream::frame_send_timing(now, now - 1ms, captured, 10e6, now - 5ms).first, now);

  // Held until they are paid for
  EXPECT_EQ(stream::frame_send_timing(now, now - 1ms, captured, 10e6, now + 40ms).first, now + 40ms);

  // Upstream's pacing of the previous frame still comes first when it ends later
  EXPECT_EQ(stream::frame_send_timing(now, now + 60ms, captured, 10e6, now + 40ms).first, now + 60ms);

  // The cap: a quarter of a second from now, however much is owed
  EXPECT_EQ(stream::frame_send_timing(now, now, captured, 10e6, now + 250ms).first, now + 250ms);
  EXPECT_EQ(stream::frame_send_timing(now, now, captured, 10e6, now + 251ms).first, now + 250ms);
  EXPECT_EQ(stream::frame_send_timing(now, now, captured, 10e6, now + 10s).first, now + 250ms);

  // A frame captured 120 ms ago is still held; one captured longer ago is not
  EXPECT_EQ(stream::frame_send_timing(now, now, now - 120ms, 10e6, now + 40ms).first, now + 40ms);
  EXPECT_EQ(stream::frame_send_timing(now, now, now - 121ms, 10e6, now + 40ms).first, now);

  // Without the controller's budget nothing is held
  EXPECT_EQ(stream::frame_send_timing(now, now, captured, 0.0, now + 40ms).first, now);
}

// @ABR-06-S8 @ABR-06-ASM-05
TEST(StreamSenderTest, TheSessionsPacketsGoToOneQueueOfItsOwnUntilItEnds) {
  stream::config_t stream_config {};
  rtsp_stream::launch_session_t launch_session {};
  launch_session.gcm_key.resize(16);
  launch_session.iv.resize(16);
  auto session = stream::session::alloc(stream_config, launch_session);
  ASSERT_NE(session.get(), nullptr);

  boost::asio::io_context io;
  boost::asio::ip::udp::socket sock {io};
  int timers_made = 0;
  const std::function<std::unique_ptr<platf::high_precision_timer>()> make_timer = [&timers_made]() {
    ++timers_made;
    return platf::create_high_precision_timer();
  };

  const auto first = stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), make_timer);
  ASSERT_NE(first, nullptr) << "with a timer the session gets a sender of its own";
  EXPECT_TRUE(first->running());
  const auto again = stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), make_timer);
  EXPECT_EQ(again, first) << "every packet of the session goes to the same queue";
  EXPECT_EQ(timers_made, 1) << "one sender, one timer";

  // Stopping the queue ends the sender; freeing the session joins it
  first->stop();
  session.reset();
  EXPECT_FALSE(first->running());
}

// @ABR-05-S1 @ABR-05-S2 @ABR-05-ASM-01
TEST(StreamSenderTest, AnNvencSessionThatCannotChangeItsBitrateSaysWhichItKeeps) {
  fixed_bitrate_nvenc_t encoder;
  const video::config_t config {1280, 720, 60, 8000, 8000, 1, 0, 1, 0, 0, 0, 0};
  auto session = video::make_encode_session(nullptr, video::software, config, 1280, 720, std::make_unique<nvenc_device_t>(&encoder));
  ASSERT_TRUE(session);
  EXPECT_EQ(session->opened_bitrate_kbps(), 8000);

  auto events = mail::man->event<int>(mail::bitrate);
  auto fixed = mail::man->event<int>(mail::bitrate_fixed);
  drain(events);
  drain(fixed);
  bool reported = false;

  // The controller asks for less: the encoder keeps what it has, and the controller is told so
  events->raise(5000);
  video::apply_requested_bitrate(*session, events, fixed, reported);
  EXPECT_TRUE(reported);
  ASSERT_TRUE(fixed->peek()) << "without it the controller holds frames to a budget the encoder ignores";
  EXPECT_EQ(fixed->pop(0ms), 8000);

  // Once per encoder
  events->raise(4000);
  video::apply_requested_bitrate(*session, events, fixed, reported);
  EXPECT_FALSE(fixed->peek());
}

// @ABR-05-S11 @ABR-05-ASM-07
TEST(StreamSenderTest, TheLatencySentCountsTheHold) {
  const auto now = std::chrono::steady_clock::now();
  const auto captured = now - 10ms;

  // Held 100 ms until the previous frames are paid for: the client is told 110 ms
  const auto [held_start, held] = stream::frame_send_timing(now, now - 1ms, captured, 10e6, now + 100ms);
  EXPECT_EQ(held_start, now + 100ms);
  ASSERT_TRUE(held);
  EXPECT_EQ(*held, 1100) << "tenths of a millisecond";

  // Unheld without a budget
  const auto [free_start, unheld] = stream::frame_send_timing(now, now - 1ms, captured, 0.0, now + 100ms);
  EXPECT_EQ(free_start, now);
  ASSERT_TRUE(unheld);
  EXPECT_EQ(*unheld, 100);

  // The hold never passes a quarter of a second
  const auto [capped_start, capped] = stream::frame_send_timing(now, now, captured, 10e6, now + 1s);
  EXPECT_EQ(capped_start, now + 250ms);
  ASSERT_TRUE(capped);
  EXPECT_EQ(*capped, 2600);

  // A frame already late is not held
  const auto [late_start, late] = stream::frame_send_timing(now, now, now - 130ms, 10e6, now + 100ms);
  EXPECT_EQ(late_start, now);
  ASSERT_TRUE(late);
  EXPECT_EQ(*late, 1300);

  // Without a capture time there is no latency to tell
  const auto [dupe_start, dupe] = stream::frame_send_timing(now, now, std::nullopt, 10e6, now + 100ms);
  EXPECT_EQ(dupe_start, now + 100ms);
  EXPECT_FALSE(dupe);
}

// @ABR-05-S12 @ABR-05-ASM-08
TEST(StreamSenderTest, AFailedTimerLeavesTheSessionOnTheBroadcastSender) {
  stream::config_t stream_config {};
  rtsp_stream::launch_session_t launch_session {};
  launch_session.gcm_key.resize(16);
  launch_session.iv.resize(16);
  auto session = stream::session::alloc(stream_config, launch_session);
  ASSERT_NE(session.get(), nullptr);

  boost::asio::io_context io;
  boost::asio::ip::udp::socket sock {io};
  int timers_made = 0;
  const std::function<std::unique_ptr<platf::high_precision_timer>()> failing = [&timers_made]() {
    ++timers_made;
    return std::unique_ptr<platf::high_precision_timer> {};
  };

  EXPECT_EQ(stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), failing), nullptr) << "no sender without a timer: the broadcast sender sends the session's packets";
  EXPECT_EQ(stream::own_sender_queue(*session, sock, std::chrono::steady_clock::now(), failing), nullptr) << "for the rest of the session";
  EXPECT_EQ(timers_made, 1) << "no retry";
}

// @ABR-22-S2
TEST(StreamSenderTest, AnEncoderReopenedInFullModeOpensAtTheControllersBitrate) {
  scoped_config_t mode {"full", 20};
  auto session = make_session(signals);
  ASSERT_NE(session.get(), nullptr);
  EXPECT_GT(stream::session::encoder_open_kbps(session.get()), 0);
}
