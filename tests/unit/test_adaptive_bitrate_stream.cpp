/**
 * @file tests/unit/test_adaptive_bitrate_stream.cpp
 * @brief Test how the adaptive bitrate session fits into the stream: audio estimate, log lines, malformed messages, FEC blocks, headers and the client bitrate.
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
#include <src/platform/common.h>
#include <src/rtsp.h>
#include <src/stream.h>
#include <src/stream_internal.h>
#include <src/stream_pacing.h>
#include <src/thread_safe.h>
#include <src/video.h>
#include <src/video_internal.h>

using namespace std::literals;
using adaptive_bitrate::mode_e;

namespace {
  constexpr auto log_file = "test_sunshine.log";

  void put32le(std::vector<char> &out, std::uint32_t v) {
    out.push_back(char(v));
    out.push_back(char(v >> 8));
    out.push_back(char(v >> 16));
    out.push_back(char(v >> 24));
  }

  // A loss stats message, laid out as moonlight-common-c sends it: little-endian
  std::vector<char> loss_stats_message(std::uint32_t lost, std::uint32_t last_good_frame) {
    std::vector<char> out;
    put32le(out, lost);
    put32le(out, 50);
    put32le(out, 1000);
    put32le(out, last_good_frame);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0);
    put32le(out, 0x14);
    return out;
  }

  constexpr int packet_size = 1024;
  const int signals = ML_FF_ADAPTIVE_SIGNALS | ML_FF_FEC_STATUS;

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

  /**
   * @brief An encoded frame of a given size, for a session.
   */
  struct test_packet_t: video::packet_raw_t {
    test_packet_t(stream::session_t *session, int64_t index, std::size_t size):
        bytes(size, 0x5a),
        index {index} {
      channel_data = session;
    }

    bool is_idr() override {
      return false;
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
   * @brief What a video datagram says about its FEC block.
   */
  struct shard_t {
    int block;
    int blocks;
    int percentage;
    int data_shards;
  };

  shard_t parse_shard(const std::vector<uint8_t> &datagram) {
    constexpr std::size_t nv_packet = sizeof(RTP_PACKET) + 4;
    std::uint32_t fec_info = 0;
    std::memcpy(&fec_info, datagram.data() + nv_packet + offsetof(NV_VIDEO_PACKET, fecInfo), sizeof(fec_info));
    const std::uint8_t multi = datagram[nv_packet + offsetof(NV_VIDEO_PACKET, multiFecBlocks)];
    return {(multi >> 4) & 3, (multi >> 6) + 1, static_cast<int>((fec_info >> 4) & 0xff), static_cast<int>(fec_info >> 22)};
  }
}  // namespace

// @ABR-05-S6 @ABR-05-ASM-03
TEST(AdaptiveBitrateStreamTest, TheAudioEstimateIsTheOpusLayoutPlus40BytesAPacketWithItsFec) {
  audio::config_t config {};
  config.packetDuration = 5;
  config.channels = 2;
  config.flags[audio::config_t::HIGH_QUALITY] = true;
  // 200 packets a second of 40 bytes of RTP, IPv4 and UDP over Opus at 512 kbps, and 2 parity
  // packets for every 4
  EXPECT_DOUBLE_EQ(stream::estimated_audio_bps(config, 28), (512000.0 + 200 * 40 * 8) * 1.5);

  config.channels = 8;
  EXPECT_DOUBLE_EQ(stream::estimated_audio_bps(config, 28), (2048000.0 + 200 * 40 * 8) * 1.5);

  config.channels = 2;
  config.flags[audio::config_t::HIGH_QUALITY] = false;
  config.packetDuration = 10;
  EXPECT_DOUBLE_EQ(stream::estimated_audio_bps(config, 28), (96000.0 + 100 * 40 * 8) * 1.5);

  config.packetDuration = 0;
  EXPECT_DOUBLE_EQ(stream::estimated_audio_bps(config, 28), (96000.0 + 200 * 40 * 8) * 1.5) << "5 ms when the client gives none";

  // An IPv6 client costs 20 bytes more on every packet
  EXPECT_DOUBLE_EQ(stream::estimated_audio_bps(config, 48), (96000.0 + 200 * 60 * 8) * 1.5);
}

// @ABR-05-S7 @ABR-05-ASM-04
TEST(AdaptiveBitrateStreamTest, TheStepLineIsVerboseAndTheSessionLineDebug) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 180; ++i) {
    const auto sent = start + std::chrono::microseconds(i * 16667);
    session.frame_sent(i + 1, sent, sent, 12, 10, 12 * 1040.0, 12 * 1130.0);
    if (i % 3 == 2) {
      auto msg = loss_stats_message(0, i + 1);
      ASSERT_TRUE(session.loss_stats(sent + 30ms, {msg.data(), msg.size()}));
    }
    session.tick(sent + 50ms, 0.060);
  }

  EXPECT_TRUE(log_checker::line_contains(log_file, "Verbose: Adaptive bitrate step: "));
  EXPECT_FALSE(log_checker::line_contains(log_file, "Debug: Adaptive bitrate step: "));
  EXPECT_TRUE(log_checker::line_contains(log_file, "Debug: Adaptive bitrate [fec]: "));
  EXPECT_FALSE(log_checker::line_contains(log_file, "Info: Adaptive bitrate [fec]: "));
}

// @ABR-05-S8 @ABR-05-ASM-05
TEST(AdaptiveBitrateStreamTest, AMalformedMessageWarnsOncePerKind) {
  using adaptive_bitrate::message_e;
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  EXPECT_TRUE(session.malformed(message_e::fec_status));
  EXPECT_FALSE(session.malformed(message_e::fec_status));
  EXPECT_FALSE(session.malformed(message_e::fec_status));
  EXPECT_TRUE(session.malformed(message_e::loss_stats)) << "each kind warns once";
  EXPECT_TRUE(session.malformed(message_e::frame_stats));
  EXPECT_FALSE(session.malformed(message_e::loss_stats));
  EXPECT_FALSE(session.malformed(message_e::frame_stats));

  adaptive_bitrate::session_t next {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  EXPECT_TRUE(next.malformed(message_e::fec_status)) << "once per session";
}

// @ABR-22-S3
TEST(AdaptiveBitrateStreamTest, ASessionThatAnnouncedNothingSendsAFrameOverFourBlocksWithNoParityAsUpstream) {
  scoped_config_t mode {"fec", 20};
  auto session = make_session(0);
  ASSERT_NE(session.get(), nullptr);

  boost::asio::io_context io;
  boost::asio::ip::udp::socket client {io, boost::asio::ip::udp::endpoint {boost::asio::ip::make_address("127.0.0.1"), 0}};
  boost::asio::ip::udp::socket sender {io, boost::asio::ip::udp::endpoint {boost::asio::ip::address_v4::any(), 0}};
  client.non_blocking(true);
  // The frame goes out in one burst: take the largest buffer the host allows (FreeBSD caps it near 2 MiB)
  for (const int bytes : {8 * 1024 * 1024, 4 * 1024 * 1024, 2 * 1024 * 1024, 1024 * 1024, 512 * 1024}) {
    boost::system::error_code refused;
    client.set_option(boost::asio::socket_base::receive_buffer_size(bytes), refused);
    if (!refused) {
      break;
    }
  }
  stream::set_video_route(*session, client.local_endpoint(), boost::asio::ip::make_address("127.0.0.1"));

  // About 893 packets: five blocks at 20 % (212 data shards fit a block), so upstream sends four with no FEC
  auto packets = mail::man->queue<video::packet_t>(mail::video_packets);
  auto shutdown = mail::man->event<bool>(mail::broadcast_shutdown);
  const auto epoch = std::chrono::steady_clock::now();
  std::thread sender_thread {stream::videoBroadcastThread, std::ref(sender)};
  video::packet_t frame = std::make_unique<test_packet_t>(session.get(), 1, 900000);
  frame->frame_timestamp = epoch;
  packets->raise(std::move(frame));

  std::vector<shard_t> shards;
  const auto until = std::chrono::steady_clock::now() + 3s;
  auto quiet_since = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < until && (shards.empty() || std::chrono::steady_clock::now() - quiet_since < 500ms)) {
    std::vector<uint8_t> buffer(4096);
    boost::system::error_code ec;
    const auto n = client.receive(boost::asio::buffer(buffer), 0, ec);
    if (ec) {
      std::this_thread::yield();  // keep draining: a sleeping reader overflows the buffer during the burst
      continue;
    }
    buffer.resize(n);
    shards.push_back(parse_shard(buffer));
    quiet_since = std::chrono::steady_clock::now();
  }

  shutdown->raise(true);
  packets->raise(std::make_unique<test_packet_t>(session.get(), 2, 100));
  sender_thread.join();
  shutdown->reset();

  ASSERT_FALSE(shards.empty()) << "the frame reached the client";
  std::map<int, int> sent_per_block;
  std::map<int, int> data_per_block;
  for (const auto &shard : shards) {
    EXPECT_EQ(shard.blocks, 4);
    EXPECT_EQ(shard.percentage, 0) << "upstream drops the FEC of a frame that needs more than four blocks";
    ++sent_per_block[shard.block];
    data_per_block[shard.block] = shard.data_shards;
  }
  EXPECT_EQ(sent_per_block, data_per_block) << "every packet of a block is a data packet";
}

// @ABR-22-S4
TEST(AdaptiveBitrateStreamTest, ABlockTheControllerSizesAtZeroParityGoesOutWithNoFec) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::fec, 20000, 60, 1024, 90, 300000.0};
  feed(session, std::chrono::steady_clock::now(), 2400);
  ASSERT_EQ(session.min_parity(2), 0) << "40 s without a loss drop the minimum parity";

  int zero = 0;
  for (int data = 1; data <= 60; ++data) {
    const int percentage = session.block_fec_percentage(data);
    ASSERT_GE(percentage, 0) << data;
    if (percentage == 0) {
      ++zero;
    }
  }
  EXPECT_GT(zero, 0) << "a clean path needs no parity on a small block, and the block is sent with 0 %";
}

// @ABR-18-S4
TEST(AdaptiveBitrateStreamTest, TheWireCostCountsTheHeadersOfTheAddressFamily) {
  using boost::asio::ip::make_address;
  EXPECT_EQ(stream::ip_udp_header_bytes(make_address("192.168.1.20")), 28) << "IPv4 20 + UDP 8";
  EXPECT_EQ(stream::ip_udp_header_bytes(make_address("fe80::1")), 48) << "IPv6 40 + UDP 8";
  EXPECT_EQ(stream::ip_udp_header_bytes(make_address("::ffff:192.168.1.20")), 28) << "an IPv4-mapped address travels as IPv4";

  stream::config_t config {};
  EXPECT_EQ(config.ipHeaderBytes, 28) << "IPv4 until the RTSP socket says otherwise";
  stream::config_t v6 {};
  v6.ipHeaderBytes = 48;
  EXPECT_EQ(stream::video_wire_overhead(v6) - stream::video_wire_overhead(config), 20) << "the controller's overhead follows the header bytes";
}

// @ABR-18-S5
TEST(AdaptiveBitrateStreamTest, TheClientsBitrateIsClampedToAnInt) {
  constexpr int vqos = 16000;
  constexpr int int_max = std::numeric_limits<int>::max();
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, 20000), 20000);
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, std::int64_t {int_max} + 1), int_max) << "no wrap to a negative or tiny ceiling";
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, std::int64_t {5'000'000'000}), int_max);
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, 0), vqos);
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, -7), vqos) << "zero or negative keeps the vqos value";
  EXPECT_EQ(adaptive_bitrate::ceiling_kbps(vqos, std::numeric_limits<std::int64_t>::min()), vqos);
}

// @ABR-18-S6
TEST(AdaptiveBitrateStreamTest, FecPercentageZeroFromTheUserStaysZero) {
  adaptive_bitrate::session_t session {adaptive_bitrate::mode_e::full, 20000, 60, 1024, 90, 300000.0};
  feed(session, std::chrono::steady_clock::now(), 180);
  ASSERT_TRUE(session.decision()->stepped);
  EXPECT_EQ(session.fec_percentage(0), 0) << "the controller does not raise a percentage the user turned off to 1";
  EXPECT_GE(session.fec_percentage(20), 1);
}

// @ABR-18-S7
TEST(AdaptiveBitrateStreamTest, AFrameOverFourBlocksKeepsItsParityWithTheController) {
  constexpr std::size_t blocksize = 1400;

  auto plan = stream::fec_plan(100 * blocksize, blocksize, 20, true);
  EXPECT_EQ(plan.blocks, 1);
  EXPECT_EQ(plan.percentage, 20);

  plan = stream::fec_plan(600 * blocksize, blocksize, 20, true);
  EXPECT_EQ(plan.blocks, 3) << "212 data shards fit a block at 20 %";
  EXPECT_EQ(plan.percentage, 20);

  // 900 packets need five blocks at 20 %; the protocol has two bits for the count
  plan = stream::fec_plan(900 * blocksize, blocksize, 20, true);
  EXPECT_EQ(plan.blocks, 4);
  EXPECT_GT(plan.percentage, 0) << "parity is not dropped";
  EXPECT_LT(plan.percentage, 20) << "it is lowered to what fits";
  constexpr int data = 225;  // 900 packets in four blocks
  EXPECT_LE(data + (data * plan.percentage + 99) / 100, 255) << "data and parity fit the 255 shards of a block";
  EXPECT_GE(data + (data * (plan.percentage + 1) + 99) / 100, 255) << "and it is the largest percentage that does";

  // Four blocks of data alone fill the field: nothing is left for parity
  plan = stream::fec_plan(1020 * blocksize, blocksize, 20, true);
  EXPECT_EQ(plan.blocks, 4);
  EXPECT_EQ(plan.percentage, 0);

  // A percentage of zero stays zero
  plan = stream::fec_plan(900 * blocksize, blocksize, 0, true);
  EXPECT_EQ(plan.blocks, 4);
  EXPECT_EQ(plan.percentage, 0);
}

// @ABR-22-S3
TEST(AdaptiveBitrateStreamTest, AFrameOverFourBlocksOfAStockSessionGoesOutWithNoFecAsUpstream) {
  constexpr std::size_t blocksize = 1400;

  auto plan = stream::fec_plan(100 * blocksize, blocksize, 20, false);
  EXPECT_EQ(plan.blocks, 1);
  EXPECT_EQ(plan.percentage, 20);

  plan = stream::fec_plan(600 * blocksize, blocksize, 20, false);
  EXPECT_EQ(plan.blocks, 3);
  EXPECT_EQ(plan.percentage, 20);

  plan = stream::fec_plan(900 * blocksize, blocksize, 20, false);
  EXPECT_EQ(plan.blocks, 4);
  EXPECT_EQ(plan.percentage, 0) << "upstream skips the FEC of a frame that needs more than four blocks";

  plan = stream::fec_plan(900 * blocksize, blocksize, 0, false);
  EXPECT_EQ(plan.blocks, 4);
  EXPECT_EQ(plan.percentage, 0);
}

// @ABR-16-S1
TEST(AdaptiveBitrateStreamTest, AnUnknownModeReadsAsDisabled) {
  for (const auto unknown : {"shadow", "bogus", "FEC", " fec"}) {
    EXPECT_EQ(adaptive_bitrate::mode_from_string(unknown), mode_e::disabled) << "a config that says '" << unknown << "' gets the default";
  }
  EXPECT_EQ(adaptive_bitrate::mode_from_string("disabled"), mode_e::disabled);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("fec"), mode_e::fec);
  EXPECT_EQ(adaptive_bitrate::mode_from_string("full"), mode_e::full);
  EXPECT_EQ(adaptive_bitrate::mode_from_string(""), mode_e::disabled);
}
