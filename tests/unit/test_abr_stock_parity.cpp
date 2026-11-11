/**
 * @file tests/unit/test_abr_stock_parity.cpp
 * @brief Test that sessions the adaptive bitrate controller does not act on stream as upstream does.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

// ffmpeg includes
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/cbs_h264.h>
}

// local includes
#include <src/adaptive_bitrate.h>
#include <src/config.h>
#include <src/display_device.h>
#include <src/utility.h>
#include <src/video.h>

using namespace std::literals;

namespace {
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
   * @brief Whether the encoder the last probe chose would be advertised as able to invalidate references.
   */
  bool advertises_invalidation(const video::encoder_t &encoder) {
    video::reset_encoder_probe_results();
    video::record_encoder_probe_results(encoder);
    const bool advertised = video::last_encoder_probe_supported_ref_frames_invalidation;
    video::reset_encoder_probe_results();
    return advertised;
  }
}  // namespace

// @ABR-02-S1 @ABR-02-S6
TEST(AbrStockParityTest, WithTheFeatureOffLibx264AdvertisesNoInvalidation) {
  scoped_mode_t mode {"disabled"};
  EXPECT_FALSE(advertises_invalidation(video::software)) << "upstream 1582fff advertises no refPicInvalidation for libx264";
}

// @ABR-15-S2
TEST(AbrStockParityTest, Libx264AdvertisesNoInvalidationInAnyMode) {
  for (const auto *name : {"disabled", "fec", "full"}) {
    scoped_mode_t mode {name};
    EXPECT_FALSE(advertises_invalidation(video::software)) << name << ": the SDP goes out before the client says who it is";
  }
}

#ifdef _WIN32  // only the Windows NVENC encoder advertises it; the FFmpeg one does not
// @ABR-02-S4
TEST(AbrStockParityTest, NvencAdvertisesInvalidationInEveryMode) {
  for (const auto *name : {"disabled", "fec", "full"}) {
    scoped_mode_t mode {name};
    EXPECT_TRUE(advertises_invalidation(video::nvenc)) << name << ": NVENC advertises it as upstream does";
  }
}
#endif

// Defined in src/video.cpp and not part of its header: the encoder session is driven here the way the
// capture loop does, without a stream around it.
namespace video {
  std::unique_ptr<platf::encode_device_t> make_encode_device(platf::display_t &disp, const encoder_t &encoder, const config_t &config);
  std::unique_ptr<encode_session_t> make_encode_session(platf::display_t *disp, const encoder_t &encoder, const config_t &config, int width, int height, std::unique_ptr<platf::encode_device_t> encode_device);
  int encode(int64_t frame_nr, encode_session_t &session, safe::mail_raw_t::queue_t<packet_t> &packets, stream::session_t *channel_data, std::optional<std::chrono::steady_clock::time_point> frame_timestamp);
}  // namespace video

/**
 * @brief libx264 sessions opened in each adaptive bitrate mode, read back from their SPS.
 */
struct AbrStockParityLibx264Test: PlatformTestSuite {
  void SetUp() override {
    BaseTest::SetUp();
    if (!avcodec_find_encoder_by_name("libx264")) {
      GTEST_SKIP() << "this FFmpeg build has no libx264";
    }
    if (!video::validate_encoder(video::software, false)) {
      FAIL() << "Software encoder not available";
    }
  }

  /**
   * @brief Open a session in `mode`, for a client that set no reference limit.
   *
   * @param mode The adaptive_bitrate setting.
   * @param repairs Whether the session's controller has taken over and repairs losses.
   * @param preset The libx264 preset.
   */
  void open(const char *mode, bool repairs, const char *preset = "superfast") {
    scoped_mode_t scoped {mode};
    const std::string saved_preset = config::video.sw.sw_preset;
    config::video.sw.sw_preset = preset;
    auto restore = util::fail_guard([&]() {
      config::video.sw.sw_preset = saved_preset;
    });
    video::config_t config {1280, 720, 60, 6000, 6000, 1, 0, 1, 0, 0, 0, 0};
    config.adaptiveRepairs = repairs;
    // Release the previous session's capture before opening the next one, in the fixture's own destruction order
    session.reset();
    image.reset();
    display.reset();
    display = platf::display(video::software.platform_formats->dev_type, display_device::map_output_name(config::video.output_name), config);
    ASSERT_TRUE(display) << "no display to capture from";
    auto device = video::make_encode_device(*display, video::software, config);
    ASSERT_TRUE(device);
    session = video::make_encode_session(display.get(), video::software, config, display->width, display->height, std::move(device));
    ASSERT_TRUE(session);
    image = display->alloc_img();
    ASSERT_TRUE(image);
    ASSERT_EQ(display->dummy_img(image.get()), 0);
    packets = mail::man->queue<video::packet_t>(mail::video_packets);
    frame = 0;
  }

  /**
   * @brief Encode one frame and return its packet.
   */
  video::packet_t encode_next(bool idr = false) {
    ++frame;
    if (idr) {
      session->request_idr_frame();
    }
    if (session->convert(*image) || video::encode(frame, *session, packets, nullptr, {})) {
      ADD_FAILURE() << "frame " << frame << " was not encoded";
      return nullptr;
    }
    session->request_normal_frame();
    auto packet = packets->pop(2s);
    if (!packet) {
      ADD_FAILURE() << "frame " << frame << " produced no packet";
    }
    return packet;
  }

  /**
   * @brief The max_num_ref_frames of the SPS the session's first frame carries: the DPB it asks of the decoder.
   *
   * @return The value, or -1 when the packet carries no SPS.
   */
  int sps_reference_frames() {
    auto packet = encode_next(true);
    auto *raw = dynamic_cast<video::packet_raw_avcodec *>(packet.get());
    if (!raw) {
      ADD_FAILURE() << "libx264 packets are avcodec packets";
      return -1;
    }
    CodedBitstreamContext *cbs = nullptr;
    if (ff_cbs_init(&cbs, AV_CODEC_ID_H264, nullptr) < 0) {
      ADD_FAILURE() << "no H.264 bitstream reader";
      return -1;
    }
    CodedBitstreamFragment fragment {};
    int frames = -1;
    if (ff_cbs_read_packet(cbs, &fragment, raw->av_packet) >= 0) {
      if (const auto *sps = static_cast<CodedBitstreamH264Context *>(cbs->priv_data)->active_sps) {
        frames = sps->max_num_ref_frames;
      }
    }
    ff_cbs_fragment_free(&fragment);
    ff_cbs_close(&cbs);
    return frames;
  }

  /**
   * @brief Report frames 2 and 3 lost after five frames, and return the frame that answers it.
   */
  video::packet_t repair_after_loss() {
    for (int i = 0; i < 4; ++i) {
      if (!encode_next()) {
        return nullptr;
      }
    }
    session->invalidate_ref_frames(frame - 2, frame - 1);
    return encode_next();
  }

  std::shared_ptr<platf::display_t> display;
  std::unique_ptr<video::encode_session_t> session;
  std::shared_ptr<platf::img_t> image;
  safe::mail_raw_t::queue_t<video::packet_t> packets;
  int64_t frame = 0;
};

// @ABR-02-S6 @ABR-02-S2
TEST_F(AbrStockParityLibx264Test, WithTheFeatureOffTheEncoderOptionsAreUpstreams) {
  // Even if a session were marked as repaired, the feature being off leaves it alone
  for (bool repairs : {false, true}) {
    open("disabled", repairs);
    EXPECT_EQ(sps_reference_frames(), 1) << "the superfast preset's single reference, as upstream 1582fff opens it";
    auto answer = repair_after_loss();
    ASSERT_TRUE(answer);
    EXPECT_TRUE(answer->is_idr()) << "a loss costs an IDR frame, as upstream";
  }
}

// @ABR-02-S3 @ABR-02-S4 @ABR-02-ASM-02
TEST_F(AbrStockParityLibx264Test, WithoutSignalsTheAdaptiveModesOpenTheEncoderAsUpstream) {
  for (const auto *mode : {"fec", "full"}) {
    open(mode, false);
    EXPECT_EQ(sps_reference_frames(), 1) << mode << ": no references are added before the controller takes over";
  }
}

// @ABR-20-S1 @ABR-20-S3
TEST_F(AbrStockParityLibx264Test, AnyLibx264SessionsLossCostsAnIdrWhateverThePresetModeAndRepairs) {
  for (const auto *preset : {"superfast", "faster"}) {
    for (const auto *mode : {"disabled", "fec", "full"}) {
      for (bool repairs : {false, true}) {
        open(mode, repairs, preset);
        if (std::string_view {preset} == "faster") {
          const int refs = sps_reference_frames();
          EXPECT_GE(refs, 2) << "the preset itself asks for two references or more";
          EXPECT_LT(refs, 16) << mode << ' ' << repairs << ": no DPB of 16 for repairs";
        } else {
          EXPECT_EQ(sps_reference_frames(), 1) << mode << ' ' << repairs << ": the superfast preset's single reference, no DPB of 16";
        }
        auto answer = repair_after_loss();
        ASSERT_TRUE(answer) << preset << ' ' << mode << ' ' << repairs;
        EXPECT_TRUE(answer->is_idr()) << preset << ' ' << mode << ' ' << repairs << ": a loss costs an IDR frame, as upstream";
        EXPECT_FALSE(answer->after_ref_frame_invalidation) << preset << ' ' << mode << ' ' << repairs;
      }
    }
  }
}
