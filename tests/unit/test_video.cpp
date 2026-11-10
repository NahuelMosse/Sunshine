/**
 * @file tests/unit/test_video.cpp
 * @brief Test src/video.*.
 */
// test includes
#include "../tests_common.h"

// standard includes
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

// ffmpeg includes
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
}

// local includes
#include <src/config.h>
#include <src/display_device.h>
#include <src/nvenc/nvenc_encoder.h>
#include <src/platform/common.h>
#include <src/video.h>

using namespace std::literals;

/**
 * @brief Packet replacement rules retain the original non-owning byte views.
 */
TEST(VideoPacketReplacementTest, CopiesViewsWithoutCopyingTheirBackingBytes) {
  std::string original = "__old__";
  std::string replacement = "__new__";
  const video::packet_raw_t::replace_t rule {std::string_view(original).substr(2, 3), std::string_view(replacement).substr(2, 3)};
  EXPECT_EQ(rule.old, "old");
  EXPECT_EQ(rule._new, "new");
  EXPECT_EQ(rule.old.data(), original.data() + 2);
  EXPECT_EQ(rule._new.data(), replacement.data() + 2);
  original[2] = 'O';
  replacement[2] = 'N';
  EXPECT_EQ(rule.old, "Old");
  EXPECT_EQ(rule._new, "New");
}

TEST(VideoFormatMappingTest, MapsSupportedHardwareDevicesAndRejectsUnsupportedOnes) {
  using enum platf::mem_type_e;
  const std::array cases {
    std::pair {AV_HWDEVICE_TYPE_D3D11VA, dxgi},
    std::pair {AV_HWDEVICE_TYPE_VAAPI, vaapi},
    std::pair {AV_HWDEVICE_TYPE_CUDA, cuda},
    std::pair {AV_HWDEVICE_TYPE_NONE, system},
    std::pair {AV_HWDEVICE_TYPE_VIDEOTOOLBOX, videotoolbox},
#ifdef SUNSHINE_BUILD_VULKAN
    std::pair {AV_HWDEVICE_TYPE_VULKAN, vulkan},
#else
    std::pair {AV_HWDEVICE_TYPE_VULKAN, unknown},
#endif
    std::pair {AV_HWDEVICE_TYPE_DXVA2, unknown},
  };

  for (const auto &[format, expected] : cases) {
    EXPECT_EQ(video::map_base_dev_type(format), expected) << format;
  }
}

TEST(VideoFormatMappingTest, MapsSupportedPixelFormatsAndRejectsUnsupportedOnes) {
  using enum platf::pix_fmt_e;
  const std::array cases {
    std::pair {AV_PIX_FMT_VUYX, ayuv},
    std::pair {AV_PIX_FMT_XV30, y410},
    std::pair {AV_PIX_FMT_YUV420P10, yuv420p10},
    std::pair {AV_PIX_FMT_YUV420P, yuv420p},
    std::pair {AV_PIX_FMT_NV12, nv12},
    std::pair {AV_PIX_FMT_P010, p010},
    std::pair {AV_PIX_FMT_YUV444P, yuv444p},
    std::pair {AV_PIX_FMT_YUV444P16, yuv444p16},
    std::pair {AV_PIX_FMT_NONE, unknown},
    std::pair {AV_PIX_FMT_RGB24, unknown},
  };

  for (const auto &[format, expected] : cases) {
    EXPECT_EQ(video::map_pix_fmt(format), expected) << format;
  }
}

struct EncoderTest: PlatformTestSuite, testing::WithParamInterface<video::encoder_t *> {
  void SetUp() override {
    BaseTest::SetUp();
    auto &encoder = *GetParam();
    if (!video::validate_encoder(encoder, false)) {
      // Encoder failed validation,
      // if it's software - fail, otherwise skip
      if (encoder.name == "software") {
        FAIL() << "Software encoder not available";
      } else {
        GTEST_SKIP() << "Encoder not available";
      }
    }
  }
};

/**
 * @brief Available encoder backends for this platform.
 */
const auto encoder_variants = std::to_array<video::encoder_t *>({
#if !defined(__APPLE__)
  &video::nvenc,
#endif
#ifdef _WIN32
  &video::amdvce,
  &video::quicksync,
#endif
#if defined(__linux__) || defined(__FreeBSD__)
  &video::vaapi,
#endif
#ifdef __APPLE__
  &video::videotoolbox,
#endif
  &video::software
});

INSTANTIATE_TEST_SUITE_P(
  EncoderVariants,
  EncoderTest,
  testing::ValuesIn(encoder_variants),
  [](const auto &info) {
    return std::string(info.param->name);
  }
);

#if !defined(_WIN32) && !defined(__APPLE__)
TEST(NvencAvcodecOptionsTest, UsesVersionStablePresetAndSpatialAqNames) {
  const std::array codecs {
    &video::nvenc.av1,
    &video::nvenc.hevc,
    &video::nvenc.h264,
  };

  for (const auto *codec : codecs) {
    const auto preset = std::ranges::find(codec->common_options, "preset"sv, &video::encoder_t::option_t::name);
    ASSERT_NE(codec->common_options.end(), preset);
    ASSERT_TRUE(std::holds_alternative<std::string *>(preset->value));
    EXPECT_EQ(&config::video.nv_legacy.preset, std::get<std::string *>(preset->value));

    const auto spatial_aq = std::ranges::find(codec->common_options, "spatial-aq"sv, &video::encoder_t::option_t::name);
    ASSERT_NE(codec->common_options.end(), spatial_aq);
    ASSERT_TRUE(std::holds_alternative<int *>(spatial_aq->value));
    EXPECT_EQ(&config::video.nv_legacy.spatial_aq, std::get<int *>(spatial_aq->value));

    EXPECT_EQ(
      codec->common_options.end(),
      std::ranges::find(codec->common_options, "aq"sv, &video::encoder_t::option_t::name)
    );
  }
}
#endif

TEST_P(EncoderTest, ValidateEncoder) {
  // todo:: test something besides fixture setup
}

/**
 * @brief Parameterized coverage for effective H.264 profile selection.
 */
struct H264ProfileTest: testing::TestWithParam<std::tuple<std::string_view, video::amf::coder_e, int, int>> {};

TEST_P(H264ProfileTest, SelectProfile) {
  const auto &[encoder_name, coder, chroma_sampling_type, expected_profile] = GetParam();
  video::config_t config {};
  config.chromaSamplingType = chroma_sampling_type;

  EXPECT_EQ(expected_profile, video::select_h264_profile(encoder_name, config, std::to_underlying(coder)));
}

INSTANTIATE_TEST_SUITE_P(
  H264ProfileTests,
  H264ProfileTest,
  testing::Values(
    std::make_tuple("h264_amf"sv, video::amf::coder_e::auto_, 0, AV_PROFILE_H264_HIGH),
    std::make_tuple("h264_amf"sv, video::amf::coder_e::cabac, 0, AV_PROFILE_H264_HIGH),
    std::make_tuple("h264_amf"sv, video::amf::coder_e::cavlc, 0, AV_PROFILE_H264_CONSTRAINED_BASELINE),
    std::make_tuple("h264_amf"sv, video::amf::coder_e::cavlc, 1, AV_PROFILE_H264_HIGH_444_PREDICTIVE),
    std::make_tuple("h264_nvenc"sv, video::amf::coder_e::cavlc, 0, AV_PROFILE_H264_HIGH)
  )
);

/**
 * @brief Parameterized coverage for resolving requested dynamic range against encoder capabilities.
 */
struct DynamicRangeTest: testing::TestWithParam<std::tuple<int, int, int, bool, bool, int>> {};

TEST_P(DynamicRangeTest, Resolve) {
  const auto &[video_format, chroma_sampling_type, requested_dynamic_range, supports_hdr, supports_hdr_yuv444, expected_dynamic_range] = GetParam();

  video::encoder_t encoder {
    "test"sv,
    {},
    {},
    {},
    {},
    0,
  };
  encoder.h264.name = "h264_test";
  encoder.hevc.name = "hevc_test";
  encoder.av1.name = "av1_test";

  video::config_t config {};
  config.videoFormat = video_format;
  config.dynamicRange = requested_dynamic_range;
  config.chromaSamplingType = chroma_sampling_type;

  auto *codec = &encoder.h264;
  if (video_format == 1) {
    codec = &encoder.hevc;
  } else if (video_format == 2) {
    codec = &encoder.av1;
  }
  (*codec)[video::encoder_t::DYNAMIC_RANGE] = supports_hdr;
  (*codec)[video::encoder_t::DYNAMIC_RANGE_YUV444] = supports_hdr_yuv444;

  const auto effective_config = video::resolve_dynamic_range(encoder, config);

  EXPECT_EQ(expected_dynamic_range, effective_config.dynamicRange);
  EXPECT_EQ(requested_dynamic_range, config.dynamicRange);
  EXPECT_EQ(video_format, effective_config.videoFormat);
  EXPECT_EQ(chroma_sampling_type, effective_config.chromaSamplingType);
}

INSTANTIATE_TEST_SUITE_P(
  DynamicRangeTests,
  DynamicRangeTest,
  testing::Values(
    std::make_tuple(0, 0, 1, false, true, 0),
    std::make_tuple(1, 0, 1, false, true, 0),
    std::make_tuple(1, 0, 1, true, false, 1),
    std::make_tuple(2, 1, 1, true, false, 0),
    std::make_tuple(2, 1, 1, false, true, 1),
    std::make_tuple(1, 0, 0, false, false, 0)
  )
);

/**
 * @brief Reset the published probe results before and after each test.
 */
struct EncoderProbeResultsTest: testing::Test {
  /**
   * @brief Start from an empty published state.
   */
  void SetUp() override {
    video::reset_encoder_probe_results();
  }

  /**
   * @brief Leave no published state behind for other tests.
   */
  void TearDown() override {
    video::reset_encoder_probe_results();
  }
};

TEST_F(EncoderProbeResultsTest, FailedCodecProbeDoesNotPublishCapabilities) {
  video::encoder_t encoder {"test"sv, {}, {}, {}, {}, 0};
  for (auto *codec : {&encoder.h264, &encoder.hevc, &encoder.av1}) {
    (*codec)[video::encoder_t::PASSED] = false;
    (*codec)[video::encoder_t::YUV444] = true;
    (*codec)[video::encoder_t::DYNAMIC_RANGE] = true;
    (*codec)[video::encoder_t::DYNAMIC_RANGE_YUV444] = true;
  }

  video::record_encoder_probe_results(encoder);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_FALSE(video::last_encoder_probe_supported_yuv444_for_codec[i]) << i;
    EXPECT_FALSE(video::last_encoder_probe_supported_dynamic_range_for_codec[i]) << i;
    EXPECT_FALSE(video::last_encoder_probe_supported_dynamic_range_yuv444_for_codec[i]) << i;
  }
}

TEST_F(EncoderProbeResultsTest, SubsequentProbeReplacesPriorResults) {
  constexpr uint32_t ref_frames_invalidation = 1 << 8;  // Mirrors REF_FRAMES_INVALIDATION in video.cpp.
  video::encoder_t first {"first"sv, {}, {}, {}, {}, ref_frames_invalidation};
  for (auto *codec : {&first.h264, &first.hevc, &first.av1}) {
    (*codec)[video::encoder_t::PASSED] = true;
    (*codec)[video::encoder_t::YUV444] = true;
    (*codec)[video::encoder_t::DYNAMIC_RANGE] = true;
    (*codec)[video::encoder_t::DYNAMIC_RANGE_YUV444] = true;
  }

  // The second encoder only supports plain 8-bit 4:2:0 H.264.
  video::encoder_t second {"second"sv, {}, {}, {}, {}, 0};
  second.h264[video::encoder_t::PASSED] = true;

  video::record_encoder_probe_results(first);
  ASSERT_TRUE(video::last_encoder_probe_supported_ref_frames_invalidation);
  ASSERT_TRUE(video::last_encoder_probe_supported_yuv444_for_codec[1]);
  ASSERT_TRUE(video::last_encoder_probe_supported_dynamic_range_for_codec[2]);
  ASSERT_TRUE(video::last_encoder_probe_supported_dynamic_range_yuv444_for_codec[2]);

  video::record_encoder_probe_results(second);

  EXPECT_FALSE(video::last_encoder_probe_supported_ref_frames_invalidation);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_FALSE(video::last_encoder_probe_supported_yuv444_for_codec[i]) << i;
    EXPECT_FALSE(video::last_encoder_probe_supported_dynamic_range_for_codec[i]) << i;
    EXPECT_FALSE(video::last_encoder_probe_supported_dynamic_range_yuv444_for_codec[i]) << i;
  }
}

#ifdef _WIN32
TEST(AmfH264OptionsTest, CoderUsesConfiguredValue) {
  const auto coder_option = std::ranges::find(video::amdvce.h264.common_options, "coder"sv, &video::encoder_t::option_t::name);

  ASSERT_NE(video::amdvce.h264.common_options.end(), coder_option);
  ASSERT_TRUE(std::holds_alternative<int *>(coder_option->value));
  EXPECT_EQ(&config::video.amd.amd_coder, std::get<int *>(coder_option->value));
}

/**
 * @brief Parameterized coverage for the AMF maximum access-unit-size option mappings.
 */
struct AmfMaxAuSizeOptionsTest: testing::TestWithParam<std::tuple<const video::encoder_t::codec_t *, bool>> {};

TEST_P(AmfMaxAuSizeOptionsTest, UsesConfiguredValueForSupportedCodecsOnly) {
  const auto &[codec, supported] = GetParam();
  const auto option = std::ranges::find(codec->common_options, "max_au_size"sv, &video::encoder_t::option_t::name);

  if (!supported) {
    EXPECT_EQ(codec->common_options.end(), option);
    return;
  }

  ASSERT_NE(codec->common_options.end(), option);
  ASSERT_TRUE(std::holds_alternative<std::optional<int> *>(option->value));
  EXPECT_EQ(&config::video.amd.amd_max_au_size, std::get<std::optional<int> *>(option->value));
}

INSTANTIATE_TEST_SUITE_P(
  AmfCodecOptions,
  AmfMaxAuSizeOptionsTest,
  testing::Values(
    std::make_tuple(&video::amdvce.h264, true),
    std::make_tuple(&video::amdvce.hevc, true),
    std::make_tuple(&video::amdvce.av1, false)
  )
);
#endif

using AmfMaxAuSizeConfigParam = std::tuple<std::string_view, std::optional<int>>;

/**
 * @brief Parameterized coverage for parsing and validating the AMF maximum access-unit size.
 */
struct AmfMaxAuSizeConfigTest: BaseTest, testing::WithParamInterface<AmfMaxAuSizeConfigParam> {
  void SetUp() override {
    BaseTest::SetUp();
    config::video.amd.amd_max_au_size.reset();
    config::stream.file_apps = SUNSHINE_SOURCE_DIR "/tests/unit/test_video.cpp";
  }

  void TearDown() override {
    config::video = original_video;
    config::audio = original_audio;
    config::stream = original_stream;
    config::nvhttp = original_nvhttp;
    config::input = original_input;
    config::sunshine = original_sunshine;
    config::modified_config_settings = original_modified_config_settings;
    BaseTest::TearDown();
  }

  config::video_t original_video {config::video};  ///< Video configuration restored after each parameterized test.
  config::audio_t original_audio {config::audio};  ///< Audio configuration restored after each parameterized test.
  config::stream_t original_stream {config::stream};  ///< Stream configuration restored after each parameterized test.
  config::nvhttp_t original_nvhttp {config::nvhttp};  ///< HTTP configuration restored after each parameterized test.
  config::input_t original_input {config::input};  ///< Input configuration restored after each parameterized test.
  config::sunshine_t original_sunshine {config::sunshine};  ///< Core configuration restored after each parameterized test.
  decltype(config::modified_config_settings) original_modified_config_settings {config::modified_config_settings};  ///< Modified settings restored after each parameterized test.
};

TEST_P(AmfMaxAuSizeConfigTest, AcceptsOnlyFfmpegSupportedRange) {
  const auto &[setting, expected] = GetParam();
  config::apply_config_for_test(setting);

  EXPECT_EQ(expected, config::video.amd.amd_max_au_size);
}

INSTANTIATE_TEST_SUITE_P(
  AmfMaxAuSizeValues,
  AmfMaxAuSizeConfigTest,
  testing::Values(
    AmfMaxAuSizeConfigParam {""sv, std::nullopt},
    AmfMaxAuSizeConfigParam {"amd_max_au_size = -2\n"sv, std::nullopt},
    AmfMaxAuSizeConfigParam {"amd_max_au_size = -1\n"sv, -1},
    AmfMaxAuSizeConfigParam {"amd_max_au_size = 0\n"sv, 0},
    AmfMaxAuSizeConfigParam {"amd_max_au_size = 800000\n"sv, 800000},
    AmfMaxAuSizeConfigParam {"amd_max_au_size = 2147483647\n"sv, std::numeric_limits<int>::max()}
  )
);

struct FramerateX100Test: BaseTest, testing::WithParamInterface<std::tuple<std::int32_t, AVRational>> {};

TEST_P(FramerateX100Test, Run) {
  const auto &[x100, expected] = GetParam();
  auto res = video::framerateX100_to_rational(x100);
  ASSERT_EQ(0, av_cmp_q(res, expected)) << "expected "
                                        << expected.num << "/" << expected.den
                                        << ", got "
                                        << res.num << "/" << res.den;
}

INSTANTIATE_TEST_SUITE_P(
  FramerateX100Tests,
  FramerateX100Test,
  testing::Values(
    std::make_tuple(2397, AVRational {24000, 1001}),
    std::make_tuple(2398, AVRational {24000, 1001}),
    std::make_tuple(2500, AVRational {25, 1}),
    std::make_tuple(2997, AVRational {30000, 1001}),
    std::make_tuple(3000, AVRational {30, 1}),
    std::make_tuple(5994, AVRational {60000, 1001}),
    std::make_tuple(6000, AVRational {60, 1}),
    std::make_tuple(11988, AVRational {120000, 1001}),
    std::make_tuple(23976, AVRational {240000, 1001}),  // future NTSC 240hz?
    std::make_tuple(9498, AVRational {4749, 50})  // from my LG 27GN950
  )
);

struct FramerateToRationalTest: testing::TestWithParam<std::tuple<int, int, AVRational>> {};

TEST_P(FramerateToRationalTest, Run) {
  const auto &[framerate, framerateX100, expected] = GetParam();
  video::config_t config {};
  config.framerate = framerate;
  config.framerateX100 = framerateX100;
  auto res = video::framerate_to_rational(config);
  ASSERT_EQ(0, av_cmp_q(res, expected)) << "expected "
                                        << expected.num << "/" << expected.den
                                        << ", got "
                                        << res.num << "/" << res.den;
}

INSTANTIATE_TEST_SUITE_P(
  FramerateToRationalTests,
  FramerateToRationalTest,
  testing::Values(
    std::make_tuple(60, 0, AVRational {60, 1}),  // no X100 value, fall back to integer framerate
    std::make_tuple(60, 5994, AVRational {60000, 1001}),
    std::make_tuple(120, 11988, AVRational {120000, 1001}),
    std::make_tuple(24, 2398, AVRational {24000, 1001})
  )
);

struct CaptureFrameIntervalTest: testing::TestWithParam<std::tuple<int, int, std::chrono::nanoseconds>> {};

TEST_P(CaptureFrameIntervalTest, Run) {
  const auto &[framerate, framerateX100, expected] = GetParam();
  video::config_t config {};
  config.framerate = framerate;
  config.framerateX100 = framerateX100;
  ASSERT_EQ(expected, video::capture_frame_interval(config));
}

INSTANTIATE_TEST_SUITE_P(
  CaptureFrameIntervalTests,
  CaptureFrameIntervalTest,
  testing::Values(
    std::make_tuple(60, 0, std::chrono::nanoseconds {16666666}),
    std::make_tuple(60, 5994, std::chrono::nanoseconds {16683333}),  // 1e9 * 1001 / 60000
    std::make_tuple(120, 11988, std::chrono::nanoseconds {8341666})  // 1e9 * 1001 / 120000
  )
);

/**
 * @brief Software encoder converts BGR0 and NV12 frames, including padded strides and
 *        backends that don't report the pixel pitch.
 */
TEST(SoftwareEncoderConversion, Bgr0AndNv12) {
  constexpr int w = 320;
  constexpr int h = 240;

  AVFrame *frame = av_frame_alloc();
  ASSERT_NE(frame, nullptr);
  frame->width = w;
  frame->height = h;
  frame->format = AV_PIX_FMT_YUV420P;

  video::avcodec_software_encode_device_t device;
  ASSERT_EQ(device.init(w, h, frame, AV_PIX_FMT_YUV420P, false), 0);
  // set_frame() takes ownership of the frame; the device frees it on destruction.
  ASSERT_EQ(device.set_frame(frame, nullptr), 0);

  // BGR0 frame (4 bytes per pixel) -- the classic KMS/DMABUF capture layout.
  std::vector<uint8_t> bgr0_buffer(static_cast<size_t>(w) * h * 4);
  platf::img_t bgr0_img {};
  bgr0_img.data = bgr0_buffer.data();
  bgr0_img.width = w;
  bgr0_img.height = h;
  bgr0_img.row_pitch = w * 4;
  bgr0_img.pixel_pitch = 4;
  EXPECT_EQ(device.convert(bgr0_img), 0);

  // NV12 frame (1 byte per pixel row pitch, Y plane + interleaved UV) -- the
  // layout delivered by PipeWire-based captures (KWin screencast / portal).
  std::vector<uint8_t> nv12_buffer(static_cast<size_t>(w) * h + static_cast<size_t>(w) * h / 2);
  platf::img_t nv12_img {};
  nv12_img.data = nv12_buffer.data();
  nv12_img.width = w;
  nv12_img.height = h;
  nv12_img.row_pitch = w;
  nv12_img.pixel_pitch = 1;
  EXPECT_EQ(device.convert(nv12_img), 0);

  // Padded-stride NV12 (alignment padding) -- the case the old row_pitch ==
  // width heuristic misdetected as BGR0, causing out-of-bounds reads.
  constexpr int padded_stride = w + 32;
  std::vector<uint8_t> padded_nv12_buffer(static_cast<size_t>(padded_stride) * h + static_cast<size_t>(padded_stride) * h / 2);
  platf::img_t padded_nv12_img {};
  padded_nv12_img.data = padded_nv12_buffer.data();
  padded_nv12_img.width = w;
  padded_nv12_img.height = h;
  padded_nv12_img.row_pitch = padded_stride;
  padded_nv12_img.pixel_pitch = 1;
  EXPECT_EQ(device.convert(padded_nv12_img), 0);

  // Capture backends that don't report pixel_pitch fall back to deriving it
  // from the row pitch (1 byte per pixel = NV12, 4 = BGR0).
  platf::img_t fallback_bgr0_img {};
  fallback_bgr0_img.data = bgr0_buffer.data();
  fallback_bgr0_img.width = w;
  fallback_bgr0_img.height = h;
  fallback_bgr0_img.row_pitch = w * 4;
  EXPECT_EQ(device.convert(fallback_bgr0_img), 0);

  platf::img_t fallback_nv12_img {};
  fallback_nv12_img.data = nv12_buffer.data();
  fallback_nv12_img.width = w;
  fallback_nv12_img.height = h;
  fallback_nv12_img.row_pitch = w;
  EXPECT_EQ(device.convert(fallback_nv12_img), 0);
}

// Defined in src/video.cpp and not part of its header: the encoder session is driven here the way the
// capture loop does, without a stream around it.
namespace video {
  std::unique_ptr<platf::encode_device_t> make_encode_device(platf::display_t &disp, const encoder_t &encoder, const config_t &config);
  std::unique_ptr<encode_session_t> make_encode_session(platf::display_t *disp, const encoder_t &encoder, const config_t &config, int width, int height, std::unique_ptr<platf::encode_device_t> encode_device);
  int encode(int64_t frame_nr, encode_session_t &session, safe::mail_raw_t::queue_t<packet_t> &packets, stream::session_t *channel_data, std::optional<std::chrono::steady_clock::time_point> frame_timestamp);
}  // namespace video

/**
 * @brief libx264 sessions, encoded frame by frame and decoded again.
 */
struct Libx264SessionTest: PlatformTestSuite {
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
   * @brief Open a software session with `refs` reference frames, libx264 unless `video_format` says otherwise.
   */
  void open(int refs, int video_format = 0) {
    const video::config_t config {1280, 720, 60, 6000, 6000, 1, refs, 1, video_format, 0, 0, 0};
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

    // An FFmpeg built without decoders still gets every frame type check
    if (const auto *h264 = avcodec_find_decoder(AV_CODEC_ID_H264)) {
      decoder.reset(avcodec_alloc_context3(h264));
      ASSERT_TRUE(decoder);
      decoder->flags |= AV_CODEC_FLAG_LOW_DELAY;
      decoder->thread_count = 1;
      ASSERT_EQ(avcodec_open2(decoder.get(), nullptr, nullptr), 0);
    }
  }

  /**
   * @brief Count the frames the decoder has ready.
   */
  void drain_decoder() {
    std::unique_ptr<AVFrame, void (*)(AVFrame *)> out {av_frame_alloc(), [](AVFrame *f) {
                                                         av_frame_free(&f);
                                                       }};
    while (avcodec_receive_frame(decoder.get(), out.get()) == 0) {
      ++decoded;
    }
  }

  /**
   * @brief Encode one frame and decode what comes out.
   *
   * @return The packet, which is the encoder's answer for that frame.
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
    // The capture loop goes back to plain frames after each one; requests come in between
    session->request_normal_frame();
    auto packet = packets->pop(2s);
    if (!packet) {
      ADD_FAILURE() << "frame " << frame << " produced no packet";
      return nullptr;
    }
    auto *raw = dynamic_cast<video::packet_raw_avcodec *>(packet.get());
    if (!raw) {
      ADD_FAILURE() << "libx264 packets are avcodec packets";
      return nullptr;
    }
    EXPECT_EQ(packet->frame_index(), frame);
    if (decoder) {
      if (avcodec_send_packet(decoder.get(), raw->av_packet) < 0) {
        ADD_FAILURE() << "frame " << frame << " does not decode";
      }
      drain_decoder();
    }
    return packet;
  }

  std::shared_ptr<platf::display_t> display;
  std::unique_ptr<video::encode_session_t> session;
  std::shared_ptr<platf::img_t> image;
  safe::mail_raw_t::queue_t<video::packet_t> packets;
  std::unique_ptr<AVCodecContext, void (*)(AVCodecContext *)> decoder {nullptr, [](AVCodecContext *c) {
                                                                         avcodec_free_context(&c);
                                                                       }};
  int64_t frame = 0;
  int decoded = 0;
};

TEST_F(Libx264SessionTest, ChangesTheBitrateOfARunningSession) {
  open(0);
  EXPECT_EQ(session->opened_bitrate_kbps(), 6000);
  EXPECT_FALSE(session->set_bitrate(0));
  EXPECT_FALSE(session->set_bitrate(-10));
  EXPECT_TRUE(session->set_bitrate(6000)) << "already what it has";
  EXPECT_TRUE(session->set_bitrate(3000));
  EXPECT_EQ(session->opened_bitrate_kbps(), 3000);
  EXPECT_TRUE(session->set_bitrate(3000));

  // The stream goes on decoding across the change, in both directions
  auto first = encode_next(true);
  ASSERT_TRUE(first);
  EXPECT_TRUE(first->is_idr());
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(encode_next());
  }
  EXPECT_TRUE(session->set_bitrate(9000));
  for (int i = 0; i < 3; ++i) {
    auto packet = encode_next();
    ASSERT_TRUE(packet);
    EXPECT_FALSE(packet->is_idr()) << "changing the bitrate must not cost a keyframe";
  }
  // The client decoded every frame it was sent
  if (decoder) {
    avcodec_send_packet(decoder.get(), nullptr);
    drain_decoder();
    EXPECT_EQ(decoded, frame);
  }
}

// @ABR-01-S2 @ABR-01-S9 @ABR-18-S9
TEST_F(Libx264SessionTest, ReportsNoRepairReachWithoutLongTermReferences) {
  open(0);
  EXPECT_EQ(session->repair_reach_frames(), 0);
}

namespace {
  /**
   * @brief An NVENC encoder that takes a new bitrate and says which one it runs at.
   */
  struct adjustable_nvenc_t: nvenc::nvenc_encoder {
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

    bool set_bitrate(std::uint32_t kbps) override {
      current = kbps;
      return true;
    }

    std::uint32_t bitrate_kbps() const override {
      return current;
    }

    std::uint32_t current = 8000;
  };

  /**
   * @brief The platform side of an NVENC session, around a given encoder.
   */
  struct adjustable_nvenc_device_t: platf::nvenc_encode_device_t {
    explicit adjustable_nvenc_device_t(nvenc::nvenc_encoder *encoder) {
      nvenc = encoder;
    }

    bool init_encoder(const video::config_t &, const video::sunshine_colorspace_t &) override {
      return true;
    }

    int convert(platf::img_t &) override {
      return 0;
    }
  };
}  // namespace

TEST(NvencSessionBitrateTest, ForwardsTheBitrateToTheEncoder) {
  adjustable_nvenc_t encoder;
  const video::config_t config {1280, 720, 60, 8000, 8000, 1, 0, 1, 0, 0, 0, 0};
  auto session = video::make_encode_session(nullptr, video::software, config, 1280, 720, std::make_unique<adjustable_nvenc_device_t>(&encoder));
  ASSERT_TRUE(session);
  EXPECT_EQ(session->opened_bitrate_kbps(), 8000);
  EXPECT_FALSE(session->set_bitrate(0));
  EXPECT_FALSE(session->set_bitrate(-10));
  EXPECT_EQ(encoder.current, 8000U) << "a bitrate that is not positive never reaches the encoder";
  EXPECT_TRUE(session->set_bitrate(5000));
  EXPECT_EQ(encoder.current, 5000U);
  EXPECT_EQ(session->opened_bitrate_kbps(), 5000);
}

namespace {
  /**
   * @brief An NVENC encoder with only what every NVENC encoder has.
   */
  struct plain_nvenc_t: nvenc::nvenc_encoder {
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
  };

  /**
   * @brief An encode session with only what every encode session has.
   */
  struct plain_session_t: video::encode_session_t {
    int convert(platf::img_t &) override {
      return 0;
    }

    void request_idr_frame() override {
    }

    void request_normal_frame() override {
    }

    void invalidate_ref_frames(int64_t, int64_t) override {
    }
  };
}  // namespace

TEST(NvencSessionBitrateTest, AnEncoderThatCannotChangeItRefusesANewBitrate) {
  plain_nvenc_t encoder;
  const video::config_t config {1280, 720, 60, 8000, 8000, 1, 0, 1, 0, 0, 0, 0};
  auto session = video::make_encode_session(nullptr, video::software, config, 1280, 720, std::make_unique<adjustable_nvenc_device_t>(&encoder));
  ASSERT_TRUE(session);
  EXPECT_FALSE(session->set_bitrate(5000));
  EXPECT_EQ(session->opened_bitrate_kbps(), 0) << "it does not say which bitrate it runs at";
}

TEST(EncodeSessionBitrateTest, ASessionThatCannotChangeItRefusesANewBitrate) {
  plain_session_t session;
  EXPECT_FALSE(session.set_bitrate(5000));
  EXPECT_EQ(session.opened_bitrate_kbps(), 0) << "it does not say which bitrate it runs at";
}

TEST_F(Libx264SessionTest, AnotherSoftwareEncoderKeepsTheBitrateItWasOpenedWith) {
  if (!avcodec_find_encoder_by_name(video::software.hevc.name.c_str())) {
    GTEST_SKIP() << "this FFmpeg build has no " << video::software.hevc.name;
  }
  // HEVC is not probed for the software encoder unless the user enables it: as if they had
  const bool probed = video::software.hevc[video::encoder_t::PASSED];
  video::software.hevc[video::encoder_t::PASSED] = true;
  open(0, 1);
  video::software.hevc[video::encoder_t::PASSED] = probed;
  if (HasFatalFailure()) {
    return;
  }

  EXPECT_FALSE(session->set_bitrate(3000)) << "only libx264 picks up a new bitrate between frames";
  EXPECT_EQ(session->opened_bitrate_kbps(), 6000);
}

TEST_F(Libx264SessionTest, AnAcknowledgementChangesNothingWithoutLongTermReferences) {
  open(0);
  auto first = encode_next(true);
  ASSERT_TRUE(first);
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(encode_next());
  }

  // The client acknowledges a frame as if it carried a long-term reference: libx264 marks none
  session->acknowledge_ltr(2);
  auto next = encode_next();
  ASSERT_TRUE(next);
  EXPECT_FALSE(next->is_idr());
  EXPECT_FALSE(next->after_ref_frame_invalidation);
  if (decoder) {
    avcodec_send_packet(decoder.get(), nullptr);
    drain_decoder();
    EXPECT_EQ(decoded, frame);
  }
}

namespace {
  /**
   * @brief An NVENC encoder with long-term references: it marks every 30th frame, takes the
   * client's acknowledgements, repairs a loss without an IDR frame and says what it was asked, in order.
   */
  struct repairing_nvenc_t: adjustable_nvenc_t {
    nvenc::nvenc_encoded_frame encode_frame(std::uint64_t frame_index, bool force_idr) override {
      calls.push_back("encode " + std::to_string(frame_index));
      nvenc::nvenc_encoded_frame frame;
      frame.data = {0, 0, 0, 1, 0x65};
      frame.frame_index = frame_index;
      frame.idr = force_idr;
      frame.ltr_frame = frame_index % 30 == 0;
      return frame;
    }

    bool invalidate_ref_frames(std::uint64_t first_frame, std::uint64_t last_frame) override {
      calls.push_back("invalidate " + std::to_string(first_frame) + "-" + std::to_string(last_frame));
      return true;
    }

    void acknowledge_ltr(std::uint32_t frame_index) override {
      calls.push_back("acknowledge " + std::to_string(frame_index));
    }

    std::uint32_t repair_reach_frames() const override {
      return 30;
    }

    std::vector<std::string> calls;
  };
}  // namespace

TEST(NvencSessionLtrTest, AnEncoderWithoutLongTermReferencesRepairsNothingFromThem) {
  plain_nvenc_t encoder;
  const video::config_t config {1280, 720, 60, 8000, 8000, 1, 0, 1, 0, 0, 0, 0};
  auto session = video::make_encode_session(nullptr, video::software, config, 1280, 720, std::make_unique<adjustable_nvenc_device_t>(&encoder));
  ASSERT_TRUE(session);
  session->acknowledge_ltr(30);
  EXPECT_EQ(session->repair_reach_frames(), 0);
}

TEST(NvencSessionLtrTest, TheSessionCarriesTheMarkAndTheAcknowledgementsOfTheEncoder) {
  repairing_nvenc_t encoder;
  const video::config_t config {1280, 720, 60, 8000, 8000, 1, 0, 1, 0, 0, 0, 0};
  auto session = video::make_encode_session(nullptr, video::software, config, 1280, 720, std::make_unique<adjustable_nvenc_device_t>(&encoder));
  ASSERT_TRUE(session);
  EXPECT_EQ(session->repair_reach_frames(), 30);

  auto packets = mail::man->queue<video::packet_t>(mail::video_packets);
  while (packets->peek()) {
    packets->pop();
  }
  ASSERT_EQ(video::encode(30, *session, packets, nullptr, {}), 0);
  ASSERT_EQ(video::encode(31, *session, packets, nullptr, {}), 0);
  auto marked = packets->pop(2s);
  ASSERT_TRUE(marked);
  EXPECT_EQ(marked->frame_index(), 30);
  EXPECT_TRUE(marked->ltr_frame) << "the packet tells the stream which frames the client acknowledges";
  auto plain = packets->pop(2s);
  ASSERT_TRUE(plain);
  EXPECT_FALSE(plain->ltr_frame);

  session->acknowledge_ltr(30);
  EXPECT_EQ(encoder.calls, (std::vector<std::string> {"encode 30", "encode 31", "acknowledge 30"}));
}

namespace video {
  void encode_run(int &frame_nr, safe::mail_t mail, img_event_t images, config_t config, std::shared_ptr<platf::display_t> disp, std::unique_ptr<platf::encode_device_t> encode_device, safe::signal_t &reinit_event, const encoder_t &encoder, stream::session_t *channel_data);
}  // namespace video

namespace {
  /**
   * @brief A display the encode loop takes blank images from; the images it encodes come from the test.
   */
  struct blank_display_t: platf::display_t {
    blank_display_t() {
      width = 1280;
      height = 720;
    }

    platf::capture_e capture(const push_captured_image_cb_t &, const pull_free_image_cb_t &, bool *) override {
      return platf::capture_e::ok;
    }

    std::shared_ptr<platf::img_t> alloc_img() override {
      return std::make_shared<platf::img_t>();
    }

    int dummy_img(platf::img_t *) override {
      return 0;
    }
  };
}  // namespace

/**
 * @brief The encode loop of a stream, on its own thread, around an NVENC encoder that needs no GPU.
 */
struct EncodeLoopTest: testing::Test {
  void SetUp() override {
    saved_fps_target = config::video.minimum_fps_target;
    // No frame goes out again for lack of a newer image unless a test asks for it
    config::video.minimum_fps_target = 0.1;
    packets = mail::man->queue<video::packet_t>(mail::video_packets);
    while (packets->peek()) {
      packets->pop();
    }
  }

  void TearDown() override {
    stop();
    config::video.minimum_fps_target = saved_fps_target;
  }

  void start() {
    loop = std::thread {[this] {
      video::encode_run(frame_nr, mail, images, config, display, std::make_unique<adjustable_nvenc_device_t>(&encoder), reinit, video::software, nullptr);
    }};
  }

  void stop() {
    images->stop();
    if (loop.joinable()) {
      loop.join();
    }
  }

  /**
   * @brief Hand the loop a captured image and wait until it takes it.
   */
  void capture(std::chrono::steady_clock::time_point at) {
    auto img = std::make_shared<platf::img_t>();
    img->frame_timestamp = at;
    images->raise(std::move(img));
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (images->peek()) {
      if (std::chrono::steady_clock::now() > deadline) {
        ADD_FAILURE() << "the encode loop did not take the image";
        return;
      }
      std::this_thread::sleep_for(1ms);
    }
  }

  repairing_nvenc_t encoder;
  safe::mail_t mail = std::make_shared<safe::mail_raw_t>();
  video::img_event_t images = std::make_shared<video::img_event_t::element_type>();
  video::config_t config {1280, 720, 60, 6000, 8000, 1, 0, 1, 0, 0, 0, 0};
  std::shared_ptr<platf::display_t> display = std::make_shared<blank_display_t>();
  safe::signal_t reinit;
  safe::mail_raw_t::queue_t<video::packet_t> packets;
  int frame_nr = 1;
  std::thread loop;
  double saved_fps_target = 0;
};

TEST_F(EncodeLoopTest, AnAcknowledgementReachesTheEncoderBeforeTheRepairItCameWith) {
  auto acks = mail->event<std::uint32_t>(mail::ltr_ack);
  auto invalidations = mail->event<std::pair<int64_t, int64_t>>(mail::invalidate_ref_frames);
  acks->raise(30u);
  invalidations->raise(std::make_pair(int64_t {31}, int64_t {32}));
  start();
  capture(std::chrono::steady_clock::now());
  auto packet = packets->pop(5s);
  stop();
  ASSERT_TRUE(packet);
  EXPECT_EQ(packet->frame_index(), 1);
  EXPECT_EQ(encoder.calls, (std::vector<std::string> {"acknowledge 30", "invalidate 31-32", "encode 1"})) << "the frame the repair predicts from may be the one just acknowledged";
}
