/**
 * @file tests/unit/test_nvenc_ltr.cpp
 * @brief The long-term reference paths of the NVENC base encoder, driven through a fake NVENC function table.
 */

#ifdef _WIN32
  // standard includes
  #include <array>
  #include <bit>
  #include <cstdint>
  #include <memory>
  #include <optional>
  #include <vector>

  // lib includes
  #include <gtest/gtest.h>

  // local includes
  #include "src/nvenc/nvenc_base.h"

namespace {
  using namespace nvenc_1301;

  /**
   * @brief What the fake NVENC driver was given and what it reports.
   */
  struct fake_driver_t {
    int gpu_max_ltr = 4;  ///< NV_ENC_CAPS_NUM_MAX_LTR_FRAMES the driver reports.
    bool rfi = true;  ///< NV_ENC_CAPS_SUPPORT_REF_PIC_INVALIDATION the driver reports.
    NV_ENC_CONFIG config {};  ///< The configuration NvEncInitializeEncoder received.
    std::vector<NV_ENC_PIC_PARAMS> pictures;  ///< Every NvEncEncodePicture call.
    std::vector<std::uint64_t> invalidated;  ///< Every frame NvEncInvalidateRefFrames was asked for.
    std::optional<std::uint64_t> fail_invalidating;  ///< The frame whose invalidation the driver refuses.
    std::array<std::uint8_t, 4> bitstream {};  ///< What every locked bitstream points at.
  };

  fake_driver_t fake;  ///< The driver the encoders below talk to.

  /**
   * @brief One long-term reference decision the encoder made for a frame, read from its picture parameters.
   */
  struct ltr_use_t {
    bool mark = false;  ///< The frame was marked as a long-term reference.
    std::uint32_t mark_index = 0;  ///< The slot it was marked into.
    bool use = false;  ///< The frame was told to predict from long-term references.
    std::uint32_t use_bitmap = 0;  ///< The slots it may predict from.
  };

  /**
   * @brief Read the long-term reference fields of a picture, whatever its codec.
   * @param picture The parameters NvEncEncodePicture received.
   * @param video_format 0 H.264, 1 HEVC, 2 AV1.
   * @return What the encoder asked of the driver.
   */
  ltr_use_t ltr_of(const NV_ENC_PIC_PARAMS &picture, int video_format) {
    const auto read = [](const auto &codec) {
      return ltr_use_t {codec.ltrMarkFrame != 0, codec.ltrMarkFrameIdx, codec.ltrUseFrames != 0, codec.ltrUseFrameBitmap};
    };
    if (video_format == 0) {
      return read(picture.codecPicParams.h264PicParams);
    }
    if (video_format == 1) {
      return read(picture.codecPicParams.hevcPicParams);
    }
    return read(picture.codecPicParams.av1PicParams);
  }

  /**
   * @brief The long-term reference setup NvEncInitializeEncoder received, whatever the codec.
   */
  struct ltr_setup_t {
    bool enabled;  ///< enableLTR.
    std::uint32_t frames;  ///< ltrNumFrames.
    std::uint32_t trust_mode;  ///< ltrTrustMode: 0 is "per picture", 1 "trust" (H.264 and HEVC only).
    std::uint32_t reference_frames;  ///< The DPB size the codec asked for.
  };

  ltr_setup_t setup_of(const NV_ENC_CONFIG &config, int video_format) {
    const auto &codec = config.encodeCodecConfig;
    if (video_format == 0) {
      return {codec.h264Config.enableLTR != 0, codec.h264Config.ltrNumFrames, codec.h264Config.ltrTrustMode, codec.h264Config.maxNumRefFrames};
    }
    if (video_format == 1) {
      return {codec.hevcConfig.enableLTR != 0, codec.hevcConfig.ltrNumFrames, codec.hevcConfig.ltrTrustMode, codec.hevcConfig.maxNumRefFramesInDPB};
    }
    return {codec.av1Config.enableLTR != 0, codec.av1Config.ltrNumFrames, 0, codec.av1Config.maxNumRefFramesInDPB};
  }

  /**
   * @brief A base encoder whose driver is `fake`: it accepts every call and records the ones that matter.
   */
  class ltr_encoder_t: public nvenc_base {
  public:
    ltr_encoder_t():
        nvenc_base(NV_ENC_DEVICE_TYPE_DIRECTX) {
      nvenc = std::make_shared<NV_ENCODE_API_FUNCTION_LIST>();
      nvenc->nvEncOpenEncodeSessionEx = [](NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS *, void **session) {
        *session = &fake;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodeGUIDCount = [](void *, std::uint32_t *count) {
        *count = 3;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodeGUIDs = [](void *, GUID *guids, std::uint32_t, std::uint32_t *count) {
        guids[0] = NV_ENC_CODEC_H264_GUID;
        guids[1] = NV_ENC_CODEC_HEVC_GUID;
        guids[2] = NV_ENC_CODEC_AV1_GUID;
        *count = 3;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodeCaps = [](void *, GUID, NV_ENC_CAPS_PARAM *params, int *value) {
        switch (params->capsToQuery) {
          case NV_ENC_CAPS_WIDTH_MAX:
          case NV_ENC_CAPS_HEIGHT_MAX:
            *value = 8192;
            break;
          case NV_ENC_CAPS_SUPPORT_REF_PIC_INVALIDATION:
            *value = fake.rfi ? 1 : 0;
            break;
          case NV_ENC_CAPS_SUPPORT_MULTIPLE_REF_FRAMES:
            *value = 1;
            break;
          case NV_ENC_CAPS_NUM_MAX_LTR_FRAMES:
            *value = fake.gpu_max_ltr;
            break;
          default:
            *value = 0;
        }
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodePresetConfigEx = [](void *, GUID, GUID, NV_ENC_TUNING_INFO, NV_ENC_PRESET_CONFIG *) {
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncInitializeEncoder = [](void *, NV_ENC_INITIALIZE_PARAMS *params) {
        fake.config = *params->encodeConfig;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncCreateBitstreamBuffer = [](void *, NV_ENC_CREATE_BITSTREAM_BUFFER *buffer) {
        buffer->bitstreamBuffer = std::bit_cast<NV_ENC_OUTPUT_PTR>(&fake.bitstream);
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncMapInputResource = [](void *, NV_ENC_MAP_INPUT_RESOURCE *input) {
        input->mappedResource = input->registeredResource;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncUnmapInputResource = [](void *, NV_ENC_INPUT_PTR) {
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncEncodePicture = [](void *, NV_ENC_PIC_PARAMS *picture) {
        fake.pictures.push_back(*picture);
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncLockBitstream = [](void *, NV_ENC_LOCK_BITSTREAM *lock) {
        const auto &picture = fake.pictures.back();
        lock->bitstreamBufferPtr = fake.bitstream.data();
        lock->bitstreamSizeInBytes = static_cast<std::uint32_t>(fake.bitstream.size());
        lock->outputTimeStamp = picture.inputTimeStamp;
        lock->pictureType = picture.encodePicFlags & NV_ENC_PIC_FLAG_FORCEIDR ? NV_ENC_PIC_TYPE_IDR : NV_ENC_PIC_TYPE_P;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncUnlockBitstream = [](void *, NV_ENC_OUTPUT_PTR) {
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncInvalidateRefFrames = [](void *, std::uint64_t frame) {
        fake.invalidated.push_back(frame);
        return fake.fail_invalidating == frame ? NV_ENC_ERR_GENERIC : NV_ENC_SUCCESS;
      };
      nvenc->nvEncUnregisterResource = [](void *, NV_ENC_REGISTERED_PTR) {
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncDestroyBitstreamBuffer = [](void *, NV_ENC_OUTPUT_PTR) {
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncDestroyEncoder = [](void *) {
        return NV_ENC_SUCCESS;
      };
      device = std::bit_cast<ID3D11Device *>(std::uintptr_t {0x1234});
    }

    ltr_encoder_t(const ltr_encoder_t &) = delete;
    ltr_encoder_t &operator=(const ltr_encoder_t &) = delete;

    ~ltr_encoder_t() override {
      destroy_encoder();
    }

  protected:
    bool init_library() override {
      return false;
    }

    bool create_and_register_input_buffer() override {
      registered_input_buffer = std::bit_cast<NV_ENC_REGISTERED_PTR>(std::uintptr_t {0x10});
      return true;
    }
  };

  /**
   * @brief The encoders below, opened at 1080p60 as the capture loop opens them.
   */
  struct NvencLtrTest: testing::Test {
    void SetUp() override {
      fake = {};
    }

    /**
     * @brief Open the encoder.
     *
     * @param format 0 H.264, 1 HEVC, 2 AV1.
     * @param repairs Whether the client announced the signals and the controller repairs losses.
     * @param reference_frames The DPB the client asked for, 0 for the default.
     * @return Whether the encoder opened.
     */
    bool open(int format, bool repairs, int reference_frames = 0) {
      video_format = format;
      ::nvenc::nvenc_config config {};
      video::config_t client_config {};
      client_config.width = 1920;
      client_config.height = 1080;
      client_config.framerate = 60;
      client_config.bitrate = 20000;
      client_config.videoFormat = format;
      client_config.numRefFrames = reference_frames;
      client_config.adaptiveRepairs = repairs;
      const auto colorspace = video::colorspace_from_client_config(client_config, false);
      return encoder.create_encoder(config, client_config, colorspace, platf::pix_fmt_e::nv12);
    }

    /**
     * @brief Encode frames first to last, the first one an IDR frame when asked.
     */
    std::vector<::nvenc::nvenc_encoded_frame> encode(std::uint64_t first, std::uint64_t last, bool idr_first = true) {
      std::vector<::nvenc::nvenc_encoded_frame> frames;
      for (auto i = first; i <= last; ++i) {
        frames.push_back(encoder.encode_frame(i, idr_first && i == first));
      }
      return frames;
    }

    ltr_use_t picture(std::size_t index) const {
      return ltr_of(fake.pictures.at(index), video_format);
    }

    ltr_encoder_t encoder;
    int video_format = 0;
  };

  /**
   * @brief The codecs the long-term references reach.
   */
  struct NvencLtrCodecTest: NvencLtrTest, testing::WithParamInterface<int> {};
}  // namespace

// @ABR-19-S1
TEST_P(NvencLtrCodecTest, ASessionTheControllerRepairsOpensWithTwoLongTermReferences) {
  ASSERT_TRUE(open(GetParam(), true));
  const auto setup = setup_of(fake.config, GetParam());
  EXPECT_TRUE(setup.enabled);
  EXPECT_EQ(setup.frames, 2U) << "two long-term references within the level's DPB";
  EXPECT_EQ(setup.trust_mode, 0U) << "marked frame by frame, not by the driver";
  EXPECT_EQ(encoder.repair_reach_frames(), 30U) << "one slot reaches back a mark period";
}

// @ABR-19-S1
TEST_P(NvencLtrCodecTest, AnyOtherSessionOpensWithoutLongTermReferences) {
  ASSERT_TRUE(open(GetParam(), false));
  EXPECT_FALSE(setup_of(fake.config, GetParam()).enabled);
  EXPECT_EQ(setup_of(fake.config, GetParam()).frames, 0U);
  EXPECT_EQ(encoder.repair_reach_frames(), 0U);
  encode(0, 40);
  for (std::size_t i = 0; i < fake.pictures.size(); ++i) {
    EXPECT_FALSE(picture(i).mark) << "frame " << i;
    EXPECT_FALSE(picture(i).use) << "frame " << i;
  }
}

// @ABR-19-S1
TEST_P(NvencLtrCodecTest, EveryThirtiethFrameIsMarkedIntoTheSlotsInTurn) {
  ASSERT_TRUE(open(GetParam(), true));
  const auto frames = encode(0, 95);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const bool marked = i % 30 == 0;
    EXPECT_EQ(picture(i).mark, marked) << "frame " << i;
    EXPECT_EQ(frames[i].ltr_frame, marked) << "the packet says so for frame " << i;
    EXPECT_FALSE(picture(i).use) << "no repair, no long-term reference is told to be used: frame " << i;
    if (marked) {
      EXPECT_EQ(picture(i).mark_index, (i / 30) % 2) << "frame " << i;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Codecs, NvencLtrCodecTest, testing::Values(0, 1, 2));

// @ABR-19-S1
TEST_F(NvencLtrTest, ASessionWithTheDpbTheClientAskedForKeepsAQuarterOfItAsLongTermReferences) {
  ASSERT_TRUE(open(0, true, 16));
  const auto setup = setup_of(fake.config, 0);
  EXPECT_EQ(setup.frames, 4U);
  EXPECT_EQ(setup.reference_frames, 16U);
  EXPECT_EQ(encoder.repair_reach_frames(), 90U);
}

// @ABR-19-S1
TEST_F(NvencLtrTest, AGpuThatKeepsFewerThanTwoLongTermReferencesOpensWithoutThem) {
  for (const int gpu_max : {0, 1}) {
    fake.gpu_max_ltr = gpu_max;
    ASSERT_TRUE(open(0, true));
    EXPECT_FALSE(setup_of(fake.config, 0).enabled) << gpu_max;
    EXPECT_EQ(encoder.repair_reach_frames(), 0U) << gpu_max;
  }
}

// @ABR-19-S1
TEST_F(NvencLtrTest, AnIdrFrameEmptiesTheLongTermReferences) {
  ASSERT_TRUE(open(0, true));
  encode(0, 40);
  encoder.encode_frame(41, true);
  EXPECT_FALSE(picture(41).use) << "an IDR frame predicts from nothing";
  encoder.acknowledge_ltr(30);
  encode(42, 44, false);

  // Frames 0 and 30 are gone with the IDR frame, so the acknowledgement of 30 was dropped as well
  EXPECT_FALSE(encoder.invalidate_ref_frames(42, 44)) << "nothing is left to predict from but the lost frames";
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ARepairPredictsFromTheNewestLongTermReferenceBeforeTheLoss) {
  ASSERT_TRUE(open(0, true));
  encode(0, 40);
  fake.invalidated.clear();

  // Frames 35 to 40 are lost: more than the two short-term references reach, and nothing was acknowledged
  EXPECT_TRUE(encoder.invalidate_ref_frames(35, 40));
  EXPECT_EQ(fake.invalidated, (std::vector<std::uint64_t> {39, 40})) << "the short-term references inside the loss";
  const auto repaired = encoder.encode_frame(41, false);
  EXPECT_TRUE(repaired.after_ref_frame_invalidation);
  const auto repair = picture(41);
  EXPECT_TRUE(repair.use);
  EXPECT_EQ(repair.use_bitmap, 1U << 1) << "frame 30 sits in slot 1";
  EXPECT_FALSE(repair.mark);

  // The next frame is plain again
  encoder.encode_frame(42, false);
  EXPECT_FALSE(picture(42).use);
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ARepairPredictsFromTheReferenceTheClientAcknowledged) {
  ASSERT_TRUE(open(0, true));
  encode(0, 40);
  encoder.acknowledge_ltr(0);
  encoder.acknowledge_ltr(30);
  encoder.acknowledge_ltr(12345);  // held by no slot: ignored
  fake.invalidated.clear();

  EXPECT_TRUE(encoder.invalidate_ref_frames(35, 40));
  encoder.encode_frame(41, false);
  EXPECT_EQ(picture(41).use_bitmap, 1U << 1) << "the newest acknowledged mark, frame 30";

  // Frame 60 is marked into the slot the acknowledged frame does not hold
  encode(42, 59, false);
  EXPECT_TRUE(encoder.invalidate_ref_frames(50, 59));
  encode(60, 60, false);
  EXPECT_EQ(picture(60).use_bitmap, 1U << 1);
  EXPECT_TRUE(picture(60).mark);
  EXPECT_EQ(picture(60).mark_index, 0U);

  // Frame 90 would go into slot 1, which the repair predicts from: the mark is skipped
  encode(61, 89, false);
  EXPECT_TRUE(encoder.invalidate_ref_frames(80, 89));
  encode(90, 90, false);
  EXPECT_EQ(picture(90).use_bitmap, 1U << 1);
  EXPECT_FALSE(picture(90).mark) << "a mark never overwrites the slot the next frame predicts from";
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ALossThatTheShortTermReferencesCoverNeedsNoLongTermOne) {
  ASSERT_TRUE(open(0, true));
  encode(0, 40);
  fake.invalidated.clear();

  EXPECT_TRUE(encoder.invalidate_ref_frames(40, 40));
  EXPECT_EQ(fake.invalidated, (std::vector<std::uint64_t> {40}));
  encoder.encode_frame(41, false);
  EXPECT_FALSE(picture(41).use) << "the short-term reference before the loss is still there";
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ALongTermReferenceInsideTheLossIsInvalidatedWithIt) {
  ASSERT_TRUE(open(0, true));
  encode(0, 65);
  fake.invalidated.clear();

  // Frame 60 (slot 0) is inside the loss: the driver is told to drop it, and the repair uses frame 30
  EXPECT_TRUE(encoder.invalidate_ref_frames(55, 65));
  EXPECT_EQ(fake.invalidated, (std::vector<std::uint64_t> {60, 64, 65}));
  encoder.encode_frame(66, false);
  EXPECT_EQ(picture(66).use_bitmap, 1U << 1);
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ALossNoLongTermReferenceBeforeItSurvivesCostsAnIdrFrame) {
  ASSERT_TRUE(open(0, true));
  encode(0, 65);
  fake.invalidated.clear();

  EXPECT_FALSE(encoder.invalidate_ref_frames(5, 65)) << "both slots were marked after frame 5";
  EXPECT_TRUE(fake.invalidated.empty());
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ADriverThatRefusesALongTermInvalidationCostsAnIdrFrame) {
  ASSERT_TRUE(open(0, true));
  encode(0, 65);
  fake.invalidated.clear();
  fake.fail_invalidating = 60;

  EXPECT_FALSE(encoder.invalidate_ref_frames(55, 65));
  EXPECT_EQ(fake.invalidated, (std::vector<std::uint64_t> {60})) << "it stops at the one the driver refused";
}

// @ABR-19-S1
TEST_F(NvencLtrTest, ASessionWithoutLongTermReferencesInvalidatesFrameByFrame) {
  ASSERT_TRUE(open(0, false));
  encode(0, 20);
  fake.invalidated.clear();

  EXPECT_TRUE(encoder.invalidate_ref_frames(18, 19));
  EXPECT_EQ(fake.invalidated, (std::vector<std::uint64_t> {18, 19, 20})) << "up to the last frame encoded";
  encoder.encode_frame(21, false);
  EXPECT_FALSE(picture(21).use);

  // More frames than the DPB holds: no reference is left
  fake.invalidated.clear();
  encode(22, 40, false);
  EXPECT_FALSE(encoder.invalidate_ref_frames(25, 30));
}
#endif
