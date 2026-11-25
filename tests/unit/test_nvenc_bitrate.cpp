/**
 * @file tests/unit/test_nvenc_bitrate.cpp
 * @brief The bitrate changes of the NVENC base encoder, driven through a fake NVENC function table.
 */

#ifdef _WIN32
  // test includes
  #include "../tests_common.h"

  // standard includes
  #include <bit>
  #include <cstdint>
  #include <limits>
  #include <memory>
  #include <optional>

  // local includes
  #include "src/nvenc/nvenc_base.h"

namespace {
  using namespace nvenc_1301;

  /**
   * @brief What the fake NVENC driver reports and what it was asked to reconfigure.
   */
  struct fake_driver_t {
    bool dynamic_bitrate = true;  ///< NV_ENC_CAPS_SUPPORT_DYN_BITRATE_CHANGE the driver reports.
    NVENCSTATUS reconfigure_status = NV_ENC_SUCCESS;  ///< What NvEncReconfigureEncoder returns.
    int reconfigurations = 0;  ///< NvEncReconfigureEncoder calls.
    std::optional<NV_ENC_RECONFIGURE_PARAMS> reconfigure;  ///< The parameters of the last call.
    std::optional<NV_ENC_CONFIG> reconfigured_config;  ///< The configuration of the last call.
    std::uint8_t bitstream = 0;  ///< Stands in for the output bitstream buffer.
  };

  fake_driver_t fake;  ///< The driver the encoder below talks to.

  constexpr std::uint32_t preset_max_bitrate = 30'000'000;  ///< Maximum bitrate the fake preset configures.

  /**
   * @brief A base encoder whose driver is `fake`.
   */
  class bitrate_encoder_t: public nvenc_base {
  public:
    bitrate_encoder_t():
        nvenc_base(NV_ENC_DEVICE_TYPE_DIRECTX) {
      nvenc = std::make_shared<NV_ENCODE_API_FUNCTION_LIST>();
      nvenc->nvEncOpenEncodeSessionEx = [](NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS *, void **session) {
        *session = &fake;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodeGUIDCount = [](void *, std::uint32_t *count) {
        *count = 1;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodeGUIDs = [](void *, GUID *guids, std::uint32_t, std::uint32_t *count) {
        guids[0] = NV_ENC_CODEC_H264_GUID;
        *count = 1;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodeCaps = [](void *, GUID, NV_ENC_CAPS_PARAM *params, int *value) {
        switch (params->capsToQuery) {
          case NV_ENC_CAPS_WIDTH_MAX:
          case NV_ENC_CAPS_HEIGHT_MAX:
            *value = 8192;
            break;
          case NV_ENC_CAPS_SUPPORT_CUSTOM_VBV_BUF_SIZE:
            *value = 1;
            break;
          case NV_ENC_CAPS_SUPPORT_DYN_BITRATE_CHANGE:
            *value = fake.dynamic_bitrate ? 1 : 0;
            break;
          default:
            *value = 0;
        }
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncGetEncodePresetConfigEx = [](void *, GUID, GUID, NV_ENC_TUNING_INFO, NV_ENC_PRESET_CONFIG *preset) {
        preset->presetCfg.rcParams.maxBitRate = preset_max_bitrate;
        preset->presetCfg.rcParams.vbvInitialDelay = 1;
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncInitializeEncoder = [](void *, NV_ENC_INITIALIZE_PARAMS *) {
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncCreateBitstreamBuffer = [](void *, NV_ENC_CREATE_BITSTREAM_BUFFER *buffer) {
        buffer->bitstreamBuffer = std::bit_cast<NV_ENC_OUTPUT_PTR>(&fake.bitstream);
        return NV_ENC_SUCCESS;
      };
      nvenc->nvEncReconfigureEncoder = [](void *, NV_ENC_RECONFIGURE_PARAMS *params) {
        ++fake.reconfigurations;
        fake.reconfigure = *params;
        fake.reconfigured_config = *params->reInitEncodeParams.encodeConfig;
        return fake.reconfigure_status;
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

    bitrate_encoder_t(const bitrate_encoder_t &) = delete;
    bitrate_encoder_t &operator=(const bitrate_encoder_t &) = delete;

    ~bitrate_encoder_t() override {
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
   * @brief A base encoder that opens at 20 Mbit/s and 60 fps, as the capture loop opens it.
   */
  struct NvencBitrateTest: testing::Test {
    void SetUp() override {
      fake = {};
    }

    /**
     * @brief Open the encoder.
     *
     * @return Whether the encoder opened.
     */
    bool open() {
      ::nvenc::nvenc_config config {};
      video::config_t client_config {};
      client_config.width = 1920;
      client_config.height = 1080;
      client_config.framerate = 60;
      client_config.bitrate = 20000;
      client_config.videoFormat = 0;
      const auto colorspace = video::colorspace_from_client_config(client_config, false);
      return encoder.create_encoder(config, client_config, colorspace, platf::pix_fmt_e::nv12);
    }

    bitrate_encoder_t encoder;
  };
}  // namespace

TEST_F(NvencBitrateTest, ReportsTheBitrateItWasOpenedWith) {
  ASSERT_TRUE(open());
  EXPECT_TRUE(encoder.can_change_bitrate());
  EXPECT_EQ(encoder.bitrate_kbps(), 20000U);
}

TEST_F(NvencBitrateTest, ReconfiguresTheRunningEncoderWithAScaledVbv) {
  ASSERT_TRUE(open());
  EXPECT_TRUE(encoder.set_bitrate(10000));

  ASSERT_EQ(fake.reconfigurations, 1);
  EXPECT_EQ(fake.reconfigure->resetEncoder, 0U) << "the encoder and its references are kept";
  EXPECT_EQ(fake.reconfigure->forceIDR, 0U) << "no keyframe is forced";
  const auto &rc = fake.reconfigured_config->rcParams;
  EXPECT_EQ(rc.averageBitRate, 10'000'000U);
  EXPECT_EQ(rc.maxBitRate, 10'000'000U) << "a maximum bitrate that was set follows the average";
  EXPECT_EQ(rc.vbvBufferSize, 20'000'000U / 60 / 2) << "the VBV is the same fraction of a frame at half the bitrate";
  EXPECT_EQ(rc.vbvInitialDelay, rc.vbvBufferSize);
  EXPECT_EQ(encoder.bitrate_kbps(), 10000U);
}

TEST_F(NvencBitrateTest, ScalesTheVbvFromTheBitrateTheEncoderWasOpenedWith) {
  ASSERT_TRUE(open());
  ASSERT_TRUE(encoder.set_bitrate(10000));
  ASSERT_TRUE(encoder.set_bitrate(40000));

  const auto &rc = fake.reconfigured_config->rcParams;
  EXPECT_EQ(rc.averageBitRate, 40'000'000U);
  EXPECT_EQ(rc.vbvBufferSize, 20'000'000U / 60 * 2);
}

TEST_F(NvencBitrateTest, DoesNothingForTheBitrateItRunsAt) {
  ASSERT_TRUE(open());
  EXPECT_TRUE(encoder.set_bitrate(20000));
  EXPECT_EQ(fake.reconfigurations, 0);
}

TEST_F(NvencBitrateTest, RefusesABitrateItCannotRepresent) {
  ASSERT_TRUE(open());
  EXPECT_FALSE(encoder.set_bitrate(0));
  EXPECT_FALSE(encoder.set_bitrate(std::numeric_limits<std::uint32_t>::max()));
  EXPECT_EQ(fake.reconfigurations, 0);
  EXPECT_EQ(encoder.bitrate_kbps(), 20000U);
}

TEST_F(NvencBitrateTest, AFailedReconfigurationKeepsTheBitrateInUse) {
  ASSERT_TRUE(open());
  fake.reconfigure_status = NV_ENC_ERR_GENERIC;
  EXPECT_FALSE(encoder.set_bitrate(5000));
  EXPECT_EQ(encoder.bitrate_kbps(), 20000U) << "the encoder still runs at what it was opened with";

  // The next attempt starts from the configuration in use, not from the failed one
  fake.reconfigure_status = NV_ENC_SUCCESS;
  EXPECT_TRUE(encoder.set_bitrate(10000));
  EXPECT_EQ(fake.reconfigured_config->rcParams.vbvBufferSize, 20'000'000U / 60 / 2);
}

TEST_F(NvencBitrateTest, AGpuWithoutDynamicBitrateChangeRefusesANewBitrate) {
  fake.dynamic_bitrate = false;
  ASSERT_TRUE(open());

  EXPECT_FALSE(encoder.can_change_bitrate());
  EXPECT_FALSE(encoder.set_bitrate(10000));
  EXPECT_EQ(fake.reconfigurations, 0);
}

TEST(NvencBitrateWithoutEncoderTest, RefusesAndReportsNoBitrate) {
  bitrate_encoder_t encoder;
  EXPECT_FALSE(encoder.can_change_bitrate());
  EXPECT_FALSE(encoder.set_bitrate(10000));
  EXPECT_EQ(encoder.bitrate_kbps(), 0U);
}
#endif
