#include "cvi_internal.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "cvi_ae.h"
#include "cvi_awb.h"

namespace cvimpp
{
namespace
{

// Lock AE to a fixed exposure and AWB to fixed gains.  Called once after the
// VI/ISP pipeline is up; the sensor runs at a constant brightness afterwards,
// which is what a fixed-threshold Lab detector wants.
bool ApplyManualIsp(VI_PIPE pipe, const ManualIspConfig &manual)
{
  if (manual.exposure_us == 0)
  {
    return true;
  }

  ISP_EXPOSURE_ATTR_S exposure{};
  if (CVI_ISP_GetExposureAttr(pipe, &exposure) != CVI_SUCCESS)
  {
    std::printf("[camera] CVI_ISP_GetExposureAttr failed\n");
    return false;
  }
  exposure.bByPass = CVI_FALSE;
  exposure.enOpType = OP_TYPE_MANUAL;
  exposure.stManual.enExpTimeOpType = OP_TYPE_MANUAL;
  exposure.stManual.u32ExpTime = manual.exposure_us;
  exposure.stManual.enGainType = AE_TYPE_GAIN;
  exposure.stManual.enISONumOpType = OP_TYPE_AUTO;
  exposure.stManual.enAGainOpType = OP_TYPE_MANUAL;
  exposure.stManual.u32AGain = manual.again;
  exposure.stManual.enDGainOpType = OP_TYPE_MANUAL;
  exposure.stManual.u32DGain = manual.dgain;
  exposure.stManual.enISPDGainOpType = OP_TYPE_MANUAL;
  exposure.stManual.u32ISPDGain = manual.isp_dgain;
  // Manual AE/AWB do not need the 3A libraries at the full frame rate.  At
  // 180 fps the per-frame 3A/statistics work is the dominant user-space cost.
  exposure.u8AERunInterval = 12;
  if (CVI_ISP_SetExposureAttr(pipe, &exposure) != CVI_SUCCESS)
  {
    std::printf("[camera] CVI_ISP_SetExposureAttr failed\n");
    return false;
  }

  ISP_WB_ATTR_S wb{};
  if (CVI_ISP_GetWBAttr(pipe, &wb) != CVI_SUCCESS)
  {
    std::printf("[camera] CVI_ISP_GetWBAttr failed\n");
    return false;
  }
  wb.bByPass = CVI_FALSE;
  // Unity gains are not a white balance: they are "no correction", which under
  // an LED or fluorescent illuminant leaves a green cast strong enough that a
  // neutral lamp classifies as a green beacon.  Only lock AWB when a gain that
  // actually corrects something is given; otherwise let the loop run, slowed
  // down, so a neutral surface stays neutral.
  const bool lock_wb = manual.wb_rgain != 0 && manual.wb_bgain != 0 &&
                       (manual.wb_rgain != 1024 || manual.wb_bgain != 1024 ||
                        manual.wb_ggain != 1024);
  wb.enOpType = lock_wb ? OP_TYPE_MANUAL : OP_TYPE_AUTO;
  if (lock_wb)
  {
    wb.stManual.u16Rgain = manual.wb_rgain;
    wb.stManual.u16Grgain = manual.wb_ggain;
    wb.stManual.u16Gbgain = manual.wb_ggain;
    wb.stManual.u16Bgain = manual.wb_bgain;
  }
  wb.u8AWBRunInterval = 12;
  if (CVI_ISP_SetWBAttr(pipe, &wb) != CVI_SUCCESS)
  {
    std::printf("[camera] CVI_ISP_SetWBAttr failed\n");
    return false;
  }

  // The vendor noise curves index strength by ISO, and with manual AE the ISO
  // never leaves the first bucket.  The defaults there are close to "off", and
  // a 100-500 us frame is read-noise limited, so the flat background comes out
  // as coloured speckle.  These are the values measured on this board: 24 of
  // 255 for the three noise reductions, and sharpening pulled back from 32 to 8
  // so it stops amplifying what is left.  A beacon is tens of pixels wide, so
  // this costs no detection margin.
  // AF is not used by the fixed-focus SC035HGS; drop its statistics entirely.
  // AE/AWB statistics run at 1/6 and 1/12 of the frame rate, which is ample for
  // locked manual exposure/white balance.
  ISP_CTRL_PARAM_S ctrl{};
  if (CVI_ISP_GetCtrlParam(pipe, &ctrl) == CVI_SUCCESS)
  {
    ctrl.u32AEStatIntvl = 12;
    ctrl.u32AWBStatIntvl = 24;
    ctrl.u32AFStatIntvl = 0;
    if (CVI_ISP_SetCtrlParam(pipe, &ctrl) != CVI_SUCCESS)
    {
      std::printf("[camera] CVI_ISP_SetCtrlParam failed\n");
    }
  }

  std::printf("[camera] manual ISP locked: exposure=%uus again=%u dgain=%u "
              "ispdgain=%u wb=%s ae_interval=%u\n",
              manual.exposure_us, manual.again, manual.dgain, manual.isp_dgain,
              lock_wb ? "manual" : "auto", exposure.u8AERunInterval);
  return true;
}

}  // namespace

namespace detail
{

ContextState::~ContextState()
{
  if (vi_started)
  {
    SAMPLE_COMM_VI_DestroyIsp(&vi_config);
    SAMPLE_COMM_VI_DestroyVi(&vi_config);
    vi_started = false;
  }

  if (system_started)
  {
    CVI_SYS_Exit();
    system_started = false;
  }
  if (vb_started)
  {
    CVI_VB_Exit();
    vb_started = false;
  }
}

}  // namespace detail

Context::Context(ContextConfig config)
{
  using detail::ContextState;

  if (config.vi_pool_size == 0 || config.vi_pool_count == 0)
  {
    status_ = Status(kInvalidArgument);
    return;
  }

  auto implementation = std::make_unique<Impl>();
  implementation->state = std::make_shared<ContextState>();
  auto &state = *implementation->state;

  // Match the vendor test_mmf setup so the CVI_TRACE_LOG diagnostics emitted by
  // SAMPLE_PLAT_VI_INIT are visible when a stage fails.
  LOG_LEVEL_CONF_S log_conf{};
  log_conf.enModId = CVI_ID_LOG;
  log_conf.s32Level = CVI_DBG_DEBUG;
  CVI_LOG_SetLevelConf(&log_conf);

  if (config.sensor_ini_path != nullptr && config.sensor_ini_path[0] != '\0')
  {
    int result = SAMPLE_COMM_VI_SetIniPath(config.sensor_ini_path);
    if (result != CVI_SUCCESS)
    {
      status_ = Status(result);
      return;
    }
  }

  SAMPLE_INI_CFG_S ini{};
  // The sample parser fills its built-in default even when no ini file is
  // present. This is the same fallback used by the vendor sample programs.
  SAMPLE_COMM_VI_ParseIni(&ini);
  if (ini.devNum == 0 || ini.devNum > VI_MAX_DEV_NUM)
  {
    status_ = Status(kInvalidArgument);
    return;
  }

  int result = CVI_VI_SetDevNum(ini.devNum);
  if (result != CVI_SUCCESS)
  {
    status_ = Status(result);
    return;
  }

  result = SAMPLE_COMM_VI_IniToViCfg(&ini, &state.vi_config);
  if (result != CVI_SUCCESS)
  {
    status_ = Status(result);
    return;
  }

  PIC_SIZE_E sensor_pic_size{};
  result = SAMPLE_COMM_VI_GetSizeBySensor(ini.enSnsType[0], &sensor_pic_size);
  if (result == CVI_SUCCESS)
  {
    result = SAMPLE_COMM_SYS_GetPicSize(sensor_pic_size, &state.sensor_size);
  }
  if (result != CVI_SUCCESS || state.sensor_size.u32Width == 0 ||
      state.sensor_size.u32Height == 0)
  {
    status_ = Status(result != CVI_SUCCESS ? result : kInvalidArgument);
    return;
  }

  // The sample table this comes from asks the VI for a *mono* channel on this
  // board even though the SC035HGS is a colour sensor: the image repo had to
  // patch maix_mmf for the same reason ("use nv21 for color sc035hgs").  A mono
  // VI channel means the ISP never runs the colour pipeline, so every chroma
  // sample is 128 and a colour beacon can never be detected -- the preview looks
  // grey and the detector sees nothing.
  // libcvimpp owns this pipeline, so it has to make the same choice: ask for
  // NV21 from the VI as well as from the VPSS channel.
  state.vi_config.astViInfo[0].stChnInfo.enPixFormat = PIXEL_FORMAT_NV21;
  std::printf("[camera] VI channel pixel format requested = %d (NV21=%d YUV400=%d)\n",
              static_cast<int>(state.vi_config.astViInfo[0].stChnInfo.enPixFormat),
              static_cast<int>(PIXEL_FORMAT_NV21), static_cast<int>(PIXEL_FORMAT_YUV_400));
  state.pipe = state.vi_config.astViInfo[0].stPipeInfo.aPipe[0];
  state.vi_channel = state.vi_config.astViInfo[0].stChnInfo.ViChn;
  state.vi_format = state.vi_config.astViInfo[0].stChnInfo.enPixFormat;

  VB_CAL_CONFIG_S calculated{};
  COMMON_GetPicBufferConfig(state.sensor_size.u32Width, state.sensor_size.u32Height,
                            state.vi_format, DATA_BITWIDTH_8, COMPRESS_MODE_NONE,
                            DEFAULT_ALIGN, &calculated);
  if (calculated.u32VBSize == 0)
  {
    status_ = Status(kInvalidArgument);
    return;
  }

  const std::uint32_t pool_size = std::max(config.vi_pool_size, calculated.u32VBSize);
  VB_CONFIG_S vb_config{};
  // Mirror the working test_mmf VB layout.  The VI channel does not only need
  // its own block: the vendor VI/VPSS/VENC path also allocates from the VO,
  // user, encoder and decoder pools during enable/start, so a single-pool
  // configuration fails with vi_sdk_enable_chn ENOMEM at 180 fps.
  // On this board the vendor ION heap is below 40 MiB, so test_mmf's
  // priv_param_init() builds only two pools (VI + H.264 encode).  Mirror that
  // exact layout; extra pools make CVI_VB_SetConfig fail.
  vb_config.u32MaxPoolCnt = 2;
  vb_config.astCommPool[0].u32BlkSize = pool_size;
  vb_config.astCommPool[0].u32BlkCnt = config.vi_pool_count;
  vb_config.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;
  vb_config.astCommPool[1].u32BlkSize =
      ALIGN(1280, DEFAULT_ALIGN) * ALIGN(720, DEFAULT_ALIGN) * 3 / 2;
  vb_config.astCommPool[1].u32BlkCnt = 4;
  vb_config.astCommPool[1].enRemapMode = VB_REMAP_MODE_CACHED;

  // Use the vendor platform entry point here, but report the exact failing
  // VB/SYS stage because CVI_TRACE_LOG is not visible in the app log.
  result = CVI_VB_SetConfig(&vb_config);
  std::printf("[camera] CVI_VB_SetConfig=%d\n", result);
  state.vb_started = false;
  state.system_started = false;
  if (result == CVI_SUCCESS)
  {
    result = CVI_VB_Init();
    std::printf("[camera] CVI_VB_Init=%d\n", result);
    state.vb_started = result == CVI_SUCCESS;
  }
  if (result == CVI_SUCCESS)
  {
    result = CVI_SYS_Init();
    std::printf("[camera] CVI_SYS_Init=%d\n", result);
    state.system_started = result == CVI_SUCCESS;
  }
  if (result != CVI_SUCCESS)
  {
    status_ = Status(result);
    return;
  }

  ISP_BIN_BYPASS_U bypass{};
  bypass.bitBypassFrameRate = CVI_TRUE;
  const char *const bypass_env = std::getenv("GUIDANCE_ISP_BIN_BYPASS");
  // Default on: without it the VI channel delivers nothing at all (measured:
  // 0 frames, 0 timeouts).  The escape hatch stays so the colour bring-up can
  // A/B it without a rebuild.
  if (bypass_env == nullptr || bypass_env[0] != '0')
  {
    result = CVI_ISP_BIN_SetBypassParams(CVI_BIN_ID_ISP0, &bypass);
  }
  else
  {
    result = CVI_SUCCESS;
  }
  std::printf("[camera] CVI_ISP_BIN_SetBypassParams=%d\n", result);
  if (result == CVI_SUCCESS)
  {
    result = SAMPLE_PLAT_VI_INIT(&state.vi_config);
    std::printf("[camera] SAMPLE_PLAT_VI_INIT=%d\n", result);
  }
  if (result != CVI_SUCCESS)
  {
    // SAMPLE_PLAT_VI_INIT performs its own partial VI cleanup on failure. The
    // state destructor must not repeat that cleanup.
    state.system_started = false;
    state.vb_started = false;
    status_ = Status(result);
    return;
  }
  state.vi_started = true;

  // What the driver actually accepted for the VI channel, as opposed to what we
  // asked for in the configuration: a YUV_400 channel here means the ISP writes
  // mono into it no matter how its own colour parameters look.
  {
    VI_CHN_ATTR_S channel_attr{};
    if (CVI_VI_GetChnAttr(state.pipe, state.vi_channel, &channel_attr) == CVI_SUCCESS)
    {
      std::printf("[camera] VI channel attr after start: fmt=%d size=%ux%u\n",
                  static_cast<int>(channel_attr.enPixelFormat), channel_attr.stSize.u32Width,
                  channel_attr.stSize.u32Height);
    }
    else
    {
      std::printf("[camera] CVI_VI_GetChnAttr failed\n");
    }
  }

  // Colour bring-up probe: the frame this pipeline hands out has neutral chroma
  // (every VU sample 128), which looks like "the beacon has no colour" to any
  // detector.  The ISP's saturation is the first thing to rule out: a PQ bin that
  // does not match the middleware can leave it at 0, and 0 is exactly a grey
  // picture.  Read it always; GUIDANCE_ISP_SAT=<0..255> forces a manual value.
  {
    ISP_SATURATION_ATTR_S saturation{};
    if (CVI_ISP_GetSaturationAttr(state.pipe, &saturation) == CVI_SUCCESS)
    {
      std::printf("[camera] ISP saturation: op=%d manual=%u\n",
                  static_cast<int>(saturation.enOpType),
                  static_cast<unsigned>(saturation.stManual.Saturation));
    }
    // The matrix that turns the demosaiced RGB into the output colour is the
    // other classic way to get grey: a CCM of all zeros (or Enable=0 with an
    // empty manual matrix) removes chroma without touching luma.
    ISP_CCM_ATTR_S ccm{};
    if (CVI_ISP_GetCCMAttr(state.pipe, &ccm) == CVI_SUCCESS)
    {
      std::printf("[camera] ISP CCM: enable=%d op=%d m=[%d %d %d %d %d %d %d %d %d]\n",
                  static_cast<int>(ccm.Enable), static_cast<int>(ccm.enOpType),
                  static_cast<int>(ccm.stManual.CCM[0]), static_cast<int>(ccm.stManual.CCM[1]),
                  static_cast<int>(ccm.stManual.CCM[2]), static_cast<int>(ccm.stManual.CCM[3]),
                  static_cast<int>(ccm.stManual.CCM[4]), static_cast<int>(ccm.stManual.CCM[5]),
                  static_cast<int>(ccm.stManual.CCM[6]), static_cast<int>(ccm.stManual.CCM[7]),
                  static_cast<int>(ccm.stManual.CCM[8]));
    }

    const char *const sat_env = std::getenv("GUIDANCE_ISP_SAT");
    if (sat_env != nullptr)
    {
      const int value = std::atoi(sat_env);
      if (value >= 0 && value <= 255)
      {
        saturation.enOpType = OP_TYPE_MANUAL;
        saturation.stManual.Saturation = static_cast<CVI_U8>(value);
        const CVI_S32 sat_result = CVI_ISP_SetSaturationAttr(state.pipe, &saturation);
        std::printf("[camera] ISP saturation <- %d (result %d)\n", value, sat_result);
      }
    }
  }

  // The PIC_SIZE table only knows the full sensor geometry, so a cropped sensor
  // mode (640x312, 640x256) is reported as 640x480 there.  Everything that
  // crops or scales the VI frame - the VPSS aspect-ratio crop in particular -
  // would then centre-cut a region the sensor never produced, and the bottom of
  // every frame would come back as unwritten memory.  The ISP public attributes
  // carry the size the sensor actually streams, so prefer them.
  {
    ISP_PUB_ATTR_S mode_attr{};
    if (CVI_ISP_GetPubAttr(state.pipe, &mode_attr) == CVI_SUCCESS)
    {
      SIZE_S mode_size = state.sensor_size;
      if (mode_attr.stWndRect.u32Width > 0 && mode_attr.stWndRect.u32Height > 0)
      {
        mode_size.u32Width = mode_attr.stWndRect.u32Width;
        mode_size.u32Height = mode_attr.stWndRect.u32Height;
      }
      else if (mode_attr.stSnsSize.u32Width > 0 && mode_attr.stSnsSize.u32Height > 0)
      {
        mode_size.u32Width = mode_attr.stSnsSize.u32Width;
        mode_size.u32Height = mode_attr.stSnsSize.u32Height;
      }
      if (mode_size.u32Width != state.sensor_size.u32Width ||
          mode_size.u32Height != state.sensor_size.u32Height)
      {
        std::printf("[camera] VI mode %ux%u overrides PIC table %ux%u\n", mode_size.u32Width,
                    mode_size.u32Height, state.sensor_size.u32Width,
                    state.sensor_size.u32Height);
      }
      state.sensor_size = mode_size;
    }
  }

  if (!ApplyManualIsp(state.pipe, config.manual_isp))
  {
    status_ = Status(kNotReady);
    return;
  }

  ISP_PUB_ATTR_S pub_attr{};
  if (CVI_ISP_GetPubAttr(state.pipe, &pub_attr) == CVI_SUCCESS)
  {
    std::printf("[camera] ISP pub attr: bayer=%d wdr=%d fps=%.1f\n",
                static_cast<int>(pub_attr.enBayer), static_cast<int>(pub_attr.enWDRMode),
                pub_attr.f32FrameRate);
  }
  if (CVI_ISP_GetPubAttr(state.pipe, &pub_attr) == CVI_SUCCESS &&
      pub_attr.f32FrameRate >= 1.0f && pub_attr.f32FrameRate <= 1000.0f)
  {
    state.sensor_fps = static_cast<int>(pub_attr.f32FrameRate + 0.5f);
  }
  else
  {
    ISP_PUB_ATTR_S configured_attr{};
    if (SAMPLE_COMM_ISP_GetIspAttrBySns(ini.enSnsType[0], &configured_attr) ==
            CVI_SUCCESS &&
        configured_attr.f32FrameRate >= 1.0f && configured_attr.f32FrameRate <= 1000.0f)
    {
      state.sensor_fps = static_cast<int>(configured_attr.f32FrameRate + 0.5f);
    }
  }

  impl_ = std::move(implementation);
  status_ = Status(CVI_SUCCESS);
}


Status Context::ApplyNoiseProfile()
{
  if (impl_ == nullptr || impl_->state == nullptr || !impl_->state->vi_started)
  {
    return Status(kNotReady);
  }
  const VI_PIPE pipe = impl_->state->pipe;

  // The strength curves are indexed by ISO, and manual AE pins the ISO in the
  // first bucket, so index 0 is the only entry that matters here.  The vendor
  // defaults at that index are close to "off" while a 500 us frame is read
  // noise limited, which is the coloured speckle.  Values are the ones measured
  // on this board; the environment overrides exist so the three strengths can be
  // swept without a rebuild.
  const auto from_env = [](const char *name, int fallback) {
    const char *text = std::getenv(name);
    if (text == nullptr || text[0] == '\0')
    {
      return fallback;
    }
    const int value = std::atoi(text);
    return value < 0 ? 0 : (value > 255 ? 255 : value);
  };
  const auto nr_strength = static_cast<CVI_U8>(from_env("GUIDANCE_NR", 24));
  const auto tnr_strength = static_cast<CVI_U8>(from_env("GUIDANCE_TNR", 24));
  const auto sharpen_gain = static_cast<CVI_U8>(from_env("GUIDANCE_SHARPEN", 8));

  int nr_readback = -1;
  int ynr_readback = -1;
  int tnr_readback = -1;
  int sharpen_readback = -1;

  ISP_NR_ATTR_S nr{};
  if (CVI_ISP_GetNRAttr(pipe, &nr) == CVI_SUCCESS)
  {
    std::printf("[camera] NRAttr enable=%d op=%d before=%u\n", static_cast<int>(nr.Enable),
                static_cast<int>(nr.enOpType), nr.stAuto.NoiseSuppressStr[0]);
    nr.Enable = CVI_TRUE;
    nr.enOpType = OP_TYPE_MANUAL;
    nr.stManual.NoiseSuppressStr = nr_strength;
    if (CVI_ISP_SetNRAttr(pipe, &nr) == CVI_SUCCESS)
    {
      ISP_NR_ATTR_S back{};
      if (CVI_ISP_GetNRAttr(pipe, &back) == CVI_SUCCESS)
      {
        nr_readback = back.stManual.NoiseSuppressStr;
      }
    }
  }

  ISP_YNR_ATTR_S ynr{};
  if (CVI_ISP_GetYNRAttr(pipe, &ynr) == CVI_SUCCESS)
  {
    std::printf("[camera] YNRAttr enable=%d op=%d\n", static_cast<int>(ynr.Enable),
                static_cast<int>(ynr.enOpType));
    ynr.Enable = CVI_TRUE;
    ynr.enOpType = OP_TYPE_MANUAL;
    ynr.stManual.NoiseSuppressStr = nr_strength;
    if (CVI_ISP_SetYNRAttr(pipe, &ynr) == CVI_SUCCESS)
    {
      ISP_YNR_ATTR_S back{};
      if (CVI_ISP_GetYNRAttr(pipe, &back) == CVI_SUCCESS)
      {
        ynr_readback = back.stManual.NoiseSuppressStr;
      }
    }
  }

  ISP_TNR_ATTR_S tnr{};
  if (CVI_ISP_GetTNRAttr(pipe, &tnr) == CVI_SUCCESS)
  {
    std::printf("[camera] TNRAttr enable=%d op=%d\n", static_cast<int>(tnr.Enable),
                static_cast<int>(tnr.enOpType));
    tnr.Enable = CVI_TRUE;
    tnr.enOpType = OP_TYPE_MANUAL;
    tnr.stManual.TnrStrength0 = tnr_strength;
    if (CVI_ISP_SetTNRAttr(pipe, &tnr) == CVI_SUCCESS)
    {
      ISP_TNR_ATTR_S back{};
      if (CVI_ISP_GetTNRAttr(pipe, &back) == CVI_SUCCESS)
      {
        tnr_readback = back.stManual.TnrStrength0;
      }
    }
  }

  // Saturation and black level do not change the signal-to-noise ratio, but they
  // change what the noise looks like: lowering saturation turns the coloured
  // speckle grey, and raising the black level clips the noisy floor of the dark
  // areas.  Both are swept with the environment so the pair can be chosen by
  // measurement instead of by eye.
  const auto from_env_int = [](const char *name, int fallback, int lo, int hi) {
    const char *text = std::getenv(name);
    if (text == nullptr || text[0] == '\0')
    {
      return fallback;
    }
    const int value = std::atoi(text);
    return value < lo ? lo : (value > hi ? hi : value);
  };
  const int saturation = from_env_int("GUIDANCE_SAT", 100, 0, 100);
  const int black_level = from_env_int("GUIDANCE_BLACK", 0, 0, 255);
  int saturation_readback = -1;
  int black_readback = -1;
  ISP_SATURATION_ATTR_S sat{};
  if (CVI_ISP_GetSaturationAttr(pipe, &sat) == CVI_SUCCESS)
  {
    sat.enOpType = OP_TYPE_MANUAL;
    sat.stManual.Saturation = static_cast<CVI_U8>(saturation);
    if (CVI_ISP_SetSaturationAttr(pipe, &sat) == CVI_SUCCESS)
    {
      ISP_SATURATION_ATTR_S back{};
      if (CVI_ISP_GetSaturationAttr(pipe, &back) == CVI_SUCCESS)
      {
        saturation_readback = back.stManual.Saturation;
      }
    }
  }
  ISP_BLACK_LEVEL_ATTR_S black{};
  if (CVI_ISP_GetBlackLevelAttr(pipe, &black) == CVI_SUCCESS)
  {
    black.Enable = CVI_TRUE;
    black.enOpType = OP_TYPE_MANUAL;
    const auto offset = static_cast<CVI_U16>(black_level * 4);
    black.stManual.OffsetR = offset;
    black.stManual.OffsetGr = offset;
    black.stManual.OffsetGb = offset;
    black.stManual.OffsetB = offset;
    if (CVI_ISP_SetBlackLevelAttr(pipe, &black) == CVI_SUCCESS)
    {
      ISP_BLACK_LEVEL_ATTR_S back{};
      if (CVI_ISP_GetBlackLevelAttr(pipe, &back) == CVI_SUCCESS)
      {
        black_readback = static_cast<int>(back.stManual.OffsetR);
      }
    }
  }

  ISP_SHARPEN_ATTR_S sharp{};
  if (CVI_ISP_GetSharpenAttr(pipe, &sharp) == CVI_SUCCESS)
  {
    std::printf("[camera] SharpenAttr enable=%d op=%d\n", static_cast<int>(sharp.Enable),
                static_cast<int>(sharp.enOpType));
    sharp.enOpType = OP_TYPE_MANUAL;
    sharp.stManual.GlobalGain = sharpen_gain;
    if (CVI_ISP_SetSharpenAttr(pipe, &sharp) == CVI_SUCCESS)
    {
      ISP_SHARPEN_ATTR_S back{};
      if (CVI_ISP_GetSharpenAttr(pipe, &back) == CVI_SUCCESS)
      {
        sharpen_readback = back.stManual.GlobalGain;
      }
    }
  }

  std::printf("[camera] noise profile nr=%d->%d ynr=%d->%d tnr=%d->%d sharpen=%d->%d\n",
              nr_strength, nr_readback, nr_strength, ynr_readback, tnr_strength,
              tnr_readback, sharpen_gain, sharpen_readback);
  std::fflush(stdout);
  return Status(CVI_SUCCESS);
}

Context::~Context() { impl_.reset(); }

}  // namespace cvimpp
