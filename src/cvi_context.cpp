#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "cvi_ae.h"
#include "cvi_awb.h"
#include "cvi_internal.hpp"

namespace cvimpp
{
namespace
{

// Lock AE to a fixed exposure and AWB to fixed gains.  Called once after the
// VI/ISP pipeline is up; the sensor runs at a constant brightness afterwards,
// which is what a fixed-threshold Lab detector wants.
bool ApplyManualIsp(VI_PIPE pipe, const ManualIspConfig& manual)
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
  const bool lock_wb =
      manual.wb_rgain != 0 && manual.wb_bgain != 0 &&
      (manual.wb_rgain != 1024 || manual.wb_bgain != 1024 || manual.wb_ggain != 1024);
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

  // AF is not used by the fixed-focus SC035HGS; drop its statistics entirely.
  // AE/AWB statistics run at 1/12 and 1/24 of the frame rate, which is ample for
  // locked manual exposure/white balance and is the dominant per-frame ISP cost
  // at 180 fps.
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

  std::printf(
      "[camera] manual ISP locked: exposure=%uus again=%u dgain=%u "
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

Context::Context(ContextConfig config, Status* error)
{
  using detail::ContextState;

  if (config.vi_pool_size == 0 || config.vi_pool_count == 0)
  {
    *error = Status(INVALID_ARGUMENT);
    return;
  }

  auto implementation = std::make_unique<Impl>();
  implementation->state = std::make_shared<ContextState>();
  auto& state = *implementation->state;

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
      *error = Status(result);
      return;
    }
  }

  SAMPLE_INI_CFG_S ini{};
  // The sample parser fills its built-in default even when no ini file is
  // present. This is the same fallback used by the vendor sample programs.
  SAMPLE_COMM_VI_ParseIni(&ini);
  if (ini.devNum == 0 || ini.devNum > VI_MAX_DEV_NUM)
  {
    *error = Status(INVALID_ARGUMENT);
    return;
  }

  int result = CVI_VI_SetDevNum(ini.devNum);
  if (result != CVI_SUCCESS)
  {
    *error = Status(result);
    return;
  }

  result = SAMPLE_COMM_VI_IniToViCfg(&ini, &state.vi_config);
  if (result != CVI_SUCCESS)
  {
    *error = Status(result);
    return;
  }

#if defined(SENSOR_SMS_SC035HGS)
  // `sensor_cfg.ini` names SMS_SC035HGS_MIPI_480P_180FPS_10BIT, and that name is
  // resolved by `snsr_type_name[]` -- a table that lives in `libsns_full.so`,
  // i.e. in the *middleware* the runtime loads.  The index it returns is then
  // compared by `SAMPLE_COMM_SNS_GetSnsObj` against the enumerator *this* library
  // was compiled against.  A middleware older than the mode resolves the name to
  // an index it does not know, so the lookup returns NULL and
  // `SAMPLE_PLAT_VI_INIT()` fails with -1 and no further message -- which is
  // exactly what happened while this build still pointed at a stale middleware
  // checkout.  Naming the enumerator turns that silent mismatch into a build
  // error.
  static_assert(SMS_SC035HGS_MIPI_480P_180FPS_10BIT < SAMPLE_SNS_TYPE_BUTT,
                "the linked CVI middleware does not provide the 180 fps SC035HGS mode");
#endif

  PIC_SIZE_E sensor_pic_size{};
  result = SAMPLE_COMM_VI_GetSizeBySensor(ini.enSnsType[0], &sensor_pic_size);
  if (result == CVI_SUCCESS)
  {
    result = SAMPLE_COMM_SYS_GetPicSize(sensor_pic_size, &state.sensor_size);
  }
  if (result != CVI_SUCCESS || state.sensor_size.u32Width == 0 ||
      state.sensor_size.u32Height == 0)
  {
    *error = Status(result != CVI_SUCCESS ? result : INVALID_ARGUMENT);
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
              static_cast<int>(PIXEL_FORMAT_NV21),
              static_cast<int>(PIXEL_FORMAT_YUV_400));
  state.pipe = state.vi_config.astViInfo[0].stPipeInfo.aPipe[0];
  state.vi_channel = state.vi_config.astViInfo[0].stChnInfo.ViChn;
  state.vi_format = state.vi_config.astViInfo[0].stChnInfo.enPixFormat;

  VB_CAL_CONFIG_S calculated{};
  COMMON_GetPicBufferConfig(state.sensor_size.u32Width, state.sensor_size.u32Height,
                            state.vi_format, DATA_BITWIDTH_8, COMPRESS_MODE_NONE,
                            DEFAULT_ALIGN, &calculated);
  if (calculated.u32VBSize == 0)
  {
    *error = Status(INVALID_ARGUMENT);
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
    *error = Status(result);
    return;
  }

  ISP_BIN_BYPASS_U bypass{};
  bypass.bitBypassFrameRate = CVI_TRUE;
  const char* const bypass_env = std::getenv("GUIDANCE_ISP_BIN_BYPASS");
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
    *error = Status(result);
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
                  static_cast<int>(channel_attr.enPixelFormat),
                  channel_attr.stSize.u32Width, channel_attr.stSize.u32Height);
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
      std::printf(
          "[camera] ISP CCM: enable=%d op=%d m=[%d %d %d %d %d %d %d %d %d]\n",
          static_cast<int>(ccm.Enable), static_cast<int>(ccm.enOpType),
          static_cast<int>(ccm.stManual.CCM[0]), static_cast<int>(ccm.stManual.CCM[1]),
          static_cast<int>(ccm.stManual.CCM[2]), static_cast<int>(ccm.stManual.CCM[3]),
          static_cast<int>(ccm.stManual.CCM[4]), static_cast<int>(ccm.stManual.CCM[5]),
          static_cast<int>(ccm.stManual.CCM[6]), static_cast<int>(ccm.stManual.CCM[7]),
          static_cast<int>(ccm.stManual.CCM[8]));
    }

    const char* const sat_env = std::getenv("GUIDANCE_ISP_SAT");
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
        std::printf("[camera] VI mode %ux%u overrides PIC table %ux%u\n",
                    mode_size.u32Width, mode_size.u32Height, state.sensor_size.u32Width,
                    state.sensor_size.u32Height);
      }
      state.sensor_size = mode_size;
    }
  }

  if (!ApplyManualIsp(state.pipe, config.manual_isp))
  {
    *error = Status(NOT_READY);
    return;
  }

  ISP_PUB_ATTR_S pub_attr{};
  const bool have_pub_attr = CVI_ISP_GetPubAttr(state.pipe, &pub_attr) == CVI_SUCCESS;
  if (have_pub_attr)
  {
    std::printf("[camera] ISP pub attr: bayer=%d wdr=%d fps=%.1f\n",
                static_cast<int>(pub_attr.enBayer), static_cast<int>(pub_attr.enWDRMode),
                pub_attr.f32FrameRate);
  }
  if (have_pub_attr && pub_attr.f32FrameRate >= 1.0f && pub_attr.f32FrameRate <= 1000.0f)
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
  *error = Status(CVI_SUCCESS);
}

std::expected<Context, Status> Context::Create(ContextConfig config)
{
  Status error;
  Context context(config, &error);
  if (!error.Ok())
  {
    return std::unexpected(error);
  }
  return context;
}

Context::Context(Context&&) noexcept = default;
Context& Context::operator=(Context&&) noexcept = default;

Context::~Context() { impl_.reset(); }

}  // namespace cvimpp
