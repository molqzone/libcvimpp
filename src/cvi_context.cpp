#include "cvi_internal.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>

namespace cvimpp
{
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
  vb_config.u32MaxPoolCnt = 1;
  vb_config.astCommPool[0].u32BlkSize = pool_size;
  vb_config.astCommPool[0].u32BlkCnt = config.vi_pool_count;
  vb_config.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;

  result = CVI_VB_SetConfig(&vb_config);
  if (result == CVI_SUCCESS)
  {
    result = CVI_VB_Init();
    state.vb_started = result == CVI_SUCCESS;
  }
  if (result == CVI_SUCCESS)
  {
    result = CVI_SYS_Init();
    state.system_started = result == CVI_SUCCESS;
  }
  if (result != CVI_SUCCESS)
  {
    status_ = Status(result);
    return;
  }

  ISP_BIN_BYPASS_U bypass{};
  bypass.bitBypassFrameRate = CVI_TRUE;
  result = CVI_ISP_BIN_SetBypassParams(CVI_BIN_ID_ISP0, &bypass);
  if (result == CVI_SUCCESS)
  {
    result = SAMPLE_PLAT_VI_INIT(&state.vi_config);
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

  ISP_PUB_ATTR_S pub_attr{};
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

Context::~Context() { impl_.reset(); }

}  // namespace cvimpp
