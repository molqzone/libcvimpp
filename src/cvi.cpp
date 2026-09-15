#include "cvimpp/cvi.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

#include "cvi_bin.h"
#include "cvi_buffer.h"
#include "cvi_isp.h"
#include "cvi_sys.h"
#include "cvi_vb.h"
#include "cvi_venc.h"
#include "cvi_vi.h"
#include "cvi_vpss.h"
#include "sample_comm.h"

namespace cvimpp
{
namespace
{

constexpr int kFrameTimeoutMs = 100;
constexpr std::size_t kMaxStreamPacks = 8;

struct ContextState
{
  SAMPLE_VI_CONFIG_S vi_config{};
  SIZE_S sensor_size{};
  VI_PIPE pipe{0};
  VI_CHN vi_channel{0};
  PIXEL_FORMAT_E vi_format{PIXEL_FORMAT_NV21};
  int sensor_fps{30};
  bool vb_started{};
  bool system_started{};
  bool vi_started{};

  ~ContextState();
};

struct VpssResource
{
  std::shared_ptr<ContextState> context;
  VPSS_GRP group{VPSS_INVALID_GRP};
  VPSS_CHN channel{VPSS_CHN0};
  bool group_created{};
  bool channel_enabled{};
  bool group_started{};
  bool vi_bound{};

  ~VpssResource();
};

struct EncoderState
{
  std::shared_ptr<ContextState> context;
  std::shared_ptr<VpssResource> converter;
  VENC_CHN channel{-1};
  PixelFormat input_format{PixelFormat::Unknown};
  PAYLOAD_TYPE_E payload{PT_H264};
  int width{};
  int height{};
  bool channel_created{};
  bool receiving{};
  bool frame_pending{};
  bool converted_frame_pending{};
  VIDEO_FRAME_INFO_S converted_frame{};

  ~EncoderState();
};

PIXEL_FORMAT_E ToVendorFormat(PixelFormat format)
{
  switch (format)
  {
    case PixelFormat::Nv21:
      return PIXEL_FORMAT_NV21;
    case PixelFormat::Rgb888:
      return PIXEL_FORMAT_RGB_888;
    case PixelFormat::Unknown:
      break;
  }
  return PIXEL_FORMAT_MAX;
}

PixelFormat FromVendorFormat(PIXEL_FORMAT_E format)
{
  switch (format)
  {
    case PIXEL_FORMAT_NV21:
      return PixelFormat::Nv21;
    case PIXEL_FORMAT_RGB_888:
      return PixelFormat::Rgb888;
    default:
      return PixelFormat::Unknown;
  }
}

PAYLOAD_TYPE_E ToPayloadType(Codec codec)
{
  switch (codec)
  {
    case Codec::Jpeg:
      return PT_JPEG;
    case Codec::H265:
      return PT_H265;
    case Codec::H264:
      return PT_H264;
  }
  return PT_BUTT;
}

bool IsSupportedPayload(PAYLOAD_TYPE_E payload)
{
  return payload == PT_JPEG || payload == PT_H264 || payload == PT_H265;
}

std::uint32_t FrameBufferSize(const VIDEO_FRAME_INFO_S &frame)
{
  std::uint64_t size = 0;
  for (const CVI_U32 length : frame.stVFrame.u32Length)
  {
    size += length;
  }
  if (size == 0 || size > std::numeric_limits<std::uint32_t>::max())
  {
    return 0;
  }
  return static_cast<std::uint32_t>(size);
}

void ReleaseFrameToVpss(VpssResource &resource, VIDEO_FRAME_INFO_S &frame)
{
  CVI_VPSS_ReleaseChnFrame(resource.group, resource.channel, &frame);
}

void ReleaseConvertedFrame(EncoderState &state)
{
  if (!state.converted_frame_pending || !state.converter)
  {
    return;
  }

  ReleaseFrameToVpss(*state.converter, state.converted_frame);
  state.converted_frame = {};
  state.converted_frame_pending = false;
}

void UnbindVi(VpssResource &resource)
{
  if (!resource.vi_bound || !resource.context)
  {
    return;
  }

  MMF_CHN_S source{};
  source.enModId = CVI_ID_VI;
  source.s32DevId = resource.context->pipe;
  source.s32ChnId = resource.context->vi_channel;

  MMF_CHN_S destination{};
  destination.enModId = CVI_ID_VPSS;
  destination.s32DevId = resource.group;
  destination.s32ChnId = resource.channel;

  CVI_SYS_UnBind(&source, &destination);
  resource.vi_bound = false;
}

std::shared_ptr<VpssResource> CreateVpssResource(
    const std::shared_ptr<ContextState> &context, const VPSS_GRP_ATTR_S &group_attr,
    const VPSS_CHN_ATTR_S &channel_attr, const VPSS_CROP_INFO_S *crop, bool bind_vi,
    int *error)
{
  auto resource = std::make_shared<VpssResource>();
  resource->context = context;
  resource->group = CVI_VPSS_GetAvailableGrp();
  if (resource->group < 0)
  {
    if (error != nullptr)
    {
      *error = resource->group;
    }
    return {};
  }

  int result = CVI_VPSS_CreateGrp(resource->group, &group_attr);
  if (result != CVI_SUCCESS)
  {
    if (error != nullptr)
    {
      *error = result;
    }
    return {};
  }
  resource->group_created = true;

  result = CVI_VPSS_ResetGrp(resource->group);
  if (result == CVI_SUCCESS)
  {
    result = CVI_VPSS_SetChnAttr(resource->group, resource->channel, &channel_attr);
  }
  if (result == CVI_SUCCESS && crop != nullptr)
  {
    result = CVI_VPSS_SetChnCrop(resource->group, resource->channel, crop);
  }
  if (result == CVI_SUCCESS)
  {
    result = CVI_VPSS_EnableChn(resource->group, resource->channel);
    resource->channel_enabled = result == CVI_SUCCESS;
  }
  if (result == CVI_SUCCESS)
  {
    result = CVI_VPSS_StartGrp(resource->group);
    resource->group_started = result == CVI_SUCCESS;
  }
  if (result == CVI_SUCCESS && bind_vi)
  {
    MMF_CHN_S source{};
    source.enModId = CVI_ID_VI;
    source.s32DevId = context->pipe;
    source.s32ChnId = context->vi_channel;

    MMF_CHN_S destination{};
    destination.enModId = CVI_ID_VPSS;
    destination.s32DevId = resource->group;
    destination.s32ChnId = resource->channel;

    result = CVI_SYS_Bind(&source, &destination);
    resource->vi_bound = result == CVI_SUCCESS;
  }

  if (result != CVI_SUCCESS)
  {
    if (error != nullptr)
    {
      *error = result;
    }
    return {};
  }

  if (error != nullptr)
  {
    *error = CVI_SUCCESS;
  }
  return resource;
}

void ConfigureAspectRatio(VPSS_CHN_ATTR_S &attribute, FitMode fit, PIXEL_FORMAT_E output_format,
                          int source_width, int source_height, int output_width,
                          int output_height, VPSS_CROP_INFO_S *crop)
{
  attribute.stAspectRatio.bEnableBgColor = CVI_TRUE;
  attribute.stAspectRatio.u32BgColor = COLOR_RGB_BLACK;
  attribute.stNormalize.bEnable = CVI_FALSE;

  if (fit == FitMode::Stretch)
  {
    attribute.stAspectRatio.enMode = ASPECT_RATIO_MANUAL;
    attribute.stAspectRatio.stVideoRect.s32X = 0;
    attribute.stAspectRatio.stVideoRect.s32Y = 0;
    attribute.stAspectRatio.stVideoRect.u32Width = static_cast<CVI_U32>(output_width);
    attribute.stAspectRatio.stVideoRect.u32Height = static_cast<CVI_U32>(output_height);
    return;
  }

  attribute.stAspectRatio.enMode = ASPECT_RATIO_AUTO;
  if (fit != FitMode::Crop || crop == nullptr)
  {
    return;
  }

  const std::uint64_t source_w = static_cast<std::uint64_t>(source_width);
  const std::uint64_t source_h = static_cast<std::uint64_t>(source_height);
  const std::uint64_t output_w = static_cast<std::uint64_t>(output_width);
  const std::uint64_t output_h = static_cast<std::uint64_t>(output_height);

  std::uint64_t crop_width = source_w;
  std::uint64_t crop_height = source_h;
  if (source_w * output_h > source_h * output_w)
  {
    crop_width = source_h * output_w / output_h;
  }
  else
  {
    crop_height = source_w * output_h / output_w;
  }

  // NV21 requires even crop dimensions and coordinates.  Rounding down keeps
  // the rectangle inside the source while preserving the requested ratio.
  if (output_format == PIXEL_FORMAT_NV21)
  {
    crop_width &= ~std::uint64_t{1};
    crop_height &= ~std::uint64_t{1};
  }

  crop->bEnable = CVI_TRUE;
  crop->enCropCoordinate = VPSS_CROP_ABS_COOR;
  std::uint64_t crop_x = (source_w - crop_width) / 2u;
  std::uint64_t crop_y = (source_h - crop_height) / 2u;
  if (output_format == PIXEL_FORMAT_NV21)
  {
    crop_x &= ~std::uint64_t{1};
    crop_y &= ~std::uint64_t{1};
  }
  crop->stCropRect.s32X = static_cast<CVI_S32>(crop_x);
  crop->stCropRect.s32Y = static_cast<CVI_S32>(crop_y);
  crop->stCropRect.u32Width = static_cast<CVI_U32>(crop_width);
  crop->stCropRect.u32Height = static_cast<CVI_U32>(crop_height);
}

void ReleaseVenc(EncoderState &state)
{
  ReleaseConvertedFrame(state);

  if (!state.channel_created)
  {
    return;
  }

  if (state.receiving)
  {
    CVI_VENC_StopRecvFrame(state.channel);
    state.receiving = false;
  }
  CVI_VENC_ResetChn(state.channel);
  CVI_VENC_DestroyChn(state.channel);
  state.channel_created = false;
  state.channel = -1;
  state.frame_pending = false;
}

}  // namespace

struct Context::Impl
{
  std::shared_ptr<ContextState> state;
};

struct Frame::Impl
{
  std::shared_ptr<VpssResource> source;
  VIDEO_FRAME_INFO_S info{};
  void *mapped{};
  CVI_U32 mapped_size{};
  PixelFormat format{PixelFormat::Unknown};
};

struct Capture::Impl
{
  std::shared_ptr<VpssResource> source;
  PixelFormat format{PixelFormat::Unknown};
};

struct Encoder::Impl
{
  std::shared_ptr<EncoderState> state;
};

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

VpssResource::~VpssResource()
{
  UnbindVi(*this);

  if (channel_enabled)
  {
    CVI_VPSS_DisableChn(group, channel);
    channel_enabled = false;
  }
  if (group_started)
  {
    CVI_VPSS_StopGrp(group);
    group_started = false;
  }
  if (group_created)
  {
    CVI_VPSS_DestroyGrp(group);
    group_created = false;
  }
  group = VPSS_INVALID_GRP;
}

EncoderState::~EncoderState() { ReleaseVenc(*this); }

bool Status::IsFrameDropped() const
{
  return code_ == static_cast<int>(CVI_ERR_VENC_FRC_NO_ENC);
}

Context::Context(ContextConfig config)
{
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
  // present.  This is the same fallback used by the vendor sample programs.
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
  if (result != CVI_SUCCESS || state.sensor_size.u32Width == 0 || state.sensor_size.u32Height == 0)
  {
    status_ = Status(result != CVI_SUCCESS ? result : kInvalidArgument);
    return;
  }

  state.pipe = state.vi_config.astViInfo[0].stPipeInfo.aPipe[0];
  state.vi_channel = state.vi_config.astViInfo[0].stChnInfo.ViChn;
  state.vi_format = state.vi_config.astViInfo[0].stChnInfo.enPixFormat;

  VB_CAL_CONFIG_S calculated{};
  COMMON_GetPicBufferConfig(state.sensor_size.u32Width, state.sensor_size.u32Height,
                            state.vi_format, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN,
                            &calculated);
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
    // SAMPLE_PLAT_VI_INIT performs its own partial VI cleanup on failure.  The
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
    if (SAMPLE_COMM_ISP_GetIspAttrBySns(ini.enSnsType[0], &configured_attr) == CVI_SUCCESS &&
        configured_attr.f32FrameRate >= 1.0f && configured_attr.f32FrameRate <= 1000.0f)
    {
      state.sensor_fps = static_cast<int>(configured_attr.f32FrameRate + 0.5f);
    }
  }

  impl_ = std::move(implementation);
  status_ = Status(CVI_SUCCESS);
}

Context::~Context() { impl_.reset(); }

Frame::Frame() = default;

Frame::~Frame() { Release(); }

Frame::Frame(Frame &&other) noexcept : impl_(std::move(other.impl_)) {}

Frame &Frame::operator=(Frame &&other) noexcept
{
  if (this != &other)
  {
    Release();
    impl_ = std::move(other.impl_);
  }
  return *this;
}

void Frame::Assign(std::unique_ptr<Impl> frame) { impl_ = std::move(frame); }

void *Frame::Data() { return impl_ != nullptr ? impl_->mapped : nullptr; }

const void *Frame::Data() const { return impl_ != nullptr ? impl_->mapped : nullptr; }

int Frame::Length() const
{
  return impl_ != nullptr ? static_cast<int>(impl_->mapped_size) : 0;
}

int Frame::Width() const
{
  return impl_ != nullptr ? static_cast<int>(impl_->info.stVFrame.u32Width) : 0;
}

int Frame::Height() const
{
  return impl_ != nullptr ? static_cast<int>(impl_->info.stVFrame.u32Height) : 0;
}

PixelFormat Frame::Format() const
{
  return impl_ != nullptr ? impl_->format : PixelFormat::Unknown;
}

FrameView Frame::View() const
{
  if (impl_ == nullptr)
  {
    return {};
  }
  return FrameView{static_cast<const std::uint8_t *>(impl_->mapped),
                   static_cast<std::size_t>(impl_->mapped_size), Width(), Height(), Format()};
}

void Frame::Release()
{
  if (impl_ == nullptr)
  {
    return;
  }

  if (impl_->mapped != nullptr)
  {
    CVI_SYS_Munmap(impl_->mapped, impl_->mapped_size);
    impl_->mapped = nullptr;
  }
  if (impl_->source != nullptr)
  {
    ReleaseFrameToVpss(*impl_->source, impl_->info);
  }
  impl_.reset();
}

Capture::Capture(Context &context, CaptureConfig config) : context_(&context)
{
  if (!context.Ok() || context.impl_ == nullptr || context.impl_->state == nullptr ||
      config.width <= 0 || config.height <= 0 || config.fps <= 0 || config.depth < 0)
  {
    status_ = context.Ok() ? Status(kInvalidArgument) : context.Result();
    return;
  }

  const PIXEL_FORMAT_E output_format = ToVendorFormat(config.format);
  if (output_format == PIXEL_FORMAT_MAX)
  {
    status_ = Status(kInvalidArgument);
    return;
  }

  auto implementation = std::make_unique<Impl>();
  auto &state = *context.impl_->state;

  VPSS_GRP_ATTR_S group_attr{};
  group_attr.u32MaxW = state.sensor_size.u32Width;
  group_attr.u32MaxH = state.sensor_size.u32Height;
  group_attr.enPixelFormat = state.vi_format;
  group_attr.stFrameRate.s32SrcFrameRate = -1;
  group_attr.stFrameRate.s32DstFrameRate = -1;
  group_attr.u8VpssDev = 0;

  VPSS_CHN_ATTR_S channel_attr{};
  channel_attr.u32Width = static_cast<CVI_U32>(config.width);
  channel_attr.u32Height = static_cast<CVI_U32>(config.height);
  channel_attr.enVideoFormat = VIDEO_FORMAT_LINEAR;
  channel_attr.enPixelFormat = output_format;
  channel_attr.stFrameRate.s32SrcFrameRate = state.sensor_fps;
  channel_attr.stFrameRate.s32DstFrameRate = config.fps;
  channel_attr.u32Depth = static_cast<CVI_U32>(config.depth);
  VPSS_CROP_INFO_S crop{};
  ConfigureAspectRatio(channel_attr, config.fit, output_format,
                       static_cast<int>(state.sensor_size.u32Width),
                       static_cast<int>(state.sensor_size.u32Height), config.width, config.height,
                       &crop);

  int result = CVI_SUCCESS;
  implementation->source = CreateVpssResource(context.impl_->state, group_attr, channel_attr,
                                               config.fit == FitMode::Crop ? &crop : nullptr, true,
                                               &result);
  if (implementation->source == nullptr)
  {
    status_ = Status(result);
    return;
  }
  implementation->format = config.format;
  impl_ = std::move(implementation);
  status_ = Status(CVI_SUCCESS);
}

Capture::~Capture() { impl_.reset(); }

int Capture::Channel() const
{
  return impl_ != nullptr && impl_->source != nullptr ? impl_->source->group : -1;
}

PixelFormat Capture::Format() const
{
  return impl_ != nullptr ? impl_->format : PixelFormat::Unknown;
}

Status Capture::Pop(Frame &frame)
{
  if (!Ok() || impl_ == nullptr || impl_->source == nullptr || frame.Valid())
  {
    return Status(kInvalidArgument);
  }

  VIDEO_FRAME_INFO_S vendor_frame{};
  const int result = CVI_VPSS_GetChnFrame(impl_->source->group, impl_->source->channel,
                                          &vendor_frame, kFrameTimeoutMs);
  if (result != CVI_SUCCESS)
  {
    return Status(result);
  }

  const std::uint32_t mapped_size = FrameBufferSize(vendor_frame);
  if (mapped_size == 0 || vendor_frame.stVFrame.u64PhyAddr[0] == 0)
  {
    ReleaseFrameToVpss(*impl_->source, vendor_frame);
    return Status(kInvalidArgument);
  }

  void *mapped = CVI_SYS_MmapCache(vendor_frame.stVFrame.u64PhyAddr[0], mapped_size);
  if (mapped == nullptr)
  {
    ReleaseFrameToVpss(*impl_->source, vendor_frame);
    return Status(CVI_FAILURE);
  }

  auto frame_impl = std::make_unique<Frame::Impl>();
  frame_impl->source = impl_->source;
  frame_impl->info = vendor_frame;
  frame_impl->mapped = mapped;
  frame_impl->mapped_size = mapped_size;
  frame_impl->format = FromVendorFormat(vendor_frame.stVFrame.enPixelFormat);
  if (frame_impl->format == PixelFormat::Unknown)
  {
    CVI_SYS_Munmap(mapped, mapped_size);
    ReleaseFrameToVpss(*impl_->source, vendor_frame);
    return Status(kInvalidArgument);
  }

  frame.Assign(std::move(frame_impl));
  return Status(CVI_SUCCESS);
}

std::span<const std::uint8_t> EncodedAccessUnitView::Pack(std::size_t index) const
{
  if (index >= count_)
  {
    return {};
  }
  return packs_[index].Bytes();
}

Encoder::Encoder(Context &context, EncoderConfig config) : context_(&context)
{
  const PAYLOAD_TYPE_E payload = ToPayloadType(config.codec);
  const PIXEL_FORMAT_E input_format = ToVendorFormat(config.format);
  if (!context.Ok() || context.impl_ == nullptr || context.impl_->state == nullptr ||
      !IsSupportedPayload(payload) || input_format == PIXEL_FORMAT_MAX || config.width <= 0 ||
      config.height <= 0 || config.input_fps <= 0 || config.output_fps <= 0 ||
      config.output_fps > config.input_fps || config.bitrate_kbps <= 0 ||
      config.jpg_quality < 0 || config.jpg_quality > 100 ||
      (payload != PT_JPEG && config.gop <= 0))
  {
    status_ = context.Ok() ? Status(kInvalidArgument) : context.Result();
    return;
  }

  VENC_CHN_ATTR_S channel_attr{};
  channel_attr.stVencAttr.enType = payload;
  channel_attr.stVencAttr.u32MaxPicWidth = static_cast<CVI_U32>(config.width);
  channel_attr.stVencAttr.u32MaxPicHeight = static_cast<CVI_U32>(config.height);
  channel_attr.stVencAttr.u32BufSize = std::max<CVI_U32>(1024u * 1024u,
                                                         static_cast<CVI_U32>(config.width) *
                                                             static_cast<CVI_U32>(config.height) *
                                                             2u);
  channel_attr.stVencAttr.bByFrame = CVI_TRUE;
  channel_attr.stVencAttr.u32PicWidth = static_cast<CVI_U32>(config.width);
  channel_attr.stVencAttr.u32PicHeight = static_cast<CVI_U32>(config.height);

  if (payload == PT_JPEG)
  {
    channel_attr.stVencAttr.bEsBufQueueEn = CVI_FALSE;
    channel_attr.stVencAttr.bIsoSendFrmEn = CVI_FALSE;
    channel_attr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
    channel_attr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
  }
  else if (payload == PT_H264)
  {
    channel_attr.stVencAttr.bEsBufQueueEn = CVI_TRUE;
    channel_attr.stVencAttr.bIsoSendFrmEn = CVI_TRUE;
    channel_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    channel_attr.stGopAttr.stNormalP.s32IPQpDelta = 2;
    channel_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    channel_attr.stRcAttr.stH264Cbr.u32Gop = static_cast<CVI_U32>(config.gop);
    channel_attr.stRcAttr.stH264Cbr.u32StatTime = 2;
    channel_attr.stRcAttr.stH264Cbr.u32SrcFrameRate = static_cast<CVI_U32>(config.input_fps);
    channel_attr.stRcAttr.stH264Cbr.fr32DstFrameRate = config.output_fps;
    channel_attr.stRcAttr.stH264Cbr.u32BitRate = static_cast<CVI_U32>(config.bitrate_kbps);
    channel_attr.stRcAttr.stH264Cbr.bVariFpsEn = CVI_FALSE;
  }
  else
  {
    channel_attr.stVencAttr.bEsBufQueueEn = CVI_TRUE;
    channel_attr.stVencAttr.bIsoSendFrmEn = CVI_TRUE;
    channel_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    channel_attr.stGopAttr.stNormalP.s32IPQpDelta = 2;
    channel_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    channel_attr.stRcAttr.stH265Cbr.u32Gop = static_cast<CVI_U32>(config.gop);
    channel_attr.stRcAttr.stH265Cbr.u32StatTime = 2;
    channel_attr.stRcAttr.stH265Cbr.u32SrcFrameRate = static_cast<CVI_U32>(config.input_fps);
    channel_attr.stRcAttr.stH265Cbr.fr32DstFrameRate = config.output_fps;
    channel_attr.stRcAttr.stH265Cbr.u32BitRate = static_cast<CVI_U32>(config.bitrate_kbps);
    channel_attr.stRcAttr.stH265Cbr.bVariFpsEn = CVI_FALSE;
  }

  auto implementation = std::make_unique<Impl>();
  implementation->state = std::make_shared<EncoderState>();
  auto &state = *implementation->state;
  state.context = context.impl_->state;
  state.input_format = config.format;
  state.payload = payload;
  state.width = config.width;
  state.height = config.height;

  int create_result = CVI_FAILURE;
  for (int channel = 0; channel < VENC_MAX_CHN_NUM; ++channel)
  {
    create_result = CVI_VENC_CreateChn(channel, &channel_attr);
    if (create_result == CVI_SUCCESS)
    {
      state.channel = channel;
      state.channel_created = true;
      break;
    }
  }
  if (!state.channel_created)
  {
    status_ = Status(create_result);
    return;
  }

  // VENC accepts YUV input.  Keep RGB capture useful by doing the conversion
  // in a small private VPSS group before SendFrame.
  if (config.format == PixelFormat::Rgb888)
  {
    VPSS_GRP_ATTR_S group_attr{};
    group_attr.u32MaxW = static_cast<CVI_U32>(config.width);
    group_attr.u32MaxH = static_cast<CVI_U32>(config.height);
    group_attr.enPixelFormat = PIXEL_FORMAT_RGB_888;
    group_attr.stFrameRate.s32SrcFrameRate = -1;
    group_attr.stFrameRate.s32DstFrameRate = -1;
    group_attr.u8VpssDev = 0;

    VPSS_CHN_ATTR_S conversion_attr{};
    conversion_attr.u32Width = static_cast<CVI_U32>(config.width);
    conversion_attr.u32Height = static_cast<CVI_U32>(config.height);
    conversion_attr.enVideoFormat = VIDEO_FORMAT_LINEAR;
    conversion_attr.enPixelFormat = PIXEL_FORMAT_YUV_PLANAR_420;
    conversion_attr.stFrameRate.s32SrcFrameRate = config.input_fps;
    conversion_attr.stFrameRate.s32DstFrameRate = config.output_fps;
    conversion_attr.u32Depth = 1;
    conversion_attr.stAspectRatio.enMode = ASPECT_RATIO_MANUAL;
    conversion_attr.stAspectRatio.stVideoRect.s32X = 0;
    conversion_attr.stAspectRatio.stVideoRect.s32Y = 0;
    conversion_attr.stAspectRatio.stVideoRect.u32Width = static_cast<CVI_U32>(config.width);
    conversion_attr.stAspectRatio.stVideoRect.u32Height = static_cast<CVI_U32>(config.height);
    conversion_attr.stAspectRatio.bEnableBgColor = CVI_TRUE;
    conversion_attr.stAspectRatio.u32BgColor = COLOR_RGB_BLACK;
    conversion_attr.stNormalize.bEnable = CVI_FALSE;

    int result = CVI_SUCCESS;
    state.converter = CreateVpssResource(state.context, group_attr, conversion_attr, nullptr,
                                         false, &result);
    if (state.converter == nullptr)
    {
      ReleaseVenc(state);
      status_ = Status(result);
      return;
    }
  }

  VENC_RECV_PIC_PARAM_S receive{};
  receive.s32RecvPicNum = -1;
  int result = CVI_VENC_StartRecvFrame(state.channel, &receive);
  if (result != CVI_SUCCESS)
  {
    ReleaseVenc(state);
    status_ = Status(result);
    return;
  }
  state.receiving = true;

  if (payload == PT_JPEG)
  {
    VENC_JPEG_PARAM_S jpeg{};
    result = CVI_VENC_GetJpegParam(state.channel, &jpeg);
    if (result == CVI_SUCCESS)
    {
      const int quality = config.jpg_quality == 0 ? 80 : config.jpg_quality;
      jpeg.u32Qfactor = static_cast<CVI_U32>(std::clamp(quality, 51, 99));
      result = CVI_VENC_SetJpegParam(state.channel, &jpeg);
    }
    if (result != CVI_SUCCESS)
    {
      ReleaseVenc(state);
      status_ = Status(result);
      return;
    }
  }

  impl_ = std::move(implementation);
  status_ = Status(CVI_SUCCESS);
}

Encoder::~Encoder() { impl_.reset(); }

int Encoder::Channel() const
{
  return impl_ != nullptr && impl_->state != nullptr ? impl_->state->channel : -1;
}

Status Encoder::Push(const Frame &frame)
{
  if (!Ok() || impl_ == nullptr || impl_->state == nullptr || !frame.Valid() ||
      frame.impl_ == nullptr || frame.impl_->source == nullptr ||
      frame.impl_->source->context == nullptr ||
      frame.impl_->source->context.get() != impl_->state->context.get() ||
      frame.impl_->format != impl_->state->input_format || frame.Data() == nullptr ||
      frame.Width() != impl_->state->width || frame.Height() != impl_->state->height ||
      impl_->state->frame_pending)
  {
    return Status(kInvalidArgument);
  }

  auto &state = *impl_->state;
  VIDEO_FRAME_INFO_S input = frame.impl_->info;
  const VIDEO_FRAME_INFO_S *send_frame = &input;
  VIDEO_FRAME_INFO_S converted{};

  if (state.converter != nullptr)
  {
    int result = CVI_VPSS_SendFrame(state.converter->group, &input, 1000);
    if (result == CVI_SUCCESS)
    {
      result = CVI_VPSS_GetChnFrame(state.converter->group, state.converter->channel, &converted,
                                    1000);
    }
    if (result != CVI_SUCCESS)
    {
      return Status(result);
    }
    send_frame = &converted;
  }

  const int result = CVI_VENC_SendFrame(state.channel, send_frame, 1000);
  if (result != CVI_SUCCESS)
  {
    if (state.converter != nullptr)
    {
      ReleaseFrameToVpss(*state.converter, converted);
    }
    return Status(result);
  }

  if (state.converter != nullptr)
  {
    state.converted_frame = converted;
    state.converted_frame_pending = true;
  }
  state.frame_pending = true;
  return Status(result);
}

Status Encoder::Consume(AccessUnitCallback callback, void *context)
{
  if (!Ok() || impl_ == nullptr || impl_->state == nullptr || callback == nullptr)
  {
    return Status(kInvalidArgument);
  }
  if (!impl_->state->frame_pending)
  {
    return Status(kNotReady);
  }

  auto &state = *impl_->state;
  std::array<VENC_PACK_S, kMaxStreamPacks> pack_storage{};
  VENC_STREAM_S stream{};
  stream.pstPack = pack_storage.data();
  const int result = CVI_VENC_GetStream(state.channel, &stream, 1000);
  if (result != CVI_SUCCESS)
  {
    ReleaseConvertedFrame(state);
    state.frame_pending = false;
    return Status(result);
  }

  struct StreamGuard
  {
    EncoderState &state;
    VENC_STREAM_S &stream;

    ~StreamGuard()
    {
      CVI_VENC_ReleaseStream(state.channel, &stream);
      ReleaseConvertedFrame(state);
      state.frame_pending = false;
    }
  } guard{state, stream};

  if (stream.u32PackCount == 0 || stream.u32PackCount > kMaxStreamPacks)
  {
    return Status(kInvalidArgument);
  }

  EncodedAccessUnitView view{};
  view.count_ = stream.u32PackCount;
  for (std::size_t index = 0; index < view.count_; ++index)
  {
    const VENC_PACK_S &pack = stream.pstPack[index];
    if (pack.pu8Addr == nullptr || pack.u32Len <= pack.u32Offset)
    {
      return Status(kInvalidArgument);
    }
    view.packs_[index].data = pack.pu8Addr + pack.u32Offset;
    view.packs_[index].size = pack.u32Len - pack.u32Offset;
  }

  callback(context, view);
  return Status(CVI_SUCCESS);
}

}  // namespace cvimpp
