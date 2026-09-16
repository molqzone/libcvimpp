#include "cvi_internal.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>

namespace cvimpp
{
namespace detail
{

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

EncoderState::~EncoderState() { ReleaseVenc(*this); }

}  // namespace detail

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
  using namespace detail;

  const PAYLOAD_TYPE_E payload = ToPayloadType(config.codec);
  const PIXEL_FORMAT_E input_format = ToVendorFormat(config.format);
  if (!context.Ok() || context.impl_ == nullptr || context.impl_->state == nullptr ||
      !IsSupportedPayload(payload) || input_format == PIXEL_FORMAT_MAX ||
      config.width <= 0 || config.height <= 0 || config.input_fps <= 0 ||
      config.output_fps <= 0 || config.output_fps > config.input_fps ||
      config.bitrate_kbps <= 0 || config.jpg_quality < 0 || config.jpg_quality > 100 ||
      (payload != PT_JPEG && config.gop <= 0))
  {
    status_ = context.Ok() ? Status(kInvalidArgument) : context.Result();
    return;
  }

  VENC_CHN_ATTR_S channel_attr{};
  channel_attr.stVencAttr.enType = payload;
  channel_attr.stVencAttr.u32MaxPicWidth = static_cast<CVI_U32>(config.width);
  channel_attr.stVencAttr.u32MaxPicHeight = static_cast<CVI_U32>(config.height);
  channel_attr.stVencAttr.u32BufSize =
      std::max<CVI_U32>(1024u * 1024u, static_cast<CVI_U32>(config.width) *
                                           static_cast<CVI_U32>(config.height) * 2u);
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
    channel_attr.stRcAttr.stH264Cbr.u32SrcFrameRate =
        static_cast<CVI_U32>(config.input_fps);
    channel_attr.stRcAttr.stH264Cbr.fr32DstFrameRate = config.output_fps;
    channel_attr.stRcAttr.stH264Cbr.u32BitRate =
        static_cast<CVI_U32>(config.bitrate_kbps);
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
    channel_attr.stRcAttr.stH265Cbr.u32SrcFrameRate =
        static_cast<CVI_U32>(config.input_fps);
    channel_attr.stRcAttr.stH265Cbr.fr32DstFrameRate = config.output_fps;
    channel_attr.stRcAttr.stH265Cbr.u32BitRate =
        static_cast<CVI_U32>(config.bitrate_kbps);
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

  // VENC accepts YUV input. Keep RGB capture useful by doing the conversion
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
    conversion_attr.stAspectRatio.stVideoRect.u32Width =
        static_cast<CVI_U32>(config.width);
    conversion_attr.stAspectRatio.stVideoRect.u32Height =
        static_cast<CVI_U32>(config.height);
    conversion_attr.stAspectRatio.bEnableBgColor = CVI_TRUE;
    conversion_attr.stAspectRatio.u32BgColor = COLOR_RGB_BLACK;
    conversion_attr.stNormalize.bEnable = CVI_FALSE;

    int result = CVI_SUCCESS;
    state.converter = CreateVpssResource(state.context, group_attr, conversion_attr,
                                         nullptr, false, &result);
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
  using namespace detail;

  if (!Ok() || impl_ == nullptr || impl_->state == nullptr || !frame.Valid() ||
      frame.slot_ == nullptr || frame.slot_->owner == nullptr ||
      frame.slot_->owner->context == nullptr ||
      frame.slot_->owner->context.get() != impl_->state->context.get() ||
      frame.format_ != impl_->state->input_format || frame.data_ == nullptr ||
      frame.width_ != impl_->state->width || frame.height_ != impl_->state->height ||
      impl_->state->frame_pending)
  {
    return Status(kInvalidArgument);
  }

  auto &state = *impl_->state;
  VIDEO_FRAME_INFO_S input = frame.slot_->info;
  const VIDEO_FRAME_INFO_S *send_frame = &input;
  VIDEO_FRAME_INFO_S converted{};

  if (state.converter != nullptr)
  {
    int result = CVI_VPSS_SendFrame(state.converter->group, &input, 1000);
    if (result == CVI_SUCCESS)
    {
      result = CVI_VPSS_GetChnFrame(state.converter->group, state.converter->channel,
                                    &converted, 1000);
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
  using namespace detail;

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
