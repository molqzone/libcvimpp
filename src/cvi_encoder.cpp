#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>

#include "cvi_internal.hpp"

namespace cvimpp
{
namespace
{

// H.264 and H.265 expose the same CBR fields under different struct names.
template <class RcAttr>
void ConfigureCbr(RcAttr& rc, const EncoderConfig& config)
{
  rc.u32Gop = static_cast<CVI_U32>(config.gop);
  rc.u32StatTime = 2;
  rc.u32SrcFrameRate = static_cast<CVI_U32>(config.input_fps);
  rc.fr32DstFrameRate = static_cast<CVI_FLOAT>(config.output_fps);
  rc.u32BitRate = static_cast<CVI_U32>(config.bitrate_kbps);
  rc.bVariFpsEn = CVI_FALSE;
}

}  // namespace

namespace detail
{

void ReleaseConvertedFrame(EncoderState& state)
{
  if (!state.converted_frame_pending || !state.converter)
  {
    return;
  }

  ReleaseFrameToVpss(*state.converter, state.converted_frame);
  state.converted_frame = {};
  state.converted_frame_pending = false;
}

void ReleaseVenc(EncoderState& state)
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

Encoder::Encoder(Context& context, EncoderConfig config, Status* error)
{
  using namespace detail;

  const PAYLOAD_TYPE_E payload = ToPayloadType(config.codec);
  const PIXEL_FORMAT_E input_format = ToVendorFormat(config.format);
  if (context.impl_ == nullptr || context.impl_->state == nullptr ||
      !IsSupportedPayload(payload) || input_format == PIXEL_FORMAT_MAX ||
      config.width <= 0 || config.height <= 0 || config.input_fps <= 0 ||
      config.output_fps <= 0 || config.output_fps > config.input_fps ||
      config.bitrate_kbps <= 0 || config.jpg_quality < 0 || config.jpg_quality > 100 ||
      (payload != PT_JPEG && config.gop <= 0))
  {
    *error = Status(INVALID_ARGUMENT);
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
  else
  {
    channel_attr.stVencAttr.bEsBufQueueEn = CVI_TRUE;
    channel_attr.stVencAttr.bIsoSendFrmEn = CVI_TRUE;
    channel_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    channel_attr.stGopAttr.stNormalP.s32IPQpDelta = 2;
    if (payload == PT_H264)
    {
      channel_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
      ConfigureCbr(channel_attr.stRcAttr.stH264Cbr, config);
    }
    else
    {
      channel_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
      ConfigureCbr(channel_attr.stRcAttr.stH265Cbr, config);
    }
  }

  auto implementation = std::make_unique<Impl>();
  implementation->state = std::make_shared<EncoderState>();
  auto& state = *implementation->state;
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
    *error = Status(create_result);
    return;
  }

  // VENC accepts YUV input. Keep RGB capture useful by doing the conversion
  // in a small private VPSS group before SendFrame.
  if (config.format == PixelFormat::RGB888)
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

    int converter_result = CVI_SUCCESS;
    state.converter = CreateVpssResource(state.context, group_attr, conversion_attr,
                                         nullptr, false, &converter_result);
    if (state.converter == nullptr)
    {
      ReleaseVenc(state);
      *error = Status(converter_result);
      return;
    }
  }

  VENC_RECV_PIC_PARAM_S receive{};
  receive.s32RecvPicNum = -1;
  int rc = CVI_VENC_StartRecvFrame(state.channel, &receive);
  if (rc != CVI_SUCCESS)
  {
    ReleaseVenc(state);
    *error = Status(rc);
    return;
  }
  state.receiving = true;

  if (payload == PT_JPEG)
  {
    VENC_JPEG_PARAM_S jpeg{};
    rc = CVI_VENC_GetJpegParam(state.channel, &jpeg);
    if (rc == CVI_SUCCESS)
    {
      const int quality = config.jpg_quality == 0 ? 80 : config.jpg_quality;
      jpeg.u32Qfactor = static_cast<CVI_U32>(std::clamp(quality, 51, 99));
      rc = CVI_VENC_SetJpegParam(state.channel, &jpeg);
    }
    if (rc != CVI_SUCCESS)
    {
      ReleaseVenc(state);
      *error = Status(rc);
      return;
    }
  }

  impl_ = std::move(implementation);
  *error = Status(CVI_SUCCESS);
}

std::expected<Encoder, Status> Encoder::Create(Context& context, EncoderConfig config)
{
  Status error;
  Encoder encoder(context, config, &error);
  if (!error.Ok())
  {
    return std::unexpected(error);
  }
  return encoder;
}

Encoder::Encoder(Encoder&&) noexcept = default;
Encoder& Encoder::operator=(Encoder&&) noexcept = default;

Encoder::~Encoder() { impl_.reset(); }

int Encoder::Channel() const
{
  return impl_ != nullptr && impl_->state != nullptr ? impl_->state->channel : -1;
}

Status Encoder::Push(const Frame& frame)
{
  using namespace detail;

  if (impl_ == nullptr || !frame.Valid() || impl_->state->frame_pending)
  {
    return Status(INVALID_ARGUMENT);
  }

  auto& state = *impl_->state;
  VIDEO_FRAME_INFO_S input = frame.slot_->info;
  const VIDEO_FRAME_INFO_S* send_frame = &input;
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

Status Encoder::Consume(AccessUnitCallback callback, void* context)
{
  using namespace detail;

  if (impl_ == nullptr || callback == nullptr)
  {
    return Status(INVALID_ARGUMENT);
  }
  if (!impl_->state->frame_pending)
  {
    return Status(NOT_READY);
  }

  auto& state = *impl_->state;
  std::array<VENC_PACK_S, MAX_STREAM_PACKS> pack_storage{};
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
    EncoderState& state;
    VENC_STREAM_S& stream;

    ~StreamGuard()
    {
      CVI_VENC_ReleaseStream(state.channel, &stream);
      ReleaseConvertedFrame(state);
      state.frame_pending = false;
    }
  } guard{state, stream};

  if (stream.u32PackCount == 0 || stream.u32PackCount > MAX_STREAM_PACKS)
  {
    return Status(INVALID_ARGUMENT);
  }

  EncodedAccessUnitView view{};
  view.count_ = stream.u32PackCount;
  for (std::size_t index = 0; index < view.count_; ++index)
  {
    const VENC_PACK_S& pack = stream.pstPack[index];
    if (pack.pu8Addr == nullptr || pack.u32Len <= pack.u32Offset)
    {
      return Status(INVALID_ARGUMENT);
    }
    view.packs_[index] = std::span<const std::uint8_t>(pack.pu8Addr + pack.u32Offset,
                                                       pack.u32Len - pack.u32Offset);
  }

  callback(context, view);
  return Status(CVI_SUCCESS);
}

}  // namespace cvimpp
