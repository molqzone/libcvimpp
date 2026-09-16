#include "cvi_internal.hpp"

#include <memory>

namespace cvimpp
{

Capture::Capture(Context &context, CaptureConfig config) : context_(&context)
{
  using namespace detail;

  if (!context.Ok() || context.impl_ == nullptr || context.impl_->state == nullptr ||
      config.width <= 0 || config.height <= 0 || config.fps <= 0 || config.depth < 0 ||
      config.depth > static_cast<int>(kMaxFrameSlots))
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
                       static_cast<int>(state.sensor_size.u32Height), config.width,
                       config.height, &crop);

  int result = CVI_SUCCESS;
  implementation->source = CreateVpssResource(
      context.impl_->state, group_attr, channel_attr,
      config.fit == FitMode::Crop ? &crop : nullptr, true, &result);
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
  using namespace detail;

  if (!Ok() || impl_ == nullptr || impl_->source == nullptr || frame.Valid())
  {
    return Status(kInvalidArgument);
  }

  VpssResource &resource = *impl_->source;
  VIDEO_FRAME_INFO_S vendor_frame{};
  const int result = CVI_VPSS_GetChnFrame(resource.group, resource.channel, &vendor_frame,
                                          kFrameTimeoutMs);
  if (result != CVI_SUCCESS)
  {
    return Status(result);
  }

  const PixelFormat format = FromVendorFormat(vendor_frame.stVFrame.enPixelFormat);
  const std::uint32_t size = FrameBufferSize(vendor_frame);
  if (format == PixelFormat::Unknown || size == 0 ||
      vendor_frame.stVFrame.u64PhyAddr[0] == 0)
  {
    ReleaseFrameToVpss(resource, vendor_frame);
    return Status(kInvalidArgument);
  }

  Frame::Slot *const slot = AcquireFrameSlot(resource, vendor_frame, size);
  if (slot == nullptr)
  {
    ReleaseFrameToVpss(resource, vendor_frame);
    return Status(CVI_FAILURE);
  }

  frame.Assign(*slot, format);
  return Status(CVI_SUCCESS);
}

}  // namespace cvimpp
