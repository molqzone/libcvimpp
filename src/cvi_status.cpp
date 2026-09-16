#include "cvi_internal.hpp"

#include <limits>

namespace cvimpp
{
namespace detail
{

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

}  // namespace detail

bool Status::IsFrameDropped() const
{
  return code_ == static_cast<int>(CVI_ERR_VENC_FRC_NO_ENC);
}

}  // namespace cvimpp
