#pragma once

#include "cvimpp/cvi.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>

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

// Frames are passed straight back to the middleware and mapping sizes are
// derived from the plane lengths. Catch a header/library mismatch at compile
// time instead of silently interpreting a different vendor structure layout.
static_assert(std::extent_v<decltype(VIDEO_FRAME_S::u32Length)> == 3u,
              "unexpected VIDEO_FRAME_S plane count");
static_assert(std::extent_v<decltype(VIDEO_FRAME_S::u64PhyAddr)> == 3u,
              "unexpected VIDEO_FRAME_S address count");

namespace detail
{

inline constexpr int kFrameTimeoutMs = 100;
inline constexpr std::size_t kMaxStreamPacks = 8;
// A VPSS channel never hands out more frames than its configured depth.
inline constexpr std::size_t kMaxFrameSlots = 8;
// VPSS output buffers come from the common VB pool and are reused.
inline constexpr std::size_t kMaxFrameMappings = 16;

struct VpssResource;

}  // namespace detail

// Every frame borrows a slot owned by the VPSS resource that produced it, so
// popping and releasing a frame never allocates and never touches a refcount.
struct Frame::Slot
{
  detail::VpssResource *owner{};
  VIDEO_FRAME_INFO_S info{};
  void *mapped{};
  CVI_U32 mapped_size{};
  bool in_use{};
};

namespace detail
{

struct FrameMapping
{
  CVI_U64 physical{};
  void *mapped{};
  CVI_U32 size{};
};

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
  std::array<Frame::Slot, kMaxFrameSlots> slots{};
  std::array<FrameMapping, kMaxFrameMappings> mappings{};
  std::size_t mapping_cursor{};

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

PIXEL_FORMAT_E ToVendorFormat(PixelFormat format);
PixelFormat FromVendorFormat(PIXEL_FORMAT_E format);
PAYLOAD_TYPE_E ToPayloadType(Codec codec);
bool IsSupportedPayload(PAYLOAD_TYPE_E payload);
std::uint32_t FrameBufferSize(const VIDEO_FRAME_INFO_S &frame);

void ReleaseFrameToVpss(VpssResource &resource, VIDEO_FRAME_INFO_S &frame);
Frame::Slot *AcquireFrameSlot(VpssResource &resource, const VIDEO_FRAME_INFO_S &frame,
                              std::uint32_t size);
std::shared_ptr<VpssResource> CreateVpssResource(
    const std::shared_ptr<ContextState> &context, const VPSS_GRP_ATTR_S &group_attr,
    const VPSS_CHN_ATTR_S &channel_attr, const VPSS_CROP_INFO_S *crop, bool bind_vi,
    int *error);
void ConfigureAspectRatio(VPSS_CHN_ATTR_S &attribute, FitMode fit,
                          PIXEL_FORMAT_E output_format, int source_width,
                          int source_height, int output_width, int output_height,
                          VPSS_CROP_INFO_S *crop);

void ReleaseConvertedFrame(EncoderState &state);
void ReleaseVenc(EncoderState &state);

}  // namespace detail

struct Context::Impl
{
  std::shared_ptr<detail::ContextState> state;
};

struct Capture::Impl
{
  std::shared_ptr<detail::VpssResource> source;
  PixelFormat format{PixelFormat::Unknown};
};

struct Encoder::Impl
{
  std::shared_ptr<detail::EncoderState> state;
};

}  // namespace cvimpp
