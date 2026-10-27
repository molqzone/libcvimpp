#include "cvi_internal.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>

namespace cvimpp::detail
{
namespace
{

void *MapFrameBuffer(VpssResource &resource, CVI_U64 physical, CVI_U32 size)
{
  for (const FrameMapping &mapping : resource.mappings)
  {
    if (mapping.mapped != nullptr && mapping.physical == physical && mapping.size == size)
    {
      CVI_SYS_IonInvalidateCache(physical, mapping.mapped, size);
      return mapping.mapped;
    }
  }

  void *const mapped = CVI_SYS_MmapCache(physical, size);
  if (mapped == nullptr)
  {
    return nullptr;
  }

  for (FrameMapping &mapping : resource.mappings)
  {
    if (mapping.mapped == nullptr)
    {
      mapping = FrameMapping{physical, mapped, size};
      return mapped;
    }
  }

  FrameMapping &victim = resource.mappings[resource.mapping_cursor];
  resource.mapping_cursor = (resource.mapping_cursor + 1u) % resource.mappings.size();
  CVI_SYS_Munmap(victim.mapped, victim.size);
  victim = FrameMapping{physical, mapped, size};
  return mapped;
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

}  // namespace

VpssResource::~VpssResource()
{
  for (const FrameMapping &mapping : mappings)
  {
    if (mapping.mapped != nullptr)
    {
      CVI_SYS_Munmap(mapping.mapped, mapping.size);
    }
  }

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

void ReleaseFrameToVpss(VpssResource &resource, VIDEO_FRAME_INFO_S &frame)
{
  CVI_VPSS_ReleaseChnFrame(resource.group, resource.channel, &frame);
}

Frame::Slot *AcquireFrameSlot(VpssResource &resource, const VIDEO_FRAME_INFO_S &frame,
                              std::uint32_t size)
{
  for (Frame::Slot &slot : resource.slots)
  {
    if (slot.in_use)
    {
      continue;
    }

    void *const mapped = MapFrameBuffer(resource, frame.stVFrame.u64PhyAddr[0], size);
    if (mapped == nullptr)
    {
      return nullptr;
    }

    slot.owner = &resource;
    slot.info = frame;
    slot.mapped = mapped;
    slot.mapped_size = size;
    slot.in_use = true;
    return &slot;
  }

  return nullptr;
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
  std::printf("[camera] vpss(%d,%d) max %ux%u src=%d dst=%d CreateGrp=%d\n", resource->group,
              resource->channel, group_attr.u32MaxW, group_attr.u32MaxH,
              channel_attr.stFrameRate.s32SrcFrameRate,
              channel_attr.stFrameRate.s32DstFrameRate, result);
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
  std::printf("[camera] CVI_VPSS_ResetGrp=%d\n", result);
  if (result == CVI_SUCCESS)
  {
    result = CVI_VPSS_SetChnAttr(resource->group, resource->channel, &channel_attr);
    std::printf("[camera] CVI_VPSS_SetChnAttr=%d\n", result);
  }
  if (result == CVI_SUCCESS && crop != nullptr)
  {
    result = CVI_VPSS_SetChnCrop(resource->group, resource->channel, crop);
    std::printf("[camera] CVI_VPSS_SetChnCrop=%d\n", result);
  }
  if (result == CVI_SUCCESS)
  {
    result = CVI_VPSS_EnableChn(resource->group, resource->channel);
    std::printf("[camera] CVI_VPSS_EnableChn=%d\n", result);
    resource->channel_enabled = result == CVI_SUCCESS;
  }
  if (result == CVI_SUCCESS)
  {
    result = CVI_VPSS_StartGrp(resource->group);
    std::printf("[camera] CVI_VPSS_StartGrp=%d\n", result);
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
    std::printf("[camera] CVI_SYS_Bind vi(%d,%d)->vpss(%d,%d)=%d\n", source.s32DevId,
                source.s32ChnId, destination.s32DevId, destination.s32ChnId, result);
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

void ConfigureAspectRatio(VPSS_CHN_ATTR_S &attribute, FitMode fit,
                          PIXEL_FORMAT_E output_format, int source_width,
                          int source_height, int output_width, int output_height,
                          VPSS_CROP_INFO_S *crop)
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

}  // namespace cvimpp::detail
