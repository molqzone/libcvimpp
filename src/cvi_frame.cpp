#include "cvi_internal.hpp"

namespace cvimpp
{

Frame::Frame() = default;

Frame::~Frame() { Release(); }

Frame::Frame(Frame &&other) noexcept
    : slot_(other.slot_),
      data_(other.data_),
      size_(other.size_),
      width_(other.width_),
      height_(other.height_),
      format_(other.format_)
{
  other.Reset();
}

Frame &Frame::operator=(Frame &&other) noexcept
{
  if (this != &other)
  {
    Release();
    slot_ = other.slot_;
    data_ = other.data_;
    size_ = other.size_;
    width_ = other.width_;
    height_ = other.height_;
    format_ = other.format_;
    other.Reset();
  }
  return *this;
}

void Frame::Assign(Slot &slot, PixelFormat format) noexcept
{
  slot_ = &slot;
  data_ = static_cast<std::uint8_t *>(slot.mapped);
  size_ = slot.mapped_size;
  width_ = static_cast<int>(slot.info.stVFrame.u32Width);
  height_ = static_cast<int>(slot.info.stVFrame.u32Height);
  format_ = format;
}

void Frame::Reset() noexcept
{
  slot_ = nullptr;
  data_ = nullptr;
  size_ = 0;
  width_ = 0;
  height_ = 0;
  format_ = PixelFormat::Unknown;
}

void Frame::Release()
{
  if (slot_ == nullptr)
  {
    return;
  }

  if (slot_->owner != nullptr)
  {
    detail::ReleaseFrameToVpss(*slot_->owner, slot_->info);
  }
  slot_->info = VIDEO_FRAME_INFO_S{};
  slot_->mapped = nullptr;
  slot_->mapped_size = 0;
  slot_->in_use = false;
  Reset();
}

}  // namespace cvimpp
