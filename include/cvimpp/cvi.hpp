#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace cvimpp
{

class Status
{
 public:
  constexpr Status() = default;
  constexpr explicit Status(int code) : code_(code) {}

  [[nodiscard]] constexpr bool Ok() const { return code_ == 0; }
  [[nodiscard]] constexpr int Code() const { return code_; }
  [[nodiscard]] bool IsFrameDropped() const;
  [[nodiscard]] constexpr explicit operator bool() const { return Ok(); }

 private:
  int code_{};
};

// Status carries the failures the middleware reports.  Allocation failure is not
// one of them: construction allocates, so it throws std::bad_alloc like any
// other type that uses the standard library.
inline constexpr int kInvalidArgument = -1;
inline constexpr int kNotReady = -2;

struct ManualIspConfig
{
  // Zero keeps the vendor automatic 3A loop.  A non-zero exposure time locks
  // AE to that exposure (microseconds) with fixed gains, and locks AWB to the
  // manual gains below.  Beacon detectors run in a dark scene and want the
  // background suppressed instead of the AE lifting it to mid-grey.
  std::uint32_t exposure_us{};
  std::uint32_t again{1024};
  std::uint32_t dgain{1024};
  std::uint32_t isp_dgain{1024};
  std::uint16_t wb_rgain{1024};
  std::uint16_t wb_ggain{1024};
  std::uint16_t wb_bgain{1024};
};

struct ContextConfig
{
  // The common pool must be large enough for the sensor frame and the VPSS
  // outputs used by the application.  The implementation raises this value
  // to the sensor's calculated minimum when necessary.
  std::uint32_t vi_pool_size{};
  std::uint32_t vi_pool_count{};

  // Null selects the middleware default (/mnt/data/sensor_cfg.ini).
  const char *sensor_ini_path{};

  // Optional manual AE/AWB lock; the default leaves 3A automatic.
  ManualIspConfig manual_isp{};
};

class Capture;
class Encoder;

class Context final
{
 public:
  explicit Context(ContextConfig config);
  ~Context();

  Context(const Context &) = delete;
  Context &operator=(const Context &) = delete;
  Context(Context &&) = delete;
  Context &operator=(Context &&) = delete;

  [[nodiscard]] bool Ok() const { return status_.Ok(); }
  [[nodiscard]] Status Result() const { return status_; }

 private:
  friend class Capture;
  friend class Encoder;

  struct Impl;
  std::unique_ptr<Impl> impl_{};
  Status status_{kNotReady};
};

enum class PixelFormat : std::uint8_t
{
  Unknown,
  Nv21,
  Rgb888,
};

enum class FitMode : std::uint8_t
{
  Stretch = 0,
  Letterbox = 1,
  Crop = 2,
};

struct CaptureConfig
{
  int width{640};
  int height{480};
  int fps{30};
  PixelFormat format{PixelFormat::Nv21};
  // VPSS channel depth, and therefore the number of frames that may be held at
  // once.  Capture rejects anything above 8.
  int depth{2};
  FitMode fit{FitMode::Crop};
};

struct FrameView
{
  const std::uint8_t *data{};
  std::size_t size{};
  int width{};
  int height{};
  PixelFormat format{PixelFormat::Unknown};

  [[nodiscard]] bool Valid() const { return data != nullptr && size != 0; }
  [[nodiscard]] std::span<const std::uint8_t> Bytes() const { return {data, size}; }
};

class Frame final
{
 public:
  Frame();
  ~Frame();

  Frame(const Frame &) = delete;
  Frame &operator=(const Frame &) = delete;
  Frame(Frame &&other) noexcept;
  Frame &operator=(Frame &&other) noexcept;

  // Borrow slot inside the VPSS resource that produced the frame.  Declared so
  // that Frame stays a value type that never allocates; only the implementation
  // defines it.
  struct Slot;

  [[nodiscard]] bool Valid() const { return slot_ != nullptr; }
  [[nodiscard]] void *Data() const { return data_; }
  [[nodiscard]] int Length() const { return static_cast<int>(size_); }
  [[nodiscard]] int Width() const { return width_; }
  [[nodiscard]] int Height() const { return height_; }
  [[nodiscard]] PixelFormat Format() const { return format_; }
  [[nodiscard]] FrameView View() const
  {
    return FrameView{data_, size_, width_, height_, format_};
  }

  // Returns the vendor frame to the VPSS resource it came from.  Safe to call
  // repeatedly.  A frame must be released before the Capture or Encoder that
  // produced it is destroyed.
  void Release();

 private:
  friend class Capture;
  friend class Encoder;

  void Assign(Slot &slot, PixelFormat format) noexcept;
  void Reset() noexcept;

  Slot *slot_{};
  std::uint8_t *data_{};
  std::size_t size_{};
  int width_{};
  int height_{};
  PixelFormat format_{PixelFormat::Unknown};
};

class Capture final
{
 public:
  Capture(Context &context, CaptureConfig config);
  ~Capture();

  Capture(const Capture &) = delete;
  Capture &operator=(const Capture &) = delete;
  Capture(Capture &&) = delete;
  Capture &operator=(Capture &&) = delete;

  [[nodiscard]] bool Ok() const { return status_.Ok(); }
  [[nodiscard]] Status Result() const { return status_; }
  [[nodiscard]] int Channel() const;
  [[nodiscard]] PixelFormat Format() const;

  // The returned frame owns the mapped view until Release() or destruction.
  [[nodiscard]] Status Pop(Frame &frame);

 private:
  struct Impl;
  Context *context_{};
  std::unique_ptr<Impl> impl_{};
  Status status_{kNotReady};
};

enum class Codec : std::uint8_t
{
  Jpeg,
  H265,
  H264,
};

struct EncoderConfig
{
  Codec codec{Codec::H264};
  int width{640};
  int height{480};
  PixelFormat format{PixelFormat::Nv21};
  int input_fps{30};
  int output_fps{30};
  int bitrate_kbps{512};
  int gop{15};
  int jpg_quality{0};
};

struct EncodedPack
{
  [[nodiscard]] std::span<const std::uint8_t> Bytes() const
  {
    return std::span<const std::uint8_t>(data, size);
  }

 private:
  friend class EncodedAccessUnitView;
  friend class Encoder;
  const std::uint8_t *data{};
  std::size_t size{};
};

class EncodedAccessUnitView final
{
 public:
  [[nodiscard]] std::size_t PackCount() const { return count_; }
  [[nodiscard]] std::span<const std::uint8_t> Pack(std::size_t index) const;

 private:
  friend class Encoder;

  std::array<EncodedPack, 8> packs_{};
  std::size_t count_{};
};

using AccessUnitCallback = void (*)(void *context, const EncodedAccessUnitView &access_unit);

class Encoder final
{
 public:
  Encoder(Context &context, EncoderConfig config);
  ~Encoder();

  Encoder(const Encoder &) = delete;
  Encoder &operator=(const Encoder &) = delete;
  Encoder(Encoder &&) = delete;
  Encoder &operator=(Encoder &&) = delete;

  [[nodiscard]] bool Ok() const { return status_.Ok(); }
  [[nodiscard]] Status Result() const { return status_; }
  [[nodiscard]] int Channel() const;

  [[nodiscard]] Status Push(const Frame &frame);

  // The view is borrowed and valid only until callback returns.
  [[nodiscard]] Status Consume(AccessUnitCallback callback, void *context);

 private:
  struct Impl;
  Context *context_{};
  std::unique_ptr<Impl> impl_{};
  Status status_{kNotReady};
};

}  // namespace cvimpp
