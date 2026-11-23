# libcvimpp

A small C++23 RAII wrapper around the CVI MPI on the SG200x (MaixCAM), covering
the capture path (VI -> VPSS) and the encoder path (VENC, H.264/H.265/JPEG).

The public header `include/cvimpp/cvi.hpp` depends only on the C++ standard
library: the CVI middleware headers are never exposed to consumers.

## Building

The middleware is prebuilt and lives in the image build cache, so the build
needs the path to that directory:

```
cmake -S . -B build -DCVIMPP_MIDDLEWARE_DIR=<...>/boards/<board>/middleware
cmake --build build
```

The CVI archives are mutually recursive, so the target links them inside one
`--start-group` together with `atomic pthread dl m`.

## Design

`Context`, `Capture` and `Encoder` keep their vendor state behind pimpl, so the
public header never names a CVI type.

`Frame` is a value handle instead: it carries the mapped pointer and the frame
geometry inline, and points at a borrow slot owned by the VPSS resource that
produced it. Popping or releasing a frame therefore allocates nothing and
touches no refcount. The slot array is bounded by the VPSS channel depth
(`CaptureConfig::depth`, at most 8).

VPSS recycles its output buffers from the common VB pool, so a channel hands out
the same few physical blocks over and over. The resource maps each block once
and keeps the mapping, invalidating the cache range per frame instead of doing
an `mmap`/`munmap` pair per frame.

The consequence is a lifetime contract: a frame must be released before the
`Capture` (or `Encoder`) that produced it is destroyed.

Construction cannot half-succeed: `Context`, `Capture` and `Encoder` are built
through `Create()` and return `std::expected<T, Status>`, so an object that
exists is an object that works. There is no `Ok()`/`Result()` pair to re-check
at every use, and a failure never leaves a partially initialized pipeline
behind. (The C++23 requirement comes from `std::expected`.)

Logging is a compile-time level rather than a dependency: `CVIMPP_INFO` /
`CVIMPP_WARN` / `CVIMPP_ERROR` write to `stderr`, the literal format is checked by
`-Wformat`, and `CVIMPP_LOG_LEVEL` removes the calls entirely in a quiet build.
The library owns no sink and exposes no logging API from its header; the host
process decides where `stderr` goes.

`Status` reports the failures the middleware returns: `Create()` uses it for
initialization, and `Pop()` / `Push()` / `Consume()` use it for the per-frame
errors that are recoverable. Allocation failure is not one of them; construction
throws `std::bad_alloc` like any other type that uses the standard library.

## License

Apache-2.0, see `LICENSE`.
