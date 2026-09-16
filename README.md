# libcvimpp

A small C++20 RAII wrapper around the CVI MPI on the SG200x (MaixCAM), covering
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

## License

Apache-2.0, see `LICENSE`.
