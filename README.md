# libcvimpp

A small C++20 RAII wrapper around the CVI MPI on the SG200x (MaixCAM), covering
the capture path (VI -> VPSS) and the encoder path (VENC, H.264/H.265/JPEG).

The public header `include/cvimpp/cvi.hpp` depends only on the C++ standard
library: the CVI middleware headers are never exposed to consumers.

## Building

The middleware is prebuilt and lives in the image build cache, so the build
needs the path to that directory:

```
cmake -S . -B build -DGUIDANCE_MIDDLEWARE_DIR=<...>/boards/<board>/middleware
cmake --build build
```

The CVI archives are mutually recursive, so the target links them inside one
`--start-group` together with `atomic pthread dl m`.

## License

Apache-2.0, see `LICENSE`.
