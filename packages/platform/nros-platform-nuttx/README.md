# nros-platform-nuttx-c

Native C implementation of the nano-ros canonical platform ABI (`<nros/platform.h>`) for [NuttX](https://nuttx.apache.org/).

NuttX exposes a POSIX-compatible surface (`pthread_*`, `clock_gettime`, `nanosleep`, `sched_yield`, `malloc`/`realloc`/`free`). The Rust `NuttxPlatform` mirrors this by forwarding every trait method to `PosixPlatform`; the C port follows the same pattern — it compiles the very same `src/platform.c` shipped by `nros-platform-posix-c` and produces `libnros_platform_nuttx.a`. No NuttX-specific source file exists.

## Build

```bash
cmake -B build
cmake --build build
```

By default the build picks up `<nros/platform.h>` from `../nros-platform-cffi/include` and the source from `../nros-platform-posix-c/src/platform.c`. Override via:

```bash
cmake -B build \
  -DNROS_PLATFORM_CFFI_INCLUDE=/path/to/include \
  -DNROS_PLATFORM_POSIX_C_SOURCE=/path/to/platform.c
```

For a NuttX application build, the NuttX build system pulls in the libc + pthread layer automatically; no extra link library is required.

## License

Apache-2.0 or MIT at your option.
