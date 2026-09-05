# nros-rmw-uorb

Phase 115.K.4 — C++ port of `nros-rmw-uorb` (the previous Rust
implementation, slated for retirement once this backend reaches
parity).

## Layout

```
packages/rmw/uorb/nros-rmw-uorb/
├── CMakeLists.txt          standalone CMake project
├── cmake/
│   └── NrosRmwUorbConfig.cmake.in
├── include/
│   └── nros_rmw_uorb.h     public C entry point: nros_rmw_uorb_register()
├── src/
│   ├── internal.hpp        shared decls across .cpp files
│   ├── vtable.cpp          assembles nros_rmw_vtable_t + register entry
│   ├── session.cpp         open / close / drive_io  (stubs in K.4.0)
│   ├── publisher.cpp       create / publish_raw / destroy (stubs in K.4.0)
│   ├── subscriber.cpp      create / take_serialized / has_data / destroy (stubs in K.4.0)
│   └── service.cpp         service slots — default UNSUPPORTED (K.4.4 may change)
└── tests/
    └── register_smoke.cpp  K.4.0 smoke test (stubs cffi_register locally)
```

## Build

Standalone (smoke test only):

```bash
cmake -S packages/rmw/uorb/nros-rmw-uorb -B build/nros-rmw-uorb \
    -DNROS_RMW_CFFI_DIR=$PWD/packages/core/nros-rmw-abi/include
cmake --build build/nros-rmw-uorb
./build/nros-rmw-uorb/nros_rmw_uorb_register_smoke
```

Inside a PX4 module (K.4.1 onward):

```cmake
find_package(NrosRmwUorb CONFIG REQUIRED)
target_link_libraries(my_px4_module PRIVATE NrosRmwUorb::NrosRmwUorb)
```

Pass `-DNROS_RMW_UORB_LINK_PX4=ON -DPX4_FIRMWARE_DIR=<path>` to
pull in the uORB / workqueue / log headers from a PX4-Autopilot
checkout.

## Status

- K.4.0 (this commit): scaffold + smoke test. Every vtable slot
  returns `NROS_RMW_RET_UNSUPPORTED`.
- K.4.1 (next): session lifecycle — stash node_name / namespace,
  `drive_io` returns OK (uORB push-based).
- K.4.2: pub/sub data plane — `orb_advertise_multi`,
  `orb_publish`, `orb_subscribe_multi`, `orb_register_callback` +
  per-subscriber ringbuffer.
- K.4.3: type-hash correlation — static topic registry mapping
  `(name, type_name) → orb_metadata *`.
- K.4.4: services (decision pending — service-over-topics vs
  permanent UNSUPPORTED).
- K.4.5: remove the legacy Rust stack (`nros-rmw-uorb`,
  `nros-px4`, `third-party/px4/px4-rs`).

See [`docs/roadmap/phase-115-runtime-transport-vtable.md` §D.7](../../../../docs/roadmap/archived/phase-115-runtime-transport-vtable.md)
for the detailed plan.
