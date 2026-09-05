# nros-rmw-xrce

Phase 115.K.2 — micro-XRCE-DDS-Client RMW backend for nano-ros, in C.

This is the C-native re-implementation of `nros-rmw-xrce` (Rust over
`xrce-sys`). It consumes the canonical `nros_rmw_vtable_t` C ABI
defined in `packages/core/nros-rmw-abi/include/nros/rmw_vtable.h`
and registers itself via `nros_rmw_cffi_register()`.

Target architecture mirrors `packages/rmw/cyclonedds/nros-rmw-cyclonedds`:
a static library + a single public header carrying the
`nros_rmw_xrce_register()` entry point.

## Status

- [x] **115.K.2.0 — vtable scaffold.** Every vtable entry returned
  `NROS_RMW_RET_UNSUPPORTED`. Wired-but-inert.
- [x] **115.K.2.1 — session lifecycle.** `xrce_session_open` parses
  `udp/host:port` (or bare `host:port`), runs `uxr_init_udp_transport`
  + `uxr_create_session_retries`, allocates reliable streams, parks
  the per-session state in `rmw_session_t::backend_data`. Close
  + drive_io fully wired.
- [x] **115.K.2.2 (this commit) — pub/sub.** `xrce_publisher_create`
  allocates 3 entity ids (TOPIC, PUBLISHER, DATAWRITER) and creates
  them via `uxr_buffer_create_*_bin`; `publish_raw` goes through
  `uxr_buffer_topic` + a 0-ms flush. `xrce_subscriber_create` does
  the symmetric DATAREADER setup, allocates a slot from the per-
  session pool of 8, and issues `uxr_buffer_request_data` for
  continuous delivery. The single per-session topic callback
  (registered at `xrce_session_open`) dispatches by datareader id
  to the matching slot. `take_serialized` reads the slot's single-msg
  ringbuffer; oversize messages flag overflow and drop.
- [x] **115.K.2.3 — services.** `xrce_service_server_create` allocates
  a REPLIER entity via `uxr_buffer_create_replier_bin` and a slot
  from the per-session pool of `XRCE_MAX_SERVICE_SERVERS=4`.
  Per-session `request_callback` (registered at session_open)
  dispatches by replier id; the captured `SampleIdentity` rides in
  the slot for `xrce_service_send_response` to feed
  `uxr_buffer_reply`. Symmetric REQUESTER path on the client side;
  `xrce_service_call_raw` busy-waits via `uxr_run_session_time` for
  up to `XRCE_SERVICE_REPLY_TOTAL_MS=5000 ms`. Single-slot inbox
  per server / client.
- [x] **115.K.2.4 — Phase 115.E custom-transport bridge.**
  `nros_rmw_xrce_set_custom_transport_ops(ops, framing)` (declared
  in `<nros_rmw_xrce.h>`) installs a runtime-supplied vtable into
  backend-local storage. `xrce_session_open` invoked with a
  `custom://` locator routes through
  `uxr_set_custom_transport_callbacks` +
  `uxr_init_custom_transport`; trampolines fan back out to the
  user's open / close / write / read. The drain-from-runtime path
  (`nros_rmw_xrce_init_custom_transport(framing)`) is a stub
  pending a `nros_rmw_take_custom_transport` C export from
  `nros-rmw-cffi`; see [`docs/reference/xrce-known-limitations.md`](../../../../docs/reference/xrce-known-limitations.md).
- [ ] **115.K.2.5 — drop the Rust crate.** Once feature parity lands,
  remove `nros-rmw-xrce` and `xrce-sys` from the workspace; the
  CMake `-DNROS_C_RMW=xrce` selector switches over.

### K.2 scope gaps (intentional)

Each gap is annotated `TODO 115.K.2.x` in source. The K.2 series
intentionally ships a smaller surface than the Rust impl:

- No QoS XML profile (`uxr_buffer_create_*_xml`). Bin profile only —
  reliability + durability + history + depth. Phase 108.C.xrce.3's
  full FastDDS XML is not ported.
- No deadline tracking, no `OfferedDeadlineMissed` /
  `RequestedDeadlineMissed` event surface.
- No async wakers — `take_serialized` is purely poll-based.
- No fragmented publish path — payloads larger than a single stream
  slot return `NROS_RMW_RET_MESSAGE_TOO_LARGE`.
- Single-slot-per-subscriber ringbuffer; concurrent inbound messages
  during read flag `overflow` and the read returns
  `NROS_RMW_RET_MESSAGE_TOO_LARGE` on the next poll.

See `docs/roadmap/phase-115-runtime-transport-vtable.md` Appendix D
§D.4 for the planned per-file split and §D.5 for risks.

## Building (scaffold)

```bash
mkdir -p build && cd build
cmake -DNROS_RMW_CFFI_DIR=/path/to/packages/core/nros-rmw-abi/include ..
make
ctest --output-on-failure
```

The scaffold does NOT link against the micro-XRCE-DDS-Client static
library — the stubs never call `uxr_*`. The `MICROXRCEDDS_CLIENT_DIR`
include path exists for forward-compat with 115.K.2.1+, where the
session / pub / sub TUs will start `#include`-ing
`<uxr/client/client.h>`.

## Why C, not C++?

`micro-XRCE-DDS-Client` is C99. The micro-ROS reference impl is C.
Staying in C means no `extern "C"` wrapping, no exception/RTTI knobs,
and the same toolchain baseline as the rest of the XRCE ecosystem.
Cyclone DDS picked C++ because Cyclone's idiomatic API is C++; the
host-language rule (`book/src/internals/rmw-backends.md`) says match
the underlying library, which lands us in C here.
