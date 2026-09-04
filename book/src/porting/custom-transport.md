# Custom Transport

`nano-ros` lets you plug a custom transport (USB-CDC, BLE GATT, RS-485
with framing, ring-buffer loopback, semihosting bridge, …) at runtime,
without changing the board crate, Cargo features, or rebuilding. This
is the runtime equivalent of micro-ROS's
`rmw_uros_set_custom_transport(framing, params, open, close, write, read)`.

## When to use

- You have a serial-over-USB device that doesn't fit `serial`,
  `ethernet`, or `wifi`.
- You're bridging through a host-side proxy (semihosting, RTT, OpenOCD).
- You're prototyping a transport on a target where the static path
  isn't built yet.
- You want a single firmware image that picks transport at boot from
  a config block.

If your transport fits one of the prebuilt static variants
(`platform-posix`, `platform-freertos`, `platform-nuttx`,
`platform-threadx`, `platform-zephyr`, `platform-bare-metal`), prefer
that — the runtime hook trades binary-size optimisation for
flexibility.

## API layering (L0 / L1 / L2)

The custom transport surface is the project's first **canonical-C-ABI**
interface, designed per
[`docs/design/0006-portable-rmw-platform-interface.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/design/0006-portable-rmw-platform-interface.md):

| Layer | Owns | Crates / files |
|-------|------|----------------|
| **L0 — canonical C ABI** | `#[repr(C)]` struct + `abi_version: u32` field + four `unsafe extern "C" fn` pointers + `user_data: *mut c_void` | `nros-rmw::custom_transport` (Rust source); `<nros/transport.h>` is the cbindgen-emitted C header |
| **L1 — language wrappers** | mechanical glue, no new design decisions | `nros-rmw::set_custom_transport` (Rust); `nros_set_custom_transport` (C); `nros::set_custom_transport` (C++) |
| **L2 — typed app API** | n/a — transport is platform-side | — |

**All design decisions live at L0.** A new feature — say, a `flush`
callback — lands in the L0 struct first; L1 wrappers follow
mechanically.

The same canonical-C-ABI pattern was generalised to
the full RMW backend surface (`nros_rmw_vtable_t`). For the
host-language policy that decides whether a given backend lives in
Rust, C, or C++, see [RMW Backends — Host-Language Policy](../internals/rmw-backends.md).

## ABI versioning

The `abi_version: u32` field at the head of the struct is mandatory.
Consumers must fill in `NROS_TRANSPORT_OPS_ABI_VERSION_V1` (Rust:
`nros_rmw::NROS_TRANSPORT_OPS_ABI_VERSION_V1`; C:
`NROS_TRANSPORT_OPS_ABI_VERSION_V1`; C++: filled in automatically by
`nros::set_custom_transport`). Mismatched versions are rejected at
registration time with `NROS_RMW_RET_INCOMPATIBLE_ABI` (`-14`); the
slot stays whatever it was before the bad call.

The version bumps under two rules:

- **Major** (e.g. `V1` → `V2`): existing fields are removed or
  reordered. Old consumers fail cleanly via the version check.
- **Minor** (struct gains an *appended* fn pointer / data field):
  version stays the same. New consumers detect the new fn via the
  size of the trailing `_reserved` region. Today there's no such
  appendage — V1 is the inaugural version.

## Implementing in another language

The L0 struct is plain C ABI (`#[repr(C)]` Rust ↔ `struct` C).
**Any language with C-FFI support can author both sides of the
boundary** — Zig, Python (`ctypes` / `cffi`), Lua-FFI, Go (`cgo`),
Swift (`@_cdecl`), etc.

The reference implementation of "custom transport written in pure C"
lives at
[`packages/rmw/cffi/tests/c_stubs/c_stub_transport.c`](https://github.com/NEWSLabNTU/nano-ros/blob/main/packages/rmw/cffi/tests/c_stubs/c_stub_transport.c).
~80 LOC; no Rust headers / cbindgen output / Rust types involved on
the C side. Use it as a template for ports to other languages.

The corresponding Rust integration test
([`tests/c_stub_transport.rs`](https://github.com/NEWSLabNTU/nano-ros/blob/main/packages/rmw/cffi/tests/c_stub_transport.rs))
exercises the round-trip: register the C-built struct → drive each
fn pointer from Rust → confirm the C-side counters bumped → confirm
abi_version mismatch is rejected. Run via:

```bash
cargo test -p nros-rmw-cffi --features c-stub-test --test c_stub_transport
```

## API

### Rust

```rust
use core::ffi::c_void;
use nros_rmw::{NrosTransportOps, set_custom_transport};

unsafe extern "C" fn my_open(_ud: *mut c_void, _params: *const c_void) -> i32 { 0 }
unsafe extern "C" fn my_close(_ud: *mut c_void) {}
unsafe extern "C" fn my_write(_ud: *mut c_void, buf: *const u8, len: usize) -> i32 {
    // hand `buf[..len]` to the underlying medium, return 0 on success
    0
}
unsafe extern "C" fn my_read(_ud: *mut c_void, buf: *mut u8, len: usize, timeout_ms: u32) -> i32 {
    // read up to `len` bytes within `timeout_ms`, return non-negative count
    0
}

unsafe {
    set_custom_transport(Some(NrosTransportOps {
        abi_version: nros_rmw::NROS_TRANSPORT_OPS_ABI_VERSION_V1,
        _reserved: 0,
        user_data: my_uart_handle as *mut c_void,
        open: my_open,
        close: my_close,
        write: my_write,
        read: my_read,
    })).expect("abi_version must match runtime");
}
```

### C

```c
#include <nros/transport.h>

static nros_ret_t my_open(void *ud, const void *params) {
    (void)params;
    return my_uart_open((my_uart_t *)ud);
}
static void my_close(void *ud) { my_uart_close((my_uart_t *)ud); }
static nros_ret_t my_write(void *ud, const uint8_t *buf, size_t len) {
    return my_uart_write((my_uart_t *)ud, buf, len);
}
static int32_t my_read(void *ud, uint8_t *buf, size_t len, uint32_t timeout_ms) {
    return my_uart_read((my_uart_t *)ud, buf, len, timeout_ms);
}

int main(void) {
    nros_transport_ops_t ops = {
        .abi_version = NROS_TRANSPORT_OPS_ABI_VERSION_V1,
        ._reserved = 0,
        .user_data = &g_uart,
        .open = my_open,
        .close = my_close,
        .write = my_write,
        .read = my_read,
    };
    nros_set_custom_transport(&ops);

    // ... continue with nros_support_init, rclc_node_init_default, etc.
}
```

### C++

```cpp
#include <nros/transport.hpp>

nros::TransportOps ops;
ops.user_data = &g_uart;
ops.open  = [](void *ud, const void*) -> int { return my_uart_open((MyUart*)ud); };
ops.close = [](void *ud)               { my_uart_close((MyUart*)ud); };
ops.write = [](void *ud, const uint8_t *buf, std::size_t len) -> int {
    return my_uart_write((MyUart*)ud, buf, len);
};
ops.read  = [](void *ud, uint8_t *buf, std::size_t len, std::uint32_t to) -> std::int32_t {
    return my_uart_read((MyUart*)ud, buf, len, to);
};
auto r = nros::set_custom_transport(ops);
NROS_TRY(r);
```

> **Captureless lambdas only.** All four fields are raw C function
> pointers, not `std::function`. Pass per-instance state through
> `user_data`.

## Threading contract

| Constraint | Rationale |
|-----------|-----------|
| `read` and `write` MAY be invoked concurrently from different threads on threaded backends. | The zenoh-pico backend runs a dedicated read-task thread on multi-threaded platforms (POSIX, Zephyr, FreeRTOS, NuttX), so `read` runs there while the application/tx thread drives `write`. Single-threaded platforms (bare-metal, ThreadX) instead serialise both through the spin-once / `drive_io` path. **A custom transport must therefore be full-duplex safe**: do not serialise `read` and `write` behind one lock held across a blocking `read`, or the read thread will starve the writer and deadlock session declaration. Most media (TCP, full-duplex UART/USB-CDC) already allow concurrent read and write; mirror that. With `std::net::TcpStream`, store the stream directly and use `&TcpStream` (which implements `Read`/`Write`) from both callbacks rather than a `Mutex<TcpStream>`. |
| Callbacks must NOT be invoked from interrupt context. | The runtime path may take internal locks; ISR context could deadlock. Wrap ISR-driven hardware in a queue + `read` poller. |
| `user_data` must outlive the transport's active period. | The runtime never copies it. Lifetime is from the first callback invocation through `close` returning. |
| `set_custom_transport` must be called BEFORE `nros_support_init`. | The active backend reads the slot during `Rmw::open`. Calling after init is implementation-defined — backends may reject with `NROS_RET_ALREADY_INIT`. |

## Return-code conventions

- `open` / `write` return `0` (`NROS_RMW_RET_OK`) on success, a
  negative `nros_ret_t` (e.g. `NROS_RMW_RET_TIMEOUT`,
  `NROS_RMW_RET_ERROR`) on failure.
- `read` returns the non-negative byte count on success (may be less
  than `len`); a negative `nros_ret_t` on error / timeout.
- `close` returns nothing — failures during teardown are best-effort.

## Framing

Some transports need wire-level framing (HDLC for serial, length-prefix
for stream sockets). The **active backend** decides whether framing is
applied; the user vtable just sees raw bytes.

- **XRCE-DDS**: pass `framing=true` to the backend's
  `init_transport_from_custom_ops(framing)` for byte-stream transports
  (UART, USB-CDC). Pass `framing=false` for packet-oriented transports
  (UDP, BLE GATT).
- **zenoh-pico**: framing is built into the wire protocol — always
  `framing=false` regardless of the underlying medium.

## Backend coverage

| Backend | Status |
|---------|--------|
| **XRCE-DDS** | ✅ Wired. `nros_rmw_xrce::init_transport_from_custom_ops(framing)` pulls the registered vtable into `uxr_set_custom_transport_callbacks` via four C trampolines. |
| **zenoh-pico** | 🟡 Deferred (zenoh). zenoh-pico's custom-link API needs a per-platform `_z_link_t` shim; tracked separately. Zenoh users with custom transports today fork a `zpico-platform-*` crate. |
| **Cyclone DDS** | ➖ N/A. Cyclone DDS binds the platform's network stack directly (POSIX / NSOS / lwIP / NetX-Duo BSD sockets) rather than going through the nano-ros custom-transport vtable; "porting" Cyclone to a new link means providing those sockets, not a transport plug-in. |

## Loopback test

`packages/rmw/xrce/nros-rmw-xrce/tests/custom_transport.rs` exercises the
slot lifecycle + the XRCE bridge round-trip with stub callbacks (no
real session). Run via:

```bash
cargo test -p nros-rmw-xrce --features platform-posix \
    --test custom_transport
```

## See also

- [`<nros/transport.h>`](https://github.com/NEWSLabNTU/nano-ros/blob/main/packages/api/nros-c/include/nros/nros_generated.h) — C header
- [`<nros/transport.hpp>`](https://github.com/NEWSLabNTU/nano-ros/blob/main/packages/api/nros-cpp/include/nros/transport.hpp) — C++ header
- [`nros_rmw::custom_transport`](https://github.com/NEWSLabNTU/nano-ros/blob/main/packages/core/nros-rmw/src/custom_transport.rs) — Rust source
- [`docs/roadmap/phase-115-runtime-transport-vtable.md`](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/archived/phase-115-runtime-transport-vtable.md) — phase doc
- [Custom platform](custom-platform.md) — when you need more than just transport
