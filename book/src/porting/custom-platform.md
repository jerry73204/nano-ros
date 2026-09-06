# Custom Platform

This guide walks through porting nano-ros to a new RTOS or bare-metal environment. A "platform" provides the OS-level primitives that nano-ros needs at runtime: clock, memory, sleep, threading, and networking. The core library is `#![no_std]` and makes zero platform calls directly -- everything flows through your platform crate.

> **Canonical interface spec.** The function-pointer signatures,
> parameter docs, ownership rules, blocking allowance, and failure
> modes for every method live in the
> [platform-cffi Doxygen reference](../api/platform-cffi/index.html).
> Read it side-by-side with this guide — the tables below summarise
> the **set** of traits you need; the Doxygen documents the
> **contract** each call must obey.

> **Quick differences table.** For a per-platform comparison
> (clock source, allocator, threading, networking, multicast
> support) across the platforms nano-ros already supports, see
> [Platform Differences](../reference/platform-differences.md).

## What you implement

All platform traits are defined in `nros-platform/src/traits.rs`. Your platform crate implements some or all of them as inherent methods on a zero-sized type (ZST). The set you need depends on your RMW backend.

### Required for all backends

| Trait | Methods | Purpose |
|---|---|---|
| `PlatformClock` | `clock_ms()`, `clock_us()` | Monotonic time. Must use a hardware timer or OS tick -- never a software counter that only advances when polled. |

### Required for zenoh-pico (rmw-zenoh)

| Trait | Methods | Purpose |
|---|---|---|
| `PlatformAlloc` | `alloc()`, `realloc()`, `dealloc()` | Heap allocation. zenoh-pico needs ~64 KB. |
| `PlatformSleep` | `sleep_us()`, `sleep_ms()`, `sleep_s()` | Delay. On bare-metal with smoltcp, poll the network during busy-wait. |
| `PlatformRandom` | `random_u8()` through `random_u64()`, `random_fill()` | PRNG for session IDs and protocol nonces. |
| `PlatformTime` | `time_now_ms()`, `time_since_epoch()` | Wall-clock time for logging. Return monotonic time if no RTC. |
| `PlatformThreading` | Tasks, mutexes, recursive mutexes, condvars (19 methods) | OS threading primitives. Single-threaded platforms provide no-op stubs. |

### Networking

| Trait | Methods | Purpose |
|---|---|---|
| `PlatformTcp` | `open()`, `read()`, `send()`, `close()`, ... | TCP client and server sockets. |
| `PlatformUdp` | `open()`, `read()`, `send()`, `close()`, ... | UDP unicast sockets. |
| `PlatformSocketHelpers` | `set_non_blocking()`, `accept()`, `close()`, `wait_event()` | Socket utility operations. |

### Optional

| Trait | When needed |
|---|---|
| `PlatformUdpMulticast` | Desktop platforms using zenoh scouting. Not needed for embedded client mode. |
| `PlatformNetworkPoll` | Bare-metal platforms using smoltcp. Called during sleep to process packets. |
| `PlatformLibc` | Bare-metal targets without a C runtime. Provides `strlen`, `memcpy`, etc. |

For full method signatures, see the [Platform API Reference](../reference/platform-api.md).

## Wiring into nros

Five files need changes to register a new platform. This example adds a fictional "MyOS" platform.

### 1. Create the platform crate

```
packages/platform/nros-platform-myos/
  Cargo.toml
  src/
    lib.rs
```

The crate must have **zero** `nros-*` dependencies. It may depend on your RTOS bindings, HAL crates, or `embedded-alloc`.

### 2. Add the feature to nros-platform

In `packages/platform/nros-platform/Cargo.toml`:

```toml
[features]
platform-myos = ["dep:nros-platform-myos"]

[dependencies]
nros-platform-myos = { version = "0.1.0", path = "../nros-platform-myos", optional = true }
```

### 3. Add the ConcretePlatform alias

In `packages/platform/nros-platform/src/resolve.rs`:

```rust
#[cfg(feature = "platform-myos")]
pub type ConcretePlatform = nros_platform_myos::MyOsPlatform;
```

### 4. Propagate through the nros facade

In `packages/api/nros/Cargo.toml`, add `platform-myos` to the feature list so users can write `nros = { features = ["rmw-zenoh", "platform-myos"] }`.

### 5. Register your platform as an ABI marker

The platform shim crates are gone (Phase 129). Each transport C library's
platform symbols now come from a **default-on C alias translation unit** that
forwards to the canonical `nros_platform_*` ABI — `zpico-sys`'s
`platform-aliases` feature for zenoh-pico (`z_*` / `_z_*`), and
`nros-rmw-xrce`'s always-compiled `src/platform_aliases.c` for XRCE-DDS
(`uxr_*`). You do not activate or configure a shim.

What you add is a **pure ABI marker** feature so each transport crate's
`build.rs` can do per-platform source selection (e.g. strip the vendor
`system/<rtos>/system.c`, gate the alias TU's network section to bare-metal):

```toml
# packages/rmw/zenoh/zpico-sys/Cargo.toml  (rmw-zenoh)
[features]
myos = []   # ABI marker only — no shim dependency
```

**Only zenoh needs one.** XRCE-DDS has no per-platform feature to add:
phase-129.C.1 deleted all of `nros-rmw-xrce-cffi`'s `platform-<rtos>`
features, and its build selection is driven by `target_os` instead, over
a platform-blind `transport_nros_udp.c`. There is also no crate left to
add one to — the `xrce-sys` crate was deleted in phase-321 W1.d and the
directory is now a submodule host for `micro-xrce-dds-client/` and
`micro-cdr/`, whose README says in as many words: do not re-add a crate
here. If your port targets XRCE only, skip this step entirely.

The alias TU (default-on `platform-aliases`) covers the full `z_*` surface —
memory, sleep, random, time, yield, threading, condvar, clock, and network.
`zpico-sys`'s `build.rs` defines `NROS_PLATFORM_ALIASES` and emits the network
wrappers only where the vendor stack doesn't already provide them (bare-metal).
If your RTOS's vendor `system.c` already supplies these symbols natively (as on
Orin SPE's FSP FreeRTOS), turn `platform-aliases` **off** for that platform to
avoid duplicate-symbol link errors.

## Rust path

This is the recommended approach. Create a ZST and implement each capability as inherent methods (not trait impls). `nros-platform-cffi` exposes these methods as the canonical `nros_platform_*` C symbols, which the alias TUs call.

### Skeleton

```rust
// packages/platform/nros-platform-myos/src/lib.rs
#![no_std]
use core::ffi::c_void;

/// Zero-sized type implementing platform methods for MyOS.
pub struct MyOsPlatform;

// -- Clock --
impl MyOsPlatform {
    pub fn clock_ms() -> u64 {
        // Call your RTOS tick API, e.g.:
        // unsafe { myos_get_tick_count() as u64 }
        todo!()
    }

    pub fn clock_us() -> u64 { Self::clock_ms() * 1000 }
}

// -- Alloc --
impl MyOsPlatform {
    pub fn alloc(size: usize) -> *mut c_void {
        // unsafe { myos_malloc(size) }
        todo!()
    }

    pub fn realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
        // If your RTOS lacks realloc: alloc new, copy, free old
        todo!()
    }

    pub fn dealloc(ptr: *mut c_void) {
        // unsafe { myos_free(ptr) }
        todo!()
    }
}

// -- Sleep --
impl MyOsPlatform {
    pub fn sleep_us(us: usize) { Self::sleep_ms(us.div_ceil(1000)); }
    pub fn sleep_ms(ms: usize) {
        // unsafe { myos_thread_sleep(ms as u32) }
        todo!()
    }
    pub fn sleep_s(s: usize) { Self::sleep_ms(s * 1000); }
}

// -- Threading (stubs for single-threaded, real impls for RTOS) --
impl MyOsPlatform {
    pub fn mutex_init(m: *mut c_void) -> i8 {
        // Create a mutex via your RTOS API. Store the handle in `m`.
        // Return 0 on success, -1 on failure.
        todo!()
    }
    pub fn mutex_lock(m: *mut c_void) -> i8 { todo!() }
    pub fn mutex_unlock(m: *mut c_void) -> i8 { todo!() }
    // ... remaining threading methods (see traits.rs for the full list)
}
```

### Key points

- **Inherent methods, not trait impls.** The shim calls `ConcretePlatform::clock_ms()` directly. The traits in `traits.rs` document the contract, but the ZST uses inherent `impl` blocks.
- **`c_void` pointers for handles.** Mutex, condvar, and task handles are opaque `#[repr(C)]` structs sized to hold your RTOS handle. Cast the `*mut c_void` to your internal type.
- **Recursive mutexes are required.** zenoh-pico locks the same mutex recursively. On FreeRTOS this maps to `xSemaphoreCreateRecursiveMutex`; on pthreads, `PTHREAD_MUTEX_RECURSIVE`.
- **Seed the PRNG.** A deterministic seed (like FreeRTOS `rand()` starting from 1) causes duplicate zenoh session IDs across QEMU instances. Seed from hardware entropy, IP address, or semihosting wall-clock.

### Reference implementation

`packages/platform/nros-platform-freertos/src/lib.rs` is a complete real-world example covering all categories: clock via `xTaskGetTickCount`, heap via `pvPortMalloc`/`vPortFree`, sleep via `vTaskDelay`, xorshift32 PRNG, and full threading with tasks, recursive mutexes, and condvars built on counting semaphores.

## C/C++ path

If your platform is easier to implement in C, use the `nros-platform-cffi` C ABI. It is the canonical layer between nros and any C-implemented platform port.

### 1. Implement the platform symbols

The canonical header lives at
`packages/platform/nros-platform-cffi/include/nros/platform.h`. It declares
roughly 45 free `extern "C"` functions — one per platform capability.
Your port supplies a definition for each; the linker resolves them
into the nros binary directly. There is no runtime registration call.
Browse the rendered reference at
[/api/platform-cffi/](../api/platform-cffi/index.html) for per-function
return-value, threading, and blocking conventions.

Include the header in your port:

```c
// my_platform.c
#include <nros/platform.h>
```

Then define each symbol:

```c
uint64_t nros_platform_clock_ms(void) {
    return myos_get_ticks();  // your RTOS tick API
}

void *nros_platform_alloc(size_t size) {
    return myos_malloc(size);
}

void nros_platform_dealloc(void *ptr) {
    myos_free(ptr);
}

void *nros_platform_realloc(void *ptr, size_t size) {
    return myos_realloc(ptr, size);
}

int8_t nros_platform_mutex_init(void *m) {
    myos_mutex_t *mx = (myos_mutex_t *)m;
    *mx = myos_mutex_create();
    return (*mx != NULL) ? 0 : -1;
}

/* ... define every other symbol declared in <nros/platform.h> ... */
```

### 2. Link

Compile your translation unit into a static or object library and link
it into the nros binary alongside the nros static library. No
registration step is required at boot:

```c
int main(void) {
    myos_init();

    /* nros calls nros_platform_* symbols directly */
    nros_executor_t exec;
    nros_executor_open(&exec, &config);
    /* ... */
}
```

### 3. Build configuration

Enable the `platform-cffi` feature instead of a platform-specific feature:

```toml
nros = { features = ["rmw-zenoh", "platform-cffi"] }
```

> The `rmw-zenoh` feature is the *lowering* of the declared RMW: the
> backend is declared once in `system.toml` (`[system].rmw` /
> `[image.<id>].rmw`) and the toolchain emits the cargo feature. The
> feature is the build mechanism, not the user-facing knob — see
> [RFC-0031](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/design/0031-rmw-selection-and-lowering.md).

All symbols declared in `<nros/platform.h>` are required. For capabilities your platform does not support (e.g., threading on single-threaded bare-metal), supply stubs that return 0 for mutex/condvar operations and -1 for `nros_platform_task_init`.

## Networking

There are two paths for providing TCP/UDP sockets to zenoh-pico.

### Option A: Rust networking (preferred)

Implement `PlatformTcp`, `PlatformUdp`, and `PlatformSocketHelpers` on your ZST. These methods map to your OS socket API (BSD sockets, lwIP, NetX Duo, etc.).

```rust
impl MyOsPlatform {
    pub fn tcp_open(sock: *mut c_void, endpoint: *const c_void, timeout_ms: u32) -> i8 {
        // Parse endpoint, call connect(), store fd in sock
        todo!()
    }

    pub fn tcp_read(sock: *const c_void, buf: *mut u8, len: usize) -> usize {
        // Call recv() on the socket fd
        todo!()
    }

    pub fn tcp_send(sock: *const c_void, buf: *const u8, len: usize) -> usize {
        // Call send() on the socket fd
        todo!()
    }

    pub fn tcp_close(sock: *mut c_void) {
        // Close the socket fd
    }
}
```

Activate the `network` shim feature in `zpico-sys` so the shim provides the `_z_open_tcp`, `_z_read_tcp`, etc. C symbols by forwarding to your Rust methods.

For bare-metal with smoltcp, use `nros-smoltcp` (in `packages/drivers/`) as the networking driver. It provides `PlatformTcp` and `PlatformUdp` implementations using smoltcp's TCP/UDP sockets. The MAC/PHY driver lives in a sibling driver crate (e.g. `lan9118-smoltcp`, `openeth-smoltcp`) and implements smoltcp's `Device` trait; `nros-smoltcp` consumes that `Device` and exposes the transport to zenoh-pico. Your platform crate implements `PlatformNetworkPoll` so the sleep loop can process packets — the platform crate stays free of smoltcp itself.

### Option B: Keep zenoh-pico's C network.c

If your platform already has a working zenoh-pico `network.c` (e.g., `freertos/lwip/network.c` or `unix/network.c`), you can compile it directly instead of implementing the Rust networking traits.

In this case, do **not** activate the `network` shim feature in zpico-sys. Instead, link the appropriate `network.c` through your build system. The C file provides the `_z_open_tcp`, `_z_read_tcp`, etc. symbols directly, bypassing the Rust shim.

This is the approach used by platforms with mature C networking stacks (lwIP on FreeRTOS, BSD sockets on NuttX, NetX Duo on ThreadX).

## Common pitfalls

- **Poll-driven clocks.** If the clock only advances when you call a function, timeouts and keep-alives break silently. Use a free-running hardware timer.
- **Stack overflow on RTOS.** The application task's frame is dominated by the transport session open and by the `Executor` value the constructor builds and returns; size it by measuring an overflow, not by copying a number. (The executor's *arena* is no longer part of that frame — it is borrowed from caller-supplied backing, which is a `.bss` static on every in-tree entry.)
- **Deterministic PRNG seeds.** Duplicate zenoh session IDs cause silent connection failures. Seed from a source that varies across instances.
- **Missing recursive mutexes.** zenoh-pico re-enters the same mutex. Non-recursive mutexes deadlock.
- **QEMU clock drift.** Use `-icount shift=auto` for QEMU targets so the virtual clock tracks wall time during WFI.

## Next steps

- [Custom Board Package](custom-board.md) -- create a board crate that ties your platform to specific hardware
- [Platform API Reference](../reference/platform-api.md) -- complete method signatures for all traits
- [Platform Porting Pitfalls](../internals/platform-porting-pitfalls.md) -- QEMU networking and timing issues
