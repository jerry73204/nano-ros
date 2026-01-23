# zenoh-pico-shim Architecture

This document describes the architecture of `zenoh-pico-shim`, a crate that provides zenoh-pico support for embedded platforms (Zephyr, smoltcp/bare-metal).

## Problem Statement

We have two separate concerns for embedded zenoh-pico support:

1. **Platform Implementation** (z_* functions)
   - zenoh-pico requires platform-specific implementations for memory, time, random, threading, and sockets
   - Built-in support exists for: Unix, Zephyr, FreeRTOS, Windows
   - **Missing**: smoltcp/bare-metal support

2. **High-Level C Shim** (zenoh_shim_* functions)
   - Needed to avoid FFI struct size mismatches between Rust and C on embedded
   - Provides simpler API for embedded Rust code
   - Manages session/publisher/subscriber state internally

| Platform | Needs Platform Impl? | Needs C Shim? | Reason |
|----------|---------------------|---------------|--------|
| Native (std) | No (built-in unix) | No | Use `zenoh-pico` Rust wrapper directly |
| Zephyr | No (built-in) | **Yes** | FFI struct size issues |
| smoltcp | **Yes** | **Yes** | No built-in support + FFI issues |

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│ Application                                                      │
└─────────────────────────────────────────────────────────────────┘
        │
        ├── native/std targets
        │         │
        │         ▼
        │   ┌─────────────────────────────────────────────────────┐
        │   │ zenoh-pico (existing safe Rust wrapper)             │
        │   │ - Config, Session, Publisher, Subscriber            │
        │   │ - Works unchanged for native targets                │
        │   └─────────────────────────────────────────────────────┘
        │         │
        │         ▼
        │   ┌─────────────────────────────────────────────────────┐
        │   │ zenoh-pico-sys (unchanged)                          │
        │   │ - Compiles zenoh-pico with unix platform            │
        │   │ - Generates FFI bindings via bindgen                │
        │   └─────────────────────────────────────────────────────┘
        │
        └── embedded targets (Zephyr, smoltcp)
                  │
                  ▼
            ┌─────────────────────────────────────────────────────┐
            │ zenoh-pico-shim (NEW)                               │
            │                                                     │
            │ Features:                                           │
            │ - zephyr: uses Zephyr's built-in zenoh-pico         │
            │ - smoltcp: compiles zenoh-pico with custom platform │
            │ - posix: for desktop testing                        │
            │                                                     │
            │ Components:                                         │
            │ ├── include/                                        │
            │ │   ├── zenoh_shim.h           # High-level C API   │
            │ │   └── zenoh_shim_platform.h  # Backend interface  │
            │ ├── src/                                            │
            │ │   ├── shim/                  # C shim layer       │
            │ │   │   ├── zenoh_shim.c       # Core (shared)      │
            │ │   │   ├── backend_zephyr.c   # Zephyr backend     │
            │ │   │   ├── backend_smoltcp.c  # smoltcp backend    │
            │ │   │   └── backend_posix.c    # POSIX backend      │
            │ │   ├── platform_smoltcp/      # z_* for smoltcp    │
            │ │   │   ├── system.c           # z_malloc, z_clock  │
            │ │   │   ├── network.c          # _z_open_tcp, etc.  │
            │ │   │   └── mod.rs             # Rust FFI exports   │
            │ │   └── lib.rs                 # Safe Rust wrapper  │
            │ └── zenoh-pico/                # Git submodule      │
            └─────────────────────────────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
┌─────────────────────────┐     ┌─────────────────────────────────┐
│ zenoh-pico C library    │     │ smoltcp (when feature enabled)  │
│ (compiled by shim)      │     │ - TCP/IP stack                  │
└─────────────────────────┘     └─────────────────────────────────┘
```

## Design Rationale

1. **Existing crates unchanged** - `zenoh-pico-sys` and `zenoh-pico` remain stable
2. **Clear separation** - `zenoh-pico` for std, `zenoh-pico-shim` for embedded
3. **Tightly coupled components together** - Platform impl and C shim are closely related for smoltcp (polling vs blocking)
4. **Feature flags** - Clean separation between Zephyr (built-in platform) and smoltcp (custom platform)

## Directory Structure

```
crates/zenoh-pico-shim/
├── Cargo.toml
├── build.rs                      # Compiles C code, links zenoh-pico
├── include/
│   ├── zenoh_shim.h              # Public C API
│   └── zenoh_shim_platform.h     # Backend interface
├── src/
│   ├── lib.rs                    # Safe Rust wrapper
│   │
│   ├── shim/                     # High-level C shim
│   │   ├── zenoh_shim.c          # Core implementation (shared)
│   │   ├── backend_zephyr.c      # Zephyr-specific (uses threads)
│   │   ├── backend_smoltcp.c     # smoltcp-specific (uses polling)
│   │   └── backend_posix.c       # POSIX for desktop testing
│   │
│   └── platform_smoltcp/         # zenoh-pico platform layer
│       ├── mod.rs                # Rust FFI exports (smoltcp calls)
│       ├── system.c              # z_malloc, z_random_*, z_clock_*
│       └── network.c             # _z_open_tcp, _z_read_tcp, etc.
│
└── zenoh-pico/                   # Git submodule
    └── ...
```

## Cargo.toml

```toml
[package]
name = "zenoh-pico-shim"
version = "0.1.0"
edition = "2021"
links = "zenoh_pico_shim"         # Prevents multiple linking

[features]
default = []
zephyr = []                       # Use Zephyr's built-in zenoh-pico
smoltcp = ["dep:smoltcp"]         # Use custom smoltcp platform
posix = []                        # For desktop testing

[dependencies]
smoltcp = { version = "0.11", default-features = false, optional = true, features = [
    "medium-ethernet",
    "proto-ipv4",
    "socket-tcp",
] }

[build-dependencies]
cc = "1.0"
cmake = "0.1"
```

## Build Process

```rust
// build.rs
fn main() {
    // For smoltcp: compile zenoh-pico with custom platform
    // For zephyr: Zephyr builds zenoh-pico via west (we only compile shim)
    // For posix: compile zenoh-pico with unix platform

    if cfg!(feature = "smoltcp") {
        // 1. Compile our custom platform C files
        cc::Build::new()
            .file("src/platform_smoltcp/system.c")
            .file("src/platform_smoltcp/network.c")
            .include("zenoh-pico/include")
            .compile("zenoh_pico_platform");

        // 2. Compile zenoh-pico with our platform
        let dst = cmake::Config::new("zenoh-pico")
            .define("BUILD_SHARED_LIBS", "OFF")
            .define("Z_FEATURE_MULTI_THREAD", "0")
            .define("Z_FEATURE_LINK_TCP", "1")
            .define("ZENOH_PLATFORM", "custom")
            .build();

        println!("cargo:rustc-link-search=native={}/lib", dst.display());
        println!("cargo:rustc-link-lib=static=zenohpico");
    }

    // Compile shim with appropriate backend
    let mut shim = cc::Build::new();
    shim.file("src/shim/zenoh_shim.c")
        .include("include")
        .include("zenoh-pico/include");

    if cfg!(feature = "zephyr") {
        shim.file("src/shim/backend_zephyr.c");
    } else if cfg!(feature = "smoltcp") {
        shim.file("src/shim/backend_smoltcp.c");
    } else if cfg!(feature = "posix") {
        shim.file("src/shim/backend_posix.c");
    }

    shim.compile("zenoh_shim");
}
```

## C API

### Public API (`zenoh_shim.h`)

```c
#ifndef ZENOH_SHIM_H
#define ZENOH_SHIM_H

#include <stdint.h>
#include <stddef.h>

// Callback type for received samples
typedef void (*zenoh_shim_callback_t)(const uint8_t *data, size_t len, void *ctx);

// Error codes
#define ZENOH_SHIM_OK              0
#define ZENOH_SHIM_ERR_INIT       -1
#define ZENOH_SHIM_ERR_SESSION    -2
#define ZENOH_SHIM_ERR_PUBLISHER  -3
#define ZENOH_SHIM_ERR_SUBSCRIBER -4
#define ZENOH_SHIM_ERR_PUBLISH    -5
#define ZENOH_SHIM_ERR_NO_SLOTS   -6

// Session management
int zenoh_shim_init(const char *locator);
int zenoh_shim_open(void);
int zenoh_shim_is_open(void);
void zenoh_shim_close(void);

// Publishers (returns handle >= 0 on success)
int zenoh_shim_declare_publisher(const char *keyexpr);
int zenoh_shim_publish(int handle, const uint8_t *data, size_t len);
int zenoh_shim_undeclare_publisher(int handle);

// Subscribers (returns handle >= 0 on success)
int zenoh_shim_declare_subscriber(const char *keyexpr, zenoh_shim_callback_t cb, void *ctx);
int zenoh_shim_undeclare_subscriber(int handle);

// Polling (for platforms without background threads)
int zenoh_shim_poll(uint32_t timeout_ms);
int zenoh_shim_spin_once(uint32_t timeout_ms);

#endif
```

### Backend Interface (`zenoh_shim_platform.h`)

```c
#ifndef ZENOH_SHIM_PLATFORM_H
#define ZENOH_SHIM_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

// Implemented by each backend (zephyr, smoltcp, posix)

// Returns true if platform requires explicit polling (smoltcp)
// Returns false if platform uses background threads (zephyr, posix)
bool zenoh_platform_uses_polling(void);

// Start/stop background tasks (for threaded platforms)
int zenoh_platform_start_tasks(void *session);
void zenoh_platform_stop_tasks(void *session);

// Poll for network events (for polling platforms)
int zenoh_platform_poll(void *session, uint32_t timeout_ms);

// Time utilities
uint64_t zenoh_platform_time_ms(void);
void zenoh_platform_sleep_ms(uint32_t ms);

#endif
```

## Platform Layer for smoltcp

The smoltcp platform implements zenoh-pico's required z_* functions, bridging zenoh-pico's blocking API to smoltcp's polling API.

### C Implementation (`platform_smoltcp/network.c`)

```c
#include <zenoh-pico/system/platform.h>

// FFI to Rust smoltcp module
extern int smoltcp_socket_open(uint8_t ip[4], uint16_t port, uint32_t timeout_ms);
extern int smoltcp_socket_send(int handle, const uint8_t *data, size_t len);
extern int smoltcp_socket_recv(int handle, uint8_t *data, size_t len, uint32_t timeout_ms);
extern void smoltcp_socket_close(int handle);

z_result_t _z_open_tcp(_z_sys_net_socket_t *sock,
                        const _z_sys_net_endpoint_t rep,
                        uint32_t tout) {
    int handle = smoltcp_socket_open(
        rep._iptcp._addr._v4.addr,
        rep._iptcp._port,
        tout
    );
    if (handle < 0) return _Z_ERR_GENERIC;
    sock->_fd = handle;
    return _Z_RES_OK;
}

size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    int n = smoltcp_socket_recv(sock._fd, ptr, len, Z_CONFIG_SOCKET_TIMEOUT);
    return (n < 0) ? SIZE_MAX : (size_t)n;
}

size_t _z_send_tcp(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    int n = smoltcp_socket_send(sock._fd, ptr, len);
    return (n < 0) ? SIZE_MAX : (size_t)n;
}

void _z_close_tcp(_z_sys_net_socket_t *sock) {
    smoltcp_socket_close(sock->_fd);
    sock->_fd = -1;
}
```

### Rust FFI (`platform_smoltcp/mod.rs`)

```rust
use smoltcp::iface::Interface;
use smoltcp::socket::{tcp, SocketSet};
use smoltcp::time::Instant;
use smoltcp::wire::IpAddress;
use core::ffi::c_int;

// Global state - initialized by application before zenoh operations
static mut IFACE: Option<&'static mut Interface> = None;
static mut SOCKETS: Option<&'static mut SocketSet<'static>> = None;
static mut DEVICE: Option<&'static mut dyn smoltcp::phy::Device> = None;

/// Initialize smoltcp platform. Must be called before any zenoh operations.
pub unsafe fn init(
    iface: &'static mut Interface,
    sockets: &'static mut SocketSet<'static>,
    device: &'static mut dyn smoltcp::phy::Device,
) {
    IFACE = Some(iface);
    SOCKETS = Some(sockets);
    DEVICE = Some(device);
}

#[no_mangle]
pub extern "C" fn smoltcp_socket_open(ip: *const u8, port: u16, timeout_ms: u32) -> c_int {
    // Parse IP, create socket, connect with polling loop and timeout
    // Returns socket handle or negative error
}

#[no_mangle]
pub extern "C" fn smoltcp_socket_recv(
    handle: c_int, data: *mut u8, len: usize, timeout_ms: u32
) -> c_int {
    // Poll until data available or timeout
    // Returns bytes read or negative error
}

#[no_mangle]
pub extern "C" fn smoltcp_socket_send(handle: c_int, data: *const u8, len: usize) -> c_int {
    // Send with polling
    // Returns bytes sent or negative error
}

#[no_mangle]
pub extern "C" fn smoltcp_socket_close(handle: c_int) {
    // Close and remove socket
}
```

## Rust Safe Wrapper

```rust
// src/lib.rs
#![no_std]

extern crate alloc;

use core::ffi::c_char;
use core::marker::PhantomData;

#[cfg(feature = "smoltcp")]
pub mod platform_smoltcp;

extern "C" {
    fn zenoh_shim_init(locator: *const c_char) -> i32;
    fn zenoh_shim_open() -> i32;
    fn zenoh_shim_is_open() -> i32;
    fn zenoh_shim_close();
    fn zenoh_shim_declare_publisher(keyexpr: *const c_char) -> i32;
    fn zenoh_shim_publish(handle: i32, data: *const u8, len: usize) -> i32;
    fn zenoh_shim_undeclare_publisher(handle: i32) -> i32;
    fn zenoh_shim_poll(timeout_ms: u32) -> i32;
    fn zenoh_shim_spin_once(timeout_ms: u32) -> i32;
}

#[derive(Debug, Clone, Copy)]
pub enum ShimError {
    InitFailed,
    SessionFailed,
    PublisherFailed,
    PublishFailed,
    NotConnected,
}

/// Context managing the zenoh session
pub struct ShimContext {
    _marker: PhantomData<*const ()>,
}

impl ShimContext {
    pub fn new(locator: &[u8]) -> Result<Self, ShimError> {
        unsafe {
            if zenoh_shim_init(locator.as_ptr() as *const c_char) < 0 {
                return Err(ShimError::InitFailed);
            }
            if zenoh_shim_open() < 0 {
                return Err(ShimError::SessionFailed);
            }
        }
        Ok(Self { _marker: PhantomData })
    }

    pub fn is_open(&self) -> bool {
        unsafe { zenoh_shim_is_open() != 0 }
    }

    pub fn poll(&self, timeout_ms: u32) -> Result<i32, ShimError> {
        let ret = unsafe { zenoh_shim_poll(timeout_ms) };
        if ret < 0 { Err(ShimError::NotConnected) } else { Ok(ret) }
    }

    pub fn spin_once(&self, timeout_ms: u32) -> Result<i32, ShimError> {
        let ret = unsafe { zenoh_shim_spin_once(timeout_ms) };
        if ret < 0 { Err(ShimError::NotConnected) } else { Ok(ret) }
    }

    pub fn declare_publisher(&self, keyexpr: &[u8]) -> Result<ShimPublisher, ShimError> {
        let handle = unsafe { zenoh_shim_declare_publisher(keyexpr.as_ptr() as *const c_char) };
        if handle < 0 { Err(ShimError::PublisherFailed) } else { Ok(ShimPublisher { handle }) }
    }
}

impl Drop for ShimContext {
    fn drop(&mut self) {
        unsafe { zenoh_shim_close() }
    }
}

pub struct ShimPublisher {
    handle: i32,
}

impl ShimPublisher {
    pub fn publish(&self, data: &[u8]) -> Result<(), ShimError> {
        let ret = unsafe { zenoh_shim_publish(self.handle, data.as_ptr(), data.len()) };
        if ret < 0 { Err(ShimError::PublishFailed) } else { Ok(()) }
    }
}

impl Drop for ShimPublisher {
    fn drop(&mut self) {
        unsafe { zenoh_shim_undeclare_publisher(self.handle) };
    }
}
```

## Usage Examples

### Zephyr (uses shim for FFI, built-in platform)

```rust
use zenoh_pico_shim::{ShimContext, ShimPublisher};

let ctx = ShimContext::new(b"tcp/192.0.2.2:7447\0")?;
let publisher = ctx.declare_publisher(b"/chatter\0")?;

loop {
    publisher.publish(&serialized_msg)?;
    // Background threads handle network automatically
    zephyr::time::sleep(Duration::from_secs(1));
}
```

### smoltcp/RTIC (uses shim + custom platform)

```rust
use zenoh_pico_shim::{ShimContext, ShimPublisher, platform_smoltcp};

// Initialize smoltcp first (in RTIC init)
unsafe {
    platform_smoltcp::init(&mut iface, &mut sockets, &mut device);
}

let ctx = ShimContext::new(b"tcp/192.168.1.1:7447\0")?;
let publisher = ctx.declare_publisher(b"/chatter\0")?;

// Main loop with explicit polling
loop {
    ctx.spin_once(10)?;  // Poll network + zenoh
    publisher.publish(&serialized_msg)?;
}
```

## Summary

| Crate | Purpose | Status |
|-------|---------|--------|
| `zenoh-pico-sys` | FFI bindings to zenoh-pico C lib | **Unchanged** |
| `zenoh-pico` | Safe Rust wrapper for std targets | **Unchanged** |
| `zenoh-pico-shim` | Embedded support | **New** |

The `zenoh-pico-shim` crate contains:
- **C shim layer** - High-level API avoiding FFI struct issues
- **Platform backends** - Zephyr (threads), smoltcp (polling), POSIX (testing)
- **smoltcp platform** - Custom z_* implementations for bare-metal
- **Rust wrapper** - Safe API for embedded applications
