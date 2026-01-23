# smoltcp + zenoh-pico Integration Design

This document describes the architecture for integrating smoltcp (pure-Rust TCP/IP stack) with zenoh-pico (C zenoh client) for bare-metal RTIC/polling embedded systems.

For implementation tasks, see [Phase 8: Embedded Networking](../roadmap/phase-8-embedded-networking.md).

## Overview

The goal is to enable network communication for bare-metal nano-ros examples (rtic-stm32f4, polling-stm32f4) using:
- **smoltcp**: Pure-Rust TCP/IP stack (no OS required)
- **zenoh-pico**: C library for zenoh protocol
- **stm32-eth**: Rust Ethernet driver for STM32

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Rust Application (RTIC/polling)                        │
│  - PollingExecutor                                      │
│  - spin_once() calls zenoh polling                      │
├─────────────────────────────────────────────────────────┤
│  nano-ros-node (Rust)                                   │
│  - Uses SmoltcpTransport                                │
├─────────────────────────────────────────────────────────┤
│  SmoltcpTransport (Rust)                                │
│  - Wraps zenoh-pico session                             │
│  - Manages smoltcp polling                              │
├─────────────────────────────────────────────────────────┤
│  zenoh-pico (C)                                         │
│  - Configured with Z_FEATURE_MULTI_THREAD=0             │
│  - Uses smoltcp platform layer                          │
├─────────────────────────────────────────────────────────┤
│  zenoh-pico-smoltcp Platform Layer (C/Rust FFI)         │
│  - Implements z_* platform functions                    │
│  - Maps blocking sockets to smoltcp polling             │
├─────────────────────────────────────────────────────────┤
│  smoltcp (Rust)                                         │
│  - Interface, TcpSocket                                 │
│  - Polling-based I/O                                    │
├─────────────────────────────────────────────────────────┤
│  stm32-eth (Rust)                                       │
│  - Device trait implementation                          │
│  - DMA-based Ethernet driver                            │
├─────────────────────────────────────────────────────────┤
│  STM32F4 Hardware                                       │
│  - Ethernet MAC                                         │
│  - PHY (LAN8742A on NUCLEO-F429ZI)                      │
└─────────────────────────────────────────────────────────┘
```

## zenoh-pico Platform Layer Requirements

Based on analysis of zenoh-pico source code, the following functions must be implemented:

### 1. Memory Management

```c
void *z_malloc(size_t size);
void *z_realloc(void *ptr, size_t size);
void z_free(void *ptr);
```

**Implementation**: Use `embedded-alloc` crate with static heap (~16KB).

### 2. Random Number Generation

```c
uint8_t z_random_u8(void);
uint16_t z_random_u16(void);
uint32_t z_random_u32(void);
uint64_t z_random_u64(void);
void z_random_fill(void *buf, size_t len);
```

**Implementation**: Use STM32 hardware RNG (RNG peripheral) or seeded PRNG.

### 3. Time & Clock

```c
z_clock_t z_clock_now(void);
unsigned long z_clock_elapsed_us(z_clock_t *time);
unsigned long z_clock_elapsed_ms(z_clock_t *time);
z_time_t z_time_now(void);
z_result_t z_sleep_us(size_t time);
z_result_t z_sleep_ms(size_t time);
```

**Implementation**: Use DWT cycle counter (Cortex-M debug unit) or SysTick.

### 4. Threading (Disabled)

With `Z_FEATURE_MULTI_THREAD=0`, threading functions become no-ops:

```c
z_result_t _z_mutex_init(_z_mutex_t *m) { return _Z_RES_OK; }
z_result_t _z_mutex_lock(_z_mutex_t *m) { return _Z_RES_OK; }
z_result_t _z_mutex_unlock(_z_mutex_t *m) { return _Z_RES_OK; }
```

### 5. Network Sockets

This is the critical integration point with smoltcp:

```c
typedef struct {
    uint8_t socket_handle;  // Index into smoltcp socket set
    bool connected;
} _z_sys_net_socket_t;

typedef struct {
    uint8_t ip[4];          // IPv4 address
    uint16_t port;
} _z_sys_net_endpoint_t;

// Functions to implement
z_result_t _z_create_endpoint_tcp(...);
z_result_t _z_open_tcp(...);
size_t _z_read_tcp(...);
size_t _z_send_tcp(...);
void _z_close_tcp(...);
```

## smoltcp Integration Details

### Global State

Since zenoh-pico expects a C-style socket API, we need global state to manage smoltcp:

```rust
static mut SMOLTCP_INTERFACE: Option<Interface> = None;
static mut SMOLTCP_DEVICE: Option<EthernetDevice> = None;
static mut SMOLTCP_SOCKETS: Option<SocketSet> = None;

const MAX_SOCKETS: usize = 4;
static mut SOCKET_HANDLES: [Option<SocketHandle>; MAX_SOCKETS] = [None; MAX_SOCKETS];
```

### Blocking Socket Operations

zenoh-pico expects blocking socket operations, but smoltcp is polling-based. Solution: **polling loops with timeout**.

```c
size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    z_clock_t start = z_clock_now();
    while (true) {
        smoltcp_poll();  // Process packets via Rust FFI
        size_t n = smoltcp_try_recv(sock.socket_handle, ptr, len);
        if (n > 0) return n;
        if (z_clock_elapsed_ms(&start) > Z_CONFIG_SOCKET_TIMEOUT) {
            return SIZE_MAX;  // Timeout error
        }
    }
}
```

### Interface Polling

smoltcp's `Interface::poll()` must be called regularly. We call it from within socket operations:

```rust
#[no_mangle]
pub extern "C" fn smoltcp_poll() {
    unsafe {
        if let (Some(iface), Some(device), Some(sockets)) =
            (&mut SMOLTCP_INTERFACE, &mut SMOLTCP_DEVICE, &mut SMOLTCP_SOCKETS)
        {
            iface.poll(smoltcp::time::Instant::now(), device, sockets);
        }
    }
}
```

## Memory Budget

| Component | Size | Notes |
|-----------|------|-------|
| Ethernet DMA buffers | ~8 KB | 4 RX + 4 TX descriptors |
| smoltcp Interface | ~1 KB | Configuration dependent |
| Socket Set | ~100 B | Metadata only |
| TCP Socket RX | 2 KB × N | Per socket |
| TCP Socket TX | 2 KB × N | Per socket |
| zenoh-pico heap | ~16 KB | Session + publishers + subscribers |
| **Total (2 sockets)** | **~33 KB** | Fits in STM32F429 (256 KB RAM) |

## Build Configuration

### CMake for zenoh-pico Cross-Compilation

```cmake
# toolchain-arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fno-exceptions")

# Disable features not needed for embedded
set(Z_FEATURE_MULTI_THREAD 0)
set(Z_FEATURE_LINK_UDP_MULTICAST 0)
set(Z_FEATURE_LINK_UDP_UNICAST 0)
set(Z_FEATURE_LINK_TCP 1)
set(Z_FEATURE_SCOUTING_UDP 0)
```

### Cargo Configuration

```toml
# examples/rtic-stm32f4/Cargo.toml
[dependencies]
smoltcp = { version = "0.11", default-features = false, features = [
    "medium-ethernet",
    "proto-ipv4",
    "socket-tcp",
] }
stm32-eth = { version = "0.6", features = ["stm32f429"] }
embedded-alloc = "0.5"
zenoh-pico-sys = { path = "../../crates/zenoh-pico-sys", features = ["smoltcp"] }
```

## Alternative: C Shim Pattern (Like Zephyr)

Instead of implementing the full platform layer in Rust, we can use the same C shim approach as the Zephyr examples:

```c
// smoltcp_shim.c
#include <zenoh-pico.h>

// Global smoltcp state (managed by Rust, called via FFI)
extern void smoltcp_init(void);
extern int smoltcp_socket_open(const char *ip, uint16_t port);
extern int smoltcp_socket_send(int handle, const uint8_t *data, size_t len);
extern int smoltcp_socket_recv(int handle, uint8_t *data, size_t len);
extern void smoltcp_socket_close(int handle);
extern void smoltcp_poll(void);

// zenoh-pico platform implementation calls these Rust functions
z_result_t _z_open_tcp(_z_sys_net_socket_t *sock, ...) {
    sock->_handle = smoltcp_socket_open(ip, port);
    return sock->_handle >= 0 ? _Z_RES_OK : _Z_ERR_GENERIC;
}
```

This approach keeps the complex smoltcp code in Rust while the zenoh-pico platform layer is simple C.

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Memory fragmentation | Use arena allocator for zenoh-pico heap |
| Timing issues (missed packets) | Aggressive polling in blocking ops |
| Complex FFI boundary | Thorough testing, minimize unsafe code |
| C cross-compilation | Document toolchain setup clearly |

## References

- [smoltcp documentation](https://docs.rs/smoltcp)
- [stm32-eth crate](https://github.com/stm32-rs/stm32-eth)
- [zenoh-pico platform layer](https://github.com/eclipse-zenoh/zenoh-pico/tree/main/src/system)
- [embedded-alloc](https://github.com/rust-embedded/embedded-alloc)
