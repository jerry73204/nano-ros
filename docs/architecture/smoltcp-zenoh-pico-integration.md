# smoltcp + zenoh-pico Integration Design

This document describes how to integrate smoltcp (pure-Rust TCP/IP stack) with zenoh-pico (C zenoh client) for bare-metal RTIC/polling embedded systems.

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

### 1. Memory Management (Required)

```c
void *z_malloc(size_t size);
void *z_realloc(void *ptr, size_t size);
void z_free(void *ptr);
```

**Implementation Options:**
- Use `embedded-alloc` crate with static heap
- Use custom arena allocator
- Link to FreeRTOS heap (if using FreeRTOS)

### 2. Random Number Generation (Required)

```c
uint8_t z_random_u8(void);
uint16_t z_random_u16(void);
uint32_t z_random_u32(void);
uint64_t z_random_u64(void);
void z_random_fill(void *buf, size_t len);
```

**Implementation:**
- Use STM32 hardware RNG (RNG peripheral)
- Or seeded PRNG (xorshift) for deterministic testing

### 3. Time & Clock (Required)

```c
z_clock_t z_clock_now(void);
unsigned long z_clock_elapsed_us(z_clock_t *time);
unsigned long z_clock_elapsed_ms(z_clock_t *time);
unsigned long z_clock_elapsed_s(z_clock_t *time);

z_time_t z_time_now(void);
unsigned long z_time_elapsed_us(z_time_t *time);
unsigned long z_time_elapsed_ms(z_time_t *time);
unsigned long z_time_elapsed_s(z_time_t *time);
```

**Implementation:**
- Use DWT cycle counter (Cortex-M debug unit)
- Or SysTick / hardware timer
- For RTIC: Use RTIC monotonic

### 4. Sleep (Required)

```c
z_result_t z_sleep_us(size_t time);
z_result_t z_sleep_ms(size_t time);
z_result_t z_sleep_s(size_t time);
```

**Implementation:**
- Busy-wait loop with cycle counter
- Or cooperative yield (in RTIC async tasks)

### 5. Threading (Can be disabled)

With `Z_FEATURE_MULTI_THREAD=0`, threading is disabled. All threading functions become no-ops:

```c
// These become no-ops or dummy implementations
z_result_t _z_mutex_init(_z_mutex_t *m);
z_result_t _z_mutex_lock(_z_mutex_t *m);
z_result_t _z_mutex_unlock(_z_mutex_t *m);
// ... etc
```

### 6. Network Sockets (Required for TCP)

This is the critical integration point with smoltcp:

```c
// Type definitions
typedef struct {
    uint8_t socket_handle;  // Index into smoltcp socket set
    bool connected;
} _z_sys_net_socket_t;

typedef struct {
    uint8_t ip[4];          // IPv4 address
    uint16_t port;
} _z_sys_net_endpoint_t;

// Functions to implement
z_result_t _z_create_endpoint_tcp(_z_sys_net_endpoint_t *ep,
                                   const char *s_address,
                                   const char *s_port);
void _z_free_endpoint_tcp(_z_sys_net_endpoint_t *ep);

z_result_t _z_open_tcp(_z_sys_net_socket_t *sock,
                       const _z_sys_net_endpoint_t rep,
                       uint32_t tout);
z_result_t _z_listen_tcp(_z_sys_net_socket_t *sock,
                         const _z_sys_net_endpoint_t rep);
void _z_close_tcp(_z_sys_net_socket_t *sock);

size_t _z_read_tcp(const _z_sys_net_socket_t sock,
                   uint8_t *ptr, size_t len);
size_t _z_read_exact_tcp(const _z_sys_net_socket_t sock,
                         uint8_t *ptr, size_t len);
size_t _z_send_tcp(const _z_sys_net_socket_t sock,
                   const uint8_t *ptr, size_t len);
```

## smoltcp Integration Details

### Global State

Since zenoh-pico expects a C-style socket API, we need global state to manage smoltcp:

```rust
// Global smoltcp state (in Rust, exposed via FFI)
static mut SMOLTCP_INTERFACE: Option<Interface> = None;
static mut SMOLTCP_DEVICE: Option<EthernetDevice> = None;
static mut SMOLTCP_SOCKETS: Option<SocketSet> = None;

// Socket handle mapping
const MAX_SOCKETS: usize = 4;
static mut SOCKET_HANDLES: [Option<SocketHandle>; MAX_SOCKETS] = [None; MAX_SOCKETS];
```

### Socket Buffer Allocation

```rust
// Static buffers for TCP sockets
const SOCKET_RX_SIZE: usize = 2048;
const SOCKET_TX_SIZE: usize = 2048;

static mut SOCKET_RX_BUFFERS: [[u8; SOCKET_RX_SIZE]; MAX_SOCKETS] = [[0; SOCKET_RX_SIZE]; MAX_SOCKETS];
static mut SOCKET_TX_BUFFERS: [[u8; SOCKET_TX_SIZE]; MAX_SOCKETS] = [[0; SOCKET_TX_SIZE]; MAX_SOCKETS];
```

### Blocking Socket Operations

zenoh-pico expects blocking socket operations, but smoltcp is polling-based. Solution: polling loops with timeout.

```rust
/// Blocking read with timeout
#[no_mangle]
pub extern "C" fn _z_read_tcp(
    sock: _z_sys_net_socket_t,
    ptr: *mut u8,
    len: usize,
) -> usize {
    let start = clock_now();
    let timeout_ms = 100; // Z_CONFIG_SOCKET_TIMEOUT

    loop {
        // Poll smoltcp interface
        poll_interface();

        // Try to receive data
        if let Some(n) = try_recv(sock.socket_handle, ptr, len) {
            return n;
        }

        // Check timeout
        if clock_elapsed_ms(start) > timeout_ms {
            return SIZE_MAX; // Error: timeout
        }

        // Check connection state
        if !is_socket_connected(sock.socket_handle) {
            return SIZE_MAX; // Error: disconnected
        }
    }
}

/// Blocking write
#[no_mangle]
pub extern "C" fn _z_send_tcp(
    sock: _z_sys_net_socket_t,
    ptr: *const u8,
    len: usize,
) -> usize {
    let start = clock_now();
    let timeout_ms = 100;
    let mut sent = 0;

    while sent < len {
        // Poll smoltcp interface
        poll_interface();

        // Try to send data
        if let Some(n) = try_send(sock.socket_handle, ptr.add(sent), len - sent) {
            sent += n;
        }

        // Check timeout
        if clock_elapsed_ms(start) > timeout_ms {
            break;
        }
    }

    sent
}
```

### Interface Polling Integration

The key insight: **smoltcp's `Interface::poll()` must be called regularly** for TCP to work. Options:

#### Option A: Poll Inside Socket Operations

Poll the interface within `_z_read_tcp` and `_z_send_tcp`:

```rust
fn poll_interface() {
    unsafe {
        if let (Some(iface), Some(device), Some(sockets)) =
            (&mut SMOLTCP_INTERFACE, &mut SMOLTCP_DEVICE, &mut SMOLTCP_SOCKETS)
        {
            iface.poll(smoltcp::time::Instant::now(), device, sockets);
        }
    }
}
```

**Pros:** Self-contained, no application changes needed
**Cons:** May miss packets between zenoh calls

#### Option B: Poll from Application Main Loop

Application explicitly polls smoltcp in main loop:

```rust
// In main polling loop
loop {
    // 1. Poll smoltcp
    smoltcp_poll();

    // 2. Poll zenoh via nano-ros
    executor.spin_once(10);

    // 3. Application logic
    publish_if_needed();
}
```

**Pros:** Application has full control
**Cons:** More complex integration

#### Option C: Combined (Recommended)

Poll both in socket operations AND in main loop:

```rust
// Exported function for main loop
#[no_mangle]
pub extern "C" fn smoltcp_poll() {
    poll_interface();
}

// Also poll within blocking operations
fn _z_read_tcp(...) {
    loop {
        poll_interface();  // Ensure we process incoming packets
        // ... try_recv ...
    }
}
```

## Memory Requirements

### Static Allocation Budget

| Component | Size | Notes |
|-----------|------|-------|
| smoltcp Interface | ~1 KB | Configuration dependent |
| Ethernet Device | ~8 KB | DMA buffers (4 RX + 4 TX) |
| Socket Set | ~100 B | Metadata only |
| TCP Socket RX | 2 KB × N | Per socket |
| TCP Socket TX | 2 KB × N | Per socket |
| zenoh-pico heap | ~16 KB | Session + publishers + subscribers |

**Total for 2 sockets:** ~30-40 KB

### Heap for zenoh-pico

zenoh-pico uses malloc extensively. Recommended: `embedded-alloc` with static heap:

```rust
use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

// In init
static mut HEAP_MEM: [MaybeUninit<u8>; 16384] = [MaybeUninit::uninit(); 16384];
unsafe {
    HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_MEM.len());
}
```

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

# zenoh-pico bindings (with smoltcp platform)
zenoh-pico-sys = { path = "../../crates/zenoh-pico-sys", features = ["smoltcp"] }
```

## Implementation Steps

### Phase 1: smoltcp Standalone Test

1. Create minimal RTIC example with smoltcp + stm32-eth
2. Implement Device trait for STM32 Ethernet
3. Test basic TCP echo server
4. Verify memory usage and timing

### Phase 2: Platform Layer Skeleton

1. Create `crates/zenoh-pico-sys/src/platform/smoltcp/` directory
2. Implement memory functions (use embedded-alloc)
3. Implement random functions (use hardware RNG or PRNG)
4. Implement time functions (use DWT or SysTick)
5. Stub out socket functions (return errors)
6. Test compilation

### Phase 3: Socket Integration

1. Implement socket type definitions
2. Implement `_z_create_endpoint_tcp` (parse IP:port)
3. Implement `_z_open_tcp` (create smoltcp socket, connect)
4. Implement `_z_read_tcp` and `_z_send_tcp` (blocking wrappers)
5. Implement `_z_close_tcp`
6. Test with zenoh router

### Phase 4: nano-ros Integration

1. Create `SmoltcpTransport` in nano-ros-transport
2. Update rtic-stm32f4 example to use real transport
3. Test publisher with native listener
4. Test subscriber with native publisher

### Phase 5: Polish and Document

1. Optimize memory usage
2. Add error handling
3. Document API and usage
4. Add to CI (if feasible)

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Memory fragmentation | Use arena allocator for zenoh-pico heap |
| Timing issues (missed packets) | Aggressive polling in blocking ops |
| Complex FFI boundary | Thorough testing, minimize unsafe |
| C cross-compilation | Document toolchain setup clearly |

## Alternative: C Shim Like Zephyr

Instead of implementing the full platform layer in Rust, we could use the same C shim approach as Zephyr:

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

// zenoh-pico platform implementation
z_result_t _z_open_tcp(_z_sys_net_socket_t *sock, ...) {
    sock->_handle = smoltcp_socket_open(ip, port);
    return sock->_handle >= 0 ? _Z_RES_OK : _Z_ERR_GENERIC;
}

size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    // Poll smoltcp in blocking loop
    while (true) {
        smoltcp_poll();
        int n = smoltcp_socket_recv(sock._handle, ptr, len);
        if (n > 0) return n;
        if (n < 0) return SIZE_MAX;
        // timeout check...
    }
}
```

This approach keeps the complex smoltcp code in Rust while the zenoh-pico platform layer is simple C.

## Conclusion

The smoltcp + zenoh-pico integration is feasible with moderate effort. The key challenges are:

1. **Blocking/polling mismatch**: Solved with polling loops inside socket operations
2. **Memory management**: Solved with embedded-alloc static heap
3. **Cross-compilation**: Requires arm-none-eabi-gcc toolchain
4. **Global state**: Managed via static variables with careful synchronization

The recommended approach is the C shim pattern (like Zephyr) as it provides the cleanest separation of concerns and leverages existing patterns in the codebase.
