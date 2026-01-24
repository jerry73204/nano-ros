# Phase 8: Embedded Networking

**Goal**: Enable network connectivity for all embedded nano-ros examples using smoltcp + zenoh-pico for bare-metal RTIC/polling, and verify native examples work correctly.

**Status**: In Progress (Phase 8.1-8.7 Complete, Phase 8.8 Pending)

## Overview

This phase provides working network connectivity for embedded systems:
- **Zephyr examples** (COMPLETE) - Using zenoh-pico via C shim
- **zenoh-pico-shim crate** - Unified crate with C shim + platform backends
- **RTIC/polling examples** - Using zenoh-pico-shim with smoltcp platform layer
- **Native examples** - Verify executor API integration works

**Architecture Documents:**
- [smoltcp-zenoh-pico-integration.md](../architecture/smoltcp-zenoh-pico-integration.md) - Network stack design
- [modular-c-shim-design.md](../architecture/modular-c-shim-design.md) - zenoh-pico-shim architecture (Option C)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│ Application Layer                                                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────┐       ┌────────────────────────────────┐  │
│  │  Native/std              │       │  Embedded (RTIC/polling/Zephyr)│  │
│  │  (posix feature)         │       │  (smoltcp/zephyr feature)      │  │
│  └────────────┬─────────────┘       └───────────────┬────────────────┘  │
│               │                                     │                    │
│               └─────────────────┬───────────────────┘                    │
│                                 │                                        │
│                                 ▼                                        │
│               ┌────────────────────────────────────────┐                 │
│               │  zenoh-pico-shim (High-level Rust API) │                 │
│               │  ├── Session, Publisher, Subscriber    │                 │
│               │  └── platform/smoltcp.rs (Rust FFI)    │                 │
│               └────────────────────┬───────────────────┘                 │
│                                    │                                     │
│                                    ▼                                     │
│               ┌────────────────────────────────────────┐                 │
│               │  zenoh-pico-shim-sys (FFI + C code)    │                 │
│               │  ├── c/shim/zenoh_shim.c               │                 │
│               │  ├── c/platform_smoltcp/*.c (optional) │                 │
│               │  └── zenoh-pico/ (submodule)           │                 │
│               └────────────────────┬───────────────────┘                 │
│                                    │                                     │
│                                    ▼                                     │
│               ┌────────────────────────────────────────┐                 │
│               │  zenoh-pico C library (submodule)      │                 │
│               │  Platform: POSIX / Zephyr / GENERIC    │                 │
│               └────────────────────────────────────────┘                 │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key Design Decisions:**
- All applications use unified `zenoh-pico-shim` API (replaces old `zenoh-pico` crate)
- `zenoh-pico-shim-sys` contains all C code (shim + platform layers + zenoh-pico submodule)
- Platform selection via feature flags:
  - `posix` - Uses zenoh-pico native POSIX platform (desktop/Linux)
  - `zephyr` - Uses zenoh-pico native Zephyr platform
  - `smoltcp` - Uses custom platform layer for bare-metal
- Clean separation: Rust code in `zenoh-pico-shim`, C code in `zenoh-pico-shim-sys`

---

## Phase 8.1: Zephyr Examples (COMPLETE)

**Status**: Complete

### Work Items

- [x] **8.1.1** Remove C-only Zephyr examples (zephyr-talker, zephyr-listener directories)

- [x] **8.1.2** Rename Rust Zephyr examples
  - `zephyr-talker-rs` → `zephyr-talker`
  - `zephyr-listener-rs` → `zephyr-listener`

- [x] **8.1.3** Create unified C shim supporting multiple pub/sub
  ```c
  int zenoh_init_config(const char *locator);
  int zenoh_open_session(void);
  int zenoh_declare_publisher(const char *keyexpr);  // Returns handle
  int zenoh_publish(int handle, const uint8_t *data, size_t len);
  int zenoh_declare_subscriber(const char *keyexpr, callback_t cb, void *ctx);
  void zenoh_close(void);
  ```

- [x] **8.1.4** Update zephyr-talker to use executor API pattern
  - Create `ZephyrContext`, `ZephyrExecutor`, `ZephyrNode`, `ZephyrPublisher`
  - Mirror native executor API

- [x] **8.1.5** Update zephyr-listener to use executor API pattern
  - Create `ZephyrSubscriber` with callback support

- [x] **8.1.6** Generate message bindings using `cargo nano-ros generate`
  - Create `package.xml` for each example
  - Generate `std_msgs` bindings
  - Create `.cargo/config.toml` with patches

- [x] **8.1.7** Update workspace exclude list for renamed directories

### Acceptance Criteria
- Zephyr examples compile with `west build`
- Examples use executor API pattern (Context → Executor → Node)
- Message bindings generated via `cargo nano-ros generate`

---

## Phase 8.2: zenoh-pico-shim Crate

**Status**: Complete
**Priority**: High

Created the unified `zenoh-pico-shim` crate that provides:
1. High-level C shim API (`zenoh_shim_*`) for simplified FFI
2. Platform backend interface for different execution models (threaded vs polling)
3. Stub backends for smoltcp and Zephyr (full implementation in Phase 8.4)

See [modular-c-shim-design.md](../architecture/modular-c-shim-design.md) for detailed design.

### Work Items

- [x] **8.2.1** Create `crates/zenoh-pico-shim/` directory structure
  ```
  crates/zenoh-pico-shim/
  ├── Cargo.toml
  ├── build.rs
  ├── README.md
  ├── include/
  │   ├── zenoh_shim.h           # Public C API
  │   └── zenoh_shim_platform.h  # Backend interface
  ├── shim/
  │   ├── zenoh_shim.c           # Core shim implementation
  │   ├── backend_zephyr.c       # Zephyr: threaded execution (stub)
  │   ├── backend_smoltcp.c      # smoltcp: polling execution (stub)
  │   └── backend_posix.c        # POSIX: for testing (working)
  └── src/
      └── lib.rs                 # Safe Rust wrapper
  ```

- [x] **8.2.2** Define public C API (`include/zenoh_shim.h`)
  - Session lifecycle: init, open, is_open, close
  - Publishers: declare_publisher, publish, undeclare_publisher
  - Subscribers: declare_subscriber, undeclare_subscriber
  - Polling: poll, spin_once, uses_polling
  - **Header auto-generated by cbindgen** from `src/ffi.rs` stub functions during build

- [x] **8.2.3** Define platform backend interface (`include/zenoh_shim_platform.h`)
  - `zenoh_platform_uses_polling()` - Query execution model
  - `zenoh_platform_start_read_task()` / `zenoh_platform_start_lease_task()` - Threaded backends
  - `zenoh_platform_poll()` - Polling backends
  - `zenoh_platform_time_ms()` / `zenoh_platform_sleep_ms()` - Time functions
  - `zenoh_platform_init()` / `zenoh_platform_cleanup()` - Lifecycle

- [x] **8.2.4** Implement core shim (`shim/zenoh_shim.c`)
  - Session management (init, open, close)
  - Publisher storage (up to MAX_PUBLISHERS=8 handles)
  - Subscriber storage with callbacks (up to MAX_SUBSCRIBERS=8)
  - Platform-agnostic: delegates to backend interface

- [x] **8.2.5** Implement Zephyr backend stub (`shim/backend_zephyr.c`)
  - Basic structure ready for Zephyr integration
  - Will use zp_start_read_task/zp_start_lease_task

- [x] **8.2.6** Implement POSIX backend (`shim/backend_posix.c`)
  - `zenoh_platform_uses_polling()` → false
  - Uses zenoh-pico background tasks (zp_start_read_task/zp_start_lease_task)
  - Uses `gettimeofday()` for time, `usleep()` for sleep
  - Enables desktop testing

- [x] **8.2.7** Implement smoltcp backend stub (`shim/backend_smoltcp.c`)
  - `zenoh_platform_uses_polling()` → true
  - Stub implementation ready for Phase 8.4

- [x] **8.2.8** Create `Cargo.toml` with feature flags
  - Features: `posix`, `zephyr`, `smoltcp`, `std`
  - Each backend enables `zenoh-pico-sys` dependency
  - Build-dependency: `cc = "1.0"`

- [x] **8.2.9** Create `build.rs` with feature-based backend selection
  - Compiles core shim + selected backend
  - Sets platform defines (ZENOH_LINUX, etc.)
  - Gracefully handles no-backend builds

- [x] **8.2.10** Create safe Rust wrapper (`src/lib.rs`)
  - `ShimContext`, `ShimPublisher`, `ShimSubscriber` types
  - `ShimError` enum with error codes
  - `no_std` compatible with optional std support
  - Safe API with proper lifetime management

- [x] **8.2.11** Migrate zephyr-talker to use `zenoh-pico-shim` crate
  - Completed in Phase 8.5.9

- [x] **8.2.12** Migrate zephyr-listener to use `zenoh-pico-shim` crate
  - Completed in Phase 8.5.9

- [ ] **8.2.13** Test Zephyr examples still work after migration
  - Deferred to Phase 8.6 (requires Zephyr hardware/QEMU)

### Acceptance Criteria
- [x] `zenoh-pico-shim` crate compiles with `posix` feature
- [x] POSIX backend enables desktop testing (tests pass)
- [x] Directory structure ready for smoltcp platform layer (Phase 8.4)
- [x] `no_std` compatible (builds for thumbv7em-none-eabihf without backend)
- [x] Zephyr examples migrated (completed in Phase 8.5.9)

---

## Phase 8.3: smoltcp Standalone Validation

**Status**: Complete
**Priority**: High

Validate smoltcp + stm32-eth works on target hardware before integrating with zenoh-pico.

### Work Items

- [x] **8.3.1** Create `examples/smoltcp-test/` standalone example
  - RTIC 2.x example with smoltcp 0.12 + stm32-eth 0.8
  - TCP echo server on port 7
  - Compiles for thumbv7em-none-eabihf

- [x] **8.3.2** Use stm32-eth Device trait implementation
  - stm32-eth provides `impl Device for &mut EthernetDMA` via `smoltcp-phy` feature
  - No custom Device implementation needed

- [x] **8.3.3** Set up static buffer allocation for smoltcp
  ```rust
  const RX_DESC_COUNT: usize = 4;
  const TX_DESC_COUNT: usize = 4;
  const TCP_RX_BUFFER_SIZE: usize = 2048;
  const TCP_TX_BUFFER_SIZE: usize = 2048;
  const MAX_SOCKETS: usize = 2;

  #[link_section = ".ethram"]
  static mut RX_RING: [RxRingEntry; RX_DESC_COUNT] = ...;
  #[link_section = ".ethram"]
  static mut TX_RING: [TxRingEntry; TX_DESC_COUNT] = ...;
  static mut TCP_RX_BUFFER: [u8; TCP_RX_BUFFER_SIZE] = ...;
  static mut TCP_TX_BUFFER: [u8; TCP_TX_BUFFER_SIZE] = ...;
  ```

- [x] **8.3.4** Implement polling loop with RTIC async task
  ```rust
  #[task(shared = [eth_dma, iface, sockets], priority = 1)]
  async fn poll_network(mut cx: poll_network::Context) {
      loop {
          let timestamp = Instant::from_millis(Mono::now().ticks() as i64);
          (&mut cx.shared.eth_dma, &mut cx.shared.iface, &mut cx.shared.sockets)
              .lock(|eth_dma, iface, sockets| {
                  iface.poll(timestamp, &mut eth_dma, sockets);
                  // Handle TCP echo
              });
          Mono::delay(10.millis()).await;
      }
  }
  ```

- [ ] **8.3.5** Test TCP connection to host PC (requires hardware)
  - Build verified, hardware testing pending
  - Instructions provided in README

### Binary Size

| Section | Size | Notes |
|---------|------|-------|
| text | 54,896 bytes | Code + constants |
| data | 728 bytes | Initialized data |
| bss | 18,292 bytes | Static buffers (DMA, TCP, sockets) |
| **Total** | 73,916 bytes | Fits STM32F429 easily |

### Dependencies Used
- stm32f4xx-hal 0.21 (required by stm32-eth 0.8)
- stm32-eth 0.8 with `smoltcp-phy` feature
- smoltcp 0.12
- RTIC 2.1 + rtic-monotonics 2.0

### Acceptance Criteria
- [x] Example compiles for thumbv7em-none-eabihf
- [x] Binary size documented (~74 KB total, <40 KB RAM used)
- [ ] TCP echo server works on NUCLEO-F429ZI (hardware test pending)

---

## Phase 8.4: smoltcp Platform Layer in zenoh-pico-shim

**Status**: Core Implementation Complete (Hardware Testing Pending)
**Priority**: High
**Depends on**: 8.2, 8.3

Implement the zenoh-pico platform abstraction (`z_*` functions) using smoltcp within the `zenoh-pico-shim` crate. This provides the low-level socket and system operations needed by zenoh-pico on bare-metal.

**Note:** The platform layer lives in `zenoh-pico-shim/platform_smoltcp/`, not in `zenoh-pico-sys`. This keeps `zenoh-pico-sys` as pure FFI bindings while `zenoh-pico-shim` owns all embedded-specific code.

### Implementation Architecture

The smoltcp platform layer uses callback-based integration:
1. Application provides a `poll_callback` function that drives smoltcp
2. Socket buffers are managed in Rust with C-callable push/pop functions
3. Clock is updated externally via `smoltcp_set_clock_ms()`
4. Memory uses a simple bump allocator (16KB static heap)

### Work Items

- [x] **8.4.1** Implement memory management (`platform_smoltcp/system.c`)
  ```c
  // Uses embedded-alloc with static heap (~16KB)
  void *z_malloc(size_t size);
  void *z_realloc(void *ptr, size_t size);
  void z_free(void *ptr);
  ```
  Rust side:
  ```rust
  // platform_smoltcp/mod.rs
  use embedded_alloc::LlffHeap;

  #[global_allocator]
  static HEAP: LlffHeap = LlffHeap::empty();

  static mut HEAP_MEM: [MaybeUninit<u8>; 16384] = [MaybeUninit::uninit(); 16384];
  ```

- [x] **8.4.2** Implement random number generation (`platform_smoltcp/system.c`)
  ```c
  uint8_t z_random_u8(void);
  uint16_t z_random_u16(void);
  uint32_t z_random_u32(void);
  uint64_t z_random_u64(void);
  void z_random_fill(void *buf, size_t len);
  ```
  - Use STM32 hardware RNG peripheral or seeded PRNG

- [x] **8.4.3** Implement time/clock functions (`platform_smoltcp/system.c`)
  ```c
  z_clock_t z_clock_now(void);
  unsigned long z_clock_elapsed_us(z_clock_t *time);
  unsigned long z_clock_elapsed_ms(z_clock_t *time);
  z_result_t z_sleep_us(size_t time);
  z_result_t z_sleep_ms(size_t time);
  ```
  - Use DWT cycle counter (Cortex-M debug unit) for timing
  - `z_sleep_*` busy-waits or yields to smoltcp polling

- [x] **8.4.4** Implement threading stubs (`platform_smoltcp/system.c`)
  ```c
  // With Z_FEATURE_MULTI_THREAD=0, these become no-ops
  z_result_t _z_mutex_init(_z_mutex_t *m) { return _Z_RES_OK; }
  z_result_t _z_mutex_lock(_z_mutex_t *m) { return _Z_RES_OK; }
  z_result_t _z_mutex_unlock(_z_mutex_t *m) { return _Z_RES_OK; }
  z_result_t _z_condvar_init(_z_condvar_t *cv) { return _Z_RES_OK; }
  // etc.
  ```

- [x] **8.4.5** Define socket type structures (`zenoh_smoltcp_platform.h`)
  ```c
  typedef struct {
      uint8_t socket_handle;  // Index into smoltcp socket set
      bool connected;
  } _z_sys_net_socket_t;

  typedef struct {
      uint8_t ip[4];          // IPv4 address
      uint16_t port;
  } _z_sys_net_endpoint_t;
  ```

- [x] **8.4.6** Implement `_z_create_endpoint_tcp` (`platform_smoltcp/network.c`)
  ```c
  z_result_t _z_create_endpoint_tcp(
      _z_sys_net_endpoint_t *ep,
      const char *s_address,
      const char *s_port
  );
  ```
  - Parse IP string "192.168.1.1" into bytes
  - Parse port string into uint16_t

- [x] **8.4.7** Implement `_z_open_tcp` (`platform_smoltcp/network.c`)
  ```c
  z_result_t _z_open_tcp(
      _z_sys_net_socket_t *sock,
      const _z_sys_net_endpoint_t rep,
      uint32_t tout
  );
  ```
  - Call Rust FFI to allocate smoltcp TcpSocket
  - Initiate connection
  - Polling loop until connected or timeout

- [x] **8.4.8** Implement `_z_read_tcp` with blocking wrapper
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

- [x] **8.4.9** Implement `_z_send_tcp` with blocking wrapper
  ```c
  size_t _z_send_tcp(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
      size_t sent = 0;
      z_clock_t start = z_clock_now();
      while (sent < len) {
          smoltcp_poll();  // Process packets
          size_t n = smoltcp_try_send(sock.socket_handle, ptr + sent, len - sent);
          sent += n;
          if (z_clock_elapsed_ms(&start) > Z_CONFIG_SOCKET_TIMEOUT) break;
      }
      return sent;
  }
  ```

- [x] **8.4.10** Implement `_z_close_tcp`
  ```c
  void _z_close_tcp(_z_sys_net_socket_t *sock) {
      smoltcp_socket_close(sock->socket_handle);
      sock->connected = false;
  }
  ```

- [x] **8.4.11** Create Rust FFI module for smoltcp operations (`platform_smoltcp/mod.rs`)
  ```rust
  // Global smoltcp state
  static mut SMOLTCP_INTERFACE: Option<Interface> = None;
  static mut SMOLTCP_DEVICE: Option<EthernetDevice> = None;
  static mut SMOLTCP_SOCKETS: Option<SocketSet> = None;

  #[no_mangle]
  pub extern "C" fn smoltcp_init(
      ip: *const u8,
      gateway: *const u8,
      mac: *const u8,
  ) -> i32 { ... }

  #[no_mangle]
  pub extern "C" fn smoltcp_poll() -> i32 {
      unsafe {
          if let (Some(iface), Some(device), Some(sockets)) =
              (&mut SMOLTCP_INTERFACE, &mut SMOLTCP_DEVICE, &mut SMOLTCP_SOCKETS)
          {
              iface.poll(Instant::now(), device, sockets);
          }
      }
      0
  }

  #[no_mangle]
  pub extern "C" fn smoltcp_socket_open() -> i32 { ... }

  #[no_mangle]
  pub extern "C" fn smoltcp_socket_connect(handle: u8, ip: *const u8, port: u16) -> i32 { ... }

  #[no_mangle]
  pub extern "C" fn smoltcp_try_recv(handle: u8, buf: *mut u8, len: usize) -> usize { ... }

  #[no_mangle]
  pub extern "C" fn smoltcp_try_send(handle: u8, buf: *const u8, len: usize) -> usize { ... }

  #[no_mangle]
  pub extern "C" fn smoltcp_socket_close(handle: u8) { ... }
  ```

- [x] **8.4.12** Complete smoltcp backend (`shim/backend_smoltcp.c`)
  ```c
  bool zenoh_platform_uses_polling(void) { return true; }

  int zenoh_platform_poll(uint32_t timeout_ms) {
      return smoltcp_poll();  // Calls Rust FFI
  }

  uint64_t zenoh_platform_time_ms(void) {
      return dwt_get_ms();  // Calls Rust FFI for DWT counter
  }

  void zenoh_platform_sleep_ms(uint32_t ms) {
      // Busy wait while polling smoltcp
      uint64_t start = zenoh_platform_time_ms();
      while (zenoh_platform_time_ms() - start < ms) {
          smoltcp_poll();
      }
  }
  ```

- [x] **8.4.13** Update zenoh-pico-shim `build.rs` for smoltcp platform layer
  ```rust
  if cfg!(feature = "smoltcp") {
      // Backend (calls smoltcp FFI)
      build.file("shim/backend_smoltcp.c");

      // Platform layer (implements z_* functions)
      build.file("platform_smoltcp/system.c");
      build.file("platform_smoltcp/network.c");

      // ARM cross-compilation flags
      if target.contains("thumbv7em") {
          build.flag("-mcpu=cortex-m4")
               .flag("-mthumb")
               .flag("-mfpu=fpv4-sp-d16")
               .flag("-mfloat-abi=hard");
      }
  }
  ```

- [ ] **8.4.14** Configure zenoh-pico CMake for bare-metal (deferred)
  - Note: Requires cross-compiling zenoh-pico library separately
  - The platform layer works with pre-built zenoh-pico library

- [x] **8.4.15** Test smoltcp platform layer compiles for x86_64-linux
  ```bash
  cargo check --features smoltcp  # Works!
  ```
  - Note: thumbv7em target requires cross-compiled zenoh-pico library

### Implementation Notes

**Files Created:**
- `platform_smoltcp/zenoh_smoltcp_platform.h` - Platform type definitions
- `platform_smoltcp/zenoh_generic_platform.h` - zenoh-pico include redirect
- `platform_smoltcp/zenoh_generic_config.h` - zenoh-pico feature configuration
- `platform_smoltcp/system.c` - System functions (memory, random, clock, threading)
- `platform_smoltcp/network.c` - TCP socket operations
- `platform_smoltcp/mod.rs` - Rust FFI module for smoltcp integration

**Integration Pattern:**
The application must provide:
1. A `poll_callback` function that drives smoltcp interface
2. Periodic calls to `smoltcp_set_clock_ms()` for timekeeping
3. Integration code that moves data between smoltcp sockets and the shim's buffers

### Acceptance Criteria
- [x] `zenoh-pico-shim` compiles with `smoltcp` feature for x86_64-linux
- [x] Platform layer implements all required `z_*` functions
- [x] smoltcp FFI functions callable from C platform code
- [ ] Can create zenoh session using smoltcp as transport (requires hardware test)
- [ ] Cross-compilation for thumbv7em (requires zenoh-pico cross-build)

---

## Phase 8.4.5: Library Restructure (zenoh-pico-shim Refactor)

**Status**: Partially Complete (Core restructure done, transport migration pending)
**Priority**: High
**Depends on**: 8.4

Restructure the zenoh-pico related crates to cleanly separate Rust and C code, and simplify the dependency graph.

### Motivation

The current structure mixes Rust and C code in single crates:
- `zenoh-pico-sys` - FFI bindings + builds zenoh-pico
- `zenoh-pico` - Safe Rust wrapper
- `zenoh-pico-shim` - C shim + Rust FFI + platform layers

The new structure separates concerns:
- `zenoh-pico-shim-sys` - All C code + FFI bindings (replaces zenoh-pico-sys)
- `zenoh-pico-shim` - High-level Rust API (replaces zenoh-pico)

### New Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        zenoh-pico-shim                              │
│                     (High-level Rust API)                           │
│                                                                     │
│  - Safe wrappers (Session, Publisher, Subscriber)                   │
│  - Platform Rust code (smoltcp_* implementations)                   │
│  - no_std compatible                                                │
└─────────────────────────────────────────────────────────────────────┘
                                 │
                    depends on   │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      zenoh-pico-shim-sys                            │
│                (FFI bindings + all C code)                          │
│                                                                     │
│  - zenoh-pico submodule (upstream C library)                        │
│  - zenoh_shim.c/h (C shim API)                                      │
│  - Platform C code (system.c, network.c for smoltcp)                │
│  - extern "C" FFI declarations                                      │
└─────────────────────────────────────────────────────────────────────┘
                                 │
                    links        │
                                 ▼
                    zenoh-pico C library (submodule)
```

### Directory Structure

```
crates/
├── zenoh-pico-shim/                    # High-level Rust API
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs                      # Public API, re-exports
│       ├── config.rs                   # Configuration types
│       ├── session.rs                  # Session management
│       ├── publisher.rs                # Publisher
│       ├── subscriber.rs               # Subscriber
│       ├── error.rs                    # Error types
│       └── platform/
│           ├── mod.rs
│           └── smoltcp.rs              # smoltcp_* Rust FFI implementations
│
├── zenoh-pico-shim-sys/                # FFI + all C code
│   ├── Cargo.toml
│   ├── build.rs                        # Compiles zenoh-pico + shim + platform
│   ├── cbindgen.toml                   # Header generation config
│   ├── src/
│   │   └── lib.rs                      # FFI declarations only
│   ├── zenoh-pico/                     # Git submodule (upstream)
│   └── c/
│       ├── shim/
│       │   ├── zenoh_shim.h            # Public C API
│       │   └── zenoh_shim.c            # Core shim implementation
│       └── platform_smoltcp/
│           ├── zenoh_generic_platform.h
│           ├── zenoh_generic_config.h
│           ├── zenoh_smoltcp_platform.h
│           ├── system.c                # z_malloc, z_clock, etc.
│           └── network.c               # _z_open_tcp, _z_read_tcp, etc.
│
└── (REMOVE: zenoh-pico-sys/, zenoh-pico/)
```

### Feature Flags

```toml
# zenoh-pico-shim-sys/Cargo.toml
[features]
default = []
std = []

# Platform selection (mutually exclusive)
posix = []      # Uses zenoh-pico native POSIX platform (ZENOH_LINUX/ZENOH_MACOS)
zephyr = []     # Uses zenoh-pico native Zephyr platform (ZENOH_ZEPHYR)
smoltcp = []    # Uses custom platform layer (ZENOH_GENERIC + system.c/network.c)

# zenoh-pico-shim/Cargo.toml
[features]
default = []
std = ["zenoh-pico-shim-sys/std"]
posix = ["zenoh-pico-shim-sys/posix"]
zephyr = ["zenoh-pico-shim-sys/zephyr"]
smoltcp = ["zenoh-pico-shim-sys/smoltcp"]
```

### Work Items

- [x] **8.4.5.1** Create `zenoh-pico-shim-sys` crate structure
  ```
  crates/zenoh-pico-shim-sys/
  ├── Cargo.toml
  ├── build.rs
  ├── cbindgen.toml
  ├── src/lib.rs
  └── c/
  ```

- [x] **8.4.5.2** Move zenoh-pico submodule to `zenoh-pico-shim-sys/zenoh-pico/`
  ```bash
  git mv crates/zenoh-pico-sys/zenoh-pico crates/zenoh-pico-shim-sys/zenoh-pico
  ```

- [x] **8.4.5.3** Move C shim code to `zenoh-pico-shim-sys/c/shim/`
  - `zenoh_shim.h` → `c/include/zenoh_shim.h` (auto-generated)
  - `zenoh_shim.c` → `c/shim/zenoh_shim.c`

- [x] **8.4.5.4** Move platform C code to `zenoh-pico-shim-sys/c/platform_smoltcp/`
  - `system.c`, `network.c`, headers

- [x] **8.4.5.5** Create `zenoh-pico-shim-sys/build.rs`
  - Compile zenoh-pico with appropriate platform defines
  - Compile shim C code
  - Compile platform C code (when smoltcp feature enabled)
  - Feature-based platform selection

- [x] **8.4.5.6** Create `zenoh-pico-shim-sys/src/lib.rs`
  - FFI declarations only (`extern "C" { ... }`)
  - No business logic
  - Constants and types re-exported from cbindgen

- [x] **8.4.5.7** Restructure `zenoh-pico-shim` as high-level Rust API
  - `src/lib.rs` - Public API and re-exports from sys crate
  - ShimContext, ShimPublisher, ShimSubscriber wrappers
  - Error types

- [x] **8.4.5.8** Move platform Rust code to `zenoh-pico-shim-sys/src/platform_smoltcp.rs`
  - smoltcp_* implementations in sys crate
  - Re-exported from zenoh-pico-shim

- [ ] **8.4.5.9** Remove old crates (DEFERRED → 8.8.6)
  - Old crates kept for backward compatibility with nano-ros-transport
  - `zenoh-pico-sys/zenoh-pico` symlinked to new location
  - Tracked in Phase 8.8.6 (after transport migration in 8.5.8)

- [x] **8.4.5.10** Update workspace `Cargo.toml`
  - Added `zenoh-pico-shim-sys` to members
  - Kept legacy crates with comments (for transport compatibility)

- [ ] **8.4.5.11** Update dependent crates (DEFERRED → 8.5.8)
  - `nano-ros-transport` still uses legacy `zenoh-pico` crate
  - Migration requires adding LivelinessToken, ZenohId, Attachment to shim API
  - Tracked in Phase 8.5.8 (transport migration task)

- [x] **8.4.5.12** Test build with all platform features
  ```bash
  cargo check -p zenoh-pico-shim-sys --features posix  # ✓
  cargo check -p zenoh-pico-shim --features posix      # ✓
  # smoltcp feature builds but requires hardware test
  ```

- [ ] **8.4.5.13** Update CLAUDE.md documentation
  - Update crate descriptions
  - Update dependency graph
  - Update build instructions

### Build Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                    zenoh-pico-shim-sys/build.rs                     │
└─────────────────────────────────────────────────────────────────────┘
                                 │
         ┌───────────────────────┼───────────────────────┐
         ▼                       ▼                       ▼
    [posix]                 [zephyr]                [smoltcp]
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐  ┌─────────────────┐  ┌──────────────────────────┐
│ cc::Build       │  │ cc::Build       │  │ cc::Build                │
│                 │  │                 │  │                          │
│ zenoh-pico/*    │  │ zenoh-pico/*    │  │ zenoh-pico/* (GENERIC)   │
│ (ZENOH_LINUX)   │  │ (ZENOH_ZEPHYR)  │  │ c/platform_smoltcp/*.c   │
│                 │  │                 │  │                          │
│ c/shim/*.c      │  │ c/shim/*.c      │  │ c/shim/*.c               │
└─────────────────┘  └─────────────────┘  └──────────────────────────┘
         │                       │                       │
         └───────────────────────┴───────────────────────┘
                                 │
                                 ▼
                    libzenoh_pico_shim.a (static library)
```

### Link-time Resolution (smoltcp)

For smoltcp platform, C code declares extern functions satisfied by Rust at link time:

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Final Binary                                  │
└─────────────────────────────────────────────────────────────────────┘
                                 │
              ┌──────────────────┴──────────────────┐
              ▼                                     ▼
┌──────────────────────────┐          ┌──────────────────────────────┐
│  libzenoh_pico_shim.a    │          │  zenoh-pico-shim (Rust)      │
│  (from -sys crate)       │          │                              │
│                          │          │  src/platform/smoltcp.rs:    │
│  system.c:               │   ───►   │    #[no_mangle]              │
│    extern smoltcp_alloc  │  links   │    pub extern "C" fn         │
│    extern smoltcp_poll   │    to    │      smoltcp_alloc() {...}   │
└──────────────────────────┘          └──────────────────────────────┘
```

### Acceptance Criteria
- [ ] `zenoh-pico-shim-sys` compiles with `posix` feature
- [ ] `zenoh-pico-shim-sys` compiles with `smoltcp` feature
- [ ] `zenoh-pico-shim` compiles with all platform features
- [ ] Old crates (`zenoh-pico-sys`, `zenoh-pico`) removed
- [ ] No Rust/C mixing within single crate directories
- [ ] All existing functionality preserved

---

## Phase 8.5: nano-ros-node Integration with zenoh-pico-shim

**Status**: Complete
**Priority**: High
**Depends on**: 8.4.5

Integrate `zenoh-pico-shim` with `nano-ros-node` to enable embedded nano-ros applications.

### Work Items

- [x] **8.5.1** Add `shim` feature to nano-ros-node
  ```toml
  # crates/nano-ros-node/Cargo.toml
  [features]
  shim = ["nano-ros-transport/shim"]
  shim-posix = ["shim", "nano-ros-transport/shim-posix"]
  shim-zephyr = ["shim", "nano-ros-transport/shim-zephyr"]
  shim-smoltcp = ["shim", "nano-ros-transport/shim-smoltcp"]
  ```

- [x] **8.5.2** Create `ShimTransport` in nano-ros-transport
  - `crates/nano-ros-transport/src/shim.rs`
  - Implements `Transport` trait with `ShimSession`, `ShimPublisher`, `ShimSubscriber`

- [x] **8.5.3** Implement `ShimTransportPublisher`
  - Wraps `zenoh_pico_shim::ShimPublisher`
  - Implements `Publisher` trait

- [x] **8.5.4** Implement `ShimTransportSubscriber`
  - Wraps `zenoh_pico_shim::ShimSubscriber`
  - Uses static buffer callback pattern for embedded compatibility
  - Implements `Subscriber` trait

- [x] **8.5.5** Create `ShimExecutor` for embedded polling
  - `crates/nano-ros-node/src/shim.rs`
  - Provides `new(locator)`, `create_node()`, `spin_once()`, `poll()`

- [x] **8.5.6** Create `ShimNode` with pub/sub support
  - `ShimNode` with `create_publisher<M>()` and `create_subscriber<M>()`
  - `ShimNodePublisher<M>` with typed `publish()` and CDR serialization
  - `ShimNodeSubscriber<M, RX_BUF>` with typed `try_recv()` and CDR deserialization

- [x] **8.5.7** Add `shim` feature to nano-ros-transport
  ```toml
  [features]
  shim = ["dep:zenoh-pico-shim"]
  shim-posix = ["shim", "zenoh-pico-shim/posix"]
  shim-zephyr = ["shim", "zenoh-pico-shim/zephyr"]
  shim-smoltcp = ["shim", "zenoh-pico-shim/smoltcp"]
  ```

- [ ] **8.5.8** Migrate nano-ros-transport from zenoh-pico to zenoh-pico-shim (DEFERRED)
  - Note: Current `zenoh` feature still uses legacy `zenoh-pico` crate
  - New `shim` feature provides alternative path for embedded platforms
  - Full migration requires adding Liveliness tokens, ZenohId, Attachment to shim API
  - Tracked for Phase 8.8 cleanup

- [x] **8.5.9** Update Zephyr examples to use zenoh-pico-shim crate
  - **zephyr-talker**: Updated CMakeLists.txt to use shared C shim from zenoh-pico-shim-sys
  - **zephyr-talker**: Updated Cargo.toml to depend on zenoh-pico-shim with zephyr feature
  - **zephyr-talker**: Simplified lib.rs to use ShimContext and ShimPublisher from crate
  - **zephyr-talker**: Removed custom zenoh_shim.c (now using shared version)
  - **zephyr-listener**: Same updates as zephyr-talker
  - **zephyr-listener**: Uses ShimContext and ShimSubscriber from crate
  - Hardware/QEMU testing pending (Phase 8.6)

### Implementation Notes

**Transport Layer (nano-ros-transport/src/shim.rs)**:
- `ShimTransport` implements `Transport` trait
- `ShimSession` implements `Session` trait with `create_publisher()`, `create_subscriber()`
- Uses static buffers for subscriber callbacks (8 max subscribers)
- Platform selection via feature flags: `shim-posix`, `shim-zephyr`, `shim-smoltcp`

**Node Layer (nano-ros-node/src/shim.rs)**:
- `ShimExecutor` provides polling-based execution
- `ShimNode` provides typed publisher/subscriber creation
- `ShimNodePublisher<M>` handles CDR serialization
- `ShimNodeSubscriber<M, RX_BUF>` handles CDR deserialization with const generic buffer size

**Zephyr Integration**:
- C shim compiled by Zephyr's build system (west/CMake), not Cargo
- Rust uses FFI declarations only (`zephyr` feature skips C compilation in build.rs)
- Unified function naming (`zenoh_shim_*`) across all platforms

### Acceptance Criteria
- [x] nano-ros-node compiles with `shim-posix` feature
- [x] `ShimExecutor` provides familiar executor API pattern
- [x] Can create publishers and subscribers through `ShimNode`
- [x] Polling-based execution works without threads
- [x] nano-ros-transport has `shim` features for embedded platforms
- [x] Zephyr examples use zenoh-pico-shim crate (not inline C shim)
- [ ] Hardware/QEMU testing (deferred to Phase 8.6)

---

## Phase 8.6: RTIC/Polling Examples Update + Zephyr Testing

**Status**: Complete
**Priority**: Medium
**Depends on**: 8.5

Updated RTIC and polling examples to use zenoh-pico-shim with smoltcp backend.
Verified zenoh-pico-shim compiles with all platform features.

### Work Items

- [x] **8.6.1** Update `examples/rtic-stm32f4/Cargo.toml`
  - Added stm32-eth 0.8, smoltcp 0.12 dependencies
  - Downgraded stm32f4xx-hal to 0.21 (required by stm32-eth)
  - Added zenoh-pico-shim with smoltcp feature
  - Updated memory.x with ethram section for DMA descriptors

- [x] **8.6.2** Update rtic-stm32f4 main.rs with working network code
  - Full smoltcp + Ethernet integration
  - RTIC 2.x async tasks for polling
  - Bridges smoltcp sockets to zenoh-pico platform buffers
  - Demonstrates the integration pattern (actual zenoh needs cross-compilation)
  ```rust
  use nano_ros_node::shim::{ShimExecutor, ShimNode};

  #[app(device = stm32f4xx_hal::pac, peripherals = true)]
  mod app {
      #[init]
      fn init(cx: init::Context) -> (Shared, Local) {
          // Initialize Ethernet peripheral (handled by zenoh-pico-shim)
          // Create executor with locator
          let executor = ShimExecutor::new(b"tcp/192.168.1.1:7447\0").unwrap();
          let node = executor.create_node("rtic_talker").unwrap();
          let publisher = node.create_publisher::<Int32>("/chatter").unwrap();
          // ...
      }

      #[task(priority = 2)]
      async fn zenoh_poll(cx: zenoh_poll::Context) {
          loop {
              cx.local.executor.spin_once(10).ok();
              Mono::delay(10.millis()).await;
          }
      }
  }
  ```

- [x] **8.6.3** Update `examples/polling-stm32f4/Cargo.toml`
  - Added stm32-eth 0.8, smoltcp 0.12 dependencies
  - Downgraded stm32f4xx-hal to 0.21 (required by stm32-eth)
  - Added zenoh-pico-shim with smoltcp feature
  - Updated memory.x with ethram section for DMA descriptors

- [x] **8.6.4** Update polling-stm32f4 main.rs with working network code
  - Full smoltcp + Ethernet integration
  - Simple polling loop with DWT cycle counter timing
  - Bridges smoltcp sockets to zenoh-pico platform buffers
  - Demonstrates the integration pattern (actual zenoh needs cross-compilation)

- [x] **8.6.5** Network configuration for examples
  - Static IP configuration implemented in both examples
  - IP: 192.168.1.10, Gateway: 192.168.1.1, Router: tcp/192.168.1.1:7447
  - Dynamic configuration can be added in future if needed

- [ ] **8.6.6** Test RTIC example on NUCLEO-F429ZI (hardware pending)
  - Publisher sends to zenoh router
  - Native listener receives messages
  - **Requires**: Cross-compiled zenoh-pico + hardware

- [ ] **8.6.7** Test polling example on NUCLEO-F429ZI (hardware pending)
  - Same tests as RTIC
  - **Requires**: Cross-compiled zenoh-pico + hardware

- [x] **8.6.8** Verify zenoh-pico-shim compiles with all platform features
  - `cargo check -p zenoh-pico-shim --features posix` - OK
  - `cargo check -p zenoh-pico-shim --features zephyr` - OK
  - `cargo check -p zenoh-pico-shim --features smoltcp` - OK
  - Full Zephyr testing requires west workspace setup

### Implementation Notes

**Fixed Issues:**
- Type mismatch in zenoh_shim.c: Changed `int` to `int32_t` for all shim API functions
- Removed unused `extern crate alloc` from zenoh-pico-shim-sys (was causing global allocator requirement)

**Architecture:**
- RTIC example uses RTIC 2.x async tasks for network polling
- Polling example uses simple main loop with DWT cycle counter for timing
- Both examples bridge smoltcp sockets to zenoh-pico platform buffers via:
  - `platform_smoltcp::smoltcp_socket_push_rx()` for incoming data
  - `platform_smoltcp::smoltcp_socket_pop_tx()` for outgoing data

**Integration Pattern:**
The examples demonstrate the correct pattern for smoltcp + zenoh-pico integration:
1. Initialize Ethernet peripheral and smoltcp interface
2. Register poll callback with `platform_smoltcp::smoltcp_set_poll_callback()`
3. Update clock with `platform_smoltcp::smoltcp_set_clock_ms()` periodically
4. Bridge data between smoltcp TCP sockets and shim's static buffers

### Acceptance Criteria
- [x] RTIC example compiles for thumbv7em-none-eabihf
- [x] Polling example compiles for thumbv7em-none-eabihf
- [x] zenoh-pico-shim compiles with posix, zephyr, smoltcp features
- [ ] Hardware testing on NUCLEO-F429ZI (pending zenoh-pico cross-compilation)
- [ ] Zephyr examples work on native_sim/QEMU (pending west workspace setup)

---

## Phase 8.7: Native Examples Verification

**Status**: Complete
**Priority**: Medium

Verified all native examples work correctly with executor API.

### Work Items

- [x] **8.7.1** Test native-talker with zenoh feature
  ```bash
  cargo run -p native-talker --features zenoh
  ```

- [x] **8.7.2** Test native-listener with zenoh feature
  ```bash
  cargo run -p native-listener --features zenoh
  ```

- [x] **8.7.3** Test native-service-server with zenoh feature
  - Builds and runs successfully
  - Service RPC transport not yet implemented (see Phase 3)

- [x] **8.7.4** Test native-service-client with zenoh feature
  - Builds and runs successfully
  - Service call returns warning: "Service client call not yet implemented"

- [x] **8.7.5** Test native-action-server with zenoh feature
  - Builds and runs successfully
  - Action transport depends on service transport (not yet implemented)

- [x] **8.7.6** Test native-action-client with zenoh feature
  - Builds and runs successfully
  - Goal sending fails due to missing service transport

- [x] **8.7.7** Test talker ↔ listener communication
  - ✅ **WORKING**: Listener successfully receives messages from talker
  - Start zenoh router: `zenohd --listen tcp/127.0.0.1:7447`
  - Run listener: `RUST_LOG=info cargo run -p native-listener --features zenoh`
  - Run talker: `RUST_LOG=info cargo run -p native-talker --features zenoh`
  - Messages received correctly (data=1 through data=7 verified)

- [x] **8.7.8** Test service client ↔ server communication
  - **NOT YET WORKING**: Service RPC transport not implemented
  - Both examples build and create nodes/servers/clients
  - Tracked in Phase 3.1 (Service infrastructure)

- [x] **8.7.9** Test action client ↔ server communication
  - **NOT YET WORKING**: Actions depend on services
  - Both examples build and create nodes/servers/clients
  - Tracked in Phase 6 (ROS 2 Actions)

- [ ] **8.7.10** Test ROS 2 interoperability
  - Deferred: requires ROS 2 environment
  - native-talker → ROS 2 listener
  - ROS 2 talker → native-listener

### Build Fixes Applied

1. **zenoh-pico-sys build.rs**: Fixed symlink handling
   - Changed `cp -r` to `cp -rL` to follow symlinks when copying source tree

2. **native-talker/src/main.rs**: Fixed borrowing issues
   - Added `mut` to node declaration
   - Fixed parameter lifetime by getting value immediately

3. **native-listener/src/main.rs**: Fixed API changes
   - Added `mut` to node declaration
   - Fixed turbofish syntax: `create_subscription::<Int32, _>(...)`

### Acceptance Criteria
- [x] All native examples compile and run
- [x] Pub/sub communication works
- [ ] Service communication works (pending Phase 3 implementation)
- [ ] Action communication works (pending Phase 6 implementation)
- [ ] ROS 2 interop verified (deferred)

---

## Phase 8.8: Documentation and Cleanup

**Status**: Not Started
**Priority**: Low
**Depends on**: 8.6, 8.7

### Work Items

- [ ] **8.8.1** Update `docs/embedded-integration.md`
  - Document Zephyr integration (complete)
  - Document smoltcp + RTIC integration
  - Document smoltcp + polling integration
  - Add hardware requirements matrix

- [ ] **8.8.2** Update example READMEs
  - rtic-stm32f4/README.md
  - polling-stm32f4/README.md
  - zephyr-talker/README.md
  - zephyr-listener/README.md

- [ ] **8.8.3** Update CLAUDE.md examples section
  - Document all working examples
  - Document build/run instructions

- [ ] **8.8.4** Add embedded example build check to CI (if feasible)

- [ ] **8.8.5** Update justfile with embedded commands
  ```makefile
  build-rtic:
      cd examples/rtic-stm32f4 && cargo build --release

  build-polling:
      cd examples/polling-stm32f4 && cargo build --release
  ```

- [ ] **8.8.6** Remove legacy zenoh crates
  - Remove `crates/zenoh-pico-sys/` (merged into zenoh-pico-shim-sys)
  - Remove `crates/zenoh-pico/` (merged into zenoh-pico-shim)
  - Remove symlink `crates/zenoh-pico-sys/zenoh-pico`
  - Update workspace `Cargo.toml` to remove legacy crates from members
  - Verify no crates depend on removed crates
  - Update CLAUDE.md crate descriptions
  - **Prerequisite**: 8.5.8 (transport migration) must be complete

- [ ] **8.8.7** Final CLAUDE.md cleanup
  - Update zenoh-pico bindings section to describe new architecture
  - Remove references to old zenoh-pico-sys/zenoh-pico crates
  - Update crate dependency diagram
  - Document shim vs zenoh feature selection

### Acceptance Criteria
- All documentation updated
- CI includes embedded checks where feasible
- justfile has embedded build commands
- Legacy crates removed (zenoh-pico-sys, zenoh-pico)
- No dead code or unused dependencies in workspace

---

## Memory Budget

| Component | Size | Notes |
|-----------|------|-------|
| Ethernet DMA buffers | ~8 KB | 4 RX + 4 TX descriptors |
| smoltcp Interface | ~1 KB | Configuration dependent |
| TCP socket buffers | ~8 KB | 2 KB × 2 sockets × 2 (RX/TX) |
| zenoh-pico heap | ~16 KB | Session + publishers + subscribers |
| **Total** | **~33 KB** | Fits in STM32F429 (256 KB RAM) |

---

## Dependencies

```
                                    ┌─────────────────────────────────────────┐
                                    │                                         │
Phase 8.1 (Zephyr) ─────────────────┤                                         │
     COMPLETE                       │                                         │
                                    │                                         │
                                    ▼                                         │
Phase 8.2 (zenoh-pico-shim) ────────┬─────────────────────────────────────────┤
     C shim + backend interface     │                                         │
     Zephyr + POSIX backends        │                                         │
                                    │                                         │
                                    │    ┌────────────────────────────────┐   │
                                    │    │                                │   │
                                    ▼    ▼                                │   │
Phase 8.3 (smoltcp validation)     Phase 8.4 (smoltcp platform)          │   │
     Hardware TCP test              z_* functions in zenoh-pico-shim     │   │
                                    smoltcp FFI layer                    │   │
                                    │                                    │   │
                                    ▼                                    │   │
                              Phase 8.4.5 (Library Restructure)          │   │
                                    zenoh-pico-shim-sys (C code)         │   │
                                    zenoh-pico-shim (Rust API)           │   │
                                    │                                    │   │
                                    ▼                                    │   │
                              Phase 8.5 (nano-ros integration)           │   │
                                    ShimExecutor, ShimNode               │   │
                                    shim feature in nano-ros-node        │   │
                                    8.5.8: Transport migration ──────┐   │   │
                                    │                                │   │   │
                                    ▼                                │   │   │
                              Phase 8.6 (RTIC/polling examples) ─────┤───┘   │
                                    Hardware testing                 │       │
                                                                     │       │
                              Phase 8.8.6 (Old crates removal) ◄─────┘       │
                                    Remove zenoh-pico-sys, zenoh-pico        │
                                                                             │
Phase 8.7 (native verification) ─────────────────────────────────────────────┤
     Verify executor API works                                               │
                                                                             │
Phase 8.8 (documentation) ───────────────────────────────────────────────────┘
```

### Crate Dependency Graph (After Phase 8.4.5 Refactor)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Applications                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────────────┐        ┌────────────────────────────────────┐  │
│  │  native-talker          │        │  rtic-stm32f4, polling-stm32f4     │  │
│  │  native-listener        │        │  zephyr-talker, zephyr-listener    │  │
│  │  native-service-*       │        │                                    │  │
│  └────────────┬────────────┘        └─────────────────┬──────────────────┘  │
│               │                                       │                      │
│               │ (posix feature)                       │ (smoltcp/zephyr)     │
│               │                                       │                      │
│               └───────────────────┬───────────────────┘                      │
│                                   │                                          │
│                                   ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         nano-ros-node                                │    │
│  │                    (shim feature for embedded)                       │    │
│  └─────────────────────────────────┬───────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  zenoh-pico-shim (High-level Rust API)                               │    │
│  │  ├── src/lib.rs         - Public API, re-exports                     │    │
│  │  ├── src/session.rs     - Session management                         │    │
│  │  ├── src/publisher.rs   - Publisher                                  │    │
│  │  ├── src/subscriber.rs  - Subscriber                                 │    │
│  │  └── src/platform/      - Platform Rust code (smoltcp_*)             │    │
│  └─────────────────────────────────┬───────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  zenoh-pico-shim-sys (FFI + all C code)                              │    │
│  │  ├── src/lib.rs                - FFI declarations only               │    │
│  │  ├── c/shim/zenoh_shim.c       - C shim implementation               │    │
│  │  ├── c/platform_smoltcp/*.c    - smoltcp platform (optional)         │    │
│  │  └── zenoh-pico/               - Git submodule (upstream)            │    │
│  └─────────────────────────────────┬───────────────────────────────────┘    │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  zenoh-pico C library (submodule, compiled by -sys crate)            │    │
│  │  Platform selection: ZENOH_LINUX / ZENOH_ZEPHYR / ZENOH_GENERIC      │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘

Removed crates (Phase 8.8.6, after transport migration):
- zenoh-pico-sys (merged into zenoh-pico-shim-sys)
- zenoh-pico (merged into zenoh-pico-shim)
- Note: Currently kept for backward compatibility with nano-ros-transport
```

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Memory fragmentation | High | Use arena allocator for zenoh-pico |
| Timing issues (missed packets) | Medium | Aggressive polling in blocking ops |
| Complex FFI boundary | Medium | Thorough testing, minimize unsafe code |
| C cross-compilation setup | Low | Document toolchain setup clearly |
| Hardware availability | Low | Test on QEMU where possible |
