# Phase 8: Embedded Networking

**Goal**: Enable network connectivity for all embedded nano-ros examples using smoltcp + zenoh-pico for bare-metal RTIC/polling, and verify native examples work correctly.

**Status**: In Progress (Phase 8.1-8.2 Complete)

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
│  │  (unchanged path)        │       │  (new zenoh-pico-shim path)    │  │
│  └────────────┬─────────────┘       └───────────────┬────────────────┘  │
│               │                                     │                    │
│               ▼                                     ▼                    │
│  ┌──────────────────────────┐       ┌────────────────────────────────┐  │
│  │  zenoh-pico (safe Rust)  │       │  zenoh-pico-shim               │  │
│  │                          │       │  ├── shim/ (C API)             │  │
│  │                          │       │  ├── platform_smoltcp/ (z_*)   │  │
│  │                          │       │  └── platform_zephyr/ (z_*)    │  │
│  └────────────┬─────────────┘       └───────────────┬────────────────┘  │
│               │                                     │                    │
│               ▼                                     │                    │
│  ┌──────────────────────────┐                       │                    │
│  │  zenoh-pico-sys (FFI)    │◄──────────────────────┘                    │
│  │  (bindings only)         │                                            │
│  └────────────┬─────────────┘                                            │
│               │                                                          │
│               ▼                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  zenoh-pico C library                                             │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key Design Decisions:**
- Native/std applications continue using `zenoh-pico` → `zenoh-pico-sys` (unchanged)
- Embedded applications use `zenoh-pico-shim` which provides:
  - High-level C shim API (`zenoh_shim_*`) for FFI simplicity
  - Platform layer implementations (`z_*` functions) for smoltcp and Zephyr
- `zenoh-pico-shim` depends on `zenoh-pico-sys` for FFI bindings

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

- [ ] **8.2.11** Migrate zephyr-talker to use `zenoh-pico-shim` crate
  - Deferred to Phase 8.6 (requires testing Zephyr backend)

- [ ] **8.2.12** Migrate zephyr-listener to use `zenoh-pico-shim` crate
  - Deferred to Phase 8.6 (requires testing Zephyr backend)

- [ ] **8.2.13** Test Zephyr examples still work after migration
  - Deferred to Phase 8.6 (requires Zephyr hardware/QEMU)

### Acceptance Criteria
- [x] `zenoh-pico-shim` crate compiles with `posix` feature
- [x] POSIX backend enables desktop testing (tests pass)
- [x] Directory structure ready for smoltcp platform layer (Phase 8.4)
- [x] `no_std` compatible (builds for thumbv7em-none-eabihf without backend)
- [ ] Zephyr examples migrated (deferred to Phase 8.6)

---

## Phase 8.3: smoltcp Standalone Validation

**Status**: Not Started
**Priority**: High

Validate smoltcp + stm32-eth works on target hardware before integrating with zenoh-pico.

### Work Items

- [ ] **8.3.1** Create `examples/smoltcp-test/` standalone example
  - Minimal RTIC example with smoltcp + stm32-eth
  - TCP echo server for testing

- [ ] **8.3.2** Implement `Device` trait for STM32 Ethernet
  ```rust
  impl smoltcp::phy::Device for EthernetDevice {
      type RxToken = EthRxToken;
      type TxToken = EthTxToken;

      fn receive(&mut self, timestamp: Instant) -> Option<(Self::RxToken, Self::TxToken)>;
      fn transmit(&mut self) -> Option<Self::TxToken>;
      fn capabilities(&self) -> DeviceCapabilities;
  }
  ```

- [ ] **8.3.3** Set up static buffer allocation for smoltcp
  ```rust
  const SOCKET_RX_SIZE: usize = 2048;
  const SOCKET_TX_SIZE: usize = 2048;
  const MAX_SOCKETS: usize = 4;

  static mut SOCKET_RX_BUFFERS: [[u8; SOCKET_RX_SIZE]; MAX_SOCKETS] = ...;
  static mut SOCKET_TX_BUFFERS: [[u8; SOCKET_TX_SIZE]; MAX_SOCKETS] = ...;
  ```

- [ ] **8.3.4** Implement main polling loop pattern
  ```rust
  loop {
      iface.poll(now, &mut device, &mut sockets);
      // Handle socket I/O
      if let Some(delay) = iface.poll_delay(now, &sockets) {
          sleep(delay);
      }
  }
  ```

- [ ] **8.3.5** Test TCP connection to host PC
  - Verify send/receive works
  - Measure latency and throughput
  - Document memory usage

### Acceptance Criteria
- TCP echo server works on NUCLEO-F429ZI
- Can connect from host PC and exchange data
- Memory usage documented (target: <40KB total)

---

## Phase 8.4: smoltcp Platform Layer in zenoh-pico-shim

**Status**: Not Started
**Priority**: High
**Depends on**: 8.2, 8.3

Implement the zenoh-pico platform abstraction (`z_*` functions) using smoltcp within the `zenoh-pico-shim` crate. This provides the low-level socket and system operations needed by zenoh-pico on bare-metal.

**Note:** The platform layer lives in `zenoh-pico-shim/platform_smoltcp/`, not in `zenoh-pico-sys`. This keeps `zenoh-pico-sys` as pure FFI bindings while `zenoh-pico-shim` owns all embedded-specific code.

### Work Items

- [ ] **8.4.1** Implement memory management (`platform_smoltcp/system.c`)
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

- [ ] **8.4.2** Implement random number generation (`platform_smoltcp/system.c`)
  ```c
  uint8_t z_random_u8(void);
  uint16_t z_random_u16(void);
  uint32_t z_random_u32(void);
  uint64_t z_random_u64(void);
  void z_random_fill(void *buf, size_t len);
  ```
  - Use STM32 hardware RNG peripheral or seeded PRNG

- [ ] **8.4.3** Implement time/clock functions (`platform_smoltcp/system.c`)
  ```c
  z_clock_t z_clock_now(void);
  unsigned long z_clock_elapsed_us(z_clock_t *time);
  unsigned long z_clock_elapsed_ms(z_clock_t *time);
  z_result_t z_sleep_us(size_t time);
  z_result_t z_sleep_ms(size_t time);
  ```
  - Use DWT cycle counter (Cortex-M debug unit) for timing
  - `z_sleep_*` busy-waits or yields to smoltcp polling

- [ ] **8.4.4** Implement threading stubs (`platform_smoltcp/system.c`)
  ```c
  // With Z_FEATURE_MULTI_THREAD=0, these become no-ops
  z_result_t _z_mutex_init(_z_mutex_t *m) { return _Z_RES_OK; }
  z_result_t _z_mutex_lock(_z_mutex_t *m) { return _Z_RES_OK; }
  z_result_t _z_mutex_unlock(_z_mutex_t *m) { return _Z_RES_OK; }
  z_result_t _z_condvar_init(_z_condvar_t *cv) { return _Z_RES_OK; }
  // etc.
  ```

- [ ] **8.4.5** Define socket type structures (`platform_smoltcp/network.c`)
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

- [ ] **8.4.6** Implement `_z_create_endpoint_tcp` (`platform_smoltcp/network.c`)
  ```c
  z_result_t _z_create_endpoint_tcp(
      _z_sys_net_endpoint_t *ep,
      const char *s_address,
      const char *s_port
  );
  ```
  - Parse IP string "192.168.1.1" into bytes
  - Parse port string into uint16_t

- [ ] **8.4.7** Implement `_z_open_tcp` (`platform_smoltcp/network.c`)
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

- [ ] **8.4.8** Implement `_z_read_tcp` with blocking wrapper
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

- [ ] **8.4.9** Implement `_z_send_tcp` with blocking wrapper
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

- [ ] **8.4.10** Implement `_z_close_tcp`
  ```c
  void _z_close_tcp(_z_sys_net_socket_t *sock) {
      smoltcp_socket_close(sock->socket_handle);
      sock->connected = false;
  }
  ```

- [ ] **8.4.11** Create Rust FFI module for smoltcp operations (`platform_smoltcp/mod.rs`)
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

- [ ] **8.4.12** Complete smoltcp backend (`shim/backend_smoltcp.c`)
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

- [ ] **8.4.13** Update zenoh-pico-shim `build.rs` for smoltcp platform layer
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

- [ ] **8.4.14** Configure zenoh-pico CMake for bare-metal
  ```cmake
  # zenoh-pico-sys/cmake/arm-none-eabi.cmake
  set(CMAKE_SYSTEM_NAME Generic)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(CMAKE_C_COMPILER arm-none-eabi-gcc)

  # Disable features not needed for embedded
  set(Z_FEATURE_MULTI_THREAD 0)
  set(Z_FEATURE_LINK_TCP 1)
  set(Z_FEATURE_LINK_UDP_MULTICAST 0)
  set(Z_FEATURE_LINK_UDP_UNICAST 0)
  set(Z_FEATURE_SCOUTING_UDP 0)
  ```

- [ ] **8.4.15** Test smoltcp platform layer compiles for thumbv7em-none-eabihf
  ```bash
  cd crates/zenoh-pico-shim
  cargo build --target thumbv7em-none-eabihf --features smoltcp
  ```

### Acceptance Criteria
- `zenoh-pico-shim` compiles with `smoltcp` feature for thumbv7em-none-eabihf
- Platform layer implements all required `z_*` functions
- smoltcp FFI functions callable from C platform code
- Can create zenoh session using smoltcp as transport

---

## Phase 8.5: nano-ros-node Integration with zenoh-pico-shim

**Status**: Not Started
**Priority**: High
**Depends on**: 8.4

Integrate `zenoh-pico-shim` with `nano-ros-node` to enable embedded nano-ros applications.

### Work Items

- [ ] **8.5.1** Add `shim` feature to nano-ros-node
  ```toml
  # crates/nano-ros-node/Cargo.toml
  [features]
  default = ["std"]
  std = ["nano-ros-transport/std"]
  zenoh = ["nano-ros-transport/zenoh"]
  shim = ["dep:zenoh-pico-shim"]  # For embedded platforms

  [dependencies.zenoh-pico-shim]
  path = "../zenoh-pico-shim"
  optional = true
  ```

- [ ] **8.5.2** Create `ShimTransport` in nano-ros-transport
  ```rust
  // crates/nano-ros-transport/src/shim.rs
  use zenoh_pico_shim::{ShimContext, ShimPublisher, ShimSubscriber};

  pub struct ShimTransport {
      context: ShimContext,
  }

  impl Transport for ShimTransport {
      type Publisher = ShimTransportPublisher;
      type Subscriber = ShimTransportSubscriber;
      // ...
  }
  ```

- [ ] **8.5.3** Implement `ShimTransportPublisher`
  - Wraps `zenoh_pico_shim::ShimPublisher`
  - Handles CDR serialization before publish

- [ ] **8.5.4** Implement `ShimTransportSubscriber`
  - Wraps `zenoh_pico_shim::ShimSubscriber`
  - Handles CDR deserialization in callback

- [ ] **8.5.5** Create `ShimExecutor` for embedded polling
  ```rust
  // crates/nano-ros-node/src/shim_executor.rs
  pub struct ShimExecutor {
      context: ShimContext,
  }

  impl ShimExecutor {
      pub fn new(locator: &[u8]) -> Result<Self, ShimError>;
      pub fn create_node(&self, name: &str) -> Result<ShimNode, ShimError>;
      pub fn spin_once(&self, timeout_ms: u32) -> Result<(), ShimError>;
  }
  ```

- [ ] **8.5.6** Create `ShimNode` with pub/sub support
  ```rust
  pub struct ShimNode { ... }

  impl ShimNode {
      pub fn create_publisher<M: RosMessage>(&self, topic: &str)
          -> Result<ShimNodePublisher<M>, ShimError>;
      pub fn create_subscriber<M, F>(&self, topic: &str, callback: F)
          -> Result<ShimNodeSubscriber<M>, ShimError>
      where M: RosMessage, F: FnMut(M) + 'static;
  }
  ```

- [ ] **8.5.7** Add `shim` feature to nano-ros-transport
  ```toml
  [features]
  shim = ["dep:zenoh-pico-shim"]

  [dependencies.zenoh-pico-shim]
  path = "../zenoh-pico-shim"
  optional = true
  ```

### Acceptance Criteria
- nano-ros-node compiles with `shim` feature for thumbv7em-none-eabihf
- `ShimExecutor` provides familiar executor API pattern
- Can create publishers and subscribers through `ShimNode`
- Polling-based execution works without threads

---

## Phase 8.6: RTIC/Polling Examples Update

**Status**: Not Started
**Priority**: Medium
**Depends on**: 8.5

Update RTIC and polling examples to use zenoh-pico-shim with smoltcp backend.

### Work Items

- [ ] **8.6.1** Update `examples/rtic-stm32f4/Cargo.toml`
  ```toml
  [dependencies]
  nano-ros-node = { path = "../../crates/nano-ros-node", default-features = false, features = ["shim"] }
  zenoh-pico-shim = { path = "../../crates/zenoh-pico-shim", features = ["smoltcp"] }
  std_msgs = { version = "*", default-features = false }
  ```

- [ ] **8.6.2** Update rtic-stm32f4 main.rs with working network code
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

- [ ] **8.6.3** Update `examples/polling-stm32f4/Cargo.toml`
  ```toml
  [dependencies]
  nano-ros-node = { path = "../../crates/nano-ros-node", default-features = false, features = ["shim"] }
  zenoh-pico-shim = { path = "../../crates/zenoh-pico-shim", features = ["smoltcp"] }
  std_msgs = { version = "*", default-features = false }
  ```

- [ ] **8.6.4** Update polling-stm32f4 main.rs with working network code
  ```rust
  use nano_ros_node::shim::{ShimExecutor, ShimNode};

  #[entry]
  fn main() -> ! {
      let executor = ShimExecutor::new(b"tcp/192.168.1.1:7447\0").unwrap();
      let node = executor.create_node("polling_talker").unwrap();
      let publisher = node.create_publisher::<Int32>("/chatter").unwrap();

      let mut count = 0i32;
      loop {
          // Poll zenoh (processes network + dispatches callbacks)
          executor.spin_once(10).ok();

          // Publish periodically
          if should_publish() {
              publisher.publish(&Int32 { data: count }).ok();
              count += 1;
          }
      }
  }
  ```

- [ ] **8.6.5** Network configuration for examples
  ```rust
  // Network config set via zenoh-pico-shim initialization
  // IP: 192.168.1.10, Gateway: 192.168.1.1, Router: tcp/192.168.1.1:7447
  let executor = ShimExecutor::new_with_config(NetworkConfig {
      ip: [192, 168, 1, 10],
      gateway: [192, 168, 1, 1],
      mac: [0x02, 0x00, 0x00, 0x00, 0x00, 0x01],
      locator: b"tcp/192.168.1.1:7447\0",
  }).unwrap();
  ```

- [ ] **8.6.6** Test RTIC example on NUCLEO-F429ZI
  - Publisher sends to zenoh router
  - Native listener receives messages

- [ ] **8.6.7** Test polling example on NUCLEO-F429ZI
  - Same tests as RTIC

### Acceptance Criteria
- RTIC example compiles and runs on hardware
- Polling example compiles and runs on hardware
- Can communicate with native nodes via zenoh router
- Uses zenoh-pico-shim with smoltcp feature

---

## Phase 8.7: Native Examples Verification

**Status**: Not Started
**Priority**: Medium

Verify all native examples work correctly with executor API.

### Work Items

- [ ] **8.7.1** Test native-talker with zenoh feature
  ```bash
  cargo run -p native-talker --features zenoh
  ```

- [ ] **8.7.2** Test native-listener with zenoh feature
  ```bash
  cargo run -p native-listener --features zenoh
  ```

- [ ] **8.7.3** Test native-service-server with zenoh feature

- [ ] **8.7.4** Test native-service-client with zenoh feature

- [ ] **8.7.5** Test native-action-server with zenoh feature

- [ ] **8.7.6** Test native-action-client with zenoh feature

- [ ] **8.7.7** Test talker ↔ listener communication
  - Start zenoh router
  - Run native-talker
  - Run native-listener
  - Verify messages received

- [ ] **8.7.8** Test service client ↔ server communication

- [ ] **8.7.9** Test action client ↔ server communication

- [ ] **8.7.10** Test ROS 2 interoperability
  - native-talker → ROS 2 listener
  - ROS 2 talker → native-listener

### Acceptance Criteria
- All native examples compile and run
- Pub/sub communication works
- Service communication works
- Action communication works
- ROS 2 interop verified

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

### Acceptance Criteria
- All documentation updated
- CI includes embedded checks where feasible
- justfile has embedded build commands

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
                              Phase 8.5 (nano-ros integration)           │   │
                                    ShimExecutor, ShimNode               │   │
                                    shim feature in nano-ros-node        │   │
                                    │                                    │   │
                                    ▼                                    │   │
                              Phase 8.6 (RTIC/polling examples) ─────────┘   │
                                    Hardware testing                         │
                                                                             │
Phase 8.7 (native verification) ─────────────────────────────────────────────┤
     Verify executor API works                                               │
                                                                             │
Phase 8.8 (documentation) ───────────────────────────────────────────────────┘
```

### Crate Dependency Graph

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
│               │ (std, zenoh features)                 │ (shim feature)       │
│               │                                       │                      │
│               ▼                                       ▼                      │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         nano-ros-node                                │    │
│  │  ┌──────────────────────┐     ┌─────────────────────────────────┐   │    │
│  │  │ ConnectedExecutor    │     │ ShimExecutor (shim feature)     │   │    │
│  │  │ (std, zenoh features)│     │                                 │   │    │
│  │  └──────────┬───────────┘     └────────────────┬────────────────┘   │    │
│  └─────────────┼──────────────────────────────────┼────────────────────┘    │
│                │                                  │                          │
│                ▼                                  ▼                          │
│  ┌──────────────────────────┐     ┌────────────────────────────────────┐    │
│  │  zenoh-pico (Rust)       │     │  zenoh-pico-shim                   │    │
│  │  Safe wrapper            │     │  ├── shim/ (C API)                 │    │
│  └──────────┬───────────────┘     │  ├── platform_smoltcp/ (z_*)       │    │
│             │                     │  └── platform_zephyr/              │    │
│             │                     └────────────────┬───────────────────┘    │
│             │                                      │                         │
│             ▼                                      │                         │
│  ┌──────────────────────────┐                      │                         │
│  │  zenoh-pico-sys (FFI)    │◄─────────────────────┘                         │
│  │  Pure bindings only      │                                                │
│  └──────────┬───────────────┘                                                │
│             │                                                                │
│             ▼                                                                │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │  zenoh-pico C library (external)                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
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
