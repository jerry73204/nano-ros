# Phase 8: Embedded Networking & Examples

This phase focuses on providing working network connectivity for embedded nano-ros examples and ensuring all examples use the unified executor API.

## Goals

1. Ensure all native examples use the executor API and work correctly
2. Create working Zephyr examples with full networking (Phase A - COMPLETE)
3. Create working Embassy + smoltcp examples with pure-Rust networking (Phase B)
4. Explore options for RTIC/polling examples to have network connectivity (Phase C)

---

## Current Status

### Working Examples

| Example | Executor API | Network | Status |
|---------|-------------|---------|--------|
| native-talker | ✅ BasicExecutor | ✅ zenoh | Working |
| native-listener | ✅ BasicExecutor | ✅ zenoh | Working |
| native-service-server | ✅ BasicExecutor | ✅ zenoh | Working |
| native-service-client | ✅ BasicExecutor | ✅ zenoh | Working |
| native-action-server | ✅ BasicExecutor | ✅ zenoh | Working |
| native-action-client | ✅ BasicExecutor | ✅ zenoh | Working |
| zephyr-talker | ✅ ZephyrExecutor | ✅ zenoh-pico | Working |
| zephyr-listener | ✅ ZephyrExecutor | ✅ zenoh-pico | Working |

### Examples Needing Network Support

| Example | Executor API | Network | Status |
|---------|-------------|---------|--------|
| rtic-stm32f4 | Pattern shown | ❌ None | Code commented out |
| polling-stm32f4 | Pattern shown | ❌ None | Code commented out |
| embassy-stm32f4 | Pattern shown | ❌ None | Code commented out |

### Directory Structure (Target)

```
examples/
├── native-talker/          # Native Linux/macOS with zenoh
├── native-listener/        # Native Linux/macOS with zenoh
├── native-service-server/  # Service server example
├── native-service-client/  # Service client example
├── native-action-server/   # Action server example
├── native-action-client/   # Action client example
├── zephyr-talker/          # Zephyr RTOS + zenoh-pico (COMPLETE)
├── zephyr-listener/        # Zephyr RTOS + zenoh-pico (COMPLETE)
├── embassy-talker/         # Embassy + smoltcp + zenoh (NEW)
├── embassy-listener/       # Embassy + smoltcp + zenoh (NEW)
├── rtic-stm32f4/           # RTIC pattern demo (API only)
├── polling-stm32f4/        # Polling pattern demo (API only)
└── qemu-test/              # QEMU testing
```

---

## Phase A: Zephyr Examples - COMPLETE

### Architecture

```
┌─────────────────────────────────────┐
│  Rust Application                   │
│  - ZephyrContext, ZephyrExecutor    │
│  - ZephyrPublisher, ZephyrSubscriber│
├─────────────────────────────────────┤
│  zenoh_shim.c                       │
│  - Thin wrapper around zenoh-pico   │
│  - Handles FFI struct size issues   │
├─────────────────────────────────────┤
│  zenoh-pico (C)                     │
│  - Built with Zephyr                │
├─────────────────────────────────────┤
│  Zephyr RTOS                        │
│  - Network stack, threading, HAL    │
└─────────────────────────────────────┘
```

### Completed Work Items

- [x] Remove C-only Zephyr examples
- [x] Rename zephyr-talker-rs → zephyr-talker
- [x] Rename zephyr-listener-rs → zephyr-listener
- [x] Create unified C shim supporting multiple pub/sub
- [x] Update zephyr-talker to use executor API pattern
- [x] Update zephyr-listener to use executor API pattern
- [x] Generate message bindings using `cargo nano-ros generate`
- [x] Update workspace exclude list

---

## Phase B: Embassy + smoltcp Examples (Pure Rust)

### Overview

Embassy examples provide a pure-Rust embedded solution using:
- **Embassy** async executor (no RTOS)
- **embassy-net + smoltcp** for TCP/IP networking
- **zenoh-pico** via platform abstraction or pure-Rust zenoh client

### Architecture Options

#### Option B1: zenoh-pico with smoltcp Platform Layer

```
┌─────────────────────────────────────┐
│  Rust Application                   │
│  - PollingExecutor                  │
│  - NodeHandle, Publisher, etc.      │
├─────────────────────────────────────┤
│  nano-ros-node (Rust)               │
│  - Uses EmbassyTransport            │
├─────────────────────────────────────┤
│  EmbassyTransport (Rust)            │
│  - Wraps zenoh-pico + smoltcp       │
├─────────────────────────────────────┤
│  zenoh-pico-smoltcp (NEW)           │
│  - Platform layer for smoltcp       │
│  - Implements z_socket_*, z_mutex_* │
├─────────────────────────────────────┤
│  embassy-net + smoltcp              │
│  - Pure Rust TCP/IP stack           │
├─────────────────────────────────────┤
│  embassy-stm32 (ETH driver)         │
└─────────────────────────────────────┘
```

**Challenges:**
- zenoh-pico expects blocking socket APIs; smoltcp is async
- zenoh-pico uses threads; Embassy is cooperative async
- Need to bridge sync/async boundary

#### Option B2: Pure-Rust Zenoh Client (Long-term)

```
┌─────────────────────────────────────┐
│  Rust Application                   │
│  - PollingExecutor                  │
│  - NodeHandle, Publisher, etc.      │
├─────────────────────────────────────┤
│  nano-ros-node (Rust)               │
│  - Uses EmbassyTransport            │
├─────────────────────────────────────┤
│  nano-ros-zenoh (NEW)               │
│  - Pure Rust zenoh implementation   │
│  - Minimal feature set for pub/sub  │
├─────────────────────────────────────┤
│  embassy-net + smoltcp              │
│  - Pure Rust TCP/IP stack           │
├─────────────────────────────────────┤
│  embassy-stm32 (ETH driver)         │
└─────────────────────────────────────┘
```

**Pros:** Clean architecture, no FFI, no C toolchain needed
**Cons:** Significant implementation effort, must implement zenoh protocol

### Work Items

#### B.1 Evaluate Platform Layer Approach

- [ ] Study zenoh-pico platform abstraction API
- [ ] Prototype smoltcp socket wrapper
- [ ] Assess async/sync bridging complexity
- [ ] Decision: proceed with platform layer or pure-Rust client

#### B.2 Create zenoh-pico-smoltcp Platform Layer (if B.1 positive)

- [ ] Create `crates/zenoh-pico-smoltcp/` crate
- [ ] Implement socket API using smoltcp TcpSocket:
  ```rust
  pub fn z_socket_open(...) -> z_result_t;
  pub fn z_socket_connect(...) -> z_result_t;
  pub fn z_socket_send(...) -> z_result_t;
  pub fn z_socket_recv(...) -> z_result_t;
  pub fn z_socket_close(...) -> z_result_t;
  ```
- [ ] Implement mutex/threading stubs (single-threaded mode)
- [ ] Implement time API using embassy-time
- [ ] Handle async/sync boundary with polling

#### B.3 Cross-Compile zenoh-pico for ARM

- [ ] Create CMake toolchain file for ARM Cortex-M:
  ```cmake
  set(CMAKE_SYSTEM_NAME Generic)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(CMAKE_C_COMPILER arm-none-eabi-gcc)
  set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
  ```
- [ ] Update zenoh-pico-sys build.rs for embedded targets
- [ ] Configure zenoh-pico with:
  - `Z_FEATURE_MULTI_THREAD=0` (single-threaded)
  - `Z_FEATURE_LINK_TCP=1`
  - `CMAKE_SYSTEM_NAME=Generic`

#### B.4 Create EmbassyTransport Backend

- [ ] Create `crates/nano-ros-transport/src/embassy.rs`
- [ ] Define `EmbassyTransport` struct
- [ ] Integrate with embassy-net `TcpSocket`
- [ ] Add `embassy` feature flag to nano-ros-transport

#### B.5 Create embassy-talker Example

- [ ] Create `examples/embassy-talker/`
- [ ] Set up embassy-net with Ethernet:
  ```rust
  #[embassy_executor::main]
  async fn main(spawner: Spawner) {
      let p = embassy_stm32::init(Default::default());

      // Initialize Ethernet
      let eth = Ethernet::new(...);
      let stack = Stack::new(...);
      spawner.spawn(net_task(stack)).unwrap();

      // Wait for network
      stack.wait_config_up().await;

      // Create nano-ros context
      let ctx = Context::new(InitOptions::new()
          .locator("tcp/192.168.1.100:7447")
      ).unwrap();

      let mut executor: PollingExecutor<1> = ctx.create_polling_executor();
      let node = executor.create_node("embassy_talker").unwrap();
      let publisher = node.create_publisher::<Int32>("/chatter").unwrap();

      // Publish loop
      loop {
          publisher.publish(&Int32 { data: counter }).unwrap();
          executor.spin_once(100);
          Timer::after(Duration::from_millis(100)).await;
      }
  }
  ```

#### B.6 Create embassy-listener Example

- [ ] Create `examples/embassy-listener/`
- [ ] Subscription with callback pattern
- [ ] Demonstrate message reception

#### B.7 Remove/Update Old Embassy Example

- [ ] Update `examples/embassy-stm32f4/` to use embassy-net
- [ ] Or delete and replace with embassy-talker/listener

#### B.8 Testing

- [ ] Test on NUCLEO-F429ZI with Ethernet
- [ ] Test interop with native nodes via zenoh router
- [ ] Test interop with Zephyr nodes

---

## Phase C: RTIC/Polling Network Options

The RTIC and polling examples are bare-metal without an RTOS. They currently demonstrate the executor API pattern but cannot actually communicate because:

1. **No network stack** - Bare-metal STM32F4 has no TCP/IP implementation
2. **No zenoh-pico** - Cross-compilation requires arm-none-eabi-gcc toolchain

### Options for RTIC/Polling Network Support

#### Option C1: Hardware TCP/IP with W5500

Use a W5500 SPI Ethernet chip which has a built-in hardware TCP/IP stack.

```
┌─────────────────────────────────────┐
│  Rust Application (RTIC/polling)    │
│  - PollingExecutor                  │
├─────────────────────────────────────┤
│  nano-ros-node                      │
│  - Uses W5500Transport              │
├─────────────────────────────────────┤
│  w5500 crate (Rust)                 │
│  - SPI driver for W5500             │
│  - Hardware sockets API             │
├─────────────────────────────────────┤
│  zenoh-pico (C) or pure-Rust        │
│  - Uses W5500 socket API            │
├─────────────────────────────────────┤
│  W5500 Chip (SPI)                   │
│  - Hardware TCP/IP offload          │
└─────────────────────────────────────┘
```

**Pros:**
- No software TCP/IP stack needed
- Works with bare-metal RTIC/polling
- Simple blocking socket API
- Widely available, inexpensive module

**Cons:**
- Requires additional hardware (W5500 module)
- Limited to 8 simultaneous sockets
- Lower performance than native Ethernet

**Work Items:**
- [ ] Add W5500 to NUCLEO-F429ZI via SPI
- [ ] Create W5500 transport backend
- [ ] Integrate zenoh-pico with W5500 sockets
- [ ] Test RTIC example with W5500

#### Option C2: smoltcp Software Stack (without Embassy)

Use smoltcp directly in RTIC/polling without Embassy's async runtime.

```
┌─────────────────────────────────────┐
│  Rust Application (RTIC/polling)    │
│  - PollingExecutor                  │
├─────────────────────────────────────┤
│  nano-ros-node                      │
│  - Uses SmoltcpTransport            │
├─────────────────────────────────────┤
│  smoltcp (polling mode)             │
│  - Manual poll() calls in main loop │
├─────────────────────────────────────┤
│  stm32-eth driver                   │
│  - Raw Ethernet frames              │
└─────────────────────────────────────┘
```

**Pros:**
- Pure Rust, no extra hardware
- Works with existing NUCLEO-F429ZI Ethernet

**Cons:**
- Complex manual polling integration
- Must poll both smoltcp and zenoh
- More memory usage than W5500

#### Option C3: Keep as API Demos (Current State)

Keep RTIC/polling examples as API pattern demonstrations without real networking.

**Pros:**
- No additional work required
- Clear documentation of the pattern

**Cons:**
- Cannot actually test embedded communication
- Less useful as reference implementations

### Recommendation

1. **Short-term:** Keep RTIC/polling as API demos, direct users to Zephyr/Embassy examples for working implementations
2. **Medium-term:** Implement W5500 support for simple hardware solution
3. **Long-term:** Pure-Rust zenoh client for cleanest architecture

---

## Phase D: Native Examples Verification

Ensure all native examples work correctly with the executor API.

### Work Items

#### D.1 Verify Native Examples

- [ ] Test native-talker with zenoh feature
- [ ] Test native-listener with zenoh feature
- [ ] Test native-service-server with zenoh feature
- [ ] Test native-service-client with zenoh feature
- [ ] Test native-action-server with zenoh feature
- [ ] Test native-action-client with zenoh feature
- [ ] Test talker ↔ listener communication
- [ ] Test service client ↔ server communication
- [ ] Test action client ↔ server communication

#### D.2 Test ROS 2 Interoperability

- [ ] Test native-talker → ROS 2 listener
- [ ] Test ROS 2 talker → native-listener
- [ ] Test service interop with ROS 2
- [ ] Document any compatibility issues

#### D.3 Update Example Documentation

- [ ] Update README for each example
- [ ] Add usage instructions with zenoh router
- [ ] Document ROS 2 interop testing commands

---

## Phase E: Cleanup and Documentation

### Work Items

#### E.1 Update RTIC/Polling Examples

- [ ] Update rtic-stm32f4 README to explain current limitations
- [ ] Update polling-stm32f4 README to explain current limitations
- [ ] Add clear guidance directing users to Zephyr/Embassy for networking

#### E.2 Update embedded-integration.md

- [ ] Document Zephyr integration (complete)
- [ ] Document Embassy + smoltcp option
- [ ] Document RTIC/polling limitations and alternatives
- [ ] Add hardware requirements matrix

#### E.3 Update CLAUDE.md

- [ ] Update examples section with current status
- [ ] Document Zephyr build workflow
- [ ] Document Embassy build workflow

#### E.4 CI/Build Scripts

- [ ] Add native example tests to CI
- [ ] Add Zephyr build check (if feasible)
- [ ] Update justfile with embedded commands

---

## Open Questions

1. **zenoh-pico platform layer complexity**: How much effort to bridge async smoltcp to sync zenoh-pico?

2. **Pure-Rust zenoh client**: Worth the significant effort for cleaner architecture?

3. **W5500 priority**: Should W5500 support be prioritized for simpler bare-metal networking?

4. **ESP32 support**: Should we add ESP-IDF/ESP32 examples with built-in WiFi?

---

## Priority Order

1. **Phase D** - Verify native examples (low effort, ensures current code works)
2. **Phase E** - Documentation cleanup (low effort, high value)
3. **Phase B** - Embassy + smoltcp (medium-high effort, pure-Rust solution)
4. **Phase C** - RTIC/polling options (evaluate based on user demand)

---

## Completed

- [x] Phase A: Zephyr examples with executor API and full networking
