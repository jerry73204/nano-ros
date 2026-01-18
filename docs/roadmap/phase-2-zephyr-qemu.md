# Phase 2: Transport, Interoperability & Zephyr Integration

**Status: COMPLETE ✅**

## Executive Summary

Phase 2 focuses on two main areas:
1. **ROS 2 Interoperability** - rmw_zenoh compatibility (COMPLETE)
2. **Zephyr Integration** - nano-ros on real RTOS (COMPLETE)

**Deployment Model:**
- ROS 2 nodes run on Linux host using `rmw_zenoh_cpp`
- nano-ros nodes run on Zephyr RTOS (embedded targets)
- Communication via zenoh router on host

---

## Phase 2A: ROS 2 Interoperability ✅ COMPLETE

### Summary

All ROS 2 interoperability features are implemented and tested. nano-ros can communicate bidirectionally with ROS 2 nodes using rmw_zenoh.

| Component | Status | Tests |
|-----------|--------|-------|
| `zenoh-pico-sys` | ✅ Complete | FFI bindings with static linking |
| `zenoh-pico` | ✅ Complete | Safe wrapper (Session, Publisher, Subscriber, Liveliness) |
| `nano-ros-transport` | ✅ Complete | ZenohTransport + RMW Attachment |
| `nano-ros-node` | ✅ Complete | ConnectedNode with transport integration |
| Native examples | ✅ Complete | talker/listener with zenoh transport |
| Integration tests | ✅ Complete | Full test suite in `tests/` |

### Acceptance Criteria (All Met)

- [x] RMW attachment included with published messages
- [x] Liveliness tokens declared for nodes/publishers/subscribers
- [x] Native examples use real transport
- [x] nano-ros talker → ROS 2 listener works
- [x] ROS 2 talker → nano-ros listener works
- [x] Full communication matrix tested (nano↔nano, nano↔ROS2, ROS2↔ROS2)

### Test Suite

Integration tests are organized in `tests/`:

```
tests/
├── run-all.sh              # Main test runner
├── common/
│   ├── utils.sh            # Shared utilities
│   └── prerequisites.sh    # Prerequisite checks
├── nano2nano/
│   └── run.sh              # nano-ros ↔ nano-ros tests
├── rmw-interop/
│   ├── nano2ros.sh         # nano-ros → ROS 2 tests
│   ├── ros2nano.sh         # ROS 2 → nano-ros tests
│   └── matrix.sh           # Full 4x4 communication matrix
└── rmw-detailed/
    ├── liveliness.sh       # Liveliness token format tests
    ├── keyexpr.sh          # Key expression format tests
    ├── qos.sh              # QoS compatibility tests
    └── attachment.sh       # RMW attachment format tests
```

Run tests:
```bash
./tests/run-all.sh              # Full suite
./tests/run-all.sh --quick      # Quick subset
./tests/run-all.sh rmw-interop  # Only RMW interop tests
```

See [docs/rmw_zenoh_interop.md](../rmw_zenoh_interop.md) for protocol details.

---

## Phase 2B: Zephyr Integration ✅ COMPLETE

### Goal

Run nano-ros on Zephyr RTOS, communicating with ROS 2 nodes on Linux host.

### Achievement Summary

nano-ros running on Zephyr native_sim successfully publishes messages via zenoh-pico
to a zenoh router, where they can be received by native Rust subscribers or ROS 2 nodes.

### Target Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Linux Host                               │
│  ┌─────────────────┐     ┌─────────────────┐                   │
│  │   ROS 2 Node    │     │   zenohd        │                   │
│  │  (rmw_zenoh)    │     │   (router)      │                   │
│  │                 │     │                 │                   │
│  │  ros2 topic pub │     │ tcp/0.0.0.0:7447│                   │
│  │  ros2 topic echo│     │                 │                   │
│  └────────┬────────┘     └────────┬────────┘                   │
│           │                       │                             │
│           └───────────┬───────────┘                             │
│                       │ zenoh protocol                          │
└───────────────────────┼─────────────────────────────────────────┘
                        │
          ┌─────────────┴─────────────┐
          │      Network (Ethernet)    │
          └─────────────┬─────────────┘
                        │
┌───────────────────────┼─────────────────────────────────────────┐
│                       │         Zephyr Device                   │
│  ┌────────────────────┴────────────────────┐                   │
│  │              zenoh-pico                  │                   │
│  │         (client mode → router)          │                   │
│  └────────────────────┬────────────────────┘                   │
│                       │                                         │
│  ┌────────────────────┴────────────────────┐                   │
│  │            nano-ros node                 │                   │
│  │                                          │                   │
│  │   Publisher: /sensor_data               │                   │
│  │   Subscriber: /commands                  │                   │
│  └──────────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────────┘
```

### Current Progress

| Item | Status | Notes |
|------|--------|-------|
| Core crates no_std | ✅ Complete | All crates build for `thumbv7em-none-eabihf` |
| QEMU semihosting tests | ✅ Complete | 9 tests validating core functionality |
| Zephyr workspace setup | ✅ Complete | `zephyr/setup.sh` (installs SDK, west, Rust targets) |
| zephyr-lang-rust | ✅ Complete | Integrated at `modules/lang/rust` |
| Zephyr talker (Rust) | ✅ Complete | `examples/zephyr-talker-rs/` with zenoh-pico transport |
| Zephyr listener (Rust) | ✅ Complete | `examples/zephyr-listener-rs/` with zenoh-pico transport |
| nano-ros CDR on Zephyr | ✅ Complete | Serialization/deserialization working |
| zenoh-pico integration | ✅ Complete | v1.5.1 via west manifest + C shim for FFI |
| TAP network setup | ✅ Complete | `scripts/setup-zephyr-network.sh` |
| End-to-end test | ✅ Complete | Zephyr → zenohd → native subscriber working |
| Hardware testing | ⏸️ Deferred | Phase 3 milestone |

### Quick Start: Building Zephyr Examples

```bash
# 1. Set up Zephyr workspace (one-time)
./zephyr/setup.sh

# 2. Source environment
source ~/zephyr-nano-ros/env.sh
# Or if using nano-ros-workspace:
source ~/nano-ros-workspace/.venv/bin/activate
export ZEPHYR_BASE=~/nano-ros-workspace/zephyr

# 3. Build talker for native_sim (64-bit, supports Rust)
cd ~/nano-ros-workspace  # or ~/zephyr-nano-ros
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs

# 4. Run (see E2E Test below for full networking test)
./build/zephyr/zephyr.exe
```

**Supported Boards for Rust:**
- `native_sim/native/64` - Linux host simulation (64-bit, best for development)
- Cortex-M boards (e.g., `nucleo_f429zi`) - future hardware validation

### End-to-End Test Procedure

This test verifies Zephyr talker → zenoh router → native subscriber communication.

**Prerequisites:**
- Zephyr workspace set up (`./zephyr/setup.sh`)
- zenohd installed and accessible
- Zephyr example built

**Step 1: Set up TAP networking (one-time, requires root)**
```bash
sudo ./scripts/setup-zephyr-network.sh
```
This creates a TAP interface `zeth` with:
- Host IP: 192.0.2.2
- Zephyr IP: 192.0.2.1 (configured in prj.conf)
- User ownership for non-root Zephyr execution

**Step 2: Start zenoh router**
```bash
zenohd --listen tcp/0.0.0.0:7447
```

**Step 3: Start native subscriber**
```bash
cd ~/repos/rusty-ros
cargo run -p zenoh-pico --example sub_test --features std
```

Expected output:
```
[sub] Connected to zenoh router
[sub] Waiting for messages on demo/chatter...
```

**Step 4: Run Zephyr talker**
```bash
cd ~/nano-ros-workspace
./build/zephyr/zephyr.exe
```

Expected output:
```
*** Booting Zephyr OS build v3.7.0 ***
<inf> net_config: IPv4 address: 192.0.2.1
<inf> rust: rustapp: nano-ros Zephyr Talker (Rust) Starting
<inf> rust: rustapp: Connecting to zenoh router at tcp/192.0.2.2:7447
<inf> rust: rustapp: Session opened
<inf> rust: rustapp: Publisher declared
<inf> rust: rustapp: [0] Published: data=0 (4 bytes)
<inf> rust: rustapp: [1] Published: data=1 (4 bytes)
...
```

**Step 5: Verify subscriber receives messages**
The native subscriber should show:
```
[sub] #0 Received Int32: 0 (4 bytes)
[sub] #1 Received Int32: 1 (4 bytes)
[sub] #2 Received Int32: 2 (4 bytes)
[sub] SUCCESS: received messages from Zephyr!
```

### Teardown

To remove the TAP interface:
```bash
sudo ./scripts/setup-zephyr-network.sh --down
```

### Work Items

#### 2B.1 Zephyr Rust Toolchain Setup ✅ COMPLETE
- [x] Create west manifest (`west.yml` at repo root)
- [x] Setup script (`zephyr/setup.sh`)
- [x] Configure zephyr-lang-rust module
- [x] Configure zenoh-pico module (v1.5.1)

**Reference:** https://github.com/zephyrproject-rtos/zephyr-lang-rust

#### 2B.2 Convert Zephyr Examples to Rust ✅ COMPLETE
- [x] Port `examples/zephyr-talker/` to Rust (`examples/zephyr-talker-rs/`)
- [x] Port `examples/zephyr-listener/` to Rust (`examples/zephyr-listener-rs/`)
- [x] Integrate nano-ros crates (CDR serialization)
- [x] C shim for zenoh-pico FFI (avoids struct size mismatch issues)
- [x] Kconfig options for zenoh-pico in prj.conf

#### 2B.3 Network Integration Testing ✅ COMPLETE
- [x] TAP networking script (`scripts/setup-zephyr-network.sh`)
- [x] Configure static IPs (Zephyr: 192.0.2.1, Host: 192.0.2.2)
- [x] User-owned TAP device (non-root Zephyr execution)
- [x] Native subscriber test (`crates/zenoh-pico/examples/sub_test.rs`)
- [x] End-to-end test verified: Zephyr → zenohd → native Rust

#### 2B.4 Hardware Validation ⏸️ DEFERRED TO PHASE 3
- [ ] Select target board (STM32F4, nRF52840, or similar)
- [ ] Flash and test on real hardware
- [ ] Verify Ethernet/WiFi connectivity to zenoh router
- [ ] Performance benchmarking (latency, throughput)

### Target Boards

| Board | MCU | Network | Priority |
|-------|-----|---------|----------|
| NUCLEO-F429ZI | STM32F429 | Ethernet | High |
| nRF52840-DK | nRF52840 | BLE/Thread | Medium |
| ESP32-DevKitC | ESP32 | WiFi | Medium |
| QEMU Cortex-M3 | Emulated | TAP/Bridge | Testing |

### Zephyr Configuration

Required Kconfig options for nano-ros:
```
# Networking
CONFIG_NETWORKING=y
CONFIG_NET_IPV4=y
CONFIG_NET_TCP=y
CONFIG_NET_SOCKETS=y

# Zenoh-pico requirements
CONFIG_POSIX_API=y
CONFIG_HEAP_MEM_POOL_SIZE=32768
CONFIG_MAIN_STACK_SIZE=4096

# For Rust (zephyr-lang-rust)
CONFIG_RUST=y
```

---

## Quick Reference

### Build Commands
```bash
just build              # Build all crates (no_std)
just build-embedded     # Build for thumbv7em-none-eabihf
just qemu-test          # Run QEMU semihosting tests
just test               # Run all tests (requires std)
just quality            # Format + clippy + tests
```

### Run Native Examples
```bash
# Start zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Run nano-ros talker
cargo run -p native-talker --release --features zenoh -- --tcp 127.0.0.1:7447

# Run nano-ros listener
cargo run -p native-listener --release --features zenoh -- --tcp 127.0.0.1:7447
```

### Test with ROS 2
```bash
# Terminal 1: Zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: nano-ros talker
cargo run -p native-talker --release --features zenoh -- --tcp 127.0.0.1:7447

# Terminal 3: ROS 2 listener
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

### Run Integration Tests
```bash
./tests/run-all.sh              # All tests
./tests/run-all.sh --quick      # Quick smoke test
./tests/run-all.sh rmw-detailed # Detailed protocol tests
```

---

## File Locations

| Component | Path |
|-----------|------|
| Core crates | `crates/` |
| Native examples | `examples/native-talker/`, `examples/native-listener/` |
| Zephyr Rust examples | `examples/zephyr-talker-rs/`, `examples/zephyr-listener-rs/` |
| Zephyr C stubs | `examples/zephyr-talker/`, `examples/zephyr-listener/` |
| QEMU test | `examples/qemu-test/` |
| Integration tests | `tests/` |
| Network setup | `scripts/setup-zephyr-network.sh` |
| West manifest | `west.yml` |
| Zephyr module config | `zephyr/module.yml` |
| Protocol docs | `docs/rmw_zenoh_interop.md` |
| Zenoh subscriber test | `crates/zenoh-pico/examples/sub_test.rs` |

---

## Implementation Notes

### zephyr-lang-rust Integration

The Zephyr Rust examples use `zephyr-lang-rust` module which provides:
- Rust build integration via `rust_cargo_application()` CMake function
- `zephyr` crate with Zephyr kernel bindings (time, logging, etc.)
- Automatic linking of Rust static library into Zephyr image

Key requirements for Rust Zephyr applications:
1. Package name must be `rustapp` in Cargo.toml
2. Entry point is `rust_main()` function (not `main`)
3. CMakeLists.txt must set `ZEPHYR_EXTRA_MODULES` before `find_package(Zephyr)`
4. CONFIG_RUST=y and CONFIG_RUST_ALLOC=y in prj.conf
5. Rust is only supported on: Cortex-M, RISC-V, or POSIX-64 platforms

### zenoh-pico Integration ✅ COMPLETE

zenoh-pico is integrated as a Zephyr module via west manifest:

**west.yml configuration:**
```yaml
- name: zenoh-pico
  remote: eclipse-zenoh
  revision: 1.5.1
  path: modules/lib/zenoh-pico
```

**prj.conf options:**
```
CONFIG_ZENOH_PICO=y
CONFIG_ZENOH_PICO_PUBLICATION=y
CONFIG_ZENOH_PICO_LINK_TCP=y
CONFIG_ZENOH_PICO_MULTI_THREAD=y
```

**C Shim Pattern:**
To avoid struct size mismatches between Rust FFI and zenoh-pico C structs,
a C shim layer is used (`examples/zephyr-talker-rs/src/zenoh_shim.c`):
```c
// C shim handles zenoh-pico types internally
int zenoh_init_config(const char *locator);
int zenoh_open_session(void);
int zenoh_declare_publisher(const char *keyexpr);
int zenoh_publish(const uint8_t *data, size_t len);
void zenoh_close(void);
```

The Rust code calls these simple C functions instead of using raw FFI bindings.

## References

- [ROS 2 rmw_zenoh Interop Analysis](../rmw_zenoh_interop.md)
- [zenoh-pico GitHub](https://github.com/eclipse-zenoh/zenoh-pico)
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh)
- [Zephyr Rust](https://github.com/zephyrproject-rtos/zephyr-lang-rust) - integrated at `modules/lang/rust`
- [Zephyr Networking](https://docs.zephyrproject.org/latest/connectivity/networking/index.html)
