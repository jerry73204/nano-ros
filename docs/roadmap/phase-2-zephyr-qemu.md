# Phase 2: Transport, Interoperability & Zephyr Integration

**Status: Phase 2A COMPLETE, Phase 2B IN PROGRESS**

## Executive Summary

Phase 2 focuses on two main areas:
1. **ROS 2 Interoperability** - rmw_zenoh compatibility (COMPLETE)
2. **Zephyr Integration** - nano-ros on real RTOS (IN PROGRESS)

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

## Phase 2B: Zephyr Integration 🔄 IN PROGRESS

### Goal

Run nano-ros on Zephyr RTOS, communicating with ROS 2 nodes on Linux host.

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
| Zephyr workspace setup | ✅ Complete | `zephyr/setup.sh` + `west.yml` manifest |
| Zephyr talker (Rust) | ✅ Complete | `examples/zephyr-talker-rs/` |
| Zephyr listener (Rust) | ✅ Complete | `examples/zephyr-listener-rs/` |
| QEMU network scripts | ✅ Complete | `scripts/qemu/` with TAP/bridge setup |
| Integration test script | ✅ Complete | `tests/zephyr/run.sh` |
| Hardware testing | ⏸️ Skipped | Focus on QEMU for now |

### Work Items

#### 2B.1 Zephyr Rust Toolchain Setup ✅ COMPLETE
- [x] Create west manifest (`zephyr/west.yml`)
- [x] Setup script (`zephyr/setup.sh`)
- [x] Configure zephyr-lang-rust module
- [x] Configure zenoh-pico module

**Reference:** https://github.com/zephyrproject-rtos/zephyr-lang-rust

#### 2B.2 Convert Zephyr Examples to Rust ✅ COMPLETE
- [x] Port `examples/zephyr-talker/` to Rust (`examples/zephyr-talker-rs/`)
- [x] Port `examples/zephyr-listener/` to Rust (`examples/zephyr-listener-rs/`)
- [x] Integrate nano-ros crates (CDR serialization)
- [x] Configure zenoh-pico FFI bindings for Zephyr

#### 2B.3 QEMU Integration Testing ✅ COMPLETE
- [x] QEMU networking scripts (`scripts/qemu/setup-qemu-network.sh`)
- [x] Configure static IPs (talker: 192.0.2.1, listener: 192.0.2.3)
- [x] Test script for Zephyr ↔ ROS 2 (`tests/zephyr/run.sh`)
- [x] Integrated into test runner (`./tests/run-all.sh zephyr`)

#### 2B.4 Hardware Validation ⏸️ DEFERRED
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
| Zephyr stubs | `examples/zephyr-talker/`, `examples/zephyr-listener/` |
| QEMU test | `examples/qemu-test/` |
| Integration tests | `tests/` |
| QEMU scripts | `scripts/qemu/` |
| Protocol docs | `docs/rmw_zenoh_interop.md` |

---

## References

- [ROS 2 rmw_zenoh Interop Analysis](../rmw_zenoh_interop.md)
- [zenoh-pico GitHub](https://github.com/eclipse-zenoh/zenoh-pico)
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh)
- [Zephyr Rust](https://github.com/zephyrproject-rtos/zephyr-lang-rust)
- [Zephyr Networking](https://docs.zephyrproject.org/latest/connectivity/networking/index.html)
