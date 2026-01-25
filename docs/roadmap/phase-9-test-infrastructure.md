# Phase 9: Test Infrastructure Expansion

**Goal**: Expand test coverage for nano-ros across all platforms (POSIX, smoltcp, Zephyr), emulators (QEMU, native_sim), and ROS 2 interoperability scenarios. Migrate from shell scripts to a Rust-based test framework for better maintainability, type safety, and debugging.

**Status**: In Progress (Phase 9.A Core Complete, 9.1-9.3 Complete)

## Overview

This phase extends the existing test infrastructure to cover:
- **Rust test framework** - Replace shell scripts with type-safe Rust integration tests
- **Platform backends** - Verify zenoh-pico-shim works on POSIX, smoltcp, and Zephyr
- **Emulator testing** - QEMU Cortex-M and Zephyr native_sim/QEMU
- **Cross-platform communication** - nano-ros nodes on different platforms communicating
- **Embedded ROS 2 interop** - Zephyr/smoltcp nodes talking to ROS 2
- **Executor API consistency** - Verify executor patterns work across all backends

**Dependencies**: Phase 8 (Embedded Networking) complete

---

## Phase 9.A: Rust Test Framework Migration

**Status**: Complete (core framework, Zephyr tests pending)
- ✅ 9.A.1: Core Framework Setup (complete)
- ✅ 9.A.2: Emulator Tests (QEMU complete, Zephyr pending)
- ✅ 9.A.3: nano2nano Tests (complete)
- ✅ 9.A.4: RMW Interop Tests (complete)
- ✅ 9.A.5: Platform Tests (complete)
- ✅ 9.A.6: CI Integration (justfile done, GitHub Actions pending)
- ✅ 9.A.7: Cleanup (complete - Zephyr kept as shell script)

**Priority**: **Critical** (Foundation for all other tests)

### Current Directory Structure

```
crates/
└── nano-ros-tests/              # Rust integration test crate
    ├── Cargo.toml
    ├── src/
    │   ├── lib.rs               # Shared utilities (wait_for_port, count_pattern, etc.)
    │   ├── process.rs           # ManagedProcess utility
    │   ├── qemu.rs              # QemuProcess utility
    │   ├── ros2.rs              # Ros2Process utility
    │   └── fixtures/
    │       ├── mod.rs           # Re-exports fixtures and utilities
    │       ├── binaries.rs      # #[fixture] qemu_binary, talker_binary, listener_binary
    │       └── zenohd_router.rs # #[fixture] zenohd, zenohd_unique
    └── tests/
        ├── emulator.rs          # QEMU Cortex-M3 tests
        ├── nano2nano.rs         # nano-ros ↔ nano-ros tests
        ├── rmw_interop.rs       # ROS 2 interop tests
        └── platform.rs          # Platform detection tests

tests/
├── README.md                    # Test documentation (Rust-only workflow)
├── rust-tests.sh                # Optional wrapper for nice output
├── zephyr/                      # Zephyr tests (shell-based, requires west + TAP)
│   └── run.sh
└── simple-workspace/            # Standalone build verification
```

### Remaining Work Items

#### 9.A.2: Emulator Tests - Zephyr
- [ ] **9.A.2.3** Create `ZephyrProcess` fixture in `src/fixtures/zephyr.rs`
  ```rust
  pub struct ZephyrProcess { handle: Child, platform: ZephyrPlatform }

  pub enum ZephyrPlatform { NativeSim, QemuArm }

  impl ZephyrProcess {
      pub fn start(binary: &Path, platform: ZephyrPlatform) -> Result<Self>;
      pub fn wait_for_output(&mut self, timeout: Duration) -> Result<String>;
  }
  ```
- [ ] **9.A.2.4** Migrate `zephyr-native-sim` tests to `tests/zephyr.rs`
  ```rust
  #[rstest]
  fn test_zephyr_native_sim_talker(zenohd_unique: ZenohRouter) {
      // Requires: west workspace, TAP network
      let zephyr = ZephyrProcess::start(&zephyr_binary, ZephyrPlatform::NativeSim)?;
      // ...
  }
  ```
- [ ] **9.A.2.5** Add `is_zephyr_available()` check to skip tests when west not configured

#### 9.A.3: nano2nano Tests - Peer Mode
- [x] **9.A.3.3** Implement peer mode test (no router) in `nano2nano.rs`
  - Added `ZENOH_MODE` and `ZENOH_LOCATOR` env var support to `Context::from_env()`
  - Test uses `spawn_command()` with environment variables
  - Gracefully handles systems without multicast support
  ```rust
  #[rstest]
  fn test_peer_mode_communication(talker_binary: PathBuf, listener_binary: PathBuf) {
      // Start listener in peer mode via environment variable
      let mut listener_cmd = Command::new(&listener_binary);
      listener_cmd.env("ZENOH_MODE", "peer");
      let mut listener = ManagedProcess::spawn_command(listener_cmd, "listener")?;

      // Start talker in peer mode
      let mut talker_cmd = Command::new(&talker_binary);
      talker_cmd.env("ZENOH_MODE", "peer");
      let mut talker = ManagedProcess::spawn_command(talker_cmd, "talker")?;

      // Verify communication without zenohd router
  }
  ```

#### 9.A.6: CI Integration
- [ ] **9.A.6.2** Create `.github/workflows/integration-tests.yml`
  ```yaml
  name: Integration Tests
  on: [push, pull_request]
  jobs:
    rust-tests:
      runs-on: ubuntu-latest
      steps:
        - uses: actions/checkout@v4
        - name: Install zenohd
          run: cargo install zenoh --features=zenohd
        - name: Run integration tests
          run: cargo test -p nano-ros-tests --tests

    ros2-interop:
      runs-on: ubuntu-latest
      container: ros:humble
      steps:
        - uses: actions/checkout@v4
        - name: Install rmw_zenoh
          run: apt-get install -y ros-humble-rmw-zenoh-cpp
        - name: Run RMW interop tests
          run: cargo test -p nano-ros-tests --test rmw_interop
  ```

---

## Phase 9.4: Embedded ROS 2 Interop Tests

**Status**: Not Started
**Priority**: Medium

ROS 2 interoperability tests for embedded platforms (Zephyr).

### Work Items

- [ ] **9.4.1** Add Zephyr → ROS 2 test cases to `tests/rmw_interop.rs`
  ```rust
  #[derive(Debug, Clone, Copy)]
  enum Direction {
      // ... existing
      ZephyrToRos2,
      Ros2ToZephyr,
  }

  #[rstest]
  #[case(Direction::ZephyrToRos2)]
  #[case(Direction::Ros2ToZephyr)]
  fn test_zephyr_ros2_interop(
      zenohd_unique: ZenohRouter,
      #[case] direction: Direction,
  ) {
      if !require_ros2() || !require_zephyr() { return; }
      // ...
  }
  ```

- [ ] **9.4.2** Create `ZephyrTalker` and `ZephyrListener` fixtures
  ```rust
  #[fixture]
  pub fn zephyr_talker_binary() -> PathBuf {
      build_zephyr_example("zephyr-talker-rs", ZephyrPlatform::NativeSim)
  }
  ```

- [ ] **9.4.3** Add QoS compatibility tests between Zephyr and ROS 2

---

## Phase 9.5: Cross-Platform Communication Tests

**Status**: Not Started
**Priority**: Medium

Tests for communication between nano-ros nodes on different platforms.

### Work Items

- [ ] **9.5.1** Create `tests/cross_platform.rs`
  ```rust
  use nano_ros_tests::fixtures::{
      zenohd_unique, talker_binary, listener_binary,
      zephyr_talker_binary, zephyr_listener_binary,
      ZenohRouter, ManagedProcess, ZephyrProcess,
  };

  #[derive(Debug, Clone, Copy)]
  enum Platform { Native, Zephyr }

  #[rstest]
  #[case(Platform::Native, Platform::Zephyr)]
  #[case(Platform::Zephyr, Platform::Native)]
  fn test_cross_platform_pubsub(
      zenohd_unique: ZenohRouter,
      #[case] talker_platform: Platform,
      #[case] listener_platform: Platform,
  ) { ... }
  ```

- [ ] **9.5.2** Add multi-node scenarios
  ```rust
  #[rstest]
  fn test_multi_node_mixed_platforms(zenohd_unique: ZenohRouter) {
      // 2 native talkers + 1 Zephyr listener
      // Verify all messages received
  }
  ```

- [ ] **9.5.3** Add latency measurement tests
  ```rust
  #[rstest]
  fn test_cross_platform_latency(zenohd_unique: ZenohRouter) {
      // Measure round-trip time between platforms
      // Assert latency < threshold
  }
  ```

---

## Phase 9.6: Executor API Tests

**Status**: Not Started
**Priority**: Medium

Tests for executor API consistency across execution models.

### Work Items

- [ ] **9.6.1** Create `tests/executor.rs`
  ```rust
  //! Executor API tests
  //!
  //! Verifies executor patterns work consistently across backends.

  use nano_ros_tests::fixtures::{zenohd_unique, ZenohRouter};
  use rstest::rstest;

  #[rstest]
  fn test_connected_executor_spin(zenohd_unique: ZenohRouter) {
      // Test ConnectedExecutor::spin_once()
      // Test ConnectedExecutor::spin()
  }

  #[rstest]
  fn test_shim_executor_poll(zenohd_unique: ZenohRouter) {
      // Test ShimExecutor polling pattern
  }
  ```

- [ ] **9.6.2** Add callback ordering tests
  ```rust
  #[rstest]
  fn test_callback_execution_order(zenohd_unique: ZenohRouter) {
      // Publish multiple messages
      // Verify callbacks execute in order
  }
  ```

- [ ] **9.6.3** Add timer callback tests
  ```rust
  #[rstest]
  fn test_timer_callbacks(zenohd_unique: ZenohRouter) {
      // Create timer with callback
      // Verify callback fires at expected intervals
  }
  ```

- [ ] **9.6.4** Add service callback tests
  ```rust
  #[rstest]
  fn test_service_callbacks(zenohd_unique: ZenohRouter) {
      // Create service server
      // Send request, verify response callback
  }
  ```

---

## Implementation Priority (Revised)

| Phase | Priority | Status | Notes |
|-------|----------|--------|-------|
| **9.A Rust Migration** | **Critical** | **Complete** | Framework established |
| 9.1 Platform Backend | High | **Complete** | Migrated to `platform.rs` |
| 9.2 smoltcp Integration | Medium | **Complete** | Rust unit tests in crate |
| 9.3 Emulator Tests | High | **Partial** | QEMU done, Zephyr pending |
| 9.4 Embedded ROS 2 | Medium | Not Started | Implement in `rmw_interop.rs` |
| 9.5 Cross-Platform | Medium | Not Started | Create `cross_platform.rs` |
| 9.6 Executor API | Medium | Not Started | Create `executor.rs` |

### Recommended Order

1. ~~**9.A** - Core framework (complete)~~
2. **9.A.2.3-5** - Zephyr fixture and tests
3. **9.A.3.3** - Peer mode test
4. **9.A.6.2** - GitHub Actions CI
5. **9.4** - Embedded ROS 2 interop (extends `rmw_interop.rs`)
6. **9.5** - Cross-platform tests (new `cross_platform.rs`)
7. **9.6** - Executor API tests (new `executor.rs`)

---

## Test Execution Commands

```bash
# Run all integration tests
cargo test -p nano-ros-tests --tests
just test-rust

# Run specific test suite
just test-rust-emulator      # QEMU Cortex-M3 tests
just test-rust-nano2nano     # nano-ros ↔ nano-ros tests
just test-rust-rmw-interop   # ROS 2 interop tests
just test-rust-platform      # Platform detection tests

# Run with nice output wrapper
./tests/rust-tests.sh all
./tests/rust-tests.sh emulator
./tests/rust-tests.sh rmw_interop

# Run Zephyr tests (requires west workspace + TAP network)
just test-zephyr
```

---

## Test Fixtures Reference

| Fixture | Type | Description |
|---------|------|-------------|
| `zenohd` | `ZenohRouter` | Zenohd on port 7447 |
| `zenohd_unique` | `ZenohRouter` | Zenohd on unique port (parallel-safe) |
| `qemu_binary` | `PathBuf` | Built qemu-test binary |
| `talker_binary` | `PathBuf` | Built native-talker binary |
| `listener_binary` | `PathBuf` | Built native-listener binary |

### Utility Structs

| Struct | Module | Description |
|--------|--------|-------------|
| `ManagedProcess` | `process` | Generic process wrapper with RAII cleanup |
| `QemuProcess` | `qemu` | QEMU Cortex-M3 emulator wrapper |
| `Ros2Process` | `ros2` | ROS 2 command wrapper |
| `ZenohRouter` | `fixtures::zenohd_router` | Managed zenohd process |

### Utility Functions

| Function | Module | Description |
|----------|--------|-------------|
| `is_zenohd_available()` | `process` | Check if zenohd is in PATH |
| `require_zenohd()` | `process` | Skip test if zenohd unavailable |
| `is_qemu_available()` | `qemu` | Check if QEMU ARM is installed |
| `is_ros2_available()` | `ros2` | Check if ROS 2 is sourced |
| `is_rmw_zenoh_available()` | `ros2` | Check if rmw_zenoh_cpp installed |
| `wait_for_port()` | `lib` | Wait for TCP port to be available |
| `count_pattern()` | `lib` | Count pattern occurrences in output |

---

## Prerequisites Summary

### Rust Test Framework
- Rust toolchain (stable)
- Dependencies: rstest, duct, thiserror, once_cell

### Emulator Tests
- QEMU: `qemu-system-arm`
- Zephyr: west workspace, Zephyr SDK
- TAP interface: `sudo ./scripts/setup-zephyr-network.sh`

### ROS 2 Interop Tests
- ROS 2 Humble or Jazzy
- rmw_zenoh_cpp: `sudo apt install ros-humble-rmw-zenoh-cpp`

---

## Success Metrics

- [x] Core Rust test framework established
- [x] QEMU emulator tests migrated to Rust
- [x] nano2nano tests migrated to Rust
- [x] RMW interop tests migrated to Rust
- [x] Platform detection tests migrated to Rust
- [x] Legacy shell scripts cleaned up
- [ ] Zephyr tests migrated to Rust
- [x] Peer mode tests implemented
- [ ] GitHub Actions CI configured
- [ ] Cross-platform tests implemented
- [ ] Executor API tests implemented
