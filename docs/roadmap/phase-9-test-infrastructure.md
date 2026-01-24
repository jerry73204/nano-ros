# Phase 9: Test Infrastructure Expansion

**Goal**: Expand test coverage for nano-ros across all platforms (POSIX, smoltcp, Zephyr), emulators (QEMU, native_sim), and ROS 2 interoperability scenarios. Migrate from shell scripts to a Rust-based test framework for better maintainability, type safety, and debugging.

**Status**: In Progress (Phase 9.1-9.3 Complete, 9.A Core Framework Complete)

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

## Phase 9.A: Rust Test Framework Migration (NEW)

**Status**: Core Complete (9.A.1, 9.A.2, 9.A.5, 9.A.6 done)
**Priority**: **Critical** (Foundation for all other tests)

Migrate from shell-based test orchestration to a Rust-based test framework for:
- Type safety and compile-time error checking
- Better error messages and stack traces
- RAII-based process management (automatic cleanup)
- IDE support (debugging, code navigation)
- Cross-platform compatibility (Windows support)
- Parallel test execution with proper isolation

### Recommended Crate Stack

| Crate         | Purpose                           | Version |
|---------------|-----------------------------------|---------|
| `rstest`      | Fixtures, parameterized tests     | 0.18+   |
| `duct`        | Process pipelines, IO redirection | 0.13+   |
| `assert_cmd`  | CLI binary testing                | 2.0+    |
| `predicates`  | Output assertions                 | 3.0+    |
| `test-binary` | Build helper binaries             | 3.0+    |
| `tempfile`    | Temporary directories             | 3.0+    |
| `port_check`  | Wait for TCP ports                | 0.2+    |

### Directory Structure

```
crates/
└── nano-ros-tests/              # Rust integration test crate (workspace member)
    ├── Cargo.toml               # Test crate with dependencies
    ├── src/
    │   ├── lib.rs               # Shared test utilities
    │   └── fixtures/
    │       ├── mod.rs           # Fixture module
    │       ├── zenohd.rs        # ZenohRouter fixture
    │       ├── qemu.rs          # QEMU process fixture
    │       ├── zephyr.rs        # Zephyr workspace fixture
    │       └── binaries.rs      # Binary build helpers
    └── tests/
        ├── emulator.rs          # QEMU/Zephyr tests
        ├── nano2nano.rs         # nano-ros ↔ nano-ros tests
        ├── rmw_interop.rs       # ROS 2 interop tests
        ├── platform.rs          # Platform backend tests
        └── executor.rs          # Executor API tests

tests/
├── run-all.sh                   # Shell runner (invokes cargo test)
├── README.md                    # Test documentation
├── emulator.sh                  # Invokes: cargo test -p nano-ros-tests emulator
├── nano2nano.sh                 # Invokes: cargo test -p nano-ros-tests nano2nano
├── rmw-interop.sh               # Invokes: cargo test -p nano-ros-tests rmw_interop
├── platform.sh                  # Invokes: cargo test -p nano-ros-tests platform
└── legacy/                      # Legacy shell-based tests (deprecated)
    ├── emulator/
    ├── platform/
    ├── smoltcp/
    └── ...
```

### Work Items

#### 9.A.1: Core Framework Setup
- [x] **9.A.1.1** Create `crates/nano-ros-tests/` crate with Cargo.toml
  ```toml
  [package]
  name = "nano-ros-tests"
  version = "0.1.0"
  edition = "2021"
  publish = false

  [dependencies]
  rstest = "0.18"
  duct = "0.13"
  assert_cmd = "2.0"
  predicates = "3.0"
  tempfile = "3.0"
  thiserror = "1.0"

  # Workspace dependencies for type access
  nano-ros-core = { path = "../nano-ros-core" }
  nano-ros-serdes = { path = "../nano-ros-serdes" }
  ```

- [x] **9.A.1.2** Add to workspace `Cargo.toml`
  ```toml
  [workspace]
  members = [
      # ... existing members
      "crates/nano-ros-tests",
  ]
  ```

- [x] **9.A.1.3** Implement `src/fixtures/zenohd.rs` - ZenohRouter fixture
  ```rust
  use duct::cmd;
  use rstest::fixture;

  pub struct ZenohRouter { handle: duct::Handle, port: u16 }

  impl ZenohRouter {
      pub fn start(port: u16) -> Result<Self> { ... }
      pub fn locator(&self) -> String { format!("tcp/127.0.0.1:{}", self.port) }
  }

  impl Drop for ZenohRouter {
      fn drop(&mut self) { let _ = self.handle.kill(); }
  }

  #[fixture]
  pub fn zenohd() -> ZenohRouter { ZenohRouter::start(7447).unwrap() }
  ```

- [x] **9.A.1.4** Implement `src/fixtures/qemu.rs` - QEMU process fixture
  ```rust
  pub struct QemuProcess { handle: duct::Handle, output_file: PathBuf }

  impl QemuProcess {
      pub fn start_cortex_m3(binary: &Path) -> Result<Self> { ... }
      pub fn wait_for_output(&self, timeout: Duration) -> Result<String> { ... }
  }

  impl Drop for QemuProcess {
      fn drop(&mut self) { let _ = self.handle.kill(); }
  }
  ```

- [x] **9.A.1.5** Implement `src/fixtures/binaries.rs` - Binary build helpers
  ```rust
  use duct::cmd;
  use std::sync::OnceLock;

  static QEMU_TEST_BINARY: OnceLock<PathBuf> = OnceLock::new();

  pub fn build_qemu_test() -> &'static Path {
      QEMU_TEST_BINARY.get_or_init(|| {
          cmd!("cargo", "build", "--release",
               "--target", "thumbv7m-none-eabi",
               "-p", "qemu-test")
              .run().expect("Failed to build qemu-test");
          PathBuf::from("examples/qemu-test/target/thumbv7m-none-eabi/release/qemu-test")
      })
  }
  ```

- [x] **9.A.1.6** Implement `src/lib.rs` - Shared utilities
  ```rust
  pub mod fixtures;

  use std::time::{Duration, Instant};
  use std::net::TcpStream;

  pub fn wait_for_port(port: u16, timeout: Duration) -> bool { ... }
  pub fn wait_for_output(output: &str, pattern: &str, timeout: Duration) -> bool { ... }
  ```

- [x] **9.A.1.7** Create shell wrapper scripts in `tests/`
  ```bash
  # tests/emulator.sh
  #!/bin/bash
  cargo test -p nano-ros-tests --test emulator -- "$@"
  ```

#### 9.A.2: Migrate Emulator Tests
- [x] **9.A.2.1** Create `crates/nano-ros-tests/tests/emulator.rs`
- [x] **9.A.2.2** Migrate `qemu-cortex-m3.sh` to Rust
  ```rust
  use nano_ros_tests::fixtures::{qemu_binary, QemuProcess};
  use rstest::rstest;

  #[rstest]
  fn test_qemu_cdr_serialization(qemu_binary: PathBuf) {
      let qemu = QemuProcess::start_cortex_m3(&qemu_binary).unwrap();
      let output = qemu.wait_for_output(Duration::from_secs(30)).unwrap();

      assert!(output.contains("[PASS] Int32 roundtrip"));
      assert!(output.contains("[PASS] Float64 roundtrip"));
      assert!(output.contains("All tests passed"));
  }
  ```
- [ ] **9.A.2.3** Migrate `zephyr-native-sim.sh` to Rust
- [ ] **9.A.2.4** Migrate `zephyr-qemu-arm.sh` to Rust
- [ ] **9.A.2.5** Create `tests/emulator.sh` shell wrapper

#### 9.A.3: Migrate nano2nano Tests
- [x] **9.A.3.1** Create `crates/nano-ros-tests/tests/nano2nano.rs`
- [x] **9.A.3.2** Implement pub/sub test with zenohd fixture
  ```rust
  use nano_ros_tests::fixtures::zenohd;
  use duct::cmd;
  use rstest::rstest;

  #[rstest]
  fn test_nano_pubsub(zenohd: ZenohRouter) {
      let _talker = cmd!("cargo", "run", "-p", "native-talker",
                         "--features", "zenoh", "--",
                         "--tcp", &zenohd.locator(), "--count", "5")
          .start().unwrap();

      let listener = cmd!("cargo", "run", "-p", "native-listener",
                          "--features", "zenoh", "--",
                          "--tcp", &zenohd.locator())
          .stdout_capture()
          .run().unwrap();

      assert!(String::from_utf8_lossy(&listener.stdout).contains("Received"));
  }
  ```
- [ ] **9.A.3.3** Implement peer mode test (no router)
- [ ] **9.A.3.4** Create `tests/nano2nano.sh` shell wrapper

#### 9.A.4: Migrate RMW Interop Tests
- [ ] **9.A.4.1** Create `crates/nano-ros-tests/tests/rmw_interop.rs`
- [ ] **9.A.4.2** Implement `src/fixtures/ros2.rs` - ROS 2 process helpers
- [ ] **9.A.4.3** Migrate nano→ROS2 test
- [ ] **9.A.4.4** Migrate ROS2→nano test
- [ ] **9.A.4.5** Implement matrix test with rstest parameterization
  ```rust
  use rstest::rstest;

  #[rstest]
  #[case("native-talker", "ros2_listener_cpp")]
  #[case("native-talker", "ros2_listener_py")]
  #[case("ros2_talker_cpp", "native-listener")]
  #[case("ros2_talker_py", "native-listener")]
  fn test_interop_matrix(
      zenohd: ZenohRouter,
      #[case] talker: &str,
      #[case] listener: &str,
  ) { ... }
  ```
- [ ] **9.A.4.6** Create `tests/rmw-interop.sh` shell wrapper

#### 9.A.5: Migrate Platform Tests
- [x] **9.A.5.1** Create `crates/nano-ros-tests/tests/platform.rs`
- [x] **9.A.5.2** Migrate POSIX platform tests
- [x] **9.A.5.3** Migrate smoltcp platform tests (already Rust unit tests)
- [x] **9.A.5.4** Migrate generic compile tests
- [x] **9.A.5.5** Create `tests/platform.sh` shell wrapper (via rust-tests.sh)

#### 9.A.6: CI Integration
- [x] **9.A.6.1** Update `justfile` with new test commands
  ```just
  # Run Rust integration tests
  test-integration:
      cargo test -p nano-ros-tests

  # Run specific test suite
  test-emulator:
      cargo test -p nano-ros-tests --test emulator

  # Run all tests (unit + integration)
  test-all:
      cargo nextest run --workspace
  ```
- [ ] **9.A.6.2** Update `tests/run-all.sh` to invoke Rust tests
  ```bash
  #!/bin/bash
  # Run all nano-ros integration tests
  cargo test -p nano-ros-tests "$@"
  ```
- [ ] **9.A.6.3** Update GitHub Actions workflow
- [ ] **9.A.6.4** Move legacy shell tests to `tests/legacy/`

### Benefits Over Shell Scripts

| Aspect          | Shell Scripts     | Rust Framework               |
|-----------------|-------------------|------------------------------|
| Type safety     | None              | Full compile-time checking   |
| Error handling  | `set -e`, fragile | `Result<T, E>`, stack traces |
| Process cleanup | Manual `pkill`    | RAII `Drop` trait            |
| Debugging       | `echo`, `set -x`  | IDE breakpoints, logs        |
| Parallelism     | Manual            | `cargo test` parallel        |
| Assertions      | `grep`, regex     | `predicates`, typed          |
| Cross-platform  | Bash-only         | Windows, macOS, Linux        |
| IDE support     | Limited           | Full (rust-analyzer)         |

### Acceptance Criteria
- [ ] All existing shell tests have Rust equivalents in `crates/nano-ros-tests/`
- [ ] Tests run with `cargo test -p nano-ros-tests`
- [ ] Process cleanup is automatic (no orphan processes)
- [ ] Test failures produce clear error messages with context
- [ ] CI runs Rust integration tests
- [ ] Shell wrappers in `tests/` invoke Rust tests
- [ ] Legacy shell tests moved to `tests/legacy/` (deprecated)

---

## Phase 9.1: Platform Backend Tests

**Status**: Complete
**Priority**: High (shell-based, migrate to Rust in 9.A)

Create tests that verify zenoh-pico-shim works correctly on each platform backend.

### Directory Structure

```
tests/platform/
├── run.sh               # Run all platform tests
├── README.md            # Platform test documentation
├── posix.sh             # POSIX platform (reference baseline)
├── smoltcp-sim.sh       # smoltcp platform (x86_64 simulation)
└── generic.sh           # Generic platform (compile-only, no network)
```

### Work Items

- [x] **9.1.1** Create `tests/platform/` directory structure
- [x] **9.1.2** Create `tests/platform/README.md` with platform test documentation
- [x] **9.1.3** Implement `tests/platform/posix.sh`
- [x] **9.1.4** Implement `tests/platform/smoltcp-sim.sh`
- [x] **9.1.5** Implement `tests/platform/generic.sh`
- [x] **9.1.6** Create `tests/platform/run.sh` orchestrator
- [x] **9.1.7** Update `tests/run-all.sh` to include platform tests

### Acceptance Criteria
- All platform tests pass on CI
- POSIX platform has full functional coverage
- smoltcp platform has unit-level coverage (allocator, clock, buffers)
- Generic platform compiles without errors

---

## Phase 9.2: smoltcp Integration Tests

**Status**: Complete
**Priority**: Medium (already Rust-based unit tests)

Tests for the smoltcp TCP/IP stack integration with zenoh-pico.

### Directory Structure

```
tests/smoltcp/
├── run.sh                   # Run all smoltcp tests
├── README.md                # smoltcp test documentation
├── allocator.sh             # Bump allocator tests
├── socket-buffers.sh        # Socket buffer management
├── clock-sync.sh            # Clock synchronization
└── poll-callback.sh         # Poll callback mechanism
```

### Work Items

- [x] **9.2.1** - **9.2.8** All complete (22 Rust unit tests)

### Implementation Notes

These tests are already implemented as Rust unit tests in `platform_smoltcp.rs`.
Shell scripts just invoke `cargo test`. Migration to 9.A framework is straightforward.

---

## Phase 9.3: Emulator Tests

**Status**: Complete
**Priority**: High (shell-based, migrate to Rust in 9.A)

QEMU-based tests for embedded targets without requiring physical hardware.

### Directory Structure

```
tests/emulator/
├── run.sh                   # Run all emulator tests
├── README.md                # Emulator test documentation
├── common/
│   └── qemu-utils.sh        # QEMU management utilities
├── qemu-cortex-m3.sh        # Cortex-M3 QEMU tests
├── zephyr-native-sim.sh     # Zephyr native_sim tests (enhanced)
└── zephyr-qemu-arm.sh       # Zephyr QEMU ARM tests
```

### Work Items

- [x] **9.3.1** - **9.3.9** All complete

---

## Phase 9.4: Embedded ROS 2 Interop Tests

**Status**: Not Started
**Priority**: Medium (implement in Rust framework from 9.A)

ROS 2 interoperability tests specifically for embedded platforms.
**Note**: Implement directly in Rust framework (Phase 9.A) instead of shell scripts.

### Work Items

- [ ] **9.4.1** Implement in `crates/nano-ros-tests/tests/rmw_interop.rs`
  - Zephyr talker → ROS 2 listener
  - ROS 2 talker → Zephyr listener
  - Full matrix with rstest parameterization

---

## Phase 9.5: Cross-Platform Communication Tests

**Status**: Not Started
**Priority**: Medium (implement in Rust framework from 9.A)

Tests for communication between nano-ros nodes running on different platforms.
**Note**: Implement directly in Rust framework (Phase 9.A) instead of shell scripts.

### Work Items

- [ ] **9.5.1** Implement in `crates/nano-ros-tests/tests/cross_platform.rs`
  - POSIX talker → Zephyr listener
  - Zephyr talker → POSIX listener
  - Multi-node scenarios

---

## Phase 9.6: Executor API Tests

**Status**: Not Started
**Priority**: Medium (implement in Rust framework from 9.A)

Tests for the executor API across different execution models.
**Note**: Implement directly in Rust framework (Phase 9.A) instead of shell scripts.

### Work Items

- [ ] **9.6.1** Implement in `crates/nano-ros-tests/tests/executor.rs`
  - ConnectedExecutor tests
  - ShimExecutor tests
  - API consistency verification

---

## Phase 9.7: Test Infrastructure Improvements

**Status**: Superseded by 9.A
**Priority**: Low (most items addressed by Rust migration)

Shell infrastructure improvements - largely superseded by Phase 9.A migration.

### Remaining Work Items (if keeping shell tests)

- [ ] **9.7.6** Add CI configuration for both shell and Rust tests

---

## Implementation Priority (Revised)

| Phase | Priority | Status | Notes |
|-------|----------|--------|-------|
| **9.A Rust Migration** | **Critical** | **Core Complete** | Foundation established |
| 9.1 Platform Backend | High | **Complete** | Migrated to Rust in 9.A |
| 9.2 smoltcp Integration | Medium | **Complete** | Already Rust unit tests |
| 9.3 Emulator Tests | High | **Complete** | Migrated to Rust in 9.A |
| 9.4 Embedded ROS 2 | Medium | Not Started | Implement in Rust directly |
| 9.5 Cross-Platform | Medium | Not Started | Implement in Rust directly |
| 9.6 Executor API | Medium | Not Started | Implement in Rust directly |
| 9.7 Infrastructure | Low | Superseded | Addressed by 9.A |

### Recommended Order

1. **9.A.1** - Core framework setup (fixtures, utilities)
2. **9.A.2** - Migrate emulator tests (highest value, validates framework)
3. **9.A.3** - Migrate nano2nano tests
4. **9.A.4** - Migrate RMW interop tests
5. **9.4-9.6** - Implement new tests directly in Rust

---

## Test Execution Matrix

| Test Suite | Shell | Rust | CI | QEMU | Zephyr |
|------------|:-----:|:----:|:--:|:----:|:------:|
| nano2nano | ✓ (legacy) | 9.A.3 | ✓ | - | - |
| rmw-interop | ✓ (legacy) | 9.A.4 | ✓* | - | - |
| rmw-detailed | ✓ (legacy) | 9.A.4 | ✓* | - | - |
| platform/* | ✓ (legacy) | 9.A.5 | ✓ | - | - |
| smoltcp/* | ✓ | ✓ (unit) | ✓ | - | - |
| emulator/* | ✓ (legacy) | 9.A.2 | ✓ | ✓ | ✓ |

*Requires ROS 2 in CI environment

---

## Prerequisites Summary

### Rust Test Framework (9.A)
- Rust toolchain (stable)
- Dev dependencies: rstest, duct, assert_cmd, predicates, tempfile

### Emulator Tests
- QEMU: `qemu-system-arm`
- Zephyr: west workspace, Zephyr SDK
- TAP interface: `sudo ./scripts/setup-zephyr-network.sh`

### ROS 2 Interop Tests
- ROS 2 Humble or Jazzy
- rmw_zenoh_cpp: `sudo apt install ros-humble-rmw-zenoh-cpp`

---

## Success Metrics (Revised)

- [ ] **Phase 9.A complete**: All tests migrated to `crates/nano-ros-tests/`
- [ ] Test failures produce clear, actionable error messages
- [ ] No orphan processes after test runs (RAII cleanup)
- [ ] Tests run with `cargo test -p nano-ros-tests`
- [ ] IDE debugging works for integration tests
- [ ] CI runs Rust integration tests with proper reporting
- [ ] Test execution time < 5 minutes for quick suite
- [ ] Cross-platform support (Linux, macOS, Windows where applicable)
