# Phase 9: Test Infrastructure Expansion

**Goal**: Expand test coverage for nano-ros across all platforms (POSIX, smoltcp, Zephyr), emulators (QEMU, native_sim), and ROS 2 interoperability scenarios.

**Status**: In Progress (Phase 9.1-9.3 Complete)

## Overview

This phase extends the existing test infrastructure to cover:
- **Platform backends** - Verify zenoh-pico-shim works on POSIX, smoltcp, and Zephyr
- **Emulator testing** - QEMU Cortex-M and Zephyr native_sim/QEMU
- **Cross-platform communication** - nano-ros nodes on different platforms communicating
- **Embedded ROS 2 interop** - Zephyr/smoltcp nodes talking to ROS 2
- **Executor API consistency** - Verify executor patterns work across all backends

**Dependencies**: Phase 8 (Embedded Networking) complete

---

## Current Test Structure

```
tests/
├── run-all.sh           # Main orchestrator
├── common/              # Shared utilities (tmpfile, cleanup, prerequisites)
├── nano2nano/           # nano-ros ↔ nano-ros (native, posix)
├── rmw-interop/         # ROS 2 rmw_zenoh interop (native)
├── rmw-detailed/        # Protocol-specific tests (liveliness, keyexpr, qos, attachment)
└── zephyr/              # Basic Zephyr native_sim test
```

---

## Phase 9.1: Platform Backend Tests

**Status**: Complete
**Priority**: High

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
  - Session lifecycle (open, close, reconnect)
  - Publisher declaration and publishing
  - Subscriber declaration and callback invocation
  - Liveliness token declaration
  - ZenohId retrieval and formatting
  - Queryable declaration (service server pattern)
  - Run zenoh-pico-shim integration tests with `--features posix`

- [x] **9.1.4** Implement `tests/platform/smoltcp-sim.sh`
  - Compile-check smoltcp platform for x86_64
  - Test bump allocator (`smoltcp_alloc`, `smoltcp_free`)
  - Test clock functions (`smoltcp_set_clock_ms`, `smoltcp_clock_now_ms`)
  - Test socket buffer management (`smoltcp_socket_push_rx`, `smoltcp_socket_pop_tx`)
  - Note: Full network test requires hardware (Phase 8.9)

- [x] **9.1.5** Implement `tests/platform/generic.sh`
  - Verify zenoh-pico-shim compiles without any platform backend
  - Compile-only test (no runtime execution)
  - Useful for CI to catch API breakage

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
**Priority**: Medium

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

- [x] **9.2.1** Create `tests/smoltcp/` directory structure
- [x] **9.2.2** Create `tests/smoltcp/README.md`
- [x] **9.2.3** Implement `tests/smoltcp/allocator.sh`
  - Test allocation/deallocation patterns
  - Test realloc behavior (note: bump allocator doesn't copy data)
  - Test allocation alignment (8-byte aligned)
  - Test free operation (no-op for bump allocator)

- [x] **9.2.4** Implement `tests/smoltcp/socket-buffers.sh`
  - Test socket open/close lifecycle
  - Test multi-socket management (up to 4 sockets)
  - Test RX buffer push (`smoltcp_socket_push_rx`)
  - Test TX buffer pop (`smoltcp_socket_pop_tx`)
  - Test invalid handle error handling
  - Test connected flag management

- [x] **9.2.5** Implement `tests/smoltcp/clock-sync.sh`
  - Test `smoltcp_set_clock_ms()` updates
  - Test `smoltcp_clock_now_ms()` returns correct value
  - Test large value handling (near u64::MAX)
  - Test RNG (xorshift32) returns varying values

- [x] **9.2.6** Implement `tests/smoltcp/poll-callback.sh`
  - Test poll callback registration
  - Test poll callback invocation on poll()
  - Test no-callback safety (null function pointer)

- [x] **9.2.7** Create `tests/smoltcp/run.sh` orchestrator
- [x] **9.2.8** Update `tests/run-all.sh` to include smoltcp tests

### Implementation Notes

These tests run on x86_64 Linux using the smoltcp platform layer compiled for the host.
They test the Rust FFI functions that C code calls, not actual network communication.
Tests are implemented as Rust unit tests in `platform_smoltcp.rs` and run via shell scripts.

**Bump Allocator Limitation**: The bump allocator does not track allocation sizes,
so `smoltcp_realloc()` does NOT copy data from the old pointer. This is documented
and callers must handle data copying if needed.

**Test Execution**: Tests must run single-threaded due to global state:
```bash
cargo test -p zenoh-pico-shim-sys --features smoltcp -- --test-threads=1
```

### Acceptance Criteria
- [x] Allocator handles typical zenoh-pico allocation patterns (22 tests passing)
- [x] Socket buffers correctly manage up to 4 concurrent sockets
- [x] Clock synchronization works correctly
- [x] Poll callback mechanism functions as expected
- [x] All tests pass in CI (no external dependencies required)

---

## Phase 9.3: Emulator Tests

**Status**: Complete
**Priority**: High

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

- [x] **9.3.1** Create `tests/emulator/` directory structure
- [x] **9.3.2** Create `tests/emulator/README.md`
- [x] **9.3.3** Create `tests/emulator/common/qemu-utils.sh`
  - QEMU process management (start, stop, wait)
  - Virtual network setup (user-mode networking, TAP)
  - Semihosting output capture
  - Timeout handling for QEMU processes

- [x] **9.3.4** Implement `tests/emulator/qemu-cortex-m3.sh`
  - Based on existing `examples/qemu-test/`
  - Test CDR serialization correctness
  - Test type metadata generation
  - Test Node API (create node, publisher, subscriber)
  - Test memory allocation patterns
  - Capture semihosting output for verification

- [x] **9.3.5** Implement `tests/emulator/zephyr-native-sim.sh` (enhanced)
  - Migrate and enhance existing `tests/zephyr/run.sh`
  - Test: Zephyr talker → native subscriber
  - Test: Native talker → Zephyr listener
  - Test: Bidirectional communication
  - Test: Multiple message types
  - Test: Reconnection after disconnect

- [x] **9.3.6** Implement `tests/emulator/zephyr-qemu-arm.sh`
  - Configure QEMU ARM target (qemu_cortex_m3)
  - Set up virtual networking (SLIRP or TAP)
  - Test: Zephyr talker in QEMU → host subscriber
  - Test: Host talker → Zephyr listener in QEMU

- [x] **9.3.7** Create `tests/emulator/run.sh` orchestrator
- [x] **9.3.8** Update `tests/run-all.sh` to include emulator tests
- [x] **9.3.9** Document QEMU setup requirements

### Prerequisites

| Test | Requirements |
|------|--------------|
| qemu-cortex-m3.sh | `qemu-system-arm`, ARM GCC toolchain |
| zephyr-native-sim.sh | Zephyr west workspace, TAP interface |
| zephyr-qemu-arm.sh | Zephyr west workspace, `qemu-system-arm` |

### Acceptance Criteria
- QEMU Cortex-M3 tests pass without hardware
- Zephyr native_sim tests cover all communication patterns
- Zephyr QEMU ARM tests demonstrate embedded execution
- All emulator tests can run in CI (headless)

---

## Phase 9.4: Embedded ROS 2 Interop Tests

**Status**: Not Started
**Priority**: High

ROS 2 interoperability tests specifically for embedded platforms.

### Directory Structure

```
tests/rmw-interop-embedded/
├── run.sh                   # Run all embedded interop tests
├── README.md                # Embedded interop documentation
├── zephyr-to-ros2.sh        # Zephyr talker → ROS 2 listener
├── ros2-to-zephyr.sh        # ROS 2 talker → Zephyr listener
└── matrix-embedded.sh       # All embedded ↔ ROS 2 combinations
```

### Work Items

- [ ] **9.4.1** Create `tests/rmw-interop-embedded/` directory structure
- [ ] **9.4.2** Create `tests/rmw-interop-embedded/README.md`
- [ ] **9.4.3** Implement `tests/rmw-interop-embedded/zephyr-to-ros2.sh`
  - Start zenoh router on host
  - Start Zephyr talker (native_sim)
  - Start ROS 2 subscriber (rmw_zenoh_cpp)
  - Verify ROS 2 receives messages from Zephyr
  - Test with Int32, String message types

- [ ] **9.4.4** Implement `tests/rmw-interop-embedded/ros2-to-zephyr.sh`
  - Start zenoh router on host
  - Start Zephyr listener (native_sim)
  - Start ROS 2 publisher (rmw_zenoh_cpp)
  - Verify Zephyr receives messages from ROS 2
  - Test with Int32, String message types

- [ ] **9.4.5** Implement `tests/rmw-interop-embedded/matrix-embedded.sh`
  - Test all combinations:
    - Zephyr talker → ROS 2 C++ listener
    - Zephyr talker → ROS 2 Python listener
    - ROS 2 C++ talker → Zephyr listener
    - ROS 2 Python talker → Zephyr listener
  - Generate compatibility matrix report

- [ ] **9.4.6** Create `tests/rmw-interop-embedded/run.sh` orchestrator
- [ ] **9.4.7** Update `tests/run-all.sh` to include embedded interop tests

### Prerequisites

- ROS 2 Humble (or Jazzy) installed
- rmw_zenoh_cpp installed (`ros-humble-rmw-zenoh-cpp`)
- Zephyr west workspace configured
- TAP interface for Zephyr networking

### Acceptance Criteria
- Zephyr nodes can publish to ROS 2 subscribers
- ROS 2 nodes can publish to Zephyr subscribers
- All message types tested (Int32, String)
- Compatibility matrix documented

---

## Phase 9.5: Cross-Platform Communication Tests

**Status**: Not Started
**Priority**: Medium

Tests for communication between nano-ros nodes running on different platforms.

### Directory Structure

```
tests/cross-platform/
├── run.sh                   # Run all cross-platform tests
├── README.md                # Cross-platform test documentation
├── posix-to-zephyr.sh       # Native talker → Zephyr listener
├── zephyr-to-posix.sh       # Zephyr talker → Native listener
└── multi-node.sh            # Multiple nodes across platforms
```

### Work Items

- [ ] **9.5.1** Create `tests/cross-platform/` directory structure
- [ ] **9.5.2** Create `tests/cross-platform/README.md`
- [ ] **9.5.3** Implement `tests/cross-platform/posix-to-zephyr.sh`
  - Native talker (POSIX) publishes
  - Zephyr listener (native_sim) subscribes
  - Verify message delivery and data integrity

- [ ] **9.5.4** Implement `tests/cross-platform/zephyr-to-posix.sh`
  - Zephyr talker (native_sim) publishes
  - Native listener (POSIX) subscribes
  - Verify message delivery and data integrity

- [ ] **9.5.5** Implement `tests/cross-platform/multi-node.sh`
  - Multiple publishers (mixed platforms)
  - Multiple subscribers (mixed platforms)
  - Verify all nodes receive expected messages

- [ ] **9.5.6** Create `tests/cross-platform/run.sh` orchestrator
- [ ] **9.5.7** Update `tests/run-all.sh` to include cross-platform tests

### Acceptance Criteria
- POSIX and Zephyr nodes can communicate bidirectionally
- Multi-node scenarios work correctly
- No platform-specific message format issues

---

## Phase 9.6: Executor API Tests

**Status**: Not Started
**Priority**: Medium

Tests for the executor API across different execution models.

### Directory Structure

```
tests/executor/
├── run.sh                       # Run all executor tests
├── README.md                    # Executor test documentation
├── connected-executor.sh        # ConnectedExecutor (std/threaded)
├── shim-executor.sh             # ShimExecutor (polling)
├── api-consistency.sh           # API consistency across executors
└── compile-check/
    ├── rtic-patterns.rs         # RTIC task patterns (compile-only)
    └── embassy-patterns.rs      # Embassy async patterns (compile-only)
```

### Work Items

- [ ] **9.6.1** Create `tests/executor/` directory structure
- [ ] **9.6.2** Create `tests/executor/README.md`
- [ ] **9.6.3** Implement `tests/executor/connected-executor.sh`
  - Test `ConnectedExecutor::new()` with various locators
  - Test `create_node()` and node lifecycle
  - Test `spin()` and `spin_once()`
  - Test graceful shutdown

- [ ] **9.6.4** Implement `tests/executor/shim-executor.sh`
  - Test `ShimExecutor::new()` with various locators
  - Test `create_node()` and node lifecycle
  - Test `spin_once()` polling behavior
  - Test `poll()` for manual control

- [ ] **9.6.5** Implement `tests/executor/api-consistency.sh`
  - Verify both executors have same public API
  - Test identical usage patterns work on both
  - Document any platform-specific differences

- [ ] **9.6.6** Create `tests/executor/compile-check/rtic-patterns.rs`
  - Compile-only test for RTIC integration patterns
  - Verify `ShimExecutor` works in RTIC context

- [ ] **9.6.7** Create `tests/executor/compile-check/embassy-patterns.rs`
  - Compile-only test for Embassy integration patterns
  - Verify async executor patterns compile

- [ ] **9.6.8** Create `tests/executor/run.sh` orchestrator
- [ ] **9.6.9** Update `tests/run-all.sh` to include executor tests

### Acceptance Criteria
- Both executor types pass same functional tests
- API is consistent across executors
- RTIC and Embassy patterns compile correctly

---

## Phase 9.7: Test Infrastructure Improvements

**Status**: Not Started
**Priority**: Low

General improvements to test infrastructure.

### Work Items

- [ ] **9.7.1** Update `tests/common/utils.sh`
  - Add `wait_for_port()` function (wait for TCP port to be listening)
  - Add `check_process_running()` function
  - Add `capture_logs()` function for better debugging
  - Add `run_with_timeout()` wrapper

- [ ] **9.7.2** Update `tests/common/prerequisites.sh`
  - Add `check_qemu_prerequisites()` function
  - Add `check_zephyr_workspace()` function
  - Add `check_ros2_rmw_zenoh()` function
  - Improve error messages with installation instructions

- [ ] **9.7.3** Create `tests/common/qemu-utils.sh`
  - `start_qemu_cortex_m3()` - Start QEMU with Cortex-M3
  - `start_qemu_with_network()` - QEMU with virtual networking
  - `wait_for_qemu_ready()` - Wait for QEMU to initialize
  - `stop_qemu()` - Clean shutdown
  - `capture_qemu_output()` - Capture semihosting output

- [ ] **9.7.4** Update `tests/run-all.sh`
  - Add support for new test suites
  - Add `--platform` filter option
  - Add `--emulator` filter option
  - Add `--embedded` filter option
  - Improve summary report with test categories

- [ ] **9.7.5** Create `tests/README.md` update
  - Document all test suites
  - Document prerequisites for each category
  - Document environment variables
  - Add troubleshooting section

- [ ] **9.7.6** Add CI configuration
  - GitHub Actions workflow for test suites
  - Matrix testing for different platforms
  - Artifact collection for test logs

### Acceptance Criteria
- Test utilities are comprehensive and well-documented
- `run-all.sh` supports filtering by category
- CI runs appropriate tests for each platform
- Test failures produce actionable error messages

---

## Updated Test Directory Structure

After Phase 9 completion:

```
tests/
├── run-all.sh                    # Main orchestrator (updated)
├── README.md                     # Comprehensive documentation
├── common/
│   ├── utils.sh                  # Shared utilities
│   ├── prerequisites.sh          # Dependency checks (updated)
│   └── qemu-utils.sh             # NEW: QEMU management
│
├── nano2nano/                    # Existing: nano-ros ↔ nano-ros
│   ├── run.sh
│   └── README.md
│
├── rmw-interop/                  # Existing: ROS 2 interop (native)
│   ├── nano2ros.sh
│   ├── ros2nano.sh
│   ├── matrix.sh
│   └── README.md
│
├── rmw-detailed/                 # Existing: Protocol tests
│   ├── liveliness.sh
│   ├── keyexpr.sh
│   ├── qos.sh
│   ├── attachment.sh
│   └── README.md
│
├── zephyr/                       # Existing: Basic Zephyr (deprecated, see emulator/)
│   └── run.sh
│
├── platform/                     # NEW: Platform backend tests
│   ├── run.sh
│   ├── README.md
│   ├── posix.sh
│   ├── smoltcp-sim.sh
│   └── generic.sh
│
├── smoltcp/                      # NEW: smoltcp integration tests
│   ├── run.sh
│   ├── README.md
│   ├── allocator.sh
│   ├── socket-buffers.sh
│   ├── clock-sync.sh
│   └── poll-callback.sh
│
├── emulator/                     # NEW: Emulator-based tests
│   ├── run.sh
│   ├── README.md
│   ├── common/
│   │   └── qemu-utils.sh
│   ├── qemu-cortex-m3.sh
│   ├── zephyr-native-sim.sh
│   └── zephyr-qemu-arm.sh
│
├── rmw-interop-embedded/         # NEW: Embedded ROS 2 interop
│   ├── run.sh
│   ├── README.md
│   ├── zephyr-to-ros2.sh
│   ├── ros2-to-zephyr.sh
│   └── matrix-embedded.sh
│
├── cross-platform/               # NEW: Cross-platform communication
│   ├── run.sh
│   ├── README.md
│   ├── posix-to-zephyr.sh
│   ├── zephyr-to-posix.sh
│   └── multi-node.sh
│
└── executor/                     # NEW: Executor API tests
    ├── run.sh
    ├── README.md
    ├── connected-executor.sh
    ├── shim-executor.sh
    ├── api-consistency.sh
    └── compile-check/
        ├── rtic-patterns.rs
        └── embassy-patterns.rs
```

---

## Implementation Priority

| Phase | Priority | Estimated Effort | Dependencies |
|-------|----------|------------------|--------------|
| 9.1 Platform Backend | **High** | 2-3 days | Phase 8 complete |
| 9.3 Emulator Tests | **High** | 3-4 days | QEMU, Zephyr workspace |
| 9.4 Embedded ROS 2 Interop | **High** | 2-3 days | ROS 2, Zephyr workspace |
| 9.2 smoltcp Integration | **Medium** | 2-3 days | Phase 9.1 |
| 9.5 Cross-Platform | **Medium** | 2 days | Phase 9.3, 9.4 |
| 9.6 Executor API | **Medium** | 2 days | Phase 9.1 |
| 9.7 Infrastructure | **Low** | 1-2 days | All phases |

---

## Test Execution Matrix

| Test Suite | CI | Local | Hardware | QEMU | Zephyr |
|------------|:--:|:-----:|:--------:|:----:|:------:|
| nano2nano | ✓ | ✓ | - | - | - |
| rmw-interop | ✓* | ✓ | - | - | - |
| rmw-detailed | ✓* | ✓ | - | - | - |
| platform/posix | ✓ | ✓ | - | - | - |
| platform/smoltcp-sim | ✓ | ✓ | - | - | - |
| platform/generic | ✓ | ✓ | - | - | - |
| smoltcp/* | ✓ | ✓ | - | - | - |
| emulator/qemu-cortex-m3 | ✓ | ✓ | - | ✓ | - |
| emulator/zephyr-native-sim | ✓ | ✓ | - | - | ✓ |
| emulator/zephyr-qemu-arm | ✓ | ✓ | - | ✓ | ✓ |
| rmw-interop-embedded/* | ✓* | ✓ | - | - | ✓ |
| cross-platform/* | ✓ | ✓ | - | - | ✓ |
| executor/* | ✓ | ✓ | - | - | - |

*Requires ROS 2 in CI environment

---

## Prerequisites Summary

### Required for All Tests
- Rust toolchain (stable)
- zenohd router
- cargo-nextest (recommended)

### Platform Tests
- No additional requirements (uses host features)

### smoltcp Tests
- No additional requirements (host simulation)

### Emulator Tests
- QEMU: `qemu-system-arm`
- Zephyr: west workspace, Zephyr SDK
- TAP interface: `sudo ./scripts/setup-zephyr-network.sh`

### ROS 2 Interop Tests
- ROS 2 Humble or Jazzy
- rmw_zenoh_cpp: `sudo apt install ros-humble-rmw-zenoh-cpp`

### Executor Tests
- ARM GCC toolchain (for compile-check only)
- `thumbv7em-none-eabihf` target: `rustup target add thumbv7em-none-eabihf`

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| QEMU setup complexity | Medium | Provide detailed setup scripts and documentation |
| Zephyr workspace maintenance | Medium | Document workspace setup, provide update scripts |
| CI resource constraints | Low | Run hardware-dependent tests only on nightly/release |
| Flaky network tests | Medium | Add retries, increase timeouts, use isolated networks |
| ROS 2 version compatibility | Low | Test on multiple ROS 2 versions (Humble, Jazzy) |

---

## Success Metrics

- [ ] 100% of platform backends have functional tests
- [ ] Emulator tests cover all embedded examples
- [ ] ROS 2 interop works bidirectionally with Zephyr
- [ ] Cross-platform tests verify message compatibility
- [ ] Executor API tests ensure consistency
- [ ] All tests can run in CI (where applicable)
- [ ] Test execution time < 10 minutes for quick suite
- [ ] Test documentation is comprehensive
