# nano-ros Integration Tests

Integration tests for nano-ros communication, platform backends, and ROS 2 interoperability.

## Overview

nano-ros uses a Rust-based test framework with rstest fixtures in `crates/nano-ros-tests/`. This provides:

- **Type safety** - Compile-time error checking
- **RAII cleanup** - Automatic process cleanup via `Drop` trait
- **Parallel execution** - Tests run concurrently with proper isolation
- **Cached builds** - Binary builds are cached across test runs
- **IDE support** - Full debugging and code navigation

## Running Tests

### Quick Start

```bash
# Run all integration tests
just test-rust

# Run via cargo directly
cargo test -p nano-ros-tests --tests -- --nocapture

# Run via wrapper script (with colored output)
./tests/rust-tests.sh
```

### Specific Test Suites

```bash
# QEMU Cortex-M3 emulator tests
just test-rust-emulator

# nano-ros ↔ nano-ros pub/sub tests
just test-rust-nano2nano

# Platform detection tests
just test-rust-platform

# ROS 2 rmw_zenoh interop tests (requires ROS 2 + rmw_zenoh_cpp)
just test-rust-rmw-interop

# Zephyr native_sim tests (requires west workspace + TAP network)
just test-zephyr
```

### Using the Wrapper Script

```bash
./tests/rust-tests.sh                 # Run all tests
./tests/rust-tests.sh emulator        # Run emulator tests
./tests/rust-tests.sh nano2nano       # Run pub/sub tests
./tests/rust-tests.sh platform        # Run platform tests
./tests/rust-tests.sh rmw_interop     # Run ROS 2 interop tests
./tests/rust-tests.sh -v all          # Verbose output
```

## Directory Structure

```
tests/
├── README.md           # This file
├── rust-tests.sh       # Optional wrapper script for nice output
├── zephyr/             # Zephyr native_sim tests (shell-based)
│   └── run.sh          # Requires west workspace + TAP network
└── simple-workspace/   # Standalone build verification

crates/nano-ros-tests/  # Rust test crate
├── Cargo.toml
├── src/
│   ├── lib.rs          # Test utilities (wait_for_pattern, count_pattern)
│   └── fixtures/
│       ├── mod.rs
│       ├── binaries.rs     # Binary build helpers (cached)
│       ├── qemu.rs         # QemuProcess fixture (RAII)
│       ├── ros2.rs         # ROS 2 process helpers
│       └── zenohd_fixture.rs # ZenohRouter fixture (RAII)
└── tests/
    ├── emulator.rs     # QEMU Cortex-M3 tests
    ├── nano2nano.rs    # nano-ros ↔ nano-ros tests
    ├── platform.rs     # Platform detection tests
    └── rmw_interop.rs  # ROS 2 interop tests
```

## Test Suites

### emulator
Tests on QEMU Cortex-M3 emulator:
- CDR serialization verification
- Node API tests
- Type metadata tests

**Requirements:** `qemu-system-arm`, `thumbv7m-none-eabi` target

### nano2nano
Tests communication between nano-ros nodes:
- Basic pub/sub with zenohd router
- Message delivery verification

**Requirements:** `zenohd` in PATH

### platform
Tests platform and toolchain detection:
- QEMU ARM availability
- ARM toolchain detection
- Embedded target availability
- Zephyr workspace detection

**Requirements:** None (detection tests)

### rmw_interop
Tests interoperability with ROS 2 using rmw_zenoh_cpp:
- nano-ros → ROS 2 communication
- ROS 2 → nano-ros communication
- Communication matrix (all directions)
- Key expression format verification
- QoS compatibility

**Requirements:** `zenohd`, ROS 2 Humble, `rmw_zenoh_cpp`

Tests gracefully skip when ROS 2 is not available.

### zephyr (shell-based)
Tests Zephyr native_sim integration:
- Zephyr talker → native subscriber
- TAP network communication

**Requirements:** West workspace, TAP network interface

```bash
# Setup (one time)
./zephyr/setup.sh
sudo ./scripts/setup-zephyr-network.sh

# Run tests
just test-zephyr
```

## Requirements

### Basic Tests
- `zenohd` in PATH
- Rust toolchain with `thumbv7m-none-eabi` target

### ROS 2 Interop Tests
- ROS 2 Humble (or later)
- `rmw_zenoh_cpp` middleware

```bash
# Install rmw_zenoh_cpp
sudo apt install ros-humble-rmw-zenoh-cpp
```

### QEMU Tests
- `qemu-system-arm`
- ARM embedded toolchain

```bash
# Install QEMU
sudo apt install qemu-system-arm
```

## Writing New Tests

Create tests in `crates/nano-ros-tests/tests/`:

```rust
use nano_ros_tests::fixtures::{zenohd_unique, ZenohRouter};
use rstest::rstest;

#[rstest]
fn test_my_feature(zenohd_unique: ZenohRouter) {
    // zenohd is automatically started and cleaned up
    let locator = zenohd_unique.locator();

    // Your test logic here
}
```

### Available Fixtures

| Fixture | Description |
|---------|-------------|
| `zenohd_unique` | Starts zenohd on unique port, auto-cleanup |
| `build_native_talker()` | Builds and caches native-talker binary |
| `build_native_listener()` | Builds and caches native-listener binary |
| `QemuProcess::run()` | Runs QEMU with semihosting, auto-cleanup |
| `Ros2Process::topic_echo()` | Runs ros2 topic echo, auto-cleanup |
| `Ros2Process::topic_pub()` | Runs ros2 topic pub, auto-cleanup |

### Test Utilities

```rust
use nano_ros_tests::{wait_for_pattern, count_pattern};

// Wait for pattern in output
let found = wait_for_pattern(&output, "Received:", Duration::from_secs(10));

// Count occurrences
let count = count_pattern(&output, "data:");
```

## Troubleshooting

### Tests timeout
- Ensure no stale `zenohd` processes: `pkill -x zenohd`
- Check for orphan test processes: `pkill -f native-talker`

### ROS 2 tests skip
- Source ROS 2: `source /opt/ros/humble/setup.bash`
- Verify rmw_zenoh: `ros2 pkg list | grep rmw_zenoh`

### QEMU tests fail
- Check QEMU installed: `qemu-system-arm --version`
- Check ARM target: `rustup target list | grep thumbv7m`

## Migration from Shell Scripts

The Rust test framework replaces the previous shell-based tests:

| Shell Script | Rust Equivalent |
|--------------|-----------------|
| `tests/emulator/` | `tests/emulator.rs` |
| `tests/nano2nano/` | `tests/nano2nano.rs` |
| `tests/platform/` | `tests/platform.rs` |
| `tests/rmw-interop/` | `tests/rmw_interop.rs` |
| `tests/rmw-detailed/` | `tests/rmw_interop.rs` |
| `tests/smoltcp/` | Unit tests in crate |
| `tests/common/` | `src/lib.rs` + `src/fixtures/` |

Benefits of Rust tests:
- Type-safe process management
- Automatic cleanup (no orphan processes)
- Better error messages with stack traces
- IDE debugging support
- Parallel test execution
