# nano-ros Integration Tests

Integration tests for nano-ros communication with itself and ROS 2.

## Directory Structure

```
tests/
├── run-all.sh           # Main test runner
├── README.md
├── common/              # Shared utilities
│   ├── utils.sh         # Logging, cleanup, helpers
│   └── prerequisites.sh # Prerequisite checks
├── nano2nano/           # nano-ros ↔ nano-ros tests
│   ├── run.sh
│   └── README.md
├── rmw-interop/         # ROS 2 rmw_zenoh interop tests
│   ├── nano2ros.sh      # nano-ros talker → ROS 2 listener
│   ├── ros2nano.sh      # ROS 2 talker → nano-ros listener
│   ├── matrix.sh        # All 4 combinations
│   └── README.md
├── rmw-detailed/        # Detailed protocol tests
│   ├── liveliness.sh    # Discovery tokens
│   ├── keyexpr.sh       # Key expression format
│   ├── qos.sh           # QoS compatibility
│   ├── attachment.sh    # RMW attachment metadata
│   └── README.md
├── platform/            # Platform backend tests
│   ├── run.sh           # Platform test runner
│   ├── posix.sh         # POSIX platform (reference)
│   ├── smoltcp-sim.sh   # smoltcp platform (simulation)
│   ├── generic.sh       # Generic compile tests
│   └── README.md
├── smoltcp/             # smoltcp integration tests
│   ├── run.sh           # smoltcp test runner
│   ├── allocator.sh     # Bump allocator tests
│   ├── socket-buffers.sh # Socket buffer operations
│   ├── clock-sync.sh    # Clock synchronization tests
│   ├── poll-callback.sh # Poll callback tests
│   └── README.md
├── emulator/            # Emulator tests (QEMU, Zephyr)
│   ├── run.sh           # Emulator test runner
│   ├── qemu-cortex-m3.sh    # QEMU bare-metal tests
│   ├── zephyr-native-sim.sh # Zephyr native_sim tests
│   ├── zephyr-qemu-arm.sh   # Zephyr QEMU ARM tests
│   ├── common/
│   │   └── qemu-utils.sh    # QEMU utilities
│   └── README.md
└── zephyr/              # Zephyr integration tests
    └── run.sh
```

## Quick Start

```bash
# Run all tests
./tests/run-all.sh

# Quick smoke test
./tests/run-all.sh --quick

# Specific test suite
./tests/run-all.sh nano2nano
./tests/run-all.sh rmw-interop
./tests/run-all.sh rmw-detailed
./tests/run-all.sh platform
./tests/run-all.sh smoltcp
./tests/run-all.sh emulator

# With verbose output
./tests/run-all.sh --verbose
```

## Test Suites

### nano2nano
Tests communication between two nano-ros nodes.
- Basic pub/sub
- Multiple messages
- Peer mode (no router)

**Requirements:** zenohd

### rmw-interop
Tests interoperability with ROS 2 using rmw_zenoh_cpp.
- nano-ros talker → ROS 2 listener
- ROS 2 talker → nano-ros listener
- Full 2x2 communication matrix

**Requirements:** zenohd, ROS 2 Humble, rmw_zenoh_cpp

### rmw-detailed
Tests specific protocol details.
- Liveliness token format
- Key expression format
- QoS compatibility
- RMW attachment format

**Requirements:** zenohd, ROS 2 Humble, rmw_zenoh_cpp, z_sub (optional)

### platform
Tests zenoh-pico-shim platform backends.
- POSIX platform (reference implementation)
- smoltcp platform (simulation mode)
- Generic compile tests (no platform backend)

**Requirements:** zenohd (for posix tests), cargo

### smoltcp
Tests the smoltcp platform layer implementation.
- Bump allocator (alloc, realloc, free)
- Socket buffer operations (push_rx, pop_tx)
- Clock synchronization (set, get)
- Poll callback (registration, invocation)

**Requirements:** cargo (no zenohd required)

### emulator
Tests on emulated hardware (QEMU, Zephyr native_sim).
- QEMU Cortex-M3 bare-metal (CDR serialization, Node API)
- Zephyr native_sim (network communication)
- Zephyr QEMU ARM (optional, limited network support)

**Requirements:** qemu-system-arm, thumbv7m-none-eabi target
**Optional:** Zephyr workspace (for Zephyr tests)

## Requirements

### Basic Tests (nano2nano)
- zenohd in PATH
- nano-ros built with zenoh feature

### ROS 2 Interop Tests
- zenohd in PATH
- ROS 2 Humble (or Jazzy)
- rmw_zenoh_cpp installed

```bash
# Install ROS 2 Humble
# See: https://docs.ros.org/en/humble/Installation.html

# Install rmw_zenoh_cpp
sudo apt install ros-humble-rmw-zenoh-cpp
```

### Optional (for detailed tests)
- z_sub/z_pub from zenoh-pico examples

## Running Individual Tests

```bash
# nano2nano
./tests/nano2nano/run.sh
./tests/nano2nano/run.sh --peer    # Test without router

# rmw-interop
./tests/rmw-interop/nano2ros.sh
./tests/rmw-interop/ros2nano.sh
./tests/rmw-interop/matrix.sh

# rmw-detailed
./tests/rmw-detailed/liveliness.sh
./tests/rmw-detailed/keyexpr.sh
./tests/rmw-detailed/qos.sh
./tests/rmw-detailed/attachment.sh

# platform
./tests/platform/run.sh           # All platform tests
./tests/platform/posix.sh         # POSIX platform
./tests/platform/smoltcp-sim.sh   # smoltcp simulation
./tests/platform/generic.sh       # Generic compile tests

# smoltcp
./tests/smoltcp/run.sh            # All smoltcp tests
./tests/smoltcp/allocator.sh      # Allocator tests
./tests/smoltcp/socket-buffers.sh # Socket buffer tests
./tests/smoltcp/clock-sync.sh     # Clock tests
./tests/smoltcp/poll-callback.sh  # Poll callback tests

# emulator
./tests/emulator/run.sh               # All emulator tests
./tests/emulator/qemu-cortex-m3.sh    # QEMU bare-metal
./tests/emulator/zephyr-native-sim.sh # Zephyr native_sim
./tests/emulator/zephyr-qemu-arm.sh   # Zephyr QEMU ARM
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| ZENOH_LOCATOR | tcp/127.0.0.1:7447 | Zenoh router address |
| ROS_DOMAIN_ID | 0 | ROS domain ID |
| TEST_TIMEOUT | 15 | Test timeout in seconds |

## Troubleshooting

### Tests hang or timeout
- Ensure zenohd is not already running
- Check for leftover processes: `pkill -x zenohd; pkill -f /talker`

### ROS 2 interop fails
1. Verify rmw_zenoh_cpp is installed
2. Check QoS settings match (BEST_EFFORT)
3. Ensure domain ID is 0

### Permission denied
```bash
chmod +x tests/**/*.sh
```

## Adding New Tests

1. Create a new directory under `tests/` or add to existing suite
2. Source common utilities:
   ```bash
   source "$(dirname "$0")/../common/utils.sh"
   source "$(dirname "$0")/../common/prerequisites.sh"
   ```
3. Use `setup_cleanup` trap for resource cleanup
4. Use `log_*` functions for output
5. Return 0 on success, non-zero on failure
