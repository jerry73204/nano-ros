# Emulator Tests

QEMU-based tests for nano-ros on embedded targets without requiring physical hardware.

## Overview

These tests verify nano-ros functionality on embedded targets using QEMU emulation:
- **qemu-cortex-m3.sh** - Cortex-M3 bare-metal tests (CDR, Node API)
- **zephyr-native-sim.sh** - Zephyr native_sim with networking
- **zephyr-qemu-arm.sh** - Zephyr on QEMU ARM (qemu_cortex_m3)

## Directory Structure

```
tests/emulator/
├── run.sh                   # Test orchestrator
├── README.md                # This file
├── common/
│   └── qemu-utils.sh        # QEMU management utilities
├── qemu-cortex-m3.sh        # Cortex-M3 bare-metal tests
├── zephyr-native-sim.sh     # Zephyr native_sim tests
└── zephyr-qemu-arm.sh       # Zephyr QEMU ARM tests
```

## Requirements

### qemu-cortex-m3.sh

| Dependency | Installation |
|------------|--------------|
| qemu-system-arm | `sudo apt install qemu-system-arm` |
| ARM GCC toolchain | `rustup target add thumbv7m-none-eabi` |
| cargo | (standard Rust toolchain) |

### zephyr-native-sim.sh

| Dependency | Installation |
|------------|--------------|
| Zephyr workspace | `./zephyr/setup.sh` |
| TAP interface | `sudo ./scripts/setup-zephyr-network.sh` |
| zenohd | See nano-ros setup |
| west | `pip install west` |

### zephyr-qemu-arm.sh

| Dependency | Installation |
|------------|--------------|
| Zephyr workspace | `./zephyr/setup.sh` |
| qemu-system-arm | `sudo apt install qemu-system-arm` |
| west | `pip install west` |

## Running Tests

```bash
# Run all emulator tests
./tests/emulator/run.sh

# Run individual tests
./tests/emulator/qemu-cortex-m3.sh
./tests/emulator/zephyr-native-sim.sh
./tests/emulator/zephyr-qemu-arm.sh

# Quick mode (skip slow tests)
./tests/emulator/run.sh --quick

# Verbose output
./tests/emulator/run.sh --verbose
```

## Test Descriptions

### qemu-cortex-m3.sh

Runs the `examples/qemu-test` binary on QEMU's Cortex-M3 LM3S6965 board.

**Tests:**
- CDR serialization (Int32, Float64, Time)
- Type metadata (TYPE_NAME, TYPE_HASH)
- CDR encapsulation header
- Node API (creation, publisher, subscriber)
- Message serialization

**How it works:**
1. Build `qemu-test` for `thumbv7m-none-eabi`
2. Run in QEMU with semihosting enabled
3. Capture semihosting output
4. Parse test results (PASS/FAIL counts)

### zephyr-native-sim.sh

Enhanced version of `tests/zephyr/run.sh` with additional test scenarios.

**Tests:**
- Zephyr talker → native subscriber (basic communication)
- Native talker → Zephyr listener (reverse direction)
- Bidirectional communication (simultaneous pub/sub)
- Multiple message types (Int32, String)
- Reconnection after disconnect

**How it works:**
1. Build Zephyr examples for native_sim/native/64
2. Set up TAP networking (requires one-time sudo setup)
3. Start zenoh router on host
4. Run Zephyr binaries as native processes
5. Verify message exchange

### zephyr-qemu-arm.sh

Runs Zephyr on QEMU ARM with virtual networking.

**Tests:**
- Zephyr in QEMU → host subscriber
- Host talker → Zephyr in QEMU

**How it works:**
1. Build Zephyr for qemu_cortex_m3 board
2. Start QEMU with SLIRP networking
3. Start zenoh router with port forwarding
4. Verify message exchange

## Troubleshooting

### QEMU not found
```bash
sudo apt install qemu-system-arm
```

### Zephyr build fails
```bash
# Ensure workspace is set up
./zephyr/setup.sh

# Source environment
cd ~/nano-ros-workspace
source .venv/bin/activate
source zephyr/zephyr-env.sh
```

### TAP interface not configured
```bash
sudo ./scripts/setup-zephyr-network.sh
```

### Tests timeout
- Increase TEST_TIMEOUT environment variable
- Check network connectivity between host and emulator
- Verify zenohd is running and accessible

## CI Integration

These tests are designed to run in headless CI environments:
- QEMU runs without display (`-nographic`)
- Semihosting output captured to files
- Timeout handling prevents hangs
- Exit codes indicate pass/fail

```yaml
# Example CI configuration
emulator-tests:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - name: Install QEMU
      run: sudo apt install qemu-system-arm
    - name: Run emulator tests
      run: ./tests/emulator/run.sh --quick
```
