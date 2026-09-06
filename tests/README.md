# nano-ros Integration Tests

Integration tests for nano-ros communication, platform backends, and ROS 2 interoperability.

## Overview

nano-ros uses a Rust-based test framework with rstest fixtures in `packages/testing/nros-tests/`. This provides:

- **Type safety** - Compile-time error checking
- **RAII cleanup** - Automatic process cleanup via `Drop` trait
- **Parallel execution** - Tests run concurrently with proper isolation
- **Cached builds** - Binary builds are cached across test runs
- **IDE support** - Full debugging and code navigation

## Running Tests

### Quick Start

```bash
# Unit tests only (no external deps)
just test-unit

# Standard tests (needs qemu-system-arm + zenohd)
just test

# All tests (needs Zephyr, ROS 2, cmake, etc.)
just test-all

# Run integration tests via cargo directly
cargo test -p nros-tests --tests -- --nocapture
```

### Test Groups

```bash
just test-unit          # Unit tests + Miri (no external deps)
just test-qemu          # QEMU bare-metal tests (needs qemu-system-arm)
just test-qemu-esp32    # ESP32-C3 QEMU tests (needs qemu-system-riscv32 + espflash)
just test-integration   # All Rust integration tests (needs zenohd)
just test-zephyr        # Zephyr E2E tests (needs west + Zephyr workspace)
just test-ros2          # ROS 2 interop tests (needs ROS 2 + rmw_zenoh_cpp)
just test-c             # C API tests (needs cmake + zenohd)
just test-docker-qemu   # QEMU networked tests in Docker (needs docker)
just test-xrce          # XRCE-DDS integration tests (needs XRCE Agent)
just test-xrce-ros2     # XRCE ↔ ROS 2 DDS interop (needs XRCE Agent + ROS 2 + rmw_fastrtps)
just test-c-xrce        # C XRCE API tests (needs cmake + XRCE Agent)
```

## Directory Structure

```
tests/
├── README.md           # This file
├── c-msg-gen-tests.sh  # C message generation tests
├── zephyr/             # Zephyr native_sim tests (shell-based)
│   └── run-c.sh        # Zephyr C examples test
└── simple-workspace/   # Standalone build verification

packages/testing/nros-tests/  # Rust test crate
├── Cargo.toml
├── src/
│   ├── lib.rs          # Test utilities (wait_for_pattern, count_pattern)
│   ├── esp32.rs        # ESP32-C3 QEMU helpers (guard functions, flash, launch)
│   └── fixtures/
│       ├── mod.rs
│       ├── binaries.rs         # Binary build helpers (cached)
│       ├── qemu.rs             # QemuProcess fixture (RAII)
│       ├── ros2.rs             # ROS 2 process helpers (zenoh + DDS)
│       ├── xrce_agent.rs       # XrceAgent / XrceSerialAgent fixtures (RAII)
│       └── zenohd_router.rs    # ZenohRouter fixture (RAII)
└── tests/
    ├── actions.rs              # Action server/client tests (zenoh)
    ├── c_api.rs                # C API tests (zenoh)
    ├── c_xrce_api.rs           # C XRCE API tests
    ├── custom_msg.rs           # Custom message tests (zenoh)
    ├── emulator.rs             # QEMU Cortex-M3 tests (ARM)
    ├── error_handling.rs       # Error handling tests
    ├── esp32_emulator.rs       # QEMU ESP32-C3 tests (RISC-V)
    ├── executor.rs             # Executor tests
    ├── multi_node.rs           # Multi-node tests (zenoh)
    ├── nano2nano.rs            # nano-ros ↔ nano-ros pub/sub tests (zenoh)
    ├── params.rs               # Parameter tests (zenoh)
    ├── platform.rs             # Platform detection tests
    ├── qos.rs                  # QoS tests (zenoh)
    ├── rmw.rs                  # RMW trait tests
    ├── rmw_interop.rs          # ROS 2 zenoh interop tests
    ├── safety_e2e.rs           # Safety E2E protocol tests
    ├── services.rs             # Service tests (zenoh)
    ├── xrce.rs                 # XRCE-DDS integration tests
    ├── xrce_ros2_interop.rs    # XRCE ↔ ROS 2 DDS interop tests
    └── zephyr.rs               # Zephyr E2E tests
```

## Test Suites

### emulator
Tests on QEMU Cortex-M3 emulator:
- CDR serialization verification
- Node API tests
- Type metadata tests

**Requirements:** `qemu-system-arm`, `thumbv7m-none-eabi` target

### esp32_emulator
Tests on QEMU ESP32-C3 emulator (Espressif fork):
- Build verification (nightly toolchain + zenoh-pico RISC-V)
- Boot test (BSP banner via UART)
- Networked E2E (talker → listener via zenohd + slirp)

**Requirements:** `qemu-system-riscv32` (Espressif fork), `espflash`, nightly toolchain, `riscv32imc-unknown-none-elf` target, zenoh-pico RISC-V library

For networked tests: zenohd (no TAP/sudo needed — uses slirp user-mode networking)

```bash
just test-qemu-esp32    # Run all ESP32 QEMU tests
```

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

**Pub/Sub Tests:**
- nano-ros → ROS 2 communication
- ROS 2 → nano-ros communication
- Communication matrix (all directions)
- Key expression format verification

**Service Tests:**
- nano-ros server → ROS 2 client
- ROS 2 server → nano-ros client
- Service discovery

**Action Tests:**
- nano-ros action server ↔ ROS 2 action client
- ROS 2 action server ↔ nano-ros action client

**Discovery Tests:**
- `ros2 node list` shows nano-ros nodes
- `ros2 topic list` shows nano-ros topics
- `ros2 service list` shows nano-ros services

**QoS Tests:**
- BEST_EFFORT ↔ BEST_EFFORT (works)
- RELIABLE ↔ RELIABLE (works)
- RELIABLE → BEST_EFFORT (works)
- BEST_EFFORT → RELIABLE (expected to fail)

**Benchmark Tests:**
- First-message latency measurement
- Message throughput measurement

**Requirements:** `zenohd`, ROS 2 Humble, `rmw_zenoh_cpp`, `example_interfaces`

Tests gracefully skip when ROS 2 is not available.

### xrce
Tests XRCE-DDS backend via Micro-XRCE-DDS Agent (14 tests):

**Pub/Sub Tests:**
- Talker startup and message publishing
- Listener startup and subscription
- Talker → listener communication (asserted)
- Multiple message delivery (≥3 messages)
- Large message publishing (fragmented streams)

**Service Tests:**
- Service server startup and readiness
- Service client startup
- AddTwoInts request/response (3 sequential calls)

**Action Tests:**
- Action server startup
- Action client startup
- Fibonacci action E2E (goal → feedback → result)

**Serial Transport Tests:**
- Serial talker startup (PTY + HDLC framing)
- Serial listener startup
- Serial talker → listener communication (via socat PTY pair)

**Requirements:** Micro-XRCE-DDS Agent (`just setup xrce`), `socat` (for serial tests)

```bash
just test-xrce          # Run all XRCE tests
just test-xrce verbose  # Verbose output
```

### xrce_ros2_interop
Tests interoperability between nano-ros XRCE nodes and ROS 2 DDS nodes (4 tests):

```
nano-ros XRCE node → XRCE Agent (Fast-DDS) ←DDS multicast→ ROS 2 node (rmw_fastrtps_cpp)
```

**Tests:**
- ROS 2 DDS detection (rmw_fastrtps_cpp availability)
- nano-ros XRCE talker → ROS 2 DDS listener (pub/sub)
- ROS 2 DDS publisher → nano-ros XRCE listener (pub/sub)
- nano-ros XRCE service server + ROS 2 DDS service client (AddTwoInts)

Tests are diagnostic/informational — they report interop status but do not hard-fail,
because DDS interop between the XRCE Agent's bundled Fast-DDS and the system's ROS 2
Fast-DDS can have version-dependent issues.

**Requirements:** Micro-XRCE-DDS Agent, ROS 2 Humble, `rmw_fastrtps_cpp`, `example_interfaces`

```bash
just test-xrce-ros2          # Run XRCE ↔ ROS 2 interop tests
just test-xrce-ros2 verbose  # Verbose output
```

### c_xrce_api
Tests C API with XRCE-DDS backend using CMake-built examples (5 tests):
- C XRCE talker build verification
- C XRCE listener build verification
- C XRCE talker startup
- C XRCE listener startup
- C XRCE talker → listener pub/sub communication

**Requirements:** `cmake`, Micro-XRCE-DDS Agent

```bash
just test-c-xrce          # Run C XRCE tests
just test-c-xrce verbose  # Verbose output
```

### zephyr (shell-based)
Tests Zephyr native_sim integration:
- Zephyr talker → native subscriber
- Networked E2E via zenohd

**Requirements:** West workspace, zenohd

```bash
# Setup (one time)
./zephyr/setup.sh

# Run tests (native_sim uses NSOS on host loopback — no TAP bridge required)
just test-zephyr
```

### c_api (Rust-managed)
Tests C API integration using CMake-built examples:
- C talker and C listener build verification
- C talker and C listener startup/initialization
- C talker → C listener pub/sub communication

**Requirements:** `cmake`, `zenohd`, Rust toolchain

```bash
just test-c             # Run all C tests
just test-c verbose     # Verbose output
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

### XRCE-DDS Tests
- Micro-XRCE-DDS Agent (`just setup xrce`)
- `socat` (for serial transport tests)

```bash
# Build XRCE Agent (one-time)
just setup xrce

# Install socat (for serial tests)
sudo apt install socat
```

### XRCE ↔ ROS 2 DDS Interop Tests
- All XRCE-DDS requirements above
- ROS 2 Humble (or later)
- `rmw_fastrtps_cpp` (default in Humble)
- `example_interfaces` package

### QEMU Tests
- `qemu-system-arm`
- ARM embedded toolchain

```bash
# Install QEMU
sudo apt install qemu-system-arm
```

## Writing New Tests

Create tests in `packages/testing/nros-tests/tests/`:

```rust
use nros_tests::fixtures::{zenohd_unique, ZenohRouter};
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
| `ZenohRouter::start(port)` | Starts zenohd on fixed port, auto-cleanup |
| `XrceAgent::start_unique()` | Starts XRCE Agent on ephemeral UDP port, auto-cleanup |
| `XrceSerialAgent::start()` | Starts XRCE Agent in serial mode (PTY pair via socat), auto-cleanup |
| `build_native_talker()` | Builds and caches native-rs-talker binary |
| `build_native_listener()` | Builds and caches native-rs-listener binary |
| `build_esp32_qemu_talker()` | Builds and caches ESP32 QEMU talker (nightly) |
| `build_esp32_qemu_listener()` | Builds and caches ESP32 QEMU listener (nightly) |
| `xrce_talker_binary` | Builds and caches XRCE talker binary |
| `xrce_listener_binary` | Builds and caches XRCE listener binary |
| `xrce_service_server_binary` | Builds and caches XRCE service server binary |
| `xrce_service_client_binary` | Builds and caches XRCE service client binary |
| `xrce_action_server_binary` | Builds and caches XRCE action server binary |
| `xrce_action_client_binary` | Builds and caches XRCE action client binary |
| `QemuProcess::run()` | Runs QEMU ARM with semihosting, auto-cleanup |
| `start_esp32_qemu()` | Starts QEMU ESP32-C3 instance, auto-cleanup |
| `Ros2Process::topic_echo()` | Runs ros2 topic echo (rmw_zenoh_cpp), auto-cleanup |
| `Ros2Process::topic_pub()` | Runs ros2 topic pub (rmw_zenoh_cpp), auto-cleanup |
| `Ros2DdsProcess::topic_echo()` | Runs ros2 topic echo (rmw_fastrtps_cpp), auto-cleanup |
| `Ros2DdsProcess::topic_pub()` | Runs ros2 topic pub (rmw_fastrtps_cpp), auto-cleanup |
| `Ros2DdsProcess::service_call()` | Runs ros2 service call (rmw_fastrtps_cpp), auto-cleanup |

### QEMU Networked Test Practices

QEMU tests use **slirp (user-mode) networking** — each QEMU instance has its own isolated NAT stack. No TAP devices, bridges, or `sudo` required. Each platform has a dedicated zenohd port so platforms run in parallel.

**Network topology (slirp):**
```
QEMU node 0 (slirp, 10.0.2.x) ---> 10.0.2.2:<port> --+
                                                        |-- zenohd (127.0.0.1:<port>)
QEMU node 1 (slirp, 10.0.2.y) ---> 10.0.2.2:<port> --+
```

**Per-platform zenohd ports** (defined in `nros_tests::platform`):
| Platform | Port |
|----------|------|
| bare-metal | 7450 |
| FreeRTOS | 7451 |
| NuttX | 7452 |
| ThreadX RISC-V | 7453 |
| ESP32 | 7454 |
| ThreadX Linux | 7455 |
| Zephyr | 7456 |

**1. Start the subscriber first, then the publisher.**

The subscriber must be running and have registered its subscription with zenohd before the publisher starts sending. Otherwise messages are lost because zenoh doesn't buffer for unknown subscribers.

**2. Add stabilization delay between subscription and publish.**

After the subscriber reports it's connected and subscribed, wait 5–10 seconds before starting the publisher. This gives zenohd time to propagate the subscription to other sessions.

**3. Use the platform port constant, not a hardcoded port.**

Each platform's zenohd port is defined in `nros_tests::platform`. Use it consistently:

```rust
use nros_tests::platform;
let _zenohd = ZenohRouter::start(platform::FREERTOS.zenohd_port)
    .expect("Failed to start zenohd");
```

**4. Use nextest test groups with `max-threads = 1` per platform.**

Each platform has its own test group in `.config/nextest.toml`. Tests within a platform run serially (one QEMU pair at a time), but different platforms run in parallel:

```toml
[test-groups.qemu-freertos]
max-threads = 1

[[profile.default.overrides]]
filter = "binary(freertos_qemu)"
test-group = "qemu-freertos"
```

**5. Bridge-networked platforms use `start_on("0.0.0.0", ...)`.**

ThreadX Linux (veth) and Zephyr (TAP) use bridge networking, not slirp. Their zenohd must bind to `0.0.0.0` instead of `127.0.0.1`:

```rust
let _zenohd = ZenohRouter::start_on("0.0.0.0", platform::ZEPHYR.zenohd_port)
    .expect("Failed to start zenohd");
```

**Example E2E test ordering:**

```
1. Start zenohd on platform port (e.g., 127.0.0.1:7451 for FreeRTOS)
2. Start listener QEMU (slirp), wait for "Waiting for messages..."
3. Sleep 10s (boot + network init + subscription propagation)
4. Start talker QEMU (slirp), wait for "Published:" output
5. Verify listener received messages
```

### Test Utilities

```rust
use nros_tests::{wait_for_pattern, count_pattern};

// Wait for pattern in output
let found = wait_for_pattern(&output, "Received:", Duration::from_secs(10));

// Count occurrences
let count = count_pattern(&output, "data:");
```

## CI Integration

### GitHub Actions Example

To run ROS 2 interop tests in CI, create a job with ROS 2 + rmw_zenoh installed:

```yaml
ros2-interop-tests:
  runs-on: ubuntu-latest
  container:
    image: ros:humble
  steps:
    - uses: actions/checkout@v4

    - name: Install dependencies
      run: |
        apt-get update
        apt-get install -y ros-humble-rmw-zenoh-cpp ros-humble-example-interfaces
        cargo install zenoh --locked

    - name: Run interop tests (Rust)
      run: |
        source /opt/ros/humble/setup.bash
        cargo test -p nros-tests --test rmw_interop -- --nocapture
```

### Test Categories for CI

| Test Suite | Command | Requirements |
|------------|---------|--------------|
| Unit tests | `just test-unit` | None |
| QEMU ARM tests | `just test-qemu` | qemu-system-arm |
| QEMU ESP32 tests | `just test-qemu-esp32` | qemu-system-riscv32 + espflash + zenohd |
| Integration tests | `just test-integration` | zenohd |
| XRCE-DDS tests | `just test-xrce` | XRCE Agent + socat |
| XRCE ↔ ROS 2 interop | `just test-xrce-ros2` | XRCE Agent + ROS 2 + rmw_fastrtps |
| Zephyr tests | `just test-zephyr` | west + zenohd |
| ROS 2 zenoh interop | `just test-ros2` | ROS 2 + rmw_zenoh |
| C API tests (zenoh) | `just test-c` | cmake + zenohd |
| C API tests (XRCE) | `just test-c-xrce` | cmake + XRCE Agent |
| Docker QEMU | `just test-docker-qemu` | docker |

Tests that require ROS 2 or XRCE Agent will gracefully skip if prerequisites are not met.

## Troubleshooting

### Tests timeout
- Ensure no stale `zenohd` processes: `pkill -x zenohd`
- Check for orphan test processes: `pkill -f native-rs-talker`

### ROS 2 tests skip
- Source ROS 2: `source /opt/ros/humble/setup.bash`
- Verify rmw_zenoh: `ros2 pkg list | grep rmw_zenoh`

### XRCE tests skip
- Build XRCE Agent: `just setup xrce`
- Check Agent binary: `ls build/xrce-agent/MicroXRCEAgent`
- Check socat installed: `socat -V` (needed for serial transport tests)
- Kill stale agents: `pkill -f MicroXRCEAgent`

### QEMU ARM tests fail
- Check QEMU installed: `qemu-system-arm --version`
- Check ARM target: `rustup target list | grep thumbv7m`

### ESP32 QEMU tests fail
- Check Espressif QEMU: `qemu-system-riscv32 --version`
- Check RISC-V target: `rustup +nightly target list --installed | grep riscv32imc`
- Check espflash: `espflash --version`
- Check zenoh-pico RISC-V: `ls build/esp32-zenoh-pico/libzenohpico.a`
- If ESP32 port (7454) is busy: `pkill -x zenohd` and wait for TIME_WAIT to expire

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
| `tests/c-tests.sh` | `tests/c_api.rs` |

Benefits of Rust tests:
- Type-safe process management
- Automatic cleanup (no orphan processes)
- Better error messages with stack traces
- IDE debugging support
- Parallel test execution
