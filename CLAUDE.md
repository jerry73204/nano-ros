# nano-ros

Lightweight ROS 2 client for embedded real-time systems (Zephyr, NuttX). `no_std` compatible.

## Workspace Structure

```
nano-ros/
├── crates/
│   ├── nano-ros/              # Unified API (re-exports all sub-crates)
│   ├── nano-ros-core/         # Core types, traits, node abstraction
│   ├── nano-ros-serdes/       # CDR serialization
│   ├── nano-ros-macros/       # #[derive(RosMessage)] proc macros
│   ├── nano-ros-params/       # Parameter server
│   ├── nano-ros-transport/    # Transport abstraction (zenoh backend)
│   ├── nano-ros-node/         # High-level node API
│   ├── nano-ros-tests/        # Integration test crate
│   ├── zenoh-pico-shim/       # Safe Rust API for zenoh-pico
│   └── zenoh-pico-shim-sys/   # FFI + C shim + zenoh-pico submodule
├── colcon-nano-ros/           # Message binding generator (cargo nano-ros)
├── examples/                  # Standalone example packages
│   ├── native-talker/         # Pub example
│   ├── native-listener/       # Sub example
│   ├── native-service-*/      # Service examples
│   ├── zephyr-talker/         # Zephyr pub example
│   └── zephyr-listener/       # Zephyr sub example
├── scripts/zephyr/            # Zephyr setup scripts
│   ├── setup.sh               # Initialize workspace
│   └── setup-network.sh       # Configure TAP interface
├── tests/                     # Test scripts and docs
├── docs/                      # Detailed documentation
├── zephyr-workspace -> ../nano-ros-workspace/  # Symlink to Zephyr workspace
└── west.yml                   # Zephyr west manifest
```

## Build Commands

```bash
just setup          # Install toolchains, cargo tools, check system deps
just build          # Build with no_std
just check          # Format + clippy
just test           # Run all tests
just quality        # Format + clippy + tests
just doc            # Generate docs

# Integration tests
just test-rust              # All Rust integration tests
just test-rust-nano2nano    # nano-ros ↔ nano-ros tests
just test-rust-rmw-interop  # ROS 2 interop tests
just test-zephyr            # Zephyr native_sim tests
```

### First-Time Setup

```bash
just setup   # Installs: rustup targets, cargo-nextest, cargo-nano-ros
             # Checks for: arm-none-eabi-gcc, qemu-system-arm, cmake
```

For missing system dependencies on Ubuntu:
```bash
sudo apt install gcc-arm-none-eabi qemu-system-arm cmake
```

## Environment Variables

Examples use `Context::from_env()` for configuration:

| Variable | Description | Default |
|----------|-------------|---------|
| `ROS_DOMAIN_ID` | ROS 2 domain ID | `0` |
| `ZENOH_LOCATOR` | Router address (e.g., `tcp/192.168.1.1:7447`) | `tcp/127.0.0.1:7447` |
| `ZENOH_MODE` | Session mode: `client` or `peer` | `client` |

## Development Practices

### Quality Checks
**Always run `just quality` after completing a task.**

### System Packages
**Never install system packages directly.** Inform the user what's needed:
```
QEMU ARM emulator required. Please run: sudo apt install qemu-system-arm
```

### Privileged Commands
**Never execute sudo commands directly.** Provide the command for the user to run.

### Unused Variables
- Rename to `_name` with a comment explaining why
- Use `#[allow(dead_code)]` for test struct fields

## Key Design Patterns

### `no_std` Support
All core crates support `#![no_std]` with optional `std`/`alloc` features.

### Message Types
Generated per-project using `cargo nano-ros generate` from `package.xml`. See [docs/message-generation.md](docs/message-generation.md).

**Installing cargo-nano-ros:**
```bash
# From the nano-ros repository root
just install-cargo-nano-ros

# Or manually:
cargo install --path colcon-nano-ros/packages/cargo-nano-ros --locked
```

**Regenerating bindings in examples (requires ROS 2 environment):**
```bash
source /opt/ros/humble/setup.bash
just generate-bindings
```

### Platform Backends
Selected via feature flags: `posix` (desktop), `zephyr` (Zephyr RTOS), `smoltcp` (bare-metal).

### ROS 2 Interop
Uses rmw_zenoh-compatible protocol. Key format for Humble:
- Data keyexpr: `<domain>/<topic>/<type>/TypeHashNotSupported`
- Liveliness: `@ros2_lv/.../<type>/RIHS01_<hash>/<qos>`

See [docs/rmw_zenoh_interop.md](docs/rmw_zenoh_interop.md).

## Development Phases

| Phase | Focus | Status |
|-------|-------|--------|
| 1 | CDR, types, proc macros | Complete |
| 2A | ROS 2 Interoperability | Complete |
| 2B | Zephyr integration | Complete |
| 3 | Services, parameters | In Progress |
| 4 | Message generation | Complete |
| 5 | RTIC integration | Complete |
| 6 | Actions | Complete |
| 7 | API alignment (rclrs) | Complete |
| 8 | Embedded networking | Complete |
| 9 | Test infrastructure | In Progress |
| 10 | C++ bindings (rclcpp) | Planning |

See [docs/roadmap/](docs/roadmap/) for details.

## Documentation Index

| Topic | Location |
|-------|----------|
| Testing | [tests/README.md](tests/README.md) |
| Message generation | [docs/message-generation.md](docs/message-generation.md) |
| Zephyr setup | [docs/zephyr-setup.md](docs/zephyr-setup.md) |
| ROS 2 interop protocol | [docs/rmw_zenoh_interop.md](docs/rmw_zenoh_interop.md) |
| Embedded integration | [docs/embedded-integration.md](docs/embedded-integration.md) |
| RTIC design | [docs/rtic-integration-design.md](docs/rtic-integration-design.md) |
| Memory requirements | [docs/memory-requirements.md](docs/memory-requirements.md) |
| WCET analysis | [docs/wcet-analysis.md](docs/wcet-analysis.md) |
| Schedulability | [docs/schedulability-analysis.md](docs/schedulability-analysis.md) |
| Real-time lints | [docs/realtime-lint-guide.md](docs/realtime-lint-guide.md) |
| Phase roadmaps | [docs/roadmap/](docs/roadmap/) |

## Quick Reference

### Manual Testing
```bash
# Terminal 1: Router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: Talker
cd examples/native-talker && cargo run

# Terminal 3: Listener
cd examples/native-listener && cargo run
```

### ROS 2 Interop
```bash
# Terminal 1: Router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: nano-ros talker
cd examples/native-talker && cargo run

# Terminal 3: ROS 2 listener
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

### Zephyr Setup
```bash
./scripts/zephyr/setup.sh              # Initialize workspace + create symlink
sudo ./scripts/zephyr/setup-network.sh # Configure TAP network
just test-zephyr                       # Run tests
```

The `zephyr-workspace` symlink points to the actual workspace (default: `../nano-ros-workspace/`).
Scripts use this symlink to locate the workspace. For custom workspace locations, update the symlink:
```bash
ln -sfn /path/to/custom-workspace zephyr-workspace
```

See [docs/zephyr-setup.md](docs/zephyr-setup.md) for details.
