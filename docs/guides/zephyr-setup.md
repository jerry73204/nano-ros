# Zephyr Development Environment Setup

Complete setup procedure for Zephyr `native_sim` testing. Networking uses
**NSOS** (Native Sim Offloaded Sockets) — each socket call is forwarded to
the host kernel, so tests run on `127.0.0.1` without TAP devices, bridges,
or `sudo`.

## Overview

nros uses a **sibling Zephyr workspace** alongside the repository. A symlink inside nros provides a stable path for scripts to locate the workspace.

```
repos/
├── nros/                     # Your repository
│   ├── scripts/zephyr/
│   │   ├── setup.sh              # Initialize workspace
│   │   ├── downloads/            # SDK tarball cache (gitignored)
│   │   └── sdk/                  # Installed Zephyr SDK (gitignored)
│   ├── zephyr/                   # Zephyr module definition
│   │   ├── Kconfig               # RMW backend, API selection, tuning
│   │   ├── CMakeLists.txt        # Transport C sources + nros-c build
│   │   └── cmake/                # nros_cargo_build(), nros_generate_interfaces()
│   ├── examples/zephyr/
│   │   ├── rust/                 # Rust examples; RMW selected by prj-<rmw>.conf
│   │   ├── c/                    # C examples; RMW selected by prj-<rmw>.conf
│   │   ├── cpp/                  # C++ examples; RMW selected by prj-<rmw>.conf
│   │   └── cmake/                # Shared Zephyr example helpers
│   ├── zephyr-workspace -> ../nano-ros-workspace/  # Symlink (gitignored)
│   └── west.yml                  # West manifest
│
└── nano-ros-workspace/           # Created by setup script
    ├── nros -> ../nros   # Symlink to your repo
    ├── zephyr/                   # Zephyr RTOS v3.7.0
    └── modules/                  # HALs, zephyr-lang-rust
```

The `zephyr-workspace` symlink allows scripts to find the workspace without hardcoding paths.
For custom workspace locations, update the symlink:
```bash
ln -sfn /path/to/custom-workspace zephyr-workspace
```

## Prerequisites

Install system packages (Ubuntu/Debian):
```bash
sudo apt install python3 python3-pip python3-venv cmake ninja-build aria2 git
```

Install Rust (if not already):
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

## Step 1: Initialize Workspace (One-Time)

```bash
just setup zephyr
```

This recipe automatically:
- Installs `west` and Python tools
- Downloads Zephyr SDK (~1.5 GB) to `scripts/zephyr/downloads/` using aria2c (parallel, resumable)
- Verifies download with sha256sum
- Installs SDK to `scripts/zephyr/sdk/`
- Creates sibling workspace `../nano-ros-workspace/`
- Symlinks nros into the workspace
- Fetches Zephyr RTOS and all modules
- Installs Rust embedded targets
- Creates `env.sh` for environment setup

**Options:**
```bash
just setup zephyr --skip-sdk    # Skip SDK download/install
just setup zephyr --force       # Recreate existing workspace
```

## Step 2: Networking

No network setup is required. `native_sim` uses the NSOS offloaded-sockets
driver, enabled by `boards/native_sim_native_64.conf` in each example:

```
CONFIG_ETH_NATIVE_POSIX=n
CONFIG_NET_SOCKETS_OFFLOAD=y
CONFIG_NET_NATIVE_OFFLOADED_SOCKETS=y
```

With NSOS, Zephyr's socket API goes straight to host syscalls. Bind to
`127.0.0.1` and reach `zenohd` / the XRCE Agent on the host loopback just
like any other native test. Multiple `native_sim` processes can coexist
without bridge configuration.

## Step 3: Build and Run Zephyr Examples

```bash
# Source environment
source ../nano-ros-workspace/env.sh

# Build Zephyr talker (Rust + Zenoh)
cd ../nano-ros-workspace
west build -b native_sim/native/64 nros/examples/zephyr/rust/talker \
  -- -DEXTRA_CONF_FILE=prj-zenoh.conf

# Run (no sudo needed)
./build/zephyr/zephyr.exe
```

## Running Multiple Instances (Talker + Listener)

Zephyr native_sim uses a deterministic entropy source. Without unique seeds,
multiple instances generate the **same Zenoh session ID**, causing the router
to reject the second session (`Close(MAX_LINKS)`).

**Using just recipes** (recommended — each invocation gets a random seed via `$RANDOM`):

```bash
# Terminal 1
just zephyr zenohd

# Terminal 2
just zephyr listener

# Terminal 3
just zephyr talker
```

**Manual launch** — pass `--seed=<unique>` to each instance:

```bash
# Terminal 1
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7456"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2
zephyr-workspace/build-listener/zephyr/zephyr.exe --seed=2000

# Terminal 3
zephyr-workspace/build-talker/zephyr/zephyr.exe --seed=1000
```

Without `--seed`, both instances produce the same ZID and the router closes
the second connection. See [docs/research/zephyr-native-sim-timing.md](../research/zephyr-native-sim-timing.md) for the full investigation.

## RMW Backend Selection

nros supports two RMW backends on Zephyr, selected via `prj.conf`:

### Zenoh (default)

Connects to a zenoh router. Requires POSIX API for zenoh-pico threads.

```ini
CONFIG_NROS=y
# CONFIG_NROS_RMW_ZENOH=y  # default, can be omitted
CONFIG_NROS_ZENOH_LOCATOR="tcp/127.0.0.1:7456"
CONFIG_POSIX_API=y
CONFIG_MAX_PTHREAD_MUTEX_COUNT=32
CONFIG_MAX_PTHREAD_COND_COUNT=16
```

### XRCE-DDS

Connects to a Micro-XRCE-DDS Agent over UDP. Requires BSD sockets.

```ini
CONFIG_NROS=y
CONFIG_NROS_RMW_XRCE=y
CONFIG_NROS_XRCE_AGENT_ADDR="127.0.0.1"
CONFIG_NROS_XRCE_AGENT_PORT=2018
CONFIG_NET_SOCKETS=y
```

## API Selection

Choose between Rust and C APIs via `prj.conf`:

### Rust API (default)

```ini
CONFIG_NROS_RUST_API=y
CONFIG_RUST=y
CONFIG_RUST_ALLOC=y
```

CMakeLists.txt uses `rust_cargo_application()`:
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_example)
rust_cargo_application()
```

### C API

```ini
CONFIG_NROS_C_API=y
```

CMakeLists.txt uses `nros_generate_interfaces()`:
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_example)
nros_generate_interfaces(std_msgs "msg/Int32.msg")
target_sources(app PRIVATE src/main.c)
```

## Kconfig Reference

All options are under `menuconfig NROS` in `zephyr/Kconfig`.

### Common Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS` | bool | n | Enable nros module |
| `CONFIG_NROS_RUST_API` | bool | y | Use Rust API |
| `CONFIG_NROS_C_API` | bool | n | Use C API |
| `CONFIG_NROS_DOMAIN_ID` | int | 0 | ROS 2 domain ID |
| `CONFIG_NROS_INIT_DELAY_MS` | int | 2000 | Network init wait (ms) |

### Zenoh Options (visible when `CONFIG_NROS_RMW_ZENOH=y`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_ZENOH_LOCATOR` | string | `"tcp/127.0.0.1:7456"` | Router address |
| `CONFIG_NROS_ZENOH_MULTI_THREAD` | bool | y | Zenoh-pico multithreading |
| `CONFIG_NROS_ZENOH_PUBLICATION` | bool | y | Publication support |
| `CONFIG_NROS_ZENOH_SUBSCRIPTION` | bool | y | Subscription support |
| `CONFIG_NROS_ZENOH_QUERY` | bool | y | Service client support |
| `CONFIG_NROS_ZENOH_QUERYABLE` | bool | y | Service server support |
| `CONFIG_NROS_ZENOH_LINK_TCP` | bool | y | TCP transport link |
| `CONFIG_NROS_MAX_PUBLISHERS` | int | 8 | Max concurrent publishers |
| `CONFIG_NROS_MAX_SUBSCRIBERS` | int | 8 | Max concurrent subscribers |
| `CONFIG_NROS_MAX_QUERYABLES` | int | 8 | Max concurrent queryables |
| `CONFIG_NROS_FRAG_MAX_SIZE` | int | 2048 | Max reassembled message size |
| `CONFIG_NROS_BATCH_UNICAST_SIZE` | int | 1024 | Max unicast batch size |
| `CONFIG_NROS_SUBSCRIBER_BUFFER_SIZE` | int | -1 (derive) | Per-subscriber buffer, small payload class |
| `CONFIG_NROS_MAX_LARGE_SUBSCRIBERS` | int | -1 (derive) | Blocks in the large payload class |
| `CONFIG_NROS_SUBSCRIBER_LARGE_SIZE` | int | -1 (derive) | Per-sample capacity of the large class |
| `CONFIG_NROS_SERVICE_BUFFER_SIZE` | int | 1024 | Per-service buffer |

`-1` on the three rows above, and on `CONFIG_NROS_SUBSCRIPTION_BUFFER_SIZE`,
means the build derives the number from this image's message-bound inventory
(phase-403 W8 / issue 0963) rather than a person choosing one. Anything you
state wins; the derived number is an upper bound over the interface closure you
LINK; and one unbounded type in that closure refuses the derivation outright and
says which. Full explanation, and what to do when the type it names is not one
you receive, in the [book](../../book/src/getting-started/zephyr.md) and
[embedded-tuning.md](embedded-tuning.md).

### XRCE Options (visible when `CONFIG_NROS_RMW_XRCE=y`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_XRCE_AGENT_ADDR` | string | `"127.0.0.1"` | Agent IP address |
| `CONFIG_NROS_XRCE_AGENT_PORT` | int | 2018 | Agent UDP port |
| `CONFIG_NROS_XRCE_TRANSPORT_MTU` | int | 512 | Transport MTU |
| `CONFIG_NROS_XRCE_MAX_SUBSCRIBERS` | int | -1 (derive) | Max concurrent subscribers |
| `CONFIG_NROS_XRCE_MAX_SERVICE_SERVERS` | int | -1 (derive) | Max service servers |
| `CONFIG_NROS_XRCE_MAX_SERVICE_CLIENTS` | int | 4 | Max service clients |
| `CONFIG_NROS_XRCE_SUBSCRIBER_RING_DEPTH` | int | 32 | Queued samples per subscriber |
| `CONFIG_NROS_XRCE_BUFFER_SIZE` | int | 1024 | Per-slot buffer size |
| `CONFIG_NROS_XRCE_STREAM_HISTORY` | int | 16 | Reliable stream depth (2–16) |

### C API Options (visible when `CONFIG_NROS_C_API=y`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_C_MAX_HANDLES` | int | 16 | Max executor handles |
| `CONFIG_NROS_C_MAX_SUBSCRIPTIONS` | int | 8 | Max subscriptions |
| `CONFIG_NROS_C_MAX_TIMERS` | int | 8 | Max timers |
| `CONFIG_NROS_C_MAX_SERVICES` | int | 4 | Max services |

## E2E Testing

```bash
# Zenoh examples
just zephyr build           # Build Rust zenoh examples
just zephyr build-c         # Build C zenoh examples
just zephyr test            # Run zenoh E2E tests

# XRCE examples
just zephyr build-xrce      # Build all XRCE examples (Rust + C)
just zephyr test-xrce       # Run XRCE E2E tests

# All examples
just zephyr build-all       # Build everything
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `west: command not found` | Run `pip3 install --user west` and add `~/.local/bin` to PATH |
| `Connection refused` | Start `zenohd` / `MicroXRCEAgent` on the host loopback (e.g. `tcp/127.0.0.1:7456`) |
| `Build fails` | Source environment: `source ../nano-ros-workspace/env.sh` |
| `XRCE Agent not found` | Install: `just setup` (installs MicroXRCEAgent) |
| Zenoh mutex exhaustion | Increase `CONFIG_MAX_PTHREAD_MUTEX_COUNT` (default 5 is too low) |
| `z_declare_publisher failed: -128` with two instances | Duplicate ZID — pass unique `--seed` to each native_sim instance |

## Network Architecture

With NSOS, Zephyr sockets are forwarded to host syscalls — there is no
emulated L2/L3 stack to configure, no static IP, and no bridge.

```
┌─────────────────────────────────────────────────────────────┐
│                      Host (Linux)                            │
│                                                              │
│   ┌────────────────────┐       ┌────────────────────────┐   │
│   │ zephyr.exe talker  │       │ zephyr.exe listener    │   │
│   │ (native_sim+NSOS)  │       │ (native_sim+NSOS)      │   │
│   └─────────┬──────────┘       └──────────┬─────────────┘   │
│             │ host socket() via NSOS       │                │
│             ▼                              ▼                │
│                 127.0.0.1 (loopback)                        │
│             │                              │                │
│             ▼                              ▼                │
│   ┌────────────────────┐       ┌────────────────────────┐   │
│   │ zenohd             │       │ MicroXRCEAgent         │   │
│   │ tcp/127.0.0.1:7456 │       │ udp/127.0.0.1:2018     │   │
│   └────────────────────┘       └────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Updating the Workspace

To update Zephyr and modules to latest versions specified in `west.yml`:

```bash
cd ../nano-ros-workspace
west update
```

To completely recreate the workspace:

```bash
just setup zephyr --force
```
