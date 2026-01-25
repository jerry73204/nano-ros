# Zephyr Development Environment Setup

Complete setup procedure for Zephyr native_sim testing with TAP networking.

## Prerequisites

Install system packages (Ubuntu/Debian):
```bash
sudo apt install python3 python3-pip python3-venv cmake ninja-build wget git
```

Install Rust (if not already):
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

## Step 1: Initialize Zephyr Workspace (One-Time)

```bash
cd /path/to/nano-ros
./zephyr/setup.sh
```

This script automatically:
- Installs `west` and Python tools
- Downloads and installs Zephyr SDK (~1.5 GB)
- Initializes workspace at `~/nano-ros-workspace`
- Fetches Zephyr RTOS and all modules
- Installs Rust embedded targets
- Creates `env.sh` for environment setup

**Options:**
```bash
./zephyr/setup.sh --workspace /custom/path  # Custom workspace location
./zephyr/setup.sh --skip-sdk                # Skip SDK if already installed
./zephyr/setup.sh --force                   # Recreate existing workspace
```

## Step 2: Configure TAP Network (One-Time, Requires Sudo)

```bash
sudo ./scripts/setup-zephyr-network.sh
```

This creates a TAP interface for Zephyr ↔ Host communication:

| Interface | IP Address | Role |
|-----------|------------|------|
| `zeth` (host side) | 192.0.2.2 | Host applications (zenohd) |
| Zephyr internal | 192.0.2.1 | Zephyr application |

The interface is owned by your user, so Zephyr runs **without sudo** afterward.

**Verify setup:**
```bash
ip addr show zeth
# Should show: inet 192.0.2.2/24
```

**Teardown (if needed):**
```bash
sudo ./scripts/setup-zephyr-network.sh --down
```

## Step 3: Build and Run Zephyr Examples

```bash
# Source environment (required for each terminal session)
source ~/nano-ros-workspace/env.sh

# Build Zephyr talker
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs

# Run (no sudo needed)
./build/zephyr/zephyr.exe
```

## Complete E2E Test

```bash
# Terminal 1: Start zenoh router (listen on all interfaces)
zenohd --listen tcp/0.0.0.0:7447

# Terminal 2: Run native subscriber
cd /path/to/nano-ros
cargo run -p zenoh-pico --example sub_test --features std

# Terminal 3: Run Zephyr talker
source ~/nano-ros-workspace/env.sh
cd ~/nano-ros-workspace
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs
./build/zephyr/zephyr.exe
```

The Zephyr talker connects to zenohd at 192.0.2.2:7447 and publishes Int32 messages.

## Run Zephyr Integration Tests

```bash
# Automated test (checks prerequisites, builds, runs)
just test-zephyr

# Or directly
./tests/zephyr/run.sh --verbose
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `west: command not found` | Run `pip3 install --user west` and add `~/.local/bin` to PATH |
| `TAP interface not found` | Run `sudo ./scripts/setup-zephyr-network.sh` |
| `Connection refused` | Ensure zenohd listens on `tcp/0.0.0.0:7447` (not just localhost) |
| `Build fails` | Source environment: `source ~/nano-ros-workspace/env.sh` |
| `Permission denied on zeth` | TAP interface owned by different user, re-run setup script |

## Network Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         Host (Linux)                         │
│                                                             │
│  ┌─────────────┐    TCP/7447    ┌─────────────────────────┐ │
│  │   zenohd    │◄──────────────►│  Native nano-ros apps   │ │
│  │ 0.0.0.0:7447│                │  (talker, listener)     │ │
│  └──────┬──────┘                └─────────────────────────┘ │
│         │                                                    │
│         │ TCP/7447                                           │
│         ▼                                                    │
│  ┌─────────────┐                                            │
│  │ TAP: zeth   │                                            │
│  │ 192.0.2.2   │                                            │
│  └──────┬──────┘                                            │
└─────────┼────────────────────────────────────────────────────┘
          │ Virtual Ethernet
          ▼
┌─────────────────┐
│ Zephyr native_sim│
│   192.0.2.1     │
│                 │
│ nano-ros app    │
│ connects to     │
│ 192.0.2.2:7447  │
└─────────────────┘
```
