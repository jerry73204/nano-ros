# Zephyr Development Environment Setup

Complete setup procedure for Zephyr native_sim testing with TAP networking.

## Overview

nano-ros uses a **sibling Zephyr workspace** alongside the repository. A symlink inside nano-ros provides a stable path for scripts to locate the workspace.

```
repos/
├── nano-ros/                     # Your repository
│   ├── scripts/zephyr/
│   │   ├── setup.sh              # Initialize workspace
│   │   ├── setup-network.sh      # Configure TAP interface
│   │   ├── downloads/            # SDK tarball cache (gitignored)
│   │   └── sdk/                  # Installed Zephyr SDK (gitignored)
│   ├── examples/
│   │   ├── zephyr-talker/        # Zephyr pub example
│   │   └── zephyr-listener/      # Zephyr sub example
│   ├── zephyr-workspace -> ../nano-ros-workspace/  # Symlink (gitignored)
│   └── west.yml                  # West manifest
│
└── nano-ros-workspace/           # Created by setup script
    ├── nano-ros -> ../nano-ros   # Symlink to your repo
    ├── zephyr/                   # Zephyr RTOS v3.7.0
    └── modules/                  # HALs, zenoh-pico, zephyr-lang-rust
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
./scripts/zephyr/setup.sh
```

This script automatically:
- Installs `west` and Python tools
- Downloads Zephyr SDK (~1.5 GB) to `scripts/zephyr/downloads/` using aria2c (parallel, resumable)
- Verifies download with sha256sum
- Installs SDK to `scripts/zephyr/sdk/`
- Creates sibling workspace `../nano-ros-workspace/`
- Symlinks nano-ros into the workspace
- Fetches Zephyr RTOS and all modules
- Installs Rust embedded targets
- Creates `env.sh` for environment setup

**Options:**
```bash
./scripts/zephyr/setup.sh --skip-sdk    # Skip SDK download/install
./scripts/zephyr/setup.sh --force       # Recreate existing workspace
```

## Step 2: Configure TAP Network (One-Time, Requires Sudo)

```bash
sudo ./scripts/zephyr/setup-network.sh
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
sudo ./scripts/zephyr/setup-network.sh --down
```

## Step 3: Build and Run Zephyr Examples

```bash
# Source environment
source ../nano-ros-workspace/env.sh

# Build Zephyr talker
cd ../nano-ros-workspace
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker

# Run (no sudo needed)
./build/zephyr/zephyr.exe
```

## Complete E2E Test

```bash
# Terminal 1: Start zenoh router (listen on all interfaces)
zenohd --listen tcp/0.0.0.0:7447

# Terminal 2: Run native subscriber (from nano-ros dir)
cargo run -p zenoh-pico --example sub_test --features std

# Terminal 3: Run Zephyr talker
source ../nano-ros-workspace/env.sh
cd ../nano-ros-workspace
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker
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
| `TAP interface not found` | Run `sudo ./scripts/zephyr/setup-network.sh` |
| `Connection refused` | Ensure zenohd listens on `tcp/0.0.0.0:7447` (not just localhost) |
| `Build fails` | Source environment: `source ../nano-ros-workspace/env.sh` |
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

## Updating the Workspace

To update Zephyr and modules to latest versions specified in `west.yml`:

```bash
cd ../nano-ros-workspace
west update
```

To completely recreate the workspace:

```bash
./scripts/zephyr/setup.sh --force
```
