# QEMU Bare-Metal Testing

This document describes how to run bare-metal nros applications in QEMU with network connectivity.

## Overview

QEMU emulates the MPS2-AN385 board (ARM Cortex-M3) with LAN9118 Ethernet, allowing full network testing without physical hardware.

### Network Topology

```
┌─────────────────────────────────────────────────────────────────┐
│                         Host System                              │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐  │
│  │  zenohd     │    │ ROS 2 Node  │    │   Test Runner       │  │
│  │  (router)   │    │ (rmw_zenoh) │    │   (cargo test)      │  │
│  └──────┬──────┘    └──────┬──────┘    └──────────┬──────────┘  │
│         │                  │                       │            │
│         └─────────┬────────┴───────────────────────┘            │
│                   │                                             │
│         ┌─────────▼─────────┐                                   │
│         │   Bridge          │  192.0.2.1                        │
│         │   (qemu-br)       │                                   │
│         └─────────┬─────────┘                                   │
│                   │                                             │
│    ┌──────────────┼──────────────┐                              │
│    │              │              │                              │
│    ▼              ▼              ▼                              │
│ ┌──────────┐ ┌──────────┐ ┌──────────┐                          │
│ │tap-qemu0 │ │tap-qemu1 │ │tap-qemu2 │                          │
│ └────┬─────┘ └────┬─────┘ └────┬─────┘                          │
└──────┼────────────┼────────────┼────────────────────────────────┘
       │            │            │
  ┌────▼────┐  ┌────▼────┐  ┌────▼────┐
  │  QEMU   │  │  QEMU   │  │  QEMU   │
  │ MPS2    │  │ MPS2    │  │ MPS2    │
  │ talker  │  │listener │  │  node   │
  │192.0.2.10│ │192.0.2.11│ │192.0.2.12│
  └─────────┘  └─────────┘  └─────────┘
```

### IP Address Allocation

| Address | Device |
|---------|--------|
| 192.0.2.1 | Host bridge (zenohd) |
| 192.0.2.10 | QEMU node 0 (tap-qemu0) |
| 192.0.2.11 | QEMU node 1 (tap-qemu1) |
| 192.0.2.12+ | Additional QEMU nodes |

## Prerequisites

### Install QEMU

```bash
sudo apt install qemu-system-arm
```

### Verify Installation

```bash
just qemu check
```

## Quick Start

### 1. Build QEMU Examples

```bash
just qemu build-examples
```

### 2. Run Tests (No Networking)

```bash
# Run all QEMU tests
just qemu test

# Run only LAN9118 driver test
just qemu test-lan9118
```

### 3. Setup Networking (For Full Stack Tests)

```bash
# Create TAP bridge (requires sudo)
just qemu setup-network

# Verify setup
just qemu status-network
```

### 4. Run with Networking

```bash
# Terminal 1: Start zenoh router
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2: Run QEMU with network
./scripts/qemu/launch-mps2-an385.sh \
    --tap tap-qemu0 \
    --ip 192.0.2.10 \
    --binary examples/qemu-arm-baremetal/rust/standalone/lan9118/target/thumbv7m-none-eabi/release/qemu-rs-lan9118
```

### 5. Teardown

```bash
just qemu teardown-network
```

## Scripts Reference

### `scripts/qemu/setup-network.sh`

Creates Linux bridge and TAP interfaces for QEMU networking.

```bash
# Create default setup (2 TAP interfaces)
sudo ./scripts/qemu/setup-network.sh

# Create with more TAP interfaces
sudo ./scripts/qemu/setup-network.sh -n 4

# Show current status
./scripts/qemu/setup-network.sh --status

# Tear down
sudo ./scripts/qemu/setup-network.sh --down
```

### `scripts/qemu/launch-mps2-an385.sh`

Launches QEMU mps2-an385 with optional networking.

```bash
# Basic usage (no network)
./scripts/qemu/launch-mps2-an385.sh --binary app.elf

# With networking
./scripts/qemu/launch-mps2-an385.sh \
    --tap tap-qemu0 \
    --ip 192.0.2.10 \
    --binary app.elf

# With GDB debugging
./scripts/qemu/launch-mps2-an385.sh --gdb --binary app.elf
# Then: arm-none-eabi-gdb -ex "target remote :1234" app.elf

# Show full command without executing
./scripts/qemu/launch-mps2-an385.sh --debug --tap tap-qemu0 --binary app.elf
```

## Justfile Recipes

| Recipe | Description |
|--------|-------------|
| `build-examples-qemu` | Build all QEMU examples |
| `test-qemu` | Run all QEMU tests |
| `test-qemu-basic` | Run basic serialization test |
| `test-qemu-lan9118` | Run LAN9118 driver test |
| `setup-qemu-network` | Create TAP bridge (sudo) |
| `teardown-qemu-network` | Remove TAP bridge (sudo) |
| `status-qemu-network` | Show network status |
| `qemu-help` | Show QEMU help |

## Available Examples

### `qemu-rs-test`

Basic nros test on Cortex-M3 (lm3s6965evb machine, no networking).

- Tests CDR serialization
- Tests Node API
- Uses semihosting for output

### `qemu-rs-lan9118`

LAN9118 Ethernet driver test on MPS2-AN385.

- Tests driver initialization
- Tests smoltcp Device trait
- Uses semihosting for output

### `qemu-rs-talker`

TCP client example with smoltcp networking on MPS2-AN385.

- Connects to a TCP server on the host (192.0.2.1:7777)
- Sends 5 test messages
- Demonstrates LAN9118 + smoltcp integration
- Static IP: 192.0.2.10

### `qemu-rs-listener`

TCP server example with smoltcp networking on MPS2-AN385.

- Listens on port 7778
- Echoes received data
- Demonstrates TCP server pattern
- Static IP: 192.0.2.11

## Debugging

### Using GDB

```bash
# Terminal 1: Start QEMU with GDB server
./scripts/qemu/launch-mps2-an385.sh --gdb --binary app.elf

# Terminal 2: Connect GDB
arm-none-eabi-gdb \
    -ex "target remote :1234" \
    -ex "load" \
    app.elf
```

### Semihosting Output

All examples use semihosting for debug output. Output appears directly in the terminal where QEMU runs.

### Common Issues

**TAP interface not found**
```
Error: TAP interface tap-qemu0 does not exist
```
Solution: Run `just qemu setup-network`

**Permission denied on TAP**
```
qemu-system-arm: -netdev tap,id=net0,ifname=tap-qemu0: could not open /dev/net/tun: Permission denied
```
Solution: Ensure TAP is owned by your user. Re-run `sudo ./scripts/qemu/setup-network.sh $USER`

**QEMU not found**
```
Error: qemu-system-arm not found
```
Solution: `sudo apt install qemu-system-arm`

**QEMU version compatibility**

The LAN9118 networking requires QEMU 7.0+ for full TAP networking support. On older versions (e.g., QEMU 6.2), the LAN9118 driver tests work but TAP networking may have issues.

Check your QEMU version:
```bash
qemu-system-arm --version
```

To install a newer QEMU version on Ubuntu:
```bash
sudo add-apt-repository ppa:canonical-server/server-backports
sudo apt update
sudo apt install qemu-system-arm
```

## Hardware Details

### MPS2-AN385 Machine

| Feature | Value |
|---------|-------|
| CPU | ARM Cortex-M3 |
| Flash | 4MB at 0x00000000 |
| SRAM | 4MB at 0x20000000 |
| Ethernet | LAN9118 at 0x40200000 |

### LAN9118 Ethernet Controller

The `lan9118-smoltcp` crate provides a Rust driver for the LAN9118.

```rust
use lan9118_smoltcp::{Lan9118, Config};

let config = Config {
    base_addr: 0x4020_0000,
    mac_addr: [0x02, 0x00, 0x00, 0x00, 0x00, 0x01],
};

let mut eth = unsafe { Lan9118::new(config) }?;
eth.init()?;

// Use with smoltcp
let mut iface = Interface::new(iface_config, &mut eth, instant);
```

## See Also

- [Phase 12 Roadmap](../roadmap/archived/phase-12-qemu-bare-metal-tests.md)
- [Embedded Integration](../design/archived/embedded-integration.md)
- [smoltcp Integration](../design/archived/smoltcp-zenoh-pico-integration.md)
