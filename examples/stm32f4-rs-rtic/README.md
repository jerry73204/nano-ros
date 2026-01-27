# RTIC STM32F4 Example

RTIC 2.x example demonstrating nano-ros with smoltcp network stack on STM32F429ZI.

## Overview

This example shows how to use nano-ros with:
- RTIC 2.x for real-time task scheduling
- smoltcp for TCP/IP networking
- stm32-eth for Ethernet DMA
- zenoh-pico-shim for ROS 2 compatible pub/sub

## Hardware Requirements

- **Board**: NUCLEO-F429ZI (STM32F429ZI with Ethernet)
- **Connection**: RJ45 Ethernet cable to host PC or switch
- **Network**: Static IP configuration (see below)

## Network Configuration

| Setting | Value |
|---------|-------|
| Device IP | 192.168.1.10 |
| Netmask | 255.255.255.0 |
| Gateway | 192.168.1.1 |
| Zenoh Router | tcp/192.168.1.1:7447 |

### Host PC Setup

```bash
# Configure host Ethernet interface
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up

# Start zenoh router
zenohd --listen tcp/0.0.0.0:7447
```

## Building

```bash
cd examples/stm32f4-rs-rtic

# Build release binary
cargo build --release

# Flash and run (requires probe-rs)
cargo run --release
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  RTIC Application                                                    │
│  ├── zenoh_poll task (priority 2) - Polls zenoh for incoming data   │
│  ├── publisher task (priority 1) - Publishes Int32 messages         │
│  └── network task (priority 3) - Polls smoltcp interface            │
├─────────────────────────────────────────────────────────────────────┤
│  nano-ros-node (ShimExecutor, ShimNode, ShimPublisher)              │
├─────────────────────────────────────────────────────────────────────┤
│  zenoh-pico-shim (shim-smoltcp feature)                              │
├─────────────────────────────────────────────────────────────────────┤
│  smoltcp + stm32-eth (Ethernet DMA)                                  │
└─────────────────────────────────────────────────────────────────────┘
```

## Memory Layout

The linker script (`memory.x`) defines:
- `.ethram` section for Ethernet DMA descriptors (must be in SRAM1/2)
- Stack and heap sizes

```
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM   : ORIGIN = 0x20000000, LENGTH = 192K
  ETHRAM: ORIGIN = 0x2001C000, LENGTH = 16K
}
```

## Dependencies

- stm32f4xx-hal 0.21
- stm32-eth 0.8
- smoltcp 0.12
- rtic 2.1
- rtic-monotonics 2.0
- zenoh-pico-shim (smoltcp feature)

## Debugging

```bash
# View defmt logs
cargo run --release

# Or with probe-rs directly
probe-rs run --chip STM32F429ZITx target/thumbv7em-none-eabihf/release/stm32f4-rs-rtic-example
```

## Known Limitations

- Requires cross-compiled zenoh-pico library for ARM (Phase 8.9)
- Currently demonstrates integration pattern; full zenoh session requires hardware testing

## See Also

- `examples/stm32f4-rs-polling/` - Simpler polling-based example
- `examples/stm32f4-rs-smoltcp/` - smoltcp validation without zenoh
- `docs/embedded-integration.md` - Full embedded integration guide
