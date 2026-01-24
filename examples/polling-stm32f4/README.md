# Polling STM32F4 Example

Bare-metal polling example demonstrating nano-ros with smoltcp network stack on STM32F429ZI.

## Overview

This example shows how to use nano-ros with:
- Simple main loop polling (no executor/RTOS)
- smoltcp for TCP/IP networking
- stm32-eth for Ethernet DMA
- DWT cycle counter for timing
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
cd examples/polling-stm32f4

# Build release binary
cargo build --release

# Flash and run (requires probe-rs)
cargo run --release
```

## Architecture

```rust
// Simple polling loop - no executor needed
fn main() -> ! {
    // Initialize hardware, Ethernet, smoltcp

    loop {
        let now_ms = dwt_cycles_to_ms();

        // Poll smoltcp interface
        iface.poll(timestamp, &mut eth_dma, &mut sockets);

        // Poll zenoh periodically
        if now_ms - last_zenoh_poll >= POLL_INTERVAL_MS {
            zenoh_shim_poll(10);
            last_zenoh_poll = now_ms;
        }

        // Publish message periodically
        if now_ms - last_publish >= PUBLISH_INTERVAL_MS {
            publisher.publish(&Int32 { data: counter })?;
            counter += 1;
            last_publish = now_ms;
        }
    }
}
```

## Memory Layout

Same as rtic-stm32f4 example - see `memory.x` for details.

## Dependencies

- stm32f4xx-hal 0.21
- stm32-eth 0.8
- smoltcp 0.12
- cortex-m 0.7
- cortex-m-rt 0.7
- zenoh-pico-shim (smoltcp feature)

## Debugging

```bash
# View defmt logs
cargo run --release

# Or with probe-rs directly
probe-rs run --chip STM32F429ZITx target/thumbv7em-none-eabihf/release/polling-stm32f4-example
```

## Comparison with RTIC Example

| Aspect | Polling | RTIC |
|--------|---------|------|
| Complexity | Lower | Higher |
| Preemption | None | Priority-based |
| Timing | Best-effort | Guaranteed response |
| Code Size | Smaller | Larger |
| Use Case | Simple sensors | Complex robots |

## Known Limitations

- Requires cross-compiled zenoh-pico library for ARM (Phase 8.9)
- Currently demonstrates integration pattern; full zenoh session requires hardware testing

## See Also

- `examples/rtic-stm32f4/` - RTIC-based example with priority scheduling
- `examples/smoltcp-test/` - smoltcp validation without zenoh
- `docs/embedded-integration.md` - Full embedded integration guide
