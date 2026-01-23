# smoltcp TCP Echo Server Example

Standalone validation of smoltcp + stm32-eth on NUCLEO-F429ZI.

This example validates that the network stack works correctly before integrating
with zenoh-pico in Phase 8.4.

## Hardware Requirements

- **Board**: NUCLEO-F429ZI (STM32F429ZI with Ethernet)
- **Connection**: RJ45 Ethernet cable to host PC or switch

## Network Configuration

### Static IP (default)

| Setting | Value |
|---------|-------|
| Device IP | 192.168.1.10 |
| Netmask | 255.255.255.0 |
| Gateway | 192.168.1.1 |
| Echo Port | 7 |

### Host PC Setup

Configure your PC's Ethernet interface:
```bash
# Linux
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up

# Or use NetworkManager
nmcli con add type ethernet con-name nano-ros ifname eth0 ip4 192.168.1.1/24
```

## Building

```bash
cd examples/smoltcp-test

# Build
cargo build --release

# Flash and run (requires probe-rs)
cargo run --release
```

## Testing

### TCP Echo Test

From host PC:
```bash
# Using netcat
nc 192.168.1.10 7
# Type text and press Enter - should see it echoed back

# Or using telnet
telnet 192.168.1.10 7
```

### Ping Test

```bash
ping 192.168.1.10
```

## Debugging

The example uses defmt for logging. View logs with:
```bash
# Using probe-rs
cargo run --release

# Or with defmt-print
probe-rs run --chip STM32F429ZITx target/thumbv7em-none-eabihf/release/smoltcp-test
```

## Memory Usage

| Component | Size |
|-----------|------|
| Ethernet DMA RX descriptors | ~0.5 KB |
| Ethernet DMA TX descriptors | ~0.5 KB |
| TCP RX buffer | 2 KB |
| TCP TX buffer | 2 KB |
| smoltcp Interface | ~1 KB |
| **Total** | ~6 KB |

## Troubleshooting

### No link LED

- Check Ethernet cable connection
- Verify board's Ethernet PHY is working (LED should blink on activity)

### Cannot ping

- Verify IP address configuration on host PC
- Check that firewall allows ICMP
- Use Wireshark to check for ARP requests/replies

### TCP connection refused

- Verify the port is correct (7)
- Check defmt logs for errors
- Ensure the echo server task is running

## Next Steps

Once this example works, proceed to Phase 8.4 to integrate smoltcp with
zenoh-pico-shim for the full nano-ros embedded networking stack.
