# Zephyr Talker Example

Zephyr RTOS example demonstrating nano-ros publisher using zenoh-pico.

## Overview

This example shows how to use nano-ros with Zephyr RTOS:
- Rust application using zenoh-pico-shim
- C shim compiled by Zephyr build system
- Network connectivity via Zephyr's network stack
- Publishes `std_msgs/msg/Int32` messages to `/chatter`

## Requirements

- Zephyr SDK installed
- west build tool
- ARM toolchain (for hardware targets)
- zenoh router for testing

## Directory Structure

```
zephyr-rs-talker/
├── CMakeLists.txt      # Zephyr build config
├── prj.conf            # Kconfig options (networking, zenoh-pico)
├── src/
│   └── lib.rs          # Rust application
├── Cargo.toml          # Rust dependencies
├── package.xml         # ROS 2 package manifest
└── generated/          # Generated message bindings
```

## Building

### For native_sim (Linux simulation)

```bash
# From Zephyr workspace root
west build -b native_sim/native/64 nano-ros/examples/zephyr-rs-talker
./build/zephyr/zephyr.exe
```

### For QEMU

```bash
west build -b qemu_cortex_m3 nano-ros/examples/zephyr-rs-talker
west build -t run
```

### For Hardware (e.g., NUCLEO-F429ZI)

```bash
west build -b nucleo_f429zi nano-ros/examples/zephyr-rs-talker
west flash
```

## Network Configuration

The example uses TAP networking for native_sim:

```bash
# Create TAP interface (one-time setup)
sudo ./scripts/setup-zephyr-network.sh
```

Default configuration:
- Device IP: 192.0.2.1 (QEMU/native_sim) or board-specific
- Router: tcp/192.0.2.2:7447

## Testing with Native Subscriber

```bash
# Terminal 1: Start zenoh router
zenohd --listen tcp/0.0.0.0:7447

# Terminal 2: Run native listener
cargo run -p native-rs-listener --features zenoh -- --tcp 127.0.0.1:7447

# Terminal 3: Run Zephyr talker (from west workspace)
./build/zephyr/zephyr.exe
```

## C Shim Integration

The C shim (`zenoh_shim.c`) is provided by `zenoh-pico-shim-sys` and compiled
by Zephyr's CMake. The Rust code uses FFI to call the shim:

```rust
extern "C" {
    fn zenoh_shim_init_config(locator: *const c_char) -> i32;
    fn zenoh_shim_open_session() -> i32;
    fn zenoh_shim_declare_publisher(keyexpr: *const c_char) -> i32;
    fn zenoh_shim_publish(handle: i32, data: *const u8, len: usize) -> i32;
}
```

## Kconfig Options

Key options in `prj.conf`:

```
# Networking
CONFIG_NETWORKING=y
CONFIG_NET_TCP=y
CONFIG_NET_IPV4=y

# zenoh-pico
CONFIG_ZENOH_PICO=y
CONFIG_HEAP_MEM_POOL_SIZE=16384

# Rust support
CONFIG_RUST=y
```

## See Also

- `examples/zephyr-rs-listener/` - Subscriber example
- `docs/embedded-integration.md` - Full embedded integration guide
- Zephyr documentation: https://docs.zephyrproject.org/
