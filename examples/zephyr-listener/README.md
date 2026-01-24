# Zephyr Listener Example

Zephyr RTOS example demonstrating nano-ros subscriber using zenoh-pico.

## Overview

This example shows how to use nano-ros with Zephyr RTOS:
- Rust application using zenoh-pico-shim
- C shim compiled by Zephyr build system
- Network connectivity via Zephyr's network stack
- Subscribes to `std_msgs/msg/Int32` messages on `/chatter`

## Requirements

- Zephyr SDK installed
- west build tool
- ARM toolchain (for hardware targets)
- zenoh router for testing

## Directory Structure

```
zephyr-listener/
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
west build -b native_sim/native/64 nano-ros/examples/zephyr-listener
./build/zephyr/zephyr.exe
```

### For QEMU

```bash
west build -b qemu_cortex_m3 nano-ros/examples/zephyr-listener
west build -t run
```

### For Hardware (e.g., NUCLEO-F429ZI)

```bash
west build -b nucleo_f429zi nano-ros/examples/zephyr-listener
west flash
```

## Network Configuration

Same as zephyr-talker - see that README for TAP interface setup.

## Testing with Native Talker

```bash
# Terminal 1: Start zenoh router
zenohd --listen tcp/0.0.0.0:7447

# Terminal 2: Run Zephyr listener (from west workspace)
./build/zephyr/zephyr.exe

# Terminal 3: Run native talker
cargo run -p native-talker --features zenoh -- --tcp 127.0.0.1:7447
```

## Subscriber Callback

The subscriber uses a callback pattern:

```rust
extern "C" fn on_message(data: *const u8, len: usize, _ctx: *mut c_void) {
    let slice = unsafe { core::slice::from_raw_parts(data, len) };
    // Deserialize and process Int32 message
    if let Ok(msg) = Int32::deserialize(&slice[4..]) {
        log::info!("Received: data={}", msg.data);
    }
}
```

## See Also

- `examples/zephyr-talker/` - Publisher example
- `docs/embedded-integration.md` - Full embedded integration guide
- Zephyr documentation: https://docs.zephyrproject.org/
