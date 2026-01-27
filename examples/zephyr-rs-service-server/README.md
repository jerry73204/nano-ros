# nano-ros Zephyr Service Server (Rust)

A ROS 2 compatible service server running on Zephyr RTOS using nano-ros.

## Overview

This example demonstrates:
- Service server using zenoh-pico queryable
- AddTwoInts service (from example_interfaces)
- Callback-based request handling
- CDR serialization for ROS 2 compatibility

## Architecture

```
Rust Application (src/lib.rs)
    └── zenoh-pico-shim (Rust wrapper)
        └── zenoh_shim.c (C shim, compiled by Zephyr)
            └── zenoh-pico (C library)
                └── Zephyr network stack
```

## Prerequisites

1. Set up the Zephyr workspace (see main README)
2. Set up TAP interface: `sudo ./scripts/setup-zephyr-network.sh`
3. Start zenoh router: `zenohd --listen tcp/0.0.0.0:7447`

## Build

```bash
source ~/nano-ros-workspace/env.sh
west build -b native_sim/native/64 nano-ros/examples/zephyr-rs-service-server
```

## Run

```bash
./build/zephyr/zephyr.exe
```

The service server will:
1. Connect to the zenoh router at 192.0.2.2:7447
2. Declare service server for `demo/add_two_ints`
3. Wait for and process service requests

## Testing

From another terminal, run the native service client:

```bash
cd nano-ros/examples/native-rs-service-client
cargo run --features zenoh
```

Or use a zenoh-based query:

```bash
# Using zenoh CLI tools
z_get -k "demo/add_two_ints" --payload "<CDR-encoded-request>"
```

## Network Configuration

- Zephyr IP: 192.0.2.1
- Host/Router IP: 192.0.2.2
- TAP interface: zeth0

See `scripts/setup-zephyr-network.sh` for network setup.
