# nano-ros Zephyr Service Client (Rust)

A ROS 2 compatible service client running on Zephyr RTOS using nano-ros.

## Overview

This example demonstrates:
- Service client using zenoh-pico query (z_get)
- AddTwoInts service (from example_interfaces)
- Blocking request/response calls
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
4. Start a service server (see below)

## Build

```bash
source ~/nano-ros-workspace/env.sh
west build -b native_sim/native/64 nano-ros/examples/zephyr-rs-service-client
```

## Run

First, start a service server:

```bash
# Option 1: Native service server
cd nano-ros/examples/native-rs-service-server
cargo run --features zenoh

# Option 2: Zephyr service server (in another terminal)
west build -b native_sim/native/64 nano-ros/examples/zephyr-rs-service-server
./build/zephyr/zephyr.exe
```

Then run the client:

```bash
./build/zephyr/zephyr.exe
```

The service client will:
1. Connect to the zenoh router at 192.0.2.2:7447
2. Send AddTwoInts requests every 2 seconds
3. Print the responses

## Network Configuration

- Zephyr IP: 192.0.2.1
- Host/Router IP: 192.0.2.2
- TAP interface: zeth0

See `scripts/setup-zephyr-network.sh` for network setup.

## Notes

- The client uses a 5-second timeout for service calls
- If no server is running, the client will report timeout errors
- Both client and server must be connected to the same zenoh router
