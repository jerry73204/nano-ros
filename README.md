# nano-ros

A lightweight ROS 2 client library for embedded systems, written in Rust.

## Features

- `no_std` compatible for bare-metal and RTOS targets
- ROS 2 interoperability via rmw_zenoh
- Runs on Zephyr RTOS and native Linux
- Zero-copy CDR serialization

## Requirements

- Rust 1.75+
- zenohd router
- (Optional) ROS 2 Humble with rmw_zenoh_cpp

## Quick Start

### 1. Clone and Build

```bash
git clone https://github.com/jerry73204/nano-ros.git
cd nano-ros
cargo build --release -p native-talker -p native-listener --features zenoh
```

### 2. Start Zenoh Router

```bash
zenohd --listen tcp/127.0.0.1:7447
```

### 3. Run Demo

Terminal 1 - Talker:
```bash
cargo run -p native-talker --release --features zenoh
```

Terminal 2 - Listener:
```bash
cargo run -p native-listener --release --features zenoh
```

## ROS 2 Interoperability

nano-ros can communicate with ROS 2 nodes using rmw_zenoh.

```bash
# Terminal 1: zenohd
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: nano-ros talker
cargo run -p native-talker --release --features zenoh

# Terminal 3: ROS 2 listener
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

## Zephyr RTOS

For embedded targets, see [docs/roadmap/phase-2-zephyr-qemu.md](docs/roadmap/phase-2-zephyr-qemu.md).

```bash
# Setup (one-time)
./zephyr/setup.sh

# Build for native_sim
cd ~/nano-ros-workspace
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs

# Run
./build/zephyr/zephyr.exe
```

## Project Structure

```
crates/
├── nano-ros-core       # Node, Publisher, Subscriber
├── nano-ros-serdes     # CDR serialization
├── nano-ros-types      # std_msgs, geometry_msgs, etc.
├── nano-ros-transport  # Zenoh transport layer
└── zenoh-pico          # Zenoh-pico Rust bindings
```

## Status

| Feature        | Status   |
|----------------|----------|
| Pub/Sub        | Complete |
| ROS 2 Interop  | Complete |
| Zephyr Support | Complete |
| Services       | Planned  |
| Parameters     | Planned  |

## License

Apache-2.0 OR MIT
