# RMW Zenoh Interoperability Tests

Tests for communication between nano-ros and ROS 2 nodes using rmw_zenoh_cpp.

## Tests

### nano2ros.sh
Tests nano-ros talker → ROS 2 listener communication.

### ros2nano.sh
Tests ROS 2 talker → nano-ros listener communication.

### matrix.sh
Tests all 4 combinations in a single run and displays results in a matrix.

## Usage

```bash
# Individual tests
./tests/rmw-interop/nano2ros.sh
./tests/rmw-interop/ros2nano.sh

# Full matrix
./tests/rmw-interop/matrix.sh

# With options
./tests/rmw-interop/matrix.sh --ros-distro jazzy --verbose
```

## Requirements

- zenohd in PATH
- ROS 2 Humble (or Jazzy with `--ros-distro jazzy`)
- rmw_zenoh_cpp installed (`sudo apt install ros-humble-rmw-zenoh-cpp`)
- nano-ros built with zenoh feature

## Architecture

```
                    ┌─────────┐
                    │ zenohd  │
                    │ router  │
                    └────┬────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
┌───────────────┐  ┌───────────────┐  ┌───────────────┐
│  nano-ros     │  │  nano-ros     │  │  ROS 2 node   │
│  talker       │  │  listener     │  │  (rmw_zenoh)  │
└───────────────┘  └───────────────┘  └───────────────┘
```

## Communication Matrix

| Talker | Listener | Test File |
|--------|----------|-----------|
| nano-ros | nano-ros | nano2nano/run.sh |
| nano-ros | ROS 2 | rmw-interop/nano2ros.sh |
| ROS 2 | nano-ros | rmw-interop/ros2nano.sh |
| ROS 2 | ROS 2 | rmw-interop/matrix.sh |

## Troubleshooting

### ROS 2 not receiving messages

1. Check QoS settings match (BEST_EFFORT/VOLATILE)
2. Ensure domain ID is 0
3. Verify type hash format (Humble uses `TypeHashNotSupported`)

### nano-ros not receiving messages

1. Verify wildcard subscriber is working
2. Check liveliness tokens are being declared
3. Enable debug logging: `RUST_LOG=debug`
