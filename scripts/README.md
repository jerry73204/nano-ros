# Scripts

Utility scripts for nano-ros development and debugging.

> **Note:** Integration tests have been moved to `tests/`. See `tests/README.md`.

## Directory Structure

```
scripts/
├── qemu/           # QEMU testing utilities
│   ├── run-multi-node-test.sh
│   ├── setup-qemu-network.sh
│   └── teardown-qemu-network.sh
└── debug/          # Debugging utilities
    ├── capture-ros2-keyexpr.sh
    ├── check-ros2-hash.sh
    ├── check-ros2-types.py
    ├── compare-keyexprs.sh
    ├── debug-keyexpr.sh
    ├── debug-liveliness.sh
    └── debug-zenoh-keys.py
```

## QEMU Scripts

Scripts for multi-node testing with QEMU.

### setup-qemu-network.sh
Sets up network bridge for QEMU VM communication.

### teardown-qemu-network.sh
Cleans up network bridge after testing.

### run-multi-node-test.sh
Runs multi-node test with QEMU Cortex-M3 and native x86 nodes.

## Debug Scripts

Utilities for debugging RMW protocol issues.

### capture-ros2-keyexpr.sh
Captures actual key expressions used by ROS 2 when publishing.

### check-ros2-hash.sh
Checks what RIHS01 hash ROS 2 uses for a given message type.

### check-ros2-types.py
Python script to inspect ROS 2 type information.

### compare-keyexprs.sh
Compares key expressions between nano-ros and ROS 2.

### debug-keyexpr.sh
Debug script to see what keyexpr nano-ros publishes on.

### debug-liveliness.sh
Captures and analyzes liveliness tokens from nano-ros and ROS 2.

### debug-zenoh-keys.py
Python script to analyze zenoh key expressions.

## Usage

```bash
# QEMU testing
./scripts/qemu/setup-qemu-network.sh
./scripts/qemu/run-multi-node-test.sh
./scripts/qemu/teardown-qemu-network.sh

# Debugging
./scripts/debug/debug-keyexpr.sh
./scripts/debug/debug-liveliness.sh
```

## See Also

- `tests/` - Integration tests
- `docs/rmw_zenoh_interop.md` - RMW protocol documentation
