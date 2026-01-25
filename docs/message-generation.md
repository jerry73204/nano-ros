# Message Binding Generation

nano-ros uses generated Rust bindings for ROS 2 message types. The `cargo nano-ros generate` command generates `no_std` compatible bindings from `package.xml` dependencies.

## Overview

The binding generator lives in `colcon-nano-ros/packages/cargo-nano-ros/` and provides:
- Standalone `cargo nano-ros` subcommand for generating bindings
- Pure Rust, `no_std` compatible output using `heapless` types
- Automatic dependency resolution via ament index
- `.cargo/config.toml` generation for crate patches

## Prerequisites

1. **ROS 2 environment sourced** - Required for ament index access
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **package.xml in project root** - Declares ROS interface dependencies
   ```xml
   <?xml version="1.0"?>
   <package format="3">
     <name>my_package</name>
     <version>0.1.0</version>
     <description>My nano-ros package</description>
     <maintainer email="dev@example.com">Developer</maintainer>
     <license>Apache-2.0</license>
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <export>
       <build_type>ament_cargo</build_type>
     </export>
   </package>
   ```

3. **cargo-nano-ros installed**
   ```bash
   cd colcon-nano-ros && just install
   # Or: cargo install --path colcon-nano-ros/packages/cargo-nano-ros
   ```

## Workflow

**Step 1: Create package.xml**

Declare your ROS interface dependencies in `<depend>` tags:
```xml
<depend>std_msgs</depend>      <!-- For std_msgs::msg::Int32, String, etc. -->
<depend>example_interfaces</depend>  <!-- For service types -->
```

**Step 2: Generate bindings**

```bash
cd my_project
cargo nano-ros generate
```

This will:
1. Parse `package.xml` to find dependencies
2. Resolve transitive dependencies via ament index
3. Filter to interface packages (those with msg/srv/action)
4. Generate bindings to `generated/` directory
5. Create `.cargo/config.toml` with `[patch.crates-io]` entries

**Step 3: Add dependencies to Cargo.toml**

Reference the generated crates using crates.io version specifiers:
```toml
[dependencies]
std_msgs = { version = "*", default-features = false }
example_interfaces = { version = "*", default-features = false }
```

The `.cargo/config.toml` patches redirect these to local paths.

**Step 4: Use in code**

```rust
use std_msgs::msg::Int32;
use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};

let msg = Int32 { data: 42 };
```

## Command Options

```bash
cargo nano-ros generate [OPTIONS]

Options:
  -m, --manifest <PATH>     Path to package.xml [default: package.xml]
  -o, --output <DIR>        Output directory [default: generated]
  -n, --nano-ros <PATH>     Path to nano-ros crates (for config patches)
  -f, --force               Overwrite existing bindings
  -v, --verbose             Enable verbose output
      --no-config           Skip .cargo/config.toml generation
```

## Generated Output Structure

```
my_project/
├── package.xml              # Your dependency declarations
├── Cargo.toml               # Your package manifest
├── src/
│   └── main.rs              # Your code using generated types
├── generated/               # Generated bindings (do not edit)
│   ├── std_msgs/
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs       # #![no_std]
│   │       └── msg/
│   │           ├── mod.rs
│   │           └── int32.rs
│   └── builtin_interfaces/  # Transitive dependency
│       └── ...
└── .cargo/
    └── config.toml          # [patch.crates-io] entries
```

## Generated Code Features

**no_std by default:**
```rust
#![no_std]

pub mod msg;
```

**std feature for optional std support:**
```toml
[features]
default = []
std = ["nano-ros-core/std", "nano-ros-serdes/std"]
```

**heapless types for embedded:**
```rust
pub struct String {
    pub data: heapless::String<256>,
}

pub struct Arrays {
    pub data: heapless::Vec<i32, 64>,
}
```

**Service types with Request/Response:**
```rust
pub struct AddTwoInts;
pub struct AddTwoIntsRequest { pub a: i64, pub b: i64 }
pub struct AddTwoIntsResponse { pub sum: i64 }

impl RosService for AddTwoInts {
    type Request = AddTwoIntsRequest;
    type Reply = AddTwoIntsResponse;
}
```

## Standalone Package Mode

Examples are configured as standalone packages (excluded from workspace) because each has its own `.cargo/config.toml` patches. Build each example from its own directory:
```bash
cd examples/native-talker && cargo build
cd examples/native-service-client && cargo build
```

## Regenerating Bindings

To regenerate after ROS package updates or dependency changes:
```bash
cargo nano-ros generate --force
```

## Troubleshooting

**"Failed to load ament index"**
- Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`

**"Package 'X' not found in ament index"**
- Check package is installed: `ros2 pkg list | grep X`
- Install if missing: `sudo apt install ros-humble-X`

**Build errors with generated code**
- Regenerate with `--force` flag
- Check nano-ros crate compatibility
