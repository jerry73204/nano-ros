# colcon-cargo-ros2

**Build Rust ROS 2 packages with automatic message binding generation.**

`colcon-cargo-ros2` is a colcon extension that enables seamless integration of Rust packages in ROS 2 workspaces. It automatically generates Rust bindings for ROS message types, manages dependencies, and installs packages in ament-compatible layout.

## Features

- **Automatic Binding Generation**: Generates Rust bindings for messages, services, and actions on-demand
- **Smart Caching**: SHA256-based checksums for fast incremental builds
- **Workspace-Level Bindings**: Bindings generated once and shared across all packages
- **Zero Configuration**: Just add dependencies to `Cargo.toml` - bindings are handled automatically
- **Ament Compatible**: Installs to standard ament locations for seamless ROS 2 integration

## Installation

### From PyPI (Recommended)

```bash
pip install colcon-cargo-ros2
```

### From Source

```bash
git clone https://github.com/jerry73204/colcon-cargo-ros2.git
cd colcon-cargo-ros2
pip install packages/colcon-cargo-ros2/
```

## Quick Start

### 1. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Create a Rust ROS 2 Package

```bash
cd src
cargo new --bin my_robot_node
cd my_robot_node
```

### 3. Add ROS Dependencies

**Cargo.toml**:
```toml
[package]
name = "my_robot_node"
version = "0.1.0"
edition = "2021"

[dependencies]
rclrs = "0.6"
std_msgs = "*"
geometry_msgs = "*"
```

**package.xml**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_node</name>
  <version>0.1.0</version>
  <description>Example Rust ROS 2 node</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cargo</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

**src/main.rs**:
```rust
use rclrs::CreateBasicExecutor;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize ROS context from environment
    let context = rclrs::Context::default_from_env()?;

    // Create executor (manages event loop)
    let executor = context.create_basic_executor();

    // Create node through executor
    let node = executor.create_node("minimal_publisher")?;

    // Create publisher for std_msgs/String on the "chatter" topic
    let publisher = node.create_publisher::<std_msgs::msg::String>("chatter")?;

    // Publish a few messages
    for i in 0..5 {
        let mut msg = std_msgs::msg::String::default();
        msg.data = format!("Hello from Rust! Message #{}", i);

        println!("Publishing: '{}'", msg.data);
        publisher.publish(msg)?;

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    println!("✓ Published 5 messages successfully!");
    Ok(())
}
```

### 4. Build with colcon

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash  # Or your ROS 2 distro
colcon build --symlink-install
```

The extension will:
1. Discover ROS dependencies from `Cargo.toml` and `package.xml`
2. Generate Rust bindings for `std_msgs` and `geometry_msgs`
3. Build your Rust package with cargo
4. Install binaries to `install/my_robot_node/lib/my_robot_node/`

### 5. Run Your Program

```bash
source install/setup.bash
ros2 run my_robot_node my_robot_node
```

Expected output:
```
Publishing: 'Hello from Rust! Message #0'
Publishing: 'Hello from Rust! Message #1'
Publishing: 'Hello from Rust! Message #2'
Publishing: 'Hello from Rust! Message #3'
Publishing: 'Hello from Rust! Message #4'
✓ Published 5 messages successfully!
```

## Package Structure

For `colcon-cargo-ros2` to recognize your package:
- **Both files required**: `package.xml` AND `Cargo.toml` in the package root
- **Build type**: `package.xml` must specify `<build_type>ament_cargo</build_type>` in the `<export>` section
- **Dependencies**: List ROS dependencies in both `Cargo.toml` and `package.xml`

Verify packages are detected:
```bash
$ colcon list
my_robot_node   src/my_robot_node   (ament_cargo)
```

### nano-ros-owned packages

A package only nano-ros can build — an entry, a board, an RMW / platform
provider, a bringup — declares its own build type instead of borrowing ament's
(RFC-0087 D2):

| `<build_type>` | Build path |
| --- | --- |
| `nros_cargo` | cargo |
| `nros_cmake` | CMake (or `west`, when the package deploys to Zephyr) |

That is the whole vocabulary: there is **no** key per language or per platform.
Which platform a package targets is read from its own consumption tag, and an
absent one means the host:

```xml
<export>
  <build_type>nros_cmake</build_type>
  <nano_ros deploy="threadx" board="riscv64-qemu" rmw="zenoh"/>
</export>
```

Without this extension installed, a colcon build of such a package refuses it
by name (`No task extension to 'build' a 'ros.nros_cargo' package`) rather than
attempting to install firmware into a prefix — which is the point of the
spelling.

## Building

### Basic Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_node

# Build with release optimizations
colcon build --cargo-args --release

# Verbose output
colcon build --event-handlers console_direct+
```

### Using Custom Interfaces

Custom interface packages follow the standard ROS 2 procedure (CMake-based with `rosidl_generate_interfaces`). Simply add them as dependencies in your Rust package's `Cargo.toml`:

```toml
[dependencies]
my_custom_interfaces = "*"
```

Bindings will be generated automatically during the build.

## How It Works

### Workspace-Level Binding Generation

When building a colcon workspace, `colcon-cargo-ros2`:

1. **Discovers Packages**: Finds all ROS dependencies via ament index
2. **Generates Bindings**: Creates Rust bindings in `build/<pkg>/rosidl_cargo/` for each interface package
3. **Creates Config File**: Writes `build/ros2_cargo_config.toml` with relative paths to all bindings
4. **Builds**: Runs `cargo build --config build/ros2_cargo_config.toml` from workspace root
5. **Installs**: Copies binaries and creates ament markers

**Workspace Structure**:
```
ros2_ws/
├── build/
│   ├── ros2_cargo_config.toml  # Workspace config with relative paths
│   ├── std_msgs/
│   │   └── rosidl_cargo/       # Rust bindings for std_msgs
│   │       └── std_msgs/
│   ├── geometry_msgs/
│   │   └── rosidl_cargo/       # Rust bindings for geometry_msgs
│   │       └── geometry_msgs/
│   └── my_interfaces/
│       └── rosidl_cargo/       # Rust bindings for custom interfaces
│           └── my_interfaces/
├── install/
│   ├── my_robot_node/
│   │   ├── lib/my_robot_node/  # Binaries
│   │   └── share/              # Metadata
│   └── my_interfaces/
└── src/
    ├── my_robot_node/
    │   ├── Cargo.toml
    │   └── package.xml
    └── my_interfaces/
```

### Benefits

- **Per-Package Organization**: Bindings follow ROS conventions (like `rosidl_cmake/`)
- **Fast Builds**: Intelligent caching skips regeneration when possible
- **Clean Workspace**: `colcon clean` removes all generated code
- **Portable**: Config file uses relative paths, making workspaces fully portable

## Advanced Features

### Installing Additional Files with `[package.metadata.ros]`

ROS 2 packages often need to install additional files beyond binaries (launch files, config files, URDF models, RViz configs, meshes, etc.). Use the `[package.metadata.ros]` section in `Cargo.toml` to specify these files:

```toml
[package]
name = "my_robot"
version = "0.1.0"

[dependencies]
rclrs = "0.6"
std_msgs = "*"

[package.metadata.ros]
install_to_share = ["launch", "config", "urdf", "README.md"]
install_to_include = ["include"]
install_to_lib = ["scripts"]
```

#### Supported Keys

- **`install_to_share`**: Files/directories installed to `install/<pkg>/share/<pkg>/`
  - Launch files, config files, URDF models, RViz configs, meshes, documentation
- **`install_to_include`**: Headers installed to `install/<pkg>/include/<pkg>/`
  - C/C++ headers for FFI libraries
- **`install_to_lib`**: Scripts/utilities installed to `install/<pkg>/lib/<pkg>/`
  - Helper scripts and executables

#### Example: Robot Description Package

```toml
[package]
name = "my_robot_description"
version = "0.1.0"

[lib]
# Library-only package (no binaries)

[package.metadata.ros]
install_to_share = ["urdf", "meshes", "launch", "rviz"]
```

**Project structure**:
```
my_robot_description/
├── Cargo.toml
├── package.xml
├── src/lib.rs
├── urdf/
│   ├── robot.urdf.xacro
│   └── robot.urdf
├── meshes/
│   ├── base.stl
│   └── arm.dae
├── launch/
│   └── display.launch.xml
└── rviz/
    └── default.rviz
```

**After `colcon build`**:
```
install/my_robot_description/
└── share/my_robot_description/
    ├── urdf/              # Directory with all URDF files
    ├── meshes/            # Directory with all mesh files
    ├── launch/            # Directory with launch files
    ├── rviz/              # Directory with RViz configs
    ├── rust/              # Source code (automatic)
    └── package.xml        # Metadata (automatic)
```

## Troubleshooting

### "Package not found in ament index"

Make sure the ROS 2 environment is sourced:
```bash
source /opt/ros/jazzy/setup.bash
```

### "error: failed to select a version"

This usually means bindings weren't generated. Try:
```bash
# Clean and rebuild
rm -rf build install
colcon build
```

### Build fails with linking errors

Ensure all dependencies are listed in both `Cargo.toml` and `package.xml`:
```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

## Requirements

- **Python**: 3.8 or later
- **ROS 2**: Humble, Iron, or Jazzy
- **Rust**: 1.70 or later (stable toolchain)
- **colcon**: Latest version

## License

Apache-2.0 (compatible with ROS 2 ecosystem)

## Contributing

See CONTRIBUTING.md for development setup, architecture details, and guidelines.

## Related Projects

- [ros2\_rust](https://github.com/ros2-rust/ros2_rust) - Official Rust bindings for ROS 2
- [r2r](https://github.com/sequenceplanner/r2r) - Alternative Rust bindings
- [colcon](https://colcon.readthedocs.io) - Build tool for ROS 2
