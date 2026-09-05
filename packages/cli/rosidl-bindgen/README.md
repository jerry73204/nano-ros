# rosidl-bindgen

Rust binding generator for ROS 2 interface packages (.msg, .srv, .action files).

## Overview

`rosidl-bindgen` generates idiomatic Rust bindings for ROS 2 interfaces, creating type-safe wrappers around ROS C libraries. Generated bindings can be used standalone or integrated with `rclrs` for building ROS 2 nodes.

## Supported Runtime Library Versions

Generated bindings depend on the following crates from crates.io:

| Crate | Version | Purpose |
|-------|---------|---------|
| `rosidl_runtime_rs` | `0.5` | Core runtime types (Message, Sequence, String) and traits |
| `rclrs` | `0.6` | ROS 2 client library for Rust (for building nodes) |

### Version Compatibility

Generated bindings are compatible with **rosidl_runtime_rs 0.5.x** API:
- `Sequence::new()` returns `Self` (panics on failure)
- No `capacity()` method or `SequenceError` type
- Standard trait bounds on `Message` and `RmwMessage`

## Usage

### As a Library

```rust
use rosidl_bindgen::{ament::Package, generator::generate_package};

// Discover ROS package
let package = Package::from_share_directory("std_msgs")?;

// Generate bindings
let output_dir = PathBuf::from("build/ros2_bindings");
let result = generate_package(&package, &output_dir)?;

println!("Generated {} messages, {} services, {} actions",
    result.message_count, result.service_count, result.action_count);
```

### With cargo-ros2

The `cargo-ros2` tool uses `rosidl-bindgen` internally to generate bindings automatically:

```bash
cargo ros2 build
```

This discovers ROS dependencies from `Cargo.toml` and generates bindings to `build/ros2_bindings/`.

## Generated Package Structure

For each ROS interface package (e.g., `std_msgs`), the generator creates:

```
build/ros2_bindings/std_msgs/
├── Cargo.toml              # Dependencies: rosidl_runtime_rs = "0.5"
├── build.rs                # Links against ROS C libraries
├── src/
│   └── lib.rs             # Public API exports
│       ├── ffi/           # C-compatible FFI types
│       │   ├── msg/       # Message FFI structs
│       │   ├── srv/       # Service FFI structs
│       │   └── action/    # Action FFI structs
│       ├── msg/           # Idiomatic message types
│       ├── srv/           # Idiomatic service types
│       └── action/        # Idiomatic action types
```

### Dependencies in Generated Cargo.toml

```toml
[dependencies]
rosidl_runtime_rs = "0.5"
serde = { version = "1.0", features = ["derive"], optional = true }

# Cross-package dependencies (path-based, relative to build/ros2_bindings/)
geometry_msgs = { path = "../geometry_msgs" }  # if referenced
```

**Note**: Generated packages use **path dependencies** for other ROS message packages but **crates.io** for the runtime library.

## User Requirements

### For Building Generated Bindings

Users must have `rosidl_runtime_rs` available from crates.io. This happens automatically when:
1. Generated `Cargo.toml` specifies the dependency
2. `cargo build` downloads it from crates.io

No manual setup required!

### For Building ROS 2 Nodes

To use generated bindings in a ROS node, add to your `Cargo.toml`:

```toml
[dependencies]
rclrs = "0.6"
rosidl_runtime_rs = "0.5"

# Generated bindings (path to build/ros2_bindings/)
std_msgs = { path = "../../build/ros2_bindings/std_msgs" }
```

## Migration from Embedded Libraries

**Previous behavior** (before 2025-01-13):
- `rosidl-bindgen` embedded `rosidl_runtime_rs` and `rclrs` source code
- Generated bindings extracted these to `build/ros2_bindings/rosidl_runtime_rs/`
- Self-contained but larger and slower

**Current behavior**:
- Generated bindings reference `rosidl_runtime_rs = "0.5"` from crates.io
- No embedded source code - smaller and faster generation
- Standard Rust dependency management

## Version Updates

To update supported versions, modify constants in `src/generator.rs`:

```rust
pub const ROSIDL_RUNTIME_RS_VERSION: &str = "0.5";
pub const RCLRS_VERSION: &str = "0.6";
```

Then rebuild `rosidl-bindgen` and regenerate bindings.

## See Also

- [rosidl-codegen](../rosidl-codegen/) - Template-based code generation engine
- [rosidl-parser](../rosidl-parser/) - IDL parser for .msg/.srv/.action files
- cargo-ros2 - Build orchestrator that uses rosidl-bindgen
