//! # Getting Started
//!
//! ## Prerequisites
//!
//! - [Rust](https://rustup.rs/) nightly toolchain (edition 2024)
//! - zenohd 1.6.2 router (build from submodule with `just build-zenohd`,
//!   or install from [zenoh releases](https://github.com/eclipse-zenoh/zenoh/releases))
//! - ROS 2 Humble (optional — only needed for message packages beyond
//!   `std_msgs` and `builtin_interfaces`, which are bundled)
//!
//! ## 1. Create a project
//!
//! ```bash
//! cargo new my-talker && cd my-talker
//! ```
//!
//! Add nros and your message crate to `Cargo.toml`:
//!
//! ```toml
//! [package]
//! name = "my-talker"
//! version = "0.1.0"
//! edition = "2024"
//!
//! [dependencies]
//! nros = { git = "https://github.com/jerry73204/nano-ros", default-features = false, features = ["std"] }
//! std_msgs = { version = "*", default-features = false }
//! ```
//!
//! ## 2. Declare message dependencies
//!
//! Create a `package.xml` in the project root:
//!
//! ```xml
//! <?xml version="1.0"?>
//! <package format="3">
//!   <name>my_talker</name>
//!   <version>0.1.0</version>
//!   <description>My first nros talker</description>
//!   <maintainer email="you@example.com">You</maintainer>
//!   <license>MIT</license>
//!   <depend>std_msgs</depend>
//!   <export><build_type>ament_cargo</build_type></export>
//! </package>
//! ```
//!
//! ## 3. Generate message bindings
//!
//! Build the `nros` tool (one-time, from a nano-ros checkout — in a
//! checkout the tree's own build is the binary it accepts, RFC-0090;
//! to just USE nano-ros, install a release with `scripts/install.sh`):
//!
//! ```bash
//! ./scripts/bootstrap.sh   # builds packages/cli/target/release/nros
//! source ./activate.sh     # puts it on PATH
//! ```
//!
//! Generate bindings (for packages beyond `std_msgs`/`builtin_interfaces`,
//! source a ROS 2 environment first):
//!
//! ```bash
//! nros generate-rust --config --nano-ros-git
//! ```
//!
//! This creates:
//! - `generated/std_msgs/` — Rust types (`Int32`, `String`, etc.)
//! - `generated/builtin_interfaces/` — `Time`, `Duration`
//! - `.cargo/config.toml` — `[patch.crates-io]` entries
//!
//! Key options: `--force` (overwrite existing), `--nano-ros-path <PATH>`
//! (local dev instead of git), `-o <DIR>` (output directory, default
//! `generated`).
//!
//! ## 4. Build and run
//!
//! ```bash
//! # Terminal 1: start zenoh router
//! ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
//!
//! # Terminal 2: run your node
//! RUST_LOG=info cargo run --features zenoh
//! ```
//!
//! To verify with a ROS 2 listener:
//!
//! ```bash
//! source /opt/ros/humble/setup.bash
//! export RMW_IMPLEMENTATION=rmw_zenoh_cpp
//! ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
//! ```
