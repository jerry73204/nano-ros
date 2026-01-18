# SPDX-License-Identifier: Apache-2.0
# nano-ros Zephyr Talker Example (Rust)
#
# A ROS 2 compatible publisher running on Zephyr RTOS using nano-ros.
#
# Build:
#   source ~/nano-ros-workspace/env.sh
#   west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs
#
# Run:
#   ./build/zephyr/zephyr.exe

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nano_ros_talker_rs)

# C shim for zenoh-pico FFI
target_sources(app PRIVATE src/zenoh_shim.c)

# Build Rust application (zephyr-lang-rust is in west manifest)
rust_cargo_application()
