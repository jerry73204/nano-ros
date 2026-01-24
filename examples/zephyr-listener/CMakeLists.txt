# SPDX-License-Identifier: Apache-2.0
# nano-ros Zephyr Listener Example (Rust)
#
# A ROS 2 compatible subscriber running on Zephyr RTOS using nano-ros.
#
# Build:
#   source ~/nano-ros-workspace/env.sh
#   west build -b native_sim/native/64 nano-ros/examples/zephyr-listener
#
# Run:
#   ./build/zephyr/zephyr.exe

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nano_ros_listener_rs)

# Use shared C shim from zenoh-pico-shim-sys
# This provides the zenoh_shim_* API used by the Rust crate
set(SHIM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/zenoh-pico-shim-sys/c)
target_sources(app PRIVATE ${SHIM_DIR}/shim/zenoh_shim.c)
target_include_directories(app PRIVATE ${SHIM_DIR}/include)

# Build Rust application (zephyr-lang-rust is in west manifest)
rust_cargo_application()
