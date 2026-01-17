# SPDX-License-Identifier: Apache-2.0
# nano-ros Zephyr Talker Example
#
# This example demonstrates a ROS 2 compatible publisher using
# nano-ros with zenoh-pico transport on Zephyr RTOS.
#
# Build:
#   west build -b qemu_cortex_m3 examples/zephyr-talker
#
# Run:
#   west build -t run

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nano_ros_talker)

# Add zenoh-pico as external project
# Note: zenoh-pico must be compiled for Zephyr separately
# See: https://github.com/eclipse-zenoh/zenoh-pico#zephyr

# Source files
target_sources(app PRIVATE
    src/main.c
)

# Include paths for nano-ros (if using C interop)
# For pure Rust, these would be handled by cargo
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/zenoh-pico-sys/zenoh-pico/include
)

# Link with zenoh-pico library
# This assumes zenoh-pico was built as a Zephyr module
# target_link_libraries(app PRIVATE zenoh_pico)
