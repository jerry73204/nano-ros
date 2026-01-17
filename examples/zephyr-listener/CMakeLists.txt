# SPDX-License-Identifier: Apache-2.0
# nano-ros Zephyr Listener Example
#
# This example demonstrates a ROS 2 compatible subscriber using
# nano-ros with zenoh-pico transport on Zephyr RTOS.
#
# Build:
#   west build -b qemu_cortex_m3 examples/zephyr-listener
#
# Run:
#   west build -t run

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nano_ros_listener)

# Source files
target_sources(app PRIVATE
    src/main.c
)

# Include paths for zenoh-pico
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../crates/zenoh-pico-sys/zenoh-pico/include
)
