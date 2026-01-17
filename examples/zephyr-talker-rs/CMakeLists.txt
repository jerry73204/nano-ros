# SPDX-License-Identifier: Apache-2.0
# nano-ros Zephyr Talker Example (Rust)
#
# This example demonstrates a ROS 2 compatible publisher using
# nano-ros with zenoh-pico transport on Zephyr RTOS.
#
# Build:
#   west build -b qemu_x86 examples/zephyr-talker-rs
#   west build -b qemu_cortex_m3 examples/zephyr-talker-rs
#   west build -b nucleo_f429zi examples/zephyr-talker-rs
#
# Run:
#   west build -t run

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nano_ros_talker_rs LANGUAGES C)

# Enable Rust support
# The actual Rust code is built by Cargo, invoked by the Zephyr build system
# when CONFIG_RUST=y is set in prj.conf

# zenoh-pico as Zephyr module
# This is linked automatically when zenoh-pico is in the Zephyr modules path
if(CONFIG_ZENOH_PICO)
    target_link_libraries(app PRIVATE zenoh_pico)
endif()

# Stub main.c that calls Rust entry point
# (Required for Zephyr's CMake build, but main logic is in Rust)
target_sources(app PRIVATE
    src/main_stub.c
)
