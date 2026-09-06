# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Action Client Example (C++) — TYPED component (RFC-0043 / phase-244.C2).
#
# Instead of a hand-written imperative `main.cpp`, the node is a stateful
# component (`FibonacciClient::configure(node)`); the Zephyr typed carrier
# generates the entry (`nano_ros_add_node(... TYPED ...)` →
# ZephyrBoard::run_components).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_cpp_action_client LANGUAGES CXX C)

# nros module (CONFIG_NROS=y, CONFIG_NROS_CPP_API=y) provides zenoh shim,
# platform support, and the nros-cpp library automatically.

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

if(CONFIG_NROS_RMW_CYCLONEDDS)
    list(APPEND CMAKE_MODULE_PATH "${NROS_CYCLONE_CMAKE_DIR}")
    include(NrosZephyrCycloneddsActionTypes)
    nros_zephyr_add_cyclonedds_action_descriptors(app)
endif()

# The Zephyr typed carrier branch keys on NANO_ROS_PLATFORM=zephyr + DEPLOY
# zephyr + TARGET app + TYPED, and target_sources/target_link_libraries the
# generated entry + component lib into the global `app`. No second executable.

nano_ros_auto_add_library(fibonacci_action_client_lib STATIC src/FibonacciClient.cpp)

nros_components_register_node(fibonacci_action_client_lib
    HEADER FibonacciClient.hpp
    PLUGIN zephyr_cpp_action_client::FibonacciClient
    EXECUTABLE fibonacci_action_client
    SHAPE configure
    TYPED
    # phase-412 -- `ENTITIES` is retired. A standalone Zephyr application has
    # no launch file, so it has no contract sidecar to state itself in; what
    # this image creates is stated as the XRCE session caps in prj-xrce.conf,
    # which is what phase-412's own guidance leaves such an image ("a system
    # with no contract yet keeps its configured" knobs). See issue 1033.
)
