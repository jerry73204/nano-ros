# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Action Client Example (C, Zenoh)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nros_zephyr_zenoh_action_client_c)

# package.xml SSoT (issue #11): resolve + generate the transitive interface
# closure declared in package.xml (<depend>example_interfaces</depend>) into
# `app` — resolve-deps pulls action_msgs / unique_identifier_msgs too.

if(CONFIG_NROS_RMW_CYCLONEDDS)
    list(APPEND CMAKE_MODULE_PATH "${NROS_CYCLONE_CMAKE_DIR}")
    include(NrosZephyrCycloneddsActionTypes)
    nros_zephyr_add_cyclonedds_action_descriptors(app)
endif()

# TYPED component (RFC-0043 / phase-244.C2). The Zephyr typed carrier branch
# (LANGUAGE C) generates the entry TU through the shared pack (a C++ entry
# that drives the C component's NROS_C_COMPONENT factory/configure seam) +
# compiles src/FibonacciClient.c into the global `app`. No hand-written main.c.
find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

nano_ros_auto_add_library(fibonacci_action_client_lib STATIC src/FibonacciClient.c)

nros_components_register_node(fibonacci_action_client_lib
    PLUGIN nros_zephyr_zenoh_action_client_c::FibonacciClient
    EXECUTABLE fibonacci_action_client
    SHAPE configure
    TYPED)
