# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Service Server Example (C++) — TYPED component (RFC-0043 / phase-244.C2).
#
# Instead of a hand-written imperative `main.cpp`, the node is a stateful
# component (`AddTwoIntsServer::configure(node)`); the Zephyr typed carrier
# generates the entry (`nano_ros_add_node(... TYPED ...)` →
# ZephyrBoard::run_components).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_cpp_service_server LANGUAGES CXX)

# nros module (CONFIG_NROS=y, CONFIG_NROS_CPP_API=y) provides zenoh shim,
# platform support, and the nros-cpp library automatically.

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)


# Phase 11W.12 — Cyclone DDS needs C `dds_topic_descriptor_t`s for each
# wire type. A service maps to a request + response topic, so generate
# both descriptors from example_interfaces/srv/AddTwoInts.srv (the
# converter emits the _Request_ / _Response_ structs) and compile them
# into the app.
if(CONFIG_NROS_RMW_CYCLONEDDS)
    list(APPEND CMAKE_MODULE_PATH "${NROS_CYCLONE_CMAKE_DIR}")
    set(IDLC_EXECUTABLE "${NROS_CYCLONE_IDLC}" CACHE FILEPATH "" FORCE)
    set(ENV{NROS_RMW_CYCLONEDDS_SCRIPTS_DIR} "${NROS_CYCLONE_SCRIPTS_DIR}")
    include(NrosRmwCycloneddsTypeSupport)
    nros_rmw_cyclonedds_generate_from_msg(_nros_cyc_types
        PKG_NAME   example_interfaces
        PKG_DIR    "$ENV{NROS_EXAMPLE_INTERFACES_DIR}"
        INTERFACES srv/AddTwoInts.srv)
    target_sources(app PRIVATE ${_nros_cyc_types})
endif()

# The Zephyr typed carrier branch keys on NANO_ROS_PLATFORM=zephyr + DEPLOY
# zephyr + TARGET app + TYPED, and target_sources/target_link_libraries the
# generated entry + component lib into the global `app`. No second executable.

nano_ros_auto_add_library(add_two_ints_server_lib STATIC src/AddTwoIntsServer.cpp)

nros_components_register_node(add_two_ints_server_lib
    HEADER AddTwoIntsServer.hpp
    PLUGIN zephyr_cpp_service_server::AddTwoIntsServer
    EXECUTABLE add_two_ints_server
    SHAPE configure
    TYPED
    # phase-412 -- `ENTITIES` is retired. A standalone Zephyr application has
    # no launch file, so it has no contract sidecar to state itself in; what
    # this image creates is stated as the XRCE session caps in prj-xrce.conf,
    # which is what phase-412's own guidance leaves such an image ("a system
    # with no contract yet keeps its configured" knobs). See issue 1033.
)
