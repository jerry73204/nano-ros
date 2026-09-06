# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Talker Example (C++) — TYPED component (RFC-0043 / phase-244.C2).
#
# Instead of a hand-written imperative `main.cpp`, the node is a stateful
# component (`Talker::configure(node)`); the Zephyr typed carrier generates the
# entry (`nano_ros_add_node(... TYPED ...)` → ZephyrBoard::run_components).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_cpp_talker LANGUAGES CXX)

# nros zephyr module (CONFIG_NROS=y, CONFIG_NROS_CPP_API=y) auto-loaded
# via `find_package(Zephyr)`; provides zenoh shim, platform support, and
# the nros-cpp library (zephyr_library_named(nros)).
#
# Phase 210.E.3.c — switch from legacy `nros_generate_interfaces(<pkg>)`
# to the canonical `find_package(<pkg>)` shape via the rclcpp source-
# compat module. `NROS_REPO_DIR` is published as a CACHE var by the
# zephyr module so the include path resolves from the app dir.
find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Phase 11W.12 — Cyclone DDS needs a C `dds_topic_descriptor_t` for
# each message type (the backend's `find_descriptor` looks it up by
# mangled name). The C++ codegen only emits C++ types, so generate the
# Cyclone C descriptor here from std_msgs/String and compile it into the
# app. Uses the host-built idlc + the msg→IDL converter.
if(CONFIG_NROS_RMW_CYCLONEDDS)
    list(APPEND CMAKE_MODULE_PATH "${NROS_CYCLONE_CMAKE_DIR}")
    set(IDLC_EXECUTABLE "${NROS_CYCLONE_IDLC}" CACHE FILEPATH "" FORCE)
    set(ENV{NROS_RMW_CYCLONEDDS_SCRIPTS_DIR} "${NROS_CYCLONE_SCRIPTS_DIR}")
    include(NrosRmwCycloneddsTypeSupport)
    nros_rmw_cyclonedds_generate_from_msg(_nros_cyc_types
        PKG_NAME   std_msgs
        PKG_DIR    "$ENV{NROS_STD_MSGS_DIR}"
        INTERFACES msg/String.msg)
    target_sources(app PRIVATE ${_nros_cyc_types})
endif()

# The Zephyr typed carrier branch keys on NANO_ROS_PLATFORM=zephyr + DEPLOY
# zephyr + TARGET app + TYPED, and target_sources/target_link_libraries the
# generated entry + component lib into the global `app`. No second executable.

nano_ros_auto_add_library(talker_lib STATIC src/Talker.cpp)

nros_components_register_node(talker_lib
    HEADER Talker.hpp
    PLUGIN zephyr_cpp_talker::Talker
    EXECUTABLE talker
    SHAPE configure
    TYPED
    # phase-412 -- `ENTITIES` is retired. A standalone Zephyr application has
    # no launch file, so it has no contract sidecar to state itself in; what
    # this image creates is stated as the XRCE session caps in prj-xrce.conf,
    # which is what phase-412's own guidance leaves such an image ("a system
    # with no contract yet keeps its configured" knobs). See issue 1033.
)
