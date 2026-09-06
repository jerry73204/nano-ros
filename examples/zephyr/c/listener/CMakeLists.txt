# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Listener Example (C) — TYPED component (RFC-0043 / phase-244.C2).
#
# Instead of a hand-written imperative `main.c`, the node is a stateful C
# component (`NROS_C_COMPONENT(listener_t, listener_configure)`); the Zephyr typed
# carrier (the shared entry pack, a C++ entry that drives the C
# component's `__nros_c_component_<pkg>_{create,configure}` seam) generates the
# entry + composes it into the Zephyr `app`. The generated typed C bindings
# (std_msgs) provide the message struct + deserializer (phase-277 W4; was
# hand-rolled CDR).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nros_zephyr_listener_c)

# nros module (CONFIG_NROS=y, CONFIG_NROS_C_API=y) provides zenoh shim,
# platform support, and the nros-c library automatically.

# package.xml SSoT: generate the typed C bindings for the interface closure
# (std_msgs) into `app` — same call the zephyr C service/action examples use.

# Phase 11W.12 — Cyclone DDS needs a C `dds_topic_descriptor_t` for
# each message type (the backend's `find_descriptor` looks it up by
# mangled name). The nros C codegen emits the rcl-style C message
# struct but not Cyclone's descriptor, so generate it here from
# std_msgs/String and compile it into the app.
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

# The Zephyr typed carrier branch (LANGUAGE C) configure_files
# the shared entry pack (`nros codegen entry-node`) + compiles src/Listener.c with
# -DNROS_PKG_NAME=<pkg sym> so its NROS_C_COMPONENT factory/configure seam links
# against the generated entry, all into the global `app`. No second executable.
find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_node(listener CLASS nros_zephyr_listener_c::Listener TYPED src/Listener.c)
