# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Talker (C) — TYPED component (RFC-0043 / phase-244.C2).
#
# Instead of a hand-written imperative `main.c`, the node is a stateful C
# component (`NROS_C_COMPONENT(talker_t, talker_configure)`); the Zephyr typed
# carrier (the shared entry pack) generates the entry + composes it
# into `app`. The generated typed C bindings (std_msgs) provide the message
# struct + serializer (phase-277 W4; was hand-rolled CDR).

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nros_zephyr_talker_c)

# package.xml SSoT: generate the typed C bindings for the interface closure
# (std_msgs) into `app` — same call the zephyr C service/action examples use.

# Phase 11W.12 — Cyclone DDS needs a C `dds_topic_descriptor_t` for
# each message type (the backend's `find_descriptor` looks it up by
# mangled name). The nros C codegen emits the rcl-style C message
# struct but not Cyclone's descriptor, so generate it here from
# std_msgs/String and compile it into the app.
if(CONFIG_NROS_RMW_CYCLONEDDS)
    # The nano-ros Zephyr module exports NROS_CYCLONE_* and its cmake
    # helper dir; pull the helper onto the module path by exported name
    # (no repo-tree walk) so include() resolves it copy-out clean.
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
# the shared entry pack (`nros codegen entry-node`) + compiles src/Talker.c with
# -DNROS_PKG_NAME=<pkg sym> so its NROS_C_COMPONENT seam links against the
# generated entry, all into the global `app`. No second executable.
find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_auto_add_library(talker_lib STATIC src/Talker.c)

nros_components_register_node(talker_lib
    PLUGIN nros_zephyr_talker_c::Talker
    EXECUTABLE talker
    SHAPE configure
    TYPED)
