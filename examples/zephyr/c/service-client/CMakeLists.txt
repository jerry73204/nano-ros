# SPDX-License-Identifier: Apache-2.0
# nros Zephyr Service Client Example (C) — TYPED component (RFC-0043 / phase-244.C2).
#
# Instead of a hand-written imperative `main.c`, the node is a stateful C
# component (`NROS_C_COMPONENT(add_client_t, client_configure)`); the Zephyr typed
# carrier (the shared entry pack, a C++ entry that drives the C
# component's `__nros_c_component_<pkg>_{create,configure}` seam) generates the
# entry + composes it into the Zephyr `app`.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nros_zephyr_zenoh_service_client_c)

# package.xml SSoT (issue #11): resolve + generate the transitive interface
# closure declared in package.xml (<depend>example_interfaces</depend>) into `app`.

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

# The Zephyr typed carrier branch (LANGUAGE C) configure_files
# the shared entry pack (`nros codegen entry-node`) + compiles src/AddTwoIntsClient.c with
# -DNROS_PKG_NAME=<pkg sym> so its NROS_C_COMPONENT factory/configure seam links
# against the generated entry, all into the global `app`. No second executable.
find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

nano_ros_auto_add_library(add_two_ints_client_lib STATIC src/AddTwoIntsClient.c)

nros_components_register_node(add_two_ints_client_lib
    PLUGIN nros_zephyr_zenoh_service_client_c::AddTwoIntsClient
    EXECUTABLE add_two_ints_client
    SHAPE configure
    TYPED)
