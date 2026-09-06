# Phase 212.M.10 fixture — talker component pkg (PX4 host-compile shape).
#
# Per §212.L.9 the C++ pkg metadata surface is the cmake fn pair
# `nano_ros_node_register` + `nano_ros_deploy` — sidecar
# `component_nros.toml` is forbidden (§Non-Goals + §212.M.10).
#
# PX4 itself is uORB-only (Phase 115.K.4 collapsed uORB to C++) and
# the canonical PX4 surface lives at
# `packages/testing/nros-px4-register-check/`, which wires components
# into the firmware via PX4's own `px4_add_module()`. This fixture is
# the *host-side* compile-check shape — it lets the planner walk the
# pkg dir without dragging in the PX4 build system.

cmake_minimum_required(VERSION 3.22)
# §212.L.4 — CLASS prefix must equal PROJECT_NAME (pkg dir name).
project(talker_pkg VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Resolve the cmake fn registry. Phase 213.D.1 — the fixture lives
# under `packages/testing/nros-tests/fixtures/multi_pkg_workspace_px4/
# <pkg>/`, but a 6-level walk-up include violates CLAUDE.md's "no
# walk-up paths" policy. Test drivers + manual users pass
# `-DNANO_ROS_CMAKE_DIR=<repo>/cmake` (or export the env var) to point
# at the source-side cmake module dir.
if(NOT DEFINED NANO_ROS_CMAKE_DIR AND DEFINED ENV{NANO_ROS_CMAKE_DIR})
    set(NANO_ROS_CMAKE_DIR "$ENV{NANO_ROS_CMAKE_DIR}")
endif()
if(NOT NANO_ROS_CMAKE_DIR)
    message(FATAL_ERROR
        "NANO_ROS_CMAKE_DIR not set — pass -DNANO_ROS_CMAKE_DIR=<repo>/cmake "
        "or export it as an env var (Phase 213.D.1 walk-up retirement).")
endif()
include("${NANO_ROS_CMAKE_DIR}/NanoRosNodeRegister.cmake")

nano_ros_auto_add_library(talker_lib STATIC src/talker.cpp)

nros_components_register_node(talker_lib
    PLUGIN talker_pkg::talker
    EXECUTABLE talker
    SHAPE configure
    DEPLOY px4)

