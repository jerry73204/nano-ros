# Phase 212.M.10 fixture — brake_arbiter component pkg (PX4 host-compile shape).
#
# Per §212.L.9 the C++ pkg metadata surface is the cmake fn pair
# `nano_ros_node_register` (nano_ros_deploy retired post-287) — sidecar
# `component_nros.toml` is forbidden (§Non-Goals + §212.M.10).

cmake_minimum_required(VERSION 3.22)
# §212.L.4 — CLASS prefix must equal PROJECT_NAME (pkg dir name).
project(brake_arbiter_pkg VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Phase 213.D.1 — resolve cmake fn registry via env var, not walk-up.
# See sibling talker_pkg/CMakeLists.txt for the same shape.
if(NOT DEFINED NANO_ROS_CMAKE_DIR AND DEFINED ENV{NANO_ROS_CMAKE_DIR})
    set(NANO_ROS_CMAKE_DIR "$ENV{NANO_ROS_CMAKE_DIR}")
endif()
if(NOT NANO_ROS_CMAKE_DIR)
    message(FATAL_ERROR
        "NANO_ROS_CMAKE_DIR not set — pass -DNANO_ROS_CMAKE_DIR=<repo>/cmake "
        "or export it as an env var (Phase 213.D.1 walk-up retirement).")
endif()
include("${NANO_ROS_CMAKE_DIR}/NanoRosNodeRegister.cmake")

nano_ros_auto_add_library(brake_arbiter_lib STATIC src/brake_arbiter.cpp)

nros_components_register_node(brake_arbiter_lib
    PLUGIN brake_arbiter_pkg::brake_arbiter
    EXECUTABLE brake_arbiter
    SHAPE configure
    DEPLOY px4)

