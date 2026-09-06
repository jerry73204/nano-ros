cmake_minimum_required(VERSION 3.22)
project(robot_entry VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT COMMAND nano_ros_workspace_pkg_guard)
    # Phase-277 W6 standard guard, workspace-pkg form: the workspace root
    # normally provides NANO_ROS_ROOT; solo builds may pass -DNANO_ROS_ROOT
    # or export NROS_REPO_DIR (nano-ros's activate.sh does).
    if(NOT NANO_ROS_ROOT AND DEFINED ENV{NROS_REPO_DIR} AND NOT "$ENV{NROS_REPO_DIR}" STREQUAL "")
        set(NANO_ROS_ROOT "$ENV{NROS_REPO_DIR}")
    endif()
    if(NOT NANO_ROS_ROOT)
        message(FATAL_ERROR
            "nano-ros: set -DNANO_ROS_ROOT=<path-to-nano-ros> or export "
            "NROS_REPO_DIR for standalone builds, or build via the workspace root.")
    endif()
    include("${NANO_ROS_ROOT}/cmake/NanoRosWorkspace.cmake")
endif()
nano_ros_workspace_pkg_guard()

nano_ros_entry(
    NAME    robot_entry
    SOURCES src/main.cpp
    BOARD   native
    BRINGUP "${CMAKE_CURRENT_SOURCE_DIR}/../demo_bringup"
    TYPED
    PANIC   platform
    DEPLOY  native)
