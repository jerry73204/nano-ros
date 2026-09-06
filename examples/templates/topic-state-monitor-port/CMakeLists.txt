# topic_state_monitor synthetic port — Phase 209.G first iteration.
cmake_minimum_required(VERSION 3.22)
project(topic_state_monitor LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(NANO_ROS_PLATFORM posix)
set(NROS_RMW "zenoh" CACHE STRING "Active RMW (zenoh|cyclonedds|xrce).")
set(NANO_ROS_RMW "${NROS_RMW}")
# Resolve the nano-ros checkout root once (phase-277 W6 standard guard):
#   1. -DNANO_ROS_ROOT=<path> cache var (explicit, e.g. a copied-out example),
#   2. NROS_REPO_DIR env var (exported by nano-ros's activate.sh),
#   3. in-repo relative walk-up (this example inside the nano-ros tree).
if(NOT DEFINED NANO_ROS_ROOT)
    if(DEFINED ENV{NROS_REPO_DIR} AND NOT "$ENV{NROS_REPO_DIR}" STREQUAL "")
        set(NANO_ROS_ROOT "$ENV{NROS_REPO_DIR}")
    else()
        get_filename_component(NANO_ROS_ROOT
            "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
    endif()
endif()
add_subdirectory("${NANO_ROS_ROOT}" nano_ros)

include("${NANO_ROS_ROOT}/cmake/compat/NrosRclcppCompat.cmake")

# package.xml is the single source of truth for the msg deps;
# nros_find_interfaces() resolves the transitive closure from its <depend> entries.
nros_find_interfaces(LANGUAGE CPP SKIP_INSTALL)

# Stock ament_cmake_auto-shape below — unchanged from what an Autoware
# CMakeLists would carry.
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(topic_state_monitor src/topic_state_monitor.cpp)
ament_target_dependencies(topic_state_monitor rclcpp diagnostic_updater)
target_link_libraries(topic_state_monitor PRIVATE
    std_msgs__nano_ros_cpp
    diagnostic_msgs__nano_ros_cpp)

ament_auto_package()
