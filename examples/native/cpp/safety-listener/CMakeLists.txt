cmake_minimum_required(VERSION 3.22)
project(cpp_safety_listener LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Phase 261 W5 — declare the E2E-safety capability via the language-neutral
# `NANO_ROS_FEATURES` surface (the C/C++ projection of `system.toml`
# `[system].features = ["safety"]`). The root `nros_lower_system_features` lowers it
# to `NANO_ROS_SAFETY_E2E=ON` before nros-cpp reads the option, building the zenoh
# backend with the CRC + validated-recv API (`Subscription::take_validated`).
# Set BEFORE find_package(nano_ros) so the axis reaches the import.
set(NANO_ROS_FEATURES "safety")

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable(cpp_safety_listener src/main.cpp)
ament_target_dependencies(cpp_safety_listener std_msgs)

install(TARGETS cpp_safety_listener DESTINATION lib/${PROJECT_NAME})
ament_package()
