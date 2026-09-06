cmake_minimum_required(VERSION 3.22)
project(cpp_safety_talker_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Phase 269 W3 — C++ talker component: publishes a counter on /chatter.
# With NANO_ROS_SAFETY_E2E=ON the zenoh backend automatically attaches a CRC on publish.
nano_ros_auto_add_library(cpp_talker_lib STATIC src/SafetyTalker.cpp)

nros_components_register_node(cpp_talker_lib
    PLUGIN cpp_safety_talker_pkg::SafetyTalker
    EXECUTABLE cpp_talker
    SHAPE configure)

if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_talker_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
