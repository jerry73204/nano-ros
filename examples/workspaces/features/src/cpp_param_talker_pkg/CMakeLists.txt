cmake_minimum_required(VERSION 3.22)
project(cpp_param_talker_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Phase 269 W1 — typed C++ component that reads publish_period_ms live each tick
# via nros_cpp_get_param_integer on the saved executor handle.
nano_ros_auto_add_library(cpp_param_talker_lib STATIC src/ParamTalker.cpp)

nros_components_register_node(cpp_param_talker_lib
    PLUGIN cpp_param_talker_pkg::ParamTalker
    EXECUTABLE cpp_param_talker
    SHAPE configure)

if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_param_talker_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
