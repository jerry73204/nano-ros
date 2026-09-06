cmake_minimum_required(VERSION 3.22)
project(cpp_listener_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


nano_ros_auto_add_library(listener_lib STATIC src/Listener.cpp)

nros_components_register_node(listener_lib
    PLUGIN cpp_listener_pkg::Listener
    EXECUTABLE listener
    SHAPE configure)

target_link_libraries(listener_lib
    PUBLIC std_msgs__nano_ros_cpp)
