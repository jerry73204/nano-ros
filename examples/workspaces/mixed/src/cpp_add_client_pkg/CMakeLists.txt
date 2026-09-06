cmake_minimum_required(VERSION 3.22)
project(cpp_add_client_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

# phase-263 A1 (services, C++) — poll-model AddTwoInts client; republishes the sum on /sum.

nano_ros_auto_add_library(add_client_lib STATIC src/AddClient.cpp)

nros_components_register_node(add_client_lib
    PLUGIN cpp_add_client_pkg::AddClient
    EXECUTABLE add_client
    SHAPE configure)

if(TARGET example_interfaces__nano_ros_cpp)
    target_link_libraries(add_client_lib
        PUBLIC example_interfaces__nano_ros_cpp)
endif()
