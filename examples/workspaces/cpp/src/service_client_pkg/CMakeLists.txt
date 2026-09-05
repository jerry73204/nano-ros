cmake_minimum_required(VERSION 3.22)
project(service_client_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

# phase-263 A1 (services, C++) — poll-model AddTwoInts client; republishes the sum on /sum.

nano_ros_auto_add_library(service_client_lib STATIC src/AddClient.cpp)

nros_components_register_node(service_client_lib
    PLUGIN service_client_pkg::AddClient
    EXECUTABLE add_client
    SHAPE configure
    # phase-412 -- `ENTITIES` is retired. What this component creates is
    # stated in demo_bringup/launch/service_client.contract.yaml, beside
    # the launch file that runs it. That contract marks the service
    # `external: server`, because the server is the separate add_server
    # image; without that mark (ros-launch-manifest v0.1.23, issue 1083)
    # a client-only tree could not be written at all.
)

if(TARGET example_interfaces__nano_ros_cpp)
    target_link_libraries(service_client_lib
        PUBLIC example_interfaces__nano_ros_cpp)
endif()
