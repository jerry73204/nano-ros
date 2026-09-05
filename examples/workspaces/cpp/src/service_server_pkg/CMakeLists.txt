cmake_minimum_required(VERSION 3.22)
project(service_server_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

# phase-263 A1 (services, C++) — typed AddTwoInts server using the TYPED bind_service<Svc,…>
# (generated example_interfaces C++ srv bindings).

nano_ros_auto_add_library(service_server_lib STATIC src/AddServer.cpp)

nros_components_register_node(service_server_lib
    PLUGIN service_server_pkg::AddServer
    EXECUTABLE add_server
    SHAPE configure
    # phase-412 -- `ENTITIES` is retired. What this component creates is
    # stated in demo_bringup/launch/service_server.contract.yaml, beside the launch
    # file that runs it, and reaches the sizing through the SystemModel.
)

# Guard with if(TARGET) (Zephyr whole-archives the generated interface; see talker_pkg).
if(TARGET example_interfaces__nano_ros_cpp)
    target_link_libraries(service_server_lib
        PUBLIC example_interfaces__nano_ros_cpp)
endif()
