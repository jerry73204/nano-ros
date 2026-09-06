cmake_minimum_required(VERSION 3.22)
project(component_node_poc LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable(component_node_poc src/main.cpp)
ament_target_dependencies(component_node_poc std_msgs)

install(TARGETS component_node_poc DESTINATION lib/${PROJECT_NAME})
ament_package()
