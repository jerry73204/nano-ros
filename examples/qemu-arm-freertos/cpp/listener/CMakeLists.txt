cmake_minimum_required(VERSION 3.22)
project(cpp_listener LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable(cpp_listener src/main.cpp)
ament_target_dependencies(cpp_listener std_msgs)

install(TARGETS cpp_listener DESTINATION lib/${PROJECT_NAME})
ament_package()
