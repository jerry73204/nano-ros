cmake_minimum_required(VERSION 3.22)
project(transform_poc LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable(transform_poc src/main.cpp)
ament_target_dependencies(transform_poc std_msgs)

install(TARGETS transform_poc DESTINATION lib/${PROJECT_NAME})
ament_package()
