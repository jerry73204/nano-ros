cmake_minimum_required(VERSION 3.16)
project(cpp_service_client CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add nano-ros-cpp as subdirectory
set(NANO_ROS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../crates/nano-ros-cpp")
add_subdirectory(${NANO_ROS_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nano-ros-cpp)

# Include the interface generation macro
include(${NANO_ROS_CPP_DIR}/cmake/nano_ros_generate_interfaces.cmake)

# Generate service interfaces
nano_ros_generate_interfaces(${PROJECT_NAME}
    "srv/AddTwoInts.srv"
)

# Create executable
add_executable(cpp_service_client src/main.cpp)
target_link_libraries(cpp_service_client
    PRIVATE
        nano_ros_cpp
        ${PROJECT_NAME}_srv
)
