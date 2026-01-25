cmake_minimum_required(VERSION 3.15)
project(nano_ros_cpp
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Option to build shared or static library
option(BUILD_SHARED_LIBS "Build shared library" ON)

# ============================================================================
# Corrosion: CMake integration for Cargo
# ============================================================================
include(FetchContent)

FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.5.1
)
FetchContent_MakeAvailable(Corrosion)

# Import the Rust crate - this builds libnanoros_cpp_bridge.a
corrosion_import_crate(MANIFEST_PATH Cargo.toml)

# Get the cxx-generated header location
# cxx generates headers in target/<profile>/cxxbridge/
set(CXX_BRIDGE_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo/build/${Rust_CARGO_TARGET_CACHED}/cxxbridge")

# ============================================================================
# C++ Wrapper Library
# ============================================================================
add_library(nano_ros_cpp
    cpp/init.cpp
)

target_include_directories(nano_ros_cpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # cxx-generated headers (nano-ros-cpp-bridge/src/lib.rs.h)
        ${CXX_BRIDGE_DIR}
        # cxx runtime headers
        ${CMAKE_CURRENT_BINARY_DIR}/cargo/build/${Rust_CARGO_TARGET_CACHED}/cxxbridge/rust
)

# Link against the Rust static library
# Note: Corrosion converts hyphens to underscores in target names
target_link_libraries(nano_ros_cpp
    PRIVATE
        nano_ros_cpp_bridge
)

# Platform-specific link dependencies
if(UNIX AND NOT APPLE)
    target_link_libraries(nano_ros_cpp PRIVATE pthread dl m)
elseif(APPLE)
    target_link_libraries(nano_ros_cpp PRIVATE pthread dl)
endif()

# ============================================================================
# Installation
# ============================================================================
include(GNUInstallDirs)

install(TARGETS nano_ros_cpp
    EXPORT nano_ros_cpp-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT nano_ros_cpp-targets
    FILE nano_ros_cpp-targets.cmake
    NAMESPACE nano_ros::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
)

# Generate package config
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nano_ros_cpp-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
)

# ============================================================================
# Example executable (optional)
# ============================================================================
option(BUILD_EXAMPLES "Build example executables" OFF)

if(BUILD_EXAMPLES)
    add_executable(nano_ros_cpp_example examples/hello.cpp)
    target_link_libraries(nano_ros_cpp_example PRIVATE nano_ros_cpp)
endif()
