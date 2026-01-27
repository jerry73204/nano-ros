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

# Get the Rust target triple (set by Corrosion after FetchContent)
if(NOT DEFINED Rust_CARGO_TARGET_CACHED)
    # Fallback: detect host target
    execute_process(
        COMMAND rustc -vV
        OUTPUT_VARIABLE RUSTC_VERSION_OUTPUT
    )
    string(REGEX MATCH "host: ([^\n]+)" _ ${RUSTC_VERSION_OUTPUT})
    set(Rust_CARGO_TARGET_CACHED ${CMAKE_MATCH_1})
endif()

# cxx generates headers in cargo/build/<target>/cxxbridge/
# Use CMAKE_BINARY_DIR (top-level) because Corrosion puts cargo output there,
# not in CMAKE_CURRENT_BINARY_DIR when used as a subdirectory
set(CXX_BRIDGE_DIR "${CMAKE_BINARY_DIR}/cargo/build/${Rust_CARGO_TARGET_CACHED}/cxxbridge")

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
        # cxx-generated headers (nano-ros-cpp-bridge/src/lib.rs.h and rust/cxx.h)
        ${CXX_BRIDGE_DIR}
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
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nano_ros_generate_interfaces.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
)

# Install the generator script
install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/nano_ros_generate_cpp.py
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp/../scripts
)
