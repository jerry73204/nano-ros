# cmake/toolchain/arm-freertos-armcm3.cmake
#
# CMake toolchain file for FreeRTOS on ARM Cortex-M3 (MPS2-AN385).
#
# Selects the arm-none-eabi cross-compiler and sets the Rust target triple
# so that Corrosion compiles nros-c / nros-cpp for thumbv7m-none-eabi.
#
# Usage:
#   cmake -S . -B build \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain/arm-freertos-armcm3.cmake \
#         -DNANO_ROS_RMW=zenoh \
#         -DNANO_ROS_PLATFORM=freertos_armcm3 \
#         -DNANO_ROS_BUILD_CODEGEN=OFF
#   cmake --build build
#   cmake --install build --prefix /path/to/prefix

set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  arm)

# Issue 1117 — WHICH arm-none-eabi-gcc, and from WHERE. The bare names below
# used to resolve on PATH alone, so a host that had never run `nros setup
# --tool arm-none-eabi-gcc` silently built with Ubuntu's 10.3.1 against a
# 13.2.rel1 pin, and said nothing. Shared module: resolution order, the
# provenance line, and the GCC floor.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosCrossToolchain.cmake")
nros_cross_toolchain_resolve(
    TOOL         arm-none-eabi-gcc
    PREFIXES     arm-none-eabi
    OVERRIDE_VAR NROS_ARM_NONE_EABI_PREFIX
    OUT_PREFIX   _NROS_ARM_PREFIX
    OUT_ORIGIN   _NROS_ARM_ORIGIN)
nros_cross_toolchain_report(
    TOOL   arm-none-eabi-gcc
    PREFIX "${_NROS_ARM_PREFIX}"
    ORIGIN "${_NROS_ARM_ORIGIN}"
    OVERRIDE_VAR NROS_ARM_NONE_EABI_PREFIX)

set(CMAKE_C_COMPILER    ${_NROS_ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER  ${_NROS_ARM_PREFIX}-g++)
set(CMAKE_ASM_COMPILER  ${_NROS_ARM_PREFIX}-gcc)
set(CMAKE_AR            ${_NROS_ARM_PREFIX}-ar  CACHE FILEPATH "Archiver")
set(CMAKE_RANLIB        ${_NROS_ARM_PREFIX}-ranlib CACHE FILEPATH "Ranlib")

set(CMAKE_C_FLAGS_INIT   "-mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -std=c++14 -ffreestanding")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m3 -mthumb")

# Rust target triple — read by Corrosion and NanoRosGenerateInterfaces.cmake
# for cross-compilation.  Must be set before FetchContent_MakeAvailable(Corrosion).
set(Rust_CARGO_TARGET "thumbv7m-none-eabi" CACHE STRING "Rust target triple" FORCE)

# Don't search host paths for libraries / headers when cross-compiling.
# PROGRAM is NEVER so CMake can still find host tools (cmake, ninja, etc.).
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)

# Skip compiler capability tests — the cross-compiler produces bare-metal
# ELFs that cannot be executed on the host.
set(CMAKE_C_COMPILER_WORKS   TRUE CACHE BOOL "Compiler works" FORCE)
set(CMAKE_CXX_COMPILER_WORKS TRUE CACHE BOOL "Compiler works" FORCE)

# Issue 0366 — COMPILER_WORKS only skips the "does it run" probe; CMake's
# SEPARATE ABI-info detection still `try_compile`s a full EXECUTABLE, and on
# arm-none-eabi newlib that pulls in unresolved `_write`/`_read`/`_sbrk`/…
# syscall stubs (the board's stubs link only in the real build), so configure
# aborts "Detecting C compiler ABI info - failed". Building the probe as a
# STATIC_LIBRARY makes ABI detection compile-only (no link) — the standard
# bare-metal-CMake idiom.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
