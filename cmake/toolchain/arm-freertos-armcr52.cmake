# cmake/toolchain/arm-freertos-armcr52.cmake
#
# CMake toolchain file for FreeRTOS on ARM Cortex-R52 (NXP S32Z270 RTU,
# phase-372). Sibling of arm-freertos-armcm3.cmake; the flag set is the
# hardware-proven one from the ASI consumer's retired hand toolchain
# (arm-cortex-r52.cmake): VFP/NEON hard-float, section GC.
#
# C++ builds WITH exceptions and RTTI here, unlike the CM3 file: the
# Cortex-R52 consumer this profile exists for (ASI) mandates C++17 with
# both (its Autoware controller and CycloneDDS C++ bindings use them),
# and the R52 has the RAM to carry the unwind tables the M3 image cannot.
#
# Rust: `armv8r-none-eabihf` — the exact ARMv8-R AArch32 target, shipped
# by rustup (rust-std available; verified 2026-08-22). If a consumer's
# toolchain predates it, `armv7r-none-eabihf` (tier 2) is the fallback:
# v7-R code runs unmodified on the R52.
#
# Usage:
#   cmake -S . -B build \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain/arm-freertos-armcr52.cmake \
#         -DNANO_ROS_RMW=cyclonedds \
#         -DNANO_ROS_PLATFORM=freertos \
#         -DNANO_ROS_BOARD=s32z270-freertos
#   cmake --build build

set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  cortex-r52)

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

set(_cr52_flags "-mcpu=cortex-r52 -mfpu=neon-fp-armv8 -mfloat-abi=hard -ffunction-sections -fdata-sections -fno-common")

set(CMAKE_C_FLAGS_INIT   "${_cr52_flags}")
set(CMAKE_CXX_FLAGS_INIT "${_cr52_flags} -fexceptions -frtti -std=c++17")
set(CMAKE_ASM_FLAGS_INIT "${_cr52_flags}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--gc-sections -nostartfiles")

# Rust target triple — read by Corrosion and NanoRosGenerateInterfaces.cmake
# for cross-compilation.  Must be set before FetchContent_MakeAvailable(Corrosion).
set(Rust_CARGO_TARGET "armv8r-none-eabihf" CACHE STRING "Rust target triple" FORCE)

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

# Issue 0366 — ABI detection must not link a full executable (newlib
# syscall stubs resolve only in the real image). Same idiom as the CM3
# file; see the comment there.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
