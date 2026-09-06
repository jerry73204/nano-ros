cmake_minimum_required(VERSION 3.16)
project(nros_rmw_uorb
    VERSION 0.1.0
    DESCRIPTION "uORB RMW backend for nano-ros (Phase 115.K.4 — C++ port)"
    LANGUAGES C CXX
)

# ---- C++ standard ----------------------------------------------------------
# Deliberately BELOW the C++17 floor `check-cxx-standard-floor` enforces —
# issue 1118. That floor exists because `component_node.hpp` needs
# `if constexpr`, and this backend includes no nros-cpp header and links no
# `NanoRos::NanoRosCpp`, so the requirement does not reach it. What DOES reach
# it is PX4: this crate is consumed by `add_subdirectory` from a PX4 module
# shell, and PX4 builds every module `-std=gnu++14 -Werror` (the same
# constraint that made an `inline` variable in `node.hpp` uncompilable in a
# PX4 module — phase-325 W2). `CXX_STANDARD_REQUIRED` would emit `-std=c++17`
# LAST and win over PX4's flag, compiling this TU at a dialect the rest of
# the firmware is not built with, untested.
# nros-cxx-floor-exempt: consumed inside PX4, which builds modules -std=gnu++14 -Werror; no nros-cpp header reaches this crate
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Phase 140 — install rules + Config.cmake.in template deleted.
# Consumed in-tree via `add_subdirectory(packages/rmw/uorb/nros-rmw-uorb)`
# from the PX4 integration shell (integrations/px4/module-template/).

# ---- Dependencies ----------------------------------------------------------
# nros-rmw-cffi C ABI: headers only (vtable + entity structs + ret codes).
set(NROS_RMW_CFFI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/nros-rmw-abi/include"
    CACHE PATH "Path to the nros-rmw-cffi public headers (rmw_vtable.h, rmw_ret.h, …)")
if(NOT EXISTS "${NROS_RMW_CFFI_DIR}/nros/rmw_vtable.h")
    message(FATAL_ERROR
        "nros-rmw-cffi headers not found at NROS_RMW_CFFI_DIR='${NROS_RMW_CFFI_DIR}'. "
        "Set -DNROS_RMW_CFFI_DIR=<path-to-packages/core/nros-rmw-abi/include>.")
endif()

# PX4 module SDK (provides <uORB/uORB.h>, <px4_platform_common/log.h>,
# ScheduledWorkItem, ModuleBase<T>). Resolution mode follows the PX4
# convention:
#   * If the project is configured INSIDE a PX4 module build,
#     px4_add_module's environment already provides the headers and
#     this find_package is a no-op.
#   * Standalone builds (CI smoke / scaffold) accept
#     `-DPX4_FIRMWARE_DIR=<path>` to fall back to the upstream tree.
# K.4.0 ships the scaffold without linking uORB at all — `find_package`
# is wrapped in a guard so the smoke test can build on hosts that
# don't have PX4 installed.
option(NROS_RMW_UORB_LINK_PX4 "Link against the PX4 module SDK (uORB, workqueue)" OFF)
# Push-wake glue.cpp pulls in `<uORB/SubscriptionCallback.hpp>` which
# transitively requires `<px4_boardconfig.h>` — a per-board Kconfig
# output that only exists inside a real `px4_add_module()` build
# (under `$PX4_BINARY_DIR`). Standalone consumers building only the
# Rust-side ABI (host-validation, smoke) keep this OFF; the actual
# PX4-module integration build flips it ON via the module's own
# CMakeLists, where the include path resolves automatically.
option(NROS_RMW_UORB_BUILD_PX4_GLUE
    "Compile the SubscriptionCallbackWorkItem push-wake glue (requires a px4_add_module build context)"
    OFF)
if(NROS_RMW_UORB_LINK_PX4)
    if(NOT DEFINED PX4_FIRMWARE_DIR)
        message(FATAL_ERROR
            "NROS_RMW_UORB_LINK_PX4=ON requires -DPX4_FIRMWARE_DIR=<path> "
            "(typically the consumer's PX4-Autopilot checkout).")
    endif()
    message(STATUS "nros-rmw-uorb-cpp: PX4 SDK path = ${PX4_FIRMWARE_DIR}")
    # PX4-Autopilot ships `<uORB/uORB.h>` under
    # `<repo>/platforms/common/uORB`. The exact include layout shifts
    # per PX4 release; the path expressions below match the upstream
    # 1.14+ tree. Older releases that staged the headers elsewhere
    # need a fork-side override of NROS_RMW_UORB_PX4_UORB_INCLUDE_DIRS.
    if(NOT DEFINED NROS_RMW_UORB_PX4_UORB_INCLUDE_DIRS)
        # `<uORB/uORB.h>` lives at `platforms/common/uORB/uORB.h` —
        # the include base is `platforms/common`, not `platforms/common/uORB`.
        # `<px4_platform_common/...>` resolves via `platforms/common/include`.
        # `<containers/...>`, `<px4.h>` resolve via `src/include`.
        # `<drivers/...>`, `<lib/...>` resolve via `src/`.
        set(NROS_RMW_UORB_PX4_UORB_INCLUDE_DIRS
            "${PX4_FIRMWARE_DIR}/platforms/common"
            "${PX4_FIRMWARE_DIR}/platforms/common/include"
            "${PX4_FIRMWARE_DIR}/src/include"
            "${PX4_FIRMWARE_DIR}/src"
            "${PX4_FIRMWARE_DIR}/src/lib")
    endif()
    # uORB symbol resolution at link time comes from the per-module
    # `px4_add_module()` macro which already brings uORB in scope; we
    # rely on that for both `orb_advertise_multi` / `orb_publish` /
    # ... declarations and definitions. Inside the PX4 module's
    # CMake context, NROS_RMW_UORB_PX4_UORB_LIBRARIES typically stays
    # empty (the module pulls uORB transitively via its module
    # dependency list). Standalone builds that link against a
    # pre-built uORB archive populate it on the cmake command line.
endif()

# ---- Static library --------------------------------------------------------
set(_nros_rmw_uorb_sources
    src/vtable.cpp
    src/session.cpp
    src/publisher.cpp
    src/subscriber.cpp
    src/service.cpp
    src/topic_registry.cpp
    src/callback_default.cpp
)
if(NROS_RMW_UORB_BUILD_PX4_GLUE)
    if(NOT NROS_RMW_UORB_LINK_PX4)
        message(FATAL_ERROR
            "NROS_RMW_UORB_BUILD_PX4_GLUE=ON requires NROS_RMW_UORB_LINK_PX4=ON.")
    endif()
    list(APPEND _nros_rmw_uorb_sources src/px4_callback_glue.cpp)
endif()

add_library(nros_rmw_uorb STATIC ${_nros_rmw_uorb_sources})
add_library(NrosRmwUorb::NrosRmwUorb ALIAS nros_rmw_uorb)

target_include_directories(nros_rmw_uorb
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(NROS_RMW_UORB_LINK_PX4)
    # Flip uorb_abi.hpp from "declare the ABI ourselves" to
    # "#include <uORB/uORB.h>" from the PX4 SDK.
    target_compile_definitions(nros_rmw_uorb PRIVATE
        NROS_RMW_UORB_USE_PX4_HEADER=1)
    target_include_directories(nros_rmw_uorb PRIVATE
        ${NROS_RMW_UORB_PX4_UORB_INCLUDE_DIRS})
    # PX4's `<uORB/uORB.h>` uses `__EXPORT` / `__PRIVATE` macros
    # supplied by PX4's toolchain via `-include visibility.h`.
    # Replicate that contract for standalone host builds.
    target_compile_options(nros_rmw_uorb PRIVATE
        -include visibility.h)
    if(DEFINED NROS_RMW_UORB_PX4_UORB_LIBRARIES)
        target_link_libraries(nros_rmw_uorb PUBLIC
            ${NROS_RMW_UORB_PX4_UORB_LIBRARIES})
    endif()
endif()

target_compile_options(nros_rmw_uorb PRIVATE
    -Wall -Wextra -Wpedantic
    # PX4 modules build without exceptions + RTTI; match.
    -fno-exceptions -fno-rtti
)

set_target_properties(nros_rmw_uorb PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    EXPORT_NAME NrosRmwUorb
)

# ---- Tests -----------------------------------------------------------------
# K.4.0 smoke test: confirms the register entry point compiles +
# links + returns OK from a minimal C++ test driver.
option(NROS_RMW_UORB_BUILD_TESTS "Build the nros-rmw-uorb smoke test" ON)
if(NROS_RMW_UORB_BUILD_TESTS)
    # Issue 0787 — `enable_testing()` + `add_test` were MISSING: the smoke
    # executable was built and never registered, so `ctest` in this build tree
    # reported "No tests were found!!!" and a lane running it would have gone
    # green over nothing. Same shape as the target itself never being built.
    enable_testing()
    add_executable(nros_rmw_uorb_register_smoke
        tests/register_smoke.cpp
    )
    target_link_libraries(nros_rmw_uorb_register_smoke PRIVATE nros_rmw_uorb)
    target_include_directories(nros_rmw_uorb_register_smoke PRIVATE
        ${NROS_RMW_CFFI_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    # nros_rmw_cffi_register lives in the Rust-side staticlib; the
    # smoke test stubs it locally (see tests/register_smoke.cpp) so
    # K.4.0 doesn't need a Rust toolchain on the build host.
    add_test(
        NAME nros_rmw_uorb_register_smoke
        COMMAND nros_rmw_uorb_register_smoke
    )
endif()

# Phase 140 — install rules deleted; consumed in-tree via add_subdirectory.
