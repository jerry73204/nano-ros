cmake_minimum_required(VERSION 3.16)
project(nros_rmw_cyclonedds
    VERSION 0.1.0
    DESCRIPTION "Cyclone DDS RMW backend for nano-ros (Phase 117)"
    LANGUAGES C CXX
)

# ---- RPATH -----------------------------------------------------------------
# Test binaries link against `libddsc.so` in the project's
# `build/install/lib`; without an rpath the dynamic linker finds the
# system-installed `ros-humble-cyclonedds` first, which is ABI-
# incompatible with the in-tree Cyclone build (our pin is 0.10.5
# but the build flags differ).
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
list(APPEND CMAKE_BUILD_RPATH "${CMAKE_PREFIX_PATH}/lib")

# ---- C++ standard ----------------------------------------------------------
# Deliberately BELOW the C++17 floor `check-cxx-standard-floor` enforces —
# issue 1118. The floor is nros-cpp's (`component_node.hpp` needs
# `if constexpr`); this wrapper includes no nros-cpp header and links no
# `NanoRos::NanoRosCpp`, so it is not a consumer of that requirement. It ships
# into Zephyr / FreeRTOS targets instead, and the Zephyr lane compiles this
# backend at `-std=c++11` (issues 1011 / 1014, phase-416) — raising the
# declaration here would assert a dialect that lane does not provide.
# nros-cxx-floor-exempt: ships into Zephyr/FreeRTOS lanes that compile it at c++11; no nros-cpp header reaches this wrapper
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Phase 140 — install rules + Config.cmake.in template deleted.
# Consumed in-tree via `add_subdirectory(packages/rmw/cyclonedds/nros-rmw-cyclonedds)`
# from the root `CMakeLists.txt` cyclonedds branch. CycloneDDS itself
# is still a third-party dependency surfaced via find_package(CycloneDDS)
# — callers point CMAKE_PREFIX_PATH at its install prefix.

# ---- Dependencies ----------------------------------------------------------
# Cyclone DDS itself. Phase 186 — resolved via nros_provide_cyclonedds():
#   prebuilt install (find_package, on CMAKE_PREFIX_PATH / CycloneDDS_DIR) →
#   else self-provision from source (CYCLONEDDS_SOURCE_DIR; the project root
#   defaults it to the pinned third-party submodule, sccache-accelerated).
# So a bare cmake build needs no `just cyclonedds` pre-step. Sets
# NROS_CYCLONEDDS_PROVENANCE (target|find_package|source).
# ---- Platform implementation (issue 0881) ----------------------------------
# The funnel is enabled by `nros_provide_cyclonedds()` only when a platform
# target exists to funnel INTO, so before this block the standalone project
# built Cyclone with its libc heap and the funnel-ON path had no lane at all:
# a native nano-ros build resolves Cyclone through `find_package` (already
# compiled, nothing to define into), and only a cross build reached the
# funnel — tier 2, a day of latency, and a cross toolchain.
#
# That mattered because the funnel is a LINK-shape change. It was this
# project's three test executables that caught the missing link dependency the
# first time; after the dependency landed, this project stopped compiling the
# funnel and the next regression of that shape would have surfaced in tier 2.
#
# The location arrives as a cache PATH defaulted to the sibling project, the
# same convention `CYCLONEDDS_SOURCE_DIR` and the header vars already use, so
# this file still never searches the tree and a consumer can override it.
# Guarded on the targets not already existing, so a parent project providing
# its own platform wins.
set(NROS_PLATFORM_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../platform/nros-platform-posix"
    CACHE PATH "Path to a project providing the nros platform ABI implementation")
if(NOT TARGET nros_platform_posix AND NOT TARGET NanoRos::Platform
   AND EXISTS "${NROS_PLATFORM_IMPL_DIR}/CMakeLists.txt")
    add_subdirectory("${NROS_PLATFORM_IMPL_DIR}" nros_platform_impl_build)
endif()
# `NanoRos::Platform` is the Phase-138 §A canonical alias the funnel gate asks
# for. A parent nano-ros build already defines it over its INTERFACE wrapper;
# standalone there is no wrapper, so alias the archive directly.
if(NOT TARGET NanoRos::Platform AND TARGET nros_platform_posix)
    add_library(NanoRos::Platform ALIAS nros_platform_posix)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ProvideCycloneDDS.cmake")
nros_provide_cyclonedds()

# nros-rmw-cffi C ABI vtable + return codes + entity structs. Headers
# only — the backend implements the vtable and registers it via
# `nros_rmw_cffi_register()`. Path is exposed via cache var so callers
# (or `find_package(NrosRmwCyclonedds)`) can override it for out-of-tree
# builds.
set(NROS_RMW_CFFI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/nros-rmw-abi/include"
    CACHE PATH "Path to the nros-rmw-cffi public headers (rmw_vtable.h, rmw_ret.h, …)")
if(NOT EXISTS "${NROS_RMW_CFFI_DIR}/nros/rmw_vtable.h")
    # A CACHE PATH survives a source-tree rename, so the usual cause is a build
    # dir configured before the headers moved from `nros-rmw-cffi/include` to
    # `nros-rmw-abi/include` — the DEFAULT above is already correct, and the
    # stale value comes from CMakeCache.txt. Say so: the previous wording sent
    # readers looking for a missing header instead of a stale build dir.
    set(_nros_rmw_hint "")
    if(NROS_RMW_CFFI_DIR MATCHES "nros-rmw-cffi/include")
        set(_nros_rmw_hint
            " This is the PRE-RENAME path, so it came from a stale CMakeCache.txt, "
            "not from this file — delete the build directory and re-configure.")
    endif()
    message(FATAL_ERROR
        "nros-rmw-cffi headers not found at NROS_RMW_CFFI_DIR='${NROS_RMW_CFFI_DIR}'."
        "${_nros_rmw_hint} "
        "Otherwise set -DNROS_RMW_CFFI_DIR=<path-to-packages/core/nros-rmw-abi/include>.")
endif()

# ---- Static library --------------------------------------------------------
# Type-support helper (idlc + register-TU codegen). Loaded from the
# source tree at configure time so it's available before the install
# step runs.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/NrosRmwCycloneddsTypeSupport.cmake")

# Phase 177.36 — bake the rmw_dds_common ParticipantEntitiesInfo descriptor into
# the backend so every Cyclone node can publish `ros_discovery_info` (the ROS 2
# node→endpoint graph stock tooling + action `wait_for_server` read). The
# hand-mangled IDL (src/idl/rmw_dds_common_graph.idl) yields the exact wire type
# `rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_`. graph.cpp calls the
# generated `register_rmw_dds_common_graph_0()` explicitly so the descriptor TU
# is linked (no whole-archive needed).
set(_NROS_GRAPH_TYPES_DIR "${CMAKE_CURRENT_BINARY_DIR}/graph-types")
nros_rmw_cyclonedds_idlc_compile(_NROS_GRAPH_DESC_SRCS
    IDL_FILE   "${CMAKE_CURRENT_SOURCE_DIR}/src/idl/rmw_dds_common_graph.idl"
    OUTPUT_DIR "${_NROS_GRAPH_TYPES_DIR}"
    TYPE_NAME  "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_")

# Phase 117.X.3: service request-id correlation now uses an in-payload
# `cdds_request_header_t` (writer_guid[16] + sequence_number) injected
# at IDL-codegen time by `msg_to_cyclone_idl.py` for every `.srv`
# input. The backend-defined ServiceEnvelope IDL from 117.7.B is no
# longer needed and has been removed.

add_library(nros_rmw_cyclonedds STATIC
    src/vtable.cpp
    src/session.cpp
    src/publisher.cpp
    src/subscriber.cpp
    src/service.cpp
    src/descriptors.cpp
    src/graph.cpp
    src/qos.cpp
    src/nros_sertype.cpp
    # Phase 212.K.7.4 — C++ bridge for the Rust DescriptorBuilder.
    # Currently a stub returning UnsupportedFieldType until the
    # Cyclone DDS pin advances to a release exposing the public
    # `ddsi_dynamic_type_*` API; the Rust ABI is stable so only this
    # TU's body needs to change at that point.
    bridge/dynamic_type_builder.cpp
    ${_NROS_GRAPH_DESC_SRCS}
)
add_library(NrosRmwCyclonedds::NrosRmwCyclonedds ALIAS nros_rmw_cyclonedds)

target_include_directories(nros_rmw_cyclonedds
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${_NROS_GRAPH_TYPES_DIR}
)

target_link_libraries(nros_rmw_cyclonedds PUBLIC CycloneDDS::ddsc)

# Phase 186 — on the self-provision (source) path, CycloneDDS::ddsc is an
# add_subdirectory target that interface-exports only Cyclone's *public* headers.
# The wrapper (sertype_min.hpp) compiles against a few semi-internal ddsi/ddsrt
# headers that the *install* bundles but the build-tree target does not export
# (dds/ddsi/ddsi_cdrstream.h, q_protocol.h, ddsi_serdata_default.h, ddsi_sertype.h,
# plus generated dds/config.h + version.h). Add the source + generated roots
# directly. The find_package path already gets them from the install include dir.
if(NROS_CYCLONEDDS_PROVENANCE STREQUAL "source")
    target_include_directories(nros_rmw_cyclonedds PRIVATE
        "${CYCLONEDDS_SOURCE_DIR}/src/core/ddsi/include"
        "${CYCLONEDDS_SOURCE_DIR}/src/core/ddsc/include"
        "${CYCLONEDDS_SOURCE_DIR}/src/ddsrt/include"
        "${NROS_CYCLONEDDS_SOURCE_BUILD_DIR}/src/ddsrt/include")
    # The root CMakeLists whole-archives the backend together with
    # ${NROS_RMW_CYCLONEDDS_DDSC_LIBRARY}; on the find_package path that's the
    # imported libddsc, set by the rpath block below. On the source path ddsc is
    # an in-tree static target, so point the var at its build artifact (a genex,
    # evaluated when the root expands the link flag) — otherwise the final link
    # drops every dds_*/ddsrt_* symbol. Phase 186.
    set(NROS_RMW_CYCLONEDDS_DDSC_LIBRARY "$<TARGET_FILE:CycloneDDS::ddsc>"
        CACHE STRING "Resolved Cyclone DDS ddsc library (source path)" FORCE)
endif()

if(DEFINED NANO_ROS_PLATFORM
   AND (NANO_ROS_PLATFORM STREQUAL "freertos"
        OR NANO_ROS_PLATFORM STREQUAL "freertos_armcm3"))
    target_compile_definitions(nros_rmw_cyclonedds PRIVATE NROS_PLATFORM_FREERTOS)
    if(TARGET freertos_kernel)
        target_link_libraries(nros_rmw_cyclonedds PRIVATE freertos_kernel)
    endif()
    # phase-370 — the platform SHIM, which carries `nros/platform.h` on its
    # PUBLIC include interface. `internal.hpp` includes that header
    # unconditionally, so without this the freertos branch cannot compile:
    # `fatal error: nros/platform.h: No such file or directory`.
    #
    # The threadx branch below has always linked its shim; this one linked only
    # the kernel and lwIP. Neither of those carries the header, so the omission
    # was not a subtle one — it had simply never been reached, which is what
    # "Cyclone on embedded FreeRTOS is ~70% plumbed with ZERO live cells"
    # (phase-370's own framing) means in practice. The POSIX simulator is the
    # first board to compile this branch.
    if(TARGET nros_platform_freertos)
        target_link_libraries(nros_rmw_cyclonedds PRIVATE nros_platform_freertos)
    endif()
    if(TARGET lwip)
        target_link_libraries(nros_rmw_cyclonedds PRIVATE lwip)
    endif()
# `MATCHES "^threadx"` (not STREQUAL): the invocations pass the variant names
# (`threadx_riscv64` / `threadx_linux` — the root CMakeLists maps them to the
# threadx platform module the same way). Pre-287-W6 the leaves baked the bare
# `set(NANO_ROS_PLATFORM threadx)`, which is why STREQUAL used to hold.
elseif(DEFINED NANO_ROS_PLATFORM AND NANO_ROS_PLATFORM MATCHES "^threadx")
    target_compile_definitions(nros_rmw_cyclonedds PRIVATE NROS_PLATFORM_THREADX)
    if(TARGET threadx_kernel)
        target_link_libraries(nros_rmw_cyclonedds PRIVATE threadx_kernel)
    endif()
    if(TARGET nros_platform_threadx)
        target_link_libraries(nros_rmw_cyclonedds PRIVATE nros_platform_threadx)
    endif()
endif()

# Propagate Cyclone's install lib dir as a build-tree rpath so test
# binaries find `libddsc.so` from `build/install/lib` instead of any
# system-installed `ros-humble-cyclonedds` that ld.so happens to see
# first. Only meaningful for the find_package path (an imported shared
# `libddsc.so`); a self-provisioned source build links a static `ddsc`
# in-tree, so there is no external `.so` to rpath. Phase 186.
#
# phase-370 — ASK the imported target which configurations it has, rather than
# guessing two names.
#
# This probed `IMPORTED_LOCATION_RELEASE` and then the config-less
# `IMPORTED_LOCATION`. ROS Humble's CycloneDDS export has NEITHER: it ships
# `CycloneDDSTargets-none.cmake`, so the property is `IMPORTED_LOCATION_NONE`
# (Debian builds Cyclone with an empty `CMAKE_BUILD_TYPE`). Both lookups
# returned empty, so `NROS_RMW_CYCLONEDDS_DDSC_LIBRARY` was never set on the
# find_package path — and the root CMakeLists then took its `else()` branch,
# whole-archiving `nros_rmw_cyclonedds` ALONE. A raw `-Wl,--whole-archive,<file>`
# flag names a file, so it carries none of the target's PUBLIC
# `CycloneDDS::ddsc`, and every `dds_*` symbol went undefined at the final link.
#
# It stayed hidden because the workspace roots hardwired `BACKEND zenoh` (fixed
# in the same phase): the two `*-native-cyclonedds` workspace rows have been
# linking zenoh, so nothing reached this path. The phase-368 W4 RPATH stamp
# below has the same dependency and was equally inert.
#
# `IMPORTED_CONFIGURATIONS` is the target's own list, so this covers RELEASE,
# NONE, DEBUG, RELWITHDEBINFO and any vendor spelling without another guess.
set(_ddsc_loc "")
if(NROS_CYCLONEDDS_PROVENANCE STREQUAL "find_package")
    get_target_property(_ddsc_cfgs CycloneDDS::ddsc IMPORTED_CONFIGURATIONS)
    if(_ddsc_cfgs)
        # Prefer the build's own config when the target has it, so a
        # multi-config export does not silently hand back the wrong one.
        set(_ddsc_cfg_order "")
        if(CMAKE_BUILD_TYPE)
            string(TOUPPER "${CMAKE_BUILD_TYPE}" _ddsc_want)
            list(APPEND _ddsc_cfg_order "${_ddsc_want}")
        endif()
        list(APPEND _ddsc_cfg_order ${_ddsc_cfgs})
        foreach(_cfg IN LISTS _ddsc_cfg_order)
            if(NOT _ddsc_loc)
                get_target_property(_ddsc_loc CycloneDDS::ddsc IMPORTED_LOCATION_${_cfg})
            endif()
        endforeach()
    endif()
    if(NOT _ddsc_loc)
        get_target_property(_ddsc_loc CycloneDDS::ddsc IMPORTED_LOCATION)
    endif()
    if(NOT _ddsc_loc)
        message(WARNING
            "nano-ros: CycloneDDS::ddsc resolved through find_package but exposes no "
            "IMPORTED_LOCATION for any of its configurations (${_ddsc_cfgs}). The "
            "backend archive will be whole-archived WITHOUT libddsc, which fails the "
            "final link on every `dds_*` symbol. Report the Cyclone package layout.")
    endif()
endif()
if(_ddsc_loc)
    get_filename_component(_ddsc_libdir "${_ddsc_loc}" DIRECTORY)
    set(NROS_RMW_CYCLONEDDS_DDSC_LIBRARY "${_ddsc_loc}"
        CACHE FILEPATH "Resolved Cyclone DDS ddsc library path")
    set_property(TARGET nros_rmw_cyclonedds APPEND PROPERTY
        INTERFACE_LINK_OPTIONS "LINKER:-rpath,${_ddsc_libdir}")
endif()

target_compile_options(nros_rmw_cyclonedds PRIVATE
    -Wall -Wextra -Wpedantic
    # Phase 171.E: keep the wrapper freestanding-friendly. Cyclone DDS
    # itself may allocate internally, but this wrapper must not rely on
    # exceptions, RTTI, or guard variables for local statics.
    -fno-exceptions -fno-rtti -fno-threadsafe-statics
    # Phase 204.13 — per-function/data sections so a `--gc-sections` link
    # (embedded + the gc-enabled native links) can drop the wrapper's unused
    # entity trampolines instead of pulling the whole archive member.
    -ffunction-sections -fdata-sections
)

set_target_properties(nros_rmw_cyclonedds PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    EXPORT_NAME NrosRmwCyclonedds
)

# Phase 204.13 — opt-in interprocedural optimization (LTO) for size/perf-critical
# consumption. Off by default (cross-language link with `rust-lld` + slim-LTO
# ddsc objects is fragile — see 204.14); enable with
# `-DNROS_RMW_CYCLONEDDS_IPO=ON` where the toolchain supports it. Pair with
# `-DCMAKE_BUILD_TYPE=MinSizeRel` (`-Os`) for the smallest wrapper.
option(NROS_RMW_CYCLONEDDS_IPO "Enable IPO/LTO on the Cyclone RMW wrapper" OFF)
if(NROS_RMW_CYCLONEDDS_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT _nros_ipo_ok OUTPUT _nros_ipo_msg)
    if(_nros_ipo_ok)
        set_target_properties(nros_rmw_cyclonedds PROPERTIES
            INTERPROCEDURAL_OPTIMIZATION ON)
    else()
        message(WARNING "NROS_RMW_CYCLONEDDS_IPO requested but unsupported: ${_nros_ipo_msg}")
    endif()
endif()

# ---- Tests -----------------------------------------------------------------
# Default the CTest harness ON only when this is the top-level project
# (`just cyclonedds build`). When add_subdirectory'd into an example or
# the root NanoRos build, the backend's own test fixtures must not be
# built — they generate descriptors for their own type set and are not
# part of the consuming app.
option(NROS_RMW_CYCLONEDDS_BUILD_TESTS "Build CTest harness" "${PROJECT_IS_TOP_LEVEL}")
if(NROS_RMW_CYCLONEDDS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
