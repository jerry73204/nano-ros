cmake_minimum_required(VERSION 3.16)
project(nros_rmw_xrce
    VERSION 0.1.0
    DESCRIPTION "micro-XRCE-DDS-Client RMW backend for nano-ros (Phase 115.K.2)"
    LANGUAGES C
)

# ---- C standard ------------------------------------------------------------
# Pure C99 — micro-XRCE-DDS-Client itself is C99, and the nros-rmw-cffi
# C ABI is C99. Stay portable so this scaffold can ship into Zephyr /
# FreeRTOS / NuttX targets without bumping the toolchain requirement.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ---- Dependencies ----------------------------------------------------------
# nros-rmw-cffi C ABI vtable + return codes + entity structs. Headers
# only — the backend implements the vtable and registers it via
# `nros_rmw_cffi_register()`. Path is exposed via cache var so callers
# (or `find_package(NrosRmwXrce)`) can override it for out-of-tree
# builds. Mirrors `nros-rmw-cyclonedds`'s convention.
set(NROS_RMW_CFFI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/nros-rmw-abi/include"
    CACHE PATH "Path to the nros-rmw-cffi public headers (rmw_vtable.h, rmw_ret.h, …)")
if(NOT EXISTS "${NROS_RMW_CFFI_DIR}/nros/rmw_vtable.h")
    message(FATAL_ERROR
        "nros-rmw-cffi headers not found at NROS_RMW_CFFI_DIR='${NROS_RMW_CFFI_DIR}'. "
        "Set -DNROS_RMW_CFFI_DIR=<path-to-packages/core/nros-rmw-abi/include>.")
endif()

set(NROS_PLATFORM_CFFI_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../../platform/nros-platform-api/include"
    CACHE PATH "Path to the nros-platform-cffi public headers (platform.h, platform_net.h, …)")
if(NOT EXISTS "${NROS_PLATFORM_CFFI_INCLUDE}/nros/platform_net.h")
    message(FATAL_ERROR
        "nros-platform-cffi headers not found at NROS_PLATFORM_CFFI_INCLUDE='${NROS_PLATFORM_CFFI_INCLUDE}'. "
        "Set -DNROS_PLATFORM_CFFI_INCLUDE=<path-to-packages/platform/nros-platform-api/include>.")
endif()

# micro-XRCE-DDS-Client + micro-CDR (vendored under xrce-sys). Phase
# 115.K.2.1 compiles their sources directly into nros_rmw_xrce
# rather than going through the upstream CMakeLists' add_subdirectory
# / find_package machinery. The upstream client's `find_package(microcdr
# REQUIRED CONFIG)` requires a separate install step that buys nothing
# in our in-tree case; the inline-compile approach was proven by the
# Rust lane first, and since issue 1068 the two lanes share ONE source
# list (`../xrce-sources.txt`) rather than mirroring it.
set(MICROXRCEDDS_CLIENT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../xrce-sys/micro-xrce-dds-client"
    CACHE PATH "Path to vendored micro-xrce-dds-client checkout")
set(MICROCDR_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../xrce-sys/micro-cdr"
    CACHE PATH "Path to vendored micro-cdr checkout")

# ---- The IP link knob ------------------------------------------------------
# Issue 1068 / phase-204.7. `NROS_LINK_IP=0` sheds the IP (UDP/TCP) transport on
# a serial-only node. The Rust lane has honoured it since 204.7; this lane
# ignored it, so a serial-only XRCE node could not be built from C or C++.
#
# It gates TWO things and both have to move together, or the generated
# `<uxr/client/config.h>` promises a profile whose TUs are not compiled:
# the `UCLIENT_PROFILE_UDP` / `UCLIENT_PROFILE_TCP` defines below, and the
# `uxr_ip_udp` source group in the shared manifest further down. `build.rs`
# makes the same pairing.
#
# `-DNROS_LINK_IP=0` wins; otherwise the ENVIRONMENT variable is honoured,
# because that is how 204.7 delivers it (a generated `.cargo/config.toml [env]`
# block on the Rust side, a plain export here). Unset → IP on, unchanged.
if(NOT DEFINED NROS_LINK_IP)
    if(DEFINED ENV{NROS_LINK_IP})
        set(_xrce_link_ip_default "$ENV{NROS_LINK_IP}")
    else()
        set(_xrce_link_ip_default ON)
    endif()
    set(NROS_LINK_IP "${_xrce_link_ip_default}"
        CACHE STRING "Link the IP (UDP/TCP) transport. 0/false/off drops it (serial-only node).")
endif()
# `if()` reads 0/OFF/FALSE/NO/N/IGNORE/NOTFOUND/"" as false, case-insensitively,
# so the `0` / `false` / `off` spellings `build.rs` accepts all land here too.
if(NROS_LINK_IP)
    set(_xrce_ip ON)
else()
    set(_xrce_ip OFF)
endif()

# ---- Vendored versions: DERIVED, never restated (issue 1069) ---------------
# NROS-XRCE-VERSIONS-BEGIN
# `MICROCDR_VERSION_STR` compiled as `"2.4.1"` here and `"2.0.2"` in the Rust
# lane, for one vendored tree that is 2.0.2 — three numbers, none authoritative.
# Both upstream templates take the same `@PROJECT_VERSION*@` tokens, and this
# file set them ONCE for the client and never reset them, so the micro-CDR
# `configure_file` below silently inherited the client's. That is not a bug an
# ordering fix would retire: four hand-written restatements of one fact is the
# shape that produced issue 1068 one file over.
#
# The fact is already in-tree, and it is upstream's OWN statement: each vendored
# CMakeLists says `project(<name> VERSION "X.Y.Z")`. Both lanes read that line
# now — `nros-rmw-xrce-cffi/build.rs` has `vendored_project_version()`, this file
# has `_nros_xrce_project_version()`. Gate: `just check xrce-vendored-versions`.
#
# NOT `git describe --tags`, which issue 1069 proposed: a submodule is fetched
# by SHA with no tags, so `git -C …/micro-xrce-dds-client describe --tags`
# answers "No tags can describe" on a normal checkout. Measured, phase-420 W9.
#
# The macros DO have a reader, contrary to the issue's "zero readers": upstream's
# own `#if UXR_CLIENT_VERSION_MAJOR >= 4` tripwire at the bottom of
# `uxr/client/config.h.in`, three `#error`s that fire when the client crosses to
# 4.x. Compiling MAJOR=2 against a 3.0.1 tree armed it a whole major late.
function(_nros_xrce_project_version _cmakelists _project _prefix)
    if(NOT EXISTS "${_cmakelists}")
        message(FATAL_ERROR
            "nros-rmw-xrce: '${_cmakelists}' not found — the vendored ${_project} "
            "checkout is missing. Run `nros setup --source micro-cdr --source "
            "micro-xrce-dds-client` (or `git submodule update --init`).")
    endif()
    file(STRINGS "${_cmakelists}" _lines
        REGEX "project[ \t]*\\([ \t]*${_project}[ \t]+VERSION")
    list(LENGTH _lines _n)
    if(NOT _n EQUAL 1)
        message(FATAL_ERROR
            "nros-rmw-xrce: expected exactly one `project(${_project} VERSION …)` "
            "line in '${_cmakelists}', found ${_n}. Upstream changed how it states "
            "its version — update BOTH lanes (this function and build.rs's "
            "`vendored_project_version`) together.")
    endif()
    list(GET _lines 0 _line)
    if(NOT _line MATCHES "VERSION[ \t]+\"?([0-9]+)\\.([0-9]+)\\.([0-9]+)\"?")
        message(FATAL_ERROR
            "nros-rmw-xrce: could not read an X.Y.Z version out of '${_line}'.")
    endif()
    set(${_prefix}_VERSION_MAJOR "${CMAKE_MATCH_1}" PARENT_SCOPE)
    set(${_prefix}_VERSION_MINOR "${CMAKE_MATCH_2}" PARENT_SCOPE)
    set(${_prefix}_VERSION_PATCH "${CMAKE_MATCH_3}" PARENT_SCOPE)
    set(${_prefix}_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}"
        PARENT_SCOPE)
endfunction()

# Re-configure when a submodule bump moves either statement; without this a
# configured build dir keeps yesterday's number in its generated headers.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${MICROXRCEDDS_CLIENT_DIR}/CMakeLists.txt"
    "${MICROCDR_DIR}/CMakeLists.txt")

_nros_xrce_project_version("${MICROXRCEDDS_CLIENT_DIR}/CMakeLists.txt"
    microxrcedds_client _uxr)
_nros_xrce_project_version("${MICROCDR_DIR}/CMakeLists.txt"
    microcdr _ucdr)
message(STATUS
    "nano-ros: nros-rmw-xrce vendored micro-XRCE-DDS-Client ${_uxr_VERSION} "
    "+ micro-CDR ${_ucdr_VERSION} (derived from their CMakeLists, issue 1069)")
# The client also states which micro-CDR it expects — `set(_microcdr_version …)`.
# Cross-checking it against `_ucdr_VERSION` catches bumping one submodule and not
# the other; it lives in the GATE rather than here so there is one implementation
# of it instead of one per lane.
# NROS-XRCE-VERSIONS-END

# Generate `<uxr/client/config.h>` from the upstream template. Values
# match the Rust lane's `posix_compat = true, mtu = 4096` defaults; further
# tuning lands when 115.K.2.5 flips the C backend on under
# `-DNROS_C_RMW=xrce`.
set(_uxr_config_in   "${MICROXRCEDDS_CLIENT_DIR}/include/uxr/client/config.h.in")
set(_uxr_config_out  "${CMAKE_CURRENT_BINARY_DIR}/uxr_generated/uxr/client/config.h")

# Map the @TOKEN@ placeholders the template expects to our chosen
# values. `set(... PARENT_SCOPE)` not needed — configure_file reads
# the current scope.
#
# `PROJECT_VERSION*` is a SHARED name: both templates take it, so it must be
# re-pointed at the right tree before each `configure_file` and never left
# standing. That is the 1069 bug — see the micro-CDR block below.
set(PROJECT_VERSION_MAJOR  ${_uxr_VERSION_MAJOR})
set(PROJECT_VERSION_MINOR  ${_uxr_VERSION_MINOR})
set(PROJECT_VERSION_PATCH  ${_uxr_VERSION_PATCH})
set(PROJECT_VERSION        ${_uxr_VERSION})

set(UCLIENT_MAX_OUTPUT_BEST_EFFORT_STREAMS 1)
set(UCLIENT_MAX_OUTPUT_RELIABLE_STREAMS    1)
set(UCLIENT_MAX_INPUT_BEST_EFFORT_STREAMS  1)
set(UCLIENT_MAX_INPUT_RELIABLE_STREAMS     1)
set(UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS 10)
set(UCLIENT_MIN_SESSION_CONNECTION_INTERVAL 1000)
set(UCLIENT_MIN_HEARTBEAT_TIME_INTERVAL    100)
set(UCLIENT_UDP_TRANSPORT_MTU              4096)
set(UCLIENT_TCP_TRANSPORT_MTU              4096)
set(UCLIENT_SERIAL_TRANSPORT_MTU           512)
set(UCLIENT_CUSTOM_TRANSPORT_MTU           4096)
set(UCLIENT_SHARED_MEMORY_MAX_ENTITIES     4)
set(UCLIENT_SHARED_MEMORY_STATIC_MEM_SIZE  10)
set(UCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT  10000)
set(UCLIENT_TWEAK_XRCE_WRITE_LIMIT         1)

# Upstream template uses #cmakedefine — every feature toggle becomes
# either `#define` or `/* #undef */`. Set vars to true/empty.
set(UCLIENT_PROFILE_DISCOVERY              TRUE)
# Paired with the `uxr_ip_udp` source group — see "The IP link knob" above.
if(_xrce_ip)
    set(UCLIENT_PROFILE_UDP                TRUE)
    set(UCLIENT_PROFILE_TCP                TRUE)
else()
    set(UCLIENT_PROFILE_UDP)
    set(UCLIENT_PROFILE_TCP)
endif()
set(UCLIENT_PROFILE_SERIAL                 TRUE)
set(UCLIENT_PROFILE_CUSTOM_TRANSPORT       TRUE)
set(UCLIENT_PROFILE_STREAM_FRAMING         TRUE)
set(UCLIENT_PROFILE_MULTITHREAD)
set(UCLIENT_PROFILE_SHARED_MEMORY)
set(UCLIENT_PROFILE_MATCHING)
set(UCLIENT_PROFILE_CAN)
set(UCLIENT_BIG_ENDIANNESS)
set(UCLIENT_HARD_LIVELINESS_CHECK)
set(UCLIENT_TWEAK_XRCE_WRITE_LIMIT_FLAG    TRUE)
set(UCLIENT_PLATFORM_POSIX                 TRUE)
set(UCLIENT_PLATFORM_POSIX_NOPOLL)
set(UCLIENT_PLATFORM_WINDOWS)
set(UCLIENT_PLATFORM_FREERTOS_PLUS_TCP)
set(UCLIENT_PLATFORM_RTEMS_BSD_NET)
set(UCLIENT_PLATFORM_ZEPHYR)

configure_file("${_uxr_config_in}" "${_uxr_config_out}" @ONLY)

# micro-CDR config.h — same configure_file pattern, and the site issue 1069 was
# filed about. `ucdr/config.h.in` takes the SAME `@PROJECT_VERSION*@` tokens as
# the client template above, so leaving the client's values standing compiled
# `MICROCDR_VERSION_STR "2.4.1"` into a 2.0.2 tree. Re-point them EXPLICITLY at
# micro-CDR — not by reordering, which would leave the next `configure_file` to
# inherit whatever ran before it.
set(PROJECT_VERSION_MAJOR  ${_ucdr_VERSION_MAJOR})
set(PROJECT_VERSION_MINOR  ${_ucdr_VERSION_MINOR})
set(PROJECT_VERSION_PATCH  ${_ucdr_VERSION_PATCH})
set(PROJECT_VERSION        ${_ucdr_VERSION})
set(CONFIG_MACHINE_ENDIANNESS 1)  # ucdrEndianness: BIG=0, LITTLE=1 (x86 / ARM common case)
configure_file(
    "${MICROCDR_DIR}/include/ucdr/config.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ucdr_generated/ucdr/config.h"
    @ONLY)

# ---- Source list ------------------------------------------------------------
# Issue 1068 — this file OWNS NO SOURCE LIST. Every C file compiled into
# `nros_rmw_xrce` — the vendored micro-XRCE-DDS-Client and micro-CDR TUs AND
# the in-repo K.2 backend TUs — is read from
# `packages/rmw/xrce/xrce-sources.txt`, which
# `nros-rmw-xrce-cffi/build.rs` reads too.
#
# There used to be two hand-copied lists here and there, held together by a
# comment. They drifted, silently: `build.rs` honoured `NROS_LINK_IP=0`
# (phase-204.7 — the knob a serial-only node uses to shed the IP link) and
# dropped `profile/transport/ip/udp/udp_transport{,_posix}.c`, while this file
# compiled them unconditionally. So a serial-only XRCE node could not be built
# through the CMake lane, and the failure was not a build error but a larger
# image than the user asked for. The comment asserting the lockstep also named
# `packages/rmw/xrce/xrce-sys/build.rs`, a file phase-321 W1.d DELETED, so for
# two phases the invariant pointed at nothing. Correcting the list would have
# restored the mirror and left the next append to drift again; deriving it is
# the fix. Gate: `just check xrce-source-manifest`.
set(_xrce_manifest "${CMAKE_CURRENT_SOURCE_DIR}/../xrce-sources.txt")
if(NOT EXISTS "${_xrce_manifest}")
    message(FATAL_ERROR
        "nros-rmw-xrce: shared source manifest not found at '${_xrce_manifest}'. "
        "It is tracked in-repo — a missing one means the checkout is incomplete.")
endif()
# Re-configure when the shared list changes; without this a manifest edit would
# leave a configured build dir compiling yesterday's set.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_xrce_manifest}")

# NROS-XRCE-CONDITIONS-BEGIN — the boolean this lane supplies for each condition
# token the manifest uses. `check-xrce-source-manifest` asserts the Rust lane
# answers exactly this token set; the manifest, not this block, decides which
# files a token covers.
#
# `posix` is unconditionally TRUE here, and that is NOT the 1068 defect wearing
# another hat: this project is POSIX-only BY CONSTRUCTION. It FATAL_ERRORs
# without an `nros_platform_posix` target (below), defines
# `_POSIX_C_SOURCE=200809L` unconditionally, and configures
# `UCLIENT_PLATFORM_POSIX TRUE` with no fanout. There is no non-POSIX target for
# the two lanes to disagree about. It is spelled out anyway so the notion exists
# by name and a future cross-compiling CMake lane has one line to change.
set(_xrce_cond_always   TRUE)
set(_xrce_cond_posix    TRUE)
set(_xrce_cond_posix_ip ${_xrce_ip})
# NROS-XRCE-CONDITIONS-END

set(_xrce_tree_uxr     "${MICROXRCEDDS_CLIENT_DIR}/src/c")
set(_xrce_tree_ucdr    "${MICROCDR_DIR}/src/c")
set(_xrce_tree_backend "${CMAKE_CURRENT_SOURCE_DIR}/src")

# `group <name> <condition>` records → `_xrce_group_cond_<name>`.
file(STRINGS "${_xrce_manifest}" _xrce_group_lines REGEX "^[ \t]*group[ \t]")
foreach(_line IN LISTS _xrce_group_lines)
    string(REGEX REPLACE "^[ \t]*group[ \t]+([^ \t]+)[ \t]+([^ \t#]+).*$" "\\1;\\2" _g "${_line}")
    list(GET _g 0 _gname)
    list(GET _g 1 _gcond)
    if(NOT DEFINED _xrce_cond_${_gcond})
        message(FATAL_ERROR
            "nros-rmw-xrce: ${_xrce_manifest} group '${_gname}' uses condition token "
            "'${_gcond}', which this lane does not answer. Add a `set(_xrce_cond_${_gcond} …)` "
            "here AND an arm in nros-rmw-xrce-cffi/build.rs — a token only one lane answers "
            "is issue 1068 again.")
    endif()
    set(_xrce_group_cond_${_gname} "${_gcond}")
endforeach()

# `src <group> <tree> <path>` records → the target's source list.
set(_xrce_all_src "")
file(STRINGS "${_xrce_manifest}" _xrce_src_lines REGEX "^[ \t]*src[ \t]")
foreach(_line IN LISTS _xrce_src_lines)
    string(REGEX REPLACE "^[ \t]*src[ \t]+([^ \t]+)[ \t]+([^ \t]+)[ \t]+([^ \t#]+).*$"
           "\\1;\\2;\\3" _r "${_line}")
    list(GET _r 0 _rgroup)
    list(GET _r 1 _rtree)
    list(GET _r 2 _rpath)
    if(NOT DEFINED _xrce_group_cond_${_rgroup})
        message(FATAL_ERROR
            "nros-rmw-xrce: ${_xrce_manifest} source '${_rpath}' names group '${_rgroup}', "
            "which no `group` line declares.")
    endif()
    if(NOT DEFINED _xrce_tree_${_rtree})
        message(FATAL_ERROR
            "nros-rmw-xrce: ${_xrce_manifest} names unknown source tree '${_rtree}'.")
    endif()
    set(_rcond "${_xrce_group_cond_${_rgroup}}")
    if(_xrce_cond_${_rcond})
        list(APPEND _xrce_all_src "${_xrce_tree_${_rtree}}/${_rpath}")
    endif()
endforeach()
if(_xrce_all_src STREQUAL "")
    message(FATAL_ERROR
        "nros-rmw-xrce: ${_xrce_manifest} selected no sources — manifest or condition drift.")
endif()
message(STATUS
    "nano-ros: nros-rmw-xrce compiles sources from xrce-sources.txt (NROS_LINK_IP=${_xrce_ip})")

# ---- Platform implementation ----------------------------------------------
# Issue 0832 — the backend ALLOCATES through `nros_platform_alloc` (see the
# `nros_xrce_calloc`/`nros_xrce_free` helpers in src/internal.h), so headers
# alone are no longer enough to link. `nros-rmw-xrce-cffi`'s build.rs states
# the same contract for the image path: these sources compile on every target
# "as long as the consumer links a platform-provider library". This project
# was the one consumer that did not.
#
# Same convention as the two header vars above: the location arrives as a
# cache PATH defaulted to the sibling project, so an out-of-tree consumer can
# point it elsewhere and this file still never searches the tree. Guarded on
# the target not already existing, so a parent project that provides its own
# platform wins.
set(NROS_PLATFORM_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../platform/nros-platform-posix"
    CACHE PATH "Path to a project providing the nros platform ABI implementation")
if(NOT TARGET nros_platform_posix AND EXISTS "${NROS_PLATFORM_IMPL_DIR}/CMakeLists.txt")
    add_subdirectory("${NROS_PLATFORM_IMPL_DIR}" nros_platform_impl_build)
endif()

# ---- Static library --------------------------------------------------------
# Every TU — including this project's own `src/*.c` — comes from the shared
# manifest above (the `backend` tree). Nothing is named here, so nothing can be
# added here alone.
add_library(nros_rmw_xrce STATIC
    ${_xrce_all_src}
)

target_include_directories(nros_rmw_xrce
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        ${NROS_PLATFORM_CFFI_INCLUDE}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MICROXRCEDDS_CLIENT_DIR}/include
        ${MICROXRCEDDS_CLIENT_DIR}/src/c
        ${MICROCDR_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/uxr_generated
        ${CMAKE_CURRENT_BINARY_DIR}/ucdr_generated
)

# PUBLIC so consumers get the provider AFTER this archive on the link line:
# the references live in this library's own objects, and being a dependency is
# what orders it. Without it the smoke test fails with `undefined reference to
# nros_platform_alloc` from `transport_nros_udp.c.o` — the same shape as the
# cyclone funnel's link (issue 0832), and fixed the same way rather than with a
# weak fallback, which would resolve the reference by calling libc and put back
# exactly the second heap this change removes.
if(TARGET nros_platform_posix)
    target_link_libraries(nros_rmw_xrce PUBLIC nros_platform_posix)
    message(STATUS "nano-ros: nros-rmw-xrce allocates via nros_platform_alloc (issue 0832)")
else()
    message(FATAL_ERROR
        "nros-rmw-xrce needs a platform ABI implementation (nros_platform_alloc/dealloc). "
        "Set -DNROS_PLATFORM_IMPL_DIR=<path> or define an `nros_platform_posix` target.")
endif()

target_compile_definitions(nros_rmw_xrce PRIVATE
    # POSIX feature gate — `time.c` calls `clock_gettime`, which
    # only resolves under `_POSIX_C_SOURCE >= 199309L`. Match
    # the Rust lane's build environment.
    _POSIX_C_SOURCE=200809L
    _DEFAULT_SOURCE
)

target_compile_options(nros_rmw_xrce PRIVATE
    -Wall -Wextra -Wpedantic
    # Vendored upstream sources trip strictness our own code wants;
    # quiet just enough to keep the lint bar useful for our TUs.
    -Wno-unused-parameter
    -Wno-pedantic
)

set_target_properties(nros_rmw_xrce PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    EXPORT_NAME NrosRmwXrce
)

# Phase 140 — install rules + Config.cmake.in template deleted.
# `nros-rmw-xrce` (the K.2 header-only C++ backend) is no longer
# wired through the root CMake — XRCE consumption goes through the
# Rust-side `nros-rmw-xrce-cffi` shim, added via Corrosion in the
# root's xrce branch. This CMakeLists builds the C++ backend for
# in-tree CTest harnesses; it's not part of the user-facing surface.

# ---- Tests -----------------------------------------------------------------
option(NROS_RMW_XRCE_C_BUILD_TESTS "Build CTest harness" ON)
if(NROS_RMW_XRCE_C_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
