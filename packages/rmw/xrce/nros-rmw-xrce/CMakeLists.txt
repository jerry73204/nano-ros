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

# ---- The vendored compile: NOT HERE ANY MORE (phase-420 W9 step 4) ---------
#
# The paragraph above described this project compiling the vendored TUs
# "directly into nros_rmw_xrce". It did — and so did
# `nros-rmw-xrce-cffi/build.rs`, from the same manifest, with different flags.
# Issues 1068 and 1069 collapsed the source list and the version; W9 steps 1-2
# collapsed the configuration; this step collapses the COMPILE. There is now one
# compile of micro-XRCE-DDS-Client and micro-CDR in the tree, and it is the
# cargo one — because that is the lane that SHIPS. `cmake/NanoRosRmwDispatch.cmake`
# maps `xrce` to `nros-rmw-xrce-cffi` for every image, `zephyr/cmake/nros_rmw_xrce.cmake`
# is an explicit no-op, and nothing in the tree `add_subdirectory()`s or
# `find_package()`s this project: its archive reached its own two CTest binaries
# and nothing else. So the CTests were validating objects no image contains.
# They now link the objects images link.
#
# Direction matters and was decided, not defaulted: making cargo shell out to
# cmake would put cmake plus a toolchain file in the path of every cross build,
# for six target families that cc-rs reaches today with neither.
#
# The archive is LOCATED, never globbed — `target/<profile>/build/
# nros-rmw-xrce-cffi-<hash>/out/` carries a hash that is neither stable nor
# predictable, and taking the first match of a glob is exactly issue 0500's
# defect. `just check rmw-xrce` reads `OUT_DIR` out of `cargo build
# --message-format=json`'s `build-script-executed` message and passes it here;
# the build script writes `nros-xrce-vendor-build.txt` into it naming the
# archive and the generated-header directory, so neither this file nor the
# recipe restates a cc-rs fact.
#
# There is deliberately NO fallback to compiling the sources here. A fallback
# would put the duplication back while every symptom said it was gone.
set(NROS_XRCE_CFFI_OUT_DIR "" CACHE PATH
    "OUT_DIR of the nros-rmw-xrce-cffi build script — the directory holding the \
vendored XRCE archive this project links.")
if(NOT NROS_XRCE_CFFI_OUT_DIR)
    message(FATAL_ERROR
        "nros-rmw-xrce: NROS_XRCE_CFFI_OUT_DIR is not set.\n"
        "This project no longer compiles the vendored micro-XRCE-DDS-Client / micro-CDR "
        "sources — it links the archive the cargo lane builds, so that the CTest harness "
        "exercises the objects images actually ship (phase-420 W9 step 4).\n"
        "Build it and configure with it:\n"
        "  just check rmw-xrce\n"
        "or, by hand:\n"
        "  OUT=$(cargo build -p nros-rmw-xrce-cffi --message-format=json \\\n"
        "        | python3 -c 'import sys,json;print([json.loads(l)[\"out_dir\"] "
        "for l in sys.stdin if \"build-script-executed\" in l "
        "and \"nros-rmw-xrce-cffi#\" in l][0])')\n"
        "  cmake -S packages/rmw/xrce/nros-rmw-xrce -B <build> -DNROS_XRCE_CFFI_OUT_DIR=\"$OUT\"")
endif()
set(_xrce_vendor_pointer "${NROS_XRCE_CFFI_OUT_DIR}/nros-xrce-vendor-build.txt")
if(NOT EXISTS "${_xrce_vendor_pointer}")
    message(FATAL_ERROR
        "nros-rmw-xrce: '${_xrce_vendor_pointer}' does not exist. NROS_XRCE_CFFI_OUT_DIR "
        "points at '${NROS_XRCE_CFFI_OUT_DIR}', which is not an OUT_DIR of a BUILT "
        "nros-rmw-xrce-cffi (its build script writes that file on every run). Run "
        "`cargo build -p nros-rmw-xrce-cffi` — or just `just check rmw-xrce`, which does "
        "both halves.")
endif()
# Re-configure when the cargo lane rebuilds: the pointer file is rewritten every
# run, so its timestamp is the freshness signal for the paths read out of it.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_xrce_vendor_pointer}")
set(NROS_XRCE_VENDOR_ARCHIVE "")
set(NROS_XRCE_CFFI_INCLUDE "")
file(STRINGS "${_xrce_vendor_pointer}" _xrce_vendor_lines REGEX "^[a-z]+=")
foreach(_line IN LISTS _xrce_vendor_lines)
    if(_line MATCHES "^archive=(.+)$")
        set(NROS_XRCE_VENDOR_ARCHIVE "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^include=(.+)$")
        set(NROS_XRCE_CFFI_INCLUDE "${CMAKE_MATCH_1}")
    endif()
endforeach()
foreach(_pair "NROS_XRCE_VENDOR_ARCHIVE;archive" "NROS_XRCE_CFFI_INCLUDE;include")
    list(GET _pair 0 _var)
    list(GET _pair 1 _key)
    if(NOT ${_var} OR NOT EXISTS "${${_var}}")
        message(FATAL_ERROR
            "nros-rmw-xrce: '${_xrce_vendor_pointer}' states no usable `${_key}=` "
            "('${${_var}}'). The cargo lane's build script writes both keys; a stale or "
            "truncated file means that build did not finish. Re-run "
            "`cargo build -p nros-rmw-xrce-cffi`.")
    endif()
endforeach()
message(STATUS
    "nano-ros: nros-rmw-xrce links the cargo lane's vendored archive "
    "${NROS_XRCE_VENDOR_ARCHIVE} (phase-420 W9 step 4 — one compile)")

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

# ---- Build configuration: DERIVED, never restated (phase-420 W9) -----------
#
# This file OWNS NO CONFIGURATION VALUE. Every `@TOKEN@` the two upstream
# `config.h.in` templates take, every `#cmakedefine` toggle, and every knob's
# minimum is read from `packages/rmw/xrce/xrce-config.txt`, which
# `nros-rmw-xrce-cffi/build.rs` reads too.
#
# There used to be a `set(UCLIENT_…)` block here and a matching pile of
# `.replace("@UCLIENT_…@", …)` calls there, with no gate between them, and they
# had already stopped agreeing about more than the version: the six
# `CONFIG_NROS_XRCE_*` Kconfig knobs reached the Rust lane and NOT this one
# (`grep -c CONFIG_NROS_XRCE` on this file was 0), so a person who set
# `CONFIG_NROS_XRCE_TRANSPORT_MTU=1024` got 1024 in one lane's TUs and 4096 in
# the other's. Measured, phase-420 W9: that is not an issue-0135 ABI split,
# because no single image holds TUs from both lanes (this project is configured
# only by `just check rmw-xrce` and its archive reaches only its own two CTest
# binaries) — it was a CTest harness validating values no image compiles.
#
# Gate: `just check xrce-config-manifest`.
set(_xrce_config "${CMAKE_CURRENT_SOURCE_DIR}/../xrce-config.txt")
if(NOT EXISTS "${_xrce_config}")
    message(FATAL_ERROR
        "nros-rmw-xrce: shared config manifest not found at '${_xrce_config}'. "
        "It is tracked in-repo — a missing one means the checkout is incomplete.")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_xrce_config}")

# The knob ladder, rungs 1, 2 and 4 — the twin of `KnobResolver` in
# `nros-rmw-xrce-cffi/build.rs`. Rung 3 (`[knobs.xrce]`) is that lane's only,
# and `xrce-config.txt` says why: it needs a TOML parser this lane does not
# have, and it can only be delivered by a cargo build.
#
# `_out` is set to the empty string when NOTHING states a value and `_default`
# is empty — that is how a `define` record leaves `internal.h`'s own `#ifndef`
# default standing, in both lanes, rather than each lane picking one.
function(_nros_xrce_knob _env _default _min _out)
    set(_v "")
    if(DEFINED ENV{${_env}} AND NOT "$ENV{${_env}}" STREQUAL "")
        # Rung 1. A person typed this in this shell; a value that is not a
        # number is an error, never a silent fall-through to a default that
        # answers a question they did not ask. build.rs panics on the same input.
        set(_v "$ENV{${_env}}")
        if(NOT _v MATCHES "^[0-9]+$")
            message(FATAL_ERROR "nros-rmw-xrce: ${_env}='${_v}' is not a number")
        endif()
    elseif(DEFINED ENV{DOTCONFIG} AND EXISTS "$ENV{DOTCONFIG}")
        # Rung 2 — Kconfig, read the way `nros_zephyr_build::dotconfig_usize`
        # reads it. A non-numeric value is ABSENT rather than an error: `-1` is
        # the documented DERIVE sentinel (phase-403 W8), so a knob left on it
        # means "nothing stated".
        file(STRINGS "$ENV{DOTCONFIG}" _hits REGEX "^CONFIG_${_env}=")
        if(_hits)
            list(GET _hits 0 _hit)
            string(REGEX REPLACE "^CONFIG_${_env}=" "" _hit "${_hit}")
            if(_hit MATCHES "^[0-9]+$")
                set(_v "${_hit}")
            endif()
        endif()
    endif()
    if("${_v}" STREQUAL "")
        set(_v "${_default}")  # rung 4
    endif()
    if(NOT "${_v}" STREQUAL "" AND _v LESS _min)
        message(FATAL_ERROR
            "nros-rmw-xrce: ${_env}=${_v} is below the minimum ${_min} stated in "
            "${_xrce_config}.")
    endif()
    set(${_out} "${_v}" PARENT_SCOPE)
endfunction()

# `value <template> <token> <literal>` and `knob <template> <token> <env>
# <default> <min>` → a cmake variable named for the token, which is what
# `configure_file @ONLY` substitutes. Both templates are filled from the same
# records; `_xrce_config_apply` is called once per template.
#
# `flag <template> <token> <condition>` → the same variable, TRUE or unset, so
# `#cmakedefine` writes `#define` or `/* #undef */`. The condition tokens are
# the ones `xrce-sources.txt` uses and the `_xrce_cond_*` block below answers —
# one vocabulary, because "which files" and "which profile defines" have to
# agree or the header promises a profile whose TUs were not compiled.
function(_xrce_config_apply _template)
    file(STRINGS "${_xrce_config}" _lines REGEX "^[ \t]*(value|knob|flag)[ \t]")
    foreach(_line IN LISTS _lines)
        string(REGEX REPLACE "[ \t]*#.*$" "" _line "${_line}")
        string(REGEX REPLACE "^[ \t]+" "" _line "${_line}")
        string(REGEX REPLACE "[ \t]+$" "" _line "${_line}")
        string(REGEX REPLACE "[ \t]+" ";" _f "${_line}")
        list(GET _f 0 _kind)
        list(GET _f 1 _tmpl)
        list(GET _f 2 _token)
        if(NOT _tmpl STREQUAL _template)
            continue()
        endif()
        if(_kind STREQUAL "value")
            list(GET _f 3 _lit)
            set(${_token} "${_lit}" PARENT_SCOPE)
        elseif(_kind STREQUAL "knob")
            list(GET _f 3 _env)
            list(GET _f 4 _def)
            list(GET _f 5 _min)
            _nros_xrce_knob("${_env}" "${_def}" "${_min}" _resolved)
            set(${_token} "${_resolved}" PARENT_SCOPE)
        else()
            list(GET _f 3 _cond)
            if(NOT DEFINED _xrce_cond_${_cond})
                message(FATAL_ERROR
                    "nros-rmw-xrce: ${_xrce_config} flag '${_token}' uses condition token "
                    "'${_cond}', which this lane does not answer. Add a "
                    "`set(_xrce_cond_${_cond} …)` here AND an arm in "
                    "nros-rmw-xrce-cffi/build.rs — a token only one lane answers is issue "
                    "1068 again.")
            endif()
            if(_xrce_cond_${_cond})
                set(${_token} TRUE PARENT_SCOPE)
            else()
                # UNSET, not empty-string: `#cmakedefine` treats both as false,
                # but a stale value from an earlier configure would not be.
                unset(${_token} PARENT_SCOPE)
            endif()
        endif()
    endforeach()
endfunction()

# NROS-XRCE-CONDITIONS-BEGIN — the boolean this lane supplies for each condition
# token the two manifests use. `check-xrce-source-manifest` asserts the Rust lane
# answers exactly this token set; a manifest, not this block, decides what a
# token covers.
#
# `posix` is unconditionally TRUE here, and that is NOT the 1068 defect wearing
# another hat: this project is POSIX-only BY CONSTRUCTION. It FATAL_ERRORs
# without an `nros_platform_posix` target (below), defines
# `_POSIX_C_SOURCE=200809L` unconditionally, and has no non-POSIX target for the
# two lanes to disagree about. It is spelled out anyway so the notion exists by
# name and a future cross-compiling CMake lane has one line to change.
set(_xrce_cond_always   TRUE)
set(_xrce_cond_posix    TRUE)
set(_xrce_cond_posix_ip ${_xrce_ip})
# phase-420 W9 — a toggle compiled off on every target. Stated rather than
# omitted, because an omitted `#cmakedefine` is `/* #undef */` here and an
# UNTOUCHED `#cmakedefine` line under build.rs's substitution: one silence, two
# different headers.
set(_xrce_cond_never    FALSE)
# NROS-XRCE-CONDITIONS-END

# Generate `<uxr/client/config.h>` from the upstream template.
set(_uxr_config_in   "${MICROXRCEDDS_CLIENT_DIR}/include/uxr/client/config.h.in")
set(_uxr_config_out  "${CMAKE_CURRENT_BINARY_DIR}/uxr_generated/uxr/client/config.h")

# `PROJECT_VERSION*` is a SHARED name: both templates take it, so it must be
# re-pointed at the right tree before each `configure_file` and never left
# standing. That is the 1069 bug — see the micro-CDR block below.
set(PROJECT_VERSION_MAJOR  ${_uxr_VERSION_MAJOR})
set(PROJECT_VERSION_MINOR  ${_uxr_VERSION_MINOR})
set(PROJECT_VERSION_PATCH  ${_uxr_VERSION_PATCH})
set(PROJECT_VERSION        ${_uxr_VERSION})

_xrce_config_apply(uxr)
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
_xrce_config_apply(ucdr)
configure_file(
    "${MICROCDR_DIR}/include/ucdr/config.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ucdr_generated/ucdr/config.h"
    @ONLY)

# ---- The two lanes' headers must be the SAME BYTES (phase-420 W9 step 4) ----
#
# W9 steps 1-2 gave the two lanes one statement of every value and two
# implementations of the ladder that resolves it, and verified BY HAND, once,
# that the generated headers matched. Step 4 makes that verification
# load-bearing rather than historical: this project now compiles its own C
# against the headers above and LINKS vendored objects the cargo lane compiled
# against ITS headers, in one binary. If the two disagree about
# `UXR_CONFIG_UDP_TRANSPORT_MTU`, `UCLIENT_PROFILE_*` or the stream history,
# that is an issue-0135 ABI split — flag-gated struct layouts disagreeing inside
# a single link — which the W9 measurement said could not happen only BECAUSE no
# single image held TUs from both lanes. It does now, so the property gets a
# check instead of an argument.
#
# Cheap (two file compares at configure time) and LOUD: the message names both
# files and the diff command, because "which value differs" is the first
# question and a byte count does not answer it.
foreach(_pair
        "uxr_generated/uxr/client/config.h;uxr/client/config.h"
        "ucdr_generated/ucdr/config.h;ucdr/config.h")
    list(GET _pair 0 _ours)
    list(GET _pair 1 _theirs)
    set(_ours "${CMAKE_CURRENT_BINARY_DIR}/${_ours}")
    set(_theirs "${NROS_XRCE_CFFI_INCLUDE}/${_theirs}")
    if(NOT EXISTS "${_theirs}")
        message(FATAL_ERROR
            "nros-rmw-xrce: the cargo lane's generated header '${_theirs}' is missing, so "
            "the archive this project links cannot be checked against the headers it "
            "compiles. Re-run `cargo build -p nros-rmw-xrce-cffi`.")
    endif()
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E compare_files "${_ours}" "${_theirs}"
        RESULT_VARIABLE _differ OUTPUT_QUIET ERROR_QUIET)
    if(NOT _differ EQUAL 0)
        message(FATAL_ERROR
            "nros-rmw-xrce: the two lanes generated DIFFERENT configuration headers.\n"
            "  this lane : ${_ours}\n"
            "  cargo lane: ${_theirs}\n"
            "  diff -u '${_theirs}' '${_ours}'\n"
            "This project compiles its own TUs against the first and links vendored "
            "objects built against the second, so a difference is an issue-0135 ABI "
            "split inside one binary, not a cosmetic one. Usual cause: the two builds "
            "saw different knob inputs (a `NROS_XRCE_*` env var or `$DOTCONFIG` set for "
            "one and not the other) — build both from the same shell, which is what "
            "`just check rmw-xrce` does.")
    endif()
endforeach()

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

# The `_xrce_cond_*` block that answers these lives with the config manifest
# above — ONE vocabulary for both manifests, because "which files" and "which
# profile defines" have to agree.

# NROS-XRCE-COMPILED-TREES-BEGIN
# The `xrce-sources.txt` trees THIS lane compiles — phase-420 W9 step 4.
#
# `backend` only. The `uxr` and `ucdr` rows arrive as OBJECTS, out of the
# archive `nros-rmw-xrce-cffi/build.rs` builds from the same manifest rows; its
# `COMPILED_TREES` names all three, because that archive is what both lanes
# link. One vendored compile in the tree, and it is the shipping one.
#
# This is the FILTER, not a note about one: the loop below skips a row whose
# tree is not in this list, so adding `uxr` here is what it would take to
# compile the vendored sources twice again — and `check-xrce-one-vendored-compile`
# fails the moment anyone does.
#
# The backend TUs stay: `just check rmw-xrce` is the ONLY place
# `nros-rmw-xrce/src/*.c` is compiled with warnings on (the cargo lane sets
# `warnings(false)` because the vendored TUs are noisy), which is the diagnostic
# issue 0787 created this recipe to get. They are compiled here and NOT linked
# into the CTest binaries — those link the archive, so what runs is what ships.
set(_xrce_compiled_trees backend)
# NROS-XRCE-COMPILED-TREES-END

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
set(_xrce_linked_src 0)
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
    if(NOT _rtree IN_LIST _xrce_compiled_trees)
        # Compiled by the cargo lane; this project LINKS the resulting objects.
        # Not an error and not a silent drop — the count is reported below.
        math(EXPR _xrce_linked_src "${_xrce_linked_src} + 1")
        continue()
    endif()
    if(NOT DEFINED _xrce_tree_${_rtree})
        message(FATAL_ERROR
            "nros-rmw-xrce: ${_xrce_manifest} names source tree '${_rtree}', which "
            "`_xrce_compiled_trees` lists but no `_xrce_tree_${_rtree}` locates.")
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
    "nano-ros: nros-rmw-xrce compiles ${_xrce_compiled_trees} sources from xrce-sources.txt "
    "and LINKS ${_xrce_linked_src} vendored objects from the cargo lane "
    "(NROS_LINK_IP=${_xrce_ip})")

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
# ---- What this project produces (phase-420 W9 step 4) ----------------------
#
# TWO targets, and the split is the point:
#
#   `nros_rmw_xrce`          — an INTERFACE library over the CARGO LANE'S
#                              ARCHIVE. This is what the CTest binaries link,
#                              so what they exercise is byte-for-byte what an
#                              image ships. It used to be a STATIC library
#                              built here from the same sources with different
#                              flags, which meant the harness validated objects
#                              no image contains.
#
#   `nros_rmw_xrce_warnings` — an OBJECT library over this project's own
#                              `src/*.c`, compiled with `-Wall -Wextra`. It is
#                              linked by NOTHING, on purpose: its whole output
#                              is diagnostics. `just check rmw-xrce` is the only
#                              place these TUs are compiled with warnings on —
#                              the cargo lane sets `warnings(false)` because the
#                              vendored TUs beside them are noisy — and issue
#                              0787 created this recipe to get exactly that.
#                              Deleting it would drop a diagnostic silently,
#                              which is worse than the duplication it costs
#                              (10 files, ~1 s).
add_library(nros_rmw_xrce_vendored STATIC IMPORTED GLOBAL)
set_target_properties(nros_rmw_xrce_vendored PROPERTIES
    IMPORTED_LOCATION "${NROS_XRCE_VENDOR_ARCHIVE}")
# An IMPORTED target's location becomes a link DEPENDENCY of every consumer, so
# a cargo rebuild relinks the tests. A raw `-Wl,<path>` would not — that is
# issue 0475's missing edge, and the reason this is a target rather than a flag.
target_include_directories(nros_rmw_xrce_vendored INTERFACE
    # The generated `<uxr/client/config.h>` / `<ucdr/config.h>` THESE OBJECTS
    # were compiled against — not this project's copies, which exist only for
    # the warnings target and are asserted identical above.
    "${NROS_XRCE_CFFI_INCLUDE}"
    "${MICROXRCEDDS_CLIENT_DIR}/include"
    "${MICROXRCEDDS_CLIENT_DIR}/src/c"
    "${MICROCDR_DIR}/include"
)

add_library(nros_rmw_xrce INTERFACE)
target_include_directories(nros_rmw_xrce INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${NROS_RMW_CFFI_DIR}
    ${NROS_PLATFORM_CFFI_INCLUDE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# The provider comes AFTER the archive on the link line: the references live in
# the archive's objects, and being a dependency is what orders it. Without it
# the smoke test fails with `undefined reference to nros_platform_alloc` from
# `transport_nros_udp.c.o` — the same shape as the cyclone funnel's link (issue
# 0832), and fixed the same way rather than with a weak fallback, which would
# resolve the reference by calling libc and put back exactly the second heap
# that change removed.
if(TARGET nros_platform_posix)
    target_link_libraries(nros_rmw_xrce INTERFACE
        nros_rmw_xrce_vendored
        nros_platform_posix)
    message(STATUS "nano-ros: nros-rmw-xrce allocates via nros_platform_alloc (issue 0832)")
else()
    message(FATAL_ERROR
        "nros-rmw-xrce needs a platform ABI implementation (nros_platform_alloc/dealloc). "
        "Set -DNROS_PLATFORM_IMPL_DIR=<path> or define an `nros_platform_posix` target.")
endif()

# ---- The warnings-only compile of this project's own C ---------------------
add_library(nros_rmw_xrce_warnings OBJECT
    ${_xrce_all_src}
)

target_include_directories(nros_rmw_xrce_warnings PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${NROS_RMW_CFFI_DIR}
    ${NROS_PLATFORM_CFFI_INCLUDE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${MICROXRCEDDS_CLIENT_DIR}/include
    ${MICROXRCEDDS_CLIENT_DIR}/src/c
    ${MICROCDR_DIR}/include
    # THIS lane's generated headers, deliberately: compiling against them is
    # what proves the ladder in this file still resolves, and the byte-compare
    # above is what proves the result agrees with the archive.
    ${CMAKE_CURRENT_BINARY_DIR}/uxr_generated
    ${CMAKE_CURRENT_BINARY_DIR}/ucdr_generated
)

target_compile_definitions(nros_rmw_xrce_warnings PRIVATE
    # POSIX feature gate — `time.c` calls `clock_gettime`, which
    # only resolves under `_POSIX_C_SOURCE >= 199309L`. Match
    # the Rust lane's build environment.
    _POSIX_C_SOURCE=200809L
    _DEFAULT_SOURCE
)

# phase-420 W9 — the backend's `-D` pool knobs, from the shared manifest's
# `define` records. This lane used to set NONE of them, so
# `CONFIG_NROS_XRCE_MAX_SUBSCRIBERS` and its four siblings shrank the session
# struct on the Rust lane and did nothing here. A `define` row states no
# default: nothing stated ⇒ nothing defined ⇒ `src/internal.h`'s own `#ifndef`
# default stands, identically in both lanes.
file(STRINGS "${_xrce_config}" _xrce_define_lines REGEX "^[ \t]*define[ \t]")
foreach(_line IN LISTS _xrce_define_lines)
    string(REGEX REPLACE "[ \t]*#.*$" "" _line "${_line}")
    string(REGEX REPLACE "^[ \t]+" "" _line "${_line}")
    string(REGEX REPLACE "[ \t]+$" "" _line "${_line}")
    string(REGEX REPLACE "[ \t]+" ";" _f "${_line}")
    list(GET _f 1 _macro)
    list(GET _f 2 _env)
    list(GET _f 3 _min)
    _nros_xrce_knob("${_env}" "" "${_min}" _resolved)
    if(NOT "${_resolved}" STREQUAL "")
        target_compile_definitions(nros_rmw_xrce_warnings PRIVATE "${_macro}=${_resolved}")
        message(STATUS "nano-ros: nros-rmw-xrce ${_macro}=${_resolved} (from ${_env})")
    endif()
endforeach()

target_compile_options(nros_rmw_xrce_warnings PRIVATE
    -Wall -Wextra -Wpedantic
    # `-Wno-unused-parameter` and `-Wno-pedantic` used to sit here because the
    # VENDORED TUs shared this target and tripped both. They no longer do — this
    # target compiles only `nros-rmw-xrce/src/*.c` — so the suppressions are
    # gone and the bar over our own C is the full one. Measured clean at the
    # time this landed (phase-420 W9 step 4).
)

set_target_properties(nros_rmw_xrce_warnings PROPERTIES
    POSITION_INDEPENDENT_CODE ON
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
