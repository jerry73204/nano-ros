function(nros_zephyr_configure_rmw_zenoh)
# -------------------------------------------------------------------------
# Zenoh-pico library (compiled from vendored submodule)
# -------------------------------------------------------------------------

# Vendored zenoh-pico submodule (single source of truth for all builds)
set(ZENOH_PICO_DIR ${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-sys/zenoh-pico)

# --- zenoh-pico sources ---
#
# phase-420 W9 — THIS FILE OWNS NO SOURCE LIST. The set of vendored zenoh-pico
# TUs compiled here is read from
# `packages/rmw/zenoh/zpico-sys/zenoh-sources.txt`, which `nros-zpico-build`
# (the cargo lane, `add_zenoh_pico_core_sources`) reads too.
#
# There used to be two copies of that selection — nine directory globs here and
# the same nine subdirectory names in `nros-zpico-build/src/lib.rs` — with
# nothing checking that they agreed. That is the defect class issue 1068 fixed
# for micro-XRCE next door: a hand-mirrored thing drifts on append, so derive
# it once instead. The per-platform divergence that IS real (Zephyr compiles
# `system/zephyr/network.c` from the tree while every other platform gets those
# entry points from the alias TU) is carried in the manifest as a named
# condition, with the ABI reasoning; see the `zephyr_system` group there.
#
# Gate: `just check zenoh-source-manifest`.
set(_zenoh_manifest "${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-sys/zenoh-sources.txt")
if(NOT EXISTS "${_zenoh_manifest}")
    message(FATAL_ERROR
        "nros_rmw_zenoh: shared source manifest not found at '${_zenoh_manifest}'. "
        "It is tracked in-repo — a missing one means the checkout is incomplete.")
endif()
# Re-configure when the shared list changes; without this a manifest edit would
# leave a configured build dir compiling yesterday's set.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_zenoh_manifest}")

# NROS-ZENOH-CONDITIONS-BEGIN — the boolean this lane supplies for each
# condition token the manifest uses. `check-zenoh-source-manifest` asserts the
# cargo lane answers exactly this token set; the manifest, not this block,
# decides which sources a token covers.
set(_zenoh_cond_always TRUE)
# This lane IS the Zephyr lane — it is reached only from a west build, so the
# platform test the cargo lane makes is a constant here. The cargo lane answers
# FALSE (Zephyr is `compiled_by = "platform"`, issue 0541).
set(_zenoh_cond_zephyr TRUE)
# RFC-0083 — the ISO-TP platform binding rides on the link Kconfig.
if(CONFIG_NROS_ZENOH_LINK_ISOTP)
    set(_zenoh_cond_zephyr_isotp TRUE)
else()
    set(_zenoh_cond_zephyr_isotp FALSE)
endif()
# NROS-ZENOH-CONDITIONS-END

set(_zenoh_tree_zenoh_pico "${ZENOH_PICO_DIR}/src")

# `group <name> <condition>` records → `_zenoh_group_cond_<name>`.
file(STRINGS "${_zenoh_manifest}" _zenoh_group_lines REGEX "^[ \t]*group[ \t]")
foreach(_line IN LISTS _zenoh_group_lines)
    string(REGEX REPLACE "^[ \t]*group[ \t]+([^ \t]+)[ \t]+([^ \t#]+).*$" "\\1;\\2" _g "${_line}")
    list(GET _g 0 _gname)
    list(GET _g 1 _gcond)
    if(NOT DEFINED _zenoh_cond_${_gcond})
        message(FATAL_ERROR
            "nros_rmw_zenoh: ${_zenoh_manifest} group '${_gname}' uses condition token "
            "'${_gcond}', which this lane does not answer. Add a `set(_zenoh_cond_${_gcond} …)` "
            "here AND an arm in nros-zpico-build's add_zenoh_pico_core_sources — a token only "
            "one lane answers is the same defect one conditional over.")
    endif()
    set(_zenoh_group_cond_${_gname} "${_gcond}")
endforeach()

# `dir <group> <tree> <path>` / `src <group> <tree> <path>` records → the
# library's source list.
#
# `CONFIGURE_DEPENDS` on the `dir` expansion, which the hand-written globs it
# replaces did NOT carry: without it a `.c` added under one of these
# directories — by an upstream rebase of the `nano-ros` patch line, say — gets
# no rebuild edge at all, and the build keeps linking yesterday's set with no
# diagnostic. Same shape as issue 0475 one layer over. It costs a glob per
# build; the alternative costs a silently wrong image.
set(_zenoh_pico_sources "")
file(STRINGS "${_zenoh_manifest}" _zenoh_src_lines REGEX "^[ \t]*(dir|src)[ \t]")
foreach(_line IN LISTS _zenoh_src_lines)
    string(REGEX REPLACE "^[ \t]*(dir|src)[ \t]+([^ \t]+)[ \t]+([^ \t]+)[ \t]+([^ \t#]+).*$"
           "\\1;\\2;\\3;\\4" _r "${_line}")
    list(GET _r 0 _rkind)
    list(GET _r 1 _rgroup)
    list(GET _r 2 _rtree)
    list(GET _r 3 _rpath)
    if(NOT DEFINED _zenoh_group_cond_${_rgroup})
        message(FATAL_ERROR
            "nros_rmw_zenoh: ${_zenoh_manifest} record '${_rpath}' names group '${_rgroup}', "
            "which no `group` line declares.")
    endif()
    if(NOT DEFINED _zenoh_tree_${_rtree})
        message(FATAL_ERROR
            "nros_rmw_zenoh: ${_zenoh_manifest} names unknown source tree '${_rtree}'.")
    endif()
    set(_cond "${_zenoh_group_cond_${_rgroup}}")
    if(NOT _zenoh_cond_${_cond})
        continue()
    endif()
    set(_root "${_zenoh_tree_${_rtree}}")
    if(_rkind STREQUAL "dir")
        if(NOT IS_DIRECTORY "${_root}/${_rpath}")
            message(FATAL_ERROR
                "nros_rmw_zenoh: ${_zenoh_manifest} lists directory '${_rtree}/${_rpath}', "
                "which is not a directory at ${_root}/${_rpath}. An upstream bump that renamed "
                "or removed it needs the manifest updated, not this file.")
        endif()
        file(GLOB_RECURSE _expanded CONFIGURE_DEPENDS "${_root}/${_rpath}/*.c")
        list(APPEND _zenoh_pico_sources ${_expanded})
    else()
        if(NOT EXISTS "${_root}/${_rpath}")
            message(FATAL_ERROR
                "nros_rmw_zenoh: ${_zenoh_manifest} lists source '${_rtree}/${_rpath}', which "
                "does not exist at ${_root}/${_rpath}.")
        endif()
        list(APPEND _zenoh_pico_sources "${_root}/${_rpath}")
    endif()
endforeach()
if(NOT _zenoh_pico_sources)
    message(FATAL_ERROR
        "nros_rmw_zenoh: ${_zenoh_manifest} selected no sources — manifest or condition drift.")
endif()

zephyr_library_sources(${_zenoh_pico_sources})

# zenoh-pico include directory
zephyr_include_directories(${ZENOH_PICO_DIR}/include)

# --- zenoh-pico compile definitions ---

# Zephyr platform backend
zephyr_compile_definitions(ZENOH_ZEPHYR)

# Opt-in zenoh-pico internal logging. -DNROS_ZENOH_DEBUG=<1..4> on the west
# command line; nothing changes without it. zenoh-pico's own CMakeLists sets
# ZENOH_DEBUG, but nano-ros compiles those sources itself, so that path never
# reaches this build and there was no way to see why a declaration failed.
if(DEFINED NROS_ZENOH_DEBUG AND NOT NROS_ZENOH_DEBUG STREQUAL "")
    zephyr_compile_definitions(ZENOH_DEBUG=${NROS_ZENOH_DEBUG})
    message(STATUS "nano-ros: zenoh-pico logging at ZENOH_DEBUG=${NROS_ZENOH_DEBUG}")
endif()

# Router-backed client-to-client routing requires interest declarations
# so zenohd knows which peers should receive each keyexpr. Keep matching
# callbacks disabled on Zephyr; they are not needed for routing and can
# create high-rate executor wakeups.
zephyr_compile_definitions(Z_FEATURE_INTEREST=1 Z_FEATURE_MATCHING=0)

# zsock serializes send/recv on a per-fd mutex, so total tx throughput is
# capped at ~one send per recv window — make the window Kconfig-tunable
# (issues 0129/0139; the vendored config.h default is #ifndef-guarded).
if(CONFIG_NROS_ZENOH_SOCKET_TIMEOUT_MS)
    zephyr_compile_definitions(Z_CONFIG_SOCKET_TIMEOUT=${CONFIG_NROS_ZENOH_SOCKET_TIMEOUT_MS})
endif()

# The transport lease. Both tokens are #ifndef-guarded in the vendored
# config.h for exactly this reason — upstream only exposes them as CMake cache
# variables, which needs zenoh-pico's own CMakeLists, and the Zephyr module
# compiles the sources directly. See NROS_ZENOH_LEASE_MS's help for why a
# serial link cares.
if(CONFIG_NROS_ZENOH_LEASE_MS)
    zephyr_compile_definitions(Z_TRANSPORT_LEASE=${CONFIG_NROS_ZENOH_LEASE_MS})
endif()


if(CONFIG_NROS_ZENOH_LEASE_FACTOR)
    zephyr_compile_definitions(Z_TRANSPORT_LEASE_EXPIRE_FACTOR=${CONFIG_NROS_ZENOH_LEASE_FACTOR})
endif()

# phase-279 (#145) / phase-290 (RFC-0049) — tx batching: one send per executor
# spin instead of one send per put. TRI-STATE forward: the definition is
# ALWAYS emitted (0 or 1) so an explicit Kconfig `n` can override an
# on-default anywhere below it, and every TU sees the same definite value.
if(CONFIG_NROS_ZENOH_TX_BATCH)
    zephyr_compile_definitions(ZPICO_TX_BATCH=1)
    # phase-282 (#145) — flush cadence: period of the dedicated tx-flush
    # thread / rate limit of the spin-driven fallback flush.
    if(CONFIG_NROS_ZENOH_TX_BATCH_FLUSH_MS)
        zephyr_compile_definitions(
            ZPICO_TX_BATCH_FLUSH_MS=${CONFIG_NROS_ZENOH_TX_BATCH_FLUSH_MS})
    endif()
else()
    zephyr_compile_definitions(ZPICO_TX_BATCH=0)
endif()

# phase-282 (#145) — split tx locking (steal batch under tx mutex, send under a
# link-write mutex). Gates transport-struct fields: applied to ALL zephyr TUs
# (issue-0135 ABI rule) — always defined, 0 or 1.
if(CONFIG_NROS_ZENOH_TX_SPLIT_LOCK)
    zephyr_compile_definitions(Z_FEATURE_TX_SPLIT_LOCK=1)
else()
    zephyr_compile_definitions(Z_FEATURE_TX_SPLIT_LOCK=0)
endif()

# Intra-image topic delivery (RFC-0015 Model 1): every node in the image
# shares ONE zenoh session, and neither zenoh-pico nor the router loops a
# publication back to the session it came from. Without local subscriber
# dispatch a same-image pub→sub pair (e.g. ws-qos-rust's reliable_talker →
# qos_listener) silently never delivers. LOCAL_SUBSCRIBER routes each put
# to matching subscribers on the local session in addition to the wire.
zephyr_compile_definitions(Z_FEATURE_LOCAL_SUBSCRIBER=1)

# Map NROS_ZENOH_* Kconfig options to Z_FEATURE_* compile definitions.
# The function strips the CONFIG_NROS_ZENOH_ prefix and replaces it with
# Z_FEATURE_, then sets =1 or =0 based on the Kconfig boolean value.
function(_nros_configure_zenoh_feature config)
    string(REPLACE "CONFIG_NROS_ZENOH_" "Z_FEATURE_" feature "${config}")
    if(${config})
        zephyr_compile_definitions(${feature}=1)
    else()
        zephyr_compile_definitions(${feature}=0)
    endif()
endfunction()

_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_MULTI_THREAD)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_PUBLICATION)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_SUBSCRIPTION)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_QUERY)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_QUERYABLE)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_TCP)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_UDP_UNICAST)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_UDP_MULTICAST)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_SCOUTING)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_SERIAL)
# RFC-0080 — CONFIG_NROS_ZENOH_LINK_CAN -> Z_FEATURE_LINK_CAN.
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_CAN)
# RFC-0083 — CONFIG_NROS_ZENOH_LINK_ISOTP -> Z_FEATURE_LINK_ISOTP.
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_ISOTP)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_LINK_WS)
# issue 0882 — CONFIG_NROS_ZENOH_AUTO_RECONNECT -> Z_FEATURE_AUTO_RECONNECT.
# zenoh-pico defaults it ON and there was no way to say otherwise from an image.
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_AUTO_RECONNECT)
_nros_configure_zenoh_feature(CONFIG_NROS_ZENOH_RAWETH_TRANSPORT)

# -------------------------------------------------------------------------
# Shared sources: zenoh shim + zpico-zephyr platform support
# -------------------------------------------------------------------------

# zpico.c — the C API layer over zenoh-pico
zephyr_library_sources(
    ${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c
)
zephyr_library_sources(${NROS_ZEPHYR_DIR}/nros_zenoh_zephyr_system.c)

# zpico_zephyr.c — Zephyr platform support (network wait, session init)
zephyr_library_sources(
    ${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-zephyr/src/zpico_zephyr.c
)

# Include directories for zpico and platform headers
zephyr_include_directories(${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-sys/c/include)
zephyr_include_directories(${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-zephyr/include)

# -------------------------------------------------------------------------
# Transport tuning: Kconfig → C preprocessor flags
# -------------------------------------------------------------------------

# Map transport tuning to ZPICO_* defines consumed by zpico.c.
#
# These read NROS_RESOLVED_* rather than CONFIG_* so the C TUs get exactly the
# value the cargo build got. Reading CONFIG_* here would reintroduce issue 0135:
# an environment override would reach the Rust side only, and the two would
# disagree about a struct's size with no diagnostic (issue 0316).
# phase-412 W1 -- a C define has NO default of its own, unlike a Rust build
# script. `_nros_resolve_derivable_knob`'s rung 4 deliberately leaves a knob
# UNRESOLVED so the reading build script falls to its own literal
# (`env_usize("ZPICO_MAX_SUBSCRIBERS", 8)`), which is the one place that literal
# is written. That contract does not reach here: an unresolved knob expands to
# nothing, `-DZPICO_MAX_SUBSCRIBERS=` reaches the compiler, and zpico.c reports
#     error: flexible array member not at end of struct
# on a struct nobody edited.
#
# It only bites since these knobs gained the `-1` DERIVE sentinel: before that
# Kconfig always carried a number. So the sentinel is safe only where the
# consumer supplies a default, and this consumer is the exception.
foreach(_zp MAX_PUBLISHERS MAX_SUBSCRIBERS MAX_QUERYABLES)
    if(NOT DEFINED NROS_RESOLVED_ZPICO_${_zp} OR
       "${NROS_RESOLVED_ZPICO_${_zp}}" STREQUAL "")
        set(NROS_RESOLVED_ZPICO_${_zp} 8)
        message(STATUS
            "nros: ZPICO_${_zp} left at the zpico literal default 8 -- nothing "
            "stated it and the entity inventory derived nothing (a first "
            "configure on a clean build dir always lands here, issue 0991)")
    endif()
endforeach()

zephyr_compile_definitions(
    ZPICO_MAX_PUBLISHERS=${NROS_RESOLVED_ZPICO_MAX_PUBLISHERS}
    ZPICO_MAX_SUBSCRIBERS=${NROS_RESOLVED_ZPICO_MAX_SUBSCRIBERS}
    ZPICO_MAX_QUERYABLES=${NROS_RESOLVED_ZPICO_MAX_QUERYABLES}
    ZPICO_MAX_LIVELINESS=${NROS_RESOLVED_ZPICO_MAX_LIVELINESS}
    ZPICO_MAX_PENDING_GETS=${NROS_RESOLVED_ZPICO_MAX_PENDING_GETS}
)
# Read straight from CONFIG_, unlike the block above, and deliberately: the
# graph cache is allocated and consumed entirely inside zpico.c. The Rust side
# copies it out through a caller-supplied buffer and never encodes its size, so
# there is no second consumer to disagree with -- the issue-0135/0316 hazard the
# NROS_RESOLVED_* indirection exists to prevent does not apply here.
zephyr_compile_definitions(
    ZPICO_GRAPH_CACHE_SIZE=${CONFIG_NROS_GRAPH_CACHE_SIZE}
    ZPICO_GET_REPLY_BUF_SIZE=${NROS_RESOLVED_ZPICO_GET_REPLY_BUF_SIZE}
    ZPICO_GET_POLL_INTERVAL_MS=${NROS_RESOLVED_ZPICO_GET_POLL_INTERVAL_MS}
    ZPICO_FRAG_MAX_SIZE=${NROS_RESOLVED_ZPICO_FRAG_MAX_SIZE}
    ZPICO_BATCH_UNICAST_SIZE=${NROS_RESOLVED_ZPICO_BATCH_UNICAST_SIZE}
)

# Issue 0626 — the read/lease task priority, normalized 0-31.
#
# Read from CONFIG_* directly, NOT through `_nros_resolve_knob`, and that is
# deliberate. The resolver exists for knobs with TWO consumers — a `build.rs`
# env var and a C define — that must agree or a struct's size splits between
# the Rust and C halves (issues 0135 / 0316). `check-kconfig-knob-forwarding`
# enforces exactly that pairing, and correctly rejected these two when they
# were routed through the resolver: no Rust build script reads them, because
# they gate no layout. One consumer, one path.
#
# `DEFINED`, not truthiness: 0 is a legal priority on this band (the least
# urgent), and a bare `if(CONFIG_...)` would silently drop it — the same
# "declared value discarded" failure this issue exists to fix.
if(DEFINED CONFIG_NROS_ZENOH_READ_PRIORITY)
    zephyr_compile_definitions(ZPICO_READ_TASK_PRIORITY=${CONFIG_NROS_ZENOH_READ_PRIORITY})
endif()
if(DEFINED CONFIG_NROS_ZENOH_LEASE_PRIORITY)
    zephyr_compile_definitions(ZPICO_LEASE_TASK_PRIORITY=${CONFIG_NROS_ZENOH_LEASE_PRIORITY})
endif()

endfunction()
