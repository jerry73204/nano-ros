# Issue 1015 -- `nros_assert_c_array_extent`, the configure-time floor on a
# number about to size a fixed C array. Included at FILE scope, ABOVE the
# function below, for the reason AGENTS.md gives at length: an `include()`
# evaluated inside a function frame loses the file's normal `set()`s when the
# frame pops (the `_NROS_ENTRY_DIR` pattern). Nothing here is a variable today,
# and putting the include where the rule says keeps that from mattering if it
# ever is.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosCArrayExtents.cmake")

function(nros_zephyr_configure_rmw_zenoh)
# -------------------------------------------------------------------------
# Zenoh-pico library (compiled from vendored submodule)
# -------------------------------------------------------------------------

# Vendored zenoh-pico submodule (single source of truth for all builds)
set(ZENOH_PICO_DIR ${NROS_REPO_DIR}/packages/rmw/zenoh/zpico-sys/zenoh-pico)

# --- zenoh-pico sources ---

file(GLOB_RECURSE _zenoh_pico_sources
    "${ZENOH_PICO_DIR}/src/api/*.c"
    "${ZENOH_PICO_DIR}/src/collections/*.c"
    "${ZENOH_PICO_DIR}/src/link/*.c"
    "${ZENOH_PICO_DIR}/src/net/*.c"
    "${ZENOH_PICO_DIR}/src/protocol/*.c"
    "${ZENOH_PICO_DIR}/src/session/*.c"
    "${ZENOH_PICO_DIR}/src/transport/*.c"
    "${ZENOH_PICO_DIR}/src/utils/*.c"
    "${ZENOH_PICO_DIR}/src/system/common/*.c"
)
# Zephyr platform: system.c (clock/memory/sleep/random/threading/time)
# is replaced by the alias TU (`packages/rmw/zenoh/zpico-sys/c/zpico/
# platform_aliases.c`) compiled inside the cargo-built Rust
# staticlib (`librustapp.a` / `libnros_c.a`). That TU forwards each
# `_z_*` to the canonical `nros_platform_*` ABI provided by
# `nros-platform-zephyr`. Phase 129 retired `zpico-platform-shim`;
# the alias TU is the single replacement provider.
#
# Phase 160.C — network.c (TCP/UDP/multicast) MUST come from
# zenoh-pico's `src/system/zephyr/network.c`, NOT from the alias
# TU. The alias TU's `_z_open_tcp` / `_z_send_tcp` / etc. see a
# generic 32-byte `_z_sys_net_socket_t` opaque (from
# `nros_zenoh_generic_platform.h`); the Zephyr-side `tx.c` /
# `link.c` (compiled here under `ZENOH_ZEPHYR`) see the
# 4-byte `{int _fd}` socket from `system/platform/zephyr.h`.
# The endpoint layouts diverge too (alias 16 B opaque vs.
# `{struct addrinfo*}` 8 B). The size mismatch propagates
# through the by-value endpoint arg and the by-pointer socket
# arg, corrupting the connect-time state → `Transport(
# ConnectionFailed)` on every Zephyr Rust app at session open.
# Same family as Phase 159 NuttX (`_z_send_tcp` ABI gate). The
# paired build.rs change extends `NROS_ZENOH_PLATFORM_USES_UNIX`
# to zephyr so the alias TU's network section is `#ifndef`-elided
# at cargo compile time — without that, both providers land and
# the alias version wins under `--allow-multiple-definition`.
zephyr_library_sources(${_zenoh_pico_sources}
    "${ZENOH_PICO_DIR}/src/system/zephyr/network.c")

# RFC-0083 — the ISO-TP platform binding. A separate TU from network.c, and
# added by name for the same reason network.c is: the `src/link/*.c` glob above
# picks up the link itself, but nothing under `src/system/zephyr/` is globbed.
if(CONFIG_NROS_ZENOH_LINK_ISOTP)
    zephyr_library_sources("${ZENOH_PICO_DIR}/src/system/zephyr/isotp.c")
endif()

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

# Issue 1015 -- the seven `-D`s below that become a FIXED C ARRAY EXTENT in
# `zpico.c` are checked here, at the point cmake commits a number to a compiler,
# rather than at any of the places the number could have come from.
#
# That seam is the point on purpose. These values have four upstreams that never
# see each other -- an environment override, a hand-set `.conf`, the entity
# inventory's derivation, and the literal-8 fallback immediately above -- and a
# check placed at any ONE of them binds only that one. phase-412 shipped six
# delivery-class defects in a single wave for exactly that reason. Checking the
# value that is about to be written binds all four, and any fifth.
#
# The reference island derived `MAX_QUERYABLES = 0` here and built a board that
# transmitted nothing for 15 s with every gate green. The derivation now floors
# its pools at 1, so this arm should be unreachable through THAT path; it is
# here because the other three paths are still open, and because "unreachable"
# is a claim about today's producers.
#
# NOT checked: FRAG_MAX_SIZE, BATCH_UNICAST_SIZE, GET_POLL_INTERVAL_MS. They are
# buffer/timing quantities, not array extents, and a floor asserted where the
# hazard does not exist is how a gate loses its meaning.
nros_assert_c_array_extent(
    MACRO ZPICO_MAX_PUBLISHERS
    VALUE "${NROS_RESOLVED_ZPICO_MAX_PUBLISHERS}"
    ARRAY "publisher_entry_t publishers[ZPICO_MAX_PUBLISHERS]"
    KCONFIG CONFIG_NROS_MAX_PUBLISHERS
    DECLARE "Derived from this image's entity inventory. A 0 means the \
inventory found no publishers: every component's nano_ros_node_register() must \
carry an ENTITIES list naming them, and a component that declares none makes \
the whole inventory refuse.")
nros_assert_c_array_extent(
    MACRO ZPICO_MAX_SUBSCRIBERS
    VALUE "${NROS_RESOLVED_ZPICO_MAX_SUBSCRIBERS}"
    ARRAY "subscriber_entry_t subscribers[ZPICO_MAX_SUBSCRIBERS]"
    KCONFIG CONFIG_NROS_MAX_SUBSCRIBERS
    DECLARE "Derived from this image's entity inventory. A 0 means the \
inventory found no subscriptions: every component's nano_ros_node_register() \
must carry an ENTITIES list naming them.")
nros_assert_c_array_extent(
    MACRO ZPICO_MAX_QUERYABLES
    VALUE "${NROS_RESOLVED_ZPICO_MAX_QUERYABLES}"
    ARRAY "queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]"
    KCONFIG CONFIG_NROS_MAX_QUERYABLES
    DECLARE "Derived from this image's entity inventory as \
`service servers + 3 * action servers`. A 0 means it found neither -- declare \
them in the ENTITIES list of nano_ros_node_register(), or state the knob. Note \
the parameter (6) and lifecycle (5) service families are NOT counted, because a \
feature enables them and the inventory cannot see it: an image carrying either \
must state the knob (issue 0460).")
nros_assert_c_array_extent(
    MACRO ZPICO_MAX_LIVELINESS
    VALUE "${NROS_RESOLVED_ZPICO_MAX_LIVELINESS}"
    ARRAY "liveliness_entry_t liveliness[ZPICO_MAX_LIVELINESS]"
    KCONFIG CONFIG_NROS_MAX_LIVELINESS)
nros_assert_c_array_extent(
    MACRO ZPICO_MAX_PENDING_GETS
    VALUE "${NROS_RESOLVED_ZPICO_MAX_PENDING_GETS}"
    ARRAY "pending_get_slot_t pending_gets[ZPICO_MAX_PENDING_GETS]"
    KCONFIG CONFIG_NROS_MAX_PENDING_GETS)
nros_assert_c_array_extent(
    MACRO ZPICO_GRAPH_CACHE_SIZE
    VALUE "${CONFIG_NROS_GRAPH_CACHE_SIZE}"
    ARRAY "uint8_t buf[ZPICO_GRAPH_CACHE_SIZE] in graph_cache_t"
    KCONFIG CONFIG_NROS_GRAPH_CACHE_SIZE)
nros_assert_c_array_extent(
    MACRO ZPICO_GET_REPLY_BUF_SIZE
    VALUE "${NROS_RESOLVED_ZPICO_GET_REPLY_BUF_SIZE}"
    ARRAY "uint8_t buf[ZPICO_GET_REPLY_BUF_SIZE] in get_reply_ctx_t"
    KCONFIG CONFIG_NROS_GET_REPLY_BUF_SIZE)

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
