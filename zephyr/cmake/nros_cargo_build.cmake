# nros Cargo Build Helpers for Zephyr
# Copyright (c) 2024 nros contributors
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Provides CMake functions for building Rust crates from the nros workspace
# and bridging Kconfig values to Cargo environment variables.

# phase-336 — the shared cargo-profile resolver (`nros profile`), included at
# FILE scope so a function body never include()s inside its own frame.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosCargoProfile.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosBoardFacts.cmake")
# `nros_host_rust_triple` — native_sim's Rust target is the HOST's (issue 0582).
# FILE scope for the same reason as the two above: an include() inside a
# function frame drops the file's vars when the frame pops.
include("${CMAKE_CURRENT_LIST_DIR}/../../packages/api/nros-c/cmake/nros-rtos-helpers.cmake")
# phase-403 W8 (issue 0940) -- `nros_message_bounds_knobs_file`, the ONE path the
# derived size knobs are written to and read from. FILE scope for the same
# reason, and here rather than via NanoRosCodegenCore.cmake because
# `nros_resolve_knobs()` runs long before the interface generator is included.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosMessageBounds.cmake")
# phase-403 W9 (issue 0965) -- `nros_entity_inventory_knobs_file`, the same one
# path for the ENTITY inventory beside it. Two inventories, two files, one
# ladder: a bound inventory prices a TYPE and this one counts the ENTITIES, and
# `NROS_EXECUTOR_MAX_CBS` is a question only the second can answer.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosEntityInventory.cmake")
# Issue 0991 -- `nros_reconfigure_settle`, for both loaders below. They are the
# EARLIEST readers of either fragment in a Zephyr configure, which is what makes
# them the right place to clear a date a previous pass armed. FILE scope, same
# reason as the two above.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosReconfigure.cmake")

# =============================================================================
# nros_detect_rust_target()
#
# Maps Zephyr CONFIG_* to a Rust target triple. Sets NROS_RUST_TARGET in
# parent scope.
# =============================================================================
function(nros_detect_rust_target)
    if(CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX)
        if(CONFIG_64BIT)
            # native_sim compiles a HOST binary, so its Rust target is whatever
            # this host is — not a constant. This said
            # `x86_64-unknown-linux-gnu`, which is the host triple on an x86
            # machine and a CROSS COMPILE on every other: on aarch64 the build
            # got as far as compiling Rust dependencies for a target whose std
            # is not installed and died in `stable_deref_trait`, naming neither
            # the triple nor native_sim (issue 0582's class, fifth site).
            #
            # `nros_host_rust_triple` is the one spelling of this question —
            # the same helper the ThreadX rustlib lookups use.
            nros_host_rust_triple(_nros_host_triple)
            set(NROS_RUST_TARGET "${_nros_host_triple}" PARENT_SCOPE)
        else()
            # 32-bit native_sim is x86-only in practice; a 32-bit ARM host would
            # want `armv7-unknown-linux-gnueabihf` here. Left as-is rather than
            # guessed, since nothing in this repo builds it.
            set(NROS_RUST_TARGET "i686-unknown-linux-gnu" PARENT_SCOPE)
        endif()
    elseif(CONFIG_CPU_CORTEX_M3)
        set(NROS_RUST_TARGET "thumbv7m-none-eabi" PARENT_SCOPE)
    elseif(CONFIG_CPU_CORTEX_M4 OR CONFIG_CPU_CORTEX_M7)
        if(CONFIG_FPU)
            set(NROS_RUST_TARGET "thumbv7em-none-eabihf" PARENT_SCOPE)
        else()
            set(NROS_RUST_TARGET "thumbv7em-none-eabi" PARENT_SCOPE)
        endif()
    elseif(CONFIG_CPU_CORTEX_M33)
        if(CONFIG_FPU)
            set(NROS_RUST_TARGET "thumbv8m.main-none-eabihf" PARENT_SCOPE)
        else()
            set(NROS_RUST_TARGET "thumbv8m.main-none-eabi" PARENT_SCOPE)
        endif()
    elseif(CONFIG_SOC_SERIES_ESP32C3)
        set(NROS_RUST_TARGET "riscv32imc-unknown-none-elf" PARENT_SCOPE)
    elseif(CONFIG_CPU_AARCH32_CORTEX_R OR CONFIG_CPU_CORTEX_R52 OR CONFIG_CPU_CORTEX_R5)
        # AArch32 Cortex-R (ARMv7-R / ARMv8-R) — Phase 117.11's
        # NXP S32Z R52. zephyr-lang-rust learns the matching
        # triple via `scripts/zephyr/cortex-r-rust-patch.sh`. The
        # FPU bit decides hard-float vs soft-float; both triples
        # are tier-2 Rust.
        if(CONFIG_FPU)
            set(NROS_RUST_TARGET "armv7r-none-eabihf" PARENT_SCOPE)
        else()
            set(NROS_RUST_TARGET "armv7r-none-eabi" PARENT_SCOPE)
        endif()
    elseif(CONFIG_CPU_CORTEX_A9 OR CONFIG_CPU_CORTEX_A7 OR CONFIG_CPU_AARCH32_CORTEX_A)
        # Cortex-A 32-bit (Phase 92's qemu_cortex_a9 + future Zynq /
        # i.MX targets). The zephyr-lang-rust workspace patches set
        # the same triple for the Rust API path; the C/C++ FFI must
        # match so the codegen FFI staticlib links cleanly.
        set(NROS_RUST_TARGET "armv7a-none-eabi" PARENT_SCOPE)
    elseif(CONFIG_ARM64 OR CONFIG_CPU_AARCH64_CORTEX_A OR
           CONFIG_CPU_AARCH64_CORTEX_R OR
           CONFIG_CPU_CORTEX_A53 OR CONFIG_CPU_CORTEX_A72)
        # AArch64 Cortex-A / Cortex-R — Phase 117.10's FVP Base_RevC
        # AEMv8-R SMP is actually AArch64 Cortex-R (CPU_AARCH64_CORTEX_R)
        # despite the name. Same Rust triple covers both. zephyr-lang-rust
        # learns the matching triple via
        # `scripts/zephyr/aarch64-rust-patch.sh`, applied at `just zephyr
        # build-fixtures` time.
        set(NROS_RUST_TARGET "aarch64-unknown-none" PARENT_SCOPE)
    else()
        # phase-340 W3 — "defaulting to host" now NAMES the host triple instead
        # of leaving the variable empty. Empty used to mean "omit --target",
        # cargo's IMPLICIT host spelling, which is a different `-C metadata`
        # identity from `--target <host-triple>` and shares nothing with the
        # rest of the tree (measured: 0 sccache hits across the two spellings).
        # The warning still stands — this branch is a guess about the ARCH — but
        # the guess is now spelled the same way every other build spells it.
        _nros_resolve_rust_target(_nros_host_triple)
        message(WARNING
            "nros: Unknown Zephyr target, defaulting to host (${_nros_host_triple})")
        set(NROS_RUST_TARGET "${_nros_host_triple}" PARENT_SCOPE)
    endif()
endfunction()

# phase-400 W5.b — the ONE keyed shared-cargo-directory rule. Included at FILE
# scope: inside a function `CMAKE_CURRENT_LIST_DIR` names the CALLER's file and
# the frame pop drops what the include defined (the `_NROS_ENTRY_DIR` pattern).
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosSharedCargoDir.cmake")

# =============================================================================
# Knob resolution (issue 0316)
#
# A "knob" is a compile-time static pool size. TWO consumers read the same
# value: the cargo build (an environment variable read by some build.rs) and,
# for the zpico C sources, a preprocessor define emitted by
# nros_rmw_zenoh.cmake. They MUST agree — a Rust/C size disagreement is a
# silent ABI break (issue 0135). That is why resolution happens exactly once,
# here, and both consumers read `NROS_RESOLVED_<KNOB>` instead of reading
# `CONFIG_*` separately.
#
# Precedence is uniform: an explicit environment value WINS over Kconfig, and a
# disagreement is REPORTED rather than silently resolved. Before this the
# `set(ENV{X} ...)` calls were unconditional, so a value exported by a shell or
# justfile was overwritten by the Kconfig default with no diagnostic — six of
# autoware_sentinel's tuned knobs were dead that way, and the two knob classes
# (overwritten vs passed through) were indistinguishable at the call site.
# =============================================================================

# Resolve one knob. `kconfig_value` is the Kconfig-derived value, used only when
# the environment does not already carry an explicit one.
function(_nros_resolve_knob env_name kconfig_value)
    if(DEFINED ENV{${env_name}} AND NOT "$ENV{${env_name}}" STREQUAL "")
        set(_resolved "$ENV{${env_name}}")
        if(NOT "${_resolved}" STREQUAL "${kconfig_value}")
            message(STATUS
                "nros: ${env_name}=${_resolved} from environment "
                "(Kconfig says ${kconfig_value}) — environment wins")
        endif()
    else()
        set(_resolved "${kconfig_value}")
    endif()

    # CACHE INTERNAL, not PARENT_SCOPE: the readers are other functions in other
    # included files, and a normal var would not survive the frame pop
    # (the `_NROS_ENTRY_DIR` pattern — see AGENTS.md CMake Pitfalls).
    set(NROS_RESOLVED_${env_name} "${_resolved}" CACHE INTERNAL
        "nros knob ${env_name}, resolved from environment or Kconfig")

    list(APPEND NROS_RESOLVED_KNOBS "${env_name}")
    list(REMOVE_DUPLICATES NROS_RESOLVED_KNOBS)
    set(NROS_RESOLVED_KNOBS "${NROS_RESOLVED_KNOBS}" CACHE INTERNAL
        "every nros knob resolved during this configure")
endfunction()

# =============================================================================
# The DERIVE sentinel (phase-403 W8, issue 0940)
#
# A Kconfig `int` always states a number, so "the image chose nothing" has no
# spelling of its own -- which is why a derived value could never be a DEFAULT
# without one. `-1` is that spelling. It is not a legal size for any knob that
# takes it, and, importantly, it is NOT `0`: W4 made
# `ZPICO_MAX_LARGE_SUBSCRIBERS = 0` a MEANINGFUL claim ("this image's types all
# fit the small class"), so `0` was already taken.
#
# The precedence ladder, highest first:
#
#   1. an explicit environment value          -- a person, right now
#   2. a Kconfig / board `.conf` value        -- a person, in the tree
#   3. the value derived from message bounds  -- the build
#   4. the crate's own default                -- nobody; the knob stays unset
#
# A derived value is therefore a DEFAULT and never an override. Rung 4 is
# "leave it unresolved", the same tri-state `NROS_EXECUTOR_ARENA_SIZE` already
# uses: an unforwarded knob reaches no cargo environment, the reading build
# script falls through to its own literal, and nothing has to restate that
# literal here where it would drift.
# =============================================================================
set(NROS_KNOB_DERIVE_SENTINEL -1)

# _nros_load_derived_message_bounds()
#
# Read the image-wide answer `nros_find_interfaces()` composed. Sets
# `NROS_DERIVED_*` in the CALLER's scope, or nothing at all.
#
# ORDERING, stated rather than assumed. A Zephyr module's CMakeLists is
# processed during `find_package(Zephyr)`, so this runs BEFORE the application
# reaches its own `nros_find_interfaces()` call -- the file being read is the
# one the PREVIOUS configure wrote.
#
# Issue 0991 -- `CMAKE_CONFIGURE_DEPENDS` was described here as closing that
# lag by itself ("ninja re-runs cmake"). MEASURED: it never did. `build.ninja`
# is written at the END of the generate step, AFTER any file the configure
# wrote, and ninja's regeneration rule fires only on an input that is NEWER
# than `build.ninja`. So the lag did not close slowly; it did not close at all
# until someone re-configured by hand -- which is how a memory-tight image came
# to fail at LINK on a clean build dir, naming 103160 bytes and no knob.
#
# The dependency is still registered (it is what makes a hand re-configure pick
# the file up, and what keeps the ninja input well-formed), but the closing is
# now done by `nros_reconfigure_on_change` at the producer. See
# `cmake/NanoRosReconfigure.cmake`.
#
# So the honest statement of the lag is: an image whose interface closure has
# just changed configures once at its previous sizes, re-configures inside the
# same build, and builds at the derived ones. It is never SILENT: the status
# line below says which of the two happened, and the re-configure announces
# itself.
#
# Issue 1002 -- and "re-configures" is PLURAL on a clean build dir, because the
# producers are a chain rather than one file. This loader reads the message-
# bound sizes; those are derived from the ENTITY inventory, which is written
# later still. So the answer that reaches HERE settles on the third configure:
#
#   pass 1  entity=placeholder  ->  sizes derived over the linked CLOSURE
#   pass 2  entity=real         ->  sizes re-derived over the SUBSCRIBED set
#   pass 3                          this loader finally reads them
#
# Ninja runs all three before the build starts, so a `west build` on a clean
# dir is correct. A HAND-driven `cmake -S -B` sequence that stops after two is
# not, and it looks correct: the fragment on disk already states the derived
# size while `NROS_RESOLVED_*` in the cache still holds the closure one. That
# is measured by case H of `tests/cmake-reconfigure-tests.sh`.
function(_nros_load_derived_message_bounds)
    nros_message_bounds_knobs_file(_knobs)
    if(NOT EXISTS "${_knobs}")
        # Created EMPTY rather than skipped, because the next line registers it
        # as a configure dependency and a ninja input with no producing rule is
        # a hard `missing and no known rule to make it` at load. Seeding a
        # placeholder makes the dependency well-formed on the very first
        # configure, and the interfaces lane overwrites it later in this same
        # one -- which is what makes ninja re-run cmake and pick the answer up.
        nros_message_bounds_seed_knobs_file("${_knobs}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_knobs}")
    # Issue 0991 -- clear a date a previous pass armed. This runs during
    # `find_package(Zephyr)`, which is the EARLIEST reader in the whole
    # configure, and that is why settling belongs here: a configure that fails
    # after this point leaves nothing future-dated behind.
    nros_reconfigure_settle("${_knobs}")
    include("${_knobs}")
    foreach(_v
        NROS_MESSAGE_BOUNDS_STATUS
        NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE
        NROS_DERIVED_SUBSCRIBER_LARGE_SIZE
        NROS_DERIVED_MAX_LARGE_SUBSCRIBERS
        NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE)
        if(DEFINED ${_v})
            set(${_v} "${${_v}}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

# _nros_load_derived_entity_inventory()
#
# phase-403 W9 (issue 0965) -- the sibling of the loader above, for the ENTITY
# inventory: WHICH entities the image creates, which is the half the bound
# inventory cannot answer and `NROS_EXECUTOR_MAX_CBS` has always needed.
#
# Same ordering, same lag, same reason it is not silent. The producer here is
# `nano_ros_entry()` rather than `nros_find_interfaces()` -- an entity count is
# a property of the registered COMPONENTS, so it is composed at the point every
# `nano_ros_node_register()` has run -- but from this module's seat both are
# "later in this configure than I am", and issue 0991's
# `nros_reconfigure_on_change` at those producers is what closes it. The
# `CMAKE_CONFIGURE_DEPENDS` registration is NOT that mechanism and never was;
# see the note on the loader above.
function(_nros_load_derived_entity_inventory)
    nros_entity_inventory_knobs_file(_knobs)
    if(NOT EXISTS "${_knobs}")
        nros_entity_inventory_seed_knobs_file("${_knobs}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_knobs}")
    nros_reconfigure_settle("${_knobs}")
    include("${_knobs}")

    # Re-export EVERY name the fragment sets, read from the fragment itself.
    #
    # This was a hand-maintained WHITELIST and it dropped a symbol three times:
    # phase-412 W1's four session pools (the image derived 10/14/0 and the
    # zenoh session was built 8/8/8 -- eight subscriber slots for ten
    # subscriptions, a RUNTIME failure), W2's MAX_NODES, and phase-403 step 3's
    # five per-kind counts. Each time the symbol loaded here, died at the
    # function boundary, and the consumer fell back to a default that looked
    # deliberate. `include()` inside a function keeps its `set()`s in THIS
    # frame, which is the whole reason a re-export is needed at all.
    #
    # The fragment is GENERATED and every line it emits is `set(<NAME> ...)`,
    # so the set of names is a property of the file rather than of anyone's
    # memory. Parsing it cannot fall behind a producer that adds a name; a list
    # can, and did, three times.
    file(READ "${_knobs}" _frag_text)
    string(REGEX MATCHALL "set\\(([A-Za-z_][A-Za-z0-9_]*)" _frag_sets "${_frag_text}")
    foreach(_hit IN LISTS _frag_sets)
        string(REGEX REPLACE "^set\\(" "" _v "${_hit}")
        if(DEFINED ${_v})
            set(${_v} "${${_v}}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

# _nros_resolve_derivable_knob(<env_name> <kconfig_value> <derived_var>
#                              [<source-name> <reason-file>])
#
# `_nros_resolve_knob` plus rungs 3 and 4 of the ladder above. Use it for a knob
# whose Kconfig option documents `-1` as "derive"; a plain knob keeps
# `_nros_resolve_knob`.
#
# The two optional trailing arguments name WHICH inventory derived the value and
# where its refusal reason is written. They default to the message-bound
# inventory, which was the only one when this function was written; phase-403 W9
# added the ENTITY inventory as a second producer on the same ladder. Naming the
# source in the status line is not cosmetic -- "no value stated and none
# derivable" pointing at the wrong file is a user reading a REASON that explains
# a different refusal.
function(_nros_resolve_derivable_knob env_name kconfig_value derived_var)
    set(_src "message-bound inventory")
    set(_reason_file "${CMAKE_BINARY_DIR}/nros/message_bound_knobs.cmake")
    if(ARGC GREATER 3)
        set(_src "${ARGV3}")
    endif()
    if(ARGC GREATER 4)
        set(_reason_file "${ARGV4}")
    endif()
    if(NOT "${kconfig_value}" STREQUAL "${NROS_KNOB_DERIVE_SENTINEL}")
        # Someone stated a number. It wins over the derivation, in both
        # directions and without comment: that is what "a derived value is a
        # DEFAULT" means.
        _nros_resolve_knob(${env_name} "${kconfig_value}")
        return()
    endif()
    if(DEFINED ENV{${env_name}} AND NOT "$ENV{${env_name}}" STREQUAL "")
        # Rung 1 still outranks rung 3. `_nros_resolve_knob` prints the
        # environment-wins line; give it the derived value (or the sentinel) as
        # the thing being overridden so the message names the real loser.
        _nros_resolve_knob(${env_name} "${${derived_var}}")
        return()
    endif()
    if(DEFINED ${derived_var} AND NOT "${${derived_var}}" STREQUAL "")
        message(STATUS
            "nros: ${env_name}=${${derived_var}} DERIVED from this image's "
            "${_src} (nothing in Kconfig or the environment states one)")
        _nros_resolve_knob(${env_name} "${${derived_var}}")
        return()
    endif()
    # Rung 4. Deliberately NOT resolved: an unforwarded knob leaves the reading
    # build script on its own literal default, which is the one place that
    # literal is written.
    message(STATUS
        "nros: ${env_name} left to its crate default -- no value stated and "
        "none derivable (see the refusal reason in ${_reason_file})")
endfunction()

# =============================================================================
# nros_resolve_knobs()
#
# Resolve every knob for the selected backend. Must run BEFORE any consumer —
# zephyr/CMakeLists.txt calls it ahead of the backend modules, because
# nros_rmw_zenoh.cmake emits its compile definitions at include time.
# =============================================================================
function(nros_resolve_knobs)
    # Drop last configure's list so a backend switch cannot leave stale knobs
    # behind (the per-knob values are overwritten, but the list would grow).
    unset(NROS_RESOLVED_KNOBS CACHE)

    # phase-403 W8 (issue 0940) -- the size knobs' rung-3 values, if this image
    # has an inventory to derive them from. Loaded once, before any resolution,
    # so every `_nros_resolve_derivable_knob` below reads the same answer.
    _nros_load_derived_message_bounds()
    if(NROS_MESSAGE_BOUNDS_STATUS STREQUAL "derived")
        message(STATUS
            "nros: size knobs may derive from this image's message-bound "
            "inventory; a Kconfig or environment value still wins")
    endif()

    # phase-403 W9 (issue 0965) -- the COUNT knobs' rung-3 value, from the
    # entity inventory. Same shape, same lag, different question.
    _nros_load_derived_entity_inventory()
    if(NROS_ENTITY_INVENTORY_STATUS STREQUAL "derived")
        message(STATUS
            "nros: NROS_EXECUTOR_MAX_CBS may derive from this image's entity "
            "inventory (${NROS_ENTITY_INVENTORY_ENTITY_TOTAL} entities declared, "
            "${NROS_DERIVED_EXECUTOR_MAX_CBS} of them claim a callback slot); "
            "a Kconfig or environment value still wins")
    endif()

    # phase-412 W1 -- the SESSION pools, on the same ladder and with the same
    # fallback. These are what the ZENOH SESSION sizes its tables from, not what
    # the executor sizes its slots from, so they count publishers (which claim
    # no callback slot) and they add the entities a declared action opens
    # without declaring.
    #
    # Verified against the shim before wiring, not assumed: ZenohSubscriber::new
    # has exactly ONE caller, and the graph cache and liveliness tokens have
    # their own storage rather than sharing the subscriber pool -- so there is
    # no shim addend to carry here.
    # Written out rather than looped: the knob is NROS_MAX_SUBSCRIBERS and the
    # derived variable is NROS_DERIVED_MAX_SUBSCRIBERS, so a loop that builds
    # `NROS_DERIVED_${_pool}` produces NROS_DERIVED_NROS_MAX_SUBSCRIBERS, which
    # names nothing. It resolves EMPTY rather than failing, the empty reaches
    # zpico.c, and the diagnostic is
    #     error: flexible array member not at end of struct
    # on a struct nobody edited. Spelling both names in full is the point.
    _nros_resolve_derivable_knob(NROS_MAX_SUBSCRIBERS
        "${CONFIG_NROS_MAX_SUBSCRIBERS}" NROS_DERIVED_MAX_SUBSCRIBERS
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
    _nros_resolve_derivable_knob(NROS_RMW_SUBSCRIBER_SLOTS
        "${CONFIG_NROS_RMW_SUBSCRIBER_SLOTS}" NROS_DERIVED_RMW_SUBSCRIBER_SLOTS
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
    _nros_resolve_derivable_knob(NROS_MAX_PUBLISHERS
        "${CONFIG_NROS_MAX_PUBLISHERS}" NROS_DERIVED_MAX_PUBLISHERS
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
    _nros_resolve_derivable_knob(NROS_MAX_QUERYABLES
        "${CONFIG_NROS_MAX_QUERYABLES}" NROS_DERIVED_MAX_QUERYABLES
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
    # phase-412 -- the POSIX mutex pool bounds the subscriber count, and nothing
    # said so until an island image spent days failing at the eleventh
    # subscription.
    #
    # zenoh-pico's Zephyr port creates one pthread mutex per subscriber sync
    # group (`_z_declare_subscriber` -> `_z_sync_group_create` ->
    # `_z_sync_group_state_create` -> `pthread_mutex_init`), and Zephyr's POSIX
    # mutex pool is STATIC. Past it, pthread_mutex_init fails, and the failure
    # reaches the application as an opaque -100 "transport error" naming
    # nothing, on a board whose console is not wired.
    #
    # DELIBERATELY NOT DERIVED. The remaining consumers are zenoh-pico's own
    # (session mutexes, liveliness subscribers, fifo/ring channels, the
    # scheduler, cancellation tokens), so a derivation would encode that crate's
    # internal structure into this build and break silently whenever upstream
    # adds a sync group. A floor with a measured constant is honest about being
    # a bound rather than a model.
    #
    # MEASURED on mr-canhubk344, 2026-09-04: 10 subscribers boot at 32, 11 fail
    # at 32 and boot at 64. One mutex per subscriber, so the fixed overhead is
    # 22. The +4 is headroom for a subscriber added without re-measuring.
    if(CONFIG_NROS_RMW_ZENOH AND DEFINED NROS_RESOLVED_NROS_RMW_SUBSCRIBER_SLOTS
       AND NOT "${NROS_RESOLVED_NROS_RMW_SUBSCRIBER_SLOTS}" STREQUAL "")
        set(_nros_zpico_mutex_overhead 22)
        math(EXPR _nros_mutex_floor
             "${NROS_RESOLVED_NROS_RMW_SUBSCRIBER_SLOTS} + ${_nros_zpico_mutex_overhead} + 4")
        if(DEFINED CONFIG_MAX_PTHREAD_MUTEX_COUNT
           AND CONFIG_MAX_PTHREAD_MUTEX_COUNT LESS _nros_mutex_floor)
            message(FATAL_ERROR
                "CONFIG_MAX_PTHREAD_MUTEX_COUNT=${CONFIG_MAX_PTHREAD_MUTEX_COUNT} is too "
                "small for ${NROS_RESOLVED_NROS_RMW_SUBSCRIBER_SLOTS} subscribers.\n"
                "\n"
                "  need at least ${_nros_mutex_floor}"
                " = ${NROS_RESOLVED_NROS_RMW_SUBSCRIBER_SLOTS} subscribers"
                " + ${_nros_zpico_mutex_overhead} zenoh-pico fixed + 4 headroom\n"
                "\n"
                "zenoh-pico takes one pthread mutex per subscriber sync group and "
                "Zephyr's POSIX mutex pool is static. Past it pthread_mutex_init "
                "fails, and by the time that crosses the C ABI it is an opaque "
                "-100 transport error that names nothing.\n"
                "\n"
                "Set CONFIG_MAX_PTHREAD_MUTEX_COUNT=${_nros_mutex_floor} or higher.")
        endif()
    endif()

    # phase-412 W2 -- one node per declared component. The one under-count is a
    # bridge, whose two nodes are runtime strings declared nowhere; that path
    # now names this knob when the table fills, which is what makes deriving it
    # safe rather than hopeful.
    # phase-403 step 3 -- the ARENA sums over what the image DECLARES, and the
    # byte arithmetic lives in nros-node's build.rs (it knows rx_buf and the
    # target's pointer width; this file knows neither). So the COUNTS travel
    # and the bytes stay put.
    #
    # These are not Kconfig symbols, so `$DOTCONFIG` cannot carry them and the
    # cargo env is the only route -- `nros_set_cargo_env_from_kconfig` exports
    # every name in NROS_RESOLVED_KNOBS, so resolving them here is what makes
    # them reachable. Forwarding a count the inventory did not publish would be
    # WORSE than not forwarding it: build.rs treats a missing count as "nobody
    # declared" and keeps the old worst case, while a zero would sum an arena
    # too small for entities that exist.
    # Spelled out, not looped. A computed name (`NROS_ENTITY_COUNT_${_kind}`)
    # is invisible to `check-kconfig-knob-forwarding`, which reads this file
    # statically: it saw the literal prefix as a forwarded knob no build script
    # reads, and failed. It is the same lesson a computed `NROS_DERIVED_${_pool}`
    # taught one wave earlier, where cmake resolved the built-up name to EMPTY
    # and the knob silently took a default. Knob names are written in full.
    if(DEFINED NROS_ENTITY_COUNT_SUBSCRIPTION AND
       NOT "${NROS_ENTITY_COUNT_SUBSCRIPTION}" STREQUAL "")
        _nros_resolve_knob(NROS_ENTITY_COUNT_SUBSCRIPTION
            "${NROS_ENTITY_COUNT_SUBSCRIPTION}")
        _nros_resolve_knob(NROS_ENTITY_COUNT_TIMER
            "${NROS_ENTITY_COUNT_TIMER}")
        _nros_resolve_knob(NROS_ENTITY_COUNT_SERVICE_SERVER
            "${NROS_ENTITY_COUNT_SERVICE_SERVER}")
        _nros_resolve_knob(NROS_ENTITY_COUNT_ACTION_CLIENT
            "${NROS_ENTITY_COUNT_ACTION_CLIENT}")
        _nros_resolve_knob(NROS_ENTITY_COUNT_ACTION_SERVER
            "${NROS_ENTITY_COUNT_ACTION_SERVER}")
    endif()

    _nros_resolve_derivable_knob(NROS_EXECUTOR_MAX_NODES
        "${CONFIG_NROS_EXECUTOR_MAX_NODES}" NROS_DERIVED_EXECUTOR_MAX_NODES
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")

    # Zenoh transport tuning (zpico-sys build.rs + zpico.c defines).
    #
    # phase-412 W1 -- the three pool defines read NROS_RESOLVED_*, NOT the raw
    # CONFIG_ value. Those knobs default to the `-1` DERIVE sentinel now, and a
    # sentinel reaching zpico.c sizes a C array negative:
    #     error: size of array 'subscribers' is negative
    # which names neither the knob nor the sentinel. The resolution above must
    # therefore stay ABOVE this block; it was written below it first, and this
    # is what that cost.
    if(CONFIG_NROS_RMW_ZENOH)
        _nros_resolve_knob(ZPICO_MAX_PUBLISHERS "${NROS_RESOLVED_NROS_MAX_PUBLISHERS}")
        _nros_resolve_knob(ZPICO_MAX_SUBSCRIBERS "${NROS_RESOLVED_NROS_MAX_SUBSCRIBERS}")
        _nros_resolve_knob(ZPICO_MAX_QUERYABLES "${NROS_RESOLVED_NROS_MAX_QUERYABLES}")
        _nros_resolve_knob(ZPICO_MAX_LIVELINESS "${CONFIG_NROS_MAX_LIVELINESS}")
        _nros_resolve_knob(ZPICO_MAX_PENDING_GETS "${CONFIG_NROS_MAX_PENDING_GETS}")
        _nros_resolve_knob(ZPICO_GET_REPLY_BUF_SIZE "${CONFIG_NROS_GET_REPLY_BUF_SIZE}")
        _nros_resolve_knob(ZPICO_GET_POLL_INTERVAL_MS "${CONFIG_NROS_GET_POLL_INTERVAL_MS}")
        _nros_resolve_knob(ZPICO_FRAG_MAX_SIZE "${CONFIG_NROS_FRAG_MAX_SIZE}")
        _nros_resolve_knob(ZPICO_BATCH_UNICAST_SIZE "${CONFIG_NROS_BATCH_UNICAST_SIZE}")

        # phase-290 (RFC-0049) — tx knob trio, tri-state: always resolved to
        # (0|1) so the cargo-built zpico config header agrees with the zephyr
        # cmake TUs (issue-0135) and an explicit Kconfig `n` overrides the
        # zephyr platform toml's on-default.
        if(CONFIG_NROS_ZENOH_TX_BATCH)
            _nros_resolve_knob(ZPICO_TX_BATCH "1")
        else()
            _nros_resolve_knob(ZPICO_TX_BATCH "0")
        endif()
        if(CONFIG_NROS_ZENOH_TX_SPLIT_LOCK)
            _nros_resolve_knob(ZPICO_TX_SPLIT_LOCK "1")
        else()
            _nros_resolve_knob(ZPICO_TX_SPLIT_LOCK "0")
        endif()
        if(CONFIG_NROS_ZENOH_TX_BATCH_FLUSH_MS)
            _nros_resolve_knob(ZPICO_TX_BATCH_FLUSH_MS
                "${CONFIG_NROS_ZENOH_TX_BATCH_FLUSH_MS}")
        endif()

        # Buffer sizing (nros-rmw-zenoh build.rs)
        #
        # The small payload class is DERIVABLE (phase-403 W8): it is the largest
        # bound at or under the class split, which the inventory knows exactly.
        # A service request is not a message type, so SERVICE_BUFFER_SIZE is not.
        # phase-403 -- ONE name, the backend-agnostic one, which is also what
        # the Kconfig symbol has always been called. This resolved under
        # `ZPICO_SUBSCRIBER_BUFFER_SIZE`, and the mismatch was not cosmetic: it
        # is what let a live delivery bug hide. `check-knob-delivery` pairs
        # NROS_DERIVED_<x> with NROS_RESOLVED_NROS_<x>, so a derived value
        # landing in a ZPICO_-spelled slot was outside every check, and the
        # island shipped the CLOSURE class (1496) while the inventory derived
        # the SUBSCRIBED one (880) -- over-sized, therefore silent.
        #
        # The concept is not zenoh's: it is the largest bound among the types
        # this IMAGE subscribes to. zenoh sizes a payload pool from it and the
        # executor sizes its receive regions from it; both consume one fact.
        _nros_resolve_derivable_knob(NROS_SUBSCRIBER_BUFFER_SIZE
            "${CONFIG_NROS_SUBSCRIBER_BUFFER_SIZE}"
            NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE)
        _nros_resolve_knob(ZPICO_SERVICE_BUFFER_SIZE
            "${CONFIG_NROS_SERVICE_BUFFER_SIZE}")

        # The payload-class trio. These size LARGE_PAYLOADS and SMALL_PAYLOADS
        # (subscriber.rs:199-200) and were reachable only from the environment
        # of whatever shell ran ninja: a consumer exporting them got the right
        # image, and the same tree rebuilt by a bare `ninja` silently got crate
        # defaults. Same class as issue 0316 / #0749.
        _nros_resolve_knob(ZPICO_SUBSCRIBER_RING_DEPTH
            "${CONFIG_NROS_SUBSCRIBER_RING_DEPTH}")
        # The two W6 named by name: "numbers a human produced by reading
        # generated headers with their eyes". Both derivable, and the count is
        # the one that has an answer of ZERO -- W4 made that legal precisely so
        # an image whose types all fit the small class stops reserving
        # RING_DEPTH x LARGE_SIZE for a class it never routes into.
        _nros_resolve_derivable_knob(ZPICO_MAX_LARGE_SUBSCRIBERS
            "${CONFIG_NROS_MAX_LARGE_SUBSCRIBERS}"
            NROS_DERIVED_MAX_LARGE_SUBSCRIBERS)
        _nros_resolve_derivable_knob(ZPICO_SUBSCRIBER_LARGE_SIZE
            "${CONFIG_NROS_SUBSCRIBER_LARGE_SIZE}"
            NROS_DERIVED_SUBSCRIBER_LARGE_SIZE)
    endif()

    # nros-rmw-cffi's static subscription-handle pool. Backend-independent:
    # the no_std slot path is in the cffi adapter, not in a transport.
    #
    # phase-412 W1/W4 -- resolved ABOVE, with the other session pools, because
    # it is derivable now. A second `_nros_resolve_knob` here would re-resolve
    # it from the raw Kconfig value, which is the `-1` DERIVE SENTINEL, and the
    # sentinel would win as though someone had stated it: the knob reached the
    # build as -1 while the inventory had derived 10. check-knob-delivery is
    # what found it, and this comment is here so it is not re-added.

    # The arena is tri-state. nros-node build.rs DERIVES a size when the knob
    # is absent, so forwarding a literal 0 would hand it a zero-byte arena
    # rather than the derivation. Resolve it only when someone actually chose
    # a value -- Kconfig non-zero, or an explicit environment override.
    if(DEFINED ENV{NROS_EXECUTOR_ARENA_SIZE}
       OR NOT "${CONFIG_NROS_EXECUTOR_ARENA_SIZE}" STREQUAL "0")
        _nros_resolve_knob(NROS_EXECUTOR_ARENA_SIZE
            "${CONFIG_NROS_EXECUTOR_ARENA_SIZE}")
    endif()

    # Application heap arena (phase-391 W3). `nros-platform`'s zephyr_heap
    # reads NROS_ZEPHYR_HEAP_SIZE via `option_env!`, but nothing forwarded it
    # into the cargo environment, so the knob was documented and unreachable
    # from a Zephyr build: exporting it had no effect and the arena stayed at
    # its 64 KiB default.
    #
    # That matters because 60b4e0c1e moved z_malloc AND __rust_alloc off the
    # kernel heap onto this arena, so CONFIG_HEAP_MEM_POOL_SIZE no longer
    # governs application allocation. A consumer needing more than 64 KiB has
    # no working way to ask for it, and starvation presents as a silent hang
    # rather than an error. See NEWSLabNTU/nano-ros#41.
    _nros_resolve_knob(NROS_ZEPHYR_HEAP_SIZE "${CONFIG_NROS_ZEPHYR_HEAP_SIZE}")

    # phase-8 — the two arenas are coupled, and nothing used to check it.
    #
    # On this path the EXECUTOR arena is a single heap allocation out of the
    # PLATFORM arena, plus a fixed overhead, so the sizes are not independent:
    #
    #     NROS_EXECUTOR_ARENA_SIZE + overhead  <=  NROS_ZEPHYR_HEAP_SIZE
    #
    # Violate it and the image does not fail at link, does not fault, and does
    # not log. It stops mid-boot on a NULL from an allocation that could never
    # have succeeded. Measured on an FVP consumer: an executor arena of 458752
    # against the 64 KiB default asked for 477104 bytes from a 66048-byte arena
    # -- 7.2x the whole arena, in ONE request, decided entirely at build time.
    # Attributing it took a nine-step bisect.
    #
    # Both values are known here, so this is a build error rather than a runtime
    # hang. The overhead is EMPIRICAL, not derived: two builds differing only in
    # NROS_EXECUTOR_ARENA_SIZE (458752 -> 100000) moved the failing request by
    # exactly that delta, leaving a constant 18352. Treat it as a floor, not a
    # formula -- which is why this adds margin instead of comparing bare sizes.
    if(DEFINED NROS_RESOLVED_NROS_EXECUTOR_ARENA_SIZE
       AND NOT "${NROS_RESOLVED_NROS_EXECUTOR_ARENA_SIZE}" STREQUAL ""
       AND NOT "${NROS_RESOLVED_NROS_EXECUTOR_ARENA_SIZE}" STREQUAL "0")
        set(_nros_arena_overhead 24576)   # 18352 measured, rounded up to 24 KiB
        math(EXPR _nros_arena_need
             "${NROS_RESOLVED_NROS_EXECUTOR_ARENA_SIZE} + ${_nros_arena_overhead}")
        if(_nros_arena_need GREATER "${NROS_RESOLVED_NROS_ZEPHYR_HEAP_SIZE}")
            message(FATAL_ERROR
                "nros: the executor arena cannot fit in the platform heap.\n"
                "  NROS_EXECUTOR_ARENA_SIZE = ${NROS_RESOLVED_NROS_EXECUTOR_ARENA_SIZE}\n"
                "  NROS_ZEPHYR_HEAP_SIZE    = ${NROS_RESOLVED_NROS_ZEPHYR_HEAP_SIZE}\n"
                "  needed (arena + ~${_nros_arena_overhead} overhead) = ${_nros_arena_need}\n"
                "\n"
                "The executor arena is ONE heap allocation out of the platform "
                "arena. Built as configured, this image would stop mid-boot on a "
                "NULL with no fault and no log. Raise CONFIG_NROS_ZEPHYR_HEAP_SIZE "
                "(or the NROS_ZEPHYR_HEAP_SIZE environment override) to at least "
                "${_nros_arena_need}, or lower NROS_EXECUTOR_ARENA_SIZE.")
        endif()
    endif()

    # XRCE transport tuning.
    #
    # `XRCE_TRANSPORT_MTU` is read unprefixed by xrce-sys/build.rs. The pool
    # knobs below are read by nros-rmw-xrce-cffi/build.rs, which spells them
    # `NROS_XRCE_*` — this bridge previously exported the UNPREFIXED names, so
    # nothing read them and five menuconfig options were inert (issue 0316
    # defect 2). The C defaults in nros-rmw-xrce/src/internal.h always won.
    if(CONFIG_NROS_RMW_XRCE)
        # issue 0968 — PREFIXED, like every sibling below it. Issue 0316 defect 2
        # is the comment directly above, and this line was the one site its sweep
        # missed: it kept exporting the unprefixed `XRCE_TRANSPORT_MTU`, which no
        # build script reads, so `CONFIG_NROS_XRCE_TRANSPORT_MTU` was documented
        # in the book, defined in Kconfig, resolved here, and consumed by nobody.
        # The UDP MTU stayed at its 4096 default and the two per-session stream
        # buffers cost 2 x 4096 x 16 = 131072 bytes against a 65536-byte heap.
        _nros_resolve_knob(NROS_XRCE_TRANSPORT_MTU "${CONFIG_NROS_XRCE_TRANSPORT_MTU}")
        # issue 1033 — the subscriber cap joins the DERIVABLE ladder, on the
        # SAME `NROS_DERIVED_MAX_SUBSCRIBERS` the zenoh pools use. Reusing that
        # value rather than counting `sub:` entries here is the point: it
        # already carries `ACTION_CLIENT_SUBSCRIPTIONS`, the multiplier held by
        # `check-infra-queryable-counts`, so an action-carrying image is sized
        # right. A second count here would be a second derivation of one fact,
        # and the one that forgot the multiplier would under-size the image and
        # fail at registration.
        #
        # This is 86% of `xrce_session_state_t`: 8 slots x 32-deep rings of
        # 1024-byte buffers = 266,368 bytes on an image with one subscriber.
        _nros_resolve_derivable_knob(NROS_XRCE_MAX_SUBSCRIBERS
            "${CONFIG_NROS_XRCE_MAX_SUBSCRIBERS}" NROS_DERIVED_MAX_SUBSCRIBERS
            "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
        # issue 1033 — from NROS_DERIVED_MAX_QUERYABLES, not from the raw
        # SERVICE_SERVER count. An XRCE service server is the same thing a
        # zenoh queryable is (a served request endpoint), and that derived
        # value already carries ACTION_SERVER_QUERYABLES — an action server
        # needs three of these underneath it, so the raw count would under-size
        # every action image and fail at registration.
        #
        # Same caveat it carries for zenoh, and it is why this is a DEFAULT and
        # not a ceiling: the parameter (6) and lifecycle (5) service families
        # are enabled by a FEATURE the inventory cannot see, so an image
        # carrying them must state the knob.
        _nros_resolve_derivable_knob(NROS_XRCE_MAX_SERVICE_SERVERS
            "${CONFIG_NROS_XRCE_MAX_SERVICE_SERVERS}" NROS_DERIVED_MAX_QUERYABLES
            "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
        _nros_resolve_knob(NROS_XRCE_MAX_SERVICE_CLIENTS
            "${CONFIG_NROS_XRCE_MAX_SERVICE_CLIENTS}")
        _nros_resolve_knob(NROS_XRCE_BUFFER_SIZE "${CONFIG_NROS_XRCE_BUFFER_SIZE}")
        _nros_resolve_knob(NROS_XRCE_STREAM_HISTORY
            "${CONFIG_NROS_XRCE_STREAM_HISTORY}")
    endif()

    # Executor limits (nros-node build.rs, shared by both Rust and C APIs)
    # C API limits are derived from MAX_CBS via Cargo `links` metadata.
    # phase-403 W9 (issue 0965) -- MAX_CBS moved from the plain resolver to the
    # DERIVABLE one. Its Kconfig default is now the `-1` sentinel, so an image
    # that states nothing gets the entity inventory's answer, and an image whose
    # inventory refuses (any component that declared no ENTITIES -- which is
    # every image built before this wave) falls through to rung 4 and the crate
    # default of 4, exactly as it did before.
    _nros_resolve_derivable_knob(NROS_EXECUTOR_MAX_CBS
        "${CONFIG_NROS_EXECUTOR_MAX_CBS}" NROS_DERIVED_EXECUTOR_MAX_CBS
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
    # Issue 0900 -- the same inventory answers how many of those slots must be
    # budgeted at the ACTION entry size. Derivable for the same reason and with
    # the same fallback: an image whose inventory refuses gets MAX_CBS, which is
    # the historical arithmetic, so no existing image moves.
    _nros_resolve_derivable_knob(NROS_EXECUTOR_ACTION_CLIENTS
        "${CONFIG_NROS_EXECUTOR_ACTION_CLIENTS}" NROS_DERIVED_EXECUTOR_ACTION_CLIENTS
        "entity inventory" "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake")
    # Issue 0316's fix listed ONE of nros-node's six build.rs knobs; the other
    # five stayed unreachable on Zephyr (the curated cargo environment drops
    # any knob not resolved here, and shell exports do not survive it), so a
    # consumer exporting NROS_SUBSCRIPTION_BUFFER_SIZE=16384 silently built
    # 1024-byte subscription buffers — every serialized sample above that is
    # dropped, and the C++ arena dispatch path drops it silently. Resolve the
    # whole class (nros-node + nros-params sizing knobs) in one place.
    #
    # phase-403 W8 (issue 0940): also DERIVABLE. This is buffer 1 -- the
    # runtime-owned take buffer -- and it is ONE global size for every
    # subscription in the image (`RX_BUF` is a const generic and the C/C++ path
    # is type-erased), so the number it wants is the largest type the image can
    # receive. The inventory knows that exactly; an eye reading a header does
    # not. Note the arena scales with it (`max_cbs * (3 * rx_buf + 512) + 2048`),
    # so an image that also PINS `NROS_EXECUTOR_ARENA_SIZE` must move the two
    # together -- which is an argument for pinning neither.
    _nros_resolve_derivable_knob(NROS_SUBSCRIPTION_BUFFER_SIZE
        "${CONFIG_NROS_SUBSCRIPTION_BUFFER_SIZE}"
        NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE)
    _nros_resolve_knob(NROS_EXECUTOR_MAX_SC "${CONFIG_NROS_EXECUTOR_MAX_SC}")
    # phase-412 W2 -- resolved ABOVE, with the other derivable knobs. A second
    # _nros_resolve_knob here re-resolves it from the raw Kconfig value, which
    # is the `-1` DERIVE SENTINEL, and the sentinel wins as though someone had
    # stated it. Third instance of this exact shape in this file
    # (NROS_RMW_SUBSCRIBER_SLOTS was the second); check-knob-delivery caught
    # all of them, and none was visible to any other gate.
    # issue 0790 — shutdown-hook slots per phase. Read by nros-node/build.rs
    # through the derived CONFIG_<name> lookup, like its five siblings above.
    _nros_resolve_knob(NROS_EXECUTOR_MAX_SHUTDOWN_CBS
        "${CONFIG_NROS_EXECUTOR_MAX_SHUTDOWN_CBS}")
    _nros_resolve_knob(NROS_PARAM_SERVICE_BUFFER_SIZE
        "${CONFIG_NROS_PARAM_SERVICE_BUFFER_SIZE}")
    _nros_resolve_knob(NROS_MAX_PARAMETERS "${CONFIG_NROS_MAX_PARAMETERS}")
endfunction()

# =============================================================================
# nros_set_cargo_env_from_kconfig()
#
# Export the resolved knobs so Cargo build.rs scripts pick them up. Works for
# both nros_cargo_build() (C path) and rust_cargo_application() (Rust path).
#
# This function no longer resolves anything — it only exports what
# nros_resolve_knobs() decided, so calling it from several places cannot
# produce different values in different cargo invocations.
# =============================================================================
function(nros_set_cargo_env_from_kconfig)
    if(NOT DEFINED NROS_RESOLVED_KNOBS)
        message(FATAL_ERROR
            "nros: nros_set_cargo_env_from_kconfig() ran before "
            "nros_resolve_knobs() — knob values are unresolved. Call "
            "nros_resolve_knobs() early in the top-level CMakeLists.")
    endif()

    foreach(_knob IN LISTS NROS_RESOLVED_KNOBS)
        set(ENV{${_knob}} "${NROS_RESOLVED_${_knob}}")
    endforeach()

    if(CONFIG_NROS_RMW_ZENOH)
        # zpico-sys build.rs needs the nros-platform-cffi header dir. In-tree dev
        # gets it from .env/direnv; set it from the known module path so a
        # module-consumer / BYO `west build` (no .env) is self-contained
        # (Phase 202.7). CMAKE_CURRENT_FUNCTION_LIST_DIR = this cmake's dir
        # (<repo>/zephyr/cmake) → ../.. = the nano-ros module root. Guarded so
        # the .env value wins, which is what the fallback framing above intends.
        if(NOT DEFINED ENV{NROS_PLATFORM_CFFI_INCLUDE})
            set(ENV{NROS_PLATFORM_CFFI_INCLUDE}
                "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../packages/platform/nros-platform-api/include")
        endif()
    endif()
endfunction()

# =============================================================================
# nros_resolve_cargo_dirs()
#
# phase-400 W5.a — resolve the ROOT workspace's cargo `--target-dir` once, and
# cache it, so the WRITER and the seven generated-header CONSUMERS cannot
# disagree about it.
#
# Same discipline, and the same reason, as `nros_resolve_knobs()` above: the
# value is read from several files in an order nobody controls, so resolving it
# per-reader is how two readers end up with two answers. Consumers ask through
# `_nros_generated_header_dir()` (cmake/NanoRosCodegenCore.cmake), which falls
# back to the historical literal when this has not run — a non-Zephyr build never
# loads this file.
#
# CACHE INTERNAL rather than PARENT_SCOPE for the `_NROS_ENTRY_DIR` reason
# (AGENTS.md CMake Pitfalls): the readers are functions in other included files,
# and a normal var does not survive the frame pop.
#
# Idempotent, and deliberately not re-resolving: a second call inside one
# configure must not be able to return a different directory than the first.
function(nros_resolve_cargo_dirs)
    if(NROS_GENERATED_HEADER_DIR)
        return()
    endif()
    # PER-IMAGE, always. The generated config headers are a function of this
    # image's Kconfig and of nros-{c,cpp}'s feature set, so two images must never
    # write one copy — that is issue 0360's variant collision, and 0528's probe
    # dir one layer down.
    set(NROS_GENERATED_HEADER_DIR "${CMAKE_BINARY_DIR}/nros-rust" CACHE INTERNAL
        "per-image root of the generated nros config headers (phase-400 W5.a)")
endfunction()

# _nros_root_cargo_dir(<out_var> <features>)
#
# phase-400 W5.b — where cargo's DEPENDENCY MASS goes for the nros root
# workspace. Shareable across images; `${CMAKE_BINARY_DIR}/nros-rust` unless a
# caller asks with `-DNROS_SHARED_CARGO_ROOT=<dir>`. Nothing passes it yet, so
# W5.b changes no build — the wiring and the measurement are W5.c.
#
# NOT cached, and NOT resolved alongside the header dir above, because its key
# contains this PACKAGE's features and `nros_cargo_build()` is called once per
# package. A single cached answer cannot be keyed on a per-package input; the
# first draft of this tried and had to name a variable that does not exist.
#
# THE KEY, field by field. Cargo hashes what it owns — features, profile,
# target, deps — into the filenames under `deps/`, so those cannot collide. What
# it does NOT hash is the UPLIFTED artifact: `libnros_c.{a,rlib,so,d}`, the
# nros-cpp trio and the zenoh staticlib pair — ELEVEN files, enumerated from a
# real tree in the phase doc. Those are named for the crate, so two
# configurations sharing a directory overwrite each other's.
#
#   triple, profile   the artifact's target and optimisation
#   features          this package's resolved set, which is what keys nros-c and
#                     nros-cpp apart — their archives genuinely differ
#   knobs             every `NROS_RESOLVED_*`, because they reach compiled code
#                     through the build-script environment. Issue 0528 measured
#                     what omitting them costs: two leaves at the same (target,
#                     features) disagreeing on `CONFIG_NROS_EXECUTOR_MAX_CBS`
#                     shared a probe dir, and one compiled against a constant
#                     sized for the other.
#
# Features are in the key rather than left to cargo's hashing precisely BECAUSE
# of the uplift: with them in, the artifacts that collide are byte-identical by
# construction, so the collision is harmless. That is how this lane avoids
# needing cargo's `--artifact-dir`, which requires `-Z unstable-options` — this
# lane forces nightly only for `armv7a|thumbv|riscv32`, so native_sim is on
# stable and the NuttX eviction mechanism is unavailable here.
function(_nros_root_cargo_dir out_var features)
    set(_dir "")
    # phase-400 W5.c — the refusal that stood here is LIFTED, because the
    # reason for it is gone rather than tolerated.
    #
    # It read: sharing the target dir leaves image B without its generated
    # headers, because B takes a cargo cache hit, its build script never runs,
    # and the per-image directory the consumers include is never written. That
    # was true while the headers came from a side channel inside the target dir.
    #
    # They no longer do. `cargo-out-dir-headers.py` places them from `$OUT_DIR`,
    # which cargo reports on its JSON stream EVEN ON A CACHE HIT (measured: 13
    # `build-script-executed` events with nothing to rebuild), and the BYPRODUCTS
    # name the per-image directory. So a cache hit still places the headers.
    if(COMMAND nros_shared_cargo_dir AND NROS_SHARED_CARGO_ROOT)
        if(NOT DEFINED NROS_RESOLVED_KNOBS)
            message(FATAL_ERROR
                "_nros_root_cargo_dir: nros_resolve_knobs() has not run, so a "
                "shared cargo directory would be keyed WITHOUT the pool sizes it "
                "has to separate on (issue 0528). Resolve the knobs first.")
        endif()
        set(_key "triple=${NROS_RUST_TARGET}" "profile=${NROS_CARGO_PROFILE}"
                 "features=${features}")
        foreach(_knob IN LISTS NROS_RESOLVED_KNOBS)
            list(APPEND _key "${_knob}=${NROS_RESOLVED_${_knob}}")
        endforeach()
        nros_shared_cargo_dir(_dir KEY ${_key})
    endif()
    if(NOT _dir)
        set(_dir "${CMAKE_BINARY_DIR}/nros-rust")
    endif()
    set(${out_var} "${_dir}" PARENT_SCOPE)
endfunction()

# =============================================================================
# _nros_cargo_workspace_root(<manifest> <out-var>)
#
# Resolve the WORKSPACE root manifest that cargo would use for <manifest>, as a
# realpath. This is the identity a `--target-dir` may serve (issue 0616): units
# are keyed by the path spelling their workspace root implies, so two roots
# sharing one directory get two copies of every shared crate.
#
# `cargo locate-project --workspace` is the authority — a manifest's workspace
# root is not derivable from the path (`packages/cli/Cargo.toml` is its own
# root while living inside the repo; a member manifest resolves UP to a root it
# does not name).
#
# Falls back to the manifest itself if cargo cannot answer, which degrades to
# "treat it as its own root" — a private target-dir. That is the safe
# direction: a needless directory costs rebuild time, a shared one costs a
# duplicate lang item.
# =============================================================================
function(_nros_cargo_workspace_root manifest out_var)
    get_filename_component(_manifest_real "${manifest}" REALPATH)

    execute_process(
        COMMAND cargo locate-project --workspace --message-format plain
                --manifest-path "${_manifest_real}"
        OUTPUT_VARIABLE _root
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _rc
        ERROR_VARIABLE _err)

    if(NOT _rc EQUAL 0 OR _root STREQUAL "")
        message(STATUS
            "nano-ros: cargo locate-project could not resolve a workspace root for "
            "${_manifest_real} (${_err}); treating it as its own root, which gives it "
            "a private cargo target-dir (issue 0616).")
        set(${out_var} "${_manifest_real}" PARENT_SCOPE)
        return()
    endif()

    get_filename_component(_root "${_root}" REALPATH)
    set(${out_var} "${_root}" PARENT_SCOPE)
endfunction()

# =============================================================================
# nros_cargo_build(PACKAGE <pkg> FEATURES <features>)
#
# Builds a Rust crate from the nros workspace using Cargo and creates an
# imported static library target. The output library is placed in the Zephyr
# build directory to avoid lock conflicts with other Cargo builds.
#
# Arguments:
#   PACKAGE  - Cargo package name (e.g., "nros-c")
#   FEATURES - Comma-separated feature list (e.g., "rmw-zenoh,platform-zephyr")
#
# Creates target: <pkg_stem>_cargo (imported static library)
#   e.g., nros-c → nros_c_cargo, nros-cpp → nros_cpp_cargo
# =============================================================================
function(nros_cargo_build)
    cmake_parse_arguments(ARG "" "PACKAGE;FEATURES;MANIFEST_PATH" "" ${ARGN})

    if(NOT ARG_PACKAGE)
        message(FATAL_ERROR "nros_cargo_build: PACKAGE is required")
    endif()

    nros_detect_rust_target()

    set(NROS_REPO_DIR ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../..)

    # phase-263 C2c-zephyr — a workspace with a Rust node bundles nros-cpp + every node into
    # the synthesised `nros_ws_runtime` umbrella crate (single-runtime invariant). That crate
    # lives OUTSIDE the nros workspace (its own `[workspace]`), so the caller passes its
    # MANIFEST_PATH; everything else (target / profile / cross-cc / build-std env) is shared.
    if(ARG_MANIFEST_PATH)
        set(_cargo_manifest "${ARG_MANIFEST_PATH}")
    else()
        set(_cargo_manifest "${NROS_REPO_DIR}/Cargo.toml")
    endif()

    # Issue 0616 — a `--target-dir` serves exactly ONE workspace root.
    #
    # The comment above says "everything else is shared", and the target-dir
    # used to be part of that. It cannot be. Cargo's `-C metadata` for a crate
    # includes the PATH SPELLING it was reached by, and one crate has two
    # spellings across two workspaces: inside the nros workspace `nros-platform`
    # is a member (recorded relative to that root), from the generated
    # `nros_ws_runtime` workspace it is an external path dep (recorded
    # absolute). Same package, same features, two `-C metadata` identities, two
    # rlibs in one `deps/` — and `nros-platform` carries the tree's ONE
    # `#[global_allocator]` (issue 0594), so BOTH copies define it. Whichever
    # compile then resolves a transitive `nros_platform` by searching
    # `-L dependency=` instead of an explicit `--extern` can bind the second
    # one and fail with the crate conflicting with itself:
    #
    #     error: the `#[global_allocator]` in nros_platform conflicts with
    #            global allocator in: nros_platform
    #
    # Sharing bought nothing to weigh against this: units are keyed by that
    # same spelling, so two workspaces can never REUSE each other's artifacts.
    # The shared directory produced only the collision. Measured on
    # ws-mixed-entry-zenoh from an empty dir: root-workspace build → 1
    # `libnros_platform-*.rlib`, then the generated-workspace build → 2.
    #
    # Keying on "is the manifest the repo root" would be wrong: `packages/cli`
    # is a separate workspace INSIDE the repo, and a path-prefix test would put
    # it back in the shared dir. Ask cargo which root it actually resolves.
    _nros_cargo_workspace_root("${_cargo_manifest}" _cargo_ws_root)
    get_filename_component(_nros_ws_root "${NROS_REPO_DIR}/Cargo.toml" REALPATH)
    if(_cargo_ws_root STREQUAL _nros_ws_root)
        # phase-400 W5.a — the ROOT workspace's directory is resolved ONCE, by
        # `nros_resolve_cargo_dirs()` below, and published so the generated-header
        # CONSUMERS can follow it instead of re-spelling it as a literal. Today it
        # still resolves to `<build>/nros-rust`, so nothing moves.
        nros_resolve_cargo_dirs()
        # Deferred until after nros_detect_rust_target() / the profile resolve
        # below, which is where the key's triple and profile come from.
        set(_nros_defer_root_cargo_dir TRUE)
    else()
        get_filename_component(_foreign_dir "${_cargo_ws_root}" DIRECTORY)
        get_filename_component(_foreign_name "${_foreign_dir}" NAME)
        string(MAKE_C_IDENTIFIER "${_foreign_name}" _foreign_name)
        set(CARGO_TARGET_DIR ${CMAKE_BINARY_DIR}/nros-rust-ws-${_foreign_name})
    endif()


    # Determine library filename from package name
    string(REPLACE "-" "_" LIB_STEM ${ARG_PACKAGE})
    set(LIB_NAME "lib${LIB_STEM}.a")

    # phase-336 — the profile and its target directory come from the shared
    # table (`nros profile`). This block used to be a fourth copy of the
    # mapping, defaulting to a literal that outlived the name it referred to.
    nros_resolve_cargo_profile()
    set(_nros_cargo_profile "${NROS_CARGO_PROFILE}")
    set(_nros_cargo_profile_dir "${NROS_CARGO_PROFILE_DIR}")

    # phase-340 W3 — one spelling, host included. `nros_detect_rust_target()`
    # always names a triple now (its unknown-arch fallback resolves the host
    # one), so the "no --target, no triple in the path" branch is gone. It was
    # the only way for this lane to emit cargo's implicit host spelling, which
    # is a distinct `-C metadata` identity that shares nothing with the
    # explicit one.
    if(NOT NROS_RUST_TARGET)
        message(FATAL_ERROR
            "nros_cargo_build: NROS_RUST_TARGET is empty. Call "
            "nros_detect_rust_target() first — a host build names its triple "
            "explicitly here (phase-340 W3).")
    endif()
    # phase-400 W5.b — the root-workspace dir needs the triple, the profile AND
    # this package's features, so it is resolved here rather than at the branch
    # that chose it. A foreign workspace root keeps its own dir (issue 0616).
    if(_nros_defer_root_cargo_dir)
        _nros_root_cargo_dir(CARGO_TARGET_DIR "${ARG_FEATURES}")
    endif()

    # The invariant, enforced rather than described (issue 0616). A naming
    # scheme keeps roots apart only until someone adds a caller; this catches
    # it at configure, where the message can name both claimants. Two foreign
    # roots whose directories share a basename would otherwise collide here
    # silently, and the failure they'd produce is a duplicate lang item six
    # build steps later.
    string(MAKE_C_IDENTIFIER "${CARGO_TARGET_DIR}" _td_key)
    get_property(_td_owner GLOBAL PROPERTY "_NROS_TARGET_DIR_OWNER_${_td_key}")
    if(_td_owner AND NOT _td_owner STREQUAL _cargo_ws_root)
        message(FATAL_ERROR
            "nros_cargo_build: two cargo workspace roots would share one --target-dir.\n"
            "  target-dir: ${CARGO_TARGET_DIR}\n"
            "  claimed by: ${_td_owner}\n"
            "  now also:   ${_cargo_ws_root}\n"
            "A target-dir serves exactly ONE workspace root: cargo keys a unit by the "
            "path spelling its root implies, so the same crate gets two `-C metadata` "
            "identities and `nros-platform`'s single `#[global_allocator]` is then "
            "defined twice. Give the new root its own directory (issue 0616).")
    endif()
    set_property(GLOBAL PROPERTY "_NROS_TARGET_DIR_OWNER_${_td_key}" "${_cargo_ws_root}")

    set(LIB_PATH ${CARGO_TARGET_DIR}/${NROS_RUST_TARGET}/${_nros_cargo_profile_dir}/${LIB_NAME})
    set(TARGET_ARGS --target ${NROS_RUST_TARGET})

    # Bridge Kconfig → env vars before invoking Cargo
    nros_set_cargo_env_from_kconfig()

    set(CARGO_ARGS
        build
        -p ${ARG_PACKAGE}
        --manifest-path ${_cargo_manifest}
        --target-dir ${CARGO_TARGET_DIR}
        --no-default-features
    )
    if(_nros_cargo_profile STREQUAL "dev")
    elseif(_nros_cargo_profile STREQUAL "release")
        list(APPEND CARGO_ARGS --release)
    else()
        list(APPEND CARGO_ARGS --profile ${_nros_cargo_profile})
    endif()

    if(ARG_FEATURES)
        list(APPEND CARGO_ARGS --features ${ARG_FEATURES})
    endif()

    if(TARGET_ARGS)
        list(APPEND CARGO_ARGS ${TARGET_ARGS})
    endif()

    # Tier-2/3 embedded targets (armv7a / thumbv* / riscv32) need a
    # nightly toolchain with rust-src + build-std. The workspace's
    # stable rust-toolchain.toml doesn't ship those targets, so
    # override via RUSTUP_TOOLCHAIN and add `-Z build-std`.
    set(_rustup_override "")
    if(NROS_RUST_TARGET MATCHES "^(armv7a|thumbv|riscv32)")
        set(_rustup_override RUSTUP_TOOLCHAIN=nightly-2026-04-11)
        list(APPEND CARGO_ARGS -Z "build-std=core,alloc,compiler_builtins")
    endif()

    # phase-400 W5.c — the headers are declared where the CONSUMERS look, which
    # is the per-image dir, not inside cargo's target dir. `cargo-out-dir-headers.py`
    # places them there from `$OUT_DIR` (cargo's own location for build-script
    # output, reported on the stable JSON stream even on a cache hit). Under a
    # SHARED target dir the old spelling named a file this image would never
    # write — image B gets a cargo cache hit and its build script never runs.
    set(_nros_hdr_root "${NROS_GENERATED_HEADER_DIR}")
    set(_cargo_byproducts ${LIB_PATH})
    if(ARG_PACKAGE STREQUAL "nros-c")
        list(APPEND _cargo_byproducts
            ${_nros_hdr_root}/nros-c-generated/nros/nros_config_generated.h
            ${_nros_hdr_root}/nros-c-generated/nros/nros_generated.h
        )
    elseif(ARG_PACKAGE STREQUAL "nros-cpp")
        # nros-cpp's Cargo dep on nros-c transitively runs nros-c's
        # build.rs, which writes both nros-c headers via cbindgen.
        # Declare them as byproducts so Ninja can order user TUs
        # that include them (`<nros/parameter.hpp>` →
        # `<nros/types.h>` → `<nros/nros_generated.h>`) after this
        # target instead of failing with "No such file or directory"
        # when only CONFIG_NROS_CPP_API=y (no separate nros-c build).
        list(APPEND _cargo_byproducts
            ${_nros_hdr_root}/nros-cpp-generated/nros/nros_cpp_config_generated.h
        )
        # Phase 168.X gap 1 — when nros-c is built separately
        # (CPP_API path now builds it alongside nros-cpp for the log
        # glue), the c-format header is already declared as a
        # byproduct of `nros_c_cargo_build`. Declaring it on both
        # targets makes ninja error with "multiple rules generate".
        # Only claim it for nros-cpp when nros-c is NOT being built.
        if(NOT TARGET nros_c_cargo_build)
            list(APPEND _cargo_byproducts
                ${_nros_hdr_root}/nros-c-generated/nros/nros_config_generated.h
                ${_nros_hdr_root}/nros-c-generated/nros/nros_generated.h
            )
        endif()
    endif()

    # Pass both ZPICO_* and XRCE_* env vars — build.rs ignores vars it
    # doesn't consume, so it's safe to pass both sets unconditionally.
    # This is intentionally an always-evaluated target instead of an OUTPUT
    # rule keyed only on the static archive: build.rs also refreshes the
    # per-build generated headers, and stale headers can break C/C++ compiles
    # even when Cargo considers the archive fresh.
    # Derive target name from package: nros-c → nros_c_cargo
    string(REPLACE "-" "_" _target_stem ${ARG_PACKAGE})
    set(_target_name "${_target_stem}_cargo")

    # Cross-compile env for the `cc` crate that nros-c / nros-cpp's
    # build.rs invoke for `weak_register_backends.c`. cc defaults to
    # the host CC, producing wrong-arch objects (`Relocations in
    # generic ELF (EM: 62)` at link time). Point at the Zephyr SDK
    # toolchain for the active Rust triple so cc picks the right
    # cross compiler. CC_<triple> uses underscores per cc's rules.
    set(_cc_env "")
    if(NROS_RUST_TARGET)
        string(REPLACE "-" "_" _cc_triple ${NROS_RUST_TARGET})
        # Try CMAKE's resolved C compiler first; fall back to the
        # ZEPHYR_SDK_INSTALL_DIR layout if cmake didn't expose it.
        set(_cc_path "${CMAKE_C_COMPILER}")
        if(NOT _cc_path AND DEFINED ENV{ZEPHYR_SDK_INSTALL_DIR})
            file(GLOB _gcc_glob
                "$ENV{ZEPHYR_SDK_INSTALL_DIR}/*-zephyr-elf/bin/*-zephyr-elf-gcc")
            list(GET _gcc_glob 0 _cc_path)
        endif()
        if(_cc_path)
            list(APPEND _cc_env
                CC_${_cc_triple}=${_cc_path}
                CFLAGS_${_cc_triple}=--sysroot=${SYSROOT_DIR}
                AR_${_cc_triple}=${CMAKE_AR}
            )
        endif()
    endif()

    # Forward every resolved knob to the cargo invocation (issue 0316).
    #
    # This matters more than it looks: `set(ENV{X})` only changes the CONFIGURE
    # -time environment, and cargo runs at BUILD time under ninja. The only
    # values that reach a build.rs are the ones named here, whose `$ENV{}` is
    # expanded at configure time and baked into the build command. A knob that
    # is resolved but not listed is silently unreachable from Kconfig.
    #
    # The list used to be hand-maintained and had drifted three ways: the XRCE
    # entries used the unprefixed spelling that no build.rs reads (the readers
    # in nros-rmw-xrce-cffi/build.rs want `NROS_XRCE_*`), while
    # NROS_EXECUTOR_MAX_CBS and the RFC-0049 tx trio were resolved into the
    # environment but never forwarded at all. Generating the list from
    # NROS_RESOLVED_KNOBS makes "resolved but not forwarded" unrepresentable.
    set(_nros_knob_env "")
    foreach(_knob IN LISTS NROS_RESOLVED_KNOBS)
        list(APPEND _nros_knob_env "${_knob}=${NROS_RESOLVED_${_knob}}")
    endforeach()

    # phase-412 -- the boot self-report. A BOOL, so it does not go through the
    # resolve ladder: that machinery carries a tri-state (env > Kconfig >
    # derived, with -1 and 0 as sentinels for "derive it") and a bool has no
    # derive state to represent. Putting one through would mean inventing a
    # third meaning for a sentinel, which is the shape that has already produced
    # three double-resolution defects in this file.
    #
    # But it rides THIS list, not `set(ENV{})`, for the reason the comment below
    # gives: the cargo that reads it is spawned by a custom target at BUILD
    # time, so a configure-time environment never reaches it. Written the wrong
    # way first, and the image linked without the record while every cmake
    # variable said it was on -- the delivery-failure class phase-412 W4 exists
    # for, arriving in the work that was meant to instrument it.
    if(CONFIG_NROS_BOOT_REPORT)
        list(APPEND _nros_knob_env "NROS_BOOT_REPORT=1")
    endif()

    # phase-351 W5 — the board FACTS + SITE config ride the same command, for
    # the same reason the knobs do: Zephyr's cargo is spawned by this custom
    # target, and `set(ENV{})` would only touch the configure-time process.
    # Corrosion's `corrosion_set_env_vars` (the seam every other lane uses) is
    # not available here — zephyr-lang-rust builds its own cargo command — so
    # the delivery is the same VALUES through this lane's own carrier.
    nros_resolve_board_facts()
    set(_nros_facts_env "")
    foreach(_fact IN LISTS NROS_BOARD_FACTS_ENV)
        list(APPEND _nros_facts_env "${_fact}")
    endforeach()

    # phase-336 — the preset's definition rides on the command, so a crate whose
    # own manifest declares no `nros-*` profile still resolves the name. Empty
    # for a user-owned profile, whose manifest governs.
    add_custom_target(${_target_name}_build
        COMMAND ${CMAKE_COMMAND} -E env
            ${_rustup_override}
            ${_cc_env}
            ${_nros_knob_env}
            ${_nros_facts_env}
            ${NROS_CARGO_PROFILE_ENV}
            NROS_PLATFORM_CFFI_INCLUDE=$ENV{NROS_PLATFORM_CFFI_INCLUDE}
            # phase-400 W5.c — cargo runs UNDER the placer, not piped into it:
            # `add_custom_target(COMMAND …)` has no shell, so there is no pipe.
            # One process tree, one exit code, and stderr untouched —
            # `json-render-diagnostics` keeps diagnostics human on stderr and
            # leaves stdout pure JSON for the placer to read `out_dir` from.
            python3 ${NROS_REPO_DIR}/scripts/build/cargo-out-dir-headers.py
                --package ${ARG_PACKAGE}
                --dest ${_nros_hdr_root}
                -- cargo ${CARGO_ARGS} --message-format=json-render-diagnostics
        BYPRODUCTS ${_cargo_byproducts}
        COMMENT "Building ${ARG_PACKAGE} via Cargo"
        VERBATIM
    )
    # nros_cargo_build() calls that share a workspace root share a
    # ${CARGO_TARGET_DIR} (issue 0616 split the foreign roots off into their
    # own). Serialize Cargo frontends to avoid artifact-dir lock stalls;
    # Cargo/rustc still get parallel compiler tokens from the inherited
    # jobserver. Kept unconditional across roots: the ordering is also what
    # makes the generated runtime crate build after the nros-c/nros-cpp
    # headers it includes, and one frontend at a time is cheap.
    if(NOT ARG_PACKAGE STREQUAL "nros-c" AND TARGET nros_c_cargo_build)
        add_dependencies(${_target_name}_build nros_c_cargo_build)
    endif()
    if(NOT ARG_PACKAGE STREQUAL "nros-c"
       AND NOT ARG_PACKAGE STREQUAL "nros-cpp"
       AND TARGET nros_cpp_cargo_build)
        add_dependencies(${_target_name}_build nros_cpp_cargo_build)
    endif()

    add_library(${_target_name} STATIC IMPORTED GLOBAL)
    set_target_properties(${_target_name} PROPERTIES
        IMPORTED_LOCATION ${LIB_PATH}
    )
    add_dependencies(${_target_name} ${_target_name}_build)
endfunction()
