# NanoRosMessageBounds.cmake -- phase-403 W8 (issue 0940): the READER for the
# bound inventory W6 exports.
#
# W6 gave every generated interface package a derived per-type serialized-size
# bound and three transports to carry it out of codegen. Nothing read any of
# them, so every size downstream was still a number a human typed -- and on the
# one bring-up that measured it, every one of those numbers was wrong at least
# once. `NROS_MAX_LARGE_SUBSCRIBERS` and `NROS_SUBSCRIBER_LARGE_SIZE` were read
# off generated C++ headers BY EYE, and the headers they were read off state an
# ESTIMATE rather than a bound (W6's own finding).
#
# This module composes the fragments and derives the FOUR knobs a bound
# inventory can actually answer.
#
# =============================================================================
# What a bound inventory can and cannot answer
# =============================================================================
#
# It knows EVERY TYPE'S SIZE. It does not know WHICH ENTITIES AN IMAGE CREATES.
# W4 established that: a resolved SystemModel carries topic wiring only where
# somebody authored a `<stem>.contract.yaml` beside the launch file (issue
# 0973 -- measured 2026-09-06, 5 of the tree's 114 resolvable models do), and
# the RFC-0043 C++ components register in their constructors at runtime. So:
#
#   DERIVABLE HERE -- a question about sizes:
#     NROS_SUBSCRIBER_BUFFER_SIZE     the small payload class
#     NROS_SUBSCRIBER_LARGE_SIZE      the large payload class
#     NROS_MAX_LARGE_SUBSCRIBERS      how many types exceed the small ceiling
#     NROS_SUBSCRIPTION_BUFFER_SIZE   the take buffer for a caller with no type
#
#   NOT DERIVABLE HERE -- a question about entity COUNTS, which needs a second
#   source (an entity inventory), not this one:
#     NROS_EXECUTOR_MAX_CBS, NROS_EXECUTOR_ARENA_SIZE,
#     NROS_MAX_SUBSCRIBERS, NROS_MAX_PUBLISHERS
#
#   A package's TYPE count is not an image's ENTITY count. Deriving those from
#   this inventory would produce exactly the plausible-wrong-number this
#   campaign exists to remove.
#
#   THAT SECOND SOURCE NOW EXISTS: `cmake/NanoRosEntityInventory.cmake`
#   (phase-403 W9, issue 0965), composed from every
#   `nano_ros_node_register(... ENTITIES ...)` in the image. It derives
#   NROS_EXECUTOR_MAX_CBS. The other three still are not derived, and that is
#   deliberate rather than unfinished: the arena and the payload classes need
#   the two inventories JOINED per subscription -- this one's per-type size
#   against that one's per-entity list -- and a total taken from either half
#   alone is the same confident wrong number.
#
# =============================================================================
# TWO BASES, because the four knobs are not four answers to one question
# =============================================================================
#
# The bound inventory holds every type in the LINKED INTERFACE CLOSURE, not just
# the received ones. A package is linked because something in the image mentions
# one of its types; the other 90 come along. Derived over that, a class size is
# the largest type the image COULD receive, not the largest it DOES -- and on
# the reference island that gap is the dominant term, not a rounding error: one
# `std_msgs/Float64MultiArray`, linked and never received, sets the small class
# for every subscription in the image.
#
# So this reader derives on TWO bases and says which it used for what.
#
#   BASIS `subscribed` -- the THREE PAYLOAD-CLASS knobs
#     NROS_SUBSCRIBER_BUFFER_SIZE, NROS_SUBSCRIBER_LARGE_SIZE,
#     NROS_MAX_LARGE_SUBSCRIBERS
#   These size the backend's two topic payload pools. Those pools are reached
#   through exactly one allocation -- `alloc_payload_block(rx_buffer_hint)` in
#   `shim/subscriber.rs` -- with exactly one caller, the `declare_subscriber`
#   path. So their population is the image's SUBSCRIPTIONS, and the entity
#   inventory (`cmake/NanoRosEntityInventory.cmake`, phase-403 W9) names them.
#   The join is this file's `ENTITY_INVENTORY` argument.
#
#   BASIS `closure` -- the take buffer
#     NROS_SUBSCRIPTION_BUFFER_SIZE
#   This one is NOT subscription-only, whatever its name says, and narrowing it
#   to the subscribed set would size a buffer too small. `nros-node/build.rs`
#   turns it into `DEFAULT_RX_BUF_SIZE`, which is the DEFAULT const generic for
#   `RawSubscription`, `RawServiceServer`, `RawServiceClient`,
#   `ActionServerCore` and `ActionClientCore` -- and `executor/types.rs` then
#   defines `DEFAULT_TX_BUF = DEFAULT_RX_BUF_SIZE`, so it is also the stack
#   array `EmbeddedPublisher::publish` serialises into. A type this image only
#   PUBLISHES still has to fit. The closure over-approximates that, in the safe
#   direction, and stays.
#
# =============================================================================
# The join REFUSES; it never quietly widens
# =============================================================================
#
# Once an image declares its entities, the payload classes are derived from
# them or not at all. Specifically, with `ENTITY_INVENTORY` naming a fragment
# whose own status is `derived`:
#
#   * its subscribed-type set REFUSED (a component declared no `ENTITIES`, or a
#     subscription states no type)      -> the payload classes REFUSE
#   * it names a type this bound inventory does not price at all
#                                       -> the payload classes REFUSE, naming it
#   * it names a type that is `unbounded`/`unresolved`
#                                       -> the whole derivation already refused
#
# None of those fall back to the closure. Falling back would publish the WRONG
# ROW of the table above while every status still read "derived", which is the
# shape that looks like it worked.
#
# The one case that DOES derive over the closure is the image that declared
# NOTHING -- no `ENTITIES` anywhere, so the entity fragment is absent or its own
# status is `refused`. That is every image built before phase-403 W9, and it
# keeps exactly the numbers it has today: `NROS_MESSAGE_BOUNDS_BASIS` reads
# `closure`, the status line says so, and the generated file carries the
# paragraph explaining what the declaration would buy. Refusing there instead
# would take those images from an over-approximate derived number back to the
# hand-set ones this whole wave exists to replace, which is a regression, not a
# safety property.
#
# =============================================================================
# An unbounded type is not silently dropped
# =============================================================================
#
# If ANY type in the closure is `unbounded` or `unresolved`, no class size is
# derived at all. The alternative -- deriving over the bounded subset -- would
# publish a maximum that a real sample can exceed, which is the silent
# BufferTooSmall drop this whole phase exists to stop. The refusal is LOUD, it
# names the types and the member that costs each one its bound, and every knob
# falls back to its configured value.
#
# =============================================================================
# Usage
# =============================================================================
#
#   include(NanoRosMessageBounds.cmake)
#   nros_derive_message_bound_knobs(
#       FRAGMENTS <nros_message_bounds.cmake>...   # or omit: uses the cache list
#       [ENTITY_INVENTORY <entity_inventory.cmake>]# the join; see above
#       [SMALL_CLASS_CEILING 2048]                 # policy, see below
#       [OUTPUT_FILE <path>]                       # write the answer + why
#       [QUIET])
#
# Sets in the CALLER's scope:
#
#   NROS_MESSAGE_BOUNDS_STATUS        derived | refused
#   NROS_MESSAGE_BOUNDS_REASON        prose, when refused
#   NROS_MESSAGE_BOUNDS_PACKAGES      the packages composed
#   NROS_MESSAGE_BOUNDS_TYPE_COUNT    types seen
#   NROS_MESSAGE_BOUNDS_BOUNDED_COUNT types with a derived bound
#   NROS_MESSAGE_BOUNDS_OPEN_TYPES    the unbounded/unresolved ones
#   NROS_MESSAGE_BOUNDS_BASIS         subscribed | closure -- which set the
#                                     three payload-class knobs were derived
#                                     over. UNSET when they were not derived.
#   NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS  derived | refused, for those three
#                                     alone. It can refuse while the take
#                                     buffer still derives, and that is not a
#                                     contradiction: two bases, two questions.
#   NROS_MESSAGE_BOUNDS_PAYLOAD_REASON  prose, when it refused
#   NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT  subscribing ENTITIES joined
#   NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE     \
#   NROS_DERIVED_SUBSCRIBER_LARGE_SIZE       |  unset when not derivable --
#   NROS_DERIVED_MAX_LARGE_SUBSCRIBERS       |  ABSENT means "no answer",
#   NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE   /   never a substituted default
#   NROS_DERIVED_LARGEST_TYPE / _LARGEST_RX  provenance for the two above
#   NROS_DERIVED_LARGE_TYPES                 which types drove MAX_LARGE
#
# A derived value is a DEFAULT. Every consumer applies it only where nothing
# else stated a number -- see `_nros_resolve_derivable_knob` in
# `zephyr/cmake/nros_cargo_build.cmake` for the precedence ladder.

include_guard(GLOBAL)

# Both constants below are `CACHE INTERNAL` and not plain variables, for the
# `_NROS_ENTRY_DIR` reason (AGENTS.md, CMake Pitfalls) sharpened by
# `include_guard(GLOBAL)`. This file is reached through
# `NanoRosCodegenCore.cmake`, and at least one caller
# (`NanoRosWorkspace.cmake`'s `nros_resolve_cli` branch) includes that from
# INSIDE a function. If that include happens first, a file-scope `set()` here
# lands in that frame and is gone when it pops -- while the guard makes every
# later include a no-op, so the constant never comes back. The FUNCTIONS
# survive (they are global) and only the variables vanish, which is the shape
# that fails far from its cause: an empty schema constant turns the version
# check into a FATAL_ERROR on every well-formed fragment.

# The inventory schema this reader understands. `bounds.rs`'s
# `INVENTORY_SCHEMA_VERSION`. A fragment that states anything else is REFUSED,
# never read field-by-field on the hope that nothing moved.
set(NROS_MESSAGE_BOUNDS_SCHEMA_SUPPORTED 1 CACHE INTERNAL
    "phase-403 W8: the nros_message_bounds fragment schema this tree reads")

# Where this module lives, so the join can reach its sibling
# `NanoRosEntityInventory.cmake` for the ENTITY fragment's schema constant.
#
# `CACHE INTERNAL` for the same `_NROS_ENTRY_DIR` reason the constant above
# gives -- this file is reachable from inside a function frame, and a file-scope
# `set()` that lands there is gone when the frame pops while `include_guard`
# makes every later include a no-op.
#
# Reading `NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED` from that module rather than
# restating the number here is deliberate: a second spelling of a version
# constant is how a producer and a reader come to disagree while both look
# right, which is the failure the constant exists to prevent.
set(_NROS_MESSAGE_BOUNDS_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL
    "phase-403: the directory NanoRosMessageBounds.cmake was included from")

# The split between the small and the large payload class, in bytes.
#
# POLICY, not a derived fact: it is `ZPICO_SUBSCRIBER_SIZE_THRESHOLD`'s shipped
# default, and the shim's own ceiling is `min(threshold, SUBSCRIBER_BUFFER_SIZE)`
# (`shim/subscriber.rs::SMALL_CLASS_CEILING`). Because the derived small size is
# by construction the largest bound AT OR UNDER this number, that `min` picks
# the derived size and the two agree. Pass the resolved threshold instead when a
# consumer has set one, so the classification here and the routing at runtime
# cannot disagree.
set(NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING 2048 CACHE INTERNAL
    "phase-403 W8: the small/large payload class split used when none is passed")

# nros_message_bounds_register_fragment(<path>)
#
# Record one package's `nros_message_bounds.cmake` in the image-wide list the
# composer reads when it is given no explicit FRAGMENTS. Called by both
# generator lanes (`cmake/NanoRosGenerateInterfaces.cmake` and
# `zephyr/cmake/nros_generate_interfaces.cmake`) so there is ONE place to look
# and not one per lane.
#
# A GLOBAL PROPERTY and not a CACHE variable, deliberately. The list has to
# cross function frames and `add_subdirectory()` boundaries, which both rule out
# a normal variable -- but it must NOT survive to the next configure, which
# rules out the cache. A cached list keeps a package that has been REMOVED from
# the closure, and its stale fragment is usually still sitting in the build dir,
# so the derivation would go on pricing a type nothing links. A global property
# is reset at the start of every configure, which is exactly the lifetime the
# closure has.
function(nros_message_bounds_register_fragment _path)
    get_property(_list GLOBAL PROPERTY NROS_MESSAGE_BOUNDS_FRAGMENTS)
    list(APPEND _list "${_path}")
    list(REMOVE_DUPLICATES _list)
    list(SORT _list)
    set_property(GLOBAL PROPERTY NROS_MESSAGE_BOUNDS_FRAGMENTS "${_list}")
endfunction()

# nros_message_bounds_fragments(<out_var>) -- the list, in the caller's scope.
function(nros_message_bounds_fragments _out_var)
    get_property(_list GLOBAL PROPERTY NROS_MESSAGE_BOUNDS_FRAGMENTS)
    set(${_out_var} "${_list}" PARENT_SCOPE)
endfunction()

# _nros_bounds_publish(<name> <value>)
#
# Set a result BOTH locally and in the caller's scope.
#
# A `set(X ... PARENT_SCOPE)` writes ONLY the parent, so the deriving function
# cannot read back what it just published -- and `_nros_message_bounds_write_output`
# reads exactly those names through the scope chain. Publishing to one of the
# two scopes wrote a file full of empty values while every returned variable was
# correct, which is the shape that reads as working.
#
# A macro and not a function: a macro runs in the CALLER's scope, so its
# `PARENT_SCOPE` is the caller's parent, which is what the name promises.
macro(_nros_bounds_publish _name _value)
    set(${_name} "${_value}")
    set(${_name} "${_value}" PARENT_SCOPE)
endmacro()

# nros_message_bounds_knobs_file(<out_var>)
#
# Where the composed, image-wide answer is written, and where a consumer reads
# it. ONE path, because the writer (`nros_find_interfaces`) and the reader
# (`nros_resolve_knobs`, in the Zephyr lane) are in different files, run at
# different points of one configure, and a second spelling is how a derived
# value silently stops arriving.
#
# `CMAKE_BINARY_DIR` and not a per-package dir: the answer is a property of the
# IMAGE, composed over every interface package it links.
function(nros_message_bounds_knobs_file _out_var)
    set(${_out_var} "${CMAKE_BINARY_DIR}/nros/message_bound_knobs.cmake" PARENT_SCOPE)
endfunction()

# nros_message_bounds_seed_knobs_file(<path>)
#
# Write a "nothing composed yet" knobs file, for a reader that runs BEFORE the
# interface lane in the same configure.
#
# It exists for one mechanical reason. A consumer registers this path with
# `CMAKE_CONFIGURE_DEPENDS`, and a ninja input that does not exist and has no
# rule producing it is a hard `missing and no known rule to make it` at LOAD,
# before any rule runs. Seeding makes the dependency well-formed on the very
# first configure; the real answer overwrites it later in that same configure.
#
# Issue 0991 -- that registration is NOT what makes the new answer reach the
# readers who already ran. It cannot be: `build.ninja` is written after the
# fragment, so the dependency is never stale. `nros_reconfigure_on_change`
# (cmake/NanoRosReconfigure.cmake) is the mechanism that re-runs cmake.
#
# Does NOT overwrite an existing file: the whole point is that the file may
# already hold a derived answer.
function(nros_message_bounds_seed_knobs_file _path)
    if(EXISTS "${_path}")
        return()
    endif()
    get_filename_component(_dir "${_path}" DIRECTORY)
    file(MAKE_DIRECTORY "${_dir}")
    file(WRITE "${_path}"
        "# GENERATED by nros (phase-403 W8, issue 0940). Do not edit.\n"
        "#\n"
        "# Placeholder: no message-bound inventory had been composed when this\n"
        "# configure first needed one. It is rewritten with the real answer by\n"
        "# nros_find_interfaces(); that rewrite arms a re-configure (issue 0991)\n"
        "# so the readers that already ran this pass see the real answer.\n"
        "set(NROS_MESSAGE_BOUNDS_STATUS \"refused\")\n"
        "set(NROS_MESSAGE_BOUNDS_REASON \"no inventory composed yet\")\n")
endfunction()

# nros_derive_message_bound_knobs(...)  -- see the header comment.
# Issue 0963 — publish the three payload-class knobs from a completed join.
#
# A MACRO, not a function: it publishes through `_nros_bounds_publish`, which
# sets in the caller's scope, and it reads a dozen locals the caller already
# holds. A function would need all of them threaded through and would put the
# publishes one scope too deep.
#
# Called from two places now, which is the point of factoring it: a closure that
# refuses the take buffer can still answer the payload classes, because they are
# a fact about what the image SUBSCRIBES to and the refusal is a fact about what
# it LINKS.
macro(_nros_bounds_publish_payload_classes)
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_BASIS "${_basis}")

    # BASIS `closure` -- this image declared no entities, so there is no
    # join to make and the payload classes keep exactly the answer W8
    # published: derived over every type in the linked closure. The label
    # and the status line are what stop that being mistaken for the joined
    # row.
    if(_basis STREQUAL "closure")
        set(_sub_small "${_small}")
        set(_sub_large_types "${_large_types}")
        set(_sub_large_max "${_large_max}")
        list(LENGTH _large_types _sub_large_count)
    endif()

    # Buffer 2, the backend's staging pools: two classes, split at the
    # policy ceiling. `_sub_small` is the largest bound AT OR UNDER the
    # ceiling among the receiving set, so the shim's own
    # `min(threshold, SUBSCRIBER_BUFFER_SIZE)` picks it and the
    # classification here is the routing at runtime.
    #
    # `_sub_small == 0` means nothing received fits under the ceiling --
    # including the case where nothing is received at all. The small class
    # is still USED (a caller that states no hint is served from it), so
    # there is nothing to derive and the configured value stands.
    if(_sub_small GREATER 0)
        _nros_bounds_publish(NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE "${_sub_small}")
    endif()

    # The large COUNT is a count of BLOCKS, so on the `subscribed` basis it
    # counts subscribing ENTITIES and not distinct types: two subscriptions
    # on one large type need two blocks, and a type count would under-reserve
    # by exactly the duplicates. On the `closure` basis there are no entities
    # to count and it stays a type count, which is what it has always been.
    _nros_bounds_publish(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS "${_sub_large_count}")
    _nros_bounds_publish(NROS_DERIVED_LARGE_TYPES "${_sub_large_types}")
    if(_sub_large_count GREATER 0)
        _nros_bounds_publish(NROS_DERIVED_SUBSCRIBER_LARGE_SIZE "${_sub_large_max}")
    endif()
endmacro()

function(nros_derive_message_bound_knobs)
    cmake_parse_arguments(_B "QUIET"
        "SMALL_CLASS_CEILING;OUTPUT_FILE;ENTITY_INVENTORY" "FRAGMENTS" ${ARGN})

    set(_fragments "${_B_FRAGMENTS}")
    if(NOT _fragments)
        nros_message_bounds_fragments(_fragments)
    endif()
    set(_ceiling "${_B_SMALL_CLASS_CEILING}")
    if(NOT _ceiling)
        set(_ceiling "${NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING}")
    endif()

    # ---- Nothing derived until proven otherwise -------------------------
    # Every out-variable starts UNSET. A knob that cannot be derived must be
    # absent, so a consumer either reads a number this function computed or
    # reads nothing -- the same rule the inventory itself holds to for a type
    # with no bound.
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_STATUS "refused")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PACKAGES "")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_TYPE_COUNT 0)
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_BOUNDED_COUNT 0)
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_OPEN_TYPES "")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "refused")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PAYLOAD_REASON
        "the derivation did not reach the payload classes")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT 0)
    # Cleared in BOTH scopes. The parent's copy so a second call cannot leave a
    # stale answer standing; this frame's copy because a function inherits the
    # caller's variables through the scope chain, and
    # `_nros_message_bounds_write_output` reads these names that way -- so a
    # refusal after a successful call would otherwise write the PREVIOUS numbers
    # into a file whose status says "refused".
    foreach(_v
        NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE
        NROS_DERIVED_SUBSCRIBER_LARGE_SIZE
        NROS_DERIVED_MAX_LARGE_SUBSCRIBERS
        NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE
        NROS_DERIVED_LARGEST_TYPE
        NROS_DERIVED_LARGEST_RX
        NROS_DERIVED_LARGE_TYPES
        NROS_MESSAGE_BOUNDS_BASIS)
        unset(${_v})
        unset(${_v} PARENT_SCOPE)
    endforeach()

    if(NOT _fragments)
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_REASON "no message-bound inventory was produced by this configure")
        _nros_message_bounds_write_output("${_B_OUTPUT_FILE}" "refused"
            "no message-bound inventory was produced by this configure" "${_ceiling}")
        return()
    endif()

    # ---- Compose ---------------------------------------------------------
    # `include()` inside a function keeps the fragment's `set()`s local to this
    # frame, which is what makes composing several packages safe: each fragment
    # APPENDs to the two lists and de-duplicates them itself.
    #
    # The schema version is checked PER FRAGMENT and BEFORE anything is read
    # from it. A mixed-version tree (one package regenerated, one not) is the
    # case that would otherwise read a moved field as if it had not moved.
    #
    # A MISSING fragment is a refusal, not a fatal: on the canonical lane
    # codegen is a build-time custom command, so on a clean tree the file is a
    # promise rather than a fact. A fragment that EXISTS and is malformed or
    # from another schema IS fatal -- that is a broken producer, not a lane
    # that has not run yet.
    set(_pending "")
    foreach(_frag IN LISTS _fragments)
        if(NOT EXISTS "${_frag}")
            list(APPEND _pending "${_frag}")
            continue()
        endif()
        unset(NROS_MESSAGE_BOUNDS_SCHEMA_VERSION)
        include("${_frag}")
        if(NOT DEFINED NROS_MESSAGE_BOUNDS_SCHEMA_VERSION)
            message(FATAL_ERROR
                "nros: ${_frag} sets no NROS_MESSAGE_BOUNDS_SCHEMA_VERSION.\n"
                "  Either it is not a message-bound fragment, or it predates "
                "the schema. Regenerate it with `nros codegen`.")
        endif()
        if(NOT NROS_MESSAGE_BOUNDS_SCHEMA_VERSION EQUAL
           NROS_MESSAGE_BOUNDS_SCHEMA_SUPPORTED)
            message(FATAL_ERROR
                "nros: ${_frag} states message-bound schema version "
                "${NROS_MESSAGE_BOUNDS_SCHEMA_VERSION}; this reader understands "
                "${NROS_MESSAGE_BOUNDS_SCHEMA_SUPPORTED}.\n"
                "  Refusing rather than reading fields that may have moved.\n"
                "  Rebuild the `nros` CLI so the producer and the reader come "
                "from one tree: `./scripts/bootstrap.sh` (contributors: "
                "`just setup-cli`).")
        endif()
    endforeach()

    if(_pending)
        list(LENGTH _pending _pending_count)
        list(LENGTH _fragments _frag_count)
        string(REPLACE ";" "\n    " _pending_block "${_pending}")
        set(_why
            "${_pending_count} of ${_frag_count} message-bound fragments have not been written yet:\n    ${_pending_block}")
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_REASON "${_why}")
        if(NOT _B_QUIET)
            message(STATUS
                "nros: message-bound sizing not available this configure -- "
                "${_pending_count} of ${_frag_count} fragments are still a "
                "build-time output. Every size knob keeps its configured value; "
                "the numbers apply from the next configure.")
        endif()
        _nros_message_bounds_write_output("${_B_OUTPUT_FILE}" "refused" "${_why}" "${_ceiling}")
        return()
    endif()

    set(_packages "${NROS_MESSAGE_BOUND_PACKAGES}")
    set(_types "${NROS_MESSAGE_BOUND_TYPES}")
    list(LENGTH _types _type_count)
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PACKAGES "${_packages}")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_TYPE_COUNT "${_type_count}")

    if(_type_count EQUAL 0)
        set(_why "the composed inventory holds no message types")
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_REASON "${_why}")
        _nros_message_bounds_write_output("${_B_OUTPUT_FILE}" "refused" "${_why}" "${_ceiling}")
        return()
    endif()

    # ---- Read every type, and refuse on the first open one ---------------
    set(_open "")
    set(_open_detail "")
    set(_bounded 0)
    set(_max_rx 0)
    set(_max_type "")
    set(_small 0)
    set(_large_types "")
    set(_large_max 0)
    foreach(_t IN LISTS _types)
        string(REGEX REPLACE "[^A-Za-z0-9]" "_" _key "${_t}")
        set(_state "${NROS_MESSAGE_BOUND_${_key}_STATE}")
        if(NOT _state STREQUAL "bounded")
            list(APPEND _open "${_t}")
            set(_reason "${NROS_MESSAGE_BOUND_${_key}_REASON}")
            if(NOT _reason)
                set(_reason "no reason recorded")
            endif()
            list(APPEND _open_detail "    ${_t} (${_state}): ${_reason}")
            continue()
        endif()
        set(_rx "${NROS_MESSAGE_BOUND_${_key}_RX}")
        if(NOT _rx MATCHES "^[0-9]+$")
            # `bounded` with no `_RX` cannot happen from a fragment this reader
            # accepts -- but a hand-edited or half-written one would, and a
            # non-numeric compared with LESS silently reads as 0.
            message(FATAL_ERROR
                "nros: ${_t} is `bounded` in the inventory and carries no "
                "numeric _RX (`${_rx}`). The fragment is malformed; regenerate "
                "it with `nros codegen`.")
        endif()
        math(EXPR _bounded "${_bounded} + 1")
        if(_rx GREATER _max_rx)
            set(_max_rx "${_rx}")
            set(_max_type "${_t}")
        endif()
        if(_rx GREATER _ceiling)
            list(APPEND _large_types "${_t}=${_rx}")
            if(_rx GREATER _large_max)
                set(_large_max "${_rx}")
            endif()
        elseif(_rx GREATER _small)
            set(_small "${_rx}")
        endif()
    endforeach()
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_BOUNDED_COUNT "${_bounded}")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_OPEN_TYPES "${_open}")

    # ---- The JOIN: which of those types does this image RECEIVE? ---------
    #
    # phase-403 step 1. Everything above is a fact about the closure. The three
    # payload-class knobs are a fact about the SUBSCRIPTIONS, and this is where
    # the second inventory supplies them. See the header for why the take
    # buffer deliberately does not take part.
    #
    # Issue 0963 — this runs BEFORE the closure refusal below, which it did not
    # until 2026-09-06. The refusal `return()`s, so an image whose every
    # SUBSCRIBED type is bounded was refused for a type it merely LINKS, and the
    # narrowing this join exists to provide was unreachable through the public
    # entry point. The join is pure (it publishes nothing), so computing it
    # early changes no output on the path that then refuses everything.
    _nros_bounds_join_subscribed("${_B_ENTITY_INVENTORY}" "${_ceiling}"
        _basis _payload_status _payload_why _sub_count
        _sub_small _sub_large_types _sub_large_max _sub_large_count)

    if(_open)
        list(LENGTH _open _open_count)
        string(REPLACE ";" "\n" _open_block "${_open_detail}")
        set(_why
            "${_open_count} of ${_type_count} types in the linked interface closure have no derived bound, so no class size can be trusted:\n${_open_block}")
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_REASON "${_why}")
        if(NOT _B_QUIET)
            message(WARNING
                "nros: message-bound sizing REFUSED -- every size knob keeps its "
                "configured value.\n"
                "  ${_open_count} of ${_type_count} types in the linked "
                "interface closure carry no bound:\n${_open_block}\n"
                "  Deriving a class size over only the bounded types would "
                "publish a maximum a real sample can exceed, which is a SILENT "
                "BufferTooSmall drop on the C/C++ arena dispatch path.\n"
                "  Remedy: bound the member in its `.msg` (`string<=64`), or cap "
                "it `inline` in the package's `nros-codegen.toml` -- `inline` is "
                "the only mode that bounds (RFC-0033). One cap on a DECLARING "
                "type is transitive: `\"std_msgs/Header.frame_id\" = { cap = 64, "
                "mode = \"inline\" }` bounds every message that nests a Header.")
        endif()
        # Issue 0963 — the take buffer refuses, and the PAYLOAD CLASSES need
        # not. They are two different facts:
        #
        #   * `NROS_SUBSCRIPTION_BUFFER_SIZE` is one global size for every
        #     entity, and `DEFAULT_TX_BUF` aliases it, so a type the image only
        #     PUBLISHES must still fit. That is a fact about the whole linked
        #     closure, and an open type in the closure genuinely poisons it.
        #   * the three payload-class knobs size the backend's staging pools for
        #     what the image RECEIVES. An unbounded type nothing subscribes to
        #     cannot reach them.
        #
        # So an image whose every SUBSCRIBED type is bounded now gets its
        # payload classes even while the take buffer keeps its configured value.
        # That was this issue's second remedy — "an image pays only for the
        # types it actually links" — and it was unreachable while this `return()`
        # ran before the join.
        #
        # ONLY on a real `subscribed` join. The `closure` basis fallback inside
        # the macro derives over `_small` / `_large_types`, which on THIS path
        # were accumulated over the bounded types only — deriving a class size
        # from those is precisely the under-derivation the refusal above exists
        # to prevent, and its failure mode is a silent `BufferTooSmall`. A
        # missing entity inventory therefore still gets nothing.
        # The payload STATUS is published either way: a reader that gets no
        # classes should be able to see whether the join declined and why,
        # rather than inferring it from absent variables.
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "${_payload_status}")
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PAYLOAD_REASON "${_payload_why}")
        _nros_bounds_publish(NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT "${_sub_count}")
        if(_payload_status STREQUAL "derived" AND _basis STREQUAL "subscribed")
            _nros_bounds_publish_payload_classes()
            if(NOT _B_QUIET)
                message(STATUS
                    "nros: payload classes DERIVED over ${_sub_count} subscribed "
                    "type(s) despite the closure refusal above -- every type this "
                    "image RECEIVES is bounded. The take buffer keeps its "
                    "configured value (issue 0963).")
            endif()
        endif()
        _nros_message_bounds_write_output("${_B_OUTPUT_FILE}" "refused" "${_why}" "${_ceiling}")
        return()
    endif()

    # ---- Derive ----------------------------------------------------------
    #
    # Buffer 1, the runtime-owned take buffer: ONE global size for every
    # ENTITY in the image (`RX_BUF` is a const generic and the C/C++ path is
    # type-erased), so it must hold the largest type the image could receive --
    # and, because `DEFAULT_TX_BUF` aliases it, the largest it could publish.
    # BASIS `closure`, always. Narrowing this one is the under-derivation.
    _nros_bounds_publish(NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE "${_max_rx}")
    _nros_bounds_publish(NROS_DERIVED_LARGEST_TYPE "${_max_type}")
    _nros_bounds_publish(NROS_DERIVED_LARGEST_RX "${_max_rx}")

    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "${_payload_status}")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_PAYLOAD_REASON "${_payload_why}")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT "${_sub_count}")

    if(_payload_status STREQUAL "derived")
        _nros_bounds_publish_payload_classes()
    endif()
    # A count of ZERO is an ANSWER, not an abstention -- W4 made
    # `ZPICO_MAX_LARGE_SUBSCRIBERS = 0` legal precisely so an image whose types
    # all fit the small class can say so and stop reserving
    # RING_DEPTH x LARGE_SIZE for a class it never routes into. The large SIZE
    # is deliberately left underived in that case: with zero blocks the pool is
    # zero bytes whatever the size says, and naming a size for a class that does
    # not exist would be inventing a number.

    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_STATUS "derived")
    _nros_bounds_publish(NROS_MESSAGE_BOUNDS_REASON "")

    if(NOT _B_QUIET)
        list(LENGTH _packages _pkg_count)
        message(STATUS
            "nros: message-bound sizing DERIVED from ${_type_count} types in "
            "${_pkg_count} interface packages (all bounded)")
        message(STATUS
            "nros:   largest type ${_max_type} at ${_max_rx} B -> "
            "NROS_SUBSCRIPTION_BUFFER_SIZE (basis: the whole closure -- this "
            "knob is also DEFAULT_TX_BUF and every raw entity's default "
            "buffer, so a type this image only publishes still has to fit)")
        if(NOT _payload_status STREQUAL "derived")
            message(WARNING
                "nros: the three PAYLOAD-CLASS knobs are REFUSED -- "
                "NROS_SUBSCRIBER_BUFFER_SIZE, NROS_SUBSCRIBER_LARGE_SIZE and "
                "NROS_MAX_LARGE_SUBSCRIBERS keep their configured values.\n"
                "  ${_payload_why}\n"
                "  They are NOT falling back to the closure: this image states "
                "what it receives, and a class sized over types it does not "
                "receive is the wrong answer wearing a `derived` status.")
        else()
            if(_sub_small GREATER 0)
                message(STATUS
                    "nros:   small payload class ${_sub_small} B -> "
                    "NROS_SUBSCRIBER_BUFFER_SIZE")
            endif()
            message(STATUS
                "nros:   ${_sub_large_count} over the ${_ceiling} B ceiling -> "
                "NROS_MAX_LARGE_SUBSCRIBERS")
            if(_sub_large_count GREATER 0)
                message(STATUS
                    "nros:   large payload class ${_sub_large_max} B -> "
                    "NROS_SUBSCRIBER_LARGE_SIZE")
            endif()
            if(_basis STREQUAL "subscribed")
                message(STATUS
                    "nros:   payload classes derived over the ${_sub_count} "
                    "SUBSCRIPTIONS this image declares, not the "
                    "${_type_count}-type closure")
            else()
                message(STATUS
                    "nros:   payload classes are an UPPER BOUND -- this image "
                    "declares no entities, so they derive over the whole "
                    "linked closure and are sized for the largest type it "
                    "COULD receive. Declare "
                    "`nano_ros_node_register(... ENTITIES sub:<pkg>/msg/<Name> "
                    "...)` to narrow them to what it does.")
            endif()
        endif()
    endif()

    _nros_message_bounds_write_output("${_B_OUTPUT_FILE}" "derived" "" "${_ceiling}")
endfunction()

# _nros_bounds_join_subscribed(<entity_fragment> <ceiling>
#                              <out_basis> <out_status> <out_why> <out_count>
#                              <out_small> <out_large_types> <out_large_max>
#                              <out_large_count>)
#
# phase-403 step 1 -- the JOIN. Reads the ENTITY inventory's subscribed-type
# set and classifies each of those types against the bounds this frame has
# already composed.
#
# Called from inside `nros_derive_message_bound_knobs`, so it reads the
# per-type `NROS_MESSAGE_BOUND_<key>_STATE` / `_RX` variables through the SCOPE
# CHAIN -- the same way `_nros_message_bounds_write_output` reads the published
# results. That is what makes it a function rather than a second composition:
# there is one `include()` of the fragments per call, and re-including them here
# would be a second read of the same files with a second chance to differ.
#
# `<out_basis>` is `subscribed` or `closure`; on `closure` the four
# classification outputs are left EMPTY and the caller substitutes its own
# closure-wide values. `<out_status>` is `derived` or `refused`, and a refusal
# NEVER degrades to `closure` -- see the header.
function(_nros_bounds_join_subscribed _frag _ceiling
         _o_basis _o_status _o_why _o_count _o_small _o_large_types _o_large_max
         _o_large_count)
    set(${_o_basis} "" PARENT_SCOPE)
    set(${_o_status} "derived" PARENT_SCOPE)
    set(${_o_why} "" PARENT_SCOPE)
    set(${_o_count} 0 PARENT_SCOPE)
    set(${_o_small} 0 PARENT_SCOPE)
    set(${_o_large_types} "" PARENT_SCOPE)
    set(${_o_large_max} 0 PARENT_SCOPE)
    set(${_o_large_count} 0 PARENT_SCOPE)

    # No fragment named, or none written yet. The image declared nothing, which
    # is every image built before phase-403 W9. Keep W8's closure answer and
    # label it; see the header for why this one case is not a refusal.
    if(NOT _frag OR NOT EXISTS "${_frag}")
        set(${_o_basis} "closure" PARENT_SCOPE)
        return()
    endif()

    # The entity fragment carries its own schema, and it is checked HERE too
    # rather than trusted because this reader has landed since it was written --
    # a version-1 fragment predates the subscribed-type set entirely, and
    # reading it would silently derive a payload class over an EMPTY set.
    #
    # The constant comes from the module that OWNS the fragment, so there is one
    # spelling of the number. The include is a no-op when that module is already
    # in (the usual case: `nano_ros_entry()` includes it), and its constants are
    # `CACHE INTERNAL`, so they survive an include from inside this frame.
    include("${_NROS_MESSAGE_BOUNDS_DIR}/NanoRosEntityInventory.cmake")
    unset(NROS_ENTITY_INVENTORY_SCHEMA_VERSION)
    unset(NROS_ENTITY_INVENTORY_STATUS)
    unset(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS)
    unset(NROS_ENTITY_SUBSCRIBED_TYPES)
    unset(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS)
    unset(NROS_ENTITY_SUBSCRIBED_TYPES_REASON)
    include("${_frag}")

    if(NOT DEFINED NROS_ENTITY_INVENTORY_SCHEMA_VERSION OR
       NOT NROS_ENTITY_INVENTORY_SCHEMA_VERSION EQUAL
           NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED)
        # LOUD, and deliberately NOT a FATAL_ERROR, which is the one place this
        # module departs from its own "refuse rather than read a moved field"
        # rule. The fragment's PRODUCER (`nano_ros_entry()`) runs LATER in this
        # same configure than this reader does, so a fatal here aborts the
        # configure before the stale fragment can ever be rewritten -- the build
        # dir would be stuck until someone deleted the file by hand. Nothing is
        # read from the fragment either way: the payload classes fall to the
        # closure, which over-approximates in the safe direction.
        message(WARNING
            "nros: ${_frag} states entity-inventory schema version "
            "`${NROS_ENTITY_INVENTORY_SCHEMA_VERSION}`; the payload-class join "
            "understands ${NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED}. It is being "
            "IGNORED -- nothing is read from it field-by-field.\n"
            "  A fragment that predates the join carries no subscribed-type set "
            "at all, so this image's payload classes derive over its whole "
            "LINKED CLOSURE, which is an upper bound and not the answer its "
            "declaration would give.\n"
            "  Rebuild the `nros` CLI so the producer and the reader come from "
            "one tree (`./scripts/bootstrap.sh`; contributors: "
            "`just setup-cli`) and re-configure; the fragment is "
            "rewritten by `nano_ros_entry()` later in that configure and this "
            "reader picks it up on the one after.")
        set(${_o_basis} "closure" PARENT_SCOPE)
        return()
    endif()

    # The image registered components but at least one declared no `ENTITIES`,
    # so W9 refused for the whole image. Nothing was declared that this join can
    # narrow to; the pre-W9 state, and it keeps the pre-W9 answer.
    #
    # Issue 0991 -- SAY SO when the refusal is the FIRST-CONFIGURE placeholder
    # rather than a real one. The producer (`nano_ros_entry()`) runs later in
    # this configure than this reader, so on a clean build dir the fragment is
    # always the placeholder and the basis is always `closure`. That is a safe
    # over-approximation for correctness and NOT safe for a part that is nearly
    # full: on the mr-canhubk344 island it sets the small class from a linked-
    # only type and the image overflows RAM by 103160 bytes at LINK, naming a
    # byte count and no knob. The recovery is one more configure, which no
    # error message used to state.
    if(NOT NROS_ENTITY_INVENTORY_STATUS STREQUAL "derived"
       AND DEFINED NROS_ENTITY_INVENTORY_REASON
       AND NROS_ENTITY_INVENTORY_REASON STREQUAL "no entity inventory composed yet")
        message(STATUS
            "nros: the entity inventory has not been composed in this build dir "
            "yet, so the payload classes derive over the LINKED CLOSURE this "
            "pass -- an over-approximation.\n"
            "  This is expected on a CLEAN build dir and resolves itself "
            "inside this build: the fragment is written at the end of this "
            "configure, the NEXT configure derives over the SUBSCRIBED set, "
            "and the one AFTER that is the first whose readers -- the knob "
            "resolver in `find_package(Zephyr)`, which runs before either "
            "producer -- hand the derived class to the compile. Three "
            "configures, because the chain is two producers deep; ninja runs "
            "them all before the build proceeds (issue 1002).\n"
            "  If this image is memory-tight, that first over-approximation can "
            "fail to LINK (issue 0991). What must not happen is reading a size "
            "or a link error out of a HAND-driven configure that stopped at "
            "two -- the fragment is right by then and the delivered value is "
            "still one pass behind it.")
    endif()
    if(NOT NROS_ENTITY_INVENTORY_STATUS STREQUAL "derived")
        set(${_o_basis} "closure" PARENT_SCOPE)
        return()
    endif()

    # From here the image HAS declared, so the join is live and every failure
    # below is a REFUSAL. A fall-back to the closure would publish a number
    # derived over types this image never receives while the status still read
    # `derived`.
    if(NOT DEFINED NROS_ENTITY_SUBSCRIBED_TYPES_STATUS)
        # The version matches, so the producer is current and the field must be
        # there. Its absence is a hand-edited or half-written fragment. REFUSED
        # and not widened to the closure: the image DID declare, so a closure
        # answer here would be the wrong row wearing a `derived` status.
        set(${_o_status} "refused" PARENT_SCOPE)
        set(${_o_why}
            "${_frag} states entity-inventory schema ${NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED}, which carries a subscribed-type set, and sets no NROS_ENTITY_SUBSCRIBED_TYPES_STATUS. The fragment is malformed -- delete it and re-configure, or regenerate the `nros` CLI with `./scripts/bootstrap.sh` (contributors: `just setup-cli`)."
            PARENT_SCOPE)
        return()
    endif()
    if(NOT NROS_ENTITY_SUBSCRIBED_TYPES_STATUS STREQUAL "resolved")
        set(${_o_status} "refused" PARENT_SCOPE)
        set(${_o_why}
            "this image declares its entities, so the payload classes are derived from them or not at all -- and the subscribed-type set did not resolve:\n${NROS_ENTITY_SUBSCRIBED_TYPES_REASON}"
            PARENT_SCOPE)
        return()
    endif()

    set(_count 0)
    set(_small 0)
    set(_large_types "")
    set(_large_max 0)
    set(_large_count 0)
    set(_unpriced "")
    foreach(_entry IN LISTS NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS)
        string(REGEX REPLACE "=[0-9]+$" "" _t "${_entry}")
        string(REGEX REPLACE "^.*=" "" _n "${_entry}")
        math(EXPR _count "${_count} + ${_n}")
        string(REGEX REPLACE "[^A-Za-z0-9]" "_" _key "${_t}")
        if(NOT DEFINED NROS_MESSAGE_BOUND_${_key}_STATE)
            list(APPEND _unpriced "${_t}")
            continue()
        endif()
        # A subscribed type that is `unbounded`/`unresolved` has no `_RX` worth
        # reading, and reading one anyway would size a payload class from a
        # blank or a placeholder -- the silent BufferTooSmall this module exists
        # to prevent.
        #
        # Today this cannot fire: the closure-wide open-type check above
        # `return()`s before this function is called, and the subscribed set is
        # a SUBSET of the closure. That is exactly why the check belongs here.
        # The invariant lives in another block, thirty lines up, and nothing
        # states the dependency -- so the first person to make the payload
        # classes derivable on an image whose CLOSURE has an unbounded type
        # (issue 0963's "narrow the closure" remedy, which is the whole point of
        # this join) removes the guard without knowing it was one.
        if(NOT NROS_MESSAGE_BOUND_${_key}_STATE STREQUAL "bounded")
            list(APPEND _open_subscribed
                 "${_t} (${NROS_MESSAGE_BOUND_${_key}_STATE})")
            continue()
        endif()
        set(_rx "${NROS_MESSAGE_BOUND_${_key}_RX}")
        if(_rx GREATER _ceiling)
            list(APPEND _large_types "${_t}=${_rx}")
            math(EXPR _large_count "${_large_count} + ${_n}")
            if(_rx GREATER _large_max)
                set(_large_max "${_rx}")
            endif()
        elseif(_rx GREATER _small)
            set(_small "${_rx}")
        endif()
    endforeach()

    # A type the entity inventory names and the bound inventory does not price
    # is the join failing, and it is loud. Today the commonest cause is
    # structural rather than a typo: the bound inventory records MESSAGES only
    # (`BoundInventory::record_message` is called for `.msg` and for nothing
    # else), so a `pkg/srv/Name_Request` or a `pkg/action/Name_Result` has no
    # entry however well-formed the declaration is.
    # Checked BEFORE `_unpriced` because it is the more specific answer: a type
    # this image receives and this tree cannot bound is a different problem from
    # one the inventory has never heard of, and saying "not in the inventory"
    # about a type that IS in it would send the reader looking for a typo.
    if(_open_subscribed)
        list(LENGTH _open_subscribed _open_sub_count)
        string(REPLACE ";" "\n    " _open_sub_block "${_open_subscribed}")
        set(${_o_status} "refused" PARENT_SCOPE)
        set(${_o_why}
            "${_open_sub_count} type(s) this image RECEIVES carry no derived bound, so their payload class cannot be sized:\n    ${_open_sub_block}\n  Bound the member in its `.msg` (`string<=64`) or cap it `inline` in the package's `nros-codegen.toml` (RFC-0033). Narrowing to the subscribed set does not help here -- these are in it."
            PARENT_SCOPE)
        return()
    endif()

    if(_unpriced)
        list(LENGTH _unpriced _unpriced_count)
        string(REPLACE ";" "\n    " _unpriced_block "${_unpriced}")
        set(${_o_status} "refused" PARENT_SCOPE)
        set(${_o_why}
            "${_unpriced_count} type(s) this image receives are not in the bound inventory, so their payload class cannot be derived:\n    ${_unpriced_block}\n  Either the declaration names a type this image does not link, or it names a service/action type -- the bound inventory prices MESSAGES (`pkg/msg/Name`) and nothing else, so `pkg/srv/*` and `pkg/action/*` have no entry to join against."
            PARENT_SCOPE)
        return()
    endif()

    set(${_o_basis} "subscribed" PARENT_SCOPE)
    set(${_o_count} "${_count}" PARENT_SCOPE)
    set(${_o_small} "${_small}" PARENT_SCOPE)
    set(${_o_large_types} "${_large_types}" PARENT_SCOPE)
    set(${_o_large_max} "${_large_max}" PARENT_SCOPE)
    set(${_o_large_count} "${_large_count}" PARENT_SCOPE)
endfunction()

# _nros_message_bounds_write_output(<path> <status> <reason> <ceiling>)
#
# The composed answer, as an `include()`able fragment AND as a readable record
# of where every number came from. Written only when the caller asked for a
# path.
#
# WRITE-IF-CHANGED, and that is load-bearing rather than tidy: issue 0991's
# `nros_reconfigure_on_change` compares this file's CONTENT before and after the
# derivation, so a producer that rewrote identical bytes would re-arm a
# re-configure forever. (The digest is over content, so this is belt and braces
# -- but the file is also read by humans, and a diff that moves every configure
# is noise.)
function(_nros_message_bounds_write_output _path _status _reason _ceiling)
    if(NOT _path)
        return()
    endif()
    set(_c "# GENERATED by nros (phase-403 W8, issue 0940). Do not edit.\n")
    string(APPEND _c "#\n")
    string(APPEND _c "# The size knobs DERIVED from this image's message-bound inventory\n")
    string(APPEND _c "# (`nros_message_bounds.cmake`, one per generated interface package).\n")
    string(APPEND _c "#\n")
    string(APPEND _c "# Every number here is a DEFAULT. A board `.conf`, a Kconfig value or an\n")
    string(APPEND _c "# environment override states a number and WINS; this file only fills in\n")
    string(APPEND _c "# what nobody stated.\n")
    string(APPEND _c "#\n")
    string(APPEND _c "# There are TWO BASES here and the file says which each number used.\n")
    string(APPEND _c "# NROS_SUBSCRIPTION_BUFFER_SIZE is always derived over the LINKED closure:\n")
    string(APPEND _c "# it is also DEFAULT_TX_BUF and the default buffer of every raw service,\n")
    string(APPEND _c "# client and action entity, so a type this image only PUBLISHES still has\n")
    string(APPEND _c "# to fit and narrowing it would size a buffer too small. The three payload\n")
    string(APPEND _c "# classes are derived over the SUBSCRIBED set when the image declares its\n")
    string(APPEND _c "# entities (phase-403 W9), and over the closure when it declares none.\n")
    string(APPEND _c "#\n")
    string(APPEND _c "# Derivation: nros_serdes::size::max_serialized_size, the same rule the\n")
    string(APPEND _c "# runtime's M::MAX_SERIALIZED_SIZE_XCDR* uses. NOT the C++ pack's\n")
    string(APPEND _c "# SERIALIZED_SIZE_MAX, which is an estimate (phase-403 W6).\n")
    string(APPEND _c "#\n")
    string(APPEND _c "# small/large class split: ${_ceiling} B (policy -- ZPICO_SUBSCRIBER_SIZE_THRESHOLD)\n")
    string(APPEND _c "\n")
    string(APPEND _c "set(NROS_MESSAGE_BOUNDS_STATUS \"${_status}\")\n")
    if(_status STREQUAL "refused")
        string(REPLACE "\\" "\\\\" _r "${_reason}")
        string(REPLACE "\"" "\\\"" _r "${_r}")
        string(REPLACE "\n" "\\n" _r "${_r}")
        string(APPEND _c "set(NROS_MESSAGE_BOUNDS_REASON \"${_r}\")\n")
        # Issue 0963 — a refusal is about the TAKE BUFFER, and the payload
        # classes can still have an answer. This branch used to write nothing
        # but the reason, so the values the derivation had already computed
        # never reached the consumer: the file IS the transport, and a knob
        # published in memory but absent from it does not exist.
        #
        # Only ever written when the join ran on the `subscribed` basis — the
        # caller enforces that, and the comment below states which knobs the
        # refusal still covers so a reader is not left inferring it.
        if(DEFINED NROS_MESSAGE_BOUNDS_BASIS
                AND NROS_MESSAGE_BOUNDS_BASIS STREQUAL "subscribed")
            string(APPEND _c
                "# The TAKE BUFFER is refused: an unbounded type in the linked closure\n"
                "# could be published through it (DEFAULT_TX_BUF aliases it), so\n"
                "# NROS_SUBSCRIPTION_BUFFER_SIZE keeps its configured value.\n"
                "#\n"
                "# The PAYLOAD CLASSES below are derived anyway: they size the staging\n"
                "# pools for what this image RECEIVES, every subscribed type IS bounded,\n"
                "# and an unbounded type nothing subscribes to cannot reach them.\n")
            string(APPEND _c
                "set(NROS_MESSAGE_BOUNDS_BASIS \"${NROS_MESSAGE_BOUNDS_BASIS}\")\n")
            string(APPEND _c
                "set(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS \"${NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS}\")\n")
            if(DEFINED NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE)
                string(APPEND _c
                    "set(NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE ${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE})\n")
            endif()
            if(DEFINED NROS_DERIVED_MAX_LARGE_SUBSCRIBERS)
                string(APPEND _c
                    "set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS ${NROS_DERIVED_MAX_LARGE_SUBSCRIBERS})\n")
            endif()
            if(DEFINED NROS_DERIVED_SUBSCRIBER_LARGE_SIZE)
                string(APPEND _c
                    "set(NROS_DERIVED_SUBSCRIBER_LARGE_SIZE ${NROS_DERIVED_SUBSCRIBER_LARGE_SIZE})\n")
            endif()
        else()
            string(APPEND _c "# No knob is derived. Every one keeps its configured value.\n")
        endif()
    else()
        string(APPEND _c "set(NROS_MESSAGE_BOUNDS_PACKAGES \"${NROS_MESSAGE_BOUNDS_PACKAGES}\")\n")
        string(APPEND _c "set(NROS_MESSAGE_BOUNDS_TYPE_COUNT ${NROS_MESSAGE_BOUNDS_TYPE_COUNT})\n")
        string(APPEND _c
            "# ${NROS_DERIVED_LARGEST_TYPE} is the largest type in the closure.\n")
        string(APPEND _c
            "set(NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE ${NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE})\n")
        string(APPEND _c "\n")
        string(APPEND _c
            "# ---- the three PAYLOAD-CLASS knobs, and the set they were derived over ----\n")
        string(APPEND _c
            "set(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS \"${NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS}\")\n")
        if(NOT NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS STREQUAL "derived")
            string(REPLACE "\\" "\\\\" _pr "${NROS_MESSAGE_BOUNDS_PAYLOAD_REASON}")
            string(REPLACE "\"" "\\\"" _pr "${_pr}")
            string(REPLACE "\n" "\\n" _pr "${_pr}")
            string(APPEND _c "set(NROS_MESSAGE_BOUNDS_PAYLOAD_REASON \"${_pr}\")\n")
            string(APPEND _c
                "# NROS_SUBSCRIBER_BUFFER_SIZE, NROS_SUBSCRIBER_LARGE_SIZE and\n"
                "# NROS_MAX_LARGE_SUBSCRIBERS are NOT derived and do NOT fall back to the\n"
                "# closure. This image states what it receives, so the classes come from\n"
                "# that or from nowhere; each keeps its configured value.\n")
        else()
            string(APPEND _c
                "set(NROS_MESSAGE_BOUNDS_BASIS \"${NROS_MESSAGE_BOUNDS_BASIS}\")\n")
            if(NROS_MESSAGE_BOUNDS_BASIS STREQUAL "subscribed")
                string(APPEND _c
                    "# Derived over the ${NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT} SUBSCRIPTIONS this image declares\n"
                    "# (`nano_ros_node_register(... ENTITIES ...)`), not over the\n"
                    "# ${NROS_MESSAGE_BOUNDS_TYPE_COUNT}-type linked closure. A type the image links and never\n"
                    "# receives cannot set a class here.\n")
            else()
                string(APPEND _c
                    "# UPPER BOUND: this image declares no entities, so the classes are derived\n"
                    "# over the whole ${NROS_MESSAGE_BOUNDS_TYPE_COUNT}-type linked closure and are sized for the largest\n"
                    "# type it COULD receive, not the largest it does. Declare\n"
                    "# `nano_ros_node_register(... ENTITIES sub:<pkg>/msg/<Name> ...)` on every\n"
                    "# component to narrow them.\n")
            endif()
        endif()
        if(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS STREQUAL "derived")
            if(DEFINED NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE)
                string(APPEND _c
                    "# The largest received type at or under the class split.\n")
                string(APPEND _c
                    "set(NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE ${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE})\n")
            else()
                string(APPEND _c
                    "# Nothing this image receives fits under the ${_ceiling} B split, so the small\n"
                    "# class size is not derivable from that set -- but the class is still used,\n"
                    "# by any caller that states no hint. NROS_SUBSCRIBER_BUFFER_SIZE keeps its\n"
                    "# configured value.\n")
            endif()
            string(APPEND _c
                "# Received types over the split: ${NROS_DERIVED_LARGE_TYPES}\n")
            string(APPEND _c
                "set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS ${NROS_DERIVED_MAX_LARGE_SUBSCRIBERS})\n")
            if(DEFINED NROS_DERIVED_SUBSCRIBER_LARGE_SIZE)
                string(APPEND _c
                    "set(NROS_DERIVED_SUBSCRIBER_LARGE_SIZE ${NROS_DERIVED_SUBSCRIBER_LARGE_SIZE})\n")
            else()
                string(APPEND _c
                    "# Zero large-class blocks, so the pool is zero bytes whatever size it\n"
                    "# would name -- NROS_SUBSCRIBER_LARGE_SIZE is deliberately not derived.\n")
            endif()
        endif()
    endif()
    set(_write TRUE)
    if(EXISTS "${_path}")
        file(READ "${_path}" _existing)
        if(_existing STREQUAL _c)
            set(_write FALSE)
        endif()
    endif()
    if(_write)
        get_filename_component(_dir "${_path}" DIRECTORY)
        file(MAKE_DIRECTORY "${_dir}")
        file(WRITE "${_path}" "${_c}")
    endif()
endfunction()

# -----------------------------------------------------------------------------
# `cmake -P` entry point.
#
# Running the derivation without configuring a project is what makes it
# TESTABLE, and it is how W6's prototype was measured in the first place. Under
# `cmake -P` the script mode is on and `CMAKE_ARGC`/`CMAKE_ARGV*` carry the
# arguments:
#
#   cmake -DNROS_BOUNDS_FRAGMENTS="a.cmake;b.cmake" \
#         [-DNROS_BOUNDS_CEILING=2048] [-DNROS_BOUNDS_OUTPUT=out.cmake] \
#         [-DNROS_BOUNDS_ENTITY_INVENTORY=entity_inventory.cmake] \
#         -P cmake/NanoRosMessageBounds.cmake
# -----------------------------------------------------------------------------
if(CMAKE_SCRIPT_MODE_FILE AND
   CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    if(NOT DEFINED NROS_BOUNDS_FRAGMENTS)
        message(FATAL_ERROR
            "usage: cmake -DNROS_BOUNDS_FRAGMENTS=\"a.cmake;b.cmake\" "
            "[-DNROS_BOUNDS_CEILING=N] [-DNROS_BOUNDS_OUTPUT=path] "
            "-P cmake/NanoRosMessageBounds.cmake")
    endif()
    set(_args FRAGMENTS ${NROS_BOUNDS_FRAGMENTS})
    if(DEFINED NROS_BOUNDS_CEILING)
        list(APPEND _args SMALL_CLASS_CEILING "${NROS_BOUNDS_CEILING}")
    endif()
    if(DEFINED NROS_BOUNDS_OUTPUT)
        list(APPEND _args OUTPUT_FILE "${NROS_BOUNDS_OUTPUT}")
    endif()
    if(DEFINED NROS_BOUNDS_ENTITY_INVENTORY)
        list(APPEND _args ENTITY_INVENTORY "${NROS_BOUNDS_ENTITY_INVENTORY}")
    endif()
    nros_derive_message_bound_knobs(${_args})
    message(STATUS "NROS_MESSAGE_BOUNDS_STATUS=${NROS_MESSAGE_BOUNDS_STATUS}")
    foreach(_v
        NROS_MESSAGE_BOUNDS_TYPE_COUNT
        NROS_MESSAGE_BOUNDS_BOUNDED_COUNT
        NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS
        NROS_MESSAGE_BOUNDS_BASIS
        NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT
        NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE
        NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE
        NROS_DERIVED_MAX_LARGE_SUBSCRIBERS
        NROS_DERIVED_SUBSCRIBER_LARGE_SIZE
        NROS_DERIVED_LARGEST_TYPE
        NROS_DERIVED_LARGE_TYPES)
        if(DEFINED ${_v})
            message(STATUS "${_v}=${${_v}}")
        endif()
    endforeach()
    if(NOT NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS STREQUAL "derived")
        message(STATUS
            "NROS_MESSAGE_BOUNDS_PAYLOAD_REASON=${NROS_MESSAGE_BOUNDS_PAYLOAD_REASON}")
    endif()
endif()
