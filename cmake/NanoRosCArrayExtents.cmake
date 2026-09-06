# NanoRosCArrayExtents.cmake -- issue 1015: a number that sizes a FIXED C ARRAY
# is refused at CONFIGURE if it is not usable, naming the knob and where the
# answer was supposed to come from.
#
# =============================================================================
# The gap this closes
# =============================================================================
#
# phase-403's rule is that a derived value carries NO headroom: it is exactly
# the declared demand, so the running image checks its own declaration. That is
# right for a table the executor INDEXES -- registering past the end returns
# `ExecutorFull`, which names the knob.
#
# It is wrong where the number sizes a fixed C array. There, zero is not a
# smaller pool; it is a DIFFERENT KIND OF OBJECT. A zero-length array is a GNU
# extension that compiles silently, and the zpico session's pools are not the
# last member of their struct, so a zero changes the layout instead of shrinking
# a table. Measured on the reference island, which declares no service servers
# and therefore derived `MAX_QUERYABLES = 0`: the board transmitted NOTHING in
# 15 s -- no panic, no log line, no fault, core in WFI. The same image at 4
# transmitted 110 bytes.
#
# Every gate in the tree was green, and correctly so. `check-knob-delivery`
# confirms the value ARRIVED -- it did; 0 was derived faithfully.
# `check-knob-fixpoint` converges, because 0 is stable. Every one of them asks
# whether the number the image was BUILT with is the number that was DERIVED.
# None asked whether the number is USABLE. This asks that, at the earliest
# point that can: before a compiler is invoked, while the person who set the
# knob is still standing there.
#
# =============================================================================
# Where this sits among the producers
# =============================================================================
#
# One rule, four enforcement points, deliberately -- a floor in one producer
# does not bind another, and the pools have three producers that never see each
# other:
#
#   1. the DERIVATION       `nros-cli-core` `entity_inventory.rs` floors the
#                           pools it derives at 1.
#   2. the CMAKE lane       here. Binds a hand-set `.conf`, a `-D` on the
#                           configure line, an environment override, and the
#                           derivation, because it checks the value that is
#                           about to become a `-D` rather than where it came
#                           from.
#   3. the CARGO lane       `ShimConfig::check_c_array_extents`, for a build
#                           with no cmake in it at all (a `fixtures.toml` row's
#                           `env = { ... }`, a shell export).
#   4. the C SOURCE         `#if <M> < 1` / `#error` beside the arrays. The
#                           backstop that binds a producer written tomorrow.
#
# `scripts/check-c-array-extent-floors.py` holds 2, 3 and 4 to the array
# declarations in `zpico.c`, so a pool added later cannot ship unguarded.
#
# =============================================================================
# Usage
# =============================================================================
#
#   include(NanoRosCArrayExtents.cmake)     # FILE scope, never inside a
#                                           # function frame
#   nros_assert_c_array_extent(
#       MACRO   ZPICO_MAX_QUERYABLES        # the C macro about to be -D'd
#       VALUE   "${NROS_RESOLVED_ZPICO_MAX_QUERYABLES}"
#       ARRAY   "queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]"
#       [KCONFIG  CONFIG_NROS_MAX_QUERYABLES]   # who states it, if anyone
#       [DECLARE  "prose: what the image was supposed to declare"])
#
# `include_guard(GLOBAL)` plus function-only content: nothing here is a normal
# variable, so the `_NROS_ENTRY_DIR` hazard (an `include()` inside a function
# frame losing the file's `set()`s when the frame pops) does not apply.
include_guard(GLOBAL)

# nros_assert_c_array_extent(MACRO <m> VALUE <v> ARRAY <a> [KCONFIG <k>]
#                            [DECLARE <prose>])
#
# FATAL_ERROR unless <v> is an integer >= 1.
#
# THREE illegal states, not one, because they have different causes and the
# message has to name the right one:
#
#   EMPTY   -- the knob resolved to nothing and the caller applied no default.
#              `-DZPICO_MAX_SUBSCRIBERS=` reaches the compiler and `zpico.c`
#              reports `flexible array member not at end of struct` on a struct
#              nobody edited. That diagnostic names neither the knob nor the
#              build step, which is why this one is worth its own arm.
#   NOT A NUMBER -- including the `-1` DERIVE SENTINEL escaping its resolver.
#              `-1` sizes the array NEGATIVE; gcc says `size of array is
#              negative`, again naming nothing.
#   ZERO or less -- issue 1015 itself.
function(nros_assert_c_array_extent)
    cmake_parse_arguments(_X "" "MACRO;VALUE;ARRAY;KCONFIG;DECLARE" "" ${ARGN})
    if(NOT _X_MACRO)
        message(FATAL_ERROR
            "nros_assert_c_array_extent() needs MACRO -- this is a bug in the "
            "caller, not in the image being built.")
    endif()

    # The tail every arm shares: WHERE the number was supposed to come from.
    # Built once so the three messages cannot drift apart.
    set(_where "")
    if(_X_ARRAY)
        string(APPEND _where "\n  It sizes: ${_X_ARRAY}")
    endif()
    if(_X_KCONFIG)
        string(APPEND _where
            "\n  State it directly with ${_X_KCONFIG} (a stated number always "
            "wins over the derivation).")
    endif()
    if(_X_DECLARE)
        string(APPEND _where "\n  ${_X_DECLARE}")
    endif()

    if("${_X_VALUE}" STREQUAL "")
        message(FATAL_ERROR
            "nros: ${_X_MACRO} resolved to NOTHING, and it sizes a fixed C array.\n"
            "  An empty value reaches the compiler as `-D${_X_MACRO}=`, which "
            "fails inside zpico.c as `flexible array member not at end of "
            "struct` -- on a struct nobody edited, naming neither this knob nor "
            "the step that dropped it.\n"
            "  A knob left UNRESOLVED is legal only where the consumer supplies "
            "its own default (rung 4 of the ladder in nros_cargo_build.cmake). "
            "A C `-D` has no default of its own, so this consumer is the "
            "exception and must be given a number."
            "${_where}")
    endif()

    if(NOT "${_X_VALUE}" MATCHES "^-?[0-9]+$")
        message(FATAL_ERROR
            "nros: ${_X_MACRO}=${_X_VALUE} is not an integer, and it sizes a "
            "fixed C array.\n"
            "  Whatever produced it did not produce a size."
            "${_where}")
    endif()

    if(_X_VALUE LESS 1)
        # ONE argument, not several. `set(_x "a" "b")` makes a cmake LIST, and a
        # list interpolated into a message renders as `a;b` -- which is how the
        # explanation of a semicolon-free defect came to be full of semicolons.
        set(_extra "")
        if(_X_VALUE EQUAL -1)
            string(APPEND _extra
                "\n  -1 is the DERIVE SENTINEL and should never reach a "
                "compiler: it means \"nobody stated a number\", not \"size the "
                "array -1\". Reaching here means the knob was resolved from the "
                "raw Kconfig value instead of through "
                "_nros_resolve_derivable_knob(), or resolved twice with the "
                "plain resolver winning (check-knob-resolved-once).")
        endif()
        message(FATAL_ERROR
            "nros: ${_X_MACRO}=${_X_VALUE} is not a usable size (issue 1015).\n"
            "\n"
            "  A pool that backs a FIXED C ARRAY has a floor of ONE. Zero is "
            "not a smaller pool -- a zero-length array is a GNU extension that "
            "compiles SILENTLY, and these are not the last member of their "
            "struct, so a zero changes the struct's layout instead of shrinking "
            "a table.\n"
            "\n"
            "  This has been MEASURED, and it is the worst shape a defect can "
            "have: the image linked, every gate stayed green, and the board "
            "transmitted 0 bytes in 15 seconds -- no panic, no log line, no "
            "fault, core in WFI. The same image with the pool at 4 transmitted "
            "110 bytes.\n"
            "\n"
            "  A ZERO HERE IS ALMOST NEVER A DELIBERATE ZERO. It is a "
            "derivation that found nothing: the image's entity inventory "
            "resolved to an empty declaration, so the demand summed to 0 and "
            "was delivered faithfully."
            "${_where}"
            "${_extra}")
    endif()
endfunction()
