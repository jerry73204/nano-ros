# cmake/toolchain/NanoRosCrossToolchain.cmake
#
# WHICH cross compiler a build picked, WHERE it came from, and whether it is
# the one this tree pins — printed, every configure. Issue 1117.
#
# WHY THIS FILE EXISTS
#
# `nros-sdk-index.toml` pins `[tool.arm-none-eabi-gcc]` at ARM's 13.2.rel1. On
# a host whose SDK store had never been provisioned with it, `nros build
# freertos` compiled happily against Ubuntu's `gcc-arm-none-eabi` package —
# `/usr/bin/arm-none-eabi-g++` 10.3.1, three major versions below the pin — and
# said NOTHING about the substitution. The build then failed inside a generated
# C++ TU, and the failure was diagnosed twice as a nano-ros codegen bug
# (docs/issues/archived/1113-*.md, retracted) before anyone asked which
# compiler was running. A change to the entry emitter was nearly made to
# accommodate a compiler this project does not ship.
#
# THE FALLBACK IS NOT THE DEFECT
#
# `activate.sh` documents the system cross-gcc fallback on purpose: "A system
# cross-gcc (e.g. /usr/bin/arm-none-eabi-gcc) still resolves when the store has
# none." A contributor with a working distro toolchain should not have to
# download 150 MB to build. That stays. What changes is that the choice is now
# RECORDED — the same remedy issue 0500 landed for Corrosion, where the
# `nano-ros: Corrosion <ver> via <origin>` line is the only evidence of which
# copy won, and issue 0774 wanted one layer lower for `libzenohc`.
#
# TWO THINGS THIS ALSO FIXES, which are not "printing"
#
#  1. The STORE now beats PATH. Before, resolution was CMake looking up a bare
#     `arm-none-eabi-gcc` on `PATH`, and the store's bin dir only reaches PATH
#     via `activate.sh` — which computed it at SOURCE time. So `nros setup
#     --tool arm-none-eabi-gcc` followed by a build IN THE SAME SHELL still
#     used `/usr/bin`, and looked exactly like provisioning having no effect.
#     Resolving the store directly makes provisioning take effect immediately
#     and makes the pin win over an unpinned distro copy. `riscv64-threadx
#     .cmake` already did this for riscv; it is now the shared behaviour.
#  2. Newest version first within the store, because the store ACCUMULATES
#     (issue 0500) and a stale copy must not shadow the pinned one.
#
# HOW A USER CHOOSES A TOOLCHAIN
#
# Resolution order, highest priority first:
#
#   1. `-DNROS_<PREFIX>_PREFIX=…` or the environment variable of the same name,
#      set to a compiler PREFIX — either a bare command prefix on PATH
#      (`arm-none-eabi`) or an absolute one (`/opt/gcc/bin/arm-none-eabi`).
#      This is the explicit knob; before issue 1117 the only such knob in the
#      tree was `NROS_RISCV64_PREFIX`, undocumented outside its own file.
#   2. The SDK store — `$NROS_SDK_STORE`, else `$NROS_HOME/sdk`, else
#      `~/.nros/sdk` — under `<tool>/<version>/bin/`, newest version first.
#      Filled by `nros setup --tool <tool>`.
#   3. `PATH`, via `find_program` (the documented distro fallback).
#
# THE FLOOR, and why it is fatal rather than a warning
#
# See `NROS_CROSS_GCC_FLOOR_MAJOR` below. Measured, not assumed.

# ---------------------------------------------------------------------------
# The oldest GCC whose C++ frontend can compile what this tree GENERATES.
#
# MEASURED on the file that started issue 1117 — a designated initializer whose
# member is a `char[N]` initialised from a string literal, which is what
# `codegen/entry/mod.rs` emits for a node name:
#
#     struct C { const char* topic; char node_name[8]; int depth; };
#     static const C c = { .topic = "t", .node_name = "abc", .depth = 3 };
#
#   arm-none-eabi-g++ 10.3.1   error: C99 designator 'node_name' outside
#                              aggregate initializer — at -std=c++14, c++17
#                              AND c++20 alike. The standard is not the
#                              variable; the frontend is.
#   g++ 11.4.0 (host)          accepts it at every one of those standards
#   arm-none-eabi-g++ 13.2.1   accepts it at every one of those standards
#
# The bug is in GCC's C++ FRONTEND, which is target-independent, so the host
# g++ 11.4 measurement is evidence about riscv and arm equally — that is why
# one constant serves every cross gcc here rather than one per architecture.
#
# A WARNING, NOT A REFUSAL — and the choice is about what was MEASURED.
#
# What is measured is that this compiler cannot build the generated C++ ENTRY.
# What is NOT measured is a pure-C cross image on GCC 10: `nros-cpp` is a Rust
# crate compiled by cargo, so a C-only workspace may hand the cross g++ nothing
# that trips the frontend bug, and refusing at configure would take away a build
# that plausibly works today. `activate.sh` documents the system cross-gcc
# fallback deliberately; a floor that turns "supported fallback" into "cannot
# build at all" for the very host that reported issue 1117 — stock Ubuntu 22.04,
# whose `gcc-arm-none-eabi` is 10.3.1 — would resolve the tension by fiat in the
# wrong direction.
#
# So the warning does the one thing that actually costs nothing and would have
# saved issue 1113 outright: it QUOTES THE ERROR THE USER IS ABOUT TO GET. The
# entire cost of 1117 was a compile error that did not name its cause; printing
# the error text before the compile, next to the version and the pin, makes the
# match instant whichever direction someone searches from.
#
# Promoting this to FATAL_ERROR is a one-constant change, and the evidence that
# would justify it is specific: a measurement showing that every nano-ros cross
# image — C-only included — feeds a generated C++ TU to this compiler.
set(NROS_CROSS_GCC_FLOOR_MAJOR 11)

# ---------------------------------------------------------------------------
# The SDK store root. `NROS_SDK_STORE` wins (nros-build-paths and
# scripts/build/riscv64-toolchain.sh both honour it); `NROS_HOME` is what
# activate.sh reads; `~/.nros` is the default both agree on.
function(_nros_ct_store_root out_var)
    if(DEFINED ENV{NROS_SDK_STORE})
        set(${out_var} "$ENV{NROS_SDK_STORE}" PARENT_SCOPE)
    elseif(DEFINED ENV{NROS_HOME})
        set(${out_var} "$ENV{NROS_HOME}/sdk" PARENT_SCOPE)
    else()
        set(${out_var} "$ENV{HOME}/.nros/sdk" PARENT_SCOPE)
    endif()
endfunction()

# The `[tool.<name>]` pin from nros-sdk-index.toml — `version` (the repackaged
# id) and `upstream` (the vendor release the pin repackages).
#
# Read from the index rather than restated here on purpose: an AUTHORED copy of
# a pinned version is a map that drifts from the territory, which is the failure
# `check-rmw-api-parity` records at length. There is nothing to keep in sync
# because there is no second copy.
function(_nros_ct_pinned tool out_version out_upstream)
    set(${out_version} "" PARENT_SCOPE)
    set(${out_upstream} "" PARENT_SCOPE)
    set(_index "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../nros-sdk-index.toml")
    if(NOT EXISTS "${_index}")
        return()
    endif()
    file(READ "${_index}" _txt)
    # The section runs from its header to the next line that STARTS a table.
    # `[^\[]*` would stop at the `smoke = [` array inside the section.
    string(REPLACE "." "\\." _tool_re "${tool}")
    string(REGEX MATCH "\n\\[tool\\.${_tool_re}\\]\n(([^\n\\[][^\n]*)?\n)*" _sec "\n${_txt}")
    if(NOT _sec)
        return()
    endif()
    if(_sec MATCHES "\nversion[ \t]*=[ \t]*\"([^\"]*)\"")
        set(${out_version} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    endif()
    if(_sec MATCHES "\nupstream[ \t]*=[ \t]*\"([^\"]*)\"")
        set(${out_upstream} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    endif()
endfunction()

# ---------------------------------------------------------------------------
# nros_cross_toolchain_resolve(
#     TOOL         <sdk-index tool name, e.g. arm-none-eabi-gcc>
#     PREFIXES     <cmd prefix> [<alternate> ...]   # first is canonical
#     OVERRIDE_VAR <NROS_..._PREFIX>                # cache var or env var
#     OUT_PREFIX   <var>                            # resolved compiler prefix
#     OUT_ORIGIN   <var>)                           # where it came from
#
# OUT_ORIGIN is one of: `override <VAR>`, `SDK store`, `system PATH`,
# `NOT FOUND`. It is the half of the answer that was missing.
function(nros_cross_toolchain_resolve)
    cmake_parse_arguments(_A "" "TOOL;OVERRIDE_VAR;OUT_PREFIX;OUT_ORIGIN" "PREFIXES" ${ARGN})
    list(GET _A_PREFIXES 0 _canonical)

    # 1. Explicit override — a `-D` cache entry or the environment variable of
    #    the same name. Named, so a build that uses one says so.
    set(_ov "")
    if(DEFINED ${_A_OVERRIDE_VAR} AND NOT "${${_A_OVERRIDE_VAR}}" STREQUAL "")
        set(_ov "${${_A_OVERRIDE_VAR}}")
    elseif(DEFINED ENV{${_A_OVERRIDE_VAR}} AND NOT "$ENV{${_A_OVERRIDE_VAR}}" STREQUAL "")
        set(_ov "$ENV{${_A_OVERRIDE_VAR}}")
    endif()
    if(NOT "${_ov}" STREQUAL "")
        set(${_A_OUT_PREFIX} "${_ov}" PARENT_SCOPE)
        set(${_A_OUT_ORIGIN} "override ${_A_OVERRIDE_VAR}" PARENT_SCOPE)
        return()
    endif()

    # 2. The SDK store, newest version first — the store ACCUMULATES (issue
    #    0500), and `find_package`-style "first that resolves" over an
    #    ascending sort is how a stale copy shadows the pin.
    _nros_ct_store_root(_store_root)
    set(_store "${_store_root}/${_A_TOOL}")
    if(IS_DIRECTORY "${_store}")
        file(GLOB _vers RELATIVE "${_store}" "${_store}/*")
        list(SORT _vers COMPARE NATURAL ORDER DESCENDING)
        foreach(_v IN LISTS _vers)
            foreach(_p IN LISTS _A_PREFIXES)
                if(EXISTS "${_store}/${_v}/bin/${_p}-gcc")
                    set(${_A_OUT_PREFIX} "${_store}/${_v}/bin/${_p}" PARENT_SCOPE)
                    set(${_A_OUT_ORIGIN} "SDK store" PARENT_SCOPE)
                    return()
                endif()
            endforeach()
        endforeach()
    endif()

    # 3. PATH — the documented distro fallback (activate.sh, near the SDK store
    #    PATH block). Supported; simply no longer silent.
    foreach(_p IN LISTS _A_PREFIXES)
        unset(_nros_ct_found CACHE)
        find_program(_nros_ct_found "${_p}-gcc")
        if(_nros_ct_found)
            unset(_nros_ct_found CACHE)
            set(${_A_OUT_PREFIX} "${_p}" PARENT_SCOPE)
            set(${_A_OUT_ORIGIN} "system PATH" PARENT_SCOPE)
            return()
        endif()
        unset(_nros_ct_found CACHE)
    endforeach()

    # Nothing anywhere. Keep the canonical spelling so CMake's own "compiler
    # not found" names a real package instead of an empty string.
    set(${_A_OUT_PREFIX} "${_canonical}" PARENT_SCOPE)
    set(${_A_OUT_ORIGIN} "NOT FOUND" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# nros_cross_toolchain_report(TOOL <t> PREFIX <resolved> ORIGIN <o>
#                             OVERRIDE_VAR <NROS_..._PREFIX>)
#
# OVERRIDE_VAR is carried through only so the remedies name the REAL knob. A
# message that says "set -DNROS_..._PREFIX" is a message that has not told
# anyone anything.
#
# Prints the line that did not exist, and enforces the floor.
function(nros_cross_toolchain_report)
    cmake_parse_arguments(_A "" "TOOL;PREFIX;ORIGIN;OVERRIDE_VAR" "" ${ARGN})

    # A toolchain file is re-read for every try_compile. Report from the real
    # configure only; the probes would print the same line N times and their
    # output is discarded unless they fail.
    get_property(_in_try_compile GLOBAL PROPERTY IN_TRY_COMPILE)
    if(_in_try_compile)
        return()
    endif()

    # …and it is read once per LANGUAGE enabled, so a C+CXX project reads it
    # twice in the same process. A global property is the memo that survives
    # both re-reads and any nested project() call; a normal or cache variable
    # would not (the file is re-read into a fresh scope, and a cache write from
    # a toolchain file is the kind of stickiness issue 0500 warns about).
    string(MAKE_C_IDENTIFIER "${_A_TOOL}" _memo)
    get_property(_seen GLOBAL PROPERTY "_NROS_CT_REPORTED_${_memo}")
    if(_seen)
        return()
    endif()
    set_property(GLOBAL PROPERTY "_NROS_CT_REPORTED_${_memo}" TRUE)

    set(_gcc "${_A_PREFIX}-gcc")
    set(_ver "unknown")
    if(NOT "${_A_ORIGIN}" STREQUAL "NOT FOUND")
        execute_process(COMMAND "${_gcc}" -dumpfullversion
                        OUTPUT_VARIABLE _ver_out RESULT_VARIABLE _rc
                        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(NOT _rc EQUAL 0)
            # gcc < 7 has no -dumpfullversion; -dumpversion is the old spelling.
            execute_process(COMMAND "${_gcc}" -dumpversion
                            OUTPUT_VARIABLE _ver_out RESULT_VARIABLE _rc
                            OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        endif()
        if(_rc EQUAL 0 AND NOT "${_ver_out}" STREQUAL "")
            set(_ver "${_ver_out}")
        endif()
    endif()

    _nros_ct_pinned("${_A_TOOL}" _pin_ver _pin_upstream)
    set(_pin_text "unknown — no [tool.${_A_TOOL}] in nros-sdk-index.toml")
    if(NOT "${_pin_upstream}" STREQUAL "")
        set(_pin_text "${_pin_upstream}, store package ${_pin_ver}")
    elseif(NOT "${_pin_ver}" STREQUAL "")
        set(_pin_text "${_pin_ver}")
    endif()

    # THE LINE. Same shape as `nano-ros: Corrosion <ver> via <origin>` — the
    # 0500 remedy, one layer down.
    message(STATUS
        "nano-ros: ${_A_TOOL} ${_ver} via ${_A_ORIGIN} — ${_gcc} (pin ${_pin_text})")

    # ---------------------------------------------------------------------
    # A BUILD TREE ALREADY LOCKED TO A DIFFERENT COMPILER.
    #
    # This is issue 1117 reappearing one level up, and it is worse than the
    # original because the line above is then a LIE. CMake bakes the chosen
    # compiler into `CMakeFiles/<ver>/CMakeCXXCompiler.cmake` on the first
    # configure and reloads it on every later one; the toolchain file's plain
    # `set(CMAKE_C_COMPILER …)` is a NORMAL variable and does not displace it.
    # Measured on the tree that found 1117: after this module landed, a
    # re-configure of the existing build dir printed
    #     nano-ros: arm-none-eabi-gcc 13.2.1 via SDK store …
    # while every object in that directory was still being compiled by
    # /usr/bin/arm-none-eabi-g++ 10.3.1, with no error from CMake at all.
    #
    # On a re-configure the persisted value is already in scope when the
    # toolchain file is read (it is empty on a first configure), so the
    # disagreement is detectable exactly here.
    #
    # FATAL, and this is one of the two states where "use a fresh build
    # directory" is the honest answer rather than the `rm -rf` antipattern:
    # CMake itself declares a compiler change unrecoverable in place, and no
    # dependency edge can express it.
    foreach(_lang_pair "C:gcc" "CXX:g++")
        string(REPLACE ":" ";" _lp "${_lang_pair}")
        list(GET _lp 0 _lang)
        list(GET _lp 1 _suffix)
        # Read the PERSISTED file, not the variable. There is normally no
        # `CMAKE_CXX_COMPILER` cache entry for a cross build: the toolchain
        # file sets it as a NORMAL variable, so CMakeDetermine<LANG>Compiler
        # records the result only in CMakeFiles/<ver>/CMake<LANG>Compiler.cmake
        # and the variable is empty again the next time this file is read.
        # Checking the variable therefore looked correct and detected nothing —
        # verified against the very build tree that motivated the check.
        set(_have "")
        file(GLOB _cc_files
             "${CMAKE_BINARY_DIR}/CMakeFiles/*/CMake${_lang}Compiler.cmake")
        foreach(_cc_file IN LISTS _cc_files)
            file(READ "${_cc_file}" _cc_txt)
            if(_cc_txt MATCHES "set\\(CMAKE_${_lang}_COMPILER \"([^\"]+)\"")
                set(_have "${CMAKE_MATCH_1}")
            endif()
        endforeach()
        if("${_have}" STREQUAL "" AND NOT "${CMAKE_${_lang}_COMPILER}" STREQUAL "")
            set(_have "${CMAKE_${_lang}_COMPILER}")
        endif()
        if("${_have}" STREQUAL "")
            continue()
        endif()
        set(_want "${_A_PREFIX}-${_suffix}")
        if(NOT IS_ABSOLUTE "${_want}")
            unset(_nros_ct_want CACHE)
            find_program(_nros_ct_want "${_want}")
            if(_nros_ct_want)
                set(_want "${_nros_ct_want}")
            endif()
            unset(_nros_ct_want CACHE)
        endif()
        get_filename_component(_have_real "${_have}" REALPATH)
        get_filename_component(_want_real "${_want}" REALPATH)
        if(NOT "${_have_real}" STREQUAL "${_want_real}")
            message(FATAL_ERROR
                "nano-ros: this build directory is already configured with a DIFFERENT "
                "${_lang} compiler.\n"
                "  build tree uses:  ${_have}\n"
                "  toolchain resolves: ${_want}  (via ${_A_ORIGIN})\n"
                "\n"
                "  CMake bakes the compiler into CMakeFiles/<ver>/CMake${_lang}Compiler.cmake on\n"
                "  the first configure and reloads it afterwards, so it CANNOT be changed in\n"
                "  place — and it does not complain, which is how issue 1117 happens twice:\n"
                "  the provenance line above would describe a compiler nothing here uses.\n"
                "  Configure into a FRESH build directory (or delete this one) so the\n"
                "  resolved toolchain is the one that compiles.\n"
                "  If the build tree's compiler is the one you want, select it explicitly:\n"
                "      -D${_A_OVERRIDE_VAR}=<prefix of ${_have}>")
        endif()
    endforeach()

    if("${_A_ORIGIN}" STREQUAL "NOT FOUND")
        message(FATAL_ERROR
            "nano-ros: no ${_A_TOOL} found.\n"
            "  Provision the pinned one:  nros setup --tool ${_A_TOOL}\n"
            "  Or point at your own:      -D${_A_OVERRIDE_VAR}=/path/to/bin/<prefix>\n"
            "                             (the same name works as an environment variable)")
    endif()

    # Off-pin is SUPPORTED and stays supported — it is told, not refused.
    # NOTICE rather than STATUS so it survives a build log: this is the
    # sentence whose absence cost issue 1113 two wrong diagnoses.
    if(NOT "${_A_ORIGIN}" STREQUAL "SDK store")
        if(_A_ORIGIN MATCHES "^override ")
            set(_why "you selected it explicitly")
        else()
            set(_why "the documented system-toolchain fallback")
        endif()
        message(NOTICE
            "nano-ros: ${_A_TOOL} is UNPINNED — ${_ver} from ${_A_ORIGIN}, not the SDK store.\n"
            "          This tree pins ${_pin_text}, per nros-sdk-index.toml.\n"
            "          Supported (${_why}); this is a notice, not an error.\n"
            "          To use the pin instead:  nros setup --tool ${_A_TOOL}")
    endif()

    if(_ver MATCHES "^([0-9]+)")
        set(_major "${CMAKE_MATCH_1}")
        if(_major LESS NROS_CROSS_GCC_FLOOR_MAJOR)
            # WARNING and not FATAL_ERROR — see the floor's own comment above.
            # The text quotes the error the user is ABOUT to get, because the
            # whole cost of issue 1117 was that the error, when it arrived, did
            # not name its cause and looked like a nano-ros bug.
            message(WARNING
                "nano-ros: ${_A_TOOL} ${_ver} (via ${_A_ORIGIN}) is below the floor of "
                "GCC ${NROS_CROSS_GCC_FLOOR_MAJOR}.\n"
                "  It CANNOT compile the C++ entry TU nano-ros generates. If this image has a\n"
                "  C++ entry, the build will fail later with\n"
                "      error: C99 designator '<member>' outside aggregate initializer\n"
                "  in *_entry_nros_main_generated.cpp. THAT ERROR IS THIS — a GCC ${_major} C++\n"
                "  frontend limitation, not a codegen bug (issue 1117; the wrong diagnosis it\n"
                "  caused is issue 1113). Raising -std= does NOT help: GCC 10 rejects it at\n"
                "  c++14, c++17 and c++20 alike, and GCC 11+ accepts it at all three.\n"
                "  Fix:  nros setup --tool ${_A_TOOL}      (installs ${_pin_text})\n"
                "  Or:   -D${_A_OVERRIDE_VAR}=/path/to/bin/<prefix>   (also honoured as an env var)")
        endif()
    endif()
endfunction()
