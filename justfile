set dotenv-load

# Workspace-wide clippy lint levels live in root `Cargo.toml` under
# `[workspace.lints]` (and per-crate `[lints] workspace = true`). The
# old `CLIPPY_LINTS` string passed through `--` is no longer needed.

# Opt-in rustc wrapper. When `sccache` is on `PATH`, every `cargo`
# invocation under any `just` recipe shares its compilation cache —
# big win across per-example builds that recompile the same
# `nros-core` / `heapless` / etc. crates over and over. When sccache
# is absent the variable is empty, which cargo treats as unset
# (verified on cargo 1.95).
export RUSTC_WRAPPER := `command -v sccache 2>/dev/null || true`

# Phase 165.perf — size the sccache disk cache for a full `build-all`
# sweep. The default 10 GiB evicts mid-sweep once the ~150 standalone
# example/fixture crates plus the Zephyr C objects (picolibc, kernel,
# Cyclone) land in the cache; 30 GiB holds a whole sweep. Only read at
# sccache server start, so it's harmless when sccache is absent.
export SCCACHE_CACHE_SIZE := "30G"

# Phase 165.perf — single global parallelism budget (total cores to
# use across a build). Defaults to nproc. Every parallel recipe reads
# `${NROS_BUILD_JOBS:-…}` for its inner make/cargo/ninja fan-out, so one
# knob scales the whole build:
#   just build-all                       # uses nproc
#   NROS_BUILD_JOBS=8 just build-all     # cap at 8 cores total
# `build-test-fixtures` runs N platforms concurrently and re-exports
# NROS_BUILD_JOBS = budget/N to each child so the product stays at the
# budget (no platform-count × inner-jobs oversubscription).
export NROS_BUILD_JOBS := env_var_or_default("NROS_BUILD_JOBS", `nproc 2>/dev/null || echo 8`)

# Cargo build profile for broad build recipes. Deliberately EMPTY by default
# (phase-336): the default lives in the profile table behind `nros profile`, and
# `scripts/build/cargo.sh` resolves it there. A literal here would be a fourth
# copy — and it would have to be evaluated at justfile PARSE time, so a wrong
# value could not even be corrected by the recipe that builds the CLI.
export NROS_CARGO_PROFILE := env_var_or_default("NROS_CARGO_PROFILE", "")

# User-local tools installed by setup modules (for example PlatformIO via
# pipx/pip --user) should be visible to all just-driven tests.
export PATH := env("HOME") / ".local/bin" + ":" + env_var_or_default("PATH", "")

LOG_DIR := "test-logs"

# Pinned nightly channel for workspace tooling (fmt, miri, llvm-cov, build-std, emit-stack-sizes).
# Source of truth: tools/rust-toolchain.toml. Read via awk so the version
# is never duplicated into build scripts.
NIGHTLY := `awk '/^channel/ {gsub(/"/, "", $3); print $3; exit}' tools/rust-toolchain.toml`

# Crates that cannot be checked for the HOST: `no_std` staticlibs (no
# panic_handler, unwinding unsupported) and build-time helpers. Defined once so
# `check::workspace` and `check::test-targets` cannot drift apart — a bare
# `--workspace` check fails on these with "`#[panic_handler]` function required".

import "just/sdk-env.just"
# phase-399 W1 — the 200 gate recipes. `import`, not `mod`: names stay flat
# (`just check fast`, not `just check fast`), so no call site anywhere in the
# tree changes. See docs/roadmap/phase-399-justfile-surface-and-event-design.md.
mod check 'just/check.just'

# =============================================================================
# Platform modules (just <platform> <recipe>)
# =============================================================================

mod freertos 'just/freertos.just'
mod nuttx 'just/nuttx.just'
mod threadx_linux 'just/threadx-linux.just'
mod threadx_riscv64 'just/threadx-riscv64.just'
mod zephyr 'just/zephyr.just'
mod esp32 'just/esp32.just'
mod esp_idf 'just/esp_idf.just'
mod qemu 'just/qemu-baremetal.just'
mod native 'just/native.just'
mod xrce 'just/xrce.just'
mod docker 'just/docker.just'
mod workspace 'just/workspace.just'
mod verification 'just/verification.just'
mod px4 'just/px4.just'
mod cyclonedds 'just/cyclonedds.just'
mod ros_editions 'just/ros-editions.just'
mod probe 'just/probe.just'

# =============================================================================
# Recipe organization (convention — keep new recipes consistent)
# =============================================================================
# Two axes:
#   * `mod <name>`  — namespaced platform/tool recipes: `just <name> <verb>`
#                     (native/zephyr/freertos/… build|test|build-fixtures|setup).
#   * `[group(...)]` — display category for ROOT recipes in `just --list`.
#
# Group taxonomy (root recipes):
#   main          headline dev loop: build, build-examples, check, format, test,
#                 test-unit, test-integration, doc.
#   ci            CI lanes + the local mirror of every standalone CI job — one
#                 recipe per workflow so CI yml is a thin `just <recipe>` caller:
#                 ci, ci-fast, check::no-std, check::sdk-index, scaffold-journey,
#                 colcon-parity, acceptance.  (See docs/development/ci-workflow-reorg.md.)
#   full-matrix   heavy build/test sweeps: build-all, build-test-fixtures, test-all.
#   verification  Kani/Verus formal verification.
#   docs          rust/C/C++/mdBook doc builds.
#   setup         provisioning entry points.
#   maintenance   clean/regenerate/version-bump.
#   debug         building blocks + diagnostics not part of the daily loop.
#
# Naming + visibility conventions:
#   * `check-*`  static/precondition gate; the individual gates are `[private]`
#               building blocks that the `check` aggregate chains. A gate that is
#               ALSO a useful standalone task (e.g. `check::no-std`) goes in `ci`.
#   * `test-*`   test runners.   `build-*` builds.   `ci` / `ci-fast` lane aggregates.
#   * Adding a CI job ⇒ add a matching recipe here (group `ci`) + call it from the
#     workflow yml. `just check` must stay a SUPERSET of the fast-gate workflow.
# =============================================================================

# phase-399 W2 — `just` with no argument answers ONE question: what do I type?
#
# It used to run `just --list`, which is 215 lines opening with a wall of
# `check-*` gates nobody types, each described by the LAST line of its rationale
# comment — so the descriptions were sentence fragments about something else.
# A listing that long is not an index, it is a wall, and the eight verbs that
# matter were invisible inside it.
#
# The full list is still one flag away (`just --list`), and a gate is still
# discovered the way gates are actually discovered: by failing, and printing its
# own name.
[group("main")]
default:
    #!/usr/bin/env bash
    # printf with one argument per line, NOT a heredoc: a heredoc
    # terminator must sit at column 0, and a column-0 line inside a recipe
    # is parsed as justfile syntax. Indentation is inside the quotes, so
    # nothing has to survive both `just`s body-dedent and a sed strip.
    printf "%s\n" \
        "nano-ros — the verbs you actually type" \
        "" \
        "  BEFORE YOU PUSH" \
        "    just format                rustfmt (nightly) + clang-format + python" \
        "    just ci l1                 compile + unit, no fixtures        ~6 min" \
        "                               ^ run this. STRICTER than the CI gate." \
        "" \
        "  WHEN SOMETHING IS WRONG" \
        "    just doctor                is this checkout/env set up correctly?" \
        "    just check fast            138 source gates, parallel          ~9 s" \
        "    just check tier-preconditions    what is stale, and why" \
        "    just post-rebase           after a rebase/pull: CLI, resolver, index" \
        "" \
        "  BIGGER TIERS  (CI runs these; you rarely need them)" \
        "    just ci                    tier 1 — host, builds fixtures" \
        "    just ci matrix             tier 2 — 1-wise platform cover" \
        "    just ci full               tier 3 — the whole matrix" \
        "" \
        "  A SCOPE  (phase-411 — one word, one position, every verb)" \
        "    just setup  <scope>        provision it" \
        "    just doctor <scope>        is it ready? (no scope: probe them all)" \
        "    just build  <scope>        build its test fixtures" \
        "    just test   <scope>        run its tests" \
        "" \
        "    scope = a platform   native zephyr freertos nuttx threadx_linux" \
        "                         threadx_riscv64 esp32 esp_idf qemu px4 xrce" \
        "                         cyclonedds" \
        "         or a preset     all native tier1 tier2 tier2-nightly" \
        "" \
        "    Naming a scope IS the specification: it is what you asked to be" \
        "    covered. Unnamed = best effort over what is provisioned, reported." \
        "    (\`just <plat> <verb>\` still works, deprecated for one release.)" \
        "" \
        "  SETUP" \
        "    source ./activate.sh       REQUIRED once per shell" \
        "    just setup-cli             rebuild the in-tree nros CLI" \
        "    just setup-hooks           git hooks + submodule legibility" \
        "" \
        "  everything else   just --list" \
        "  what CI runs      docs/roadmap/phase-399-justfile-surface-and-event-design.md"
# Show every recipe including private/internal ones.
# Maintainer/CI flow. End users want `just --list`.
[group("debug")]
list-all:
    #!/usr/bin/env bash
    set -e
    awk '
        # Skip attribute lines, comments, blank, indented (recipe bodies).
        /^[[:space:]]/ || /^#/ || /^\[/ || /^$/ { next }
        # Recipe head: "name[ params]:" — capture the name.
        /^[a-zA-Z_][a-zA-Z0-9_-]*([[:space:]]|:|\*)/ {
            n = $1
            sub(/:.*/, "", n)
            print n
        }
    ' justfile | sort -u
    echo ""
    echo "(Run \`just <name>\` for any of these. Public subset: \`just --list\`.)"

# =============================================================================
# Entry Points
# =============================================================================

# Build tiers (each tier is a strict superset of the previous):
#
#   build               workspace (native + embedded) + transports (zenohd, zenoh-pico).
#                       Fast — typical dev iteration.
#   build-examples      `build` + every example crate + per-RTOS example builds
#                       (native, freertos, threadx_linux, threadx_riscv64).
#                       Use to verify the example matrix compiles.
#   build-test-fixtures Per-test staged binaries: feature variants
#                       (--target-dir target-tls / target-safety / target-zero-copy
#                       / target-large-buf) and C / C++ fixture binaries built via
#                       cmake. Required before `just test-all`.
#   build-all           = build + non-fixture examples + fixture leaves.
#                       Slow — expect 15-40 min depending on machine.
#
# Default `build` recipe: refresh bindings + workspace + transports.
#
# Phase 140 — `install-local` removed; `add_subdirectory(<repo-root>)`
# is the only supported C/C++ consumption shape. CMake-driven crates
# build in-tree via Corrosion when an example invokes them.
#
# phase-411 W3 — `just build [<scope>…]`. A scope in the first argument
# position, the same word `setup`, `doctor` and `test` take:
#
#   just build tier2       the tier-2 fixture cover (was: build-test-fixtures lane=tier2)
#   just build native      every native fixture row     (was: lane=native)
#   just build zephyr      one platform's fixtures      (was: just zephyr build-fixtures)
#
# A LANE token routes through `build-test-fixtures`, because that is what
# writes the coverage stamp `_require-fixtures` reads; a PLATFORM token routes
# to `just <plat> build-fixtures`, which is the same call that recipe's own
# fan-out makes. Same scope, the machinery each already has.
#
# WITH NO SCOPE this stays the workspace build — deliberately NOT "the fixtures
# for everything provisioned". `just build` is the documented fast inner loop
# (book/src/internals/build-system.md), minutes not hours, and silently
# promoting it to a fixture sweep would be the most expensive surprise in the
# tree. What it gains instead is the derived default scope PRINTED, so the
# question "what would `just build <scope>` cover here?" is answered without
# anyone recording an answer.
#
# Workspace + transports; with a scope, that scope's test fixtures.
[group("main")]
build *scope:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/scope.sh
    source scripts/build/scope.sh
    scoped=({{scope}})
    # CODEGEN FIRST, for every scope (issue 0992). `colcon build` generates
    # messages; so does this. It used to be reachable only from the unscoped
    # path below, so `just build <scope>` — the spelling every CI job uses —
    # built against whatever `generated/` the tree happened to carry.
    just _codegen
    if [ "${#scoped[@]}" -gt 0 ]; then
        nros_scope_validate_all "${scoped[@]}" || exit 2
        nros_scope_report build "${scoped[@]}"
        for tok in "${scoped[@]}"; do
            just _build-scope "$tok"
        done
        exit 0
    fi
    # The pre-407 dependency list, called rather than depended on: a recipe
    # with a variadic parameter still runs its dependencies unconditionally,
    # and a scoped `just build zephyr` must not first rebuild the workspace.
    just build-workspace build-workspace-embedded
    just qemu build-zenoh-pico
    echo 'Workspace + transports built. Run "just build-examples" for example crates, "just build <scope>" for `test-all` fixture staging, or "just build-all" for everything.'
    echo ""
    nros_scope_report build

# One scope token's fixture build.
[private]
_build-scope tok:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/scope.sh
    source scripts/build/scope.sh
    tok="$(nros_scope_normalize "{{tok}}")"
    # LANE first, unlike `_test-scope`: a lane build stamps its coverage
    # (`nros_fixtures_stamp_write`), and `native` is a lane, so the stamped
    # path is the better answer for the one token that is both.
    if nros_scope_is_lane "$tok"; then
        nros_scope_exec just build-test-fixtures "lane=$tok"
        exit 0
    fi
    if nros_scope_is_platform "$tok"; then
        nros_scope_require_module_verb "$tok" build-fixtures
        echo "note: a single-platform build writes no fixture stamp; \`just test\` still" >&2
        echo "      wants a lane build (\`just build native|tier1|tier2|all\`)." >&2
        nros_scope_exec just "$tok" build-fixtures
        exit 0
    fi
    nros_scope_reject "$tok"

# `build` + every example crate + per-RTOS example builds (native,
# freertos, threadx_linux, threadx_riscv64). Use to verify the
# example matrix still compiles after a core change.
[group("main")]
build-examples: build \
    native::build-examples \
    freertos::build-examples threadx_linux::build-examples threadx_riscv64::build-examples
    @echo "Workspace + examples built."

# Internal build-all example tier. Public `build-examples` stays broad and
# convenient, but build-all must not call it because fixture tiers rebuild
# the same role examples for FreeRTOS, ThreadX, QEMU, and several native
# cases. This recipe only builds Cargo examples that are not already staged
# by platform fixture tiers.
[group("full-matrix")]
build-example-extras:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    export cargo_profile_args
    # issue 0635 — the target-dir resolver, sourced and EXPORTED here because
    # `build_one` is `export -f`'d into subshells that never source these files
    # (the same reason the profile flags are resolved here).
    # shellcheck source=scripts/build/fixtures-target-dir.sh
    source scripts/build/fixtures-target-dir.sh
    export -f nros_example_build_target_dir nros_fixture_group nros_fixture_group_slug \
              nros_fixture_platform_is_shared _nros_fixture_variant_sig \
              nros_build_root nros_build_dir
    if [ "${NROS_JOBSERVER:-}" = "1" ]; then
        cargo_frontends="$(nros_cargo_frontend_jobs)"
    else
        cargo_frontends="${NROS_BUILD_JOBS:-75%}"
    fi
    echo "Building build-all example extras (cargo-frontends=$cargo_frontends, profile=$(nros_cargo_profile_name))..."
    list="$(mktemp)"
    rg --files examples -g Cargo.toml \
        | sed 's#/Cargo.toml$##' \
        | grep -Ev '^examples/(zephyr|qemu-arm-freertos|qemu-arm-nuttx|threadx-linux|qemu-riscv64-threadx|qemu-arm-baremetal)/' \
        | grep -Ev '^examples/native/rust/(talker|listener|lifecycle-node|custom-msg|service-server|service-client|action-server|action-client|talker-rtic|listener-rtic|service-server-rtic|service-client-rtic|action-server-rtic|action-client-rtic|serial-talker|serial-listener)$' \
        | sort > "$list"

    build_one() {
        local dir="$1"
        local platform
        platform="$(echo "$dir" | cut -d/ -f2)"
        local env_prefix=""
        local toolchain=""
        if [ "$platform" = "esp32" ] || [ "$platform" = "qemu-esp32-baremetal" ]; then
            env_prefix="SSID=${SSID:-test} PASSWORD=${PASSWORD:-test}"
            toolchain="+{{NIGHTLY}}"
        fi
        echo "  build $dir"
        # issue 0635 — never the leaf's own `target/` (phase-340 P2). The
        # resolver answers with the platform's shared group when there is one,
        # so this walk reuses the fixture build instead of compiling a second
        # copy, and falls back to `build/example-build/<leaf>` for a leaf with
        # no coordinate to join.
        local tdir
        tdir="$(nros_example_build_target_dir "$dir")"
        ( cd "$dir" && eval $env_prefix cargo $toolchain build $cargo_profile_args --target-dir "$tdir" )
    }
    export -f build_one
    export NIGHTLY="{{NIGHTLY}}"

    # Issue 0466 — fan out under the JOBSERVER, not GNU parallel. cargo is a
    # jobserver client (rust-lang/cargo#4110), so N of them under one pool share
    # ONE token budget instead of each starting its own full-width build; `-P N`
    # on top of a self-parallelising tool is the classic multiplication. This
    # branch also used to collapse two DIFFERENT reasons into one silent serial
    # walk — an outer jobserver (correct) and a missing `parallel` (a degrade) —
    # so you could not tell which one you got. Only the first remains, and the
    # pool owns the rest of the degrade path itself.
    if [ "${NROS_JOBSERVER:-}" = "1" ]; then
        while read -r dir; do build_one "$dir"; done < "$list"
    else
        source scripts/build/jobserver-pool.sh
        units="$(mktemp "${TMPDIR:-/tmp}/nros_build_units.XXXXXX")"
        while read -r dir; do
            plat="$(echo "$dir" | cut -d/ -f2)"
            e=""; tc=""
            case "$plat" in
                esp32 | qemu-esp32-baremetal)
                    e="SSID=${SSID:-test} PASSWORD=${PASSWORD:-test}"; tc="+{{NIGHTLY}}" ;;
            esac
            # issue 0635 — the flag spelled in the FORMAT, not folded into a
            # variable: `check::example-leaf-target-dirs` reads an emitted
            # command as text and cannot see through a `$flag`.
            t="$(nros_example_build_target_dir "$dir")"
            printf 'cd %s && %s cargo %s build %s --target-dir %s\n' \
                "$dir" "$e" "$tc" "$cargo_profile_args" "$t"
        done < "$list" > "$units"
        nros_pool_run build-all-extras < "$units"
        rm -f "$units"
    fi
    rm -f "$list"
    echo "Build-all example extras built."

# True superset: workspace + non-fixture examples + per-test fixture variants.
# Pre-populates everything `just test-all` consumes. Slow.
[group("full-matrix")]
build-all:
    #!/usr/bin/env bash
    set -e
    # phase-319 W1 (issue 0351) — CLEAR the success marker before the attempt it
    # certifies. Written only on success and previously removed by nothing, the
    # stamp answered "did this EVER succeed?": a run that failed left the OLD
    # stamp in place and `_require-fixtures` waved `test-all` through on it.
    # Same discipline `compile-check-fixtures.sh` already applies to each
    # per-fixture `.compile-ok` one level down.
    source scripts/build/fixture-lane.sh
    nros_fixtures_stamp_clear
    # issue 0726 — the version match forks nothing.
    #
    # The version test used to be a quiet `-q` match piped from `make
    # --version`, inline in the condition below. Such a match cannot
    # distinguish a NON-MATCH from a matcher that failed to START, and
    # `check::grep-q-error-conflation` ratchets on that shape. Here
    # the mis-read is quiet rather than loud — a failed grep reads as "not 4.4"
    # and silently drops to the slower non-jobserver path — which is exactly the
    # kind of degradation nobody would ever trace back to a fork.
    #
    # A `case` on the captured string is fork-free and composes with the chain
    # through a plain variable.
    _nros_make_ver=""
    if [ -x third-party/make/make ]; then
        _nros_make_ver=$(third-party/make/make --version 2>/dev/null | head -1)
    fi
    case "$_nros_make_ver" in
        *4.4*) _nros_make_44=1 ;;
        *)     _nros_make_44=0 ;;
    esac
    if [ -z "${NROS_NO_JOBSERVER:-}" ] \
       && [ "$_nros_make_44" = 1 ] \
       && [ -x third-party/ninja/ninja ]; then
        echo "build-all: unified jobserver path (make 4.4 + ninja 1.13; NROS_NO_JOBSERVER=1 to opt out)"
        exec just build-all-jobserver
    fi
    echo "build-all: static split (install make>=4.4 + ninja>=1.13 — nros setup --tool make / --tool ninja — for the jobserver path)"
    just build
    just build-example-extras
    just build-test-fixtures-leaves
    # Stamp like the public `build-test-fixtures` so `_require-fixtures` lets
    # `test-all` run after `build-all` (the `-leaves` recipe doesn't stamp).
    # `build-all` is unconditionally the whole matrix, hence lane=all.
    nros_fixtures_stamp_write all
    echo "All builds completed (workspace + examples + test fixtures)."

# Phase 176 — `build-all` under one GNU-make fifo jobserver shared across
# every stage (cargo + build-script cc + ninja-via-west + cmake), instead
# of the static per-platform scheduler split. When the fast
# platforms finish, their tokens flow to the long pole automatically.
# Needs the pinned make >=4.4 + ninja >=1.13 (`nros setup --tool make` /
# `--tool ninja`). NROS_BUILD_JOBS (default nproc) = the token budget.
# Recipes detect the inherited jobserver (NROS_JOBSERVER=1) and skip their
# own explicit -j so the tools draw from the shared pool.
[group("full-matrix")]
build-all-jobserver:
    ./scripts/build-all-jobserver.sh

# Internal: invalidate stale nros-* cargo fingerprints in a cmake build
# dir's per-build cargo cache when shared-core source content has
# changed since the last build.
#
# Why: corrosion gives each cmake build dir its own cargo target tree
# under `build/cmake-<rmw>/cargo/...`. That tree's fingerprint check
# is mtime-based — when a `git checkout`, `git stash pop`, or similar
# operation rewrites a file's content WITHOUT bumping mtime past the
# fingerprint's `invoked.timestamp`, cargo decides "clean", reuses the
# pre-edit `.rlib`, and the resulting `lib<...>.a` carries stale
# code into every linked binary (zephyr, freertos, …). Cost us a
# multi-hour debug on cpp/xrce action E2E (post Phase 96.1).
#
# This guard hashes every shared-core `.rs` file and compares against
# a stamp file under the cmake build dir. Hash changed → nuke
# `nros*` fingerprints under that build dir → next cargo invocation
# revalidates. Hash unchanged → no-op (~200 ms hashing only).
_cmake-cargo-stale-guard build_dir:
    #!/usr/bin/env bash
    set -e
    BUILD_DIR="{{build_dir}}"
    [ -d "$BUILD_DIR" ] || exit 0
    # `git ls-files`, not `find` — these are tracked sources, so the index
    # already knows them and no walk is needed. `target/` needs no pruning
    # either: it is gitignored, so it was never in the index.
    SRC_HASH=$(git ls-files \
        packages/core \
        packages/rmw/xrce/nros-rmw-xrce \
        packages/rmw/zenoh/nros-rmw-zenoh \
        | grep '\.rs$' \
        | sort \
        | tr '\n' '\0' \
        | xargs -0 sha1sum 2>/dev/null \
        | sha1sum | cut -d' ' -f1)
    STAMP="$BUILD_DIR/.shared-cores-hash"
    LAST_HASH=$(cat "$STAMP" 2>/dev/null || true)
    if [ "$SRC_HASH" != "$LAST_HASH" ]; then
        echo "[stale-guard] shared-core source hash changed → invalidating nros-* fingerprints in $BUILD_DIR/cargo"
        find "$BUILD_DIR/cargo" -type d -path '*/.fingerprint/nros*' -exec rm -rf {} + 2>/dev/null || true
        echo "$SRC_HASH" > "$STAMP"
    fi

# The cmake build dirs hold their own cargo target tree
# (`build/cmake-<rmw>/cargo/...`) whose incremental cache can hand
# back stale `.rlib`s after edits to deeply-shared crates like
# `nros-node`. The Phase 140 `add_subdirectory` shape consumes nano-ros
# in-tree per-example, so the only persistent build dirs are the user's
# per-example `build/` directories; flush by removing those.

# Format everything: Rust workspace + examples, C, C++, Python
#
# Issue 0474 — `_require-leaf-includes` FIRST. `native::format` runs
# `cargo fmt` in every example leaf, and an unsynced leaf makes cargo fail
# during MANIFEST PARSE with a path that never mentions `nros sync` (issue
# 0463). That guard was wired to `build-test-fixtures-leaves` and
# `rust-rtos-link-check` — the two sites where the failure had been seen — and
# `format` is a third walking the same leaves. CLAUDE.md tells you to run
# `just format` BEFORE broad changes, so it is the site a newcomer hits first.
[group("main")]
format: _require-leaf-includes format-workspace native::format format-c format-cpp format-python
    @echo "All formatting completed!"

# Profile a project's build — passive, read-only (phase-251). Parses the timing
# artifacts a normal build already emitted under DIR (build*/.ninja_log for
# west/cmake/idf; target*/cargo-timings/ for cargo) into a stage table. It never
# builds. For per-crate cargo detail, build with `cargo build --timings` first.
#   just profile examples/zephyr/rust/talker
#   just profile examples/native/rust/talker --deep
# The analyzer bin is also runnable standalone for external copy-out projects:
#   ./target/debug/nros-build-profile <dir> --deep
[group("main")]
profile dir="." flags="":
    @cargo build -q -p nros-build-profile --bin nros-build-profile
    # profile-literal-ok: unprofiled: the build PROFILER tool (phase-251), built by a plain `cargo build`
    @"{{justfile_directory()}}/target/debug/nros-build-profile" {{dir}} {{flags}}

# phase-400 W4 — what did THIS build actually compile, and who required it?
#
# Reads `<target-dir>/**/.fingerprint/*/*.json`, which cargo writes for every
# unit it built, and reports the crate -> requirers table from the edges of the
# build that RAN. Passive: it never builds anything.
#
# It exists because three phase-400 estimates were computed from the WORKSPACE
# graph (`cargo tree`, subtree differences) and then assumed to describe a leaf.
# All three were wrong and all three were optimistic (31.9%->12.6%, 43 crates->6,
# 20.6 s->0). A subtree difference bounds what COULD leave; only the build knows
# what DOES — and the leaves that matter here often will not resolve standalone.
#
#   just leaf-graph <target-dir>                      # crate -> requirers
#   just leaf-graph <target-dir> "--exclusive-to nros-macros"
#
# Host and target sides are reported separately on purpose: a cross build has two
# graphs in one dir, and conflating them is how a host-only tool gets counted
# against firmware.
[group("main")]
leaf-graph dir flags="":
    @python3 "{{justfile_directory()}}/scripts/nros-leaf-graph.py" {{dir}} {{flags}}

# phase-400 W5 — are these cargo target dirs SAFE to collapse into one?
#
#   just shared-dir-churn "build-a build-b build-c"
#   just shared-dir-churn --self-test
#
# Reads the build-script fingerprints the builds ALREADY WROTE and asks whether
# the SAME unit recorded a different env value (churn) or a different
# watched-path set (a correctness hazard — cargo decides freshness from the
# RECORDED list, so the smallest set governs the whole shared dir). Cargo hashes
# features/profile/target itself; these two are what it does not.
[group("main")]
shared-dir-churn dirs:
    @python3 "{{justfile_directory()}}/scripts/nros-shared-dir-churn.py" {{dirs}}



# Two sessions opened `phase-350` for unrelated work on 2026-08-13, neither able
# to see the other — the same check-then-act race as issue ids, in the third
# numbered series. (The later one became `phase-352`.)
#
# Only for work needing its OWN number: a phase number is NOT unique per file
# (26 of 342 carry several docs, one effort split across them), so adding a doc
# to an existing effort reuses that number and skips this.
#
# Reserve the next free PHASE number atomically across parallel sessions.
[group("docs")]
phase-new slug="":
    @scripts/reserve-phase-id.sh {{slug}}

# phase-375 W2/W9 — scaffold a board package.
#
# Onboarding used to be a scavenger hunt across the five gates `s32z270` landed
# red on (weak symbols, board tiers, leaf lock, provider announcements, matrix
# orphan). Each was correct; the cost was meeting them one at a time, on main.
# The scaffold's acceptance is that a board it created is green on
# `just check fast` before its first commit.
#
# `--out-of-tree <dir>` writes the same package elsewhere: a user's board and
# ours are one shape, and which root found it is the only difference.
#
#   just board-new my-board --platform freertos
#   just board-new my-board --platform zephyr --west-board plank/soc/smp
#   just board-new my-board --platform freertos --out-of-tree ~/src/my-boards
[group("develop")]
board-new *ARGS:
    #!/usr/bin/env bash
    set -e
    # Same resolution order every other recipe uses: an explicit $NROS_CLI, then
    # the in-tree release build. Not PATH — a stale `~/.nros/bin/nros` shadowing
    # the in-tree CLI is the museum-binary trap `nros_resolve_cli` documents.
    _cli="${NROS_CLI:-packages/cli/target/release/nros}"
    if [ ! -x "$_cli" ]; then
        echo "board-new: no nros CLI at $_cli — run \`just setup-cli\` first." >&2
        exit 1
    fi
    "$_cli" board new {{ARGS}}

# Two RFC-0087s existed on 2026-09-04 — `package-identity-and-provider-format`
# and `ros2-api-adoption-and-the-compile-or-conform-rule`, four hours apart,
# neither session able to see the other. RFCs were the last of the three
# numbered series still numbered by reading the directory.
#
# ALWAYS, for every new RFC: unlike a phase number, an RFC id is unique per
# DOCUMENT and gated, so a collision is a hard red. It is also the most
# expensive to unwind — the number is cited in prose tree-wide (104 files at the
# 0087 collision) and a stale citation resolves to the other RFC instead of
# failing.
#
# Reserve the next free RFC number atomically across parallel sessions.
[group("docs")]
rfc-new slug="":
    @scripts/reserve-rfc-id.sh {{slug}}

# Phase 378 W1 — move every `ros-launch-manifest` pin to one tag, atomically.
#
# Four manifests across TWO workspaces pin this crate. Bumping a subset does not
# fail informatively: two revisions resolve as two same-named, incompatible
# types and the compiler blames a type mismatch, not the pin. So this validates
# the tag on the REMOTE first, rewrites every discovered manifest, refreshes both
# locks, and verifies each names exactly one revision — restoring everything if
# any step fails. A bogus tag changes nothing.
[group("main")]
bump-manifest tag="" flag="":
    @bash scripts/bump-manifest.sh {{tag}} {{flag}}

# Query the issue ledger — area, id, status or free text over title and body.
# Offline and ~20 ms, so it is the fast way to ask "has this been seen before?"
# before filing. CLAUDE.md has named this verb since the ledger existed; the
# recipe did not exist, so every reader who tried it got
# `Justfile does not contain recipe `issues``. Issue 0936.
#
#   just issues --area rmw          just issues 0934        just issues zenoh router
[group("docs")]
issues *ARGS:
    @python3 scripts/issues.py {{ARGS}}

# Phase 379 — regenerate the three user-API comparison pages (C, C++, Rust).
#
# MECHANICAL. Two code inputs, one authored input:
#   * our surface        re-extracted every run (clang for C/C++, nightly
#                        rustdoc for Rust) by the same extractors the parity
#                        gate uses -- never a hand-maintained list
#   * ROS 2's surface    docs/reference/api-surface/{rclc,rclcpp,rclrs}.json
#   * why, and what      docs/reference/api-parity-ledger/*.json -- the ONLY
#     answers it         file a human writes. `why` is the reason a row
#                        diverges, `provides` names our items that answer an
#                        upstream one when the mapping is not 1:1.
#
# Which of the seven states a row lands in is COMPUTED from the correlator's
# bucket and the ledger's verdict, both already gated. Adding a reason or a
# re-mapping arrow means editing the ledger and re-running this; it must never
# mean editing a page. `--self-test` (on the fast line, inside
# `check-api-parity-ledger`) pins the derivation rules and the template
# placeholders so the tool cannot rot into a hand-edited artifact.
#
# Needs clang + nightly rustdoc, no ROS install (the ROS side is recorded).
# ~30 s. Output is gitignored -- these are artifacts, not tracked docs.
#
#   just api-comparison                       # -> tmp/api-comparison/
#   just api-comparison --out tmp             # publish paths
#   just api-comparison --urls urls.json      # cross-page nav links
#
# Regenerate the C / C++ / Rust user-API comparison pages.
[group("docs")]
api-comparison *args:
    @python3 scripts/gen-api-comparison.py {{args}}

# Reserve the next free issue id ATOMICALLY across parallel sessions, and print
# it. Use this instead of eyeballing the highest existing number: that is a
# check-then-act race, and it has produced six id collisions (see
# `scripts/reserve-issue-id.sh` for why an instruction cannot fix it).
# phase-419 W3 — candidates for a roadmap verification pass. A REPORT, never a
# gate: supersession, a reversed premise and a claim of absence all need
# judgment, and W2's `check-roadmap-claims` already took everything that does
# not. Exit 0 whatever it finds.
[group("docs")]
roadmap-audit *ARGS:
    @python3 scripts/roadmap-audit.py {{ ARGS }}

[group("docs")]
issue-new slug="":
    @scripts/reserve-issue-id.sh {{slug}}

# Install the repo's git hooks (pre-push refuses a duplicate issue id, or a
# submodule pin that moved backward) AND the three git builtins that make
# submodule pointer moves legible. Idempotent; safe to re-run. Not automatic —
# pointing `core.hooksPath` at tracked scripts means a clone can run repo code on
# push, so it stays opt-in and `just setup` calls it explicitly.
#
# The builtins are VISIBILITY; `check::submodule-pins` + the hook are ENFORCEMENT.
# Git has no setting that refuses a rewind, but it does know how to describe one,
# and by default it does not: a pin move renders as two hex strings
# (`-Subproject commit d3f0d26` / `+Subproject commit 43ddb0e`) whose order no
# reader can tell. That is how a Zephyr `socklen_t` fix got silently unshipped on
# 2026-08-15 inside a 24-file commit about issue-ID renumbering.
#
#   diff.submodule=log          `git diff/show/log` prints "(rewind)" and lists
#                               the dropped commits with `<` before each subject.
#   status.submoduleSummary     `git status` shows the same BEFORE you commit —
#                               the earliest point anyone can catch it.
#   push.recurseSubmodules=check  refuses a push whose pins name commits that are
#                               on no remote (the "push the submodule FIRST" rule).
[group("main")]
setup-hooks:
    #!/usr/bin/env bash
    set -euo pipefail
    # Issue 0840 — the settings are DATA (config/git-settings.txt), applied here
    # and verified by `just doctor` through the same reader. Two hand-authored
    # copies of one list is precisely the pair that drifted in issue 0833 one
    # day earlier, and the checker was the copy that lost.
    # shellcheck source=scripts/lib/git-settings.sh
    source scripts/lib/git-settings.sh
    while IFS=$'\t' read -r key value _sev; do
        [ -n "$key" ] || continue
        git config "$key" "$value"
        echo "  git config $key $value"
    done < <(nros_git_settings)
    echo "hooks installed: core.hooksPath -> .githooks"

# Repair build dirs whose generated `build.ninja` no longer LOADS.
#
# ninja raises a manifest error (`multiple rules generate <x>` and friends) at
# LOAD, before any rule runs — so the rule that would re-run cmake and rewrite
# the manifest never gets the chance, and the dir stays wedged no matter what
# you ask ninja to build (issue 0882). `cmake <build-dir>` re-runs configure
# from the cached settings and regenerates the manifest in place.
#
# This never deletes a build dir. `rm -rf` proves only that a full build works,
# which was never in doubt, and it destroys the one reproduction that would
# have shown which dependency edge was missing — so a dir this cannot repair is
# REPORTED and left alone.
#
# `just reconfigure-stale check` reports without repairing.
[group("dev")]
reconfigure-stale mode="":
    #!/usr/bin/env bash
    set -euo pipefail
    if [ "{{mode}}" = "check" ]; then
        exec ./scripts/nros-reconfigure-stale.sh --check
    fi
    exec ./scripts/nros-reconfigure-stale.sh

# issue 0445 — which coordinates have produced no runtime result, and for how
# long. The probes write one line per non-running fixture under
# `target/nros-fixture-staleness/`; a fresh resolution deletes it. A cell stale
# for one run is your last edit; a cell stale for eleven is where a runtime
# defect accumulates unseen.
[group("test")]
fixture-staleness:
    #!/usr/bin/env bash
    set -euo pipefail
    dir=target/nros-fixture-staleness
    shopt -s nullglob
    entries=("$dir"/*.stale)
    if [ ${#entries[@]} -eq 0 ]; then
        echo "No non-running fixtures recorded — every probed coordinate resolved fresh."
        echo "(The ledger is written by the freshness probes; it is empty after a clean.)"
        exit 0
    fi
    now=$(date +%s)
    printf '%-6s  %-10s  %s\n' "STALE" "SINCE" "FIXTURE"
    for f in "${entries[@]}"; do
        read -r n since path < "$f"
        age=$(( now - since ))
        if   [ "$age" -lt 5400 ];   then human="$(( age / 60 ))m"
        elif [ "$age" -lt 172800 ]; then human="$(( age / 3600 ))h"
        else                             human="$(( age / 86400 ))d"
        fi
        printf '%-6s  %-10s  %s\n' "x${n}" "$human" "$path"
    done | sort -r
    echo ""
    echo "A coordinate here is NOT failing and NOT passing — it is not running."
    echo "Rebuild it (\`just build-test-fixtures\`); if the count keeps climbing,"
    echo "suspect the probe before trusting the verdict (issue 0445)."

# Regenerate the committed cbindgen headers (issue 0452). THE single writer:
# `nros_generated.h`, `nros_cpp_ffi.h` and `zpico.h` are committed, and build
# scripts only compare against them and warn. Run this after changing any
# `#[repr(C)]` / `extern "C"` surface, and commit the result.
[group("main")]
regen-c-headers:
    @cargo run -q -p nros-cbindgen-headers

# THE sanctioned way to change a lockfile (issue 0359 / 0378).
#
# A lockfile exists so someone else's build resolves what yours did, so its
# contents change ONLY when a dev asks for it. Everything else — `just check`,
# fixture builds, CI — must run `--locked` and FAIL on a mismatch rather than
# quietly rewriting the file.
#
#   just lock-update                     # root workspace, minimal refresh
#   just lock-update serde               # one crate, latest compatible
#   just lock-update serde 1.0.203       # one crate, exact version
#   just lock-update "" "" <dir>         # a leaf crate's own lock
#
# Bare `cargo generate-lockfile` is deliberately NOT what this runs: it
# re-resolves EVERY package to latest-compatible. That is how 26 leaf locks
# once moved 5388 lines in a single "cleanup" — a supply-chain change nobody
# reviewed. `cargo update` touches what you name and leaves the rest pinned.
[group("main")]
lock-update crate="" version="" dir=".":
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{justfile_directory()}}/{{dir}}"
    if [ -n "{{crate}}" ] && [ -n "{{version}}" ]; then
        cargo update -p "{{crate}}" --precise "{{version}}"
    elif [ -n "{{crate}}" ]; then
        cargo update -p "{{crate}}"
    else
        # No crate named: refresh only what the manifests now REQUIRE, without
        # bumping anything already satisfied.
        #
        # NO `|| cargo update` FALLBACK. `--workspace` is supported by the
        # pinned cargo (1.97.1), so the fallback could only fire when the
        # NARROW refresh failed — and a bare `cargo update` is a whole-graph
        # re-resolve, the operation issue 0359 records as having moved 5388
        # lines in one "cleanup". Escalating from the safe form to the
        # dangerous one, silently, is the opposite of what this recipe is for:
        # `lock-update` exists so a lock moves only when someone means it.
        #
        # `2>/dev/null` went with it. It hid the reason the narrow refresh
        # failed, which is the one thing the caller needs in order to decide.
        cargo update --workspace
    fi
    echo ""
    echo "[lock-update] REVIEW THE DIFF before committing:"
    echo "    git diff -- '*Cargo.lock'"
    echo "  Added/removed packages are a dependency change, not a refresh."

# Start the zenoh router — issue 0654's SSoT entry point.
#
# Eight per-platform `just <plat> zenohd` recipes already delegate to
# `nros_router_exec`; they differ only in which locator that platform's images
# dial. This is the same call without a platform, for the common case of "start
# a router on localhost" — and it is the command documentation can name instead
# of pasting a command line, which is how 92 copies of one accreted.
#
# The router itself is resolved by `nros_zenohd_bin` (issue 0653): an explicit
# `NROS_RMW_ZENOHD`, the prefixes you have SOURCED, then `$ROS_DISTRO` under
# /opt/ros. Nothing is searched that you did not name.
[group("main")]
zenohd locator="tcp/127.0.0.1:7447":
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/dev/zenohd.sh
    source "{{justfile_directory()}}/scripts/dev/zenohd.sh"
    nros_router_exec "{{locator}}"

# Test tiers (each tier is a strict superset of the previous):
#
#   test-unit         workspace lib/bin tests except nros-tests crate.
#                     ~5s, no external deps.
#   test-integration  nros-tests integration tests excluding heavy QEMU /
#                     Zephyr / ROS-2-interop groups. ~30s, needs zenohd.
#   test              = test-unit + test-integration. Default dev tier.
#                     No miri, no heavy QEMU/Zephyr.
#   test-doc          rustdoc doctests for the `nros` umbrella crate.
#   test-miri         Miri UB scan on embedded-safe crates. Standalone, ~min.
#   test-all          = test + heavy QEMU / Zephyr / threadx-linux /
#                     ros2-interop groups + test-doc + test-miri + C codegen.
#                     True superset, requires `just build-test-fixtures` first.
#
# Per-platform tests (just <plat> test|test-all|ci) are organized in
# the matching just/<plat>.just files — see CLAUDE.md for the matrix.

# issue 0328 — RUN the `#[ignore]`d tests. Nothing did before: no recipe, no
# workflow and no nextest profile passed `--run-ignored`, so 24 ignored tests
# across six crates were dead code that read like coverage. The worst of them
# are `rosidl-codegen`'s heap/borrowed storage-mode compile checks, which are
# that feature's ONLY gate.
#
# Not in `just ci`: several genuinely need external infrastructure (a zenohd
# router on a fixed port, an XRCE agent), which is exactly why they were
# ignored. The point of this recipe is that they are REACHABLE and their state
# is knowable — an ignored test with no lane that runs it should fail review
# the same way `#[allow(dead_code)]` without a reason does.
#
#   just test-ignored                  # every ignored test
#   just test-ignored rosidl-codegen   # one crate
[group("main")]
test-ignored package="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    echo "Running #[ignore]d tests (external infra may be required)…"
    rc=0
    if [ -n "{{package}}" ]; then
        # A named package may live in EITHER workspace; try the root first,
        # then the cli sub-workspace.
        cargo nextest run "${cargo_nextest_args[@]}" -p "{{package}}" \
            --run-ignored ignored-only \
        || cargo nextest run --manifest-path packages/cli/Cargo.toml \
            -p "{{package}}" --run-ignored ignored-only \
        || rc=$?
    else
        # BOTH workspaces. 16 of the 24 ignored tests live in packages/cli
        # (rosidl-codegen's storage-mode compile checks among them), which the
        # root workspace cannot see at all — a root-only recipe would have
        # reported success while running none of the ones that matter most.
        #
        # nros-tests is excluded for the same reason test-unit excludes it: its
        # fixtures need `just build-test-fixtures` staging first.
        cargo nextest run "${cargo_nextest_args[@]}" --workspace --exclude nros-tests \
            --run-ignored ignored-only || rc=$?
        echo "--- packages/cli sub-workspace ---"
        cargo nextest run --manifest-path packages/cli/Cargo.toml --workspace \
            --run-ignored ignored-only || rc=$?
    fi
    exit "$rc"

# Workspace lib/bin/unit tests, excluding the integration crate.
[group("main")]
test-unit verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    # `nros-rmw-{zenoh,dds,xrce}-cffi` excluded for the same reason as
    # `check::workspace`: their `*Rmw` type imports are platform-feature
    # gated, and `cargo nextest run --workspace` activates no features.
    # Real coverage of these shims comes from their per-feature
    # invocations under `check::workspace-features`.
    args=(--workspace --exclude nros-tests \
          --exclude nros-rmw-xrce-cffi \
          --exclude nros-rmw-xrce-cffi-staticlib \
        --exclude nros-build-paths \
          --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    # issue 0388 — `nros_tests::skip!` panics with `[SKIPPED]` for an unmet
    # precondition, and nextest has no native skip, so those land as FAILURES and
    # the tier exits 100. `test-all` and `_nextest-platform` already rewrite them
    # to `<skipped>` and tally only REAL failures; tier 1 did not, so the tier
    # CLAUDE.md tells everyone to run reported red for "you are missing a
    # binary" exactly as it does for "you broke something". Same handling here.
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    just _rewrite-skipped-junit || true
    [ $rc -eq 0 ] && exit 0
    # Issue #29 — a build/setup failure (nextest exit != 100, or no junit) must
    # NOT be masked by the [SKIPPED] tolerance: a crate that fails to COMPILE
    # emits zero junit cases, which would otherwise tally as "0 real failures"
    # and green a broken build. Exit 100 means "tests ran and some failed".
    if [ "$rc" -ne 100 ] || [ ! -f target/nextest/default/junit.xml ]; then
        echo "ERROR: unit-test build/setup failed (nextest exit $rc) — not a [SKIPPED] precondition."
        exit 1
    fi
    real="$(just _count-real-failures)"
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s):"
        just _name-real-failures || true
        just _check-skip-budget || true
        exit 1
    fi
    # Issue 0584 — the success path is exactly where an unnoticed skip lives:
    # "all failures were skips" is the sentence a lane that ran nothing also
    # prints. Assert the skips before believing it.
    just _check-skip-budget
    echo "All failures were [SKIPPED] preconditions — treating as pass."

# Run an ARBITRARY nextest filter with honest skip accounting (issue 1016).
#
# `nros_tests::skip!` panics with `[SKIPPED]`, and nextest has no native skip —
# so a bare `cargo nextest run -E ...` reports every unmet precondition as a
# FAILURE. The summary line for "six cells were never built" is then
# character-for-character the summary line for "six cells ran and failed":
#
#     Summary [295.800s] 9 tests run: 0 passed, 9 failed, 45 skipped
#
# `test-all` and `test-integration` already rewrite those to `<skipped>` before
# tallying, but both carry a FIXED filter, so anyone running a subset by hand
# got the raw count. That is not hypothetical: it produced a wrong reading in
# issue 0968 — six of nine zephyr cells reported as failures when they were
# out-of-lane skips, and only the panic frame distinguished them.
#
#     just test-select 'test(/example_e2e::case_(19|2[0-7])_xrce_/)'
#     just test-select 'binary(rtos_e2e) and test(Platform__Nuttx)'
#
# Deliberately NOT calling `_check-skip-budget`: that guard exists so a full
# sweep cannot report green having run nothing, and its budget is a property of
# the sweep. A filtered run may legitimately skip everything it selected (every
# cell out of lane is the normal case), so applying it here would fail correct
# runs. The skip REASONS are printed instead — read them.
test-select filter verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/test/nextest-profile.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-E '{{filter}}')
    if [ -n "{{verbose}}" ]; then args+=(--no-capture); fi
    nros_nextest_junit_reset
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    junit="$(nros_nextest_junit_path)"
    just _rewrite-skipped-junit "$junit" || true
    [ $rc -eq 0 ] && exit 0
    # Same refusal as `test-all`: a build/setup failure emits no junit cases and
    # would otherwise tally as "0 real failures" (issue #29).
    # Exit 4 is nextest's "no tests to run" — the filter matched nothing. That
    # is a distinct failure from a broken build and deserves its own sentence:
    # a filter selecting nothing is the hazard phase-373 names, because it looks
    # exactly like a lane with nothing to do.
    if [ "$rc" -eq 4 ]; then
        echo "ERROR: the filter selected NO tests — nothing ran."
        echo "       '{{filter}}' matched none of the test names in this tree."
        echo "       This is not a pass. Check the expression against \`cargo nextest list\`."
        exit 1
    fi
    if [ "$rc" -ne 100 ] || [ ! -f "$junit" ]; then
        echo "ERROR: nextest build/setup failed (nextest exit $rc) — not a [SKIPPED] precondition."
        exit 1
    fi
    real="$(just _count-real-failures "$junit")"
    just _test-summary "$junit" || true
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s):"
        just _name-real-failures "$junit" || true
        exit 1
    fi
    # Say the number outright. `_test-summary` counts `<failure>` elements, and
    # the rewrite has just turned these into `<skipped>` — so it correctly
    # reports "Real failures: 0 / 0", which on its own reads as "nothing
    # happened" rather than "nine cells declined to run".
    skipped="$(grep -c '<skipped' "$junit" 2>/dev/null || echo 0)"
    echo "All failures were [SKIPPED] preconditions — treating as pass."
    echo "  $skipped selected test(s) SKIPPED, 0 ran. A filtered run that skips"
    echo "  everything is legitimate (out-of-lane is the normal case) — but it is"
    echo "  NOT evidence about the code. Reasons are in the [SKIPPED] lines above."

# nros-tests integration tests, skipping heavy cross-compile / QEMU groups.
# Filters mirror the `test` recipe's `-E` predicate, just scoped to
# `package(nros-tests)` so the workspace unit tests aren't re-run.
[group("main")]
test-integration verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    # Issue #57: exclude the QEMU/Zephyr e2e binaries by binary() too — nextest
    # assigns rtos_e2e/zephyr tests to GRANULAR sub-groups (qemu-freertos-pubsub,
    # qemu-zephyr-pubsub-rust, … first-match-wins, .config/nextest.toml), so the
    # umbrella group() exclusions never match them; (the retired phase_118_collapse binary is gone; its cells live in binary(zephyr))
    # at all. On a runner WITH qemu-system-arm + no prebuilt firmware they hard-fail
    # instead of skipping. All three binaries are entirely QEMU/Zephyr e2e.
    exclude='not (group(=qemu-baremetal) or group(=qemu-freertos) or group(=qemu-nuttx) or group(=qemu-threadx-riscv) or binary(esp32_emulator) or group(=threadx-linux) or group(=qemu-zephyr) or group(=qemu-zephyr-xrce) or group(=zephyr-fvp) or group(=ros2-interop) or binary(xrce_ros2_interop) or binary(rtos_e2e) or binary(zephyr))'
    args=(-p nros-tests --no-fail-fast -E "$exclude")
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    # `nros_tests::skip!` panics with `[SKIPPED]` for unmet preconditions
    # (missing fixture/binary/emulator/agent/SDK) — nextest has no native skip,
    # so those count as failures and exit non-zero. Treat the run as passing iff
    # there are no *real* (non-[SKIPPED]) failures — same contract as
    # `_nextest-platform`. Real failures still fail the recipe.
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    just _rewrite-skipped-junit || true
    [ $rc -eq 0 ] && exit 0
    # Issue #29 — distinguish a real BUILD/setup failure from test-level
    # [SKIPPED] preconditions. `cargo nextest` exits 100 ONLY when tests ran and
    # some failed; any other non-zero (101 = compile/build error, ENOSPC, a
    # missing junit) is a setup failure that the [SKIPPED] tolerance must NOT
    # mask as a pass — otherwise a fixture/test that fails to *compile* produces
    # zero junit testcases, `_count-real-failures` sees 0, and the lane greens
    # over a broken build.
    if [ "$rc" -ne 100 ] || [ ! -f target/nextest/default/junit.xml ]; then
        echo "ERROR: nros-tests build/setup failed (nextest exit $rc) — not a [SKIPPED] precondition."
        just _test-summary || true
        exit 1
    fi
    real="$(just _count-real-failures)"
    just _test-summary || true
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s):"
        just _name-real-failures || true
        just _check-skip-budget || true
        exit 1
    fi
    # Issue 0584 — the success path is exactly where an unnoticed skip lives:
    # "all failures were skips" is the sentence a lane that ran nothing also
    # prints. Assert the skips before believing it.
    just _check-skip-budget
    echo "All failures were [SKIPPED] preconditions — treating as pass."

# Shared helper: run a single nros-tests integration test binary with the
# standard verbose-flag handling. Used by per-platform `test` / `test-all`
# recipes in just/<platform>.just so the args/verbose boilerplate lives in
# one place.
# Issue 0673 — the ONE place `nros_tests::skip!` is interpreted, so the marker
# means the same thing in every lane that runs tests.
#
# `skip!` panics carrying `[SKIPPED…]` because Rust's harness has no runtime
# skip, so a BARE `cargo nextest run` counts every unmet precondition as a
# failure. Only the junit rewrite turns them back into skips — and it used to
# live inside `test-all` and `_nextest-platform`, so a lane that called nextest
# directly (`check::required-features-tests`) reported thirteen capability skips
# as a tier-1 red on any host without `ros-<distro>-rmw-zenoh-cpp`, hiding every
# step after it.
#
# Takes the nextest arguments verbatim; callers keep their own `--features` /
# `--test` spelling so `check::required-features-reachable` can still read
# reachability off the literal text.
[private]
_nextest-tolerant +nextest_args:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/test/nextest-profile.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=({{nextest_args}})
    # A nextest `-E` expression cannot travel through `{{nextest_args}}`: just
    # interpolates a variadic parameter UNQUOTED, so `-E 'test(Nuttx)'` lands in
    # the generated script as `args=(-E test(Nuttx))` and bash dies with
    # "syntax error near unexpected token `('". That is why the nightly's
    # `nuttx` and `threadx_linux` jobs failed — they are the only two callers
    # whose filter contains parens (phase-395 W0.5).
    #
    # An env var crosses the `just -> just` boundary without re-parsing, so the
    # expression never passes through interpolation at all.
    if [ -n "${NROS_NEXTEST_FILTER:-}" ]; then
        args+=(-E "$NROS_NEXTEST_FILTER")
    fi
    # issue 0695 — the junit path is DERIVED, not the hardcoded default:
    # nextest writes it under the target dir, so a lane with a scoped
    # CARGO_TARGET_DIR (test-zpico-multisession) has its junit there, and
    # reading `target/nextest/default/junit.xml` here would tally whatever
    # unrelated run last wrote it.
    # Clear every candidate first, then read back whichever nextest wrote:
    # the store does NOT follow CARGO_TARGET_DIR (see nextest-profile.sh), so
    # the path cannot be predicted before the run.
    nros_nextest_junit_reset
    # `nros_tests::skip!` panics with `[SKIPPED]` for unmet preconditions
    # (missing fixture/binary/emulator) — nextest has no native skip, so those
    # count as failures and exit non-zero. Treat a run as passing iff there are
    # no *real* (non-[SKIPPED]) failures, per `_count-real-failures`. Real
    # failures still fail the recipe.
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    junit="$(nros_nextest_junit_path)"
    # Phase 214.R.1: rewrite [SKIPPED] failures → <skipped> before tallying.
    just _rewrite-skipped-junit "$junit" || true
    [ $rc -eq 0 ] && exit 0
    # Issue #29 — a build/setup failure (nextest exit != 100, or no junit) must
    # NOT be masked by the [SKIPPED] tolerance: a binary that fails to compile
    # emits zero junit cases, which would otherwise tally as "0 real failures".
    if [ "$rc" -ne 100 ] || [ ! -f "$junit" ]; then
        echo "ERROR: nextest build/setup failed (nextest exit $rc) — not a [SKIPPED] precondition."
        exit 1
    fi
    real="$(just _count-real-failures "$junit")"
    just _test-summary "$junit" || true
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s):"
        just _name-real-failures "$junit" || true
        just _check-skip-budget "$junit" || true
        exit 1
    fi
    # Issue 0584 — the success path is exactly where an unnoticed skip lives:
    # "all failures were skips" is the sentence a lane that ran nothing also
    # prints. Assert the skips before believing it.
    just _check-skip-budget "$junit"
    echo "All failures were [SKIPPED] preconditions — treating as pass."

_nextest-platform test_name verbose="" feature_args="" filter="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test {{test_name}} --no-fail-fast)
    # `filter` is a nextest `-E` expression, for a lane whose tests live in a
    # SHARED target rather than a per-platform one. The NuttX lane needs it: its
    # own suite was one boot micro-test that `rtos_e2e`'s `Platform::Nuttx` cells
    # already subsumed, so the lane now selects those cells out of `rtos_e2e`
    # instead of running a target of its own. Without this the choice is running
    # every platform's cells or losing the [SKIPPED] junit rewrite.
    if [ -n "{{filter}}" ]; then
        # NOT `args+=(-E '{{filter}}')`: these args are handed to
        # `_nextest-tolerant`, whose variadic parameter is interpolated
        # unquoted, so a filter containing parens becomes a bash syntax error
        # there. Hand it over in the environment instead.
        export NROS_NEXTEST_FILTER='{{filter}}'
    fi
    # A target behind `required-features` is skipped SILENTLY by cargo — not
    # reported as filtered, not counted anywhere — so a caller needing one must
    # ask for its feature. The caller passes the WHOLE FLAG (`--features rmw`),
    # not a bare feature name, because `check::required-features-reachable` reads
    # reachability off the literal `--features` text in this file: a
    # `--features {{{{feature_args}}}}` here would leave the real feature name
    # spelled nowhere the gate can see, which is the gate-narrower-than-its-rule
    # shape of issue 0196.
    if [ -n "{{feature_args}}" ]; then
        args+=({{feature_args}})
    fi
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    just _nextest-tolerant "${args[@]}"

# Run rustdoc doctests for the `nros` umbrella crate.
# Nextest does not execute doctests, so we run them separately.
# This catches drift between rustdoc examples and the real API.
[group("main")]
test-doc:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    # phase-361 W3 — `std` explicit: `nros` no longer defaults to it, and the
    # doc examples are hosted.
    cargo test $cargo_profile_args --doc -p nros --features std

# Rewrite [SKIPPED]-marker <failure> entries in the junit.xml to <skipped>
# so downstream consumers (CI dashboards, _count-real-failures, _test-summary,
# scripts/test/failed-filterset.py) see them as skips, not failures.
# Idempotent + safe on missing files. See `scripts/test/rewrite-skipped-junit.py`
# and `docs/development/test-harness.md` (Phase 214.R).
_rewrite-skipped-junit junit="target/nextest/default/junit.xml":
    #!/usr/bin/env bash
    python3 scripts/test/rewrite-skipped-junit.py "{{junit}}"
    # phase-395 W5 — demote QUARANTINED failures here, before the snapshot, for
    # the same reason the rewrite above happens here: every downstream consumer
    # then reads one agreed account of the run instead of re-deriving it.
    python3 scripts/test/quarantine.py --demote "{{junit}}"
    # Issue 0527 — SNAPSHOT the rewritten file. `junit.xml` is written by every
    # `cargo nextest` invocation, so the doctest phase that runs after this, and
    # every suite a human re-runs while triaging, overwrite the one artifact
    # that knew which failures were real. `junit-real.xml` is written only here.
    if [ -f "{{junit}}" ]; then
        cp -f "{{junit}}" "$(dirname "{{junit}}")/junit-real.xml" || true
    fi

# Count real (non-[SKIPPED]) test failures from the latest junit.xml.
# Tests that panic with `[SKIPPED] ...` (via the nros_tests::skip! macro)
# are environment-conditional skips and excluded from the real failure count.
# Counts only `<failure ` entries whose `message=` attribute contains [SKIPPED],
# not raw `[SKIPPED]` strings (which also appear in `<system-err>`).
#
# Phase 214.R.1 added `_rewrite-skipped-junit` which converts those entries
# to native `<skipped>` BEFORE this counter runs at the recipe tail — so on a
# post-rewrite junit this returns 0. The legacy grep path here is kept as a
# defence in depth for callsites that haven't yet been hooked up.
_count-real-failures junit="target/nextest/default/junit.xml":
    #!/usr/bin/env bash
    junit="{{junit}}"
    if [ ! -f "$junit" ]; then
        echo 0
        exit 0
    fi
    # `grep -c` prints 0 on no-match and exits 1, so no `|| echo 0` fallback
    # is needed — the fallback would double-emit "0\n0" and break $(( )).
    total=$(grep -c '<failure ' "$junit")
    # A failure is environment-skipped if its <failure> tag's content contains [SKIPPED].
    # We grep for `<failure ` lines plus the next line (the panic message body).
    skipped=$(grep -A1 '<failure ' "$junit" | grep -c '\[SKIPPED\]')
    real=$((total - skipped))
    if [ $real -lt 0 ]; then real=0; fi
    echo "$real"

# Issue 0527 — NAME the real failures, not just count them. Reads the
# `junit-real.xml` snapshot by default (see `_rewrite-skipped-junit`), because
# the live `junit.xml` is whatever ran most recently. Always exits 0: this runs
# on a path that has already decided to fail.
_name-real-failures junit="":
    #!/usr/bin/env bash
    python3 scripts/test/name-real-failures.py {{junit}}

# Issue 0584 — ASSERT the run's skips, do not merely count them. Two derived
# properties, no declaration file to drift: no `lane` skip for a coordinate the
# lane selected, and no skip whose reason is a missing fixture (that is a hard
# failure since 0584 part 2). Reads the `junit-real.xml` snapshot and
# `$NROS_TEST_COORDS`.
_check-skip-budget junit="":
    #!/usr/bin/env bash
    python3 scripts/test/check-skip-budget.py {{junit}}

# Print a one-line summary of test outcomes from junit.xml.
_test-summary junit="target/nextest/default/junit.xml":
    #!/usr/bin/env bash
    junit="{{junit}}"
    if [ ! -f "$junit" ]; then
        echo "No junit.xml found"
        exit 0
    fi
    total=$(grep -c '<failure ' "$junit")
    skipped=$(grep -A1 '<failure ' "$junit" | grep -c '\[SKIPPED\]')
    real=$((total - skipped))
    if [ $real -lt 0 ]; then real=0; fi
    if [ $skipped -gt 0 ]; then
        echo "Environment-skipped tests: $skipped (missing prerequisites)"
        grep -A1 '<failure ' "$junit" | grep -o '\[SKIPPED\][^<&]*' \
            | sort | uniq -c | sort -rn | sed 's/^/  /'
    fi
    echo "Real failures: $real / $total total failures"

# Print the slowest nextest tests from junit.xml.
[private]
_nextest-slow-tests junit="target/nextest/default/junit.xml" limit="20":
    #!/usr/bin/env bash
    python3 scripts/test/nextest-slow-tests.py \
        "{{junit}}" \
        --limit {{limit}}

# Default dev tier — workspace unit tests + integration tests, with
# heavy QEMU / Zephyr / ROS-2-interop groups skipped. Does NOT run
# Miri (use `test-miri` or `test-all` for that).
# issue 0393 — the ONE cell that exercises the multi-session zpico paths.
#
# `ZPICO_MAX_SESSIONS` defaults to 1, which is correct for a shipped target: the
# C shim's session pool and the Rust shim's session-indexed SERVICE_BUFFERS /
# REPLY_WAKERS (phase-328 / issues 0348, 0376) collapse to a single entry and the
# static footprint stays minimal. But with nothing in the tree raising it,
# `two_sessions_deliver_cross_session_through_router` skipped on every host in
# every tier, so the code those two issues added was never executed by CI.
#
# Raising it globally is the wrong fix twice over: cargo MERGES .cargo/config.toml
# up the directory tree, so an `[env]` at the repo root reaches the in-tree
# examples too (verified) — every embedded example would double those tables.
# So: one lane, one env, and its OWN target dir, because the value is a build
# input (`rerun-if-env-changed`) and sharing `target/` with the default-1 tiers
# would rebuild the shim back and forth on every alternation.
[group("test")]
test-zpico-multisession verbose="":
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    export CARGO_TARGET_DIR="$(nros_scoped_target_dir zpico-multisession)"  # issue 0400: box-aware
    export ZPICO_MAX_SESSIONS=2
    # issue 0695 — through `_nextest-tolerant`, not a bare `cargo nextest run`
    # (issue 0673's rule): `nros_tests::skip!` raised in ANOTHER package's own
    # test binary — this lane's `zenoh_integration` — is still a skip, and a
    # bare run turned its `[SKIPPED]` panic into a hard red no fix can clear.
    # The tolerant helper rewrites the junit (which lands under this lane's
    # scoped CARGO_TARGET_DIR, exported above) and fails only on real failures.
    # The `two_sessions` filter is POSITIONAL (name substring), equivalent to
    # the old `-E 'test(~two_sessions)'`: `_nextest-tolerant` splices its args
    # into a bash array literal, where the `(`…`)` would not survive.
    args=(-p nros-rmw-zenoh --features platform-posix --test zenoh_integration
          two_sessions --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    just _nextest-tolerant "${args[@]}"
    # issue 0652 — `loan_e2e` for the same reason, and it is why the feature is
    # off `check::required-features-tests`: it runs a publisher and a subscriber
    # in ONE process (same-process pub/sub on a single session hits zenoh-pico's
    # write filter), so it needs the pool this lane's env provides. #0652 named
    # this recipe as its home and left the wiring undone, so the target stayed
    # in no lane after all.
    loan_args=(-p nros-tests --features loan-e2e --test loan_e2e --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        loan_args+=(--success-output never --failure-output never)
    fi
    just _nextest-tolerant "${loan_args[@]}"

#
# Heavy groups are skipped via a CLI `-E` predicate keyed off nextest
# test-groups (`qemu-{baremetal,freertos,nuttx,threadx-riscv,esp32,zephyr}`,
# `threadx-linux`, `ros2-interop`, `xrce_ros2_interop`). New heavy
# binaries inherit the skip by assigning to one of those groups in
# `.config/nextest.toml`. `group(...)` is a CLI-only predicate
# (nextest 0.9.133+), so the list lives here rather than under a
# `[profile.fast]` default-filter.
#
# phase-411 W3 — `just test [<scope>…]`, one vocabulary in one position:
#
#   just test              this host, best effort: what is provisioned runs,
#                          what is not SKIPS — and the scope line says which
#   just test zephyr       one platform, NAMED, so it must work rather than skip
#   just test tier2        the lane's run, narrowed to the lane's coordinates
#
# The first positional used to be `verbose`, so `just test 1` no longer means
# what it did — the incompatibility phase-411 accepts. `verbose` / `-v` /
# `--verbose` are still recognised, as FLAGS anywhere in the argument list, so
# the spelling people actually type keeps working.
#
# Run the tests for a scope; with no scope, this host, best effort.
[group("main")]
test *scope:
    #!/usr/bin/env bash
    # shellcheck source=scripts/build/scope.sh
    source scripts/build/scope.sh
    verbose=""
    scoped=()
    for tok in {{scope}}; do
        case "$tok" in
            -v | --verbose | verbose) verbose="verbose" ;;
            *) scoped+=("$tok") ;;
        esac
    done
    if [ "${#scoped[@]}" -gt 0 ]; then
        nros_scope_validate_all "${scoped[@]}" || exit 2
        nros_scope_report test "${scoped[@]}"
        rc=0
        for tok in "${scoped[@]}"; do
            just _test-scope "$tok" "$verbose" || rc=1
        done
        exit "$rc"
    fi
    # The DEFAULT scope: derived by probing, reported before anything runs.
    # phase-411's "best-effort" meaning lives here and nowhere else — a NAMED
    # platform must work (W2), an unnamed one may skip and is always reported.
    nros_scope_report test
    echo ""
    # Pre-407 these were recipe DEPENDENCIES. A dependency runs whatever the
    # arguments say, so a scoped `just test zephyr` would have paid for the
    # host fixture preflight it does not use.
    #
    # `|| exit` is load-bearing: this body has no `set -e` (the nextest tallying
    # below manages its own exit codes), and a dependency that used to ABORT the
    # recipe must not degrade into a warning the run continues past. That would
    # be exactly the defect phase-411 is about — a preflight that reports and is
    # then ignored.
    just _require-build-sources _require-fixtures-ready test-zpico-multisession || exit 1
    # issue 0923 — sweep BEFORE nextest, for the same reason `test-all` does
    # (issue 0659). This lane runs the same suites and spawns the same peers,
    # and a SIGKILLed run leaves them: `PR_SET_PDEATHSIG` reaches `bash` only,
    # so `timeout`/`ros2`/the node reparent to init and hold DDS discovery
    # ports until something reaps them. Having the reaper on one of the two
    # lanes that need it meant orphans survived until somebody happened to run
    # the other one — 67 of them, oldest 3 h, measured 2026-08-30.
    #
    # Not mid-run: a concurrent test's peers are recorded and alive, so a sweep
    # then would kill them.
    cargo run -q -p nros-tests --bin nros-peer-sweep 2>/dev/null || true
    source scripts/build/cargo.sh
    source scripts/test/nextest-profile.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    nextest_run_profile_args=($(nros_nextest_run_profile_args))
    nextest_fail_fast_args=($(nros_nextest_fail_fast_args))
    set +e
    failed=0
    # Issue #57: exclude the QEMU/Zephyr e2e binaries by binary() too — nextest
    # assigns rtos_e2e/zephyr tests to GRANULAR sub-groups (qemu-freertos-pubsub,
    # qemu-zephyr-pubsub-rust, … first-match-wins, .config/nextest.toml), so the
    # umbrella group() exclusions never match them; (the retired phase_118_collapse binary is gone; its cells live in binary(zephyr))
    # at all. On a runner WITH qemu-system-arm + no prebuilt firmware they hard-fail
    # instead of skipping. All three binaries are entirely QEMU/Zephyr e2e.
    exclude='not (group(=qemu-baremetal) or group(=qemu-freertos) or group(=qemu-nuttx) or group(=qemu-threadx-riscv) or binary(esp32_emulator) or group(=threadx-linux) or group(=qemu-zephyr) or group(=qemu-zephyr-xrce) or group(=zephyr-fvp) or group(=ros2-interop) or binary(xrce_ros2_interop) or binary(rtos_e2e) or binary(zephyr))'
    args=(--workspace "${nextest_run_profile_args[@]}" "${nextest_fail_fast_args[@]}" -E "$exclude")
    # A shell variable, not a recipe parameter: the first positional is SCOPE
    # now, and verbosity is a flag parsed out of it at the top of this body.
    if [ -z "$verbose" ]; then
        args+=(--success-output never --failure-output never)
    fi
    nros_nextest_record_begin test
    nros_nextest_record_write_command \
        cargo nextest run "${cargo_nextest_args[@]}" "${NROS_NEXTEST_RECORD_ARGS[@]}" "${args[@]}"
    nros_nextest_junit_reset
    cargo nextest run "${cargo_nextest_args[@]}" "${NROS_NEXTEST_RECORD_ARGS[@]}" "${args[@]}"
    nextest_exit=$?
    # Read back whichever candidate nextest wrote — the store does not follow
    # CARGO_TARGET_DIR, so this cannot be resolved before the run.
    junit="$(nros_nextest_junit_path)"
    # Phase 214.R.1: rewrite [SKIPPED] failures → <skipped> before tallying so
    # downstream junit consumers (CI dashboards, _count-real-failures, etc.)
    # see them as native skips rather than failures.
    just _rewrite-skipped-junit "$junit" || true
    real_failures=$(just _count-real-failures "$junit")
    if [ "$nextest_exit" -ne 0 ] && [ ! -f "$junit" ]; then
        failed=1
    elif [ "$nextest_exit" -ne 0 ] && [ "$real_failures" -gt 0 ]; then
        failed=1
    fi
    echo ""
    just _test-summary "$junit"
    echo ""
    just _nextest-slow-tests "$junit"
    echo ""
    nros_nextest_record_finish
    echo ""
    echo "JUnit XML: $junit"
    just _check-skip-budget || failed=1
    if [ $failed -ne 0 ]; then
        echo "FAIL: Some tests failed."
        exit 1
    else
        echo "All standard tests passed! (Miri skipped — run \`just test-miri\` or \`just test-all\`.)"
    fi

# One scope token's test run.
#
# PLATFORM first, unlike `_build-scope`: `native` is both a platform module and
# a lane, and for a RUN the module is the right machinery — `just native test`
# is that platform's suite, whereas the `native` LANE narrows nothing (it is
# module-level) and would hand `test-all` the whole tree. Both readings are the
# same SCOPE, which is what `check-scope-namespace` asserts; only the machinery
# differs, and each verb picks the one that exists for it.
[private]
_test-scope tok verbose="":
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/scope.sh
    source scripts/build/scope.sh
    tok="$(nros_scope_normalize "{{tok}}")"
    if nros_scope_is_platform "$tok"; then
        nros_scope_require_module_verb "$tok" test
        nros_scope_exec just "$tok" test {{verbose}}
        exit 0
    fi
    if nros_scope_is_preset "$tok"; then
        # shellcheck source=scripts/build/fixture-lane.sh
        source scripts/build/fixture-lane.sh
        if [ "$tok" = "all" ]; then
            nros_scope_exec env NROS_FIXTURE_LANE=all just test-all {{verbose}}
            exit 0
        fi
        # The lane's own coordinate file reaching BOTH the preflight and the
        # run — the same two-variable pairing `ci::matrix` uses, and for the
        # same reason (issue 0482: a lane-scoped build accepted for an
        # unnarrowed run fails ~231 fixtures later).
        coords="$(nros_lane_coords_file "$tok")"
        coords="$(cd "$(dirname "$coords")" && pwd)/$(basename "$coords")"
        nros_scope_exec env "NROS_FIXTURE_LANE=$tok" "NROS_TEST_COORDS=$coords" \
            just test-all {{verbose}}
        exit 0
    fi
    nros_scope_reject "$tok"

# Build ONLY the compile-check fixtures (issue 0034 / issue 0871).
#
# `check::source-gates` runs three `cargo test`s that ASSERT a prebuilt
# `.compile-ok` stamp. It sits in `check::build`, which CI runs on pull requests
# in a job that builds no fixtures at all — so every PR failed with
#
#     Test fixture binary not prebuilt: build/compile-check-fixtures/
#     platform_hdr_posix_cpp_heap/.compile-ok
#
# while `main` stayed green, because the same job runs only `check::fast` on
# push. A gate `main` never runs is the issue-0196 class: the build side and the
# test side disagreeing about what exists.
#
# Separate from `build-test-fixtures` because that builds the whole matrix; this
# is the small subset `check::source-gates` actually asserts, so CI can afford it
# as a step. `build-test-fixtures` calls THIS, so there is one spelling.
[group("build")]
build-compile-check-fixtures builder="":
    @NROS_FIXTURE_BUILDER="{{builder}}" bash scripts/build/compile-check-fixtures.sh


# Pre-build every example binary the test suite reaches.
#
# The contract is: tests only verify a binary exists at a known path —
# they never compile fixtures themselves. This recipe is the build
# phase. Splitting the build phase from the test phase lets cargo/cmake
# use full host parallelism without competing with N concurrent QEMU +
# zenohd processes during the nextest run, which used to stretch a 14 s
# test out to 125 s under load. Run this before `just test-all`.
# Phase 150.F — `generate-bindings` precondition: every per-platform
# `build-fixtures` recipe assumes `generated/<pkg>/` is populated for
# each fixture crate. Without it `cargo build` fails on
# `unable to update generated/builtin_interfaces`. Make the dep
# explicit so `just build-test-fixtures` (and `just test-all` via
# the bench fixtures it consumes) is self-contained.
#
# `lane` (issue 0393) narrows the build to one CI lane's fixture coordinates —
# `all` (default, every row), `tier1`, `tier2`, `tier2-nightly`. The selection
# comes from `lane-coords`, the same binary `_lane-gate` uses, so the build, the
# staleness gate and the test run derive from ONE computation, which is what
# `ci_lane.rs` already claimed and only two of the three actually did.
#
# issue 0677 — `check::fast` runs FIRST, before anything expensive.
#
# Every other dependency here asks "is the ENVIRONMENT ready to build?"
# (`_require-build-sources`, `_require-leaf-includes`, the generators). None
# asked "is the TREE in a state where building is MEANINGFUL?", so a defect a
# static gate already names was discovered by a multi-hour multi-platform
# compile instead: #0532 item 5 retired the wall-clock pair, `nros-c` kept
# calling it, and `check::retired-platform-clock-symbols` — which names both
# symbols and was already failing — sat in a lane nothing on this path ran.
# The link error surfaced two fixture rebuilds later.
#
# `check::fast` is the right edge precisely because of the contract documented
# on it: BUILDLESS and SOURCE-FREE, no CLI, no `nros sync`, no provisioned
# toolchain, green in 23s on a pristine detached worktree. That is what makes
# this dependency affordable in front of a build measured in hours, and it is
# the property to preserve — a gate added to `check::fast` that needs the
# environment makes THIS edge expensive, and an expensive edge gets deleted.
#
# "Run `just ci` first" is not a substitute: fixtures must already be fresh for
# `test-all` to mean anything, so the honest order is build-then-test, which
# puts the expensive step first by construction.
[group("full-matrix")]
build-test-fixtures lane="all": check::fast _require-build-sources _clear-fixture-stamp _codegen build-zenoh-posix-fixture (build-test-fixtures-leaves lane)
    #!/usr/bin/env bash
    set -e
    source scripts/build/fixture-lane.sh
    # Compile-check fixtures (issue 0034): build-stage `cargo check` of small
    # template crates whose tests only prove they compile — the test asserts the
    # `.compile-ok` stamp instead of running cargo at run time.
    #
    # ONE spelling, because `check::source-gates` needs these too and CI runs it
    # in a job that builds no fixtures (issue 0871). Calling the recipe rather
    # than re-invoking the script keeps the two callers from drifting.
    just build-compile-check-fixtures
    # Drop a stamp so `_require-fixtures` (the test-all/test preflight) can
    # fast-fail with a build hint instead of letting the suite run and
    # surface dozens of "Binary not found" failures. The body only runs
    # after every dependency above succeeds. Phase 177.9.
    #
    # Issue 0393 — the stamp records the LANE AND ITS COORDINATES, not just a
    # timestamp, so the preflight can ask "does what was built cover what I am
    # about to run?" instead of "did a build finish?".
    nros_fixtures_stamp_write "$(nros_lane_arg "{{lane}}")"
    # issue 0499 option 2 — record the identity reading HERE, where the tree is
    # known-fresh, because this is the one moment its number can be trusted. In
    # `check::fast` the same script reads whatever a long-lived tree accumulated;
    # here the stamp was just written, so `started_at` filters to exactly what
    # this build produced.
    #
    # REPORT, never fail: a build that produced its artifacts correctly must not
    # be failed by a budget, and a red at the end of a 40-minute build is the
    # kind nobody can act on. The gate in `check::fast` still fails; this only
    # makes the trustworthy reading visible, so drift shows up as a moving
    # number in build logs instead of surfacing days later on a stale tree.
    bash scripts/check-artifact-identity-budget.sh || true
    # issue 0616 — the archives exist now, so ask them whether any image ships
    # two allocators. This is the check `check::feature-contract` clause (e)
    # cannot make: it counts DEFINITIONS IN SOURCE (exactly one, always), while
    # the invariant is per LINKED ARTIFACT and there are four staticlib roots.
    # Hard failure, not `|| true`: a duplicate lang item is a broken image, not
    # a drifting number.
    bash scripts/check-archive-lang-items.sh

# phase-319 W1 (issue 0351) — clear the stamp BEFORE building, so a failed or
# interrupted run leaves none and `_require-fixtures` fails with its build hint
# instead of certifying a build stage that had stopped working.
#
# A DEPENDENCY, not a line in `build-test-fixtures`'s body: that body runs AFTER
# its dependencies, and the dependencies are what do the building. The clear was
# in the body, so 0351's "clear before building" held for `build-all` (which
# builds in its own body) and was defeated here — observed 2026-08-02, when a
# failing native fixture build left a three-day-old stamp in place, exactly the
# state 0351 was filed about. Dependencies run left-to-right, so first is first.
[private]
_clear-fixture-stamp:
    @bash -c 'source scripts/build/fixture-lane.sh && nros_fixtures_stamp_clear'

# Internal fixture fan-out without root prereqs. Public `build-test-fixtures`
# keeps the self-contained UX; aggregate paths that already ran `build` use
# this to avoid repeating `generate-bindings` and `build-zenoh-posix-fixture`.
[private]
build-test-fixtures-leaves lane="all": _require-leaf-includes
    #!/usr/bin/env bash
    set -e
    # (The phase-177.9 `NROS_FIXTURE_SHARED_SIG` export lived here until
    # 2026-08-02. Phase 181.7c deliberately retired the content-hash staleness
    # mechanism in favour of the `cmake --build` self-heal probe and deleted
    # `nros_fixture_shared_sig` along with every consumer — but left this
    # producer behind, so every fixture build printed
    # `nros_fixture_shared_sig: command not found` to stderr and exported an
    # empty string nothing read. `set -e` never caught it because `export
    # V="$(cmd)"` takes the exit status of the `export` builtin, not of the
    # substitution — a plain `V="$(cmd)"` would have aborted the recipe on the
    # first run. Nothing else in this recipe used `fixture-matrix.sh`, so the
    # `source` went with it.)
    # Issue 0393 — lane narrowing, in two layers that have to agree:
    #
    #   modules  which `just <mod> build-fixtures` runs at all (the big saving:
    #            tier 1 drops eight of nine cross families outright)
    #   coords   which manifest ROWS each surviving module builds, via
    #            NROS_FIXTURE_COORDS -> fixtures-build.sh / workspace-fixtures-
    #            build.sh -> fixtures-manifest.py --coords-from
    #
    # Both derive from `lane-coords`, so they cannot select different sets.
    source scripts/build/fixture-lane.sh
    lane="$(nros_lane_arg "{{lane}}")"
    lane_modules=""
    if [ "$lane" != "all" ]; then
        lane_modules="$(nros_lane_modules "$lane")"
        [ -n "$lane_modules" ] || {
            echo "build-test-fixtures: lane $lane selected zero modules — refusing to build nothing" >&2
            exit 2
        }
        # `native` is module-level (build every native row); the tier lanes also
        # narrow the ROWS each surviving module builds.
        coords_file="$(nros_lane_coords_file "$lane")"
        if [ -n "$coords_file" ]; then
            export NROS_FIXTURE_COORDS="$(cd "$(dirname "$coords_file")" && pwd)/$(basename "$coords_file")"
            echo "build-test-fixtures: lane=$lane coords=$(wc -l < "$NROS_FIXTURE_COORDS")"
        fi
        echo "build-test-fixtures: lane=$lane modules=$(echo $lane_modules | tr '\n' ' ')"
    fi
    # Keep the canonical ORDER (zephyr first / solo with the full budget) and
    # filter it, rather than iterating the lane's set — scheduling is a property
    # of the platform, not of the lane.
    in_lane() {
        if [ -z "$lane_modules" ]; then return 0; fi
        printf '%s\n' "$lane_modules" | grep -qx "$1"
    }
    # Phase 226.C — direct fallback fixture fan-out uses a temporary make graph
    # instead of GNU parallel or a raw Zephyr background lane. The pinned fifo
    # jobserver path enters through build-all; this fallback still centralizes
    # platform scheduling under ordinary make when invoked directly.
    log_dir="${NROS_BUILD_LOG_DIR:-$(pwd)/tmp/build-test-fixtures-$(date +%Y%m%d-%H%M%S)-$$}"
    mkdir -p "$log_dir" tmp
    log_dir="$(cd "$log_dir" && pwd)"
    ln -sfn "$log_dir" tmp/build-test-fixtures-latest
    joblog="$log_dir/build-test-fixtures.joblog"
    makefile="$log_dir/build-test-fixtures.mk"
    printf 'stage\tstart_epoch\tend_epoch\tduration_seconds\tstatus\n' > "$joblog"
    echo "build-test-fixtures: log-dir=$log_dir"
    run_stage() {
        local stage="$1"
        shift
        local start end status
        start="$(date +%s)"
        status=0
        echo "== $stage =="
        "$@" || status=$?
        end="$(date +%s)"
        printf '%s\t%s\t%s\t%s\t%s\n' "$stage" "$start" "$end" "$((end - start))" "$status" >> "$joblog"
        return "$status"
    }
    budget="${NROS_BUILD_JOBS}"
    if [ "${NROS_JOBSERVER:-}" = "1" ]; then
        echo "build-test-fixtures: NROS_JOBSERVER=1 — serial launcher; child tools inherit fifo tokens"
        # `in_lane … && run_stage …` would abort the recipe under `set -e` when
        # the module is filtered OUT (a false compound command is a failure), so
        # the skip is an explicit `if`.
        #
        # phase-411 W2 — `NROS_LANE_INCLUDED` is what makes a skip legitimate
        # here: the operator asked for a LANE, not for these platforms by name,
        # so an unprovisioned one is reported (78 -> SKIPPED) rather than failed.
        # It is set per stage rather than exported once, so it names the platform
        # in the child's environment and cannot leak to an unrelated `just` this
        # recipe might later run. Unset is NAMED — see scripts/build/lane-skip.sh.
        if in_lane zephyr; then run_stage zephyr env NROS_LANE_INCLUDED=zephyr just zephyr build-fixtures; fi
        for platform in native qemu freertos nuttx threadx_linux threadx_riscv64 esp32 px4; do
            in_lane "$platform" || continue
            run_stage "$platform" env "NROS_LANE_INCLUDED=$platform" just "$platform" build-fixtures
        done
        exit 0
    fi
    case "$budget" in
        ''|*[!0-9]*)
            echo "Invalid NROS_BUILD_JOBS=$budget; expected positive integer" >&2
            exit 2
            ;;
    esac
    [ "$budget" -ge 1 ] || {
        echo "Invalid NROS_BUILD_JOBS=$budget; expected positive integer" >&2
        exit 2
    }
    # Issue 0726 — POOLED launcher, opt-in via NROS_BUILD_POOL=1.
    #
    # The static split below measured 45% of its wall clock running ONE stage
    # against an inner cap of 8 on a 32-core host: a 25% ceiling, because a
    # fixed partition cannot reclaim capacity as stages drain, which is exactly
    # when the longest stage is still going. The serial NROS_JOBSERVER=1 path
    # is not the answer either — measured 3-4/32 runnable, because one stage at
    # a time starves whenever that stage's own graph is narrow (zephyr's west
    # configure steps are largely single-threaded).
    #
    # What the evidence asks for is BOTH: stages overlapping so narrow ones run
    # together, and ONE token pool so the tail can expand into what the others
    # release. That is possible now because both heavy children are jobserver
    # CLIENTS — cargo always was, and ninja since 1.13 (verified here on 1.13.2:
    # 8 ninja edges under `make -j2 --jobserver-style=fifo` peaked at 2). So the
    # outer jobserver no longer has to be hidden from them, and `-j` per child
    # can be dropped entirely: make hands out `budget` tokens and every cargo
    # and ninja in the tree draws from that one pool.
    if [ "${NROS_BUILD_POOL:-}" = "1" ]; then
        # Count the lane's stages HERE rather than reading $lane_platforms,
        # which this recipe does not compute until further down — referencing it
        # early made it empty, `grep -c .` returned 1, and `set -e` killed the
        # recipe before the banner even printed.
        outer=0
        for _p in zephyr native qemu freertos nuttx threadx_linux threadx_riscv64 esp32 px4; do
            if in_lane "$_p"; then outer=$((outer + 1)); fi
        done
        [ "$outer" -lt 1 ] && outer=1
        [ "$outer" -gt "$budget" ] && outer="$budget"
        inner=""            # no static split; children inherit the jobserver
        make_jobs="$budget"
        echo "build-test-fixtures: POOLED — make -j$budget, $outer stage(s), shared tokens"
        # Children INHERIT the jobserver; nothing is unset.
        NROS_STAGE_ENV=""
        # And they must not ALSO be handed an explicit width. A stage exports
        # CMAKE_BUILD_PARALLEL_LEVEL from its budget, which becomes ninja's
        # `-j` — and an explicit -j overrides jobserver throttling, so 7
        # concurrent stages each ran 32 wide. Measured peak 44 runnable on 32
        # cores. The per-platform recipes already know how to unset it; they
        # just keyed on NROS_JOBSERVER alone, so tell them the same fact.
        export NROS_INHERIT_JOBSERVER=1
    else
    outer=4
    [ "$outer" -gt "$budget" ] && outer="$budget"
    inner=$(( budget / outer )); [ "$inner" -lt 1 ] && inner=1
    make_jobs=$((outer + 1))
    # The outer jobserver is a LAUNCHER width, not a build budget, so it must
    # not leak into children that would join the tiny pool instead of using the
    # explicit split they were handed.
    NROS_STAGE_ENV="-u MAKEFLAGS -u CARGO_MAKEFLAGS"
    fi
    # The generated recipes reference it as `$$NROS_STAGE_ENV`, i.e. the SHELL
    # expands it at stage-run time, so it has to be in the environment.
    export NROS_STAGE_ENV
    echo "build-test-fixtures: budget=$budget, make-jobs=$make_jobs, pool=$outer×$inner + zephyr=$budget (solo)"
    # Issue 0393 — the lane-filtered platform list, computed ONCE. The graph
    # names its targets in three places (.PHONY, `all:`, the rule loop) and they
    # must agree, so they read one variable rather than three copies of the
    # literal list.
    lane_platforms=""
    for platform in zephyr native qemu freertos nuttx threadx_linux threadx_riscv64 esp32 px4; do
        if in_lane "$platform"; then lane_platforms="$lane_platforms $platform"; fi
    done
    lane_platforms="${lane_platforms# }"
    [ -n "$lane_platforms" ] || {
        echo "build-test-fixtures: lane $lane selected zero platforms — refusing to build nothing" >&2
        exit 2
    }
    {
        printf 'SHELL := /bin/bash\n'
        printf '.SHELLFLAGS := -eu -o pipefail -c\n'
        printf '.DELETE_ON_ERROR:\n'
        printf '.PHONY: all %s\n' "$lane_platforms"
        printf 'all: %s\n\n' "$lane_platforms"
        # The banner's "zephyr (solo)" promise is enforced by an ORDER-ONLY
        # prerequisite (`| zephyr`): every other family waits for zephyr to
        # finish before starting, so zephyr really does run alone with the
        # full budget instead of concurrently with 4 sibling families
        # (~2x oversubscription, observed in the 2026-08-03 jobs audit).
        zephyr_prereq=""
        if in_lane zephyr; then zephyr_prereq=" | zephyr"; fi
        for platform in $lane_platforms; do
            # Pooled: hand each stage the full budget and let the shared
            # jobserver throttle. A jobserver client asks for tokens before it
            # runs anything, so `budget` is a ceiling it will not reach unless
            # the machine is actually free — which is the entire point.
            child_jobs="${inner:-$budget}"
            prereq="$zephyr_prereq"
            if [ "$platform" = "zephyr" ]; then
                child_jobs="$budget"
                prereq=""
            fi
            log="$log_dir/$platform.log"
            printf '%s:%s\n' "$platform" "$prereq"
            # `env -u MAKEFLAGS -u CARGO_MAKEFLAGS`: this outer make's own
            # jobserver (make_jobs tokens — a LAUNCHER width, not a build
            # budget) must not leak into the children, where a bare ninja or
            # cargo would join the tiny pool instead of using the explicit
            # NROS_BUILD_JOBS split it was handed (same audit).
            # issue 0599 — THREE verdicts, not two. rc 78 (`nros_lane_skip`,
            # scripts/build/lane-skip.sh) means the lane could not run because a
            # precondition is missing; that is neither OK nor FAILED, and
            # printing it as OK is what hid an unprovisioned Zephyr workspace
            # until `_lane-gate` failed on artifacts twenty minutes later. The
            # reason comes back through the lane log's `NROS_LANE_SKIP:` marker.
            #
            # phase-411 W2 — `NROS_LANE_INCLUDED=<platform>` is what ENTITLES
            # this stage to that third verdict. These platforms were selected by
            # a lane, not typed by the operator, so an absent SDK is reported and
            # not failed. A direct `just <plat> build-fixtures` leaves the
            # variable unset, is therefore NAMED, and fails instead. Unset =
            # NAMED is the deliberate default: a driver that forgets this
            # assignment goes red rather than quietly staying green.
            printf '\t+@start=$$(date +%%s); status=0; echo "== %s =="; ( env %s NROS_LANE_INCLUDED=%q NROS_BUILD_JOBS=%q just %q build-fixtures ) >%q 2>&1 || status=$$?; end=$$(date +%%s); printf "%%s\\t%%s\\t%%s\\t%%s\\t%%s\\n" %q "$$start" "$$end" "$$((end - start))" "$$status" >>%q; if [ "$$status" -eq 78 ]; then echo "== %s == SKIPPED ($$(sed -n "s/^NROS_LANE_SKIP: //p" %q | tail -1))"; else if [ "$$status" -ne 0 ]; then echo "== %s == FAILED (rc=$$status); log tail:"; tail -40 %q || true; exit "$$status"; fi; echo "== %s == OK"; fi\n\n' \
                "$platform" "$NROS_STAGE_ENV" "$platform" "$child_jobs" "$platform" "$log" "$platform" "$joblog" "$platform" "$log" "$platform" "$log" "$platform"
        done
    } > "$makefile"
    # issue 0762 — run the fan-out under ONE process group, so killing this
    # launcher takes the whole make/just/cmake/cargo tree with it instead of
    # orphaning it. Nested launchers see NROS_SUBTREE_GUARD and pass through.
    source scripts/build/subtree-guard.sh
    nros_guard_exec fixtures make -j "$make_jobs" -f "$makefile"
    echo "All test fixtures built."

# Phase 150.E rev3 — single deterministic fixture serving both
# `nros-tests::zenoh_header_parity` (reads the canonical
# `zenoh_generic_config.h`) and `nros-tests::zenoh_archive_symbols`
# (reads `libnros_rmw_zenoh_staticlib.a`). Both artefacts are
# products of `cargo build -p nros-rmw-zenoh-staticlib --features
# platform-posix`; bundle them into one dedicated --target-dir so
# the tests always read the POSIX-policy variant, not whichever
# feature set hit the shared workspace `target/` last (a cross-
# target `just threadx_riscv64 build-fixtures` would otherwise
# overwrite both with Phase 146.2 `LinkPolicy::threadx()` content).
#
# Output (deterministic — one `zpico-sys-<hash>` per --target-dir):
#   build/zenoh-fixture-posix/release/libnros_rmw_zenoh_staticlib.a
#   build/zenoh-fixture-posix/release/build/zpico-sys-*/out/
#       zenoh-config/zenoh_generic_config.h
#
# Tests discover these paths via the `NROS_TESTS_ZENOH_ARCHIVE`
# and `NROS_TESTS_ZENOH_HEADER` env vars when set (out-of-tree /
# CI override); otherwise walk this directory.
#
# `--release` matters and stays LITERAL (phase-336 allow-list):
# `zenoh_archive_symbols.rs` predates this recipe and was written
# against `target/release/`, and `scripts/check-zenoh-archive-symbols.sh`
# is invoked with that path below. This archive is a symbol-inspection
# fixture — its optimization level is irrelevant, its PATH is not — so
# pinning both sides to one built-in profile is the stable choice.
[group("full-matrix")]
build-zenoh-posix-fixture:
    #!/usr/bin/env bash
    set -e
    # issue 0535 — was `--target-dir target-zenoh-fixture-posix`, a repo-root
    # cache dir named by a literal here AND in the two tests that read it. It is
    # under the one build root now (RFC-0070 R1) and both sides name the KIND.
    source scripts/build/build-root.sh
    # profile-literal-ok: symbol fixture: path asserted by zenoh_archive_symbols + the parity script
    # phase-361 W8.d — `platform-posix` selects the platform, not the standard
    # library. This archive is a HOSTED artifact, so it names `std` itself.
    cargo build --release \
        -p nros-rmw-zenoh-staticlib \
        --features std,platform-posix \
        --target-dir "$(nros_build_dir "$NROS_KIND_ZENOH_FIXTURE_POSIX")"

# Workflow (Phase 177.9): `just test-all` (full coverage) → read the failures →
# debug/fix → `just test-failed` (reruns just those) → repeat until clean.
# Reuses the same cargo profile + nextest run-profile + per-platform groups as
# the full run; builds a nextest `-E` filterset from the JUnit report and
# overwrites it with the subset result, so each rerun naturally shrinks.
#
# Rerun only the real (non-[SKIPPED]) failed tests from the latest JUnit run.
[group("full-matrix")]
test-failed verbose="":
    #!/usr/bin/env bash
    source scripts/build/cargo.sh
    source scripts/test/nextest-profile.sh
    junit="$(nros_nextest_junit_path)"
    if [ ! -f "$junit" ]; then
        echo "No JUnit report at $junit — run 'just test-all' (or 'just test') first."
        exit 1
    fi
    filterset="$(python3 scripts/test/failed-filterset.py "$junit")"
    if [ -z "$filterset" ]; then
        echo "No real (non-[SKIPPED]) failures in $junit — nothing to rerun."
        exit 0
    fi
    count="$(python3 scripts/test/failed-filterset.py "$junit" --names | grep -c . || true)"
    echo "Rerunning $count failed test(s) from $junit:"
    python3 scripts/test/failed-filterset.py "$junit" --names | sed 's/^/  /'
    echo ""
    cargo_nextest_args=($(nros_cargo_nextest_args))
    nextest_run_profile_args=($(nros_nextest_run_profile_args))
    args=(--workspace "${nextest_run_profile_args[@]}" --no-fail-fast -E "$filterset")
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output immediate)
    fi
    rm -f "$junit"
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    nextest_exit=$?
    # Phase 214.R.1: rewrite [SKIPPED] failures → <skipped> before tallying.
    just _rewrite-skipped-junit "$junit" || true
    echo ""
    just _test-summary "$junit"
    real_failures=$(just _count-real-failures "$junit")
    if [ "$real_failures" -gt 0 ]; then
        echo "Still failing: $real_failures — fix and rerun 'just test-failed'."
        exit 1
    fi
    echo "All previously-failing tests now pass."

# Preflight for the full suite: fast-fail with a build hint if test fixtures
# were never built, instead of running the whole matrix and surfacing dozens
# of "Binary not found" failures. The stamp is written by build-test-fixtures.
# Bypass with NROS_SKIP_FIXTURE_CHECK=1 if fixtures were built another way
# (e.g. scoped `just <plat> build-fixtures`). Phase 177.9.
#
# Issue 0393 — the check is COVERAGE, not existence. `NROS_FIXTURE_LANE` names
# the lane this run needs (set by `ci` / `ci-matrix` / `ci-matrix-nightly`;
# `all` by default), and the stamp records the lane + coordinates the build
# actually produced. A tier-1 stamp therefore no longer waves a tier-3 run
# through, and a tier-3 stamp still satisfies every lane.
[private]
_require-fixtures:
    #!/usr/bin/env bash
    if [ "${NROS_SKIP_FIXTURE_CHECK:-0}" != "0" ]; then
        exit 0
    fi
    source scripts/build/fixture-lane.sh
    nros_fixtures_stamp_require "${NROS_FIXTURE_LANE:-all}"

# Issue 0463 — preflight: a tracked leaf `.cargo/config.toml` reaches
# sync-generated content through `include` (the central `nros-patch.toml` since
# #272, the `nros-managed-patch.toml` sidecar since #457). Both targets are
# gitignored, so a clone has neither, and cargo treats a missing include as a
# HARD error during MANIFEST PARSE — `cargo metadata` and every gate that reads
# the leaf fail too, four frames deep, naming a path with no mention of sync.
# (Two comments in cmd/ws.rs claimed cargo drops a missing include silently;
# measured on 1.97.1 it does not.) Say "run `nros sync`" once here rather than
# once per leaf in cargo's words. Bypass with NROS_SKIP_LEAF_INCLUDE_CHECK=1.
[private]
_require-leaf-includes:
    #!/usr/bin/env bash
    if [ "${NROS_SKIP_LEAF_INCLUDE_CHECK:-0}" != "0" ]; then
        exit 0
    fi
    python3 scripts/build/leaf-config-includes.py

# issue 1038 — WRITE the central `nros-patch.toml` that 47 tracked leaf
# `.cargo/config.toml` files `include`. The PRODUCER for the dependency the
# preflight above only checks.
#
# Only `nros sync` wrote that file, and sync needs a WORKSPACE. A lane building
# standalone leaves has no workspace to sync, so it got the file as a SIDE
# EFFECT of whichever neighbouring lane ran `workspace-fixtures-build.sh` first.
# Six lanes do (esp32, threadx-linux, native, nuttx, zephyr-ci, freertos);
# `threadx-riscv64` does not, which is why it is the one whose leaves died at
# manifest parse with
#
#   failed to load config include `../../../../../nros-patch.toml`
#
# while the identical config in a freertos leaf resolved. The lanes that worked
# were working by accident.
#
# `nros ws central-patch` is the SAME writer `sync` calls, not a second spelling
# of the content — the table must agree with sync's byte for byte or two leaves
# resolve against different crates. Idempotent (skip-write when unchanged), so a
# lane that already has the file pays one process spawn.
[private]
_ensure-central-patch:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    nros_ensure_central_patch

# The codegen stage of the build verb (issue 0992).
#
# `colcon build` generates messages, which is why a ROS user never types a
# separate codegen command. Our equivalent must do the same, or the step is
# mandatory and reachable from no verb in the documented
# `setup -> build -> test` chain.
#
# It was reachable from ONE build lane and not the other. `build-test-fixtures`
# listed the generators itself; `rust-rtos-link-check` listed only the
# `_require-leaf-includes` preflight — which asserts codegen has ALREADY run.
# On a developer tree the leaves were synced by some earlier run and the
# omission is invisible; on a fresh checkout the preflight is unsatisfiable,
# because the only thing that would satisfy it is downstream of itself. That is
# what `build-wide` failed on every run: 54 unresolved `include` targets, from
# a lane whose whole job is to build the leaves that would resolve them.
#
# ORDER IS THE POINT, and the old dependency list had it backwards: it ran
# `generate-bindings` BEFORE `setup-launch-resolve`. `nros sync` refuses to
# resolve a SystemModel without that helper (issue 0409 — it errors rather than
# reusing a museum model), so on a tree that lacks the binary codegen dies
# naming a helper the very next dependency would have built. Provision, then
# generate, then verify the generated tree resolves.
#
# The verify step stays a CHECK, not a repair: after codegen has run, an
# unresolved `include` means something is wrong with the generated tree, and
# 0463's message is the right one to print.
[private]
_codegen: setup-launch-resolve generate-bindings _require-leaf-includes

# issue 0390 — preflight: the repo's build stage needs the UNION of vendored
# `[source.*]` (every RMW's `-sys` source + the platform sources the workspace
# graph path-deps), NOT the per-board slice `nros setup <board>` provisions. Fail
# fast, naming `nros setup --source <name>` per missing, instead of dying deep in
# a raw cargo / build-script error naming a path with no mention of setup. The
# union is the index's top-level `build_sources`. Bypass with
# NROS_SKIP_BUILD_SOURCE_CHECK=1.
[private]
_require-build-sources:
    #!/usr/bin/env bash
    if [ "${NROS_SKIP_BUILD_SOURCE_CHECK:-0}" != "0" ]; then
        exit 0
    fi
    source scripts/build/cargo.sh
    "$(nros_cli_bin)" setup --build-sources --check

# Warn (non-fatal) about prebuilt fixture cells whose inputs changed since the
# binary was built — sources edited without re-running build-fixtures. Runs the
# fixture's incremental build (rust: cargo `"fresh":false` probe; C/C++: cmake
# self-heal) so a stale cell is rebuilt in place before the test run. Skipped
# under NROS_SKIP_FIXTURE_CHECK=1.
#
# phase-278 (issue #147): this preflight WARN+self-heals, but only under
# `just test-all`. The fixture RESOLVER now also guards independently — a bare
# `cargo nextest run` hard-fails "… is STALE" (naming the newer source) instead
# of silently running a stale binary, via a detect-only dep-info probe
# (cargo `<binary>.d` / `ninja -t deps`; never rebuilds). Both honour
# NROS_SKIP_FIXTURE_CHECK=1.
# Issue 0681 direction 2 — ONE name for "the fixtures this lane needs are ready".
#
# Two gates answer two questions and every caller wants both:
#
#   `_require-fixtures`      the STAMP — was a build run whose coverage includes
#                            this lane's coordinates?
#   `_check-fixtures-stale`  per-fixture FRESHNESS — is each `.inputsig` still
#                            newer than its inputs?
#
# A stamp can cover the lane while fixtures have gone stale underneath it, so
# neither answer implies the other. Requiring callers to remember the pair is
# the seam that produced issue 0443 (the two reached the lane under different
# variable names) and issue 0681 (the precondition batch knew about only one,
# reported OK, and `just ci` died on the other minutes later).
#
# Order is load-bearing: stamp FIRST. With no build at all, the freshness audit
# has nothing to compare and its message would describe the wrong problem.
#
# Both derive their scope from NROS_FIXTURE_LANE (0443), so this takes ONE scope
# and cannot disagree with itself. Prefer this over naming either half.
[private]
_require-fixtures-ready: _require-fixtures _check-fixtures-stale

[private]
_check-fixtures-stale:
    #!/usr/bin/env bash
    set -euo pipefail
    # Issue 0443 — the lane used to reach the two fixture gates under two
    # different names: `_require-fixtures` reads NROS_FIXTURE_LANE, this gate
    # reads NROS_FIXTURE_SCOPE (+ NROS_FIXTURE_COORDS). `just ci` sets both;
    # `ci-matrix` set only the lane, so SCOPE fell back to `all` and the gate
    # audited the whole tier-3 fixture set while the run, the build and the
    # stamp were all scoped to tier 2 — demanding out-of-lane fixtures with a
    # remedy that builds a different platform.
    #
    # Nothing could detect that: `all` is a legitimate value, so the gate cannot
    # tell "the caller wants everything" from "the caller forgot the second
    # variable". So DERIVE the scope from the lane instead of asking every
    # caller to keep two spellings in sync. An explicit SCOPE still wins, which
    # keeps `just ci` and the per-lane check-fixtures-stale recipe unchanged.
    # The scope this lane implies. One mapping, used both to DERIVE the scope
    # and to check an explicitly-set one against it.
    case "${NROS_FIXTURE_LANE:-}" in
        ""|all) implied=all ;;
        native) implied=native ;;
        *)      implied=coords ;;
    esac

    # An explicit SCOPE still wins — `just ci` sets both, and `_lane-gate`
    # supplies its own coordinate file. But it may not CONTRADICT the lane.
    # Deriving fixed the over-audit direction (SCOPE wider than LANE: merely
    # obstructive). The dangerous direction is the reverse — a SCOPE narrower
    # than the LANE reports a lane green having freshness-checked less than it
    # ran, which is the laundering this gate exists to prevent. Nothing caught
    # that before, because `all` is a legitimate value and the gate could not
    # tell "wants everything" from "forgot the second variable".
    if [ -n "${NROS_FIXTURE_SCOPE:-}" ]; then
        if [ -n "${NROS_FIXTURE_LANE:-}" ] && [ "${NROS_FIXTURE_SCOPE}" != "$implied" ]; then
            echo "ERROR: NROS_FIXTURE_SCOPE='${NROS_FIXTURE_SCOPE}' contradicts" >&2
            echo "       NROS_FIXTURE_LANE='${NROS_FIXTURE_LANE}', which implies" >&2
            echo "       scope '${implied}'. These are two spellings of ONE fact" >&2
            echo "       (issue 0443): the run, the build, the stamp and this gate" >&2
            echo "       must agree on what the lane contains. Set the LANE alone" >&2
            echo "       and the scope is derived, or make the two match." >&2
            exit 2
        fi
        NROS_FIXTURE_SCOPE_ORIGIN=explicit exec ./scripts/check-fixtures-stale.sh
    fi
    case "$implied" in
        all)
            NROS_FIXTURE_SCOPE_ORIGIN="lane:${NROS_FIXTURE_LANE:-unset}" \
                exec ./scripts/check-fixtures-stale.sh
            ;;
        native)
            NROS_FIXTURE_SCOPE=native NROS_FIXTURE_SCOPE_ORIGIN="lane:${NROS_FIXTURE_LANE}" \
                exec ./scripts/check-fixtures-stale.sh
            ;;
        coords)
            # The SAME coordinate file `_lane-gate` and the fixture build use,
            # so gate and build cannot disagree about what the lane contains.
            source scripts/build/fixture-lane.sh
            coords="$(nros_lane_coords_file "${NROS_FIXTURE_LANE}")"
            NROS_FIXTURE_SCOPE=coords NROS_FIXTURE_COORDS="$coords" \
                NROS_FIXTURE_SCOPE_ORIGIN="lane:${NROS_FIXTURE_LANE}" \
                exec ./scripts/check-fixtures-stale.sh
            ;;
    esac

# Run all tests including Zephyr, ROS 2 interop, C API, XRCE, NuttX, FreeRTOS, large_msg
# Single nextest run (entire workspace) + Miri + C codegen
#
# Fixtures are NOT auto-built — run `just build-test-fixtures` first.
# issue 0348 / 0393 — `test-zpico-multisession` is a DEPENDENCY here, not a step
# on the `ci` line (issue 0319's rule: a suite named only on `ci` is a suite
# `just check` never runs). It cannot fold into the nextest run below because it
# needs `ZPICO_MAX_SESSIONS=2` — a BUILD input — and its own target dir.
#
# It was wired into `just test` (the dev tier) and nowhere else, so `just ci`,
# `ci-matrix` and `ci-full` all reached `test-all` WITHOUT it and the
# multi-session paths ran in no CI tier at all. That is the state 0393 set out
# to fix: `two_sessions_deliver_cross_session_through_router` and `loan_e2e`
# skipped on every host in every tier, so phase-328's session pool was never
# executed by CI. Three tests, ~14 s.
[group("full-matrix")]
test-all verbose="": _require-fixtures-ready test-zpico-multisession
    #!/usr/bin/env bash
    # issue 0659 — reap peer process groups a previous SIGKILLed run left behind,
    # BEFORE nextest starts. Not mid-run: a concurrent test's peers are recorded
    # and alive, so a sweep then would kill them. Orphans hold DDS discovery
    # ports and surface later as `failed to bind to ANY:8650: address in use` on
    # an unrelated test.
    cargo run -q -p nros-tests --bin nros-peer-sweep 2>/dev/null || true
    # phase-373 W1 — a nextest filter that selects nothing is not an error and
    # not visible: `show-config test-groups` prints the override either way, and
    # a filter whose OTHER disjunct matches leaves the group looking populated.
    # That is how `zephyr-qos-port` sat switched off since phase-329. The
    # `binary()` half is gated statically on the fast line; the `test()` half
    # needs the test list, which only this lane has.
    just check nextest-test-filters
    source scripts/build/cargo.sh
    source scripts/test/nextest-profile.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    nextest_run_profile_args=($(nros_nextest_run_profile_args))
    nextest_fail_fast_args=($(nros_nextest_fail_fast_args))
    set +e
    failed=0
    just init-test-logs
    # The workspace run below uses DEFAULT features, so every target behind
    # `required-features` is silently absent from it — cargo does not report
    # such a target as filtered, deselected or skipped, it simply never builds
    # it. `custom_transport_loopback` is the one target behind `rmw`, and it
    # needs native fixtures, which is why it belongs HERE (`test-all` depends on
    # `_require-fixtures`) rather than in the fixture-free
    # `check::required-features-tests`.
    #
    # BEFORE the main run, not after, and that ordering is load-bearing: every
    # `cargo nextest` invocation rewrites `junit.xml`, and `_rewrite-skipped-junit`
    # re-snapshots `junit-real.xml` from it. A junit-writing lane placed in the
    # tail therefore overwrites BOTH — the main sweep's real-failure record
    # (issue 0527's whole purpose) and the input `_check-skip-budget` asserts on
    # (issue 0584) — leaving a one-test file that reports `0 skip(s)` no matter
    # what the sweep did. Here it is harmless: the `rm -f "$junit"` below clears
    # it, and this lane reports its own verdict through its exit status.
    echo "=== Required-features fixture tests ==="
    just _nextest-platform custom_transport_loopback "{{verbose}}" "--features rmw" || failed=1
    args=(--workspace "${nextest_run_profile_args[@]}" "${nextest_fail_fast_args[@]}")
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    # Phase 185.2 / 186.4 — toolchain-gated exclusion of embedded-RTOS Cyclone
    # tests. Since Phase 186 the embedded Cyclone backend self-provisions from
    # source via CMake (no `build/cyclonedds-<rtos>-install` artifact any more),
    # so the gate is the CROSS TOOLCHAIN: if it's present the example build can
    # self-provision + boot, so run the tests; if it's absent (lighter tier),
    # filter them OUT so they report `skipped`, not `failed` (`skip!` is a panic
    # ⇒ a nextest failure; only *filtering* yields a skip).
    env_exclude=()
    # RFC-0061 / phase-318 W4 — scope the RUN to the lane. `NROS_TEST_SCOPE=native`
    # (tier 1) drops every non-host binary; the exclusions are DERIVED from
    # `PlatformId`, so adding a platform extends them with no second edit
    # (ci_lane::tests::lane_filter_tokens_cover_every_non_native_platform gates it).
    while IFS= read -r _lane_expr; do
        [ -n "$_lane_expr" ] && env_exclude+=("$_lane_expr")
    done < <(bash scripts/test/lane-filter.sh "${NROS_TEST_SCOPE:-all}")
    source scripts/test/toolchain-gate.sh   # phase-300 W4 — shared predicate (issue-0030 lockstep)
    nros_toolchain_present arm-none-eabi \
        || env_exclude+=("not (binary(freertos_qemu) and test(~cyclonedds))")
    nros_toolchain_present riscv64-elf \
        || env_exclude+=("not (binary(threadx_riscv64_qemu) and test(~cyclonedds))")
    # Issue 0030 — deselect OPTIONAL-toolchain suites when their toolchain is
    # absent, the same way the embedded-Cyclone tests above are gated. These
    # suites already `nros_tests::skip!` at runtime (→ `[SKIPPED]` panic →
    # rewritten to `<skipped>` by `_rewrite-skipped-junit`, so they never count
    # as real failures), but the *live nextest console* still shows the skip!
    # panic as a red FAIL — the "non-bug failure" a user shouldn't have to fight.
    # Filtering deselects them entirely: no scary console line, no wasted in-test
    # build attempt. Each suite runs (and skip!s with an actionable reason) the
    # moment its toolchain is present, so this only loosens lighter tiers.
    if ! { command -v idf.py >/dev/null 2>&1 || [ -n "${IDF_PATH:-}" ] || [ -n "${NROS_ESP_IDF_ENV_SHIM:-}" ]; }; then
        env_exclude+=("not binary(cli_bringup_esp_idf)")
        env_exclude+=("not binary(esp32_idf_talker_builds)")
        env_exclude+=("not binary(esp32_idf_listener_builds)")
    fi
    # ros_editions (phase-309): the multi-edition harness lanes are OPT-IN — they
    # need docker, a slow-to-build `nano-ros-ros:<edition>` image, AND a
    # per-edition-regenerated publisher fixture (not part of build-test-fixtures).
    # Always deselect from the default sweep so `just ci` never depends on docker;
    # run them explicitly with `just ros_editions ci <distro>`.
    env_exclude+=("not binary(~ros_editions)")
    if ! bash scripts/zephyr/resolve-fvp-bin.sh >/dev/null 2>&1; then
        env_exclude+=("not binary(fvp_smoke)")
        # phase-298 W4 — the legacy fvp_runtime/fvp_runtime_rust binaries are
        # deleted; fvp_runtime_ws is the runtime gate over ws-entry.
        env_exclude+=("not binary(fvp_runtime_ws)")
        # board_import west-builds the FVP board (needs the FVP SDK gate).
        env_exclude+=("not binary(board_import)")
    fi
    # zephyr west build-fixtures (issue 0041): deselect when west / a provisioned
    # Zephyr workspace is absent — the west fixtures can't be built there. Mirror
    # the workspace-discovery ladder scripts/build/west-fixtures.sh uses (explicit
    # ZEPHYR_BASE/NROS_ZEPHYR_WORKSPACE, in-repo, or the sibling
    # ../nano-ros-workspace[-4.4] a `just zephyr setup` lands) so a sibling-layout
    # host still RUNS these instead of wrongly deselecting buildable fixtures.
    if ! command -v west >/dev/null 2>&1 \
        || { [ -z "${ZEPHYR_BASE:-}" ] \
             && [ ! -d "${NROS_ZEPHYR_WORKSPACE:-/nonexistent}/zephyr" ] \
             && [ ! -d zephyr-workspace/zephyr ] \
             && [ ! -d ../nano-ros-workspace/zephyr ] \
             && [ ! -d ../nano-ros-workspace-4.4/zephyr ]; }; then
        env_exclude+=("not binary(cli_bringup_zephyr)")
        env_exclude+=("not binary(zephyr_self_pkg)")
        env_exclude+=("not binary(board_import)")
    fi
    if ! command -v qemu-system-riscv32 >/dev/null 2>&1 || ! command -v espflash >/dev/null 2>&1; then
        env_exclude+=("not binary(esp32_emulator)")
    fi
    if [ "${#env_exclude[@]}" -gt 0 ]; then
        env_filter="${env_exclude[0]}"
        for _e in "${env_exclude[@]:1}"; do env_filter="$env_filter and $_e"; done
        echo "test-all: toolchain-gated suites filtered OUT (reported deselected, not failed); install the toolchain to run them: $env_filter"
        args+=(-E "$env_filter")
    fi
    nros_nextest_record_begin test-all
    nros_nextest_record_write_command \
        cargo nextest run "${cargo_nextest_args[@]}" "${NROS_NEXTEST_RECORD_ARGS[@]}" "${args[@]}"
    nros_nextest_junit_reset
    cargo nextest run "${cargo_nextest_args[@]}" "${NROS_NEXTEST_RECORD_ARGS[@]}" "${args[@]}"
    nextest_exit=$?
    # Read back whichever candidate nextest wrote — the store does not follow
    # CARGO_TARGET_DIR, so this cannot be resolved before the run.
    junit="$(nros_nextest_junit_path)"
    # Phase 214.R.1: rewrite [SKIPPED] failures → <skipped> before tallying.
    just _rewrite-skipped-junit "$junit" || true
    real_failures=$(just _count-real-failures "$junit")
    if [ "$nextest_exit" -ne 0 ] && [ ! -f "$junit" ]; then
        failed=1
    elif [ "$nextest_exit" -ne 0 ] && [ "$real_failures" -gt 0 ]; then
        failed=1
    fi
    echo ""
    just _test-summary "$junit"
    echo ""
    just _nextest-slow-tests "$junit"
    echo ""
    nros_nextest_record_finish
    echo ""
    echo "=== Doctests ==="
    just test-doc || failed=1
    echo ""
    echo "=== Miri ==="
    just test-miri || failed=1
    echo ""
    echo "=== C Codegen Tests ==="
    just native _test-c-codegen {{verbose}} || failed=1
    echo ""
    echo "JUnit XML:  $junit"
    echo "Other logs: {{LOG_DIR}}/latest/"
    # Issue 0584 — assert the skips HERE too. The budget was first wired into the
    # three `$real`-counting tails, and `test-all` — the recipe `ci-matrix`
    # actually runs — reports through this path instead, so a full sweep never
    # reached it. Same class as the fixture gate: the sites that were found got
    # the fix, the site in use did not.
    just _check-skip-budget || failed=1
    if [ $failed -ne 0 ]; then
        echo "FAIL: Some tests failed."
        exit 1
    else
        echo "All tests passed!"
    fi

# Phase 146.3 — embedded-RTOS Rust-link regression gate.
#
# `cargo build` of one Rust example per hosted-RTOS that ships an
# embedded zenoh-pico variant (FreeRTOS, NuttX, ThreadX-Linux).
# These three are the targets whose link-symbol drift between
# `platform_aliases.c`, the zenoh-pico vendor TUs, and the
# `LinkPolicy` mask surfaced as Phase 146 A/B/C. Catches the next
# regression of the same shape (duplicate `_z_task_*`, undefined
# `_z_*_serial_internal`, etc.) immediately during `just ci`
# rather than during `just test-all`'s full QEMU sweep.
#
# Best-effort: each RTOS's build skips cleanly if its cross
# toolchain or board crate prerequisites are absent.
[private]
rust-rtos-link-check: _codegen
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    # Issue 0511 — build each leaf at ITS PLATFORM's profile, not the ambient
    # one. This lane used `nros_cargo_profile_arg_string` for all three, which
    # resolves to `nros-relwithdebinfo` — whose own comment in Cargo.toml reads
    # `opt-level = 3   # Performance. Size lives in nros-minsizerel`. Both ARM
    # leaves carve out to `nros-minsizerel` for exactly the reason this lane
    # exists: they link into a fixed ROM. Measured on the nuttx talker, same
    # revision: 538800 bytes of ROM overflow at the ambient profile vs 419992 at
    # the platform's — so the mismatch was ~119 KB of the figure that made the
    # lane unreadable (the remaining ~420 KB is the real defect 0511 bisects).
    #
    # Building what the platform ships also stops this lane writing a SECOND
    # profile directory beside the fixtures — the shape phase-340 P2 names as a
    # permanent false-STALE source when a probe and a builder disagree.
    # These two wrote `examples/<leaf>/target/` — a plain `cd <leaf> && cargo
    # build`, which is verbatim the second build path phase-340 P2 names. The
    # gate `check::example-leaf-target-dirs` calls that shape either residue or
    # "a writer this gate cannot see"; it was the latter, found by deleting the
    # dirs and watching exactly these two come back during `just ci`.
    #
    # NOT the shared fixture group: `nros_fixture_target_dir_flag` defers RTOS
    # rows, so it returns empty for both platforms and the build would fall
    # straight back to the leaf `target/`. A per-leaf `target-*` sibling is what
    # the threadx line below already uses, it is globally gitignored, and it
    # keeps ONE workspace root per target dir — which is the constraint issue
    # 0616 is about, and the reason these must not simply share a directory.
    echo "== Phase 146.3 — embedded-RTOS Rust link check =="
    if command -v arm-none-eabi-gcc >/dev/null; then
        echo "  freertos talker ($(nros_cargo_platform_profile freertos)):"
        # #60 T5: the freertos talker Node pkg is platform/RMW-agnostic now —
        # the `rmw-zenoh` parity feature was removed (RMW flows from the board
        # crate). Build with default features, mirroring the nuttx talker below.
        mapfile -t freertos_profile < <(nros_cargo_profile_args_for "$(nros_cargo_platform_profile freertos)")
        ( cd examples/qemu-arm-freertos/rust/talker && cargo build "${freertos_profile[@]}" --target-dir target-link-check ) >/dev/null
        echo "  nuttx talker ($(nros_cargo_platform_profile nuttx)):"
        mapfile -t nuttx_profile < <(nros_cargo_profile_args_for "$(nros_cargo_platform_profile nuttx)")
        ( cd examples/qemu-arm-nuttx/rust/talker && cargo build "${nuttx_profile[@]}" --target-dir target-link-check ) >/dev/null
    else
        echo "  [SKIPPED] freertos + nuttx: arm-none-eabi-gcc not installed"
    fi
    # threadx-linux is HOSTED — no ROM region, so the ambient profile is right
    # for it and `nros_cargo_platform_profile` returns exactly that. Routed
    # through the same accessor anyway, so the three leaves read alike and a
    # future carve-out reaches this one without an edit here.
    echo "  threadx-linux talker ($(nros_cargo_platform_profile threadx-linux)):"
    mapfile -t threadx_profile < <(nros_cargo_profile_args_for "$(nros_cargo_platform_profile threadx-linux)")
    ( cd examples/threadx-linux/rust/talker && \
        cargo build "${threadx_profile[@]}" --no-default-features --features rmw-zenoh --target-dir target-zenoh ) >/dev/null
    echo "Rust-RTOS link check OK."

# Run CI: format check + clippy + every test tier (never modifies code).
# `test-all` already covers test-doc + test-miri internally. Phase
# 117.16 — `cyclonedds::ci` runs the C++ Cyclone DDS RMW backend's
# CTest harnesses (entity smoke + POSIX E2E vs stock
# `rmw_cyclonedds_cpp`). Phase 146.3 adds the `rust-rtos-link-check`
# gate ahead of `test-all` so the embedded-RTOS link-symbol
# regression class surfaces immediately on `just ci`.
# RFC-0061 / phase-318 W4 — the tier ladder. `ci` is TIER 1: the lane a developer
# can afford to run per task. It gates only host fixtures and runs only host
# binaries, so a stale ThreadX fixture cannot block it — which is exactly what
# happened on 2026-07-28, when every code stage passed and 40 cross-platform
# workspace fixtures failed the preflight of a native-intent run.
#
#   just ci               tier 1 — every commit / pre-push
#   just ci matrix        tier 2 — when the diff touches packages/core, codegen, cmake/
#   just ci matrix-nightly       — the pairwise cover, nightly
#   just ci full          tier 3 — pre-release, on demand (the former `ci`)
#
# SSoT for which test BUCKET runs at which tier: `nros_tests::buckets::BUCKET_TIERS`
# (phase-329 W7). `CiTier::just_recipe` names these four recipes, and
# `ci_tier_ladder_matches_justfile_recipes` fails if this ladder and those recipe
# names ever drift. The cell-COORDINATE selection within tiers 1/2/nightly is the
# separate `nros_tests::ci_lane` computation (emitted by the `lane-coords` bin).
#
# Issue 0393 — tier 1's BUILD narrows too, not just its gate and its run:
#
#     just build-test-fixtures lane=native   # one module, ~180 of 337 rows
#     just ci
#
# `NROS_FIXTURE_LANE=native` makes `_require-fixtures` accept that scoped build
# and — the other half — REJECT it for an unscoped `test-all`, so a tier-1 stamp
# can no longer vouch for a tier-3 run.
#
# Why `native` and not `tier1`: this lane scopes its run with
# `NROS_TEST_SCOPE=native`, which selects every native test BINARY. That is a
# broader set than `coords(Tier1)` (10 of 50 coordinates), so building only the
# tier-1 coordinates would leave the remaining native binaries absent and the
# run would mass-fail "Binary not found". The build set has to cover the run
# set, not the gate set.
# Phase-395 W2 — the lanes, named. See
# docs/development/multi-agent-ci-workflow.md for why they are cut by EXECUTION
# COST rather than by combinatorial coverage.
#
# L0 and L1 need NO FIXTURES. That is not a new property, it is one this tree
# already had and did not expose: only `test` and `test-all` depend on
# `_require-fixtures-ready`, while `check::fast`, `check::build` and `test-unit`
# do not. Measured 2026-08-28: 89 of 163 test FILES call a fixture resolver and
# 74 do not, so a large share of the suite was gated behind a precondition it
# never needed.
#
# An agent or an outside contributor can run L0+L1 on a fresh clone with no SDK,
# no QEMU and no cross toolchain.

# L0 is `just ci-fast`, which already exists (`check::fast check::no-std`). It is
# NOT re-spelled here as `ci-l0`: a second name for one lane is the "two
# spellings" defect this tree keeps paying for, and the map from lane to verb
# belongs in the doc, not in a duplicate recipe.

# phase-395 W17 — does each platform's CI evidence SUPPORT the tier it claims?
#
# The network half of `check::board-tiers`. That gate is in `check::fast`: offline
# and deterministic, so it can prove a tier's obligation is STRUCTURALLY met and
# cannot know whether the lane is GREEN — it printed "Board support tiers match
# the evidence" while the 0 7 nightly failed on three platforms.
#
# REPORTS, NEVER GATES. Exit 0 whatever the evidence says; only a broken tool
# (no gh, not authenticated, unreadable registry) exits non-zero. There is
# deliberately no `--check`: Rust's and Zephyr's tier policies both refuse to
# auto-demote on a red, because auto-demotion is the pressure that makes people
# silence tests. A tier changes when someone decides, with a record.
[group("ci")]
tier-health *args:
    @python3 scripts/ci/tier-health.py {{args}}

# Did the last nightly REPORT, or merely go red? (issue 0878) Never gates.
[group("ci")]
nightly-triage runs="3":
    @python3 scripts/ci/nightly-triage.py --runs {{runs}}

# Which open PRs can never merge because nothing ever ran on them? A PR with
# ZERO check suites is not failing and not pending — it is silently ineligible,
# and auto-merge can sit armed against a check that was never requested (PR #71,
# thirteen hours). Read-only; never gates (needs network + authenticated gh).
#
#   just pr-verdicts           just pr-verdicts --min-age 5
[group("setup")]
pr-verdicts *ARGS:
    @bash scripts/ci/pr-verdict-check.sh {{ARGS}}

# Triage a merge-queue ejection — answers the one question GitHub cannot:
# is this MY defect, or is the check red for everybody? Re-queuing an unchanged
# commit re-runs the same tree and fails the same way, one batch slot at a time.
#
#   just queue-triage          just queue-triage 6
[group("setup")]
queue-triage *ARGS:
    @bash scripts/ci/queue-triage.sh {{ARGS}}

# Show (or apply) the merge-queue + branch-protection settings — phase-395 W7.
# Read-only by default; `--apply` touches branch protection, which affects
# everyone, so it is never the default. `--status` shows what is set now.
[group("setup")]
merge-queue *ARGS:
    @scripts/ci/enable-merge-queue.sh {{ARGS}}

# --- self-hosted runner (phase-395 W6) ---------------------------------------
# Thin callers for scripts/ci/runner-*.sh. These are OPERATOR verbs, not CI
# steps: none run inside a workflow.

# Verify a self-hosted runner HAS what its labels claim. Read-only.
#
# A runner labelled `nros-sdk-zephyr` without the SDK wins jobs it cannot run,
# and the red lands on the PR author's change looking like a code failure. That
# is the most expensive kind of CI red, because it sends the wrong person
# looking in the wrong place.
[group("setup")]
runner-doctor labels *ARGS:
    @scripts/ci/runner-doctor.sh {{labels}} {{ARGS}}

# Make a runner's labels TRUE, reusing `just setup <platform>` so a runner and a
# contributor provision identically. `nros-ros2` is not provisionable here — it
# needs root, and nothing in this repo sudos; the script prints the command and
# fails honestly rather than pretending.
[group("setup")]
runner-provision labels *ARGS:
    @scripts/ci/runner-provision.sh {{labels}} {{ARGS}}

# Register this machine as an EPHEMERAL self-hosted runner (needs `gh` auth).
# Runs the doctor FIRST and refuses a host that fails it. Installing the systemd
# service is opt-in (`--with-service`) because it is the only sudo in these
# scripts — the default registers and prints the commands for a human.
[group("setup")]
runner-register labels *ARGS:
    @scripts/ci/runner-register.sh {{labels}} {{ARGS}}

# Prefer this to `runner-register` when the machine carries unrelated work —
# which every machine here does. It refuses `--privileged` and a Docker-socket
# mount rather than trusting the operator to remember, and that refusal IS the
# security argument: it holds only because our self-hosted lanes need neither
# (measured — no Docker, no KVM, no device access in any of the four; QEMU runs
# `-icount`, which is incompatible with KVM anyway). A job that must BUILD an
# image breaks the premise, and then the answer is a microVM, not a flag.
#
# Needs GH_REPO and a short-lived RUNNER_TOKEN in the environment.
#
# Build the image and start an EPHEMERAL runner in an unprivileged container.
[group("setup")]
runner-container labels *ARGS:
    @scripts/ci/runner-container.sh {{labels}} {{ARGS}}

# The whole procedure in ONE verb, for the common case. Resolves the repo from
# `gh repo view` (the origin remote — never a hardcoded default, which would
# silently attach a fork's runner to the wrong repo) and mints a registration
# token via `gh api`. Missing or non-admin `gh` is reported as what it is, with
# the manual form to copy: `--repo OWNER/REPO`, `--token -` to read stdin.
#
# Prefer `RUNNER_TOKEN=… just runner-up …` or `--token -` over `--token <val>`:
# an argv token is visible in `ps` and lands in shell history.
#
# Stops short of `just merge-queue --apply --self-hosted-ready` on purpose —
# that changes repo-wide settings, so it stays a human's decision.
#
# Stand up a contained self-hosted runner in one command. Takes --check.
[group("setup")]
runner-up labels *ARGS:
    @scripts/ci/runner-up.sh {{labels}} {{ARGS}}

# Keep an ephemeral runner AVAILABLE: register, take one job, sweep, repeat.
#
# Not a convenience wrapper. `--ephemeral` retires the runner after every job,
# and `L3 (cross build + link)` is a REQUIRED check only a self-hosted runner
# can satisfy — so with nothing re-registering, the second queue entry waits on
# a verdict from a runner that no longer exists. A required check that never
# reports blocks forever instead of failing, and it reads as GitHub being slow
# rather than as breakage. This is the "something must re-register" that
# `runner-register` tells you to supply.
#
# Foreground and long-lived: run it under tmux/systemd. --once / --max N bound it.
[group("setup")]
runner-loop labels *ARGS:
    @scripts/ci/runner-loop.sh {{labels}} {{ARGS}}

# Between jobs: reap orphaned process groups and run budgeted disk GC.
# On a shared runner one leaked peer is every later job's flake. Takes --check.
[group("setup")]
runner-sweep *ARGS:
    @scripts/ci/runner-sweep.sh {{ARGS}}

# phase-395 W10 — the fixture cache in SHADOW MODE. It computes the key, lets
# the build happen anyway, and records whether the key PREDICTED the artifact.
#
# There is no lookup verb and no restore verb, deliberately. For a staleness
# PROBE an incomplete input set is survivable: it errs toward rebuilding. For a
# CACHE there is no fallback — a hit skips the build, so an incomplete key does
# not cost a redundant rebuild, it silently serves a WRONG artifact. That is the
# museum-binary failure mode with its one safeguard removed.
#
# Two of the four known-invisible input classes are NOT in the key. Run
# `just fixture-cache-coverage` before believing a quiet report.
[group("test")]
fixture-cache-shadow *args:
    cargo run -q -p nros-tests --bin fixture-cache-shadow -- {{args}}

# Re-observe every artifact already in the shadow store, then report. Run this
# after each change the W10 spec names — a rebase, a Kconfig edit, an env
# change, a linker-flag change, a toolchain bump — because each is a test the
# key has to pass. Seed the store first, either with
# `just fixture-cache-shadow record <artifact>...` or by running a sweep under
# NROS_FIXTURE_CACHE_SHADOW=1, which makes the resolvers record what they
# resolve FRESH.
[group("test")]
fixture-cache-shadow-sweep:
    #!/usr/bin/env bash
    set -uo pipefail
    # `record` exits non-zero when an artifact REFUSED to be observed; that is
    # information for the report, not a reason to skip it.
    cargo run -q -p nros-tests --bin fixture-cache-shadow -- record --all-known
    cargo run -q -p nros-tests --bin fixture-cache-shadow -- report

# Which known-invisible build inputs the key covers, and which it records as
# UNCOVERED with the reason (issues 0475 / 0491 / 0460 / 0627).
[group("test")]
fixture-cache-coverage:
    cargo run -q -p nros-tests --bin fixture-cache-shadow -- coverage

# Re-run the failures from the last run ALONE, which is the evidence that
# separates a flake from a defect — phase-395 W5.
#
# "It failed once" is indistinguishable from a real intermittent defect, and
# quarantining a real defect is how it ships. A test that fails in a 32-way
# sweep and passes solo with a wide margin was STARVED; one that fails both ways
# is broken. This produces that comparison from the junit the red run already
# wrote, so nobody has to reconstruct the failing set by hand.
#
# Output is the evidence line for a `.config/flake-quarantine.toml` entry.
[group("test")]
retest-failures-solo junit="":
    #!/usr/bin/env bash
    set -euo pipefail
    junit="{{junit}}"
    if [ -z "$junit" ]; then
        junit="target/nextest/default/junit-real.xml"
        [ -f "$junit" ] || junit="target/nextest/default/junit.xml"
    fi
    if [ ! -f "$junit" ]; then
        echo "no junit at $junit — run a test lane first; this reads ITS failures." >&2
        exit 1
    fi
    filter="$(python3 scripts/test/failed-filterset.py "$junit")"
    if [ -z "$filter" ]; then
        echo "no real failures in $junit — nothing to re-run solo."
        exit 0
    fi
    echo "re-running solo (--test-threads=1), one at a time:"
    python3 scripts/test/failed-filterset.py "$junit" --names | sed 's/^/    /'
    echo
    # --test-threads=1 is the point: the hypothesis under test is CONTENTION,
    # so a solo re-run that keeps the parallelism proves nothing.
    source scripts/build/cargo.sh
    set +e
    NROS_NEXTEST_FILTER="$filter" just _nextest-tolerant --no-fail-fast --test-threads=1
    rc=$?
    set -e
    echo
    if [ $rc -eq 0 ]; then
        echo "PASSED SOLO. That is the flake evidence: same tests, no contention."
        echo "  Record the MARGIN (solo duration vs the lane's timeout) in the"
        echo "  entry's \`evidence\` — a 16x margin means starved, a 1.2x margin"
        echo "  means the budget is simply too tight and quarantine is the wrong fix."
    else
        echo "FAILED SOLO TOO — this is a DEFECT, not a flake. Do not quarantine it."
    fi
    exit $rc

# Claim a unit of work — `refs/claims/<id>` — ATOMICALLY across parallel agents
# and across machines, by the same origin-side compare-and-swap `just issue-new`
# uses: a push creating a ref that already exists is REJECTED, so the SERVER
# arbitrates rather than trust. phase-395 W8.
#
# Unlike an issue id, a claim EXPIRES (default 4h) — an agent that dies must not
# freeze the work behind it. The TTL is hours because an OPEN PR SUPERSEDES the
# claim: the ref only governs the window before first push.
#
# Claims are ADVISORY. There is no mechanical map from a diff to a claim, so
# nothing enforces this — it prevents accidental duplication between COOPERATING
# agents and nothing more. Outside contributors cannot write refs at all, so
# check the GitHub issue's assignee too.
#
#   just claim issue-0827        just claim phase-392-W3a --ttl 2
#
# rc: 0 ok | 2 held (live) | 3 remote unreachable | 4 held (expired, use steal)
[group("docs")]
claim id="" *flags:
    @scripts/reserve-claim.sh claim {{id}} {{flags}}

# Refresh a lease. Idempotent, and it re-claims a released id so a timer works
# from its first tick. Drive it from a LIVENESS supervisor while the agent
# process is alive — not between the agent's steps, or a 40-minute fixture build
# looks like death.
[group("docs")]
claim-renew id="" *flags:
    @scripts/reserve-claim.sh renew {{id}} {{flags}}

# Keep a claim alive while its agent is — phase-395 W14. Renewal keyed on the
# PROCESS being alive, never on progress: a 40-minute fixture build is not
# death, and a claim that lapses mid-task lets another agent steal live work
# with the tooling's blessing.
#
#   just claim issue-0827
#   just claim-supervise issue-0827 --pid $$ &
[group("docs")]
claim-supervise id="" *flags:
    @bash scripts/ci/claim-supervisor.sh {{id}} {{flags}}

# Drop a claim on completion. Do NOT wait for expiry, or every finished task
# leaves hours of phantom occupancy.
[group("docs")]
claim-release id="" *flags:
    @scripts/reserve-claim.sh release {{id}} {{flags}}

# Take an EXPIRED claim. `--force-with-lease` means two agents racing to steal
# one dead claim cannot both win. Prints what the dead agent left — a pushed
# `fix/<id>` branch nobody would look for, partially landed commits — and says
# to comment on the issue, because an agent dying there is otherwise invisible
# and recurs.
[group("docs")]
claim-steal id="" *flags:
    @scripts/reserve-claim.sh steal {{id}} {{flags}}

# Every claim on origin, with holder, age and state.
[group("docs")]
claim-list:
    @scripts/reserve-claim.sh list


# ---------------------------------------------------------------------------
# The CI ladder lives in `just/ci.just` as a MODULE — `just ci <lane>`.
#
# `mod`, not `import`, and this is the one family where that is right: "which
# tier" is the question a person is actually asking, so a namespace reads
# better than seven flat names. Phase-399 chose `import` for the 200 `check-*`
# gates on the opposite reasoning — nobody types those, and `mod` would have
# renamed every call site.
#
# `just ci` still means tier 1: a module invoked with no argument runs its
# `default` recipe. That keeps 594 references across 262 files true, most of
# them historical records of what someone ran, which a rename would have
# falsified rather than updated.
#
# The flat spellings below are thin forwarders for the same reason. They are
# not a second implementation — each is one line delegating into the module —
# and they are what the remaining ~180 references in docs and issues resolve to.
mod ci 'just/ci.just'

# The lane-contract tests: `[[fixture]]`/`[[workspace_fixture]]` bookkeeping,
# checked WITHOUT building a fixture.
#
# Issue 0922 — these live in `nros-tests`, which `test-unit` excludes wholesale
# because that crate's tests generally need `just build-test-fixtures` staged
# first. These do not: they read the manifest and shell out to
# `fixtures-manifest.py`, and the whole target runs in ~0.2 s. The exclusion was
# by CRATE while the real property is per-TARGET, so the one lane everybody runs
# before every push could not see them — and a commit that changed the build-side
# lane predicate while leaving the run-side model behind landed red and stayed
# red on its PR, which is exactly what these tests exist to prevent.
#
# Admissible in an affordability tier under the `check-lane-contracts` rule: no
# fixture stamp is resolved and no fixture is built. Keep it that way — a target
# added here that needs a staged artifact breaks the tier's promise.
[group("main")]
test-lane-contracts:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    cargo nextest run "${cargo_nextest_args[@]}" -p nros-tests \
        --test lane_run_narrowing --test matrix_fixture_coverage

[group("ci")]
ci-l1:
    @just ci l1

[group("ci")]
ci-l3:
    @just ci l3

[group("ci")]
ci-matrix:
    @just ci matrix

[group("ci")]
ci-matrix-nightly:
    @just ci matrix-nightly

[group("ci")]
ci-full:
    @just ci full

[group("ci")]
ci-fast:
    @just ci fast






# Run the staleness gate over exactly one lane's fixture coordinates.
# Separate recipe because `ci-matrix` and `ci-matrix-nightly` differ ONLY in which
# lane they name — the shared helper, not a second spelling.
[group("ci")]
[private]
_lane-gate lane:
    #!/usr/bin/env bash
    set -euo pipefail
    coords="$(mktemp)"
    trap 'rm -f "$coords"' EXIT
    cargo run -q -p nros-tests --bin lane-coords -- {{lane}} > "$coords"
    echo "[{{lane}}] $(wc -l < "$coords") fixture coordinate(s):"
    sed 's/^/  /' "$coords"
    NROS_FIXTURE_SCOPE=coords NROS_FIXTURE_COORDS="$coords" \
        bash scripts/check-fixtures-stale.sh

# phase-318 W5.a — run ONE family's tests, then optionally free its artifacts.
#
# Tier 3 builds every family, then tests every family, so peak disk is the SUM of
# all of them: ~800 GB, and it hit 11 MB free twice on 2026-07-28 — which ended
# that run more decisively than any test failure. Interleaving
# build -> test -> drop keeps peak disk at roughly one family.
#
#   just <platform> build-fixtures        # build verbs differ per platform,
#   just sweep-family <platform> drop=1   # so the caller owns that step
#
# `drop=1` deletes that family's MANIFEST-DECLARED build dirs after its tests
# pass — reproducible artifacts; the result is what needs keeping. Default drop=0,
# and drop-family-artifacts.sh is dry-run by default: deleting build trees on a
# typo costs hours.
[group("ci")]
sweep-family platform drop="0":
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/test/nextest-profile.sh
    echo "[sweep-family] testing {{platform}}"
    # `skip!` is a panic, so a BARE nextest run reports skipped cells as
    # FAILURES — the documented pitfall (CLAUDE.md). `test-all` rewrites them via
    # the junit pass; mirror that here or every unavailable-toolchain family looks
    # red. `|| true` on the run, then the rewrite decides.
    rc=0
    cargo nextest run $(nros_nextest_run_profile_args) -E 'binary(~{{platform}})' || rc=$?
    just _rewrite-skipped-junit || true
    if [ "$rc" -ne 0 ]; then
        echo "[sweep-family] {{platform}} had failures (skips are rewritten in the junit;"
        echo "               read it before treating this as a code red)"
    fi
    if [ "{{drop}}" = "1" ] && [ "$rc" -eq 0 ]; then
        bash scripts/build/drop-family-artifacts.sh {{platform}} --confirm
    elif [ "{{drop}}" = "1" ]; then
        echo "[sweep-family] NOT dropping {{platform}} artifacts — the run was not clean;"
        echo "               you will want them to debug."
        exit "$rc"
    else
        bash scripts/build/drop-family-artifacts.sh {{platform}} || true
        echo "[sweep-family] artifacts kept (pass drop=1 to free them)"
    fi


# =============================================================================
# CI reorg (step A) — local mirrors of the standalone CI workflows + a fast lane.
# Goal: every CI job is runnable locally by a named recipe. These wrap the jobs
# whose workflow yml previously carried only raw-shell steps. The heavy lane stays
# `just ci` / `just test-all`; this is the fast per-push tier.
# =============================================================================

# Issue 0805 — the shared Corrosion cargo store only grows: a key dir is named
# by a hash of the leaf's configuration, so any key change orphans the old one.
# Reports by default; `just gc-shared-cargo --prune` deletes. Reachability is
# exact — a dir is live iff some leaf's `cargo` symlink resolves to it.
gc-shared-cargo *ARGS:
    @python3 scripts/build/gc-shared-cargo.py {{ARGS}}

# Issue 0805 — zephyr keeps ONE CARGO DIRECTORY PER PROFILE inside every west
# build dir and never removes the ones a later configure stopped using. Deliberately
# NOT the shared-target-dir treatment the other lanes got: the per-image generated
# headers live inside that directory and differ by RMW, so sharing it is the
# sizes-header class. Reports by default; `--prune` deletes.
gc-zephyr-builds *ARGS:
    @python3 scripts/build/gc-zephyr-builds.py {{ARGS}}

# Phase 376 W3.d — call sites that test an RMW status by its SIGN. Reporting
# only: the dual-return list is the CONTRACT today and changes with the slots.
[group("check")]
rmw-ret-sign:
    @python3 scripts/check-rmw-ret-sign.py

# Phase 376 W2 — how far our vtable is from mirroring upstream, slot by slot and
# arg by arg. REPORTING ONLY, deliberately not on the `check` line: `--check`
# fails by construction until the W3+ migration lands, and a gate that cannot
# pass is a gate people learn to skip. It joins `check` at the end of W3.
[group("check")]
rmw-abi-shape:
    @python3 scripts/rmw-abi-shape.py

# Phase 379 — report how the C / C++ / Rust user API differs from rclc / rclcpp
# / rclrs. Needs clang and the recorded ROS 2 surfaces under
# docs/reference/api-surface/ (committed, so no ROS install is required); the
# Rust lane also needs the nightly toolchain for rustdoc JSON.
#
#   just api-parity              # all three languages
#   just api-parity cpp          # one
#
# Re-derive the ROS 2 side after a distro or upstream bump:
#   scripts/api-parity.py --refresh --rclc <ros2/rclc checkout> \
#                                   --rclrs <ros2_rust/rclrs crate dir>
#
# Report the user API against rclc / rclcpp / rclrs.
[group("check")]
api-parity lang="":
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -n "{{ lang }}" ]; then
        python3 scripts/api-parity.py --lang "{{ lang }}"
    else
        python3 scripts/api-parity.py
    fi

# Static-memory report for a built image: RAM by symbol, by crate, and by
# declared pool, with the unattributed gap called out. Phase 392 opened with
# this table pasted by hand into a roadmap doc; every wave of that campaign is
# a saving measured against it, so it needs to be re-runnable.
#
#   just mem-report examples/native/rust/talker/target-zenoh/nros-fast-release/talker
#   just mem-report <elf> --json > before.json   # then --baseline before.json
[group("dev")]
mem-report *args:
    @python3 scripts/nros-mem-report.py {{args}}

# Scaffold-journey: a `nros new` project resolves end-to-end via the generated
# `[patch.crates-io]` path block (the `scaffold-journey` job in pr-checks.yml).
[group("ci")]
scaffold-journey: setup-cli
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)" scripts/ci/scaffold-journey-check.sh

# colcon-parity: the `local-msg-package` template must also build under stock
# colcon (the `colcon-parity` job in pr-checks.yml). Needs ROS 2 + colcon on the host;
# skips cleanly when colcon is absent.
[group("ci")]
colcon-parity:
    #!/usr/bin/env bash
    set -e
    if ! command -v colcon >/dev/null 2>&1; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip colcon-parity "colcon not found (apt install python3-colcon-common-extensions)"
        exit 0
    fi
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    cd examples/templates/local-msg-package
    # CI builds from a fresh checkout; locally, wipe colcon + per-pkg cargo
    # artifacts first so a stale generated msg crate (e.g. a pre-codegen-bump
    # sensor_msgs lacking RosMessage) can't produce a false failure.
    rm -rf build install log src/*/target src/*/generated
    colcon build --base-paths src --merge-install --event-handlers console_direct+
    test -x install/lib/consumer/consumer || { echo "consumer binary not produced"; exit 1; }
    file install/lib/consumer/consumer

# acceptance (local, from-source): scaffold + build + run a fresh project with the
# in-tree nros CLI — proves the documented user flow (bootstrap → new → sync →
# cargo build → run). The prebuilt fresh-machine CI twin died with release.yml
# (phase-288 D1/D2: source distribution, no prebuilt nros). Work dir under tmp/
# (gitignored). Note: the pre-288 recipe drove the Phase-222-removed `nros build`
# verb — builds go through the platform tool (cargo here), never `nros`.
[group("ci")]
acceptance: setup-cli
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    repo="$(pwd)"
    nros="$(nros_cli_bin)"
    work="$repo/tmp/acceptance"
    rm -rf "$work"; mkdir -p "$work"; cd "$work"
    NROS_REPO_DIR="$repo" "$nros" new accept_app --platform native --lang rust --use-case talker
    cd accept_app
    NROS_REPO_DIR="$repo" "$nros" sync
    cargo build
    # profile-literal-ok: unprofiled: accept_app is a plain `cargo build` smoke binary
    timeout 10 target/debug/accept_app 2>&1 | grep -q "accept_app"
    echo "acceptance OK."



# =============================================================================
# Test Infrastructure
# =============================================================================

# Kill orphaned test processes from previous runs
[group("maintenance")]
test-kill-orphans:
    #!/usr/bin/env bash
    echo "Killing orphaned test processes..."
    pkill -9 -f 'zenohd.*--listen.*--no-multicast' 2>/dev/null || true
    pkill -9 -f 'nano-ros/examples/.*/target/' 2>/dev/null || true
    pkill -9 -f 'nano-ros/examples/.*/build/' 2>/dev/null || true
    pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
    pkill -9 -f 'ros2 topic' 2>/dev/null || true
    pkill -9 -f 'ros2 service' 2>/dev/null || true
    pkill -9 -f 'ros2 action' 2>/dev/null || true
    echo "Done."

# Initialize timestamped log directory for non-nextest test output (QEMU, C)
[private]
init-test-logs:
    #!/usr/bin/env bash
    timestamp=$(date +%Y%m%d-%H%M%S)
    mkdir -p "{{LOG_DIR}}/$timestamp"
    ln -sfn "$timestamp" "{{LOG_DIR}}/latest"

# View JUnit XML test report (requires: npm install -g junit-cli-report-viewer)
[group("debug")]
test-report:
    @junit-cli-report-viewer target/nextest/default/junit.xml

# =============================================================================
# Workspace
# =============================================================================

# Build workspace (no_std, native)
# nros-c/nros-cpp and standalone RMW staticlib wrappers excluded from
# no_std native build: staticlib/cdylib requires panic handler unless a
# concrete platform feature supplies the right runtime.
[private]
build-workspace:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    cargo_nextest_args=($(nros_cargo_nextest_args))
    cargo build $cargo_profile_args --workspace --no-default-features \
        --exclude nros-c \
        --exclude nros-cpp \
        --exclude nros-rmw-zenoh-staticlib \
        --exclude nros-rmw-xrce-cffi-staticlib \
        --exclude nros-build-helpers \
        --exclude nros-zpico-build \
        --exclude nros-build-paths \
    # Mirror the build excludes: under `--no-default-features` nros-c /
    # nros-cpp reference the per-platform `nros_platform_log_write` ABI
    # (Phase 88 log facade default sink) which no platform impl supplies
    # without a platform feature, so their test binaries fail to link.
    # The staticlib wrappers need a panic handler. All four are covered
    # by the per-feature `test-*` matrices instead.
    cargo nextest run "${cargo_nextest_args[@]}" --workspace --no-run \
        --exclude nros-c \
        --exclude nros-cpp \
        --exclude nros-rmw-zenoh-staticlib \
        --exclude nros-rmw-xrce-cffi-staticlib \
        --exclude nros-build-helpers \
        --exclude nros-zpico-build \
        --exclude nros-build-paths \

# Build workspace for embedded target (Cortex-M4F)
# Excludes zpico-sys: requires native system headers for CMake build
# Excludes nros-tests: requires std (test framework dependencies)
# Excludes nros-c/nros-cpp/standalone RMW staticlib wrappers:
# staticlib/cdylib requires a platform-specific panic/runtime setup.
[private]
build-workspace-embedded:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    # issue 0287 — same DERIVED exclude list as check::workspace-embedded. These
    # two carried byte-identical 20-line hand-written copies; keeping one list
    # in one place is the point, since a member added to one and not the other
    # fails in whichever lane was forgotten, in an unrelated crate.
    excludes="$(bash scripts/build/host-only-members.sh)"
    cargo build $cargo_profile_args --workspace --no-default-features --target thumbv7em-none-eabihf \
        $excludes

# Format workspace code.
# #310 — PLAIN `cargo fmt` (never `--all`). Plain fmt stays inside the invoked
# workspace's members; `--all` follows path-deps across workspace boundaries into
# the vendored SUBMODULES under `packages/cli/third-party/` (nros-macros →
# ros-launch-manifest-model, nros-orchestration-ir → …-sched), reformatting
# another repo and leaving it `-dirty` (which then surfaces as a baffling
# `git rebase` error). Submodules are formatted separately in their own forks.
[private]
format-workspace: format-cli
    cargo +{{NIGHTLY}} fmt

# #310 — the in-tree `packages/cli` sub-workspace is EXCLUDED from the root
# workspace, so plain `cargo fmt` at the repo root never reaches it; without this
# its sources silently drift out of rustfmt-clean. Plain `cargo fmt` here formats
# only the cli members (NOT `--all`), so the vendored submodule path-deps stay
# untouched.
[private]
format-cli:
    #!/usr/bin/env bash
    set -e
    cd "{{justfile_directory()}}/packages/cli" && cargo +{{NIGHTLY}} fmt
    # Issue 0363 — reformatting in place makes the built CLI STALE, and
    # CLAUDE.md tells people to run `just format` before broad changes. So the
    # documented workflow creates the condition the guard then trips on, deep
    # in a later lane. Say so HERE, where the cause is obvious. Not an
    # auto-rebuild: compiling from a format recipe is surprising, and
    # `just setup-cli` is the sanctioned producer.
    # ASK the binary rather than recomputing the predicate here — this block
    # used to be a 15-line mtime walk, i.e. a third spelling of the same
    # question, and it shipped with a bug (git's repo-root-relative paths vs
    # this recipe's `cd`) that made it silently never fire.
    bin="{{justfile_directory()}}/packages/cli/target/release/nros"
    if [ -x "$bin" ] && ! "$bin" source-stamp >/dev/null 2>&1; then
        echo "[format-cli] NOTE: reformatting left the built CLI stale." >&2
        echo "             Rebuild before running codegen lanes:  just setup-cli" >&2
    fi

# Provision the pinned clang-format (SSoT: `.clang-format-version`) as a
# PROJECT-LOCAL binary at `build/clang-format/bin/clang-format` — exactly like
# `build/qemu/bin/`. clang-format output drifts across major
# versions, so pinning is the only way `just format` / `check-*-fmt` stay consistent
# between machines + CI. We fetch the exact-version, cross-platform PyPI `clang-format`
# WHEEL (a zip carrying a standalone `clang_format/data/bin/clang-format` binary) and
# extract just that binary — NO venv, NO `pip install`, NOTHING user-wide (pip is used
# only to *download* the right wheel for this host, with no cache footprint). Idempotent.
setup-clang-format:
    #!/usr/bin/env bash
    set -e
    want="$(cat .clang-format-version)"
    dest="build/clang-format"
    bin="$dest/bin/clang-format"
    if [ -x "$bin" ] && "$bin" --version 2>/dev/null | grep -q "$want"; then
        echo "clang-format $want already provisioned: $bin"; exit 0
    fi
    echo "Provisioning clang-format $want into $dest (project-local binary; no install) ..."
    mkdir -p "$dest/bin"
    tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
    # Download (NOT install) the platform wheel for THIS host — pip resolves the right
    # manylinux/macos tag. --no-cache-dir → no ~/.cache/pip footprint.
    python3 -m pip download --no-cache-dir --no-deps --only-binary=:all: \
        -d "$tmp" "clang-format==$want" >/dev/null
    whl="$(ls "$tmp"/clang_format-*.whl 2>/dev/null | head -1)"
    [ -n "$whl" ] || { echo "ERROR: clang-format==$want wheel not found for this host" >&2; exit 1; }
    # The wheel is a zip; the real standalone binary is clang_format/data/bin/clang-format.
    python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$whl" "$tmp/x"
    cp "$tmp/x/clang_format/data/bin/clang-format" "$bin"
    chmod +x "$bin"
    "$bin" --version

# Format C code (nros-c headers, zpico C, C examples) with the pinned clang-format
[private]
format-c:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/clang-format.sh
    CF="$(nros_clang_format)"
    echo "Formatting C code... ($CF)"
    find packages/api/nros-c/include -name '*.h' -not -name 'nros_generated.h' -print0 | xargs -0 "$CF" -i
    "$CF" -i packages/rmw/zenoh/zpico-zephyr/src/*.c packages/rmw/zenoh/zpico-zephyr/include/*.h
    # phase-338 — EVERY platform's C examples, not just native. The portability
    # gate compares copies line by line, so a formatter that reaches one copy and
    # not its siblings manufactures divergence: `examples/native/c/listener` was
    # clean while its five siblings had drifted, and matching them meant leaving
    # native unformatted. Same file, same rule, every platform.
    git ls-files -z 'examples/*/c/**/*.c' 'examples/*/c/**/*.h' ':!examples/px4/**' | xargs -0 "$CF" -i
    echo "C code formatted."

# Format C++ headers (nros-cpp) with the pinned clang-format
[private]
format-cpp:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/clang-format.sh
    CF="$(nros_clang_format)"
    echo "Formatting C++ headers... ($CF)"
    "$CF" -i packages/api/nros-cpp/include/nros/*.hpp
    # phase-338 — the C++ EXAMPLES were never formatted by anything. Same reason
    # as the C side: the portability gate compares line breaking across copies.
    git ls-files -z 'examples/*/cpp/**/*.cpp' 'examples/*/cpp/**/*.hpp' ':!examples/px4/**' | xargs -0 "$CF" -i
    echo "C++ headers + examples formatted."

# Format Python code with ruff. Phase 195.D — the colcon extension moved to the
# nros-cli repo with the retired packages/codegen submodule; no in-tree Python
# package remains to format (nros-cli's own CI owns it).
[private]
format-python:
    @echo "No in-tree Python package to format (nros-cli owns the colcon extension)."

# Run Miri to detect undefined behavior in embedded-safe crates (no FFI)
[group("debug")]
test-miri:
    @echo "Running Miri on embedded-safe crates..."
    CARGO_PROFILE_DEV_OPT_LEVEL=0 cargo +{{NIGHTLY}} miri test -p nros-serdes -p nros-core -p nros-params


# =============================================================================
# Static Analysis
# =============================================================================

# Inspect generated assembly for a function (requires cargo-show-asm)
# Usage: just show-asm <package> <function> [target]
# Examples:
#   just show-asm nros-serdes 'CdrWriter::write_string'
#   just show-asm nros-serdes 'CdrWriter::write_string' thumbv7m-none-eabi
#   just show-asm nros-core 'Duration::from_nanos'
[group("debug")]
show-asm pkg fn target="":
    #!/usr/bin/env bash
    set -euo pipefail
    args=(-p "{{pkg}}" --lib "{{fn}}" --rust)
    if [[ -n "{{target}}" ]]; then
        args+=(--target "{{target}}" --no-default-features)
    fi
    cargo asm "${args[@]}"

# Show llvm-mca throughput analysis for a function (requires cargo-show-asm)
# Usage: just show-asm-mca <package> <function> [target]
[group("debug")]
show-asm-mca pkg fn target="":
    #!/usr/bin/env bash
    set -euo pipefail
    args=(-p "{{pkg}}" --lib "{{fn}}" --mca)
    if [[ -n "{{target}}" ]]; then
        args+=(--target "{{target}}" --no-default-features)
    fi
    cargo asm "${args[@]}"

# List all non-inlined functions in a crate (useful for finding inspectable symbols)
# Usage: just show-asm-list <package> [target]
[group("debug")]
show-asm-list pkg target="":
    #!/usr/bin/env bash
    set -euo pipefail
    args=(-p "{{pkg}}" --lib)
    if [[ -n "{{target}}" ]]; then
        args+=(--target "{{target}}" --no-default-features)
    fi
    cargo asm "${args[@]}" || true

# Run Kani bounded model checking on core crates (requires kani-verifier)
# Proves panic-freedom, roundtrip correctness, and bounded behavior
[group("verification")]
verify-kani:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "=== Kani Verification ==="
    failed=0
    for crate in nros-serdes nros-core nros-params nros-ghost-types nros-node; do
        echo ""
        echo "--- Verifying $crate ---"
        cargo kani -p "$crate" || { echo "[FAIL] $crate"; failed=$((failed + 1)); }
    done
    echo ""
    echo "--- Verifying nros-c ---"
    cargo kani -p nros-c --features "rmw-zenoh,platform-posix,ros-humble" || { echo "[FAIL] nros-c"; failed=$((failed + 1)); }
    echo ""
    if [ "$failed" -gt 0 ]; then
        echo "[FAIL] $failed crate(s) failed verification"
        exit 1
    fi
    echo "[OK] All Kani proofs verified"

# Run Verus unbounded deductive verification (requires Verus toolchain)
# Proves properties for ALL inputs using Z3 SMT solver
[group("verification")]
verify-verus:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "=== Verus Verification ==="
    # phase-422 W2 — Verus is an SDK-store tool (`[tool.verus]`), so its
    # directory is CONSTRUCTED from the index pin rather than assumed to be
    # `./tools` (issue 0625). `--require` prints every store path it tried and
    # the provisioning command, which is strictly more than the bare
    # "not found at ./tools/verus" this used to say; the line below adds the
    # `just` spelling, which also installs the rust toolchain Verus pins.
    if ! VERUS_DIR="$(nros sdk-path verus --require)/bin"; then
        echo "Run 'just verification verus' to install" >&2
        exit 1
    fi
    export PATH="$VERUS_DIR:$PATH"
    cd packages/verification/nros-verification
    cargo verus verify
    echo "[OK] All Verus proofs verified"

# issue 0650 — clear the check-skip ledger at the head of the lane, so a run in
# which everything ran is not reported against last run's missing tools.
[private]
_check-skip-reset:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/check-skip.sh
    source scripts/build/check-skip.sh
    nros_check_skip_reset

# Remove cargo artifacts a later build superseded, in a workspace build tree.
#
# The identity budget counts `-C metadata` identities, and cargo never collects
# the artifact of an identity it has moved past — so a long-lived incremental
# tree stacks one copy per build era in the same slot and can bust the budget on
# history alone, while the current build sits exactly on it.
#
# Keeping the newest per (dir, crate, ext) is free: that copy is the one cargo
# links and the older ones are unreferenced, so nothing rebuilds. `just prune-artifacts`
# is a DRY RUN; add `apply=1` to delete.
prune-artifacts dir="examples/workspaces/features/build-workspace-fixtures" apply="":
    @python3 scripts/build/prune-superseded-artifacts.py {{dir}} {{ if apply == "" { "" } else { "--apply" } }}

# Verify Phase 118.E size-probe rigorization: cross-mode parity,
# cross-target build under isolated mode, concurrency soak.
[group("debug")]
verify-size-probe:
    bash packages/testing/nros-tests/tests/size_probe_verify.sh

# Run all verification: Kani bounded model checking + Verus deductive verification
[group("verification")]
verify: verify-kani verify-verus

# Run branch coverage on safety-critical crates (requires nightly + cargo-llvm-cov)
# MC/DC is attempted first; falls back to branch-only if unsupported
[group("verification")]
coverage:
    #!/usr/bin/env bash
    set -euo pipefail

    if ! command -v cargo-llvm-cov &>/dev/null; then
        echo "ERROR: cargo-llvm-cov not found. Install with: cargo install cargo-llvm-cov --locked"
        exit 1
    fi

    CRATES=("nros-rmw --features safety-e2e" "nros-serdes" "nros-core")
    OUTPUT_DIR="target/llvm-cov/html"

    echo "=== Branch Coverage (safety-critical crates) ==="
    echo ""

    # Clean once at start so --no-clean preserves each crate's HTML output
    cargo +{{NIGHTLY}} llvm-cov clean --workspace

    for entry in "${CRATES[@]}"; do
        crate=$(echo "$entry" | awk '{print $1}')
        extra_args=$(echo "$entry" | cut -d' ' -sf2-)
        report_dir="$OUTPUT_DIR/$crate"
        mkdir -p "$report_dir"

        echo "--- $crate ---"

        # Try MC/DC first (--mcdc implies branch), fall back to branch-only
        # --no-clean preserves HTML from prior crate runs
        if cargo +{{NIGHTLY}} llvm-cov test --no-clean \
            -p "$crate" $extra_args \
            --mcdc \
            --html --output-dir "$report_dir" 2>/dev/null; then
            echo "  [OK] MC/DC + branch coverage → $report_dir/"
        else
            echo "  [INFO] MC/DC not supported on this toolchain, using branch coverage"
            cargo +{{NIGHTLY}} llvm-cov test --no-clean \
                -p "$crate" $extra_args \
                --branch \
                --html --output-dir "$report_dir"
            echo "  [OK] Branch coverage → $report_dir/"
        fi
        echo ""
    done

    echo "=== Coverage reports: $OUTPUT_DIR/ ==="

# =============================================================================
# Zenoh
# =============================================================================

# Build zenoh transport
[private]
build-zenoh:
    cargo build -p nros-rmw --features std

# Build zenoh-pico C library (standalone, for debugging)
[group("debug")]
build-zenoh-pico:
    @echo "Building zenoh-pico..."
    cd packages/rmw/zenoh/zpico-sys/zenoh-pico && mkdir -p build && cd build && cmake .. -DBUILD_SHARED_LIBS=OFF && make
    @echo "zenoh-pico built at: packages/rmw/zenoh/zpico-sys/zenoh-pico/build"

# =============================================================================
# Benchmarks
# =============================================================================
# Message Bindings
# =============================================================================

# Phase 218 — alias kept for callers still typing the pre-218 name.
# Delegates to `setup-cli` (builds the in-tree `packages/cli/`
# sub-workspace). The historical external-release install path
# (Phase 195.D — NEWSLabNTU/nros-cli Releases) is retired by the
# Phase 218 monorepo merge; phase-288 D1/D2 made source-build the only
# route (`scripts/bootstrap.sh` is the user front door for the same build).
[group("maintenance")]
install-nros-cli: setup-cli
    @echo "nros CLI built in-tree at packages/cli/target/release/nros (Phase 218)."

# Phase 218.D.1 — build the in-tree `nros` CLI sub-workspace into
# `packages/cli/target/release/nros`. Idempotent: a no-op when the binary
# is newer than `packages/cli/Cargo.lock`. Required by every recipe that
# shells out to `nros setup …` / `nros codegen …`; `just setup` runs
# this first so downstream provisioning has the binary on hand.
# Build the in-tree nros CLI (packages/cli/target/release/nros).
[group("setup")]
setup-cli:
    #!/usr/bin/env bash
    set -e
    root="{{justfile_directory()}}"
    bin="$root/packages/cli/target/release/nros"
    lock="$root/packages/cli/Cargo.lock"
    # Phase 220.A.2 — emit a stale-shadow warning whenever we hand the user
    # a freshly built `nros` binary. If `which nros` resolves to a path
    # that ISN'T the one we just built / are about to build, the user is
    # still picking up a pre-218 install (`~/.cargo/bin/nros` from a long-
    # ago `cargo install`, or `~/.nros/bin/nros` from the retired
    # `scripts/install-nros.sh`). Warn now; the next `just doctor` will
    # FAIL hard. We intentionally do NOT exit non-zero — setup-cli's job
    # is to produce the binary, not enforce shell hygiene.
    warn_stale_shadow() {
        if ! command -v nros >/dev/null 2>&1; then
            return
        fi
        local resolved
        resolved="$(command -v nros)"
        local resolved_real
        resolved_real="$(readlink -f "$resolved" 2>/dev/null || echo "$resolved")"
        local bin_real
        bin_real="$(readlink -f "$bin" 2>/dev/null || echo "$bin")"
        if [ "$resolved_real" != "$bin_real" ]; then
            echo "[setup-cli] WARNING: \`which nros\` -> $resolved" >&2
            echo "[setup-cli]   This shadows the in-tree CLI we just built ($bin)," >&2
            echo "[setup-cli]   and recipes resolve PATH first, so THAT binary is the" >&2
            echo "[setup-cli]   one this tree would run." >&2
            echo "[setup-cli]   Put the checkout first:" >&2
            echo "[setup-cli]       source ./activate.sh" >&2
            echo "[setup-cli]   A pre-218 install that still wins is removable:" >&2
            echo "[setup-cli]       rm -f \"\$HOME/.cargo/bin/nros\"" >&2
            echo "[setup-cli]   (\`just doctor\` FAILS until this is resolved — phase-431 W2." >&2
            echo "[setup-cli]    It said so from phase 220 and did not; now it does.)" >&2
        fi
    }
    # Up-to-date iff the binary exists and NO cli SOURCE (Cargo.toml/lock or any
    # `*.rs`) is newer than it. The old `bin -nt lock` guard only checked
    # Cargo.lock, so a SOURCE-only change (e.g. a new subcommand, lock unchanged)
    # was missed — setup-cli skipped the rebuild and handed back a stale binary
    # (phase-265 `nros sync` was "unrecognized" until a manual `cargo build`).
    # `target/`/`generated/` are pruned so the scan is fast; `-quit` stops at the
    # first newer source.
    # `testing_workspaces`/`third-party` pruned too — cli-test fixtures and the
    # vendored submodules are NOT nros build inputs, and a parallel session
    # touching them shouldn't force a rebuild (or trip the cargo.sh #197 guard).
    # `git ls-files` + an mtime walk, NOT `find`. Same reason as everywhere else
    # (see scripts/check-no-tracked-file-find.sh): these are tracked sources, so
    # the index knows them and no filesystem walk is needed. 0.52s -> 0.022s.
    # phase-318 W1: `.jinja` is in the set because askama compiles the templates
    # INTO the binary — a template-only edit changes emitted code while touching
    # no `.rs`, so the old filter handed back a stale `nros` that still emitted
    # the previous bytes. Caught by the W1.d acceptance run, which saw the codegen
    # fingerprint refuse to move after a template edit (a direct `cargo build`
    # moved it). Same shape as issue 0196: a probe watching fewer inputs than the
    # thing it gates.
    # `generated`/`target` need no exclusion here — they are gitignored, so the
    # index never had them. `third-party`/`testing_workspaces` still do: they
    # ARE tracked but are not nros build inputs, and a parallel session touching
    # them must not force a rebuild.
    # Issue 0363 — ASK the binary whether it matches its sources, rather than
    # re-deriving that here. This was the FOURTH copy of the predicate (after
    # cargo.sh, stale_guard.rs and format-cli), and copies drift: this one and
    # the content stamp disagreed in the very run that introduced the stamp —
    # the mtime walk found nothing newer and skipped, handing back a binary the
    # stamp knew was stale.
    #
    # `source-stamp` exits non-zero when stale, when the binary predates the
    # verb, or when it is unrunnable — all of which mean "build it".
    if [ -x "$bin" ] && "$bin" source-stamp >/dev/null 2>&1; then
        # Quiet on no-op — `just setup` invokes us unconditionally.
        warn_stale_shadow
        exit 0
    fi
    echo "[setup-cli] building nros CLI (packages/cli)…"
    # profile-literal-ok: host tool: builds the nros CLI itself
    cargo build --release --manifest-path "$root/packages/cli/Cargo.toml" --bin nros
    # phase-302 W5 added this to stop mtime-based scans flagging the CLI stale
    # FOREVER when cargo skipped a relink. Issue 0363 removed that need — those
    # scans are gone and freshness is a content stamp, which a `touch` cannot
    # affect either way. It survives ONLY for the resolver comparison below,
    # which legitimately asks "which of these two artifacts was built later".
    touch "$bin"
    echo "[setup-cli] built: $bin"
    # Issue 0363 C — the CLI and `nros-launch-resolve` are built by SEPARATE
    # recipes and must agree on an argument list, with nothing gating the pair.
    # A CLI rebuilt past a resolver invocation change leaves the resolver older
    # and the mismatch surfaces deep in a fixture build. Warn (do not fail):
    # setup-cli's job is to produce the binary, and the resolver has its own
    # skip conditions (submodule absent, no CPython), so hard-failing here would
    # block a legitimate CLI-only setup.
    # issue 0596 — SOURCE staleness, not `bin -nt resolver`. Having just built
    # the CLI, that comparison was true forever: `setup-launch-resolve` no-ops
    # when the resolver's sources are unchanged, so it never relinks. Same
    # helper as check::tier-preconditions, one spelling.
    source scripts/build/launch-resolve-stale.sh
    if nros_launch_resolve_stale "$root"; then
        echo "[setup-cli] WARNING: nros-launch-resolve is older than its own SOURCES." >&2
        echo "            It and this CLI must agree on an argument list" >&2
        echo "            (issue 0363 C). Rebuild it:  just setup-launch-resolve" >&2
    fi
    warn_stale_shadow

# After a rebase / pull / stash — restore the derived state, in ORDER.
#
# The three steps below are each documented somewhere and the SEQUENCE is
# documented nowhere, which is the whole cost. A rebase rewrites tracked files,
# so it restales the CLI source stamp, and everything keyed on that stamp goes
# with it. Getting the order wrong is not free: fixtures key on the CLI stamp,
# so rebuilding fixtures BEFORE the CLI re-stales everything you just built.
#
# Paid four times in one session, each time as a different-looking failure:
# three red `check::fast` gates (a stale CLI), a `nros sync` refusing on the 0409
# pin guard (a stale resolver), and `check::issue-index` red (concurrent filings
# on main). None of them says "you just rebased".
#
# What it deliberately does NOT do is touch submodules. AGENTS.md is explicit
# that `git submodule update` is a HUMAN decision, because it discards work in a
# submodule someone is mid-edit on — a worse failure than the one it prevents.
# So divergence is REPORTED with the command, and you decide.
#
# Idempotent and safe to over-run: each step no-ops when its input is unchanged.
[group("main")]
post-rebase:
    #!/usr/bin/env bash
    set -uo pipefail
    echo "post-rebase: restoring derived state (CLI -> resolver -> index)."
    echo ""
    # 1. The CLI first. Fixtures key on its stamp, so anything built before it
    #    would be stale the moment it relinks.
    just setup-cli || exit 1
    # 2. The resolver second. It and the CLI must agree on the play_launch pin
    #    or `nros sync` refuses (issue 0409) — deep in a fixture build, not here.
    just setup-launch-resolve || exit 1
    # 3. The generated issue list. `merge=union` means concurrent filings on
    #    main both land; the generator re-sorts and `check::issue-index` is the
    #    backstop for whatever residue that leaves.
    python3 scripts/gen-issue-index.py || exit 1
    echo ""
    # Submodules: REPORT, never update. See the note above.
    # `+` and `-` are DIFFERENT states and only one of them is about a rebase.
    #
    #   +  the checkout is at a different commit than the pin — what a rebase
    #      that moved a pointer leaves behind, and the actionable case.
    #   -  uninitialised. Usually DELIBERATE here: RFC-0060 keeps play_launch's
    #      layer-3 submodules uninitialised, and the qemu / agent / tracing
    #      trees are provisioned on demand. Reporting those as a problem on
    #      every run is how a message earns its way into being ignored.
    moved="$(git submodule status --cached 2>/dev/null | grep -E '^\+' || true)"
    uninit_n="$(git submodule status --cached 2>/dev/null | grep -cE '^-' || true)"
    if [ -n "$moved" ]; then
        echo "post-rebase: submodule checkout(s) at a different commit than the pin:"
        printf '%s\n' "$moved" | sed 's/^/    /'
        echo ""
        echo "  NOT synced automatically — that would discard work in a submodule"
        echo "  you may be mid-edit on (AGENTS.md). When you are sure the checkout"
        echo "  holds nothing you need:"
        echo "      git submodule update <path>"
    else
        echo "post-rebase: every initialised submodule matches its pin."
    fi
    if [ "${uninit_n:-0}" -gt 0 ]; then
        echo "             ($uninit_n uninitialised — normal; init only what you build.)"
    fi
    echo ""
    echo "post-rebase: done. Fixtures are NOT rebuilt — \`just check tier-preconditions\`"
    echo "             reports what the tier you are about to run still needs."


# Build the launch-resolution helper (issue 0285).
#
# `nros sync` needs a resolver that can execute Python for `.launch.py`
# trees, which cannot be linked into the portable `nros` binary. It used to
# shell out to `play_launch` BY NAME through PATH — where an unrelated ROS 2
# record/replay tool of the same name shadowed it and every platform's
# fixture build died inside a cmake configure.
#
# This builds our own distinctly-named binary from the pinned
# `ros-launch-resolve` submodule (RFC-0060 layer 2), versioned with the CLI,
# and `nros` invokes it by ABSOLUTE PATH.
# Neither tool can shadow the other.
#
# Its own cargo workspace, so its dependency graph stays separate from the
# main CLI's. Needs CPython (pyo3) but NOT ROS/ament/colcon — that is now a
# property of the layer-2 package graph, not of a feature flag.
[group("setup")]
setup-launch-resolve:
    #!/usr/bin/env bash
    set -e
    root="{{justfile_directory()}}"
    crate="$root/packages/cli/nros-launch-resolve"
    # Honour CARGO_TARGET_DIR: cargo writes there, so the staleness check and
    # the reported path must look there too. Hardcoding `$crate/target` made a
    # box build land in the redirected dir while this recipe declared success
    # about a HOST binary sitting at the old path — and since that binary links
    # the host's libpython, `nros sync` inside the box then died with
    # `libpython3.14.so.1.0: cannot open shared object file`. Issue 0400.
    # profile-literal-ok: host tool: the launch resolver's own binary
    bin="${CARGO_TARGET_DIR:-$crate/target}/release/nros-launch-resolve"
    if [ ! -f "$crate/../third-party/play_launch/src/ros-launch-resolve/resolve/Cargo.toml" ]; then
        # issue 0409 — FAIL, do not skip. This recipe's job is to produce the
        # resolver binary; exiting 0 without producing one let `nros sync` run
        # with whatever stale binary was left on disk, and a resolver predating
        # rlm v0.1.1 silently drops every `[[component]].params` /
        # `params_files` projection. No error, no warning, exit 0 — 22 models in
        # `features/` alone lost their params that way, and the reds surfaced far
        # from the cause (`model params: {}` in a QoS-override test).
        #
        # It was worse than a plain skip: `setup-cli` WARNS when the resolver is
        # older than the CLI and tells you to run this recipe, so running it and
        # getting exit 0 made the warning look addressed while nothing was built.
        echo "[setup-launch-resolve] FAILED: play_launch submodule not initialised" >&2
        # NON-recursive on purpose (RFC-0060): layer 2 (resolve + parser) is
        # regular files inside play_launch; its layer-3 submodules (vendor/*,
        # container, msgs) are never built by nano-ros and must stay uninitialised.
        echo "  git submodule update --init packages/cli/third-party/play_launch" >&2
        echo "" >&2
        echo "  A resolver that cannot be built must not be silently replaced by an" >&2
        echo "  older one: the failure mode is missing DATA in generated models, not" >&2
        echo "  a build error (issue 0409)." >&2
        echo "  For a deliberate CLI-only setup with no resolver:" >&2
        echo "      NROS_ALLOW_NO_LAUNCH_RESOLVE=1 just setup-launch-resolve" >&2
        if [ "${NROS_ALLOW_NO_LAUNCH_RESOLVE:-0}" = "1" ]; then
            echo "[setup-launch-resolve] skipping anyway (NROS_ALLOW_NO_LAUNCH_RESOLVE=1)" >&2
            # Remove any binary left behind, so a later `nros sync` fails LOUD on a
            # missing resolver instead of quietly resolving with a stale one.
            rm -f "$bin"
            exit 0
        fi
        exit 1
    fi
    # `find -newer` errors when the reference file is absent, and `set -e`
    # would abort the very first build — check existence before comparing.
    #
    # The probe MUST watch the vendored resolver tree too, not just this crate:
    # the binary compiles `play_launch/src/ros-launch-resolve` in, and that
    # advances by the play_launch SUBMODULE PIN. Watching only `$crate` meant a
    # pin bump left the old binary in place — a fix that had landed upstream,
    # with a regression test for it, kept failing here, and the symptom
    # (`node '/listener' is not placed`) looked like a code regression on main
    # rather than a museum binary. Same class as issue 0196: a build-side probe
    # that misses an input the build consumes.
    # The Python half is HALF THE TOOL, so "already built" has to mean both
    # artifacts. Keyed on the binary alone, this recipe short-circuits on a tree
    # that has the resolver and no `libplay_launch_parser_pyexec.so` beside it —
    # which is precisely the state that left `host-tests` red: the recipe
    # reported success, and every `$(eval …)` launch file failed at parse time
    # with "this build has no Python backend" on hosts that have Python.
    if [ -x "$bin" ] && [ ! -f "$(dirname "$bin")/libplay_launch_parser_pyexec.so" ]; then
        echo "[setup-launch-resolve] the Python half is missing beside the binary; rebuilding."
    elif [ -x "$bin" ]; then
        # `git ls-files` + mtime walk, not `find`. The resolver tree lives inside
        # the play_launch SUBMODULE, so `git ls-files` is run inside it (`-C`) —
        # from the superproject the index holds only the gitlink, which would
        # silently match nothing and make every pin bump look current, the exact
        # museum-binary failure this probe exists to catch. Scoped to the layer-2
        # subdir (`src/ros-launch-resolve`), which is regular files — no
        # `--recurse-submodules`: ros-launch-manifest is no longer nested (it is a
        # tag-pinned cargo git dep since phase-332 W2), and play_launch's layer-3
        # submodules are deliberately uninitialised.
        #
        # phase-318 W1.b builds `scripts/build/resolve-fingerprint.sh` on top of
        # this probe: it hashes the MODELS this binary emits, cached by the
        # binary's own sha256, to decide fixture freshness. That cache key makes
        # a stale binary worse than unnoticed — it is LAUNDERED: stable hash,
        # stable fingerprint, every fixture reported fresh indefinitely. This
        # probe is the only thing standing between the two, so its blind spots
        # become that mechanism's blind spots.
        # Layer 2 (resolve + parser) is REGULAR FILES under
        # `play_launch/src/ros-launch-resolve` (phase-332: folded into the
        # play_launch repo). No `--recurse-submodules` — ros-launch-manifest is
        # no longer nested here; it is a tag-pinned cargo git dep (RFC-0060
        # amendment / phase-332 W2), so its staleness is gated by the tag + the
        # lock, not by a source-tree walk. Scope the ls-files to the layer-2
        # subdir so play_launch's UNINITIALISED layer-3 submodules (vendor/*,
        # container, msgs) are neither walked nor required.
        # phase-363 / issue 0596 — the THIRD copy of this walk, and the one that
        # decides whether to rebuild. Now the same helper the two warning sites
        # use, asking about CONTENT: the mtime form was falsified by any rebase
        # or stash, which rewrites tracked files with identical bytes.
        # shellcheck source=scripts/build/launch-resolve-stale.sh
        source "$root/scripts/build/launch-resolve-stale.sh"
        if ! nros_launch_resolve_stale "$root"; then
            # SAY SO. A bare `exit 0` made this recipe print nothing on the
            # skip path and `built:` on the build path, so "I rebuilt it" and
            # "I decided not to" were indistinguishable from the outside —
            # which is how issue 0921 stayed hidden: the recipe reported
            # success, the binary kept a stale pin, and the disagreement
            # surfaced two steps later in `nros sync` naming neither.
            #
            # The PIN is the informative part, because it is the value the 0409
            # guard compares against the CLI's. Printing it turns "why does sync
            # think these disagree?" into one line of output.
            _pin="$("$bin" --version 2>/dev/null \
                | sed -n 's/.*play_launch \([0-9a-f]*\).*/\1/p' | head -1)"
            echo "[setup-launch-resolve] already current: $bin"
            echo "                       play_launch ${_pin:-unknown} — sources unchanged, not rebuilt"
            exit 0
        fi
    fi
    echo "[setup-launch-resolve] building nros-launch-resolve…"
    # The vendored resolver embeds pyo3 (RFC-0060 layer 2 needs CPython), pinned
    # at 0.24, whose maximum supported interpreter is 3.13. A rolling distro
    # outruns that — Arch ships Python 3.14 and nothing older, so the build dies:
    #
    #   error: the configured Python interpreter version (3.14) is newer than
    #          PyO3's maximum supported version (3.13)
    #
    # This is pyo3's own documented remedy for exactly that case: build against
    # the stable ABI instead. The variable ONLY suppresses the too-new check, so
    # it is inert on a host whose interpreter pyo3 already supports (Ubuntu
    # 22.04's 3.10, 24.04's 3.12) — no need to detect the version and no second
    # code path to keep in step. Revisit when the vendored pin moves past 0.24.
    export PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1
    # profile-literal-ok: host tool: builds nros-launch-resolve
    cargo build --release --manifest-path "$crate/Cargo.toml"
    # The Python HALF, and it is not optional packaging — it is half the tool.
    #
    # 0897 W3 made the resolver load an interpreter at RUNTIME instead of linking
    # one, which is what lets ONE binary serve any CPython and lets a host with
    # none still resolve XML/YAML. The loader finds the Python half by looking
    # beside the executable (`pyload::pyexec_beside_exe`), so a build that emits
    # only the binary produces a resolver that is *correctly* version-agnostic
    # and *cannot evaluate anything*: `$(eval …)` and `.launch.py` fail with "this
    # build has no Python backend" on a host that has Python installed.
    #
    # That is exactly what happened. `host-tests` went red on main the moment the
    # W2b pin removed the link, on
    # `demo_bringup/launch/multihost.launch.xml`'s `$(eval "robot1" in (...))`,
    # and stayed red — the artifact SHAPE was verified (`readelf -d` shows no
    # `libpython`) while the capability was not.
    #
    # `--features extension-module,abi3` — two flags doing two jobs (issue 0915).
    # `extension-module` is the half that must NOT link libpython:
    # its `Py_*` symbols stay undefined so they bind to whichever interpreter the
    # loader `dlopen`s with RTLD_GLOBAL. `abi3` is what makes those undefined
    # symbols ones every CPython >= 3.10 exports UNCHANGED — without it the
    # half referenced six CPython internals and was pinned to whatever version
    # pyo3 found at build time, which is the pin this whole design removes.
    # Built from the parser's own workspace,
    # which is where the crate lives and is deliberately NOT in the resolver's
    # dependency graph — a normal dep would put the link back.
    _pyexec_ws="$crate/../third-party/play_launch/src/ros-launch-resolve/parser"
    echo "[setup-launch-resolve] building the Python half (dlopen'ed at runtime)…"
    # profile-literal-ok: host tool: the resolver's own Python half
    cargo build --release --manifest-path "$_pyexec_ws/Cargo.toml" \
        -p play_launch_parser_pyexec --lib --features extension-module,abi3
    _pyexec_so="$(dirname "$bin")/libplay_launch_parser_pyexec.so"
    # profile-literal-ok: host tool: reads back where that build put it
    _pyexec_built="${CARGO_TARGET_DIR:-$_pyexec_ws/target}/release/libplay_launch_parser_pyexec.so"
    if [ ! -f "$_pyexec_built" ]; then
        echo "[setup-launch-resolve] FAILED: the Python half was not produced at" >&2
        echo "  $_pyexec_built" >&2
        echo "  Without it the resolver cannot evaluate \`\$(eval …)\` or a" >&2
        echo "  .launch.py, and says so at parse time rather than here." >&2
        exit 1
    fi
    cp "$_pyexec_built" "$_pyexec_so"
    # Record WHAT it was built from. Without this the content check has nothing
    # to compare against and reports stale forever — issue 0596's shape, one
    # mechanism over.
    #
    # This replaces a `touch "$bin"`, which existed only to make the old
    # `source -nt binary` comparison come out right after cargo declined to
    # relink. A stamp answers the question directly, so nothing needs its mtime
    # nudged.
    # shellcheck source=scripts/build/launch-resolve-stale.sh
    source "$root/scripts/build/launch-resolve-stale.sh"
    nros_launch_resolve_stamp "$root" > "$bin.nros-source-stamp"
    # Same reasoning as the skip path above: name the pin, since that is what
    # the 0409 guard compares and what a mismatch will be reported against.
    _pin="$("$bin" --version 2>/dev/null \
        | sed -n 's/.*play_launch \([0-9a-f]*\).*/\1/p' | head -1)"
    echo "[setup-launch-resolve] built: $bin"
    echo "                       play_launch ${_pin:-unknown}"

# Regenerate Rust bindings in all examples and rcl-interfaces
# Uses bundled interfaces (std_msgs, builtin_interfaces) — no ROS 2 environment required
[group("maintenance")]
generate-bindings:
    ./scripts/regenerate-bindings.sh

# Remove generated/ directories in examples (not rcl-interfaces — it's a workspace member)
[group("maintenance")]
clean-bindings:
    #!/usr/bin/env bash
    set -e
    echo "Removing generated bindings..."
    # Auto-discover all generated/ directories under examples/ — walk is
    # legitimate here (generated/ is an untracked product), but the prune
    # list comes from the phase-300 shared SSoT.
    source scripts/build/prune-dirs.sh
    # generated/ is itself IN the prune list (we're looking for it) — build
    # a prune group without it.
    _prune=('(') ; for _d in "${NROS_PRUNE_DIRS[@]}"; do [ "$_d" = generated ] && continue; _prune+=(-name "$_d" -o); done
    unset '_prune[-1]'; _prune+=(')' -prune)
    for d in $(find examples "${_prune[@]}" -o -name generated -type d -print | sort); do
        rm -rf "$d"
        echo "  removed $d"
    done
    # Phase 131.B — relocated bench/test-fixture crates under packages/testing/
    for d in $(find packages/testing/nros-bench packages/testing/nros-tests/bins packages/testing/nros-smoke \
                    "${_prune[@]}" -o -name generated -type d -print 2>/dev/null | sort); do
        rm -rf "$d"
        echo "  removed $d"
    done
    echo "All generated bindings removed."

# Regenerate rcl-interfaces bindings (workspace member with nros- prefix)
[private]
generate-rcl-interfaces:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)"
    echo "Regenerating rcl-interfaces bindings..."
    cd packages/interfaces/rcl-interfaces
    rm -rf generated/humble/nros-builtin-interfaces generated/humble/nros-rcl-interfaces
    $NROS generate-rust --force -o generated/humble \
        --rename builtin_interfaces=nros-builtin-interfaces \
        --rename rcl_interfaces=nros-rcl-interfaces
    echo "✓ rcl-interfaces regenerated"

# Regenerate diagnostic-msgs bindings (RFC-0052 W3b.1; capacities from its
# nros-codegen.toml — keep /diagnostics entries small and embeddable)
[private]
generate-diagnostic-msgs:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)"
    echo "Regenerating diagnostic-msgs bindings..."
    cd packages/interfaces/diagnostic-msgs
    rm -rf generated/humble
    $NROS generate-rust --force -o generated/humble --codegen-config nros-codegen.toml \
        --rename diagnostic_msgs=nros-diagnostic-msgs \
        --rename std_msgs=nros-std-msgs-diag \
        --rename builtin_interfaces=nros-builtin-interfaces-diag
    rm -rf generated/humble/geometry_msgs
    echo "✓ diagnostic-msgs regenerated"

# Regenerate rosgraph-msgs bindings (workspace member with nros- prefix).
# phase-425 W2 — `rosgraph_msgs/msg/Clock` is the wire form of ROS time; the
# `-clock` rename on builtin_interfaces is not cosmetic: a generated crate
# names its deps by CRATE name, so two trees generating `builtin_interfaces`
# collide in one workspace (same reason diagnostic-msgs carries `-diag`).
[private]
generate-rosgraph-msgs:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)"
    echo "Regenerating rosgraph-msgs bindings..."
    cd packages/interfaces/rosgraph-msgs
    rm -rf generated/humble
    $NROS generate-rust --force -o generated/humble \
        --rename rosgraph_msgs=nros-rosgraph-msgs \
        --rename builtin_interfaces=nros-builtin-interfaces-clock
    echo "✓ rosgraph-msgs regenerated"

# Regenerate lifecycle-msgs bindings (workspace member with nros- prefix)
[private]
generate-lifecycle-msgs:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)"
    echo "Regenerating lifecycle-msgs bindings..."
    cd packages/interfaces/lifecycle-msgs
    rm -rf generated/humble/nros-lifecycle-msgs
    $NROS generate-rust --force -o generated/humble \
        --rename lifecycle_msgs=nros-lifecycle-msgs
    echo "✓ lifecycle-msgs regenerated"
    echo "NOTE: re-apply workspace inheritance to the generated Cargo.toml"
    echo "      (version.workspace, edition.workspace, etc.) — see rcl-interfaces."

# Clean and regenerate all bindings from scratch
[group("maintenance")]
regenerate-bindings: clean-bindings generate-bindings

# =============================================================================
# Setup & Doctor orchestrators
#
# `just setup`     — print setup choices; does not fetch/install.
# `just setup base` — safe quick-start setup (workspace).
# `just setup all` — full contributor setup (all platforms + services).
# `just doctor`    — read-only diagnosis of install status.
#
# Each module has its own `setup`/`doctor` recipes. The orchestrator walks
# them all, treats individual failures as non-fatal, and prints a summary.
# Run any module independently: e.g. `just nuttx setup`, `just zephyr doctor`.
# =============================================================================

# Install SDK/tooling dependencies.
#
# Common flows:
#   just setup              # print choices
#   just setup base         # base quick-start tier
#   just setup all          # full contributor / test-all tier
#   just setup tier=all     # explicit tier form
#   just setup zephyr       # shorthand for: just zephyr setup
#   just zephyr setup       # focused platform setup
#
# Print setup choices with no args; otherwise run a tier or focused setup.
[group("setup")]
setup target="" tier="" *extra:
    #!/usr/bin/env bash
    set -e
    chosen_tier="{{tier}}"
    target="{{target}}"
    extra_args="{{ extra }}"
    # phase-422 W3 — `just` fills positional parameters in order, so
    # `just setup zephyr --skip-sdk` binds the flag to `tier`, not to the
    # variadic tail. A tier is never a flag, so re-home it rather than making
    # the user write `just setup zephyr "" --skip-sdk`.
    case "$chosen_tier" in
        -*)
            extra_args="$chosen_tier $extra_args"
            chosen_tier=""
            ;;
    esac
    if [[ -z "$target" && -z "$chosen_tier" ]]; then
        printf '%s\n' \
          "nano-ros setup choices:" \
          "" \
          "  just setup base              # first-time native/ROS/zenoh quick start" \
          "  just setup <scope>           # focused setup, e.g. zephyr, freertos, nuttx" \
          "  just setup all               # full contributor/test-all setup; fetches all SDKs" \
          "" \
          "SCOPE is one word in one position, for every verb (phase-411):" \
          "" \
          "  just setup zephyr            # provision it" \
          "  just doctor zephyr           # is it ready?" \
          "  just build zephyr            # build its test fixtures" \
          "  just test zephyr             # run its tests" \
          "" \
          "Platform scopes:" \
          "" \
          "  native zephyr freertos nuttx threadx_linux threadx_riscv64" \
          "  esp32 esp_idf qemu px4 xrce cyclonedds" \
          "" \
          "Preset scopes (a named set of platforms — the fixture lanes):" \
          "" \
          "  all native tier1 tier2 tier2-nightly" \
          "" \
          "Readiness checks:" \
          "" \
          "  just doctor                  # host tooling + a PROBE of every platform" \
          "  just doctor <scope>          # one platform, or a preset's set" \
          "  just doctor tier=all         # the pre-407 tier walk (adds workspace/verification)" \
          "" \
          "Fresh checkout without just:" \
          "" \
          "  scripts/bootstrap.sh         # installs/checks just, then shows this menu" \
          "  scripts/bootstrap.sh base" \
          "  scripts/bootstrap.sh platform zephyr" \
          "  scripts/bootstrap.sh all" \
          "" \
          "After setup:" \
          "" \
          "  source ./activate.sh         # get nano-ros binaries on PATH"
        exit 0
    fi
    if [[ -n "$target" ]]; then
        case "$target" in
            tier=*)
                chosen_tier="${target#tier=}"
                ;;
            base|quickstart|minimal|default|all|everything|contributor|extended)
                chosen_tier="$target"
                ;;
            # phase-411 W3 — the platform arm is now the SCOPE predicate, not a
            # second hand-written list of platform names. The list had already
            # drifted (`native` was missing, `rmw_zenoh` names no module), and a
            # scope that resolves for `doctor`/`build`/`test` and silently falls
            # through here is exactly the two-vocabularies defect this phase
            # exists to remove.
            #
            # `workspace` and `verification` stay spelled out: they are
            # deliberately NOT scope tokens (see scripts/build/scope.sh) but
            # they do have `setup`, and `just setup workspace` predates 407.
            workspace | verification | rmw_zenoh)
                just _setup-common
                exec just "$target" setup $extra_args
                ;;
            *)
                # shellcheck source=scripts/build/scope.sh
                source scripts/build/scope.sh
                if nros_scope_is_platform "$target" \
                   && nros_scope_module_has_verb "$target" setup; then
                    # Focused platform setup may still shell `nros setup …`;
                    # provision the CLI + resolver first so the binaries exist.
                    just _setup-common
                    exec just "$target" setup $extra_args
                fi
                # phase-411 W4 — a PRESET provisions the set it names.
                #
                # The menu above has advertised "Preset scopes … all native
                # tier1 tier2 tier2-nightly" since W3, while the dispatch fell
                # through to the legacy `--target=<platform>-<rmw>` arm and
                # died with "is not a <platform>-<rmw> tuple". A documented
                # scope that does not resolve is the same two-vocabularies
                # defect one verb over, and it is what stopped a CI job from
                # being `just setup <scope>` + `just test <scope>`.
                #
                # Expansion comes from `nros_lane_modules`, the same function
                # `build` uses, so setup and build cannot disagree about what a
                # preset contains.
                if nros_scope_is_preset "$target"; then
                    source scripts/build/fixture-lane.sh
                    mods="$(nros_lane_modules "$target")" || exit 1
                    if [ -z "$(echo $mods)" ]; then
                        # `all` means "no narrowing", so its module list is
                        # EMPTY — and an empty loop below would provision
                        # nothing and exit 0, which is the silent no-op this
                        # whole phase exists to stop. `all` is caught by the
                        # tier arm above and cannot reach here; any other
                        # preset that expands to nothing is a bug in the lane
                        # tables, not a reason to succeed.
                        printf 'setup: preset %s expands to NO modules — refusing to\n' "$target" >&2
                        printf '       report success for provisioning nothing. Use `just setup all`\n' >&2
                        printf '       for the full contributor setup, or name platforms.\n' >&2
                        exit 1
                    fi
                    printf 'setup: preset %s -> %s\n' "$target" "$(echo $mods)"
                    just _setup-common
                    for m in $mods; do
                        if nros_scope_module_has_verb "$m" setup; then
                            printf '\n=== just setup %s ===\n' "$m"
                            just "$m" setup
                        else
                            printf 'setup: %s has no `setup` recipe — nothing to provision\n' "$m"
                        fi
                    done
                    exit 0
                fi
                exec "$(pwd)/tools/setup.sh" --target="$target"
                ;;
        esac
    fi
    # Phase 218.D.2 — Tier 0: build the in-tree nros CLI (and the launch
    # resolver) before any provisioning step. Downstream module recipes shell
    # `nros setup --source …`; that command requires the binary to exist.
    just _setup-common
    # phase-263 — pin clang-format (every tier): `just format` / `just ci`'s
    # check-{c,cpp}-fmt drift across clang-format major versions, so a consistent
    # pinned binary (`.clang-format-version`) is part of base dev setup. Idempotent.
    just setup-clang-format || echo "  (clang-format provisioning skipped — python3 venv unavailable)"
    just _orchestrate setup "$chosen_tier"
    echo ""
    echo "✅ nano-ros setup complete."
    echo "   Activate this shell with the shipped binaries on PATH:"
    echo ""
    echo "     source ./activate.sh     # bash / zsh"
    echo "     source ./activate.fish   # fish"
    echo ""

# Provisioning every `just setup <arg>` needs, whichever arm it takes (issue 0992).
#
# These two ran only on the TIER path. A PRESET (`just setup tier2`) and a
# PLATFORM (`just setup zephyr`) each `exit 0`/`exec` before reaching them, so
# the spelling phase-411 W4 tells every CI job to use provisioned the CLI and
# nothing else. `build-wide` runs exactly `just setup tier2`, and its build
# step needs the launch resolver — `nros sync` refuses to resolve a SystemModel
# without it rather than reusing a museum model (issue 0409). The gap was
# invisible on a developer machine, which had built the resolver months ago.
#
# The submodule init is the same guarded, NON-recursive step `bootstrap.sh`'s
# `ensure_cli_submodules` performs for `base` (RFC-0060: play_launch's layer-3
# runtime submodules are never built here). `setup-launch-resolve` fails loud
# without it and prints this exact command; a provisioning verb should run it
# rather than print it. It is a no-op once the tree is populated, so a
# developer never sees it.
[private]
_setup-common:
    #!/usr/bin/env bash
    set -e
    sub="packages/cli/third-party/play_launch"
    # Issue 1081 — the guard tests the PIN, not merely presence. Testing for a
    # file answers "is it initialised", and a checkout can be initialised and at
    # the WRONG COMMIT: a persistent CI workspace keeps whatever the last run
    # left, and `actions/checkout` runs with `submodules: false`, so nothing
    # moves it back. That is not hypothetical — the merge queue was frozen for
    # SEVEN pull requests with an identical `cannot update the lock file …
    # because --locked was passed`, because the runner sat on `8fda8d89`
    # (heads/main) while the pin was `4c214a63`. `nros-launch-resolve`'s lock
    # matches the PIN, so cargo wanted to rewrite it and `--locked` refused. The
    # same red on unrelated PRs is never a property of any of them.
    #
    # `git submodule status --cached` prefixes `+` for "checkout differs from
    # the recorded pointer" and `-` for "not initialised" — different states,
    # and only one of them is safe to fix by force.
    _sub_state="$(git submodule status --cached "$sub" 2>/dev/null | cut -c1)"
    if [ "$_sub_state" = "-" ]; then
        echo "setup: initialising in-tree CLI submodule ($sub)"
        # NON-recursive on purpose (RFC-0060): layer 2 is regular files inside
        # play_launch; its layer-3 submodules are never built by nano-ros.
        git submodule update --init "$sub"
    elif [ "$_sub_state" = "+" ]; then
        # Present but at another commit. Updating DISCARDS work in the
        # submodule, which AGENTS.md makes a human decision — so this only
        # forces it when there is provably nothing to lose, and otherwise stops
        # rather than resolving against a tree the lockfiles do not describe.
        if [ -z "$(git -C "$sub" status --porcelain 2>/dev/null)" ]; then
            echo "setup: $sub is at $(git -C "$sub" rev-parse --short HEAD), not the recorded pin — updating (worktree clean)"
            git submodule update "$sub"
        else
            echo "setup: FAILED — $sub is at $(git -C "$sub" rev-parse --short HEAD), not the pin this tree records," >&2
            echo "  and it has uncommitted changes, so this cannot move it for you (issue 1081)." >&2
            echo "  Building against it resolves a graph the lockfiles do not describe, which" >&2
            echo "  surfaces as \`cannot update the lock file … --locked\` several steps later." >&2
            echo "" >&2
            echo "  Commit or stash the submodule work, then:  git submodule update $sub" >&2
            exit 1
        fi
    fi
    just setup-cli
    just setup-launch-resolve
    # Host-provisioning facts that `check-tier-preconditions` asserts for EVERY
    # tier, so every `just setup <scope>` must satisfy them — otherwise the
    # documented `setup -> build -> test` chain does not clear the checks its
    # own test step runs, which is how host-tests sat red on three unmet
    # preconditions it had no step to provision.
    #
    # AFTER `setup-cli`: `install-corrosion` resolves its prefix with
    # `nros sdk-path corrosion`, so it needs the binary to exist.
    # Both are idempotent -- corrosion short-circuits on its version stamp,
    # `rustup target add` is a no-op when the target is present.
    just workspace rust-targets
    just workspace install-corrosion
    # `check fast` runs `c-fmt`/`cpp-fmt`, and EVERY tier runs `check fast`, so
    # clang-format is a host fact every tier asserts — the same argument as the
    # two above. Without it host-tests' `just ci tier1` reached the gate and
    # died on "clang-format not found" after ~40 minutes of fixture builds.
    # Idempotent: the recipe version-checks `build/clang-format/bin/clang-format`
    # and exits early. Project-local, no install.
    just setup-clang-format
    # phase-422 W6 — TELL the user about the system closure. Measured: 24 of the
    # 46 `[prereq.*]` keys had no consumer at all, nothing in the repo installs
    # via `--system`, and `just setup <scope>` never even printed it — so a
    # missing `cmake`/`ninja`/`make` surfaced as a build failure much later.
    #
    # REPORT, never install. Composing the command is this tool's job and
    # running it is the user's (RFC-0062); a provisioning verb that sudo-installs
    # behind someone's back is worse than the gap it closes.
    #
    # Narrowed to the HOST roles: `infra` keys (emulators, cross toolchains) come
    # from the scope's own module setup, and listing them here would bury the two
    # lines that matter under a dozen that do not.
    #
    # Non-fatal on purpose: `--check` exits 1 when anything is missing, and setup
    # must not die for a package it is not allowed to install.
    #
    # `--check` is a doctor, so it exits 1 and eyre prints `Error:` with a
    # source Location. Correct for a doctor, wrong here: an "Error:" plus a
    # stack location in a path that deliberately continues is how people learn
    # to scroll past errors. Capture and re-print without the backtrace.
    if ! _sysout="$(nros setup --system --check --role package --role workspace 2>&1)"; then
        printf '%s\n' "$_sysout" | grep -vE '^(Error:|Location:|\s+nros-cli-core/src|\s*$)' || true
        printf '\nsetup: system package(s) above are MISSING; nothing here installs them.\n'
        printf 'setup: NOT fatal — provisioning continues. Run the command shown, or\n'
        printf '       `nros setup --system --sudo` to execute it.\n\n'
    fi

# Focused platform setup. Equivalent to `just <platform> setup`.
[group("setup")]
setup-platform platform:
    @just "{{platform}}" setup

# Diagnose install status (read-only) — `just doctor [<scope>…]`.
#
# phase-411 W3. The scope is the SPECIFICATION, in the same argument position
# every other verb takes it:
#
#   just doctor              this host: the shared tooling, then a PROBE of
#                            every platform, so the answer to "what am I
#                            covered for?" is derived rather than remembered
#   just doctor zephyr       one platform's readiness, with its remedies
#   just doctor tier2        every platform that lane covers
#   just doctor tier=all     the pre-407 tier spelling, still honoured
#
# The tier form stays because it is in the book, in `scripts/bootstrap.sh` and
# in muscle memory, and because the tier fan-out reaches modules that are not
# scopes (`workspace`, `verification`) — see `scripts/build/scope.sh` for why
# those are deliberately outside the scope namespace.
#
# Readiness for a scope; with no scope, the host plus a probe of every platform.
[group("setup")]
doctor *scope:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/scope.sh
    source scripts/build/scope.sh
    chosen_tier=""
    scoped=()
    for tok in {{scope}}; do
        case "$tok" in
            # The TIER vocabulary (`_orchestrate`), which is not the scope
            # vocabulary: it walks tooling modules too. Kept verbatim.
            tier=*) chosen_tier="${tok#tier=}" ;;
            # `all` takes the TIER reading here, so `just setup all` and
            # `just doctor all` mirror each other and `scripts/bootstrap.sh
            # doctor all` keeps working. The tier is a strict superset of the
            # `all` PRESET (it adds `workspace` and `verification`), so nothing
            # a scope reading would have reported is lost.
            all|base|quickstart|minimal|default|everything|contributor|extended)
                chosen_tier="$tok" ;;
            *) scoped+=("$tok") ;;
        esac
    done
    if [ "${#scoped[@]}" -gt 0 ]; then
        if [ -n "$chosen_tier" ]; then
            echo "doctor: 'tier=$chosen_tier' and a scope name together — pick one." >&2
            echo "        The scope namespace replaces the tier one:  just doctor ${scoped[*]}" >&2
            exit 2
        fi
        nros_scope_validate_all "${scoped[@]}" || exit 2
        nros_scope_report doctor "${scoped[@]}"
        rc=0
        for tok in "${scoped[@]}"; do
            just _doctor-scope "$tok" || rc=1
        done
        exit "$rc"
    fi
    if [[ -z "$chosen_tier" ]]; then
        chosen_tier="${NROS_SETUP_TIER:-base}"
    fi
    # `set -e` would stop here on a host FAIL, and doctor's job is the whole
    # picture: a shadowing `nros` must not hide an absent Zephyr SDK.
    host_rc=0
    just _doctor-host || host_rc=1
    just _orchestrate doctor "$chosen_tier" || host_rc=1
    # The DERIVED default scope — every platform probed, nothing recorded.
    # This is the line that makes an absent platform visible at the moment a
    # person is asking about readiness, instead of as a skip inside a green run.
    echo ""
    nros_scope_report doctor
    exit "$host_rc"

# One scope token's readiness. `native` is the host, so its readiness IS the
# shared-tooling block — there is no `native` module `doctor` to call and there
# should not be one, since a second host probe is a second answer.
[private]
_doctor-scope tok:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/scope.sh
    source scripts/build/scope.sh
    tok="$(nros_scope_normalize "{{tok}}")"
    if nros_scope_is_platform "$tok"; then
        echo ""
        echo "=== $tok ==="
        if [ "$tok" = "native" ]; then
            # Propagate: `just doctor native` and the default scope reach ONE
            # block, so they must reach one verdict too (phase-431 W2).
            rc=0
            nros_scope_exec just _doctor-host || rc=$?
            exit "$rc"
        fi
        nros_scope_require_module_verb "$tok" doctor
        nros_scope_exec just "$tok" doctor
        exit 0
    fi
    if nros_scope_is_preset "$tok"; then
        rc=0
        for p in $(nros_scope_preset_expand "$tok"); do
            just _doctor-scope "$p" || rc=1
        done
        exit "$rc"
    fi
    nros_scope_reject "$tok"

# The host's shared tooling — the CLI, python, clang-format, sccache, the ROS
# router. Extracted from `doctor` by phase-411 W3 so `just doctor native` and
# the default scope reach the SAME block rather than a second copy of it.
[private]
_doctor-host:
    #!/usr/bin/env bash
    set -e
    # Phase 218.D.4 — CLI binary + version on a single line. Read-only;
    # uses the same resolver as every recipe that shells `nros …`, so a
    # skew between resolver and what doctor reports is impossible.
    # shellcheck disable=SC1091
    host_rc=0
    if . "{{justfile_directory()}}/scripts/build/cargo.sh" 2>/dev/null && \
       cli_bin="$(nros_cli_bin 2>/dev/null)"; then
        cli_ver="$("$cli_bin" --version 2>/dev/null | head -1)"
        echo "  [OK] nros CLI: ${cli_ver:-unknown} ($cli_bin)"
    else
        echo "  [MISSING] nros CLI — run: just setup-cli"
    fi
    # phase-431 W2 — A SHADOWING `nros` IS A FAILURE, not a warning.
    #
    # `setup-cli` has warned about this since phase 220 and its own text said
    # "`just doctor` will FAIL until this is resolved". It did not; the promise
    # was the whole enforcement. That was survivable while no release existed —
    # a non-checkout `nros` was then somebody's old `cargo install` — and stops
    # being survivable the day phase-431 ships one, because the shadow becomes
    # the NORMAL state of a user's machine and a contributor's PATH inherits it.
    #
    # What it costs is not hypothetical: `nros_cli_bin` resolves PATH BEFORE the
    # per-checkout binary (cargo.sh, phase 218.D.3), so every recipe that shells
    # `nros …` runs the shadow, and it emits ITS OWN codegen against this tree.
    # RFC-0090's version does not catch that — a release at the same version is
    # different, not incompatible. `refuse_if_foreign_to_workspace` (W1) refuses
    # it at `nros build`; this is the same answer at the moment someone is
    # asking whether their machine is ready, which is cheaper than at build time.
    if command -v nros >/dev/null 2>&1; then
        in_tree="{{justfile_directory()}}/packages/cli/target/release/nros"
        shadow="$(command -v nros)"
        shadow_real="$(readlink -f "$shadow" 2>/dev/null || echo "$shadow")"
        in_tree_real="$(readlink -f "$in_tree" 2>/dev/null || echo "$in_tree")"
        if [ "$shadow_real" != "$in_tree_real" ]; then
            echo "  [FAIL] \`nros\` on PATH is not this checkout's binary."
            echo "         on PATH:  $shadow_real"
            echo "         expected: $in_tree_real"
            echo "         Recipes resolve PATH first, so every \`nros …\` this tree"
            echo "         runs would come from somewhere else and emit its own"
            echo "         codegen (RFC-0090; \`nros build\` refuses it outright)."
            echo "         Fix:  just setup-cli && source ./activate.sh"
            echo "         A pre-218 install lingers at ~/.cargo/bin/nros or"
            echo "         ~/.nros/bin/nros — remove it if activate.sh does not win."
            host_rc=1
        else
            echo "  [OK] \`nros\` on PATH is this checkout's binary"
        fi
    else
        echo "  [INFO] no \`nros\` on PATH — recipes fall back to the in-tree"
        echo "         binary. \`source ./activate.sh\` to type \`nros\` yourself."
    fi
    # issue 0653 — a RETIRED SDK store entry that is still installed.
    #
    # The store accumulates and nothing prunes it (issue 0500), so a tool that
    # nano-ros stopped shipping stays where it was put — and until this was
    # found, `zenohd` also stayed on PATH, because it was still named in
    # `scripts/sdk-path-tools.txt`. That is worse than wasted disk: `command -v
    # zenohd` returned a RETIRED router (1.7.2) months after RFC-0075 moved the
    # router to ROS's `rmw_zenohd` (zenoh-c 1.8.0). Reported rather than deleted
    # — doctor is read-only, and this is the user's machine.
    retired_store=""
    while IFS= read -r retired_tool; do
        case "$retired_tool" in ''|\#*) continue ;; esac
        d="${NROS_HOME:-$HOME/.nros}/sdk/$retired_tool"
        [ -d "$d" ] || continue
        retired_store="$retired_store $retired_tool"
    done < "{{justfile_directory()}}/scripts/sdk-retired-tools.txt"
    if [ -n "$retired_store" ]; then
        echo "  [WARN] retired SDK store entries still installed:$retired_store"
        echo "         nano-ros no longer ships these; nothing prunes the store."
        for retired_tool in $retired_store; do
            echo "         rm -rf ${NROS_HOME:-$HOME/.nros}/sdk/$retired_tool"
        done
    fi
    # Python, which nano-ros CHECKS and never installs.
    #
    # Zephyr's build scripts, west, colcon, rosidl_adapter and 37 of this
    # repo's own gates are Python, but provisioning an interpreter is a
    # host decision (PEP 668 refuses `pip --user` outright on Arch/Fedora/
    # Debian 12+, and the venv/pipx/distro-package choice differs per host).
    # So doctor reports; `scripts/zephyr/setup.sh` refuses to continue.
    #
    # The interpreter reported is the one a lane would USE — `NROS_PYTHON`
    # when set, else the `scripts/zephyr/.venv` that activate.sh adopts when
    # present, else PATH's python3. Reporting a different one from the one
    # that gets used is how "setup succeeded" stopped meaning anything.
    py_for_lane="${NROS_PYTHON:-}"
    if [ -z "$py_for_lane" ] && [ -x "{{justfile_directory()}}/scripts/zephyr/.venv/bin/python3" ]; then
        py_for_lane="{{justfile_directory()}}/scripts/zephyr/.venv/bin/python3"
    fi
    [ -n "$py_for_lane" ] || py_for_lane="$(command -v python3 || true)"
    if [ -z "$py_for_lane" ]; then
        echo "  [MISSING] python3 — Zephyr's build scripts and 37 repo gates are Python"
    elif py_report="$(python3 "{{justfile_directory()}}/scripts/check-python-deps.py" \
            --python "$py_for_lane" west zephyr-build 2>&1)"; then
        echo "  [OK] python: $("$py_for_lane" -V 2>&1) ($py_for_lane) — west + zephyr-build deps"
    else
        echo "  [WARN] python deps missing for the Zephyr lanes ($py_for_lane):"
        echo "$py_report" | sed 's/^/         /'
    fi
    # NuttX's kconfig frontend. This one nano-ros DOES self-provision, into a
    # repo-local venv (scripts/nuttx/build-nuttx.sh), and it stays that way:
    # issue 0431 was every NuttX cell silently skipping on a host that had the
    # toolchain, qemu and sources but no kconfig, and `pip install kconfiglib`
    # is refused on PEP 668 distros while a venv's own pip is not. It differs
    # from the Zephyr venv on the three counts that matter — repo-local so
    # nothing outside NuttX sees it, last-resort (only when neither
    # `kconfig-conf` nor `olddefconfig` is present), and self-cleaning on
    # failure. Reported here so it is visible rather than invisible.
    if command -v kconfig-conf >/dev/null 2>&1 || command -v olddefconfig >/dev/null 2>&1; then
        echo "  [OK] kconfig frontend on PATH (NuttX needs no venv)"
    elif [ -x "{{justfile_directory()}}/build/nuttx-kconfig-venv/bin/olddefconfig" ]; then
        echo "  [OK] kconfig: repo-local venv (build/nuttx-kconfig-venv) — NuttX only"
    else
        echo "  [INFO] no kconfig frontend; the NuttX lane will provision one into"
        echo "         build/nuttx-kconfig-venv on first use (issue 0431), or install"
        echo "         a distro kconfig-frontends-nox package."
    fi
    # clang-format pin (consistent C/C++ formatting across machines + CI).
    want_cf="$(cat "{{justfile_directory()}}/.clang-format-version" 2>/dev/null || echo 17.0.6)"
    pinned_cf="{{justfile_directory()}}/build/clang-format/bin/clang-format"
    if [ -x "$pinned_cf" ] && "$pinned_cf" --version 2>/dev/null | grep -q "$want_cf"; then
        echo "  [OK] clang-format: $want_cf (pinned, build/clang-format)"
    elif command -v clang-format >/dev/null 2>&1; then
        have_cf="$(clang-format --version 2>/dev/null | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
        if [ "$have_cf" = "$want_cf" ]; then
            echo "  [OK] clang-format: $have_cf (PATH, matches pin)"
        else
            echo "  [WARN] clang-format $have_cf on PATH != pinned $want_cf — run: just setup-clang-format"
        fi
    else
        echo "  [MISSING] clang-format — run: just setup-clang-format"
    fi
    # phase-333 / issue 0394 — a `generated/` msg crate left over from before
    # constant versioning still carries the ament version of whatever interface
    # source THIS host had (`std_msgs 4.9.1`, `action_msgs 1.2.2`, …). It is
    # gitignored, so a fresh clone never sees it and CI cannot catch it, but on
    # a long-lived checkout it breaks any leaf whose lock is TRACKED: cargo
    # wants to re-resolve and `--locked` refuses, with an error that names the
    # lock and says nothing about the real cause. Report the remedy here rather
    # than let the next person debug a lockfile that was never wrong.
    stale_gen=$(find "{{justfile_directory()}}/packages" "{{justfile_directory()}}/examples" \
        -path "*/generated/*/Cargo.toml" -not -path "*/target/*" 2>/dev/null \
        | while read -r f; do
              v=$(grep -m1 '^version' "$f" 2>/dev/null | cut -d'"' -f2)
              [ -n "$v" ] && [ "$v" != "0.0.0" ] && echo "$f"
          done | sed 's|/generated/.*||' | sort -u)
    if [ -n "$stale_gen" ]; then
        n=$(printf '%s\n' "$stale_gen" | wc -l)
        echo "  [WARN] $n workspace(s) carry PRE-0.0.0 generated msg crates (env-baked versions)."
        echo "         Harmless where the lock is untracked; breaks \`--locked\` where it is not."
        echo "         Re-sync the ones that fail to build:  cd <dir> && nros sync"
    else
        echo "  [OK] generated msg crates: all constant-versioned (0.0.0)"
    fi
    # Compiler cache. `RUSTC_WRAPPER` above auto-uses sccache when it's on PATH,
    # which roughly halves clean/CI rebuilds (measured ~46%, see
    # docs/development/build-ux-audit.md). Surface its absence so it's a known
    # choice, not a silent slowdown. Host C builds (e.g. the zenoh-pico compile)
    # additionally need `CC`/`CXX="sccache cc"` — opt-in, since it only wraps
    # host compiles (cross toolchains set their compiler explicitly).
    if command -v sccache >/dev/null 2>&1; then
        echo "  [OK] sccache: $(sccache --version 2>/dev/null | head -1) — rustc caching on"
    else
        echo "  [INFO] sccache not found — builds are uncached (RUSTC_WRAPPER empty);"
        echo "         installing it ~halves clean rebuilds. See docs/development/build-ux-audit.md"
    fi
    # `rmw_zenoh_cpp` FROM THE ROS INSTALL. The interop lanes need the apt
    # package, not merely a router binary: the peer runs
    # `source /opt/ros/<distro>/setup.bash && export
    # RMW_IMPLEMENTATION=rmw_zenoh_cpp`, so the RMW must resolve in the ROS
    # prefix. Without it those lanes report `[SKIPPED:capability]`, which reads
    # as green — three issues were closed "unverifiable" that way on 2026-08-17.
    #
    # There is no source overlay to fall back on: it and its submodule were
    # removed once measurement showed nothing used them (RFC-0075, amended
    # 2026-08-19). The apt package is the only source of the peer.
    # issue 0654 — ask the SSoT where the router is, do not construct a path.
    # Constructing `/opt/ros/$ROS_DISTRO/...` here reported "not installed" on a
    # host whose ROS lives anywhere else, which is the case `AMENT_PREFIX_PATH`
    # was added to `nros_zenohd_bin` for (issue 0653). doctor telling a working
    # host it is broken is the failure mode doctor exists to prevent.
    # shellcheck source=scripts/dev/zenohd.sh
    . "{{justfile_directory()}}/scripts/dev/zenohd.sh"
    ros_distro="${ROS_DISTRO:-humble}"
    zenoh_rmw="$(nros_zenohd_bin 2>/dev/null)" || zenoh_rmw=""
    if [ -n "$zenoh_rmw" ] && [ -x "$zenoh_rmw" ]; then
        echo "  [OK] rmw_zenoh_cpp: ${zenoh_rmw} (interop lanes runnable)"
    elif [ -n "${NROS_RMW_ZENOHD:-}" ] && [ -x "${NROS_RMW_ZENOHD}" ]; then
        echo "  [WARN] rmw_zenoh_cpp not installed under /opt/ros/${ros_distro};"
        echo "         NROS_RMW_ZENOHD supplies a ROUTER, but the interop peer"
        echo "         resolves its RMW from the ROS prefix, so those lanes still fail."
        echo "         Install:  nros setup --system    (declared in nros-sdk-index.toml)"
    else
        echo "  [INFO] rmw_zenoh_cpp not installed — every zenoh interop lane will"
        echo "         SKIP (\`[SKIPPED:capability]\`), which reads as green rather"
        echo "         than as absent coverage."
        echo "         Install:  nros setup --system    (declared in nros-sdk-index.toml)"
    fi
    # One verdict, at the END. doctor's contract is that it reports every unmet
    # precondition in one run (the `check tier-preconditions` shape), so a FAIL
    # above must not short-circuit the blocks below it.
    exit "$host_rc"

# Internal: walk every module in `tier` calling the requested recipe
# (setup or doctor). `base` is the safe quick-start tier; `all` is the
# full contributor/test-all tier. Unknown tier exits non-zero so a typo
# doesn't silently pick the wrong module list.
[private]
_orchestrate verb tier="everything":
    #!/usr/bin/env bash
    set +e
    failed=()
    run() {
        local mod=$1
        echo ""
        echo "=== $mod ==="
        if just "$mod" {{verb}}; then
            :
        else
            failed+=("$mod")
        fi
    }
    # Tiers:
    #   - `base` : quick start for first-time users (workspace)
    #             phase-362 — no router here: it comes from ROS (`rmw_zenoh_cpp`),
    #             which is not ours to provision.
    #   - `all`  : full contributor / test-all setup
    # Legacy aliases:
    #   - `minimal` and `default` -> base
    #   - `everything` and `extended` -> all
    case "{{tier}}" in
        base|quickstart|minimal|default)
            run workspace
            ;;
        all|everything|contributor|extended)
            run workspace
            run verification
            run qemu
            run freertos
            run nuttx
            run threadx_linux
            run threadx_riscv64
            run esp32
            run zephyr
            run xrce
            # No `run rmw_zenoh` — phase-362 / RFC-0075 retired the vendored
            # router (`third-party/zenoh/zenoh/` and `build/zenohd/` are GONE);
            # the router now comes from ROS as `rmw_zenoh_cpp`. The line
            # survived the retirement and named a module that does not exist,
            # so `just setup all` / `just doctor tier=all` reported a failure
            # nobody could fix — measured 2026-08-31: "doctor finished with 3
            # failure(s): workspace verification rmw_zenoh".
            run cyclonedds
            run esp_idf
            run px4
            ;;
        *)
            echo "unknown tier '{{tier}}' — expected one of: base, all" >&2
            echo "(aliases: quickstart/minimal/default -> base; contributor/everything/extended -> all)" >&2
            exit 2
            ;;
    esac
    echo ""
    # Phase 142.6 — repeat the qemu < 7.2 PPA hint at the end of
    # `just doctor` so users don't scroll past it during the qemu
    # block. Skipped for `setup` (it would just duplicate the
    # `just qemu setup` output) and for `base` (no qemu in
    # that tier). Best-effort: silent if qemu missing entirely.
    if [[ "{{verb}}" == "doctor" && "{{tier}}" != "base" && "{{tier}}" != "quickstart" && "{{tier}}" != "minimal" && "{{tier}}" != "default" ]]; then
        if command -v qemu-system-arm >/dev/null 2>&1; then
            ver=$(qemu-system-arm --version 2>/dev/null | head -1 | sed -E 's/^[^0-9]*([0-9]+\.[0-9]+).*/\1/')
            major=${ver%%.*}
            minor=${ver##*.}
            if [ -n "$ver" ] && ! { [ "$major" -gt 7 ] || { [ "$major" -eq 7 ] && [ "$minor" -ge 2 ]; }; }; then
                echo "================================================================="
                echo "  REMINDER — system qemu-system-arm is $ver (< 7.2)."
                echo "================================================================="
                echo "  NuttX DDS multi-instance + ThreadX RV64 DDS tests need"
                echo "  '-netdev dgram,local.type=unix,...' from QEMU 7.2+."
                echo ""
                echo "  Primary remedy (no sudo, portable): just qemu setup-qemu"
                echo ""
                # issue 0726 — fork-free distro test. A matcher that fails to
                # START reads as "not ubuntu" and silently prints the
                # build-from-source remedy to an Ubuntu user, which is a wrong
                # ANSWER rather than an error anyone would trace back.
                _nros_os_id=""
                if [ -f /etc/os-release ]; then
                    while IFS= read -r _line; do
                        case "$_line" in ID=ubuntu) _nros_os_id=ubuntu ;; esac
                    done < /etc/os-release
                fi
                if [ "$_nros_os_id" = ubuntu ]; then
                    echo "  Fallback (system-wide, requires sudo) — Canonical PPA:"
                    echo "    sudo add-apt-repository ppa:canonical-server/server-backports"
                    echo "    then: nros setup --system   (composes the install command)"
                else
                    echo "  Fallback: build from source — https://www.qemu.org/download/#source"
                fi
                echo "================================================================="
                echo ""
            fi
        fi
    fi
    if [ ${#failed[@]} -gt 0 ]; then
        echo "{{verb}} finished with ${#failed[@]} failure(s): ${failed[*]}"
        echo "Re-run individually: just <module> {{verb}}"
        echo "(tier: {{tier}})"
        exit 1
    fi
    echo "{{verb}} complete! (tier: {{tier}})"

# Generate Rust API documentation (rustdoc)
[group("docs")]
doc-rust:
    cargo doc --workspace --no-deps

# Generate C API documentation (Doxygen)
# Requires doxygen — skips with a warning if not installed.
# The generated header must exist (run `cargo build -p nros-c` first).
[group("docs")]
doc-c:
    #!/usr/bin/env bash
    set -e
    if ! command -v doxygen &>/dev/null; then
        echo "WARNING: doxygen not found — skipping C API docs."
        echo "Install with: nros setup --system   (composes the install command; [system.doxygen])"
        exit 0
    fi
    header="packages/api/nros-c/include/nros/nros_generated.h"
    if [ ! -f "$header" ]; then
        echo "Generated header not found, building nros-c first..."
        # phase-361 W3 — `std` explicit (host build; nros-c `default = []` now).
        cargo build -p nros-c --features "std,rmw-zenoh,platform-posix,ros-humble"
    fi
    mkdir -p target/doxygen/c
    (cd packages/api/nros-c && doxygen Doxyfile)
    echo "C API docs generated: target/doxygen/c/html/index.html"

# Verify hand-written C headers are syntactically correct.
# Signature drift against Rust is caught at link time by `just test-c`.
[private]
doc-c-check:
    #!/usr/bin/env bash
    set -e
    echo "Checking C headers for syntax errors..."
    cc -fsyntax-only \
        -Ipackages/api/nros-c/include \
        -include packages/api/nros-c/include/nros/nros.h \
        -x c /dev/null
    echo "All C headers are syntactically correct."

# Generate C++ API documentation (Doxygen).
[group("docs")]
doc-cpp:
    #!/usr/bin/env bash
    set -e
    if ! command -v doxygen &>/dev/null; then
        echo "WARNING: doxygen not found — skipping C++ API docs."
        echo "Install with: nros setup --system   (composes the install command; [system.doxygen])"
        exit 0
    fi
    mkdir -p target/doxygen/cpp
    (cd packages/api/nros-cpp && doxygen Doxyfile)
    echo "C++ API docs generated: target/doxygen/cpp/html/index.html"

# Generate Doxygen for the RMW vtable (porter-facing).
[private]
doc-rmw-cffi:
    #!/usr/bin/env bash
    set -e
    if ! command -v doxygen &>/dev/null; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip docs-rmw-cffi "doxygen not found"
        exit 0
    fi
    mkdir -p target/doxygen/rmw-cffi
    # Issue 0581 — the Doxyfile lives with the ABI crate, not the shim crate.
    # phase-321 W2.e moved the RMW SHIM crates into packages/rmw/; the ABI
    # headers (and this Doxyfile, whose PROJECT_NAME is "nros rmw-cffi" and
    # whose OUTPUT_DIRECTORY is target/doxygen/rmw-cffi) stayed in
    # packages/core/nros-rmw-abi. The recipe followed the crates and the
    # Doxyfile did not move with them.
    (cd packages/core/nros-rmw-abi && doxygen Doxyfile)
    ls target/doxygen/rmw-cffi/html/*_8h.html >/dev/null 2>&1 || {
        echo "doc-rmw-cffi: doxygen emitted NO header pages — INPUT paths stale?" >&2
        exit 1
    }
    echo "rmw-cffi docs generated: target/doxygen/rmw-cffi/html/index.html"

# Generate Doxygen for the platform ABI (porter-facing), from the SSoT C
# headers in nros-platform-api (RFC-0054). The pre-0054 version of this
# recipe cbindgen-built a header out of the cffi crate and pointed doxygen
# at a path that stopped existing when the direction flipped — doxygen
# warns-and-succeeds on missing INPUT, so the published "canonical
# reference" was an EMPTY shell (mainpage, zero header pages) and nothing
# went red. Hence the emitted-page assertion below, on both doc recipes.
[private]
doc-platform-cffi:
    #!/usr/bin/env bash
    set -e
    if ! command -v doxygen &>/dev/null; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip docs-platform-cffi "doxygen not found"
        exit 0
    fi
    mkdir -p target/doxygen/platform-cffi
    (cd packages/platform/nros-platform-api && doxygen Doxyfile)
    ls target/doxygen/platform-cffi/html/*_8h.html >/dev/null 2>&1 || {
        echo "doc-platform-cffi: doxygen emitted NO header pages — INPUT paths stale?" >&2
        exit 1
    }
    echo "platform-cffi docs generated: target/doxygen/platform-cffi/html/index.html"

# Generate all documentation (Rust + C + C++ + cffi vtables + book).
[group("docs")]
doc: doc-rust doc-c doc-cpp doc-rmw-cffi doc-platform-cffi

# Install mdBook tooling used by `just book` and `just book-serve`.
[group("docs")]
setup-docs:
    #!/usr/bin/env bash
    set -e
    ensure_cargo_tool() {
        local tool="$1"
        local crate="$2"
        local version="$3"
        local current=""
        if command -v "$tool" >/dev/null 2>&1; then
            current="$($tool --version | head -1 | grep -Eo '[0-9]+\.[0-9]+\.[0-9]+' | head -1 || true)"
        fi
        if [ "$current" = "$version" ]; then
            echo "  [OK] $tool: $($tool --version | head -1)"
        else
            if [ -n "$current" ]; then
                echo "Installing $tool $version (current: $current)..."
            else
                echo "Installing $tool $version..."
            fi
            cargo install --locked --force "$crate" --version "$version"
        fi
    }
    # mdBook comes from the project's own PREBUILT release, not `cargo install`:
    # it drags a large dependency tree and a docs tool has no business costing a
    # compile. The pinned copy wins over any on PATH, so the book renders the
    # same on every host (issue 0500's lesson — a tool resolved by PATH is
    # whichever one happens to be first). phase-422 W2: the pin, the per-host
    # asset and its sha256 live in `[tool.mdbook]`, so this is a thin
    # `nros setup --tool` caller like every other provisioning line.
    nros setup --tool mdbook
    # PAIRED, and the pairing is the whole reason this line moved with the one
    # above: mdbook-mermaid 0.17 speaks the mdBook 0.5 preprocessor protocol and
    # 0.14 speaks 0.4's. Bumping mdBook to 0.5.4 without this fails at render
    # time, not at install time.
    ensure_cargo_tool mdbook-mermaid mdbook-mermaid 0.17.1
    if ! command -v doxygen >/dev/null 2>&1; then
        echo "  [INFO] doxygen not found; install with your package manager for API docs."
    else
        echo "  [OK] doxygen: $(doxygen --version | head -1)"
    fi

# Build mdBook + stage rustdoc/Doxygen output beneath book/book/api/.
# Mirrors the deploy-book.yml workflow so contributors can preview the
# full deployed site (book + native API docs) locally.
#
# `target/doc/` is wiped before `cargo doc` so prior `cargo doc --workspace`
# runs don't leak into the deployed rustdoc tree (everything under
# target/doc/ gets copied verbatim).
[group("docs")]
book:
    #!/usr/bin/env bash
    set -e
    rm -rf target/doc target/doxygen
    # `nros::Executor`, `nros::Promise`, `nros::Node`, etc. only re-export
    # under an rmw feature, so pass an rmw + platform combo or the deployed
    # rustdoc omits the public-facing types and the reference stub's
    # `[Executor](struct.Executor.html)` link 404s.
    #
    # Issue 0581 — this said `rmw-zenoh` and had not compiled since the RMW
    # backends moved behind the CFFI seam (RFC-0054): `nros` carries
    # `rmw-cffi`, `rmw-cyclonedds`, `rmw-lending` and no `rmw-zenoh`, so cargo
    # failed with "none of the selected packages contains this feature" before
    # rustdoc ran — which also means `mdbook build` never ran, so a book-only
    # change could not be previewed at all. The gate in `nros/src/lib.rs` is
    # `#[cfg(feature = "rmw-cffi")]` today; the old comment's
    # `rmw-xrce / rmw-dds` are gone with it.
    # std,env,macros joined the list when phase-359/361 made them optional:
    # without them rustdoc drops ExecutorConfigEnvExt::from_env, the alloc-
    # gated ExecutorNodeRuntime::spin and `nros::node!` — and every doc
    # comment linking those fails the build (2026-08-21 book red).
    # The crate + feature set is DATA, in scripts/build/rustdoc-set.sh, because
    # `just check rustdoc-links` builds the same set on every pull request
    # (issue 1110). A gate documenting a different set from the one that
    # deploys can be green while the deploy is red, which is the state that
    # issue records.
    source scripts/build/rustdoc-set.sh
    mapfile -t _nros_doc_pkgs < <(nros_rustdoc_package_args)
    cargo doc --no-deps \
        --features "$NROS_RUSTDOC_FEATURES" \
        "${_nros_doc_pkgs[@]}"
    just doc-c
    just doc-cpp
    just doc-rmw-cffi
    just doc-platform-cffi
    # Prefer the PINNED binary; fall back to PATH; fail with the remedy rather
    # than `mdbook: command not found` after rustdoc has already run for minutes.
    #
    # The pinned path is CONSTRUCTED from the index pin via `nros sdk-path`,
    # never searched for in the store (issue 0625) — `--require` is deliberately
    # NOT passed, because a missing install must reach the PATH fallback and
    # then the remedy below, not abort with a store diagnostic.
    MDBOOK_DIR="$(nros sdk-path mdbook 2>/dev/null || true)"
    if [ -n "$MDBOOK_DIR" ] && [ -x "$MDBOOK_DIR/bin/mdbook" ]; then
        MDBOOK="$MDBOOK_DIR/bin/mdbook"
    elif command -v mdbook >/dev/null 2>&1; then
        MDBOOK=mdbook
    else
        echo "ERROR: mdbook not found. Provision the pinned one:" >&2
        echo "           just setup-mdbook" >&2
        exit 1
    fi
    echo "book: using $MDBOOK ($($MDBOOK --version))"
    # The inventory is generated and untracked, and SUMMARY.md links it, so it
    # must exist before mdbook runs or the build 404s on a listed chapter.
    python3 scripts/gen-pool-inventory.py
    python3 scripts/gen-config-surface.py
    "$MDBOOK" build book
    mkdir -p book/book/api
    rm -rf book/book/api/rust book/book/api/c book/book/api/cpp \
           book/book/api/rmw-cffi book/book/api/platform-cffi
    cp -r target/doc                          book/book/api/rust
    cp -r target/doxygen/c/html               book/book/api/c
    cp -r target/doxygen/cpp/html             book/book/api/cpp
    cp -r target/doxygen/rmw-cffi/html        book/book/api/rmw-cffi
    cp -r target/doxygen/platform-cffi/html   book/book/api/platform-cffi
    # rustdoc has no top-level index when invoked with multiple `-p`; stage
    # a tiny redirect so visiting `api/rust/` lands on the umbrella crate.
    cat > book/book/api/rust/index.html <<'HTML'
    <!doctype html>
    <meta http-equiv="refresh" content="0; url=nros/index.html">
    <link rel="canonical" href="nros/index.html">
    <p>Redirecting to <a href="nros/index.html">nros</a>…</p>
    HTML
    echo "Built: book/book/index.html (open with xdg-open book/book/index.html)"

# Serve mdBook with live reload (book chapters only — does not rebuild
# rustdoc/Doxygen API docs; use `just book` for the full deployed view).
[group("docs")]
book-serve:
    mdbook serve book/ --open

# Clean example build artifacts across platform namespaces.
[group("maintenance")]
clean-examples:
    just native clean
    just qemu clean
    just freertos clean
    just nuttx clean
    just threadx_linux clean
    just threadx_riscv64 clean
    just zephyr clean
    just esp32 clean
    just esp_idf clean
    just px4 clean
    @echo "All example artifacts cleaned"

# Clean fixture-only orchestration outputs.
[group("maintenance")]
clean-fixtures:
    #!/usr/bin/env bash
    set -e
    rm -rf tmp/build-test-fixtures-* tmp/build-test-fixtures-latest
    rm -rf build/zephyr-fixtures
    find tests -maxdepth 2 -type d -name build -exec rm -rf {} + 2>/dev/null || true
    find tests -maxdepth 2 -type f \( -name sdkconfig -o -name 'sdkconfig.old' \) \
        -delete 2>/dev/null || true
    echo "Fixture orchestration artifacts cleaned"

# Clean BUILD-stage artifacts (examples, fixtures, cargo target) created by the
# broad build + test-fixture recipes.
#
# Phase 184.1 — `clean` removes only build-stage outputs; it MUST NOT delete
# SDK/tool installs produced by `just setup` (build/{install,cyclonedds,qemu,
# xrce-agent,zenohd,zephyr-cache}). The old `rm -rf build` + `clean-zenohd`
# nuked those, so a `clean → setup → build → test` cycle on the default (base)
# tier left Cyclone (build/install), the XRCE Agent, and the patched qemu gone,
# producing ~16+ false test-all failures. Build-stage subdirs under build/ are
# removed explicitly below; everything else under build/ is a setup install and
# survives. Use `just clean-setup` to remove the SDK installs (full re-setup).
[group("maintenance")]
clean: clean-examples clean-fixtures
    cargo clean
    # The codegen workspace (packages/codegen/packages) is NOT cleaned: the host
    # `nros-codegen` CLI it produces is a setup-stage TOOL (built by
    # `just workspace build-codegen` / `just setup`, like idlc/zenohd), so it
    # survives `clean`. The find below already excludes it. `just clean-setup`
    # removes it for a full tool re-build.
    # Clean stale per-crate target/ dirs inside workspace members (left by standalone builds)
    find packages -maxdepth 4 -name target -type d -not -path '*/codegen/packages/*' -exec rm -rf {} + 2>/dev/null || true
    # Catch-all for example target/ dirs the per-platform `clean` recipes miss
    # (e.g. a west-built entry leaf, fixture entry crates, …).
    # `-prune` so we don't recurse into a target we're already deleting.
    find examples packages/testing/nros-tests/fixtures -type d -name target -prune -exec rm -rf {} + 2>/dev/null || true
    # Ephemeral scratch target dirs (issue 0400). Each is box-aware: the recipe
    # that writes it roots the suffix at the active base via
    # `nros_scoped_target_dir` (scripts/build/cargo.sh) — host `target-<suffix>`,
    # ROS-distrobox `$CARGO_TARGET_DIR-<suffix>`. `clean` is non-shebang so it
    # cannot source the helper; the same expansion is inlined here to remove
    # BOTH forms (the box variant is a no-op dup of the host one when unset).
    rm -rf target-embedded target-zpico-multisession
    rm -rf "${CARGO_TARGET_DIR:-$PWD/target}-embedded" "${CARGO_TARGET_DIR:-$PWD/target}-zpico-multisession"
    # Build-stage outputs under build/ (SDK installs preserved — see clean-setup).
    source scripts/build/build-root.sh
    rm -rf build/zephyr-fixtures "$(nros_build_dir "$NROS_KIND_ESP32_QEMU")" \
        "$(nros_build_dir "$NROS_KIND_QEMU_ZENOH_PICO")"
    @echo "Build artifacts cleaned (SDK installs + host nros-codegen preserved; 'just clean-setup' to remove them)"

# Remove SDK/tool installs produced by `just setup` (Cyclone, XRCE Agent,
# patched qemu, zenohd, zephyr cache, host nros-codegen). Full blanket nuke —
# re-run `just setup tier=all` afterwards. Phase 184: per-platform setup-undo
# (uninstall just one platform's SDKs) is deferred pending design discussion.
[group("maintenance")]
clean-setup:
    # phase-362 — `build/zenohd` joins the list; the vendored router is gone,
    # but an existing checkout still has one to remove.
    rm -rf build/install build/cyclonedds build/qemu build/xrce-agent build/zephyr-cache build/zenohd
    # The Zephyr SDK install + downloads live under `scripts/zephyr/` (gitignored,
    # ~9 GB) — a `just setup`-stage tool install, so nuke it here too. Re-fetched
    # by the zephyr setup recipe.
    rm -rf scripts/zephyr/sdk scripts/zephyr/downloads
    # Phase 218 — `nros` builds in-tree at `packages/cli/target/`; that
    # tree is gitignored and a regular `cargo clean` (run from the
    # CLI sub-workspace) removes it. The transitional `~/.nros/`
    # install location for pre-218 users can be cleaned with:
    #   rm -rf "${NROS_HOME:-$HOME/.nros}".
    @echo "SDK/tool installs removed. Re-run 'just setup tier=all'; the nros CLI rebuilds via 'just setup-cli'."

# Phase 218.J — JetPack-style bundle version bump.
#
# Updates `[workspace.package].version` in BOTH the runtime workspace
# at `Cargo.toml` AND the CLI sub-workspace at `packages/cli/Cargo.toml`
# atomically, then runs `scripts/check-version-lockstep.sh` to confirm.
# Distribution model is git tag + release-page artifacts (no
# crates.io); after `just release-bump 0.4.1`, the maintainer:
#   1. `git commit -am 'release: nros-v0.4.1'`
#   2. `git tag nros-v0.4.1`
#   3. `git push origin main nros-v0.4.1`
# The Phase 218.G release workflow builds the four-triple CLI binaries
# off the tag + attaches them to the GitHub release.
[group("release")]
release-bump version:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ ! "{{version}}" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9.-]+)?$ ]]; then
        echo "release-bump: version must look like X.Y.Z (optionally -prerelease); got '{{version}}'" >&2
        exit 1
    fi
    bump_workspace_version() {
        local toml="$1" newver="$2"
        awk -v newver="$newver" '
            /^\[workspace\.package\]/ { in_section = 1; print; next }
            /^\[/                     { in_section = 0 }
            in_section && /^version[ \t]*=[ \t]*"/ {
                sub(/"[^"]*"/, "\"" newver "\"")
                in_section = 0
            }
            { print }
        ' "$toml" > "$toml.tmp"
        mv "$toml.tmp" "$toml"
    }
    bump_workspace_version Cargo.toml "{{version}}"
    bump_workspace_version packages/cli/Cargo.toml "{{version}}"
    ./scripts/check-version-lockstep.sh
    echo "release-bump: bundle bumped to {{version}}. Review with: git diff Cargo.toml packages/cli/Cargo.toml"

# =============================================================================
# Docker: use `just docker build`, `just docker shell`, `just docker test`, etc.
# =============================================================================

# Preview the assembled release notes WITHOUT consuming the fragments.
[group("docs")]
changelog:
    @python3 -m towncrier build --draft --version "$(git describe --tags --abbrev=0 2>/dev/null || echo unreleased)"

# Add a changelog fragment (issue 0885). One file per change, so two pull
# requests never edit the same region of CHANGELOG.md — towncrier's whole
# argument, and the same fix issue 0884 applied to the issue ledger.
#
#   just changelog-add 885 feat "just issues queries the ledger offline"
#
# QUOTE THE TEXT, and prefer single quotes if it contains backticks: the
# argument reaches a bash recipe, so `like this` is COMMAND SUBSTITUTION and
# your prose silently disappears. Measured while writing this recipe — a
# fragment came out as "towncrier fragments replace a shared CHANGELOG;
# writes one", with the backticked words evaluated and gone. Markdown code
# spans are exactly what a changelog line wants, so this will bite.
[group("docs")]
changelog-add issue type text:
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{type}}" in
        fix|feat|perf|breaking|docs) ;;
        *) echo "type must be one of: fix feat perf breaking docs" >&2; exit 2 ;;
    esac
    f="changelog.d/$(printf '%04d' "{{issue}}").{{type}}.md"
    if [ -e "$f" ]; then echo "exists: $f" >&2; exit 2; fi
    printf '%s\n' "{{text}}" > "$f"
    echo "wrote $f"

# Consume fragments into CHANGELOG.md and DELETE them. Release-time only.
[group("docs")]
changelog-release version:
    @python3 -m towncrier build --yes --version "{{version}}"

# Dev utilities — check the environment, then install the repo's own tools
# into it (issue 0885).
#
#   just dev-tools              report what is missing
#   just dev-tools --install    install it into the probed interpreter
#
# The line nano-ros draws: it does not provision an INTERPRETER — no venv
# creation, no choosing between system / `--user` / pipx on your behalf, because
# that is a decision about your machine and a wrong guess surfaces four frames
# inside cmake as `Error finding board: mps2`. But `towncrier` or a pinned
# `clang-format` inside a Python you already chose is just a tool the repo
# needs, so `--install` puts it there.
#
# DEV groups only. `--install zephyr-build` is refused: a build environment is
# yours to assemble, and installing into it silently is how three interpreters
# end up in play with nobody knowing which one a lane will use.
#
# On a PEP 668 host the system interpreter refuses, and the error says exactly
# what to do instead (venv, or `--user`). That refusal is pip's and it is
# better than any pre-flight guess this script could make.
# Provision the pinned mdBook from the project's prebuilt release (no compile).
# Separate from `setup-docs` so the book tool can be fixed without re-running
# the whole docs setup.
#
# A thin `nros setup --tool` caller (phase-422 W2): the version, the per-host
# asset URL and its sha256 are `[tool.mdbook]` in `nros-sdk-index.toml`. It used
# to be `scripts/setup-mdbook.sh` plus a hand-kept `scripts/mdbook-checksums.txt`
# — a second provisioning mechanism, with a second pin, for a tool the index
# already knows how to describe.
[group("docs")]
setup-mdbook:
    @nros setup --tool mdbook

[group("docs")]
dev-tools *args:
    @python3 scripts/check-python-deps.py dev-tools {{args}}
