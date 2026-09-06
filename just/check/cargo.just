# nano-ros gates: cargo
#
# Cargo manifests, lockfiles, workspace shape, and the build-script
# fingerprints that decide what rebuilds.
#
# One of the topic files `just/check.just` imports. The names stay FLAT -- a
# gate is still `just check <name>` -- because `import` merges definitions
# rather than namespacing them, and ~350 references across docs, scripts and
# workflows spell them that way.
#
# Adding a gate here does not touch any other file, which is the point: for
# most of this repo's life every new gate appended to ONE file, and
# `just/check.just` became the worst conflict surface in the tree at 48 of
# 300 commits.

# issues 0320 / 0334 — no build-host absolute paths in tracked code/config.
# A pure grep, so it belongs in the source-free tier (see #337 for what happens
# when a gate here needs more than the index).
[private]
absolute-paths:
    @scripts/ci/absolute-path-check.sh

# phase-336 — no build site may NAME a cargo profile: the flag and the artifact
# path both come from `nros profile`. The failure this prevents is silent (the
# builder writes one directory, the reader looks in another), which is why it
# is a gate and not a convention.
[private]
build-profile-literals:
    @bash scripts/check-build-profile-literals.sh

# RFC-0070 R1/R3 — the build-cache root derivation.
#
# phase-334 W2.b step 1 replaced `fixtures-target-dir.sh`'s hardcoded
# `$root/build/...` with `nros_build_dir`; step 2 did the same for the
# compile-check / cmake-fixtures / idf-fixtures / west-fixtures writers, their
# staleness probe and their Rust resolvers. Buildless and instant, so it belongs
# in `check-fast`: its whole value is asserting the emitted path is UNCHANGED,
# which is the property that makes "derivation first, paths later" safe.
#
# It also covers the `export -f` make-leaf path, where step 1 shipped a resolver
# that emitted an EMPTY `--target-dir` because it sourced build-root.sh from
# inside a function. An in-process-only assertion could not see that.
[private]
build-root:
    @bash packages/testing/nros-tests/tests/build_root_derivation.sh

[private]
build-rs-rerun-paths:
    @python3 scripts/check-build-rs-rerun-paths.py

# issue 0490 — a `cargo:rerun-if-changed` naming a path that does not exist makes
# cargo treat the unit as permanently dirty, so the build script and everything
# above it recompile on every invocation, silently and forever. Found in
# `packages/rmw/cffi/build.rs`, which sits under every image. Self-tests its own
# checker on every run.
# RFC-0087 D2 / phase-420 W2 — the `<build_type>` vocabulary AND the class
# boundary it exists for: a provider, board, entry or bringup may not claim an
# ament build type (a stock colcon would try to install firmware into a prefix),
# and an interface package may not claim a nano-ros one (a ROS 2 node consumes
# its output). Classification is from EVIDENCE — provisions, a sibling
# system.toml, an nros dep, an image-building cmake verb — never a hand list.
#
# A ratchet over scripts/build-type-spelling-baseline.json, because the tree is
# not migrated yet: 301 rows today, which phase-420 W3 empties. A NEW package
# binds immediately, and swapping one violating spelling for another is a new
# violation rather than a grandfathered one. Buildless, ~0.2 s.
[private]
build-type-spelling:
    @python3 scripts/check-build-type-spelling.py

# Issue 0701 — the OTHER half of ARCHITECTURE §2 clause (a).
#
# `check-feature-contract` enforces "a capability does not GRANT the heap" by
# scanning manifests. Nothing enforced the rest of the same sentence — "emit
# `compile_error!` naming the feature" — so a capability whose gated code calls
# `std::` with no guard passed every gate and failed the USER's build with a
# bare `cannot find crate std`, four frames deep, naming nothing they could act
# on. Two of them shipped that way (`nros-cpp`'s `metadata-mode` and `env`),
# each free-riding on a stricter guard in `nros` until issue 0669's follow-up
# correctly relaxed it.
#
# Lives in the census script because it needs exactly what the census walk
# needs — which cfg gates a given `std::` line — and a second spelling of that
# walk is the antipattern this repo keeps paying for. The counting path is
# untouched; this is a separate mode, with its own `--self-test`.
[group("ci")]
capability-flavour-guards:
    @python3 scripts/check-std-census.py --self-test
    @python3 scripts/check-std-census.py --check-guards

# A leaf `.cargo/config.toml` is tracked iff it holds content `nros sync`
# cannot regenerate. `**/.cargo/config.toml` is gitignored (most are pure sync
# output); this is the discrimination the blanket rule cannot make.
cargo-config-tracked:
    @bash scripts/check-cargo-config-tracked.sh

# issue 0820 — a cmake custom command that RUNS cargo must carry a `DEPFILE`.
#
# `add_custom_command(OUTPUT ...)` rebuilds only when an input it NAMES is newer
# than the output, and cargo's inputs are a graph over the workspace. Every
# hand-written `DEPENDS` here approximated that graph and every one of them was
# wrong the same way — it named the crate's own files and no nano-ros Rust — so
# the artifact came out NEWER than its sources holding older code, and nothing
# in the graph could notice. That is the museum binary 0820 reported, and it is
# indistinguishable from a runtime bug until someone wipes the directory.
#
# Cargo already writes the exact answer beside the artifact (`<artifact>.d`,
# Makefile format), which is the shape `DEPFILE` consumes. Three commands in
# this tree run cargo; two of them had no depfile when this gate landed, and
# both survived the issue's own 2026-09-01 sweep because that sweep classified
# FILES ("codegen") rather than commands. Hence a per-command predicate.
#
# Buildless — pure source analysis over the tracked cmake files, milliseconds.
[private]
cargo-custom-command-depfile:
    @python3 scripts/check-cargo-custom-command-depfile.py

# issue 0359/0378 — `--locked` is injected project-wide by the scripts/bin/cargo
# PATH shim (cargo has no config/env knob for it, and per-site flags would miss
# cmake/corrosion, which invoke `cargo` by name). This asserts the mechanism is
# still wired; without it every build silently rewrites Cargo.lock. Buildless.
[private]
cargo-locked:
    @bash scripts/check-cargo-locked.sh

# A cargo profile defined in BOTH `Cargo.toml` and `.cargo/config.toml` must
# agree. Both are load-bearing — the manifest one applies to the root
# workspace, the config one to the ~48 leaf crates outside it — so editing one
# silently gives half the tree different optimization settings, with no error
# anywhere. Buildless.
[private]
cargo-profile-mirror:
    @bash scripts/check-cargo-profile-mirror.sh

# phase-340 W3 — ONE `--target` spelling for every cargo command cmake emits.
#
# `--target <host-triple>` and no `--target` are different cargo identities on
# the same machine, and they share nothing — not even sccache entries (0 hits /
# 62 misses across the two spellings, measured on a private cold cache). Every
# corrosion target already passes `--target` (corrosion hardcodes it), so
# nano-ros' own cargo custom commands normalise to the explicit spelling and
# `_nros_resolve_rust_target()` is the single answer to "which triple".
#
# The gate configures a NONE-language cmake project against the module in the
# scopes that matter — no Corrosion at all, a toolchain variable, only
# Corrosion's CACHE copy (the phase-155 scope where a PARENT_SCOPE publish did
# not cross `add_subdirectory()`), and nothing at all — and asserts the last one
# FAILS rather than falling back to the implicit spelling. Buildless: no
# compiler, no cargo, no fixtures.
[private]
cargo-target-spelling:
    @bash packages/testing/nros-tests/tests/cargo_target_spelling.sh

# issue 0478 — every cc::Build must name the nano-ros cc policy helper
# (the strict diagnostics of 0383, and the clang-only frame-pointer flag gcc
# rejects). Both classes escaped once through an unrouted call site.
[private]
cc-build-policy:
    @bash scripts/check-cc-build-policy.sh

# Issue 0604 — `packages/cli/cli-source-dirs.txt` must equal cargo's resolve.
#
# That file IS the CLI freshness closure: `source_stamp.rs` hashes `packages/cli`
# plus the dirs it names, and every fixture in the repo keys on the resulting
# verdict. It replaced a textual `path = "…"` walk that was wrong in both
# directions at once (23 dirs where cargo resolves 8) — blind to
# `workspace = true` deps, so edits to `nros-core`/`nros-rmw` left the stamp
# FRESH, and blind to `optional = true`, so 17 crates the CLI never compiles
# re-staled it and, through it, every fixture.
#
# The generated file is only as trustworthy as this check: a stale list is a
# silent wrong stamp in whichever direction it drifted.
[group("ci")]
cli-source-dirs:
    @python3 scripts/gen-cli-source-dirs.py --check

# phase-340 W2.d — `--core-only` selects by the derived variant predicate; this
# holds it equivalent to the authored-`target_dir` spelling it replaced, on
# every platform a caller actually passes.
[private]
core-only-predicate:
    @bash packages/testing/nros-tests/tests/core_only_predicate.sh

# phase-361 W4 — the `std`/`alloc` feature contract (ARCHITECTURE.md §2).
# Six clauses over every crate in `packages/`: `std` implies `alloc` in the
# MANIFEST and nowhere else, the heap gate has one spelling, no `no_std` crate
# defaults to `std`, no declared `std`/`alloc` feature is inert, no `default`
# feature is unreachable from its dep-sites, and exactly one
# `#[global_allocator]` exists (nros-platform's).
#
# Buildless — reads manifests and sources. `--self-test` drives all six over
# synthetic trees and asserts each FIRES on a deliberate reintroduction; it
# found a real violation (`nros-rmw-zenoh-staticlib`) on its first real run,
# which W2.a's hand-sweep had missed.
[group("ci")]
feature-contract:
    @python3 scripts/check-feature-contract.py

# Phase: crate-version lockstep — every workspace crate shares the release
# version (the bump script edits them atomically). Mirrors the `check.yml`
# version-lockstep step so `just check` ⊇ the CI fast gate (single source of
# truth). Buildless.
# phase-314 W5 — one feature-set SSoT. Every failure this phase fixed was
# SILENT (a hook that applied on one path, an edition hardcoded so a non-humble
# build failed on the wire, capabilities a mixed workspace lost), so drift here
# needs a check rather than a convention. Buildless.
[private]
feature-set-ssot:
    @./scripts/check-feature-set-ssot.sh

# phase-359 W9 — every matrix platform resolves to exactly ONE std flavour, so a
# lane keyed on the platform cannot mix std and no_std images. Buildless (reads
# board manifests + the registry), so it belongs in the fast tier.
#
# The flavour is DERIVED, never listed: a board is std iff it enables `std` on
# its nros/nros-platform deps, followed through board->board deps. That is how
# NuttX resolves to std — the qemu board enables nothing itself, but the base
# board it links does. `--print` feeds `lane-filter.sh nostd`, so the gate and
# the lane share one derivation.
[group("ci")]
flavour-lanes:
    @python3 scripts/check-flavour-lanes.py

# Issue 0582 — a place meaning "the host" spelled as a literal triple. Nine
# sites wore this bug; five failed SILENTLY, because `NO_DEFAULT_PATH` turns a
# wrong path into an empty result, a skipped block and a much later failure that
# names neither. Invisible on x86, which is why it survived a year after being
# written down.
[group("main")]
host-triple-literals:
    @python3 scripts/check-host-triple-literals.py

# Issue 0660 — every `just <recipe>` inside a recipe body must name a real one.
#
# `just` resolves a recipe reference only when the recipe RUNS, so deleting a
# recipe leaves its callers parsing fine and dying on invocation. phase-362 W4
# retired `build-zenohd` and left TWELVE callers dead across three lane files;
# tier 1 stayed green because `ci` runs `test-all` and never touches the
# per-family `native test-*` recipes a developer actually types.
#
# Parses definitions rather than `just --summary`, which omits private recipes —
# bodies call those constantly (`just _count-real-failures`).
[group("ci")]
just-recipe-refs:
    @python3 scripts/check-just-recipe-refs.py

# Issue 0359 — leaf `Cargo.lock` files outside the root workspace must keep
# satisfying their own manifests. Nothing ran `--locked` over them, so drift
# grew silently with every manifest edit and a drifted lock pins NOTHING (the
# leaf resolves fresh on every build). Baselined, because 26 are drifted today
# and regenerating them is a supply-chain decision, not a cleanup — the gate
# fails on NEW drift and on a baselined leaf that stops drifting, so the
# backlog can only shrink. ~30s (it resolves each leaf; NOT `--offline`, which
# would conflate a cold cargo cache with real drift).
[private]
leaf-lockfiles:
    @bash scripts/check-leaf-lockfiles.sh

# issue 0722 — a manifest is read by whichever workspace claims it, and this
# tree has many roots. A crate no root claims can carry a duplicate key that
# cargo REFUSES, while `cargo metadata` from the repo root stays green.
[private]
manifests-parse:
    @python3 scripts/check-manifests-parse.py

# A nested workspace's `exclude` needs a matching repo-root exclude, or cargo
# run from INSIDE the excluded leaf dies with "current package believes it's in
# a workspace when it's not" — the west-built zephyr entries are all in this
# shape. phase-331's renames left five root excludes pointing at deleted `ws-*`
# paths and two live leaves unprotected, and nothing caught it: the root
# resolves fine, so the break only shows up in the embedded lane. Pure text, no
# cargo invocation. Buildless.
[private]
nested-workspace-excludes:
    @bash scripts/check-nested-workspace-excludes.sh

# Phase 251 — forbid `--allow-multiple-definition` in the build system (it lets
# two same-named-but-different functions coexist → wrong-copy hazard). Fails on
# any non-allowlisted real use; allowlist (scripts/allow-multiple-def-allowlist.txt)
# carries the audited exceptions, target empty. Buildless.
[private]
no-allow-multiple-def:
    @bash scripts/check-no-allow-multiple-def.sh

[group("check")]
no-tracked-file-find:
    @./scripts/check-no-tracked-file-find.sh

# issue 0491 — the sibling rule to the gate above: a `cargo:rerun-if-env-changed`
# on a PATH-valued variable fingerprints the SPELLING of a directory, and one
# directory has a different spelling per example leaf (`relative = true`), from
# `just` (absolute) and unset. Rows sharing one `--target-dir` (phase-340 groups)
# then invalidate each other forever — six FreeRTOS rows rebuilt 6 units on every
# probe, permanently. Self-tests its own classifier on every run.
[private]
path-env-fingerprints:
    @python3 scripts/check-path-env-fingerprints.py

# Issue 0336 — a retired submodule path must not survive anywhere that a user or
# a CI job would follow it. RFC-0060's sweep missed scripts/bootstrap.sh, nine
# doc copies and eight workflow refs; this grep would have caught all of them.
[private]
retired-submodule-refs:
    @bash scripts/check-retired-submodule-refs.sh

# Issue 0945 item 1 — the SELF-TEST half of the shared-cargo-dir witness.
#
# The witness itself is a BUILD-time check (it needs an artifact and a build
# dir, so there is nothing for it to look at here) armed by
# `nros_assert_shared_cargo_dir_used()` on every Corrosion leaf that shares.
# What this lane keeps honest is the check's own logic: five fixtures covering
# the healthy state and the four ways the redirect can be dead, including the
# one only the symlink arm can catch. A witness that silently stopped
# witnessing would be the defect it exists to prevent, one level up.
shared-cargo-dir-witness:
    @bash scripts/check-shared-cargo-dir-used.sh --self-test

# Issue 0570 — the vendored NuttX `libc` fork mirrors NuttX's opaque types as
# byte blobs, but NuttX sizes several of them from Kconfig, so a mirror that is
# too small is a stack smash at every libc call that writes the whole struct
# (#167's `pollfd`, #570's `pthread_attr_t` — both landed on a saved return
# address). Compiles a `sizeof` probe against the configured headers. NOT
# buildless: reports NOT CHECKED, exit 0, on a tree with no configured NuttX or
# no cross compiler, which is why it also runs in the nuttx lane where both
# exist.
# phase-363 W3/W4 — self-test for the one source-signature helper: no type
# filter, build output cannot leak in, deterministic order, dep-info parsed as
# Make syntax, and failures fatal. Hermetic (throwaway git repo in $TMPDIR).
[private]
source-manifest:
    @bash scripts/check-source-manifest.sh

# The sibling of `check-submodule-pins`: that one asks whether a pin moved the
# right DIRECTION, this one asks whether the commit it names exists anywhere a
# CLONE can reach.
#
# AGENTS.md's first submodule rule is "push the submodule commit BEFORE the
# superproject pins it", because nothing downstream recovers from getting it
# wrong — every clone and every CI run dies at `git submodule update` with
# `reference is not a tree`, and the offending commit has to be rewritten,
# since a pin cannot point at nothing. The rule was written down and nothing
# enforced it; `git push` will happily publish a gitlink naming a commit that
# exists only in someone's working tree.
#
# NOT on the `check-fast` line, and that is deliberate rather than an
# oversight: that tier is source-free and network-free by contract, while this
# gate's entire job is to ask the REMOTE. A local `cat-file -e` would be free
# and would pass on precisely the one host where the mistake is invisible.
# It runs as its own step on pull requests, and the pre-push hook runs it over
# the pins a push actually MOVES.
#
# Exit 2 (network unavailable) is reported, not swallowed and not treated as a
# failure — "your submodule is unpushed" is a false and very specific
# accusation to make at an offline laptop, and the remedy it implies is wrong.
[group("check")]
submodule-commits-reachable:
    #!/usr/bin/env bash
    set -uo pipefail
    source scripts/build/lane-skip.sh
    bash scripts/ci/submodule-commits-reachable.sh
    rc=$?
    # `nros_lane_skip`, NOT `echo … ; exit 0`. A gate that cannot ask its
    # question must report a THIRD verdict, or "nobody checked" is indexed as
    # "nothing wrong" — issue 0658, and `check-lane-skip-protocol` caught
    # exactly that spelling here on the first run.
    if [ "$rc" -eq 2 ]; then
        nros_lane_skip "no network: submodule pins were NOT verified against their remotes"
    fi
    exit "$rc"

# Issue 0550 — a submodule checked out BEHIND the commit the superproject
# records. Not in `check-fast`: drift is a working-copy state, so the index and
# the commit always agree in anything you can push and a source gate could never
# observe it. It runs as the first item of `check-tier-preconditions`, ahead of
# the CLI stamp, because `git submodule update` rewrites source mtimes and would
# re-stale anything cleared before it. Exposed standalone for diagnosis.
submodule-drift:
    @bash scripts/check-submodule-drift.sh

# issue 0560 — a lock whose dependency versions are decided by a SUBMODULE's
# manifest goes stale when the pointer moves, and the two halves are updated by
# different commits. `nros-launch-resolve` sat unbuildable on main that way:
# `--locked` refused the stale pin, and the only consumer is a dependency of the
# ~40-minute fixture lane, so it waited for whoever ran that next.
#
# `cargo metadata --locked --offline` — resolution, not a build, because
# resolution is what breaks. Sub-second, no network, and the leaf set is DERIVED
# from `.gitmodules` rather than listed.
[private]
submodule-pinned-locks:
    @python3 scripts/check-submodule-pinned-locks.py

[private]
submodule-pins:
    @bash scripts/ci/submodule-pins-check.sh

# phase-344 W7 — RFC-0070 R1 at WORKSPACE scope only. examples/** copy-out
# leaves keep the Cargo/CMake convention and are deliberately exempt.
[private]
workspace-build-output:
    @bash scripts/check-workspace-build-output.sh

[private]
workspace-order:
    @bash packages/testing/nros-tests/tests/workspace_order_gate.sh
