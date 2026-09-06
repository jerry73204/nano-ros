# nano-ros gates: fixtures
#
# Test fixtures, the lane/coordinate machinery, and the nextest
# filters that decide what runs.
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

# Issue 0900 (option 3) -- does an image's NROS_EXECUTOR_ACTION_CLIENTS agree
# with what it actually LINKS? The knob decides how many executor arena slots
# are budgeted at the worst-case (ActionClient) size, and the arena is inline on
# the TASK STACK, so `just mem-report` cannot see it: the checkable thing is the
# ENTITY that justifies the size, and `nm` reads a cross ELF perfectly well.
# This is the only half of 0900 that reaches embedded images -- the
# sidecar/metadata half cannot, because a cross-compiled component is
# unprobeable by construction.
#
# Cheap where it matters: instant on a tree with nothing built (it exits 78 and
# records a SKIP rather than reporting a pass over zero images), ~18 s on a
# fully built 100 GB dev tree, which is one `nm` per image and no compilation.
action-client-arena-budget:
    #!/usr/bin/env bash
    set -euo pipefail
    _rc=0
    python3 scripts/check-action-client-arena-budget.py || _rc=$?
    if [ "$_rc" -eq 78 ]; then
        # 78 is `lane-skip.sh`'s "I did no work", reused so there is ONE
        # spelling of it. A gate that examined nothing must not read as a pass.
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip action-client-arena-budget \
            "no built image with an attributable nros_node_config.rs in this tree"
        exit 0
    fi
    exit "$_rc"

# Per-example rustfmt --check (AST-only, no codegen/deps). The `check.yml`
# per-example-fmt step as a recipe (SSoT).
#
# issue 0320 — this invokes `rustfmt` DIRECTLY, not `cargo fmt`. `cargo fmt`
# shells out to `cargo metadata`, which loads the leaf's `.cargo/config.toml`,
# which does `include = ["…/nros-patch.toml"]` — a file `nros sync` GENERATES
# and `.gitignore` excludes. A fresh checkout does not have it, so on CI every
# leaf died with:
#
#   error: could not load Cargo configuration
#     failed to load config include `../../../../../nros-patch.toml`
#
# That held `pr-checks` red on main for 60+ consecutive runs. Since phase-315
# the same call also needs the generated `nros-selection` facade crates,
# because a workspace member's `[dependencies]` path-dep must exist for
# `cargo metadata` to resolve at all.
#
# The deeper problem was placement: this gate lives in `check-fast`, whose
# contract is "BUILDLESS, SOURCE-FREE … no cargo tree/metadata". `cargo fmt`
# violated that. Formatting needs no dependency graph, so calling `rustfmt`
# per file honours the tier and cannot regress this way again.
[private]
example-fmt:
    #!/usr/bin/env bash
    set -e
    # Enumerate via the git index (tracked files only) — no filesystem
    # traversal, so the multi-million-file build/target trees (untracked by
    # definition) can never slow this down or leak in. `NF>=5` = the old
    # `-mindepth 4` (leaf crates, not workspace-root manifests).
    git ls-files 'examples/**/Cargo.toml' | awk -F/ 'NF>=5' \
        | grep -vE '/(target|generated|build|build-[^/]*|install|log)/|examples/zephyr/|/multi-package-workspace/|qemu-esp32-baremetal/rust/dds/|examples/qemu-arm-freertos/|examples/qemu-arm-nuttx/|examples/threadx-linux/|examples/qemu-riscv64-threadx/|examples/px4/' \
        | sort | while read -r toml; do
        dir="$(dirname "$toml")"
        # Read the edition from the manifest rather than assuming one: rustfmt
        # needs it explicitly (cargo fmt used to supply it), and the tree is
        # not uniform — the zephyr leaves are not 2024.
        edition="$(sed -n 's/^edition[[:space:]]*=[[:space:]]*"\([0-9]*\)".*/\1/p' "$toml" | head -1)"
        edition="${edition:-2024}"
        # Tracked .rs files only — the same index-driven discipline as above,
        # so generated/ and build trees cannot leak in.
        mapfile -t files < <(git ls-files "$dir/*.rs" "$dir/**/*.rs")
        [ "${#files[@]}" -eq 0 ] && continue
        echo "  fmt $dir (edition $edition, ${#files[@]} files)"
        rustfmt "+{{NIGHTLY}}" --check --edition "$edition" "${files[@]}"
    done

# issue 0718 — an example leaf builds in-tree (RFC-0026), so every build
# directory a recipe gives it must be named in that leaf's own .gitignore.
# The six threadx rust leaves ignored `build-cyclonedds` and not `build-zenoh`,
# and their c/cpp siblings ignored both, so no single leaf looked wrong.
[private]
example-leaf-build-dirs:
    @python3 scripts/check-example-leaf-build-dirs.py

# phase-340 P2 — nothing may write an `examples/**/target/` dir. That is the
# invariant item 7 / P4 stands on (`examples/**/target-*/` is globally ignored;
# a plain `target/` survives only through 391 per-leaf .gitignore files), and it
# was being broken by a SECOND build path with no manifest row — hence no
# coordinate, hence no shared cargo group. See the script's header for the
# measurement. Self-tests its own classifier on every run.
[private]
example-leaf-target-dirs:
    @python3 scripts/check-example-leaf-target-dirs.py

# Phase 118.I.5 — keep collapsed examples from regrowing a retired RMW
# directory axis without an explicit documented carve-out.
[private]
example-matrix:
    @bash scripts/check-example-matrix.sh

# Issue 1025 -- a packer must not invent the row's cargo args or env. The shared
# cargo group key is a function of (platform, args, env); a consumer supplying
# two of the three gets a directory the build never wrote to, and only once that
# row gains a variant. Silent until then, which is how every ESP32 QEMU flash
# image stopped being packable without a single lane going red.
[group("check")]
fixture-artifact-dir-inputs:
    @python3 scripts/check-fixture-artifact-dir-inputs.py

# issue 0720 — a test asks a fixture resolver for an artifact BY NAME, and a
# name no CMake target produces resolves to a missing path, which the test
# reports as a `fixture missing` SKIP. So the failure mode is silence, not red.
[private]
fixture-binary-names:
    @python3 scripts/check-fixture-binary-names.py

[private]
fixture-groups:
    @python3 scripts/check-fixture-groups.py
    @bash packages/testing/nros-tests/tests/fixture_group_collision_gate.sh

# Issue 0406 — a fixture builder narrowed to an id it cannot match must FAIL,
# not exit 0 having built nothing. Buildless: exercises the shared guard and
# one real builder invocation that stops before any compilation.
fixture-id-guard:
    @bash scripts/check-fixture-id-guard.sh

# Issue 0835 — a fixture staleness probe decides from the ARTIFACT's BYTES, not
# from whether the build did work. Corrosion's cargo step is an always-dirty
# edge and the phase-340 cargo groups evict rows nobody touched, so an
# activity-keyed probe reports work on EVERY run forever: it reported the same
# 17 cmake cells + 23 rust rows stale on a tree whose `lane=all` build had just
# gone green, which is what failed ~190 tests on every `just ci matrix`.
#
# Fixed in 2fa1ed09f with nothing guarding it. This runs the REAL probe scripts
# against a two-file cmake cell and a one-binary cargo leaf, and carries its own
# negative control: the pre-fix decision rules are re-applied to the same trees
# and must FIRE.
fixture-staleness-probes:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja cargo cc; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    # The probe asks the CLI for the platform's cargo profile, so the leaf
    # cannot be built without it — and a missing CLI would surface as every case
    # failing on `profile ... is not defined`, which reads as a probe defect.
    _cli="${NROS_CLI:-packages/cli/target/release/nros}"
    [ -x "$_cli" ] || _missing="$_missing nros(just setup-cli)"
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip fixture-staleness-probes "missing:$_missing"
        exit 0
    fi
    ./tests/fixture-staleness-probe-tests.sh

# A fixture stamp may not claim what the build did not achieve. The stamp used
# to record the lane's NOMINAL coordinates, so a module that skipped a missing
# prerequisite still read as covered and the run skipped its tests to a green
# sweep (phase-411 W1).
[private]
fixture-stamp-honesty:
    @python3 scripts/check-fixture-stamp-honesty.py

# The fixture manifest's own validators — every `[[workspace_fixture]]` and
# `[[compile_check_fixture]]` row must name files that EXIST and a target the
# CMakeLists/Cargo.toml actually defines.
#
# These validators shipped with no caller: `git grep validate-workspaces` found
# only the script's own usage text and dispatch. Unrun, they drifted until 74 of
# 86 workspace rows failed — the detector still looked for pre-RFC-0048 verbs and
# demanded a `[system].default_launch` that phase-296 retired. Both were checker
# staleness, not fixture breakage, and nothing could tell, because nothing ran.
# That is the issue-0309 silent-lane class: a gate nobody wires in decays into
# noise, then into a gate you cannot afford to turn on.
#
# Buildless and source-free (path existence + regex over tracked files), ~0.1s
# for all 112 rows, so it belongs in the per-push fast tier.
[private]
fixtures-manifest:
    @python3 scripts/build/fixtures-manifest.py validate-workspaces
    @python3 scripts/build/fixtures-manifest.py validate-compile-checks
    # Issue 0482 — plain `[[fixture]]` rows had NO validator at all, while the
    # other two kinds had one. A row missing `platform` or `lang` has no
    # coordinate, so it keeps building under `lane=all` and silently leaves
    # every coordinate-scoped lane; a row naming a typo'd `rmw` lands on a
    # coordinate no lane and no matrix cell can hold.
    @python3 scripts/build/fixtures-manifest.py validate-fixtures
    # phase-350 W0 (issue 0538) — `fixture-inventory.py`'s hand-authored list
    # asserts "this build is NOT in the manifest" for each of its rows, and had
    # NO consumer, so four of them went on asserting it after the build migrated
    # in — one of them a row phase-344 W2 added for exactly that reason. Same
    # shape as `examples_fixture_coverage.rs`'s stale-exception arm: an
    # exception that came true must fail, or the list rots into a false
    # negative read as authoritative precisely when someone audits coverage.
    @python3 scripts/build/fixture-inventory.py --check
    # phase-350 W1 (issue 0535) — the zephyr west leaves now have manifest rows,
    # but `zephyr-fixture-leaves.sh` still derives its own matrix, so the two are
    # briefly two spellings of one thing. This makes them unable to drift until
    # the emitter reads the rows. Read-only: the emitter runs no build tool.
    @python3 scripts/check-zephyr-fixture-rows.py

[private]
lane-scope-consumers:
    @python3 scripts/check-lane-scope-consumers.py

# phase-433 W4 / issue 1127 — `matrix_fixture_coverage.rs` G1 promises "a Runtime
# cell nothing runs ... fails here" and asserts that `tests/<test>.rs` EXISTS.
# This is the other half: every `Tier::Runtime` cell in `interop::CELLS` names a
# test binary some `just` recipe or workflow actually INVOKES. 8 of 17 did when
# it was written; the other 9 are `.config/interop-cells-without-runner.txt`,
# which only shrinks. Buildless — parses `interop.rs`, reads the justfiles and
# the workflows.
[private]
interop-cell-runners:
    @python3 scripts/check-interop-cell-runners.py

# phase-433 W5 / issue 1127 — the half that outlives W4's runner gate. A cell can
# be aimed at and still never have met a peer: `skip!` becomes `<skipped>`, every
# consumer counts that as not-a-failure, and a cell that has skipped on every host
# since the day it was written is indistinguishable from one that passes. So what
# a live peer has ANSWERED is written down (`.config/interop-verdicts.toml`) and
# an absent entry means NEVER RUN — the default, so nothing here is satisfiable by
# a cell that never runs. This gate checks the ledger's integrity only, which is
# why it is green on a host with no ROS: real cell, required fields, cases that
# still exist, an OPEN issue behind every `fail`, and no BINDING test cited as
# evidence (`cases_bound_to_interop_cells` passes with no peer, forever — citing
# one would put the defect inside the fix). Report: `just interop-verdicts`.
[private]
interop-verdict-ledger:
    @python3 scripts/check-interop-verdicts.py

# Issue 0743 fallout — a `binary()` in .config/nextest.toml naming a deleted test
# target makes nextest refuse to PARSE the config, which kills every nextest run
# in the repo, not just that lane. It went unnoticed behind a green `just check`
# because `just check` does not run nextest; this gate is that missing coverage.
# `test()` names are deliberately NOT checked — they are rstest-generated case
# names that appear nowhere in the sources (see the script's header).
[group("ci")]
nextest-binary-filters:
    @python3 scripts/check-nextest-binary-filters.py --self-test
    @python3 scripts/check-nextest-binary-filters.py

# phase-373 W1 — every `test()`/`binary()` predicate in .config/nextest.toml must
# match at least one real test. The `binary()` half is already covered statically
# by `check-nextest-binary-filters` on the fast line; this is the `test()` half,
# which that gate cannot do because rstest case names appear nowhere in the
# sources. It needs the test binaries, so it runs from `test-all` rather than
# `just check`, which builds none.
[private]
nextest-test-filters:
    @python3 scripts/check-nextest-test-filters.py

[private]
no-tracked-models:
    @bash scripts/check-no-tracked-models.sh

# phase-383 W10.c — an example workspace ROOT is generated, never tracked.
#
# `nros build` emits it from the discovered packages plus `[image.*]` (RFC-0065
# D3); a tracked one is used AS-IS instead, so it silently overrides the
# declarations it came from. The class is "someone re-adds a hand-written root",
# and it already happened once during the migration that removed them — a
# `git add -A` re-added three, which `.gitignore` cannot prevent because it does
# not apply to files git already tracks.
[private]
no-tracked-workspace-roots:
    @python3 scripts/check-no-tracked-workspace-roots.py

# Issue 0652 — a `required-features` target no recipe enables is invisible:
# cargo does not report it as filtered, it simply never builds it, so it looks
# like coverage while running nothing. Baseline is a shrinking backlog.
[group("main")]
required-features-reachable:
    @python3 scripts/check-required-features-reachable.py

# A test that can SKIP must be run by a runner that understands skips.
# `nros_tests::skip!` PANICS with a `[SKIPPED]` marker (Rust's harness has no
# native skip), and only `_nextest-tolerant` rewrites it before tallying — so a
# bare runner turns an unmet precondition into a red naming a test the change
# never touched.
[private]
skippable-tests-tolerant:
    @python3 scripts/check-skippable-tests-tolerant.py

# issue 0445 / 0442 — every freshness probe shares ONE exemption rule and ONE
# verdict. A staleness verdict is absorbing: the fixture never launches, so the
# runtime result it would have produced is replaced by a self-explaining
# message. Issue 0444 hid behind 0442 (an exemption applied on one probe arm
# and not its sibling) for exactly as long as those cells read STALE.
[private]
staleness-probe-exemptions:
    @bash scripts/check-staleness-probe-exemptions.sh

# Issue 0580 — a test must ASSIGN its ROS domain, never name one: a literal is a
# shared bus, and two concurrent runs colliding on it presents as WRONG DATA
# rather than as a collision.
test-domain-assignment:
    @bash scripts/check-test-domain-assignment.sh

# Issue 0991 -- the two inventories above are written LATER in a configure than
# they are READ, and the recovery three call sites described in comments
# ("CMAKE_CONFIGURE_DEPENDS, so ninja re-runs cmake by itself") was measured to
# never fire: `build.ninja` is written after the fragment, so the dependency is
# never stale. A memory-tight image therefore sized itself from the "nothing
# composed yet" placeholder on every clean build dir and failed at LINK naming
# 103160 bytes and no knob.
#
# This gates the mechanism that replaces it. It is a REAL configure plus a real
# `ninja`, not `cmake -P`, because the claim is about what ninja does with an
# mtime -- and the thing being fixed is a mechanism that read as working in
# every review while firing never. The control case reproduces that bug, so the
# gate would notice a fix that quietly stopped working.
#
# cmake + ninja only: `project(... NONE)`, no compiler, no nano-ros build, no
# codegen, ~2 s. Fast line, beside the two readers it protects.
# Issue 0870's neighbour, found while restoring `declared-qos-header` above: a
# test script with NO CALLER is a test that runs nowhere, and nothing detected
# it. `e569d9a55` -- a commit about which Zephyr SDK the CI image bakes --
# deleted that recipe and its lane entry, its message mentioning neither, and
# `tests/cmake-declared-qos-header-tests.sh` sat tracked and unreachable for a
# day while guarding phase-403 step 2's delivery half.
#
# The repo's recurring class one level up: a mechanism that is correct and
# unreachable. Every previous instance (0896, 0963, the flag-only selftest, the
# NuttX printk arm, `just xrce setup`'s short-circuit) was found by a human
# noticing, which is not a control.
#
# Pure `git ls-files` + line scan, no build, ~0.2 s -- fast line.
test-scripts-have-callers:
    python3 scripts/check-test-scripts-have-callers.py

# Issue 1137 — if the PEER's bus is pinned, OUR side's must be too.
#
# Issue 1009 confined every DDS interop peer to loopback, and measured that
# pinning ONE side is worse than pinning neither (0 of 15, empty output). It was
# then half-applied: `ros2_env_setup_rmw_with_domain` builds a shell string, so
# the pin reached the `ros2` peer and never the bare `std::process::Command` our
# own binaries are spawned from. Two days later
# `cyclone_enumerates_a_stock_ros2_node` — which had PASSED live on 2026-08-30
# (issue 0927) — enumerated exactly one node, itself, against a stock talker
# that was up, because the talker sat on `127.0.0.1` with multicast off while
# our probe kept Cyclone's default interface. That symptom is
# indistinguishable from a broken `ros_discovery_info` reader, and was filed as
# one twice.
#
# Keys on the HOST peer helpers, never on the `Middleware` enum: `DockerRosEnv`
# names the same variants and its pairs are symmetric-UNPINNED (a container
# cannot read a host profile path), so pinning our half of one would create the
# bug rather than fix it.
#
# Pure `git ls-files` + line scan, no build, no ROS — fast line.
dds-isolation-symmetry:
    python3 scripts/check-dds-isolation-symmetry.py
