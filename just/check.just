# The gate tiers and the 198 gates they run — `just check <name>`.
#
# `mod`, not `import`. Phase-399 moved these 200 recipes out of the root
# justfile and deliberately kept them FLAT, on the argument that nobody types a
# gate name so a namespace buys nothing while renaming every call site. That
# was right about the typing and wrong about the listing: 62 of the gates were
# public, so `just --list` still opened with a wall of `check-*`.
#
#   just check                the full tier (default recipe below)
#   just check fast           138 source gates, parallel            ~9 s
#   just check fast-serial    the same set, one at a time
#   just check build          the compile tier
#   just check <gate>         one gate, by the name it prints when it fails
#
# ONE spelling, no forwarders: the ~330 references across docs, scripts,
# packages and workflows were rewritten with the rename rather than aliased,
# so what a gate is called is what the tool prints, what the docs say, and what
# you type — with no second name that drifts.
#
# `default` must be the FIRST recipe: `just <module>` runs a module's first
# recipe, not the one named `default`. Getting that wrong made `just ci`
# silently run a weaker tier during the sibling reorg.

set working-directory := '..'

# Both re-derived here rather than inherited: a `mod` does not see the root's
# variables, and every sibling module (native, esp32, workspace) already does
# exactly this. NIGHTLY reads the PINNED toolchain file rather than copying a
# version string, so the four copies cannot disagree about which nightly —
# they all answer from `tools/rust-toolchain.toml`.
NIGHTLY := `awk '/^channel/ {gsub(/"/, "", $3); print $3; exit}' tools/rust-toolchain.toml`

# Workspace members that cannot be checked on the host: staticlib/cdylib crates
# whose link step needs a target toolchain. Used only by the gates, so it lives
# here now — the root had no other user.
HOST_UNCHECKABLE := "--exclude nros-c --exclude nros-cpp --exclude nros-rmw-zenoh-staticlib --exclude nros-rmw-xrce-cffi --exclude nros-rmw-xrce-cffi-staticlib --exclude nros-build-helpers --exclude nros-zpico-build --exclude nros-build-paths"

# Check everything: Rust (native + embedded + features + examples), C, C++, Python
# `decoupling` is intentionally NOT in this gate: it guards the Phase-104.A
# "no concrete backend/platform refs in nros/nros-node" goal, which RFC-0031
# (Stable) superseded — the `?/` forwarding + optional backend deps were
# deliberately restored (Phase 214.S / 227.3) as the unified RMW-selection model.
# The recipe stays runnable (`just decoupling`) for anyone revisiting the
# bridge-decoupling track, but it must not fail the green `default` gate.
# Full static gate = the fast (buildless) tier + the build tier. `just default`
# runs both (local default + the PR/nightly CI lane). The per-push CI lane runs
# only `fast` so it completes under a rapid push cadence (the build tier's
# workspace/example clippy + nros-tests/staticlib compiles are minutes; cancelled
# repeatedly otherwise). See docs/development/ci-workflow-reorg.md.
# `cli-fresh` FIRST, ahead of both lanes — issue 0363's script says its
# whole contribution is POSITION ("a stale CLI used to surface … minutes into
# `just default`; here it is the first thing that runs"), and that was not true
# for a direct `just default`. It was first within `build`, but `fast`
# runs earlier and contains recipes that EXEC the CLI, so the in-binary guard
# fired there instead and the dedicated probe never got its turn. Measured
# 2026-08-17: the stale-CLI error landed at line 83 of a 96-line run, after 13
# gates had passed, where the probe itself costs 0.21 s.
#
# Listed here as well as in `build`: just runs a dependency once per
# invocation, so this only moves it earlier — it does not run twice. `just ci`
# also probes via `tier-preconditions`; that duplicate is 0.21 s and buys
# the property that ANY future lane gaining a CLI-using recipe stays covered.
[group("main")]
default: cli-fresh fast build api-parity
    #!/usr/bin/env bash
    set -e
    # issue 0650 — same reason as `fast`'s closing line: "All checks
    # passed!" must not stand for a gate that never ran. The ledger is shared,
    # so this reports the fast tier's skips plus any from the build tier.
    # shellcheck source=scripts/build/check-skip.sh
    source scripts/build/check-skip.sh
    nros_check_skip_report "All checks passed!"

# The source-gate tier — FANNED OUT since phase-395 W11 (issue 0726 closed).
#
# 90 s serial -> ~8.3 s at -P24, and it runs on every pull request, every merge
# group and every push, so this is the most-paid-for second in the tree. The 133
# gates are 56 s of work spread over 90 s at 1-2 runnable cores; the floor is
# now one gate (`rmw-ret-sign`, ~8.5 s).
#
# This was opt-in for two phases because one gate went red under fan-out and
# green standalone. The cause was a `grep -q` that could not tell a forked
# grep's EAGAIN from "no match" — 76 such sites are now converted to
# `nros_grep_q` across every `fast`-reachable script, and
# `grep-q-error-conflation` keeps new ones out. 17 fan-out runs green,
# including three at `NROS_GATE_JOBS=64` for deliberate fork pressure.
#
# `just fast-serial` is the escape hatch: fail-fast ordering, one gate's
# output at a time.
[group("main")]
fast:
    @bash scripts/build/run-gates-parallel.sh

# The terminator both gate registries end on — a no-op that exists only to give
# the LAST real gate a trailing `\`.
#
# Without it the final entry is the one line in the list with different syntax,
# so every gate that sorts last must edit SOMEONE ELSE's line to add the
# backslash as well as adding its own. Two such pull requests then conflict with
# certainty rather than by bad luck — the "trailing comma" problem, and the one
# residual conflict class the one-name-per-line format could not reach.
#
# `_`-prefixed on purpose: `run-gates-parallel.sh` filters derived names with
# `^[a-z][a-z0-9-]*$`, so the sentinel is excluded from the fan-out without that
# script needing to know it exists. `check-gate-lists` DOES know — it requires
# the sentinel to be last and excludes it from the sort, which is what keeps
# this from being a name someone quietly moves.
[private]
_gate-list-end:

# The GATE LIST, and the serial runner — phase-395 W11.
#
# `fast` (below) fans these out; this recipe both DECLARES them and runs
# them one at a time. Keep it: `run-gates-parallel.sh` derives its list from
# this dependency line, so the list has exactly one home, and a serial run is
# the right thing when you want fail-fast ordering or unshredded output.
[group("main")]
fast-serial: \
    abi-bindings \
    absolute-paths \
    action-client-arena-budget \
    activate-shells \
    api-parity-ledger \
    artifact-identity-budget \
    atomic-sync-writes \
    board-abi-mirror \
    board-cargo-config-applied \
    board-facts-delivery \
    board-manifest-drift \
    board-tiers \
    board-vocabulary \
    book-links \
    book-no-just \
    boot-report-layout \
    box-sync-covers-tracked-source \
    build-profile-literals \
    build-root \
    build-rs-rerun-paths \
    build-type-spelling \
    c-fmt \
    capability-flavour-guards \
    capability-slot-counts \
    cargo-config-tracked \
    cargo-custom-command-depfile \
    cargo-locked \
    cargo-profile-mirror \
    cargo-target-spelling \
    cbindgen-headers \
    cbindgen-pin \
    cc-build-policy \
    ci-doc-workflow-refs \
    ci-image-python-deps \
    ci-no-fixture-tolerance \
    ci-no-verb-fallback \
    claim-protocol \
    cli-fmt \
    cli-source-dirs \
    cmake-find-program-shadowed \
    cmake-image-policy \
    cmake-support-library \
    codegen-invocation \
    codegen-size-bound-golden \
    codegen-stamp-inputs \
    codegen-tool-reconfigure \
    codegen-version-refusal \
    codegen-version-surface \
    config-header-single-writer \
    config-knob-census \
    config-surface \
    core-only-predicate \
    cpp-ffi-error-mapping \
    cpp-fmt \
    cpp-freestanding-includes \
    cpp-no-std-stdio \
    cyclone-backend-sources \
    cyclone-domain-not-pinned \
    declared-qos-header \
    default-gates-run-somewhere \
    deploy-board-resolves \
    derived-descriptor-fields \
    doc-recipe-refs \
    doc-refs \
    emitter-just-spelling \
    entity-inventory-knobs \
    entity-slot-costs \
    entry-locator-ssot \
    entry-session-name \
    example-fmt \
    example-leaf-build-dirs \
    example-leaf-target-dirs \
    example-matrix \
    example-platform-not-shadowed \
    export-f-closure \
    eyre-context-alias \
    feature-contract \
    feature-set-ssot \
    ffi-struct-mirrors \
    fixture-artifact-dir-inputs \
    fixture-binary-names \
    fixture-groups \
    fixture-id-guard \
    fixture-staleness-probes \
    fixture-stamp-honesty \
    fixtures-manifest \
    flake-quarantine \
    flavour-lanes \
    format-macro-scope \
    gate-cache-keys-agree \
    gate-lists \
    gate-selftests \
    gate-visibility \
    generated-leaf-regenerable \
    generated-schema-coverage \
    goal-cdr-stripped \
    grep-q-error-conflation \
    host-platform-vocabulary \
    host-triple-literals \
    image-panic-policy \
    image-paths-apply-policy \
    infra-queryable-counts \
    interface-glob-configure-depends \
    interlock-visibility \
    issue-ids \
    issue-index \
    just-recipe-refs \
    kconfig-knob-forwarding \
    kconfig-overridden-values \
    knob-delivery \
    knob-fixpoint \
    knob-resolved-once \
    knob-single-reader \
    lane-contracts \
    lane-scope-consumers \
    lane-skip-class \
    lane-skip-protocol \
    leaf-lockfiles \
    ledger-doc-single-home \
    literal-domain-id \
    manifests-parse \
    message-bound-knobs \
    mirror-header-precedence \
    msg-dep-is-path \
    named-lane-fails \
    nested-workspace-excludes \
    nextest-binary-filters \
    no-absolute-model-paths \
    no-allow-multiple-def \
    no-board-init \
    no-direct-kernel-alloc \
    no-silent-sample-drop \
    no-std-stdio \
    no-tracked-file-find \
    no-tracked-models \
    no-tracked-workspace-roots \
    no-vacuous-tests \
    nuttx-integration-makefile \
    nuttx-libc-struct-sizes \
    nuttx-links-snapshot \
    nuttx-shared-tree-headers \
    opaque-storage-guards \
    orphan-generated-stamp \
    package-xml-comments \
    package-xml-uses \
    path-env-fingerprints \
    peer-sweep-lanes \
    platform-abi-mirror \
    platform-provider-features \
    pool-inventory \
    posix-platform-purity \
    preconditions-provisioned \
    prelude-tiers \
    prereq-roles \
    profile-board-mirror \
    provider-announcements \
    provider-index \
    ps-zombie-blind \
    px4-archive-header-pairing \
    python \
    readiness-marker-literals \
    reconfigure-on-change \
    reconfigure-stale \
    required-contexts-reportable \
    required-features-reachable \
    retired-platform-clock-symbols \
    retired-submodule-refs \
    rmw-abi-shape \
    rmw-alloc-sites \
    rmw-api-comparison \
    rmw-api-parity \
    rmw-descriptors \
    rmw-force-link-anchor \
    rmw-required-slots \
    rmw-ret-sign \
    rmw-slot-producers \
    rmw-slot-table \
    rmw-vtable-order \
    roadmap-boxes \
    roadmap-claims \
    roadmap-status \
    ros-env-spelling \
    runner-doctor-sdk-resolution \
    runner-sweep-eviction-order \
    rust-stdio-on-zephyr \
    rust-targets-covered \
    scope-namespace \
    sdk-guard-can-fire \
    sdk-store-not-enumerated \
    serdes-descriptors \
    shared-cargo-dir-witness \
    single-rust-staticlib \
    site-config \
    sizes-header-mirrors \
    skip-marker-matching \
    skippable-tests-tolerant \
    source-manifest \
    staleness-probe-exemptions \
    std-census \
    string-conventions \
    submodule-pinned-locks \
    submodule-pins \
    subtree-guard \
    sysdep-remedies \
    test-domain-assignment \
    test-scripts-have-callers \
    tests-can-fail \
    tier-has-ci-owner \
    tier-priority-plan \
    tier-spin-gap \
    vendor-fetch-pinned \
    version-lockstep \
    wait-evidence-discarded \
    weak-symbols \
    workflow-indexed-apt \
    workflow-repo-env \
    workflow-runner-isolation \
    workflow-setup-spelling \
    workspace-build-output \
    workspace-fmt \
    workspace-order \
    xrce-source-manifest \
    zenoh-config-template \
    zenoh-platform-macros \
    zenohd-flag-invocations \
    zenohd-resolution-parity \
    zenohd-router-skips \
    zenohd-spawn-sites \
    zephyr-knob-agreement \
    zpico-config-keys \
    _gate-list-end
    # `_check-skip-reset` is shared skip-ledger infrastructure at the ROOT
    # (`run-gates-parallel.sh` calls it too), not a gate — and a module cannot
    # depend on a root recipe, so it is called rather than depended on. It must
    # still run FIRST: it truncates the ledger every gate below appends to.
    @just _check-skip-reset
    #!/usr/bin/env bash
    set -e
    # issue 0650 — the closing sentence is REPORTED, not asserted. Six gates in
    # this lane skip on a missing optional tool (bindgen, ROS 2, colcon, the
    # in-tree CLI), which they must: this tier is documented to run green on a
    # pristine worktree. What they may not do is let "Fast checks passed!" stand
    # for gates that never ran.
    # shellcheck source=scripts/build/check-skip.sh
    source scripts/build/check-skip.sh
    nros_check_skip_report "Fast checks passed!"

# Build tier — gates that COMPILE or need the workspace to RESOLVE (workspace +
# embedded clippy, feature combos, riscv32 no_std, nros-tests source gates,
# staticlib link-proof, dep-chain codegen, the example-matrix clippy, and the
# embedded feature-unification `cargo tree` — which needs every `-sys` source
# submodule present to resolve). Minutes + source/CLI prereqs.
#
# WHERE THIS ACTUALLY RUNS, and why the line above used to lie about it.
#
# This comment read "runs on PR + nightly (`default.yml` non-push)" for long
# enough to mislead. It does NOT run on a pull request and it does not run on a
# merge group: `gate.yml`'s only invocation is guarded
# `schedule`/`workflow_dispatch`, because the lane needs generated message
# bindings and prebuilt `.compile-ok` artifacts that no PR job produces
# (phase-396 — it was on the merge group once and could never pass there).
#
# So NOTHING in this list gates a merge, and a red here is invisible until
# somebody reads a nightly. Issue 0981 is what that costs: `codegen_golden` sat
# red on `main` for a day while the required `CI` context stayed green, and two
# separate changes reached a green pull request while failing `just ci gate`
# locally.
#
# The fix is not to make this lane cheaper — it is to keep only the gates that
# genuinely NEED it. Issue 0993 moved 20 gates to the fast line after measuring
# each in a PRISTINE worktree: all 20 pass with no build artifacts at all, and
# cost ~1.4 s of wall clock at -P32. A gate belongs here only if it cannot run
# without something built, which is a property you can TEST rather than assume —
# `borrowed-e2e` stayed because it compiles `nros-c` and reads the generated
# config header, and the pristine run is what proved it.
[group("main")]
build:
    @NROS_GATE_LANE=build-serial bash scripts/build/run-gates-parallel.sh

# The BUILD gate list, and the serial runner — mirrors `fast-serial` above.
# `run-gates-parallel.sh` derives its list from this dependency line, so the
# list has exactly one home.
[private]
build-serial: \
    borrowed-e2e \
    c \
    cli-fresh \
    cli-tests \
    cpp \
    embedded-feature-unification \
    launch-resolve-builds \
    mem-report \
    no-alloc-image \
    node-std-tests \
    nros-log-riscv32 \
    required-features-tests \
    rmw-cyclonedds \
    rmw-uorb \
    rmw-xrce \
    sched-dim-arms \
    source-gates \
    staticlib-symbols \
    test-targets \
    workspace-all \
    workspace-features \
    _gate-list-end
    # `native::check`, which was a DEPENDENCY at the root. A module cannot name
    # another module's recipe — from in here `native::` would mean a submodule
    # of `check` — so it becomes an explicit call. Same constraint the sibling
    # `ci` module hit.
    @just native check
    @echo "Build checks passed!"

# nano-ros gate recipes — phase-399 W1.
#
# The 200 `check-*` recipes that `check-fast-serial` / `check-build` fan out
# over. Brought in with `import`, NOT `mod`, so every name stays exactly what it
# was: these are load-bearing in five workflow files, in
# `scripts/build/run-gates-parallel.sh`'s awk over `check-fast-serial`'s
# dependency list, in CLAUDE.md and AGENTS.md, and in every issue that quotes a
# command. A nicer spelling (`just check fast`) is not worth a tree-wide edit
# plus a long tail of stale documentation.
#
# WHAT IS NOT HERE, deliberately: `check`, `check-fast`, `check-fast-serial` and
# `check-build` stay in the root justfile, because their DEPENDENCY LISTS are the
# contract — `run-gates-parallel.sh` reads `check-fast-serial`'s out of the root
# file to build the fan-out set, and moving it would silently empty the parallel
# runner rather than fail.
#
# These are not an interface. Nobody types `check-cmake-find-program-shadowed`;
# it runs as a unit of `check-fast` and is discovered by FAILING, which prints
# its own name. That is why 200 of them living in one file is fine and why they
# do not belong in `just --list`'s default view.

# Fast tier — BUILDLESS, SOURCE-FREE gates only (fmt/clang-format AST checks,
# ABI/board mirrors, manifest + convention scripts). No cargo build/clippy/test
# AND no `cargo tree`/metadata (which would need the workspace — i.e. every `-sys`
# source submodule — to resolve). So it needs neither the nros CLI nor any
# provisioned source, finishes in ~1 min, and survives the per-push cadence. This
# is the per-push CI gate (`gate.yml`).
#
# That description is now TRUE, and was not (issue 0466): `check-cli-fresh` and
# `check-test-targets` both lived here and both needed what the paragraph says
# this tier does without. Verified rather than asserted — a pristine detached
# worktree with no CLI, no sources and no `nros sync` runs this lane green in
# 23s. If you add a gate here, check it against that, not against your own
# provisioned tree, where everything passes for the wrong reason.
# `check-fast`'s gates, fanned out across the machine (issue 0726).
#
# Same gate set, derived from check-fast's own dependency line so it cannot
# drift. 111 gates, 90s serial -> 8s at -P32 on a 32-core host; the floor is the
# slowest single gate (~7.7s), so there is little left to win by optimising
# individual gates — the distribution has no outlier (mean 501ms).
#
# Reports EVERY failing gate rather than stopping at the first, which serial
# just-dependency ordering cannot do. That is the same trade `check-tier-
# preconditions` makes: one failure per attempt is what makes a check something
# people stop running.
# Kept as an alias for anyone with it in muscle memory; `check-fast` IS this now.
[private]
fast-parallel:
    @bash scripts/build/run-gates-parallel.sh

# Root-workspace rustfmt. `check-example-fmt` and `check-cli-fmt` already sit in
# the fast tier; the ROOT workspace's `fmt --check` was the one left in the build
# tier, so an unformatted file you just wrote survived `check-fast` — which is
# how `model_location.rs` (phase-330 W3.b) shipped needing a reflow.
#
# Buildless and seconds-long: rustfmt parses, it does not compile.
#
# NIGHTLY, always: `rustfmt.toml` enables nightly-only options and stable
# produces different output (CLAUDE.md).
[private]
workspace-fmt:
    cargo +{{NIGHTLY}} fmt --check

# Compile the TEST targets. Buildless gates never touch `#[cfg(test)]` code, and
# neither does `check-workspace`'s clippy (no `--all-targets`), so a field added
# to a shared struct can leave every test initializer broken while `check-fast`
# stays green. That happened twice in one session — `TierRtosSpec` gained four
# fields (phase-330 W1.a) and `ComponentConfig` gained `class` (issue 0392 B) —
# and both landed green with seven initializers unbuildable.
#
# `cargo check`, not `cargo test`: this is about COMPILING the test targets;
# running them belongs to `test-all`. Warm cost ~13s (root) + ~19s (CLI); the
# first run on a cold target dir is minutes, like any other compile here.
#
# Issue 0466 — this is a check-BUILD gate, and used to sit in `check-fast`.
# Compiling the workspace needs the `-sys` SOURCE submodules, which the push
# lane deliberately does not provision (gate.yml gates that on
# `event_name != 'push'`), so on every push it died:
#
#     error: failed to run custom build command for `zpico-sys`
#
# `just` stops at the first failed dependency, and 25 gates sat behind this one
# — every fmt gate, the FFI/sizes mirrors, doc-refs, string conventions. So its
# real per-push coverage was ZERO while it masked all of them. Four source-level
# reds reached main during exactly that window.
#
# Moving it costs no coverage anyone actually runs: `just check` is
# `check-fast` + `check-build`, so a local `just check` / `just ci` runs the
# identical set, and CI still runs it on PR + nightly. What changes is that the
# push lane can now finish — measured on a pristine, source-free, CLI-free
# checkout: `just check fast` green in 23s, two graceful skips, no failures.
#
# The reason it was written (a struct field breaking every test initializer
# while buildless gates stay green — twice in one session) is untouched: the
# gate still runs, in the tier that can actually compile.
[private]
test-targets:
    #!/usr/bin/env bash
    set -euo pipefail
    # clippy, not `cargo check`: it COMPILES the same targets, so linting here
    # costs nothing extra over the compile this gate already needed, and it
    # closes the second half of the same blind spot — a lint in code you just
    # wrote could only be seen by the build tier. `-D warnings` on both, which
    # the host clippy did not previously enforce (the embedded one always did).
    cargo clippy --quiet --workspace --all-targets --no-default-features \
        {{HOST_UNCHECKABLE}} -- -D warnings
    # The CLI is its own workspace (own manifest + lock) and its exclusions have
    # a single home in `check-cli-clippy` — call it rather than restate them.
    just check cli-clippy
    echo "clippy + test targets clean (root + cli)."

# issue 0560 reason 2 — `setup-launch-resolve` was a dependency of
# `build-test-fixtures` and NOTHING else, so a compile regression in the resolver
# waited for whoever next ran the ~40-minute fixture lane rather than failing its
# author. #560 gated the LOCK drift (sub-second, in `check-fast`); this covers
# the compile, which needs a build tier.
#
# It invokes the REAL recipe rather than a second `cargo check` spelling: same
# profile, same flags, and it catches link errors a `cargo check` would miss —
# ~14 s warm against 6 s, which is the cheaper half of the trade. The artifact it
# leaves is the one `nros sync` wants anyway.
#
# SKIPS when the submodule is absent, so `just check` still runs on a bare clone.
# That is deliberately NOT what `setup-launch-resolve` itself does: issue 0409
# made that recipe FAIL there, because its job is to produce the binary and
# exiting 0 without one let `nros sync` run on a stale resolver. A verification
# lane answers a different question — "can this be checked?" — and the honest
# answer without the submodule is no.
[private]
launch-resolve-builds:
    #!/usr/bin/env bash
    set -euo pipefail
    resolve="packages/cli/third-party/play_launch/src/ros-launch-resolve/resolve/Cargo.toml"
    if [ ! -f "$resolve" ]; then
        echo "[check-launch-resolve-builds] SKIP — play_launch submodule not initialised"
        echo "    git submodule update --init packages/cli/third-party/play_launch"
        exit 0
    fi
    just setup-launch-resolve

# issue #202 — run the CLI sub-workspace's test suite (unit tests across
# nros-cli-core / rosidl-* / nros-build + the plan-pipeline e2e). Before this
# lane existed NOTHING ran `cargo test` on packages/cli — the orchestration
# e2e suite sat 17/17 red for months without any lane noticing (the #181
# silent-lane class). The metadata-mode tests compile tiny probe crates at
# runtime BY DESIGN (the verb under test is a compile-driver; see
# nros-cli-core/tests/plan_pipeline_e2e.rs).
# Issue 0981 — the two codegen tests that pin the derived size bound, lifted out
# of `cli-tests` and onto the FAST line so they gate a merge.
#
# `cli-tests` below runs the whole `packages/cli` workspace, including a
# plan-pipeline e2e that compiles probe crates at runtime; it costs minutes and
# needs the play_launch submodule, so it lives on `check::build` — which is
# `schedule`/`workflow_dispatch` only in CI. No pull request and no merge group
# runs it. That is how `codegen_golden` sat red on `main` for a day: the
# required `CI` context is `check-fast` + `test-unit`, and both were green on
# the commit that broke it.
#
# These two need none of that. They are pure functions of the corpus and the
# emitters — measured 8.1 s from a COLD target dir, 0.1 s warm, inside the fast
# line's existing spread. So the narrow thing moves and the expensive suite
# stays put; promoting `cli-tests` wholesale would put a runtime-compiling e2e
# on the merge-gating lane, which `check-lane-contracts` exists to forbid.
[private]
codegen-size-bound-golden:
    @cargo test --manifest-path packages/cli/Cargo.toml -p rosidl-codegen \
        --test codegen_golden --test message_size_bound_parity --quiet
    @echo "codegen size-bound golden + C/C++/Rust parity: OK"

# RFC-0090 / phase-429 W5 — the codegen-version check must FIRE.
#
# `check-fast` proves the tree is green. Only a negative control proves the
# mechanism is armed, and this one guards the failure that stopped nano-ros
# shipping a prebuilt `nros`: a binary emitting code the runtime does not
# accept, which COMPILES and is wrong.
#
# Three arms, from the REAL emitted artifact rather than a hand-written
# imitation — the block is extracted verbatim from a committed golden header, so
# a template edit that dropped the check takes these with it. In range compiles;
# out of range is refused in both C and C++; a config header that defines
# neither bound is refused as MISSING rather than as out-of-range, because an
# undefined macro reads as 0 to `#if` and would otherwise produce a range error
# about a version nobody set.
#
# The Rust half is NOT covered here — see the script's header for why, and for
# the by-hand verification that stands in for it.
[private]
codegen-version-refusal:
    @./tests/codegen-version-refusal-tests.sh

# phase-429 W3 (RFC-0090 D1/D5) — `NROS_CODEGEN_VERSION` is an AUTHORED integer,
# and this tree's authored mirrors drift: `check-rmw-api-parity`'s map read green
# while 25 symbols disagreed with the vtable. Drift here is quieter and worse —
# every generated tree on disk keeps stamping the old number, so a runtime that
# can no longer run them accepts them all.
#
# So the surface generated code names is EXTRACTED (from the codegen packs AND
# the emitter's own format strings) and stamped beside the integer. The hash is
# evidence; the token stays an integer, because a reader must be able to order
# two of them. Buildless — pure source analysis, ~0.3 s.
[private]
codegen-version-surface:
    @python3 scripts/check-codegen-version-surface.py

# Issue 0984 — `libnros_rmw_cyclonedds.a` is built from TWO authored source
# lists (the cmake target, and `let cpp_files` in the sys crate's build.rs), and
# #0970 added `nros_sertype.cpp` to only one. The cmake path kept linking, so
# nothing looked wrong; every RUST fixture that reached the link stage died on
# an undefined `create_nros_sertype` referenced from inside the archive.
#
# `build.rs` already carried a comment warning about this exact drift. A comment
# is not a check.
# Issue 0985 — the per-build sizes header must have ONE writer, the mirror
# script, which since issue 0978 knows that the leaf's own copy is present and
# arbitrarily old once leaves share a cargo target dir.
#
# A second writer does not merely duplicate that precedence, it DEFEATS it:
# `nros-cpp/CMakeLists.txt` healed the mirror at configure time with a plain
# `file(COPY_FILE)` from the leaf's binary dir, wrote a museum header over a
# correct one, and stamped it with a new mtime — so ninja found the mirror's
# OUTPUT newer than its trigger and skipped the command that would have fixed
# it. A repair for drift that caused drift and suppressed its own fix.

# phase-412 -- the boot self-report is decoded POSITIONALLY out of a memory
# dump, so a field reordered on one side and not the other does not fail: it
# decodes, and prints an arena size that is really a callback count.
[private]
boot-report-layout:
    @python3 scripts/check-boot-report-layout.py

# Static, no build. The whole 0088 -> 0114 -> 0122 -> 0123 -> 0245 -> 0268 ->
# 0978 -> 0985 family is one shape: two ways to answer "which bytes are the
# current sizes header".
[private]
config-header-single-writer:
    @python3 scripts/check-config-header-single-writer.py

[private]
cyclone-backend-sources:
    @python3 scripts/check-cyclone-backend-sources.py

[private]
cli-tests:
    #!/usr/bin/env bash
    set -e
    cargo test --manifest-path packages/cli/Cargo.toml --workspace --quiet
    echo "CLI tests passed!"

# nros-node's `std`-gated unit tests. `just test-all` runs `cargo nextest
# --workspace`, which builds each crate with its OWN default features — and
# `std` is not one of nros-node's — so seven `#[cfg(feature = "std")]` tests in
# `executor/tests.rs` existed in no lane at all. Confirmed against a full tier-1
# sweep: 152 `nros-node` cases ran and not one of those seven was among them.
#
# A test nothing runs is worse than no test, and this is what that costs:
# `violations_beyond_the_ring_are_counted` (issue 0514's overflow-counting
# proof) asked for `MAX_VIOLATIONS + 4` callback slots against a `MAX_CBS` whose
# default has been 4 since 2026-03, so it panicked `ExecutorFull` on the fifth
# `register_timer`. It had NEVER passed — it landed broken 2026-08-11 and no
# lane noticed for four days.
#
# Same shape as issue 0319 (a backend suite with no lane, red on main for two
# days) and the issue-0196 rule generally: a gate whose coverage is narrower
# than the thing it is supposed to enforce.
#
# issue 0687 added the second entry: `nros`'s `env` module — the tree's one
# reader of the process environment — is behind a non-default `env` feature for
# the same reason, so its 11 tests were in no lane the hour they were written.
# Same recipe, because it is the same defect class, not a new one.
node-std-tests:
    #!/usr/bin/env bash
    set -e
    # phase-425 W3 added `sim-time` to the same list for the same reason: the
    # `/clock` time source is behind a non-default feature, so `--workspace`
    # builds it with `sim-time` off and its tests are in no lane at all. Note
    # `has_rmw` is false here and the module is `cfg(any(has_rmw, test))`, so it
    # compiles under `cargo test` exactly as its unit tests need.
    # `param-services` joined for W3b: `use_sim_time` is a PARAMETER, so its two
    # tests need the store — and `Executor::declare_parameter`, the seam W3b
    # hooks, lives in a `#[cfg(feature = "param-services")]` impl block.
    cargo test -p nros-node --lib --features std,sim-time,param-services --quiet
    # Issue 1036 — the boot self-report, third entry and the same defect class.
    # `boot_report` is behind a BUILD-TIME cfg set from an env var
    # (`NROS_BOOT_REPORT=1`, Zephyr `CONFIG_NROS_BOOT_REPORT=y`), and nothing in
    # `just` or `.github/` set it, so every test in `boot_report::tests` — the
    # ones issue 1036's resolution calls "unit-tested on the host" — had never
    # run. An env-var cfg is worse than a cargo feature here: `--workspace`
    # cannot even unify it in by accident.
    #
    # TWO INVOCATIONS, and that is not tidiness. The record is a process-global
    # static keeping only the FIRST allocation failure; `boot_report::tests` has
    # a `note_alloc_failed(100, 8)` case, and the link test asserts the record
    # is untouched before it writes. Sharing a binary means sharing a process
    # and the link test reads the other's numbers — measured, it failed with
    # `(100, 8)`. Two cargo invocations are two processes.
    NROS_BOOT_REPORT=1 cargo test -p nros-node --lib --features std --quiet boot_report::tests
    NROS_BOOT_REPORT=1 cargo test -p nros-node --lib --features std --quiet -- \
        --exact executor::tests::arena_exhaustion_reaches_the_boot_record
    # `env,std` and NOT `rmw-cffi`: the cffi flavour makes the wall clock an
    # extern the linker wants a platform port for, and this lane links none.
    cargo test -p nros --lib --features env,std --quiet
    # phase-359 W10 follow-up — `std` AND `platform-clock`, the combination where
    # "which wall clock wins" is observable. `platform_port_outranks_std_for_the
    # _wall_clock` DEFINES the port symbol itself, so this lane needs no port
    # linked; without the combination the test is in no lane at all, which is how
    # the `not(std)` gate on `platform_wall_clock` survived unnoticed.
    cargo test -p nros-core --lib --no-default-features --features std,platform-clock --quiet
    echo "non-default-feature unit tests passed (nros-node std + boot-report, nros env, nros-core clock)!"

# Issue 0652 — the `required-features` targets that were in NO lane.
#
# Cargo skips such a target SILENTLY: not reported as filtered, simply never
# built, so it reads as coverage while running nothing. Seven of them sat that
# way, and running them found three real defects that had rotted unobserved —
# a missing force-link anchor (`trigger_conditions` failed
# `Transport(InvalidConfig)`, which reads like a bad locator), an unused import
# fatal under `-D warnings` (`dispatch_strategy` did not compile at all), and a
# pre-phase-258 observable (`component_param` asserted the runtime's Vec where
# the seam moved to the executor's registry).
#
# `loan_e2e` is NOT here: it opens two in-process sessions and needs
# `ZPICO_MAX_SESSIONS=2`, which is a BUILD input — it belongs to
# `test-zpico-multisession`, which already owns that env and its own target dir.
# `custom_transport_loopback` is not here either; it needs a native fixture, so
# it wants a fixture-gated lane rather than this one.
#
# `signal_fd_wake` joined later (issue 0612), from `nros-node`, where it could
# not pass by construction: `NodeWake` is gated `all(alloc, rmw-cffi)` and
# `nros_node::mock` is gated `not(rmw-cffi)`, so the feature set that makes the
# wake path live is the one that removes the only session that crate can open.
# What it needed was a registered backend and a router fixture, which is what
# this crate has. Its first real execution failed outright — the signalfd worker
# asks for an 8192-byte stack against glibc's PTHREAD_STACK_MIN of 16384, so
# `Executor::signal_fd()` had been dead on every Linux host (issue 0667). Same
# thesis as the three above, one capability further out.
[group("test")]
required-features-tests:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    # Issue 0673 — through `_nextest-tolerant`, not a bare `cargo nextest run`.
    # These targets need a real zenoh router, and on a host without
    # `ros-<distro>-rmw-zenoh-cpp` every one of them raises
    # `[SKIPPED:capability]` — which a bare run counts as a FAILURE, making tier 1
    # red for an environment fact and hiding the five steps after it. A real
    # failure here still fails: the tolerance is keyed on the marker, and a
    # build/setup error (nextest exit != 100) is never absorbed.
    just _nextest-tolerant \
        -p nros-tests --no-fail-fast \
        --features trigger-test,component-runtime-test,phase216-substrate,signal-fd-wake-test \
        --test trigger_conditions --test wake_latency \
        --test component_runtime --test tier_filter \
        --test component_dispatch --test component_param \
        --test dispatch_strategy --test signal_fd_wake
    # Issue 0779 — the FILE-CFG half. These are gated by a crate-level
    # `#![cfg(feature = "lending")]` rather than a manifest `required-features`,
    # which is worse: cargo BUILDS them with the feature off, so nextest runs a
    # binary containing zero tests and calls it green. Both files had stopped
    # compiling a phase earlier and nothing noticed.
    #
    # Plain `cargo nextest run`, not `_nextest-tolerant`: these need no router
    # and no capability probe, so a red here is a red. And nextest's default
    # "no tests to run" ERROR is doing real work — it is the signal that would
    # have caught this class, so do not pass `--no-tests`.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-cffi --features alloc,lending \
        --test loan_fallback --test loan_native
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-zenoh --features lending,platform-posix \
        --test lending_traits
    # Issue 0972 — the same class one crate over, and a MODULE rather than a
    # test file. `nros-c`'s `mod node` is inside `rmw_modules!`, whose expansion
    # is `#[cfg(feature = "rmw-cffi")]`; `just test-unit` runs
    # `cargo nextest --workspace`, which activates NO features (its own comment
    # says so). So `node.rs` was never compiled by the lane that runs tests, and
    # `check::workspace-features` compiles it only to CLIPPY it — lint, never
    # execute. Fifty-two unit tests in that file, including the phase-379 W4
    # reference-invalidation set, had therefore never run anywhere.
    #
    # Found by adding a test for 0972 that passed against the BROKEN code: it
    # was not being compiled. A test that cannot fail is worth less than no
    # test, because it reads as coverage.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-c --features rmw-cffi --lib
    # Issue 0779 — two more of the baseline five, retired. Both were reachable
    # all along; what kept them dark was that the FEATURE NAMED IN THE ISSUE IS
    # NOT ENOUGH. Each file's crate-level cfg is a CONJUNCTION:
    #
    #   custom_transport  #![cfg(all(feature = "platform-posix", feature = "link-custom"))]
    #   registry_race     #![cfg(all(feature = "std",            feature = "bridge-stub"))]
    #   descriptor_seam   #![cfg(feature = "bridge-stub")]
    #
    # So a lane passing only `--features link-custom` builds the target, runs
    # zero tests, and reports green — the very shape 0779 exists to kill, one
    # level in. Pass the whole conjunction or do not bother; the `--features`
    # text here is also what `check-required-features-reachable` reads.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-cyclonedds --features std,bridge-stub \
        --test descriptor_seam --test registry_race
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-zenoh --features platform-posix,link-custom \
        --test custom_transport
    # Issue 0779 — the last three. BASELINE is now empty.
    #
    # `unix-mock` is the third conjunction in this set (`std` too), and the only
    # one whose crate says so: nvidia-ivc carries a `compile_error!` naming the
    # missing feature. The other two failed SILENTLY — a target that builds and
    # runs zero tests — which is why they sat unnoticed while this one would have
    # stopped anyone who tried.
    #
    # `c-stub-test` needed real repairs, not just a flag: its abi_layout_check.c
    # had not compiled since phase-321 W2.e moved the crate (a stale `-I`), and
    # c_stub_platform asserted a call count for a trait that had since shrunk.
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-platform-cffi --features posix-c-port \
        --test c_port_posix --test c_port_posix_critical_section \
        --test c_port_posix_net --test c_port_posix_timer \
        --test c_port_posix_wake --test wake_wrapper
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-platform-cffi --features c-stub-test \
        --test c_stub_platform
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nros-rmw-cffi --features c-stub-test \
        --test c_stub_transport
    cargo nextest run "${cargo_nextest_args[@]}" \
        -p nvidia-ivc --features std,unix-mock \
        --test loopback
    echo "required-features test targets passed!"

# issue 0379 — clippy gate for the CLI sub-workspace. No lane ran clippy on
# packages/cli, so ~107 warnings accreted unnoticed. Mirrors cli-tests
# (separate workspace, its own Cargo.toml/lock). Two deliberate deviations from
# a bare `--workspace` clippy:
#   * `--exclude ros-launch-manifest-{model,sched,types}` — these are vendored
#     submodule crates under third-party/, pulled in as path-dep members; their
#     lints are upstream's, out of scope here.
#   * `--locked` sits BEFORE `--`: the scripts/bin/cargo shim (issues 0359/0378)
#     appends $NROS_CARGO_FLAGS at the END of argv, which would land after `--`
#     and reach clippy-driver ("Unrecognized option: 'locked'"). Passing it
#     ourselves makes the shim skip its own injection.
[private]
cli-clippy:
    #!/usr/bin/env bash
    set -e
    cargo clippy --manifest-path packages/cli/Cargo.toml --workspace --all-targets \
        --exclude ros-launch-manifest-model \
        --exclude ros-launch-manifest-sched \
        --exclude ros-launch-manifest-types \
        --locked -- -D warnings
    echo "CLI clippy passed!"

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

# Forbid `find` scans for git-tracked files (7m36s -> 0.8s, measured).
# Ratchet: no NEW `grep -q` conditional that reads a tool error as a
# non-match (issue 0726). Existing sites are baselined per file.
[private]
grep-q-error-conflation:
    @python3 scripts/check-grep-q-error-conflation.py

# Issue 0737 — an example's message callback may not drop a sample silently.
# From outside the process a silent `return` and a message that never arrived
# are the SAME observation; that ambiguity cost two hosts an investigation each.
[private]
no-silent-sample-drop:
    @python3 scripts/check-no-silent-sample-drop.py

# phase-373 W1 — every `test()`/`binary()` predicate in .config/nextest.toml must
# match at least one real test. The `binary()` half is already covered statically
# by `check-nextest-binary-filters` on the fast line; this is the `test()` half,
# which that gate cannot do because rstest case names appear nowhere in the
# sources. It needs the test binaries, so it runs from `test-all` rather than
# `just check`, which builds none.
[private]
nextest-test-filters:
    @python3 scripts/check-nextest-test-filters.py

[group("check")]
no-tracked-file-find:
    @./scripts/check-no-tracked-file-find.sh

[private]
version-lockstep:
    @./scripts/check-version-lockstep.sh

# Compile-time SOURCE/precondition gates that ship as `nros-tests` test binaries
# (header-ABI mirror, two-libc precedence, zephyr prj.conf requirements). These
# are the `cargo test -p nros-tests --test …` steps `check.yml` runs inline;
# wrapped here so `just check` runs the identical set. Compiles nros-tests, so
# slower than the buildless gates but still a static/precondition check.
[private]
source-gates:
    #!/usr/bin/env bash
    set -e
    # These tests assert a BUILD-STAGE stamp (`.compile-ok`), so this gate has
    # to PRODUCE it. Without this line the lane silently depends on
    # `build-test-fixtures`, which contradicts ci-l1's documented "NO FIXTURES"
    # contract and made the required CI check permanently red the moment it
    # became required: `BuildFailed("Test fixture binary not prebuilt")` on a
    # runner that builds no fixtures. It costs ~13 s — the snippets are small
    # `cargo check` template crates, which is the whole point of issue 0034's
    # design. The push lane never noticed because it runs `check-fast` only.
    # ONLY the lane this gate asserts. `platform_header_compile` consumes the
    # nine `platform_hdr_*` stamps, and ALL NINE are `cxx-syntax` records —
    # verified against `examples/fixtures.toml`, not inferred from the ids. An
    # earlier version of this line said `cargo-check` on exactly that inference
    # and excluded the only lane the test needs; it passed locally because the
    # stamps already existed from previous full builds, and failed in CI where
    # they do not.
    #
    # Building every lane instead is what this replaced: that also built
    # `freertos_firmware`, a `cargo-build` row needing the FreeRTOS KERNEL
    # SUBMODULE which CI does not provision.
    NROS_COMPILE_CHECK_LANES=cxx-syntax bash scripts/build/compile-check-fixtures.sh
    # Issue 0673 — through `_nextest-tolerant`, not bare `cargo test`.
    #
    # `cross_libc_precedence_gate` already does the right thing: it calls
    # `nros_tests::skip!` when `arm-none-eabi-g++` has no usable libstdc++, with
    # a comment saying "Skip rather than false-fail". But `skip!` PANICS with a
    # `[SKIPPED]` marker, and a bare runner counts that as a FAILURE — only the
    # junit rewrite in `_nextest-tolerant` turns it back into a skip. So a
    # correct skip was reported as a red, and it made the merge group fail on an
    # ENVIRONMENT FACT rather than on anything in the change.
    #
    # That fact is not incidental either: this gate lives in `check-build` ->
    # `ci-l1`, which promises "no SDK, no QEMU and no cross toolchain". A C++
    # capable newlib cross is exactly a cross toolchain, so the skip is the lane
    # contract working, and swallowing it as a failure was the bug.
    #
    # A real failure still fails: the tolerance keys on the marker, and a
    # build/setup error (nextest exit != 100) is never absorbed.
    just _nextest-tolerant \
        -p nros-tests --no-fail-fast \
        --test platform_header_compile \
        --test cross_libc_precedence_gate \
        --test zephyr_prjconf_requirements

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

# Link-determinism gate (RFC-0042 D3) — build the host staticlib pair, then assert
# the `--allow-multiple-definition` masked dups are ONLY the shared Rust dep
# closure (no app ODR violation). The `check.yml` staticlib step (SSoT).
[private]
staticlib-symbols:
    #!/usr/bin/env bash
    set -e
    bash scripts/build/link-determinism-fixture.sh
    # Skip-capable: routed through the tolerant runner so an unmet
    # precondition reads as a SKIP rather than a red naming a test the
    # change never touched. Gated by `check-skippable-tests-tolerant`.
    just _nextest-tolerant -p nros-tests --no-fail-fast --test staticlib_duplicate_symbols

# Borrowed-view runtime E2E (RFC-0033 / #0423) — link the C + C++ proof binaries at
# the build stage, then RUN them (they assert every borrowed view aliases the CDR
# buffer). Bespoke recipe like `check-staticlib-symbols` because the composite
# (cargo + codegen + a raw gcc/g++ link with a weak config-variant anchor) fits no
# `compile_check_fixture` builder.
[private]
borrowed-e2e:
    #!/usr/bin/env bash
    set -e
    bash scripts/build/borrowed-e2e-fixture.sh
    # Skip-capable: routed through the tolerant runner so an unmet
    # precondition reads as a SKIP rather than a red naming a test the
    # change never touched. Gated by `check-skippable-tests-tolerant`.
    just _nextest-tolerant -p nros-tests --no-fail-fast --test borrowed_e2e

# Embedded feature-unification guard — no `feature "std"` activation path may
# reach an embedded target's production-link view. The `check.yml` step (SSoT).
[private]
embedded-feature-unification:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/lib/grep-q.sh
    source scripts/lib/grep-q.sh
    tree=$(cargo tree -p nros-serdes --edges=normal,build \
        --target thumbv7em-none-eabihf --no-default-features --workspace 2>&1)
    # issue 0726/0732 — a `grep -q` that cannot start reports "no `std` here",
    # which is this gate's PASS. A pipe makes it likelier still: the writer sees
    # SIGPIPE when grep dies early. `nros_grep_q` exits 2 rather than verdicting.
    nros_grep_q 'feature "std"' <<<"$tree" && found=0 || found=$?
    if [ "$found" -eq 0 ]; then
        echo "feature std activation paths under embedded target:" >&2
        echo "$tree" | grep -B2 'feature "std"' | head -50 >&2
        echo "Move the offending dep under [target.'cfg(not(target_os = \"none\"))'.dependencies]." >&2
        exit 1
    fi
    echo "no feature std paths under embedded target."

# Canonical `nros codegen` invocation-shape guard. The `check.yml` step (SSoT).
[private]
codegen-invocation:
    @scripts/ci/codegen-invocation-check.sh

# A configure-time emitter must make a rebuilt `nros` re-run its configure
# (issue 1018). The build-time generator says so in `DEPENDS`; `execute_process`
# cannot, so the tool goes in `CMAKE_CONFIGURE_DEPENDS` via
# `nros_codegen_tool_reconfigure()`. #182 registered it at ONE of four sites,
# inline, so the other three kept museum generated code after a `just setup-cli`
# unless the image also happened to call `nano_ros_entry()`.
[group("checks")]
codegen-tool-reconfigure:
    @python3 scripts/check-codegen-tool-reconfigure.py

# String-convention guards (forbidden org / retired-tool refs in user surfaces).
# The `check.yml` step (SSoT).
[private]
string-conventions:
    @scripts/ci/string-conventions-check.sh

# Issue 0481 — a readiness grep naming an AMBIGUOUS literal waits out its whole
# timeout in silence and still passes. Buildless: reads sources only.
[private]
readiness-marker-literals:
    @bash scripts/check-readiness-marker-literals.sh

# Issue-id integrity: ids unique across docs/issues/ + archived/, and each
# file's `id:` frontmatter matching its filename. Parallel sessions kept
# picking the same "next free" id — six were duplicated across thirteen files
# before this gate existed, which made every `See 0051-*` pointer ambiguous.
[private]
issue-ids:
    @scripts/ci/issue-ids-check.sh

# phase-400 W8 — a knob migrated into the RFC-0049 ladder has exactly one
# reader. Retirement is a wave, not a side effect: a mechanism that still
# resolves is one people still use, and a fallback that silently wins is how
# issues 0135 and 0316 happened.
[private]
knob-single-reader:
    @python3 scripts/check/check-knob-single-reader.py

# phase-400 — the config census, and the ratchet on it. The ladder knob count
# is the number every wave gate is stated against; it was unmeasurable until
# this existed, because the phase doc recorded a table and no method.
config-knob-census:
    @python3 scripts/check/config-knob-census.py --check

# The reverse of `check-sysdep-remedies` — phase-413 W3.
#
# That gate refuses a hand-written native install command in a `just` recipe,
# and `[prereq.doxygen]`'s own `why` reads "found undeclared by
# check-sysdep-remedies".
#
# DESCRIBED, not quoted — `check-sysdep-remedies` is a text scan and cannot tell
# a command from a sentence about one, so a comment naming the forbidden
# spelling fails it. `provision-zenohd`'s comment says exactly this and takes
# exactly this way out; writing that phrase here anyway cost this PR a red.
#
# Nothing guarded the other direction: a WORKFLOW may
# not restate a package `[prereq.*]` already declares. Three did — doxygen and
# graphviz in `docs.yml`, clang and libclang-dev in `nightly.yml` — each with
# its dnf/pacman/brew spellings and a presence probe sitting in the index.
#
# Buildless and source-only, so it belongs on the fast line.
# A workflow must provision with `just setup <scope>`, not `just <scope> setup`.
# Only the dispatcher runs `_setup-common`, where the host facts every tier
# asserts get provisioned; the module spelling skips it silently and the lane
# fails later on a precondition nothing provisioned. nightly's platform job did
# exactly that and carried a hand-rolled cross-target step to compensate.
#
# The sibling of `check-preconditions-provisioned`: that one asks whether setup
# PROVIDES the facts, this asks whether workflows INVOKE the form that runs them.
#
# phase-422 W3 added a second arm over PROSE — book/, AGENTS.md, CLAUDE.md, the
# living `docs/` subtrees and every README — because 64 occurrences across 38
# files taught the module spelling to readers. The three RECORD series
# (`docs/roadmap/`, `docs/issues/`, `docs/research/`) are deliberately NOT
# scanned: they narrate what a command DID at a time, so gating them would be a
# steady false-positive stream. The recipe keeps its `workflow-` name because
# the workflow arm is still the one a red usually means.
[group("checks")]
workflow-setup-spelling:
    @python3 scripts/check-workflow-setup-spelling.py

[group("main")]
workflow-indexed-apt:
    @python3 scripts/check-workflow-indexed-apt.py

# issue 0933 — a CI step that invokes `just`/`nros`/`west` must source the repo
# environment. `activate.sh` is what exports `nano_ros_ROOT`, and that is how
# `find_package(nano_ros)` resolves: the config lives at the checkout root and a
# generated workspace root sets no prefix on purpose. `just doctor` enforces this
# for a developer shell; nothing enforced it for a CI step, which is the one
# place the shell is fresh every time.
#
# Pure YAML + regex over `.github/workflows/`, so it belongs in the source-free
# tier. It skips comment prose AND heredoc bodies — both false positives were
# live in the tree (a comment mentioning `nros sync`; a pull-request comment
# whose TEXT tells the author to run `just queue-triage`).
[private]
workflow-repo-env:
    @python3 scripts/check-workflow-repo-env.py

# issue 0936 — a `just <recipe>` printed in a DOCUMENT must name a recipe that
# exists. `check-just-recipe-refs` covers justfiles and workflows and hands
# documents to `check-doc-refs`, which checks links and paths, not recipe names —
# so a document was the one place an invocation could rot unwatched, while being
# exactly where a reader copies one from.
#
# A RATCHET over `.config/doc-recipe-refs-baseline.txt` (102 known dead
# references), because fixing them is a research task per line and guessing
# would replace a dead command with a wrong one. Buildless: markdown + the
# justfile grammar.
[private]
doc-recipe-refs:
    @python3 scripts/check-doc-recipe-refs.py

# phase-411 — a CI step must not fall back from one `just` verb to another.
# `build-all || build-examples || build` made a FAILED verb build something
# smaller and report success, which is the "skip and report success" shape
# phase-411 removes, written in shell instead of a skip helper. Buildless: YAML
# plus a regex, and it skips comments and heredocs for the reason
# `check-workflow-repo-env` documents.
[private]
ci-no-verb-fallback:
    @python3 scripts/check-ci-no-verb-fallback.py

# issues 0320 / 0334 — no build-host absolute paths in tracked code/config.
# A pure grep, so it belongs in the source-free tier (see #337 for what happens
# when a gate here needs more than the index).
[private]
absolute-paths:
    @scripts/ci/absolute-path-check.sh

# Per-platform (board, rmw) dependency-chain resolution — proves each cell's dep
# chain resolves (nros setup --dry-run + codegen + cargo tree, no compile). The
# `check.yml` step (SSoT). Needs ROS 2 sourced (for std_msgs .msg defs) + the
# nros CLI; SKIPS cleanly when ROS isn't sourced so `just check` still runs
# everywhere (CI sources ROS).
[private]
dep-chain:
    #!/usr/bin/env bash
    set -e
    if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
        if [ -f /opt/ros/humble/setup.bash ]; then
            source /opt/ros/humble/setup.bash
        else
            # shellcheck source=scripts/build/check-skip.sh
            source scripts/build/check-skip.sh
            nros_check_skip dep-chain "ROS 2 not sourced (AMENT_PREFIX_PATH unset)"; exit 0
        fi
    fi
    source scripts/build/cargo.sh
    NROS="$(nros_cli_bin)" scripts/ci/dep-chain-check.sh

# Phase 121.4.b — verify <nros/platform.h> matches the Rust extern block
# and the `nros_platform_export_*!` macro emissions in nros-platform-cffi.
[private]
platform-abi-mirror:
    @bash scripts/check-platform-abi-mirror.sh

# Issue 0978 — the generated config header a leaf compiles against must be the
# CURRENT one, not merely a PRESENT one.
#
# `mirror-generated-header.sh` chooses between the leaf's own corrosion copy and
# the leaf-independent copy in the shared cargo target dir. Issue 0805 made it
# prefer the leaf's and fall back only when that was ABSENT, on the grounds that
# the two are "the same bytes" — true only when one build-script run wrote both.
# Once leaves share a cargo target dir the script runs for ONE of them, so every
# other leaf keeps its copy from whenever it last ran: present, and arbitrarily
# old. Measured 19 of 20 native leaves six days stale, caught by the 0369 size
# anchor as `undefined reference to nros_config_variant_sz_<hash>` — a link
# error that names a hash and no file.
#
# The self-test is in the script so the rule and its cases live together; it
# stages temp dirs only (no cargo, no cmake, milliseconds), so it belongs on the
# fast line.
[private]
mirror-header-precedence:
    @bash scripts/build/mirror-generated-header.sh --self-test


# Issue 0980 — `runner-doctor` answered "where is the Zephyr SDK?" from the job
# CHECKOUT while the build answers it from the CMake user package registry.
# `scripts/zephyr/sdk/` is gitignored and `actions/checkout` runs
# `git clean -ffdx`, so a self-hosted runner's SDK is necessarily somewhere
# else — which made the derived path wrong by construction and failed every
# merge-group `L3` job on a host that builds fine.
#
# Same shape as the gate above, for the same reason: the self-test lives in the
# script so the rule and its cases cannot drift apart, and it stages temp dirs
# only (no SDK, no cmake, no network), so it belongs on the fast line.
[private]
runner-doctor-sdk-resolution:
    @bash scripts/ci/runner-doctor.sh --self-test

# issue 0946 — the connect locator has exactly ONE producer.
#
# `NROS_ENTRY_LOCATOR` (the address an embedded image dials) was computed by two
# independent per-platform ladders that DISAGREED on threadx and freertos, one of
# which carried the note "Mirrors NanoRosNodeRegister.cmake freertos branch; keep
# in sync" — a hand-sync instruction with nothing enforcing it. The invariant is
# ONE PRODUCER, not equal literals: the lanes cite different networks and may
# each be right, so the gate demands one table rather than one value. Drift is
# silent (a wrong locator builds and links; the image just connects nowhere), so
# this needs a check and not a convention. Buildless — pure source analysis.
[private]
entry-locator-ssot:
    @python3 scripts/check-entry-locator-ssot.py

# Issue 1050 defect (3) — `NROS_ENTRY_RMW` may only name a backend that
# registers under that name.
#
# The RMW selector gained a BAKED rung, so `nano_ros_entry()` /
# `nros_px4_add_module()` now put a NAME on the target and the core resolves it
# through `nros_rmw_cffi::resolve_backend`. That is a LOOKUP, and a miss is
# `Unknown` — a hard error, not a fallback to the registry default — so a rename
# on either side turns every image built for that RMW into one that dies at
# `nros::init()`.
#
# The two vocabularies (cmake's `NROS_RMW_KNOWN`, and the string each backend
# passes to `nros_rmw_cffi_register_named`) agreed by convention and nothing
# held them there. Buildless: both are read out of the sources.
[private]
entry-rmw-vocabulary:
    @python3 scripts/check-entry-rmw-vocabulary.py

# RFC-0054 (phase-299 W3) — the committed bindgen output must match a fresh
# regeneration from the C-header SSoT packages. Loud skip when bindgen-cli
# (pinned; maintainer/CI-only dep) is absent — embedded consumers never
# need it.
# phase-383 W7.e/W7.f (RFC-0065 D12) — `nano_ros_support_library()` must emit the
# force-link flag AND the `LINK_DEPENDS` that gives it a rebuild edge (issue
# 0475), and must route `.ld` fragments to whichever seam the platform has. The
# test configures throwaway projects against the HOST compiler only — no
# nano-ros build, no SDK, seconds — so it belongs on the fast line where every
# task runs it. Loud skip when the host has no cmake/ninja/compiler: `check-fast`
# is documented to run green on a pristine worktree.
cmake-support-library:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja nm ar "${CC:-cc}"; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip cmake-support-library \
            "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-support-library-tests.sh

# phase-403 W8 (issue 0940) -- the READER for the exported message-bound
# inventory must derive the four size knobs a bound inventory can answer, and
# must REFUSE loudly rather than derive over a closure holding an unbounded
# type. The second half is the one worth gating: deriving over the bounded
# subset yields a plausible number a real sample can exceed, and the drop is
# SILENT on the C++ arena dispatch path. `cmake -P` against hand-written
# fragments -- no build, no codegen, no toolchain, seconds -- so it belongs on the
# fast line beside its cmake sibling above.
message-bound-knobs:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip message-bound-knobs "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-message-bounds-tests.sh

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
# phase-403 W9 (issue 0965) -- the READER for the ENTITY inventory, the second
# source `NanoRosMessageBounds.cmake` says by name that it needs.
#
# The derivation itself is unit-tested in `nros_cli_core::entity_inventory`
# (`cargo test -p nros-cli-core`), including the island's 33-entities /
# 19-slots case. This gates what CMAKE owns: a refusal publishes NO number, an
# unknown schema FATALs rather than being read field-by-field, a missing input
# refuses rather than breaking a clean-tree configure, and a broken declaration
# is fatal and names the component. The CLI is STUBBED, so this needs cmake and
# nothing else -- no cargo build, which is what keeps it affordable on the fast
# line beside its sibling above.
entity-inventory-knobs:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! command -v cmake >/dev/null 2>&1; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip entity-inventory-knobs "missing host tool(s): cmake"
        exit 0
    fi
    ./tests/cmake-entity-inventory-tests.sh

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

# RESTORED (2026-09-04). This recipe and its fast-lane list entry were deleted
# by `e569d9a55`, a commit about which Zephyr SDK the CI image bakes, whose
# message does not mention them -- an accidental loss in a rebase, not a
# decision. For a day the test script sat tracked with NO caller anywhere in
# `just/`, `justfile` or `.github/`: a gate that runs nowhere, guarding the
# delivery half of phase-403 step 2, which had merged hours earlier.
#
# `check-test-scripts-have-callers` now catches that class -- it was written
# from this incident and fires on exactly this state.
# phase-403 step 2 -- does the declared `@depth=` REACH A COMPILER?
#
# `just check cpp` proves the CHECK: it compiles a TU whose declared depth and
# passed QoS disagree and asserts the build is rejected, with the topic and both
# numbers in the diagnostic. It hands the table to the compiler with a `-I` of
# its own, so it says nothing about DELIVERY.
#
# This is the delivery half, and it is the half that goes wrong silently: a
# component whose generated table never lands on its include path compiles with
# every declared-depth assertion disabled, and looks exactly like a component
# whose depths all agree. Six mechanisms in this campaign have been correct and
# unreachable for precisely that reason.
#
# Drives `_nros_emit_declared_qos_header()` in a real configure with the real
# CLI. Needs cmake, a C compiler and a built `nros` -- and SKIPS rather than
# passing when any is missing, because a gate that examined nothing must not
# read as a pass.
declared-qos-header:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    command -v cmake >/dev/null 2>&1 || _missing="$_missing cmake"
    command -v cc >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1 \
        || _missing="$_missing cc"
    _cli="${NROS_CLI:-packages/cli/target/release/nros}"
    [ -x "$_cli" ] || _missing="$_missing nros(just setup-cli)"
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip declared-qos-header "missing:$_missing"
        exit 0
    fi
    ./tests/cmake-declared-qos-header-tests.sh

# Issue 0882 / the stale-`build.ninja` class -- a manifest error is raised at
# LOAD, before any rule runs, so ninja can never re-run cmake to repair itself.
# The dir is wedged and every ninja invocation in it reports the same error
# whatever you ask for. `just reconfigure-stale` repairs such dirs in place by
# re-running cmake; it never deletes one, because a wedged build dir is the
# only reproduction of whatever wedged it.
#
# This gate is the NEGATIVE CONTROL for that script. On a healthy tree the
# script prints "OK, N build dir(s) load", which is exactly what a probe that
# can never fail would print -- so the check that matters is whether it still
# detects a dir that is genuinely unloadable. Real cmake + real ninja,
# `project(... NONE)`, no compiler, ~2 s.
reconfigure-stale:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip reconfigure-stale "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-reconfigure-stale-tests.sh

reconfigure-on-change:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip reconfigure-on-change "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-reconfigure-tests.sh
abi-bindings:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! command -v bindgen >/dev/null 2>&1; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip abi-bindings \
            "bindgen-cli not installed (cargo install bindgen-cli --locked --version 0.72.1)"
        exit 0
    fi
    bash scripts/gen-abi-bindings.sh >/dev/null
    if ! git diff --exit-code --quiet -- \
        packages/rmw/cffi/src/generated.rs \
        packages/platform/nros-platform-cffi/src/generated.rs \
        packages/boards/nros-board-cffi/src/generated.rs; then
        git --no-pager diff --stat -- \
            packages/rmw/cffi/src/generated.rs \
            packages/platform/nros-platform-cffi/src/generated.rs \
            packages/boards/nros-board-cffi/src/generated.rs
        echo "ERROR: committed ABI bindings are stale — headers changed without rerunning scripts/gen-abi-bindings.sh; commit the regenerated files."
        exit 1
    fi
    echo "ABI bindings match the C-header SSoT."

# Phase 176.4 — verify <nros/board.h> matches the Rust extern block
# and the `nros_board_export!` macro emission in nros-board-cffi.
[private]
board-abi-mirror:
    @bash scripts/check-board-abi-mirror.sh

# issue 0488 residue 4 follow-up — the NuttX integration Makefile is on a path no
# lane executes (`CONFIG_NROS` is set by no shipped defconfig), so it rots.
[private]
nuttx-integration-makefile:
    @bash scripts/check-nuttx-integration-makefile.sh

# Issue 0555 — `nros_platform_clock_{ms,us}` are `static inline` wrappers in
# `nros/platform.h` now (RFC-0073), so the header IS the definition. The rename
# broke four consumers in a row, each visible only after the previous cleared
# (#541, upstream 5dc2fa869, #547, #548), and #548 asked for this gate by name.
# Two arms: a use with the header out of scope, and a hand-written `extern`
# declaration — which compiles and then fails at link. Buildless.
retired-platform-clock-symbols:
    @python3 scripts/check-retired-platform-clock-symbols.py

# eyre's `Context` alias is behind `#[cfg(feature = "anyhow")]` from 0.6.13, so
# code using it compiles only against a graph that resolves 0.6.12.
[private]
eyre-context-alias:
    @bash scripts/check-eyre-context-alias.sh

# phase-340 W2.d — `--core-only` selects by the derived variant predicate; this
# holds it equivalent to the authored-`target_dir` spelling it replaced, on
# every platform a caller actually passes.
[private]
core-only-predicate:
    @bash packages/testing/nros-tests/tests/core_only_predicate.sh

# phase-344 W7 — RFC-0070 R1 at WORKSPACE scope only. examples/** copy-out
# leaves keep the Cargo/CMake convention and are deliberately exempt.
[private]
workspace-build-output:
    @bash scripts/check-workspace-build-output.sh

# issue 0478 — every cc::Build must name the nano-ros cc policy helper
# (the strict diagnostics of 0383, and the clang-only frame-pointer flag gcc
# rejects). Both classes escaped once through an unrouted call site.
[private]
cc-build-policy:
    @bash scripts/check-cc-build-policy.sh

# Issue 1003 / 1017 — every emitted `run_components` call must NAME a session.
#
# Ten CMake entry templates passed only the setup fn, which binds the overload
# that hard-codes `"node"`. Every image built through `nano_ros_add_node` then
# registered under that name, and the XRCE client key derives from it — a talker
# and a listener hashed to ONE client. It survived from 2026-06-13 to 2026-09-03
# with a CORRECT sibling producer beside it (`emit_cpp.rs` has passed the
# boot-config name since 2026-06-27): two spellings of one fact, uncompared.
#
# The check covers BOTH producers, and deliberately does NOT know the per-board
# overload sets — encoding those here would make this a third authored copy,
# which is the defect it guards. See the script's header.
entry-session-name:
    @python3 scripts/check-entry-session-name.py

# Issue 0160 — hand-mirrored FFI structs (component.h vs cbindgen's
# nros_cpp_ffi.h) must not drift on append (the #131 stale-mirror ABI class).
[private]
ffi-struct-mirrors:
    @bash scripts/check-ffi-struct-mirrors.sh

# RFC-0088 D5 / phase-421 W6 — `NROS_SERIALIZATION_FORMAT{,_ID}` names the
# format of THE image's single backend. A bridge image links two and has no
# single answer, so a bridge-linked TU referencing the macro is a wrong answer
# that compiles. Pure source scan (~1 s, no build, no fixture, no SDK), so it
# is affordable on the fast line — and it carries its own vacuity guards, since
# a gate whose subject set emptied reports the same green as one that passed.
[private]
format-macro-scope:
    @python3 scripts/check-format-macro-scope.py

# Issue 0268 — the per-build sizes headers (`nros_{,cpp_}config_generated.h`)
# mirrored onto the consumer include path must equal the copy build.rs wrote.
# A stale mirror sizes the C `_opaque` buffers from museum data while Rust
# placement-constructs the current object into them — silent corruption
# (0268: freertos C, 336 bytes; 0245: zephyr C++, 32 bytes). Scans whatever
# build trees exist locally; prints the tree count so a vacuous pass is visible.
[private]
sizes-header-mirrors:
    @bash scripts/check-sizes-header-mirrors.sh

# Issue 0336 — a retired submodule path must not survive anywhere that a user or
# a CI job would follow it. RFC-0060's sweep missed scripts/bootstrap.sh, nine
# doc copies and eight workflow refs; this grep would have caught all of them.
[private]
retired-submodule-refs:
    @bash scripts/check-retired-submodule-refs.sh

# phase-327 W3 (issue 0368) — no hand-written sudo-apt remedy text in just
# recipes; the [system.*] index class + `nros setup --system` own that text.
sysdep-remedies:
    @bash scripts/check-sysdep-remedies.sh

# Issue 0926 / phase-327 W4 — a dist's `system = [..]` must cover the libraries
# it ACTUALLY needs, re-measured from the provisioned dists themselves.
#
# NOT on the fast line, and not in an affordability tier: it needs a provisioned
# store, which no such lane builds (`check-lane-contracts`). It SKIPS loudly
# when there is no store, so running it anywhere is safe — it simply cannot
# answer without its input, and says which.
#
# `just doctor` is its home: doctor is where host state is examined, and this is
# the one gate whose subject lives in `~/.nros/sdk` rather than in the tree.
[private]
dist-runtime-deps:
    @python3 scripts/check-dist-runtime-deps.py

# issue 0372 — `activate.sh` / `activate.fish` are SOURCED, so anything that
# aborts them mid-file silently drops every export below the failure. Two
# unmatched SDK-store globs did exactly that under zsh (fatal `nomatch`), on
# empty stores AND on the versioned layout `nros setup` writes, while no lane
# sourced either file under a non-bash shell. This gate sources both in every
# available shell against both store shapes and asserts they reach their last
# line. zsh/fish absent = loud skip, never a silent pass.
[private]
activate-shells:
    @bash scripts/check-activate-shells.sh

# Phase 215.F.2 — board-crate manifest drift gate. For every
# `packages/boards/nros-board-*` carrying BOTH a `board.cmake` sidecar
# and a `[package.metadata.nros.board]` table, run `nros board info
# <name> --check-drift` and fail on any field-by-field drift between the
# cmake face and the Cargo face. Skips when the in-tree `nros` CLI isn't
# built (the packages/cli phase215_f integration test still covers it).
[private]
board-manifest-drift:
    @bash scripts/check-board-manifest-drift.sh

# Phase 230.0.2 (RFC-0034) — no crate may call the host kernel allocator
# directly except a platform port; everything routes through
# nros_platform_alloc. Advisory until Wave 1 migrates the inventory
# (set NROS_ALLOC_GATE_HARD=1 to enforce).
[private]
no-direct-kernel-alloc:
    @bash scripts/check-no-direct-kernel-alloc.sh

# Phase 251 — forbid `--allow-multiple-definition` in the build system (it lets
# two same-named-but-different functions coexist → wrong-copy hazard). Fails on
# any non-allowlisted real use; allowlist (scripts/allow-multiple-def-allowlist.txt)
# carries the audited exceptions, target empty. Buildless.
[private]
no-allow-multiple-def:
    @bash scripts/check-no-allow-multiple-def.sh

# Phase 313 W6 — forbid the retired `nros_board_common::board_init` API from
# creeping back (boards use `nros_platform::board::*` or the C ABI). Buildless.
[private]
no-board-init:
    @bash scripts/check-no-board-init.sh

# Issue 0330 (class of 0155/0163) — a Zephyr Rust example whose `rmw-*` feature
# forwards to a real backend dep must invoke `nros::force_link_backend!`, or
# rustc's staticlib DCE drops the backend's `#[no_mangle]` register export and
# the image boots with NO backend registered. Mutation-verified silent: removing
# the anchor still builds AND links. The anchor used to be emitted by
# `zephyr_component_main!` (so it could not go missing); 0330 moved it to the
# app crate, which is what makes this gate necessary. Buildless.
[private]
rmw-force-link-anchor:
    @bash scripts/check-rmw-force-link-anchor.sh

# Issues 0332 + 0349 — the RMW vtable's required-slot list and its
# `.expect("rmw vtable: …")` dispatch sites must be the SAME set. A slot
# expect-ed but not required panics mid-spin on no_std (0332); a slot required
# but not expect-ed refuses working backends (0349 — how xrce became
# unregistrable). Checked both directions. Buildless.
[private]
rmw-required-slots:
    @bash scripts/check-rmw-required-slots.sh

# phase-320 W2 — board support tiers must match the evidence, and every board
# package must be enumerated. A tier that is merely asserted drifts: the book
# claimed ARM FVP was "Tested" (legend: "boots in CI") for a license-walled
# target, and matrix.rs carried FVP `Runtime` cells whose tests always skip.
# Also checks the generated support table is not stale. Buildless.
# Issue 0833 — RFC-0035's slot table is GENERATED from `rmw_vtable.h` plus
# `first_missing_vtable_slot`'s `require!()` list, the same source
# `check-rmw-required-slots` reads, so the table and that gate cannot disagree.
# It had drifted to describing a 33-slot ABI that was really 74: 17 dead names,
# 61 slots unmentioned. Buildless.
[private]
rmw-slot-table:
    @python3 scripts/gen-rmw-slot-table.py --self-test
    @python3 scripts/gen-rmw-slot-table.py --check

[private]
board-tiers:
    @python3 scripts/check-board-tiers.py
    @python3 scripts/gen-board-support-table.py --check

# RFC-0079 — a tier priority pin must respect its port's declared address plan.
# Scheduling priority is ONE space shared by application tiers and system tasks;
# pins were hand-chosen into it with no record of what was already there, and
# 34 of 38 in the tree were colliding, preempting, or on a port that declares
# nothing. A pin landing ON a reserved band FAILS; one that merely outranks a
# band warns until `above = "<band>"` exists to make it a stated choice. Also
# cross-references each plan against the code it describes, so the plan cannot
# drift into a second spelling of the same fact. Buildless.
[private]
tier-priority-plan:
    @python3 scripts/check-tier-priority-plan.py

# RFC-0079 §4.1 — finish the check `check-tier-priority-plan` DEFERS.
#
# Zephyr's reserved band is computed per IMAGE from Kconfig, so the static gate
# cannot judge its pins and says so instead of pretending. This resolves the
# band from every built `.config` and checks the pins against each. With no
# argument it examines EVERY image in the tree: a derived band is a property of
# an image, so letting the caller pick which one to prove is how a green comes
# to mean less than it looks.
#
# SKIPS loudly on a host with no built Zephyr image, rather than passing —
# issue 0599's rule. Buildless (reads `.config`, builds nothing).
tier-priority-plan-image config="":
    @python3 scripts/check-tier-priority-plan-image.py {{config}}

# Issue 0363 — a stale in-tree `nros` used to surface at `check-dep-chain`,
# minutes in, as nine failed cells whose printed cause was a cargo resolution
# error. Same predicate (it CALLS `nros_cli_bin`), better position. Buildless.
#
# Issue 0466 — that position is the head of `check-build`, NOT of `check-fast`.
# `check-dep-chain`, the gate this exists to front-run, is a build-tier gate; no
# fast-tier gate execs the CLI at all. Sitting in the fast tier it contradicted
# that tier's whole contract ("needs neither the nros CLI nor any provisioned
# source"): every buildless gate became unreachable on a tree whose CLI was
# merely out of date — which ANY pull, rebase or stash makes it, since the stamp
# covers CLI sources. Measured: `just check fast` failed in 0.77s having checked
# nothing. Four source-level reds sat on main behind exactly this.
#
# phase-395 W12 — `check-dep-chain` has since MOVED OUT of `check-build` to
# post-submit (158 s measured, an 8-cell board×rmw matrix that no single pull
# request changes), so this no longer front-runs it and must not be read as
# doing so. It stays at the head of `check-build` on its own merits: a stale CLI
# breaks codegen for every gate below it, and failing in 0.1 s with the real
# cause beats failing minutes in with a cargo resolution error. The post-submit
# lane runs this same predicate before its own dep-chain step.
[private]
cli-fresh:
    @bash scripts/check-cli-fresh.sh

# Issue 0652 — a `required-features` target no recipe enables is invisible:
# cargo does not report it as filtered, it simply never builds it, so it looks
# like coverage while running nothing. Baseline is a shrinking backlog.
[group("main")]
required-features-reachable:
    @python3 scripts/check-required-features-reachable.py

# Issue 0582 — a place meaning "the host" spelled as a literal triple. Nine
# sites wore this bug; five failed SILENTLY, because `NO_DEFAULT_PATH` turns a
# wrong path into an empty result, a skipped block and a much later failure that
# names neither. Invisible on x86, which is why it survived a year after being
# written down.
[group("main")]
host-triple-literals:
    @python3 scripts/check-host-triple-literals.py

# Issues 0400/0706/0712 — EVERY `export -f` list in scripts/build/ must close
# over its call graph, or a helper added to an exported function dies
# "<name>: command not found" in the make WORKER and nowhere else. Three times
# now. Supersedes the #0717 gate, which checked one entry point on the reasoning
# that build_root_derivation.sh covered the other — true, but that scenario
# EXECUTES one call path, so a helper on a branch not taken is invisible to it.
[private]
export-f-closure:
    @bash scripts/check-export-f-closure.sh

[group("main")]
literal-domain-id:
    @python3 scripts/check-literal-domain-id.py

# Issue 0550 — a submodule checked out BEHIND the commit the superproject
# records. Not in `check-fast`: drift is a working-copy state, so the index and
# the commit always agree in anything you can push and a source gate could never
# observe it. It runs as the first item of `check-tier-preconditions`, ahead of
# the CLI stamp, because `git submodule update` rewrites source mtimes and would
# re-stale anything cleared before it. Exposed standalone for diagnosis.
submodule-drift:
    @bash scripts/check-submodule-drift.sh

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

# RFC-0067 D1 / phase-333 W2 — a generated message crate must be referenced as a
# PATH dep, never by registry name. Replaces the interim `check-msg-dep-redirect`,
# which could only assert that SOME `[patch.crates-io]` redirect existed up the
# config chain — a mitigation cargo ignores when it loads config from a different
# cwd (`--manifest-path` from the repo root), the hole issue 0378 called
# unclosable. A path dep has no registry in its resolution at all, from any cwd.
# Also fails on leftover retired message patch entries. Buildless.
[private]
msg-dep-is-path:
    @bash scripts/check-msg-dep-is-path.sh

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

# phase-339 W3 / issue 0433 — no NuttX consumer may link the SHARED live kernel
# tree. Both arches build in one in-tree checkout, so `staging/` belongs to
# whichever built last; a consumer that links it silently stales the OTHER
# architecture's fixtures. Consumers resolve the per-arch export snapshot
# through `nros_board_common::nuttx_export` instead. Source grep, buildless.
[private]
nuttx-links-snapshot:
    @bash scripts/check-nuttx-links-snapshot.sh

# issue 0445 / 0442 — every freshness probe shares ONE exemption rule and ONE
# verdict. A staleness verdict is absorbing: the fixture never launches, so the
# runtime result it would have produced is replaced by a self-explaining
# message. Issue 0444 hid behind 0442 (an exemption applied on one probe arm
# and not its sibling) for exactly as long as those cells read STALE.
[private]
staleness-probe-exemptions:
    @bash scripts/check-staleness-probe-exemptions.sh

# issue 0460 — a capability's service count must match the slots the executor
# sizing reserves for it. The counts live in `executor_sizing` (only a crate the
# proc-macro depends on can supply a const at expansion time) while the services
# live in `nros-node`, which does not depend on it — so nothing in the type
# system ties them together.
[private]
capability-slot-counts:
    @bash scripts/check-capability-slot-counts.sh

# issue 0460 — a knob Kconfig forwards must be read by the Rust lane too. The
# cmake `set(ENV{...})` exports reach the C lane's re-baked command and NOT
# zephyr-lang-rust's `rust_cargo_application`, so every Zephyr Rust image
# compiled its crates' defaults whatever Kconfig said — silently, for every
# knob at once.
[private]
kconfig-knob-forwarding:
    @bash scripts/check-kconfig-knob-forwarding.sh

# issue 0440 — a leaf that deploys to a board must carry that board's STATIC
# link args. `nros-board.toml`'s `cargo_config` is the SSoT (RFC-0032 third
# leg), the leaf `.cargo/config.toml` is TRACKED and `nros sync` leaves it
# alone, so the two drift by hand. phase-338 W2's `-entry` collapse kept the
# node package's config and dropped the whole `-l<kernel lib>` group — valid
# TOML, happy cargo, and every NuttX Rust entry failing at LINK time.
[private]
board-cargo-config-applied:
    #!/usr/bin/env bash
    # phase-341 W4 — REPLACED. The old gate (a shell script of this name,
    # deleted 2026-08-21 once nothing invoked it) asked whether a leaf still
    # carried a REPRESENTATIVE `-l` arg copied from its board, because the leaf
    # mirrored the descriptor BY HAND. It caught a lost GROUP (issue 0440) but
    # not a lost argument, and it covered 8 leaves of the 59 that carried a
    # board block.
    #
    # The block is now a generated projection, so the question changes from "did
    # a human copy enough of it" to "is the committed file what the descriptor
    # renders to" — an EXACT comparison, which makes drift uncommittable rather
    # than detectable. `nros ws check-board-projections` answers it by sharing
    # the renderer with `nros sync`; a second implementation in shell is the
    # drift this phase exists to remove.
    set -euo pipefail
    nros="packages/cli/target/release/nros"
    # ABSENT and STALE are one verdict: this lane is contractually source-free
    # and CLI-free, so it cannot demand a CLI, and asserting against a binary
    # built from other sources is worse than not asserting. One restaled stamp
    # used to surface as three red gates naming one remedy.
    # shellcheck source=scripts/build/cli-usable.sh
    source scripts/build/cli-usable.sh
    if ! nros_cli_usable "$nros"; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip board-projections "$nros_cli_unusable_reason"
        exit 0
    fi
    fail=0
    while IFS= read -r cfg; do
        leaf="$(dirname "$(dirname "$cfg")")"
        NROS_REPO_DIR="$PWD" "$nros" ws check-board-projections "$leaf" >/dev/null 2>&1 || {
            NROS_REPO_DIR="$PWD" "$nros" ws check-board-projections "$leaf" 2>&1 | sed 's/^/  /' >&2
            fail=1
        }
    done < <(git ls-files '*/.cargo/nros-board.toml')
    [ "$fail" = 0 ] || exit 1
    echo "board projections OK ($(git ls-files '*/.cargo/nros-board.toml' | wc -l) leaf/leaves match their descriptor)"

# phase-330 W7.e — committed SystemModels are BANNED: the model is a build
# artifact (generated into <ws>/build/nros/models by `nros sync`); tracking
# one re-opens the issue-0380 hand-edit/regeneration conflict. Supersedes
# check-model-dims (W5.b: the dim baseline protected committed files that no
# longer exist; `nros ws model-dims` remains for inspection).
# issue 0571 — a matrix consumer the lane filter cannot reach by NAME must
# narrow its own cell list (`nros_tests::lane_scope::admits`). Four consumers
# are ONE generically-named test each over every platform's cells, so
# `scripts/test/lane-filter.sh native` excludes nothing of theirs: tier 1 boots
# whatever images exist, and the cells whose images do not exist vanish into a
# green. Buildless.
# phase-351 W5 — every cargo target cmake creates must receive the resolved
# board facts + site config. The failure this guards is NOT a wrong value but no
# value, defaulted, with no diagnostic (issue 0529's shape) — which is how the
# board rung stayed dead from phase-290 to here. Buildless.
# issue 0606 — every `[deploy.*].board` resolves to exactly ONE descriptor. The
# field carries the DOWNSTREAM ecosystem's board id, so the descriptor covering
# it must claim that spelling; otherwise the deploy resolves to nothing and
# Phase 380 W0 — every COMMITTED generated message must carry `Message::FIELDS`.
#
# The schema is what `nros_serdes::size` computes a serialized-size bound from,
# and a missing one is SILENT: phase-380 W4's build-time buffer assertion has
# nothing to check and passes, so the subscription drops samples at runtime
# instead. Measured before W0: 27 of 64 structs had none, purely because three
# crates predated the schema emitter and nobody said so.
[private]
generated-schema-coverage:
    @python3 scripts/check-generated-schema-coverage.py

# issue 1040 — `check-api-parity` sat in `just check`'s DEFAULT list and ran in
# NO workflow at all: not pull_request, not merge_group, not nightly, not
# dispatch. Its reds were therefore invisible between local full-tier runs, and
# three landed on main in a single day alongside four others.
#
# The bar is a DAILY signal, not a blocking one — `check-build` is
# schedule/dispatch-only for good reason and must stay that way.
[private]
default-gates-run-somewhere:
    @python3 scripts/check-default-gates-run-somewhere.py

# `nros sync` skips the leaf with a count instead of a name. Buildless.
# RFC-0087 D4 / phase-420 W5 — a descriptor carries only what no convention can
# produce, so a field it DOES state must equal the value convention derives.
# Six of eleven rmw fields were convention written longhand; five are now gone
# and this keeps the rest honest. A ratchet: `crate` is grandfathered for
# `nros-rmw-xrce`, which ships no Cargo.toml and names a sibling — the
# measurement that showed D4's own table was wrong about `crate` being
# derivable. Buildless, ~0.2 s.
[private]
derived-descriptor-fields:
    @python3 scripts/check-derived-descriptor-fields.py

[private]
deploy-board-resolves:
    @python3 scripts/check-deploy-board-resolves.py

[private]
board-facts-delivery:
    @python3 scripts/check-board-facts-delivery.py

[private]
lane-scope-consumers:
    @python3 scripts/check-lane-scope-consumers.py

[private]
no-tracked-models:
    @bash scripts/check-no-tracked-models.sh

# issue 0359/0378 — `--locked` is injected project-wide by the scripts/bin/cargo
# PATH shim (cargo has no config/env knob for it, and per-site flags would miss
# cmake/corrosion, which invoke `cargo` by name). This asserts the mechanism is
# still wired; without it every build silently rewrites Cargo.lock. Buildless.
[private]
cargo-locked:
    @bash scripts/check-cargo-locked.sh

# Issue 0452 — cbindgen's requirement must stay EXACT and inherited, because the
# headers it writes are COMMITTED and it rewrites them in place on every build.
# The root lock does not govern the leaves that regenerate them (an example's
# lock is never committed), so a caret req is how 0.29.4 output lands in a tree
# whose lock says 0.29.3. Buildless.
[private]
cbindgen-pin:
    @bash scripts/check-cbindgen-pin.sh

# Issue 0525 — NuttX is built IN PLACE and one checkout serves both arches, so
# `$NUTTX_DIR/include/nuttx/config.h` belongs to whichever arch was configured
# LAST. A compile input taken from there silently gets the other arch (issue
# 0511: the ARM image linked with the RISC-V memory map). Buildless.
[private]
nuttx-shared-tree-headers:
    @python3 scripts/check-nuttx-shared-tree-headers.py

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

[private]
nuttx-libc-struct-sizes:
    @python3 scripts/check-nuttx-libc-struct-sizes.py

# Issue 0586 — the C++ FFI must not discard a backend error in favour of
# `-100 TRANSPORT_ERROR`, the code its own source documents as the catch-all for
# UNMAPPED variants. Also holds the two mappers exhaustive, so a NEW variant
# fails to compile until someone maps it (issue 0557 is what the collapse cost).
[private]
cpp-ffi-error-mapping:
    @python3 scripts/check-cpp-ffi-error-mapping.py

# A submodule pin may only move FORWARD. Every submodule here keeps linear
# history on its branch, so a bump is a fast-forward to a descendant; a rewind
# silently unships whatever the skipped commits fixed, and `-Subproject commit
# <hex>` is not something a reviewer can order by eye. 2026-08-15: a 24-file
# commit about issue-ID renumbering moved zenoh-pico BACK over a Zephyr
# `socklen_t` build fix, and nothing noticed for seven hours.
# Deliberate rollback: NROS_ALLOW_SUBMODULE_REWIND=1 (and say why).
# Every tier RFC-0061 declares must be RUN by some workflow. Two of four were
# invoked by nothing at all, and a tier nobody runs cannot go red to say so
# (phase-411).
[private]
tier-has-ci-owner:
    @python3 scripts/check-tier-has-ci-owner.py

# A `-z "${VAR:-}"` guard on a variable sdk-env.just always exports can never
# fire. Six did, and they broke the UNUSUAL way: an intended skip became a cmake
# failure on an unprovisioned host (issue 0955).
[private]
sdk-guard-can-fire:
    @python3 scripts/check-sdk-guard-can-fire.py

# CI may not set NROS_FIXTURES_OPTIONAL. A lane names its scope, so a missing
# fixture INSIDE that scope is a failure; the flag turned it into a skip and made
# correctness depend on remembering to unset a variable (phase-411 W4). The
# reader survives as a local opt-in — the rule is about WHERE it is set.
[private]
ci-no-fixture-tolerance:
    @python3 scripts/check-ci-no-fixture-tolerance.py

# A fixture stamp may not claim what the build did not achieve. The stamp used
# to record the lane's NOMINAL coordinates, so a module that skipped a missing
# prerequisite still read as covered and the run skipped its tests to a green
# sweep (phase-411 W1).
[private]
fixture-stamp-honesty:
    @python3 scripts/check-fixture-stamp-honesty.py

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

# Issue 1018 -- the `generated/` regeneration stamp keys on what the CLI EMITS.
#
# `scripts/build/codegen-stamp.sh` decides whether a leaf's cached `generated/`
# tree survives. It watched `nros-core/src/action.rs` and nothing about the
# emitters, so an edit to `rosidl-codegen` left the tree in place -- and in
# `just/zephyr-ci.just`, the one lane whose `nros sync` is conditional, the leaf
# then compiled message crates the PREVIOUS CLI emitted.
#
# The term added is `nros codegen-fingerprint`, not the binary: 168 distinct
# `nros` binaries on this host produced 11 distinct fingerprints, so a
# binary-keyed stamp would wipe every leaf on 157 rebuilds that emit identical
# code (phase-424's constraint). Both halves are asserted, because either alone
# has a trivial wrong implementation, and the pre-fix rule runs on the same
# trees as the negative control.
[private]
codegen-stamp-inputs:
    @./tests/codegen-stamp-tests.sh

# Every CI job gated on a `vars.*` interlock must have a job that reports
# whether it ran. A skipped job does not colour its run, so post-submit reported
# SUCCESS on runs where the tier-2 matrix — every fixture build and every E2E
# cell in that lane — had not executed at all.
[private]
interlock-visibility:
    @python3 scripts/check-interlock-visibility.py

[private]
submodule-pins:
    @bash scripts/ci/submodule-pins-check.sh

# zenoh-pico's `config.h` is BOTH a cmake-generated artifact (configure_file
# writes it into the SOURCE tree) and a committed, hand-maintained file that is
# the only config nano-ros reads. Being both is how it drifted three ways at
# once — dropped `#ifndef` guards that made every `-D` inert, three of our own
# features missing from the header, and a `#ifdef ZENOH_NUTTX` split the
# template could not express. Phase 415 made generation lossless; this keeps it
# so. Same shape as RFC-0054's committed bindgen output plus
# `check-abi-bindings`. Offline: no cmake, no configure.
# `ros2-box-sync.sh` excludes build-output directories by NAME, and the tree has
# tracked SOURCE whose names match. That collision has eaten tracked files FOUR
# times, each found as a build failure inside the box naming something else —
# most recently `packages/cli/build-support/`, which left the box unable to
# compile `nros` at all. Anchoring fixed one instance and directory-only
# patterns fixed another; neither fixed the class.
[private]
box-sync-covers-tracked-source:
    @python3 scripts/check-box-sync-covers-tracked-source.py

[private]
zenoh-config-template:
    @python3 scripts/check-zenoh-config-template.py

# phase-206 W3 — the zpico run-time config-key table vs zenoh-pico's config.h.
#
# `zp_config_insert(config, Z_CONFIG_<X>_KEY, value)` is zenoh-pico's PRIMARY
# configuration method and its only one — the pico client has no config-file
# format (JSON5/YAML belongs to `zenohd`). So the name -> key map in `zpico.c`
# is the entire configuration surface, and it was a HAND-WRITTEN closed list
# covering 10 of the 23 keys upstream defines, with the other 13 unreachable
# and an unknown name silently dropped. The list is derived from config.h now;
# this gate is what keeps it derived across an upstream bump.
#
# Skips rather than fails when the zenoh-pico submodule is absent: `check-fast`
# must stay green on a pristine detached worktree (issue 0650).
[private]
zpico-config-keys:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f packages/rmw/zenoh/zpico-sys/zenoh-pico/include/zenoh-pico/config.h ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip zpico-config-keys \
            "zenoh-pico submodule not checked out (git submodule update --init packages/rmw/zenoh/zpico-sys/zenoh-pico)"
        exit 0
    fi
    python3 scripts/gen-zpico-config-keys.py --check

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

# Issue 0589 — `std::println!`/`eprintln!` from a crate Zephyr links does not
# print on native_sim, it recurses in `zvfs_write` until the stack is gone. Go
# through `cpp_diag!`, which uses std stdio only where that is safe.
[private]
rust-stdio-on-zephyr:
    @python3 scripts/check-rust-stdio-on-zephyr.py

# Issue 0452 — the committed cbindgen headers must match a fresh generation.
# The Rust->C mirror of `check-abi-bindings`, which has guarded the C->Rust
# direction since RFC-0054. Builds no longer regenerate these in place, so
# without this gate a stale header could sit in the tree indefinitely.
[private]
cbindgen-headers:
    @cargo run -q -p nros-cbindgen-headers -- --check

# A cargo profile defined in BOTH `Cargo.toml` and `.cargo/config.toml` must
# agree. Both are load-bearing — the manifest one applies to the root
# workspace, the config one to the ~48 leaf crates outside it — so editing one
# silently gives half the tree different optimization settings, with no error
# anywhere. Buildless.
[private]
cargo-profile-mirror:
    @bash scripts/check-cargo-profile-mirror.sh

# phase-336 — no build site may NAME a cargo profile: the flag and the artifact
# path both come from `nros profile`. The failure this prevents is silent (the
# builder writes one directory, the reader looks in another), which is why it
# is a gate and not a convention.
[private]
build-profile-literals:
    @bash scripts/check-build-profile-literals.sh

# Issue 0320 — committed SystemModels must be portable: no absolute host paths in
# `meta.inputs[].path`. Buildless; regenerate with `nros sync`.
[private]
no-absolute-model-paths:
    @bash scripts/check-no-absolute-model-paths.sh

# Issue 0332 — nros-cpp public headers must not include a hosted STL header
# (`<string>`/`<vector>`/…) outside an `#ifdef NROS_CPP_STD` region. Source-level
# gate: the `-ffreestanding` compile probe runs against the host's full
# libstdc++ and cannot see the 0112 class.
[private]
cpp-freestanding-includes:
    @bash scripts/check-cpp-freestanding-includes.sh

# Phase 247 W2 (issue 0050) — fast source-level weak-symbol gate: fail when an
# owned C/C++/asm file outside the audited allowlist
# (scripts/weak-symbols-allowlist.txt, shared with weak_symbol_audit.rs)
# introduces a weak symbol, or a listed file's count drifts. Buildless + fast.
# The deeper image gate is `just check weak-symbols-image` (needs fixtures).
[private]
weak-symbols:
    @bash scripts/check-weak-symbols.sh

# Phase 176.3 — verify the orchestration generator's PlatformProfile
# board-crate references match the actual board crates (existence +
# `run` entry). Skips when the colcon-nano-ros submodule is absent.
[private]
profile-board-mirror:
    @bash scripts/check-profile-board-mirror.sh

# Phase 247 W1 (issue 0050) — image-level weak-symbol gate: assert each
# override-default weak symbol is STRONG-overridden in the final linked images
# (firmware ELFs / executables), not silently left weak. Needs prebuilt
# fixtures (skips covered classes whose artifacts are absent) — NOT in the fast
# `check` aggregate; run after the fixture build / in the per-platform CI lanes.
# The fast source-level half is `weak_symbol_audit.rs` (in `just test`).
weak-symbols-image:
    @bash scripts/check-weak-symbols-image.sh

# Phase 118.I.5 — keep collapsed examples from regrowing a retired RMW
# directory axis without an explicit documented carve-out.
[private]
example-matrix:
    @bash scripts/check-example-matrix.sh

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

# issue 1094 — `runner-sweep` must evict the least recently WRITTEN entry, not
# the one with the oldest directory inode. A cargo `--target-dir` writes several
# levels down, so a shared fixture group's dir mtime freezes near creation while
# the group is used daily: eleven of twenty `cargo-fixtures` children were wrong
# by ~20 days, and the sweep proposed deleting the 12.8 GiB NATIVE group — used
# the previous day — to reclaim a 60 MiB overrun. Silent either way; the cost
# shows up as the next job rebuilding a fixture nobody touched. ~1 s, tmpdir.
[private]
runner-sweep-eviction-order:
    @bash tests/runner-sweep-eviction-order-tests.sh

# Every `docs/{design,issues}/NNNN-*.md` path written anywhere — prose, issue
# frontmatter, or a cmake error message — must resolve. Renumbering on an id
# collision is what breaks these.
doc-refs:
    @bash scripts/check-doc-refs.sh

# Every relative link in the book must resolve. `check-doc-refs` answers a
# different question — "does issue 0123 exist?", by ID, and deliberately
# resolves through `archived/` — so a book link naming the pre-archival PATH
# keeps it green while the rendered page 404s. Nine links were dead this way;
# seven were plain depth errors (`../../docs/…` from `book/src/<dir>/` is
# `book/docs/…`). Buildless: reads tracked files, resolves paths, no mdbook.
[private]
book-links:
    # Two chapters are GENERATED and no longer tracked, so on a fresh clone they
    # do not exist and every link to them reads as dead. `just book` and the
    # docs workflow already generate them before mdbook; this gate has to as
    # well, and it cannot rely on `check-{pool-inventory,config-surface}`
    # running first — `check-fast` runs gates in PARALLEL, so that would be a
    # race that passes locally and fails in CI, or vice versa.
    @python3 scripts/gen-pool-inventory.py >/dev/null
    @python3 scripts/gen-config-surface.py >/dev/null
    @python3 scripts/check-book-links.py

# The book's USER track (getting-started/, user-guide/, start-here/,
# platform-guides/) teaches no `just` — it is a contributor dependency a user
# does not have (phase-368 W12). An invocation is allowed only inside an
# explicitly contributor-marked block ("**Contributors (…):**" within 8
# lines). Buildless: tracked files + regex, no mdbook.
[private]
book-no-just:
    @python3 scripts/check-book-no-just.py

# Tool-emitted messages must not prescribe a bare `just` recipe — users do
# not have it; the front door covers both binaries. Found live: the exact
# error a fresh user hits first said `just setup-launch-resolve`, and a
# threadx board error named `just setup-threadx`, a recipe that does not
# exist. Same class as check-book-no-just, one layer down.
[private]
emitter-just-spelling:
    @bash scripts/check-emitter-just-spelling.sh

# The "Open issues" list in docs/issues/README.md must name EXACTLY the files in
# docs/issues/. The rule is already written in that file's Conventions #3 —
# nothing enforced it, and it drifted twice in two consecutive pulls (#0465,
# #0474): each was archived with a `git mv` while its README row stayed in the
# OPEN spelling, so the index advertised an open issue whose file was gone.
[private]
issue-index:
    @python3 scripts/gen-issue-index.py --self-test
    # `docs/issues/open.md` is a gitignored ARTIFACT since 2026-09-03, so there
    # is no committed copy to compare against and `--check` has nothing to be
    # stale against. REGENERATE, then verify — which is the stronger order
    # anyway: `check-issue-index.sh` derives the file list INDEPENDENTLY and
    # compares it to the rows, so the two computations still have to agree. What
    # is gone is only the drift class that existed because the list was
    # committed and hand-editable.
    @python3 scripts/gen-issue-index.py
    @bash scripts/check-issue-index.sh

# Issue 0498 — a sync-owned file a CONCURRENT process reads must be written
# temp + `rename(2)`, never `fs::write` (which truncates to zero, then fills).
# `cmd/ws.rs` already had a private `atomic_write` whose doc called it "the
# discipline every other sync-owned file here uses"; the metadata sidecar one
# directory over had three plain `fs::write` writers and died mid-sweep on an
# empty read. Buildless — greps the guarded function bodies.
[private]
atomic-sync-writes:
    @bash scripts/check-atomic-sync-writes.sh

# Issue 0617 — an RTOS `platform-*` feature must SUPPLY malloc and panic. A
# no_std final artifact needs exactly one `#[global_allocator]` and one
# `#[panic_handler]`; a HOST build cannot catch a missing provider because
# `std` supplies both, which is how NuttX shipped with neither.
platform-provider-features:
    @python3 scripts/check-platform-provider-features.py

# phase-365 W4 / issue 0625 — the SDK store is CONSTRUCTED from the project's
# pin, never enumerated. A wildcard where the version belongs means a consumer
# is picking a version by searching a store SHARED between projects, to answer a
# pin that is PER-PROJECT.
sdk-store-not-enumerated:
    @python3 scripts/check-sdk-store-not-enumerated.py

# Issue 0454 / phase-354 W3 — an FFI taking `goal_cdr` must strip the
# encapsulation header before handing bytes to `send_goal_raw`, which takes
# FIELDS and appends them after a header the writer already wrote.
goal-cdr-stripped:
    @python3 scripts/check-goal-cdr-stripped.py

# Issue 0580 — a test must ASSIGN its ROS domain, never name one: a literal is a
# shared bus, and two concurrent runs colliding on it presents as WRONG DATA
# rather than as a collision.
test-domain-assignment:
    @bash scripts/check-test-domain-assignment.sh

# Issue 0763 — a test's ROS 2 environment is spelled in ONE place
# (`nros_tests::ros_env::RosEnv` + `Middleware`, RFC-0058 / phase-309, over the
# `ros2::ros2_env_setup_*` helpers). A hand-rolled `source /opt/ros/<distro>/
# setup.bash && export ...` elsewhere is a second place, and every second place
# has drifted invisibly: one dropped the peer's ROS_DOMAIN_ID (the first segment
# of an rmw_zenoh keyexpr, and the key ros2cli's daemon is singleton on),
# another opened with a `ros2 daemon stop` that killed the daemon a parallel
# test was mid-query against, another hardcoded `humble` so every guarded test
# SKIPPED forever on a jazzy host. The gate does not forbid the exception — it
# makes adding one a reviewable line in a diff. Buildless: greps tracked source.
[private]
ros-env-spelling:
    @python3 scripts/check-ros-env-spelling.py

# Issue 0573 — ZenohRouter must stay the only zenohd spawner.
zenohd-spawn-sites:
    @bash scripts/check-zenohd-spawn-sites.sh

# Issue 0670 — a timed-out wait's evidence must not be thrown away.
#
# `wait_for_output*` returns `Err` carrying what the process PRINTED;
# `.unwrap_or_default()` replaces it with `""`, so the assertion reports `got:`
# with nothing after it. `contract_monitor_parity` failed exactly that way, and
# the empty `got:` is why its real cause (issue 0671 — an unguarded
# `epoch_us_fn` clobber leaving the age monitor with no clock) took a separate
# investigation instead of being readable off the failure.
#
# A gate rather than a sweep, because the obvious mechanical fix is WRONG:
# `unwrap_or_else(|e| e.to_string())` folds in text that NAMES the pattern being
# waited for, so `seen.contains(<pattern>)` matches the complaint about the
# missing pattern and the test passes exactly when it should fail. Each site
# needs its assertion read. `collect_until_count` returns the two on separate
# channels for that reason.
#
# 87 sites are baselined as a SHRINKING backlog; what this buys today is that an
# eighty-eighth cannot arrive silently.
wait-evidence-discarded:
    @python3 scripts/check-wait-evidence-discarded.py

# Issue 0653 — the shell and Rust router resolvers must resolve the SAME router.
#
# `just <plat> zenohd` is shell and the test harness is Rust; neither can call
# the other, so `zenohd.sh` carried "the two must agree" as a comment and they
# drifted regardless — both searching only `/opt/ros` while `AMENT_PREFIX_PATH`
# is what a sourced ROS actually announces. On a ROS built from source, or this
# repo's own Arch/Fedora/NixOS distrobox route, that is not `/opt/ros`: you could
# source a working ROS and be told there is no router.
#
# The answer to two implementations is one TABLE
# (`scripts/dev/zenohd-resolution-cases.tsv`), answered here for the shell and by
# `zenohd_resolution_matches_the_shared_table` for the Rust — behaviour on both
# sides, not a diff of two languages.
zenohd-resolution-parity:
    @bash scripts/check-zenohd-resolution-parity.sh

# Issue 0654 — the router takes NO command-line configuration, so a `zenohd
# --listen …` line is not merely a stale name: the flags are UNREAD rather than
# rejected, and the reader gets a default-configured router with no diagnostic.
# Gated because the class regenerates by copy-paste from a neighbouring example
# header — which is how it reached ~95 files.
# Issue 1039 — a port may declare its own platform to zenoh-pico, never
# somebody else's. `nros-platform-nuttx` defined ZENOH_LINUX to reach the
# `getrandom()` arm; the arm was right and the name was not, and the claim
# then became load-bearing in three unrelated guards. Scans BOTH producers
# (the platform manifests and `runner.rs`'s `build.define`), and runs its own
# negative control first.
zenoh-platform-macros:
    @python3 scripts/check-zenoh-platform-macros.py

zenohd-flag-invocations:
    @python3 scripts/check-zenohd-flag-invocations.py

# phase-363 (W2's class) — `file(GLOB)` over `.msg`/`.srv`/`.action` captures the
# interface set at CONFIGURE time, so a newly added message is invisible until
# an unrelated reconfigure and the build ships the OLD generated sources. W2
# fixed the file it was looking at; the Zephyr COPY kept the bug until the
# phase's standing re-sweep found it. Gated so there is no third occurrence.
interface-glob-configure-depends:
    @python3 scripts/check-interface-glob-configure-depends.py

# Issue 0466 — report EVERY unmet tier precondition at once (CLI stamp, leaf
# includes, build sources, fixtures for the lane) instead of one per ~40-minute
# attempt. Run at the head of `ci`; callable directly to check a tree before
# committing to a run.
tier-preconditions:
    @bash scripts/check-tier-preconditions.sh

# phase-379 W5 — `nros::prelude` is the rclrs-shaped API, `nros::embedded` is
# the RTOS machinery. Membership is a LEDGER QUERY (a name may sit in the
# prelude only if the parity ledger does not call it an `extension`), plus an
# argued allow-list for the extensions a node cannot start without.
[private]
prelude-tiers:
    @python3 scripts/check-prelude-tiers.py

# `book/src/reference/rmw-api-comparison.md` is GENERATED from the ROS 2
# signature extract, the nano-ros ABI headers, and the same reason map
# `rmw-api-parity` enforces — so the book cannot drift from the headers. A
# hand-maintained comparison describes the tree on the day it was written;
# phase-379's own doc claimed a missing shutdown hook months after it shipped.
[private]
rmw-api-comparison:
    @python3 scripts/gen-rmw-api-comparison.py --check

# Every lane that runs the peer-spawning suites sweeps first (issues 0659,
# 0923). The reaper's own unit tests call `sweep_in` directly, so they cannot
# see whether a LANE calls it — `just test` ran the same suites without one and
# left 67 orphans holding DDS ports.
[private]
peer-sweep-lanes:
    @bash scripts/check-peer-sweep-lanes.sh

# issue 1103 — a phase SECTION whose declared `**Files:**` all exist while its
# boxes stay open. `check-roadmap-claims` beside this compares a document against
# ITSELF (status text vs box counts, an `**Owns:**` link vs an issue's
# frontmatter); nothing asked whether the work described had ARRIVED. phase-215
# read 20 of 38 boxes while 27 were done — a P1 phase advertising seven finished
# items as available.
#
# SECTION level, not box level, and that was measured rather than assumed: the
# per-box rule ("an unticked box naming a path that exists") catches 2 of those
# 7 and fires on 2 false positives tree-wide, because most boxes name a variable,
# a function, a recipe, or prose. The section rule catches 3 of the 4 relevant
# sections with zero findings on main. Ratcheted; `--write-baseline` to accept.
[private]
roadmap-boxes:
    @python3 scripts/check-roadmap-boxes.py

# phase-419 W2 — an active phase must not contradict its own body. Sibling of
# `roadmap-status`, which asserts a status EXISTS and deliberately stops there
# ("staleness of the CONTENT is a human call"). That is right for supersession
# and wrong for a header that says "nothing landed" over ticked work items —
# the document contradicts ITSELF, and no judgment is involved. Ratcheted;
# the judgment half is phase-419 W3, a report and never a gate.
[private]
roadmap-claims:
    @python3 scripts/check-roadmap-claims.py

# Every ACTIVE roadmap phase carries a findable status line; a finished one
# belongs in `docs/roadmap/archived/`. A one-off pass does not hold — four
# phases lost theirs within days of `ecc195ed6` doing exactly that.
[private]
roadmap-status:
    @bash scripts/check-roadmap-status.sh

# A leaf `.cargo/config.toml` is tracked iff it holds content `nros sync`
# cannot regenerate. `**/.cargo/config.toml` is gitignored (most are pure sync
# output); this is the discrimination the blanket rule cannot make.
cargo-config-tracked:
    @bash scripts/check-cargo-config-tracked.sh

# A leaf that consumes its own `generated/` must be visible to
# `scripts/regenerate-bindings.sh` (which globs tracked package.xml), or its
# bindings freeze silently — eight test bins had frozen several phases back.
generated-leaf-regenerable:
    @bash scripts/check-generated-leaf-regenerable.sh

# Issue 0406 — a fixture builder narrowed to an id it cannot match must FAIL,
# not exit 0 having built nothing. Buildless: exercises the shared guard and
# one real builder invocation that stops before any compilation.
fixture-id-guard:
    @bash scripts/check-fixture-id-guard.sh

# Phase 134.5 — verify the in-tree zenoh staticlib's internal symbol
# parity. For every defined `_z_f_link_*_<transport>` wrapper, the
# matching `_z_*_<transport>` impl must also be defined. Pre-Phase-134
# the POSIX CMake path shipped wrappers without multicast impls and
# every C/C++ native link broke. Run after
# `cargo build -p nros-rmw-zenoh-staticlib --release`.
[group("debug")]
zenoh-archive:
    # profile-literal-ok: symbol fixture: the archive built by build-zenoh-posix-fixture
    @bash scripts/check-zenoh-archive-symbols.sh target/release/libnros_rmw_zenoh_staticlib.a

# Phase 104.A.4 — assert `nros` + `nros-node` Cargo deps stay free of
# concrete RMW / platform crates. The umbrella must consume only the
# generic ABI (`nros-rmw-cffi` vtable + `nros-platform-cffi` C header);
# selecting a backend or platform is the outer build system's job.
#
# Today this guard is EXPECTED TO FAIL — Phase 104.A is the migration
# that brings it to green. Wire it as a required check once the
# migration completes.
[group("debug")]
decoupling:
    @bash scripts/check-decoupling.sh

# A test that can SKIP must be run by a runner that understands skips.
# `nros_tests::skip!` PANICS with a `[SKIPPED]` marker (Rust's harness has no
# native skip), and only `_nextest-tolerant` rewrites it before tallying — so a
# bare runner turns an unmet precondition into a red naming a test the change
# never touched.
[private]
skippable-tests-tolerant:
    @python3 scripts/check-skippable-tests-tolerant.py

# A tier's affordability claim must be TRUE. `ci-l1` says "NO FIXTURES"; it
# reached a fixture-resolving test for as long as anyone had written the claim
# down, and nothing noticed until that lane became a REQUIRED status check and
# CI went permanently red.
[private]
lane-contracts:
    @python3 scripts/check-lane-contracts.py

# nano-ros is PUBLIC and the self-hosted runners share machines with unrelated
# research work. A self-hosted job reachable from a fork's `pull_request` runs
# contributor code on that hardware. The property held by care alone until
# phase-407 — this makes a `runs-on` typo fail loudly instead of silently.
[private]
workflow-runner-isolation:
    @python3 scripts/check-workflow-runner-isolation.py

# Issue 0975 — a REQUIRED check that cannot report on a pull request freezes the
# repo, and it freezes it SILENTLY: no red check, no message, just PRs that never
# merge. Requiring `L3` (a merge_group-only job) did exactly that to seven PRs.
# Also rejects a required job gated on `vars.`/`secrets.`, which can be switched
# off in Settings with nothing in any diff to show it.
[private]
required-contexts-reportable:
    @python3 scripts/check-required-contexts-reportable.py

# A gate must exercise its own failure path EVERY time it runs, not once behind
# a flag on the day it was written. A baseline ratchet: the debt may only
# shrink, and a script that gains a selftest must leave the baseline.
# Issue 0993 — a gate no pull_request or merge_group job runs is a gate that
# rots. `just check build` is invoked once, in `gate.yml`, behind a
# `schedule`/`workflow_dispatch` guard, so nothing in it gates a merge and a red
# there is invisible until somebody reads a nightly (issue 0981: a red sat on
# `main` for a day behind a green required `CI`, and two changes reached a green
# pull request while failing `just ci gate` locally).
#
# The ungated set is written down in `.config/ungated-gates.txt` and may only
# SHRINK — same ratchet as the selftest baseline. Membership is TESTED, not
# argued: a pristine worktree, then `just check <gate>`. Twenty gates passed
# that with no build artifacts and moved to this line; the nineteen that remain
# each fail it for a stated reason.
[private]
gate-visibility:
    @python3 scripts/check-gate-visibility.py

[private]
gate-selftests:
    @python3 scripts/check-gate-selftests.py

# A `ps` scan enumerating process-GROUP membership must exclude zombies — a
# corpse keeps its pgid and, under a PID 1 that never reaps (a GitHub Actions
# container job: `tail -f /dev/null`), never leaves `ps`. Issue 0853: three
# sites had the idiom and all three were wrong.
[private]
ps-zombie-blind:
    @bash scripts/check-ps-zombie-blind.sh

# A PX4 module compiles against a per-build generated header and links a
# separately-built `libnros_cpp.a`. The two are independent files that each
# outlive the build that wrote them, so nothing made them agree — issue 1050's
# link wall (`undefined reference to nros_cpp_config_variant_...`) is that
# disagreement, found ~1100 targets into a ten-minute build. Issue 1046 is why
# the guard is on CONTENT: the predicate it replaced asserted a DIRECTORY, which
# survives the build it stood for, so it passed on exactly the tree it existed to
# reject. This gate runs the real cmake predicate against fixtures in BOTH
# directions; a guard nobody has seen fail is the bug it is guarding against.
[private]
px4-archive-header-pairing:
    @bash scripts/check-px4-archive-header-pairing.sh

# A CI doc must not cite a workflow file that does not exist. Same class as
# `check-just-recipe-refs`: a reference that silently stops resolving, in a
# place that still reads as current.
[private]
ci-doc-workflow-refs:
    @python3 scripts/check-ci-doc-workflow-refs.py

# The zephyr CI image must install every Python module its own checker demands
# (issue 0878). It did not, so all 21 Zephyr nightly cells failed at
# `just zephyr setup` before any build — and a uniformly red lane cannot report
# a regression, which is how issue 0876 rode in.
[group("checks")]
ci-image-python-deps:
    @python3 scripts/check-ci-image-python-deps.py

# A Kconfig line that a later CONF_FILE fragment overrides is a line that does
# nothing, and nothing said so (issue 0876). A RATCHET, not a prohibition —
# shared board fragments are legitimately an override layer. Refresh with
# `python3 scripts/check-kconfig-overridden-values.py --update`, and say why.
# Every `[prereq.*]` key declares who may NAME it (RFC-0062 amendment 3).
# The Rust field defaults to `unclassified` so it could land incrementally;
# without this gate a new key inherits that default and `nros setup --workspace`
# reports a toolchain as a content dependency — the conflation `role` removes.
# phase-422 W7 — every `deploy=`/`board=` a package.xml exports must resolve.
# `nros setup --workspace` turns those values into a provisioning command, so a
# value meaning nothing becomes a remedy that fails. Five namespaces exist for
# the concept; this does not unify them, it stops a value in none of them.
[group("checks")]
board-vocabulary:
    @python3 scripts/check-board-vocabulary.py

# gate.yml's warm job must cache the SAME key+path as the job it warms.
# GitHub scopes a cache to the ref that created it, so only `main` writes a
# scope pull_request/merge_group can read — a drifted key means the warm job
# succeeds, uploads, and warms nothing, with no error anywhere.
[group("checks")]
gate-cache-keys-agree:
    @python3 scripts/check-gate-cache-keys-agree.py

# The api-parity ledger's schema has ONE home (issue 1095). It used to be copied
# into all 17 shards, so a schema change touched 17 files and concurrent PRs
# conflicted in up to 17 paths without disagreeing about anything — the class
# issues 0883/0884 record, one surface over.
[group("checks")]
ledger-doc-single-home:
    @python3 scripts/check-ledger-doc-single-home.py

[group("checks")]
prereq-roles:
    @python3 scripts/check-prereq-roles.py

# Every tier precondition must have a provisioner, and it must be reachable
# from `just setup <scope>`. `check-tier-preconditions` asserts host facts;
# `_setup-common` provisions them; nothing kept the two lists in agreement, so
# the gaps surfaced one 40-minute CI run at a time (issue: host-tests tier1 ->
# three unmet preconditions -> then clang-format one layer deeper).
#
# Classifies each precondition as setup / build / manual and checks the map in
# BOTH directions, because a map checked one way drifts silently.
[group("checks")]
preconditions-provisioned:
    @python3 scripts/check-preconditions-provisioned.py

[group("checks")]
kconfig-overridden-values:
    @python3 scripts/check-kconfig-overridden-values.py

# Quarantine registry gate — every entry has an open issue and an unexpired
# review date. Expiry is a HARD failure on purpose: quarantine without expiry is
# deletion with extra steps.
[private]
flake-quarantine:
    @python3 scripts/test/quarantine.py --check

# The claim CAS, proven against a throwaway bare repo — hermetic, no network,
# ~2 s. On the fast line because a coordination primitive that cannot fail is
# worse than none: it would report agreement it never established.
[private]
claim-protocol:
    @bash scripts/reserve-claim.sh --selftest

# A COMPILE SMOKE TEST for the pull-request gate — phase-395.
#
# `check-build` compiles the workspace roughly ten times: six clippy feature
# combos, `--all-targets` with `-D warnings`, plus the C/C++/Cyclone/XRCE
# backend lanes. That is 587 s of an 878 s gate, and it is why the compile tier
# moved to the merge group.
#
# But moving it there means a plainly broken pull request is caught by the QUEUE
# rather than by its own gate, where it costs a batch slot and an ejection. This
# compiles the workspace ONCE — no lints, no feature matrix, no backends — to
# catch that class on the PR while leaving the expensive verification batched.
#
# `--all-targets` on purpose: tests and benches are where a signature change
# most often fails to propagate, and they are not built by a bare `cargo check`.
#
# It does NOT replace `check-build` and must never be confused for it: no
# clippy, so no `-D warnings`; no feature combinations, so a break that only
# appears under `--no-default-features` survives this and is caught in the
# queue. It is a smoke test, and the name says so.
[private]
compile-smoke:
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    # The same exclusions `check-test-targets` uses: these crates are `no_std`
    # or build-only and cannot compile for the host — a bare `--workspace` dies
    # on `#[panic_handler] function required` and reports a defect that is not
    # one.
    cargo check --workspace --all-targets --quiet {{HOST_UNCHECKABLE}}
    echo "compile smoke: workspace + all targets check clean (no lints; see check-build)."

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

# phase-359 W0 — the `std` census ratchet. Buildless (reads sources), so it
# belongs in the fast tier next to the other convention gates.
#
# The campaign to drop `std` from the core crates spans 181 cfg sites and 425
# `std::` paths over nine crates. This freezes those counts per crate and FAILS
# when one goes up — including a crate entering scope that the baseline has
# never seen. Counts going DOWN also "fail", loudly and on purpose: the fix is
# to lower the baseline in the same commit, so progress lands in the diff
# instead of a claim in a message.
[group("ci")]
std-census:
    @python3 scripts/check-std-census.py

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

# Issue 0589 — no `std::`-qualified stdio in a `#![no_std]` crate's `src/`.
#
# On Zephyr `native_sim` a Rust `std::eprintln!` does not print, it SIGSEGVs the
# image: `zvfs_write(1, …)` re-enters itself through `stdinout_write_vmeth` and
# `k_mutex` is recursive, so the stack runs out. The print that found this was a
# diagnostic — so an error path got LESS informative the more it said. The Kconfig
# is identical in cells that pass, which makes it latent in every native_sim image
# rather than a bug in any one of them; a gate is the only thing that keeps the
# next `eprintln!` out.
#
# Buildless. `--self-test` drives 16 synthetic trees, including the two false
# readings this gate shipped with and had to fix: a `cfg_attr(not(feature =
# "std"), no_std)` crate read as hosted, and two calls on one line counted once.
[group("ci")]
no-std-stdio:
    @python3 scripts/check-no-std-stdio.py

# issue 0974 — `CONFIG_NROS_CYCLONE_DOMAIN_ID` defaults to `NROS_DOMAIN_ID` so
# the two knobs cannot split; pinning a literal breaks that link silently, which
# is phase-180's split-brain (issue 0161). Both `nros new`'s template and the
# Zephyr getting-started page were emitting the pin, so every generated
# cyclonedds project started split-brained and the book taught the pattern.
[group("check")]
cyclone-domain-not-pinned:
    @python3 scripts/check-cyclone-domain-not-pinned.py --self-test
    @python3 scripts/check-cyclone-domain-not-pinned.py

# issue 0942 — the C++ twin of the rule above. `std::fprintf` in a TU that is
# cross-compiled for a board's own toolchain depends on a guarantee freestanding
# libstdc++ does not make; it built on every hosted target and failed only on
# threadx-riscv64.
[group("check")]
cpp-no-std-stdio:
    @python3 scripts/check-cpp-no-std-stdio.py

# A test whose body only PRINTS cannot fail, so it reports PASS on exactly the
# host it was supposed to warn about. The 2026-08-21 cleanup removed 17 of these
# (10 files, 2 of them literal cross-file duplicates); this keeps the shape from
# growing back. Self-testing: `--self-test` covers the delegating-helper and
# multi-line-print cases that a naive "has no assert!" rule gets wrong.
[group("ci")]
no-vacuous-tests:
    @python3 scripts/check-no-vacuous-tests.py --self-test
    @python3 scripts/check-no-vacuous-tests.py

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

# Issue 0726 — `find_program(VAR ...)` is a NO-OP when VAR is already defined,
# so the "initialise then search" spelling never searches. All three sites in
# this tree paired it with a quiet fallback (FetchContent clone, "no SDK
# libstdc++", host-idlc), so none failed loudly.
cmake-find-program-shadowed:
    @python3 scripts/check-cmake-find-program-shadowed.py --self-test
    @python3 scripts/check-cmake-find-program-shadowed.py

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

# The gate REGISTRIES hold one name per line, sorted — phase-400 follow-up to
# issues 0883/0884.
#
# `fast-serial` and `build` are the two lists every gate-adding PR must append
# to, and they used to pack up to fifteen names per line with new gates going
# on the tail. 22 of the last 31 commits touching this file edited such a line,
# so unrelated gates collided and the merge queue ejected them. 0884's union
# merge is not available for an authored file; one name per line, sorted, is
# the fix that needs no merge driver.
[group("hygiene")]
gate-lists:
    @python3 scripts/check/check-gate-lists.py


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

# phase-366 M6 / RFC-0077 — every image declares exactly one ending, and what it
# declares matches what it supplies. `check-archive-lang-items` counts per LINK
# LINE, which catches duplication and is blind to ABSENCE (an image with no
# provider has no archive to count — issue 0617). Buildless, source-level.
image-panic-policy:
    @python3 scripts/check-image-panic-policy.py

# issue 0719 — the C/C++ half of the same question. `check-image-panic-policy`
# reads what a RUST image declares (`nros::main!(panic = …)`) and says outright
# that it cannot see "the C/C++ side, where the policy is a cargo feature on the
# staticlib". This is that side: every cmake path that links `NanoRos::NanoRos*`
# into an executable must apply an ending, directly or through the entry verbs.
cmake-image-policy:
    @python3 scripts/check-cmake-image-policy.py

# Issue 0636 option 3 — every tier spin loop reaches a scheduling point.
[group("check")]
tier-spin-gap:
    @python3 scripts/check-tier-spin-gap.py

# Issue 0773 — the length-or-status shape, gated.
#
# This script was a REPORT (`just rmw-ret-sign`), on no lane, because its job
# was a one-off migration sweep before W3.d step B flipped the return values.
# The flip then broke a class it could not see — backend-internal helpers
# multiplexing a byte count with an `NROS_RMW_RET_*` code — and nobody was
# running it anyway. It has a structural check now and a non-zero exit, so it
# belongs on the line.
rmw-ret-sign:
    @python3 scripts/check-rmw-ret-sign.py

# Issue 0780 — a POSITIONAL vtable initialiser must agree with the header.
#
# Two backends fill `nros_rmw_vtable_t`'s 70-odd slots positionally, annotated
# only by `/*slot*/` comments that nothing checks. Insert a slot mid-header and
# every line below initialises its neighbour while still carrying the old name.
# The compiler catches that only when the shifted types disagree — which is
# luck: adjacent slots sharing a signature swap silently.
rmw-vtable-order:
    @python3 scripts/check-vtable-positional-order.py

# Issue 0787 — host lanes for the two C backends that had none.
#
# `check-rmw-cyclonedds` has compiled that backend on the fast line for
# phases; xrce and uORB were only ever built by tier 2 fixture builds, so five
# phase-376 signature changes crossed their C ABI seam with no compiler
# looking. Both turned out to be buildable on any host all along:
#
#   xrce  vendors micro-XRCE-DDS-Client + micro-CDR as submodules
#   uORB  defaults `NROS_RMW_UORB_LINK_PX4=OFF` precisely so it builds
#         without the PX4 SDK
#
# So the SDKs were never the blocker. Nobody had written the recipe.
#
# Each skips LOUDLY (counted by `scripts/build/check-skip.sh`) when its
# prerequisite is absent, so "not built here" can never read as "builds".
rmw-xrce:
    #!/usr/bin/env bash
    set -e
    source scripts/build/build-root.sh
    if [ ! -f packages/rmw/xrce/xrce-sys/micro-xrce-dds-client/CMakeLists.txt ]; then
        source scripts/build/check-skip.sh
        nros_check_skip rmw-xrce \
            "micro-XRCE-DDS-Client submodule not initialised (git submodule update --init packages/rmw/xrce/xrce-sys/micro-xrce-dds-client)"
        exit 0
    fi
    BD="$(nros_build_dir "$NROS_KIND_XRCE_CHECK")"
    cmake -S packages/rmw/xrce/nros-rmw-xrce -B "$BD" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BD" --parallel
    ctest --test-dir "$BD" --output-on-failure

rmw-uorb:
    #!/usr/bin/env bash
    set -e
    source scripts/build/build-root.sh
    BD="$(nros_build_dir "$NROS_KIND_UORB_CHECK")"
    # No skip arm: this backend needs nothing but a C++14 compiler. PX4 is
    # opt-in (`NROS_RMW_UORB_LINK_PX4`), and the default OFF build is exactly
    # the surface the RMW ABI changes touch.
    cmake -S packages/rmw/uorb/nros-rmw-uorb -B "$BD" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BD" --parallel
    ctest --test-dir "$BD" --output-on-failure

# Phase 376 — every symbol in the rmw implementation contract is classified:
# a vtable slot, another layer, or a declined RTOS reason. Reads the RECORDED
# contract, so it needs no ROS install; regenerate that with `--contract` in the
# distrobox when the distro moves.
[group("check")]
rmw-api-parity:
    @python3 scripts/rmw-api-parity.py --self-test
    @python3 scripts/rmw-api-parity.py --check

# Phase 376 W5 — the campaign's claim, re-proven per commit.
#
# `check-rmw-api-parity` asks "is every contract symbol CLASSIFIED?"; this asks
# "does the vtable actually MIRROR upstream?" — name, args and return type per
# slot, with every difference declared. Both read committed snapshots
# (`docs/reference/rmw-implementation-{contract,signatures}.txt`), so neither
# needs a ROS install and both run on the fast line.
rmw-abi-shape:
    @python3 scripts/rmw-abi-shape.py --self-test
    @python3 scripts/rmw-abi-shape.py --check

# issue 0777 — seven ABI deviations were justified by "pools are baked", a claim
# true of one backend in five. The conclusion held; the reason did not. This is
# the re-run: every allocation site in a backend, split by whether it is on the
# per-message path or at entity creation. `--check` fails on a steady-state
# allocation with no declared reason, so a new one is argued for rather than
# merged. Source-only, no ROS install, fast line.
rmw-alloc-sites:
    @python3 scripts/rmw-alloc-sites.py --self-test
    @python3 scripts/rmw-alloc-sites.py --check

# issue 0800 — "the slot exists" was reading as "the capability works": 42 slots
# had no producer, mixing optional-by-design with real gaps so no reader could
# tell them apart. This splits by producer AND consumer. A slot nothing writes
# and nothing reads is legitimate — an ABI mirroring upstream reserves the shape
# first — but it must be a DECISION, so it belongs to a declared family. Source
# -only, no ROS install, fast line.
rmw-slot-producers:
    @python3 scripts/check-rmw-slot-producers.py --self-test
    @python3 scripts/check-rmw-slot-producers.py --check

# issue 0804 — `ZenohRouter::start*(...).expect(...)` turns "this host has no ROS
# router" into a test failure. `fixtures::or_skip` is the one reading of
# `RouterUnavailable` (skip), and it keeps every other router error a failure.
# 31 sites went around it, which is why tier 2 reported 7 real failures on a
# host whose ROS lives in a distrobox. Source-only, fast line.
zenohd-router-skips:
    @python3 scripts/check-zenohd-router-skips.py --self-test
    @python3 scripts/check-zenohd-router-skips.py

# Phase 379 — the user API against the ROS 2 client library it mirrors. This is
# the LEDGER gate, not the parity gate: it checks that every ledger row names a
# verdict the tool knows and gives a reason, and that the correlator's own
# matching rules still hold (a defaulted parameter must not read as a
# divergence -- that bug reported all 11 of the C++ lane's first "findings").
#
# Buildless: no clang, no ROS, no nightly. The full comparison needs all three,
# so it is `just api-parity`, not a gate. `--check` joins this recipe in W2,
# once the ledger classifies the ~2000 rows the first report found; wiring a
# gate that fails on all of them today is how a gate gets switched off.
#
# Check the API-parity ledger and the correlator's matching rules.
[group("check")]
api-parity-ledger:
    @python3 scripts/api-parity.py --self-test
    @python3 scripts/gen-api-comparison.py --self-test

# Phase 379 W2 (landed 2026-08-25) — every item where the nano-ros user API does
# not correspond to rclc/rclcpp/rclrs carries a written verdict. 2158 rows across
# 17 topic shards under docs/reference/api-parity-ledger/.
#
# NOT buildless, which is why it is here and not on the fast lane: it re-extracts
# OUR surface every run (clang for C/C++, nightly rustdoc for Rust) so an edit
# that moves us away from ROS 2 fails the gate instead of aging the ledger. The
# ROS 2 side is the recorded surface and needs no ROS install.
#
# A new divergence fails this with the row's key and the bucket it landed in.
# Add a row to the matching topic shard; `--topic <name>` shows the stage.
#
# Verify the user API still corresponds to rclc / rclcpp / rclrs.
[group("check")]
api-parity:
    @python3 scripts/api-parity.py --check

# issue 0734 — a binary links exactly ONE nano-ros Rust staticlib. A staticlib
# bundles its whole dependency closure, so linking two duplicates it — and
# because they are separate cargo builds with different `-C metadata`, the
# duplicated statics do not collide and BOTH get allocated. Sibling if/else arms
# are fine and expected; the gate only flags two umbrellas on ONE branch.
single-rust-staticlib:
    @python3 scripts/check-single-rust-staticlib.py

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

# Issue 0833 — every Rust cross target DECLARED in-tree (board nros-board.toml,
# cmake/toolchain Rust_CARGO_TARGET, leaf .cargo/config.toml [build] target) has
# a row in config/rust-targets.txt, the list both `just workspace rust-targets`
# and `just doctor` read.
#
# The shared list fixes the divergence that existed; this fixes the one that
# would happen next. A board can land a new triple in three different files and
# none of them is the provisioning recipe — armv8r-none-eabihf was declared by
# two boards, one toolchain file and two fixture rows while the doctor's own
# copy of the list had never heard of it, so `just doctor` said [OK] and `just
# freertos build-fixtures` failed at cmake configure with an error four layers
# from its cause.
[group("ci")]
rust-targets-covered:
    @python3 scripts/check-rust-targets-covered.py --self-test
    @python3 scripts/check-rust-targets-covered.py

# Issue 0739 — every build-time sizing knob is enumerable, and the pools that
# declared their arithmetic carry a byte figure. Issue 0271 audited a 256 KB
# image that already tuned NINE of these and still inherited ~145 KB of defaults
# it did not know existed; four separate features had each added a static pool
# with a knob, silently. Generated, not transcribed — a hand-written list is the
# thing that goes stale the first time a feature lands.
[private]
pool-inventory:
    # WRITES the inventory rather than diffing a committed copy: the file is
    # generated and no longer tracked, so `--check` would report "stale" on
    # every fresh clone (a missing file compares unequal to the rendered text).
    # The assertion that mattered is `self_test()` + a scan that evaluates every
    # formula, and both still run here; the artifact the book needs falls out.
    @python3 scripts/gen-pool-inventory.py

# Issue 0934 — one fact, up to nine spellings. Most are CARRIERS (how a decision
# crosses a layer), not decisions; nothing recorded which was which, so a reader
# saw nine ways to set a locator. This enumerates every Kconfig symbol and
# demands each be declared public, carrier or dead. RATCHETED rather than
# absolute: 55 of 74 are unclassified today, and a gate that demanded zero would
# be switched off on day one.
[private]
config-surface:
    # WRITES the page rather than diffing a committed copy: it is generated
    # and no longer tracked, so `--check` would report "stale" on every fresh
    # clone. The assertions that matter (the knob scan and its self-test) run
    # either way; the artifact the book needs falls out.
    @python3 scripts/gen-config-surface.py

# Phase 392 W1 — the declared arithmetic has to survive contact with a linker.
# `check-pool-inventory` proves the FORMULA is present and evaluates; this proves
# it equals the bytes the compiler actually reserved. Source-only here (the
# selftest), because `check-build` runs before any fixture exists; the
# fixture-backed comparison is `static_memory_declared_pools`, which resolves a
# built image through the harness and so inherits the staleness probe.
[private]
mem-report:
    @python3 scripts/nros-mem-report.py --selftest

# Issue 0816 — the book promises no-alloc integrations and nothing verified the
# LINKED image; a Cargo feature gate is necessary and not sufficient, because a
# vendored C dependency or a weak-symbol fallback pulls an allocator in without
# any feature moving. Selftest + claims roster only: BUILDLESS, so it sits on
# the fast line beside the other self-testing generators.
#
# The image-level tiers are deliberately NOT wired into `ci` yet. `--claims`
# reports 0 of 4 book claims backed, because no example is BUILT no-alloc (all
# 13 bare-metal Rust leaves enable `alloc`) — the missing thing is a fixture,
# not a gate, and that is 0816's remaining half. The `--tier unified` object
# check is green on the zenoh funnel and red on a cyclone leaf (`operator new`
# in four TUs), so it waits on a fix or an `--allow` carrying a tracked issue id.
[private]
no-alloc-image:
    @python3 scripts/check-no-alloc-image.py --selftest
    @python3 scripts/check-no-alloc-image.py --claims

# Issue 1052 — the ESP32 stack is the LINKER LEFTOVER after `.bss`
# (`_stack_start - _stack_end`), so every byte a static gains is a byte the
# stack loses, and there is no runtime overflow guard on the target: the image
# writes frames into `.bss` and dies later as a wild jump somewhere unrelated.
# `nros-board-esp32-qemu/src/node.rs` has instructed "check `.stack` after
# changing ANY large static" since issue 0190; a comment cannot be run on a
# schedule, and the talker duly shipped with 18,572 B against a ~67 KB budget.
#
# Only the artifact-free half runs here — the real per-image assertion lives in
# `just esp32 build-qemu`, on the ELF that step just linked. Resolving a built
# image from a `check` lane would need a fixture (banned) or would skip when
# absent, and a skip is exactly the silence this gate exists to end.
[private]
stack-floor:
    @python3 scripts/check-stack-floor.py --selftest
    @python3 scripts/check-stack-floor.py --claims

# Issue 0961 — `Executor::open_in` builds the executor as a VALUE in its own
# frame and `nros_cpp_init` moves it again, so an image whose main thread is
# smaller overflows in a function PROLOGUE, before any statement runs and so
# before any check written inside those functions could execute. Four bring-up
# sessions went into finding that once. The answer is a compile-time guard in
# the CALLER's translation unit; it exists in all four producers and nothing
# kept it there. Source-level by necessity: the generated headers are build
# artifacts and an affordability tier may not resolve one, but "does every
# producer still emit the define AND still compare against it" is exactly the
# regression that would silently disarm it.
[private]
executor-stack-floor:
    @python3 scripts/check-executor-stack-floor.py

# The per-RMW capability page is derived from the two C vtables + the zenoh
# shim's trait overrides. Prose versions of this table drifted in BOTH
# directions (cyclone services documented unsupported after service.cpp landed;
# zenoh liveliness documented as a no-op while wired) — generated, not
# transcribed, same reasoning as check-pool-inventory.
[private]
rmw-feature-matrix:
    @python3 scripts/gen-rmw-feature-matrix.py --check

# Support-status page (tiers, RTOS/RMW/toolchain pins, editions) — version
# strings parsed from nros-sdk-index.toml + rust-toolchain.toml so a pin bump
# moves the page in the same commit.
[private]
support-status:
    @python3 scripts/gen-support-status.py --check

# Every `nros…` identifier the book quotes must exist in the tree. The
# 2026-08-21 persona review's dominant failure class was prose naming renamed
# or never-existing symbols (max_callbacks_per_spin, nros_board_init_clocks,
# try_recv_safe…) — plausible spellings nothing failed on until a reader typed
# them. First run of this gate caught 8 more.
[private]
book-identifiers:
    @python3 scripts/check-book-identifiers.py

# Scheduling wiring matrix — parsed from sched_caps_for() (the function the
# realizer executes) + SchedClass dispatch sites in spin.rs. Prose versions of
# this drifted three ways at once (scheduling-models said EDF unused while
# spin.rs dispatches it). Same reasoning as check-rmw-feature-matrix.
[private]
sched-matrix:
    @python3 scripts/gen-sched-matrix.py --check

# Issue 0584 — an out-of-lane skip must SAY `lane`; a plain `skip!` is read as
# `capability`, so a fixture the lane deliberately did not build gets counted as
# a missing capability and the sweep summary lies about which gap a run has.
[private]
lane-skip-class:
    @python3 scripts/check-lane-skip-class.py

# no_std core-crate compile check across the embedded targets `ci.yml` gates
# (.github/workflows/ci.yml). Bare portable crates only — no SDKs, no link.
[group("ci")]
no-std:
    #!/usr/bin/env bash
    set -e
    crates="-p nros-core -p nros-log -p nros-serdes -p nros-params \
        -p nros-platform-api -p nros-platform-cffi -p nros-platform-critical-section -p nros-rmw \
        -p nros-node -p nros-platform -p nros-diagnostics -p nros"
    # phase-359 W1 — the four crates on the second line joined 2026-08-15. This
    # lane covered 9 of the 32 crates that DECLARE `no_std`, and `nros-node` —
    # the one with 85 of the ~190 `cfg(feature = "std")` sites, i.e. the crate
    # where a `std::` slip is most likely — was not among them. Until now the
    # only thing catching such a slip was an embedded fixture build: a real
    # backstop, but a ~40-minute one that fails whoever runs it rather than its
    # author.
    #
    # `$rmw_crates` below is NOT redundant with `$crates`, and the reason is the
    # whole point of the work item. `nros-node`'s executor lives behind
    # `#[cfg(any(has_rmw, test))]` — `has_rmw` is set by build.rs only when an
    # RMW feature is on — so a bare `--no-default-features` check compiles the
    # crate SHELL and none of the 85 sites. Verified rather than assumed: a
    # `std::string::String` planted in `node_record.rs` passes the bare check
    # (0.06s, cached) and fails the RMW-enabled one with
    # `cannot find module or crate `std``. A lane that only ran the bare check
    # would have been decoration — the issue-0196 shape, in a gate written to
    # prevent it.
    #
    # Still NOT covered: the board/platform-specific crates (they need SDKs and
    # a linker, which this lane deliberately excludes).
    rmw_crates="-p nros-node -p nros"
    # issue 0814 — the LENDING slice, heap-free. `nros-rmw-cffi/lending` and
    # `nros-node/rmw-lending` are the whole zero-copy surface, and NOTHING
    # builds them: the feature is enabled by exactly one crate in the tree
    # (`packages/testing/nros-tests`, and it names `platform-posix` while doing
    # it), it appears zero times under `cmake/` and `zephyr/`, and
    # `nros-rmw-zenoh-staticlib` declares no forwarder. So the one surface
    # sold as "for RAM-tight embedded targets" was compiled for no embedded
    # target at all, and the C half of it had in fact stopped compiling
    # entirely when issue 0812 retyped the loan token — for months, silently,
    # because no lane looked.
    #
    # This is a RATCHET, not new work: both rows passed before they were
    # written (measured on thumbv7em-none-eabi and re-measured on this lane's
    # two targets). What it buys is that they cannot quietly stop.
    #
    # NO `alloc`, deliberately — that combination is the one issue 0814's
    # `try_lend_slot` fix is about, and it is the tier where a heap is denied
    # outright.
    #
    # Why not a QEMU lane, which issue 0814 originally proposed: it would not
    # prove what it appears to. zenoh-pico memcpys the payload into a
    # non-expandable wire buffer regardless of whether we aliased it
    # (`_z_buf_encode` takes its no-copy branch only for an expandable wbuf;
    # every transport builds its TX wbuf with `_z_wbuf_make(mtu, false)`), so a
    # green "zero-copy works on hardware" lane would be green while every byte
    # is still copied. This costs seconds and catches the class that actually
    # bit.
    #
    # NOT here: `nros-c --features rmw-cffi,lending`. Its build script needs
    # `arm-none-eabi-gcc` for the C anchor, and SDK-needing crates are what
    # this lane deliberately excludes (see the note above). Its host-target
    # row lives in `check::workspace-features` instead — same class, same
    # issue, a lane that can afford it.
    lending_feats="nros-rmw-cffi/lending,nros-node/rmw-cffi,nros-node/rmw-lending"
    # `rustup target add` FIRST, serially: two concurrent adds of different
    # targets touch the same toolchain dir.
    for target in thumbv7m-none-eabi riscv32imc-unknown-none-elf; do
        rustup target add "$target" >/dev/null 2>&1 || true
    done
    # The two targets are independent `cargo check`s, so run them under the
    # jobserver rather than back to back (phase-336 W7). nros-rmw-cffi needs ptr
    # atomics — Cortex-M only (riscv32imc lacks them; mirrors ci.yml's per-target
    # crate set).
    source scripts/build/jobserver-pool.sh
    printf '%s\n' \
        "echo '== check-no-std: thumbv7m-none-eabi ==' && cargo check $crates -p nros-rmw-cffi --no-default-features --target thumbv7m-none-eabi" \
        "echo '== check-no-std: riscv32imc-unknown-none-elf ==' && cargo check $crates --no-default-features --target riscv32imc-unknown-none-elf" \
        "echo '== check-no-std: executor slice (alloc+rmw-cffi), thumbv7m ==' && cargo check $rmw_crates --no-default-features --features alloc,rmw-cffi --target thumbv7m-none-eabi" \
        "echo '== check-no-std: executor slice (alloc+rmw-cffi), riscv32imc ==' && cargo check $rmw_crates --no-default-features --features alloc,rmw-cffi --target riscv32imc-unknown-none-elf" \
        "echo '== check-no-std: lending slice (NO alloc), thumbv7m ==' && cargo check -p nros-rmw-cffi -p nros-node --no-default-features --features $lending_feats --target thumbv7m-none-eabi" \
        "echo '== check-no-std: lending slice (NO alloc), riscv32imc ==' && cargo check -p nros-rmw-cffi -p nros-node --no-default-features --features $lending_feats --target riscv32imc-unknown-none-elf" \
        | nros_pool_run no-std
    echo "no-std OK."

# Verify nros-sdk-index.toml + the QEMU configure flags
# (the `sdk-index` job in .github/workflows/gate.yml). Buildless + fast.
#
# The third one is issue 1006: a qemu recipe with no `dist` is source-built by
# EVERY user, so a backend it leaves auto-detected makes the runtime dependency
# set a property of that user's machine. `check-dist-runtime-deps` can only see
# that after a provision (and CI never provisions esp32-qemu), so the static
# half lives here.
[group("ci")]
sdk-index:
    python3 scripts/sdk/verify-index.py nros-sdk-index.toml
    ./scripts/sdk/check-qemu-configure.sh
    python3 scripts/sdk/check-qemu-source-features.py nros-sdk-index.toml

# #310 — gate the in-tree cli sub-workspace's rustfmt-cleanliness (it is outside
# the root workspace that `check-workspace`'s `cargo fmt --check` covers).
#
# issue 0337 — `rustfmt` DIRECTLY, not `cargo fmt`, for the same reason as
# `check-example-fmt`: `cargo fmt` runs `cargo metadata`, which has to resolve
# `ros-launch-manifest-model` (historically through a nested launch submodule;
# since phase-332 a git-tag dep that needs a network fetch). This gate is in
# `check-fast`, which is source-free — inits no submodules, fetches nothing —
# so on CI's push lane resolution failed and the gate died before formatting
# anything. Formatting needs no dependency graph.
#
# Scope matches what `cargo fmt` covered: the workspace MEMBERS only. Test
# fixture crates under `tests/fixtures/` are separate packages, not members,
# and were never formatted by this gate — several are deliberately on older
# editions.
[private]
cli-fmt:
    #!/usr/bin/env bash
    set -euo pipefail
    ws_edition="$(sed -n 's/^edition[[:space:]]*=[[:space:]]*"\([0-9]*\)".*/\1/p' packages/cli/Cargo.toml | head -1)"
    ws_edition="${ws_edition:-2024}"
    # Members, read from the manifest rather than duplicated here.
    members="$(sed -n '/^members = \[/,/^]/p' packages/cli/Cargo.toml \
        | sed -n 's/^[[:space:]]*"\([^"]*\)".*/\1/p')"
    for m in $members; do
        toml="packages/cli/$m/Cargo.toml"
        [ -f "$toml" ] || continue
        edition="$(sed -n 's/^edition[[:space:]]*=[[:space:]]*"\([0-9]*\)".*/\1/p' "$toml" | head -1)"
        edition="${edition:-$ws_edition}"   # `edition.workspace = true` inherits
        mapfile -t files < <(git ls-files "packages/cli/$m/*.rs" "packages/cli/$m/**/*.rs" \
            | grep -v '/tests/fixtures/' || true)
        [ "${#files[@]}" -eq 0 ] && continue
        echo "  fmt $m (edition $edition, ${#files[@]} files)"
        rustfmt "+{{NIGHTLY}}" --check --edition "$edition" "${files[@]}"
    done

# Check workspace: formatting and clippy (no_std, native)
# nros-c/nros-cpp/standalone RMW staticlib wrappers excluded from no_std
# check: staticlib/cdylib requires a platform-specific panic/runtime setup.
# nros-rmw-{zenoh,dds,xrce}-cffi excluded because their `*Rmw` type
# imports are platform-feature-gated by the underlying impl crate
# (e.g. `ZenohRmw` only exists when one of `platform-{posix,zephyr,…}`
# is on). `--no-default-features --workspace` strips every feature
# from every member at once, so the cffi shim's `RustBackendAdapter<R>`
# can't resolve its type parameter. Real consumers always specify
# a platform; the per-feature combinations are covered by
# `check-workspace-features` further down.
# Kept as a name others may invoke. Both of its former jobs moved to the FAST
# tier, where they catch a mistake before a build tier nobody runs per task:
# host clippy -> `check-test-targets` (now `--all-targets`, `-D warnings`),
# `fmt --check` -> `check-workspace-fmt`.
[private]
workspace: workspace-fmt
    @echo "check-workspace: fmt + clippy now run in the fast tier."

# Check workspace for embedded target (Cortex-M4F)
# Excludes zpico-sys: requires native system headers for CMake build
# Excludes nros-tests: requires std (test framework dependencies)
# Excludes nros-c/nros-cpp/standalone RMW staticlib wrappers:
# staticlib/cdylib requires a platform-specific panic/runtime setup.
#
# Builds into a dedicated `target-embedded/` (CARGO_TARGET_DIR) so the
# thumbv7 artifacts never share cargo's per-target-dir build lock with the
# host clippy — letting `check-workspace-all` run the two concurrently.
[private]
workspace-embedded:
    #!/usr/bin/env bash
    set -euo pipefail
    # issue 0287 — the exclude list is DERIVED, not hand-written.
    #
    # cargo unifies features across every workspace member when this builds the
    # workspace for a thumb target, regardless of what firmware can reach. One
    # host-only member therefore turns `std` on for everything and the lane dies
    # in an unrelated crate (`can't find crate for `std`` in nros-serdes, which
    # is merely the first no_std crate cargo reaches). The 20-line hand list
    # that used to live here carried no reasons and nothing tied an entry to its
    # crate; each crate now declares `[package.metadata.nros] host-only = true`
    # with a reason, and the script below derives the flags.
    echo "Checking workspace for embedded target..."
    source scripts/build/cargo.sh
    excludes="$(bash scripts/build/host-only-members.sh)"
    # issue 0400: box-aware target dir — a relative `target-embedded` overrides
    # the ROS distrobox's CARGO_TARGET_DIR redirect and shares build-script
    # binaries with the host (GLIBC-mismatch crash + a misleading host-only hint).
    emb_target="$(nros_scoped_target_dir embedded)"
    if ! CARGO_TARGET_DIR="$emb_target" cargo clippy --quiet --workspace \
            --no-default-features --target thumbv7em-none-eabihf \
            $excludes -- -D warnings; then
        echo "" >&2
        echo "[hint] If this failed with \`can't find crate for \\\`std\\\`\` in a crate you did" >&2
        echo "       not touch, a NEW host-only member is leaking \`std\` through cargo" >&2
        echo "       feature unification (issue 0287). The named crate is a victim, not" >&2
        echo "       the cause. Declare the new crate host-only:" >&2
        echo "" >&2
        echo "         [package.metadata.nros]" >&2
        echo "         host-only = true" >&2
        echo "         host-only-reason = \"...\"" >&2
        exit 1
    fi

# Run the host + embedded workspace clippy CONCURRENTLY. They share no
# target-dir (host = `target/`, embedded = `target-embedded/`), so cargo's
# build lock doesn't serialize them; sccache (global RUSTC_WRAPPER) shares the
# dep cache across both. The `NROS_BUILD_JOBS` budget is split in half to each
# via `CARGO_BUILD_JOBS` so total parallelism stays bounded (same knob the
# build recipes thread — no hardcoded `-j`). Both still run standalone.
[private]
workspace-all:
    #!/usr/bin/env bash
    set -uo pipefail
    jobs="${NROS_BUILD_JOBS:-$(nproc 2>/dev/null || echo 8)}"
    half=$(( jobs / 2 )); [ "$half" -lt 1 ] && half=1
    CARGO_BUILD_JOBS="$half" just check workspace &
    host=$!
    CARGO_BUILD_JOBS="$half" just check workspace-embedded &
    emb=$!
    rc=0
    wait "$host" || rc=1
    wait "$emb" || rc=1
    exit "$rc"

# Phase 166.R.5 — guard `nros-log` on CAS-less ESP32-C3 /
# riscv32imc so portable-atomic fallback regressions surface in
# the standard check tier.
[private]
nros-log-riscv32:
    @echo "Checking nros-log for riscv32imc..."
    cargo check -p nros-log --target riscv32imc-unknown-none-elf --no-default-features

# Check workspace with various feature combinations
[private]
workspace-features:
    @echo "Checking feature combinations..."
    # Phase 128.C.3 — `nros/rmw-zenoh-cffi` feature deleted; the
    # umbrella now only carries `rmw-cffi`. Backend selection is
    # done by adding the matching `nros-rmw-<name>` dep.
    # Phase 248 C5c — the `nros` umbrella dropped its `platform-*` features
    # (platform now comes from `nros-platform`/board/RMW crates, not the umbrella),
    # so the combo lints `nros` without `platform-posix` (nros-c/nros-cpp still
    # carry it — see the nros-c combo below).
    @echo "  - nros: cffi + humble"
    cargo clippy --quiet -p nros --no-default-features --features "std,rmw-cffi,ros-humble"
    @echo "  - nros: cffi + iron"
    cargo clippy --quiet -p nros --no-default-features --features "std,rmw-cffi,ros-iron"
    @echo "  - nros-c: zenoh-cffi + posix + humble"
    cargo clippy --quiet -p nros-c --no-default-features --features "std,rmw-cffi,platform-posix,ros-humble"
    @echo "  - nros: cffi (no_std)"
    cargo clippy --quiet -p nros --no-default-features --features "rmw-cffi"
    @echo "  - transport: sync-critical-section"
    cargo clippy --quiet -p nros-rmw --no-default-features --features "sync-critical-section" --target thumbv7em-none-eabihf
    @echo "  - nros-rmw (std)"
    cargo clippy --quiet -p nros-rmw --features "std"
    # Phase 214.G.2 — workspace-wide no-default-features smoke. Catches
    # the feature-unification regression class (Track F) at `just check`
    # time rather than waiting for `just test-unit`. `--no-run` compiles
    # all tests without executing — keeps the gate fast (no test runs)
    # while still exercising the trans-feature dep graph.
    #
    # `--exclude nros-c`: pre-existing latent test-compile bug in
    # `packages/api/nros-c/src/cdr.rs:565` references `std::ffi::CStr`
    # but the lib is no_std-by-default. Filed for separate fix; gate
    # remains valid for every other crate. Remove the exclude once the
    # nros-c lib-test gating lands.
    @echo "  - workspace: test-compile --no-default-features"
    cargo test --no-run --workspace --exclude nros-c --no-default-features --quiet
    # issue 0619 — the exclude above is what let nros-c's lib test rot unnoticed:
    # excluded from the ONLY test-compile gate, it was covered by nothing at all.
    # The exclude is about --no-default-features specifically (no panic handler,
    # no platform port), so the fix is not to drop it but to give nros-c its own
    # line with DEFAULT features, where it does link. Issue-0196 rule: a gate
    # must cover the class it claims to.
    @echo "  - nros-c: test-compile (default features)"
    cargo test --no-run -p nros-c --quiet
    # issue 0814 — the loan/borrow surface, which had NOT COMPILED since issue
    # 0812 retyped the backend token (`*mut c_void` -> `*mut rmw_loan_token_t`)
    # and left these three FFI call sites behind. Seven errors, for months,
    # caught by nothing: no lane enables `lending`, so the crate's whole
    # zero-copy API was never built — while the committed cbindgen header went
    # on declaring all five symbols unconditionally.
    #
    # Both rows, because the two differ in what they even contain: without
    # `alloc` the receive half is `cfg`-ed out (it boxes a `RecvView` per
    # borrow — 0812's own open item), so the no-`alloc` row is the publish
    # surface and the `std` row is the whole thing. Checking one would leave
    # the other exactly as dark as before.
    #
    # The embedded-target twin of this is the lending slice in
    # `check::no-std`; `nros-c` cannot go there because its build script needs
    # a cross C toolchain.
    @echo "  - nros-c: lending (publish surface, no alloc)"
    # `panic-platform` is nros-c's own `default` and is named here for the
    # reason its manifest gives: `--no-default-features` builds this crate's
    # staticlib, which is a FINAL artifact and needs a `#[panic_handler]`.
    cargo clippy --quiet -p nros-c --no-default-features --features "panic-platform,rmw-cffi,lending" -- -D warnings
    @echo "  - nros-c: lending + std (publish AND borrow surface)"
    cargo clippy --quiet -p nros-c --no-default-features --features "std,rmw-cffi,lending,platform-posix" -- -D warnings
    @echo "All feature checks passed!"

# Check C formatting only (clang-format) — BUILDLESS, source-free → push lane.
[private]
c-fmt:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/clang-format.sh
    CF="$(nros_clang_format)"
    echo "Checking C formatting... ($CF)"
    echo "  - clang-format (nros-c headers)"
    find packages/api/nros-c/include -name '*.h' -not -name 'nros_generated.h' -print0 | xargs -0 "$CF" --dry-run --Werror
    echo "  - clang-format (zpico C)"
    "$CF" --dry-run --Werror packages/rmw/zenoh/zpico-zephyr/src/*.c packages/rmw/zenoh/zpico-zephyr/include/*.h
    echo "  - clang-format (C examples, ALL platforms)"
    git ls-files -z 'examples/*/c/**/*.c' 'examples/*/c/**/*.h' ':!examples/px4/**' | xargs -0 "$CF" --dry-run --Werror
    echo "C formatting OK."

# The Cyclone RMW's own CMake/ctest suite (descriptor builder + registry).
#
# This used to be a dedicated `cyclonedds-ci` step on the top-level `ci` line —
# the only RMW with a named slot there. It is not special: it is one backend's
# native test suite, so it belongs with the other per-component lanes here.
#
# Moving it into `check` is also the fix for how issue 0319 survived: that red
# sat on main for two days because `cyclonedds-ci` ran only in `just ci`, and
# `just check` is the recipe people actually run. Warm cost is ~22s.
#
# Best-effort: skips cleanly when the pinned Cyclone submodule is not
# initialised (typical for contributors not touching the DDS backend). The
# `cyclonedds::ci` recipe itself fails hard on real test failures.
[private]
rmw-cyclonedds:
    #!/usr/bin/env bash
    set -e
    if [ ! -f third-party/dds/cyclonedds/CMakeLists.txt ]; then
        echo "Cyclone DDS skip: submodule not initialised"
        echo "  (run \`just cyclonedds setup\` to enable)"
        exit 0
    fi
    just cyclonedds ci

# Check C code: formatting + nros-c umbrella header syntax. COMPILES nros-c
# (→ nros-macros → nros-build → nros-cli-core → the ros-launch-manifest submodule;
# issue 0083) to emit the OPAQUE_U64S macro header, so it needs sources/CLI
# submodule → build tier (check-build), NOT the source-free push lane.
[private]
c: c-fmt
    #!/usr/bin/env bash
    set -e
    echo "Checking C code (build + syntax)..."
    echo "  - syntax (nros-c umbrella header)"
    # The per-variant `<nros/nros_config_generated.h>` (defining the
    # OPAQUE_U64S macros referenced by `<nros/nros_generated.h>`) is
    # emitted by `nros-c`'s build.rs into `target/nros-c-generated/`.
    # Build first so the syntax check has those macros; otherwise
    # the source-tree stub fires its `#error`.
    # issue 0400 follow-up — this build GENERATES the headers the checks below
    # compile against, so its failure must not be swallowed. It used to run
    # `--quiet 2>/dev/null || true`, which turned "the generator never ran" into
    # "your C headers are broken": every `*_OPAQUE_U64S` came back undeclared
    # from a `cc` line that says nothing about cargo. Diagnosing that cost four
    # wrong hypotheses; the build's own error names the cause in one line.
    cargo build -p nros-c --no-default-features --features "std,rmw-cffi,platform-posix,ros-humble" --quiet
    # Variant dir FIRST so its `nros_config_generated.h` (with the
    # real OPAQUE_U64S macros) wins over the source-tree stub.
    cc -fsyntax-only \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -include packages/api/nros-c/include/nros/nros.h \
        -x c /dev/null
    echo "  - executor verb spelling + deprecated aliases (issue 0338)"
    # The entity-registration family is `nros_executor_add_*` (rclc's spelling);
    # the old `register_*` names survive one release as macro aliases. The probe
    # takes function POINTERS, so an alias naming a symbol that does not exist
    # fails here rather than at some consumer's link step.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/executor_verb_aliases.c
    echo "  - graph query entry points are DECLARED, not just defined (phase-381 W4)"
    # The first version of these five was macro-generated. It compiled, this
    # lane passed, and cbindgen emitted nothing for them — cbindgen does not
    # expand macros, so four of five had no header declaration and were
    # uncallable from C while the Rust side stayed green. The probe takes
    # function POINTERS, so a missing declaration is a build failure here.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/graph_query_entry_points.c
    echo "  - a message's serialization format matches the linked backend (RFC-0088 D5)"
    # phase-421 W6 — every generated C message header carries a
    # `_Static_assert` comparing its format id against the image's. The positive
    # TU proves the assertion is reachable and does not fire on a matching pair;
    # the probe below proves it fires on a mismatched one.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/serialization_format.c
    # Expected-failure, and asserted-present first for the same reason the
    # parameter probe below is: a missing file is also a non-zero cc, and reads
    # as a pass.
    test -f packages/api/nros-c/tests/compile/serialization_format_mismatch_probe.c || { \
        echo "ERROR: serialization_format_mismatch_probe.c is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; \
        exit 1; \
    }
    if cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/serialization_format_mismatch_probe.c 2>/dev/null; then \
        echo "ERROR: a message declaring a format the linked backend does not speak compiled clean." >&2; \
        echo "       NROS_ASSERT_MESSAGE_FORMAT is not reaching callers, so a uORB-encoded" >&2; \
        echo "       message would publish through a CDR backend with no diagnostic." >&2; \
        exit 1; \
    fi
    echo "  - parameter name spelling + deprecated forwarders (phase 379 W5)"
    # The parameter family is `nros_parameter_*`; the old `nros_param_*` names
    # survive one release as NROS_DEPRECATED_MSG `static inline` forwarders. The
    # TU takes function POINTERS with the DECLARED signatures, so a forwarder
    # whose argument list drifted from the function it forwards to fails here.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/param_name_aliases.c
    # ...and the forwarders must WARN. A deprecation nobody is told about is an
    # alias, and the whole reason this family used forwarders instead of issue
    # 0338's macros was to get the diagnostic. So the assertion is a NEGATIVE
    # one: with `-Werror=deprecated-declarations` the old spelling must FAIL to
    # compile. Written as an expected-failure because a passing compile is
    # exactly what a silently-dropped attribute looks like.
    # An expected-failure compile cannot distinguish "the deprecation
    # attribute fired" from "the file is not there" -- both are a non-zero
    # cc, and the second reads as a pass. Phase-379 W5 landed this check
    # without its probe and the gate was silently vacuous. Assert first.
    test -f packages/api/nros-c/tests/compile/param_deprecation_probe.c || { \
        echo "ERROR: param_deprecation_probe.c is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if cc -fsyntax-only -std=c11 -Werror=deprecated-declarations \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/param_deprecation_probe.c 2>/dev/null; then \
        echo "ERROR: param_deprecation_probe.c compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The NROS_DEPRECATED_MSG on the nros_param_* forwarders in" >&2; \
        echo "       nros/parameter.h is not reaching callers, so the rename ships" >&2; \
        echo "       with no migration warning at all." >&2; \
        exit 1; \
    fi
    echo "  - service reply verb spelling + deprecated forwarders (phase 379 W5)"
    # `send_reply` -> `send_response`, same shape as the parameter family one
    # slot up. TWO old spellings forward to `nros_service_send_response_raw`:
    # `nros_service_send_reply_raw` (the plain rename) and
    # `nros_service_send_response` (a permanent NROS_RET_NOT_INIT stub from
    # before the polling API, kept so the un-suffixed name cannot read like a
    # second, working entry point). Function POINTERS again, so a forwarder
    # whose argument list drifted fails here.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/service_name_aliases.c
    # ...and they must WARN, asserted as the same expected-failure compile.
    # An expected-failure compile cannot distinguish "the deprecation
    # attribute fired" from "the file is not there" -- both are a non-zero
    # cc, and the second reads as a pass. Phase-379 W5 landed this check
    # without its probe and the gate was silently vacuous. Assert first.
    test -f packages/api/nros-c/tests/compile/service_deprecation_probe.c || { \
        echo "ERROR: service_deprecation_probe.c is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if cc -fsyntax-only -std=c11 -Werror=deprecated-declarations \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/service_deprecation_probe.c 2>/dev/null; then \
        echo "ERROR: service_deprecation_probe.c compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The NROS_DEPRECATED_MSG on the send_reply forwarders in" >&2; \
        echo "       nros/service.h is not reaching callers, so the rename ships" >&2; \
        echo "       with no migration warning at all." >&2; \
        exit 1; \
    fi
    # The active-goal count pair collapsed into ONE function in rcl's shape,
    # `nros_action_server_get_active_goal_count(server, size_t *out) ->
    # nros_ret_t`. The old polling-tier spelling
    # `nros_action_server_active_goal_count_raw` survives one release as an
    # NROS_DEPRECATED_MSG `static inline` forwarder that must keep its ORIGINAL
    # `int32_t` tri-state convention. The old callback-tier spelling cannot
    # survive at all -- it IS the merged name with a different signature -- so
    # this TU also pins the merged signature, which is what proves the old
    # `size_t` form is gone. Function POINTERS again, so a forwarder whose
    # argument list or return type drifted fails here.
    echo "  - action goal-count merge + deprecated forwarder (phase 379 W6)"
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/action_name_aliases.c
    # ...and the forwarder must WARN, asserted as the same expected-failure
    # compile. An expected-failure compile cannot distinguish "the deprecation
    # attribute fired" from "the file is not there" -- both are a non-zero cc,
    # and the second reads as a pass. Phase-379 W5 landed two of these checks
    # without their probes and both gates were silently vacuous. Assert first.
    test -f packages/api/nros-c/tests/compile/action_deprecation_probe.c || { \
        echo "ERROR: action_deprecation_probe.c is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if cc -fsyntax-only -std=c11 -Werror=deprecated-declarations \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/action_deprecation_probe.c 2>/dev/null; then \
        echo "ERROR: action_deprecation_probe.c compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The NROS_DEPRECATED_MSG on the active_goal_count_raw forwarder in" >&2; \
        echo "       nros/action.h is not reaching callers, so the merge ships" >&2; \
        echo "       with no migration warning at all." >&2; \
        exit 1; \
    fi
    echo "  - receive verb spelling + deprecated forwarders (phase 379 W6)"
    # `try_recv` -> `take`, and `_raw` -> `_serialized` for the pre-CDR byte
    # form (ROS 2's word: `rcl_take_serialized_message`). Note which direction
    # this moves: the RMW vtable one layer down has said `take` / `take_request`
    # / `take_response` / `take_sequence` since W3.b, so the USER API was the
    # only layer still spelling it the Rust-channel way. Old spellings survive
    # one release as NROS_DEPRECATED_MSG `static inline` forwarders. Function
    # POINTERS again, so a forwarder whose argument list drifted fails here.
    cc -fsyntax-only -std=c11 \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/receive_verb_aliases.c
    # ...and they must WARN, asserted as the same expected-failure compile.
    # An expected-failure compile cannot distinguish "the deprecation
    # attribute fired" from "the file is not there" -- both are a non-zero cc,
    # and the second reads as a pass. Phase-379 W5 landed two of these checks
    # without their probes and both gates were silently vacuous. Assert first.
    test -f packages/api/nros-c/tests/compile/receive_deprecation_probe.c || { \
        echo "ERROR: receive_deprecation_probe.c is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if cc -fsyntax-only -std=c11 -Werror=deprecated-declarations \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-c/tests/compile/receive_deprecation_probe.c 2>/dev/null; then \
        echo "ERROR: receive_deprecation_probe.c compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The NROS_DEPRECATED_MSG on the try_recv* forwarders in" >&2; \
        echo "       nros/subscription.h, nros/service.h and nros/client.h is not" >&2; \
        echo "       reaching callers, so the rename ships with no migration warning." >&2; \
        exit 1; \
    fi
    echo "  - cross-include (nros_cpp_ffi.h + component.h in one TU)"
    # Issue 0160 — the C prototypes and struct typedefs component.h re-declares
    # must stay compatible with cbindgen's canonical nros_cpp_ffi.h (the
    # phase-273 callback_group arity drift class). Including BOTH headers in
    # one TU (ffi.h first, so component.h's hand mirrors are guarded out) makes
    # the compiler the drift gate: any divergence is a "conflicting types"
    # error. Field-level struct-mirror parity is the buildless
    # ffi-struct-mirrors gate (push lane).
    # issue 0400 follow-up — this build GENERATES the headers the checks below
    # compile against, so its failure must not be swallowed. It used to run
    # `--quiet 2>/dev/null || true`, which turned "the generator never ran" into
    # "your C headers are broken": every `*_OPAQUE_U64S` came back undeclared
    # from a `cc` line that says nothing about cargo. Diagnosing that cost four
    # wrong hypotheses; the build's own error names the cause in one line.
    cargo build -p nros-cpp --no-default-features --features "std,rmw-cffi,platform-posix,ros-humble" --quiet
    cc -fsyntax-only \
        -Itarget/nros-c-generated \
        -Itarget/nros-cpp-generated \
        -Ipackages/api/nros-c/include \
        -Ipackages/api/nros-cpp/include \
        -include packages/api/nros-cpp/include/nros/nros_cpp_ffi.h \
        -include packages/api/nros-c/include/nros/component.h \
        -x c /dev/null
    echo "  - rmw ABI layout static-asserts (issue #238/#239)"
    # The RMW C headers and their Rust `#[repr(C)]` mirrors are hand-kept
    # in lockstep. `abi_layout_check.c` is a `_Static_assert`-only TU that
    # pins the C-side widths (event-kind int-size, qos size, handle
    # alignment, vtable pointer-slot count); the Rust half is the
    # `abi_layout` const-assert block in nros-rmw-cffi/src/lib.rs. Either
    # side drifting fails exactly one guard. `-fsyntax-only` still
    # evaluates static asserts.
    cc -fsyntax-only \
        -Ipackages/core/nros-rmw-abi/include \
        packages/rmw/cffi/tests/c_stubs/abi_layout_check.c
    echo "All C checks passed!"

# Check C++ formatting only (clang-format) — BUILDLESS, source-free → push lane.
[private]
cpp-fmt:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/clang-format.sh
    CF="$(nros_clang_format)"
    echo "Checking C++ formatting... ($CF)"
    echo "  - clang-format (nros-cpp headers)"
    "$CF" --dry-run --Werror packages/api/nros-cpp/include/nros/*.hpp
    echo "  - clang-format (C++ examples, ALL platforms)"
    git ls-files -z 'examples/*/cpp/**/*.cpp' 'examples/*/cpp/**/*.hpp' ':!examples/px4/**' | xargs -0 "$CF" --dry-run --Werror
    echo "C++ formatting OK."

# Check C++ headers: formatting + freestanding syntax + nros-cpp clippy. The
# clippy (rmw-zenoh-cffi) + syntax probe COMPILE nros-cpp/nros-c (zpico-sys pulls
# the zenoh-pico source submodule) → source-dependent → build tier (check-build),
# NOT the source-free push lane.
[private]
cpp: cpp-fmt
    #!/usr/bin/env bash
    set -e
    echo "Checking C++ headers (build + syntax + clippy)..."
    echo "  - freestanding syntax (c++14)"
    # parameter.hpp re-exposes the C-side `nros_parameter_*` API from
    # nros-c, so the syntax probe needs nros-c on the include path too.
    # The per-variant `<nros/nros_cpp_config_generated.h>` (defining
    # `NROS_CPP_EXECUTOR_STORAGE_SIZE` and friends, referenced by
    # `executor.hpp`'s `uint8_t storage_[NROS_CPP_EXECUTOR_STORAGE_SIZE]`)
    # is emitted by `nros-cpp`'s build.rs into
    # `target/nros-cpp-generated/`. Same C-side header for nros-c.
    # Build both first; variant dirs go FIRST on the include path so
    # their real headers win over the source-tree stubs.
    # issue 0400 follow-up — this build GENERATES the headers the checks below
    # compile against, so its failure must not be swallowed. It used to run
    # `--quiet 2>/dev/null || true`, which turned "the generator never ran" into
    # "your C headers are broken": every `*_OPAQUE_U64S` came back undeclared
    # from a `cc` line that says nothing about cargo. Diagnosing that cost four
    # wrong hypotheses; the build's own error names the cause in one line.
    cargo build -p nros-c -p nros-cpp --no-default-features --features "std,rmw-cffi,platform-posix,ros-humble" --quiet
    # issue 0730 — the (zephyr-embedded × C++ × cyclonedds) coordinate is a
    # pairwise-class gap no runtime tier covers (cyclone fixtures run on
    # native_sim, which appends `,std`), and it failed E0599 at COMPILE time:
    # the cyclone branch was the one bare-`rmw-cffi` composition, so
    # `TransportError::BackendDynamic` (alloc-gated) vanished under the
    # exhaustive mapper. Check that exact composition on an embedded target.
    # The feature string is READ FROM the zephyr module's cyclone branch, so
    # this gate follows the lane instead of being a second spelling of it.
    if rustup target list --installed | grep -qx aarch64-unknown-none; then
        _cyc_features="$(sed -n '/CONFIG_NROS_RMW_CYCLONEDDS/,/endif()/ s/.*set(_nros_cpp_features "\([^"]*\)").*/\1/p' zephyr/CMakeLists.txt | head -1)"
        if [ -z "$_cyc_features" ]; then
            echo "check-cpp: could not extract the cyclone _nros_cpp_features line from zephyr/CMakeLists.txt" >&2
            exit 1
        fi
        echo "  - embedded cyclone coordinate (issue 0730): ${_cyc_features},panic-platform @ aarch64-unknown-none"
        cargo check -p nros-cpp --no-default-features \
            --features "${_cyc_features},panic-platform" \
            --target aarch64-unknown-none --quiet
    else
        echo "  - embedded cyclone coordinate (issue 0730): SKIP — aarch64-unknown-none not installed"
    fi
    for hdr in packages/api/nros-cpp/include/nros/*.hpp; do
        # Phase 209 — `rclcpp_compat.hpp` is a source-compat shim still
        # being aligned with the live nros::Result / nros::QoS API. The
        # clang-format check above still covers it; the freestanding
        # C++14 probe stays opt-out until 209 lands its API touch-ups.
        case "$hdr" in *rclcpp_compat.hpp) continue ;; esac
        # issue #52 — `main.hpp` is the HOSTED entry runtime (LinuxBoard / NuttX):
        # its rtos_e2e readiness/sample banners call `::std::printf`, which
        # `-ffreestanding` is not required to expose from `<cstdio>` (only the global
        # `printf`). Probe it hosted so it keeps full syntax coverage; every other
        # header stays freestanding.
        free="-ffreestanding"
        case "$hdr" in *main.hpp) free="" ;; esac
        # issue #52 — `nros-platform-api/include` carries `<nros/platform.h>`, pulled
        # by `heap_sequence.hpp` (Phase 229.5); without it the probe fails
        # `fatal error: nros/platform.h: No such file or directory`.
        c++ -fsyntax-only -std=c++14 $free -fno-exceptions -fno-rtti \
            -Itarget/nros-cpp-generated \
            -Itarget/nros-c-generated \
            -Ipackages/api/nros-cpp/include \
            -Ipackages/api/nros-c/include \
            -Ipackages/platform/nros-platform-api/include \
            -include "$hdr" -x c++ /dev/null
    done
    # Issue 0089 gap-4 — typed-API INSTANTIATION probe (the header loop only
    # parses templates). Compiles a TU that instantiates `nros::bind_service`
    # against a generated-shape service type, so the template body is checked.
    echo "  - typed bind_service instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/bind_service.cpp
    echo "  - graph query entry points are DECLARED, not just defined (phase-381 W4)"
    # Same trap as the C side: the C++ FFI header is cbindgen-generated, and
    # cbindgen does not expand macros — a macro-generated `#[no_mangle]` gets no
    # declaration and is uncallable while the Rust side stays green. Method
    # POINTERS make that a build failure here.
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/graph_query_entry_points.cpp
    echo "  - a module can NAME the backend it means (issue 1050 defect 3)"
    # `nros::init()` opens whichever backend registered FIRST, and on a hosted
    # target that order is decided before `main` by `.init_array` ctors. A PX4
    # uORB-only module linked against a zenoh-carrying archive was handed zenoh
    # and failed dialling a router nobody asked for. The capability existed on
    # `Executor::open_with_rmw` and `NodeBuilder().rmw(...)`; the one-liner the
    # examples use could not reach it. Surface probe, including the defaults
    # that make it a one-liner.
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/init_names_its_backend.cpp
    echo "  - a message's serialization format matches the linked backend (RFC-0088 D5)"
    # phase-421 W6 — codegen specialises `nros::format_of<M>` per message and the
    # typed creators `static_assert` on it. The positive TU proves the assertion
    # is reachable; the probe proves it fires.
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/serialization_format.cpp
    echo "  - an unbounded type cannot size a buffer (issue 0964)"
    # Expected-failure, and asserted-present first for the same reason every
    # other probe here is: a missing file is also a non-zero c++, and reads as a
    # pass.
    test -f packages/api/nros-cpp/tests/compile/unbounded_buffer_probe.cpp || { \
        echo "ERROR: unbounded_buffer_probe.cpp is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++17 \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/unbounded_buffer_probe.cpp 2>/dev/null; then \
        echo "ERROR: a type with NO derived bound sized a buffer and compiled clean." >&2; \
        echo "       detail::buffer_bounds' unbounded arm has been disarmed, so an" >&2; \
        echo "       INVENTED number is back in a stack array -- and the estimate" >&2; \
        echo "       under-states as well as over-states, so the buffer can be too" >&2; \
        echo "       small for a message the user legitimately sends (issue 0964)." >&2; \
        exit 1; \
    fi
    test -f packages/api/nros-cpp/tests/compile/serialization_format_mismatch_probe.cpp || { \
        echo "ERROR: serialization_format_mismatch_probe.cpp is MISSING -- the expected-failure" >&2; \
        echo "       check below would PASS on a missing file." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/serialization_format_mismatch_probe.cpp 2>/dev/null; then \
        echo "ERROR: a message declaring a format the linked backend does not speak compiled clean." >&2; \
        echo "       NROS_CPP_ASSERT_MESSAGE_FORMAT is not reaching the typed creators." >&2; \
        exit 1; \
    fi
    # issue 0338 — `spin` verb SHAPE probe: `spin()` blocks until shutdown
    # (rclcpp/C/Rust semantics) and the bounded verb is `spin_for(...)`. The
    # defect was the shape of the API, so a compile-time assertion on which
    # arities exist is what catches a regression.
    echo "  - spin verb shape (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/spin_verbs.cpp
    # issue 0278 — PollingSubscription<M> (latest-value polling subscriber)
    # instantiation: the wrapper bodies (drain/take_data/take_new_data/take)
    # and the create_polling_subscription factory path are type-checked against
    # a generated-shape message type.
    echo "  - PollingSubscription instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/polling_subscription.cpp
    # phase-408 W1/W4 (issue 0896) — the receive-buffer hint a typed C++
    # subscription supplies is `nros::rx_size_bound<M>`, the type's own DERIVED
    # bound, and no longer `M::SERIALIZED_SIZE_MAX` (an estimate that
    # under-sizes a nested type over 512 B — issue 0964). That is a template
    # BODY, so the header parse loop above never type-checks it; this TU
    # instantiates `bind_subscription` and pins the three shapes the trait
    # distinguishes: derived-and-bounded, no-derivation (legacy fallback), and
    # derived-but-poisoned (which must be INCLUDABLE without error).
    # issue 0964 — the same TU also covers `nros::rx_buffer_capacity<M>`, which
    # is what the 13 RECEIVE buffers inside these headers now stack, and
    # instantiates every one of them for a BOUNDED and an UNBOUNDED type. The
    # second column is the point: a blanket switch to `rx_size_bound<M>` would
    # be a compile error for 81 of 120 stock Humble types.
    echo "  - rx_size_bound / rx_buffer_capacity / receive sites (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/rx_size_bound.cpp
    # issue 0278 Half B — Client<Svc>::call_polling (callback-safe bounded
    # service call, no executor spin) instantiation: the method body
    # (serialize -> nros_cpp_service_client_call_raw(..., timeout_ms) ->
    # deserialize) is type-checked against a generated-shape service type.
    echo "  - Client::call_polling instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/service_client_call_polling.cpp
    # issue 0796 — the action CALLBACK tier: the accepted-goal hook
    # (`set_accepted_callback`) and the client-side `cancel_goal`, both of
    # which C and Rust had and C++ did not. Template BODIES, so the header
    # parse loop above never type-checks them; this TU instantiates them and
    # static-asserts that the per-goal `CancelResponse` and the RPC
    # `CancelReturnCode` stay distinct types.
    echo "  - action callback tier instantiation (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/action_callback_tier.cpp
    # phase-379 W5 — the QoS surface under its rclcpp names. The policy enums
    # are namespace-scope and public, the getters return them instead of `int`,
    # and the three time windows take/return `nros::Duration`. This TU asserts
    # the RETURN TYPES and the millisecond boundary, not just that the names
    # resolve: a `deadline()` that came back as `uint32_t` would pass a name
    # check and re-introduce the ledgered defect. It also static_asserts that
    # `nros_cpp_qos_t`'s `*_ms` FIELDS are still `uint32_t` milliseconds — the
    # same token is a struct field (ABI, must not move) and was a class method
    # (renamed), so a textual sweep is exactly what this catches (issue 0160).
    # `-Wno-deprecated-declarations` because the TU names the old spellings
    # deliberately; the probe below is what proves they still warn.
    echo "  - QoS policy accessors + ms boundary (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Wno-deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/qos_policy_accessors.cpp
    # ...and the old spellings must WARN. A deprecation nobody is told about is
    # an alias. NEGATIVE assertion, the C half's shape one language over
    # (`param_deprecation_probe.c`): with `-Werror=deprecated-declarations` the
    # old spelling must FAIL to compile. Written as an expected-failure because
    # a passing compile is exactly what a silently-dropped attribute looks like.
    # An expected-failure compile cannot distinguish "the deprecation
    # attribute fired" from "the file is not there" -- both are a non-zero
    # cc, and the second reads as a pass. Phase-379 W5 landed this check
    # without its probe and the gate was silently vacuous. Assert first.
    test -f packages/api/nros-cpp/tests/compile/qos_deprecation_probe.cpp || { \
        echo "ERROR: qos_deprecation_probe.cpp is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Werror=deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/qos_deprecation_probe.cpp 2>/dev/null; then \
        echo "ERROR: qos_deprecation_probe.cpp compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The [[deprecated]] attributes on QoS::*_raw() / QoS::*_ms() /" >&2; \
        echo "       QoS::Liveliness in nros/qos.hpp are not reaching callers, so the" >&2; \
        echo "       rename ships with no migration warning at all." >&2; \
        exit 1; \
    fi
    # phase-403 step 2 — the DECLARED `@depth=` reaches the compiler, and a
    # disagreement between it and the QoS at the call site FAILS THE BUILD.
    #
    # `-std=c++17` and not the c++14 of its neighbours: C++17 is what the
    # component lane actually compiles with, and the absence of `__VA_OPT__`
    # there is the reason `NROS_SUBSCRIBE` dispatches on argument count at all.
    # Checking the macro under a standard no component uses would test a
    # different preprocessor.
    #
    # The fixture dir carries `nros/nros_declared_qos_generated.h` — what
    # `nros ws entity-inventory --output-header` renders from the
    # `entities.json` beside it, and what `nano_ros_node_register()` puts on a
    # real component's include path. Putting it on `-I` here exercises the same
    # `__has_include` pickup. The header and its input are held together by
    # `the_committed_compile_fixture_is_what_this_emitter_renders` in
    # nros-cli-core, so this fixture cannot drift into a table that matches
    # nothing (which would leave every assertion below vacuous and green).
    echo "  - declared QoS depth: the three cases COMPILE (c++17)"
    c++ -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti \
        -Ipackages/api/nros-cpp/tests/compile/declared-qos-fixture \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/declared_qos_depth.cpp
    # ...and a DISAGREEMENT must not. NEGATIVE assertion, and the one that makes
    # the check reachable rather than merely correct: six mechanisms in this
    # campaign were right and never ran. The positive TU above is compiled FIRST
    # so a missing include path or a fixture that matches nothing shows up as a
    # failure there, not as a silent pass here — an expected-failure compile
    # cannot tell "the assertion fired" from "the file is not there".
    test -f packages/api/nros-cpp/tests/compile/declared_qos_depth_probe.cpp || { \
        echo "ERROR: declared_qos_depth_probe.cpp is MISSING -- the expected-failure check" >&2; \
        echo "       below would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    echo "  - declared QoS depth: a MISMATCH fails the build (c++17, expected failure)"
    _dq_out="$(c++ -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti \
        -Ipackages/api/nros-cpp/tests/compile/declared-qos-fixture \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/declared_qos_depth_probe.cpp 2>&1)" && { \
        echo "ERROR: declared_qos_depth_probe.cpp COMPILED. The fixture declares" >&2; \
        echo "       sub:std_msgs/msg/Int32:/chatter@depth=1 and the call site passes" >&2; \
        echo "       nros::QoS(10); NROS_SUBSCRIBE's static_assert did not fire, so the" >&2; \
        echo "       declared depth is not reaching the compiler and nothing is checked." >&2; \
        exit 1; \
    }
    # A rejection is not enough: it must be OUR rejection, naming the topic and
    # BOTH depths. A typo or a missing header would also exit non-zero, and a
    # diagnostic the reader cannot act on is the same as no diagnostic.
    for _want in '"/chatter"' 'declared_depth_agrees<1, 10>' 'nano_ros_node_register'; do \
        case "$_dq_out" in \
            *"$_want"*) ;; \
            *) echo "ERROR: the declared-depth diagnostic does not contain: $_want" >&2; \
               echo "       It must name the TOPIC and BOTH depths, or a reader cannot tell" >&2; \
               echo "       which subscription is wrong or which of the two numbers to fix." >&2; \
               echo "--- compiler said ---" >&2; echo "$_dq_out" >&2; \
               exit 1 ;; \
        esac; \
    done
    # phase-379 W6 decision 1 — the receive verb under its rclcpp name.
    # `Subscription<M>::take` / `take_serialized` / `take_sequence` /
    # `take_validated` and `Service<S>::take_request`, with the `try_recv*`
    # spellings kept as `[[deprecated]]` forwarders. The header `-fsyntax-only`
    # loop above only PARSES the templates; this TU INSTANTIATES them, so a
    # forwarder whose argument list drifted from the method it forwards to
    # fails here. `-Wno-deprecated-declarations` because the TU names the old
    # spellings deliberately; the probe below is what proves they still warn.
    echo "  - receive verb aliases + deprecated forwarders (c++14)"
    c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Wno-deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/receive_verb_aliases.cpp
    # ...and the old spellings must WARN. NEGATIVE assertion, the C half's shape
    # one language over. An expected-failure compile cannot distinguish "the
    # deprecation attribute fired" from "the file is not there" -- both are a
    # non-zero c++, and the second reads as a pass. Assert first.
    test -f packages/api/nros-cpp/tests/compile/receive_deprecation_probe.cpp || { \
        echo "ERROR: receive_deprecation_probe.cpp is MISSING -- the expected-failure check below" >&2; \
        echo "       would PASS on a missing file, which is why this asserts first." >&2; \
        exit 1; \
    }
    if c++ -fsyntax-only -std=c++14 -fno-exceptions -fno-rtti -Werror=deprecated-declarations \
        -Itarget/nros-cpp-generated \
        -Itarget/nros-c-generated \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/api/nros-c/include \
        -Ipackages/platform/nros-platform-api/include \
        packages/api/nros-cpp/tests/compile/receive_deprecation_probe.cpp 2>/dev/null; then \
        echo "ERROR: receive_deprecation_probe.cpp compiled clean under -Werror=deprecated-declarations." >&2; \
        echo "       The [[deprecated]] attributes on Subscription::try_recv* /" >&2; \
        echo "       Service::try_recv_request are not reaching callers, so the" >&2; \
        echo "       rename ships with no migration warning at all." >&2; \
        exit 1; \
    fi
    # issue #201 — HeapSequence element-destructor RUNTIME probe: compiled AND
    # executed (counting allocator in the TU; asserts zero live allocations
    # across dtor / move-assign / clear / reserve-relocation of a two-level
    # heap element shape).
    echo "  - HeapSequence element-lifetime runtime probe (c++14)"
    mkdir -p target/nros-cpp-tests
    c++ -std=c++14 -fno-exceptions -fno-rtti \
        -Ipackages/api/nros-cpp/include \
        -Ipackages/platform/nros-platform-api/include \
        -o target/nros-cpp-tests/heap_sequence_lifetime \
        packages/api/nros-cpp/tests/compile/heap_sequence_lifetime.cpp
    ./target/nros-cpp-tests/heap_sequence_lifetime
    echo "  - nros-cpp clippy (zenoh-cffi + posix + humble)"
    cargo clippy --quiet -p nros-cpp --no-default-features --features "std,rmw-zenoh-cffi,platform-posix,ros-humble"
    echo "All C++ checks passed!"

# Check Python code: formatting + linting with ruff
[private]
python:
    @echo "No in-tree Python package to check (nros-cli owns the colcon extension)."

# Analyze per-function stack usage (requires nightly + llvm-tools)
# Usage: just check stack [example-dir] [top]
# Default: examples/qemu/rs-wcet-bench, top 30
[group("debug")]
stack example="packages/testing/nros-bench/wcet-cycles-qemu" top="30":
    ./scripts/stack-analysis.sh {{example}} --top {{top}}

# Analyze stack usage of a pre-built ELF (e.g. Zephyr west build output)
# Usage: just check stack-elf <path-to-elf> [top]
[group("debug")]
stack-elf elf top="30":
    ./scripts/stack-analysis.sh --elf {{elf}} --top {{top}}

# Analyze stack usage of C examples (requires cmake + gcc)
# Usage: just check stack-c [example-dir] [top]
# Default: examples/native/c/talker, top 30
[group("debug")]
stack-c example="examples/native/c/talker" top="30":
    ./scripts/stack-analysis-c.sh {{example}} --top {{top}}

# Analyze stack usage of all examples (requires nightly + llvm-tools + cmake)
# Covers: QEMU ARM, native Rust, and native C examples
# ESP32/STM32F4 excluded (need platform-specific SDKs)
[group("debug")]
stack-all top="10":
    #!/usr/bin/env bash
    set -euo pipefail
    failed=0
    # Rust examples (QEMU ARM — no exclude, show full picture)
    for example in \
        packages/testing/nros-bench/wcet-cycles-qemu \
        packages/testing/nros-tests/bins/cdr-roundtrip-qemu \
        examples/qemu-arm-baremetal/rust/talker \
        examples/qemu-arm-baremetal/rust/listener \
    ; do
        echo "================================================================"
        ./scripts/stack-analysis.sh "$example" --top {{top}} || { echo "[FAIL] $example"; failed=$((failed + 1)); }
        echo ""
    done
    # Rust examples (native — exclude tracing/regex infrastructure noise)
    for example in \
        examples/native/rust/talker \
        examples/native/rust/listener \
        examples/native/rust/custom-msg \
        examples/native/rust/service-server \
        examples/native/rust/service-client \
        examples/native/rust/action-server \
        examples/native/rust/action-client \
    ; do
        echo "================================================================"
        ./scripts/stack-analysis.sh "$example" --top {{top}} --exclude "regex_automata|regex_syntax|aho_corasick|env_filter|env_logger|driftsort" || { echo "[FAIL] $example"; failed=$((failed + 1)); }
        echo ""
    done
    # C examples (native)
    for example in \
        examples/native/c/talker \
        examples/native/c/listener \
        examples/native/c/custom-msg \
        examples/native/c/custom-transport-loopback \
    ; do
        echo "================================================================"
        ./scripts/stack-analysis-c.sh "$example" --top {{top}} || { echo "[FAIL] $example"; failed=$((failed + 1)); }
        echo ""
    done
    if [ "$failed" -gt 0 ]; then
        echo "[WARN] $failed example(s) failed"
        exit 1
    fi
    echo "[OK] All stack analyses complete"

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

# phase-340 W2 — the preconditions for putting a platform's rows in a SHARED
# cargo target dir. A group writes into one flat `<profile>/` namespace, and
# cargo does not hash the final artifact name the way it hashes `deps/`, so two
# rows in one group producing a binary of the same name overwrite each other
# and one test silently runs the other's binary. Nothing checked this: the one
# migrated platform (qemu-arm-baremetal) is collision-free by luck, and `linux`
# — the next candidate — is not.
#
# `check-fast`: buildless and source-free (it reads examples/fixtures.toml and
# the tracked leaf Cargo.tomls, invokes no cargo and no rustc), so it runs green
# on the pristine per-push checkout the tier is measured against.
#
# phase-343 W3 — the gate itself is tripwired. It has been widened twice (W1's
# row-keyed owners, B1's unhashed LIB artifacts) and both widenings were verified
# by hand into a commit message, which is not something a later reader can
# re-run. `fixture_group_collision_gate.sh` collides a binary name and a
# staticlib name on real leaf manifests and asserts the gate REPORTS each by
# name — matching the message, not just the exit code, because a malformed
# perturbation exits non-zero too (the first draft's T2 passed with B1 reverted,
# on a TOMLDecodeError). Each arm was confirmed to fail when its half of the gate
# is disabled.
# phase-347 W2 — the RMW descriptors (RFC-0071) agree with the live lowering.
#
# W2 adds `nros-rmw.toml` per backend and changes nothing else, so a descriptor
# is a SECOND derivation of what `resolve_rmw()` already computes. This gate is
# what makes it one: while both exist they must agree, and W3 may only delete
# the closed lists once this has been green. Buildless — it reads the generated
# `NanoRosRmwDispatch.cmake`, which is itself gated for staleness against the
# resolver, so agreeing with it IS agreeing with the resolver.
[private]
rmw-descriptors:
    @python3 scripts/check-rmw-descriptors.py

# phase-421 W4 / RFC-0088 — the `serdes` provider descriptors. Catches: an
# `impl` that is not "schema"/"codegen"; a `format_id` that is missing, not an
# integer, or outside 1..=255; an unknown key under `[serdes]`, which lowers to
# nothing while reading like a decision; two providers claiming one name or —
# the wire-visible one — one `format_id`; a descriptor whose reserved
# discriminant disagrees with `nros_serdes::format::SerializationFormatId`
# (`cdr = 1`, `uorb = 2` are written in both places, and this is what stops them
# diverging), in EITHER direction; and a package announcing
# `<nano_ros_provides kind="serdes"/>` with no descriptor beside it, whose
# symptom is otherwise a defaulted `impl` nobody chose. Runs its own selftest on
# the normal path. Buildless: TOML plus a regex over one Rust file.
[private]
serdes-descriptors:
    @python3 scripts/check-serdes-descriptors.py

# Issue 1025 -- a packer must not invent the row's cargo args or env. The shared
# cargo group key is a function of (platform, args, env); a consumer supplying
# two of the three gets a directory the build never wrote to, and only once that
# row gains a variant. Silent until then, which is how every ESP32 QEMU flash
# image stopped being packable without a single lane going red.
[group("check")]
fixture-artifact-dir-inputs:
    @python3 scripts/check-fixture-artifact-dir-inputs.py

[private]
fixture-groups:
    @python3 scripts/check-fixture-groups.py
    @bash packages/testing/nros-tests/tests/fixture_group_collision_gate.sh

# phase-340 P2 — nothing may write an `examples/**/target/` dir. That is the
# invariant item 7 / P4 stands on (`examples/**/target-*/` is globally ignored;
# a plain `target/` survives only through 391 per-leaf .gitignore files), and it
# was being broken by a SECOND build path with no manifest row — hence no
# coordinate, hence no shared cargo group. See the script's header for the
# measurement. Self-tests its own classifier on every run.
[private]
example-leaf-target-dirs:
    @python3 scripts/check-example-leaf-target-dirs.py

# issue 0718 — an example leaf builds in-tree (RFC-0026), so every build
# directory a recipe gives it must be named in that leaf's own .gitignore.
# The six threadx rust leaves ignored `build-cyclonedds` and not `build-zenoh`,
# and their c/cpp siblings ignored both, so no single leaf looked wrong.
[private]
example-leaf-build-dirs:
    @python3 scripts/check-example-leaf-build-dirs.py

# issue 0835 — an example leaf must not overrule the platform its own build
# passes. Six threadx rust leaves carried a plain `set(NANO_ROS_PLATFORM
# threadx)`, a NORMAL variable that shadows the cache entry
# `-DNANO_ROS_PLATFORM=threadx_riscv64` wrote — so each leaf and its own
# CMakeCache disagreed, written in the same second. Visible cost: four shared
# corrosion cargo groups where the configuration space has two, 2.8 GB of
# duplicate output. Quieter cost: two `stdc++` link decisions had come to depend
# on the wrong label, so making the leaves honest would have stopped them
# matching, silently. Buildless, reads the manifest and the leaves.
[private]
example-platform-not-shadowed:
    @python3 scripts/check-example-platform-not-shadowed.py

# issue 0720 — a test asks a fixture resolver for an artifact BY NAME, and a
# name no CMake target produces resolves to a missing path, which the test
# reports as a `fixture missing` SKIP. So the failure mode is silence, not red.
[private]
fixture-binary-names:
    @python3 scripts/check-fixture-binary-names.py

# issue 0722 — a manifest is read by whichever workspace claims it, and this
# tree has many roots. A crate no root claims can carry a duplicate key that
# cargo REFUSES, while `cargo metadata` from the repo root stays green.
[private]
manifests-parse:
    @python3 scripts/check-manifests-parse.py

# phase-411 W3 — `just <verb> <scope>` puts platform names and preset names in
# ONE argument position, so a name may not mean two things. `native` is both a
# platform module and a fixture lane, which is legal only while they denote the
# same scope; nothing checked that, and a preset that quietly means something
# else re-scopes a run without changing a single command anyone types. Buildless
# (NROS_SCOPE_NO_BUILD=1) — a collision whose expansion needs a compiled
# selector is itself the failure. Self-tests its own rule on every run.
[private]
scope-namespace:
    @python3 scripts/check-scope-namespace.py

# issue 0650 — a fixture lane that cannot run must report SKIPPED (rc 78), never
# build nothing and print "<platform> test fixtures built." with exit 0. That
# false OK is how six diverged riscv64 examples reached main: every host without
# the toolchain reported the lane green. Self-tests its own classifier.
[private]
lane-skip-protocol:
    @python3 scripts/check-lane-skip-protocol.py

# phase-411 W2 — 0599/0650 gave a lane the SKIPPED verdict; this says WHO MAY
# HAVE IT. A platform reached by a preset or the broad default may skip and is
# reported; a platform the operator NAMED (`just zephyr build-fixtures`) must
# work, because naming it is the whole specification. Runs the protocol both
# directions on every invocation — a named skip regressing to a pass is
# invisible to a grep, so the gate makes the call instead of reading it.
[private]
named-lane-fails:
    @python3 scripts/check-named-lane-fails.py

# issue 0658 — `[SKIPPED:<class>]` does not contain `[SKIPPED]`, so a hand-rolled
# literal match reclassifies every CLASSED skip as a FAILURE. Five matrix
# aggregators had written that literal independently, turning five tier-2 lane
# skips into tier-2 reds. One helper per language now owns the match
# (`nros_tests::skip_marker`, `scripts/test/skip_marker.py`). Self-tests its own
# matcher against the actual pre-fix line on every run.
[private]
skip-marker-matching:
    @python3 scripts/check-skip-marker-matching.py

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

[private]
build-rs-rerun-paths:
    @python3 scripts/check-build-rs-rerun-paths.py

# issue 0491 — the sibling rule to the gate above: a `cargo:rerun-if-env-changed`
# on a PATH-valued variable fingerprints the SPELLING of a directory, and one
# directory has a different spelling per example leaf (`relative = true`), from
# `just` (absolute) and unset. Rows sharing one `--target-dir` (phase-340 groups)
# then invalidate each other forever — six FreeRTOS rows rebuilt 6 units on every
# probe, permanently. Self-tests its own classifier on every run.
[private]
path-env-fingerprints:
    @python3 scripts/check-path-env-fingerprints.py

# issue 0702 — a test that cannot fail is worse than no test. Rejects an `Err`
# arm that prints a diagnosis and decides nothing, which is the shape eight
# separate tests had been wearing for months (see the script's docstring for the
# roll-call). Buildless: it reads tracked test sources, so it belongs on the
# fast line beside the other source gates.
[private]
tests-can-fail:
    @python3 scripts/check-tests-can-fail.py

# issue 0651 — every Zephyr symbol `zephyr/Kconfig` names must EXIST on BOTH
# supported lines (3.7 LTS + the 4.4 rolling line). Kconfig answers an undefined
# symbol with a WARNING, so a `select` that is right on 3.7 and misspelled on
# 4.4 silently drops what it would have enabled — and 4.4 builds only in the
# nightly, so that surfaces a day later against whatever else moved.
#
# Needs SOURCE, not a build, and not a west workspace. Off the fast line because
# it has a real precondition (a tree must be present); it FAILS rather than
# passes when it can check nothing (issue 0702).
#
#   3.7:  just zephyr setup
#   4.4:  just zephyr kconfig-trees      (two shallow clones, no west workspace)
#
# Issue 0651 second half: an unchecked supported line is now a FAILURE, not an
# OK with a footnote. It used to fail only when NO line was present, so the
# ordinary dev host — 3.7 present, 4.4 nowhere — went green having measured
# nothing about the line the gate exists for.
zephyr-kconfig-symbols:
    @python3 scripts/check-zephyr-kconfig-symbols.py

# phase-340 W4 — the artifact-identity budget: how many times one crate is
# COMPILED, and how many dirs one compilation is written into, for a single
# workspace at a single feature set. `examples/workspaces/mixed` is the fixture
# (Rust + C++ node packages over a shared `nros-core`), and today it answers 8
# identities for `nros-core` — three ×2 axes, zero sharing. The numbers and the
# reasoning live in the script; when W2/W3/W5 land, lower them there.
#
# `check-fast`, for two reasons. It is buildless in the strict sense — it reads
# FILENAMES under a build tree that already exists, never invoking cargo, rustc
# or workspace resolution — and it must not be able to fail a BUILD: a
# long-lived incremental tree accumulates rlibs from earlier builds, so an
# over-count from history alone is possible, and a gate that reds a build on
# that gets switched off. Failing a static check whose remedy is "wipe the tree
# and rebuild" is survivable. Consequence, stated rather than hidden: on the
# pristine per-push CI checkout there is no tree and this gate SKIPS (loudly,
# naming the build command); its live coverage is the developer who just built
# fixtures and then ran the tier their change earned.
# issue 0616 — at most ONE Rust archive per image defines the allocator.
#
# The companion to `check-feature-contract` clause (e), which counts
# `#[global_allocator]` DEFINITIONS IN SOURCE — there is exactly one, and there
# always will be. `packages/` has four `crate-type = ["staticlib"]` roots, each
# of which BAKES that one definition into its own `.a`; linking two gives an
# image two allocators while the source still has one. Source counting cannot
# see that, so this asks the artifacts.
#
# Needs built output, so it belongs with the lanes that produce it rather than
# in `check-fast`.
[private]
archive-lang-items:
    @bash scripts/check-archive-lang-items.sh

# Issue 0260 / phase-356 W3 — the sched-dim ACCEPT arms no build otherwise
# reaches. Syntax-only: it type-checks our `vTaskCoreAffinitySet` /
# `sched_setaffinity` / `tx_thread_smp_core_exclude` call sites against the
# vendored headers' REAL declarations, using a synthetic SMP config, because
# those arms sit behind macros no image defines — so an API misuse in them is
# invisible today.
#
# A green here is NOT an arm being observed ACCEPTING at runtime: that needs a
# real SMP board and is phase-356's separate, larger item.
#
# All THREE arms are covered — freertos (`vTaskCoreAffinitySet`), nuttx
# (`pthread_setaffinity_np`) and threadx (`tx_thread_smp_core_exclude`) — each
# against its own vendored headers. This comment previously said nuttx and
# threadx "are not yet" covered and pointed at the script's output for the
# truth; the script has covered all three since it landed, so the comment was
# describing a state that never shipped. Read the script, not this line, when
# they disagree: it prints one section per arm.
sched-dim-arms:
    @bash scripts/check-sched-dim-arms-compile.sh

# Issue 0719 — every path that BUILDS an image applies the image's policies.
# `nano_ros_add_executable` delegates to `nano_ros_entry`, so ~160 call sites are
# covered by construction; the handful that cannot call the entry (a board seam,
# the ESP-IDF component shim) must call `nros_apply_panic_policy` instead. Two of
# them did not, and each surfaced as a `#[panic_handler]` error four crates from
# its cause (#0688, #0700).
#
# STRUCTURAL, and it self-tests the one thing that makes it so: the sweep that
# preceded it keyed on the text `nano_ros_entry(` and excluded a file because a
# COMMENT there mentioned the name.
image-paths-apply-policy:
    @bash scripts/check-image-paths-apply-policy.sh

# `--self-test` (issue 0661) drives both era verdicts over a synthetic tree,
# because the failure mode here is a wrong VERDICT rather than a wrong count:
# the early widening branch used to print "counts ALL … an accumulated tree can
# inflate it" and then, from an uninitialised flag, "accumulation is ruled out".
# A reader who believed the second one went looking for a regression that was
# not there, which is how 0661 spent its length on two ordinary host
# build-dependency artifacts.
[private]
artifact-identity-budget:
    @bash scripts/check-artifact-identity-budget.sh

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

# issue 0516 — a COMMENTED-OUT element in a package.xml must not read as a
# declaration. cmake has no XML parser, so every reader here regexes raw text;
# before the `nros_read_package_xml_body()` helper, a package.xml that merely
# DOCUMENTED a tag declared it. Covers the greedy-strip failure mode too, where
# `<!--.*-->` silently deletes everything between two comments. Buildless.
# phase-348 W2 — a provider announces itself (package.xml) and declares what it
# lowers to (its descriptor). Nothing structural keeps the two name lists equal,
# so compare them. ONE gate for every provider family: a copy of the rule beside
# each family's descriptors is the second-spelling antipattern (#282 -> #326).
#
# Catches: a descriptor with no `<nano_ros_provides>` beside it (invisible to
# the phase-348 scan while looking migrated); a name list that disagrees with
# the descriptor's, or is in the wrong order, for the three families whose
# descriptors still carry `names`; and, for a family whose descriptor carries
# none (`serdes`, RFC-0087 D4 — the announcement is the only spelling), two
# providers announcing one name. phase-421 W4 added that fourth family as one
# FAMILIES row, not a second script.
[private]
provider-announcements:
    @python3 scripts/check-provider-announcements.py

# phase-348 W3 — the provider index and the cmake seam that reads it. cmake asks
# the CLI for TAB rows rather than parsing the index (no second parser to
# drift), every recorded package.xml is watched for reconfigure, and a
# provider added AFTER the index was written is caught by rescan-and-compare —
# the case no file watch can cover (issue 0196's shape). Needs `just setup-cli`.
# phase-351 W2 — this repo's site config (`[deploy.<n>.nros]`) agrees with
# `just/sdk-env.just`. Both spellings are live during the migration, so the gate
# is what stops them drifting (the phase-347 pattern). `--write` renders missing
# blocks; generator and gate are one file so they cannot disagree.
[private]
site-config:
    @python3 scripts/check-site-config.py

# issue 0529 — Zephyr's zenoh tx knobs have TWO sources: `zephyr/Kconfig`
# defaults (forwarded to the C lane by nros_rmw_zenoh.cmake) and
# `config/zephyr/nros-platform.toml`'s [knobs.zenoh.tx] (the RFC-0049 ladder,
# Rust lane). They agree today only by coincidence; this compares them so a
# divergence is loud. Buildless.
[private]
zephyr-knob-agreement:
    @python3 scripts/check-zephyr-knob-agreement.py

# Issue 0934 R1 / phase-405 W2 — `nano_ros_workspace(BACKEND … SYSTEM …)` names
# the RMW on one line and, on the next, a bringup whose `system.toml` names it
# again. `NanoRosWorkspace.cmake` builds the path to that file (`:351`) and
# never parses it, so the two authored copies can contradict each other with no
# diagnostic: the C/C++ lane links BACKEND, the CLI resolves `[system].rmw`.
# This fails on a disagreement and NAMES the one-sided declarations that can
# become one. It does not decide which side is the SSoT — that is W2's
# follow-up. Buildless.
[private]
workspace-rmw-agreement:
    @python3 scripts/check-workspace-rmw-agreement.py

# Issue 0472 — every `*_OPAQUE_U64S` macro must have a compile-time size guard,
# so a probe that under-states a size is a build error and not a short buffer
# written past in C.
opaque-storage-guards:
    @python3 scripts/check-opaque-storage-guards.py

# phase-348 W4 — build order derived from package.xml `<depend>`, not from the
# order a SUBDIRS list happens to be written in. Every workspace CMakeLists
# carries "Node pkgs BEFORE entries so the entry codegen sees their metadata";
# the entry packages already state that as <exec_depend>. Covers the cycle and
# bad-subdir rejections and the cmake seam. Needs `just setup-cli`.
# Issue 1068 / RFC-0087's mirror class — micro-XRCE-DDS is compiled by TWO
# lanes from one vendored tree, and the two source lists drifted: `build.rs`
# honoured `NROS_LINK_IP=0` and dropped the UDP transports while the CMakeLists
# compiled them unconditionally, so a serial-only node built from C/C++ carried
# an IP transport the user had removed — a larger image, not an error.
#
# `packages/rmw/xrce/xrce-sources.txt` is now the one list, and this asserts
# that neither lane names a `.c` of its own, that both answer exactly the
# manifest's condition tokens (both directions — a token only one lane answers
# IS 1068, one conditional over), and that no backend source is compiled by
# nobody. Buildless, ~40 ms.
[private]
xrce-source-manifest:
    @python3 scripts/check-xrce-source-manifest.py

[private]
workspace-order:
    @bash packages/testing/nros-tests/tests/workspace_order_gate.sh

[private]
provider-index:
    @bash packages/testing/nros-tests/tests/provider_index_gate.sh

[private]
package-xml-comments:
    @bash packages/testing/nros-tests/tests/package_xml_comment_stripping.sh

# RFC-0087 D3 / phase-420 W1 — `<nano_ros_uses kind= name=/>` and the
# `<nano_ros deploy= board= rmw=/>` sugar must mean the same thing to the cmake
# reader, and `deploy=` must NOT become a selection (it names a system.toml
# block, not a provider). Divergence between the two spellings would be
# invisible: both still configure, against different values. Buildless.
[private]
package-xml-uses:
    @bash packages/testing/nros-tests/tests/package_xml_uses_general_form.sh

# issue 0762 — a killed build launcher must take its whole subtree with it.
# Drives real process trees and asserts on `ps`: the orphan bug is silent by
# construction (the terminal comes back while the build keeps running), so
# nothing short of watching the processes can catch a regression.
[private]
subtree-guard:
    @bash packages/testing/nros-tests/tests/subtree_guard.sh

# Check zenoh transport
[private]
zenoh:
    cargo clippy --quiet -p nros-rmw --features std

# Issue 0827 — the infrastructure-queryable counts have ONE definition each,
# tied to the number of servers actually created. A service server IS a zenoh
# queryable, so these are a term in every service-buffer pool the RMW sizes;
# they had SEVEN spellings and no definition, two of them already wrong.
[private]
infra-queryable-counts:
    @python3 scripts/check-infra-queryable-counts.py

# phase-412 W4 — the value that reaches the COMPILE is the value the resolver
# produced. The knob gates beside this one check whether the derived number is
# RIGHT; this one checks whether it ARRIVED, which is a different question and
# the one that kept going wrong. W1 shipped six delivery failures in one wave
# with every other gate green: a second consumer reading raw CONFIG_*, a name
# that resolved to empty, a C consumer with no default under rung 4, a loader
# whitelist that dropped symbols at a function boundary, and a knob resolved
# twice where the second call won with the DERIVE sentinel. Two of those were
# live in pushed work, and one built the zenoh session with 8 subscriber slots
# for 10 subscriptions — a runtime failure, invisible at link.
#
# Self-test only here: the end-to-end assertion needs a CONFIGURED build dir,
# which the fast tier does not have. Point it at one by hand:
#     python3 scripts/check-knob-delivery.py <build-dir>
[private]
knob-delivery:
    @python3 scripts/check-knob-delivery.py --self-test

# Issue 1002 — a configure sequence must reach a FIXPOINT, and the pass count
# is not the instrument. 0991's remedy says configure TWICE; the receive
# payload class needed THREE, and four island builds shipped the stale value
# because the stale one is LARGER and therefore silent. This configures until
# nothing moves rather than counting, so a knob that gains a longer chain fails
# here instead of over-sizing for a month.
#
# Self-test only in the fast tier: the real assertion needs a configured build
# dir. Point it at one by hand:
#     python3 scripts/check-knob-fixpoint.py <build-dir> --configure "<cmd>"
[private]
knob-fixpoint:
    @python3 scripts/check-knob-fixpoint.py --self-test

# A knob is resolved EXACTLY ONCE. phase-412 hit this three times: a knob
# gained a derivable resolution and kept its old plain one, the second call
# won, and what it passed was the raw Kconfig value — which for a derivable
# knob is the `-1` DERIVE SENTINEL, resolved as though someone had stated it.
# Invisible to every other gate, because the derived value was right and the
# resolver did run.
#
# Branch-aware: two arms of one conditional are not duplicates. Its first real
# run said they were, and a checker that flags a correct idiom trains people to
# ignore it.
[private]
knob-resolved-once:
    @python3 scripts/check-knob-resolved-once.py

# phase-403 W9 (issue 0965) — the per-kind CALLBACK SLOT cost has ONE
# definition, tied to the registration sites that actually claim a slot.
#
# `nros ws entity-inventory` derives NROS_EXECUTOR_MAX_CBS by summing
# `EntityKind::callback_slots()`, and that function is a MIRROR: the CLI is a
# host binary and nros-node is no_std and built for the target, so it restates
# what `Executor::next_entry_slot()` decides. Same rule as the queryable counts
# above — a mirror is acceptable only while something holds it to the
# definition. The specific claim it defends is that a PUBLISHER claims no slot,
# which is why the island's 33 declared entities want 19 and why the bring-up's
# hand-count of "33 handles" over-sized by 14.
[private]
entity-slot-costs:
    @python3 scripts/check-entity-slot-costs.py

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

# `native` is a ROLE (host build); `posix` and `linux` are two REACHES and a
# board has one. See CLAUDE.md "Naming".
[private]
host-platform-vocabulary:
    @python3 scripts/check-host-platform-vocabulary.py

# `nros-platform-posix` is named for a standard — keep it holding to one, so
# the board layer stays the only place that is Linux-only (phase-401 W4).
[private]
posix-platform-purity:
    @python3 scripts/check-posix-platform-purity.py

# issue 0834 — a per-build sizes header must never lose its `.stamp` twin.
#
# The pair is written by ONE function, header first, so a stamp with no header
# is a state the writers cannot produce — and it does not repair itself: a build
# whose probe yields 0 declines to write the header and reports success. The
# consumer-side symptom is a stub `#error` (C++) or an undeclared macro (C),
# from a file nobody was looking at. This names the directory instead.
[private]
orphan-generated-stamp:
    @python3 scripts/check-orphan-generated-stamp.py

# RFC-0087 D5 / phase-420 W8 — a fetch without a digest is the same defect as an
# unpinned submodule, one mechanism over. `check-submodule-pins` can go and ask
# the submodule because a gitlink IS a full commit id; a `GIT_TAG v0.6.1` names
# a ref on someone else's server, and if they move it this tree switches trees
# with NO local diff at all. The two gates partition the surface: gitlink there,
# CMake argument here.
#
# Rejects a short sha (a prefix the REMOTE resolves, not an identity), MD5/SHA1
# (forgeable), and `URL_SHA256` — which is not a CMake option at all, so CMake
# takes it as an unknown keyword and verifies nothing while reading as more
# careful than the correct spelling.
#
# Reports NOTHING TO CHECK with the population searched when a rule has no
# subject, because "the rule holds" and "the rule has no subject" are different
# facts. Buildless, ~0.15 s.
[private]
vendor-fetch-pinned:
    @python3 scripts/check-vendor-fetch-pinned.py
