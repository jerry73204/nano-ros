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


# The gates themselves, by topic. `import` and not `mod`: it MERGES
# definitions, so every gate keeps the flat name it is invoked and
# documented by. A name defined twice across these files is a hard parse
# error, which is the property that makes the split safe.
#
# NOT `import?`: an optional import that cannot be found is silently
# tolerated, and a missing topic file would take its gates with it while
# every lane still reported OK.

import 'check/abi.just'
import 'check/cargo.just'
import 'check/cmake.just'
import 'check/codegen.just'
import 'check/conventions.just'
import 'check/docs.just'
import 'check/fixtures.just'
import 'check/gates.just'
import 'check/lanes.just'
import 'check/platform.just'
import 'check/rmw.just'
import 'check/tools.just'
import 'check/workflows.just'

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

# The serial runner — phase-395 W11, emptied by this commit.
#
# The 230-name dependency list that used to live here is GONE, for the third
# time of asking. Issue 1072 removed the registry as a CONCEPT — `check-fast`
# membership is derived (`check-gate-lists.py --list`), and
# `run-gates-parallel.sh`, `check-gate-lists.py` and `check-default-gates-run-
# somewhere.py` all say in their own comments that the line no longer exists.
# It did. `6c52892cc`, whose subject is "delete the 218-name gate registry",
# left all 230 names in place, so the repo has been carrying a shared append
# target that its own documentation describes as deleted.
#
# It had already drifted, which is the argument rather than the tidiness:
# `entry-rmw-vocabulary` is in the DERIVED fast lane and was absent from this
# list, so `just check fast-serial` silently ran 230 gates where `just check
# fast` ran 231. A gate that one lane skips is exactly what the derivation
# exists to prevent.
#
# And it was the worst conflict surface in the repo. `just/check.just` is
# touched by 48 of the last 300 commits on main — four times the next file —
# and two pull requests adding alphabetically adjacent gate names insert at the
# same line with certainty. Two of the four genuinely conflicting open PRs on
# 2026-09-06 collided here and nowhere else.
#
# `--serial` runs the same derived set one gate at a time, fail-fast, output
# unshredded. `run-gates-parallel.sh` already owns the skip-ledger reset and
# the qualified closing report, so the body is the one line.
[group("main")]
fast-serial:
    @bash scripts/build/run-gates-parallel.sh --serial

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
