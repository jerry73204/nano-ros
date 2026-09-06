#!/usr/bin/env python3
"""phase-359 W0 — the `std` census, and a ratchet that only turns one way.

Two modes. `--check-guards` (issue 0701) is a DIFFERENT question over the same
walk: not "how many `std::` sites are there" but "is every capability-gated one
declared", i.e. the half of ARCHITECTURE §2 clause (a) that
`check-feature-contract` does not reach. It shares this file because it needs
exactly what the counting walk needs — which cfg gates a given line — and a
second spelling of that walk is the antipattern this repo keeps paying for.
`--self-test` exercises the attribution on a synthetic crate. The counting path
is unchanged by either.

## Why a census at all

The campaign to drop `std` from the core crates is a migration, not a patch:
242 `cfg` mentions of the `std` feature and 421 `std::` paths over nine crates,
after excluding two crates for cause (below). Two earlier figures were wrong and
are superseded: the "~190" from planning was a hand-grep, and W0's own first
"181" came from a regex that could not see `cfg(all(feature = "std", ...))`. Without a committed baseline "did that work item land?" is
unanswerable, and — the failure mode that actually matters — nothing stops a
new `std::` site appearing in a crate someone already finished.

So this counts, and it FAILS when a count goes UP. Going down is the point;
going down means updating the baseline below, which makes progress visible in
the diff rather than asserted in a commit message.

## What is counted, and why two metrics

* `cfg` — lines carrying `cfg(feature = "std")` / `cfg(not(feature = "std"))`
  (including `cfg_attr` and inner `#![cfg(...)]`). This is the SHAPE of the
  split: the branches a reader has to hold in their head.
* `path` — occurrences of a `std::` path in live code. This is the DEPENDENCY
  itself; it must reach zero for the crate to compile without `std`.

They move independently. W2 (collapsing duplicated fields) deletes `cfg`
branches without touching `path` counts; W4/W5 (routing time and threads
through the platform seam) delete `path`s. Tracking one would hide the other.

## `#[cfg(test)]` code is excluded, and that is a CORRECTION not a win

Host unit tests link `std` even in a `no_std` crate, so their `std::` use can
never block a target build. Counting it made the ruler answer the wrong
question: `nros-node` read 309 paths, of which **209 were in its `#[cfg(test)]`
module** — two thirds of the number was code that does not ship. W5's premise
("29 `std::thread` sites to migrate") came from that inflation; 15 of those 29
were `thread::sleep` in tests.

Excluding them LOWERS the counts without anything being fixed. The drop is
recorded as a metric correction in the phase doc, not as progress.

## Comments are excluded, and that is not fussiness

This file's own sibling commits added doc comments that NAME `std::sync::Condvar`
and `std::time::Instant` while REMOVING uses of them. Counting comment text
would have scored that as a regression and taught everyone to ignore the gate.
Line comments and the doc-comment forms are stripped before matching; block
comments are rare in this tree and handled naively (a `/*` line is skipped).

## Scope

`packages/core/*` and `packages/api/*` — the crates the campaign covers. Board
and platform crates are out of scope: several are legitimately std-hosted
(`nros-board-linux`, and NuttX via `nros-board-nuttx`), and deciding their fate
is phase-359 W7, not this gate's business.
"""
import re
import sys
from pathlib import Path
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent / "lib"))
from tracked import tracked  # issue 0721: index lookup, not a walk

REPO = Path(__file__).resolve().parents[1]
SCOPE = ["packages/core", "packages/api"]

# Out of scope, for reasons that are properties of the crate and not judgement
# calls. Both were caught by measuring rather than by the guess this baseline
# started as:
#
#   nros-macros            `proc-macro = true`. It runs on the HOST at compile
#                          time and is always std; its `std::` occurrences are
#                          TOKENS IT EMITS for generated code, not a dependency
#                          of anything embedded. Counting them would make the
#                          ruler lie in both directions.
#   nros-orchestration-ir  says so itself: "host code (serde + thiserror); it
#                          carries no runtime/`no_std`". A schema crate for
#                          `system.toml`, consumed by the CLI.
EXCLUDE = {"nros-macros", "nros-orchestration-ir"}

# Any `feature = "std"` appearing inside a cfg attribute, in ANY nesting.
#
# The first version of this anchored on `cfg(` / `cfg(not(` immediately followed
# by the feature, which silently missed every `cfg(all(feature = "std", ...))` —
# 26 of them in `spin.rs` alone. It was caught by W2: four cfg lines were
# deleted and the gate reported no movement. A ruler that cannot see the most
# common form of the thing it measures is worse than no ruler, because it reads
# as progress.
CFG_FEATURE_RE = re.compile(r'feature\s*=\s*"std"')
PATH_RE = re.compile(r'\bstd::')

# phase-359 baseline. W0 measured it; the corrected cfg metric re-measured
# everything; then `nros-node` came down per work item: W2 139->131 / 346->342,
# W3 131->127 / 342->321, W4 127->112 / 321->309, W6 309->285 (pure re-export
# spellings). Excluding `#[cfg(test)]` code then re-based every number — a
# metric correction, not progress; see the module docstring. W8 then re-gated
# `nros`'s node_metadata from `std` to `alloc`: cfg 64 -> 25; then
# `core::error::Error` made `nros-core`'s Error impls unconditional (cfg 7 -> 5,
# path 5 -> 2), and `nros-params`'s ParameterVariant impls (cfg 13 -> 7,
# path 8 -> 1).
# Lower these as work items land; the gate rejects any increase.
BASELINE = {
    # phase-361 W8.e: +1, a `compile_error!` guard. `metadata-mode` used to
    # ENABLE `std`; it now REQUIRES it and says so. A guard must NAME the
    # feature it checks, so making an implicit enable explicit costs one
    # counted site and removes one implicit enable. Same shape as nros-node.
    # phase-359 W10 follow-up: +1 on top of that campaign's own reduction (13),
    # the `env` `compile_error!` guard. `env` used to ENABLE `std`
    # (`env = ["std"]`), which is the implicit-flavour shape this campaign
    # removes and clause (a) forbids; it now REQUIRES it. The guard must NAME
    # the feature it checks, so the count goes up by one while an implicit
    # enable goes away. Same trade phase-361 W8.e made for `metadata-mode`
    # directly above.
    #
    # Then 14 -> 12, same day, different session: `init` and the `NROS_RMW`
    # read moved off `std` onto `env`, and `ExecutorNodeRuntime::spin`/`halt`
    # lost a gate that described a convention rather than a requirement.
    # Met issue 0589 across the 2026-08-16 rebases: `nros`'s node-declaration
    # diagnostic moved from `std::eprintln!` to `nros_log`. Set from the TREE,
    # which is what the note above prescribes — three separate merges here have
    # now produced a number that is neither side's nor their arithmetic.
    #
    # Then 12 -> 10: the `env` capability took `init` (above), and the
    # `FileParamStore` re-export went with the type issue 0080 retired.
    #
    # phase-359 W10 — the SPELLING pass. Items that live in `alloc` or `core`
    # were named through `std`: `format!`, `String`, `Vec`, `Box`, `Arc`,
    # `atomic::{AtomicBool, Ordering}`. None was a `std` need; each made its
    # crate look like it wanted the flavour. What still names `std` is what only
    # `std` has: `env::var`, `Mutex`, `OnceLock`, `Path`, `fs`, `Instant`.
    #
    # issue 0687 — path 9 -> 22, deliberately, and it is the SAME 13 sites that
    # left `nros-node` (20 -> 7). Reading the process environment moved to this
    # crate's `src/env.rs`, which is the whole point: `nros` is the hosted edge
    # and may have an environment; the core may not. The campaign's target is
    # the core, so a number that moves from `nros-node` to `nros` is progress
    # even though the TOTAL does not move. Reducing this crate's count needs a
    # different decision (a host-side crate below the facade), not more
    # spelling.
    #
    # issue 0687 follow-up — 22 -> 16: `init`'s `read_env_context` was a THIRD
    # parse of the same four variables (no deprecation warning on the legacy
    # spellings, no domain range check), and `from_env` a fourth read of
    # `$ROS_DOMAIN_ID` that silently resolved a malformed value to domain 0.
    # Both go through `try_resolve_hosted` now. What is left is the reader
    # itself plus `Mutex`/`OnceLock`, `Path` and the test module's `EnvGuard`.
    #
    # issue 0669 sibling — 10/16 -> 9/15: `metadata-mode` stopped being a `std`
    # capability. Its only `std` was `std::sync::Mutex` guarding a process-global
    # recorder, and the guard beside it claimed a reason ("writes a file") that
    # belongs to `nros-cpp`. On `nros_rmw::sync::Mutex` the capability is heap +
    # a lock, so it requires `alloc`; the `spin` edge rides the feature, so no
    # firmware image gains it. The `OnceLock` in `env.rs` is NOT going the same
    # way, and the file says why: that module calls `std::env::var`.
    #
    # phase-359 W10 follow-up — cfg 9 -> 8: `time::now`'s `std` arm became
    # `all(std, not(rmw-cffi))` and its port arm dropped `not(std)`, so the two
    # arms stopped OVERLAPPING on a native build. The path count does not move
    # and should not: the `Instant` arm is the answer for a build with no port,
    # which is shipped (the metadata probe), not vestigial.
    #
    # phase-359 W10 RULING — 8/15 -> 5/14. `nros::time` is `rmw-cffi`-gated:
    # the module exists where a port does, and is ABSENT where none is linked,
    # which is the definition rather than a gap. What remains is the `env`
    # capability's own reads (12 of the 14), `init`'s `Path`, and the
    # `RMW_IMPLEMENTATION` hint — all of them `env`, which REQUIRES `std`
    # because a process environment is a `std` facility.
    "nros": {"cfg": 5, "path": 14},
    #
    # phase-359 W10: 13 -> 2 cfg, 8 -> 1 path. `platform.rs` was three std/no_std
    # PAIRS — clock, wall clock, sleep — and every C consumer links a platform
    # port, so all three are one implementation reading the ABI. One of them was
    # a defect, not a split: the no_std "system time since the Unix epoch"
    # returned the MONOTONIC counter. Goal-ID generation had the same shape and
    # the same fix. `$NROS_RMW` selection moved onto `env`, which this crate now
    # names. What remains is `extern crate std` and the lang-item plumbing.
    #
    # issue 0687 — path 1 -> 0. Its only `std::` was a third reading of
    # `$NROS_RMW`; it consumes `nros_node::rmw_selector` now.
    "nros-c": {"cfg": 2, "path": 0},
    #
    # phase-359 W10 (backend tier): 3 -> 5. Two arms select the WALL CLOCK —
    # the platform's `time_since_epoch_*` when a port is linked, the steady
    # counter otherwise. They mention `std` only to say "not std", and they are
    # what let `nros-rmw-zenoh`, `nros-rmw-cffi` and `nros-bridge` stop
    # forwarding `nros-core/std`: `Clock::system()` was the single thing that
    # forward carried. Two counted sites here, three backends' worth of
    # implicit `std` removed from every native graph.
    #
    # phase-359 W10 follow-up — cfg 5 -> 3: `platform_wall_clock` lost its
    # `not(std)` gate. It had one, which meant a `std` build with a port linked
    # read `SystemTime` while the executor read the port: two wall clocks in one
    # image, agreeing on POSIX by coincidence and diverging the moment a port has
    # an opinion. The port outranks `std` now, as it already did on `no_std`.
    #
    # phase-359 W10 RULING — 3/1 -> 1/0, same rule. `Clock::now()` was TWO
    # functions, one per flavour, and the `std` one answered `SystemTime` where
    # the other answered the platform's wall clock. They are one function now:
    # the port when `platform-clock` is on, the caller-advanced counter when it
    # is not. The 1 that remains is `extern crate std`.
    "nros-core": {"cfg": 1, "path": 0},
    #
    # phase-359 W10: 9 -> 7 cfg, 21 -> 18 path. `nros_cpp_time_ns` was the same
    # clock pair, and it is what `nros::Future::wait()` budgets its spin
    # against — so a hosted image and a target image were timing that loop from
    # different clocks. The remaining sites are the `cpp_diag!` stdio macro and
    # the native-runtime component API, which wants its own pass.
    #
    # phase-359 W10: 7 -> 5 cfg, 18 -> 16 path. The two hosted COMPONENT
    # entries resolve their locator, domain and spin bound from the process
    # environment and build a `CString` — `env` plus `alloc`, no flavour — so
    # they say that now, and this crate gained the `env` feature to say it with.
    #
    # The four that remain are real: `extern crate std`, the `cpp_diag!` stdio
    # macro pair (printing IS the hosted thing), and the tier runtime
    # `nros_board_native_run_tiers` with its `NativeTierSpecC`, which spawns one
    # `std::thread` per tier. That one is portable in principle — the platform
    # task ABI carries name, stack and priority — but `nros-node`'s
    # allocate-spawn-join helper is `pub(crate)`, so porting it means either
    # exposing that internal surface or writing a THIRD copy of the sequence
    # (`nros-node` and `nros-board-nuttx` have the other two). That is an API
    # decision, not a flavour cleanup.
    #
    # phase-359 W10: 5 -> 3 cfg, 16 -> 15 path. `nros_board_native_run_tiers`
    # spawns one PLATFORM TASK per tier instead of one `std::thread`, so its
    # gate is `env` (the locator/domain/spin-bound it reads) rather than the
    # flavour. Two fields the API documented as ignored — `priority` ("advisory
    # ... applied by nobody") and `stack_bytes` ("informational on native") —
    # reach the kernel now, because the ABI attribute carries them.
    #
    # phase-359 W10: 3 -> 1 cfg. `cpp_diag!` was three arms — `eprintln!` on
    # hosted, `nros_log` on Zephyr, and a NO-OP on no_std elsewhere ("nothing to
    # write to"). There is something to write to: `nros_log`'s default sink is
    # `nros_platform_log_write`, which five ports implement and `nros-c`
    # installs for every C/C++ image, so FreeRTOS and ThreadX images were
    # discarding the diagnostics that explain a failed tier. One arm now.
    # Hosted output stays on stderr — the POSIX port writes there too.
    #
    # The 1 that remains is `extern crate std`, which is what the feature IS.
    #
    # phase-359 W10 — the SPELLING pass. Items that live in `alloc` or `core`
    # were named through `std`: `format!`, `String`, `Vec`, `Box`, `Arc`,
    # `atomic::{AtomicBool, Ordering}`. None was a `std` need; each made its
    # crate look like it wanted the flavour. What still names `std` is what only
    # `std` has: `env::var`, `Mutex`, `OnceLock`, `Path`, `fs`, `Instant`.
    #
    # issue 0687 follow-up — 7 -> 2: the two native entries each read
    # `$NROS_LOCATOR` / `$ROS_DOMAIN_ID` and passed them DOWN as the baked rung,
    # which `nros_cpp_init` then re-resolved from the same environment. Deleted,
    # not unified: the resolver was already doing it, and doing it better (the
    # entries dropped the legacy locator name and coerced a bad domain to 0).
    # `$NROS_ENTRY_SPIN_MS` went from two parses at two widths to one. The 2 that
    # remain are that reader and `metadata_hooks`' `fs::write`.
    #
    # issue 0701 — cfg 1 -> 3, and it is the trade this file has recorded twice
    # before (phase-361 W8.e for `metadata-mode`, W10 for `env`): a guard must
    # NAME the feature it checks, so making an implicit requirement explicit
    # costs a counted site and removes an unnamed one. Both capabilities here
    # genuinely need `std` — `metadata-mode` WRITES the sidecar file, `env`
    # reads `$NROS_ENTRY_SPIN_MS` — and both had been free-riding on `nros`'s
    # guards until issue 0669's follow-up correctly relaxed one of them.
    "nros-cpp": {"cfg": 3, "path": 2},
    # 2026-09-06: cfg 1 -> 0. The one site was the crate attribute itself,
    # `#![cfg_attr(not(feature = "std"), no_std)]`, and it bought nothing --
    # this crate's `std` is `std = ["alloc"]` over `alloc = []`, so it enables
    # no std functionality and the crate uses none. The conditional form only
    # meant `std` compiled a DIFFERENT crate: a second configuration nobody
    # wanted and no merge-gating lane built. Now `#![no_std]` unconditionally,
    # held there by `check-core-crates-are-no-std`.
    "nros-log": {"cfg": 0, "path": 0},
    # phase-361 W8.e: +1, the `signal-fd-wake` `compile_error!` guard — the
    # feature used to list `"std"` and now requires it by name.
    # Two changes met here and both are counted. `c3a16a529` (#607) raised
    # this to 108/76 by splitting the env cache on
    # `all(feature = "std", test)` / `not(test)`; phase-359 W10 then removed
    # the OS-priority pool's, the signalfd forwarder's and the condvar path's
    # `std` — so the measured figure after both is 91/40, not either side's
    # number. Set from the tree rather than from arithmetic on the two diffs.
    # phase-359 W10 follow-up: 86 -> 87, the `env` `compile_error!` guard.
    # W10 made the process environment a capability but wrote it
    # `env = ["std"]`, which GRANTS the standard library instead of requiring
    # it — the implicit flavour this campaign exists to remove, and a clause
    # (a) violation that sat red on main. Requiring it costs one counted site
    # and removes one silent enable.
    #
    # Then 87 -> 85: `Executor`'s halt/wake API was ONE `std`-gated impl block
    # and only its wall-clock spin loops need `std`, so it split three ways and
    # `halt_flag` joined `wake_flag` on `alloc`.
    # Same rebase, same rule: issue 0589 moved four `cfg(feature = "std")`
    # diagnostics to `nros_log`. Measured, not derived.
    #
    # phase-359 W10: 84 -> 60, path 36 -> 33. The no_std wake MIRROR is gone.
    # `wake_alloc.rs` held a `WakeCtxAlloc` that was `WakeCtx` minus an
    # `Option`, with its own callback, its own installers and its own field —
    # a second implementation of a thing that had stopped differing when the
    # condvar fallback was deleted. One `WakeCtx` on `alloc + rmw-cffi` now,
    # and the two `primary_drive_timeout_ms` arms are one arm.
    #
    # phase-359 W10: 60 -> 47 cfg. Two clusters. `$NROS_RMW` selection moved off
    # `std` onto `env`, where reading the process environment belongs. And the
    # executor's CLOCK became one provider: `rmw-cffi` means a platform port is
    # linked, so a hosted build now reads `nros_platform_clock_ns` like an
    # embedded one instead of an `Instant` through a separate `clock_base`
    # field. W4 unified the accessor and named the provider as what remained.
    #
    # path 33 -> 34 is that trade paid once: `Instant` survives in exactly one
    # function, the `std`-without-a-port reader for mock-session tests.
    #
    # phase-359 W10: 47 -> 39 cfg, 34 -> 25 path. The THREAD and SPIN-LOOP
    # cluster. `open_threaded` spawns a platform task instead of a
    # `std::thread` (the third and last executor-owned worker to move), and
    # `spin_blocking` / `spin_period` / `spin_one_period_timed` pace on the
    # executor's own `now_us()` plus `nros_platform_sleep_us` instead of
    # `Instant` + `thread::sleep`. `SpinOptions` and `SpinPeriodResult` follow
    # them onto `alloc`.
    #
    # phase-359 W10: 39 -> 28 cfg, 25 -> 22 path. `handles.rs` reaches ZERO.
    # `WaitBudget` was a `std`/`no_std` pair — an `Instant` deadline against an
    # iteration COUNT — so dropping `std` from a hosted consumer silently
    # converted every timeout in that file into "N spins". It now picks on
    # whether a clock EXISTS. `types.rs` keeps one site: the `SystemTime` wall
    # clock for a build with no platform port.
    #
    # phase-359 W10: 28 -> 17 cfg. The singles, and two of them were STALE
    # rather than merely split: `signal-fd-wake`'s guard still demanded `std`
    # for a wake context that moved to `alloc`, and the per-callback sporadic
    # accounting was gated "until a board-side monotonic-microsecond accessor
    # lands" — it landed, so target builds get the accounting too. Also
    # `measure_us` stops asking `cfg!(feature = "std")` and asks whether a
    # clock exists, which is the question it always meant.
    #
    # phase-359 W10: 17 -> 12 cfg. `wake_probe`'s CSV parser and percentile
    # reader are `core` — `str::lines`, `str::parse`, arithmetic — so their
    # gates described who CALLS them (a host analysis tool) rather than what
    # they need; its tests keep a gate, on `alloc`, which is what building a
    # string actually costs. `node_record` had the same call written twice
    # under complementary gates, left over from the wake-mirror merge.
    #
    # phase-359 W10: 12 -> 11 cfg. `handoff` carried its gate TWICE — an inner
    # `#![cfg]` and the `#[cfg]` on its `mod` line — so one of the two was pure
    # duplication. The module stays hosted on purpose: freeing it costs a `spin`
    # dependency edge in a core crate for a public type with no caller anywhere,
    # which is issue 0669's decision to make, not this campaign's.
    #
    # phase-359 W10 — the SPELLING pass. Items that live in `alloc` or `core`
    # were named through `std`: `format!`, `String`, `Vec`, `Box`, `Arc`,
    # `atomic::{AtomicBool, Ordering}`. None was a `std` need; each made its
    # crate look like it wanted the flavour. What still names `std` is what only
    # `std` has: `env::var`, `Mutex`, `OnceLock`, `Path`, `fs`, `Instant`.
    #
    # issue 0687 — cfg 11 -> 10, path 20 -> 7. The `env` capability is GONE from
    # the core: `from_env`, the env cache and `try_resolve`'s hosted rung all
    # read `std::env::var`, and all three moved to `nros::env`. What the core
    # takes instead is a VALUE — `ExecutorConfig::resolve_with(baked,
    # Some(EnvRung { .. }))` — which is both `no_std` and more expressive than
    # the `hosted_env: bool` it replaces (that flag was a compile-time constant
    # at every call site in the tree). The 7 that remain are `Mutex`/`OnceLock`,
    # `Instant` and `ffi`, each carrying its own design question (issue 0687's
    # closing list).
    #
    # issue 0669 — 10/7 -> 9/6: `executor::handoff` is deleted. Unused public
    # API (no call site in the tree since phase 104), `std`-gated on
    # `std::sync::Mutex` alone, and the portable mutex would have cost a `spin`
    # edge on EVERY build of this crate — measured: `spin` is in no board's
    # graph. The six-line pattern it wrapped is written out in the issue.
    #
    # phase-359 W10 RULING — 9/6 -> 3/0. **No `std::` path left in the executor
    # crate.** The clock question is settled by definition rather than by
    # fallback: THE PLATFORM API IS THE CLOCK. A build with a port reads it; a
    # build without one has no default clock and says so (`None`, and a timed
    # spin now fails loud — issue 0709). The `std::time::Instant` and
    # `SystemTime` arms are deleted, and with them the third answer that made
    # "what time is it" depend on whether some crate in the graph named `std`.
    # A caller with a clock but no port installs it through
    # `ExecutorConfig::clock_us` / `Executor::from_session_with`, the same seam
    # a board uses — which is what the 96 mock-session tests now do.
    # The 3 that remain are `extern crate std`, a `dead_code` allow, and the
    # `drive_io` timeout policy for the mock configuration.
    "nros-node": {"cfg": 3, "path": 0},
    #
    # phase-359 W10: 7 -> 5. Both were gates expressing a preference rather than
    # a constraint: the `heapless::String` parameter impl was excluded on `std`
    # although an `alloc`-without-`std` build has always had it AND the `alloc`
    # one (different types, no conflict), and the issue-0323 regression tests
    # asked for `std` to build a `Vec` and a `String`. What remains is real:
    # `extern crate std`, `FileParamStore` (a FILESYSTEM store) with its
    # re-export, and its tests.
    #
    # phase-359 W10: 5 -> 1 cfg, 1 -> 0 path. `FileParamStore` is deleted.
    # Issue 0080 ruled on-device parameter persistence a NON-GOAL in July and
    # listed this type in its own follow-up cleanup; it had no constructor call
    # anywhere in the tree, so it was a filesystem backend for a feature nobody
    # could reach. Its five tests went with it (46 -> 41). The SEAM stays —
    # `ParamStore`/`NullParamStore` are what the executor holds.
    #
    # The 1 is `extern crate std`, and this crate now has NO `std::` path at
    # all: the feature survives only to forward to dependencies.
    "nros-params": {"cfg": 1, "path": 0},
    "nros-rmw": {"cfg": 1, "path": 0},
    "nros-serdes": {"cfg": 1, "path": 0},
}


def is_test_gate(stripped: str) -> bool:
    """A `#[cfg(test)]` / `#[cfg(all(test, ...))]` attribute."""
    return stripped.startswith("#[cfg(") and re.search(r"\btest\b", stripped) is not None


def strip_comments(line: str) -> str:
    """Drop comment text so a doc comment naming `std::` is not a std USE."""
    s = line.strip()
    if s.startswith("//") or s.startswith("/*") or s.startswith("*"):
        return ""
    # Trailing `// …` on a code line. Naive on `"http://"`-style literals, which
    # do not occur in these crates; revisit if that changes.
    return line.split("//", 1)[0]


def census():
    out = {}
    for scope in SCOPE:
        root = REPO / scope
        if not root.is_dir():
            continue
        for crate_dir in sorted(root.iterdir()):
            src = crate_dir / "src"
            if not src.is_dir():
                continue
            cfg = path = 0
            for rs in tracked(src, suffix=".rs"):
                # Generated bindings are not hand-written std use.
                if rs.name == "generated.rs":
                    continue
                # `executor/tests.rs` is declared `#[cfg(all(test, …))] mod tests;`
                # — the whole FILE is host-test code, and the gate lives in the
                # parent module where this per-file scan cannot see it.
                if rs.name == "tests.rs":
                    continue
                # Skip `#[cfg(test)] mod … { … }` bodies by brace depth, and
                # skip whole files that are themselves test modules.
                test_depth = None
                depth = 0
                pending_test = False
                for line in rs.read_text(errors="replace").splitlines():
                    stripped = line.strip()
                    if test_depth is None and is_test_gate(stripped):
                        pending_test = True
                    code = strip_comments(line)
                    opens = code.count("{")
                    closes = code.count("}")
                    if pending_test and opens:
                        test_depth = depth
                        pending_test = False
                    elif pending_test and stripped.endswith(";"):
                        pending_test = False  # `#[cfg(test)] mod tests;`
                    # …and never count the TEST GATE ITSELF. `#[cfg(all(test,
                    # feature = "std", …))]` contains `feature = "std"`, and on
                    # that line `test_depth` is still None — it is only set when
                    # the body's brace opens on a LATER line — so the attribute
                    # was counted as a std requirement of shipped code while its
                    # body was correctly skipped.
                    #
                    # That contradicts this file's own stated policy ("`#[cfg(test)]`
                    # code is excluded, and that is a CORRECTION not a win"), and it
                    # punished the change that made four host-only test modules
                    # DECLARE what they had always needed: `nros-node` went 3 -> 7
                    # for four attributes on `mod` lines, none of which ships.
                    if test_depth is None and code and not is_test_gate(stripped):
                        if "cfg" in code:
                            cfg += len(CFG_FEATURE_RE.findall(code))
                        path += len(PATH_RE.findall(code))
                    depth += opens - closes
                    if test_depth is not None and depth <= test_depth:
                        test_depth = None
            if (cfg or path) and crate_dir.name not in EXCLUDE:
                out[crate_dir.name] = {"cfg": cfg, "path": path}
    return out



# ---------------------------------------------------------------------------
# issue 0701 — the OTHER half of ARCHITECTURE §2 clause (a).
#
# The clause is "a capability REQUIRES the heap / the standard library, it does
# not grant it — emit `compile_error!` naming the feature".
# `check-feature-contract` enforces "does not grant it" by scanning manifests.
# Nothing enforced "emit `compile_error!` naming the feature", so a capability
# whose gated code calls `std::` with no guard passed every gate and failed the
# USER's build with a bare `cannot find crate std`, four frames deep, naming
# nothing they could act on.
#
# It happened: `nros-cpp`'s `metadata-mode` (the sidecar `fs::write`) and `env`
# (`$NROS_ENTRY_SPIN_MS`) had never needed guards BY ACCIDENT — `nros`'s
# stricter guards fired first — until issue 0669's follow-up correctly relaxed
# `nros/metadata-mode` to `alloc`. Relaxing a guard in one crate exposed a
# capability in another that had never named its own requirement.
#
# This lives HERE rather than in its own script because it needs exactly what
# the census walk needs — which cfg gates a given `std::` line — and a second
# spelling of that walk is the antipattern the repo keeps paying for. The
# counting path above is untouched; this is a separate mode.
#
# DELIBERATELY CONSERVATIVE. Only `feature = "x"` and `all(...)` conjunctions
# are attributed; `any(...)` alternatives are ignored, because a site reachable
# through either of two features does not let this gate say WHICH one needs the
# guard. It therefore under-reports rather than crying wolf — a gate nobody
# trusts is worse than a narrow one.
# ---------------------------------------------------------------------------

FEATURE_RE = re.compile(r'feature\s*=\s*"([A-Za-z0-9_-]+)"')
CFG_ATTR_RE = re.compile(r"^#!?\[cfg\((.*)\)\]$")
MOD_DECL_RE = re.compile(r"^(?:pub(?:\([^)]*\))?\s+)?mod\s+([A-Za-z_][A-Za-z0-9_]*)\s*;")
FLAVOURS = frozenset({"std", "alloc"})


def strip_not(expr: str) -> str:
    """Drop `not(...)` groups — a feature required to be OFF is not a need."""
    out, i = [], 0
    while i < len(expr):
        if expr.startswith("not(", i):
            depth, j = 0, i + 3
            while j < len(expr):
                if expr[j] == "(":
                    depth += 1
                elif expr[j] == ")":
                    depth -= 1
                    if depth == 0:
                        break
                j += 1
            i = j + 1
            continue
        out.append(expr[i])
        i += 1
    return "".join(out)


def required_features(expr: str) -> set:
    """Features that must be ON for a `cfg(expr)` site to compile.

    `any(...)` contributes nothing (see the conservatism note above).
    """
    expr = strip_not(expr)
    if "any(" in expr:
        return set()
    return set(FEATURE_RE.findall(expr))


def module_cfgs(src: Path) -> dict:
    """Map each file under `src` to the features its `mod` declaration requires.

    `metadata_hooks.rs` is the case that makes this necessary: its `std::fs`
    call is gated `metadata-mode` at the ITEM, while the file itself is only
    compiled under `rmw-cffi` — declared in `lib.rs`, where no per-file scan can
    see it. The effective gate is the conjunction of the two.
    """
    out = {}
    pending = [(src / "lib.rs", set())]
    while pending:
        parent, inherited = pending.pop()
        if not parent.is_file():
            continue
        out.setdefault(parent, set()).update(inherited)
        attrs = set()
        for line in parent.read_text(errors="replace").splitlines():
            s = line.strip()
            m = CFG_ATTR_RE.match(s)
            if m:
                attrs |= required_features(m.group(1))
                continue
            if s.startswith("#["):
                continue  # a non-cfg attribute does not clear the pending cfgs
            m = MOD_DECL_RE.match(s)
            if m:
                name = m.group(1)
                child_cfgs = inherited | attrs
                for cand in (
                    parent.parent / f"{name}.rs",
                    parent.parent / name / "mod.rs",
                ):
                    if cand.is_file():
                        out.setdefault(cand, set()).update(child_cfgs)
                        pending.append((cand, child_cfgs))
            attrs = set()
    return out


def guarded_features(src: Path) -> set:
    """Features named by a `compile_error!` guard anywhere in the crate."""
    out = set()
    for rs in tracked(src, suffix=".rs"):
        text = rs.read_text(errors="replace")
        for m in re.finditer(r"compile_error!", text):
            # The guard's own `#[cfg(all(feature = "F", not(feature = "std")))]`
            # sits directly above it; 400 chars covers the wrapped spellings in
            # this tree without reaching the previous item.
            head = text[max(0, m.start() - 400) : m.start()]
            cut = head.rfind("#[cfg(")
            if cut >= 0:
                out |= set(FEATURE_RE.findall(head[cut:]))
    return out - FLAVOURS


def site_features(rs: Path, file_cfgs: set):
    """Yield `(line_no, required_features)` for each `std::` site in `rs`."""
    lines = rs.read_text(errors="replace").splitlines()
    stack = []  # (depth_at_open, features)
    pending = set()
    pending_live = False
    depth = 0
    for n, line in enumerate(lines, 1):
        s = line.strip()
        m = CFG_ATTR_RE.match(s)
        if m:
            if s.startswith("#!["):
                file_cfgs = file_cfgs | required_features(m.group(1))
            else:
                pending |= required_features(m.group(1))
                pending_live = True
            continue
        code = strip_comments(line)
        opens = code.count("{")
        closes = code.count("}")
        if code and PATH_RE.search(code):
            here = set(file_cfgs)
            for _, feats in stack:
                here |= feats
            here |= pending  # a cfg'd `use std::…;` on the next line
            yield n, here
        if pending_live:
            if opens:
                stack.append((depth, pending))
                pending, pending_live = set(), False
            elif s.endswith(";"):
                # An item with no body (`mod x;`, `use …;`) — the cfg applied to
                # it and is spent. Anything else is a CONTINUATION: a multi-line
                # `fn` signature holds the cfg until its `{`. Clearing on the
                # first non-attribute line instead lost exactly that case, and
                # the case is `nros_cpp_metadata_dump` — the site this gate was
                # written for.
                pending, pending_live = set(), False
        depth += opens - closes
        while stack and depth <= stack[-1][0]:
            stack.pop()



def default_guard_crates():
    """The nine census crates, plus every other `no_std` crate in `packages/`.

    The census is scoped to the nine because that is what phase-359 ratchets.
    This gate is not: an unnamed `std` requirement is just as opaque to a user
    in a board or a backend, and widening it is FREE — measured at 132 further
    `no_std` crates and zero violations, so the wider scope costs a walk and
    buys the whole tree.
    """
    out = []
    for scope in SCOPE:
        root = REPO / scope
        if root.is_dir():
            out += sorted(root.iterdir())
    skip = {"target", "build", "third-party", "generated", ".git", "node_modules"}
    stack = [REPO / "packages"]
    while stack:
        d = stack.pop()
        for child in sorted(d.iterdir()):
            if not child.is_dir() or child.name in skip:
                continue
            lib = child / "src" / "lib.rs"
            if (child / "Cargo.toml").is_file() and lib.is_file():
                if "#![no_std]" in lib.read_text(errors="replace"):
                    out.append(child)
            stack.append(child)
    seen, uniq = set(), []
    for c in out:
        if c not in seen:
            seen.add(c)
            uniq.append(c)
    return uniq


def guard_check(crate_dirs=None):
    """Report capabilities whose gated `std::` use names no `compile_error!`."""
    bad = []
    if crate_dirs is None:
        crate_dirs = default_guard_crates()
    for _scope in (None,):
        for crate_dir in crate_dirs:
            src = crate_dir / "src"
            if not src.is_dir() or crate_dir.name in EXCLUDE:
                continue
            mod_cfgs = module_cfgs(src)
            guards = guarded_features(src)
            for rs in tracked(src, suffix=".rs"):
                if rs.name in ("generated.rs", "tests.rs"):
                    continue
                for n, feats in site_features(rs, mod_cfgs.get(rs, set())):
                    if feats & FLAVOURS or "test" in feats:
                        continue  # gated on the flavour itself, or test-only
                    caps = feats - FLAVOURS
                    if caps and not (caps & guards):
                        try:
                            shown = rs.relative_to(REPO)
                        except ValueError:
                            shown = rs  # the self-test's synthetic tree
                        bad.append((crate_dir.name, shown, n, sorted(caps)))
    return bad



def self_test():
    """Fire the gate on a deliberate reintroduction, then on its fix.

    The attribution is the subtle part — a site's gate is the CONJUNCTION of its
    module declaration and its enclosing item, and the item's `#[cfg]` can sit
    several attribute lines above a multi-line signature. Both shapes are here
    because both were wrong in the first version of this code.
    """
    import shutil
    import tempfile

    tmp = Path(tempfile.mkdtemp(prefix="nros-0701-selftest-"))
    try:
        src = tmp / "fake-crate" / "src"
        src.mkdir(parents=True)
        (src / "lib.rs").write_text(
            '#![no_std]\n#[cfg(feature = "cap")]\nmod inner;\n'
        )
        # A multi-line signature under a second attribute, which is the shape
        # `nros_cpp_metadata_dump` has.
        (src / "inner.rs").write_text(
            '#[cfg(feature = "writes-a-file")]\n'
            "#[unsafe(no_mangle)]\n"
            "pub extern \"C\" fn dump(\n"
            "    path: *const u8,\n"
            ") -> i32 {\n"
            "    let _ = std::fs::write(\"x\", \"y\");\n"
            "    0\n"
            "}\n"
        )
        crate = tmp / "fake-crate"

        bad = guard_check([crate])
        if len(bad) != 1 or sorted(bad[0][3]) != ["cap", "writes-a-file"]:
            print(f"[self-test FAIL] expected one hit naming both gates, got {bad}", file=sys.stderr)
            return 1

        # Same tree, with the guard the clause asks for.
        (src / "lib.rs").write_text(
            '#![no_std]\n'
            '#[cfg(all(feature = "writes-a-file", not(feature = "std")))]\n'
            'compile_error!("`writes-a-file` writes a file: add \\"std\\"");\n'
            '#[cfg(feature = "cap")]\n'
            "mod inner;\n"
        )
        bad = guard_check([crate])
        if bad:
            print(f"[self-test FAIL] guard present but still reported: {bad}", file=sys.stderr)
            return 1

        # A site gated on the FLAVOUR itself is not a capability violation.
        (src / "inner.rs").write_text(
            '#[cfg(feature = "std")]\n'
            "pub fn t() -> u64 {\n"
            "    std::time::Instant::now().elapsed().as_micros() as u64\n"
            "}\n"
        )
        (src / "lib.rs").write_text('#![no_std]\n#[cfg(feature = "cap")]\nmod inner;\n')
        bad = guard_check([crate])
        if bad:
            print(f"[self-test FAIL] a `std`-gated site is not a violation: {bad}", file=sys.stderr)
            return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("check-std-census --check-guards: self-test OK (3 cases)")
    return 0


def main_guards():
    bad = guard_check()
    if not bad:
        print(
            "capability flavour guards: OK "
            "(every capability-gated `std::` site names a `compile_error!`)"
        )
        return 0
    print(
        "[FAIL] capability-gated `std::` use with no `compile_error!` naming it:",
        file=sys.stderr,
    )
    for crate, rel, n, caps in bad:
        print(f"    {rel}:{n}  reachable only with {caps} (crate `{crate}`)", file=sys.stderr)
    print(
        "\n  ARCHITECTURE §2 clause (a): a capability REQUIRES the standard library,\n"
        "  it does not grant it — emit a `compile_error!` NAMING the feature, e.g.\n"
        '    #[cfg(all(feature = "<cap>", not(feature = "std")))]\n'
        '    compile_error!("`<cap>` <what it does>: add \\"std\\" to this crate\'s features");\n'
        "  Without it the user gets a bare `cannot find crate std`, four frames deep,\n"
        "  naming no feature they can act on (issue 0701).",
        file=sys.stderr,
    )
    return 1


def main():
    if "--self-test" in sys.argv:
        return self_test()
    if "--check-guards" in sys.argv:
        return main_guards()
    now = census()
    show = "--show" in sys.argv

    width = max([len(k) for k in now] + [len("crate")])
    print(f"{'crate':<{width}}  {'cfg':>5} {'path':>5}   baseline")
    regressions, improvements, unknown = [], [], []
    for name in sorted(set(now) | set(BASELINE)):
        cur = now.get(name, {"cfg": 0, "path": 0})
        base = BASELINE.get(name)
        if base is None:
            unknown.append(name)
            note = "NEW — not in baseline"
        else:
            note = f"cfg {base['cfg']}, path {base['path']}"
            for metric in ("cfg", "path"):
                if cur[metric] > base[metric]:
                    regressions.append((name, metric, base[metric], cur[metric]))
                elif cur[metric] < base[metric]:
                    improvements.append((name, metric, base[metric], cur[metric]))
        print(f"{name:<{width}}  {cur['cfg']:>5} {cur['path']:>5}   {note}")

    total_cfg = sum(v["cfg"] for v in now.values())
    total_path = sum(v["path"] for v in now.values())
    print(f"\ntotal: {total_cfg} cfg site(s), {total_path} std:: path(s)")

    if show:
        print("\nBASELINE = {")
        for name in sorted(now):
            v = now[name]
            print(f'    "{name}": {{"cfg": {v["cfg"]}, "path": {v["path"]}}},')
        print("}")
        return 0

    rc = 0
    if unknown:
        print(
            f"\n[FAIL] {len(unknown)} crate(s) carry `std` and are not in the baseline: "
            + ", ".join(unknown),
            file=sys.stderr,
        )
        print(
            "  A new crate in scope must be entered deliberately, not absorbed silently.",
            file=sys.stderr,
        )
        rc = 1
    if regressions:
        print(f"\n[FAIL] {len(regressions)} count(s) went UP:", file=sys.stderr)
        for name, metric, was, now_ in regressions:
            print(f"    {name}: {metric} {was} -> {now_}", file=sys.stderr)
        print(
            "\n  phase-359 is removing these, so an increase is a step backwards.\n"
            "  If the new site is genuinely required, say so in the commit AND raise\n"
            "  the baseline in scripts/check-std-census.py — deliberately, in the diff.",
            file=sys.stderr,
        )
        rc = 1
    if improvements:
        print(f"\n{len(improvements)} count(s) went DOWN — update the baseline:")
        for name, metric, was, now_ in improvements:
            print(f"    {name}: {metric} {was} -> {now_}")
        print("  Run `scripts/check-std-census.py --show` for a paste-ready block.")
        rc = 1
    if rc == 0:
        print("\nstd census: OK (no crate moved)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
