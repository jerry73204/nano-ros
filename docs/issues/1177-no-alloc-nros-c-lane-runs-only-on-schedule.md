---
id: 1177
title: "The no-alloc `nros-c` lane exists, is RED on main, and no merge-gating
  event runs it -- an embedded consumer finds the break instead"
status: open
type: bug
area: testing, build, c-api
severity: high
related: [issue-0196, issue-0952, phase-395, phase-417]
---

## What happens

`check::workspace-features` builds `nros-c` without `std` and without `alloc`:

```
cargo clippy -p nros-c --no-default-features \
    --features "panic-platform,rmw-cffi,lending" -- -D warnings
```

On `origin/main` at `fdc0604c5` that command reports **11 errors**. The lane is
not missing. It is red, and nothing that gates a merge runs it:

* `workspace-features` sits in the `build-serial` tier, i.e. `check-build`.
* `gate.yml` states, in its own comment, that `check-build` is **schedule /
  dispatch ONLY** -- `pull_request` gets `check-fast + compile-smoke +
  cli-tests`, `merge_group` adds `test-unit + workspace-all`, and only the
  daily `schedule` adds `check-build`.

So a no-alloc break can land on a green PR, pass the merge queue, and sit on
main until either the nightly notices or a consumer does.

## How it surfaced

The safety island (MR-CANHUBK344, `thumbv7em-none-eabihf`, no `std`, no
`alloc`) could not build against main. `nros-c`'s spin loop calls five methods
that no longer exist for it:

```
error[E0599]: no method named `enter_spin_loop` found for
              `&mut Executor<'static>`
    --> packages/api/nros-c/src/executor.rs:2891:45
```

`4b80db633` gated `impl Executor` on `#[cfg(feature = "alloc")]`, which is
CORRECT for `nros-node` -- `halt_flag` is an `Arc` and genuinely needs alloc.
What it did not do is gate, stub, or otherwise handle the five ungated callers
in `nros-c`: `enter_spin_loop`, `exit_spin_loop`, `is_halted`, `cancel`,
`is_spinning`. The fix moved the breakage from one crate to the next, and the
lane that would have said so does not run on a pull request.

The commit before it, `4b80db633~1`, does not build the island either -- it
fails with duplicate `halt`/`is_halted` definitions and a missing `spinning`
field, which is what `4b80db633` was fixing. phase-417 is mid-flight and main
is red for embedded consumers in a moving way; this issue is about the
COVERAGE, not about that phase's in-progress state.

## Why this is the issue-0196 shape

A gate that never runs is indistinguishable from a gate that passes. Three
feature-gate defects of exactly this form -- a `#[cfg]`-gated item with an
ungated caller -- were found by hand on 2026-09-05/06 rather than by CI:

1. `nros_platform_api::task` gated on `alloc` while the executor's stack-
   headroom rule called it on every feature set;
2. `SimTimeGuard` gated on `sim-time` while `ros_time_timer_follows_the_
   simulated_clock` was not;
3. this one.

Each was invisible to the lanes that DO gate a merge, because those lanes build
with `std`, and `std` implies `alloc`.

## The tension, stated rather than wished away

`check-build` was moved off the merge group for a real reason, recorded in
`gate.yml` and in `check-lane-contracts`: it resolves generated bindings and
prebuilt `.compile-ok` artifacts that no CI job builds, so it could never pass
there and left the required check red for every pull request for a day. The
rule that came out of it -- a gate in an affordability tier may only resolve
artifacts the job itself builds -- is right and should not be reverted.

So the fix is NOT "put check-build back on the merge group".

## Directions

* **Move only the FEATURE checks.** `workspace-features` is clippy over feature
  combinations. If it needs no fixture and no SDK, it belongs in a tier that
  runs per PR. The obstacle is named in `lanes.just` itself: `nros-c`'s build
  script needs a cross C toolchain, which is why `nros-c` is absent from
  `check::no-std`. Whether the PR runner has one is the question to answer
  first, and it is answerable in one CI run.
* **Or add a narrow no-alloc smoke to `compile-smoke`**, which already runs per
  PR, covering just the crates an embedded consumer links.
* **Or accept the daily latency and make it loud** -- `just nightly-triage`
  exists; a red `workspace-features` that nobody reads is the same defect one
  level up.

## Do not

Do not fix the symptom by giving `nros-c` `std` in the lane. The whole point of
the combination is that an embedded image has neither `std` nor `alloc`, and a
lane that builds the easy configuration is the lane we already have.
