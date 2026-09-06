---
id: 1169
title: "`nros-c`'s cancel/is_spinning surface needs `alloc` unconditionally, and the crate's no-alloc \"lending\" build has never compiled"
status: open
type: bug
area: build, api
severity: high
found: 2026-09-06
related: [phase-413, phase-417, issue-0812, issue-0619]
---

## Symptom

`just check` (and therefore `just ci tier1`, and therefore the `host-tests`
workflow's *integration* job) is red at:

```
$ cargo clippy --quiet -p nros-c --no-default-features --features "panic-platform,rmw-cffi,lending" -- -D warnings

error[E0599]: no method named `enter_spin_loop` found for mutable reference `&mut nros_node::Executor<'static>` in the current scope
    --> packages/api/nros-c/src/executor.rs:2891:45
error[E0599]: no method named `is_halted` found for mutable reference `&mut nros_node::Executor<'static>` in the current scope
    --> packages/api/nros-c/src/executor.rs:2913:53
error[E0599]: no method named `exit_spin_loop` found ...  (x4, two call sites x2 functions)
error[E0599]: no method named `cancel` found for mutable reference `&mut nros_node::Executor<'static>` in the current scope
    --> packages/api/nros-c/src/executor.rs:3096:45
error[E0599]: no method named `is_spinning` found for mutable reference `&mut nros_node::Executor<'static>` in the current scope
    --> packages/api/nros-c/src/executor.rs:3145:43

error: could not compile `nros-c` (lib) due to 10 previous errors
```

Reproduced locally, no fixture build needed (`cargo clippy` alone, ~1s once
deps are warm). This is the `just check` line labelled
`"nros-c: lending (publish surface, no alloc)"` in `just/check/lanes.just`
(issue 0812/0814's own gate for the no-alloc publish-only surface).

## How this was found

Diagnosing phase-413 W2 item 1 ("`host-tests` dies in *Build workspace
fixtures*"). That specific step is **no longer broken** — two unrelated
build breaks that used to fail it (issue 1162, duplicate
`nros_node_get_fully_qualified_name` definition; issue 1136, the `LAUNCH_ARGS`
cmake keyword regression) are both already fixed and are ancestors of current
`main`/HEAD. The most recent `host-tests` run
(`34027078269`, head `fdc0604c5b`) confirms *Build workspace fixtures* is
green; the job is still red because the **next** step, `just ci tier1`
(which runs `just check`), now hits this.

A second, unrelated but adjacent defect was found and fixed in the same
sweep: three unit tests in `packages/api/nros-c/src/node.rs` still called
`nros_node_get_fully_qualified_name` with the pre-#1162 3-argument signature
(the merge that resolved #1162 kept the 4-argument form but missed these
three call sites in the crate's own test module). Fixed directly — trivial,
mechanical, matches the file's own `core::ptr::null_mut()` idiom used
elsewhere for the same parameter.

## Cause

`phase-417 W4.c` added `rclcpp`-style `Executor::cancel()` / `is_spinning()`
to `nros-node`, and wired the C API (`nros-c/src/executor.rs`) to call them
unconditionally from `rclc_executor_spin`, `rclc_executor_spin_period`,
`nros_executor_cancel`, and `nros_executor_is_spinning` — with **no
`#[cfg(feature = "alloc")]` gate** at any of those call sites.

The methods themselves — and the fields behind them — are gated:

```rust
// packages/core/nros-node/src/executor/spin.rs
#[cfg(feature = "alloc")]
pub(crate) halt_flag: portable_atomic_util::Arc<portable_atomic::AtomicBool>,
#[cfg(feature = "alloc")]
pub(crate) spinning: portable_atomic::AtomicBool,
...
#[cfg(feature = "alloc")]
impl<'s> Executor<'s> { pub fn cancel(&self) { ... } }
// same impl block: is_halted, is_spinning, enter_spin_loop, exit_spin_loop
```

`nros-c`'s own `default` feature set is `["panic-platform"]` (deliberately —
see the crate's doc comment: a bare `staticlib`/`cdylib` build is a *final*
artifact and only needs a panic handler, not `alloc`). The crate has an
established "no-alloc publish-only surface" (the `lending` feature's receive
half is already `cfg`-gated out without `alloc`, per the comment at
`just/check/lanes.just:1554`) — this is precedented, not novel.

So a `nros-c` build without `alloc` (the `lending`-no-alloc row, and likely
any real embedded no-alloc consumer that calls into `rclc_executor_spin`)
does not compile.

### MEASURED (2026-09-06): this is not a regression — it never compiled

The first draft of this issue said "no longer compiles", implying phase-417's
`#[cfg(feature = "alloc")]` on the lifecycle impl broke a working build. Checked
rather than assumed, by building the same row at the commit BEFORE that gate
(`4b80db633~1`):

```
error[E0609]: no field `halt_flag` on type `&executor::spin::Executor<'s>`
error[E0609]: no field `spinning` on type `&executor::spin::Executor<'s>`
```

Broken there too, with different errors. Before the gate the impl was un-gated
while its FIELDS were `alloc`-only, so `nros-node` itself could not build
without `alloc`; the gate fixed that and moved the same underlying gap one crate
along, from "the executor does not compile" to "`nros-c` calls methods that are
not there". `spinning` did not exist at all until the same commit added it.

That matters for the fix direction below: **no-alloc cancellation has never
worked**, so restoring it is an ADDITION, not a repair, and the "did it ever
work?" open question is answered — no.

## Why this is not a quick `#[cfg]` patch

`is_halted()` is **load-bearing for cancellation**, not just observability.
Inside `rclc_executor_spin`'s loop:

```rust
while executor_ref.state == ...SPINNING && !get_executor(...).is_halted() { ... }
// comment: "The cancel flag is the second exit, and it is the one
// `nros_executor_cancel` raises from a signal handler: `state` is only
// readable by whoever holds the struct, while the flag is what a
// `drive_io` blocked mid-poll is woken on."
```

Simply `#[cfg(feature = "alloc")]`-gating the call sites (making them no-ops
without `alloc`) would silently make a no-alloc `rclc_executor_spin` **only**
exitable via `session_io_failures()` hitting `SPIN_ERROR_TOLERANCE` — i.e.
`nros_executor_cancel()` from a signal handler would stop working for any
no-alloc consumer, with no compile error to say so. That is a real
capability regression hiding behind a green build, which this codebase's own
practice treats as worse than the current honest compile error (CLAUDE.md:
"the honest failure ... is the policy working").

Open questions for whoever picks this up:

1. ~~Did a no-alloc `Executor` support externally-triggered cancellation~~
   ANSWERED above: no, it never compiled. What remains is whether it SHOULD.
   *before* phase-417 W4.c (e.g. via a plain, non-`Arc` atomic that didn't
   need `alloc`), and W4.c's rename/consolidation onto `halt_flag`/`Arc`
   silently dropped it? Or was cancellation always an `alloc`-only
   capability, and the C API simply never had a no-alloc row that exercised
   it until 0812/0814's `lending`-no-alloc gate started building
   `executor.rs` in that configuration?
2. If cancellation should stay available without `alloc`: `halt_flag` needs
   a non-`Arc` `portable_atomic::AtomicBool` field for the no-`alloc` case
   (own storage, no cross-thread handle) — a second field shape, not just a
   cfg on the impl.
3. If cancellation is legitimately `alloc`-only: the `nros-c` C API needs a
   real no-alloc behaviour for `nros_executor_cancel` / `_is_spinning`
   (probably: keep exporting the symbols, `cancel()` sets `state` directly
   instead of the flag so the *state* check alone still exits the loop
   promptly, and `is_spinning()` reads `state == SPINNING` instead of a
   dedicated flag) rather than a `#[cfg]`-out that changes what the same C
   header promises depending on how the crate was built.

## Reproduction

```
cargo clippy --quiet -p nros-c --no-default-features --features "panic-platform,rmw-cffi,lending" -- -D warnings
```

No fixture build required.

## Scope note

This is a *different* class from issue 1156 (a Zephyr cmake feature-string
that misses `alloc` on one of two build paths) — same symptom family (a
capability added behind `alloc` reaches only one of the crate's build
configurations) but a different site and, here, an open design question
about whether the capability can even be made no-alloc-safe rather than a
pure wiring fix.
