---
id: 1175
title: "`nros-c` calls the executor lifecycle methods un-gated, but they are `#[cfg(feature = \"alloc\")]` in `nros-node` — the no-alloc staticlib does not compile"
status: open
type: bug
area: api, core, build
severity: high
found: 2026-09-06
related: [0417, 1162, 1170, 0952]
---

# `check-build` red on main, in the lane no merge-gating event runs

```
cargo clippy --quiet -p nros-c --no-default-features --features "panic-platform,rmw-cffi,lending" -- -D warnings
error[E0599]: no method named `enter_spin_loop` found for mutable reference `&mut nros_node::Executor<'static>`
    --> packages/api/nros-c/src/executor.rs:2891:45
error[E0599]: no method named `is_halted` found ...      executor.rs:2913
error[E0599]: no method named `exit_spin_loop` found ... executor.rs:2917, :2929
```

`packages/api/nros-c/src/executor.rs` is byte-identical between this branch and
`origin/main`, so this is main's. It comes from `4f4b901c7 fix(#417 W4.c): wire
is_spinning into the three loops that actually spin`, which added the call sites
without matching the gate on the other side.

The failing row is the one WITHOUT `alloc`. `nros-c`'s `std` implies `alloc`, so
every other row in `check-workspace-features` passes; the no-default-features
staticlib row is the only one that reaches it, and that row exists because the
staticlib is a FINAL artifact.

## The gate is on the fields, and only two of five are gated by association

`spin.rs`'s lifecycle block is `#[cfg(feature = "alloc")]` with the reason
stated: *"every method here reads `halt_flag` or `spinning` and both are `alloc`
fields."* Per method, that is not uniform:

| method | reads | genuinely needs `alloc`? |
| --- | --- | --- |
| `cancel()` | `halt_flag`, `wake_flag` | **yes** — `halt_flag` is `Arc<AtomicBool>` |
| `is_halted()` | `halt_flag` | **yes** |
| `enter_spin_loop()` | `halt_flag` (clears a stale cancel) + `spinning` | **yes**, through `halt_flag` |
| `exit_spin_loop()` | `spinning` only | no |
| `is_spinning()` | `spinning` only | no |

`spinning`'s own doc comment says as much — *"It rides `alloc` for the same
reason the impl that reads it does"* — so it is gated by association, while
`halt_flag` is an `Arc` and the allocator is a real requirement (phase-359 W10
moved it from `std` to `alloc` deliberately).

So splitting the impl block does not by itself fix this: `enter_spin_loop` and
`is_halted` still need `alloc`, and `nros-c` calls both.

## Six call sites, three C entry points

`rclc_executor_spin` (`enter_spin_loop`, `is_halted`, `exit_spin_loop` x2),
`nros_executor_cancel` (`cancel`), `nros_executor_is_spinning` (`is_spinning`).
The clippy run stops at the first errors, so the reported four are not the whole
set.

## Shape of a fix — and the constraint that decides it

**The C symbols must exist in every build.** A symbol that disappears by cargo
feature is an ABI break against the committed header, which is what
`check-rmw-api-parity` and `check-rmw-abi-shape` exist to hold. So the no-alloc
build needs DEFINED BEHAVIOUR for each entry point, not a missing one.

One helper per verb in `nros-c`, with the `cfg` in one place each rather than
six inline, and each degradation named:

* `enter_spin_loop` / `exit_spin_loop` — no-op. Nothing to record.
* `is_halted()` — `false`. The loop then exits on `state` alone, which is
  exactly what it did before W4.c: a documented return to the old semantics, not
  a new invention.
* `cancel()` — the C `state` transition still happens, and
  `rclc_executor_spin`'s loop reads `state`, so same-thread cancel still works.
  What is lost is the cross-thread / signal-handler wake, which is the whole
  point of the flag. Say so.
* `is_spinning()` — falls back to `state == SPINNING`.

Alternatively, `nros-node` un-gates `spinning` (it needs nothing) and splits the
impl so `exit_spin_loop` / `is_spinning` are always available. That is correct on
its own terms and worth doing, but it is not sufficient — see the table.

Deliberately not fixed here: `executor.rs` is phase-417's file and that campaign
is in flight, so this is filed for its owner rather than edited underneath them.

## The pattern worth naming

This is the **third** main-red found this session that `check-build` catches and
no merge-gating event runs — after issue 1162 (a symbol defined twice) and issue
1170 (its dedup leaving three stale callers). Each was invisible for the same
structural reason, which 1162's own text already states about itself.
`check-build` is `schedule` / `workflow_dispatch` only, and it cannot simply be
added to the merge queue: it needs generated bindings and prebuilt `.compile-ok`
artifacts that no CI job builds, which is why it was removed from the required
set in the first place. The no-alloc staticlib row here costs ~11 minutes and
needs no fixtures — a candidate for a cheap merge-gating subset, which would
have caught all three.
