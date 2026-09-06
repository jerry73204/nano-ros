---
id: 1175
title: "`nros-c` calls the executor lifecycle methods un-gated, but they are `#[cfg(feature = \"alloc\")]` in `nros-node` — the no-alloc staticlib does not compile"
status: wontfix
type: bug
area: api, core, build
severity: high
found: 2026-09-06
resolved: 2026-09-06
related: [0417, 1162, 1163, 1170, 1177, 0952]
---

> **DUPLICATE of issue 1177 — retracted 2026-09-07.** Diagnosed and fixed
> independently while `9fd7e3457 fix(#1177): the halt flag needs an allocator to
> be SHARED, not to exist` was landing on main. Same symptom, same call sites.
> 1177 has the number.
>
> **Their fix is better and is what ships, and the difference is not cosmetic.**
> I read `halt_flag` as needing `alloc` because it is an `Arc<AtomicBool>`, so I
> kept `cancel` / `halt` gated and gave `nros-c` a `#[cfg(feature = "alloc")]`
> on its one `cancel()` call — which makes cancellation a silent no-op on a
> no-`alloc` image. 1177 saw the real requirement: the allocator is needed to
> SHARE the flag (`halt_flag()` hands out a refcounted handle), never to HOLD
> it, so the field is a plain `AtomicBool` without `alloc` and the whole
> lifecycle block un-gates. `cancel` keeps working on a no_std executor, and
> `nros-c` needs no cfg at all.
>
> My spin.rs and nros-c changes were dropped in favour of theirs. What is kept
> below is the diagnosis of how it stayed hidden — the no-`alloc` staticlib row
> is the only one in `check-workspace-features` that reaches it, and
> `check-build` runs on no merge-gating event (issue 1040, and issue 1163 for
> the gate that would catch it).

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

## Fixed — the gate follows the FIELD, not the block

Only `halt_flag` needs an allocator, so only what touches it stays gated:

* `spinning` is un-gated — a plain `AtomicBool`, and its own doc comment already
  admitted it rode `alloc` "for the same reason the impl that reads it does".
* `is_spinning` / `exit_spin_loop` move to an un-gated impl block, unchanged.
* `enter_spin_loop` moves too, with the stale-cancel clear as the one
  `#[cfg(feature = "alloc")]` line inside it. Without a flag there is no stale
  cancel to clear.
* `is_halted` moves too and answers `false` without `alloc`. Truthful rather
  than a stub: `cancel()` is the only thing that could set the flag, and it is
  not compiled in either, so nothing can have asked this executor to stop.
* `cancel` and its deprecated `halt` forwarder stay `alloc`-gated. They STORE
  into an `Arc<AtomicBool>`, and there is no fallback because there is no flag.

That leaves **one** `cfg` in `nros-c`, on the single `cancel()` call, instead of
six. Gating each of the six call sites would have made every one of them invent
the same fallback independently.

**What a no-`alloc` build loses, precisely.** `nros_executor_cancel` still
performs the C `state` transition, and `rclc_executor_spin`'s loop condition
reads `state`, so a cancel from the same thread still stops the spin. What the
flag buys and this build cannot is the cross-thread / signal-handler wake:
`state` is readable only by whoever holds the struct, while the flag is what a
`drive_io` blocked mid-poll is woken on. Such an image waits out its poll
interval — the same bounded behaviour `cancel`'s own ADOPT-BOUNDED envelope
already promises. The C symbol stays defined in every build either way.

## Acceptance

```
cargo clippy -p nros-c --no-default-features \
  --features "panic-platform,rmw-cffi,lending" -- -D warnings    rc=0
just check workspace-features    All feature checks passed!
just check node-std-tests        PASS
just check c / cpp / no-std      PASS
```

The `cfg(not(alloc))` arm is reachable by construction: that clippy row failed
`E0599` on `is_halted` before this change and compiles after, so the arm being
compiled there is the one the fix added.

## The gate hole is already filed, as 1163

This is the third main-red found this session that `check-build` catches and no
merge-gating event runs — after issue 1162 (a symbol defined twice) and 1170
(its dedup leaving three stale callers). **Issue 1163 already names this gate**
and proposes the fix: put the SHIPPED feature set into `compile-smoke`, read
from `check-c`'s one spelling, plus a `check-lane-contracts` rule for the class.
That is the right layer and it is in flight (PR 631), so nothing competing is
added here — this issue is the symptom, 1163 is the gate.

Worth recording for 1163's benefit: the row that catches THIS one is
`-p nros-c --no-default-features --features "panic-platform,rmw-cffi,lending"`,
the no-`alloc` staticlib. It needs no fixtures and no ROS, and it is the only row
in `check-workspace-features` that reaches the defect — `nros-c`'s `std` implies
`alloc`, so every other row compiles the gated methods and passes.
