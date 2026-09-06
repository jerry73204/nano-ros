---
id: 1192
title: "The executor's blocking wait is not bounded by the next timer deadline, so it sleeps past a deadline it owns"
status: open
type: bug
area: executor
related: [issue-0505, issue-0515, phase-436]
---

## Problem

`spin_once` bounds its blocking wait by two things and only two: the caller's
`timeout`, and `session.next_deadline_ms()` — the *backend's* next internal
event (lease keepalive, heartbeat, ACK-NACK).

```rust
let timeout_ms = match self.session.next_deadline_ms() {
    Some(next) => timeout_ms.min(next.min(i32::MAX as u32) as i32),
    None => timeout_ms,
};
```
(`packages/core/nros-node/src/executor/spin.rs:6412`)

Nothing caps it by the next **timer** expiry. Grepping `spin.rs` for any
next-timer computation — `next_timer`, `earliest`, `until_next`, `next_expiry`,
`soonest` — returns zero hits.

So the executor's wake set is transport-only, while the one class of deadline
the executor itself *owns* — its registered timers — cannot shorten the sleep.

## Evidence

`spin_default()` uses a 50 ms timeout (`spin.rs:7361`). Register a 10 ms timer
under it and the executor blocks up to 50 ms in `NodeWake::wait_ms` (or in the
transport's `recv`), during which the 10 ms deadline passes unserved. Timers
are only evaluated in the readiness scan *after* the wait returns, driven by
the accumulated `delta_us` (`spin.rs:6847`).

The consequence at the callback is then the backlog replay that **issue 0505**
described and fixed policy for: several activations back to back once the
executor runs again.

## Why this is not 0505 or 0515

* **0505** (resolved) is the *overrun policy* — what to do about a backlog that
  has already accumulated. It takes the stall as given.
* **0515** (resolved) is *grid quantization* — a timer period that is not a
  multiple of the tier's spin period alternates between two values. Its model
  is explicitly "a timer fires on the first spin boundary at or after its
  period elapses", and the fix was a runtime warning naming the two values.

Both accept the spin boundary as fixed and describe what happens around it.
This issue is that **the boundary should move**: a wait that knows a timer is
due in 10 ms must not sleep for 50 ms. Fixing it removes the input to 0505's
backlog and shrinks 0515's grid to the timer's own deadline, without replacing
either — 0515's warning still catches hand-written loops, and 0505's policy
still governs a genuine preemption stall.

## Fix direction

Compute the wait as `min(caller timeout, next timer expiry, next_deadline_ms)`.
The timer headers are already walked once per executor in
`audit_spin_quantization` (`spin.rs:2340`), so the traversal exists; it needs to
become a per-spin `min` over `period_us - elapsed_us` for live timer entries.

Two constraints on the fix:

* It must not turn a long idle wait into a poll. With no timers registered the
  bound is absent and the caller's timeout stands, unchanged.
* The result is what makes the wait **analyzable**: the wake time becomes
  `min` over a set of declared deadlines rather than an arbitrary caller
  constant, which is the property a schedulability argument needs.

See also issue 1193 — the bound cannot be expressed below 1 ms today, so a
sub-millisecond timer deadline is unrepresentable even once this is fixed.
