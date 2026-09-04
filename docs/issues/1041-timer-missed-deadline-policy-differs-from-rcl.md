---
id: 1041
title: "A repeating timer CATCHES UP after a stall where rcl SKIPS — N callbacks
  instead of one, and phase-417's node-stack collapse made a ported node meet it"
status: open
type: bug
area: api, core
related: [phase-417, rfc-0087, rfc-0002]
---

## Problem

`Timer::fire` keeps the residual (`nros-node/src/timer.rs:298`):

```rust
TimerMode::Repeating => {
    self.elapsed_ms = self.elapsed_ms.saturating_sub(self.period_ms);
}
```

So after a stall of N periods, `elapsed_ms` holds N × period and the callback
fires once per `spin_once` until it drains — **N invocations**.

`rcl_timer_call` advances the next-call time by whole missed periods and fires
**once**, then resumes on phase.

Both policies are defensible and the difference is not obviously a bug:

* ours never LOSES an invocation, which is right for a counter or a sequence
  number;
* rcl never BURSTS, which is right for a control loop — five integration steps
  delivered back-to-back is a spike, and dropping four is the safer failure.

What is not defensible is that a ported ROS 2 node silently gets the policy it
was not written against.

## Why this became reachable now

Before phase-417's node-stack collapse, the compat shim ran its own dispatch
loop and re-phased on a large overrun:

```cpp
t->next_fire += t->period;
if (now > t->next_fire) t->next_fire = now + t->period;   // rcl's policy
```

That code was an RFC-0019 violation — a second dispatch implementation in the
wrapper, owning timer scheduling and callback ordering — and deleting it was
correct. But it happened to implement rcl's missed-deadline policy, so
**collapsing onto one dispatch path made this one behaviour LESS faithful while
making the structure more so.** Worth stating plainly: the fix was right and it
had a cost, and the cost is this issue rather than a reason to keep two loops.

## Why it cannot be made loud

RFC-0087's rule is that a contract must never differ silently, and that a
refusal fires at the earliest point the defect is knowable. Neither point is
available here: the difference is visible only after a stall that has already
happened, in a callback that is running normally. There is no signature to
reject and no value to inspect at the call.

So this is a case the rule does not cover — a divergence that is neither
compile-time nor call-time detectable — and the only honest options are to
match rcl or to state the envelope. Matching rcl is preferable under the
campaign's own principle (respect the official shape where no platform
constraint forbids it), and nothing here forbids it.

## Fix

Give `Timer` a missed-deadline policy, defaulting to rcl's:

* `Skip` (default) — on fire, `elapsed_ms = elapsed_ms % period_ms`, so whole
  missed periods are dropped and phase is preserved;
* `CatchUp` — today's behaviour, for callers that must not lose an invocation.

Rust-side work in `nros-node`; the C and C++ layers forward, and the knob is a
candidate for the `[knobs.executor]` tenant rather than a Cargo feature
(RFC-0086 D5, issue 1037).

The envelope is documented meanwhile in `rclcpp_compat.hpp`'s `TimerBase`
comment, so the difference is written down where a porting user meets it.

## Also found with it

A shim `create_wall_timer(500us, …)` truncates to a 0 ms period and fires every
spin: `nros_cpp_timer_create` takes whole milliseconds. Sub-millisecond
periods were never honoured — activations always landed on spin boundaries —
but the floor is now explicit where it used to be hidden inside the shim's own
`steady_clock` comparison. A sub-millisecond request should probably be a
build-time error rather than a silent 0.
