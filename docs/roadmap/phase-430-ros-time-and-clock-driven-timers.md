# Phase 430 — ROS time, and timers that work in production and under rosbag replay

**Status (2026-09-05). Planned.** Adds a second time base so a nano-ros image
can be debugged against a rosbag. Amends RFC-0089's timer study, which concluded
`GenericTimer`'s clock parameter was unportable *because we had one clock* —
that premise is what this phase changes.

## What ROS time is, and what it costs

ROS time is a clock driven by the `/clock` topic
(`rosgraph_msgs/msg/Clock`) instead of by hardware. Under `use_sim_time`, a
node's ROS clock advances only when a `/clock` message arrives. It can **pause**
(the player is stopped), **jump forward** (seek), **jump backward** (loop), and
run at a rate unrelated to wall time.

Three consequences that shape the API more than the clock itself does:

1. **A ROS-time timer's wake source is a MESSAGE, not a timeout.** A wall timer
   is "sleep until deadline"; a ROS-time timer is "fire when the received time
   passes the deadline". They cannot share one wait computation.
2. **Time can go backwards.** A bag loop rewinds `/clock`, and a timer holding a
   future deadline in old time would either stall or fire a storm. rcl handles
   this with jump callbacks; we need the same rule stated.
3. **A ROS-time timer with no `/clock` must NOT fall back to wall time.** That
   is exactly the compile-and-differ this campaign forbids, moved to runtime: a
   debug session would silently behave like production and the bag would look
   correct.

## The proposed shape

**Still flat. No hierarchy, no vtable** — this is the outcome RFC-0089's timer
study predicted for the case where ROS time arrives, and the prediction holds:
the executor dispatches through a raw function pointer in the arena, so nothing
ever needs a type-erased base.

Upstream distinguishes wall from clock-driven timers **by type**
(`WallTimer` vs `GenericTimer<ClockT>`) *and* **by verb**
(`create_wall_timer` vs `create_timer(clock, …)`). The TYPE distinction is not
portable — it needs the hierarchy we refuse — but **the verb distinction is, and
the verb is what ported code writes**:

```cpp
// steady clock, always, unaffected by /clock. Already exists.
Result create_wall_timer(Timer& out, uint64_t period_ms, cb, void* ctx) noexcept;

// clock-driven. NEW.
Result create_timer(Timer& out, Clock& clock, uint64_t period_ms, cb, void* ctx) noexcept;
```

`Timer` gains a clock-source field — a small enum, not a pointer to a
polymorphic clock — so the type stays one word wider and
`check-cpp-capability-layout` still holds.

`Node::create_timer(period, cb)` without an explicit clock follows **the node's
own clock**, which is ROS time when `use_sim_time` is set and steady otherwise.
That is upstream's rule and it is what makes a ported file behave correctly
under replay without an edit.

## Work items

* **W1 [rust] — a second time base in the executor.** The scheduler keeps wall
  deadlines as today and gains a ROS-time deadline set evaluated only when ROS
  time advances. The wall timeout passed to the platform wait is computed from
  wall timers ONLY; ROS-time timers are checked after each `/clock` update.
  *Acceptance:* a test advances ROS time in steps and asserts a 100 ms
  ROS-timer fires once per 100 ms of RECEIVED time, at whatever wall rate the
  steps arrive.

* **W2 [rust] — the `/clock` subscription and `use_sim_time`.**
  `rosgraph_msgs/msg/Clock`, subscribed when the parameter is set. Depends on
  phase-426, because `use_sim_time` must be a real parameter in the Rust store
  rather than a third place to keep node state.
  *Acceptance:* `ros2 bag play` drives an in-image node's ROS clock;
  `ros2 param set use_sim_time` at runtime is either honoured or refused
  loudly, not ignored.

* **W3 [rust] — jump semantics.** Backward jumps reset outstanding ROS-time
  deadlines rather than firing a catch-up storm; forward jumps fire **at most
  once** per timer. Pausing stops ROS timers and leaves wall timers running.
  *Acceptance:* a bag loop does not produce a burst at the wrap; a 10-second
  seek fires each timer once.

* **W4 [api] — the two verbs in C, C++ and Rust.** `create_timer(clock, …)`
  beside the existing `create_wall_timer`, and `Node::create_timer` following
  the node clock.
  *Acceptance:* a ported rclcpp file using `create_wall_timer` is unaffected by
  `/clock`; the same file using `node->create_timer(clock, …)` follows the bag.

* **W5 [loudness] — no silent fallback.** A ROS-time timer that has never
  received a `/clock` message does not fire, and the image says so once rather
  than looking idle.
  *Acceptance:* a test with `use_sim_time` set and no publisher asserts zero
  firings AND a diagnostic; a reviewer can tell "paused" from "misconfigured".

* **W6 [cost] — the embedded budget.** `/clock` is a subscription: an arena
  slot, a buffer, and a topic. Feature-gated, off by default on freestanding
  targets, and the cost is documented next to `ZPICO_MAX_QUERYABLES`'s.
  *Acceptance:* an image without the feature has byte-identical timer
  scheduling to today's.

## Why this is worth the second time base

Replay debugging is the case a fixed-arena embedded image is worst at
reproducing by hand: the input is a recorded graph, and without ROS time the
image runs at wall rate against data that was captured at another. Every other
approach — rewriting timestamps, rate-limiting the player — moves the problem
into the test harness, where it is one more thing that can be wrong.

## Amends

RFC-0089 §"Timer, studied against RTOS semantics" concluded `GenericTimer` was
unportable because the clock it is generic over did not exist. That reasoning
was correct and its premise is now scheduled to change. **The conclusion does
not:** the clock becomes a runtime field and a second verb, not a type
parameter and not a hierarchy, because the reason the hierarchy was refused —
the executor needs no type-erased base — is unaffected by having two clocks.
