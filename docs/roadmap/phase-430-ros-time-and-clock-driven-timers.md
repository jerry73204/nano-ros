# Phase 430 — ROS time: the delta after phase-425

**Status (2026-09-07). MEASURED. Phase-425 owns sim-time and shipped it; this
phase is exactly the delta below, and nothing else.**

This document was first written as if ROS time were unstarted work. It was not:
phase-425 ([`phase-425-ros-time-clock-semantics.md`](phase-425-ros-time-clock-semantics.md))
landed `rosgraph_msgs/msg/Clock`, the `/clock` time source, `use_sim_time`,
clock-driven timers and an end-to-end fixture while this was being drafted, and
the collision was found in a rebase conflict rather than by looking. The
2026-09-06 rescope kept the design argument and said the delta was NOT yet
measured. It is now, item by item, against the code on `origin/main` at
`1976727d8` and RFC-0089's ROS-time decisions.

The design reasoning this document used to carry — what ROS time is, the three
properties, the flat-`Timer`-plus-verb shape — is not repeated here. It lives in
RFC-0089 §"Timer, studied against RTOS semantics" (with the 2026-09-05
amendment) and in phase-425's "The situation". Where a property turned out to
be answered differently from what this document argued, the table says so and
the design section is not restated to argue back.

## Delta, measured (2026-09-07)

Verdicts: **DONE** (425 shipped it — commit, file:line), **PARTIAL** (what is
missing, one line), **NOT STARTED**, **NO LONGER APPLIES** (the design settled
it another way). Commits are on `origin/main`; line numbers are the tree this
was measured in, `phase-428-w10-qos-ssot` at `c27128a40` (45 commits ahead of
`main`, none behind).

| # | Item (source) | Verdict | Evidence |
| --- | --- | --- | --- |
| 1 | Property 1 — a ROS-time timer's wake source is a message; the wall timeout handed to the platform must come from wall timers only (430 W1) | NO LONGER APPLIES | The executor derives NO wait from timers at all: `spin_once(timeout)` is the caller's, capped only by the session's next internal deadline (`executor/spin.rs:6441`). Every timer, wall or ROS, is polled in `timer_try_process` (`executor/arena.rs:1959`) and fires on a spin boundary; a `/clock` arrival is data, so it wakes `drive_io` like any sample. There is no second deadline set to build. The latency bound is the spin cadence, which is why `bins/sim-clock-listener` spins at 5 ms. |
| 2 | A 100 ms ROS timer fires once per 100 ms of RECEIVED time at whatever wall rate the samples arrive (430 W1 acceptance) | DONE | `8cfd19315`; `ros_time_timer_follows_the_simulated_clock` step 2 (`executor/tests.rs`, "RATE": four 100 ms steps in ~40 ms wall = four activations); end-to-end `tests/sim_time_clock_e2e.rs` (`6ec3764f2`) measured ~98 ROS activations/s against 10 wall on a 10x replay. |
| 3 | `rosgraph_msgs/msg/Clock` + the `/clock` subscription (430 W2) | DONE | `caa546fb2` (`packages/interfaces/rosgraph-msgs`), `f176079a5` (`Executor::install_ros_time_source`, `executor/spin.rs:9291`; `time_source.rs`), `QoSProfile::clock_default()` = rclcpp's `ClockQoS`. |
| 4 | `use_sim_time` is a real parameter that attaches the source (430 W2) | DONE | `5200437c5`; hook at `Executor::declare_parameter` → `note_reserved_parameter` (`executor/spin.rs:7943`, `:7971`), reconciled at the head of every spin (`reconcile_ros_time_source`, `:9221`). Unit: `use_sim_time_attaches_and_detaches_the_clock_source`, `a_non_bool_use_sim_time_attaches_nothing`. |
| 5 | A runtime `ros2 param set … use_sim_time true` is honoured or refused loudly (430 W2 acceptance) | PARTIAL | Honoured when the app DECLARED `use_sim_time` (`refresh_use_sim_time_from_store`, `executor/spin.rs:9257`, after a parameter service handled a request). On a node that never declared it the store refuses the set (`allow_undeclared` is false, `nros-params/src/server.rs:216`) — refused, but not how ROS 2 behaves: rclcpp declares `use_sim_time=false` on EVERY node, so the `ros2 param set` works everywhere and `ros2 param list` shows it. Ours shows nothing until the app names it. → **W2**. |
| 6 | Backward jump resets outstanding deadlines instead of stalling or storming (430 W3, property 2) | DONE | `executor/arena.rs:1970` (`step_ns < 0` → `elapsed_us = 0`, no activation); test step 3 ("back 9.4 s … the NEXT period must fire on schedule"). |
| 7 | Forward jump fires at most once per timer (430 W3) | DONE | Under the default `TimerOverrunPolicy::Skip` a backlog coalesces into ONE activation (`executor/arena.rs:1981`); `CatchUp` replays, by explicit opt-in. Documented in `book/src/user-guide/simulated-time.md` "A jump forwards". |
| 8 | Pause stops ROS timers and leaves wall timers running (430 W3) | DONE | Test step 1 (360 ms wall, 0 ROS activations, ≥3 wall); e2e asserts the ROS timer stops DEAD when `/clock` stops while the wall timer keeps cadence. |
| 9 | C++ `create_timer(clock, …)` beside `create_wall_timer` (430 W4; RFC-0089's verb) | DONE | `2829ab92a` freed the name (no alias); `8cfd19315` added `Node::create_timer(Timer&, const Clock&, period_ms, cb, ctx)` (`nros-cpp/include/nros/node.hpp:794`) and `nros::create_timer(Node&, Timer&, const Clock&, ms, fn)` (`std_compat.hpp:111`). ONE dispatch: `nros_cpp_timer_create_on_clock` → `Executor::register_timer_on_clock` (`nros-cpp/src/timer.rs:145`). |
| 10 | The Rust verb (430 W4) | PARTIAL | Executor level DONE: `Executor::register_timer_on_clock(period, TimerClockSource, cb)` (`executor/spin.rs:5124`), `nros::TimerClockSource` re-exported (`api/nros/src/lib.rs:1028`). NOT on `NodeCtx` (its only timer verb is `create_timer_in`, `executor/node.rs:1567`) and NOT on the declarative `nros::Node::create_timer` (`api/nros/src/node.rs:1084` — `EntityMetadata` has no clock field, and `node_runtime.rs:1459` always calls `register_timer`). A `nros::main!` component cannot own a ROS-time timer today. → **W4**. |
| 11 | The C verb (430 W4; RFC-0089 "C takes rcl's spellings") | NOT STARTED | `nros_timer_init(timer, support, period_ns, cb, ctx)` (`nros-c/src/timer.rs:117`) takes no clock; `rcl_timer_init` takes an `rcl_clock_t*`. `rclc_executor_add_timer` registers via `register_timer` only (`nros-c/src/executor.rs:1931`). → **W5**. |
| 12 | `ComponentNode` (430 W4) | NOT STARTED | Only `create_wall_timer` (`component_node.hpp:425`, `:442`); no clock-taking overload, no `NROS_CREATE_TIMER` macro. → **W6**. |
| 13 | The hosted `rclcpp::` spelling a ported file writes (430 W4 acceptance: "the same file using `create_timer(clock, …)` follows the bag") | NOT STARTED | rclcpp humble has exactly one clock-taking form, the FREE `rclcpp::create_timer(node, clock, period, cb[, group])` (`docs/reference/api-surface/rclcpp.json:9545`). The hosted shim has `rclcpp::Node::create_wall_timer` → `TimerBase::SharedPtr` (`nros.hpp:812`) and nothing for the clock-taking verb. A ported file using ROS time fails to compile — honest under the compile-or-conform rule, but the verb 430 called portable is the one that is missing. → **W6**. |
| 14 | `Node::create_timer(period, cb)` with no clock follows the node's clock (430 W4) | NO LONGER APPLIES | Not in the captured upstream surface: rclcpp humble has no clock-less `Node::create_timer` member (`rclcpp.json` lists `Node::create_wall_timer` and the free `rclcpp::create_timer` only); it arrived in Iron. Nothing to port until the surface bumps; noted in W6 so it is not re-invented ahead of the surface. |
| 15 | Property 3 — a ROS-time timer with no `/clock` must NOT fall back to wall time (430 W5) | NO LONGER APPLIES | 425 chose the opposite, deliberately and with rclcpp: a `Ros` timer with no override reads system time (`executor/arena.rs:232`, `node.hpp:781`, `nros-core/src/clock.rs` `RosTime` arm). "A node written for simulation still runs standalone" is upstream's contract, and a timer that refuses to fire would be a compile-and-differ of its own. The behaviour was decided, not skipped. |
| 16 | …and the image SAYS so once, so "paused" is distinguishable from "misconfigured" (430 W5) | NOT STARTED | Nothing reports `use_sim_time` true with no sample ever received; `Clock::started()` (`clock.hpp`) is a poll the app has to think to make. → **W1**. |
| 17 | Feature-gated, off by default on freestanding targets (430 W6) | DONE | `sim-time` feature on `nros-node` and forwarded by the `nros` umbrella (`api/nros/Cargo.toml:199`), additionally gated on `has_rmw`. |
| 18 | Byte-identical timer scheduling without the feature (430 W6 acceptance) | PARTIAL | `TimerEntry` carries `clock_source` + `last_clock_ns` unconditionally (`executor/arena.rs:282`, ~16 B per timer entry) and `timer_try_process` matches on the source every poll; the `Steady` arm is the pre-425 code path. Measured cost is below the noise of one arena slot; recorded here as the decision NOT to fork the struct on a cfg (→ **W8**, closed by this note). The cost line lives in `time_source.rs`'s module doc and the book page, not beside `ZPICO_MAX_QUERYABLES`. |
| 19 | Runtime clock-source enum, not a type parameter (RFC-0089 amendment) | DONE | `TimerClockSource { Steady, Ros, System }` (`executor/arena.rs:227`), `#[repr(u8)]`. |
| 20 | A clock field on the flat `Timer` (RFC-0089 amendment) | DONE, better than asked | The field lives in the ARENA entry, not the C++ handle: `nros::Timer` is still `{executor_, handle_id_, initialized_}`, so `check-cpp-capability-layout` was never touched. |
| 21 | No `TimerBase`, no hierarchy (RFC-0089 §"Timer, studied…" decision) | DONE by 425; contradicted elsewhere | 425 added no type. But phase-417 W1.a had already added a hosted `rclcpp::TimerBase` with a virtual destructor and `detail::WallTimer : TimerBase` (`nros-cpp/include/nros/timer.hpp:232`) — see finding A. |
| 22 | `Clock::now()` under sim time, all three languages | DONE | Rust `Clock::ros_time().now()` reads the override (`nros-core/src/clock.rs`); C `nros_clock_get_now` → `get_ros_time_ns` (`nros-c/src/clock.rs:116`); C++ `node.get_clock()->now()` (a `Node`'s clock is `NROS_CLOCK_ROS_TIME`, `node.hpp:218`). |
| 23 | `sleep_for` / `sleep_until` / `wait_until_started` on a ROS clock | NO LONGER APPLIES | Declined by RFC-0021 (a blocking helper that does not drive the executor): ledger rows `cpp:Clock::sleep_for`, `cpp:Clock::sleep_until`, `cpp:Clock::wait_until_started`; book "Limits". `rclcpp::Rate` here is wall (`nros_cpp_time_ns()`, `nros.hpp:1228`) and so is humble's (`GenericRate<system_clock>`); a ROS-time `Rate` is a Jazzy+ surface. |
| 24 | rosbag behaviours: rate ≠ 1, loop (time backwards), pause | DONE | Rows 2, 6, 8. One interaction the fixture found and both sides now document: each `/clock` sample is a jump of `step × rate`, so under `Skip` the TICK rate is 1x while `now()` runs at `rate` unless `step × rate ≤ period` (`sim_time_clock_e2e.rs` `STEP_MS`). |
| 25 | The red this doc cited on 2026-09-06 (`use_sim_time_attaches_and_detaches_the_clock_source` failing at `is_active()` on pristine `main`) | DONE (fixed) | `7195c9e00` (issue 1104): the two tests that share the process-global `/clock` gate take a lock and restore what they found (`SimTimeGuard`). Green, see below. |

### The tests, run (2026-09-07)

`just check node-std-tests`' first line, on this branch:

```
cargo test -p nros-node --lib --features std,sim-time,param-services --quiet
test result: ok. 366 passed; 0 failed; 2 ignored; 0 measured; 0 filtered out; finished in 1.46s
```

and the five sim-time tests by name:

```
test time_source::tests::a_pre_epoch_sample_installs_nothing ... ok
test time_source::tests::a_clock_sample_becomes_nanoseconds ... ok
test executor::tests::a_non_bool_use_sim_time_attaches_nothing ... ok
test executor::tests::ros_time_timer_follows_the_simulated_clock ... ok
test executor::tests::use_sim_time_attaches_and_detaches_the_clock_source ... ok
test result: ok. 5 passed; 0 failed; 0 ignored; 0 measured; 363 filtered out
```

**The lane did not compile on this branch before this PR.** Phase-428 W6 made
`ParameterServer::declare` and `Executor::declare_parameter` `#[must_use]`
and swept no test caller: eleven `unused return value … that must be used`
errors (eight in `parameter_services.rs`'s tests, three in the sim-time tests)
took `check node-std-tests` red on `phase-428-w10-qos-ssot`, invisible to the
PR gate because that lane is not in it. Fixed in this PR's first commit, as
`assert!`s — a declaration a test depends on is a precondition. The fixture
test (`sim_time_clock_e2e.rs`, cyclone, two processes) was not run here.

### Reverse check — what the tree holds that RFC-0089's design did not ask for

* **A. `rclcpp::TimerBase` exists, with a vtable.** Not 425's: phase-417 W1.a
  added `class TimerBase { virtual ~TimerBase(); }` and
  `detail::WallTimer : TimerBase` under `NROS_CPP_HAS_SHARED_PTR`
  (`timer.hpp:232`), so that `rclcpp::TimerBase::SharedPtr timer_;` names
  something in a ported file. RFC-0089 §"Timer, studied against RTOS
  semantics" decided "No `TimerBase`, no `WallTimer`, no `GenericTimer`" and
  the amendment repeats "no `TimerBase`". The RFC and the header disagree, and
  the vtable is exactly the kind RFC-0089 clause 1 refuses: no dispatch uses
  it (`WallTimer::trampoline` is a static function; the only virtual is the
  destructor). It is hosted-only and costs a freestanding image nothing, which
  is the argument for keeping it. **Whichever way this settles, the RFC must
  say it** (→ W7), and W6 builds on the ruling: a hosted `create_timer(clock,
  …)` returns the SAME cell type as `create_wall_timer` with the clock in the
  arena entry, never a `GenericTimer` sibling.
* **B. A third clock source.** `TimerClockSource::System` is not in the
  two-clock design. It is the honest mapping of `NROS_CLOCK_SYSTEM_TIME`, which
  `create_timer(clock, …)` would otherwise have to reject at runtime, and it
  is what rclrs's `TimerClock::SystemTime` names. Accepted; ledger row
  `rust:TimerClock` records it.
* **C. A second spelling for attaching the source.**
  `NodeCtx::install_ros_time_source()` / `install_ros_time_source_on(topic)`
  exist beside `use_sim_time`. rclcpp has no such verb — a remapped `/clock` is
  `-r /clock:=…`. The explicit form is what a `MockSession` test and an image
  without `param-services` need, so it is a warranted second path under
  RFC-0089 §"When a second path is warranted", but it is ours-only and has no
  ledger row because the parity extractor builds the Rust surface without
  `sim-time` (425 W3 said so). → W7.
* **D. The switch is a process-global with a default of TRUE** plus three
  per-executor fields (`sim_time_source`, `sim_time_requested`,
  `sim_time_stated`, `executor/spin.rs:1401`). The default-true is what lets an
  explicit install need no second call to arm it; it is also what made two
  tests race (issue 1104). No hidden thread anywhere — the source is a
  subscription callback, nothing else (checked: `time_source.rs` and the
  `sim-time` paths of `spin.rs` spawn nothing).
* **E. The reserved-parameter hook fires before the store's verdict.**
  `note_reserved_parameter` (`executor/spin.rs:7971`) runs unconditionally
  ahead of `params.server.declare(...)`, so a re-declaration the store REFUSES
  (`declare` → `false`, which phase-428 W6 made a refusal) still flips the
  switch: the store keeps `use_sim_time=true` while the source detaches. Found
  while making the lane compile; `use_sim_time_attaches_and_detaches_the_clock_source`
  now pins the current behaviour with a comment naming this item. → W3.

**Conclusion.** The capability is 425's and it is complete. What remains is
REACH (three surfaces cannot ask for a ROS-time timer), one loudness item, and
two parameter-semantics corrections. That is a real delta, small, and not
worth closing 430 into 425's archive: the items below are it.

## Work items — the delta only

Every item that creates a timer ends at **one dispatch**:
`Executor::register_timer_on_clock(period, TimerClockSource, cb)`
(`executor/spin.rs:5124`). The C++ verb already does (`nros-cpp/src/timer.rs:145`);
the Rust, C and `ComponentNode` verbs below must reach the same function
through the same `TimerClockSource`, flat, with the clock in the arena entry —
**no `TimerBase` derivative, no `GenericTimer`, no second cell type.** A second
spelling of the enum or a per-language clock-to-source mapping is the drift
0135/0160 measured one layer down.

* **W1 [loudness] — `use_sim_time` true and no `/clock` ever: say so once.**
  After the source is installed and a wall interval (a named const, on the
  order of seconds) passes with `!Clock::is_ros_time_override_active()`, emit
  ONE `nros_log` warning naming the parameter, the topic and the fact that
  ROS-time timers are running on system time meanwhile. Not a behaviour
  change: row 15 stands. Reset the one-shot if the source is later detached
  and re-attached.
  *Acceptance:* a `MockSession` unit test declares `use_sim_time` true, spins
  past the interval, and observes exactly one diagnostic (through an executor
  accessor the test can read, so the assertion is not a log grep); a second
  spin past the interval adds none; a sample arriving before the interval
  produces none. The book's "Limits" section names the diagnostic.

* **W2 [params] — declare `use_sim_time = false` on every node, as rclcpp does.**
  When `sim-time` and `param-services` are both on, `ensure_parameter_store`
  (`executor/spin.rs:7875`) declares `use_sim_time` `Bool(false)` if the app has
  not, so `ros2 param list` shows it and `ros2 param set <node> use_sim_time
  true` reaches `refresh_use_sim_time_from_store` on a node with no sim-time
  code at all — the acceptance 430 W2 wrote and row 5 found half-met. An app's
  own declaration (launch bake, YAML, code) wins because it happens first or
  replaces the default through the store, never through a second hook.
  *Acceptance:* `parameters_roundtrip` (or a unit test on `MockSession`) lists
  `use_sim_time=false` for a node that never named it; setting it true through
  the store attaches the source on the next spin; an image without
  `param-services` is unchanged.

* **W3 [correctness] — the reserved-parameter hook follows the store's verdict.**
  Move `note_reserved_parameter` after `params.server.declare(...)` and call it
  only on `true`; a refused declaration leaves the switch alone (a runtime
  change still arrives through `refresh_use_sim_time_from_store`). Finding E.
  *Acceptance:* re-declaring `use_sim_time` false on a node that declared it
  true is refused AND the source stays active; the comment this PR left in
  `use_sim_time_attaches_and_detaches_the_clock_source` is replaced by the
  test flipping the switch through the store.

* **W4 [api, Rust] — the clock axis on the node-level surfaces.**
  `NodeCtx::create_timer_on_clock(period, TimerClockSource, cb)` (and the
  `_in` group form if `create_timer_in` keeps one), and on the declarative
  `nros::Node` a `create_timer_on_clock` whose `EntityMetadata` carries the
  source so `node_runtime.rs:1459` registers through `register_timer_on_clock`
  instead of `register_timer`. `TimerClockSource` stays the one enum; no
  `TimerOptions` builder (ledger `rust:IntoTimerOptions` is declined and stays
  so).
  *Acceptance:* a `nros::main!` component declares a ROS-time timer and
  `sim-clock-listener` can be rewritten against `NodeCtx` with no executor
  access; `check-feature-contract` green (the capability reaches the umbrella);
  `just check api-parity` rows `rust:NodeCtx::create_timer_on_clock` /
  `rust:Node::create_timer_on_clock` filed as extension, `rust:TimerClock`
  re-read.

* **W5 [api, C] — `nros_timer_init` takes rcl's clock.** rcl's shape is
  `rcl_timer_init(timer, clock, context, period, callback, allocator)`; ours
  gains the `nros_clock_t*` in rcl's position and `rclc_executor_add_timer`
  (`nros-c/src/executor.rs:1879`) maps the clock's type to `TimerClockSource`
  and calls `register_timer_on_clock` — the same mapping
  `nros_cpp_timer_create_on_clock` performs, so factor it into ONE
  `TimerClockSource::from_clock_type(u8)` both FFI crates call. `nros_timer_t`
  grows one field: re-run the opaque-size mirror (`opaque_sizes.rs`,
  `check-ffi-struct-mirrors`).
  *Acceptance:* a C timer on a `NROS_CLOCK_ROS_TIME` clock stops when `/clock`
  stops and a `NROS_CLOCK_STEADY_TIME` one does not (a C unit test over the
  mock, mirroring `ros_time_timer_follows_the_simulated_clock`); `check-c`
  green; ledger row `c:timer_init` written (there is none today).

* **W6 [api, C++] — `ComponentNode` and the hosted `rclcpp::` spelling.**
  `ComponentNode::create_timer(clock, period_ms, cb)` beside its
  `create_wall_timer` (both overload shapes, plus `NROS_CREATE_TIMER` if the
  macro family is kept), and under `NROS_CPP_STD` the free
  `rclcpp::create_timer(node, clock, period, cb[, group])` — humble's only form
  (row 13) — returning the same `std::shared_ptr<TimerBase>` cell
  `rclcpp::Node::create_wall_timer` returns (`nros.hpp:812`), with the clock in
  the arena entry. Do NOT add a clock-less `Node::create_timer(period, cb)`
  until the captured surface is Iron or later (row 14).
  *Acceptance:* a ported file `rclcpp::create_timer(node, node->get_clock(),
  100ms, cb)` compiles unchanged under `NROS_CPP_STD` and follows the bag
  (extend the sim-time fixture pair with a C++ listener, or a hosted unit
  test over the mock); `create_wall_timer` in the same file is unaffected by
  `/clock`; `check-cpp` and `check-api-parity` green; rows `cpp:create_timer`
  and `cpp:ComponentNode::create_timer` re-verdicted.

* **W7 [ledger, RFC, docs] — record what is true.** RFC-0089's amendment
  ("phase-430 brings ROS time") becomes "phase-425 brought it; phase-430 is the
  reach delta", and the RFC settles finding A in one sentence: either the
  hosted `TimerBase` is the sanctioned exception (a name for `SharedPtr` to
  hang on, vtable = one destructor, freestanding pays nothing) or it goes.
  Ledger: rows for `install_ros_time_source*` once the extractor builds the
  Rust surface with `sim-time` (finding C), plus the rows W4–W6 name. Book
  page: the W1 diagnostic and the W2 default.
  *Acceptance:* `just check doc-refs`, `just check api-parity` green; no
  document in the tree still says ROS time is unstarted or that the delta is
  unmeasured.

* **W8 [cost] — closed by row 18.** The per-timer `clock_source` +
  `last_clock_ns` stay unconditional; a cfg fork of `TimerEntry` for ~16 bytes
  and one predictable branch would cost more in mirror drift than it saves.

## Sequencing

W3 first (smallest, a correctness fix with a pinned test waiting for it), then
W2 (it makes W1 observable on a node with no sim-time code), then W1. W4, W5
and W6 are independent of each other and of the first three; each is one
surface and each ends at the one dispatch. W7 last, because it records what
landed.

## Amends

RFC-0089 §"Timer, studied against RTOS semantics", amendment of 2026-09-05:
the sentence "phase-430 brings ROS time" is superseded by this measurement —
phase-425 brought it; the conclusion ("still flat, a runtime field and a
second verb, no hierarchy") held in the code 425 landed and is what W4–W6 must
preserve. Finding A is the one place the tree and the RFC disagree, and W7
owns the ruling.
