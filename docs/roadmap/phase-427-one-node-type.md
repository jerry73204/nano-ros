# Phase 427 — one node type, named `rclcpp::Node`, compiling freestanding

**Status (2026-09-05). Planned.** Implements RFC-0089 §"The node API, proposed
under the governing principle". Preconditions are met by phase-417: the
`pump()` blocker is gone, `check-cpp-capability-layout` measures the layout
rule, and the `-nostdinc++` lane can see a freestanding regression.

## What this is

Three C++ node shapes collapse to one type:

| today | after |
| --- | --- |
| `nros::Node` — out-ref creation, freestanding, 103 files | `rclcpp::Node` |
| `rclcpp::Node` — hosted, `shared_ptr`, derivable | the same type, hosted members |
| `nros::ComponentNode` — derivable, entry-supplied handle, 5 dirs | DELETED; its handle becomes a constructor |

`nros::Node` survives only as a deprecated alias so the remaining call sites are
optional to migrate rather than a flag day.

## Work items

* **W1 [cpp] — the out-of-line hosted block.** `void* hosted_` replaces every
  hosted-only member; `detail::NodeHosted` is allocated lazily on the first
  hosted-shape call and never on a freestanding target.
  *Acceptance:* `check-cpp-capability-layout` passes with the hosted members
  present, and `sizeof(rclcpp::Node)` is identical with and without
  `-DNROS_CPP_STD`.

* **W2 [cpp] — construction.** `Node(const char*)` + `init()` + `ok()`
  freestanding; upstream's `std::string`/`NodeOptions` constructors hosted.
  *Acceptance:* the `-nostdinc++` lane compiles a TU that constructs a node and
  creates a publisher; a hosted TU compiles upstream's constructor verbatim.

* **W3 [cpp] — one name, two signatures.** The out-ref `create_*` family and
  the `shared_ptr` family coexist as overloads on the one type;
  `create_wall_timer` gains the member-binding template overload, retiring
  `bind_timer`.
  *Acceptance:* a component binds a member function with no allocation; a
  ported file's `create_publisher<M>("chatter", 10)` compiles hosted and FAILS
  TO COMPILE freestanding, with a diagnostic naming the out-ref overload.

* **W4 [cpp] — `ComponentNode` deleted.** Its 5 directories move to the one
  type; RFC-0047's several-named-nodes survives as a documented ours-only
  capability on it. RFC-0044 is amended, not deleted — its Q2 boot-failure
  reasoning becomes `ok()`'s.
  *Acceptance:* zero `ComponentNode` in the tree; the RFC-0047 subnode packages
  build and run; **one of them builds for a freestanding target**, which is the
  test of whether the merged type still fits.

* **W5 [cpp] — `get_logger()` follows ROS 2.** The `"nros.compat"` sentinel is
  replaced by a logger named for the node (RFC-0089 decision 1).
  *Acceptance:* two nodes in one image emit records under distinct logger names.

* **W6 [loudness] — the two items the design creates.** `[[nodiscard]]` on
  `Result` (`NROS_NODISCARD` for C++14) so a discarded `rclcpp::init(argc,
  argv);` warns; and a story for a hand-written `main` that never checks
  `Node::ok()`.
  *Acceptance:* a TU that discards `init()`'s result fails a `-D warnings`
  lane. The `ok()` half may end as documentation — if so, say so in the book
  rather than leaving it implied.

* **W7 [migration] — `nros::Node` deprecated, then deleted.** Alias for one
  release with `NROS_DEPRECATED_MSG`, then removed.

## What stays invented — REVISED after review (2026-09-05)

The first version of this table had four entries. Reviewed against the recorded
upstream surface, **two were mislabelled** (RFC-0089 §"Review of the invented
parts"):

| item | verdict |
| --- | --- |
| `rclcpp::Timer` | **partly ported.** Upstream's `TimerBase` is a POLYMORPHIC base and adopting the hierarchy is refused by clause 1 — a vtable in every image that holds a timer. But the common ported line is a member declaration, `rclcpp::TimerBase::SharedPtr timer_;`, which is a NAME. So hosted gains `using TimerBase = Timer;` plus a `SharedPtr` typedef, and a file that tries to DERIVE fails to compile. `WallTimer`/`GenericTimer` stay absent — templates over a clock type we do not have. |
| `rclcpp::spin_once(timeout_ms)` | **invention, kept.** Upstream's verb is ALREADY ported — `spin_some(node)` exists here with upstream's drain-what-is-ready semantics as a 0-timeout spin. Ours is a blocking wait with a budget, which rclcpp has no verb for and an RTOS task needs. |
| `Node::init` / `Node::ok` | kept. `ok()` matches `rclcpp::ok()`'s own spelling; `init()` is the channel a `-fno-exceptions` target needs instead of a throwing constructor. |
| out-ref `create_*` | **not an invention.** It shares upstream's name with a changed signature, and the signature is forced: the arena stores `&entity` as its dispatch context and has NO unregister, so a value-returning form would move the object and dangle the context on the first callback. |

The `rclpy.spin_once` collision recorded earlier was **overstated**. rclpy's
signature is `(node, timeout_sec)`, so a user carrying that habit writes
`spin_once(node, 0.1)` in C++ and gets no matching overload — mechanical, not
silent. The ledger row records the sibling for the reader; it is not a reason to
rename.

## Superseded first-pass table

### (first pass, kept for the diff)

Tabulated in RFC-0089; repeated here because it is the part a reviewer should
argue with. Each needs a ledger row with `disposition: extension`, and each is
watched by the collision gate.

| name | why upstream has nothing | collision risk |
| --- | --- | --- |
| `rclcpp::Timer` | ours is a HANDLE; upstream's `TimerBase`/`WallTimer` are a virtual hierarchy, and we have no vtable budget | **real** — `Timer` is absent upstream today while both siblings are present |
| `rclcpp::spin_once(timeout_ms)` | upstream's one-cycle verb is `spin_some(node)` — no timeout, no return | **real** — `rclpy.spin_once` exists with `(node, timeout_sec)` |
| `Node::init` / `Node::ok` | upstream throws; a `-fno-exceptions` target needs a channel | low |
| the out-ref `create_*` family | caller-owned storage has no upstream counterpart | none — it shares upstream's NAME, so it is a ported signature, not an invention |

## Not in scope

* Parameters — phase-426. W4 touches the facades but does not depend on it.
* The C API's node shape — phase-428's sweep decides whether it follows.
* Migrating the 409 remaining `nros::` call sites; the alias makes that
  optional and incremental.


## W8 [error channel] — one rule, and `[[nodiscard]]`

The review surfaced that we ship two error channels with no stated rule.
`Result` (no value) and `Expected<T>` (value or error) both live in
`result.hpp`, which — measured — carries ZERO capability gates and parses under
the ThreadX `-nostdinc++` shim, so both reach every target.

The rule, now in RFC-0089: ours-only fallible operations return `Result`, or
`Expected<T>` when they produce a value; a state query that cannot fail returns
`bool`; **a PORTED api keeps upstream's channel even when that is `bool`** —
`rclcpp::Node::get_parameter` returns `bool` upstream, so ours does. Making that
uniform would be a preference recorded as a divergence, which RFC-0036 forbids.

*Acceptance:* `Result` and `Expected<T>` carry `[[nodiscard]]`
(`NROS_NODISCARD` on C++14); a TU that discards `rclcpp::init(argc, argv)`
fails a `-D warnings` lane. This subsumes W6's first item.
