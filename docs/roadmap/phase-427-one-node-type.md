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

## What stays invented

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
