# Phase 426 — parameters get a Rust SSoT, and `ros2 param list` works

**Status (2026-09-05). Planned.** Implements RFC-0089 §"Parameters:
feature-complete, Rust-side SSoT" and RFC-0019/0020's rule that behaviour lives
in Rust and the C/C++ APIs are thin wrappers. Closes the C++ half of issue 0793.

## Why

Three stores exist for one concept.

| store | where | who can see it |
| --- | --- | --- |
| `nros_params::ParameterTable` | Rust, on the **executor** | the six ROS 2 parameter services |
| `ParameterServer<NROS_RCLCPP_MAX_PARAMS>` | C++, inline on `rclcpp::Node` | that node's own getters, nothing else |
| a second facade | C++, on `ComponentNode` (`nros.hpp:394-405` flags the duplication against itself) | that object's own getters |

A parameter declared through either C++ facade is invisible to `ros2 param
get`, because the services read the Rust table. That is not a missing feature —
it is a second implementation of one, which is exactly what RFC-0019/0020
forbids.

**The service list is already complete**, and this is worth stating so the work
is not mis-scoped: `register_parameter_services` (`executor/spin.rs:7182`)
registers `GetParameters`, `SetParameters`, `SetParametersAtomically`,
`ListParameters`, `DescribeParameters` and `GetParameterTypes` — the full
upstream set.

**The defect is KEYING, not coverage.** Those six are registered under ONE FQN,
built from the *executor's* `node_name` and `namespace`, and the table is
executor-global. But an image composes several nodes onto one executor — that is
the whole model (RFC-0089 §"The rclcpp node model"). So:

* `ros2 param list` enumerates the executor's node, not the image's nodes;
* two nodes declaring the same parameter name collide in one flat table;
* nothing distinguishes `/talker`'s `rate` from `/listener`'s `rate`.

Upstream's model is one table and one set of six services **per node**.

## Work items

Ordered, and the order is load-bearing: **W4 must be last**, because deleting a
store before its replacement exists is how a capability disappears quietly.

* **W1 [rust] — key the table by node.** `ParameterTable` gains a node
  identity per entry; declare/get/set take it. The executor keeps one table
  (fixed arena, no per-node allocation), so this is a key widening, not a
  container per node.
  *Acceptance:* two nodes on one executor declare the same name with different
  values and both read back correctly; a unit test asserts the pre-change
  behaviour would have collided.

* **W2 [rust] — the missing writer.** There is no `Executor::set_parameter`
  and no `nros_cpp_set_param` (`grep -c` is 0). `SetParameters` and
  `SetParametersAtomically` are registered services with no store-side writer
  reachable from the wrapper path.
  *Acceptance:* `ros2 param set` changes a value an in-image node then reads;
  the atomic variant either applies all or none, asserted by a test that makes
  the second parameter fail.

* **W3 [rust+rmw] — six services PER NODE, and the sizing decision that comes
  with it.** Register the set under each node's FQN so `ros2 param list`
  enumerates the image's nodes.
  **This is not free and must not be discovered at runtime:** a service server
  IS a zenoh queryable. CLAUDE.md already records `[param_services]` claiming
  six slots against `ZPICO_MAX_QUERYABLES` (8 embedded by default); per-node
  registration makes that six times the node count. W3 therefore includes
  choosing the default, making the ceiling a build-time error rather than a
  runtime `-80`, and documenting the knob.
  *Acceptance:* a two-node image exposes both FQNs to `ros2 param list`; a
  three-node image on the default `ZPICO_MAX_QUERYABLES` fails the BUILD with a
  message naming the knob, not the boot.

* **W4 [cpp] — delete both C++ stores.** `rclcpp::Node`'s inline
  `ParameterServer` member and `ComponentNode`'s facade become forwarders to
  the FFI, then the members go. Only after W1–W3.
  *Acceptance:* a parameter declared through `rclcpp::Node::declare_parameter`
  is visible to `ros2 param get`; `check-cpp-capability-layout` still passes
  (the members leaving must not reintroduce a probe-dependent layout);
  `NROS_RCLCPP_MAX_PARAMS` is gone or documented as an arena bound, not a
  second store.

* **W5 [c] — the C surface follows.** `nros_parameter_*` keeps its shape and
  points at the same table, so C and C++ cannot disagree about what a node's
  parameters are.
  *Acceptance:* a mixed C/C++ workspace fixture declares from one language and
  reads from the other.

* **W6 [test] — the cell that would have caught this.** An interop cell that
  runs `ros2 param list` / `get` / `set` against a running nano-ros image and
  asserts the node FQNs. There is no such cell today, which is why three stores
  could coexist without a red.
  *Acceptance:* the cell fails on a tree with W1 reverted.

## Not in scope

* Parameter **callbacks** (`add_on_set_parameters_callback`) — a separate
  ledger question, and adding a callback path before the store is single would
  be a third implementation.
* Parameter **overrides from launch/`NodeOptions`** — RFC-0060 territory.
* Making `nros::ComponentNode` disappear; that is the node-type merge
  (RFC-0089 decision 2), which W4 touches but does not depend on.

## Risk

W3 is the one that can go wrong quietly. Six queryables per node against a
default of 8 means a two-node image is already over on embedded, and the
failure mode today is a runtime `-80` from zenoh-pico, not a build error. The
acceptance criterion is written to force the build-time answer, because a
runtime one is how issue 0460 read to the person who hit it.
