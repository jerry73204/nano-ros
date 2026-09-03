# Phase 417 — ROS 2 user-API adoption

**Status (2026-09-04). Planning. Implements RFC-0087.** No work item has
started. Stage 0 is a prerequisite for measuring any of the others, so it is
also the first thing to land.

Goal: a ROS 2 node written against rclcpp / rclc / rclrs compiles and behaves
against nano-ros, or fails loudly. End state is that our API carries ROS 2's
names as first-class aliases and `rclcpp_compat.hpp` is gone.

Two ordering principles, both from RFC-0087.

**The rename is cheap and cosmetic; the compatibility is the work.** The rename
lands last, after the shapes match, because before then it would relabel the
gaps without closing one — and would spend the property that makes mismatches
visible while the mismatches are still there.

**The Rust API is the implementation source of truth** (RFC-0019/0020). Every
item below carries *which layer implements this?*, and the order within an item
is Rust → FFI slot → C → C++. Ergonomics — aliases, forwarders, diagnostics,
conversions — may live in the wrapper; behaviour may not. Items marked
**[wrapper]** are pure delegation over behaviour Rust already has and is the
cheapest work here; items marked **[rust-first]** need the Rust side to grow
before either wrapper can expose anything, and planning them as C/C++ tasks
would mis-cost them by an order of magnitude.

## Stage 0 — measure the thing we are changing (issue 1020)

Blocks everything. Today the C++ lane reads `nros.hpp`, `component_node.hpp`
and `nros.hpp -DNROS_CPP_STD`, and filters to namespace `nros`, so the 589-line
compat shim contributes zero rows. Every "how far are we" number is currently
about the native API rather than about what a ported file reaches.

* W0.a — admit `rclcpp_compat.hpp` as a fourth TU with namespace `rclcpp`, and
  decide the three questions issue 1020 records: does a `rclcpp::` alias
  resolving to a `nros::` type correlate as `same`; are "how close is the
  native API" and "what does a ported file hit" one surface or two; how is the
  shim's std-only reachability marked.
* W0.b — add a `disposition` field to the ledger (adopt / adopt-bounded /
  refuse-loud / absent) per RFC-0087's consequence section, and a gate that a
  `declined` row declares one.

**Acceptance:** the C++ report distinguishes native-API distance from
ported-file distance, and every `declined` row says what a porting user gets.

## Stage 1 — the cheap unblockers

Small, independent, and they unblock the first lines of nearly every ported
file. Roughly 75 lines total.

* W1.a **[wrapper]** — nested `SharedPtr` / `ConstSharedPtr` / `UniquePtr` on `Publisher`,
  `Subscription`, `Service`, `Client`, `Timer`, and on `rclcpp::TimerBase`.
  `detail::SharedPtrTrait` (`rclcpp_compat.hpp:96`) was written for this and is
  dead code — one occurrence in all of `packages/api/`, its own definition.
  Unblocks `rclcpp::Publisher<T>::SharedPtr member_;`, which is close to
  universal in rclcpp source.
* W1.b — **ALREADY DONE; verify and close.** `write_b8_alias_header`
  (`packages/cli/cargo-nano-ros/src/lib.rs:1858`) is called from three sites
  (`:1706,1748,1790`), and `rosidl-bindgen` has the same. This item was written
  from the ledger, which measures the native API and cannot see generated
  headers. Confirm on a fresh `nros sync` and strike it.
* W1.c **[wrapper]** — `NROS_CPP_STD`-gated `std::string` interop on `FixedString<N>`
  (`operator=`, conversion, `operator==`), plus ungated `size()`/`empty()`.
* W1.d **[wrapper]** — forward `now()`, `get_clock()`, `get_name()`, `get_namespace()` on the
  shim `Node`; alias `rclcpp::Time`/`Duration`/`Clock`. All four already exist
  on `nros::Node` (`node.hpp:217,223,249,260`).

W1.a and W1.c also gate stages 6's lifecycle and action work, which hand
entities back and therefore need the nested pointer types first.

**Acceptance:** `examples/templates/cpp-port-minimal-publisher/` compiles with
its source byte-identical to upstream's tutorial. Today its README claims
"verbatim" and three lines differ — W1.a and W1.c are exactly those three.
That claim becomes true or the stage is not done.

## Stage 2 — the node surface

Where a real node stops being a tutorial. Each item is independently useful.

* W2.a **[wrapper]** — **one parameter store.** Rust already has the single
  store; this is thin-wrapper COMPLIANCE work, not new capability. Today there are three arrangements: C has two
  disjoint stores (issue 0793), C++'s `ComponentNode` owns a private
  `ParameterServer` and reads the executor store exactly once at boot under
  `#if defined(NROS_SYSTEM_PARAM_SERVICES)` (`component_node.hpp:536`), and
  Rust has one. Converge on the executor's, with node facades as views.
  Unfiled twin of 0793 on the C++ side; file it as part of this item.
* W2.b **[wrapper]** — rclcpp-shaped `declare_parameter<T>` / `get_parameter<T>` /
  `set_parameter<T>` / `has_parameter` on the node reachable from the umbrella.
  The implementation exists at `component_node.hpp:568-659`; the blocker is
  that `nros.hpp` does not include `component_node.hpp` (15 of 46 headers are
  unreachable from the umbrella).
* W2.c **[wrapper]** — `create_service` / `create_client` on the node; `async_send_request`
  returning something the existing `spin_until_future_complete` accepts.
  `nros::Client<S>` and `nros::Future` already exist.
* W2.d **[wrapper, carefully]** — `rclcpp::Rate` / `WallRate` as a FORWARDER
  onto `nros::spin(remaining_ms, poll_ms)` (`nros.hpp:175`), which drives the
  executor. The obvious fifteen-line C++ class with its own sleep loop is both
  RFC-0020 violation class 2 (a polling loop that spins the executor from
  inside the wrapper) and the thing RFC-0021 forbids. Same capability, and only
  one of the two shapes is admissible.

**Acceptance:** a node that declares a parameter, calls a service and reads the
clock compiles unmodified. Measured by a new ported template, not by inspection.

## Stage 2b — graph forwarders **[wrapper]**, 18 rows

`nros::Executor` already ships the whole graph surface — `get_node_names`,
`count_publishers`/`count_subscribers`, the four `*_names_and_types_by_node`
forms, `get_{publishers,subscriptions}_info_by_topic` (`executor.hpp:205-301`)
over FFI that exists. The rows are open because `Node`/`LifecycleNode` do not
forward to it, not because the capability is missing. Pure delegation, no new
behaviour, and it needs a `TopicEndpointInfo` value type.

## Stage 3 — the loudness pass (the safety gate)

RFC-0087's rule applied to everything already adopted. **This stage gates the
rename**: until it is done, taking more upstream names increases the number of
ways a ported program can compile and differ.

* W3.a — issue 1019: `RCLCPP_*_STREAM` discards its message; the family routes
  to a sink that is a no-op on every embedded target. Fix or refuse loudly.
* W3.b **[rust-first]** — `rclcpp::init(argc, argv)` drops `--ros-args`
  silently (`rclcpp_compat.hpp:238`), turning a remap into a wrong-topic bug at
  runtime. Honour them on posix boards, or refuse the two-argument form.
  Honouring them is remap resolution — name construction, RFC-0020 violation
  class 4 — so the parser belongs beside `nros::resolve_name`, not in the shim.
* W3.c — write the REFUSE-LOUD diagnostics. **69 rows collapse into 17
  messages** — a refusal is per-concept, not per-symbol; the sixteen inert
  `NodeOptions` setters share one. ~120 lines for the whole pass.
* W3.f — **the two live inversions the shim already ships**, which a rename
  would inherit: `ParametersQoS()` returns `QoS(10)` where upstream is
  `KEEP_LAST, 1000` (`rclcpp_compat.hpp:117`), and ten `NodeOptions` setters
  store their argument and are never read (`:125`, "intentionally inert
  today"). Adopting the named QoS profiles at all is ADOPT **only** with each
  profile's values transcribed from upstream and a table-driven test against
  `rmw_qos_profile_*` — the shim already got two wrong, which is the evidence
  for why the test is not optional.
* W3.d — document the ADOPT-BOUNDED envelopes in the doc comments, starting
  with `create_wall_timer` (period accuracy is the spin cadence, because the
  timer is polled from `Node::pump()`, not driven by the executor).
* W3.e — a gate: no adopted upstream name may lack a disposition, and no
  `adopt` row may carry a known contract inversion.

**Acceptance:** every upstream name we define either behaves or fails to
compile. Demonstrated by an expected-failure probe per REFUSE-LOUD item, in the
shape `check c` / `check cpp` already use.

## Stage 4 — make our own three languages agree

A per-language drop-in claim is undermined while our own surfaces disagree
about the same capability. 37 such disagreements are catalogued; C++ is the odd
one out in 15.

* W4.a **[wrapper]** — parameters: setter, type query, undeclare, descriptors (ranges,
  read-only) in all three.
* W4.b **[mixed]** — actions: a goal-id TYPE in C++ (`uint8_t[16]` today, so it cannot be
  stored or compared); `succeed`/`abort`/`canceled` verbs; Rust's `cancel`
  renamed to `canceled` with a deprecated forwarder.
* W4.c **[wrapper]** — executor: `cancel`/`is_spinning` in all three (a C++ node cannot stop
  spinning today without tearing down the session).
* W4.d **[wrapper]** — logging: named loggers, per-logger levels, throttle, sinks. C and C++
  have none of it; Rust has it but the façade does not re-export it, which is
  why `std::println!` is the path of least resistance from the façade — fatal
  on Zephyr native_sim (issue 0589).
* W4.e **[rust-first]** — guard conditions: one owner and one creation shape. The ledger already
  records this as undecided ("Nothing about no_std picks between these").

## Stage 5 — C

* W5.a **[rust-first]** — **typed subscription delivery.** rclc delivers a deserialised message
  on a path with no allocator, by having the caller own the storage:
  `rclc_executor_add_subscription(executor, subscription, void *msg, callback,
  invocation)` with `void (*)(const void *)`. Ours has no `msg` slot and
  delivers `(const uint8_t *, size_t)`. The generated `<Msg>_deserialize`
  already writes into caller storage and the typed publish half already
  shipped. Every ported callback body currently has to be rewritten, and the
  ledger blames an allocator that rclc does not have either. The FFI needs an
  `add_subscription` variant carrying caller-owned storage before C can expose
  anything, so this is Rust work with a C surface, not C work.
* W5.b **[wrapper]** — `<nros/rcl_compat.h>` mapping `RCL_RET_*` onto ours. `nros_ret_t`'s
  own doc says "Compatible with `rcl_ret_t` for familiarity"
  (`nros_generated.h:840`) and only `OK` agrees: ours are −1/−2/−3/−7, rcl's
  are 1/2/11/101. Map, do not renumber.
* W5.c **[wrapper]** — the node and timer accessors filed as `gap`, all thin forwarders over
  state the executor already holds.
* W5.d **[wrapper]** — rclc-shaped preset constructors as `static inline` forwarders; rodata
  only, retires ~18 declined rows.
* W5.e **[rust-first]** — a typed service/client path; `service.h.jinja` generates only
  `_get_type_name`/`_get_type_hash` today, so every C service in the tree is
  raw bytes with hand-written CDR.

## The structural blocker the rename cannot paper over

**Two spin worlds, and no compile signal.** The shim `rclcpp::Node` dispatches
subscription callbacks and wall timers from its own `pump()`, driven only by
`rclcpp::spin(node)` / `rclcpp::spin_some(node)`
(`rclcpp_compat.hpp:428-451,454-470`). A ported file that instead calls
`nros::spin_once()`, `nros::spin()`, or drives an `nros::Executor` directly —
all of which are in the umbrella, all of which a mixed file plausibly reaches —
gets **zero callbacks and no diagnostic**.

Today the two node types have different names, which is the only thing making
this visible at all. **After the rename they would have the same name**, so the
rename makes it strictly worse. No message can be written for it: both spellings
are legitimate, and which one is wrong depends on which node object the file
holds.

This is why the rename is gated on more than the loudness pass. The fix is
structural — one node stack, the shim `Node` re-parented onto the arena-driven
one rather than pumping its own — and it belongs before stage 6, not in it.

## Stage 6 — the rename

Only after stages 1–4. Mechanical once the shapes match.

* W6.a — declare the ROS 2 spellings as first-class aliases IN the API headers;
  delete `rclcpp_compat.hpp` as a separate file.
* W6.b — `#error` when upstream rclcpp's include guard is already defined, so
  the ODR collision RFC-0087 names is a compile error rather than a silent one.
* W6.c — migrate the in-tree call sites (110 C++ and 75 C example files) and
  the book.

**Acceptance:** the compat shim is gone, ported templates still build, and no
in-tree source names a spelling that upstream does not have — except where a
disposition says why.

## The cheapest work is wrapper work, and there is a lot of it

Of the 37 capabilities where our own three surfaces disagree, roughly half are
ones **Rust already has and one or both wrappers never exposed** — named
loggers and per-logger levels, parameter type queries, undeclare, descriptors
and ranges, QoS equality, GID attribution, name resolution. The behaviour
exists and is tested; the wrapper is a forwarder. Under RFC-0019 that is both
the cheapest work in this phase and the work that reduces the most divergence
per line, which is why stage 4 is not deferred behind the C-side stages.

## What this phase does NOT promise

Not the whole rclcpp surface. Four upstream idioms account for most of what we
decline and three are incompatible with ours by construction: the wait-set /
`Waitable` executor (~128 rows, needs an allocator), runtime-queryable graph and
middleware (~55, needs discovery), runtime type erasure (~45, needs a dynamic
loader). The claim this phase can honestly reach is about PROGRAMS — the shapes
a ROS 2 node is actually written in compile and behave, everything else fails
loudly — and the in-tree ported templates are the measurement.
