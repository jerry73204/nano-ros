# Phase 417 — ROS 2 user-API adoption

**Status (2026-09-04). In flight. Implements RFC-0089.** Stages 0, 1 and 2b are
LANDED, and so is the correction track (issues 1012 and 1022 resolved, 92 rows).
Stage 1's acceptance is met: `cpp-port-minimal-publisher/src/minimal_publisher.cpp`
is upstream's file again. Stages 2 (node surface), 3 (loudness), 4 (cross-language)
and 5 (C) are open; stage 6 (the rename) is gated on stage 3 AND on the structural
blocker below.

**What stage 0 revealed, and it reframes the phase.** Admitting the compat shim as
a fourth TU made the ported surface look better — `same` 84 → 110 — while the
defects were unchanged. `ParametersQoS` and `NodeOptions::use_intra_process_comms`
now correlate `same`: the first returns `QoS(10)` where upstream is `KEEP_LAST,
1000`, the second stores its argument and is never read. Correlation compares names
and shapes and neither differs, so **the instrument cannot see the defect it is
measuring.** A row can be `same` by shape and `refuse-loud` by disposition at once,
which is why `disposition` is not bookkeeping — it is the only thing between a
compatibility number and a false one.

Goal: a ROS 2 node written against rclcpp / rclc / rclrs compiles and behaves
against nano-ros, or fails loudly. **End state is that our API carries ROS 2's
names** — `nros::` stops being the spelling a user writes — and
`rclcpp_compat.hpp` is gone because its content moved into the headers it was
shimming.

This phase is also the **home for every correction, migration and retirement
job** in the API campaign: the tracks are listed below, and each names its
issue. An issue with no work item is an issue nobody is accountable for.

Two ordering principles, both from RFC-0089.

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
  refuse-loud / absent) per RFC-0089's consequence section, and a gate that a
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

RFC-0089's rule applied to everything already adopted. **This stage gates the
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

## The structural blocker — RESOLVED by phase-417 itself (corrected 2026-09-05)

**This section described a blocker that no longer exists, and nothing scheduled
the node-type merge because the two documents gating it still described the
pre-fix world.** Recording what it said, and what removed it.

It said: the shim `rclcpp::Node` dispatched subscription callbacks and wall
timers from its own `pump()`, driven only by `rclcpp::spin(node)` /
`spin_some(node)` (citing `rclcpp_compat.hpp:428-451,454-470`). A ported file
that instead called `nros::spin_once()`, `nros::spin()`, or drove an
`nros::Executor` got **zero callbacks and no diagnostic** — and after a rename
the two node types would share a name, making it strictly worse.

**That file and that method are gone.** `rclcpp_compat.hpp` was deleted in stage
6 step A; `pump()` was deleted by the one-dispatch-path pass. `nros.hpp:939-948`
says so in its own words — *"`pump()` IS GONE… there is nothing left for a
node-local sweep to do and mixing spin spellings is harmless"* — and
`nros.hpp:622-631` records the change at the call site: every entity a
`rclcpp::Node` creates is now arena-registered through
`nros::create_subscription_raw`, the same call the native path makes, so the
executor dispatches it whichever spin verb the caller drives.

So the precondition the merge was waiting on has been satisfied. What remains is
not a blocker but two open questions, and they are different in kind:

* **The instrument.** `just check cpp`'s "freestanding syntax" probe was host
  `c++ -ffreestanding` against the host's COMPLETE libstdc++, so it could not
  fail on anything a merge risks. A `-nostdinc++` lane against the real
  ThreadX and Zephyr shims is now in `check-cpp`, and it found a live red on
  its first run (`::std::abort` unexported by the ThreadX `cstdlib`, breaking
  every ThreadX C++ image since `rclcpp::init` entered the umbrella header).
* **`sizeof(nros::Node)`.** A merged node whose layout depends on a
  preprocessor capability probe is an ODR hazard, and those probes have now
  been measured wrong three separate times. That is a risk decision, not a
  reading — see the RFC.

Two further claims this section rested on were also measured and are false.
`rclcpp::Node` is **not hosted-only**: it is declared on bare-metal NuttX
armv7a and riscv32, and the real partition is `-nostdinc++` versus not, which
only Zephyr and ThreadX-RISCV64 fall on. And five of W-B5's six C++ items —
`spin_once`, `Timer`, the lifecycle set, `GoalResponse`/`CancelResponse` — have
**zero upstream occurrences**, so they are ours-only names for capabilities ROS
2 does not have: retireable never, not later, and never contingent on this
merge. Only `nros::Node` ever was.

## Stage 6 — the rename, in two steps

Only after stages 1–5 and the structural blocker above. Mechanical once the
shapes match; the two steps exist so the irreversible half happens once.

**Step A — alias (reversible).**

* W6.a — declare the ROS 2 spellings as first-class names IN the API headers;
  delete `rclcpp_compat.hpp` as a separate file. Both spellings work.
* W6.b — `#error` when upstream rclcpp's include guard is already defined. Not
  because we expect a build to link both — we do not, they interoperate over
  the wire and the host tooling is a separate process — but because the guard
  is one line and the failure it prevents is silent.

**Step B — replace (irreversible for out-of-tree consumers).**

* W6.c — deprecate `nros::` / `nros_` spellings, per the settled policy:
  `NROS_DEPRECATED_MSG` static inline for C, `[[deprecated("…")]]` for C++,
  and nothing for Rust trait methods (a rename there breaks implementors, and
  a compile error is what a backend author wants).
* W6.d — migrate the in-tree call sites: **110 C++ and 75 C example files**,
  the six compat-built templates, and the book.
* W6.e — remove the deprecated spellings as ONE batch, with a changelog entry.
  Carry phase-379 W7 step 4's two warnings: **C cannot portably deprecate a
  `typedef`** (MSVC rejects the attribute, `[[deprecated]]` is C23), so renamed
  C types disappear silently for anyone who never rebuilt; and a defaulted
  trait method's alias is weaker than it looks, because an out-of-tree
  override is silently ignored rather than warned.

**Acceptance:** the compat shim is gone, ported templates still build, and no
in-tree source names a spelling upstream does not have — except where a
disposition says why.

## The cheapest work is wrapper work, and there is a lot of it

Of the 37 capabilities where our own three surfaces disagree, roughly half are
ones **Rust already has and one or both wrappers never exposed** — named
loggers and per-logger levels, parameter type queries, undeclare, descriptors
and ranges, QoS equality, GID attribution, name resolution. The behaviour
exists and is tested; the wrapper is a forwarder. Under RFC-0019 that is both
the cheapest work in this phase and the work that reduces the most divergence
per line, which is why stage 4 is not deferred behind the C-side stages.

## Stage 6 step B — surveyed 2026-09-04, its own PR

Step A landed: the ROS 2 spellings are first-class, `rclcpp_compat.hpp` is
deleted, and every `rcl_compat.h` FUNCTION alias became an identity and was
deleted with it. Step B retires the old spellings. **It is the only irreversible
step for an out-of-tree consumer**, so it is one batch, one PR, one changelog
entry — never opportunistic.

### The size, measured rather than estimated

| surface | sites | files | shape |
| --- | ---: | ---: | --- |
| Rust logging macros | 208 | 50 | pure rename |
| C symbols (43 deprecated names) | 206 | 40 | pure rename, **except the one below** |
| C++ `nros::` → `rclcpp::` | 645 | 119 | pure rename |
| docs / book | — | 57 | prose + code blocks |
| the forwarders themselves | 110 | 12 headers | deletion |

Largest concentrations: `examples/native/{c,rust}` (27 files), the five RTOS
example families (25), `packages/testing/nros-tests` (12).

### One site is not mechanical, and it is the whole risk

`nros_node_init` is the single argument REORDER in the campaign:

```
nros_node_init        (node, support, name, namespace_)     <- 42 call sites
rclc_node_init_default(node, name, namespace_, support)
```

A sweep that renames without reordering produces code that **compiles with a
warning and passes the wrong values** — C diagnoses an incompatible pointer
argument as a warning even under `-Wall -Wextra`, which is the measurement that
forced RFC-0089's "a C reorder ships with a rename beside it". Here the rename
IS the guard: the old identifier disappears, so a missed site fails to compile
rather than silently mis-binding.

Surveyed for tractability: **all 42 are single-line, zero multi-line**, and the
shape is uniform (`nros_node_init(&x, &y, "name", "/")`). So a regex codemod is
honest here — but it must be a codemod that reorders, not a `sed s/old/new/`,
and the two must not be run as separate passes.

### Work items

* **W-B1 [rust]** — flip `nros-log/deprecate-legacy-names`, then fix the 208
  sites it names across 50 files. The flip comes FIRST because
  `warnings = "deny"` makes it enumerate the work instead of a grep guessing at
  it.
* **W-B2 [c]** — the 43 deprecated C names across 206 sites / 40 files,
  **including the `nros_node_init` reorder in the same pass**. Rename and
  reorder must not be separate passes: the rename is what makes a missed
  reorder fail to compile.
* **W-B3 [cpp]** — **LANDED, and the survey was wrong.** Not 645 pure renames:
  measured 697 code occurrences over 120 files, of which only **288 (41 %) are
  pure renames**. 409 stay `nros::`, and the reason is the campaign's own rule
  rather than effort:
  * `nros::Node` — **109 sites, 103 files**. `rclcpp::Node` is a DISTINCT CLASS,
    not an alias: `make_shared` + shared_ptr-returning `create_publisher<M>`
    against ours' out-ref `create_node(node, …)`. It is also hosted-only, so it
    does not exist on the freestanding targets most of those files build for.
  * `nros::init` (13) — `rclcpp::init()` returns **void**; every call site
    consumes our `Result`. Renaming drops the error check.
  * `nros::Timer` (28), `nros::spin_once` (32) — no counterpart with that
    contract.
  * ~200 more with no `rclcpp::` counterpart at all: the `bind_*` family,
    `create_node`, `Seq`/`HeapString`, `GoalResponse`/`CancelResponse`/
    `GoalStatus`, the whole lifecycle set (there is no `rclcpp_lifecycle`
    namespace in our headers).

* **W-B5 [retire] — PRECONDITION, added after W-B3 measured it.** The C
  forwarders (43 names) and the five Rust logging forwarders CAN go: every one
  has a live replacement. The C++ `nros::` spellings for `Node`, `init`,
  `Timer`, `spin_once`, lifecycle and the action-response enums **cannot** —
  deleting them removes a capability rather than a spelling. Retiring those
  waits on the node-type merge that `nros.hpp`'s own comment defers to "a later
  step".
* **W-B4 [docs]** — 57 files in `book/` and `docs/` naming an old spelling in
  prose or a code block. No compiler checks these.

* **W-B6 [changelog]** — one entry, carrying phase-379 W7 step 4's two
  warnings.

### Order, and why

1. **Flip `nros-log/deprecate-legacy-names`.** The workspace sets
   `warnings = "deny"`, so arming the attribute turns all 208 Rust sites into
   hard errors at once. That is the point: it enumerates the work rather than
   trusting a grep.
2. **Migrate Rust, then C, then C++.** Each is a rename; the compiler names
   every site.
3. **The `node_init` codemod, alone, with its own review.** Not folded into the
   C rename sweep — it is the one change where a mistake is silent.
4. **Docs and book.** `just book` builds them; a stale code block is not caught
   by any compiler.
5. **Delete the forwarders** — 110 `NROS_DEPRECATED_MSG` declarations across 12
   headers, plus the five Rust forwarders, plus the feature flag itself.
6. **Changelog entry**, carrying phase-379 W7 step 4's two warnings: C cannot
   portably deprecate a `typedef`, so renamed C TYPES disappear with no warning
   for anyone who never rebuilt; and a defaulted trait method's alias is weaker
   than it looks, because an out-of-tree override is silently ignored.

### Acceptance

* every in-tree source builds with `deprecate-legacy-names` ON and zero
  forwarders present;
* `examples/templates/cpp-port-minimal-publisher` still byte-identical to
  upstream's tutorial;
* `rcl_compat.h` holds only the `RCL_RET_*` mapping and handle typedefs — the
  two things RFC-0089 says cannot dissolve;
* `just ci gate` green, and the ported templates build with **every** compat
  layer deleted.

## Correction track — the ledger says false things, and they misdirect

The measurement is the campaign's instrument; a wrong row is worse than a
missing one because it forecloses the fix. Three issues, ~140 rows, no code.

* **W-C1 — issue 1012** (15 rows): prose names a symbol a rename retired.
  Fifteen describe the CURRENT tree with a dead spelling; a further 21 name one
  legitimately (a deprecated-alias row must). The distinguishing property is
  tense, which is why a sweep on the spelling alone breaks the correct ones.
  The issue's durable half — whether to gate it — needs the tense made
  machine-readable first.
* **W-C2 — issue 1022** (~95 rows): prose states something FALSE about our own
  code. "There is no runtime options struct" where seven ship;
  `nros_borrowed_str_t` for `nros_view_str_t`; `get_actual_qos` "would return
  its own input" when `set_qos_overrides` makes it differ; `cpp:` rows
  answering in C spellings; ~14 rows verdicted `declined` whose own prose
  describes a capability we have under another name.
* **W-C3 — issue 1022's systematic half**: a divergence justified by a
  constraint the compared surface refutes. Byte-oriented delivery is blamed on
  "no allocator"; rclc has no allocator on that path either and delivers typed
  by making the caller own the storage. RFC-0036 forbids recording a preference
  as a divergence and has no rule against recording a real divergence under a
  false cause. Closing W-C3 means adding one.

### The `gap` rows are the queue, and they are counted (2026-09-06)

Phase-379 re-measured the ledger: **158 `gap` rows**, C++ 82, C 41, Rust 35.
By shard: `param` 32, `graph` 30, `pubsub` 22, `lifecycle` 15, `log` 13,
`timer` 11, `service` 10, `qos` 8, `action` 6, `node` 6, `exec` 2, one each in
`boot`, `types`, `other`. That is this phase's work queue for stages 2–5, and
the shard distribution is the order argument: parameters and graph are a third
of it between them.

**The precedent for how one closes.** `node_get_fully_qualified_name` shipped
for all three languages in PR #567. What made it worth more than the row: the
tree already had three hand-rolled copies of the join, and collapsing them onto
one implementation found that they disagreed — `Node::fully_qualified_name`
returned `my_ns/my_node` for a namespace with no leading slash, which is not
fully qualified. A `gap` row is a prompt to look, and the defect is often
underneath rather than in the missing name.

Two decisions from it that generalise. First, the C form takes a caller buffer
plus an `out_len` rather than rcl's `const char *`, because rcl can return a
pointer only by STORING the joined string and caching it here costs
`MAX_NAME_LEN + MAX_NAMESPACE_LEN` per node to hold what two fields already
hold. Second, it returns `NROS_RET_FULL` and not a `NROS_RET_`-prefixed
`BUFFER_TOO_SMALL` — that constant lives on the RMW ABI
(`NROS_RMW_RET_BUFFER_TOO_SMALL`) and `nros_ret_t` has never defined a twin.
The doc comment that claimed otherwise was issue 1126, now closed: the sweep it
prompted found the same defect in `rmw_vtable.h` and two more places, so the
class is gated (`just check ret-code-citations`).

## Migration track — what moves, and in what order

Inherited from phase-379 W7, which owns steps 1–3 and is in flight. This phase
owns what follows.

* **W-M1** — 379 W7 steps 1–3 (collapse landed rows, execute the W6 verb
  decisions, the remaining open rows). **Stays homed in 379**; listed here so
  the sequence is readable end to end.
* **W-M2** — the disposition pass: every upstream item gets adopt /
  adopt-bounded / refuse-loud / absent (stage 0 W0.b), because a `declined` row
  that does not say what a porting user GETS is not actionable.
* **W-M3** — stages 1–5 above, in dependency order.
* **W-M4** — stage 6 step A (alias), then step B (replace).

## Retirement track — irreversible, batched, once

* **W-R1 — phase-379 W7 step 4**, in flight there: the `nros_param_*` family
  (25 forwarders), the service reply verbs, the QoS `*_raw()`/`*_ms()`
  accessors, `BoardConfig::zenoh_locator` and `ThreadxConfig::zenoh_locator`,
  the five `with_zenoh_locator()` builders.
* **W-R2 — `rclcpp_compat.hpp` itself** (stage 6 step A). Not a deletion of
  capability: its content moves into the headers. What retires is the file and
  the force-include.
* **W-R3 — the `nros::` / `nros_` spellings** (stage 6 step B). The only step
  in this phase that breaks an out-of-tree consumer who did nothing wrong.

Retirement happens once per batch, deliberately, with a changelog entry — never
opportunistically as each rename lands. That rule is phase-379's and it carries
forward unchanged.

## Issues homed here

Every issue this phase owns, with what closing it means. A mention is not an
owner.

| issue | track | closing it means |
| --- | --- | --- |
| 1012 | correction | 15 rows re-worded; a decision on whether tense becomes machine-readable |
| 1019 | correction + loudness (W3.a) | `RCLCPP_*` reaches `nros_log` on embedded, or refuses; `_STREAM` stops discarding its message |
| 1020 | stage 0 | the C++ lane sees the compat shim; native-API distance and ported-file distance are distinguishable |
| 1022 | correction | ~95 rows corrected; RFC-0036 gains the rule that a divergence's cause must not be one the compared surface also operates under |
| 0793 | stage 2 (W2.a) | one parameter store in C; the unfiled C++ twin filed and fixed with it |
| 0829 | stage 5 | `SYSTEM_DEFAULT` stops disagreeing with itself; folds into the named-profile transcription |
| 0589 | stage 4 (W4.d) | the façade re-exports `nros_log`, so it is the easy path rather than `std::println!` |
| 1126 | correction — CLOSED | `nros_publisher_publish_streamed`'s doc (the raw entry point was never the site) stopped promising a `NROS_RET_`-prefixed `BUFFER_TOO_SMALL`; correct-the-doc won, no caller needs the two failures apart. The sweep found two more live sites and the class is now gated |

## What this phase does NOT promise

Not the whole rclcpp surface. Four upstream idioms account for most of what we
decline and three are incompatible with ours by construction: the wait-set /
`Waitable` executor (~128 rows, needs an allocator), runtime-queryable graph and
middleware (~55, needs discovery), runtime type erasure (~45, needs a dynamic
loader). The claim this phase can honestly reach is about PROGRAMS — the shapes
a ROS 2 node is actually written in compile and behave, everything else fails
loudly — and the in-tree ported templates are the measurement.
