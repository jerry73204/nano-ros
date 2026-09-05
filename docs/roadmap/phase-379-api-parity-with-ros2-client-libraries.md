# Phase 379 — the user API is rclc / rclcpp / rclrs, and something checks that

**Status (2026-08-30). W1 AND W2 COMPLETE; W3-W5 DECIDED and in progress.**
Every blocking decision the phase was holding is settled below — W3 lands as a
clean break, `handle-owns-node` stays with an identity instead of a pointer,
actions are addressed by UUID rather than by handle, and prelude membership is a
query over the ledger rather than taste. The waves are implementation now, not
adjudication.

**Status (2026-08-25). W1 AND W2 COMPLETE.** The correlator runs on all three
languages; every one of the 2397 items where the nano-ros user API does not
correspond to rclc/rclcpp/rclrs carries a written verdict, in 17 topic shards
under `docs/reference/api-parity-ledger/`. The Rust reference is **rclrs
v0.7.0** — the version question W5 was holding is settled, see below. `just check api-parity` is green and
wired into `just check`. W3–W5 are the corrections and are not started; W5 has
one decision recorded below that is not an implementer's to make.

**Implements.** RFC-0036 (divergences from the ROS 2 standard client APIs),
which this phase converts from a prose catalog into a checked one. Touches
RFC-0018 (C++ API design), RFC-0019/0020 (thin-wrapper discipline), RFC-0022
(entity API tiers), RFC-0037 (Rust/C user API surface).

## Why

nano-ros makes a drop-in claim: a ROS 2 developer can read and write it, and a
ported source file compiles against it with a build-glue change rather than a
rewrite. Phase 209 built the C++ half of that (`nros/rclcpp_compat.hpp`, the
`cmake/compat/include/rclcpp/` shim, `Findrclcpp.cmake`), so a `.cpp` that says
`#include <rclcpp/rclcpp.hpp>` really does compile here.

The claim is only worth something if the SHAPE underneath matches, and nothing
checked that it did. RFC-0036 is the catalog of the divergences we permit — and
it is prose. Prose about an API goes stale silently: RFC-0036 shipped calling
the Rust error type `RclrsError` when it had been `NanoRosError` for months, and
had to carry a "naming note" correcting itself. Issue 0338 is the same class one
level down: `Executor::spin` meant the OPPOSITE of `rclcpp::Executor::spin` here
(bounded, not blocking), so a user who wrote `exec.spin()` got a compile error
and the nearest-looking alternative `spin(ms)` silently returned early. That was
found by a person reading, once.

So the deliverable is a correlator: extract both surfaces from their real
sources, line them up, and report every item that does not correspond.

## W1 — the correlator (landed)

`scripts/api-parity.py`, with the extractors under `scripts/api_parity/`.

    scripts/api-parity.py                 # report all three languages
    scripts/api-parity.py --lang cpp      # one language
    scripts/api-parity.py --check         # fail on anything unledgered
    scripts/api-parity.py --suggest-renames   # pair look-alike unmatched names
    scripts/api-parity.py --include-internal  # compare the whole ROS 2 surface
    scripts/api-parity.py --topic pubsub              # one stage, all languages
    scripts/api-parity.py --by-topic                  # what each stage owes
    scripts/api-parity.py --refresh …     # re-derive the ROS 2 side
    scripts/api-parity.py --self-test

### How each side is obtained

Both sides are parsed, never grepped. The question the campaign asks is "do the
ARGUMENTS agree", and arguments are exactly what a regex over headers gets
wrong — default values, template parameters, `const &` versus value, and
macro-expanded visibility attributes (`RCLCPP_PUBLIC`) all defeat it.

| lane | ours | theirs |
| --- | --- | --- |
| C++ | `nros/nros.hpp` via clang JSON AST | `rclcpp` + `rclcpp_action` + `rclcpp_lifecycle` from `/opt/ros/<distro>` |
| C | `nros/nros.h` via clang JSON AST | `rclc` checkout **plus `rcl`** |
| Rust | rustdoc JSON over the `nros` facade, with the runtime features enabled | rustdoc JSON over `rclrs` **v0.7.0** |

Three things about that table are decisions rather than mechanics:

**Our side parses with no build.** `-DNROS_PLATFORM_NUTTX` selects the
COMMITTED size header (`nros_cpp_config_generated_nuttx.h`); every other
platform's sizes come from `build.rs`, which would make this tool depend on a
fixture being fresh. Both our surfaces parse with zero clang errors, and that is
enforced rather than tolerated — a partial AST silently drops declarations, and
a dropped declaration reads as a gap in our surface that is not really there.

**The C reference is rclc AND rcl.** rclc is a convenience layer, not a whole
API: its own examples call `rcl_publish`, `rcl_take` and `rcl_*_fini` directly
(`rclc_examples` has 23 `rclc_executor_init` calls against 6 `rcl_publish`).
Comparing against rclc alone scored our publish and take entry points as
inventions when they are the ROS 2 C API doing its job — 129 reference records
became 747 once `rcl` was included.

**The ROS 2 side is cached, and re-derivable.** `docs/reference/api-surface/*.json`,
for the reason `scripts/rmw-api-parity.py` caches its contract: the comparison
must run on a host with no ROS, no rclc checkout and no rclrs workspace, or it
runs on one host and rots everywhere else. Each file records its provenance
(distro, git ref, crate version). OUR side is never cached — caching it would
defeat the tool, which exists to notice when an edit moves us away from ROS 2.

### Only PUBLIC ROS 2 items are compared

nano-ros aligns to the API a ROS 2 user writes. It does not align to rclcpp's
callback type erasure, rcl's wait-set plumbing, or the generated accessors of
`rcl_interfaces`, and counting those as gaps manufactures work that should never
be done.

`public_surface.py` decides on the DECLARING FILE, not the name — a path is a
fact about the library's own organisation, a name is a guess about intent.
`AnyExecutable` looks internal and is; `Waitable` does not and is; their headers
say so either way. Three tiers, in order of how much judgement each needs:
generated message packages (`*_msgs`, `*_interfaces`, `rosidl_*`), `detail/`
directories (upstream's own marker), and a short enumerated list of plumbing
headers each carrying the reason a user never writes it.

The first tier alone is **216 of the C lane's 632** `theirs-only` rows — every
`rcl_interfaces__srv__GetParameters_Request__init` and its family. Those are
codegen output on BOTH sides, governed by RFC-0023/0033; comparing them compares
two code generators, not two APIs.

The report always prints what each tier removed. A filter that quietly shrinks a
number is indistinguishable from progress. `--include-internal` turns it off.

### Systematic signature changes are stated once, not per site

Some divergences are not per-item decisions — they are one decision applied
everywhere. `rcl` threads an `rcl_allocator_t *` through six entry points;
nano-ros has one global allocator, so it appears in none of them. That is one
sentence, and writing it into six ledger rows is how the sentence stops being
read.

`signature_rules.py` holds those rules, each with the constraint it answers and
the entry points it covers. A divergence a rule explains is bucketed
**`systematic`** and inherits the rule's constraint; only what NO rule explains
stays `differs` and needs a ledger row. The five rules, all read off the first
report rather than guessed:

| rule | constraint |
| --- | --- |
| `no-allocator` | one global allocator (`nros_platform_alloc`, gated by `check-no-direct-kernel-alloc`) — a per-call allocator argument could only be passed the same value or a wrong one |
| `compile-time-options` | QoS and entity options are selected at compile time (RFC-0036, RFC-0045); accepting an options struct would promise a negotiation the backends do not perform |
| `no-argv` | an embedded image has no argc/argv; boot config is baked |
| `executor-owns-no-entity-storage` | the callback and message buffer bind to the ENTITY at creation (RFC-0041), so the executor has no per-entity storage to be told the size of |
| `handle-owns-node` | our entity handles retain their node, so teardown does not ask the caller to still hold it — one pointer per entity against a lifetime the caller would otherwise enforce with no allocator and no ownership types |

Three kinds of rule, because there are three ways one decision shows up:

* **A dropped parameter class** — the five above. Reconciles an arity.
* **A type substitution** (`TYPE_EQUIVALENCES`) — the arity already matches and
  one position is spelled differently on each side: `const char*` against
  `const std::string&` (`cstr-not-string`, RFC-0018), a value or reference
  against a `SharedPtr` (`no-shared-ptr`, RFC-0022), `&mut Self` against
  `&Arc<Self>` (`no-arc-self`), `int` against `size_t` (`sized-integer`).
  Without these, `create_subscription`, `create_service`, `create_client`,
  `publish`, `QoS::keep_last` and a dozen more each read as an unexplained
  difference when between them they are three sentences.
* **An alignment** (`ALIGNMENTS`) — the two signatures agree about everything
  except where the RESULT goes. `create_publisher(Publisher<M>& out, const
  char*, const QoS&)` against `create_publisher(const std::string&, const QoS&,
  const PublisherOptions&)`: same arity, every position off by one. Comparing
  in place finds no agreement; comparing after the shift finds two.

Matching drops the FEWEST parameters that explain the difference, in priority
order, stopping the moment the arities overlap. Applying every rule at once
over-explains: `rcl_publisher_init` takes both an options struct and a node,
ours takes the node and not the options, and dropping both leaves theirs one
parameter shorter than ours — a difference invented by the explanation. The
report names the rules actually needed.

Deleting a rule re-opens every row it covers, which is the intended way to
challenge one.

### What the tool refuses to do

**It does not use an authored name map.** A map for ~2000 items is a document
nobody finishes and nobody re-reads. Names already correspond by construction —
that is the project's stated goal — so the tool ASSUMES correspondence and makes
disagreement the thing a human has to write about. That puts the labour on
exactly the rows the campaign cares about.

**It does not compare full types by default.** A type difference is usually
RFC-0018's `std::string` → `const char *` rule applied again; reporting those
would bury the real findings under hundreds of rows restating a decision made
once. Arity is the primary comparison, because an arity difference means the two
APIs ask the user for different things. Full parameter lists print alongside.

### Four tool defects found before trusting the output

Each produced findings that looked real. They are recorded because the tool's
credibility is the deliverable:

1. **File-based scoping.** clang emits `loc.file` only when it CHANGES, so
   recovering a decl's file means carrying state across a strict pre-order walk
   of a 400 MB AST. Getting it subtly wrong attributed `std::shared_mutex` and
   `builtin_interfaces::msg::Time_` to rclcpp while dropping `rclcpp::Node`
   entirely. Fixed by scoping on NAMESPACE, which is already on the path down
   and cannot drift.
2. **Single-crate rustdoc.** rustdoc writes one JSON per crate, and a
   re-exported item's id belongs to the crate that DEFINED it. `nros` is a
   facade, so without cross-crate resolution the entire executor, node-context
   and publisher surface read as absent — 168 items instead of ~750.
3. **Default arguments not counted.** clang marks a defaulted parameter with
   `"init": "c"` and attaches the default expression as a child of whatever
   literal was written — `IntegerLiteral` for `= 10`, not something ending in
   `Expr`. Counting declared parameters reported
   `nros::Executor::spin(int32_t poll_ms = 10)` as diverging from
   `rclcpp::Executor::spin()`, when `exec.spin()` compiles in both — which is
   precisely the convergence issue 0338 landed on purpose. **A checker that
   flags a convergence someone deliberately made is worse than no checker.**

4. **rclcpp's inheritance split read as divergence.** rclcpp splits every entity
   into a type-erased base and a typed subclass — `Publisher<T>` IS-A
   `PublisherBase`, and `get_topic_name`, `assert_liveliness`,
   `wait_for_service` and `cancel` are declared on the base. `nros::Publisher`
   is one class, so those appeared as an `ours-only` row and a `theirs-only` row
   that never mentioned each other. Folded the suffix, as the rclrs `XState`
   split already was. Note the fold has to reach the TYPE key and not only a
   method's owner: member keys are built from the type key, so folding the
   owner alone changes nothing — which it did, silently, until the numbers
   refused to move.

Before defect 3 was fixed the C++ lane reported 11 argument divergences. After
it, zero. All eleven were the tool's. Defect 4 then moved 7 more rows into
`same` and removed 35 phantom `theirs-only`.

## The first report

    same       both sides have the name, and the arguments agree
    arity-only the arities overlap and NOTHING else does -- the agreement is in
               the count. `init` is the example: ours takes (locator, domain),
               rclcpp's takes (argc, argv), and both take two.
    systematic the arguments differ and a signature rule explains it
    differs    the arguments differ and NOTHING explains it
    +          ours only
    -          theirs only

Against the PUBLIC ROS 2 surface only:

| lane | reference | same | arity-only | systematic | differs | ours-only | theirs-only |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| C++ | rclcpp (humble) | 44 | 9 | 8 | **0** | 217 | 766 |
| C | rclc+rcl (humble) | 67 | 0 | 24 | **8** | 306 | 385 |
| Rust | rclrs 0.5.1 | 40 | 3 | 1 | **0** | 709 | 327 |

Not-API excluded: 216 message + 33 plumbing (C), 2 + 12 (C++), 2 (Rust).

The C++ and Rust lanes show 0 `systematic` because they show 0 `differs` — no
rule is ever consulted. The out-parameter rule (`status-return-out-param`) is
carried for when a C++ row does diverge; it fires nowhere today, and the phase
doc says so rather than implying the rule earned its place.

Each language has a different problem, and none of them is the one the campaign
was opened to fix.

**C++ — the shape is right; the COVERAGE is not.** Zero argument divergences.
Every name we share with rclcpp, we spell the same and accept the same arity.
What is missing is surface: `Node::declare_parameter`, `Node::create_wall_timer`,
`Node::get_clock`, `Node::count_publishers`, `Node::get_fully_qualified_name`,
and types a ported node names directly — `Clock`, `Duration`, `Context`,
`HistoryPolicy`, `DurabilityPolicy`, `FutureReturnCode`, `CallbackGroupType`.
The 804 includes rclcpp internals a user never writes (`AnyExecutable`,
`GenericRate`, the memory strategies), so it is an upper bound, not a work list
— W2 turns it into one.

**C — 32 argument divergences, of which 24 are five decisions and 8 are open.**
The rules above cover the allocator, the options structs, argv, the
executor's entity storage and the node-carrying handles. What remains is the
real work list, and it is specific:

* `timer_get_period` — ours RETURNS the period, rcl writes it through an
  `int64_t *`. The inversion is ours and has no stated reason.
* five `lifecycle_*` entry points — ours take
  `struct Option_LifecycleCallbackFnCtx` plus a `void *` context where rclc
  takes a bare `int (*)(void)`. A Rust `Option<...>` type name reaching the C
  ABI is its own finding, separate from the arity.
* `lifecycle_change_state` — rclc takes a trailing `bool`; we do not.
* `make_node_a_lifecycle_node` — ours takes 2 parameters against rclc's 5.
* `action_publish_feedback` — ours `(server, goal, buf, len)` against
  `(goal_handle, void *)`.

Eight rows, each needing one decision. That is a tractable W4, where 32 was a
survey.

**Rust — we EXPORT far too much.** 709 items the `nros` facade makes public that
rclrs has no equivalent for: `BOOT_SET_DOMAIN`, `BakedBootConfig`, `BoardConfig`,
`ActionExecutor`, `CallbackCtx`, `ActionTag`, and hundreds more. Some are
genuine RTOS extensions and belong. Many are internals that reached `pub use`
because a facade re-exports whatever it is handed. A user reading `nros::` to
learn the API meets all 709, which is its own kind of divergence from rclrs.

Note the reference: **rclrs 0.5.1**, while RFC-0036 says we target 0.7.0. 0.5
introduced the `Node = Arc<NodeState>` split that the correlator has to fold
(the methods live on `NodeState`; a user writes `Node`). Which version we mirror
is a decision W5 has to make and record, not a detail.

## W2 — classify every non-matching row (COMPLETE, 2026-08-25)

2158 rows, 17 stages, all three languages. Ten stages ran as parallel agents,
one per topic, each owning one ledger shard.

| stage | gap | rename | divergence | extension | declined | total |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| types | 2 | 0 | 7 | 2 | 5 | 16 |
| metadata | 0 | 0 | 0 | 36 | 0 | 36 |
| init | 4 | 0 | 7 | 4 | 15 | 30 |
| node | 7 | 1 | 40 | 30 | 23 | 101 |
| pubsub | 21 | 16 | 125 | 81 | 109 | 352 |
| service | 8 | 24 | 85 | 47 | 77 | 241 |
| timer | 36 | 12 | 76 | 36 | 87 | 247 |
| qos | 8 | 53 | 59 | 22 | 34 | 176 |
| param | 32 | 22 | 25 | 16 | 31 | 126 |
| action | 7 | 8 | 67 | 57 | 23 | 162 |
| exec | 2 | 5 | 30 | 94 | 91 | 222 |
| lifecycle | 19 | 3 | 22 | 38 | 44 | 126 |
| log | 13 | 2 | 19 | 1 | 7 | 42 |
| graph | 36 | 1 | 8 | 0 | 16 | 61 |
| serde | 0 | 5 | 0 | 37 | 1 | 43 |
| boot | 1 | 1 | 0 | 11 | 0 | 13 |
| other | 1 | 1 | 16 | 122 | 24 | 164 |
| **total** | **198** | **155** | **732** | **634** | **678** | **2397** |

(Totals are post-`v0.7.0`; the per-stage rows above were counted against 0.5.1
and the bump added 239 Rust rows, mostly to `action`.)

### What the campaign was for, and what it actually found

It was opened to align our API with ROS 2's. **The findings that matter most are
not ROS 2 mismatches — they are defects, and places where our own three
languages disagree with each other.** Nine issues, each reproduced before
filing:

* **0796** — the action server's result slab only ever grows; once full, every
  result is silently dropped and `complete_goal_raw` returns `()`. And the C++
  callback tier hardcodes `Succeeded`, so an **aborted goal is reported to the
  client as succeeded**.
* **0792** — twelve C lifecycle entry points cannot be called from C at all
  (`Option_LifecycleCallbackFnCtx` is an opaque forward declaration passed by
  value). `nros_lifecycle_init` discards the node it is handed.
* **0793** — C ships two disjoint parameter stores; parameters declared on the
  one `parameter.h` reaches first are invisible to `ros2 param`, and its
  accept/reject callback is installed where no service reads it.
* **0791** — we are visible in the ROS graph and cannot read it: 12 vtable slots,
  all `None`, while both backends already run the discovery machinery.
* **0794** — the baked boot config defines four fields and the emitter sets one;
  `BOOT_SET_NAMESPACE` is set by nothing.
* **0795** — `nros.h` omits `log.h` and `borrowed.h`, so a C author who finds no
  logger writes `printf` — fatal on Zephyr native_sim (issue 0589).
* **0789** — nros-cpp has no clock/time/duration surface; ROS time exists in
  Rust and not in C.
* **0790** — no shutdown hook in any language.
* **0788** — six verbs spelled differently across our three languages, twice
  with one language shipping both spellings.

**RFC-0036 is stale in three separate places** — the Errors row (0783), the
lifecycle line (we implement full REP-2002 and register five services; only
`~/transition_event` is missing), and the parameter-callbacks line, which is
wrong in both directions. That pattern is why this phase built a checker instead
of another prose catalog.

### Three correlator defects the stages found

Each had been silently skewing results, and each is fixed:

1. **The Rust surface was extracted with `nros`'s empty default feature set.**
   `default = []` and 18 `#[cfg(feature = "rmw-cffi")]` blocks gate the entire
   runtime, so the correlator documented a facade with `Executor`, `NodeHandle`,
   `EmbeddedPublisher` and `Promise` removed — inflating `theirs-only` and
   deflating `ours-only` by 437 rows. `rust:Executor` was reported as
   theirs-only. Fixed with an explicit feature list.
2. **The rclrs `*State` fold was applied to a method's owner and not to the
   type**, so `PublisherState`'s members never met ours. Closed 109 decisions at
   once. The C++ `*Base` fold had the identical bug in W1.
3. **The topic patterns were written in the ROS 2 spelling and are
   case-sensitive**, so our spelling of the same word fell through to `other` —
   `Qos` against `QoS` alone stranded ~52 rows, splitting both halves of a rename
   across two shards. `other` was never residue; it was four topics leaking plus
   one real one (`metadata`, now a stage).

### Two things the correlator structurally cannot see

Recorded so nobody reads a `same` verdict as a guarantee:

* `spin_once` correlates as `same`. rclcpp's executes the **next one** ready
  item; ours executes **all** of them (RFC-0002 §3).
* `GoalResponse`/`CancelResponse` correlate as `same` while our discriminants
  are 0-based against rclcpp_action's 1-based. Enumerator comparison is a
  feature the tool does not have.

### The ledger, and how to add to it

One shard per topic, `docs/reference/api-parity-ledger/<topic>.json`, holding
that topic's rows in all three languages. A row on a TYPE covers its members
while they share a bucket; a glob row (`c:action_*`) covers a flat C family and
must declare the bucket it covers. `just check api-parity` fails on any
unexplained divergence and on any row filed in the wrong shard, naming both.

## W3 — close the C++ coverage gaps a ported node actually hits

Driven by W2's `gap` rows, ordered by what phase 209's port templates and the
autoware survey nodes call. The `types` stage has already produced two:
`cpp:FutureReturnCode` (we express SUCCESS and TIMEOUT, not INTERRUPTED, so a
ported `spin_until_future_complete` caller cannot tell shutdown from timeout)
and `rust:RclReturnCode` (we have the type and do not export it — issue 0783).
~~`init` added a third and it is the largest of them: **nano-ros has no shutdown
hook at all**.~~ **DONE — corrected 2026-08-30.** True when written; the
capability landed since. `Executor` carries `add_pre_shutdown_callback` /
`add_on_shutdown_callback` (plus `remove_*` and `shutdown_callback_count`) over
fixed-capacity arrays sized by `MAX_SHUTDOWN_CBS`, and the C surface exposes
`nros_executor_add_pre_shutdown_callback` / `..._add_on_shutdown_callback`. The
prediction in the original text held exactly: nothing about `no_std` prevented a
fixed-capacity callback array.

Recorded rather than deleted, because it was nearly re-implemented from this
paragraph — a phase doc that describes a gap someone has since closed is a
worse trap than one that describes nothing, and the only thing that caught it
was grepping for the symbol before writing it.

`node` added the first `rename` rows, and they are the cheapest work in the
campaign: **`Node::create_subscriber` should be `create_subscription`.** rclrs,
rclcpp and rclc all say subscription, and so do our own C
(`nros_subscription_init`) and C++ (`Node::create_subscription`) — Rust is the
odd one out among our three languages as well as against ROS 2.
`subscriber_count` and `subscriber_topic_info` move with it.

`pubsub` added two more rename families, both of them one word used
consistently on each side:

* **`take` → we say `try_recv`.** rcl, rclcpp and rclrs all spell the
  non-blocking receive `take`. Both are non-blocking and both report emptiness
  without failing, so nothing asks for the other word — and `try_recv` is Rust
  channel vocabulary that reads as a different contract to a ROS 2 user.
  **EXECUTED 2026-09-03 (W6 decision 1).**
* **SERIALIZED → we say RAW.** `publish_serialized_message`/`take_serialized`
  against `publish_raw`/`try_recv_raw`. **Receive half executed 2026-09-03**
  (`try_recv_raw` -> `take_serialized`); `publish_raw` still says RAW, and
  `c:publish_serialized_message` now owns that half on its own.

Plus `create_publisher`/`create_subscription` against our free-function
`make_publisher`/`make_subscription`, and `Publisher::borrow_loaned_message`
against `Publisher::loan` — the C loan API has a real shape reason (a token over
a byte range, because C has no templates and the wrapper would need an allocator
to own), but the C++ one returns a typed RAII handle on both sides, so only the
verb differs there.

Also `Node::now`
(the accessor a ported rclcpp publisher uses to stamp a header, tied to the
`Clock` gap), `node_get_domain_id`, `node_get_fully_qualified_name`,
`node_resolve_name` and `node_is_valid`. Expected shape: `create_wall_timer` as a name
alongside `create_timer`, `declare_parameter` over the current parameter
surface, `get_clock`/`Clock`/`Duration`, the QoS policy enums under their rclcpp
names.

`--suggest-renames` already names the cheapest ones, and they are cheap because
none has a platform reason: `Service::send_reply` against rclcpp's
`send_response`, `Service::try_recv_request` against `take_request`,
`Subscription::try_recv` against `take`, `make_publisher`/`make_subscription`
against `create_publisher`/`create_subscription`, and `Timer::is_cancelled`
against `is_canceled` — a spelling. The QoS accessors (`deadline_ms`,
`lifespan_ms`, `liveliness_lease_ms`) are a different case: the `_ms` suffix
encodes that we take an integer where rclcpp takes a `Duration`, so the name
follows whatever W3 decides about `Duration`, not the other way round.

## W4 — settle the eight open C divergences

The 24 systematic rows are already argued, in `signature_rules.py`; W4's job
there is only to check each constraint still holds and move the text into
RFC-0036. The eight above each need a decision: a `divergence` row naming a
constraint, or a signature change. No row survives as "that is just how it is".

`Option_LifecycleCallbackFnCtx` is the one to look at first — a generated Rust
type name in the C ABI is a leak regardless of what the arity comparison says
about it.

## W5 — the Rust facade, and which rclrs we mirror

Two decisions, neither of them mechanical:

* Which rclrs version is the target — 0.5.1 (what exists here) or 0.7.0 (what
  RFC-0036 claims). They differ in the `Node`/`NodeState` split, which changes
  what "matching rclrs" even means.
* What `nros::` should export. 709 items is not a surface a user can read. The
  likely answer is a `nros::prelude` that IS the rclrs-shaped API and an
  explicit second tier for the RTOS-specific machinery — but that is RFC work,
  not a rename sweep.

## Blocking decisions

### Settled: the Rust reference is rclrs v0.7.0 (2026-08-25)

The latest release, which is what RFC-0036 already claimed. The recorded surface
was 0.5.1 until then; the bump took it from 129 records to 213 and produced two
findings that outlive the version question:

* **rclrs gained ACTIONS**, which 0.5.1 did not have, and modelled a goal's life
  as a TYPESTATE chain — `RequestedGoal` → `AcceptedGoal` → `ExecutingGoal` →
  `TerminatedGoal`, each a type whose methods are only the transitions legal
  from that state. rclcpp_action uses `async_send_goal` with a shared
  `ClientGoalHandle`. **ROS 2 does not agree with itself**, so "match ROS 2" has
  no single answer for actions and W5 must pick a side and record which. Ours is
  neither: goals live in a static arena addressed by UUID, because a typestate
  chain hands the application an owning handle per goal and with a
  fixed-capacity arena the storage cannot follow the handle.
* **rclrs converged on our timer model.** 0.7.0 has `create_timer_inert` /
  `create_timer_oneshot` / `create_timer_repeating` — the same three modes our
  `TimerMode` carries, arrived at independently. Rows this campaign recorded as
  divergences against 0.5.1 are now shape agreements with different placement
  (ours sets a mode on an entity in the static table; rclrs constructs one per
  mode), and they say so. `cpp:Node::create_timer_oneshot` is still an extension
  against rclcpp, but no longer an invention.

The lesson worth keeping: **the reference moves.** A prose catalog would have
gone on describing 0.5.1; `--refresh` re-derived the surface and the ledger's own
gate found the 124 rows that needed re-deciding.

### Settled: W3 lands as a CLEAN BREAK (2026-08-30)

No deprecation shims. The renames are source-breaking on the Rust surface and
they land in one release, one commit per family — a user who upgrades twice and
is renamed twice is worse off than one who is renamed once.

Ordering constraint that survives the decision: the QoS accessors (`deadline_ms`,
`lifespan_ms`, `liveliness_lease_ms`) encode that we take an integer where
rclcpp takes a `Duration`. Their names FOLLOW the `Clock`/`Duration` decision, so
that decision comes first or they are renamed twice — which is exactly what a
clean break is meant to avoid.

### Settled: `handle-owns-node` STAYS, and becomes checkable (2026-08-30)

The signature keeps its shape — `nros_publisher_fini(&pub)`, not rcl's
`rcl_publisher_fini(&pub, &node)` — but the STORAGE changes, because the rule as
written asserted a platform decision that the implementation did not earn.

Today each entity holds `const struct nros_node_t *node`, which assumes the
node's ADDRESS is stable for the entity's lifetime. Examined per platform:

* **Bare metal is the safe case.** No MMU, no runtime relocation, XIP, handles
  in `.bss`. An address taken at init is stable until reset.
* **RTOS with dynamic tasks is not.** A task declaring `nros_node_t node;` on
  its stack and creating a publisher that outlives it leaves the entity pointing
  into memory `vTaskDelete` has freed. "Handle on the task stack" is a normal C
  idiom, not a pathological one.
* **C cannot prevent the copy.** `nros_node_t a = b;` is legal and silent, and
  every entity built against `b` still points at `b`. Rust would forbid it with
  ownership types; C has none and we have no allocator to hide an indirection
  behind.
* **Teardown order becomes an unenforceable caller obligation.** Destroy the
  node first and `*_fini` dereferences a dead pointer — on the path that runs
  when something has already gone wrong.

rcl's alternative does NOT remove that assumption; it relocates it to the caller
and makes it unverifiable — pass a different node and it is UB, pass a dangling
one and it is the same dereference spelled at the call site.

So: keep the signature, replace the pointer with an identity into the
`NodeRecord` table that already exists (`NROS_EXECUTOR_MAX_NODES`):

```c
typedef struct nros_node_ref_t { uint16_t slot; uint32_t generation; } nros_node_ref_t;
```

`generation` is bumped when a slot is RELEASED, and `0` is reserved so a zeroed
struct — the common C mistake — is never accidentally valid. Resolution bounds
the slot and compares the generation, so every hazard above turns from a
dereference into a return code. `u32` rather than `u16` deliberately: 16 bits
would leave a 65 535-cycle wrap to reason about, and 4 bytes buys the argument
away.

ABI break, accepted: nothing is released yet, and it lands with the W3 break.

The constraint the `divergence` row can now NAME: the caller cannot be asked to
enforce a lifetime C gives it no tools to enforce, so the runtime enforces
IDENTITY instead — and identity is checkable where a pointer is not.

### Settled: actions are addressed by UUID, not by handle (2026-08-30)

W5 must pick a side because ROS 2 does not agree with itself. It picks NEITHER
library, and the reason is in rclrs's own types rather than in preference.

`rclrs` 0.7.0 (`action/action_server/executing_goal.rs`):

```rust
pub struct ExecutingGoal<A: Action> { live: Arc<LiveActionServerGoal<A>> }

pub fn succeeded_with(self, result: A::Result) -> TerminatedGoal {
    self.live.transition_to_succeed(result);
    TerminatedGoal { uuid: *self.live.goal_id() }
}
```

The typestate is a compile-time wrapper OVER AN `Arc` — not an alternative to
refcounting, a lint on top of it. `rclcpp_action`'s
`shared_ptr<ServerGoalHandle>` is the same requirement stated plainly. Both need
an allocator; neither is portable to `no_std`.

And note where `succeeded_with` lands: `TerminatedGoal { uuid }`. **rclrs itself
falls back to the UUID** once the typestate ends, because the UUID is what the
ACTION SPEC defines — `SendGoal`/`CancelGoal`/`GetResult` and the feedback and
status topics all key on `goal_id`. The handle is the ergonomic layer, and it is
the layer that costs an allocator.

Ours keeps the spec's identity and omits the ergonomics:

```rust
server.publish_feedback(exec, &goal_id, &fb)?;
server.succeed(exec, &goal_id, result)?;   // W3 rename, see below
```

Honest asymmetry, to be recorded in the row rather than only the wins:
`succeeded_with(self)` makes use-after-terminal a COMPILE error; ours can only
make it a runtime one.

### Settled: prelude membership is a LEDGER QUERY, not taste (2026-08-30)

709 items is not a readable surface, and the current `nros::prelude` mixes user
API with machinery — `CdrReader`, `HandleSet`, `InvocationMode`,
`MetadataRecorder`, `NodeRuntimeAdapter` sit beside `Node` and `QoSProfile`.

Two tiers, with a mechanical rule:

> A name belongs in `nros::prelude` IFF the parity ledger gives it a
> non-`extension` verdict — i.e. it has a correspondent in rclrs/rclcpp/rclc.

Everything else stays reachable at `nros::`; the RTOS machinery moves to
`nros::embedded`. The rule needs a small allow-list of extensions that are
load-bearing (`ExecutorConfig`, `SpinOptions` — nothing starts without them),
each with a one-line reason, same discipline as a `divergence` row.

Gated by `check-api-parity`: `prelude ⊆ {non-extension} ∪ ALLOWED_EXTENSIONS`.
Without the gate the rule decays into a comment, which is the failure mode this
repo keeps re-learning.

## W6 — the three verb decisions, settled 2026-09-03

Three rows blocked the `try_recv`/`take`, readiness and goal-count families —
about a third of the remaining work. All three are settled, and they share one
root cause worth stating first:

> **Upstream separates "did the check work" from "what is the answer". Every one
> of our divergences here is a place where we collapsed those two channels into
> one value.**

`rcl` does this explicitly. From `rcl/graph.h:829`, on
`rcl_service_server_is_available(node, client, bool *is_available)`:

> `RCL_RET_OK` if the check was made successfully **(regardless of the service
> readiness)**

Return code = did it work. Out-param = the answer. rclcpp then collapses to a
bare `bool` and moves the error to **exceptions** — confirmed by its sibling
`Subscription::take`, which documents `\throws any rcl errors from rcl_take`.

RFC-0018 forbids exceptions (RTOS portability), so rclcpp's collapse is not
available to us. That is not a divergence to argue for — it is the reason we
must keep upstream's two channels, in a carrier that works without unwinding.

### Decision 1 — `try_recv` becomes `take`; the squatter is deleted

**LANDED 2026-09-03** — see “How decision 1 landed” at the end of this section.

`Subscription::try_recv(M&) -> Result` and `rclcpp::Subscription::take(M&,
MessageInfo&) -> bool` are THE SAME OPERATION: non-blocking, consuming. Ours
already carries `success` / `TryAgain` / `NotInitialized` / `Error` in a
`Result` where rclcpp carries `bool` + a throw. So the rename is semantic
parity, not a cosmetic move, and `try_recv` is Rust-channel vocabulary that
reads as a different contract to a ROS 2 user.

**The blocker was a name squatter with the opposite contract.**
`PollingSubscription::take(M& out) -> bool` drains to the newest sample and
returns *"true if any value has ever been received (cached-or-new)"* — NON
consuming, retained-latest. A ported node writing the idiomatic drain loop

```cpp
while (sub.take(msg)) { process(msg); }
```

terminates under rclcpp and **spins forever on one stale sample** under ours.
Same name, same signature, no compile error.

It yields: it is a convenience duplicating `take_data()`, and it has **zero real
callers** (one compile test) against 5 for the faithful Autoware mirrors
`take_data()` / `take_new_data()` (issue 0278), which stay.

**How decision 1 landed — 2026-09-03.**

The rename is one mapping, applied to the receive verb wherever it names the
subscription / service / client operation, in all three languages:

| was | is |
| --- | --- |
| `try_recv` | `take` |
| `try_recv_raw` | `take_serialized` |
| `try_recv_request` | `take_request` |
| `try_recv_reply_raw` | `take_response_raw` |
| `try_recv_sequence` | `take_sequence` |
| `try_recv_validated` | `take_validated` |

`take_serialized`, not `take_raw`: ROS 2 says SERIALIZED for the pre-CDR byte
form, which is what `c:take_serialized_message` settles. The publish half
(`publish_raw`) is deliberately NOT in this move — it is
`c:publish_serialized_message`'s own row, which now carries that decision alone.

**The direction is worth stating, because it is the opposite of what a "rename
for parity" usually means.** The RMW vtable one layer down
(`nros-rmw-abi/include/nros/rmw_vtable.h`) has spelled these slots `take`,
`take_request`, `take_response`, `take_sequence` since phase-376 W3.b. So the
USER API was the only layer still saying `try_recv`: this move takes it onto the
vocabulary its own foundation already used, and rcl/rclcpp agreeing is a
consequence rather than the argument. Nothing in the vtable layer was touched.

Deprecation, per the settled policy:

* **C** — `NROS_DEPRECATED_MSG` `static inline` forwarders in the hand-written
  module headers (`nros/subscription.h`, `nros/service.h`, `nros/client.h`),
  the shape `nros/parameter.h`'s `nros_param_*` family established. An inline
  definition has no external linkage, so the `take_*` name stays the only
  exported symbol and the ABI cost is zero — a SOURCE promise, not a binary one.
* **C++** — `[[deprecated("…")]]` header-only forwarders naming the replacement,
  the shape `qos.hpp`'s `*_raw()` established.
* **Rust** — nothing. A rename there breaks IMPLEMENTORS of the RMW traits
  rather than callers, and a compile error is what a backend author wants.

Both halves are gated, because a dropped attribute otherwise reads as a pass:
`receive_verb_aliases.{c,cpp}` prove the old spellings still resolve WITH THE
SAME SIGNATURE (via function pointers in C, instantiation in C++), and
`receive_deprecation_probe.{c,cpp}` are expected-FAILURE compiles under
`-Werror=deprecated-declarations`. All four run in `just check c` / `just
check cpp`, beside the W5 parameter and service pairs.

What the ledger did with it: `c:take` and `c:take_serialized_message` are
`divergence` with `rename.status: resolved` (what is left is the byte
orientation — no typesupport dispatch, no allocator — which is `c:publish`'s
constraint mirrored on the receive side); `c:take_sequence`, `c:take_request`
and `c:take_response` are `declined` (only the module prefix differs, which is
`c:get_zero_initialized_publisher`'s precedent); `c:take_request_with_info` and
`c:take_response_with_info` are `gap` (they were deferring to the verb, and what
they were hiding is an absent `rmw_service_info_t` channel, not a spelling).
`cpp:Subscription::take` and `cpp:Service::take_request` are `divergence` on
their remaining parameter, not on their name. The `try_recv*` rows survive as
`extension` rows describing DEPRECATED spellings, retired with the rest of the
set in W7 step 4.

### Decision 2 — `service_is_ready`, returning `Expected<bool>`

The ledger asked to merge three spellings into `service_is_ready`. It is right
about the NAME and wrong about the DIRECTION: the tri-state is not ours to drop,
it is rcl's design.

| ours | shape | disposition |
| --- | --- | --- |
| `ClientTrait::server_available() -> Result<bool, E>` | exactly rcl's two channels | **keep**, rename to `service_is_ready` |
| `ClientTrait::is_server_ready() -> bool`, default `true` | rclcpp's collapse WITHOUT exceptions | **delete** |
| `nros_client_server_available(c, int32_t *out)` | rcl's shape, `-1` inside the out-param | keep, drop the `-1` |
| `nros_client_service_is_ready(c) -> bool` | the collapse | **delete** |

`is_server_ready`'s default of `true` tells a backend that cannot answer (XRCE
has no participant enumeration) that the server is ready. That is an optimistic
lie, and rclcpp only earns its bare `bool` by throwing.

**The carrier, and the general rule.** `nros::Expected<T>` already exists
(`result.hpp:198`), is NOT gated behind `NROS_CPP_STD`, stores inline and
allocates nothing:

```cpp
Expected<bool> service_is_ready() const;
//  ok(true)                      -> a server is discovered
//  ok(false)                     -> none yet
//  error(ErrorCode::Unsupported) -> the backend cannot answer
```

So, stated once rather than per site — **an RFC-0036 amendment**:

> Wherever rclcpp returns `T` and throws, nano-ros returns `nros::Expected<T>`.
> Rust returns `Result<T, E>`. C keeps rcl's literal shape,
> `nros_ret_t f(args…, T *out)`.

This formalises existing practice (`Node::make() -> Expected<Node>`) rather than
inventing a convention. It also removes the `int32_t *out` carrying `-1`, which
duplicates in the out-param what the return code already says.

### Decision 3 — the action goal-count pair takes rcl's shape

Upstream ships no count function; the nearest is
`rcl_action_server_get_goal_handles(server, ***handles, size_t *num_goals) ->
rcl_ret_t` — enumeration, count as a by-product, **the same two channels**.

Ours ships two, tier-disjoint, each collapsing errors into the answer:

* `..._get_active_goal_count(server) -> size_t` — callback tier
  (`STATE_INITIALIZED`, executor arena). Returns **0** on error, so "error" and
  "no goals" are indistinguishable.
* `..._active_goal_count_raw(server) -> int32_t` — polling tier
  (`STATE_POLLING`, `PollingServerCore`). **Negative** = error code.

Zero callers of either. Both collapse; neither matches upstream. So:

```c
nros_ret_t nros_action_server_get_active_goal_count(
    struct nros_action_server_t *server, size_t *out);
```

One function, branching on `state` internally to serve both tiers. Resolves the
duplicate AND both bad error channels in one move. `get_` stays — it is the C
convention here (49 `nros_*_get_*` functions), and W5 group A deliberately ADDED
it to C (`nros_lifecycle_get_state` -> `..._get_current_state`).

## W7 — migration order

The ledger's 95 remaining `rename` verdicts are not 95 units of work. Measured
2026-09-03:

| class | rows | |
| --- | ---: | --- |
| landed, row never collapsed | 29 | bookkeeping, no code |
| genuinely open | ~34 | |
| deprecated aliases awaiting retirement | 24 | **last** |
| blocked on a decision | 8 | settled by W6 |

Do them in this order. It is not arbitrary: each step shrinks the input to the
next, and the last step is irreversible for out-of-tree consumers.

**Step 1 — collapse the landed rows (no code).** 29 rows report bucket `same`
while still carrying verdict `rename`; find them with

```
python3 scripts/api-parity.py --show same | grep -E 'rename\s+same'
```

The campaign's own convention is that a paired row COLLAPSES on landing: the
ours-only half is deleted and the retained row keeps the history with a dated
note. These never collapsed. They break nothing — the gate is green — but they
inflate the work list and mislead the next reader about what is left.

**Step 2 — execute W6's three decisions.** In dependency order:

1. Delete `PollingSubscription::take()` (frees the name).
2. `try_recv` -> `take` across C, C++ and Rust, plus `try_recv_raw` ->
   `take_serialized`, `try_recv_request` -> `take_request`,
   `try_recv_reply_raw` -> `take_response_raw`. This is the big one and it
   unblocks the ~32 rows in `service` and `pubsub` that defer to `c:take`.
   **DONE 2026-09-03.**
3. `service_is_ready` returning `Expected<bool>` / `Result<bool, E>` /
   `nros_ret_t + bool *out`; delete the two collapsing spellings.
4. The goal-count merge.

Deprecated forwarders per the settled policy: `NROS_DEPRECATED_MSG` static
inline for C, `[[deprecated("…")]]` for C++, and NOTHING for Rust trait methods
— a rename there breaks implementors rather than callers, and a compile error is
what a backend author wants.

**Step 3 — the remaining open rows**, which are mostly `param`, `pubsub`,
`service` and `timer` leftovers once step 2 lands.

**Step 4 — retire the deprecated aliases, as ONE batch, last.** By then the set
is: the `nros_param_*` family (**34 function forwarders plus 4 silent typedefs = 38 items**; the
earlier "25" was a miscount, remeasured 2026-09-04 from `parameter.h`'s
`#ifndef NROS_NO_DEPRECATED_PARAM_ALIASES` block — the 15 array forms,
`declare`/`get`/`set` x byte/bool/integer/double/string, account for the gap.
The 4 typedefs are the ones that vanish WITHOUT ever having warned, because C
cannot portably deprecate a typedef; the changelog has to say so), the service reply verbs, the
QoS `*_raw()` / `*_ms()` accessors, `BoardConfig::zenoh_locator` and
`ThreadxConfig::zenoh_locator` (two, not one — the row names only the first),
and the five `with_zenoh_locator()` builders.

Retirement is the only irreversible step for an out-of-tree consumer, so it
happens once, deliberately, with a changelog entry — not opportunistically as
each rename lands. Two things to carry into it:

* **C cannot portably deprecate a `typedef`** (MSVC rejects the attribute,
  `[[deprecated]]` is C23), so the four renamed C TYPES have plain aliases that
  compile without warning. They will disappear silently for anyone who never
  rebuilt. Say so in the changelog.
* `ThreadxConfig::zenoh_locator`'s alias is WEAKER than it looks: a defaulted
  trait method's OVERRIDE is no longer consulted, so an out-of-tree board that
  overrode it is silently ignored rather than warned.

## Acceptance

* W1: `scripts/api-parity.py --self-test` green; the report above reproduces on
  a host with ROS Humble, an `ros2/rclc` checkout and an `rclrs` checkout.
  **Met.**
* W2: `--check` green and wired into `just check`.
* W3: a phase-209 port template compiles without the compat header supplying a
  name rclcpp already has.
* W4: every C `differs` row carries a verdict; RFC-0036 gains the ones that are
  divergences.
* W5: an RFC recording the rclrs target version and the facade's export policy.
* W6: the three verb decisions recorded above, with RFC-0036 carrying the
  `Expected<T>` rule so it is stated once rather than per site.
* W7: `--show same | grep 'rename same'` returns nothing (step 1 complete), and
  the deprecated set is retired in a single commit with a changelog entry.

## Notes for whoever picks this up

* Re-derive before believing a stale count: `scripts/api-parity.py --refresh
  --rclc <checkout> --rclrs <crate dir>`. The recorded surfaces carry their
  provenance so a mismatch with your ROS install is visible.
* The 766 / 385 / 327 `theirs-only` counts are the PUBLIC surface we do not
  have. They are still upper bounds — a row can be a legitimate `declined` —
  but they no longer contain generated messages or library plumbing.
* `--include-internal` restores the unfiltered comparison. Use it when checking
  whether the public-surface rule dropped something it should not have; do not
  quote its numbers.
* `--suggest-renames` pairs unmatched names by SIMILARITY. It is the fastest
  route into W2 (it finds `send_reply` -> `send_response`, `try_recv_request`
  -> `take_request`, `try_recv` -> `take`, `make_publisher` ->
  `create_publisher`, `is_cancelled` -> `is_canceled`) and it also pairs
  `Timer` with `Time`. Suggestions never satisfy `--check`; a human confirms
  each pair and writes the ledger row.
* `--show all` prints matching rows too, which is the fastest way to check
  whether a name you are about to add already correlates.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0783](../issues/0783-rust-facade-hides-the-error-vocabulary.md) | `RclReturnCode` exists and is unreachable, and RFC-0036 documents a Rust surface we do not export |
| [#0784](../issues/0784-nros-facade-node-surface-is-three-audiences.md) | `nros::` publishes three different audiences under one namespace — parity is unreadable until the surface is separable |
| [#0829](../issues/archived/0829-two-system-default-qos-presets-disagree-on-depth.md) | two `SYSTEM_DEFAULT` QoS presets ship under one meaning and disagree on it |


## Adopted issues (2026-09-04) — four parity defects with no phase

Four open issues describe the gap between what this phase claims and what a
ported node gets. They had no home; they belong here because each is a place the
parity CLAIM and the parity MEASUREMENT disagree.

* **[#1008](../issues/1008-wait-for-service-never-waits.md)** — `wait_for_service`
  returns `Ok(true)` immediately on every real backend. The API is present and
  the behaviour is not, which is the exact failure mode a parity ledger exists to
  catch and did not.
* **[#1019](../issues/1019-rclcpp-compat-log-macros-discard-output.md)** — every
  `RCLCPP_*` log call in a ported C++ node is discarded on embedded targets. A
  port that compiles and says nothing.
* **[#1020](../issues/1020-parity-cpp-lane-cannot-see-the-compat-shim.md)** — the
  C++ parity lane measures the NATIVE API against rclcpp and cannot see the
  compat shim, so the number it reports is about the wrong surface.
* **[#1012](../issues/1012-ledger-prose-cites-renamed-symbols.md)** — 15 ledger
  rows describe symbols a rename retired. The ledger is the artifact this phase
  reasons from, so prose that names dead symbols is a measurement error, not a
  typo.

#1020 and #1012 are the same problem as #1008 one level up: the instrument is
what is wrong, so every number taken with it needs re-reading before it is used
to close anything.
