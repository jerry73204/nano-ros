# RFC-0089 — ROS 2 API adoption, and the compile-or-conform rule

**Status:** Draft (2026-09-04)
**Amends / refines:** RFC-0036 (divergence catalog) — adds the DISPOSITION half:
a divergence must say what a porting user gets, not only why we differ.
RFC-0018 (C++ surface mirroring rclcpp) — states the condition under which the
mirror may take upstream's names.
**Motivated by:** phase-379's measurement of the three user APIs against rclc /
rclcpp / rclrs, and the intent to retire `rclcpp_compat.hpp` by making our own
names ROS 2's.
**Implements-tracked-by:** phase-417-ros2-api-adoption
**Governed by:** RFC-0019 / RFC-0020 (thin-wrapper discipline) — the Rust API
is the implementation source of truth; C and C++ delegate. This RFC does not
relax that, and §"Who implements an adopted name" states what it means for
adoption.
**Related:** RFC-0002 (one executor per RTOS task), RFC-0021 (blocking API
rules), RFC-0035 (RMW seam), RFC-0044 (component model); issues 1012, 1019, 1020.

## The intent

nano-ros should be a near drop-in replacement for ROS 2 client code. The
end state is that our user API carries ROS 2's own names — `rclcpp::` for C++,
`rcl_`/`rclc_` for C, `rclrs::` for Rust — and `rclcpp_compat.hpp` is deleted,
because a shim exists only to bridge two spellings and there would be one.

This RFC is about what has to be true BEFORE that rename, and the rule that
decides, item by item, whether we may take a name at all.

## The hazard, stated once

Phase-379 W6 found `PollingSubscription::take()`. It had rclcpp's name, rclcpp's
signature, and the opposite contract: it drained to newest and returned true if a
value had ever arrived, so the idiomatic

```cpp
while (sub.take(msg)) { … }
```

terminates under rclcpp and spun forever here. **No compile error.** It was
deleted, and the deletion is recorded with the trap in the header.

That was one method found by accident. Adopting ROS 2's names across the whole
user API generalises exactly that hazard to every item whose semantics differ —
and 62 % of the rclcpp surface we do not cover is declined, meaning we do not
have those semantics.

A rename is therefore not a cosmetic change. It is the act of promising, for
every name taken, that the contract behind it holds.

And this is not hypothetical: the compat shim already ships two of them.
`ParametersQoS()` returns `QoS(10)` where `rmw_qos_profile_parameters` is
`KEEP_LAST, 1000` — a hundred-fold history difference under a name that claims
to be the ROS 2 profile, costing samples under load with nothing to read. Ten
`NodeOptions` setters store their argument in a private field that nothing
reads, and return `*this` so the idiomatic chained call compiles and configures
nothing; the header calls them "intentionally inert today" (`:125`), which is
the deferral this RFC is written to end. **A rename would inherit both, not
introduce them.**

## The rule

> **An upstream name may be adopted only if its observable contract is the same,
> or strictly weaker in a documented, non-inverting way. A contract that
> inverts, or that silently drops data or configuration, must fail to COMPILE.**
>
> Never compile and differ.

The rule is checkable against cases we already have, which is why it is stated
this way rather than as "match ROS 2 where possible":

| case | rule says | today |
| --- | --- | --- |
| `PollingSubscription::take()` — loop inverts | must not exist under that name | deleted (W6) |
| `RCLCPP_INFO_STREAM` — message discarded | must fail to compile | compiles, logs `""` (issue 1019) |
| `rclcpp::init(argc, argv)` — `--ros-args` dropped | must fail to compile, or honour them | compiles, drops them silently |
| `create_wall_timer` — period accuracy is the spin cadence | may be adopted, envelope documented | adopted, envelope undocumented |
| `Publisher::get_publisher_handle` — reaches past the API | absent is correct | absent (RFC-0035) |
| `ParametersQoS()` — history depth 10, upstream's is 1000 | must not carry that name at that value | **shipping** (`rclcpp_compat.hpp:117`) |
| `NodeOptions::use_intra_process_comms(true)` — stored, never read | must fail to compile | **shipping**, chainable, inert (`:125`) |

Note what the rule does NOT say. It does not require the implementation to
match; it requires the *contract a caller can observe* to match. A timer whose
period is quantised to the spin cadence is weaker, not inverted, and a caller
who is told so can act on it. A `take()` whose loop never terminates is not
weaker — it is a different function wearing the same name.

## Four dispositions

Every upstream item is exactly one of:

- **ADOPT** — same name, same contract.
- **ADOPT-BOUNDED** — same name and contract, weaker within an envelope stated
  in the doc comment. The envelope is part of the API, not a footnote.
- **REFUSE-LOUD** — we cannot have the contract, but the name is common enough
  that a user will reach for it. The name EXISTS as a deleted overload or a
  `static_assert`, whose message names the constraint and the nano-ros
  alternative.
- **ABSENT** — the name does not exist. Correct for rclcpp internals a user
  program never names.

REFUSE-LOUD is the disposition this RFC exists to introduce. Absence produces
`no member named 'create_generic_subscription'`, which is honest and teaches
nothing. A deleted overload produces the migration at the point of failure:

```cpp
// static_assert message, not prose in a book nobody opens
"rclcpp::Node::create_generic_subscription resolves the type by NAME at "
"runtime, which needs a dynamic loader and a heap. nano-ros resolves types "
"at build time. Use create_subscription<T>() with the concrete type, or "
"take_serialized() if you genuinely want bytes."
```

That costs one line and is the highest-leverage documentation in the design.

## Where the refusal fires: the earliest point the defect is KNOWABLE

`rclcpp::init(argc, argv)` forced this and it generalises.

The rule says a contract that silently drops configuration must fail to compile.
Applied literally to `init`, that means a `static_assert` on the two-argument
overload — which rejects **every** caller, including the overwhelmingly common
embedded one that forwards `main`'s argv and has no ROS arguments at all. It
also makes upstream's own tutorial `main` unportable, which collides head-on
with phase-417 stage 1's acceptance that the ported template be byte-identical
to upstream. Two parts of this design cannot both hold under the literal
reading.

The resolution is that the rule's real requirement is **loudness**, and compile
time is the earliest point loudness is *available* — not the only point it is
permitted:

> A refusal fires at the earliest point the defect is KNOWABLE. When the
> signature or the type carries it, that is compile time and a `static_assert`
> or `= delete` is correct. When only the VALUE carries it, that is the call,
> and a loud abort naming the unsupported input is correct. Silence remains
> forbidden at every point.

So `init(argc, argv)` compiles, scans for `--ros-args`, and aborts naming the
flag if it is present. A program with no ROS arguments is unaffected, because
nothing was dropped; a program that passes them dies immediately instead of
running for three hours on a remap that was never applied.

**The predicate must be separately checkable, or this becomes a check nothing
runs.** An abort inlined into `init` can only be observed by a process that then
dies, which is the shape of a gate that quietly stops working. Ours is
`constexpr` (`rclcpp_compat.hpp`, `detail::argv_has_ros_args`), so its cases are
`static_assert`ed in the ordinary compile lane — including the two mutations
that would silently stop it refusing: a null `argv` entry ending the scan early,
and a prefix match treating `--ros-args-extra` as the flag. Both were
mutation-tested and both fail their own named assertion.

This is a refinement of the rule, not an exemption from it. A disposition of
ADOPT-BOUNDED with a runtime refusal is still a promise that the contract never
differs silently; it just makes the promise where the information exists.

## Who implements an adopted name

RFC-0019 is Stable and says it plainly: **the Rust API is the source of truth;
C and C++ are thin shims that delegate.** Adopting ROS 2's names must not become
a licence to grow a second implementation behind them — a compat surface is
exactly where that pressure appears, because the fastest way to make a ported
line compile is often to write the logic in the wrapper.

The distinction that decides the hard cases:

> **Ergonomics may live in the wrapper. Behaviour may not.** A second
> *spelling* is free; a second *code path that can produce a different answer*
> is a violation.

Not a second implementation, and therefore fine in C/C++ alone:

* type aliases and nested typedefs (`Publisher<T>::SharedPtr`)
* inline forwarders and convenience overloads that convert and delegate
* `= delete` / `static_assert` diagnostics — a REFUSE-LOUD name emits no code
* doc comments, including the ADOPT-BOUNDED envelopes
* container/string conversions that copy and call through (`FixedString` ↔
  `std::string`)

A second implementation, and therefore Rust-side work first — these are
RFC-0020's five violation classes, restated as they show up in adoption:

* state machines (goal lifecycle, request/reply tracking)
* retry / timeout / polling loops that spin the executor from inside the wrapper
* CDR serialisation or deserialisation
* topic / service / action name construction, and remap resolution
* direct transport calls

### The consequence for planning

**A C or C++ gap is frequently a Rust gap wearing a wrapper's clothes**, and the
two cost very different amounts. So every work item carries the question *which
layer implements this?* and the order is Rust → FFI slot → C → C++.

The cross-language audit makes this cheap to exploit: of the 37 capabilities
where our own three surfaces disagree, roughly half are ones **Rust already has
and one or both wrappers never exposed** — named loggers and per-logger levels,
parameter type queries, undeclare, descriptors and ranges, QoS equality, GID
attribution, name resolution. Under SSoT those are the cheapest work in the
whole roadmap: the behaviour exists and is tested, and the wrapper is a
forwarder.

The remainder genuinely need Rust to grow first, and planning them as C/C++
tasks would have mis-costed them by an order of magnitude:

* typed C subscription delivery needs an `add_subscription` variant that carries
  caller-owned message storage through the FFI (stage 5);
* `rclcpp::init(argc, argv)` honouring `--ros-args` is remap resolution — name
  construction, violation class 4 — so the parser belongs beside
  `nros::resolve_name`, not in the shim;
* `rclcpp::Rate` is the sharpest case: a loop in the wrapper that spins the
  executor is violation class 2 *and* the thing RFC-0021 forbids. It is
  admissible only as a forwarder onto an executor-driving entry point that
  already exists in Rust — which is why it is planned that way rather than as
  the obvious fifteen-line C++ class.

### When a second path is warranted

Only when the wrapper must satisfy a language contract the Rust side cannot
express, and then it is recorded rather than assumed. The bar is the same one
RFC-0020 audits against, and a new second path needs: what language contract
forces it, why no FFI slot can carry it, and what keeps the two from drifting.
`nros::Expected<T>` is the model — it exists because RFC-0018 forbids
exceptions and Rust's `Result` cannot cross `extern "C"`, and it holds no state
of its own.

## Naming: replace, with alias as the migration step

The end state is the one the rename intends: **our API carries ROS 2's names.**
`nros::` stops being the spelling a user writes.

Getting there is a two-step, and the intermediate step is what makes it safe
rather than a flag day:

1. **Alias.** The ROS 2 spelling becomes a first-class name declared in the API
   headers, `nros::` remains and both work. `rclcpp_compat.hpp` disappears as a
   separate file because its content has moved into the headers it was shimming.
2. **Replace.** `nros::` is deprecated, then removed, and in-tree call sites
   (110 C++ and 75 C example files) migrate. This is the irreversible step for
   out-of-tree consumers and happens once, deliberately, with a changelog entry
   — the same discipline phase-379 W7 step 4 applies to the deprecated-alias
   batch.

### On ODR

An earlier draft of this RFC treated the ODR collision — nano-ros defining
`rclcpp::Node` in a build that also links real rclcpp — as a primary argument
for keeping `nros::` permanently. **That is overstated: nano-ros and ROS 2 are
not linked into one binary.** They interoperate over the wire, and the host-side
tooling that talks to a nano-ros image is a separate process.

It is kept as a guard rather than as an argument, because the guard is one line
and the failure it prevents is silent:

```cpp
#ifdef RCLCPP__RCLCPP_HPP_
#error "nano-ros and ROS 2's rclcpp are both in this translation unit. \
        They define the same names differently. Link one."
#endif
```

Cheap insurance against a configuration nobody intends, not a reason to keep two
vocabularies forever.

### What survives the correction

The ordering does, and it never depended on ODR:

> **The rename is cheap and cosmetic; the compatibility is the work.** Adopting
> ROS 2's names does not by itself make one more ported line compile. What makes
> lines compile is nested `SharedPtr` types, `create_service`, parameters on the
> node, `now()`. Those are needed whatever the namespace is called.

So the rename lands last. Doing it first would relabel the gaps without closing
one, and would spend the property that currently makes a mismatch visible —
our names differ, so a shape that differs is *visible* — while the mismatches
are still there. The two live inversions above (`ParametersQoS`, the inert
`NodeOptions` setters) are exactly what "inherited by a rename" looks like.

There WAS one mismatch the rename made strictly worse, and it is fixed
(corrected 2026-09-05). The shim `Node` pumped its own callbacks while
`nros::Node` was arena-driven, so a file mixing them got no callbacks and no
error, and only the differing NAMES made that visible at all. `pump()` and
`rclcpp_compat.hpp` were both deleted by this phase — every entity a
`rclcpp::Node` creates is now arena-registered through the same call the native
path makes, and `nros.hpp` says so at the call site. This paragraph outlived the
fix by a month and was still being cited as the reason the node-type merge was
not scheduled, which is the cost of stating a prerequisite in two documents and
retiring it in neither.

## Settled: C takes rcl's spellings (2026-09-04)

The question the disposition pass could not answer, because it is a decision and
not a reading: does the C API keep `nros_publisher_get_zero_initialized`
(module-first, our convention) or take `rcl_get_zero_initialized_publisher`
(rcl's free-function shape)?

**It takes rcl's.** The goal is drop-in replacement, and a ported file's line is
rcl's line. That resolves a real split — the same class landed `adopt` in four
ledger shards and `refuse-loud` in one, which was never agent disagreement but
an unmade decision surfacing five times.

Consequences, in the order they bite:

* **Every row declined on SPELLING alone flips to `adopt`.** Those declines
  argued from our naming convention, and RFC-0036 already forbids recording a
  preference as a divergence. They were the largest coherent group in the C
  ledger with no platform content.
* **New C entry points land under `nros_` and reach the user as `rcl_`.** The
  alias comes from `<nros/rcl_compat.h>`, not from renaming as we go: a
  half-renamed C surface is worse than either end state, and RFC-0089's
  migration is alias-then-replace precisely so the two steps are separable.
* **`nros_ret_t` is the sharpest case and is NOT a rename.** Its doc claims
  compatibility with `rcl_ret_t` and only `OK` agrees — ours are −1/−2/−3/−7,
  rcl's are 1/2/11/101. The compat header MAPS the constants; renumbering ours
  would silently flip the meaning of every stored return code across three FFI
  seams. This is the one place where taking rcl's spelling must not mean taking
  rcl's values.

## Settled: each language follows ITS OWN upstream (2026-09-04)

Where the three upstreams disagree with each other, we follow each language's
own. The worked case is logging:

```
rclrs    log_info!(logger, "…")
rclcpp   RCLCPP_INFO(logger, "…")
rcl/rcutils  RCUTILS_LOG_INFO_NAMED(name, "…")
```

There is no single "ROS 2 spelling" to match, so "respect the official API" does
not resolve it on its own. The tie-break is what a porting user actually reads:
**one client library, not three.** A Rust node ported from rclrs is read beside
rclrs; matching rclcpp there would be matching a library that user never opens.

**This deliberately gives up part of stage 4's convergence, and that is the
cost, stated rather than hidden.** Stage 4 spent real effort making our three
surfaces agree — and that was always INSTRUMENTAL to drop-in, never an end in
itself. Where agreeing with each other and agreeing with upstream pull apart,
upstream wins, because upstream is what a ported file was written against.

The rule survives the exception: our languages must still agree about
CAPABILITY and CONTRACT. What may differ is the spelling, and only where the
upstreams differ first. A capability present in one of our languages and absent
in another is still the defect stage 4 exists to remove.

## Settled: the typesupport parameter keeps OUR type (2026-09-04)

`rcl_publisher_init` takes `const rosidl_message_type_support_t *`; ours takes
`const nros_message_type_t *` in the same position. We keep ours.

Not a preference — a rosidl typesupport's MEMBERS are its contract
(`typesupport_identifier`, `data`, and the `func` dispatcher that resolves the
implementation at runtime), against our flat `type_name` / `type_hash` /
`serialized_size_max`. Adopting the name would claim a dispatcher we do not
have, and the "compiles and differs" that follows is exactly what this RFC
forbids.

It costs a ported call site nothing today, because the argument comes from our
codegen either way — `ROSIDL_GET_MSG_TYPE_SUPPORT` does not exist here. What it
costs is one type NAME visible in a signature after the compat layers are gone,
and that is the honest price of not faking a structure.

## End state: no compat layer survives

Confirmed 2026-09-04. The compat headers are SCAFFOLDING with a demolition
date, not a permanent surface:

* `rclcpp_compat.hpp` and `<nros/rcl_compat.h>` exist only to bridge two
  spellings of one thing. When there is one spelling they have nothing to
  bridge, so they do not have to be argued away — **they dissolve by
  construction**, which is the property that keeps them from rotting into
  permanent debt.
* `cmake/compat/` likewise: it shadows `<rclcpp/rclcpp.hpp>` and stubs
  `find_package(rclcpp)` because our headers live elsewhere under other names.

The acceptance test for the end state is therefore sharp and mechanical: **the
ported templates must build with every compat layer deleted.** Not "still
compile with the shim present" — deleted.

### The one thing that does NOT dissolve, and why that is fine

`rcl_compat.h` does two jobs, and only one is a spelling. The other is the
`RCL_RET_*` value mapping, and RFC-0089 forbids renumbering ours to match rcl's
— that would silently flip the meaning of every stored return code across three
FFI seams.

So `RCL_RET_TIMEOUT` does not go away; it becomes the NAME of our constant, with
our value. A ported `if (ret == RCL_RET_TIMEOUT)` is correct. A program that
hardcodes `if (ret == 2)` is not, and never was. That is a defensible line: we
adopt rcl's vocabulary, not its numbering, and the vocabulary is what source
compatibility is made of.

## Taking rclc's argument ORDER forces a decision on RFC-0041

Measured over the seven entry points: only one is a pure permutation
(`node_init`). Three more — `subscription_init`, `service_init`, `timer_init` —
differ because **ours carry `callback` and `context` that rclc's do not**:

```
rclc_subscription_init_default(sub, node, type_support, topic_name)         [4]
nros_subscription_init        (sub, node, type_info, topic_name, cb, ctx)   [6]
```

That is RFC-0041 (Stable): the callback binds to the ENTITY at creation, where
rclc binds it at `rclc_executor_add_subscription`. So "sort the arguments to
match" is not reachable by reordering — those two arguments have to go
somewhere else, or stay.

### CORRECTED — RFC-0041 does not decide this, and the rule cited instead does not exist

I first wrote that taking rclc's order requires reversing RFC-0041, which is
Stable, and that this was the last blocker for deleting the C compat header.
**Both halves are wrong**, and checking took one grep:

* **RFC-0041 says nothing about a binding site.** Its normative content is the
  DISPATCH MODEL — every callback-capable entity is callback-based by default,
  the executor pumps once per `spin_once`, and an entity must be arena-registered
  to be dispatched at all. Where the callback is *passed* — `*_init` or
  `executor_add_*` — appears nowhere in it.
* **`executor-owns-no-entity-storage`, cited by name in ten ledger rows as the
  reason, is defined nowhere.** Zero occurrences in `docs/design/`. It reads as
  a settled principle and is a phrase the ledger invented for itself.

So the binding site is an implementation habit that was retroactively attributed
to an RFC that does not mandate it. That is issue 1022's class — prose citing a
source that does not support it — in the rows that were about to justify keeping
a compat layer forever.

**What IS real**, and is the only constraint found: `c:timer_exchange_callback`
gives a concrete reason for the RUNTIME case — "the executor's static dispatch
table cannot be rewritten while it is being walked". That forbids *swapping* a
callback on a live entity. It says nothing about which call first supplies one.

**So the decision is smaller and already unblocked.** Moving the callback from
`*_init` to `executor_add_*` needs no RFC amendment; it needs the same shape
W5.a shipped and proved this week
(`nros_executor_add_subscription_typed(exec, sub, msg, cb, ctx, invocation)` —
rclc's arguments in rclc's order). What remains is making it the only shape and
migrating in-tree callers.

**What still needs writing down** is the reverse: the ten rows asserting a rule
that does not exist have to be corrected, and if the habit has a real
justification, it belongs in an RFC rather than in ledger prose that cites
itself.

## Argument ORDER follows the same decision, and is mostly not a separate task

Settled 2026-09-04 with the spellings: where we and rclc/rclcpp name the same
function, our parameters take THEIR order. A ported line has to compile
unchanged, and an argument list is as much of the line as the name.

Measured before committing to it, over the seven entry points the compat work
touches. Only ONE is a reordering:

```
rclc_node_init_default(node, name, namespace_, support)
nros_node_init        (node, support, name, namespace_)     <-- permutation
```

The other six differ for reasons a reorder cannot fix, and reading them is what
makes the task tractable:

| pair | difference | what closes it |
| --- | --- | --- |
| `publisher_init`, `client_init` | `type_support` vs `type_info` — a TYPE difference in the same position | a decision about the typesupport handle, not order |
| `subscription_init`, `service_init` | ours carries `+2` (`callback`, `context`) | RFC-0041 binds the callback to the ENTITY at creation; changing it is a design reversal, not a rename |
| `timer_init` | ours carries `+1` (`context`); also `timeout_ns` vs `period_ns` | same RFC-0041 question, plus a naming one |
| `executor_add_subscription` | rclc carries `+2` (`msg`, `callback`) | W5.a's typed delivery — it converges when that lands |

So argument order is **downstream of the shape decisions**, not parallel to
them. Reordering the one true permutation is a day's work; the other six
converge or do not depending on RFC-0041 and W5.a, and forcing an order onto
lists of different lengths would produce a signature that matches upstream in
neither.

### The hazard, CORRECTED — a distinguishable type is not enough in C

The first version of this section said `node_init` was safe to reorder because
the moved parameter is a `struct nros_support_t *` crossing two `const char *`,
so a stale caller "fails to compile". **That is false for C**, and it was
falsified by a mutation test rather than by review:

```
C   : warning: passing argument 1 of 'f' from incompatible pointer type
      ...even under -Wall -Wextra.
C++ : error: cannot convert 'support_t*' to 'const char*'
```

C permits an incompatible pointer argument with a diagnostic that is a WARNING
by default. So a reorder in the C API is silent-by-default for exactly the
callers it must not be silent for: out-of-tree consumers, who do not build with
our flags.

**The rule, restated:**

* A reorder is safe only where the build treats an incompatible pointer as an
  ERROR. Ours does; a user's does not, and we do not control that.
* Therefore a C reorder must be accompanied by a RENAME, so a stale call fails
  on the identifier — which C does diagnose fatally — rather than on the
  argument type, which it does not.
* Where the moved parameter's type is INDISTINGUISHABLE from its neighbours
  (two `const char *`), it is silent in C++ too, and the rename is not optional
  in either language.

This is why the compat header's reordering forwarder is a `static inline` that
names each parameter and never a macro, and why its probe pins the reorder with
`#pragma GCC diagnostic error "-Wincompatible-pointer-types"` scoped to that TU:
the guard cannot be advisory when the reorder is the whole reason the forwarder
exists. Without the pragma the guarding mutation PASSED.

## What "mostly full compat" can honestly mean

It cannot mean the whole rclcpp surface. Four upstream idioms account for most
of what we decline, and three of them are load-bearing for ROS 2's design and
incompatible with ours:

| idiom | rows | can we? |
| --- | ---: | --- |
| executor as wait-set assembler over polymorphic `Waitable`s | ~128 | no — dynamic membership needs an allocator (RFC-0002) |
| parameters as a distributed service with owned-storage values | ~56 | partly — server side yes, client side is a product choice |
| graph and middleware queryable at runtime | ~55 | **partly, and the table above was wrong.** `nros::Executor` already ships `get_node_names`, `count_publishers`/`count_subscribers`, the four `*_names_and_types_by_node` forms and `get_{publishers,subscriptions}_info_by_topic` (`executor.hpp:205-301`) over shipped FFI. 18 rows are pure forwarders from `Executor` to `Node`. What is genuinely impossible is a GRANTED QoS read-back and graph *events* |
| types erased and resolved at runtime | ~45 | no — no dynamic loader |

The honest claim is therefore about PROGRAMS, not surface: *the shapes a ROS 2
node is actually written in compile and behave, and everything else fails
loudly.* That is measurable — the in-tree ported templates are the measurement —
and it is what the roadmap targets.

## The split, measured

All 717 uncovered rclcpp items, classified against the rule:

| disposition | rows | distinct sites | cost |
| --- | ---: | ---: | --- |
| ADOPT | 117 | ~45 | ~820 lines |
| ADOPT-BOUNDED | 123 | ~50 | ~1150 lines |
| REFUSE-LOUD | 69 | **17** | ~120 lines, diagnostics only |
| ABSENT | 408 | — | 0 |

ABSENT is 57 % and is not a concession — 83 rows are the wait-set/`Waitable`
protocol, 42 runtime type erasure, 35 parameter-client internals, 18
`get_node_*_interface`, 21 `make_shared` on types a user never constructs. A
ported user program names none of them.

The shape worth noticing: **69 refusals collapse into 17 diagnostics**, because
a refusal is per-CONCEPT, not per-symbol — the sixteen inert `NodeOptions`
setters share one message. That is the whole loudness pass at about 120 lines,
which is why it can gate the rename without gating the schedule.

## A refusal is a DECLARATION, so refusing improves the numbers

This is the stage-0 finding one level deeper, and it is worse.

REFUSE-LOUD is implemented as a `= delete` or a `static_assert` inside a
template. Both are DECLARATIONS. The extractor sees a declaration, so the symbol
appears on OUR side and the row moves off `theirs-only`:

```
cpp:NodeOptions::enable_logger_service   declined   ours-only
```

That row is `ours-only` because we refused it. Before the refusal it was a name
only rclcpp had; after, it is a name we "have". **The more of the surface we
refuse, the better the correlation looks**, and nothing in the bucket says why.

So a correlation count cannot distinguish:

* *we have this* — `adopt`;
* *we declared it in order to refuse it* — `refuse-loud`.

**Consequence, and it is a rule about how this campaign may report itself:** a
headline compatibility number must EXCLUDE `refuse-loud` rows. Counting them as
coverage counts refusals as features, which is the same error as counting
`ParametersQoS` as `same` while it differs hundredfold — arrived at from the
opposite direction.

It also settles what `disposition` is for. It is not commentary on a verdict; it
is the only field that separates a name we serve from a name we declared in
order to say no. The correlator cannot recover that, because at the level of
names and shapes there is nothing to recover.

## Four ways a row correlates `same` without our having implemented it

A collapse pass over rows that correlate `same` looked mechanical and was not.
Measured 2026-09-04, and each of these was hit:

1. **The gate reads a DIFFERENT surface than the headline bucket.** `--check`
   gates the NATIVE bucket; the shim surface is opt-in (`--check-ported`). Of 53
   rows reading `same`, only 16 were `same` natively — the other 37 are
   `theirs-only` without the shim and still gated. A blind collapse takes the
   gate red.
2. **A refusal correlates `same` by construction** — a `static_assert` template
   or an inert getter is a declarable name, and correlation compares names and
   shapes.
3. **A sentinel correlates `same` and is neither.** `rclcpp::Logger` in the shim
   has the right name and the right member and carries no information to the
   sink.
4. **UPSTREAM moving produces `same` with no change of ours at all.** The rclrs
   reference was re-pinned 0.5.1 → 0.7.0, and 0.7.0 ships actions. Nine rows
   saying "rclrs ships NO action API at all; the convergence point is whenever
   rclrs grows actions" became false without anyone touching our tree.

(4) is the one worth naming as a class: **the ledger is a join over two moving
surfaces, so it goes stale from THEIR side too.** Every other staleness rule in
this campaign — issues 1012, 1022 — assumes we moved. A `--refresh` that bumps
the recorded surface must be followed by a re-read of every row that argued from
its absence, and nothing enforces that today.

## `disposition` is not evidence, and W-M2 proved it

The disposition pass was PROSE-DRIVEN: it read each row's `why` and classified.
That is why it stamped `refuse-loud` on `cpp:Rate`, `Rate::sleep`,
`Rate::reset` and `WallRate` — rows whose prose argued from RFC-0021 — when
W2.d had already SHIPPED them (`rclcpp_compat.hpp:1327`), answering the
objection rather than overriding it: `sleep()` computes a deadline and makes one
call into `nros::spin(remaining_ms, poll_ms)`, so the executor keeps running.

So the field records an intent, and an intent can be wrong about the code. A
later pass must verify against the header, not against the field. Recorded here
because the failure is in a mechanism this RFC introduced, not in the rows.

## Dispositions apply to every upstream item, not only `declined` ones

The four values were measured over all 717 uncovered rclcpp items (117 / 123 /
69 / 408). `--require-disposition` gates only `declined` rows, and two
independent passes over 515 of them returned **zero `adopt-bounded`** — 0 of 307
and 0 of 208.

That is structural, not an omission. `adopt-bounded` means *we have this name,
with a weaker but non-inverting envelope*; `declined` means *we do not have the
contract*. The two cannot both hold, so on a declined row the disposition is a
three-way choice.

`adopt` and `adopt-bounded` live where the RFC's own examples live — `gap` and
`divergence` rows. `create_wall_timer`'s quantised period is a divergence, not a
decline. So:

* on `declined` rows the vocabulary is `adopt` (a verdict bug — the row should
  not be `declined`), `refuse-loud`, or `absent`;
* the gate's scope is narrower than the taxonomy's, deliberately, because
  declines are where a porting user is most likely to be surprised — but a
  reader must not take the zero for a missing population.

## `absent` is a FREQUENCY test, not an internals test

The first wording said `absent` is "correct for rclcpp internals a user program
never names". Both classification passes reported applying a different test, and
the different test is the right one: **would a real ported program name this?**

Two large `absent` populations are not internals by any reading — rcl/rclc
struct-lifecycle plumbing that is public but has nothing to act on
(`*_impl_t`, `*_options_fini`, `get_zero_initialized_*_options`), and whole
rclrs subsystems (the async executor machine, Workers, dynamic messages) that a
user CAN name but almost never does.

A corollary both passes derived independently, worth stating: **a member reached
only through a refused type is `absent`, because the refusal already fired at
the type.** Diagnosing it twice teaches nothing and puts a message where no call
site exists.

## Prerequisite: the measurement must include the shim

The C++ parity lane reads three translation units and filters to namespace
`nros`, so `rclcpp_compat.hpp` contributes zero rows (issue 1020). Every number
in this RFC about "how far we are" is therefore measured against the NATIVE API,
not against what ported code reaches. Fixing that is stage 0 of the roadmap:
without it, progress and noise are indistinguishable.

## The measurement can flatter, which is why `disposition` is not bookkeeping

Stage 0 admitted `rclcpp_compat.hpp` as a fourth translation unit, so the lane
now reports two surfaces: the NATIVE API's distance from rclcpp, and what a
ported file actually reaches. The ported surface is better by construction —
`same` goes 84 → 110, `theirs-only` 717 → 692.

**Two of those newly-`same` rows are the live inversions this RFC was written
about.** Measured 2026-09-04:

```
ParametersQoS                          state=same   surface=ported
NodeOptions::use_intra_process_comms   state=same   surface=ported
```

`ParametersQoS()` returns `QoS(10)` where upstream is `KEEP_LAST, 1000`, and
that `NodeOptions` setter stores its argument and is never read. Both now
correlate `same`, because correlation compares NAMES and SHAPES and neither
differs. The instrument cannot see the defect it is measuring.

So a row can be `same` by shape and `refuse-loud` by disposition at the same
time, and those are not in tension — they answer different questions. Without
the disposition, "the ported surface matches on 110 items" is a claim the
campaign cannot back, because two of the 110 are known to differ silently and
nothing in the row says so.

That is the argument for the field. It is not metadata about the ledger; it is
the only thing standing between a compatibility number and a false one. The
gate exists (`--require-disposition`) and is off until W-M2 populates the rows.

## Consequences for RFC-0036

RFC-0036 catalogues divergences and says a preference may not be recorded as one.
This RFC adds the missing half: a divergence must also declare its DISPOSITION —
whether the upstream name is adopted, bounded, refused loudly, or absent. A
`declined` verdict with no disposition does not say whether a ported program
gets a compile error or a surprise, which is the only thing a porting user needs
to know.


## Settled: the three questions the node-type merge was waiting on (2026-09-05)

The structural blocker was retired (see above); what remained were three
decisions, not readings. All three are now made.

### 1. `get_logger()` follows ROS 2 after the merge

Today the two spellings collide on an identical signature: `rclcpp::Node::
get_logger()` returns a `rclcpp::Logger` built from the hardcoded sentinel
`"nros.compat"`, while `nros::Node::get_logger()` returns the real
`nros_log::Logger` handle keyed on the node's own name. Same name, same arity,
different observable behaviour — case (3) of "four ways a row correlates `same`
falsely", which this RFC already names.

**The merged accessor takes ROS 2's behaviour: a logger named for the node.**
The sentinel was a placeholder for a shim that no longer exists, and a logger
that cannot tell you which node emitted a record is worse than the one it
replaced. This is a strict improvement in both directions, so it needs no
disposition beyond `adopt`.

### 2. Mirror ROS 2's node types — and the count that implies is ONE

The instruction was to mirror rclcpp and avoid duplicates. Measured, the mapping
that motivated the question is **inverted**, so recording it before acting:

| type | where it is actually used |
| --- | --- |
| `nros::Node` | everywhere, and MOSTLY IN WORKSPACES — 51 files under `workspaces/features`, 41 under `workspaces/cpp`, 30 under `workspaces/c`, 27 under `workspaces/mixed`. The RFC-0043 typed component takes it by reference: `Result Configure(::nros::Node&)`. |
| `nros::ComponentNode` | **five directories total** — one standalone POC (`native/cpp/component-node-poc`) and two workspace subnode packages. |

So it is not "`Node` for standalone, `ComponentNode` for workspaces". Both live
in workspaces; they differ by ROLE. `nros::Node` is a node you are handed;
`ComponentNode` is a base class you derive from, constructed from an
entry-supplied `NodeHandle`.

Upstream has no counterpart to that distinction. `rclcpp_components` composes
plain `rclcpp::Node`s — a ported component derives from `rclcpp::Node` and there
is no second node type to mirror. Mirroring therefore means **one node type**,
with `ComponentNode`'s two distinguishing features demoted from a type to
constructors on it:

* construction from an entry-supplied `NodeHandle` rather than the global
  executor — a constructor overload;
* RFC-0047's "one component, several named nodes" — genuinely ours-only,
  because an rclcpp component IS one node. It stays, as a documented divergence
  with a disposition, not as a second class.

This also closes the duplication `nros.hpp` already flags against itself
("KNOWN DUPLICATION… There should be ONE helper"): the parameter facade exists
twice, once in C++14 on `rclcpp::Node` and once in C++17 `if constexpr` on
`ComponentNode`. One node type is what makes one facade possible.

### 3. Parameters live in Rust; C and C++ are thin wrappers

RFC-0019/0020 already say the Rust API is the implementation SSoT and that
ergonomics may live in the wrapper while behaviour may not. A node-local C++
parameter store is behaviour living in the wrapper, and it has the visible
consequence that a parameter declared through `rclcpp::Node` does not appear in
`ros2 param list` — the store the parameter SERVICES read is the executor's, in
Rust.

**The merged node's parameters are the Rust store.** `rclcpp::Node`'s inline
`ParameterServer<NROS_RCLCPP_MAX_PARAMS>` member and `ComponentNode`'s facade
both become forwarders to it.

The gap this exposes is small and specific: the FFI has
`nros_cpp_declare_param` plus typed getters (integer / double / bool / string)
and **no setter at all** — `grep -c nros_cpp_set_param` is 0, and there is no
`set_parameter` on the Rust executor either. `set_parameter<T>` therefore has
nothing to forward to yet, so the merge carries one piece of Rust-side work
rather than a pure C++ refactor. That is the right shape: the capability is
missing where the SSoT is, and adding it in C++ would have been the second
implementation this rule exists to prevent.

Resolves the C++ half of issue 0793.


## The rclcpp node model, and what nano-ros actually needs from it (2026-09-05)

Decision 2 above says "mirror rclcpp, which means one node type". That is the
conclusion; this section is the reasoning, because the question it answers —
*we don't build standalone node binaries, so doesn't that force a node type of
our own?* — is a good one with a surprising answer.

### What rclcpp actually has

Four things that are easy to conflate:

1. **`rclcpp::Node`** — an ordinary C++ object. Creating one does not create a
   process, a thread, or a scheduler. It registers a node with the middleware
   and owns the entities you create through it.
2. **An executor** — a separate object that *drives* nodes. Nodes are added to
   it (`add_node`), and it dispatches their callbacks. `rclcpp::spin(node)` is
   sugar that constructs a `SingleThreadedExecutor`, adds the node, and spins.
3. **Composition** — several nodes in ONE process, on one executor. Upstream
   supports two ways to get there, and this is the crux:
   * *dynamic composition*: `rclcpp_components` `dlopen`s a shared library and
     instantiates a registered class into a running container process;
   * **manual composition**: the components are LINKED into one executable
     whose `main` constructs each of them and adds them to one executor.
4. **`rclcpp_lifecycle::LifecycleNode`** — a genuinely distinct type, with a
   state machine and its own interface set.

### The answer to "we link, we don't spawn"

**Manual composition is upstream's own answer to exactly our constraint, and it
uses a plain `rclcpp::Node`.** A component in that model is a class that derives
from (or holds) `rclcpp::Node`, with a constructor taking `NodeOptions`; the
`RCLCPP_COMPONENTS_REGISTER_NODE` macro is what makes it *additionally*
loadable, and omitting it costs nothing but that.

So the constraint that motivated a nano-ros-specific node type — one image, no
`dlopen`, no per-node binaries — is a constraint upstream already lives under
whenever anyone does manual composition. It does not require a new node type. It
requires the *component* shape, which is a class, not a node type.

That is why the mapping collapses:

| rclcpp | nano-ros |
| --- | --- |
| `rclcpp::Node` | the one node type |
| manual composition (`main` constructs components, adds to one executor) | **exactly our model** — the generated entry constructs each component and the executor drives them |
| `RCLCPP_COMPONENTS_REGISTER_NODE` + `dlopen` | absent, deliberately: no dynamic loading on firmware |
| `NodeOptions` | `NodeOptions` |
| `LifecycleNode` | our lifecycle set, a separate type as upstream has it |

`nros::ComponentNode` was invented for the row that turns out not to need a new
type. What it really carries is two things, and neither is a node kind:

* **an entry-supplied `NodeHandle`** instead of the global executor. Upstream's
  equivalent is that a manually-composed node is added to an executor the `main`
  owns. This is a CONSTRUCTOR, and its nearest upstream spelling is passing
  `NodeOptions` and letting the caller do `executor.add_node(...)`.
* **RFC-0047's one-component-several-named-nodes.** This one IS ours-only: an
  rclcpp component is one node, and a process wanting two makes two objects.
  Ours exists because a fixed-arena image benefits from one object owning
  several node identities. It stays — as a documented divergence with a
  disposition, on the single node type, not as a second class.

### What this means for the merge

One node type, three constructors: the global-executor one (`rclcpp::Node`'s,
what a ported file writes), the handle-taking one (what a generated entry
writes), and the several-identities one (ours-only, RFC-0047). The RFC-0043
typed component keeps taking a node by reference, which is unchanged — it was
never a node type, only a `configure(Node&)` convention.

The thing to NOT do is preserve two types because they have two construction
paths. `rclcpp::Node` already has several constructors and remains one type;
construction is not identity.

## Parameters: feature-complete, Rust-side SSoT, and `ros2 param list` must work

Decision 3 says the store lives in Rust. Stating what "feature complete" costs,
because the gap is larger than the missing setter.

**What exists.** The executor owns a `nros_params::ParameterTable` and
registers all six ROS 2 parameter services — `GetParameters`, `SetParameters`,
`SetParametersAtomically`, `ListParameters`, `DescribeParameters`,
`GetParameterTypes` (`executor/spin.rs:7182`). Service-wise that is the
complete upstream set.

**The gap is not the service list; it is the KEYING.** Those services are
registered under ONE FQN, built from the executor's own `node_name` and
`namespace`. The store is likewise executor-global. But an image composes
several nodes onto one executor — that is the whole point of the model above.
So today:

* `ros2 param list` shows the executor's node, not the image's nodes;
* two nodes declaring the same parameter name collide in one flat table;
* a parameter declared through a C++ node facade is invisible to the services
  entirely, because the facade's store is a different store (issue 0793).

Upstream's model is one store and one set of six services PER NODE. Matching it
means:

1. **Key the table by node**, not by executor.
2. **Register the six services per node identity**, under each node's FQN, so
   `ros2 param list` enumerates the image's nodes as a ROS 2 user expects. Note
   the cost this lands on: a service server IS a zenoh queryable, six per node,
   against `ZPICO_MAX_QUERYABLES` — the CLAUDE.md pitfall about
   `[param_services]` claiming six slots becomes six PER NODE, which is a
   sizing decision, not an oversight to discover at runtime.
3. **Add the missing setter.** `grep -c nros_cpp_set_param` is 0 and there is no
   `set_parameter` on the executor either, so `SetParameters` currently has a
   service without a store-side writer for the wrapper path.
4. **Delete the C++ stores**, both of them, once 1–3 land. Not before: deleting
   a store that has no replacement is how a capability disappears quietly.

That ordering matters. Steps 1–3 are Rust-side, and step 4 is what makes the
C/C++ side a thin wrapper rather than a second implementation — which is
RFC-0019/0020's rule, and the reason this is a Rust job rather than a C++ one.


## The proposed node shape (2026-09-05)

Grounded in the three shapes that actually exist in the tree, not in the two the
merge question implied.

### The three working shapes today

**A — standalone `main`** (`examples/native/cpp/talker`, and every
`examples/<plat>/<lang>/` leaf):

```cpp
nros::init();
nros::Node node;                        // caller-owned storage
nros::create_node(node, "talker");      // out-ref, returns Result
node.create_publisher(pub, "/chatter"); // out-ref, returns Result
while (running) nros::spin_once(100);
```

**B — workspace typed component** (RFC-0043; the dominant workspace shape —
41 files in `workspaces/cpp` alone):

```cpp
class Talker {
    Publisher<Int32> pub_;   // inline storage, no allocator
    Timer timer_;
    void on_tick();
  public:
    Result configure(nros::Node& node);   // node passed BY REFERENCE
};
```

**C — `ComponentNode`** (RFC-0044/0047; five directories):

```cpp
class SubNode : public nros::ComponentNode {
    SubNode(NodeHandle h) : ComponentNode(h, "sub_node") {}
};
```

And upstream's component, for comparison:

```cpp
class Talker : public rclcpp::Node {
    explicit Talker(const NodeOptions& o) : Node("talker", o) {
        pub_ = create_publisher<M>("chatter", 10);           // returns shared_ptr
        timer_ = create_wall_timer(1s, [this]{ on_tick(); });
    }
};
```

C is upstream's shape with the handle swapped for the executor. **B has no
upstream counterpart at all** — and B is the one that carries our hardest
constraint, because it needs neither an allocator nor derivation nor a vtable.

### The proposal

**One node type. Three constructors. Two entity-creation shapes. B unchanged.**

```cpp
namespace nros { class Node; }
namespace rclcpp { using Node = ::nros::Node; }   // unconditional alias
```

*Constructors* — because construction is not identity, and `rclcpp::Node`
already has several:

1. `Node()` + `nros::create_node(node, name, ns)` — shape A, out-ref, no
   allocator, works on every target. Stays exactly as it is.
2. `Node(const std::string& name, const NodeOptions& = {})` — upstream's, for a
   ported file. Hosted-only, because `std::string` is.
3. `Node(NodeHandle handle, const char* name, const char* ns = nullptr)` —
   shape C's, promoted from a second class to a constructor on the one type.

*Entity creation* — both shapes on the one type, distinguished by argument
order, which is already how the overloads resolve:

| shape | signature | reach |
| --- | --- | --- |
| ours | `create_publisher(Publisher<M>& out, const char* topic, qos) -> Result` | every target |
| upstream | `create_publisher<M>(const std::string& topic, qos) -> shared_ptr<Publisher<M>>` | hosted |

The out-ref family is an **ours-only name for a capability upstream lacks**
(caller-owned storage), which is a permanent divergence under this RFC's own
rules, not a transitional one. The `shared_ptr` family allocates, and that is
inherent to the arena contract rather than to the spelling: the arena stores a
raw pointer and has no unregister, so anything handed back as a pointer needs a
second owner with node lifetime (`owned_entities_`).

*Derivation* — `class Talker : public rclcpp::Node` becomes available, which is
what a ported component writes. It requires the hosted constructor, so it is
hosted-only, and that is honest: on a freestanding target the answer is shape B.

*Shape B survives untouched.* `Result configure(nros::Node&)` takes the one node
type by reference. It was never a node type — only a convention — and after the
merge it is the same convention over the same type. **This is the shape to keep
recommending for firmware**, because it is the only one of the three that needs
no allocator and no vtable.

*`ComponentNode`* becomes `using ComponentNode = Node;` for one release, then
goes. Its two real contents are already accounted for: the handle constructor
(3 above) and RFC-0047's several-named-nodes, which stays as an ours-only
capability on the single type.

### What this costs, stated rather than implied

* **`get_logger()` changes behaviour** on the `rclcpp::` spelling: a node-named
  logger replaces the `"nros.compat"` sentinel. Decided above; strictly better,
  but it IS a behaviour change and belongs in the changelog.
* **A vtable appears** only if someone derives. Shapes A and B add none, so no
  embedded image pays for the hosted shape unless it uses it.
* **`sizeof(Node)` must not move with a capability probe** — the hosted members
  live behind an unconditional owner member, enforced by
  `check-cpp-capability-layout` and already the reason `timers_` is gone.
* **The 409 `nros::` call sites do not have to move.** The alias is
  unconditional and both spellings name one type, so migration becomes
  optional per file rather than a flag day.
