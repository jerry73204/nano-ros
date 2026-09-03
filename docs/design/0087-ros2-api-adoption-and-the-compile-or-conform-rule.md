# RFC-0087 — ROS 2 API adoption, and the compile-or-conform rule

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

There is also one mismatch the rename makes strictly worse, and it is not
fixable by a diagnostic: the shim `Node` pumps its own callbacks while
`nros::Node` is arena-driven, and a file that mixes them gets no callbacks and
no error. Today the two types have different NAMES, which is the only thing
making that visible. That is a structural prerequisite for step 2, not a
loudness item — see phase-417.

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

## Prerequisite: the measurement must include the shim

The C++ parity lane reads three translation units and filters to namespace
`nros`, so `rclcpp_compat.hpp` contributes zero rows (issue 1020). Every number
in this RFC about "how far we are" is therefore measured against the NATIVE API,
not against what ported code reaches. Fixing that is stage 0 of the roadmap:
without it, progress and noise are indistinguishable.

## Consequences for RFC-0036

RFC-0036 catalogues divergences and says a preference may not be recorded as one.
This RFC adds the missing half: a divergence must also declare its DISPOSITION —
whether the upstream name is adopted, bounded, refused loudly, or absent. A
`declined` verdict with no disposition does not say whether a ported program
gets a compile error or a surprise, which is the only thing a porting user needs
to know.
