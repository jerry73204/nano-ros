---
id: 964
title: "The C++ header states an ESTIMATED size for every type, including types that have no bound"
status: resolved
area: codegen
severity: medium
related: [0896, 0939, 0940, phase-403, phase-380]
---

# One type, two numbers, and the wrong one is the one in the header

## What was measured

`rosidl-codegen`'s C++ pack computes `SERIALIZED_SIZE_MAX` with
`compute_serialized_size_max`, which ESTIMATES: a flat 512 per nested message, a
flat default capacity per string, and it ALWAYS returns a value. It therefore
cannot report "unbounded".

Over 120 types in 12 stock ROS Humble packages:

* **81 of 120 have no derived bound, and the C++ header states a size for every
  one of them.**
* Of the 39 that are bounded, the estimate matched the derived bound **zero
  times**: 38 over, 1 under. `geometry_msgs/Twist` reads 1028 against a derived
  64.

On the island entry the same divergence, per type:

| type | C++ header | derived |
| --- | ---: | ---: |
| `autoware_control_msgs/Control` | 2052 | 114 |
| `nav_msgs/Odometry` | 1804 | unbounded, until a cap |
| `autoware_vehicle_msgs/SteeringReport` | 527 | 24 |
| `autoware_adapi_v1_msgs/OperationModeState` | 572 | 27 |

## Why it matters

phase-403 W6 exports the DERIVED bound. The C++ header keeps emitting the
ESTIMATE. The same type now carries two numbers, and the estimate is the one a
user reads, since it is the constant the header advertises and the one
`{Msg}_RX_MAX_SERIALIZED_SIZE` names.

It also violates phase-380's rule directly: a number nobody chose, substituted
where the honest answer is "no bound exists". That rule is why an unbounded type
is a build error at all, and this path quietly opts out of it.

Real cost, not hypothetical: the island's sizing was planned against the
estimate, so the receive buffer, arena and payload classes were all budgeted for
`Control` at 2052 bytes when it serializes to 114.

## Options

1. **Delete the estimator** and emit the derived bound, poisoning the constant
   for an unbounded type exactly as the C pack already does
   (`unbounded_token` + `unbounded_reason`). Behaviour change: a type that is
   unbounded stops having a size constant, which is the point.
2. **Keep both, renamed.** The estimate becomes something honestly named
   (`..._ESTIMATED_SIZE`) and the derived bound takes the load-bearing name.
   Cheaper, but leaves a number nobody chose in the header.
3. **Emit the derived bound only where one exists** and nothing otherwise,
   which is (1) without the poison token.

(1) matches what the C pack does today and what phase-380 requires. It is a
behaviour change with a blast radius across every C++ consumer, which is why it
is filed rather than done.

## Partly addressed 2026-09-01 (phase-408) — the derived number is now IN the header

Option 2, minus the rename. The C++ pack emits `Msg::TX_MAX_SERIALIZED_SIZE` /
`Msg::RX_MAX_SERIALIZED_SIZE` from the derived bound beside the estimate, and an
unbounded type states neither — it carries the poison templates instead, so
"this type has no bound" is now expressible in a C++ header, which it was not
when this issue was filed.

**One consumer moved: `nros::bind_subscription<M, C, Method>`.** It passed
`M::SERIALIZED_SIZE_MAX` as the subscription's `rx_buffer_hint`; it now passes
`nros::rx_size_bound<M>::value`, the derived RX bound.

**What is still open, stated precisely, because the survey behind this issue
undercounted the risk.** Thirty call sites still stack
`uint8_t buf[...::SERIALIZED_SIZE_MAX]` — 28 in `nros-cpp` headers, 2 in the
example workspaces — and they are NOT all transmit scratch:

* **RECEIVE buffers — the dangerous direction, an under-estimate TRUNCATES.**
  `Subscription<M>::take` / `take_validated` (`subscription.hpp:127,153`),
  `Client<Svc>::call`'s response buffer (`client.hpp:131`),
  `Future<T>`/`Stream<T>` (`future.hpp:46,166`, `stream.hpp:54`), the action
  client's result/feedback buffers (`action_client.hpp:147,238`,
  `polling_action_client.hpp:110,148`), the polling action server's goal buffer
  (`polling_action_server.hpp:82`), and `tick_ctx.hpp:125`.
* **TRANSMIT buffers — an over-estimate only wastes stack.** the request
  buffers in `client.hpp` / `service.hpp` / `tick_ctx.hpp`, the action goal and
  result/feedback serialize buffers, and two example workspaces.

`bind_subscription` was fixed here because it feeds a `rx_buffer_hint`, which is
a HINT — the change cannot break anyone. Retargeting the list above cannot make
that claim: the buffer IS the capacity, so switching an unbounded type from the
estimate to the derived bound turns a working (if arbitrarily-sized) call into a
compile error, and none of these has a `_sized` escape hatch the way
`bind_subscription_sized` does. That is the blast radius this issue was filed
for, and it is unchanged. `nros::rx_size_bound<M>` / `nros::tx_size_bound<M>`
are the spellings whoever takes it should use.

## The receive half is done (2026-09-01) — bounded types, and an escape hatch for the rest

The 30 sites re-counted from the tree: **28 in `nros-cpp` headers, 2 in the
example workspaces**, which is what the survey above says. Their SPLIT is not:

**13 are receive, not 12** — the survey put `service.hpp:81` under "transmit"
with the other request buffers. It is `Service<S>::try_recv_request`, and the
server deserializes out of it, so an under-estimate truncates a request exactly
as it truncates a message. Whoever reads a call site's DIRECTION off the word
in its name gets this one wrong; `Client<S>`'s request buffer really is
transmit, and they sit two files apart.

What landed:

* **`nros::rx_buffer_capacity<M>` / `nros::tx_buffer_capacity<M>`**
  (`size_bound.hpp`) — the derived bound where the type has one, the legacy
  estimate where it does not. It shares `detail::shape_of<M>()` with
  `rx_size_bound<M>`, so the two traits cannot disagree about which arm a type
  is in; that predicate reads the emitted CONSTANTS, because a `static_assert`
  failure is a hard error and never SFINAE, so the poison templates cannot be
  probed. `nros::has_derived_size_bound<M>` exposes the predicate.
* **All 13 receive sites size from it**, and each grew a `_sized` twin taking
  the capacity explicitly: `try_recv_sized<N>`, `try_recv_validated_sized<N>`,
  `try_recv_request_sized<N>`, `call_sized<N>`, `call_polling_sized<N>`,
  `send_request_sized<N>`, `try_next_sized<N>`, `wait_next_sized<N>`,
  `get_result_sized<N>`, `get_result_future_sized<N>`,
  `try_recv_feedback_sized<N>`, `try_recv_result_sized<N>`,
  `try_recv_goal_request_sized<N>`, `TickCtx::call_sized<Req, Resp, N>`.
  `Future<T>` holds its buffer as a MEMBER, so its capacity is a class template
  argument (`Future<T, Cap>`, defaulted) rather than a function one — which is
  why `send_request`'s return TYPE is where the number is observable.
* The 17 TRANSMIT sites are untouched, deliberately: an over-estimate there
  only wastes stack, and mixing the two directions in one change makes the
  risky half unreviewable.

Coverage is in `packages/api/nros-cpp/tests/compile/rx_size_bound.cpp`
(`just check cpp`), which instantiates every one of the 13 for a bounded type
AND an unbounded one, and asserts `send_request`'s return type is
`Future<Bounded, 137>` — the derived RX bound, against the 1170 the estimate
would give.

## What is still open: what flipping the unbounded arm would cost

Option (1) — the unbounded arm poisons instead of falling back — would land on
**all 13 receive sites**, and through `SvcOf`/action typedefs on every service,
client and action tier over an unbounded payload. On the measured corpus that
is **81 of 120 stock Humble types**, including `nav_msgs/Odometry`; a user's
first symptom is that code which compiled yesterday no longer does.

The migration per site is small but has to be made SOMEWHERE for each:

1. **Give the type a bound** — bound the field in the `.msg` (`string<=64`), or
   an INLINE `cap` in `nros-codegen.toml` (`[fields]` / `[types]` /
   `[packages]` / `[defaults]`). One edit fixes every site for that type, and
   it is the honest fix: the buffer then has a number somebody chose.
   A `heap` or `view` cap does NOT bound (RFC-0033), so it does not help here.
2. **Or call the `_sized` form** with a byte count, per call site. Cheap to
   write, but it moves the number back into user code, which is what the
   estimate was doing badly in the first place.

Both are now available, which is the point of this change: before it, (2) did
not exist, so a flip had exactly one remedy and it was a `.msg` edit. The flip
itself stays a product decision — the `unbounded` arm of `detail::buffer_bounds`
is the one line it turns on, and the compile test PINS today's answer so making
it is a deliberate edit and not a drift.


## The bytes, measured 2026-09-04 — the delta issue 0896 closed without

[Issue 0896](0896-c-cpp-subscriptions-never-state-a-buffer-hint.md)
closed on "the mechanism being correct, not on a measured saving", and said what
was missing: the C++ receive buffers are STACK, so `just mem-report` cannot see
them and "whoever wants the bytes needs two different instruments". This is one
of them — read from the emitted constants rather than from any image, because
the constant is what sizes the buffer.

Both numbers are in every generated C++ header (`SERIALIZED_SIZE_MAX`, the
estimate; `RX_MAX_SERIALIZED_SIZE`, the derived bound), so the comparison needs
no build. Over the 52 distinct stock types paired from this tree's generated
trees (`std_msgs`, `geometry_msgs`, `action_msgs`, `example_interfaces`,
`builtin_interfaces`):

| | bytes |
| --- | ---: |
| estimate, summed over the 52 types | 11,498 |
| derived bound, same 52 | **2,264** |
| difference | **9,234** |

**Read that as per-buffer capacity, not as any image's saving.** It is the sum
of what one buffer of each type would reserve; an image pays it once per type
per receive site it instantiates, of which there are 13. The per-type figures
are what a reader can act on:

| type | estimate | derived | per buffer |
| --- | ---: | ---: | ---: |
| `action_msgs/GoalInfo` | 1,028 | 40 | −988 |
| `geometry_msgs/Twist` | 1,028 | 64 | −964 |
| `geometry_msgs/Wrench`, `Accel` | 1,028 | 64 | −964 |
| `geometry_msgs/Pose`, `Transform` | 1,028 | 72 | −956 |
| `geometry_msgs/TwistWithCovariance` | 1,056 | 356 | −700 |

The `Twist` row is 0896's own spot-check ("`geometry_msgs/Twist` reading 1028
against a derived 64"), reproduced here from a different direction, which is why
this extraction is trustworthy at all — it agrees with a number nobody derived
it from.

**The split: 40 of 52 over-state, 10 agree, 2 under-state.** The direction
matters more than the magnitude, and 0896 says why: an over-estimate wastes
stack, an under-estimate sizes a buffer that truncates. Both under-states here
are `Empty` (`std_msgs` and `example_interfaces`), estimate 4 against a derived
8 — the 4-byte CDR encapsulation plus the one padded dummy byte ROS 2 emits for
a memberless message. Harmless in practice, since nothing reads an `Empty`
payload, but it is the estimate being wrong in the dangerous direction on the
simplest type in the corpus, which is a fair summary of why it was replaced.

Not measured: an image delta. That still needs the second instrument 0896 named
— these are stack frames, and the number of live buffers depends on which of the
13 sites an image instantiates.


## The lever was connected to nothing — rerouted, and the RX/TX split audited (2026-09-05)

The open half above says the flip is "the `unbounded` arm of
`detail::buffer_bounds` … the one line it turns on". That line existed;
**nothing called it.** `buffer_bounds` had no consumer anywhere outside
`size_bound.hpp`, so flipping it would have changed nothing at all.

### Where the estimate actually was

| | sites |
| --- | ---: |
| `uint8_t buf[M::SERIALIZED_SIZE_MAX]`, bypassing both templates | **15** |
| strict `rx_size_bound` / `tx_size_bound` (poisons on unbounded) | 2 |

(24 grep hits, but 8 are doc comments and one is a stub type defining its own
constant. 15 are real buffer declarations.)

Spread over `client` (3), `action_client` (3), `polling_action_server` (3),
`action_server` (2), `tick_ctx` (2), `service` (1), `polling_action_client` (1).

### The audit answer: all 15 are TX, and none is the 0896 hazard

Every one of the fifteen is
`X::ffi_serialize(&value, buf, sizeof(buf), &len)` with the return checked.
**There is no RX buffer sized by the estimate** — the receive paths already go
through `component.hpp`, which passes `nros::rx_size_bound<M>::value`, the
STRICT template that poisons on an unbounded type.

That matters for the product decision, and it lowers the stakes:

* the silent-truncation failure 0896 exists to remove **does not apply here**.
  These buffers are written by us, bounds-checked against `sizeof(buf)`, and an
  over-long message returns a non-zero code rather than corrupting anything;
* so an under-sized estimate on these paths is a LOUD runtime failure, not
  data loss. Flipping the unbounded arm would convert that into a compile-time
  failure — better in kind, but a usability change, not a safety fix.

The safety argument for the flip should not be made from these sites. It has to
be made about `component.hpp`'s receive path, which is already strict.

### What changed here

The 15 sites now size through `::nros::detail::buffer_bounds<T>::tx`, and
`action_server.hpp` gained the `size_bound.hpp` include it had been getting by
luck. This is deliberately NOT a semantic no-op — I predicted it would be and
was wrong, so the measurement is on real generated headers rather than an
assumption. `buffer_bounds` answers `TX_MAX_SERIALIZED_SIZE` for a DERIVED type,
so those buffers shrink to the bound:

| | bytes |
| --- | ---: |
| estimate, over the 19 types carrying both constants | 1,928 |
| derived, same 19 | **202** |
| TX buffers shrink by | **1,726** |

Zero buffers grow. The other 21 generated types in that tree state no derived
bound, so they keep the estimate and are untouched — which is exactly the
`legacy`/`unbounded` arm doing its job.

`just check cpp`, `just check c` and `just check fast` (213/213) all pass.

### What this leaves for the decision

The flip is now genuinely one line at one control point, and it is a real lever.
Staging it is possible where it was not before: the arm can be turned on for RX
before TX, and `has_derived_size_bound<M>` already exists for a per-site opt-in.

Still a product decision, and unchanged in substance: 81 of 120 stock Humble
types have no derived bound, so the flip makes code that compiled yesterday stop
compiling until each type gains a `cap` or each call site uses the `_sized`
form.


## DECIDED and LANDED 2026-09-05: the unbounded arm poisons

Maintainer decision, in their words: avoid silent truncation; over-long messages
fail with explicit errors; **an under-size estimate should be a compile-time
error, because the calculation is incorrect and cannot accept the message size
the user expects.**

That is the argument this issue could not make for itself, and it is the right
one. The estimate is not a conservative bound — `compute_serialized_size_max`
uses a flat 512 per nested message and a default capacity per string, and over
the measured corpus it over-stated 38 of 39 bounded types and UNDER-stated one.
An under-state sizes a stack array that cannot hold what the program
legitimately sends, and no run-time check repairs a number already baked into
`uint8_t buf[N]`. It is a fact about the build, so it is a build error.

`detail::buffer_bounds<M, bound_shape::unbounded>` now delegates to
`strict_bounds`. The `legacy` arm is deliberately NOT flipped: a legacy type
predates the bound marker and never had a derivation to disagree with, so it
keeps the estimate and an existing consumer keeps compiling.

### What it cost in-tree: nothing

Measured, not assumed. `compile-check-fixtures.sh` builds **36 rows across 5
builders** with **zero** `states no serialized-size bound` errors, and
`just check cpp`, `just check c` and `just check fast` (213/213) all pass. Every
in-tree C++ consumer either uses a bounded type on these paths or does not reach
them.

(That run needed the zenoh-pico INET6 fix present, which is a separate PR. The
first attempt reported zero poison hits with `cxx=0` — the C++ checks had not
run at all, because the config-header step died on that unrelated bug. A zero
from a check that did not execute is not a zero.)

### How it is held

* `unbounded_buffer_probe.cpp` — an EXPECTED-FAILURE TU that asks an unbounded
  type to size a buffer, in both directions, and must not compile. A
  `static_assert` cannot express "this must not compile", so the assertion that
  the poison still fires has to live in a TU of its own.
* Wired into `check-cpp`, asserted-present first, because a missing file is also
  a non-zero `c++` and reads as a pass.
* Mutation-tested: reverting the arm to the estimate makes `check-cpp` fail with
  "a type with NO derived bound sized a buffer and compiled clean".

### What a user does now

Unchanged from the analysis above, and the cost is real — 81 of 120 stock Humble
types state no derived bound:

1. **Bound the type** — `string<=64` in the `.msg`, or a `cap` in
   `nros-codegen.toml` (`[fields]` / `[types]` / `[packages]` / `[defaults]`).
   One edit fixes every site for that type. `heap` / `view` caps do NOT bound.
2. **Or call the `_sized` form** at the one site that needs it.

The receive paths were already strict before this (`component.hpp` passes
`rx_size_bound`), so what changed is the 15 TX sites the previous section
rerouted — and on those the old failure was loud rather than silent. The
decision was made on the correctness of the number, not on a silent-data-loss
risk that did not exist there.
