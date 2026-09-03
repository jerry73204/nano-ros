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

## Naming: alias, not replace

Two readings of "rename our items to ROS 2's names":

**(a) Replace.** `nros::` becomes `rclcpp::`; one name, one namespace.
**(b) Alias.** `nros::` stays the definition; the ROS 2 spelling becomes a
first-class alias declared in the API headers themselves, and the separate
`rclcpp_compat.hpp` file disappears because its content moved into the headers.

**This RFC chooses (b)**, for three reasons.

1. **ODR.** If we define `rclcpp::Node` and a host build also links real
   rclcpp — which host-side tests, bridges and tooling plausibly do — the two
   definitions collide silently. Under (b) the collision is still possible, so
   the compat header must `#error` when upstream's include guard is already
   defined; under (a) there is no fallback spelling to recover with.
2. **Diagnosability.** A compiler error, a stack trace or a `nm` dump should be
   able to say *which* implementation this is. Under (a) it cannot.
3. **It costs nothing.** An alias adds no member. `rclcpp::Publisher<T>::SharedPtr`
   requires a nested `SharedPtr` on our `Publisher` whatever the namespace is
   called.

Point 3 generalises into the strategic claim of this RFC:

> **The rename is cheap and cosmetic; the compatibility is the work.** Adopting
> ROS 2's names does not by itself make one more ported line compile. What makes
> lines compile is nested `SharedPtr` types, `create_service`, parameters on the
> node, `now()`. Those are needed whatever the namespace is called.

So the rename goes LAST. Doing it first would relabel the gaps without closing
one, and would spend the safety property — "our names are different, so a
mismatch is visible" — before the mismatches are fixed.

## What "mostly full compat" can honestly mean

It cannot mean the whole rclcpp surface. Four upstream idioms account for most
of what we decline, and three of them are load-bearing for ROS 2's design and
incompatible with ours:

| idiom | rows | can we? |
| --- | ---: | --- |
| executor as wait-set assembler over polymorphic `Waitable`s | ~128 | no — dynamic membership needs an allocator (RFC-0002) |
| parameters as a distributed service with owned-storage values | ~56 | partly — server side yes, client side is a product choice |
| graph and middleware queryable at runtime | ~55 | no — no dynamic discovery |
| types erased and resolved at runtime | ~45 | no — no dynamic loader |

The honest claim is therefore about PROGRAMS, not surface: *the shapes a ROS 2
node is actually written in compile and behave, and everything else fails
loudly.* That is measurable — the in-tree ported templates are the measurement —
and it is what the roadmap targets.

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
