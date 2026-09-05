---
id: 1092
title: "`rmw-abi-shape` licenses a deviation without pinning it — 7 of 24
  mutations left every RMW gate green, including a `void` return and a gutted
  slot"
status: open
type: bug
area: ci, rmw
related: [phase-393, phase-428, rfc-0089]
---

## What is true today

The drift CLAUDE.md records is **gone**, and this was re-derived independently
rather than taken from the tools' exit codes:

* the 88-symbol contract holds exactly — three `librmw_*_cpp.so` export
  byte-identical sets and `diff` against the recorded file is empty;
* an independent brace-matching parser of `rmw_vtable.h` finds 68 slots, and
  `diff` against `rmw-abi-shape.vtable_slots()` is empty;
* the map partitions the contract exactly: 63 vtable / 20 declined / 3 layer /
  2 global / **0 gap**, no orphan slot, no missing symbol.

`check_against_vtable` works in both directions — four mutations that attack it
are all caught.

## The defect

**The cross-check is narrower than the green light it produces.** Coverage,
computed from the tools' own tables:

| | slots |
| --- | ---: |
| signature compared AND args exactly enforced | **15** |
| in `ARG_DEVIATIONS`, which accepts *any* argument list | **39** |
| never signature-compared at all (11 `ADDED` + 3 grouped-only) | **14** |
| total | 68 |

`ARG_DEVIATIONS` has 42 entries and every value is a **reason string**. The
branch is

```python
elif slot in ARG_DEVIATIONS and (ret_ok or slot in RET_DEVIATIONS):
```

— membership, not content. The entry says *that* a slot deviates and never
*how*, so once a slot is listed, the header may say anything.

## Mutations that left all four commands green

Twenty-four applied to a scratch mirror; **seven survived**.

| mutation | why nothing saw it |
| --- | --- |
| **`has_data` gutted** — return → `void`, handle → `rmw_publisher_t *`, two junk args | it is in `ADDED`, which is checked for non-emptiness and non-empty reasons. Nothing asserts an `ADDED` slot EXISTS or keeps its shape |
| **`has_data` deleted entirely** | same hole; only `rmw-vtable-order` noticed, via positional-initialiser drift |
| **`create_node` return `rmw_ret_t` → `void`** | `RET_DEVIATIONS` membership |
| **`take` gains `uint64_t bogus_extra`** | `ARG_DEVIATIONS` membership |
| **`take`'s handle → `const rmw_publisher_t *`** | same |
| **`create_session` → `void(int, char)`** | grouped-only, never compared |
| **`subscription_take_event` → `void(rmw_publisher_t*, int)`** | same |

**The `void`-return one is a regression of the exact defect the gate exists
for.** W5 found six slots returning `void` where upstream returns `rmw_ret_t` —
"the axis that decides whether a caller can detect failure at all" — and
reintroducing it on any of the six `RET_DEVIATIONS` slots is now invisible.

`rmw-api-comparison` reddens for four of the seven, but it is a **staleness**
gate: it compares the rendered doc to the committed one, so regenerating the
doc clears it and the parity instruments stay green on a broken ABI.

## The unguarded exception mechanism

There are **zero `gap` rows today**, so clause 3 is vacuously satisfied. The
mechanism is not: the deferral rule matches `\bissue[ -]?(\d{4})\b` and never
resolves the number.

* a gap reason naming **issue 9999**, which does not exist → all four green;
* a gap reason naming **issue 0776**, which is `resolved` and archived → all
  four green;
* authored `status = "not-implemented", issue = 9999` → green; `issue = "banana"`
  → green (`check_status` tests truthiness only).

The exemplar cited in the shape script's docstring, its error text and its
self-test probes is itself issue 0776 — resolved and archived.

## Fix, ranked

1. **Assert every `ADDED` key is a live slot and pin its signature** (or at
   minimum its return type). Closes the two mutations nothing but bindgen saw.
2. **Make `ARG_DEVIATIONS` / `RET_DEVIATIONS` values carry the EXPECTED
   signature**, not a reason alone, so an entry pins the difference instead of
   licensing all of them. Raises exactly-enforced slots from 15/68 toward 54/68.
3. **Compare grouped-only targets** against the upstream signature of the
   symbol grouped onto them.
4. **Resolve every `issue NNNN`** in a gap reason and every authored `issue =`
   to a file in `docs/issues/` with `status: open`.

## Not verified

`check-abi-bindings` was not run (it writes `generated.rs` into the tree). A
scratch re-run of the pinned bindgen 0.72.1 against the mutated header produces
a different `generated.rs`, so it would go red — on a host that has bindgen; the
recipe skips when it is absent. The snapshot files were not mutated; the
contract half re-derives byte-identically on this Humble install and nothing is
claimed about other distros.
