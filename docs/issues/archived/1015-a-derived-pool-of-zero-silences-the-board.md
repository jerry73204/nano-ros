---
id: 1015
title: "A derived session pool of ZERO silences the board, with no diagnostic"
status: resolved
area: cmake
severity: high
related: [1002, 0991, 0940, 0965, 1033, 1131, phase-403, phase-412]
---

# Zero is not a smaller pool, it is a different kind of object

## Measured

Same image, same derived knobs, one variable changed:

| `ZPICO_MAX_QUERYABLES` | serial output in 15 s |
| ---: | ---: |
| 0 (derived) | **0 bytes** |
| 4 (floored) | **110 bytes** |

At zero the board transmits NOTHING. No panic, no log line, no fault -- the
core sits in WFI and the ROS graph shows nothing. The Z4-verified image
(everything hand-set) transmits 108 bytes on the same board, cable and router,
so nothing outside the image is implicated.

## Cause

phase-412 W1 derives

    MAX_QUERYABLES = COUNT_SERVICE_SERVER + actions * ACTION_SERVER_QUERYABLES

The reference island declares no service servers and no actions, so it derives
exactly 0. In `zpico.c`:

    queryable_entry_t queryables[ZPICO_MAX_QUERYABLES];   // line 435

A zero-length array, and NOT the last member of the struct. It is a GNU
extension that compiles silently and changes what the struct is.

## The rule that was missing

phase-403 states the derived value carries NO headroom, deliberately: it is
exactly the declared demand, which makes the running image a checker of its own
declaration. That is right for a table the executor INDEXES -- registration
past the end returns `ExecutorFull`, which names the knob.

It is wrong when the number backs a FIXED-SIZE C ARRAY. There, zero is not a
smaller pool; it is a different kind of object, and the failure is a layout
change rather than a bounds check.

**A derived pool that backs a C array has a floor of 1.**

## Why every gate was green

This is the seventh delivery-class defect of the campaign and the first that
produced NO diagnostic at all:

* the image links;
* `check-knob-delivery` confirms the value ARRIVED, because it did -- 0 was
  delivered faithfully, it was simply the wrong answer;
* `check-knob-fixpoint` converges, because 0 is stable;
* the fast tier is green, because nothing here is a host-testable property.

Every check in the tree asks whether the number the image was built with is the
number that was derived. None asks whether the number is USABLE.

## Options

1. **Floor it in the derivation** (done in the fix commit): any pool backing a C
   array derives `max(1, demand)`. Cheapest, and it cannot regress a
   correctly-declared image because 0 was never a legal size for these arrays.
   *(The last clause is FALSE, and the Resolution below is about how: the same
   derivation feeds the XRCE pools, where 0 is legal and worth 33,296 bytes.)*
2. **Floor it in C**, `#if ZPICO_MAX_X < 1 #error`. Louder, and it fails at
   BUILD rather than at silence -- worth having as well, since a floor in one
   producer does not bind another.
3. **Make zero legal**, giving each pool a `[1]` placeholder. Rejected: it
   spends a slot on every image to make an illegal value representable.

## Not covered

Nothing asserts a derived knob is USABLE, only that it is faithfully delivered.
A gate that knows which knobs back fixed C arrays and requires them positive
would have caught this at configure time, and is cheap. The wider question --
what other derived value has an illegal special case at some boundary -- is
open, and 0 is the obvious one to check first for every pool this campaign has
touched.

## Resolution

Options 1 and 2 landed together (PR #283, `a38382164`). Option 1 was in the
WRONG LAYER, and this is the part worth reading.

### One derivation, two consumers, two different legal minima

`EntityInventory::derive` is not zenoh's. `NROS_DERIVED_MAX_SUBSCRIBERS` and
`NROS_DERIVED_MAX_QUERYABLES` also feed `NROS_XRCE_MAX_SUBSCRIBERS` and
`NROS_XRCE_MAX_SERVICE_SERVERS` (`zephyr/cmake/nros_cargo_build.cmake`, the
`if(CONFIG_NROS_RMW_XRCE)` block), where **zero is the answer**: issue 1033
lowered those minima from 1 to 0 on purpose, worth 33,296 bytes of heap per
subscriber slot and 4,384 per service-server slot, read from the zephyr cpp
listener's own DWARF.

The dates are the whole story. The floor landed 2026-09-04 19:56; 1033's
`fix(#1033): a cap with no user is ZERO, not one -- the image now fits its heap`
landed 2026-09-05 07:28. From then on the XRCE `build.rs` accepted a 0 that the
derive rung could no longer produce, on every image that takes the default
(`CONFIG_NROS_XRCE_MAX_SUBSCRIBERS` defaults to the `-1` DERIVE sentinel). Two
correct-looking fixes, one silently defeating the other, and no gate could see
it -- the number was derived correctly and delivered faithfully, which is the
sentence this issue is already about, one layer up.

So: **the derivation publishes DEMAND; the floor belongs to the consumer that
names the knob.**

* `entity_inventory.rs` derives the raw counts again (0 included).
* The zenoh consumers floor: `c_array_pool_floor()` on the cargo-leaf `[env]`
  sidecar, `_nros_c_array_pool_floor()` at the CMake `ZPICO_*` bridge. Measured
  on the shipped cmake function: `0 -> 1`, `3 -> 3`, `12 -> 12`, and EMPTY stays
  empty (empty is rung 4, "nothing resolved"; turning it into a 1 would state a
  number where the build script is supposed to use its own default).
* An explicit environment value is NOT floored: it is applied after, so a person
  who states 0 gets option 2's `#error` naming the knob rather than a number
  they did not ask for.
* The XRCE pools take the demand, so 1033's zero works again.

### The gate this issue asked for

`check-c-array-pool-floors` (fast lane, self-testing). It refuses the general
rule "pools are positive" -- 1033 measured that false -- and requires a DECISION
per knob instead: a `#if <KNOB> < 1` + `#error` beside the array, or a
`ZERO_LEGAL` entry naming what measured the zero. Both tables are checked in
both directions, so an entry that no longer describes the tree fails rather than
reading as coverage (the RMW-parity lesson: two green tools, 25 symbols apart).
It also holds the layering above: each producer of a guarded knob floors it, and
the shared derivation does not.

Mutation-checked against the real tree, not only its own fixtures:

| mutation | gate |
| --- | --- |
| drop the `ZPICO_MAX_QUERYABLES` guard from `zpico.c` | **FAIL** |
| put `.max(1)` back in `EntityInventory::derive` | **FAIL** |
| drop the cmake floor for `ZPICO_MAX_SUBSCRIBERS` | **FAIL** |
| unmodified tree | OK (21 knob-sized arrays: 3 guarded, 3 zero-legal, 15 unclassified) |

### What is NOT covered, still

* **The mechanism at zero was never isolated.** The board is out of tree; what
  is on record is the input/output pair (0 -> silence, 1 and 4 -> 110 bytes).
  This issue's stated cause -- "a zero-length array that is not the last member
  of its struct" -- is a plausible reading and NOT a measurement, and 1033
  measured that such an array compiles and walks correctly one backend over. The
  floor is justified by the board's behaviour, not by that explanation.
* **Fifteen knob-sized C arrays are still unruled** -> issue 1131. None of them
  is derived today, so 1015's shape cannot reach them yet; `ZPICO_MAX_LIVELINESS`
  and `ZPICO_MAX_PENDING_GETS` are the ones phase-392 is about to make derived.
