---
id: 1015
title: "A derived session pool of ZERO silences the board, with no diagnostic"
status: resolved
area: cmake
severity: high
related: [1002, 0991, 0940, 0965, 0460, 1033, phase-403, phase-412]
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
2. **Floor it in C**, `#if ZPICO_MAX_X < 1 #error`. Louder, and it fails at
   BUILD rather than at silence -- worth having as well, since a floor in one
   producer does not bind another.
3. **Make zero legal**, giving each pool a `[1]` placeholder. Rejected: it
   spends a slot on every image to make an illegal value representable.

## Fix

Options 1 and 2 landed in `a38382164`. They were not enough, and the reason is
this file's own "Not covered" note: **a floor in one producer does not bind
another**, and that fix guarded THREE of the ten fixed array extents in
`zpico.c`. `ZPICO_MAX_LIVELINESS` and `ZPICO_MAX_PENDING_GETS` are
Kconfig-settable and size arrays in the very same struct; they were left loaded.

The class, closed. One rule -- **a number that sizes a fixed C array has a floor
of ONE** -- at every point that can enforce it:

| where | what | binds |
| --- | --- | --- |
| the DERIVATION | `floor1` in `nros-cli-core` `entity_inventory.rs` (already there) | the derived pools |
| the CMAKE lane | `nros_assert_c_array_extent()`, `cmake/NanoRosCArrayExtents.cmake`, called from `nros_rmw_zenoh.cmake` for all 7 extents it `-D`s | environment, `.conf`, derivation, literal fallback -- all four, because it checks the value about to be written rather than where it came from |
| the CARGO lane | `ShimConfig::check_c_array_extents()`, `nros-zpico-build` | a build with no cmake in it: a `fixtures.toml` `env = {}` row, a shell export, a bare `cargo build` |
| the C SOURCE | `#if <M> < 1` / `#error`, extended 3 -> **10** extents | every producer, including one written tomorrow |

The cmake refusal is the one this issue asked for: it fires at CONFIGURE, before
a compiler is invoked, and it names the macro, the array it sizes, the Kconfig
option that overrides it, and where the declaration was supposed to come from.
It also distinguishes the three illegal states, which have different causes:
EMPTY (the knob resolved to nothing, `-DX=` -> `flexible array member not at end
of struct` on a struct nobody edited), the `-1` DERIVE SENTINEL (escaped its
resolver -- `size of array is negative`), and ZERO (this issue).

## The gate this file asked for

`scripts/check-c-array-extent-floors.py`, on the fast line as
`just check c-array-extent-floors`. It is keyed off the ARRAY DECLARATIONS in
`zpico.c`, **not** off a list, because a list is exactly what the first fix was:

* **R1** every macro used as a fixed array extent in `zpico.c` has a
  `#if <M> < 1` guard whose body actually raises `#error` (an empty body reads
  exactly like a guard);
* **R2** every extent the Zephyr cmake lane `-D`s is asserted first -- **R2b**
  guarding the SAME expression that is emitted (a guard on a different variable
  is exactly as green and exactly as useless -- phase-412 W1's built-up
  `NROS_DERIVED_${_pool}` that resolved EMPTY), and **R2c** BEFORE the emission;
* **R3** every extent `ShimConfig::defines()` emits is in
  `C_ARRAY_EXTENT_DEFINES`, **and nothing else is** -- a floor asserted where
  the hazard does not exist is how a check stops meaning anything.

Measured against the pre-fix tree: **15 problems**, including the 7 unguarded
extents. Nine self-test cases, one of them the scanner's own negative control (a
`zpico.c` with no arrays is REPORTED, not passed -- a pattern that matches
nothing satisfies R1 vacuously).

Behaviour, as opposed to presence, is
`tests/cmake-c-array-extent-tests.sh` (`just check
c-array-extent-floor-behaviour`, `cmake -P`, ~1 s, 26 assertions).

## Proof, both directions

* **Still builds.** A pool of 1 -- which is what the derivation's own floor
  produces, so it is the value a real single-service image arrives with -- 4, 8
  and 65536 all configure. `cargo test -p nros-zpico-build` 34/34;
  `nros-cli-core` entity-inventory 49/49. A tree-wide sweep finds **nothing**
  setting any of these knobs to 0 in any `.conf`, `.toml`, `Kconfig` or
  justfile, so no existing image moves.
* **Fails loudly.** 0 in any of the ten extents is refused by every producer
  that supplies it. The C backstop was verified by extracting the ten guard
  blocks VERBATIM from `zpico.c` and compiling them: all ten at 1 compiles
  clean, and each one at 0 fails the compile naming itself.
* **The tests catch a removed floor.** Mutating `floor1` to the identity makes
  `every_session_pool_floors_at_one` and
  `topics_become_per_node_subscriptions_and_publishers` both fail.

## Still open, deliberately

The wider question this file raised -- *what other derived value has an illegal
special case at some boundary* -- is not closed by this. What is closed is the
`0`-on-a-C-array case, for every pool in the zenoh backend, with a gate that
extends itself to a pool added later. The XRCE backend's caps are the same
shape one layer over and are issue 1033's, not this one's.
