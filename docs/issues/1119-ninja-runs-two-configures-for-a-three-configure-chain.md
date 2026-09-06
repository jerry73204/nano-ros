---
id: 1119
title: "The knob chain needs three configures and one `west build` runs two, so
  a clean build dir ships the pass-2 value -- 24 KB of arena on the island"
status: open
type: bug
area: build, boards, memory
severity: high
related: [issue-0991, issue-1002, phase-403, phase-412]
---

## The claim, and the measurement that contradicts it

`cmake/NanoRosReconfigure.cmake` and `zephyr/cmake/nros_cargo_build.cmake` both
state the same invariant, in detail and with a table:

> ninja re-runs cmake until `build.ninja` stops being stale, so all three
> passes happen inside ONE `west build` and the build compiles at 880.

Measured on the safety island (MR-CANHUBK344), 2026-09-06, clean build dir, one
the island's `board-build` recipe -- which is one `west build`:

    configures actually run        2
    re-configures armed            3
    NROS_SUBSCRIBER_BUFFER_SIZE    1496   <- pass-2 value, the linked CLOSURE
    NROS_DERIVED_SUBSCRIBER_...    880    <- the fragment on disk, SUBSCRIBED

A second run of it on the same dir runs one more configure and
delivers 880. So the chain converges -- in two invocations, not one.

The arming is not the defect: three arms were requested and the fragments were
future-dated. Ninja ran the generator twice and started the build.

## What it costs

The arena model bills each subscription three receive buffers at the delivered
class. On this image, 11 subscriptions:

    delivered 1496   arena 70296    RAM 307640 B / 320 KB   93.88%
    delivered  880   arena 46272    RAM 280536 B / 320 KB   85.61%

24,024 bytes of arena and 27,104 bytes of RAM, on a part with 320 KB, and the
measured arena peak is 17,736 either way. Both images boot and both serve the
full graph, which is the problem: over-sizing is silent. The island read the
70296 figure as "the derivation over-budgets" and nearly went looking for a
modelling error that does not exist.

`check-knob-delivery` DOES catch it -- pointed at the unconverged dir it names
the knob, the derived value and the resolved one exactly. Nothing runs it on a
board build, so it caught nothing.

## Why the two-deep chain is the shape

    pass 1   entity=placeholder   bounds derived over the CLOSURE   readers: nothing
    pass 2   entity=real          bounds derived over SUBSCRIBED    readers: 1496 (stale)
    pass 3   entity=real          bounds unchanged                  readers: 880

The readers run FIRST in a configure (`nros_resolve_knobs` inside
`find_package(Zephyr)`) and the producers run mid- and end-configure, so each
link in the chain costs a whole pass. Pass 3 exists only to let the readers see
what pass 2 wrote, and pass 2's fragment is UNCHANGED from pass 3's point of
view -- so whatever makes ninja restart has to survive a pass in which nothing
the producer writes has changed.

## Directions, none of them free

* Make ninja actually restart the third time. It regenerates and re-executes,
  but it did not do so twice here; whether that is a ninja bound, a west/cmake
  wrapper, or the future-dating losing a race is not established, and guessing
  is how this gets a second wrong fix.
* Shorten the chain so readers do not run before producers -- read the fragment
  lazily at the point of use instead of loading it in `find_package(Zephyr)`.
  Removes the ordering entirely and is the largest change.
* Fail the build instead of shipping the stale value. The safety island does
  this now (`board-build` runs `check-knob-delivery` and refuses), which is a
  guard rather than a fix: it converts a silent 24 KB into "run it again".

## Do not

Do not "fix" it by making the recipe build twice. That hides how many passes
the chain needs, and the number of passes is the bug.
