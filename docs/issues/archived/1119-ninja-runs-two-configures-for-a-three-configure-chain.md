---
id: 1119
title: "The knob chain needs three configures and one `west build` runs two, so
  a clean build dir ships the pass-2 value -- 24 KB of arena on the island"
status: resolved
type: bug
area: build, boards, memory
severity: high
related: [issue-0991, issue-1002, phase-403, phase-412]
resolved_in: "cmake/NanoRosReconfigure.cmake -- one baseline and one arm per PASS"
---

## The measurement

Safety island (MR-CANHUBK344), 2026-09-06, clean build dir, one `west build`:

    configures actually run        2
    re-configures armed            3
    NROS_SUBSCRIBER_BUFFER_SIZE    1496   <- pass-2 value, the linked CLOSURE
    NROS_DERIVED_SUBSCRIBER_...    880    <- the fragment on disk, SUBSCRIBED

A second invocation on the same dir ran one more configure and delivered 880.
The chain converged -- in two invocations, not one.

Cost, on an image with 11 subscriptions (the arena bills three receive buffers
per subscription at the delivered class):

    delivered 1496   arena 70296    RAM 307640 B / 320 KB   93.88%
    delivered  880   arena 46272    RAM 280536 B / 320 KB   85.61%

24,024 bytes of arena and 27,104 of RAM on a 320 KB part. Both images boot and
serve the full graph, which is the problem: over-sizing is silent, and the
island nearly filed a modelling bug against a model that is right.

## Why the chain is three passes

    pass 1   entity=placeholder   bounds derived over the CLOSURE   readers: nothing
    pass 2   entity=real          bounds derived over SUBSCRIBED    readers: 1496 (stale)
    pass 3   entity=real          bounds unchanged                  readers: 880

The readers run FIRST in a configure (`nros_resolve_knobs` inside
`find_package(Zephyr)`) and the producers run mid- and end-configure, so each
link in the chain costs a whole pass. That part is by design and unchanged.

## The cause, established 2026-09-06

Not ninja, not west, not the clock. **The producer runs once per PACKAGE and
the re-configure mechanism kept its state per CALL.**

`nros_find_interfaces()` -- the bounds producer -- runs from every leaf that
declares an interface dependency, via `_nros_generate_declared_interfaces` in
`cmake/NanoRosVerbs.cmake`, which `nano_ros_add_executable`, `nano_ros_add_node`
and `nano_ros_auto_add_library` all call. The island's Zephyr image reaches it
SIX times in one configure:

    src/island_interfaces/CMakeLists.txt          calls the verb directly
    autoware_mrm_emergency_stop_operator          nano_ros_auto_add_library
    autoware_mrm_comfortable_stop_operator        nano_ros_auto_add_library
    autoware_stop_mode_operator                   nano_ros_auto_add_library
    autoware_mrm_handler                          nano_ros_auto_add_library
    src/zephyr_entry (nano_ros_add_executable)    package.xml has an exec_depend

Two calls are already enough to lose the arm; six is what the image has.

Each call ran the whole verb body, and two of its steps were hazards to the
call before it:

* the `nros_reconfigure_settle()` at the head of the verb found the fragment
  future-dated -- by the PREVIOUS call, in the same configure -- read that as
  "a date a previous PASS left behind", and cleared it. `build.ninja` was then
  written after the fragment, so ninja found nothing stale and never restarted.
* the per-call `nros_reconfigure_snapshot()` took the bytes the previous call
  had written as its `_before`. So a fragment that HAD changed since the
  readers of the pass ran compared EQUAL, took the `settled` branch, and
  cleared the per-fragment pass counter with it.

Either alone loses the arm. Both together also give the counts exactly: pass 1
arms bounds and entity (2), pass 2 arms bounds once and then takes it back (1)
-- three arms, two configures, compiled at pass 2's 1496.

### The reproduction

A five-line cmake project with the same two-deep chain, no Zephyr, no board and
no SDK. `tests/cmake-reconfigure-tests.sh` case J drives the same shape through
the REAL modules:

    producer calls per configure   configures   armed   delivered
                               1            3       3         880
                               2            2       3        1496
                               3            2       3        1496

One call converges; two do not, and reproduce the island's numbers digit for
digit. Case H was green throughout because it drives ONE producer call per
configure -- a shape no image in this tree has.

### Ruled out, and how

* **A ninja restart bound.** ninja 1.10.1's regeneration loop is bounded at
  100 cycles, and the same project restarts twice when the producer is called
  once. `ninja -d explain` names the reason each time it restarts (`output
  build.ninja older than most recent input bounds.cmake`) and prints nothing at
  all once the date has been cleared.
* **The manifest being byte-identical between passes.** Tested by routing the
  delivered value through a side file so `build.ninja` could not change: it
  still restarted twice. CMake also rewrites `build.ninja` with a fresh mtime
  on every configure whether the content changed or not, so no
  copy-if-different / `restat` interaction is in play.
* **`NROS_RECONFIGURE_FUTURE_SECONDS` losing a race** (the configure's tail
  outrunning the 120 s date). Consistent with the counts but not needed to
  explain them: the reproduction holds with a configure well under a second.
* **west / `cmake --build` swallowing a restart.** The reproduction is a bare
  `ninja` and shows the same two configures.

## The fix

`cmake/NanoRosReconfigure.cmake`: the BASELINE and the ARM are properties of the
CONFIGURE, held in GLOBAL properties -- born and dying with one configure, which
is exactly the lifetime "what the readers of this pass consumed" has. (The cache
is the wrong lifetime; issue 1002 already paid for using it that way.)

* `nros_reconfigure_settle()` clears a date at most once per fragment per
  configure, and takes the pass baseline while it is there. A later reader
  cannot undo an arm this pass raised.
* `nros_reconfigure_on_change()` measures against that baseline. The caller's
  `_before` is used only when nothing in the configure has touched the fragment
  yet -- the lane with no earlier reader, where the producer IS the first
  toucher.
* a second producer call that still differs from the baseline RE-DATES the
  fragment (its own write moved the mtime to now) and spends no further pass of
  `NROS_RECONFIGURE_MAX_PASSES`: it is the same pass. Counting CALLS meant an
  image with N interface leaves spent N of the bound per configure.
* a call that takes the fragment BACK to the baseline clears a date an earlier
  call in the same pass armed, because that pass no longer needs a
  re-configure.

No call site changed and nothing builds twice.

## Not covered

* The island's `board-build` still runs `check-knob-delivery` and refuses an
  image that fails it. That guard stays: it answers "did the value ARRIVE"
  about a real build dir, which no host-side gate can.
* Case J bounds the mechanism at 1, 2 and 3 producer calls. It does not bound a
  chain DEEPER than two producers; the pass count for that is still the chain's
  depth plus one, against `NROS_RECONFIGURE_MAX_PASSES` = 3.
* The chain is still three passes long. Reading the fragments lazily at the
  point of use, instead of loading them in `find_package(Zephyr)`, would remove
  the ordering entirely -- still the larger change, still not done.
