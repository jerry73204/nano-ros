# Phase 374 — test-suite speed-up

**Status (2026-08-21). Survey only, nothing landed.** Opened after a tier-2 run
took ~2.5 h and looked test-bound. It is not: on that lane the slowest test was
15.9 s and the wall was dominated by fixture building and the gate/check lane.
The opportunities below are real but should be sized against that before anyone
spends a day on them. Build-side work lives in
[phase-371](archived/phase-371-build-cpu-utilization.md).

## What the tier-2 run actually showed

| | |
| --- | --- |
| slowest single test | **15.9 s** |
| next slowest | 9.9 s, then 0.67 s and below |
| Cyclone RMW suite | 17 tests, 100% pass, well under a minute |
| `== zephyr ==` first appears at | log line 1317 of 8300+ |

So test EXECUTION was a small fraction of a 2.5 h run. Anything in this phase
that speeds up tests should be justified against `test-all` and the nightly
pairwise sweep — where far more grouped tests actually run — rather than against
tier 2, where it will not show.

## The serialization inventory

24 test-groups, 29 rules, thread caps summing to 96 against 32 cores, and no
global `test-threads` override (nextest defaults to num-cpus). So groups are not
globally throttling; seven specific ones are fully serial.

### Class A — a build-time-baked port forces the serialization (FIXABLE)

| group | why |
| --- | --- |
| `zephyr-qos-port` | "one baked image serves two tests… the qos zephyr entry image + its baked router port (allocator 7460) are shared… serialize them so the routers never collide" |
| `qemu-esp32-pubsub-port` | esp32↔esp32 e2e plus two esp32↔native interop tests "all start a router on it" |

Both serialize because the router port is **baked into the image at build time**,
so two tests cannot hold it at once. Issue **0166** already established the fix
shape for exactly this — make the port runtime-locatable instead of
build-time-baked — and resolved it for the Zephyr zenoh e2e lane. These two are
the same defect in places 0166 did not reach.

This is the only class here where the constraint is an artifact of how we build,
rather than a property of what is being tested.

### Class B — resource or discovery contention (harder, maybe correct as-is)

| group | why |
| --- | --- |
| `native-qos-discovery` | a peer intermittently misses one pair's SUBSCRIPTION inside the settle window, and WHICH cell fails rotates; each passes solo. Serialized deliberately "rather than lengthening the sleep — the assertion is about what is advertised, not about how fast" |
| `ros2-interop` | each test spawns zenoh sessions for discovery; parallel runs cause contention and discovery timeouts, and some list/info helpers remain daemon-sensitive |
| `qemu-freertos-entry`, `zephyr-fvp`, `matrix-consumers-serial` | not yet examined |

`native-qos-discovery`'s serialization is a considered trade, not an oversight —
the alternative was a longer sleep, which weakens the assertion. Changing it
needs a way to make advertisement observable without racing, not just more
threads.

### Not a bottleneck: ROS domain assignment

Already solved and worth recording so nobody re-investigates. Natives get
distinct domains from `nros_tests::unique_ros_domain_id()`; Cyclone fixture pairs
bake distinct domains (50–58) for parallel SPDP. Domain collision is not what
serializes anything here.

### Not parallelism at all: QEMU

16 of the 24 groups are QEMU lanes. Their caps (3, 9, 6, 4, 2…) exist because
QEMU is single-threaded per instance and the host has finite cores — raising them
oversubscribes rather than accelerates. QEMU lanes are also known to flake under
sweep load (287-W7: six nuttx lanes failed 3/3 in-sweep, passed solo), so the
caps are partly a correctness device. Treat them as fixed unless someone
re-measures.

## Candidate work, unranked until sized

1. **Runtime-locatable router ports** for `zephyr-qos-port` and
   `qemu-esp32-pubsub-port`, following 0166. Removes two serial groups.
2. **Size the win first.** Instrument `test-all` and the nightly sweep for
   per-group wall time before touching anything — this phase exists because a
   2.5 h run *looked* test-bound and was not.
3. `slow-timeout` is `30s` with `terminate-after = 2`, so a wedged test costs 60 s.
   Fine, but worth knowing when reading a slow sweep.
4. Examine the three unexamined serial groups (`qemu-freertos-entry`,
   `zephyr-fvp`, `matrix-consumers-serial`) for which class they fall into.

## Measurement discipline

Inherited from phase-371, where four conclusions had to be retracted:

* Take every timing **twice**. A single measurement in a disturbed tree is not
  evidence — cold caches produced a 5x spread there.
* Scope samplers by **process lineage**, not by process name. A concurrent
  unrelated build on the same host contaminated a whole set of numbers.
* Check **preconditions** before attributing a red to code. Failing solo
  separates a concurrency artifact from a real failure; it does NOT separate a
  real failure from a stale CLI or a moved submodule pointer.
