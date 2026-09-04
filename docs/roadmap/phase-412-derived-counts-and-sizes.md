# Phase 412 — every count and size the image can state about itself

**Status (2026-09-02). Opened from the mr-canhubk344 island audit.** phase-403
proved an image can derive its own buffer sizes; phase-409 proved the same for
the `Executor` value. This phase is the campaign that finishes the job: no
hand-picked count or size in a board `.conf` that the image already knows.

## Why a campaign rather than another knob

The island's board `.conf` was, until this week, twenty-odd numbers a person
chose. Every one of them is a claim about the image, and every one can be wrong
in two directions. Too small halts the board; too large costs RAM on a part
that has 320 KiB. Neither failure names the knob that caused it, and none of the
numbers carries the reasoning that produced it.

Four of them now derive (phase-403 W8/W9). The audit that followed found the
rest split cleanly into two groups, and the split is the point of this phase:

* **The number is already computed and published, and nothing consumes it.**
  Six knobs. No new arithmetic — a consumer and a precedence entry each.
* **No derivation exists yet.** Six more, each blocked on something specific
  and nameable.

The first group is the recurring defect this repo keeps producing: a mechanism
that is correct, tested, and unreachable from a real build. `rx_buffer_hint`
sized nothing (0896). The bound inventory had no reader (0963). A cap could not
reach codegen (#152). The entity inventory publishes eight per-kind counts and
exactly one of them becomes a knob (this phase). Each was found by an audit, not
by a gate, which is why W4 below exists.

## Where the numbers already are

`nros_entity_inventory_knobs_file` publishes, per configure, for the island:

```
NROS_ENTITY_INVENTORY_COMPONENT_COUNT 4
NROS_ENTITY_INVENTORY_ENTITY_TOTAL   28
NROS_ENTITY_COUNT_SUBSCRIPTION       10
NROS_ENTITY_COUNT_PUBLISHER          14
NROS_ENTITY_COUNT_TIMER               4
NROS_ENTITY_COUNT_SERVICE_SERVER      0
NROS_ENTITY_COUNT_SERVICE_CLIENT      0
NROS_ENTITY_COUNT_ACTION_SERVER       0
NROS_ENTITY_COUNT_ACTION_CLIENT       0
NROS_ENTITY_COUNT_GUARD_CONDITION     0
```

Of these, one knob is derived: `NROS_DERIVED_EXECUTOR_MAX_CBS` (14 — the
subscriptions, timers and service servers that claim a callback slot;
publishers claim none).

## W1 — wire the six. LANDED 2026-09-03, five of six

Each is a `_nros_entity_publish` of a value that is already in the file, plus
W8's precedence ladder (env > Kconfig/board > derived > crate default) and its
refusal rule (derive nothing when the inventory's own status is not `derived`).

| knob | island hand-set | published input | derives to |
| --- | ---: | --- | ---: |
| `NROS_MAX_SUBSCRIBERS` | 12 | `COUNT_SUBSCRIPTION` | 10 |
| `NROS_RMW_SUBSCRIBER_SLOTS` | 12 | `COUNT_SUBSCRIPTION` | 10 |
| `NROS_MAX_PUBLISHERS` | 16 | `COUNT_PUBLISHER` | 14 |
| `NROS_MAX_QUERYABLES` | 4 | `COUNT_SERVICE_SERVER` | 0 |
| `NROS_EXECUTOR_MAX_NODES` | 6 | `INVENTORY_COMPONENT_COUNT` | 4 |
| `NROS_EXECUTOR_ACTION_CLIENTS` | 0 | `COUNT_ACTION_CLIENT` | 0 |

**Both blocking questions, answered by reading the code.**

*Do the session pools need shim headroom?* No addend, verified rather than
assumed: `ZenohSubscriber::new` has exactly ONE caller (`create_subscription`),
and the two things that looked like they might share the pool do not -- the
graph cache lives in its own `graph_cache.sub` field, liveliness tokens in
`liveliness[ZPICO_MAX_LIVELINESS]`.

But the RAW per-kind count was still the wrong input, which is what made the
question worth blocking on. A declared ACTION is one entity that costs several
session slots:

    action server -> 3 queryables + 2 publishers
    action client -> 1 subscription

Wiring `MAX_QUERYABLES = COUNT_SERVICE_SERVER` would have under-sized every
image with an action. The multipliers now live beside the calls that decide
them (`ACTION_SERVER_QUERYABLES`, and new here `ACTION_SERVER_PUBLISHERS`,
`ACTION_CLIENT_SUBSCRIPTIONS`), held there by `check-infra-queryable-counts`.

Deliberately NOT counted: `PARAM_SERVICE_QUERYABLES` (6) and
`LIFECYCLE_SERVICE_QUERYABLES` (5). A feature enables those and the inventory
cannot see it, so counting them would guess. An image carrying either states
the knob, which is what "the derived value is a DEFAULT" is for.

*Is a component always a node?* One `ComponentNode` is one `Node::create` is
one name, and the executor keys node slots by NAME -- "a repeated name must
reuse its record". But `nros_create_node_on` (the bridge) creates TWO nodes per
bridge, OUTSIDE the component model, so `COMPONENT_COUNT` is a lower bound.

**So `NROS_EXECUTOR_MAX_NODES` is NOT wired.** Under-counting halts the board,
phase-403's rule is refuse rather than under-derive, and the island would save
6 -> 4. It moves to W2 with the bridge as its blocker.

**Measured on the island**, both configures (issue 0991, since fixed):

| knob | hand-set | derived |
| --- | ---: | ---: |
| `NROS_MAX_SUBSCRIBERS` | 12 | 10 |
| `NROS_RMW_SUBSCRIBER_SLOTS` | 12 | 10 |
| `NROS_MAX_PUBLISHERS` | 16 | 14 |
| `NROS_MAX_QUERYABLES` | 4 | 0 |

    RAM   324834 (99.13%) -> 291144 (88.85%)   -33690 B
    DTCM   93744 (71.52%) ->  89320 (68.15%)    -4424 B

38114 bytes, and RAM headroom goes from 0.87% to 11.15% -- about 36 KB free
where the Z4 image had under 3.

An earlier draft of this section said 17170 bytes at 95.24%, measured before W4
existed. That build had the pools DERIVED correctly and NOT DELIVERED: the
inventory said 10/14/0 and the zenoh session was compiled with 8/8/8. The
number understated the saving and, worse, described an image with eight
subscriber slots for ten subscriptions -- a RUNTIME rejection, invisible at
link. It is corrected here rather than left standing, and W4 is why it was
found.

## THE RULE W1 COST, and it binds the rest of this phase

**The `-1` DERIVE sentinel is safe only where the consumer supplies its own
default.** `_nros_resolve_derivable_knob`'s rung 4 deliberately leaves a knob
UNRESOLVED so the reading build script falls to its own literal
(`env_usize("ZPICO_MAX_SUBSCRIBERS", 8)`) -- "the one place that literal is
written". A C compile definition has no such literal. An unresolved knob
expands to nothing, `-DZPICO_MAX_SUBSCRIBERS=` reaches the compiler, and
`zpico.c` reports `flexible array member not at end of struct` on a struct
nobody edited.

It only appears once a knob GAINS the sentinel, because before that Kconfig
always carried a number. Every knob W2 converts must therefore be checked for a
consumer with no default of its own, and that check belongs before the switch,
not after.

## THREE DELIVERY FAILURES, and what they say about W4

W1's derived values were RIGHT at every step -- 10, 10, 14, 0 sat correctly in
the fragment throughout. All three failures were in DELIVERY, and every gate
stayed green through all three, because the gates check the inventory and the
resolver and all three failures were downstream of both:

1. **A second consumer.** The zpico C defines read raw `CONFIG_*`, bypassing the
   resolver, and ran BEFORE it. Symptom: `size of array 'subscribers' is
   negative`.
2. **A name that resolved to empty.** A `foreach` building
   `NROS_DERIVED_${_pool}` produced `NROS_DERIVED_NROS_MAX_SUBSCRIBERS`, which
   names nothing; CMake yields EMPTY for an unknown name rather than failing.
3. **A consumer with no default**, the rule above.

W4 as first written -- "every published symbol has a consumer" -- would have
caught NONE of them. The symbol had a consumer in all three cases; a different
consumer was reading around it, or the name never matched, or the value was
legitimately absent. **The gate that catches all three asserts, per knob, that
the value reaching the COMPILE equals the value the resolver produced.** That is
the form W4 should take, and it is now backed by three instances rather than by
the argument that opened this phase.

## W2 — the ones with no derivation, and what each is blocked on

Named here so nobody re-audits them, with the blocker rather than a shrug.

| knob | island | blocked on |
| --- | ---: | --- |
| `NROS_EXECUTOR_ARENA_SIZE` | 40960 | **phase-403 step 3**, which is blocked on step 2 (QoS depth). Depth MULTIPLIES the bound — 86108 B at depth 10 against 24516 at depth 1 for the same ten subscriptions — and the entity record carries `kind`, `type_name`, `name` and no depth. Defaulting is the worst option: ROS's default is 10, so assuming it inflates tenfold and assuming 1 under-sizes in the unsafe direction |
| `CONFIG_MAIN_STACK_SIZE` | 16384 | needs frame analysis, not an inventory. **phase-409** established the method (`objdump`, summing every `sub`/`sub.w`/`subw sp`) and the numbers for one call chain; nothing turns that into a knob |
| `NROS_ZEPHYR_HEAP_SIZE` | 94208 | runtime allocation. No static model, and the honest first step is a high-water reporter, not a derivation |
| `NROS_GRAPH_CACHE_SIZE` | 4096 | sized by the PEER graph. Not a property of this image and probably never derivable from it |
| `NROS_MAX_LIVELINESS` | 32 | same — remote peers |
| `NROS_ZEPHYR_TASK_SLOTS`, `..._TASK_STACK_SIZE` | 5, 8192 | transport tasks, not entities. Derivable in principle from the transport's own declaration; nothing declares it today |

`NROS_SUBSCRIBER_LARGE_SIZE` is a seventh case and a different one: it is
DELIBERATELY not derived (see `NanoRosMessageBounds`), and on the island it now
sizes nothing because `MAX_LARGE_SUBSCRIBERS` derives to 0. A number that sizes
nothing should be deleted from a `.conf`, not derived.

## W2 — `NROS_EXECUTOR_MAX_NODES`. LANDED 2026-09-03

Wired, after the blocker was REMOVED rather than worked around. This phase first
said not to derive it: a bridge creates two nodes whose names are runtime
strings, declared nowhere, so `COMPONENT_COUNT` is a lower bound and
under-counting halts the board.

What changed the answer is the failure MODE, not the count. A short node table
raises `NodeError::NodeTableFull`, which NAMES the knob -- the same property
that justified deriving `MAX_CBS`, where the shortfall surfaces as
`ExecutorFull`. Every consumer had that property except one: the bridge mapped
it to a bare `NROS_RMW_RET_ERROR` and lost the name.

So the bridge was fixed FIRST. Both `create_node_on` calls in
`nros_pubsub_bridge_create` now match `NodeTableFull` and say which knob to
raise, through `nros_log::nros_error!` rather than `eprintln!` (issue 0589 --
std stdio SIGSEGVs a Zephyr native_sim image). Only then is deriving the knob
safe rather than hopeful: every path that can exhaust the table names it.

The derivation is one node per declared component, asymmetric on purpose:

* OVER-counts if two components share a node name, because slots are keyed by
  name and a repeat reuses its record. The safe direction.
* UNDER-counts only for a bridge, which is now legible at the point it fails.

Island: 6 hand-set -> 4 derived. DTCM 89320 (68.15%) -> 86896 (66.30%).

**A THIRD instance of one shape.** `check-knob-delivery` caught
`NROS_EXECUTOR_MAX_NODES` derived as 4 and resolved as `-1`: the knob was
resolved TWICE in `nros_cargo_build.cmake`, and the second call passed the raw
Kconfig value, which is the DERIVE SENTINEL, so the sentinel won as though
someone had stated it. `NROS_RMW_SUBSCRIBER_SLOTS` was the second instance and
a zpico define the first.

The rule that falls out: **converting a knob to derivable means REMOVING its old
`_nros_resolve_knob` call, not just adding a derivable one.** That file has a
long tail of plain calls and the duplicate is invisible to every gate except
this one. A check that no knob is resolved twice in that file would make the
rule enforceable rather than remembered; it is not built here.

Worth the contrast, since it is the argument for having built W4 first:

| | W1, before the gate | W2, with it |
| --- | --- | --- |
| delivery bugs | 6 | 1 |
| found by | build failures, then the gate after merge | the gate, in seconds |
| reached `main` | yes -- 8/8/8 shipped | no |

## W3 — the arena, once phase-403 step 2 lands

The arena is the single largest hand-set number on the island (40960 B) and the
last big one. It is listed here rather than duplicated: **phase-403 owns steps 2
and 3**, and this phase consumes them. When depth reaches the entity record,
W3 is the consumer that turns it into `NROS_DERIVED_EXECUTOR_ARENA_SIZE`.

Ordering is not a preference. An under-sized arena halts during entity creation,
BEFORE the first spin, so 0900's advisory never prints — the failure cannot
report itself. `MAX_CBS` was the right first consumer for the mirror-image
reason: it fails at registration with `ExecutorFull`, which names the knob.

## W4 — the delivery gate. LANDED 2026-09-03

`scripts/check-knob-delivery.py`. For each knob, the number handed to the
COMPILER must equal the number the resolver decided. Read from `build.ninja`
rather than from cmake state, because that is the last artifact before the
compiler and so cannot agree with cmake while disagreeing with the build.

It asserts two identities: a DERIVED value must survive the trip to the
resolver (the fragment's number equals `NROS_RESOLVED_*`), and every C define
must equal its resolved knob, never be empty, and never hold two different
values in one build.

**What it caught, immediately, on work already pushed.** W1 produced SIX
delivery failures and every other gate stayed green through all of them,
because the others check the INVENTORY and the RESOLVER and all six were
downstream of both:

1. a second consumer -- the zpico C defines read raw `CONFIG_*`, before the
   resolver ran
2. a name that emptied -- a `foreach` built `NROS_DERIVED_NROS_MAX_SUBSCRIBERS`,
   which names nothing; cmake yields EMPTY rather than failing
3. no consumer default -- rung 4 leaves a knob UNRESOLVED for a Rust build
   script with its own literal; a C define has none
4. **a loader whitelist** -- `_nros_load_derived_entity_inventory` exports a
   NAMED list, so a symbol the fragment sets and the list omits dies at the
   function boundary
5. **a knob resolved TWICE** -- the second call passed the raw Kconfig value,
   which is the `-1` DERIVE SENTINEL, and it won as though someone had stated it
6. the sentinel then reaching the build as `-1`

Numbers 4 and 5 were LIVE IN PUSHED WORK and produced the mis-measurement
corrected above. The gate names the knob and the mechanism; the compiler named
a struct nobody edited, or nothing at all.

The gate as this phase FIRST specified it -- "every published symbol has a
consumer" -- catches none of the six. In every case the symbol had a consumer.

**Self-test only in the fast tier**, because the end-to-end assertion needs a
CONFIGURED build dir and the fast tier has none. Six cases, one modelled on
each real failure, and each asserts a failure the gate must catch so a gate
that stopped matching cannot report success. Point it at a real dir by hand:

    python3 scripts/check-knob-delivery.py <build-dir>

Verified against the island both ways: it FAILS on the pre-fix build naming all
four dropped knobs, and passes on the fixed one.

## W6 -- the instrument, because the campaign ran out of oracle. LANDED 2026-09-04

W1 through W4 all assume a way to tell a correct image from a broken one. On
the island there is not one.

The six derived knobs were unpinned, the graph came up degraded, and the
bisect that followed could not converge because the signal itself was not
reliable: **one unchanged configuration produced 4, 0, 0, 4, 4 nodes across
five runs.** Three hypotheses were raised and each disproved by measurement --
the arena (the 40,960 image failed identically), `MAX_CBS` (the restore trial
gave 0 after giving 4), and a printk sink of my own (removing it changed
nothing). None of those retractions was cheap, and all three were caused by
reasoning against a 60%-reliable instrument rather than by reasoning badly.

So the board `.conf` is currently pinned back to hand-set values with that
evidence recorded in comments, and **acceptance 1 below is NOT met.** That is
the honest state: the numbers derive, and nobody can say whether the derived
image is right.

### Why every stream was already disqualified

| channel | why not |
| --- | --- |
| console UART | `lpuart0`, not wired on the MR-CANHUBK344 |
| second UART | `lpuart2` carries the zenoh serial transport |
| `nros_log` | therefore reaches nothing -- including BOTH of issue 0900's arena diagnostics |
| SEGGER RTT | tried; could not discriminate. Working and derived images both printed only the Zephyr banner, and a deliberate positive control produced nothing |
| semihosting | halts the core until a probe answers, and FAULTS with none attached, so the image cannot run standalone |

What those share is that they are STREAMS: they need the board still running,
and somebody attached at the moment the interesting thing happens. The failure
this phase is chasing is the opposite shape -- an under-sized arena halts
DURING entity creation, before the first spin, so the advisory that would
explain it never runs. **W3 is blocked on this and not on arithmetic**, which
is why it is still open with its derivation already written.

### What landed

`packages/core/nros-node/src/boot_report.rs` -- a fixed 60-byte record in RAM,
not a stream. Fifteen `AtomicU32`s: the header, the stage boot reached, the
knobs the image was COMPILED with, and the arena allocation that did not fit.
Read back by halting the core and dumping memory, with the address resolved
from the ELF's own symbol table so a relinked image cannot be read at a stale
one:

    python3 scripts/read-boot-report.py build/zephyr/zephyr.elf dump.bin

A PARTIAL record is the useful case rather than a lost one, which is the whole
difference from a log: the last stage reached and the failed allocation are
exactly what names the knob to change.

Opt-in (`NROS_BOOT_REPORT=1`, Zephyr `CONFIG_NROS_BOOT_REPORT=y`), so an image
that does not enable it is byte-identical -- the rule issue 0900's arena knob
and phase-403's `rx_buffer_from_type()` both keep.

**The compile-time half is the part this phase needed most.** W4's
`check-knob-delivery.py` asserts that the number reaching the compiler equals
the number the resolver decided, read from `build.ninja`. The record asserts
the same identity one step further along -- from the silicon. Six delivery
failures reached `main` before W4 existed; this is the arm that would catch a
seventh that survives the build system entirely.

### Found while building it

[#1036](../issues/1036-arena-exhaustion-is-half-silent-and-wholly-unreachable.md):
`arena_alloc_with_trailing` reported NOTHING on failure while its sibling
`arena_alloc` has named the knob since issue 0900. Half of arena exhaustion was
silent -- and it is the half carrying buffered subscriptions and action
entries, which is what an island image actually allocates. Fixed with W5.

### What W6 does NOT settle

No cell builds an image with the record on, exhausts its arena on purpose, and
asserts the dump names the shortfall. The instrument is verified in every part
except the one that runs on silicon, which is the same shape as the defect it
exists to find. Filed in #1036 rather than claimed here.

## Acceptance

1. The island board `.conf` states no NROS count or size that W1 covers, and the
   image builds and boots with every one of them derived.
2. Each W1 knob's derived value is READ FROM THE BUILD and compared against the
   hand-set number it replaces — the value, never the exit code. A cap or a join
   that silently does nothing has misled this campaign twice already.
3. W2's table is in the board `.conf` as a comment, so the next person reads the
   blocker instead of re-deriving the audit.
4. W4's gate fails on a deliberately unread published symbol, proving it can.

## FIXED 2026-09-03: the first configure used to derive the wrong basis

`docs/issues/archived/0991`. On a CLEAN build dir the payload classes derived
over the linked closure, because W9's producer runs later in the configure than
W8's reader. On the island that over-approximation overflowed RAM by 103160
bytes at LINK.

The lag was supposed to close itself — three call sites said
`CMAKE_CONFIGURE_DEPENDS` made ninja re-run cmake once the producer wrote
different bytes. **It never did, and measurably so**: `build.ninja` is written
after the fragment, so the dependency is never stale. Not on that build, not on
the next; only an explicit re-configure moved it. Every "configure twice"
instruction in this phase was working around that, without anyone knowing it was
the whole mechanism rather than a slow one.

`cmake/NanoRosReconfigure.cmake` now closes it inside the same
`west build`/`cmake --build`, at both producers, gated by
`just check reconfigure-on-change` (whose control case reproduces the old bug).
So a measurement no longer has to state how many times its build dir had been
configured — though a number read off a build dir of unknown history still
should be re-measured, since the numbers already recorded above were taken under
the old rule.

## W5 — the cargo carrier, scoped by measurement (issue 0827, 2026-09-04)

Every knob this phase derives reaches CMake-configured images only. A
cargo-built leaf — every native example, every fixture this repo tests against —
takes crate defaults, and on the smallest possible image that is **279,176 bytes,
67.2% of its attributed static RAM** (issue 0827, re-measured at HEAD and
byte-identical to its 2026-09-01 figures).

Two things are settled, both measured rather than argued:

* **The transport works.** `.cargo/config.toml`'s `[env]` reaches the pool
  build scripts: 415,469 -> 136,293 bytes on `examples/native/rust/talker` with
  nothing in the shell environment and no CMake. So this is not "build a second
  derivation path" — it is publishing the existing derivation into a carrier
  that already exists.
* **The value cannot be derived inside the cargo graph.** `nros-rmw-zenoh` is a
  DEPENDENCY of the leaf and is compiled before it; the entities are declared in
  the leaf's own `src/lib.rs`. Nothing leaf-side can reach backwards across that
  edge, and `nros ws entity-inventory` reads a `nros-metadata.json` that only a
  CMake configure writes.

The place to write it is the gitignored `nros sync` sidecar, not the tracked
`.cargo/config.toml` — same split as `[patch.crates-io]` (issues 0457/0463):
generated content is regenerable and uncommitted, authored content is tracked.
Hand-setting the numbers in each example was declined by 0827 and stays
declined; the objection is to the AUTHORSHIP, not the location.

**What is NOT settled, and is the actual W5 question:** what a standalone leaf
derives FROM. A CMake image answers it with `nano_ros_node_register(... ENTITIES
…)`; a cargo leaf states its entities in executable Rust, which is not readable
at the time the number is needed. Answering that is a design decision — declare
entities somewhere sync can read, or accept a stated knob for standalone leaves
— and it is this phase's to make.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0991](../issues/archived/0991-a-clean-build-of-an-entity-declaring-image-does-not-link.md) | a clean build of an entity-declaring image derived the WRONG payload basis and did not link — **RESOLVED 2026-09-03**; the recovery every reader relied on had never fired |

