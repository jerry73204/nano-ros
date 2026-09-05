---
id: 963
title: "The derived-bound inventory has readers now — what is left is the executor arena alone (was: nothing reads it)"
status: resolved
area: codegen, memory, build
severity: medium
related: [0896, 0900, 0939, 0965, phase-403, phase-403-W6, phase-403-W8, phase-412]
---

## Retitled and downgraded 2026-09-03

The title asserted that NOTHING reads the inventory. That stopped being true on
2026-09-02 and the body already said so in two dated sections — the front matter
did not, so every reader of the issue list saw the original claim at HIGH.

Readers that exist on `main`: `nros_derive_message_bound_knobs()`
(`cmake/NanoRosMessageBounds.cmake:304`), joined against the entity inventory at
`cmake/NanoRosCodegenCore.cmake:853-856`, and the Zephyr precedence ladder
`_nros_resolve_derivable_knob()` (`zephyr/cmake/nros_cargo_build.cmake:276`)
applying it to `NROS_SUBSCRIBER_BUFFER_SIZE`, `NROS_MAX_LARGE_SUBSCRIBERS`,
`NROS_SUBSCRIBER_LARGE_SIZE`, `NROS_EXECUTOR_MAX_CBS` and
`NROS_SUBSCRIPTION_BUFFER_SIZE`.

Of the three consumers this issue asked for, 1 and 3 are DONE (`afeab01d5`,
`36622cadd`). **Item 2, the executor arena, is the only one left** — and it is
in flight in PR #244, gated on #239, not unowned.

The seam is one thing, not two: 0963 supplies the per-TYPE size, 0965 the
per-IMAGE entity list, and a knob needs the JOIN. Their remaining item is
literally the same one. 0961 is NOT part of it — that is stack, not arena, and
phase-409 severed the last coupling (`size_of::<Executor>()` is 1016 at both the
shipped knobs and at `MAX_CBS=36`).

Severity `high` → `medium`: what remains is one knob with an owner, not a
mechanism that does not exist.

# The build knows every number and cannot say any of them

## What exists

phase-403 W6 exports each package's derived bounds over three transports from
one data model (`rosidl_codegen::bounds::BoundInventory`):

* `<gen>/nros_message_bounds.json` -- canonical
* `<gen>/nros_message_bounds.cmake` -- an `include()`able fragment, written at
  configure time so it exists before anything downstream configures
* a generated `build.rs` + `links` key, so a dependent reads
  `DEP_NROS_MSGS_<PKG>_BOUNDS_JSON`

Measured on the island entry: 11 inventories, 84 types, 60 bounded.

## What consumes it

**Nothing, when this was filed. That is no longer true — measured 2026-09-02.**

`nros_cargo_build.cmake` loads the fragments (`_nros_load_derived_message_bounds`)
and resolves `NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE`, `..._SUBSCRIBER_LARGE_SIZE`,
`..._MAX_LARGE_SUBSCRIBERS` and `..._SUBSCRIPTION_BUFFER_SIZE` through
`_nros_resolve_derivable_knob`, the same ladder `NROS_EXECUTOR_MAX_CBS` uses.
phase-403 W8/W9 built that while this issue stayed open.

**The transport works end to end.** On `examples/workspaces/mixed`, after a full
build, a reconfigure finds 10 `nros_message_bounds.cmake` fragments and reads
them. The one-configure lag the fragments carry ("still a build-time output ...
the numbers apply from the next configure") RESOLVES; it is not a permanent
stall.

## The blocker moved: the data refuses, and the refusal is right

What the consumer now prints on that workspace is not silence but a reasoned
abstention:

```
nros: message-bound sizing REFUSED -- every size knob keeps its configured value.
  16 of 35 types in the linked interface closure carry no bound:
    action_msgs/msg/GoalStatusArray (unbounded): status_list (sequence<T>)
    example_interfaces/msg/String (unbounded): data (string)
    example_interfaces/msg/*MultiArray (unbounded): layout.dim, data
    ... 16 total
  Deriving a class size over only the bounded types would publish a maximum a
  real sample can exceed, which is a SILENT BufferTooSmall drop on the C/C++
  arena dispatch path.
```

Fifteen of the sixteen are `example_interfaces` — the `*MultiArray` family,
`String`, `WString`, `MultiArrayDimension`, `MultiArrayLayout` — plus
`action_msgs/GoalStatusArray`. These are unbounded in their own `.msg`
definitions, so no amount of wiring fixes them.

**So the next action is not "wire a consumer".** It is one of:

* **bound them** in `nros-codegen.toml` (RFC-0033 `cap`). This bullet used to
  say the option "runs into [issue 0962]" — that it is blocked because nested
  bounded sequences cost the PRODUCT of their caps. **0962 is resolved and was
  never the blocker this implies** (2026-09-04): a UNIFORM cap indeed does not
  terminate, but a uniform cap was never the remedy. phase-403 W7b landed a
  diagnostic that names the worst nesting chain and its factors, and capping ONE
  level of that chain divides the whole product. 0962's pathological case is
  `visualization_msgs` nesting five deep; the types blocking THIS issue are the
  `example_interfaces` `*MultiArray` family, `String`, `WString` and
  `action_msgs/GoalStatusArray`, which nest one or two levels. See
  [archived/0962](0962-nested-bounded-sequences-cost-the-product-of-their-caps.md);
* **narrow the closure**, so an image pays only for the types it actually links
  rather than everything `example_interfaces` ships — the `mixed` workspace uses
  a handful of these and the closure carries all sixteen;
* or **accept the refusal** for images with unbounded types and let the knobs
  stay hand-set there, which is what happens today and is at least honest.

Whoever takes it should re-read this section rather than the one above: the
"nothing consumes it" framing sent this issue's severity to `high` and is now
the wrong problem.

## What that costs, measured rather than argued

Every size downstream is still set by a human, and on one bring-up every one of
them was set wrong at least once:

| knob | how it was actually set |
| --- | --- |
| `NROS_EXECUTOR_ARENA_SIZE` | SIX bisections against a board, flashing each |
| `NROS_EXECUTOR_MAX_CBS` | counted by grep: 17, when the answer was 33 |
| `NROS_MAX_LARGE_SUBSCRIBERS`, `NROS_SUBSCRIBER_LARGE_SIZE` | read off generated C++ headers by eye |
| `NROS_SUBSCRIBER_BUFFER_SIZE`, `NROS_MAX_SUBSCRIBERS` | ditto |
| W4's payload class boundaries | blocked ON this inventory; the wave says so |

The grep failure is worth keeping: `MAX_CBS` counts total HANDLES, and most
subscriptions are the `NROS_SUBSCRIBE` macro rather than a literal
`create_subscription`, so grepping the obvious string found 1 of 11
subscriptions and none of the 6 timers.

Issue 0900 W1 landed the runtime half -- `arena_used()` / `arena_capacity()` and
a first-spin advisory naming the value to set. It cannot help in the case that
matters: an image whose arena is too small halts during entity creation, which
is BEFORE the first spin, so the advisory never prints.

## What would resolve it

A consumer per number, each reading the inventory rather than a human:

1. `MAX_CBS` from the entity count the model and codegen already know.
2. The executor arena from the entities that will occupy it -- phase-403 W5's
   remaining half, and 0939's multiplication makes a derived total more
   valuable, not less.
3. zenoh's payload class boundaries from the distinct sizes present, which is
   W4's stated goal and its stated blocker.

The transports exist and are verified end to end (a CMake `-P` run over five
packages composed 97 types and derived `NROS_MAX_LARGE_SUBSCRIBERS=3` /
`NROS_SUBSCRIBER_LARGE_SIZE=364`). What is missing is the reader.

## Partly fixed 2026-08-31 (phase-403 W8): consumer 3 of 3 exists

Still `open`, and the remaining half is named below rather than implied.

**What landed.** `cmake/NanoRosMessageBounds.cmake` composes every package's
fragment and derives the size knobs. `nros_find_interfaces()` runs it over the
image's whole interface closure and writes the answer plus its provenance to
`<build>/nros/message_bound_knobs.cmake`; the Zephyr knob resolver reads that,
with `-1` as the Kconfig spelling of "nothing here chose a number" and a
precedence ladder of environment > Kconfig / board `.conf` > derived > crate
default. Item 3 of "What would resolve it" -- zenoh's payload class boundaries
-- is done, plus `NROS_SUBSCRIPTION_BUFFER_SIZE`.

Measured on the island entry (11 packages, 84 types, regenerated from the args
files its build dir already holds): `NROS_MAX_LARGE_SUBSCRIBERS` **2 -> 0**,
`NROS_SUBSCRIBER_LARGE_SIZE` **2560 -> not derived**, because nothing in the
closure exceeds the 2048 B class split. The hand-set pair reserved
`2 x 4 x 2560 = 20,480` bytes of `.bss` for a class no type can route into.
It also found `CONFIG_NROS_SUBSCRIPTION_BUFFER_SIZE=512` against a derived
`nav_msgs/Odometry` bound of 880 -- an undersized take buffer on the image's
own largest subscribed type, whose failure mode is a silent drop.

**What is still uncovered, and why it is not a patch away.** Items 1 and 2 --
`NROS_EXECUTOR_MAX_CBS` and the arena -- are questions about WHICH ENTITIES AN
IMAGE CREATES, and this inventory answers only what every TYPE'S SIZE is. A
package's type count is not an image's entity count. Deriving one from the
other would produce exactly the plausible-wrong-number this issue exists to
remove, so W8 declined to.

A second source would have to supply, per image: the number of subscriptions,
publishers, timers, service servers/clients and action entities, each bound to
a type NAME the inventory can then price. `entity_facts.rs::describes_wiring`
is the extension point and abstains on all 115 resolved SystemModels today, and
the RFC-0043 C++ components register in constructors at runtime, so the wiring
would have to come from codegen at component-registration time or from an
author-stated manifest -- not from the resolved model as it exists.

**The same gap sets the price of what did land.** The derived numbers are
UPPER BOUNDS: the closure is what the image LINKS, not what it subscribes to.
On the island the derived `1496` comes from `std_msgs/Float64MultiArray`, which
it never receives, against `880` for its real worst case -- `29,568` B of
`SMALL_PAYLOADS` and `66,528` B of arena, spent on types nothing reads. Safe in
direction, expensive in magnitude, and the entity inventory is what would make
it tight.

## Why this is severity high

A knob nobody can enumerate is a knob nobody sets, which is the 0271 / 0739
shape this repo keeps rediscovering. The difference here is that the number is
no longer unknowable: it is computed, written to disk in three formats, and
then ignored.

## Update 2026-08-31 (phase-403 W9, issue 0965): item 1 has its source

The "second source" this issue named -- per image, the number of subscriptions,
publishers, timers and service entities, each bound to a type NAME the bound
inventory can price -- now exists: `nros_cli_core::entity_inventory`, composed
from `nano_ros_node_register(... ENTITIES ...)` and read by
`cmake/NanoRosEntityInventory.cmake`. Item **1** of "What would resolve it" is
done: `NROS_EXECUTOR_MAX_CBS` derives.

It also corrects a number this issue repeats. "`MAX_CBS` counts total HANDLES"
is right; "the answer was 33" is not. 33 is the island's ENTITY count and 14 of
those are publishers, which claim no callback slot -- `create_publisher` never
reaches `Executor::next_entry_slot()`. The slot demand is **19** against a
hand-set 36.

Items **2** (the arena) and **3** (the payload classes) are still open, and now
for a different reason than "no entity inventory exists": both need the two
inventories JOINED per subscription -- W8's per-type size against W9's
per-entity list. A total from either half alone is the same plausible wrong
number, so the join is its own work.


## The join landed — and "narrow the closure" is still unreachable, by ORDERING (2026-09-04)

Items 2 and 3 above are marked "both need the two inventories JOINED". **That
join exists**: `_nros_bounds_join_subscribed` in `NanoRosMessageBounds.cmake`,
phase-403 step 1, with `NROS_MESSAGE_BOUNDS_BASIS` reading `subscribed` or
`closure`. And `examples/workspaces/cpp` now declares `ENTITIES` (issue 0965),
so for the first time there is an image that can drive it.

It does not lift this issue's blocker, and the reason is ordering rather than
data. Driving the reader directly over that image's fragments, both ways:

| basis | entity inventory | result |
| --- | --- | --- |
| closure | absent | `refused` — 16 unbounded types in the closure |
| subscribed | present, `std_msgs/msg/Int32` only | **`refused`, identically** |

`NROS_MESSAGE_BOUNDS_BASIS` comes back EMPTY in the second run, which is the
tell: the join never executed. `nros_derive_message_bound_knobs` evaluates the
closure-wide open-type check and `return()`s at
`NanoRosMessageBounds.cmake:481`; the join is called at :512.

So an image whose every SUBSCRIBED type is bounded still refuses, because
something it merely links is not. That is exactly this issue's second remedy —
"narrow the closure, so an image pays only for the types it actually links
rather than everything `example_interfaces` ships" — being unavailable through
the mechanism built to provide it.

**It is not a simple reorder, and the module is not wrong today.** Its header
states the rule deliberately ("it names a type that is `unbounded`/`unresolved`
-> the whole derivation already refused"), and the two knob families genuinely
differ: the take buffer `NROS_SUBSCRIPTION_BUFFER_SIZE` is also `DEFAULT_TX_BUF`
and every raw entity's default, so a type the image only PUBLISHES must fit it,
and it must keep refusing on the closure. Only the three payload-class knobs are
a fact about subscriptions.

### The fix, scoped

1. Run the join BEFORE the closure-wide refusal.
2. On a closure that has open types: refuse the take buffer as today, but
   publish the payload classes if the join derived them. The output format
   already carries `NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS` separately — it is only
   written inside the `derived` branch.
3. Check the consumer: `nros_cargo_build.cmake` gates on the OVERALL status, so
   a "take buffer refused, payload classes derived" answer needs the consumer to
   read the payload status too, or it will ignore values that are present.

Not done here. It changes a documented refusal contract on a path whose failure
mode is a silent `BufferTooSmall`, and it needs the consumer change in the same
commit.

### What IS done: the join no longer trusts an invariant it cannot see

The join reads `NROS_MESSAGE_BOUND_<t>_RX` without ever checking `_STATE`. It is
safe **only** because the closure refusal above it guarantees every type is
bounded — an invariant that lives thirty lines away, in the block step 1 above
proposes to move. The first person to make the payload classes derivable on an
image with an unbounded closure type removes the guard without knowing it was
one, and the failure is a payload class sized from a blank `_RX`.

So the join now checks boundedness itself and refuses naming the offending type,
distinct from the "not in the inventory" refusal beside it (a type this tree
cannot bound is a different problem from one it has never heard of).

**It cannot fire through the public entry point today** — that is the point of
it — so it is tested by calling the join directly: case O in
`tests/cmake-message-bounds-tests.sh`, one assertion that a fully bounded
subscribed set still derives and two that an unbounded one refuses and names the
type. A check that has never executed is indistinguishable from one that does
not work, and the first version of this test passed for the wrong reason: the
join iterates `TYPE_COUNTS`, not `TYPES`, so a fixture listing the unbounded type
in only the latter never visited it.


## The ordering blocker is LIFTED (2026-09-06) — all three steps

The scoped fix above is done, and the third step turned out to be somewhere
other than where it was predicted.

**Step 1 — the join runs before the closure refusal.** It is pure (it publishes
nothing), so computing it early changes no output on the path that then refuses
everything.

**Step 2 — a refused closure still answers the payload classes**, but only on a
real `subscribed` join. That guard is the load-bearing part: the macro's
`closure` basis fallback derives over `_small` / `_large_types`, which on the
refused path were accumulated over the BOUNDED types only. Publishing those
would be exactly the under-derivation the refusal exists to prevent, whose
failure mode is a silent `BufferTooSmall`. So an image with no entity inventory
still gets nothing.

**Step 3 was not where this issue said it was.** The prediction was that
`nros_cargo_build.cmake` "gates on the OVERALL status … or it will ignore values
that are present". It does not: `_nros_resolve_derivable_knob` keys on
`DEFINED ${derived_var}`, and `_nros_load_derived_message_bounds` forwards any
of the five names it finds, neither consulting the status. Read rather than
assumed, and the assumption was wrong.

The real gap was one layer earlier, in `_nros_message_bounds_write_output`: its
`refused` branch wrote the reason and the line "No knob is derived. Every one
keeps its configured value.", and nothing else. **The file IS the transport** —
a knob published in memory but absent from it does not reach the consumer at
all. That branch now emits the payload-class knobs, the basis and the payload
status when the join was `subscribed`, with a comment saying which knob the
refusal still covers and why.

### Held by case P, which asserts the FILE and not just the variables

`tests/cmake-message-bounds-tests.sh` case P builds a closure with an unbounded
type and subscribes only to the bounded one. It asserts `payload=derived`,
`basis=subscribed`, the small class sized from the subscribed type, and that the
take buffer is NOT derived. Then it asserts the same of the OUTPUT FILE — which
is what caught the writer gap: every in-memory assertion passed while the file
carried nothing.

It also asserts the no-inventory case publishes nothing, in memory and in the
file, so the `closure`-fallback hazard has a negative control rather than a
comment.

Mutation-tested three ways, each restoring the old behaviour and each failing
the suite: dropping the `subscribed` guard, moving the join back after the
refusal, and reverting the writer.

92 assertions in the suite (was 83). `just check fast` 230/230.
