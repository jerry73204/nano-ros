---
id: 1084
title: "The declared-QoS-depth static_assert survives its producer -- the renderer,
  the header pickup and the compile test all still work, and nothing writes a
  table any more"
status: resolved
type: bug
area: cli, codegen
severity: low
related: [phase-403, phase-412, issue-0965, issue-0196, issue-1033, issue-1119]
---

## What happened

phase-403 step 2 carried a declared history depth to the COMPILER, so a
`NROS_SUBSCRIBE` call site whose QoS disagrees with the declaration beside it
fails the build rather than surprising someone at runtime. The chain was:

```
nano_ros_node_register(ENTITIES sub:std_msgs/msg/Int32:/chatter@depth=20)
  -> nros-metadata.json
  -> nros ws entity-inventory --component <pkg>::<name> --output-header
  -> <build>/nros-declared-qos/<component>/nros/nros_declared_qos_generated.h
  -> target_include_directories(<lib> PRIVATE ...)
  -> nros/declared_qos.hpp picks it up with __has_include
  -> NROS_SUBSCRIBE static_asserts against it
```

phase-412 retired `ENTITIES`, which was the FIRST link and the only producer.
Every other link stayed intact and tested, so what regressed was reachability:
the issue-0196 shape, a mechanism verified in every part except the one that
would run.

## The producer, restored

The depth was already in the model -- a contract states it as
`qos: { depth: N }` on a subscriber endpoint, the resolver puts it at
`contracts.sub_endpoints[*].qos.depth`, and `EntityInventory::from_model` reads
it into `EntityDecl::depth`. What was missing was a place to RENDER it, and the
two objections recorded above `_nros_emit_declared_qos_header` were real:

* **ORDERING.** `nano_ros_entry()` runs after every register call, so a table
  written there reaches a component's TUs one configure late.
* **KEYS.** The table is keyed `(type, topic)`, and two components in one image
  may subscribe to the same topic at different depths.

Both are answered by SPLITTING the one function in two, because the two halves
need different moments and only one of them needs a model:

* `_nros_declared_qos_arm(<component> <target> <lang>)` runs inside
  `nano_ros_node_register()`, in the directory scope that created the target. It
  computes `<binary dir>/nros-declared-qos/<component>` -- a pure function of
  `(PROJECT_NAME, component)`, which needs no model -- creates it, puts it on
  the target's PRIVATE include path, and schedules the render with
  `cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR})`.
* `_nros_emit_declared_qos_header(<pkg> <component> <dir>)` is that deferred
  call. Deferred to the TOP-LEVEL directory, it runs after every
  `add_subdirectory()` and therefore after `nano_ros_entry()`, which by then has
  recorded its resolved model through `_nros_declared_qos_record_model()`.

So ORDERING costs no extra configure pass: the include path exists before
generate, the header lands in it later in the SAME configure, and no compile
happens until configure is over. Nothing arms `nros_reconfigure_on_change` --
a generated header is an ordinary compile input that ninja tracks through the
depfile, so an edited table reaches the next COMPILE by itself. (Issue 1119
measures the reconfigure chain converging a pass late; a pass this did not need
is a pass not spent.)

KEYS is answered by `--component`, which already existed: the table is rendered
from an inventory narrowed to one row, so two components sharing a topic get two
different tables.

## What had to change besides the wiring

**`from_model` was keying the rows on the wrong string.** A contract addresses a
subscription as `/ns/node/<local name>` and wires it to an absolute topic;
`EntityDecl::name` is documented as "the topic / service / action name" and every
consumer reads it that way, but `from_model` recorded the ENDPOINT REF. On the
island that is `/mrm_handler/emergency_stop_status` against
`/system/mrm/emergency_stop/status`, and only the second is what a call site
writes -- so a table built from the model would have matched no call site in any
image and every assertion over it would have been green and vacuous. `from_model`
now records the topic (and, for the same reason, the service / action name).

**`qos: { depth: 0 }` was not refused.** The `ENTITIES` grammar rejected
`@depth=0` outright, on the ground that `KEEP_LAST(0)` holds no sample so a zero
is a typo for "I did not want to say". That rule did not travel to the contract:
a zero parsed, reached `EntityDecl::depth` as `Some(0)` and would have rendered a
row that fails the build at every call site on that topic. `nros ws
entity-inventory` now refuses a model that states one, naming the endpoint.

## Limits, stated

* **The table is keyed on the RESOLVED topic**, which is the string a call site
  writes only when the call site spells the topic absolutely -- which every
  `NROS_SUBSCRIBE` in this tree and on the island does. A call site whose topic
  is relative, or that a launch `<remap>` moves, gets NO row and is checked
  against nothing. That is the safe direction (absence never asserts) but it is
  a false NEGATIVE, and the honest statement is that the check covers absolute,
  unremapped call sites.
* **A configure that resolves more than one SystemModel abstains.** Several
  entries may share one configure and nothing here decides which system
  describes a given component; a table picked from one of two candidates would
  assert against call sites the other one sizes. The renderer says so with a
  `STATUS` message rather than guessing.
* **No image in this tree declares a depth yet**, the island included -- its
  contract states rates and endpoints, not `qos:`. So the capability is now
  REACHABLE and still unexercised by any shipping image. Adding one `qos:
  { depth: N }` to a contract is all it takes.

## Do not

Do not restore `ENTITIES` for this. It was retired because a per-component list
hand-maintained beside the code went wrong on real hardware (the island's
mrm_handler declared six subscriptions for a node that creates seven), and a
depth attribute on that list has the same defect for the same reason.

## Gate

`just check declared-qos-header` (`tests/cmake-declared-qos-header-tests.sh`)
drives the real seam in a real configure with the real CLI, and now carries the
negative control this issue was filed about: it COMPILES a call site that
disagrees with the header that configure just rendered and requires the build to
fail, naming the topic and both numbers -- with the agreeing call site compiled
first, so a table matching nothing cannot read as a pass. It also asserts the
producer is WIRED (`nano_ros_node_register` arms it, `nano_ros_entry` feeds it a
model), because every other case drives the seam directly and would stay green
with the call deleted. That is the state this issue recorded.
