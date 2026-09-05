---
id: 1084
title: "The declared-QoS-depth static_assert survives its producer -- the renderer,
  the header pickup and the compile test all still work, and nothing writes a
  table any more"
status: open
type: bug
area: cli, codegen
severity: low
related: [phase-403, phase-412, issue-0965, issue-0196]
---

## What happens

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
Every other link is intact and still tested: the renderer is exercised by
`the_committed_compile_fixture_is_what_this_emitter_renders`, and `check-cpp`
compiles three agreeing cases plus one deliberately disagreeing case that must
fail. `_nros_emit_declared_qos_header` is still there and
`tests/cmake-declared-qos-header-tests.sh` still drives it through a real
configure.

What is gone is any path by which a REAL component gets a table. No image in
this tree declared a depth -- the only `@depth=` in the repo was always the
compile fixture's own `entities.json` -- so nothing regressed in behaviour.
What regressed is reachability, and that is the issue-0196 shape: a mechanism
verified in every part except the one that would run.

## What a producer would look like

The depth is already in the model. A contract states it as
`qos: { depth: N }` on a subscriber endpoint, the resolver puts it at
`contracts.sub_endpoints[*].qos.depth`, and `EntityInventory::from_model` reads
it into `EntityDecl::depth` (tested:
`a_declared_depth_reaches_the_inventory_and_an_undeclared_one_stays_absent`).
So the DATA is there and the RENDERER accepts it. The open question is WHERE to
render.

`_nros_emit_declared_qos_header` runs inside `nano_ros_node_register`, and the
comment above it gives two reasons that was the right place: ORDERING (the
entry runs after every register call, so a table written there reaches a
component's TUs one configure late) and KEYS (the table is keyed `(type,
topic)`, and two components in one image may subscribe to the same topic at
different depths, which an image-wide table cannot represent). A model-driven
producer has to answer both. The ordering half may already be answered --
`nros_reconfigure_on_change` exists for exactly the one-configure-late problem
and the entity inventory itself relies on it (issue 0991) -- but the register
call also does not know which bringup, and therefore which model, its component
belongs to.

## Do not

Do not restore `ENTITIES` for this. It was retired because a per-component list
hand-maintained beside the code went wrong on real hardware (the island's
mrm_handler declared six subscriptions for a node that creates seven), and a
depth attribute on that list has the same defect for the same reason.
