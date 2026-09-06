---
id: 1120
title: "The entity inventory's refusal tells you to add `ENTITIES`, which phase-412 retired and made a hard configure error"
status: open
type: bug
area: cli, docs
severity: medium
found: 2026-09-06
related: [0965, 1121]
---

# Two halves of one phase disagree, and a new user meets the wrong half first

phase-412 retired `nano_ros_node_register(... ENTITIES ...)`. The cmake side
enforces it — `cmake/NanoRosNodeRegister.cmake:1130` is a `FATAL_ERROR`:

```
nano_ros_node_register(heartbeat): ENTITIES was retired (phase-412).
    What this component creates is now stated ONCE PER SYSTEM, in a contract
    sidecar beside the launch file that runs it:
        <bringup>/launch/<stem>.contract.yaml   (beside <stem>.launch.xml)
```

The CLI side did not follow. `entity_inventory.rs:1081` still refuses with:

```
1 of 1 components in this image declare no entities:
    heartbeat_pkg::heartbeat (heartbeat_pkg::Heartbeat)
… Add `ENTITIES ...` to each `nano_ros_node_register()` above --
`ENTITIES NONE` for a component that really creates none.
```

So the tool tells you to write the thing the build will then refuse. Following
that advice is a round trip through a `FATAL_ERROR`.

## Which half a newcomer meets first

The refusal, because it is reachable with a workspace that compiles. Reproduced
on a first out-of-tree consumer
([`orin-spe-heartbeat`](https://github.com/jerry73204/orin-spe-heartbeat)): the
image built, the inventory printed `STATUS "refused"` with the message above, I
added `ENTITIES ...` exactly as instructed, and configure then died on the
retirement error.

## Why it matters beyond the round trip

The refusal is not cosmetic — it is what leaves every pool at its host-sized
default. On this consumer, deriving instead of refusing removed `SMALL_PAYLOADS`
(32,768 B) and `SERVICE_BUFFERS` (35,424 B) from a publish-only image. On a
256 KB target that difference decides whether the image fits, so the message
that stands between a user and the derivation should name the mechanism that
still exists.

## Scope

`entity_inventory.rs` mentions `ENTITIES` 25 times. This is about the ones a
USER reads; the parser legitimately still knows the token, because
`NanoRosNodeRegister.cmake:1115` keeps it PARSED in order to fail loudly on it.
The fix is to make the refusal name the contract sidecar, its path shape, and
what to write for a component that really creates nothing — the cmake error is
the model, and it is good.
