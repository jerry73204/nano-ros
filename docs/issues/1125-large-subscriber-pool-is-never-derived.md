---
id: 1125
title: "`ZPICO_MAX_LARGE_SUBSCRIBERS` is never derived, and `LARGE_PAYLOADS` is the biggest symbol left in the esp32 image"
status: open
area: rmw, memory, codegen
severity: medium
related: [0827, 0963, 0896, 1052, 1061]
---

## What

Issue 0827's derivation now sizes a leaf's pools from what it declares, and issue
0963's join sizes the payload CLASSES from what an image subscribes to. Neither
answers `ZPICO_MAX_LARGE_SUBSCRIBERS`.

Measured on `examples/native/rust/talker` with the derived budget applied:

| bytes | symbol |
| ---: | --- |
| **131,072** | `nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS` |
| 83,784 | `g_sessions` |
| 4,504 | `SERVICE_BUFFERS` |
| 4,096 | `SMALL_PAYLOADS` |

`LARGE_PAYLOADS` is `MAX_LARGE_SUBSCRIBERS × RING_DEPTH × LARGE_SIZE` and is the
single largest symbol in that image — larger than everything 0827 recovered
(176,720 B) leaves behind in the same class.

## Why neither derivation reaches it

The entity inventory counts SUBSCRIPTIONS. It has no notion of a *large*
subscriber, because "large" is not a property of the declaration — it is a
property of the TYPE's bound relative to the class ceiling
(`NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING`). That is the message-bound
inventory's question, not the entity inventory's.

Issue 0963's join DOES compute `NROS_DERIVED_MAX_LARGE_SUBSCRIBERS` — over the
SUBSCRIBED types, counting entities rather than distinct types so two
subscriptions on one large type reserve two blocks. So the number exists for a
CMake image whose payload classes derive.

What is missing is the cargo-leaf side: `leaf_entity_env` writes the
`[env]` sidecar from the ENTITY inventory alone, and the message-bound inventory
is not part of that path.

## What answering it needs

The leaf's per-type bounds. `nros codegen` already emits the message-bound
fragments (`NROS_MESSAGE_BOUND_<t>_{STATE,RX}`); the question is whether a cargo
leaf can read them the way the CMake path does, and whether the SUBSCRIBED set is
knowable there — issue 1061's manifest declaration carries the type in
`sub:std_msgs/msg/String:/chatter`, so the input may already be present.

## Do NOT guess it

Same rule as `ZPICO_MAX_QUERYABLES` (issue 1061): a pool short of what the image
receives is not a smaller pool, it is a dropped sample or a refused creation. The
crate default is large and safe. A derived number here has to come from the
bounds, or not be written.
