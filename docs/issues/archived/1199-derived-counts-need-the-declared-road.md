---
id: 1199
title: "Seven more derived pool knobs were computed on every lane and consumed only under `zephyr/` — the DECLARED road now carries the whole set the cargo-leaf road already carries"
status: resolved
type: bug
area: cmake, rmw-zenoh, core, memory
severity: high
found: 2026-09-07
resolved: 2026-09-07
related: [1122, 1125, 1061, 1033, 1015, 0827, 0460]
---

# Three roads, and only two of them were built

A number CMake derives reaches a build script by one of three roads:

| road | who it serves | mechanism |
| --- | --- | --- |
| resolver | Zephyr images | `nros_resolve_knobs()` -> `NROS_RESOLVED_*` |
| sidecar | Rust **leaves** | `nros sync` writes cargo `[env]` (`leaf_entity_env.rs`) |
| declared | CMake images with **no** Rust leaf | `corrosion_set_env_vars` -> `NROS_DECLARED_*` |

Issue 1122 built the third road and put one knob on it. Everything else was
still computed on every lane, written to `entity_inventory.cmake` /
`message_bound_knobs.cmake`, and read only under `zephyr/` — so a FreeRTOS,
ThreadX, NuttX or posix image with no Rust leaf kept every crate default.

## What travels now, and why this set

The set is **not chosen here**. It mirrors `DERIVED_ENV_KEYS` and
`DERIVED_PAYLOAD_ENV_KEYS` in `packages/cli/nros-cli-core/src/leaf_entity_env.rs`
— the same decisions already made for the leaf road. Two roads delivering
different key sets is how an image's sizing comes to depend on which lane built
it, which is 1122 with the roads swapped.

| knob | declared fact | floor |
| --- | --- | --- |
| `ZPICO_MAX_SUBSCRIBERS` | `NROS_DECLARED_MAX_SUBSCRIBERS` | **1**, at the consumer |
| `ZPICO_MAX_PUBLISHERS` | `NROS_DECLARED_MAX_PUBLISHERS` | **1**, at the consumer |
| `NROS_RMW_SUBSCRIBER_SLOTS` | `NROS_DECLARED_RMW_SUBSCRIBER_SLOTS` | none — 0 is the answer |
| `NROS_EXECUTOR_MAX_CBS` | `NROS_DECLARED_EXECUTOR_MAX_CBS` | none |
| `NROS_EXECUTOR_ACTION_CLIENTS` | `NROS_DECLARED_EXECUTOR_ACTION_CLIENTS` | none — 0 is the point |
| `NROS_SUBSCRIBER_BUFFER_SIZE` | `NROS_DECLARED_SUBSCRIBER_BUFFER_SIZE` | none |
| `ZPICO_SUBSCRIBER_LARGE_SIZE` | `NROS_DECLARED_SUBSCRIBER_LARGE_SIZE` | none |

Three names stay off the road, each for a reason the tree already recorded:
`ZPICO_MAX_QUERYABLES` is deliberately not derived (the count excludes the
param and lifecycle service families a FEATURE enables — issue 1061, and the
CMake road completes it through `NROS_DECLARED_INFRA_QUERYABLES` instead);
`NROS_EXECUTOR_MAX_NODES` was withheld from phase-412 W1 because under-counting
halts the board; `NROS_SUBSCRIPTION_BUFFER_SIZE` is absent from the leaf road
too and also feeds the arena derivation, so it is a second decision.

## The two rules this had to respect

**A derived value is a DEFAULT, never an override.** The tree's ladder is
`env > Kconfig/board > derived > crate default`
(`_nros_resolve_derivable_knob`). So every consumer takes the declared fact as
its *default*, and a named knob still wins — verified at each site. Setting the
knob itself through `corrosion_set_env_vars` would have made the override
unreachable, because the child environment is not overridable the way cargo
`[env]` (without `force`) is. That asymmetry is why the two roads spell the
same number differently, and `ROAD_PAIRS` in the gate is where the pairing is
written down.

**The floor belongs to the consumer, not the derivation.** The derivation
publishes DEMAND and zero is a legitimate demand. The two `ZPICO_*` counts size
fixed C arrays in `zpico.c`, where zero is a `#error` (issue 1015); the same
numbers reach pools where zero IS the answer and is worth 33,296 bytes a slot
(issue 1033). So the floor is applied where the knob is named — here in
`nros-zpico-build`'s `declared_floored`, one boundary over from the sidecar's
`c_array_pool_floor`, for the same reason.

## Acceptance — every ladder, measured by building

| | default | declared | named override |
| --- | ---: | ---: | ---: |
| `ZPICO_MAX_SUBSCRIBERS` | 8 | 3 | 9 |
| `ZPICO_MAX_PUBLISHERS` | 8 | 2 | 8 |
| `ZPICO_MAX_{SUB,PUB}` at declared **0** | — | **1** (floored) | — |
| `NROS_RMW_SUBSCRIBER_SLOTS` | 8 | **0** (honoured) | 3 |
| `NROS_EXECUTOR_MAX_CBS` | 4 | 7 | 5 |
| `NROS_EXECUTOR_ACTION_CLIENTS` | 4 | **0** | 5 |
| `NROS_SUBSCRIBER_BUFFER_SIZE` | 1024 | 880 | 333 |
| `ZPICO_SUBSCRIBER_LARGE_SIZE` | 16384 | 4096 | 999 |

Carrier: 8 cases in `tests/cmake-message-bounds-tests.sh` §Q, driven in
`cmake -P` beside the writer. Mutating away either payload guard fails it.

```
just check declared-fact-carriers  10 facts produced, consumed and watched (was 3)
just check message-bound-knobs     100 assertions held
just check entity-inventory-knobs / c-array-pool-floors / knob-delivery / node-std-tests  PASS
just check fast                    260 ran, 1 ledger skip, 0 failed
```

## Three gates caught this work, and each was right

* **`check-declared-fact-carriers` caught itself.** The first entity carrier
  built its names by interpolation (`NROS_DECLARED_${_name}`), so the delivered
  names appeared nowhere in the file as text. That is phase-412's second
  recorded delivery failure — a `foreach` composing `NROS_DERIVED_${_pool}`
  produced a name matching nothing, and CMake yields EMPTY for an unknown
  variable rather than failing. Both names are now written in full.
* Its read rule then had to widen: the new readers take the env name as a
  *parameter*, so `env::var("…")` stopped being where the name appears, and the
  gate reported a wired fact as unconsumed. Widening it over-reached into CLI
  sources, where a `NROS_DECLARED_*` literal is the PRODUCER naming what it
  writes — so the scan is build-side only now.
* **`config-knob-census`** required the three new read idioms be registered and
  the seven new facts CLASSIFIED rather than guessed at.

## Not measured here

The byte delta. Issue 1122 priced the whole set by hand at **−324,416 B** on the
SPE-candidate image and `MAX_LARGE_SUBSCRIBERS` alone at −131,072; the rest of
that is what this delivers, but confirming it needs the out-of-tree consumer and
a cross toolchain, which is 1122's Reproduction section and not something the
in-tree lanes can run. What is proven in-tree is every link in the chain.
