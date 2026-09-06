---
id: 1198
title: "`MAX_NODES` and `MAX_SC` are undeclared defaults, so every executor backing
  carries a constant 12,416 B of tables sized for 4 nodes and 8 scheduling
  contexts whatever the image declares"
status: open
type: tech-debt
area: [core, embedded]
related: [1145, 1171, 0827, 0857, 0900, 1061, phase-392]
---

## What

The executor backing is `arena + fixed tables`
(`nros-node/src/executor/storage.rs::compute_offsets`). Four inputs size it:

| input | source | default |
| --- | --- | ---: |
| `MAX_CBS` | **derived** per leaf by `nros sync` (`NROS_EXECUTOR_MAX_CBS`) | 4 |
| `ARENA_SIZE` | **derived** from `MAX_CBS`, `RX_BUF_SIZE`, `ACTION_CLIENTS` | floor 8192 |
| `MAX_SC` | `NROS_EXECUTOR_MAX_SC`, nothing derives it | **8** |
| `MAX_NODES` | `NROS_EXECUTOR_MAX_NODES`, nothing derives it | **4** |

The first two follow the declaration through the issue 0827 / 1061 channel. The
last two do not: no leaf states them, no sidecar derives them, so every image
gets 8 scheduling contexts and 4 node slots regardless of what it declares.

## Measured

Six FreeRTOS Rust leaves, `arm-none-eabi-nm -S` on
`build/cargo-fixtures/freertos/thumbv7m-none-eabi/nros-relwithdebinfo/<leaf>`,
against the `nros_node_config.rs` each `nros-node` build unit emitted:

| leaf | `MAX_CBS` | `ARENA_SIZE` | `EXECUTOR_BACKING` | backing − arena |
| --- | ---: | ---: | ---: | ---: |
| `talker` | 1 | 8,192 | 20,608 | **12,416** |
| `action-server` | 1 | 20,096 | 32,512 | **12,416** |

**The table term is identical to the byte** across leaves whose declarations
differ, because `cbs`, `sc` and `nodes` are the same in both. All per-leaf
variation is the arena. So ~12.4 KiB of every FreeRTOS image's backing is
insensitive to what the image actually contains.

`MAX_NODES` is the heavier of the two: it multiplies **seven** tables, by the
layout's own comment — "one worst-case extra session, extra-session id,
node-sched binding, dispatch slot, component slot and callback-group filter
entry per Node, so ONE count covers all seven".

## Why this is worth more than the pairing it was found under

Issue 1145 is about 20–32 KiB reserved twice on FreeRTOS. This is ~12.4 KiB
reserved **once but for nothing**, in every image on every platform, and it is
the larger and simpler target: a single-node talker pays for four nodes.

It is also the same defect class the campaign has already fixed twice —
issue 0827 (pools sized at the backend, where the entity set is unknown, instead
of at the image, where it is known) and issue 0857 (`ENTITY_BOUNDS` defaulting to
the knob caps because 81 of 99 classes declared nothing). Both were closed by
making the count follow the declaration. `MAX_NODES` and `MAX_SC` are the two
that were left.

## What has to be answered first

**Is the node count knowable at sync time?** `MAX_CBS` is, because the entity
declaration names publishers, subscribers, timers and services. Nodes are a
different axis: an entry may create several `Node`s, and the tiered boot paths
open one executor per tier. Whether the count is derivable from the model or the
contract sidecar, or must be declared by the entry, is the design question — and
it is close to issue 0973's answer (wiring is AUTHORED), so the honest first step
is to find out which artifact already states it.

**`MAX_SC` is scheduling, not entities.** Scheduling contexts come from the tier
model / RFC-0016, not from the entity inventory, so it may belong to a different
declaration than `MAX_CBS`.

## Do not

Lower the defaults by argument. The failure mode is a runtime registration
failure (`NodeError::BufferTooSmall`, the shape issue 0900's arena derivation
already hit) rather than a link error, and phase-392's standing rule is that no
wave claims a saving it did not measure. Whatever number replaces 4 and 8 must
be derived from a declaration or measured on a running image.

## Reproduce

```
bash scripts/build/fixtures-build.sh freertos rust
for f in $(find build/cargo-fixtures/freertos -name nros_node_config.rs); do
    grep -hE 'pub const (MAX_CBS|MAX_SC|ARENA_SIZE|MAX_NODES)' "$f"
done
arm-none-eabi-nm -S <leaf-elf> | grep EXECUTOR_BACKING
```
