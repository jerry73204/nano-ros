---
id: 810
title: "The executor arena is sized at MAX_CBS x sizeof(ActionClient) whatever the
  entries actually are, so every real image ships a hand-picked override and
  undersizing fails at runtime instead of at link"
status: open
type: tech-debt
area: core
related: [phase-392, phase-390, phase-403, 0900, 1190]
---

## Problem

`packages/core/nros-node/build.rs` derives the executor arena from the *worst
possible* entry shape, not from the entries an image actually registers:

```rust
const ACTION_CLIENT_PER_SERVICE: usize = 4096 + 384;
let per_entry = ACTION_CLIENT_SERVICES * ACTION_CLIENT_PER_SERVICE   // 3 x 4480
    + ACTION_CLIENT_FEEDBACK_SUBS * rx_buf_size                      // 3 x rx_buf
    + ACTION_CLIENT_SUB_OVERHEAD;                                    // 1536
let derived_arena = (max_cbs * per_entry + ARENA_BASE_OVERHEAD).max(ARENA_FLOOR);
```

The comment says so outright: *"Subscription / service entries are strictly
smaller, so budget every slot at the action-client size."*

## Evidence

Measured on the mr_canhubk3/s32k344 safety-island image
(`MAX_CBS=14`, `NROS_SUBSCRIPTION_BUFFER_SIZE=1024`), which registers **no
action clients at all**:

| | bytes |
| --- | --- |
| `per_entry` (action-client worst case) | 18,048 |
| derived: `14 x 18,048 + 2,048` | **254,720** (249 KiB) |
| what the image actually ships | **52,224** (51 KiB) |

A **4.9x hand-tuned override**. The derivation is not what sizes any real
board; a human guess is. That guess is unchecked in both directions — too big
wastes RAM silently, too small fails at runtime.

## Why runtime and not link

Kconfig's own help predicts it: *"too small fails at runtime, not at link."*
The failure is `NodeError::BufferTooSmall` on the SECOND node registration, so
an image boots, registers one node, and dies. build.rs already carries a scar
from the same area — issue 0460 made `knob_usize` read `$DOTCONFIG` directly,
which made the `0 = derive` sentinel guard inert and shipped a zero-byte arena
on every defaulted Zephyr image.

## Why it cannot be fixed in build.rs alone

The thing that knows the real entries is a *different stage*:
`packages/cli/nros-cli-core/src/codegen/entry/emit_cpp.rs` already emits one
`__nros_comp_buf_N` per real component class. `build.rs` is a Cargo build
script and cannot see it. So precise sizing means the entry generator emits the
requirement and build.rs consumes it, rather than deriving a bound it has no
information for.

## The worst case was not a worst case (issue 1190, 2026-09-07)

The comment quoted above — *"Subscription / service entries are strictly
smaller"* — was FALSE, and the correction is the opposite direction from this
issue's. A subscription's arena claim scales with its QoS history, and the model
priced that history at three slots (the `depth <= 1` triple buffer) when the ROS
default is KEEP_LAST(10) = eleven slots plus a length array. One ordinary
subscription costs 12,144 bytes, not 3,584. The action-client term's slack —
18,048 modelled against 14,600 measured — absorbed it on every image that
budgets every slot at it, so this issue's over-provision was also what kept that
under-provision invisible.

So the two are one mechanism: a model nobody could check against the allocator.
1190 landed the checkable half — the model is emitted as `config::arena_model`
and each term is asserted to be an upper bound on the entry types it stands for,
in `executor/arena.rs`, where `size_of` and the linked backend's handle are
visible. What remains open here is the other half, sizing DOWN from what the
image really registers.

## Not covered by this issue

Hand-written `main`s create entities at runtime and have no generated entry, so
they cannot be sized statically at all. They need a different mechanism (a
high-water mark, checked in CI). Deliberately left to
[phase 392](../roadmap/phase-392-static-memory-space-campaign.md) rather than
guessed at here.
