---
id: 1197
title: "FreeRTOS still budgets `configTOTAL_HEAP_SIZE` for an executor backing that
  moved to `.bss`, and neither of the two mechanisms that fixed Zephyr can be
  used here — the board crate cannot see the size, and stating it would fight
  the per-leaf derivation"
status: open
type: tech-debt
area: [embedded, core, build]
related: [1145, 1171, 1146, 0827, 1061, phase-392]
---

## What

phase-392 W6 moved the executor's per-entry storage out of a `Box::leak` into
the named `.bss` static `EXECUTOR_BACKING`. On an RTOS the allocator it used to
come out of is itself a fixed reservation, so an image that does not lower that
reservation now holds the same bytes twice. Issue 1145 paired Zephyr; FreeRTOS
is not paired, and this issue is why it was not done by hand.

## Measured

`bash scripts/build/fixtures-build.sh freertos rust`, then
`arm-none-eabi-nm -S` on each `build/cargo-fixtures/freertos/thumbv7m-none-eabi/
nros-relwithdebinfo/<leaf>`:

| leaf | `EXECUTOR_BACKING` | `ucHeap` |
| --- | ---: | ---: |
| `talker` | 20,608 | 2,097,152 |
| `listener` | 20,608 | 2,097,152 |
| `service-server` | 20,608 | 2,097,152 |
| `service-client` | 21,832 | 2,097,152 |
| `action-server` | 32,512 | 2,097,152 |
| `action-client` | 32,512 | 2,097,152 |

So between 20 KiB and 32 KiB per image is reserved twice — about 1–1.5 % of the
2 MiB heap. Small, and that is part of the argument below.

## Why neither Zephyr mechanism transfers

**The size is already DERIVED per leaf, and correctly.** It varies across the
six because `nros sync` writes each leaf a `nros-managed-env.toml` carrying
derived executor knobs — the issue 0827 / 1061 channel:

```
# examples/qemu-arm-freertos/rust/talker/.cargo/nros-managed-env.toml
NROS_EXECUTOR_ACTION_CLIENTS = "0"      # action-server: "1"
NROS_EXECUTOR_MAX_CBS = "1"
```

`EXECUTOR_BACKING_DEFAULT_U64S` is `ExecutorSizing::DEFAULT.u64_len()` over
those knobs, so the backing already tracks the declaration. Nothing is broken on
that side.

**1171's answer — STATE the size — is wrong here.** On Zephyr it was right
because the derived size is target-dependent (87,256 B on mps2_an385 against
88,328 B on native_sim for one conf), so no single derived number could pair
both boards. FreeRTOS has one target, and stating a number would pin a value
that `nros sync` derives, so the next declaration change would leave the stated
number stale — with the const assert only catching the case where it falls below
`EXECUTOR_BACKING_DEFAULT_U64S`, not the case where it is needlessly large.
Stating here fights the derivation instead of fixing anything.

**Lowering the heap by a copied number is 1171's original defect.** The
subtrahend would be a literal in a leaf `[env] NROS_FREERTOS_HEAP_KB`, copied out
of `nm`, stale the moment an entity count moves — on a platform where the size is
per-leaf, so it would be six literals rather than one.

## The mechanism that WOULD work, and what blocks it

The board's heap default should subtract the backing itself. It already computes
one (`packages/boards/nros-board-freertos/build.rs`):

```rust
let zenoh_default_kb = (env::var("CARGO_FEATURE_RMW_ZENOH").is_ok()).then(|| 2048_usize);
```

`nros-node` declares `links = "nros_node"` and already exports build facts to
dependents (`cargo:max_cbs`, `cargo:arena_size`, `cargo:rx_buf_size` ->
`DEP_NROS_NODE_*`), so adding `cargo:backing_u64s` is one line.

**The blocker: `nros-board-freertos` does not depend on `nros-node`**, so the
`DEP_NROS_NODE_*` channel does not reach it. Cargo only exposes those variables
to direct dependents of the `links` crate. Fixing this means either giving the
board that dependency — a real layering change, not a knob — or carrying the
figure through the same file-based route the platform rungs already use
(`BuildRungs::from_build_env`, which this build script already consults).

## Two reductions target the same budget, and they must not be done separately

`build.rs` justifies the 2 MiB against a demand list that includes "the
`nros_app` task stack (now 384 KiB)". Issue 1146 lowers that default to
131,072 B — 256 KiB per task — and says so explicitly: *"Heap budget, not
`.bss`, until `configTOTAL_HEAP_SIZE` follows (issue 1145)."*

So there are two pending reductions to one number, one of them ~10x the other.
Doing them in separate hand-edited commits invites a double subtraction that
nothing would catch until an image malloc-fails at runtime — and issue 1146
measured exactly that failure mode, finding it reports `*** MALLOC FAILED ***`
rather than `*** STACK OVERFLOW ***`, because heap_4 hands out the task stacks.

## Recommended order

1. Land issue 1146 (the stack default).
2. Add `cargo:backing_u64s` to `nros-node` and route it to the board, or decide
   the layering question above.
3. Re-derive the heap default ONCE against both reductions, and verify by
   RUNNING every affected image on QEMU against a live router — the bar issue
   1146 already met for the stack, and the only bar that catches a too-small
   heap.

## Not done here, deliberately

No conf was edited. Issue 1145's Zephyr sweep is landed and this is the
FreeRTOS half of the same issue; recording the measurement and the blocker is
worth more than six hand-copied literals that would need re-deriving one PR
later.
