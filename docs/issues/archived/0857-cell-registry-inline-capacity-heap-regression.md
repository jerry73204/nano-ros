---
id: 857
title: "ComponentCell's inline registries cost worst-case × biggest-payload heap per component"
status: resolved
type: bug
area: api
severity: high
found: 2026-08-28
resolved_in: "phase-391 W5-endgame (mechanism) + this fix (declarations + gate)"
phase: phase-391
related: [0843, 0816, 1130]
---

# ComponentCell's inline registries cost worst-case × biggest-payload heap per component

## Symptom

`test_esp32_workspace_entry_e2e` red 3/3 solo after the phase-391 W5 heapless
port: the image dies with

```
memory allocation of 17468 bytes failed
```

right after `Ethernet ready.` — a clean-looking OOM in the #184 class. The
single-node esp32 examples stay green (same `.stack` = 60.4 KB, so the #64/#190
stack-overflow explanation is dead; this is genuine heap exhaustion).

Symbolized backtrace (addr2line on the exact fixture ELF) puts the failing
`Box::new_uninit` inside `listener_pkg::register` — it is the **second**
component's `Arc<ComponentCell>` allocation. The first cell plus the executor
arena backing (`NROS_EXECUTOR_ARENA_SIZE` 16384 + 1084 table overhead — the
same 17468, a coincidence that misdirected the first hour of triage) had
already consumed the 48 KB esp-alloc heap.

## Cause

W5 ported the cell registries from `alloc::Vec` to inline
`heapless::Vec<_, CELL_REG_CAP>`. That moved the cost model from
**pay-per-actual-entity** to **pay-worst-case-always**, and the worst case
multiplies by the biggest payload type:

```
publishers: heapless::Vec<(IdStr, EmbeddedRawPublisher), 8>
```

`EmbeddedRawPublisher` embeds `TxArena<DEFAULT_LOAN_BUF = 1024>`, so one entry
is ~1.35 KB and the vec alone is ~10.8 KB. With the three other registries
(~140 B × 8 × 3) the cell reaches ~17.5 KB — and it is still `Arc::new`'d, so
the whole thing lands on the Rust heap per component. A listener with ZERO
publishers pays for eight.

Pre-W5 the same cell cost ~a few hundred bytes plus per-actual-entity growth.

## What landed, in three parts

**W5-endgame steps 2a/2b/2c** built the MECHANISM (already on main when this
issue was reopened for a fix):

* `ComponentCell` and `ComponentSlotStorage` became generic over the five
  registry capacities, defaulted to the `NROS_RUNTIME_MAX_CELL_ENTITIES` knob.
* The cell moved into the macro-emitted per-class `static` beside the slot
  store, so it is `.bss` and not heap. **The heap half of this issue's title is
  therefore stale**: the cost is real and is static RAM, not an allocation.
* `Node::ENTITY_BOUNDS` gave a class a way to state its own capacities, and
  `nros::node!` feeds them to the store.

**What was missing was the declarations.** 81 of the 99 in-tree component
classes stated nothing, so each still got the knob caps. The mechanism read as
done and delivered nothing to almost every image in the tree.

**This fix (2026-09-06)** closes that:

* every in-tree `nros::node!` class now states `EntityBounds::exact(...)`,
  including the two scaffold templates (`cargo nano-ros new`, `nros new-node`),
  so a generated project starts out declaring;
* `check-component-entity-bounds` (fast lane) fails a class that states none,
  and fails a declaration BELOW what a straight-line read of `register()`
  finds. It is a verifier, never a supplier — a `for` loop it cannot read is
  still caught at runtime by the loud `push_publisher` → `NodeDeclError`, which
  is the safety property the whole design rests on.

## Measured

One build directory, one file changed, `examples/workspaces/rust`'s
`native_service_server_entry` (a single component declaring one service
server):

```
$ bash scripts/build/workspace-fixtures-build.sh linux rust \
      --id workspace-rust-native-service-server
$ nm -S --size-sort .../native_service_server_entry | grep SLOT_STORE
$ size .../native_service_server_entry
```

| | `__NROS_COMPONENT_service_server_pkg_SLOT_STORE` | image `.bss` |
| --- | --- | --- |
| no `ENTITY_BOUNDS` | 50,824 B | 348,202 B |
| `exact(0, 1, 0, 0, 0)` | 568 B | 298,954 B |
| delta | **−50,256 B** | **−49,248 B (−14.1 %)** |

`just mem-report <elf> --baseline <before.json>` reports the same
`−50,256` against the symbol.

Two more from the same tree, for the shape of the spread (`nm -S` on the
prebuilt native entries, 2026-09-05, before this fix): `action_server_pkg`
50,832 B undeclared; `talker_pkg` 3,888 B at `exact(1,0,0,0,0)`;
`listener_pkg` 272 B at `exact(0,0,0,0,0)`. The store is
`MAX_CLASS_INSTANCES` (2) copies of the cell, so a per-instance publisher slot
is the ~1.35 KB the Cause section prices.

`packages/api/nros/src/node_runtime.rs`'s
`a_declared_cell_pays_for_what_it_declares` pins the same quantity without a
link step.

## What this does NOT do

`Node::ENTITY_BOUNDS` still DEFAULTS to the knob caps, so an out-of-tree
component that declares nothing pays the worst case exactly as before. That is
deliberate — a large safe default beats a wrong small one — but it means the
saving reaches a user only when they write the const. Issue 1130 tracks
deriving the number instead, through the channel the 0827/0965/1061 campaign
already built (`entity_inventory`'s counting rules → the `[env]` sidecar for a
cargo leaf, the derivable-knob resolver for a configured image), so a user's
image gets it the way its pool budgets already do.

## Sweep

```
python3 scripts/check-component-entity-bounds.py --report   # every class, counted vs declared
python3 scripts/check-component-entity-bounds.py            # the gate
```
