---
id: 1121
title: "Adding a `<stem>.contract.yaml` does not invalidate the SystemModel it feeds — `nros sync` leaves the stale model in place"
status: open
type: bug
area: cli, tooling
severity: medium
found: 2026-09-06
related: [1120, 0475, 1018]
---

# The file that sizes the image is not an input to the thing that reads it

phase-412 moved "what this component creates" out of
`nano_ros_node_register(ENTITIES ...)` and into a contract sidecar beside the
launch file. The resolver folds it into the SystemModel, and
`nros ws entity-inventory --model` sizes the image's pools from that.

**Creating that sidecar does not cause the model to be re-resolved.**

## Reproduction, measured

On a first out-of-tree consumer
([`orin-spe-heartbeat`](https://github.com/jerry73204/orin-spe-heartbeat)):

1. Wrote a new `src/spe_bringup/launch/system.contract.yaml`.
2. `nros sync` — reported `source metadata — 1 rebuilt, 0 already current`, and
   left `build/nros/models/spe_bringup/system_model.yaml` untouched. Timestamps
   at that point: model `07:59`, contract `10:52`.
3. Rebuilt the image. `entity_inventory.cmake` still carried
   `NROS_ENTITY_INVENTORY_STATUS "refused"`.
4. `rm -rf build/nros/models` then `nros sync` — the model was regenerated and
   now carried the contract:

   ```yaml
   contracts:
     node_paths:
       /heartbeat/on_tick:
         output:
         - /heartbeat/heartbeat
   ```

5. Rebuilt: `NROS_ENTITY_INVENTORY_STATUS "derived"`,
   `NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "resolved"`, and `SERVICE_BUFFERS`
   fell from 35,424 B to 4,428 B (8 slots to 1).

   **Correction, from issue 1122's measurement:** `SMALL_PAYLOADS` did NOT
   shrink — it is still 32,768 B in that image. The first reading of this
   reproduction claimed it had, because it dropped out of a `tail -5` of the
   largest symbols once `SERVICE_BUFFERS` shrank and reordered the list. An
   absence from a truncated top-N is not an absence.

So the only thing standing between "refused" and "derived" was deleting the
model by hand — which is the wipe-and-rebuild antipattern CLAUDE.md names, and
the reason it works here is a missing dependency edge, exactly as in issues 0475
(a lib inside a raw `-Wl,` flag gets no rebuild edge) and 1018 (a configure-time
emitter has no `DEPENDS` on its tool).

## Why this is worse than it looks

The failure is SILENT and it is sizing. A user who writes a contract, sees
`nros sync` succeed and the image build, has no reason to suspect the sidecar
was ignored — the numbers are simply the defaults. On a host that is invisible;
on a 256 KB target it is the difference between fitting and not.

It also compounds issue 1120: the refusal message tells the user to add
`ENTITIES`, which phase-412 made a hard error, so the correct move is to write a
contract — and then the contract does not take effect.

## Not established

Where the freshness decision lives. `nros sync`'s model-writing path was not
located during this investigation, and the issue is filed on the observed
behaviour rather than on a guess about the code. Whoever picks it up should
start by finding what decides a model needs re-resolving, and whether the
contract sidecars are in that input set at all — `cmd/entity_facts.rs:116`
notes the tree has "93 `*.launch.xml` and 0 `*.contract.yaml`", so in-tree
coverage of this path is thin.
