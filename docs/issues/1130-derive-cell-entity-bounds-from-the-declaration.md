---
id: 1130
title: "A component's cell registries are sized by a const the author must write, so an out-of-tree image still pays the worst case"
status: open
type: enhancement
area: api, cli
severity: medium
found: 2026-09-06
related: [0857, 0827, 0900, 0965, 1061]
---

# The number is derivable and is still hand-written

`Node::ENTITY_BOUNDS` decides how big a component's static cell registries
are. It defaults to `EntityBounds::knob_caps()` —
`NROS_RUNTIME_MAX_CELL_ENTITIES` (8) per kind — and a publisher slot embeds a
loan arena, so undeclared costs real static RAM: measured at **50,824 B of
`.bss`** for a one-service component against **568 B** declared
(issue 0857, `nm -S` on `native_service_server_entry`).

Issue 0857 landed the declarations for every in-tree class and
`check-component-entity-bounds` to keep them there. **An out-of-tree component
still pays the full 50 KB unless its author writes the const**, and nothing
tells them to. The default is deliberately the safe one — a budget short of
what the image needs is a boot failure, not a saving — so the fix is not a
smaller default.

## Why this is derivable

The number is a per-COMPONENT, per-KIND count over exactly five kinds
(publishers, service servers, service clients, action clients, action
servers), and the 0827/0965/1061 campaign already computes counts of that
shape from the same declarations:

* `nros_cli_core::entity_inventory` holds the counting rules — one
  implementation, rendered into JSON, a CMake fragment and `KEY=VALUE` env.
* `leaf_entity_env` turns a cargo leaf's `nros sync` probe output (or its
  manifest `[package.metadata.nros.component] entities`) into those rows and
  writes a gitignored `[env]` sidecar that the leaf's whole dep graph reads.
* `nros ws entity-inventory` does the same for a configured image from
  `nros-metadata.json`, and `_nros_resolve_derivable_knob` delivers the
  result to the cargo lane.

`NROS_RUNTIME_MAX_CELL_ENTITIES` is read from the environment by
`packages/api/nros/build.rs` (via `nros_zephyr_build::knob_usize`, which does
emit `rerun-if-env-changed`), so it fits both carriers unchanged.

## Direction

1. Add the figure to `DerivedEntityKnobs` — `max_cell_entities` = max over
   components of the max over those five kinds. **In `entity_inventory`, not
   beside it**: a second counting implementation is how two answers drift.
2. Emit it from `to_env` / `to_cmake` / `render_env_sidecar`, alongside
   `NROS_EXECUTOR_MAX_CBS`.
3. Compose it across entries by MAX (one runtime staticlib per configure must
   satisfy the largest), and abstain entirely if any entry's inventory
   refuses — the rule `NROS_DECLARED_SERVICE_SERVERS` already follows in
   `NanoRosEntityFacts.cmake`.

The safety argument is the one `MAX_CBS` already relies on: the inventory
refuses on incomplete data, the derived value carries no headroom, and a short
registry is a loud registration error (`push_publisher` → `NodeDeclError`), not
a silent drop.

An explicit `ENTITY_BOUNDS` must keep winning over the derived knob — it is
per-CLASS where the knob is per-IMAGE, so it is always the tighter answer.
