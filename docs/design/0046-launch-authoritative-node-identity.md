---
rfc: 0046
title: "Launch-authoritative node identity (name + namespace), unified across languages"
status: Draft
since: 2026-06
last-reviewed: 2026-06
implements-tracked-by: [phase-268]
supersedes: []
superseded-by: null
---

# RFC-0046 — Launch-authoritative node identity, unified across languages

> **Implemented by phase-268 (2026-06-29).** Realized for Rust + C/C++; #105 resolved. One nuance the
> design under-weighted: the single resolution site is `Executor::node_builder` (correct), but on the
> hosted/CFFI path the per-entity node name must also cross the RMW vtable — it was being overwritten
> by the session's open-time name in `CffiSession::make_view`. Fixed at the caller (`entity_view`
> threads per-entity `node_name`/`namespace` into the per-call session view) with **no vtable ABI
> change**. See phase-268 Outcome for the full data path. Flip to Stable with the ARCHITECTURE.md
> update when convenient.

## Summary

Adopt the standard ROS 2 model for node identity: a node's **name** and **namespace** come from
the **launch file** (`<node name="…" namespace="…">`), which is the single source of truth; the
**`exec=`** attribute selects the *entity* (the compiled node — a Cargo `[[bin]]` / CMake target);
and the node's own code provides only a **fallback default** that the launch overrides (model A —
rclcpp semantics). This identity resolution must be **identical across Rust, C, and C++**, applied
at the one shared site both go through, so a multi-component launch yields per-component graph
nodes named from the launch (the runtime half is [#105](../issues/archived/0105-multi-node-per-node-graph-naming.md)).

This RFC extends RFC-0004 (the configuration model) on the node-identity axis and is the naming
half of #105; #104 (node liveliness token) is the graph-declaration half.

## Problem

Node identity is sourced inconsistently today:

| Language | name source | namespace |
| --- | --- | --- |
| **C / C++** | codegen reads launch `<node name>` → `n.name.unwrap_or(n.exec)` baked into the emitted `nros_cpp_node_create("talker", …)` (`emit_c.rs:106`) | launch (codegen) |
| **Rust** | **hardcoded** in the node's hand-written `register()`: `ctx.create_node(NodeOptions::new("talker"))` | hardcoded |

So C/C++ are already launch-driven but Rust names live in node code, the launch `name=` has no
authority over a Rust node, and there is no single precedence rule. The launch name *is* parsed
(for tier/param assignment) but never overrides the code's `create_node` name. Multi-node entries
also collapse to one graph node (#105) regardless.

## Design

### One precedence rule (identical for all languages)

Per node, resolved independently for name and namespace:

```
name:       launch <node name="…">     >  code default (create_node arg)  >  launch exec=  >  "node"
namespace:  launch <node namespace="…"> >  code default                    >  "/"
```

The launch attributes are authoritative (a remap, as in rclcpp). The code-provided default is the
node's self-name when no launch overrides it — in practice always overridden, since a nano-ros node
pkg is a *library* run only through an entry that carries a launch; the default keeps the direct
node API usable and standalone-testable.

### One resolution site

Both languages create nodes through `Executor::node_builder(name).build()`
(`nros-node/src/executor/node_record.rs:270`) — Rust via `NodeContext::create_node` →
`ExecutorNodeRuntime::create_node` → `node_builder`; C/C++ via `nros_cpp_node_create_ex` →
`ctx.executor.node_builder(name).build()` (`nros-cpp/src/lib.rs:1007`). The override is applied
**there**, once: `node_builder` (or the create path just above it) checks for an injected launch
identity and uses it in place of the caller-supplied default.

### Per-component injection (mirrors the W4a param rail)

The entry already parses every launch `<node pkg= exec= name= namespace=>` and already injects
per-component data into the runtime before each `register()` — that is exactly how W4a baked
`<param>` values (`runtime.params = &[…]` per component, read by `ctx.param`). Node identity rides
the same rail:

- **Rust (`nros::main!`):** for each component, set the injected identity (e.g.
  `runtime.node_identity = ("talker", "/robot1")`) before the `<pkg>::register(runtime)` call, from
  the parsed launch node. `create_node(NodeOptions::new(default))` then resolves to the injected
  identity when present.
- **C / C++ (`nros codegen entry`):** the codegen — which today bakes the launch name straight into
  the `nros_cpp_node_create` call — instead **injects** the launch identity and passes the node's
  default (`exec`) as the `create_node` argument, so the SAME shared resolver applies. Net
  observable behavior is unchanged for C/C++ (still launch-named), but the resolution rule is now
  the single shared one rather than a parallel codegen-time computation.

This keeps the rule DRY: one resolver, one precedence, three front-ends that all feed it the launch
identity + a default.

### `exec=` selects the entity (unchanged)

`exec=` maps to the compiled node — the Cargo `[[bin]]`/`[package.metadata.nros.node]` (Rust) or the
CMake target (C/C++). It identifies *which code* runs; it is not the node's graph name (only the
`unwrap_or(exec)` fallback uses it when a launch node omits `name=`, matching ROS).

## Relationship to #104 / #105

- **#104** (done) declared a node liveliness token for the *primary* session node from config.
- **#105** declares a per-component liveliness token on the shared session (one zenoh session hosts
  N graph nodes — the NN keyexpr identifies by node name, not session id).
- **This RFC** decides *what name* those per-component tokens carry: the **launch** name, uniformly
  across languages. #105 implements the graph declaration; RFC-0046 fixes the name's provenance.
- Gate the #104 primary `/node` token off when ≥1 named component node exists, so a multi-node
  entry shows exactly its components (`/talker` + `/listener`), not an extra `/node`.

## Migration / compatibility

- Existing Rust node code (`NodeOptions::new("talker")`) keeps working — that string becomes the
  default, and current examples keep their names because launch + code agree by convention. A launch
  that sets a *different* `name=` now wins (new, desired).
- C/C++ observable behavior unchanged (still launch-named); only the resolution moves to the shared
  site.
- No change to `exec=`/entity selection or to tier/param assignment (which already key off the
  launch node instance).

## Cross-references

- RFC-0004 (configuration-and-transports) — the config model this extends.
- RFC-0045 (boot-config resolution) — the same single-resolution-site discipline, for the primary
  session; this RFC is the per-node analogue.
- #105 (multi-node per-node graph naming) — implements the graph + naming together; tracked-by this
  RFC.
- #98 / #104 (single-node naming / node liveliness) — the primary-node precursors.
