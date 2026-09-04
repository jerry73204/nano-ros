---
rfc: 0037
title: "Rust and C user-API surfaces"
status: Draft
since: 2026-06
last-reviewed: 2026-06
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# RFC-0037 — Rust and C user-API surfaces

## Summary

The C++ user API is frozen by RFC-0018 (Stable). The **Rust** (`nros-node` /
`nros-core`) and **C** (`nros-c`) user APIs have no equivalent surface RFC —
RFC-0019/0020 govern the C *thin-wrapper discipline* (not the surface), and
RFC-0022 describes a *planned* Rust entity-tier refactor (not the current
surface). This RFC **records the current Rust and C user surfaces** as the
reference, so consumers and reviewers have a citable contract. It is `Draft`
deliberately: the Rust surface should not flip to `Stable` until RFC-0022's
entity-tier builder model lands (or is dropped), because that refactor would
revise it; the C surface is closer to freezable.

## Motivation / problem

- Examples, tests, and the C/C++ shims all depend on these surfaces, but a
  reviewer has no single doc to check a change against — only the code.
- Without a recorded surface, accidental breaking changes (renamed `create_*`,
  changed error variants) pass review unnoticed.

## Design

### Rust surface (`nros-node`, `nros-core`)

**Entry / lifecycle.** `Executor::open(&ExecutorConfig)` /
`Executor::open_multi(&[SessionSpec])` → `Executor`; `create_node(name) ->
NodeCtx<'_>`. Domain id is in the config / baked (RFC-0036). No `Arc<Node>` —
`NodeCtx<'_>` borrows `&mut Executor` (RFC-0022).

**Entities** (mirroring rclrs `create_*`):
- `create_publisher_on::<M>(...) -> Publisher<M>` — `Publisher::publish(&M)`.
- `create_subscription*` — callback `FnMut(&M)` (owned today; borrowed view is
  RFC-0033 § borrowed / phase-229.6).
- `register_service::<Svc, F>` / `register_service_client_raw`.
- service/action clients via `Promise` handles **or** spin-dispatched callbacks
  (RFC-0041): `NodeCtx::create_client_with_callback::<Svc, _>(name, FnMut(&Reply))`
  and `create_action_client_with_callbacks::<A>(…)` (goal-response / feedback /
  result), dual-mode with the `Promise` path.
- `register_timer::<F>`.

**Spin** (RFC-0021 blocking rule — helpers take `&mut Executor`):
`spin_once(timeout) -> SpinOnceResult`, `spin(timeout) -> !`,
`spin_blocking(SpinOptions)`. Service/action waits: `promise.wait(&mut executor,
timeout)`.

**Message contracts** (`nros-core`): trait `RosMessage: Serialize + Deserialize`,
`RosService`, `RosAction`; the executor takes `M: MessageForRmw`. Value types:
`Time`, `Duration`, `MessageInfo`, `GoalStatus`, `LifecycleState`.

**Error:** `NodeError` (enum: `Max*Reached`, `SerializationFailed`,
`DeserializationFailed`, `BufferTooSmall`, `TransportError`, `NotConnected`,
`*NameTooLong`). Distinct from `nros-core`'s `NanoRosError`/`RclReturnCode`
(RFC-0036) — `NodeError` is the node-builder layer.

### C surface (`nros-c`)

~120 functions (cbindgen → `nros_generated.h` + hand-written
`visibility.h`/`platform.h`/`types.h`). Where rclc/rcl has a counterpart the
entry point carries **rclc's own name**, in rclc's argument order and arity
(RFC-0087, settled 2026-09-04); everything with no counterpart keeps the
`nros_` prefix:

- init: `nros_executor_init`, `rclc_node_init_default`, `nros_support_init`.
- pub/sub: `rclc_publisher_init_default`,
  `nros_publisher_init_{with_qos,with_options}`, `nros_publish_raw`,
  `rclc_subscription_init_default`,
  `nros_subscription_init_{with_qos,polling}`.
- services/clients: `rclc_service_init_default`, `nros_service_init_polling`,
  `nros_service_send_response_raw`,
  `rclc_client_init_default`, `nros_client_send_request_raw`, `nros_client_call`;
  callback receive (RFC-0041): `nros_client_set_response_callback` +
  `nros_client_send_request_async` (reply dispatched at `rclc_executor_spin_some`).
- actions: `nros_action_{server,client}_init`, `nros_action_send_goal`,
  `nros_action_get_result`; callback receive (RFC-0041):
  `nros_action_client_set_{goal_response,feedback,result}_callback`.
- executor: `nros_executor_add_subscription*`, `rclc_executor_add_timer`,
  `rclc_executor_spin[_some]`.
- guard/lifecycle/params: `nros_guard_condition_*`, `nros_lifecycle_*`,
  `nros_param_*`.

Entities are opaque structs (`nros_node_t`, `nros_publisher_t`, …). Error:
`nros_ret_t` enum. The C layer is a **thin wrapper** — no logic re-impl
(RFC-0019); it delegates to `nros-node`.

### Freeze policy

- **C surface** — eligible to flip to `Stable` once the RFC-0019 opaque-entity
  refactor settles; at that point this RFC lists the frozen function set and an
  add-only rule (new `nros_*` fns append; signatures of existing fns are stable).
- **Rust surface** — stays `Draft` until RFC-0022 resolves. When it does, freeze
  the resulting `create_*` / `spin*` / `NodeError` shape here.
- Both: breaking a recorded signature requires updating this RFC in the same PR.

## Alternatives considered

- **One combined API RFC (Rust + C + C++).** Rejected — C++ already has the
  Stable RFC-0018; merging would muddy its status. Keep per-language.
- **Freeze the Rust surface now.** Rejected — RFC-0022's entity-tier builder
  would immediately supersede it; recording-as-Draft avoids a churned Stable doc.

## Open questions

1. Does RFC-0022 land as specced (fork/clone tiers), get trimmed, or get
   dropped? The Rust freeze waits on this.
2. Should `NodeError` and `NanoRosError`/`RclReturnCode` be unified, or is the
   two-layer split (builder vs core) intentional? Proposed: keep split; document
   here.

## Changelog

- 2026-06 — created (Draft). Recorded the current Rust (`nros-node`/`nros-core`)
  and C (`nros-c`) user surfaces and the freeze policy; C++ remains RFC-0018.
- 2026-06 (phase-239) — added the RFC-0041 callback-receive surfaces:
  Rust `create_client_with_callback` / `create_action_client_with_callbacks`; C
  `nros_client_set_response_callback` + `_send_request_async` and the action
  client `set_{goal_response,feedback,result}_callback` setters.
