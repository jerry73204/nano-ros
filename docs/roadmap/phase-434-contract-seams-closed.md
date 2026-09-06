# Phase 434 — the contract seams with play_launch are closed on this side

**Status (2026-09-06). Complete.** Implements nothing new in the RMW; consumes
three model fields play_launch has carried across the model boundary for this
crate since its phases 67/68, and which this side never read.

## What was open

play_launch's phase 70 W1 census (2026-09-05) found the claim in its own
docs — that nano-ros reads `node_concurrency` and `max_jitter_ms` from the
model — false in every branch here. The real reads of `model.contracts` were
`node_paths`, `pub_endpoints`, `sub_endpoints`. The seam had teeth:

- `mapper_input_from_model` built `MapperPath` with `max_jitter_ms: None`
  and `miss: None` while the model carried both — a jitter requirement and a
  weakly-hard miss tolerance that could never reach `chain_aware_rank`.
- `MapperNode::claims_concurrency` was never set, so a contract's exclusion
  relation — which decides whether summing a chain's latencies is sound and
  whether a per-thread reservation is — was invisible to the mapper.
- `infer_callback_groups` gave every chainless callback its own `reentrant`
  group, while the contract's absent `concurrency:` means everything
  serialises: the default `rclcpp`'s implicit callback group uses and
  `default_cbg_type` here uses. Two toolchains, opposite defaults, each
  picked silently.

## What closes it

**Pin** `ros-launch-manifest` v0.1.23 → v0.1.31 (eight tags of vocabulary
work upstream; one compile error was expected from the model changes and
none occurred — every model literal here already uses `..Default::default()`).

**Mapper input** (`nros-orchestration-ir::mapper_input`): `max_jitter_ms`
and `miss` read straight through from `contracts.node_paths`;
`claims_concurrency` is true when a node's `node_concurrency` entry leaves
any path outside every `exclusive` set — the author claiming MORE
concurrency than the safe default.

**Planner** (`nros-cli-core::orchestration::planner`): the record a model
produces now carries `node_concurrency` (always present when the record came
from a model, even empty — its presence is what says the contract's default
applies). `infer_callback_groups_with_declarations` prefers the declaration:

| record | node entry | chainless callbacks become |
| --- | --- | --- |
| legacy (no key) | — | one `reentrant` group each (unchanged) |
| from a model | none | ONE `mutually_exclusive` group per node — `declared: default` |
| from a model | `exclusive: [[a, b]]` | `[a, b]` mutually exclusive (`declared: exclusive`); the rest `reentrant`, because that is the claim |

Chains keep their mutually-exclusive groups either way: dataflow coupling is
a fact the contract does not override. Path names are matched to callback
ids by the callback's own name (`…/plan` ↔ `plan`); a declared set naming no
callback of the node is ignored rather than guessed at.

## Still open on this side

- `srv_endpoints`, `max_response_ms` and `tolerance_ms` are carried in the
  model and read by nothing here. They stay on play_launch's census
  baseline as this repo's debt until something consumes them.
- Whether `claims_concurrency` should also relax the executor's dispatch
  (a `Reentrant` group where the author claimed it) is the multi-worker
  executor question RFC-0047 OQ1 already holds.

Cross-reference: play_launch `docs/roadmap/phase-70-consumer-census.md`
(the finding), `docs/design/contract-axes.md` §3.4 (the exclusion relation).
