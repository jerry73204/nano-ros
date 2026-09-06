---
id: 1151
title: "`ros2 param set` on an undeclared name silently CREATES it — a typo reports success and the real parameter is untouched"
status: resolved
type: bug
area: params
related: [phase-426, phase-428, issue-1150]
---

## Problem

`packages/core/nros-params/src/parameter_services.rs:627-635`, `apply_one`:
`if has { set } else if declare { Success }`. Upstream
(`rclrs/src/parameter.rs:526-533`) rejects a set on an undeclared name unless
`allow_undeclared` is on.

So `ros2 param set /node max_speeed 5.0` reports success, creates a parameter
nobody reads, and `max_speed` keeps its old value.

`grep -rn "undeclared\|allow_undeclared"` over the module and `tests/params.rs`
→ 0 hits. Undocumented, untested, unledgered — not a recorded decision, the
absence of one.

## Fix

Reject unless an explicit opt-in exists. If auto-declare is wanted as a
nano-ros capability, that is a ledger row with a disposition, not a silent
branch.

## Resolved 2026-09-06

**The decision now lives on the store, in one place.** `ParameterServer`
(`packages/core/nros-params/src/server.rs`) carries `allow_undeclared: bool`
(default `false`, as rclcpp's `NodeOptions::allow_undeclared_parameters` and
rclrs's `allow_undeclared`), with `set_allow_undeclared(bool)` /
`allows_undeclared()`. The wire-facing set is `ParameterServer::apply(name,
value)` with its dry-run twin `check_apply`: an existing name goes through
`set`; an unknown name is `SetParameterResult::Undeclared` unless the flag is
on, in which case it is declared (or `StorageFull`).

**Every handler site uses it.** The `if has { set } else { declare }` branch
existed in FOUR spellings in `packages/core/nros-node/src/parameter_services.rs`
— `apply_one` (streaming `SetParameters`), the streaming atomic pass-2, and
the two `#[cfg(test)]` by-value oracles (`handle_set_parameters`,
`handle_set_parameters_atomically`) — plus two inline copies of the atomic
PRE-check. All six now call `apply` / `check_apply`. `set_or_declare` stays as
the explicit in-image auto-declare a Rust caller opts into by naming it;
nothing on the wire path reaches it.

**The reason text is rclrs's, verbatim:** `UNDECLARED_REASON =
"Parameter was not declared and undeclared parameters are not allowed"`, in
the one reason table (`set_result_reason`; `to_rcl_set_result` now reads the
same table instead of carrying a second copy).

**Tests.** `server.rs`: the issue's typo is refused, the store is untouched,
`max_speed` keeps `1.0`; under `allow_undeclared` the same set declares;
`allow_undeclared` on a full store reports `StorageFull`; `apply` enforces
read-only/type/step through the same predicate as `set`. `nros-node`: the
streaming `SetParameters` refuses `max_speeed` with the reason string decoded
off the wire; the same request succeeds on a permissive store; the streaming
`SetParametersAtomically` rejects a batch of {good, typo} WHOLE — the good
half does not apply — and applies it whole on a permissive store. The
oracle-vs-streaming comparison runs against a permissive empty store (its
"everything declares" case), a default empty store (everything refused) and
the described store, with `max_speeed` in the hard cases.

**Where the flag is set from.** Today a Rust image reaches it through the
executor's `params_mut()`; no `NodeOptions` field exists yet. Phase-426 W1
keys the table by node, which is where a per-node `allow_undeclared` belongs
— the store-level flag is the shape that widening keeps (one field beside the
node key), not one it undoes.

**Not covered here.** `packages/testing/nros-tests/tests/params.rs`
`test_ros2_param_set` sets a DECLARED name (`start_value`) and asserts
success; no integration test sets an undeclared one. Phase-426 W6's live-peer
cell is the place for that.
