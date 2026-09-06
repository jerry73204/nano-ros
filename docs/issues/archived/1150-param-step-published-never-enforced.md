---
id: 1150
title: "parameter `step` is stored, published to `ros2 param describe`, and never enforced — while two ledger rows state that it is"
status: resolved
type: bug
area: params
related: [phase-426, phase-428, issue-1022, issue-1151]
---

## Problem

`packages/core/nros-params/src/types.rs:203-205` and `:226-228`: `contains()`
is `value >= min && value <= max`. `self.step` appears nowhere. Upstream
enforces it (`rclrs/src/parameter/range.rs:173-177`).

Worse than an omission: `parameter_services.rs:270` and `:868` PUBLISH the
step in `DescribeParameters`, so `ros2 param describe` tells the operator a
constraint exists that the node does not apply.

## The ledger asserts the opposite as fact

`param.json` `rust:ParameterDescriptor`: *"both with min/max/step, both
enforced by `ParameterDescriptor::validate_range` on every set."* Same sentence
in `rust:ParameterRange`. Issue-1022 class, live.

## Why a green lane never caught it

Both range tests use a step that cannot fail: `server.rs:737` uses `step 0.0`,
`typed.rs:481` uses `step 1` from `min 0`.

Related: an out-of-range DEFAULT is accepted at declare (upstream
`InitialValueOutOfRange`); an invalid range (`lower > upper`, `step <= 0`) is
never validated (upstream `InvalidRange`), so a negative `step` becomes a huge
`u64` on the wire at `:270`/`:868`.

## Fix

Enforce step in `contains()`; validate range and default at declare; add a
range test whose step CAN fail. Correct both ledger rows. Phase-426 territory.

## Resolved 2026-09-06

**One predicate.** `ParameterRange::contains(&ParameterValue)` in
`packages/core/nros-params/src/types.rs` is now THE range verdict;
`ParameterDescriptor::validate_range` forwards to it, and both arms mirror
rclrs `range.rs`:

- integer: `min <= v <= max`, and when `step != 0`, `(v - min) % step == 0`;
  the upper bound is accepted even off-lattice (upstream checks `upper == v`
  before the step). Checked arithmetic, so an extreme `min` cannot overflow.
- double: either endpoint accepted within tolerance; otherwise inside the
  bounds and, when `step != 0`, within 100 ULP (upstream's `are_close`) of the
  nearest `min + k*step`. `round` is spelled by hand because `f64::round` is
  not in `core`.

**Every set reaches it through one path.** `ParameterServer::set` and the
new `check_apply` / `apply` share `check_set_existing` (read-only, type,
range — in that order), and the four handler sites in
`packages/core/nros-node/src/parameter_services.rs` (`apply_one`, the
streaming atomic pre-check and apply pass, and the two `#[cfg(test)]`
by-value oracles) all call it. The atomic pre-check used to re-spell the three
checks inline; it now asks the same function the apply would, so the two
cannot disagree.

**Declare validates.** Both declare paths (`declare_with_descriptor`,
`declare_parameter`) go through `declaration_is_consistent`: an ill-formed
range (`min > max`, `step < 0`, NaN step) or a default the range refuses is
rejected — `false` from the bool API, `Err(SetParameterResult::InvalidRange)`
from the typed one. The typed builder's `integer_range` / `float_range`
refuse an ill-formed range at the call site with `ParameterError::InvalidRange`.
A negative step therefore can no longer reach the `u64` cast in
`DescribeParameters`.

**The tests that can fail.** `types.rs`: integer lattice accept/reject,
upper-bound-off-lattice, step 0, overflow at `i64::MIN`; float lattice with
tolerance (`0.1 + 0.2` against step `0.1`), endpoints, step 0, nonzero min.
`server.rs`: `set` refused off-step for both arms, declare refused for an
ill-formed range and for an off-lattice default. `nros-node`: the streaming
`SetParameters` refuses `4.2` against `0..10 step 0.5` with
`"Value out of range"`, and the `described_server` oracle fixture now
ASSERTS every declare — its `count` default was `3` against `step 2`, which
the new declare check would have silently dropped from the fixture.

**Ledger.** Both `why` sentences (`rust:ParameterDescriptor`,
`rust:ParameterRange`) now say what is true and date when it became true.

**Not covered here.** The `ros2 param` integration tests in
`packages/testing/nros-tests/tests/params.rs` (`test_ros2_param_set`,
`test_ros2_param_describe`) set an unconstrained parameter and grep `describe`
for the type line; neither exercises a step, so a live-peer cell for the
constrained path is still phase-426 W6 territory.

Sweep: `grep -rn 'validate_range\|\.contains(' packages --include='*.rs' |
grep -i range` — every hit resolves to `ParameterRange::contains`.
