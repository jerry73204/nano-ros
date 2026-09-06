---
id: 1150
title: "parameter `step` is stored, published to `ros2 param describe`, and never enforced — while two ledger rows state that it is"
status: open
type: bug
area: params
related: [phase-426, phase-428, issue-1022]
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
