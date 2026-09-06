---
id: 1151
title: "`ros2 param set` on an undeclared name silently CREATES it — a typo reports success and the real parameter is untouched"
status: open
type: bug
area: params
related: [phase-426, phase-428]
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
