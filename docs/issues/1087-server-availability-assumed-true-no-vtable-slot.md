---
id: 1087
title: "`wait_for_service` still returns immediately on every C-ABI backend —
  issue 1008's sibling, in a method pair with NO vtable slot"
status: open
type: bug
area: rmw, api
related: [issue-1008, phase-417, phase-428, rfc-0089]
---

## Problem

Issue 1008 fixed `is_server_ready`'s optimistic trait default. **The pair one
frame down has the same defect and no slot to override it through.**

```rust
// nros-rmw/src/traits.rs:2818
fn poll_server_discovery(&mut self) -> Result<Option<bool>, Self::Error> {
    Ok(Some(true))            // "server is always assumed reachable"
}
```

`grep -n discovery packages/core/nros-rmw-abi/include/nros/rmw_vtable.h` finds
**nothing**: the pair has no vtable slot at all. So `CffiClient` inherits both
defaults, and a Rust backend that overrides them **loses the override crossing
the C ABI**. Cyclone and XRCE leave `service_server_is_available` NULL
(`cyclonedds/src/vtable.cpp:338`, `xrce/src/vtable.c:89`).

Six wait paths read `Some(true) => return Ok(true)`: `handles.rs:2196`,
`handles.rs:2855`, `nros-c/src/service.rs:1734`, `nros-c/src/action/client.rs:376`,
`nros-cpp/src/service.rs:837`, `nros-cpp/src/action.rs:1029`.

Zenoh, which *does* override, has the other half of the defect: `server_seen`
(`zenoh/shim/service.rs:730`) is set at `:1018` and **never cleared**, so a
server that dies still reads available for the client's lifetime.

## The justification is false

`handles.rs:2157-2160` and `zenoh/shim/service.rs:1005-1008` both assert this
"matches rclcpp's snapshot semantic". It does not.
`rclcpp::ClientBase::service_is_ready()` calls `rcl_service_server_is_available`
on **every** invocation — no snapshot, no latch. Upstream `rmw.h:237-238` says
the outcome reflects a QoS-compatibility **change**, in either direction.

## Why 1008's fix missed it

1008 hardened the fast path so `Err` falls through to the wait loop. **The loop
it now always falls through to has the identical defect.** The method 1008 fixed
had a vtable slot; this pair does not, which is why the same grep did not reach
it.

## Impact

`wait_for_service` returns `Ok(true)` immediately on cyclone, XRCE and uORB. The
failure surfaces as a request-side timeout 30 s later — the startup-ordering
race `server_available` was added to prevent.

## Fix

The honest three-way answer already exists in this tree:
`nros-c/src/service.rs:1632-1637` returns `*out = -1` ("don't know") with
`NROS_RET_OK` — upstream's two-channel shape.

1. Give the discovery pair a vtable slot, or delete it in favour of
   `service_server_is_available` which already has one and already has the
   right shape.
2. Remove the optimistic defaults; a backend that cannot answer must say so.
3. Zenoh: clear `server_seen` when the liveliness token goes away.
4. Correct the two comments claiming rclcpp latches.
