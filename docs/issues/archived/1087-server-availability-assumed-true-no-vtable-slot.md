---
id: 1087
title: "`wait_for_service` still returns immediately on every C-ABI backend —
  issue 1008's sibling, in a method pair with NO vtable slot"
status: resolved
type: bug
area: rmw, api
related: [issue-1008, phase-417, phase-428, rfc-0089]
resolved_in: "phase-428 W13"
---

## Problem (as filed, 2026-09-05)

`ClientTrait::poll_server_discovery` defaulted to `Ok(Some(true))` — "server is
always assumed reachable" — and the `start_server_discovery` /
`poll_server_discovery` pair had **no vtable slot**, so `CffiClient` inherited
the default and every C-ABI backend (cyclone, XRCE, uORB) returned `Ok(true)`
from `wait_for_service` without asking anything. Six wait loops read
`Some(true) => return Ok(true)`. Zenoh, the one backend that overrode the pair,
had the other half of the defect: `server_seen` was set on the first liveliness
reply and **never cleared**, so a server that died read available for the
client's lifetime. Both sites justified themselves with "rclcpp's snapshot
semantic", which does not exist — `ClientBase::service_is_ready()` calls
`rcl_service_server_is_available` on every invocation.

Issue 1008 had hardened the fast path (`service_is_ready`, which HAS a slot) so
`Err` falls through to the wait loop; the loop it fell through to had the
identical optimism one frame down.

## Resolution — phase-428 W13 (2026-09-06)

The shape mismatch was downstream of not keeping the state upstream keeps.
`rmw_service_server_is_available` answers immediately upstream because DDS
maintains a discovery cache; ours answered with a start-then-poll
`z_liveliness_get`, which is why the pair existed and why the synchronous
method had nothing to return but a latch.

**The zenoh backend now keeps a matched-server SET, and `service_is_ready`
reads it.** The set is the session's graph cache (phase-381 / issue 0903): one
standing `z_liveliness_declare_subscriber` on `@ros2_lv/<domain>/**` with
`history = true`, fed PUT on declare and DELETE on undeclare
(`zpico_graph_cache_start`; the set mutation is now the pure
`zpico_graph_set_apply`, tested from Rust against the real C for PUT, duplicate
PUT, DELETE, absent DELETE and overflow). `ZenohSession::create_client` starts
it, once per session. `ZenohServiceClient::service_is_ready` walks the cache
through the one shared walk (`session::graph_cache_for_each`) and matches an
`SS` token on `(mangled service name, DDS type name)` — the pair
`rmw_zenoh_cpp` keys a service on (`liveliness_utils.cpp`, `Entity::Entity`,
humble 0.1.9: `@ros2_lv/<domain>/<zid>/<nid>/<eid>/SS/<enclave>/<ns>/<node>/
<%service>/<pkg::srv::dds_::Type_>/<hash>/<qos>`). Three answers: `Ok(true)`,
`Ok(false)` when the set is complete, `Err(Unsupported)` when the cache is not
running or has DROPPED tokens (an absence then proves nothing).

**The pair is DELETED** — from `ClientTrait`, from the zenoh shim (with
`server_seen`, `discovery_handle`, `discovery_keyexpr` and the
`service_server_keyexpr_wildcard` builder), from `ActionClientCore`, and from
all six wait loops (`handles.rs` ×2, `nros-c` service + action, `nros-cpp`
service + action), which are now one shape: spin the executor, ask
`service_is_ready`, `Ok(true)` returns, anything else waits until the deadline.
`SERVER_DISCOVERY_PROBE_TIMEOUT_MS` went with it. `grep -rn
poll_server_discovery packages/` returns nothing.

Liveliness was never compiled out: `Z_FEATURE_LIVELINESS 1` is unconditional
in `nros-zpico-build`'s generated config on every platform, and the cache's
64 KiB buffer (`ZPICO_GRAPH_CACHE_SIZE`) was already resident in every session
since phase-381 — so the static RAM delta of this change is zero; what changes
is that the standing subscriber now runs from the first client rather than the
first graph query.

The native Rust `service-client` example gates its first request on the new
`TickCtx::service_is_ready_for_name` (rclcpp's `while (!wait_for_service(1s))
"service not available, waiting again..."` idiom, one check per timer tick),
and the `ZenohServiceRos2Server` interop cell now starts the client BEFORE the
ROS 2 `add_two_ints_server` and asserts both halves: the waiting line with no
server (and no request sent), then the result once the server's token lands.

## What this does NOT close

* **W13.c** — cyclone and XRCE still leave the `service_server_is_available`
  vtable slot NULL, so on those backends `service_is_ready` is `Err` and
  `wait_for_service` waits out its budget and reports `false`: honest, and
  slower than the DDS cache could answer. Tracked as phase-428 W13.c.
* QoS compatibility is not part of the match (upstream's slot folds it in).
* Whether a zenoh-pico liveliness subscriber sees its OWN session's tokens
  (client and server in one image) is not measured here; the `wait_for_service`
  callers in-tree pair with a separate server process.
