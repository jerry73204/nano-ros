---
id: 203
title: "mixed-workspace cpp codegen over-generates the interface set — cross-language service pairs blocked"
status: resolved
type: bug
area: codegen
related: [phase-263, phase-269, rfc-0026]
resolved_in: "e2c413582 (2026-07-16)"
---

## Summary

In the MIXED (umbrella) workspace, `nros_find_interfaces(LANGUAGE CPP)`
over-generates the **full** interface set — including `action_msgs` — whose
per-pkg cpp FFI crate references `builtin_interfaces` types that are not in
scope (`cannot find type builtin_interfaces_msg_time_t`). The same pkg
compiles fine in the single-language cpp workspace; it breaks only under the
mixed multi-pkg generation.

Found during phase-263 A1 (2026-06-28, mixed services wave) and deferred
there without an issue — this files it (surfaced by the 2026-07-16 phase-263
audit).

## Impact

Cross-LANGUAGE service/action pairs (e.g. **C server + cpp client**) cannot be
demonstrated in `examples/workspaces/mixed`: its service demo is C+C
(`c_add_server_pkg` + `c_add_client_pkg`, reused from the C workspace), with
cross-language preserved only at the workspace level (C talker + C++ listener
+ Rust heartbeat). The phase-263 "no faking" guardrail keeps the demo honest
but degraded.

Also the reason the mixed workspace carries no cpp feature pkgs for the
phase-269-delivered surfaces (params/lifecycle/safety/tiers are demo'd in
`ws-*-c` / `ws-*-cpp` but have no mixed variants).

## Repro sketch

Add a cpp service pkg (e.g. a `cpp_add_client_pkg` mirroring the C one) to
`examples/workspaces/mixed/src/` + the launch, configure the mixed workspace →
the generated per-pkg cpp FFI crate for the over-generated `action_msgs`
fails: `cannot find type builtin_interfaces_msg_time_t`.

## Direction

Either scope `nros_find_interfaces(LANGUAGE CPP)` generation to the pkg's
declared dependency closure (don't emit `action_msgs` unless depended on), or
make the generated cpp FFI crate carry its own `builtin_interfaces` type
imports so over-generation is at worst wasteful, not broken.

## Resolution — `e2c413582`, 2026-07-16

Closed by the commit that landed the demo it was blocking: *"fix(#203): mixed
cross-LANGUAGE service pair (C server + C++ client); close #203"*. The mixed
workspace's service pair is `c_add_server_pkg` + `cpp_add_client_pkg` today,
paired in `demo_bringup/system.toml` — which is verbatim the Repro sketch
above, so the thing this issue said could not be built is what is now built.

**The title is a misread, and the commit says so.** There was no
over-generation: `example_interfaces` genuinely depends on `action_msgs` (it
ships `.action` files), so the resolved closure was always correct. What was
broken — and only historically — was the cross-scope cpp compile, fixed en
route by phase-263 A4's `NanoRosGenerateInterfaces` idempotency/sibling repairs
and phase-269's header-mirror work. The "Direction" section above proposes two
fixes for a defect that turned out not to be the one that existed.

### Why this file said `open` for five weeks after being closed

That same commit archived it and never flipped `status:`. It is issue 0937's
shape three weeks earlier — an archiving move that carries the record
unchanged, so the resolution the commit message states never reaches the file.
It went unnoticed until `check-archived-issue-status` was written for 0937 and
found this one too, which is the argument for that gate in one line.

### What still guards it, and what does not

The compile-level regression site stands: `examples/workspaces/mixed` carries
fixture rows and two `Lang::Mixed` cells in `matrix::CELLS`
(`Linux`/`ZephyrNativeSim` × `EntryPubsub` × `Workspace`), so a cpp package in
the mixed multi-pkg generation is still compiled on a lane.

The three runtime tests `e2c413582` cited as its verification —
`mixed_service_roundtrip_xprocess_e2e`, `mixed_action_roundtrip_xprocess_e2e`,
`mixed_multihost_e2e` — no longer exist, and the surviving mixed cells are
`EntryPubsub`, not service or action. So the COMPILE this issue was about is
covered and the cross-language service ROUND TRIP is not. That is not a
regression of #203, but it is a gap worth knowing about before trusting this
file's "verified" line.
