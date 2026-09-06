---
id: 364
title: "`<node machine=>` is ROS 1 syntax, not ROS 2"
status: resolved
type: bug
area: launch
resolved_in: "phase-326, `5f784d5da` (+ fork chain: `8774077` -> `aac489f` -> `967031a`)"
---

# 0364 — `<node machine=>` is ROS 1 syntax, not ROS 2

**Status:** Resolved (phase-326, `5f784d5da` + fork chain parser `8774077` →
rlm `aac489f` → ros-launch-resolve `967031a`)
**Filed:** 2026-07-31 · **Resolved:** 2026-07-31

## Summary

The multi-host partitioning added in phase-211.F was built on
`<node machine="robot1">` — ROS 1 roslaunch syntax. ROS 2 has no such
attribute (nothing in `ros2/launch_ros` reads it), its strict XML frontend
raises `Unexpected attribute(s) found in 'node': {'machine'}`, and the
multi-machine design proposal (ros2/design #255) was closed unmerged. All
four `multihost.launch.xml` example workspaces were unrunnable by
`ros2 launch`, and play_launch's Rust parser accepted what its Python parser
rejected (a parser-parity break).

## Resolution — phase-326, the partition moved to resolve time

Upstream (fork chain, in dependency order): `play_launch_parser` dropped the
`machine` capture end to end; `ros-launch-manifest` dropped `model::Deploy.host`
and the issue-0291 `by_machine` placement fallback (with multiple self blocks,
`nodes = [..]` is now the only placement source — pre-0291 behaviour restored);
`ros-launch-resolve` dropped `launch_dump::NodeRecord.machine` and the
model_builder `machine=` → `deploy.host` mapping.

nano-ros: the launch files use a standard `<arg name="host" default="all"/>`
plus `if=$(eval …)` conditions — plain ROS 2 XML — and each host gets a
committed per-host model (`multihost_robot<N>_model.yaml`, resolved with
`host:=robot<N>`; the binding is recorded in `meta.args` and replayed by
`nros sync` on refresh). `Plan::for_host`/`PlanNode.host`, `codegen entry
--host`, `nros::main!(host = …)`, cmake `HOST`, the planner's
`machine=`→`host_id` lowering and `NodeSpec.machine` are all deleted; the
removed knobs fail loud with migration guidance.

Found on the way (both fixed): the `nros-launch-resolve` helper never
forwarded launch arguments to the parser — its vestigial `launch_file`
positional swallowed the first `KEY:=VALUE`, and the binding was passed only
to the model metadata, never to `parse_launch_file` — so `<arg>` overrides
and `if=` conditions were inert through the helper.

See `docs/roadmap/archived/phase-326-multihost-via-launch-args.md` for the
work breakdown.
