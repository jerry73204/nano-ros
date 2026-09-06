---
id: 1127
title: "`interop::CELLS` declares 17 runnable live-peer cells and no recipe reaches 10 of them — G1 checks the test FILE exists, not that anything invokes it"
status: open
type: bug
area: testing, rmw
severity: high
found: 2026-09-06
related: [0352, 0791, 0903, 0759, 1055]
---

# Seventeen runnable cells, seven reachable

`nros_tests::interop::CELLS` is the intent list for every test whose subject is
a LIVE ROS 2 peer (RFC-0051, phase-324). It has 18 rows, 17 of them `Runtime`
(one is a declared carve-out with no test). Grepping `just/` and
`.github/workflows/` for the test binaries those rows name:

| test binary | cells | runnable by |
| --- | --- | --- |
| `interop_e2e` | 5 | `just test-ros2`, `just test-ros2-lifecycle` |
| `xrce_ros2_interop` | 2 | `just/xrce.just` |
| `graph_interop` | 2 | **nothing** |
| `qos_zephyr_ros2_interop_e2e` | 1 | **nothing** |
| `qos_override_e2e` | 1 | **nothing** |
| `params` | 1 | **nothing** |
| `rust_multi_node_per_node_graph` | 1 | **nothing** |
| `cpp_multi_node_entry` | 1 | **nothing** |
| `declarative_bridge_zenoh_to_cyclonedds` | 1 | **nothing** |
| `declarative_bridge_zenoh_to_xrce` | 1 | **nothing** |
| `bridge_zenoh_to_cyclonedds` | 1 | **nothing** |

`just test-all` does not close the hole — it explicitly EXCLUDES them:

```
-E 'not (group(=ros2-interop) or binary(xrce_ros2_interop) or …)'
```

which is correct on its own terms (they need a ROS 2 CLI and serialise on the
daemon), but no other recipe picks them up. Seven cells are reachable by hand,
ten by nothing at all, and no sweep reaches any of the seventeen.

## Why nobody noticed

`matrix_fixture_coverage.rs` G1 is the gate that exists for exactly this, and
its own doc comment says "A Runtime cell nothing runs … fails here". What it
asserts is:

```rust
if !tests_dir.join(format!("{}.rs", c.test)).is_file() {
```

The test FILE existing. Not a lane invoking it. G2 checks the build channel can
build the coordinate, G3 checks the build recipe's spelling, G4 checks the peer
declaration, G5 checks a `fixtures.toml` row produces the coordinate — five
gates around the cell, none of them asking whether anything runs it.

This is the repo's recurring shape stated in phase-393's own closing warning:
*a slot's EXISTENCE reads as coverage*. Here it is one level up — a cell's
existence reads as a lane.

## What it costs, concretely

Phase-381 shipped twelve `rmw` graph slots: produced, reachable from three
languages, mutation-tested, `check-api-parity` clean. **The feature did not
work at all** — `z_liveliness_get` is an interest, so a sweep saw an arbitrary
handful of the domain's tokens. Issue 0903 was several stacked defects on top
of a mechanism that could not work, and none of it manifests except against a
real peer. `graph_interop.rs` is the committed form of that lesson and it has
never executed.

Cyclone's graph reader (phase-381 W5) has never run against a live participant
either; the cell says so in a comment.

## The second half — ROS-2-facing tests with no cell at all

Eleven test files call `ros2_env_setup` or drive the `ros2` CLI and do not call
`interop::assert_test_bound`:

```
cpp_c_param_live_read_e2e  entry_e2e  param_live_read_e2e  ros2_action_e2e
ros_editions_bridge  ros_editions_smoke  ros_editions_nano_interop
ros_editions_e2e  output_marker_gate  workspace_features_e2e
zephyr_leaf_staleness
```

`ros_editions_*` is the docker edition axis and is deliberately not a cell
(CLAUDE.md says so). The others are not accounted for anywhere.
`ros2_action_e2e` is the notable one: **actions have no interop cell**, and
issue 0902 reports action goals completing between 20 % and 90 % of the time on
the same build.

## Why the box makes this structural, not lazy

This host has no ROS: `/opt/ros` does not exist, `ros2` is not on PATH,
`ROS_DISTRO` is empty. ROS 2 Humble lives in the `ros2` distrobox, and the
standing rule (issue 0759) is that a box in play means EVERY job runs in the
box on its OWN tree, because the compiler and libc differ and the artifacts are
shared with nothing checking they agree.

So a live-peer lane is not "add `--test graph_interop` to a recipe". It is a
lane that runs inside the box against the mirror tree, and that mirror is
`/mnt/wd/data/projects/nano-ros-box`, currently **323 commits behind** `main`
(at `d1d88f660`, 2026-09-04). There is also a `nano-ros-box-box` at
`ea9fbfae9` (2026-08-30) — a mirror of the mirror, from a sync run that
started inside the box tree. That is debris and should be removed once someone
confirms it holds nothing unique.

## Direction

Not "wire the tests in" — that lands a red lane with no signal capacity, which
is the failure mode CLAUDE.md already names. The order that works:

1. A box-resident recipe that runs ONE cell end to end and reports honestly.
2. Run each cell once, by hand, and record the verdict per cell — a cell that
   fails is a finding, not a blocker.
3. Only then a lane, and only over the cells that passed, so the lane starts
   green and a red means something.
4. A gate that a Runtime cell's test binary is named by at least one recipe —
   the check G1's doc comment already claims to be.

Phase 433 owns this work.
