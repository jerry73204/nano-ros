---
id: 1127
title: "the 17 live-peer cells run on every push to main and SKIP on every one — a skip is not a verdict, and nothing tells 'skipped forever' apart from 'covered'"
status: open
type: bug
area: testing, rmw
severity: high
found: 2026-09-06
related: [0352, 0445, 0759, 0791, 0903, 1055]
---

# They run. They skip. Forever.

`nros_tests::interop::CELLS` is the intent list for every test whose subject is
a LIVE ROS 2 peer (RFC-0051, phase-324). It has 18 rows, 17 of them `Runtime`
(one is a declared carve-out with no test), across 11 test binaries.

**They are invoked, routinely.** The root `just test-all` (`justfile:2172`)
runs `cargo nextest --workspace` with **no exclude filter**, so all 11 are in
it; `just ci tier1` is preconditions + check + `rust-rtos-link-check` +
`test-all`; and `host-tests.yml` runs `just ci tier1` on **every push to
`main`**.

**And they skip, every time.** That runner has no ROS. `host-tests.yml`
installs none — no `ros-humble` package, no `setup-ros` action, no
`ROS_DISTRO` — and the workflow's own `on:` comment says so out loud:

> This lane needs ROS on the runner and its two jobs failed on every pull
> request; a red that is STRUCTURAL is not a gate, it is the noise that
> teaches people to ignore CI.

So every one of the seventeen resolves to `nros_tests::skip!`, the junit
rewrite turns that into a skip, and the lane is green. **A skip is not a
verdict.** Nothing in the system distinguishes "skipped on every host since the
day it was written" from "covered" — which is exactly the absorbing-verdict
class of issue 0445 (a STALE verdict replaces whatever the fixture would have
done at runtime with a message explaining itself), one lane over.

## The second problem: no focused runner for most of them

`--workspace` can reach a binary but cannot let a human run *one cell against a
live peer*, which is what verifying any of this requires. Of the 11 binaries,
three are named by a recipe:

| test binary | cells | focused runner |
| --- | --- | --- |
| `interop_e2e` | 5 | `just native test-ros2`, `just native test-ros2-lifecycle` |
| `xrce_ros2_interop` | 2 | `just xrce test-ros2` |
| `params` | 1 | `just native test-ros2-params` |
| `graph_interop` | 2 | **none** |
| `qos_zephyr_ros2_interop_e2e` | 1 | **none** |
| `qos_override_e2e` | 1 | **none** |
| `rust_multi_node_per_node_graph` | 1 | **none** |
| `cpp_multi_node_entry` | 1 | **none** |
| `declarative_bridge_zenoh_to_cyclonedds` | 1 | **none** |
| `declarative_bridge_zenoh_to_xrce` | 1 | **none** |
| `bridge_zenoh_to_cyclonedds` | 1 | **none** |

`just native test-all` aggregates the three, and is itself called by nothing.
`just native test` (a different lane) *does* exclude the ros2 groups — that
exclusion is correct on its own terms and is not the problem here; misreading
it as the root sweep's is what produced the first version of this issue.

## Why nobody noticed

`matrix_fixture_coverage.rs` G1 is the gate closest to this, and its own doc
comment says "A Runtime cell nothing runs … fails here". What it asserts is:

```rust
if !tests_dir.join(format!("{}.rs", c.test)).is_file() {
```

The test FILE existing. G2 checks the build channel can build the coordinate,
G3 checks the build recipe's spelling, G4 checks the peer declaration, G5
checks a `fixtures.toml` row produces the coordinate — five gates around the
cell, and **not one of them looks at whether the cell has ever produced a
result.** They are all statements about declarations.

This is the repo's recurring shape stated in phase-393's own closing warning:
*a slot's EXISTENCE reads as coverage*. Here it is one level up, and the
existence in question is a green tick.

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

Not "wire the tests in" — they are already wired in, and that is the point.
Nor a red lane, which has no signal capacity (the failure mode CLAUDE.md
names). The order that works:

1. A box-resident recipe that runs ONE cell end to end and reports honestly.
2. Run each cell once, by hand, and record the verdict per cell — a cell that
   fails is a finding, not a blocker.
3. Only then a lane, and only over the cells that passed, so the lane starts
   green and a red means something.
4. A gate that a Runtime cell's test binary is named by at least one FOCUSED
   recipe — what G1's doc comment claims to be, and what `--workspace` cannot
   substitute for.
5. **The durable fix is making a permanent skip visible.** A cell that has
   never produced a non-skip result on any host is indistinguishable from a
   passing one today. Whatever form that takes — a per-cell last-verdict
   record, a skip budget, a report — it is the only one of these five that
   stops the problem recurring.

Phase 433 owns this work.
