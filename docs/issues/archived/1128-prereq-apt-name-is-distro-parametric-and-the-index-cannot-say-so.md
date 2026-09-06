---
id: 1128
title: "A `[prereq.*]` apt name can be distro-parametric, and the index can only spell one — so CI composes its own"
status: resolved
type: tech-debt
area: cli, ci
severity: low
found: 2026-09-06
resolved_in: "phase-422 (follow-on to W5's audit)"
related: [0609, 0653, 0996, 1101]
---

# One package, two spellings, and only one of them was data

`[prereq.ros-rmw-zenoh-cpp]` declared `apt = ["ros-humble-rmw-zenoh-cpp"]` while
the package name is distro-parametric. The index had no way to say so — its own
`why` said as much — so `just ci provision-zenohd` composed
`ros-${ROS_DISTRO}-rmw-zenoh-cpp` itself. Two spellings of one package, agreeing
only on humble: on a jazzy runner the lane installed `ros-jazzy-…` while
`nros setup --system` went on reporting `ros-humble-…` missing.

**Resolved** by making the parameter data:

```toml
apt = ["ros-{ros_distro}-rmw-zenoh-cpp"]     # expands from $ROS_DISTRO, default humble
```

```
$ nros setup --system --role package | grep -o 'ros-[a-z]*-rmw-zenoh-cpp'
ros-humble-rmw-zenoh-cpp
$ ROS_DISTRO=jazzy nros setup --system --role package | ...
ros-jazzy-rmw-zenoh-cpp
```

`provision-zenohd` asks `scripts/sdk/prereq-packages.py` for the name now. It
keeps `$ROS_DISTRO` for the /opt/ros PREFIX probe, which is a path on the host
rather than a package name.

## What the fix cost, and the guard that came with it

The expansion has TWO implementations that cannot call each other —
`PrereqContext::expand` in the CLI, `expand()` in `prereq-packages.py` (which
exists so a job needing two package names need not build the CLI first,
phase-413 W3). Two implementations of one rule is this repo's recurring drift;
what makes it survivable is that the rule is **one line long in each**, and that
holds only while the vocabulary is one name.

`check-prereq-placeholders` holds it to one: it refuses an unknown placeholder,
refuses a vocabulary entry the index does not use, and checks both expanders by
**exercising** them. The first version grepped, and a Python `expand` mutated to
return its input unchanged still passed — the name was in a comment three lines
away. Three mutations verified.

`PrereqContext` is a struct, not an env read inside `packages_for`, so the
expansion is pure and its tests need no `set_var` (issue 1101's hazard). The
environment is read once, at the CLI edge.

## What was deliberately NOT pulled in

The issue asked whether a second parametric prereq exists. For **prereqs**, no —
this is the only one. For the **shape**, yes: `scripts/ros/domain-bridge-repro.sh`
(`ros-${DISTRO}-domain-bridge`) and `docker/ros-editions/Dockerfile` (six
packages) also compose by hand. Neither names a package the index declares, so
converting them is a separate decision about what belongs in `[prereq.*]` rather
than a mechanical follow-through.
