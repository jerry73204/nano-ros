---
id: 1128
title: "A `[prereq.*]` apt name can be distro-parametric, and the index can only spell one — so CI composes its own"
status: open
type: tech-debt
area: cli, ci
severity: low
found: 2026-09-06
related: [0609, 0653, 0996]
---

# One package, two spellings, and only one of them is data

`nros-sdk-index.toml` declares the ROS zenoh RMW as a system prerequisite:

```toml
[prereq.ros-rmw-zenoh-cpp]
apt = ["ros-humble-rmw-zenoh-cpp"]
why = "… NOTE the apt name is DISTRO-PARAMETRIC — `ros-<distro>-rmw-zenoh-cpp`;
       humble is declared here because it is the documented default …"
```

The index's own `why` says the name is parametric and that it cannot express
that, so it pins humble. `just ci provision-zenohd` therefore composes the name
itself:

```bash
distro="${ROS_DISTRO:-humble}"
pkg="ros-${distro}-rmw-zenoh-cpp"
```

Two spellings of one package, agreeing only when `ROS_DISTRO` is humble. On a
jazzy or iron runner, CI installs `ros-jazzy-rmw-zenoh-cpp` while
`nros setup --system` reports `ros-humble-rmw-zenoh-cpp` missing — and the
doctor keeps saying so after the lane has provisioned successfully.

## Why the recipe is not simply wrong

It is careful about the half it can be careful about. Its REMEDY text is derived
(`check-sysdep-remedies` rejected a hand-written apt line there), and the
non-root branch refuses through the lane-skip protocol rather than reporting a
false success (`check-lane-skip-protocol` rejected the bare form). What it
cannot do is ask the index for a name the index has no way to hold.

Found by the phase-422 W5 audit, which was looking for second PRODUCERS of
indexed tools. This is not one — a `[prereq.*]` OS package is a different class,
where composing the command is nano-ros's job and running it is the user's
(RFC-0062). `check-one-producer-per-tool` deliberately does not judge it.

## What a fix would need

Some way for a prereq to declare a name with a hole in it, plus what fills the
hole. Sketch, not a decision:

```toml
[prereq.ros-rmw-zenoh-cpp]
apt = ["ros-{ros_distro}-rmw-zenoh-cpp"]
```

with `ros_distro` resolved from `$ROS_DISTRO` and defaulted, the way
`nros_zenohd_bin` already resolves the ROS prefix (issues 0653/0654). The cost
is a template language in the index, which is exactly the "new machinery for one
consumer" that W2 declined for clang-format — so the honest first question is
whether a second parametric prereq exists. Today, this is the only one.

Until then the drift is bounded and visible: it only bites a non-humble runner,
and this issue is the record that it does.
