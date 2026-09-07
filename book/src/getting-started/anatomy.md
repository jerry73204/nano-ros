# Anatomy of What You Just Built

Four directories, three roles, one configuration file. Everything you
will ever add to this project is another instance of one of these three
roles — the shape never changes, it only gains packages.

If you come from ROS 2, each role is something you already know:

| Your workspace | Role | In rclcpp terms |
|---|---|---|
| `src/talker_pkg/`, `src/listener_pkg/` | **Node package** | a composable node (`rclcpp_components`) |
| `src/demo_bringup/` | **Bringup package** | `<pkg>_bringup` — launch files + config, no code |
| `src/robot_entry/` | **Entry package** | the composition container / `main()` |

## Node packages — the code

A node package is one node as a reusable library. The C++ shape is a
class with a `configure(nros::Node&)` method that binds real member
callbacks:

```text
src/talker_pkg/
├── CMakeLists.txt          # nano_ros component library
├── package.xml             # ROS package manifest — deps live here
├── include/talker_pkg/Talker.hpp
└── src/Talker.cpp          # bind_timer → publish on /chatter
```

`package.xml` is standard ROS: message dependencies (`std_msgs`) are
declared there and code-generated during the build. You never hand-write
message types.

## The bringup package — the one configuration file

`src/demo_bringup/` holds **no code**: a launch file and `system.toml`.
This is where every configuration axis lives:

```toml
[system]
name = "demo"
rmw = "cyclonedds"                   # the RMW axis
domain_id = 0
default_launch = "system.launch.xml"

[[component]]                        # the topology: which nodes exist
pkg = "talker_pkg"
class = "talker_pkg::Talker"
name = "talker"

[image.native]                       # one block per BUILDABLE image
board = "native"

[host.native]                        # WHERE the nodes run; no `nodes` = all
```

The launch file is the **ROS 2 launch XML schema, verbatim** —
`<node>`, `<param>`, `<remap>`, `<group>`, `<include>` with `$(find)` /
`$(var)` substitutions. A launch file from nav2 or Autoware pastes in
unchanged (Python `.launch.py` is not supported). The scaffold's launch
even carries a per-topic QoS override to show the plumbing:

```xml
<node pkg="talker_pkg" exec="talker" name="talker">
  <param name="qos_overrides./chatter.publisher.reliability" value="best_effort"/>
</node>
```

When you later target hardware, you add an `[image.<id>]` block per
board — the code and topology stay put, and no new directory appears. That is
the growth rule: **configuration is a new block, never a restructure.**

## The entry package — the deliverable

`src/robot_entry/` is the binary. Its CMakeLists calls
`nano_ros_entry(... BRINGUP ... LAUNCH default)`, which at configure
time resolves the bringup's launch file and generates a `main()` that
constructs each declared component, applies launch parameters and QoS
overrides, and runs them on one executor — the launch product *is* the
binary. There is no separate `ros2 launch` step; running the entry is
launching the system.

One binary per deployable image — but *image* is a row, not a directory. A
robot with a perception image and a control image is one workspace, one
bringup, and two `[image.*]` blocks in its `system.toml`; `nros build` derives
each entry from `(launch, args, board)` and generates it under `build/`. The
scaffold ships this one as a package so you can read it; deleting the directory
is what makes the next build generate it instead. See
[Images](workspace-entry-pkg.md).

## What was generated vs. what is yours

Everything under `src/` is yours to edit. The build tree (`build/`)
holds everything derived: generated message code, the resolved system
model, the entry's generated `main()`. Nothing derived is ever
committed — if you can regenerate it, it does not belong in git.

## The same anatomy in Rust

`nros new --workspace --lang rust` produces the identical shape: node
packages are library crates with `nros::node!(Talker)`, the entry is a
one-line `nros::main!(launch = "demo_bringup", spin = "forever")`, and
the bringup directory is byte-for-byte the same idea. One difference:
Rust message codegen runs as a pre-build step (`nros sync`) instead of
inside CMake.

## Next

- The Quick Start continues: [Your own message
  package](your-own-msg-package.md)
- Add a second talker or your own node: [Node packages](workspace-node-pkgs.md)
- Point it at real hardware: [How Integration Works](how-integration-works.md)
- The full role reference: [Bringup packages](workspace-bringup.md) ·
  [Images](workspace-entry-pkg.md)
