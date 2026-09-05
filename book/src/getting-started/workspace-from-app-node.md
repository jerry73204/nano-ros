# Multi-Node Project Layout

You built the single-file talker in
[`examples/native/rust/talker/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/rust/talker):
one `main.rs`, one `Cargo.toml`, one package — open a terminal, run
`cargo run`, done. That shape covers a lot of ground: demos, driver
tests, single-purpose microcontroller apps.

At some point it stops being enough.

## When you outgrow one app

Three common triggers:

1. **Two or more nodes.** You want a talker *and* a listener, or a
   sensor driver alongside a control loop. Splitting them into one
   entry binary per node file works until you need to tune their
   wiring or deploy them to different boards.

2. **A shared launch topology.** You want to describe *once* how nodes
   are named, remapped, and parameterised — and reuse that description
   across a dev laptop, a hardware bring-up board, and a sim target.

3. **Multiple deploy targets.** The same talker logic goes on a native
   Linux host for integration testing and on an STM32F4 for
   production. The node logic is identical; only the boot and board
   differ.

4. **Mixed implementation languages.** You want to keep a C driver or
   legacy C node alongside C++ or Rust ones. The Node-pkg register ABI
   is language-neutral, so a C Node pkg links into the same binary as
   C++ or Rust Node pkgs.

That's when you split your project into a multi-node workspace.

## Canonical layout

Start with the whole project before diving into the parts:

```text
my_robot_ws/
├── .colcon_workspace           # the tracked marker: this directory IS a workspace root
└── src/
    ├── talker_pkg/             # Node pkg: reusable node logic, no main()
    ├── listener_pkg/           # Node pkg: another reusable node
    └── robot_bringup/          # Bringup pkg: launch XML + system.toml + [image.*]
```

Two kinds of package, and that is the whole tracked tree. There is no root
`Cargo.toml` or `CMakeLists.txt` to write and no binary package to write:
`nros build` generates both from the packages it discovers plus the
`[image.*]` table in the Bringup pkg (RFC-0065 D3/D4).

| Role | Owns | Does not own |
|---|---|---|
| **Node pkg** | Publishers, subscriptions, timers, services, actions, callback bodies | Board choice, launch topology, `main()` |
| **Bringup pkg** | Which nodes run, names, remaps, parameters — and the `[image.*]` rows saying which programs get built out of them | Compiled code |

A typical product has many Node pkgs, one Bringup pkg per logical system, and
one `[image.*]` row per board or deploy target. The same `talker_pkg` and
`listener_pkg` are linked into a native host image for integration testing and
a Cortex-M image for hardware, with nothing duplicated between them.

## Reading order

This group starts broad and then drills into each part:

1. **Project layout** — this page: when to split and how the roles fit.
2. **Node packages** — reusable node libraries with `nros::node!`.
3. **Bringup packages** — launch XML, `system.toml`, remaps, parameters.
4. **Images** — the `[image.*]` row that becomes a program, and the one
   platform that still needs a package of its own.
5. **C / C++ multi-node workspaces** — the same structure through CMake.
6. **Mixed-language workspaces** — C Node pkgs beside C++ and Rust ones.
7. **Role reference** — metadata fields and macro forms in reference style.

## Prereqs

Pick one path from a fresh checkout — `just` is NOT a prereq.

**A. Front door** (bare machine OK — no Rust, no `just`):
```sh
./scripts/bootstrap.sh
```
Installs rustup if needed and builds the in-tree `nros` CLI from
source at `packages/cli/target/release/nros`, leaving it on PATH for
this shell (nano-ros is a source distribution — no prebuilt `nros`).

**B. Already have cargo** (equivalent — same build, same binary):
```sh
git submodule update --init packages/cli/third-party/play_launch
cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros
export PATH="$PWD/packages/cli/target/release:$PATH"
```

Every subsequent shell sources the workspace env via one of:
```sh
direnv allow                  # if you use direnv
source ./activate.sh          # bash / zsh
source ./activate.fish        # fish
```

Then provision the native host:
```sh
nros setup native --rmw zenoh
```

## The three roles in practice

**Node pkg** — a `lib` crate that contains one node's logic. It
declares `nros::node!(T)` and carries
`[package.metadata.nros.node]` in its `Cargo.toml`. It has no
`fn main()` — no package you write has one. One Node pkg per node.
Think of it as a composable building block: the same `talker_pkg` lib
can be assembled into a native binary *and* an embedded binary without
any source change.

**Bringup pkg** — a purely declarative directory that owns the launch
topology and the images built from it. It contains a `package.xml`, a
`system.toml` (which nodes run, how they're wired, and an `[image.<id>]`
row per program), and a `launch/` directory with ROS 2 launch XML. **No
`Cargo.toml`, no compiled code.** Naming convention `<system>_bringup`,
matching nav2 / Autoware / turtlebot3.

**Image** — not a package. `[image.native] board = "native"` is a row in
that `system.toml`, and `nros build native` turns it into a program:
`nros build` generates the root build file *and* the entry that links the
Node pkgs and the board crate and hands control to the nano-ros runtime.
A second board is a second row, not a second directory.

The app-node shape you already know (`examples/native/rust/talker/`) is a
single package that is both the logic and the boot point — no bringup, no
images, `nros::main!()` reading its own `Cargo.toml`. That fusion is fine —
and encouraged — for single-node work. Only split when you actually need
the flexibility.

## ROS 2 ↔ nano-ros command map

If you're coming from ROS 2, here's the mapping of the commands you
already know:

| ROS 2 | nano-ros | Notes |
|---|---|---|
| `ros2 pkg create` | `nros new node <name>` | scaffolds a Node pkg and adds its `[[component]]` row |
| `colcon build` | `nros build <image>` | resolves the image, generates the root, hands off to cargo / cmake / west / idf.py |
| `ros2 launch <pkg> <file>` | run the image's binary | the image IS the launch product; the old launch wrapper was removed in nros 0.5.0 |
| (plan/validate) | `nros plan` → `nros check` | resolve + statically check the topology |
| `ros2 run <pkg> <exe>` | run the image's binary | one `[image.*]` row per board |

`nros build` hands off rather than taking over: it generates what the build
system needs and then runs your platform's own tool (`cargo`, `cmake`, `west`,
`idf.py`), so compiler errors stay the compiler's, unchanged. Running the tool
yourself keeps working. One image = one binary = one process; multi-process
orchestration (the equivalent of several `ros2 launch` nodes in separate
processes) is several images plus a shell script or tmux session, not a CLI
verb.

## The app-node shape stays valid

There is no obligation to restructure. If your project is one node on
one board, the app-node shape (`src/main.rs` + one package = both
logic and boot) is perfectly idiomatic and has no runtime penalty. The
three-role split is a *tool* for when you need it, not a gatekeeping
requirement.

## Where to go next

Walk through the multi-node project model step by step:

1. [Node packages](./workspace-node-pkgs.md) — scaffold and
   implement Node pkgs with `nros::node!`.
2. [Bringup packages](./workspace-bringup.md) — declare
   your topology in a Bringup pkg.
3. [Images](./workspace-entry-pkg.md) — declare the
   `[image.*]` row that becomes a runnable program.
4. [C / C++ multi-node workspaces](./workspace-cpp.md) — use the same
   project shape with CMake.

For C Node pkgs alongside C++ and Rust ones, see
[Mixed-language workspace](./workspace-mixed-language.md).

For the full API reference covering all three roles, see
[Role reference](../user-guide/component-and-entry-pkg.md).
