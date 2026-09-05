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
   legacy C node, but host the composed system in a C++ or Rust Entry
   pkg. The Node-pkg register ABI is language-neutral, so a C Node pkg
   can link into the same Entry binary as C++ or Rust Node pkgs.

That's when you split your project into a multi-node workspace.

## Canonical layout

Start with the whole project before diving into the parts:

```text
my_robot_ws/
├── Cargo.toml                  # Rust workspace root, or CMakeLists.txt for C/C++
└── src/
    ├── talker_pkg/             # Node pkg: reusable node logic, no main()
    ├── listener_pkg/           # Node pkg: another reusable node
    ├── robot_bringup/          # Bringup pkg: launch XML + system.toml
    └── native_entry/           # Entry pkg: one runnable binary for one board
```

The roles are deliberately separated:

| Role | Owns | Does not own |
|---|---|---|
| **Node pkg** | Publishers, subscriptions, timers, services, actions, callback bodies | Board choice, launch topology, `main()` |
| **Bringup pkg** | Which nodes run, names, remaps, parameters, per-target topology | Compiled code |
| **Entry pkg** | Board/runtime selection and the runnable binary | Node behavior |

A typical product has many Node pkgs, one Bringup pkg per logical system,
and one Entry pkg per board or deploy target. The same `talker_pkg` and
`listener_pkg` can be linked into a native host Entry pkg for integration
testing and a Cortex-M Entry pkg for hardware.

## Reading order

This group starts broad and then drills into each part:

1. **Project layout** — this page: when to split and how the roles fit.
2. **Node packages** — reusable node libraries with `nros::node!`.
3. **Bringup packages** — launch XML, `system.toml`, remaps, parameters.
4. **Entry packages** — the board-specific binary that boots the topology.
5. **C / C++ multi-node workspaces** — the same structure through CMake.
6. **Mixed-language workspaces** — C Node pkgs hosted by C/C++ Entry pkgs.
7. **Role reference** — metadata fields and macro forms in reference style.

## Prereqs

Pick one path from a fresh checkout — `just` is NOT a prereq.

**A. Front door** (bare machine OK — no Rust, no `just`):
```sh
./scripts/bootstrap.sh
```
Installs rustup if needed and builds the in-tree `nros` CLI from
source at `packages/cli/target/release/nros`, leaving it on PATH for
this shell (in a checkout, the tree's own build is the binary this tree
accepts — RFC-0090 / phase-431).

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
`fn main()` — that lives in the Entry pkg. One Node pkg per node.
Think of it as a composable building block: the same `talker_pkg` lib
can be assembled into a native binary *and* an embedded binary without
any source change.

**Bringup pkg** — a purely declarative directory that owns the launch
topology. It contains a `package.xml`, a `system.toml` (listing which
nodes run, how they're wired, per-target deploy config), and a
`launch/` directory with ROS 2 launch XML. **No `Cargo.toml`, no
compiled code.** Naming convention `<system>_bringup`, matching nav2 /
Autoware / turtlebot3. This role is *optional* — a workspace with a
single deploy target can fold the launch files and `system.toml`
directly into the Entry pkg.

**Entry pkg** — a `bin` crate that boots a topology on one specific
board. It carries `[package.metadata.nros.entry]` with `deploy =
"<board>"` and a `src/main.rs` that is just `nros::main!(...)`. One
Entry pkg per deploy target. The Entry pkg links in all Node pkg libs,
links in the board crate, and hands control to the nano-ros runtime.

The app-node shape you already know (`examples/native/rust/talker/`) is
effectively a *fused* Entry + Node pkg: a single package that is both
the logic and the boot point. That fusion is fine — and encouraged —
for single-node work. Only split when you actually need the
flexibility.

## ROS 2 ↔ nano-ros command map

If you're coming from ROS 2, here's the mapping of the commands you
already know:

| ROS 2 | nano-ros | Notes |
|---|---|---|
| `ros2 pkg create` | `nros new <name> --platform <plat> [--lang <lang>]` | scaffolds a Node pkg |
| `colcon build` | `cargo build` (Rust) / `cmake --build build` (C++) | use the underlying tool directly |
| `ros2 launch <pkg> <file>` | `cargo run -p <entry_pkg>` | composed Entry pkg IS the launch product (Phase 212.N + 222.D); the old launch wrapper was removed in nros 0.5.0 |
| (plan/validate) | `nros plan` → `nros check` | resolve + statically check the topology |
| `ros2 run <pkg> <exe>` | run the Entry pkg binary (`cargo run`) | one Entry pkg per board |

Build verbs (`cargo` for Rust, `cmake` for C/C++, `west` for Zephyr,
`idf.py` for ESP-IDF) are used directly — there is no CLI build
indirection. The composed Entry pkg binary IS the launch product:
one Entry pkg = one binary = one process. Multi-process orchestration
(equivalent to multiple `ros2 launch` nodes in separate processes) is
a separate Entry pkg per deploy + a shell script / tmux session, not
a CLI verb.

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
3. [Entry packages](./workspace-entry-pkg.md) — write
   the Entry pkg that boots everything together.
4. [C / C++ multi-node workspaces](./workspace-cpp.md) — use the same
   project shape with CMake.

For C Node pkgs hosted by a C++ Entry pkg, see
[Mixed-language workspace](./workspace-mixed-language.md).

For the full API reference covering all three roles, see
[Role reference](../user-guide/component-and-entry-pkg.md).
