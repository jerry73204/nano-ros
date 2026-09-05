# Bringup packages

A **Bringup pkg** is the declarative glue that ties your Node packages together
into runnable programs. It owns the launch file, the wiring between nodes, and
the `[image.*]` table saying which programs get built out of them — all without
any compiled code of its own.

> **Pre-requisite:** You've scaffolded your Node packages following the
> [Node packages](./workspace-node-pkgs.md) guide. This page adds the
> `demo_bringup` layer that turns them into something you can build and run.

---

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

---

## What a Bringup pkg is

A Bringup pkg is **pure declarative** — no `Cargo.toml`, no `CMakeLists.txt`,
no `src/`. Its job is to describe *which* nodes run, how they're wired, and
where they deploy. Naming convention: `<system>_bringup` (aliased `<system>_launch`),
matching nav2 / Autoware / turtlebot3.

A multi-node workspace needs exactly one per logical system. It is where
`[image.*]` lives, so it is the package a buildable workspace cannot do
without — `nros build` reads that table to decide what programs exist. (The
single-package copy-out shape under `examples/<platform>/<lang>/` has no
bringup and no images; it boots itself with a bare `nros::main!()`.)

---

## Anatomy

```
src/demo_bringup/
├── package.xml          # ROS 2 manifest; <exec_depend> per node pkg
├── system.toml          # [system] + [[component]] + [image.<id>] + [host.<name>]
├── launch/
│   └── system.launch.xml   # ROS 2 launch schema, verbatim
└── config/                 # optional — params.yaml, per-target overrides
```

---

## `system.toml` — node wiring + deploy targets

The `system.toml` is the machine-readable topology. It lists every node the
system runs, its class path, and one or more deploy targets (which board, which
RMW, which domain).

Below is a minimal two-node example adapted from the real fixture at
`packages/testing/nros-tests/fixtures/orchestration_e2e/demo_pkg_bringup/system.toml`:

```toml
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::Talker"
name = "talker"

[[component]]
pkg = "listener_pkg"
class = "listener_pkg::Listener"
name = "listener"

[image.native]
board = "native"

[host.native]
```

Key fields:

| Field | Meaning |
|---|---|
| `[system] name` | Logical system name; used by `nros plan`/`check` |
| `[system] rmw` | Default RMW for all components (`zenoh`, `xrce`, `cyclonedds`) |
| `[system] domain_id` | ROS 2 domain (compile-time on embedded, runtime env on host) |
| `[[component]] pkg` | The ROS package name (matches `<name>` in `package.xml`) |
| `[[component]] class` | Fully-qualified Rust type (`crate::TypeName`) |
| `[[component]] name` | Node name at runtime |
| `[image.<id>]` | A buildable image; read by `nros build`, `nros check` and Entry codegen |
| `[image.<id>] board` | The board this image is built for; its descriptor supplies the rustc triple |
| `[host.<name>]` | A machine nodes run on. No `nodes` means every node |
| `[board_config.<board>]` | Site facts for that board (SDK roots, netstack) |

For multi-domain setups or cross-domain bridges add `[[domain]]` and
`[[bridge]]` sections — see `docs/design/0024-multi-node-workspace-layout.md` §11
for the full schema.

---

## `launch/system.launch.xml` — ROS 2 launch schema

The launch file uses the **ROS 2 launch XML schema verbatim** — nano-ros reads
it with the same parser so existing nav2/Autoware/turtlebot3 XML pastes in and
Just Works.

```xml
<launch>
  <node pkg="talker_pkg" exec="talker" name="talker"/>
  <node pkg="listener_pkg" exec="listener" name="listener"/>
</launch>
```

### v1 tag set

| Tag | Purpose |
|---|---|
| `<launch>` | Root element |
| `<arg name="…" default="…"/>` | Declare a launch argument |
| `<node pkg="…" exec="…" name="…"/>` | Instantiate a node |
| `<param name="…" value="…"/>` | Set one parameter inline (nested inside `<node>`) |
| `<param from="…param.yaml"/>` | Load a ROS parameter FILE (nested inside `<node>`) |
| `<remap from="…" to="…"/>` | Topic/service remapping (nested inside `<node>`) |
| `<group ns="…">` | Namespace a group of nodes |
| `<include file="…"/>` | Nest another launch file |

### Substitutions

- `$(find <pkg>)` — resolves to the package's install/source path
- `$(var <arg>)` — expands a launch argument
- `$(env <name>)` — reads an environment variable

A richer example using args and remapping (taken from the real fixture):

```xml
<launch>
  <arg name="talker_name" default="talker" />

  <node pkg="talker_pkg" exec="talker" name="$(var talker_name)" output="screen">
    <param name="rate_hz" value="25" />
    <remap from="chatter" to="/chatter" />
  </node>

  <node pkg="listener_pkg" exec="listener" name="listener"/>
</launch>
```

### Parameter files

`<param from="…"/>` takes a standard ROS parameter file. Relative paths resolve
against the launch file's own directory.

```yaml
# config/talker.param.yaml
/**:                      # applies to every node
  ros__parameters:
    use_sim_time: false
talker:                   # applies to the node named `talker` only
  ros__parameters:
    rate_hz: 25
    limits:
      max_accel: 1.5      # reaches the node as `limits.max_accel`
```

```xml
<node pkg="talker_pkg" exec="talker" name="talker">
  <param from="config/talker.param.yaml"/>
  <param name="rate_hz" value="50"/>   <!-- wins over the file -->
</node>
```

Precedence matches ROS: the `/**` wildcard block first, then the
node-specific block, then inline `<param name= value=>` last. A node block
may be keyed by fully-qualified name (`/ns/talker`), bare name (`talker`),
or `/**/talker`.

These values are resolved and **baked into the generated entry at build
time** — embedded targets do no runtime file loading, the same approach the
domain ID uses. Editing the YAML re-runs codegen. A `from=` naming a file
that doesn't exist is a build error, not a silent skip.

> **Note:** the baked values are seeded through the parameter services, so
> they reach nodes only when `param_services` is enabled in `system.toml`.

> **Note:** Python `.launch.py` files ARE resolved by `nros sync` (the
> RFC-0060 resolver executes them with its bundled CPython; `$(find-pkg-share)`
> against ROS packages needs a sourced `AMENT_PREFIX_PATH`). The XML schema
> above is the native authoring format.

---

## `package.xml`

A standard ROS 2 manifest. List each Node package as an `<exec_depend>`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>demo_bringup</name>
  <version>0.1.0</version>
  <description>Bringup package for the demo system</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>talker_pkg</exec_depend>
  <exec_depend>listener_pkg</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

No `<build_depend>` entries — there is nothing to compile.

---

## Workflow: check → run

Once your Bringup pkg is written, `nros check` validates it and `nros build`
turns one of its images into a program:

```bash
# 1. Lint the bringup pkg (pure-declarative check — no Cargo.toml, stray files, etc.)
nros check --bringup src/demo_bringup

# 2. Lint the whole workspace (pkg/class rows, duplicate system.toml, etc.)
nros check --workspace .

# 3. Build an image. With no name, `nros build` lists what this workspace declares.
nros build native

# 4. Run it. Zenoh needs a router first, in another shell:
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd &
```

The binary boots every node the launch file names, composed into one process.
It lands beside the build `nros build` drove — under `target/` on the cargo
driver, `build/<coordinate>/cmake/` on the cmake one.

Both `nros check` forms pass for the canonical template at
`examples/workspaces/rust/`.

> **Caveat — `nros plan` with this template**
>
> - **`nros plan demo_bringup`** resolves a topology into `plan.json` for
>   static type/QoS checks, but it currently requires pre-collected
>   source-metadata sidecars (`record.json` + per-pkg `_metadata/*.json`).
>   The automatic metadata-build path (`nros metadata --build`) is not yet
>   wired for lib-only Node pkgs, so `nros plan` does not produce a plan
>   straight from this template. See
>   `packages/testing/nros-tests/fixtures/orchestration_e2e/` for the
>   pre-collected-sidecar pipeline.
>
> The canonical template README at
> `examples/workspaces/rust/README.md` is the source of
> truth for the current CLI state.

---

## Runnable copy-out

`examples/workspaces/rust/` is the canonical Rust workspace that pairs with
this guide. Copy the whole directory out and rename the packages, then:

```bash
nros setup native
nros build native
```

`nros build` walks the packages, resolves the image, checks the toolchain,
generates the root manifest and the entry, and hands off to cargo. The older
`nros sync` / `nros codegen-system` / `nros check` sequence still works; it is
just no longer something you have to type.

The workspace README at `examples/workspaces/rust/README.md`
documents the exact CLI commands that are verified green today.

---

## Naming a launch file from an entry

The `nros::main!` macro's `launch =` argument names the bringup package and,
optionally, a launch file within it — the SAME inputs you author, and the same
pair an `[image.*]` row names. You will read this in a generated entry, and
write it yourself only in a single-package project or a materialized one. The
build resolves the pair to a SystemModel under
`<workspace>/build/nros/models/` (run `nros sync`, or let the build system
drive it); you never reference the model file, though you can inspect it
there:

```rust
// Multi-node: the bringup's default launch (system.launch.xml)
nros::main!(launch = "demo_bringup");

// A named launch file in the bringup pkg
nros::main!(launch = "demo_bringup:sim.launch.xml");

// A launch-argument binding, declared as [[model]] in system.toml
nros::main!(launch = "demo_bringup:multihost.launch.xml", args = [("host", "robot1")]);
```

The [Images](./workspace-entry-pkg.md) page covers the macro forms and the
generated entry in full.

---

## Where to go next

- [Images](./workspace-entry-pkg.md) — the `[image.*]` row that becomes a program
- [Role reference](../user-guide/component-and-entry-pkg.md) — full reference for the package roles and the image table
- [Project layout](./workspace-from-app-node.md) — start here if you haven't read it yet
