# Entry packages

An **Entry pkg** is the binary that boots a topology on a specific board.
Where a Node pkg is a library (no `fn main`) and a Bringup pkg is purely
declarative, the Entry pkg is the one thing that actually runs: it names a
board, wires the runtime, and — for multi-node setups — points at a Bringup
pkg that describes which nodes should be launched.

You have one Entry pkg per deploy target. A workspace targeting both a native
workstation and a bare-metal Cortex-M board has two Entry pkgs that reference
the same Node pkgs; only the board and (optionally) the launch target differ.

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

Then provision the native host (the canonical first Entry pkg target;
for Zephyr / FreeRTOS / ESP32 swap in the matching `nros setup` board):
```sh
nros setup native --rmw zenoh
```

## Package layout

```
src/native_entry/
├── package.xml
├── Cargo.toml           # [[bin]] + deps on node pkgs + board crate
│                        # + [package.metadata.nros.entry]
└── src/main.rs          # nros::main!(launch = "demo_bringup");
```

No library code lives here. The Entry pkg links the Node pkg rlibs and hands
them to the runtime that `nros::main!()` generates.

## `Cargo.toml` metadata

The `[package.metadata.nros.entry]` table tells the CLI which deploy target
this binary is built for. The embedded example
`examples/qemu-arm-baremetal/rust/talker-rtic/` uses:

```toml
[package.metadata.nros.entry]
deploy = "rtic-mps2-an385"
```

A native Entry pkg that references a Bringup pkg looks like:

```toml
[package.metadata.nros.entry]
deploy = "native"

[package.metadata.nros.deploy.native]
board     = "posix"
rmw       = "zenoh"
domain_id = 0
```

`deploy` is the key that `nros check` and the Entry macro use to
find the board crate and verify the topology. Keep it short and descriptive —
it becomes the identifier in `nros plan` output and in `system.toml`'s
`[image.<id>]` table when you later add a Bringup pkg.

## `nros::main!()` — the forms

```rust
// 1. Single-node self-bringup: reads [package.metadata.nros.entry] deploy
//    from Cargo.toml and boots the Node pkg that is the only member of
//    this workspace (or the one marked default).
nros::main!();

// 2. Single-node, explicit board type.
nros::main!(board = LinuxBoard);

// 3. Multi-node (CANONICAL): name your INPUT — the Bringup pkg (and
//    optionally a launch file inside it; default is its default launch).
//    `nros sync` resolves it into a SystemModel build artifact that the
//    macro bakes from.
nros::main!(launch = "demo_bringup");
nros::main!(launch = "demo_bringup:variant_b.launch.xml");

// 4. Multi-host slice: point at a PER-HOST model. The launch file gates
//    nodes on a `host` launch argument (`if=` conditions), and the model
//    is resolved with `host:=robot1`, so it already contains only this
//    host's nodes — no extra macro key needed.
nros::main!(launch = "demo_bringup:multihost.launch.xml", args = [("host", "robot1")]);

// 5. DEPRECATED (expert override): name the resolved model ARTIFACT
//    directly instead of the input. Skips the sync-freshness contract;
//    warns at bake time (phase-330 W7).
nros::main!(model = "demo_bringup");
```

`launch` and `model` are mutually exclusive. `launch` is the **canonical**
spelling (phase-330 W7): it names your *input* — the bringup package and
launch file — and the entry consumes the SystemModel that `nros sync`
resolves from it. The same resolved artifact drives the Linux runtime
(play_launch) and every embedded image, so contract budgets, tiers, and
QoS never drift between runtimes. `model` names the resolved artifact
directly — a deprecated expert override. Produce/refresh the model with:

```console
$ nros sync    # resolves models into <ws>/build/nros/models/<bringup>/
```

`nros sync` (the pre-build step every workflow already runs) re-resolves
the model whenever a bringup's launch XML or `system.toml` is newer than
it. **SystemModels are build artifacts — never commit one** (the
`check-no-tracked-models` gate enforces this).

`nros sync` is the only verb you need. Under the hood it runs the pinned
`nros-launch-resolve` helper (RFC-0060 layer 2; **contributors** build it
in-tree via `just setup-launch-resolve`) and invokes it by **absolute path** — never by a
bare name on `$PATH`, because an unrelated ROS 2 `play_launch` on `PATH` used to
win that race and break every platform's fixture build (issue 0285). The macro reads `[package.metadata.nros.entry]` at compile
time to select the right board and executor backend; the same model resolves
its tier table for whichever RTOS that board targets. On Embassy / RTIC targets it emits the
framework-specific `#[embassy_executor::main]` or `#[rtic::app]` body so your
`src/main.rs` stays a single line.

The real `examples/qemu-arm-baremetal/rust/talker-rtic/src/main.rs` collapses
to exactly this:

```rust
#![no_std]
#![no_main]

use panic_semihosting as _;

nros::main!();
```

## Escape hatch

If you need more control than the macro provides — custom startup ordering,
hardware init before the runtime, or a fully manual executor loop — you can
bypass `nros::main!()`:

```rust
// Option A: delegate init to the board crate, supply your own closure.
<LinuxBoard as BoardEntry>::run(|runtime| {
    let node = runtime.create_node("talker", "/", &Default::default())?;
    // ...
    Ok(())
});

// Option B: fully manual — no board crate.
let executor = nros::Executor::open(&ExecutorConfig::default())?;
// wire nodes, spin manually ...
```

Option A is the right choice when you need to run something before the first
spin (e.g. DMA setup, flash unlock). Option B is there for board-bringup
authors adding a new platform.

## Running a native Entry pkg

The verified path for the canonical Rust workspace is `cargo run -p native_entry`.
Start a Zenoh router first, then boot the Entry binary from the workspace root:

```bash
# in another shell:
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd &

cargo run -p native_entry
```

`native_entry` opens the executor against the router, registers `talker` +
`listener` (composed into a single process), and runs the topology.

The canonical Rust workspace is at `examples/workspaces/rust/`.
For Zephyr, QEMU, ESP-IDF, and other non-native targets, use the platform's
native build/run tool.

## Running on Zephyr

On Zephyr the RTOS framework *is* the workflow: `west build` is the build
verb, Kconfig selects the RMW, and the Entry is an ordinary Zephyr
application. There is no `nros build` / `nros launch` build path here, and you
do not type the RMW as a Cargo `--features` flag or bake the board into the
package.

```sh
source ./activate.sh

# Provision message bindings once. This is platform-agnostic workspace
# provisioning (sibling to `west update` / `rosdep`), NOT a compile step —
# the same `nros sync` output feeds every board and every RMW.
nros sync

# west is the build verb. Choose the board with `-b`, and select the RMW with
# the matching Kconfig overlay via -DCONF_FILE. The Entry source never changes.
west build -b native_sim/native/64 src/zephyr_entry \
    -- -DCONF_FILE="prj.conf;prj-zenoh.conf"

west build -t run            # native_sim; `west flash` for hardware
```

Swap `-b native_sim/native/64` for any other Zephyr board (`-b nrf52840dk/nrf52840`,
`-b stm32f4_disco`, …) and `prj-zenoh.conf` for `prj-xrce.conf` /
`prj-cyclonedds.conf` to pick a different RMW — nothing else changes. **One
Zephyr Entry pkg (`src/zephyr_entry/`) covers every Zephyr board**: unlike the
board-specific FreeRTOS / ThreadX Entries, Zephyr owns its board abstraction,
so the board is chosen at `west build -b` time rather than baked into the
package (see [One Entry pkg per board](#one-entry-pkg-per-board)).

The Entry source is identical to the native, FreeRTOS, and ThreadX Entries —
the same one-line macro, with the committed SystemModel as the single source
of truth for the node set:

```rust
// examples/workspaces/rust/src/zephyr_entry/src/lib.rs
nros::main!(launch = "demo_bringup");
```

## One Entry pkg per board

Each deploy target gets its own Entry pkg. A workspace that runs on both
`native` and `rtic-mps2-an385` would have two Entry pkgs that share the same
Node pkg library:

| Entry pkg | `deploy` key | Board crate |
|---|---|---|
| `native_entry` | `"native"` | `nros-board-linux` |
| `mps2_entry` | `"rtic-mps2-an385"` | `nros-board-mps2-an385` (`rtic` feature) |

Both reference the same `talker_pkg` and `listener_pkg` Node pkg rlibs. The
board crate provides the `BoardEntry` impl and any hardware-specific
initialisation; the Node pkgs are board-agnostic.

**Zephyr is the exception — one Entry per *RTOS*, not per board.** Zephyr
already owns its board abstraction, so a single `zephyr_entry` covers
`native_sim`, `nrf52`, `stm32`, `aemv8r`, … with the board chosen at
`west build -b <board>` time. Contrast FreeRTOS / ThreadX, whose board crates
are board-specific, so each of those Entries bakes one board. See
[Running on Zephyr](#running-on-zephyr).

The `examples/qemu-arm-baremetal/rust/talker-rtic/` example demonstrates the
embedded shape: `deploy = "rtic-mps2-an385"` + `nros::main!();` on a
`no_std / no_main` binary that delegates everything to the `RticMps2An385`
board crate.

## C / C++ Entry packages

C and C++ Entry packages use the same role split through CMake. See
[C / C++ multi-node workspaces](./workspace-cpp.md).

## Where to go next

- [Role reference](../user-guide/component-and-entry-pkg.md) — full reference for all three roles.
- [Bringup packages](./workspace-bringup.md) — the `system.toml` + launch XML that an Entry pkg points at.
- [Node packages](./workspace-node-pkgs.md) — the Node pkgs your Entry pkg links.
- [Project layout](./workspace-from-app-node.md) — the full 3-role picture and when to use it.
