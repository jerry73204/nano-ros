# First Node — Rust (Linux)

Build, run, and verify a single nano-ros publisher node on Linux in
about ten minutes. Uses the canonical Zenoh backend. The zenoh router comes from ROS 2
(`ros2 run rmw_zenoh_cpp rmw_zenohd`) — nano-ros no longer ships one.
No ROS 2 on this machine? Use `--rmw cyclonedds` instead: it needs no
router at all (see [Choosing an RMW](../user-guide/rmw-choosing.md)).

> **Stuck?** See [Troubleshooting — First 10 Minutes](./troubleshooting-first-10-min.md) for the common first-build errors.

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

Then provision the native host (installs the zenoh client stack into a
shared store; the router itself comes from your ROS 2 install):
```sh
nros setup native --rmw zenoh
```

See [Install + first build (Linux)](./installation.md) for more.

## Project layout

The talker is a **standalone Cargo package** that pulls nano-ros in
via a path dependency. Three files matter:

```text
examples/native/rust/talker/
├── Cargo.toml          # registry-style deps + `nros sync` patch table
├── package.xml         # ROS-style manifest (drives codegen tooling)
├── generated/          # auto-generated message bindings (gitignored)
└── src/
    ├── main.rs         # one line: nros::main!(spin = "forever")
    └── lib.rs          # the ~65-line talker node body
```

Native (host) talkers read the locator / domain from environment variables
(`NROS_LOCATOR` — legacy alias `ZENOH_LOCATOR` — and `ROS_DOMAIN_ID`)
— no config file is needed. Embedded targets bake their config from
`[package.metadata.nros.deploy.<target>]` instead — the shape shows up
under the Embedded Starters section.

The `Cargo.toml` is the contract that wires nano-ros into your
package. Every example is its own standalone Cargo root (an empty
`[workspace]` table stops `cargo` walking up the filesystem), and
nano-ros crates are declared **registry-style** (`version = "*"`);
the example's tracked `.cargo/config.toml` carries an auto-managed
`[patch.crates-io]` block (written by `nros sync`) that resolves them
into a nano-ros checkout. Verbatim from
[`examples/native/rust/talker/Cargo.toml`](https://github.com/NEWSLabNTU/nano-ros/blob/main/examples/native/rust/talker/Cargo.toml)
(trimmed to the docs-relevant fields — the in-tree file also exposes
`rmw-cyclonedds` / `rmw-xrce` features for the multi-RMW build path):

```toml
[package]
name    = "native-rs-talker"
version = "0.1.0"
edition = "2024"
license = "MIT OR Apache-2.0"
publish = false

[[bin]]
name = "talker"
path = "src/main.rs"

[features]
default   = ["rmw-zenoh"]
rmw-zenoh = ["dep:nros-rmw-zenoh", "nros-board-linux/rmw-zenoh"]

[dependencies]
nros = { version = "*", default-features = false,
         features = ["std", "alloc", "env", "rmw-cffi", "macros"] }
nros-platform-cffi = { version = "*", features = ["posix-c-port"] }
nros-board-linux = { version = "*", default-features = false }
nros-rmw-zenoh = { version = "*", default-features = false,
                   features = ["std", "platform-posix", "ros-humble"],
                   optional = true }
# The ONE non-registry dep: your own generated message crate, by path.
std_msgs = { path = "generated/std_msgs", default-features = false }
log = "0.4"
env_logger = "0.11"

[workspace]
```

**Copying this out of the workspace?** That is the intended workflow —
nano-ros crates are not published to crates.io, so the `version = "*"`
deps resolve through the `[patch.crates-io]` block in
`.cargo/config.toml`, and `nros sync` rewrites that block (plus the
generated message crates) for wherever the directory lives now:

```bash
cp -r examples/native/rust/talker ~/my-talker && cd ~/my-talker
NROS_REPO_DIR=/path/to/nano-ros nros sync
cargo build && RUST_LOG=info cargo run
```

Prefer vendoring instead? `examples/templates/multi-package-workspace/`
documents the path-dep workspace layout.

## Configure

Three runtime knobs, each an env override on a built-in default:

| Knob | Default | Env override |
|---|---|---|
| Zenoh locator | `tcp/127.0.0.1:7447` | `NROS_LOCATOR` (legacy alias: `ZENOH_LOCATOR`) |
| ROS domain ID | `0` | `ROS_DOMAIN_ID` |
| Zenoh mode | client | `NROS_SESSION_MODE` (legacy alias: `ZENOH_MODE`) |

No config file on native. Embedded targets bake these from
`[package.metadata.nros.deploy.<target>]` instead — see the
[Configuration Guide](../user-guide/configuration.md).

## Build

```bash probe=30
cd examples/native/rust/talker
nros sync             # once per checkout location: writes the generated/
                      # message bindings + the [patch.crates-io] table
cargo build           # or: cargo build --release
```

First build pulls dependencies (~3 minutes). Re-builds finish in
seconds.

## Run

Three terminals (each command below blocks; keep them open):

```bash
# Terminal 1 — zenoh router (ROS 2's own). Blocks the shell until Ctrl-C.
source /opt/ros/humble/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd    # or: just zenohd

# Terminal 2 — the talker. The talker logs via `log::info!`, so set
# RUST_LOG=info — without it `env_logger` only shows errors and the
# `Publishing:` lines stay hidden.
cd examples/native/rust/talker
RUST_LOG=info cargo run
# Expected output (on stderr):
#   [INFO  talker] Publishing: 'Hello World: 1'
#   [INFO  talker] Publishing: 'Hello World: 2'
#   …
```

That's the nano-ros side fully working. **Optional step:** verify
interop with stock ROS 2.

```bash
# Terminal 3 — stock ROS 2 with rmw_zenoh_cpp. It talks to the SAME
# router already running in terminal 1 (default tcp/127.0.0.1:7447).
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Talker publishes best-effort; stock `ros2 topic echo` defaults to
# RELIABLE, so the QoS-mismatched echo silently delivers nothing.
# Force best-effort to receive:
ros2 topic echo /chatter std_msgs/msg/String --qos-reliability best_effort
```

If `ros2 topic echo` shows no output despite the talker printing
`Publishing:`, the routers aren't peering — confirm both processes
point at the same port (default `tcp/127.0.0.1:7447`).

**Readiness signal.** Within ~6 seconds of `RUST_LOG=info cargo run`
(session open + the first 1 s timer tick), the talker should print
`Publishing: 'Hello World: 1'` — the count starts at 1, matching the
official ROS 2 `demo_nodes_cpp` talker. If no `Publishing:` line in 30
seconds:

1. Confirm `RUST_LOG` is set. Without `RUST_LOG=info` (or `debug`),
   `env_logger` filters out the `Publishing:` lines and the run looks
   silent even when it's working.
2. Confirm the router is running (terminal 1). Without it, the talker
   blocks on `Executor::open` indefinitely.
3. Re-run with `RUST_LOG=debug cargo run` and look for "RMW session
   open failed" — usually a wrong locator or wrong port.
4. See [Troubleshooting — First 10 Minutes](./troubleshooting-first-10-min.md).

## GitHub source

Canonical, copy-out:
[`examples/native/rust/talker/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/rust/talker)

Copy the directory, run `NROS_REPO_DIR=<nano-ros checkout> nros sync`
inside it, rename the package, and your starter is ready to modify.

## Next

- Add a subscription:
  [`examples/native/rust/listener/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/rust/listener)
- Generate bindings for custom `.msg` / `.srv` / `.action` files:
  [Message Generation](../user-guide/message-generation.md)
- Cross-compile for an RTOS: pick the right Embedded Starter from
  the next section (FreeRTOS / Zephyr / NuttX / ThreadX / ESP32 /
  Bare-metal Cortex-M3).
