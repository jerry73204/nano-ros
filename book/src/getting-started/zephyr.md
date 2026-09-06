# Zephyr (contributor / in-tree workflow)

> **Looking for the user-facing path?** This page covers building
> nano-ros's own Zephyr examples from this repository.
> If you're consuming nano-ros as a Zephyr module in YOUR Zephyr
> workspace, see [Integration: Zephyr (`west` module)](./integration-zephyr.md)
> instead.

Complete setup procedure for Zephyr `native_sim` testing. Networking uses
**NSOS** (Native Sim Offloaded Sockets) — each socket call is forwarded to
the host kernel, so tests run on `127.0.0.1` without TAP devices, bridges,
or `sudo`.

## Overview

nros uses an **in-tree Zephyr workspace** at `zephyr-workspace/` (gitignored).
Set `$NROS_ZEPHYR_WORKSPACE` to install elsewhere.

```
nros/
├── scripts/zephyr/
│   ├── setup.sh                      # Initialize workspace
│   ├── migrate-workspace.sh          # Move legacy sibling install in-tree
│   ├── downloads/                    # SDK tarball cache (gitignored)
│   └── sdk/                          # Installed Zephyr SDK (gitignored)
├── zephyr/                           # Zephyr module definition
│   ├── Kconfig                       # RMW backend, API selection, tuning
│   ├── CMakeLists.txt                # Transport C sources + nros-c build
│   └── cmake/                        # nros_cargo_build(), nros_generate_interfaces()
├── examples/zephyr/
│   ├── rust/                   # Rust + zenoh (talker, listener, ...)
│   ├── rust/xrce/                    # Rust + XRCE-DDS (talker, listener)
│   ├── c/                      # C + zenoh (talker, listener)
│   └── c/xrce/                       # C + XRCE-DDS (talker, listener)
├── west.yml                          # West manifest
└── zephyr-workspace/                 # Created by setup.sh (gitignored)
    ├── nros -> ../                   # Symlink back to repo root
    ├── zephyr/                       # Zephyr RTOS v3.7.0
    └── modules/                      # HALs, zephyr-lang-rust
```

### Migrating from the legacy sibling layout

Legacy setups put the workspace at `../nano-ros-workspace/` with an
in-tree symlink. Both layouts work (**contributors:** the in-tree
`just zephyr` recipes auto-detect either), but to consolidate run:

```bash
./scripts/zephyr/migrate-workspace.sh --dry-run     # preview
./scripts/zephyr/migrate-workspace.sh               # execute
```

## Prerequisites

Build the in-tree `nros` CLI (Phase 218), then let it provision the toolchain:

```bash
./scripts/bootstrap.sh      # builds packages/cli/target/release/nros
source ./activate.sh        # OR: direnv allow / source ./activate.fish
```

`nros setup` provides most components prebuilt per platform per RMW — the
cross-compiler, emulator, RMW host daemon, and SDK sources (including the Zephyr
west workspace + Zephyr SDK bits) are fetched from a pinned index into a shared
store at `${NROS_HOME:-~/.nros}/sdk`. You do not hand-install a cross-toolchain, and you do not
need ROS 2 installed.

Packages are prebuilt where the index has a binary for your host and built from
source otherwise — `zenohd` is the notable source build, and zenoh is the default
`--rmw`. `--dry-run` prints the plan for your host. See
[Installation](installation.md#provision-your-toolchain-with-nros-setup) for the full explanation.

## Step 1: Initialize Workspace (One-Time)

```bash
nros setup zephyr --rmw zenoh      # --rmw defaults to zenoh; xrce | cyclonedds also valid
source ./activate.sh
```

This provisions:
- The Zephyr west workspace + Zephyr SDK bits
- The emulator
- The RMW host daemon (`zenohd` for zenoh, the Micro-XRCE-DDS agent for xrce)
- The in-tree workspace at `zephyr-workspace/` (gitignored; auto-detects legacy `../nano-ros-workspace/`), with nros symlinked in
- Rust embedded targets

> **Contributors:** the in-tree `just setup zephyr` recipe still works and now
> delegates to `nros setup zephyr` under the hood.

The RMW host daemon must be **running** before any example: `zenohd` for zenoh,
the Micro-XRCE-DDS agent for xrce. `nros setup zephyr --rmw <rmw>` installs it.

## Step 2: Networking

No network setup is required. `native_sim` uses the NSOS offloaded-sockets
driver, enabled by `boards/native_sim_native_64.conf` in each example:

```
CONFIG_ETH_NATIVE_POSIX=n
CONFIG_NET_SOCKETS_OFFLOAD=y
CONFIG_NET_NATIVE_OFFLOADED_SOCKETS=y
```

With NSOS, Zephyr's socket API goes straight to host syscalls. Bind to
`127.0.0.1` and reach `zenohd` / the XRCE Agent on the host loopback just
like any other native test. Multiple `native_sim` processes can coexist
without bridge configuration.

## Step 3: Build and Run Zephyr Examples

```bash
# Source environment
source zephyr-workspace/env.sh

cd zephyr-workspace

# Rust — `nros sync` first, once per checkout location. It writes the
# generated message bindings and the [patch.crates-io] table the leaf's
# .cargo/config.toml includes. This applies to cargo under WEST too: west
# drives the same leaf, so without sync the build fails while PARSING the
# manifest, with an error that never names sync (issue 0694).
#
# `--no-metadata` because the source-metadata probe builds a host binary that
# path-deps this leaf, dragging in the `zephyr` crate — whose build.rs needs a
# DOTCONFIG from a Zephyr cmake configure that has not run yet, and cannot,
# since this sync is its prerequisite (issue 0318). Without the flag the sync
# still succeeds but prints a "no producer" error the reader cannot act on.
# Codegen and the patch table — the reason to run it — are unaffected.
nros sync --no-metadata ../examples/zephyr/rust/talker

# Build Zephyr talker (Rust + zenoh, default backend).
#
# `../examples/...` rather than `<name>/examples/...`: the workspace's link
# back to the checkout is named after YOUR checkout directory
# (`scripts/zephyr/setup.sh` uses `basename "$NANO_ROS_ROOT"`), so it reads
# `nano-ros/` for a plain `git clone` and `nros/` only if you renamed the
# directory. For the in-tree workspace this page sets up, `..` IS the
# checkout and needs no name at all.
west build -b native_sim/native/64 ../examples/zephyr/rust/talker

# Run (no sudo needed)
./build/zephyr/zephyr.exe
```

## RMW Backend Selection

nros supports three RMW backends on Zephyr, selected via `prj.conf`:

### Zenoh (default)

Connects to a zenoh router. Requires POSIX API for zenoh-pico threads.

```ini
CONFIG_NROS=y
# CONFIG_NROS_RMW_ZENOH=y  # default, can be omitted
CONFIG_NROS_ZENOH_LOCATOR="tcp/127.0.0.1:7447"
CONFIG_POSIX_API=y
CONFIG_MAX_PTHREAD_MUTEX_COUNT=32
CONFIG_MAX_PTHREAD_COND_COUNT=16
```

### XRCE-DDS

Connects to a Micro-XRCE-DDS Agent over UDP. Requires BSD sockets.

```ini
CONFIG_NROS=y
CONFIG_NROS_RMW_XRCE=y
CONFIG_NROS_XRCE_AGENT_ADDR="127.0.0.1"
CONFIG_NROS_XRCE_AGENT_PORT=2018
CONFIG_NET_SOCKETS=y
```

### Cyclone DDS

Brokerless RTPS, wire-compatible with stock ROS 2 (`rmw_cyclonedds_cpp`).
Cyclone's source is C++, so `CONFIG_CPP=y` is required even for Rust
callers. Cyclone is resource-heavy — it needs a large heap, libc malloc
arena, and pthread pools. The bool prerequisites (thread-local storage,
dynamic threads, `NET_TCP`, …) are `select`ed automatically by
`CONFIG_NROS_RMW_CYCLONEDDS` in `zephyr/Kconfig`; the size knobs stay in
`prj.conf`:

```ini
CONFIG_NROS=y
CONFIG_NROS_RMW_CYCLONEDDS=y
CONFIG_CPP=y
CONFIG_POSIX_API=y
CONFIG_NET_IPV4_IGMP=y                  # RTPS SPDP uses UDP multicast
CONFIG_MAIN_STACK_SIZE=524288
CONFIG_HEAP_MEM_POOL_SIZE=4194304
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=16777216
CONFIG_DYNAMIC_THREAD_STACK_SIZE=32768
```

On `native_sim`, add the NSOS host-socket offload
(`CONFIG_NET_SOCKETS_OFFLOAD=y` + `CONFIG_NET_NATIVE_OFFLOADED_SOCKETS=y`)
so discovery uses host BSD sockets instead of zeth/TAP. See
`examples/zephyr/rust/talker/prj-cyclonedds.conf` for the full overlay.

## API Selection

Choose between Rust and C APIs via `prj.conf`:

### Rust API (default)

```ini
CONFIG_NROS_RUST_API=y
CONFIG_RUST=y
CONFIG_RUST_ALLOC=y
```

CMakeLists.txt uses `rust_cargo_application()`:
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_example)
rust_cargo_application()
```

### C API

```ini
CONFIG_NROS_C_API=y
```

CMakeLists.txt uses `nros_generate_interfaces()`:
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_example)
nros_generate_interfaces(std_msgs "msg/Int32.msg")
target_sources(app PRIVATE src/main.c)
```

## Kconfig Reference

All options are under `menuconfig NROS` in `zephyr/Kconfig`.

### Common Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS` | bool | n | Enable nros module |
| `CONFIG_NROS_RUST_API` | bool | y | Use Rust API |
| `CONFIG_NROS_C_API` | bool | n | Use C API |
| `CONFIG_NROS_DOMAIN_ID` | int | 0 | ROS 2 domain ID |
| `CONFIG_NROS_INIT_DELAY_MS` | int | 2000 | Network init wait (ms) |

### Zenoh Options (visible when `CONFIG_NROS_RMW_ZENOH=y`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_ZENOH_LOCATOR` | string | `"tcp/127.0.0.1:7447"` | Router address |
| `CONFIG_NROS_ZENOH_MULTI_THREAD` | bool | y | Zenoh-pico multithreading |
| `CONFIG_NROS_ZENOH_PUBLICATION` | bool | y | Publication support |
| `CONFIG_NROS_ZENOH_SUBSCRIPTION` | bool | y | Subscription support |
| `CONFIG_NROS_ZENOH_QUERY` | bool | y | Service client support |
| `CONFIG_NROS_ZENOH_QUERYABLE` | bool | y | Service server support |
| `CONFIG_NROS_ZENOH_LINK_TCP` | bool | y | TCP transport link |
| `CONFIG_NROS_MAX_PUBLISHERS` | int | 8 | Max concurrent publishers |
| `CONFIG_NROS_MAX_SUBSCRIBERS` | int | 8 | Max concurrent subscribers |
| `CONFIG_NROS_MAX_QUERYABLES` | int | 8 | Max concurrent queryables |
| `CONFIG_NROS_FRAG_MAX_SIZE` | int | 2048 | Max reassembled message size |
| `CONFIG_NROS_BATCH_UNICAST_SIZE` | int | 1024 | Max unicast batch size |
| `CONFIG_NROS_SUBSCRIBER_BUFFER_SIZE` | int | -1 (derive) | Per-subscriber buffer, small payload class |
| `CONFIG_NROS_MAX_LARGE_SUBSCRIBERS` | int | -1 (derive) | Blocks in the large payload class |
| `CONFIG_NROS_SUBSCRIBER_LARGE_SIZE` | int | -1 (derive) | Per-sample capacity of the large class |
| `CONFIG_NROS_SERVICE_BUFFER_SIZE` | int | 1024 | Per-service buffer |

#### `-1` means "derive it from the message bounds"

Three of the rows above, plus `CONFIG_NROS_SUBSCRIPTION_BUFFER_SIZE`, default to
`-1`. That is not a size: it is how an image says *nothing here chose a number,
work it out* (phase-403 W8, issue 0940).

Codegen derives an exact serialized-size bound for every message type it
generates and writes it beside the code as `nros_message_bounds.cmake`. The
build composes those, and the configure prints what it concluded:

```
-- nros: message-bound sizing DERIVED from 84 types in 11 interface packages (all bounded)
-- nros:   largest type std_msgs/msg/Float64MultiArray at 1496 B -> NROS_SUBSCRIPTION_BUFFER_SIZE
-- nros:   0 types over the 2048 B ceiling -> NROS_MAX_LARGE_SUBSCRIBERS
```

Three things to know before you rely on it:

* **Anything you state wins.** A value in your `prj.conf`, a board `.conf`, or
  an environment override outranks the derivation, silently and in both
  directions. The derived number is a default, never an override.
* **It is an UPPER BOUND.** The inventory covers every type in the interface
  closure you LINK, not the ones you subscribe to, so the number is the largest
  message the image *could* receive. That is the safe direction, and it can
  still be far above what you need: the line above names
  `std_msgs/Float64MultiArray`, which that image never receives. When the type
  it names is not one of yours, either state your own value or stop linking the
  package.
* **One unbounded type refuses the lot.** If any type in the closure has no
  derived bound, nothing is derived, every knob keeps its configured value, and
  the configure names the types and the members that cost them their bound.
  Deriving over the rest would publish a maximum a real sample can exceed, and
  that drop is silent on the C++ arena path. The remedy is a bound in the
  `.msg` or an `inline` cap in `nros-codegen.toml`; one cap on a declaring type
  (`"std_msgs/Header.frame_id"`) reaches every message that nests it.

The answer, and why it came out that way, is written to
`<build>/nros/message_bound_knobs.cmake`.

### XRCE Options (visible when `CONFIG_NROS_RMW_XRCE=y`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_XRCE_AGENT_ADDR` | string | `"127.0.0.1"` | Agent IP address |
| `CONFIG_NROS_XRCE_AGENT_PORT` | int | 2018 | Agent UDP port |
| `CONFIG_NROS_XRCE_TRANSPORT_MTU` | int | 512 | Transport MTU |
| `CONFIG_NROS_XRCE_MAX_SUBSCRIBERS` | int | -1 (derive) | Max concurrent subscribers |
| `CONFIG_NROS_XRCE_MAX_SERVICE_SERVERS` | int | -1 (derive) | Max service servers |
| `CONFIG_NROS_XRCE_MAX_SERVICE_CLIENTS` | int | 4 | Max service clients |
| `CONFIG_NROS_XRCE_SUBSCRIBER_RING_DEPTH` | int | 32 | Queued samples per subscriber |
| `CONFIG_NROS_XRCE_BUFFER_SIZE` | int | 1024 | Per-slot buffer size |
| `CONFIG_NROS_XRCE_STREAM_HISTORY` | int | 16 | Reliable stream depth (2-16) |

### C API Options (visible when `CONFIG_NROS_C_API=y`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `CONFIG_NROS_C_MAX_HANDLES` | int | 16 | Max executor handles |
| `CONFIG_NROS_C_MAX_SUBSCRIPTIONS` | int | 8 | Max subscriptions |
| `CONFIG_NROS_C_MAX_TIMERS` | int | 8 | Max timers |
| `CONFIG_NROS_C_MAX_SERVICES` | int | 4 | Max services |

## E2E Testing

> **Contributors:** the in-tree fixture/test lanes for this platform are in
> [Per-Platform Contributor Lanes](../internals/platform-lanes.md#zephyr).

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `west: command not found` | Run `pip3 install --user west` and add `~/.local/bin` to PATH |
| `Connection refused` | Start `zenohd` / `MicroXRCEAgent` on the host loopback (e.g. `tcp/127.0.0.1:7447`) |
| `Build fails` | Source environment: `source zephyr-workspace/env.sh` |
| `XRCE Agent not found` | Provision the xrce daemon: `nros setup zephyr --rmw xrce` |
| Zenoh mutex exhaustion | Increase `CONFIG_MAX_PTHREAD_MUTEX_COUNT` (default 5 is too low) |

## Network Architecture

With NSOS, Zephyr sockets are forwarded to host syscalls — there is no
emulated L2/L3 stack to configure, no static IP, and no bridge.

```
┌─────────────────────────────────────────────────────────────┐
│                      Host (Linux)                            │
│                                                              │
│   ┌────────────────────┐       ┌────────────────────────┐   │
│   │ zephyr.exe talker  │       │ zephyr.exe listener    │   │
│   │ (native_sim+NSOS)  │       │ (native_sim+NSOS)      │   │
│   └─────────┬──────────┘       └──────────┬─────────────┘   │
│             │ host socket() via NSOS       │                │
│             ▼                              ▼                │
│                 127.0.0.1 (loopback)                        │
│             │                              │                │
│             ▼                              ▼                │
│   ┌────────────────────┐       ┌────────────────────────┐   │
│   │ zenohd             │       │ MicroXRCEAgent         │   │
│   │ tcp/127.0.0.1:7447 │       │ udp/127.0.0.1:2018     │   │
│   └────────────────────┘       └────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Updating the Workspace

To update Zephyr and modules to latest versions specified in `west.yml`:

```bash
cd zephyr-workspace
west update
```

To completely recreate the workspace:

> **Contributors:** the in-tree workspace-recreate lane is in
> [Per-Platform Contributor Lanes](../internals/platform-lanes.md#zephyr).
