# `nros` CLI reference

The `nros` binary is the user entry point for nano-ros: scaffolding,
message codegen, SDK provisioning, topology planning, static checks,
diagnostics, and board introspection.

The old `cargo nano-ros …` cargo subcommand has been removed. Use `nros`
directly.

## Install

Two front doors, and which one is right depends on who you are.

**Using nano-ros** — install a release, with no checkout and no cargo:

```sh
curl -fsSL https://raw.githubusercontent.com/NEWSLabNTU/nano-ros/main/scripts/install.sh | sh
```

It verifies the sha256, installs into `~/.nros/sdk/nros/<version>/` and points
`~/.nros/bin/nros` at the newest installed version — one command, however many
versions accumulate. (No release is cut yet; the installer says so and sends
you to the source build.)

**Working on nano-ros** — build the CLI from the checkout. That is not a
fallback: in a checkout the tree's own build is the only binary this tree
accepts, because a released `nros` emits its own generated code and this
runtime is what compiles it (RFC-0090, phase-431 W1). `just doctor` fails when
another `nros` shadows it. The CLI ships from the in-tree sub-workspace at
`packages/cli/`; bootstrap builds it (installing rustup if needed), then
activate the workspace to put it on PATH:

```sh
./scripts/bootstrap.sh
source ./activate.sh        # OR: direnv allow / source ./activate.fish
```

Equivalent, if you already have cargo (same build, same binary):
`git submodule update --init packages/cli/third-party/play_launch`
then `cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros`
(bootstrap runs both for you; `just setup-cli` is the internal alias).

The resulting binary lives at `packages/cli/target/release/nros`. One
checkout = one CLI version = one runtime ABI; contributors with
multiple nano-ros worktrees get per-tree CLIs with no global PATH
skew. Once on PATH, `nros --help` lists every verb. The transitional
`${NROS_HOME:-~/.nros}/bin/nros` install location remains supported
for users mid-migration from the pre-218 release-fetch shape.

## Verbs

### `nros setup [<board>] [--rmw <rmw>] [--tool <name>] [--source <name>] [--prefix <dir>] [--list] [--licenses] [--dry-run]`

Provision the toolchain/SDK for a board. `nros setup` is the single
canonical provisioning command. Most components are **prebuilt per platform
per RMW** (cross-compiler, emulator, RMW host daemon, SDK sources), fetched
from a pinned index into a shared store (`${NROS_HOME:-~/.nros}/sdk`). No
hand-installed cross-toolchains; no ROS distro required.

Packages are prebuilt where the index has a binary for your host and built
from source otherwise; the vendored submodules (`zenoh-pico`, `mbedtls`,
RTOS sources) are the source builds. `--dry-run` prints the plan without
fetching. Full explanation:
[Installation](../getting-started/installation.md#provision-your-toolchain-with-nros-setup).

Note `setup` does **not** provision a zenoh router: since RFC-0075 that is
ROS's `rmw_zenoh_cpp/rmw_zenohd`. See
[Do I need ROS 2 installed?](../getting-started/installation.md#do-i-need-ros-2-installed).

```sh
nros setup native --rmw zenoh            # host build (router comes from ROS)
nros setup qemu-arm-freertos --rmw xrce  # arm-none-eabi-gcc, qemu, FreeRTOS, XRCE agent
nros setup zephyr                        # Zephyr west workspace + SDK bits
```

| Flag | Effect |
|---|---|
| `<board>` | resolve + fetch this board's package set (see [Supported Boards](supported-boards.md)) |
| `--rmw <zenoh\|xrce\|cyclonedds>` | also provision the RMW's host daemon/tool (default `zenoh`); resolves `board ∪ rmw` packages |
| `--tool <name>` | install a single tool by name (e.g. `--tool qemu`) |
| `--source <name>` | provision a single source package by name (repeatable) |
| `--prefix <dir>` | install a `--tool` here instead of the shared store |
| `--list` | list every package in the index + its version |
| `--licenses` | show license-gated packages + how to install them |
| `--dry-run` | resolve + print the plan without fetching anything |

Board names: `native`, `posix`, `qemu-arm-baremetal`, `mps2-an385`,
`stm32f4`, `qemu-arm-freertos`, `qemu-arm-nuttx`, `qemu-riscv64-threadx`,
`threadx-linux`, `qemu-esp32-baremetal`, `zephyr`, and more —
run `nros setup --list` or `nros setup <board> --dry-run`.

### `nros init [<dir>]`

Generate a project `CMakePresets.json` (into `<dir>`, default the current
directory) that `include`s the per-board preset fragments `nros setup
<board>` wrote under `~/.nros/presets/` (RFC-0048 §6). After it,
`cmake --preset <board>` cross-configures a nano-ros ament package with no
hand-set `-DCMAKE_TOOLCHAIN_FILE` / `-Dnano_ros_ROOT`. Idempotent — re-run
after provisioning a new board to pick up its fragment.

```sh
nros setup qemu-arm-nuttx
nros init                 # writes ./CMakePresets.json
cmake --preset qemu-arm-nuttx
```

### `nros new <name> --platform <plat> [--rmw <rmw>] [--lang <lang>] [--use-case <case>] [--force]`

Scaffold a new nano-ros project. Emits a colcon-compatible
`package.xml` plus a hello-world Rust / C / C++ skeleton tuned for the
chosen platform.

| Flag | Values | Default |
|---|---|---|
| `--platform` | `native`, `freertos`, `nuttx`, `threadx`, `zephyr`, `esp32`, `posix`, `baremetal` | (required) |
| `--rmw` | `zenoh`, `xrce`, `cyclonedds` | `zenoh` |
| `--lang` | `rust`, `c`, `cpp` | `rust` |
| `--use-case` | `talker`, `listener`, `service`, `action` | `talker` |
| `--force` | overwrite an existing directory | off |

**Image / host mode:** `nros new --image <name> [--board <b>]` scaffolds an `[image.<name>]` — a buildable image (RFC-0065 D6) — into the bringup package's `system.toml`; `nros new --host <name>` scaffolds a `[host.<name>]`, a machine nodes run on. Both accept `[--bringup <pkg>] [--from-launch <path>] [--from-profile <name>]`. The bringup package is discovered automatically when the workspace exposes exactly one; pass `--bringup <pkg>` to pick one when there are several. `--from-launch` also seeds the bringup `[system].default_launch`; `--from-profile` forks an existing block of the same table.

An image names a `--board` and derives its rustc triple from that board's descriptor, so there is no `--target`. A host takes no board: a host says WHERE nodes run, and a board is what an image is BUILT for — passing one is refused rather than written.

`nros new --deploy <name>` is DEPRECATED (issue 0951): `[deploy.*]` split into those two tables. It still works, dispatched by the `--kind` it always took — `self` scaffolds a host, anything else an image — and says which flag to use instead.

### `nros generate <lang> [--manifest <path>] [--output <dir>] [--ros-edition <edition>] [--force] [--verbose] [--generate-config]`

Generate ROS 2 message bindings from a `package.xml`. Rust users should
prefer the direct `nros generate-rust` command; C and C++ users normally
use the CMake integration.

| Argument | Values | Default |
|---|---|---|
| `<lang>` | `rust`, `c`, `cpp`, `all` | (required) |
| `--manifest` | path to `package.xml` | `package.xml` |
| `--output` | output directory | `generated` |
| `--ros-edition` | `humble`, `iron` | `humble` |
| `--generate-config` | emit `.cargo/config.toml` patches (Rust only) | off |

### `nros generate-rust [--manifest <path>] [--output <dir>] [--generate-config] [--nano-ros-path <p> | --nano-ros-git] [--force] [--verbose]`

The Rust-only codegen **primitive**. Same binding generation as
`nros generate rust`, but with **no side-effects** unless asked:
`--generate-config` (alias `--config`) additionally writes the
`[patch.crates-io]` entries, pointing at a local checkout
(`--nano-ros-path`) or the GitHub repo (`--nano-ros-git`). Most
in-tree workflows use `nros sync` instead, which wraps this plus the
patch write per consumer.

### `nros generate-px4-msgs`

Generate CDR-serializable `px4_msgs::msg::*` bindings from a PX4
`.msg` tree — the XRCE/PX4 interop path (see the
[PX4 starter](../getting-started/integration-px4.md)).

### `nros codegen <resolve-deps|cyclonedds-descriptors|entry> …`

**Internal build-tool interface** — invoked by the CMake integration and
`build.rs` scripts, not meant for direct use. Subcommands resolve
message-package dependency closures, emit CycloneDDS type descriptors,
and generate C/C++ entry glue. (`nros codegen-system` below is the
user-facing system bake.)

### `nros metadata <system_pkg> [--workspace <path>] [--out-dir <dir>] [--metadata <existing.json>] [--build [--nano-ros-workspace <path>]]` — walk a colcon-style workspace under `<workspace>/src/`
collecting component source metadata into
`build/<system_pkg>/nros/source-metadata.json`. The result feeds
`nros plan`. With `--build`, any declared component (`component_nros.toml`)
missing its source-metadata is produced by the **metadata-mode build**:
nano-ros compiles + runs the component against an in-memory recorder to
emit the JSON. `--build` needs the nano-ros workspace
(`--nano-ros-workspace` or `NROS_WORKSPACE`).

### `nros plan <system_pkg> <launch_file> [LAUNCH_ARGS...] [options]` — resolve a ROS 2 launch file (or a precomputed
`play_launch_parser` `record.json`) plus the source metadata into a
typed `build/<system_pkg>/nros/nros-plan.json` IR. Picks per-component
SchedContext bindings, node graph wiring, parameter remaps, and
generated-package layout.

| Flag | Use |
|---|---|
| `--record <file>` | Skip launch-file parsing; consume an existing `record.json` |
| `--workspace <path>` | Override the workspace root (default `$PWD`) |
| `--out-dir <dir>` | Override `build/<system_pkg>/nros/` |
| `--metadata <file>` | Reuse a prior `source-metadata.json` |
| `--manifest <file>` | ROS launch manifest YAML overlay |
| `--nros-toml <file>` | nano-ros deployment overlay (`nros.toml`) |

### `nros check [plan]`

Validate an `nros-plan.json` (default `build/nros/nros-plan.json`):
static checker — catches unconnected required topics, conflicting QoS,
missing parameters, and SchedContext binding errors before the platform
build runs. A `.toml` argument is instead validated as a **root `nros.toml`**
(the workspace deployment SSOT) — `[system]`/`[image.<id>]`/`[host.<name>]` shape,
default-deploy + system references, bridge endpoints, etc.

### `nros explain [<plan>]`

Render a generated `nros-plan.json` in human-readable form.

```sh
nros explain                            # reads build/nros/nros-plan.json
nros explain path/to/nros-plan.json     # explicit path
```

Run after `nros plan` to inspect the resolved component graph, topic
wiring, parameter bindings, and SchedContext assignments before
committing to a platform build.

### `nros config show --system <pkg> [--workspace <dir>]`

Prints the **resolved effective config** for a bringup system (`rmw`,
`domain_id`, `locator`, and the `safety` / `param_services` / `lifecycle`
capability axes) with a **per-value provenance column** — `system.toml
[section]` vs the built-in `default`. If a legacy per-package `nros.toml`
overlay still declares any of those blocks, it is flagged DEPRECATED by
name (RFC-0004 §3.1). Omit the `<pkg>` value to default to the workspace's
`default_system` (or the sole bringup); `--workspace` defaults to the cwd.
The `nros check` command surfaces the same overlay warnings when validating
a `system.toml`.

> The legacy `nros config show/check --config <path>` reader for `config.toml`
> was removed: `config.toml` is retired (RFC-0004 §8) and no example
> ships one. Embedded runtime config lives in `[package.metadata.nros.deploy.<t>]`.

### `nros sync`

Codegen all `*.msg` packages + write the `[patch.crates-io]` config to match
the declared deps — for a **standalone package** or a **colcon-style workspace**
(picks single-pkg vs workspace mode by layout). The patch lands in each Rust
consumer's `.cargo/config.toml` (never edits `Cargo.toml`).
Pre-cargo step; run once after editing `*.msg` files, then `cargo build` works.

```sh
eval "$(nros ws env)"   # add src/ to NROS_INTERFACE_SEARCH_PATH
nros sync               # codegen msg pkgs + write [patch.crates-io] → .cargo/config.toml
```

`nros generate-rust` stays the low-level codegen-only primitive (no patch side
effects).

### `nros ws <subcommand>`

Workspace-level message-package utilities (the `env` / `status` / `list` /
`clean` / `doctor` helpers; `sync` was promoted to top-level `nros sync`).

```sh
eval "$(nros ws env)"   # add src/ to NROS_INTERFACE_SEARCH_PATH
nros ws status          # freshness check (non-fatal): n up-to-date / n stale / n missing
nros ws list            # list discovered msg + Rust-consumer pkgs
nros ws clean           # remove generated/ + auto-managed patch entries
nros ws doctor          # lint workspace pkgs (package.xml markers, stale patches, …)
```

| Subcommand | Description |
|---|---|
| `env` | Print shell export adding `<dir>` (default `./src`) to `NROS_INTERFACE_SEARCH_PATH` |
| `status` | Non-fatal freshness check — sibling of `nros sync --check` |
| `list` | List discovered msg + Rust-consumer pkgs (kind, name, dir per row) |
| `clean` | Remove `generated/` + auto-managed `[patch.crates-io]` entries from each `.cargo/config.toml`; leaves user-written keys + sections alone |
| `doctor` | Lint: warn on missing `<member_of_group> rosidl_interface_packages</member_of_group>`, malformed `package.xml`, missing nros-managed `[patch.crates-io]` entries in the authority `.cargo/config.toml` |

### `nros codegen-system [--bringup <pkg>] [--target <triple>] [--out <dir>] [--launch <file>] [--ahead-of-vendor <pio|px4>]`

Host-time system bake: reads `<bringup>/system.toml` +
`<bringup>/launch/system.launch.xml` and emits the baked compile-time C
config + component registration glue consumed by every embedded RTOS
adapter.

```sh
nros codegen-system --bringup demo_bringup
nros codegen-system --bringup demo_bringup --target thumbv7em-none-eabihf
nros codegen-system --bringup demo_bringup --ahead-of-vendor pio   # + PlatformIO library.json
nros codegen-system --bringup demo_bringup --ahead-of-vendor px4   # + PX4 module dirs
```

| Flag | Description |
|---|---|
| `--workspace <path>` | Workspace root (default: cwd) |
| `--bringup <pkg>` | Bringup pkg name or path. Defaults to `[workspace.metadata.nros].default_system` |
| `--target <triple>` | Target triple for cross-compile bake context |
| `--out <dir>` | Output directory; `nros-system/` subdir created inside. Default: `<workspace>/build/<bringup>/` |
| `--launch <file>` | Multi-launch disambiguation: pick `<bringup>/launch/<file>` (`--file` is an alias) |
| `--exec <exec>` | `<node exec="…">` override for synthesised launches |
| `--ahead-of-vendor` | `pio`: emit PlatformIO `library.json`; `px4`: emit one PX4-native `nros_<component>/` module dir per component |

### `nros doctor [--platform <name>] [--workspace <path>]`

Shells out to `just doctor` from the auto-detected workspace root.
The justfile orchestrates every per-module doctor recipe (`just nuttx
doctor`, `just zephyr doctor`, …) and is the source of truth for
"healthy". `--platform <name>` scopes the check to a single module.

### `nros board list [--workspace <path>]`

Enumerate every `nros-board-*` crate under `packages/boards/`. Output
columns are `name | description`; structured `chip | flash | ram |
supported_rmw` fields are deferred to a future board-descriptor TOML.

### `nros version`

Print toolchain + library versions.

### `nros completions <shell>`

Emit shell completion scripts to stdout.

| Shell | Install snippet |
|---|---|
| `bash` | `nros completions bash > ~/.local/share/bash-completion/completions/nros` |
| `zsh` | `nros completions zsh > "${fpath[1]}/_nros"` |
| `fish` | `nros completions fish > ~/.config/fish/completions/nros.fish` |
| `powershell` | `nros completions powershell > $PROFILE.parent\nros.ps1` |

## Comparison: `nros` vs. `just`

| Want to … | Use |
|---|---|
| Scaffold a project | `nros new …` |
| Generate Rust bindings (+ `.cargo` patches) | `nros sync` (primitive: `nros generate-rust`) |
| Plan + check a multi-component system from a ROS 2 launch file | `nros metadata` → `nros plan` → `nros check` |
| Build / flash / run | Platform tools: `cargo`, `cmake --build`, `west`, `idf.py`, `probe-rs`, or focused `just <platform> …` recipes |
| Orchestrate the workspace (setup, doctor, CI, multi-platform sweeps) | `just …` |

`nros` routes through the `nros-cli-core` library. `just` recipes that
wrap user-flow operations call into `nros` for consistency; internal
recipes (build matrices, CI orchestration) keep their current shape.

## Release pipeline status

There is no crates.io / Arduino zip / ESP-IDF binary / GitHub Releases
tarball channel. Per the archive decision (2026-05-19), nano-ros is
consumed by `git clone --branch=v<X.Y.Z>` plus the in-tree build
recipes documented at
[Installation](../getting-started/installation.md). (A hidden,
maintainer-only `nros release` verb exists behind the `release` cargo
feature; the default build does not carry it.)
Downstream RTOS package managers (Zephyr `west`,
ESP-IDF `idf_component_yml`, NuttX `Kconfig`) consume the same source
tree via the integration shells under `integrations/<rtos>/`.
