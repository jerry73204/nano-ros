# nros-cli

**`nros` — the command-line tool for [nano-ros](https://github.com/NEWSLabNTU/nano-ros), a lightweight ROS 2 client for embedded RTOS.**

`nros` scaffolds projects, generates message bindings, provisions SDK toolchains,
resolves launch topologies, checks workspace metadata, and inspects supported
boards. Platform-native tools build, flash, run, and monitor applications
(`cargo`, `cmake --build`, `west`, `idf.py`, `probe-rs`, and focused `just`
recipes). This repo builds the `nros` binary;
nano-ros itself lives at [NEWSLabNTU/nano-ros](https://github.com/NEWSLabNTU/nano-ros).

> `nros` is a **generic tool**: it knows no nano-ros directory layout. All
> workspace/toolchain knowledge lives in nano-ros's committed
> `nros-sdk-index.toml` (board → package sets, toolchain URLs, source
> git/ref/dest). `nros` just executes that data — fixes are index edits, not new
> binaries.

## Install

Users install a release and need no checkout:

```bash
curl -fsSL https://raw.githubusercontent.com/NEWSLabNTU/nano-ros/main/scripts/install.sh | sh
```

From a nano-ros checkout, the front door builds it instead — installing rustup
if needed. That is the correct binary here and a released one is not: it emits
its own generated code, and this tree's runtime is what has to compile it
(RFC-0090, phase-431 W1/W2).

```bash
./scripts/bootstrap.sh
# binary lands at packages/cli/target/release/nros; `source ./activate.sh`
# puts it on PATH in every subsequent shell
```

Equivalent by hand (same build, same binary):

```bash
git submodule update --init packages/cli/third-party/play_launch
cargo build --release --manifest-path packages/cli/Cargo.toml --bin nros
```

## Setup — one command for toolchains + sources

`nros setup <board>` provisions everything that board needs, **board-scoped**,
from nano-ros's SDK index: prebuilt host toolchains (cross-GCC, QEMU, the RMW
daemon) **and** the target source it builds against (RTOS kernels, lwip, …),
checked out from the index's pinned git/ref into its declared destination. Run it
from a nano-ros checkout (it reads `./nros-sdk-index.toml`):

```bash
nros setup qemu-arm-freertos      # arm-none-eabi-gcc + qemu + FreeRTOS-Kernel + lwip
nros setup native --rmw cyclonedds
nros setup --list                 # every package + version
nros setup --licenses             # license-gated SDKs (NVIDIA SPE, ARM FVP)
```

Prebuilt where available, source-built fallback otherwise — same install layout
either way. This unifies what used to be scattered across `just <module> setup`
recipes.

## Build / run

**As a user** — import nano-ros into your project and use the platform tool:

```bash
nros new talker --platform zephyr   # scaffold from a template
west build                          # or: cargo build / cmake --build / idf.py build
nros doctor                         # check SDK paths / toolchains / env
```

**As a contributor** — build and test inside the nano-ros checkout (`just ci`,
`just <plat> test`); `nros` drives codegen + orchestration there.

## Commands

| | |
|---|---|
| `nros new` | scaffold a project (talker / listener / service / action) |
| `nros generate` / `generate-rust` | message bindings from `package.xml` |
| `nros codegen` | build-tool C/C++ binding generation (cmake/build.rs interface) |
| `nros setup` | provision a board's toolchains + sources (above) |
| `nros doctor` / `board` | health-check; inspect supported boards |
| `nros plan` / `check` / `explain` | launch-file → plan resolution¹ |

¹ Launch parsing shells out to the separate
[`play_launch_parser`](https://github.com/jerry73204/play_launch_parser)
(`pip install play-launch-parser`) so `nros` itself stays python-free; the build
system runs it internally to produce the plan record.

## License

See nano-ros. Built from this repo's Rust workspace (`packages/`).
