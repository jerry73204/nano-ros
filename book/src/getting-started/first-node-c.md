# First Node — C (Linux)

Build, run, and verify a single nano-ros publisher node on Linux from
C. Uses CMake, the Zenoh backend, and `add_subdirectory` consumption.

> **Stuck?** See [Troubleshooting — First 10 Minutes](./troubleshooting-first-10-min.md) for the common first-build errors.

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

Then provision the native host (installs the zenoh client stack into a
shared store; the router itself comes from your ROS 2 install —
`ros2 run rmw_zenoh_cpp rmw_zenohd`):
```sh
nros setup native --rmw zenoh
```

See [Install + first build (Linux)](./installation.md) for more.

## Project layout

The talker is a **standalone CMake project** that pulls nano-ros via
`add_subdirectory(<repo-root>)`. Four files matter:

```text
examples/native/c/talker/
├── CMakeLists.txt      # add_subdirectory + targets
├── package.xml         # ROS-style manifest (drives codegen tooling)
└── src/
    └── main.c          # ~100-line talker
```

On native, locator + domain come from env vars (`NROS_LOCATOR`,
`ROS_DOMAIN_ID`) with built-in defaults; embedded targets bake them at
build time via the package.xml `<nano_ros deploy=… rmw=…/>` tuple (see
[Configuration](../user-guide/configuration.md)).

The CMake preamble matches the canonical example shape in
[`examples/native/c/talker/CMakeLists.txt`](https://github.com/NEWSLabNTU/nano-ros/blob/main/examples/native/c/talker/CMakeLists.txt) —
five `set(...)` lines plus the per-target link:

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_talker LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Pull nano-ros in. Adjust the relative path to your repo root.
# `NROS_RMW` is the user-facing cache var (overridable via
# `-DNROS_RMW=<rmw>`); the example forwards it to `NANO_ROS_RMW`,
# the var the nano-ros add_subdirectory reads.
set(NANO_ROS_PLATFORM posix)
set(NROS_RMW "zenoh" CACHE STRING
    "Active RMW (zenoh|xrce|cyclonedds) — selects the backend linked into my_talker.")
set(NANO_ROS_RMW "${NROS_RMW}")
add_subdirectory(<rel-path-to-nano-ros> nano_ros)

# Generate C bindings for std_msgs (transitively depends on
# builtin_interfaces; the dependency is declared explicitly).
nros_generate_interfaces(builtin_interfaces SKIP_INSTALL)
nros_generate_interfaces(std_msgs DEPENDENCIES builtin_interfaces
                                  SKIP_INSTALL)

add_executable(my_talker src/main.c)
target_link_libraries(my_talker PRIVATE
    std_msgs__nano_ros_c
    NanoRos::NanoRos)
nros_platform_link_app(my_talker)
```

`nros_platform_link_app(my_talker)` transitively wires the active
RMW's strong `nros_app_register_backends()` stub (calling
`nros_rmw_zenoh_register()` for the zenoh build above). On POSIX you
do **not** call `nano_ros_link_rmw()` explicitly — the platform
module handles it.

The C entry point is **`int nros_app_main(int argc, char **argv)`**
(not `main`); `<nros/app_main.h>` provides the OS-side `main`
stub that wires signal handling and forwards to your function.

```c
#include <nros/app_main.h>
#include <nros/init.h>
#include <nros/executor.h>
#include <nros/node.h>
#include <nros/publisher.h>
#include <nros/timer.h>
#include "std_msgs.h"

int nros_app_main(int argc, char** argv) {
    // 1. nros_support_init() — opens the zenoh session
    // 2. nros_executor_init() / rclc_node_init_default()
    // 3. std_msgs_msg_int32_publisher_init() — typed publisher
    // 4. nros_timer_init() with a 1 Hz period + publish callback
    // 5. rclc_executor_spin() until SIGINT
}
```

## Configure

Three runtime knobs:

| Knob | Default | Env override |
|---|---|---|
| Zenoh locator | `tcp/127.0.0.1:7447` | `NROS_LOCATOR` |
| ROS domain ID | `0` | `ROS_DOMAIN_ID` |
| Node name | `talker` | hard-coded in source |

On an embedded target the same knobs are compile-baked from the
the package.xml `<export><nano_ros deploy="<t>" rmw="<rmw>"/>` tuple
call in `CMakeLists.txt` (see
[Configuration](../user-guide/configuration.md)).

## Build

```bash probe=40
cd examples/native/c/talker
cmake -B build
cmake --build build
```

No `nros sync` here — a C leaf's message bindings are generated inside
CMake, and unlike the Rust leaves it carries no `.cargo/config.toml` to
resolve. See
[Workflow by Platform and Language](../user-guide/workflow-by-platform.md).

The first configure pulls and builds nano-ros's Rust staticlibs
(~3 minutes). Re-builds finish in seconds.

## Run

Three terminals.

```bash
# 1. Start the zenoh router (ROS 2's own):
source /opt/ros/humble/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd    # or: just zenohd

# 2. Run the talker:
cd examples/native/c/talker
./build/c_talker
# Expected output:
#   Publishing: 'Hello World: 1'
#   Publishing: 'Hello World: 2'
#   Publishing: 'Hello World: 3'
#   …

# 3. Verify from stock ROS 2:
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Talker publishes best-effort; stock `ros2 topic echo` defaults to
# RELIABLE, so the QoS-mismatched echo silently delivers nothing.
# Force best-effort to receive:
ros2 topic echo /chatter std_msgs/msg/String --qos-reliability best_effort
```

**Readiness signal.** Within ~6 seconds of `./build/c_talker` (session
open + the first 1 s timer tick), the binary should print
`Publishing: 'Hello World: 1'` on stdout — Rust + C + C++ all start
the count at 1, matching the official ROS 2 demo talker. If no
`Publishing:` line in 30 seconds:

1. Confirm the router is running (terminal 1). Without it,
   `nros_support_init` returns immediately with `-4`
   (`NROS_RET_NOT_FOUND` — connection refused).
2. Wrong locator / unreachable host → same `-4` signature in stderr.
   Reachable host but mismatched port → talker hangs on session-open
   handshake rather than returning a code.
3. See [Troubleshooting — First 10 Minutes](./troubleshooting-first-10-min.md).

## GitHub source

Canonical, copy-out:
[`examples/native/c/talker/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/c/talker)

## Next

- Add a subscription:
  [`examples/native/c/listener/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/c/listener)
- Service / action shapes:
  [`service-client/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/c/service-client),
  [`action-client/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/c/action-client)
- Custom `.msg` / `.srv` / `.action`:
  [Message Generation](../user-guide/message-generation.md)
- Cross-compile for an RTOS: pick the right Embedded Starter from
  the next section.
