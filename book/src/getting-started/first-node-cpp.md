# First Node — C++ (Linux)

Build, run, and verify a single nano-ros publisher node on Linux from
C++14. Uses CMake, the Zenoh backend, and `add_subdirectory`
consumption.

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
shared store; the router itself comes from your ROS 2 install —
`ros2 run rmw_zenoh_cpp rmw_zenohd`):
```sh
nros setup native --rmw zenoh
```

See [Install + first build (Linux)](./installation.md) for more.

## Project layout

The talker is a **standalone CMake project** that pulls nano-ros via
`add_subdirectory(<repo-root>)`. Two files matter:

```text
examples/native/cpp/talker/
├── CMakeLists.txt      # add_subdirectory + targets
└── src/
    └── main.cpp        # ~70-line talker
```

Native (host) talkers read the locator + domain from arguments passed to
`nros::init(...)`; no config file is needed. Embedded variants under
`examples/<plat>/cpp/talker/` bake theirs from the
package.xml `<nano_ros deploy=…/>` tuple.

The CMakeLists is the **ament shape you already know** — this is the
complete canonical file at
[`examples/native/cpp/talker/CMakeLists.txt`](https://github.com/NEWSLabNTU/nano-ros/blob/main/examples/native/cpp/talker/CMakeLists.txt),
verbatim (RFC-0048: `nano_rosConfig.cmake` at the checkout root makes
`find_package(nano_ros)` resolve source-side, and the message-package
Find-stubs make `find_package(std_msgs)` generate bindings on the fly):

```cmake
cmake_minimum_required(VERSION 3.22)
project(cpp_talker LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable(cpp_talker src/main.cpp)
ament_target_dependencies(cpp_talker std_msgs)

install(TARGETS cpp_talker DESTINATION lib/${PROJECT_NAME})
ament_package()
```

**`LANGUAGES C CXX`** (not `CXX` alone): the per-target register stub
is a C translation unit, so C must be enabled in this directory scope
or the link fails. Backend selection is a configure-line cache var
(`-DNROS_RMW=zenoh|xrce|cyclonedds`), not CMake code.

The C++ entry point is **`int nros_app_main(int argc, char** argv)`**
(same as C); `<nros/app_main.h>` provides the OS-side `main` stub.

The body uses typed `nros::Publisher<M>` / `nros::Subscription<M>`
wrappers over the C ABI:

```cpp
#include <nros/app_main.h>
#include <nros/nros.hpp>
#include "std_msgs.hpp"

#define NROS_TRY_LOG(file, line, expr, ret) \
    std::fprintf(stderr, "[nros] %s:%d %s -> %d\n", file, line, expr, (int)ret)

int nros_app_main(int argc, char** argv) {
    NROS_TRY_RET(nros::init("tcp/127.0.0.1:7447", 0), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "talker"), 1);

    nros::Publisher<std_msgs::msg::String> pub;
    NROS_TRY_RET(node.create_publisher(pub, "/chatter"), 1);

    // ... register a timer + spin
}
```

`NROS_TRY_RET` short-circuits on any non-OK return code and logs the
expression that failed. Define `NROS_TRY_LOG` once (any sink — here
`std::fprintf`) and reuse it across every call site.

## Configure

Three runtime knobs:

| Knob | Default | Override |
|---|---|---|
| Zenoh locator | `tcp/127.0.0.1:7447` | First arg to `nros::init` |
| ROS domain ID | `0` | Second arg to `nros::init` |
| Node name | `talker` | First arg to `nros::create_node` |

Reading from env in C++ is `std::getenv("NROS_LOCATOR")` plus the
same `nros::init` call — see the GitHub source for the full pattern.

## Build

```bash
cd examples/native/cpp/talker
cmake -B build          # with ./activate.sh sourced, nano_ros_ROOT is
                        # exported; a copy-out passes
                        # -Dnano_ros_ROOT=<path-to-nano-ros> explicitly
cmake --build build
```

First configure builds nano-ros's Rust staticlibs (~3 minutes).
Re-builds finish in seconds.

## Run

Three terminals.

```bash
# 1. zenoh router (ROS 2's own):
source /opt/ros/humble/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd    # or: just zenohd

# 2. Run the talker:
cd examples/native/cpp/talker
./build/cpp_talker
# Expected:
#   nros C++ Talker
#   ===================
#   Node created: talker
#
#   Publishing messages (Ctrl+C to exit)...
#
#   Publishing: 'Hello World: 1'
#   Publishing: 'Hello World: 2'
#   …

# 3. Verify from stock ROS 2:
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Talker publishes best-effort; stock `ros2 topic echo` defaults to
# RELIABLE, so the QoS-mismatched echo silently delivers nothing.
# Force best-effort to receive:
ros2 topic echo /chatter std_msgs/msg/String --qos-reliability best_effort
```

**Readiness signal.** Within ~6 seconds of `./build/cpp_talker`
(session open + the first 1 s timer tick), the binary should print
`Publishing: 'Hello World: 1'` — Rust + C + C++ all start the count
at 1, matching the official ROS 2 demo talker. If no `Publishing:`
line in 30 seconds:

1. Confirm the router is running (terminal 1). Without it,
   `nros::init` returns `-100` (TransportError) — the
   `NROS_TRY_RET` macro logs the failed call to stderr.
2. Check stderr for `[nros] …/main.cpp:LINE nros::init(...) -> -N`
   diagnostics. `-3` / `-100` both indicate transport open failed.
3. See [Troubleshooting — First 10 Minutes](./troubleshooting-first-10-min.md).

## GitHub source

Canonical, copy-out:
[`examples/native/cpp/talker/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/cpp/talker)

## Next

- Add a subscription:
  [`examples/native/cpp/listener/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/cpp/listener)
- Services + actions:
  [`service-client/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/cpp/service-client),
  [`action-client/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/cpp/action-client)
- Parameters:
  [`parameters/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/native/cpp/parameters)
- Custom `.msg` / `.srv` / `.action`:
  [Message Generation](../user-guide/message-generation.md)
- Cross-compile for an RTOS: pick the right Embedded Starter from
  the next section.
