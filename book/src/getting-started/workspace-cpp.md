# C / C++ multi-node workspaces

The four previous chapters
([project layout](./workspace-from-app-node.md),
[node packages](./workspace-node-pkgs.md), [bringup packages](./workspace-bringup.md),
[entry packages](./workspace-entry-pkg.md)) describe the canonical three-role
node + bringup + entry shape against the Rust path
(`nros::node!(…)` + `nros::main!(launch = …)`). This chapter shows the
**C and C++ path** through the same shape, role-for-role.

The parity gap is closed. Same launch.xml, same `package.xml`,
same `system.toml`, same workspace pkg-index — the only thing that
changes language-side is the cmake-fn / macro surface.

## TL;DR — side-by-side

| Role | Rust | C / C++ |
|---|---|---|
| **Node pkg** | `lib.rs` with `nros::node!(MyNode)` + `[package.metadata.nros.node]` in `Cargo.toml` | `Talker.{hpp,cpp}` with a `configure(::nros::Node&)` component method (C++) / `NROS_C_COMPONENT` (C); `CMakeLists.txt` calling `nano_ros_auto_add_library` + `nros_components_register_node` (RFC-0057) |
| **Bringup pkg** | `package.xml` + `system.toml` + `launch/*.launch.xml` (no `Cargo.toml`) | identical (language-agnostic) |
| **Entry pkg** | `src/main.rs` with `nros::main!(launch = "demo_bringup")` | `src/main.cpp` with `NROS_MAIN(nros::board::LinuxBoard, "demo_bringup:system.launch.xml")`; `CMakeLists.txt` calling `nano_ros_entry(NAME … BRINGUP "…/demo_bringup" LAUNCH default DEPLOY native)` |
| **Workspace root** | `Cargo.toml [workspace] members = […]` | `CMakeLists.txt` calling `nano_ros_workspace(BACKEND zenoh PLATFORM posix SUBDIRS src/talker_pkg src/listener_pkg src/native_entry)` |
| **Build** | `nros sync` + `cargo build -p native_entry` | `nros sync` + `cmake -S . -B build` + `cmake --build build` |
| **Boot** | `cargo run -p native_entry` | `./build/.../native_entry` |

The reference C++ workspace ships in-tree at
[`examples/workspaces/cpp/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/workspaces/cpp).
Copy the whole directory, rename the packages.

## Workspace layout

Identical structure to the Rust template, swapping `Cargo.toml` →
`CMakeLists.txt`:

```text
my_ws/
├── CMakeLists.txt                # nano_ros_workspace(SUBDIRS …)
└── src/
    ├── talker_pkg/               # Node pkg (C++)
    │   ├── package.xml
    │   ├── CMakeLists.txt        # nano_ros_auto_add_library + nros_components_register_node
    │   └── src/{Talker.hpp,Talker.cpp}
    ├── listener_pkg/             # Node pkg (C++)
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   └── src/{Listener.hpp,Listener.cpp}
    ├── demo_bringup/             # Bringup pkg (language-agnostic — copy/paste
    │   ├── package.xml           #          works between Rust and C++ workspaces)
    │   ├── system.toml
    │   └── launch/system.launch.xml
    └── native_entry/             # Entry pkg (C++)
        ├── package.xml
        ├── CMakeLists.txt        # nano_ros_entry(BRINGUP … LAUNCH …)
        └── src/main.cpp          # NROS_MAIN(…) one-liner
```

## Workspace root

Four declarations:

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_ws LANGUAGES C CXX)
include(<nano-ros>/cmake/NanoRosWorkspace.cmake)

nano_ros_workspace(
    BACKEND        zenoh                # zenoh | xrce | cyclonedds
    PLATFORM       posix                # posix | … (default posix)
    NANO_ROS_ROOT  <path-to-nano-ros>   # also: -D cache var, $NANO_ROS_ROOT,
                                        # or auto-walk for nros-sdk-index.toml
    SUBDIRS        src/talker_pkg
                   src/listener_pkg
                   src/native_entry
)
```

`nano_ros_workspace()` does the heavy lifting in one call:

1. Sets `NANO_ROS_PLATFORM=posix` + `NANO_ROS_RMW=zenoh`.
2. `add_subdirectory(<nano-ros>)` **once** at root scope (so per-pkg
   subdirs don't collide on re-include).
3. `include(NanoRosNodeRegister.cmake)` + `include(NanoRosEntry.cmake)`
   once.
4. `add_subdirectory(<each member>)` for each `SUBDIRS` entry.

Subdir CMakeLists begin with the dual call:

```cmake
nano_ros_workspace_pkg_guard()  # no-op inside a workspace; bootstraps standalone solo
```

— the cmake equivalent of cargo `[workspace]` discipline. Every member
compiles standalone (with `-DNANO_ROS_ROOT=<path>`) or as part of the
workspace; the per-pkg CMakeLists doesn't change between modes.

## Node pkg

A **typed component** (RFC-0043) — no `main()`. The pkg ships a class with a
`configure(::nros::Node&)` method that creates real entities (a `Publisher`, a
`Timer`) and binds member callbacks **by identity** (member-fn-pointer template
param, no string callback name, no interpreter). The Entry pkg constructs the
object and calls `configure(node)`; the executor dispatches the callbacks.

```cmake
# src/talker_pkg/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(talker_pkg LANGUAGES C CXX)
nano_ros_workspace_pkg_guard()
nros_find_interfaces(LANGUAGE CPP SKIP_INSTALL)

# RFC-0057: the ament shape — the library owns the sources, the register
# owns the identity (keyword parity with rclcpp_components_register_node).
# Interface deps (std_msgs__nano_ros_cpp) are wired automatically.
nano_ros_auto_add_library(talker_lib STATIC src/Talker.cpp)

nros_components_register_node(talker_lib
    PLUGIN talker_pkg::Talker      # any qualified name — upstream namespaces port verbatim
    EXECUTABLE talker
    SHAPE configure)               # this walkthrough uses the configure(Node&) shape
```

### Telling the build what the component creates

`NROS_EXECUTOR_MAX_CBS`, the subscriber slots and the arena are `const` sizes
compiled into nros-node before a component TU is compiled, while an RFC-0043
component wires itself in its CONSTRUCTOR, at runtime. Nothing the macros emit
can supply those sizes — it exists only after linking. So somebody has to state
what the image creates, ahead of the build.

That statement is a **contract sidecar** beside the launch file, not something
in this CMakeLists. `<stem>.contract.yaml` next to `<stem>.launch.xml` in the
bringup package:

```yaml
# src/demo_bringup/launch/system.contract.yaml
version: 1

nodes:
  talker:
    paths:
      on_timer:                                  # a path with a timer trigger
        trigger: { timer: { rate_hz: 1 } }       # IS the periodic callback
        output: [chatter]
    pub:
      chatter: {}
  listener:
    sub:
      chatter: {}

topics:
  /chatter:
    type: std_msgs/msg/Int32
    pub: [talker/chatter]
    sub: [listener/chatter]
```

`nodes:` keys are LOCAL endpoint names — an absolute path there is concatenated
onto the node's FQN and matches no topic. `topics:`, `services:` and `actions:`
wire those endpoints to absolute names and carry the type. `paths:` is where
timers live: the model records a path with no `input` as the periodic callback.

`nros sync` folds it into the resolved SystemModel, and the entry passes that
model to the sizing automatically — nothing here takes an argument for it.
Inspect the composed answer with:

```console
$ nros ws entity-inventory --metadata <build>/nros-metadata.json       --model <ws>/build/nros/models/demo_bringup/system_model.yaml
```

The derivation is all-or-nothing per image: **a component the contract does not
describe makes that image derive nothing** and keep its configured
`NROS_EXECUTOR_MAX_CBS`. A count taken over only the components that answered
is smaller than the image needs, and a short `MAX_CBS` fails entity creation at
boot — so "some of them" is refused rather than guessed at.

Note that a publisher is inventoried but claims no callback slot, so the entity
count and the derived `MAX_CBS` are legitimately different numbers.

> Earlier versions stated this per component, as
> `nano_ros_node_register(ENTITIES ...)`. That keyword is retired (phase-412)
> and now fails with a pointer here. It was a list beside the code with nothing
> comparing the two, and on the safety island one of them declared six
> subscriptions for a node that creates seven: every pool derived from it was
> short by one, and the eleventh subscription failed at boot with a transport
> error that named nothing.

```cpp
// src/talker_pkg/include/talker_pkg/Talker.hpp
#pragma once
#include <nros/component.hpp>
#include <nros/nros.hpp>
#include "std_msgs.hpp"

namespace talker_pkg {

class Talker {
    ::nros::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick();  // real body; bound via &Talker::on_tick (no name)

  public:
    ::nros::Result configure(::nros::Node& node);
};

}  // namespace talker_pkg
```

```cpp
// src/talker_pkg/src/Talker.cpp
#include "talker_pkg/Talker.hpp"

namespace talker_pkg {

void Talker::on_tick() {
    std_msgs::msg::Int32 m;
    m.data = count_++;
    (void)pub_.publish(m);
}

::nros::Result Talker::configure(::nros::Node& node) {
    ::nros::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    // Member-fn-pointer-as-template-param → no-alloc trampoline; `this` is ctx.
    return ::nros::bind_timer<Talker, &Talker::on_tick>(node, timer_, 1000, this);
}

}  // namespace talker_pkg
```

The Entry pkg constructs `Talker` in static storage and calls `configure(node)`
on the real executor — the same component model the Rust `nros::node!(Talker)` +
the C `NROS_C_COMPONENT` paths use, so C++, C, and Rust Node pkgs interoperate in
one launch graph.

Scaffold a C++ Node pkg with:

```bash
$ nros new --component my-talker --lang cpp --use-case talker
✓ Created nano-ros C++ Node pkg 'my-talker'
  Class     : my_talker::Talker
  Node      : talker
  Kind      : typed component (RFC-0043)
```

## Bringup pkg

Language-agnostic — copy verbatim from the
[bringup chapter](./workspace-bringup.md). `package.xml` +
`system.toml` + `launch/system.launch.xml`. No `Cargo.toml`, no
`CMakeLists.txt`. Stock ROS 2 launch.xml from nav2 / Autoware /
turtlebot3 pastes in modulo unsupported tags.

```xml
<!-- src/demo_bringup/launch/system.launch.xml -->
<launch>
  <node pkg="talker_pkg" exec="talker" name="talker"/>
  <node pkg="listener_pkg" exec="listener" name="listener"/>
</launch>
```

## Entry pkg

The C++ Entry pkg's `CMakeLists.txt` calls
`nano_ros_entry(BRINGUP … LAUNCH …)` — you name the input, and the entry
bakes from the SystemModel that `nros sync` resolved from it (`MODEL` is
the deprecated artifact-naming override):

```cmake
# src/native_entry/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(native_entry LANGUAGES C CXX)
nano_ros_workspace_pkg_guard()

nano_ros_entry(
    NAME    native_entry
    SOURCES src/main.cpp                              # user-authored
    BOARD   native
    BRINGUP "${CMAKE_CURRENT_SOURCE_DIR}/../demo_bringup"
    LAUNCH  default
    DEPLOY  native)
```

At configure time the cmake fn shells `nros codegen entry --lang cpp --typed`,
emits `${CMAKE_BINARY_DIR}/native_entry_nros_main_generated.cpp` (the canonical
`int main()` body that constructs each launch node's component + calls
`configure(node)` on the real executor via `LinuxBoard::run_components`),
appends it to the target's sources, and auto-links every
`<pkg>_<exec>_component` static lib the launch XML named.

### `BRINGUP` + `LAUNCH` — name your input (canonical)

The canonical spelling names your *input*: the Bringup package and a
launch selector. `nros sync` resolves that into a SystemModel build
artifact (under `<ws>/build/nros/models/<bringup>/` — never committed;
gate `check-no-tracked-models`), and the entry bakes from it, so the
same artifact drives the Linux runtime and every embedded image
(contracts, tiers, QoS never drift). `MODEL` — naming the resolved
artifact file directly — is a deprecated expert override; the two are
mutually exclusive. This is the shape the shipped workspace uses:

```cmake
nano_ros_add_executable(native_entry
    SOURCES src/main.cpp
    BOARD   native
    BRINGUP "${CMAKE_CURRENT_SOURCE_DIR}/../demo_bringup"
    LAUNCH  default
    TYPED
    DEPLOY  native)
```

`nros sync` resolves/refreshes the model whenever the bringup's launch
XML or `system.toml` is newer than it. The user's `main.cpp` is a single
declarative line:

```cpp
// src/native_entry/src/main.cpp
#include <nros/main.hpp>
NROS_MAIN(nros::board::LinuxBoard, "demo_bringup:system.launch.xml")
```

`NROS_MAIN(...)` is a sentinel macro — the cmake fn owns the generated
body, the user's TU is documentation + IDE hint.

`nros new` scaffolds an Entry pkg:

```bash
$ nros new my-entry --lang cpp --platform native
```

## Build + boot

```bash
# `nros sync` resolves the SystemModel (via the pinned nros-launch-resolve
# helper) whenever the launch XML or system.toml is newer than the model.
nros sync
cmake -S . -B build -DNANO_ROS_ROOT=<path-to-nano-ros>
cmake --build build
./build/src/native_entry/native_entry
```

The build produces:

- `src/talker_pkg/libtalker_pkg_talker_component.a` — the Node pkg static lib
  (`_component` is the compatibility target suffix).
- `src/listener_pkg/liblistener_pkg_listener_component.a` — ditto.
- `src/native_entry/native_entry` — the Entry exe, with the generated
  `int main()` + register-call sequence + Board boot stub linked in.

`cmake configure` is incremental — pinned `CMAKE_CONFIGURE_DEPENDS` on
every file `nros codegen entry` reads (depfile from the CLI), so any
launch.xml / `package.xml` / Node pkg edit re-runs codegen.

## What runs where

| Concern | Lives in |
|---|---|
| Node entities + real callbacks | Node pkg `configure(::nros::Node&)` |
| Topology + launch args + per-target deploy | Bringup pkg `system.toml` + `launch/*.launch.xml` |
| `int main()` + executor init + spin | Generated TU emitted by the Entry pkg's cmake fn |
| Board + RMW selection | Entry pkg's `nano_ros_entry(BOARD …)` arg |

Same partition as the Rust track — the only thing that changes is the
syntax the user types into the three pkgs.

## C / C++ scaffolding via `nros new`

| Command | Output |
|---|---|
| `nros new <name> --lang cpp --platform native` | C++ Entry pkg (single-Node self-bringup; swap in a multi-Node `LAUNCH` arg post-219.D) |
| `nros new <name> --lang c --platform native` | C Entry pkg (same shape) |
| `nros new --component <name> --lang cpp --use-case talker` | C++ Node pkg; `--component` is the compatibility scaffold flag |
| `nros new system <name>_bringup --components a,b` | Bringup pkg (language-agnostic — works for both Rust and C++ workspaces) |

The C-side compatibility scaffold (`nros new --component … --lang c`) is
available for pure-C Node pkgs. Pure-C and mixed C/C++ workspace examples
live under `examples/workspaces/`.

## See also

- [`examples/workspaces/cpp/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/workspaces/cpp)
  — the canonical reference workspace (talker + listener Node pkgs +
  Bringup pkg + Entry pkg, all C++).
- [C++ Entry-pkg roadmap doc](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/roadmap/archived/phase-219-cpp-entry-pkg.md)
  — full landing order + acceptance bar.
- [Multi-node workspace layout design](https://github.com/NEWSLabNTU/nano-ros/blob/main/docs/design/0024-multi-node-workspace-layout.md)
  §11 — LOCKED canonical shape (`Bringup + Node + Entry`).
