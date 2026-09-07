# Mixed-language workspace

nano-ros Node pkgs are linked through a C ABI register trampoline, so one
image can compose Node pkgs written in different languages. The native
reference shape is:

- C Node pkgs for C code you want to keep in C.
- C++ and Rust Node pkgs beside them, in the same workspace.
- One Bringup pkg with the normal ROS 2 launch XML and the `[image.*]` rows.

The mixed workspace is in
[`examples/workspaces/mixed/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/workspaces/mixed).
For a pure-C workspace, use
[`examples/workspaces/c/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/workspaces/c).

## Layout

```text
my_ws/
├── .colcon_workspace             # tracked marker; the root CMakeLists.txt is GENERATED
└── src/
    ├── c_talker_pkg/             # C Node pkg
    ├── cpp_listener_pkg/         # C++ Node pkg
    ├── rust_heartbeat_pkg/       # Rust Node pkg
    └── demo_bringup/             # launch XML + system.toml (+ [image.*])
```

Mixed is not a special case. The driver is chosen by the **board**, and cmake
wins whenever the package graph crosses languages — corrosion makes cargo
consumable from cmake, and nothing makes cmake consumable from cargo (RFC-0024
§6.3). So a mixed workspace builds exactly like the C++ one:

```console
$ cd examples/workspaces/mixed
$ nros build native
```

`nros build` writes the root `CMakeLists.txt` — a `nano_ros_workspace(…)` call
listing the discovered packages, plus one `nano_ros_add_executable(…)` per
image on that coordinate — into `build/<coordinate>/`, configures it into
`build/<coordinate>/cmake/`, and builds. Neither the root nor the entry is
yours to write.

## C Node pkg

A C Node pkg is a **typed component** (RFC-0043): no `main()`. It defines a
state struct + a `configure(node, executor, self)` function and exports the
C-ABI factory/configure seam via `NROS_C_COMPONENT(StateT, configure_fn)` — the
typed Entry creates the node and runs `configure` on the real executor.

```cmake
# Raw `/chatter` publisher carries the type name as a string → no generated C
# bindings needed (so no nros_find_interfaces for this pkg).
nano_ros_auto_add_library(talker_lib STATIC src/Talker.c)

nros_components_register_node(talker_lib
    PLUGIN c_talker_pkg::Talker
    EXECUTABLE talker
    SHAPE configure
    TYPED
    DEPLOY native)
```

```c
#include <stdint.h>
#include <nros/component.h>

typedef struct {
    _Alignas(8) uint8_t pub[NROS_C_PUBLISHER_STORAGE_SIZE];
    int32_t count;
} c_talker_pkg_t;

static void on_tick(void* ctx) {
    c_talker_pkg_t* self = (c_talker_pkg_t*)ctx;
    uint8_t buf[8] = {0x00, 0x01, 0x00, 0x00};  /* CDR_LE header + LE int32 */
    buf[4] = (uint8_t)self->count;
    (void)nros_cpp_publish_raw(self->pub, buf, sizeof(buf));
    self->count++;
}

static nros_ret_t talker_configure(const nros_cpp_node_t* node, void* executor,
                                   c_talker_pkg_t* self) {
    self->count = 0;
    int32_t rc = nros_cpp_publisher_create(node, "/chatter",
        "std_msgs::msg::dds_::Int32_", "", nros_c_qos_default(), self->pub);
    if (rc != 0) return rc;
    size_t timer;
    return nros_cpp_timer_create(executor, /*period_ms=*/1000, on_tick, self, &timer);
}

NROS_C_COMPONENT(c_talker_pkg_t, talker_configure)
```

`nano_ros_auto_add_library` injects `NROS_PKG_NAME` (language inferred from the sources), so
`NROS_C_COMPONENT` exports the `__nros_c_component_<pkg>_{create,configure}`
seam the typed Entry calls — interoperable with C++ `configure(Node&)` and Rust
`nros::node!` components in one launch graph.

## The image

Nothing about the language mix reaches the image declaration. One row per
program, in `src/demo_bringup/system.toml`:

```toml
[image_defaults]
rmw = "zenoh"

[image.native]
board = "native"
```

The launch file names the Node pkgs, whatever they are written in:

```xml
<launch>
  <node pkg="c_talker_pkg" exec="talker" name="talker"/>
  <node pkg="cpp_listener_pkg" exec="listener" name="listener"/>
</launch>
```

From that pair, `nros build` emits this into the generated root. `LANG` follows
the workspace's own sources; `BOARD` and `DEPLOY` come from the resolved image,
so a generated entry cannot disagree with the board it was generated for. There
is no `SOURCES`, because the verb generates the translation unit carrying
`main`:

```cmake
nano_ros_add_executable(native_entry
    BOARD   native
    BRINGUP "${CMAKE_CURRENT_SOURCE_DIR}/../../src/demo_bringup"
    LAUNCH  default
    LANG    cpp
    TYPED
    DEPLOY  native)
```

The generated entry translation unit calls each package's
`__nros_component_<pkg>_register` symbol and the CMake sidecar links the
matching static libraries. The symbol keeps the legacy `component` spelling for
ABI compatibility; the user-facing package role is still Node pkg.

Note there is no `MODEL` here. The SystemModel is a build artifact resolved
from the launch file by `nros sync`; naming it in a build file is the
deprecated expert override, and a *committed* one is banned outright (gate
`check-no-tracked-models`).

## Scaffolding

```sh
# Current compatibility scaffold for Node pkgs:
nros new --component c_talker_pkg --lang c --use-case talker
nros new --component cpp_listener_pkg --lang cpp --use-case listener
nros new system demo_bringup --components c_talker_pkg,cpp_listener_pkg
```

For a complete working tree, copy the template instead of creating
each package separately.
