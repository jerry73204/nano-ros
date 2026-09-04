# C API and CMake Integration

## Overview

Phase 140 — nano-ros is consumed exclusively via
`add_subdirectory(<repo-root>)` from the user's CMakeLists. There is
no install step, no install prefix, no `find_package(NanoRos)`.

The nros C API is built via CMake + [Corrosion](https://github.com/corrosion-rs/corrosion) (v0.6.1), which integrates Cargo into CMake. Each `add_subdirectory(nano-ros)` invocation builds the staticlib(s) for the selected `(NANO_ROS_PLATFORM, NANO_ROS_RMW)` tuple in-tree.

## Usage

```cmake
set(NANO_ROS_PLATFORM posix)
set(NANO_ROS_RMW     zenoh)
add_subdirectory(third_party/nano-ros nano_ros)

add_executable(my_app src/main.c)
target_link_libraries(my_app PRIVATE NanoRos::NanoRos)
nros_platform_link_app(my_app)
```

This wires include dirs, RMW staticlib, platform shim, and per-build
config header automatically. See
[build-as-subdirectory.md](../../book/src/getting-started/build-as-subdirectory.md)
for the full walkthrough.

**RMW backend selection:** Pass `-DNANO_ROS_RMW=<zenoh|dds|xrce|cyclonedds>` to cmake.

### `NANO_ROS_RMW` vs `NROS_RMW`

Two cache variables select the RMW backend and they are **not** the same knob:

- **`NANO_ROS_RMW`** — the root API knob, read directly by
  `packages/api/nros-c/CMakeLists.txt` when you `add_subdirectory(nano-ros)`
  it yourself (as in the `Usage` example above). This is the variable the
  nano-ros build itself branches on.
- **`NROS_RMW`** — a shorthand used by the standalone-example and
  workspace-helper CMake (`cmake/NanoRosWorkspace.cmake`, `nano_ros_workspace()`
  / `nano_ros_workspace_pkg_guard()`, ~lines 158-243). It is set from the
  `BACKEND` argument (or read back as a default when omitted) and then mapped
  onto `NANO_ROS_RMW` before nano-ros is imported — so anything set via
  `-DNROS_RMW=…` on one of the copy-out `examples/**/CMakeLists.txt` ultimately
  drives the same root knob.

In short: writing your own top-level `CMakeLists.txt` against nano-ros directly
→ set `NANO_ROS_RMW`. Using the `nano_ros_workspace()` / workspace-pkg-guard
helper (as the standalone examples and `templates/` workspaces do) → set
`NROS_RMW` (or the `BACKEND` argument), and let the helper translate it.

## Code Generation

C code generation uses `nano_ros_generate_interfaces()` (from `cmake/NanoRosGenerateInterfaces.cmake`, included automatically by the root `CMakeLists.txt` once nano-ros is `add_subdirectory`'d). The codegen tool (`nros-codegen`) is a Corrosion-built target reachable via `$<TARGET_FILE:nros-codegen>`.

Always use `nano_ros_generate_interfaces()` for message/service/action types — never hand-write CDR serialization or struct definitions. The API mirrors `rosidl_generate_interfaces()` from standard ROS 2: interface files are positional arguments resolved first locally, then via ament index, then from bundled interfaces.

```cmake
# Standard ROS 2 package — resolved via AMENT_PREFIX_PATH or bundled
nano_ros_generate_interfaces(std_msgs
    "msg/Int32.msg"
    SKIP_INSTALL
)

# Another standard package
nano_ros_generate_interfaces(example_interfaces
    "srv/AddTwoInts.srv"
    "action/Fibonacci.action"
    SKIP_INSTALL
)

# Custom project-local interfaces
nano_ros_generate_interfaces(${PROJECT_NAME}
    "msg/Temperature.msg"
    SKIP_INSTALL
)
```

Resolution order for each file: `${CMAKE_CURRENT_SOURCE_DIR}/<file>` → `${AMENT_PREFIX_PATH}/share/<target>/<file>` → `<nano-ros-repo-root>/share/nano-ros/interfaces/<target>/<file>` (the in-tree bundled-interfaces fallback; `_NANO_ROS_PREFIX` resolves to the `add_subdirectory`'d repo root, NOT a system install prefix — Phase 140 deleted those).

Type info structs (`nros_message_type_t`, `nros_service_type_t`, `nros_action_type_t`) are all defined in `nros/types.h`.

## Async Action Client (Executor-Driven)

Action and service client entrypoints split into two families:

- `nros_action_send_goal_async()`, `nros_action_get_result_async()`,
  `nros_action_cancel_goal()`, `nros_client_send_request_async()` — non-blocking,
  return immediately. Replies arrive via callbacks invoked from
  `rclc_executor_spin_some()`.
- `nros_action_send_goal()`, `nros_action_get_result()`, `nros_client_call()` —
  blocking convenience wrappers. They call the async variant and then drive the
  executor (`rclc_executor_spin_some` internally in a wall-clock budgeted loop)
  until the reply lands or timeout. They never call `zpico_get` directly — all
  I/O still flows through the registered executor. Calling any of them from
  inside a dispatch callback returns `NROS_RET_REENTRANT`.

The action-client blocking helpers (`nros_action_send_goal`,
`nros_action_get_result`) take the executor as an explicit `executor`
parameter because the action client stores its arena handle as an opaque
pointer into the executor's `_opaque` storage (set up by
`nros_executor_register_action_client`) rather than a wrapper pointer.
`nros_action_client_wait_for_action_server()` and
`nros_action_client_action_server_is_ready()` take the same explicit
executor argument for the same reason. `nros_client_call()` does not —
the service client stashes the executor pointer on
`nros_executor_register_client()` and recovers it internally.

Canonical call pattern (C):

```c
nros_support_init(&support, locator, domain_id);
rclc_node_init_default(&node, "c_action_client", "/", &support);
nros_action_client_init(&client, &node, "/fibonacci", &type_info);
nros_action_client_set_feedback_callback(&client, on_feedback, NULL);
nros_action_client_set_result_callback(&client, on_result, NULL);

nros_executor_init(&executor, &support, /*pool=*/4);
nros_executor_register_action_client(&executor, &client);

/* Warm-up: let zenoh discover the action server. */
for (int i = 0; i < 300; ++i) {
    rclc_executor_spin_some(&executor, 10000000ULL); /* 10 ms */
}

/* Async path: returns immediately; goal_response_callback fires during spin. */
nros_goal_uuid_t goal_uuid;
nros_action_send_goal_async(&client, goal_buf, goal_len, &goal_uuid);
while (!state.goal_responded) {
    rclc_executor_spin_some(&executor, 10000000ULL);
}

/* Blocking convenience: same async-then-spin pattern, packaged. */
nros_goal_status_t status;
uint8_t result_buf[512];
size_t result_len = 0;
nros_action_get_result(&client, &executor, &goal_uuid,
                       &status, result_buf, sizeof(result_buf), &result_len);
```

Phase 122.3 added a separate **L1 polling** family
(`nros_action_client_init_polling` + `nros_action_client_send_goal_raw` /
`_try_recv_goal_response_raw` / `_send_get_result_request_raw` /
`_try_recv_result_raw` / `_send_cancel_request_raw` /
`_try_recv_cancel_response_raw` / `_try_recv_feedback_raw`) for callers
that drive the action lifecycle without an executor arena. The L1 family
stores its `ActionClientCore` inline in the `nros_action_client_t._opaque`
slot and does not require `nros_executor_register_action_client`.

## System Install

There is no system install. Phase 140 deleted every `install(...)`
rule. nano-ros ships in source form — the user project owns whatever
install layout it needs for *its* binaries.

## FreeRTOS / NuttX Cross-Compilation

Cross-compiled examples consume nano-ros the same way as the native (host) build:
`add_subdirectory(<repo>)` with `NANO_ROS_PLATFORM` set to the
target RTOS. Pass `CMAKE_TOOLCHAIN_FILE` for the cross-compiler.

**FreeRTOS (ARM Cortex-M3, MPS2-AN385):**

```bash
cmake -S examples/qemu-arm-freertos/c/talker -B build/talker \
    -DCMAKE_TOOLCHAIN_FILE=$(pwd)/cmake/toolchain/arm-freertos-armcm3.cmake \
    -DNROS_RMW=zenoh
cmake --build build/talker
```

**NuttX (ARM Cortex-A7):**

```bash
cmake -S examples/qemu-arm-nuttx/cpp/talker -B build/talker \
    -DNANO_ROS_PLATFORM=nuttx \
    -DNANO_ROS_BOARD=nuttx-qemu-arm
cmake --build build/talker
```

The example's own `CMakeLists.txt` add_subdirectory's nano-ros; the
Corrosion target tree under `build/talker/cargo/` holds the
per-build staticlib (`libnros_c.a`, `libnros_rmw_zenoh_staticlib.a`, …).

## Board capabilities & deterministic build (RFC-0042)

The C/C++ build contract is **structural**, not convention-enforced — see
[RFC-0042](../design/0042-platform-build-determinism.md). Three pieces a board /
integrator interacts with:

- **One canonical `<nros/platform.h>`** (owned by `nros-platform-api`). There is
  exactly one header by that include name — no `-I`/`-isystem` ordering decides
  which ABI you get. It declares the full `nros_platform_*` C ABI (clock, alloc,
  sleep, tasks, sync, log, …) plus the capability macros below.
- **`[board.capabilities]` in `nros-board.toml`** is the single source of truth for
  a board's `heap` / `atomics` / `threads`. `cmake/NanoRosCapabilities.cmake`
  (`nros_board_capability_defines(<board_dir> OUT)`) reads it and emits the matching
  `-DNROS_PLATFORM_HAS_MALLOC` / `_ATOMICS` / `_MUTEX` — you do NOT hand-set these
  per build. A heap-capable bare-metal board declares `heap = true`; without it the
  generated-message **heap containers fail to *compile*** (a clear error), not link
  silently — the #38 gate. Declare capabilities once, in the board toml.
- **RMW backend link is manifest-driven.** `resolve_rmw()` emits
  `cmake/NanoRosRmwDispatch.cmake` (drift-guarded): the backend rlib + any extra
  link libs (e.g. Cyclone's `+libddsc +libstdc++`) come from that generated
  manifest, not hand-maintained cmake conditionals. Select the backend with
  `-DNROS_RMW=<zenoh|xrce|cyclonedds>`; the manifest supplies the rest.

## Zephyr Integration

**RMW backend selection** in `prj.conf`:
```ini
# Zenoh (default — connects to zenohd router)
CONFIG_NROS=y
CONFIG_NROS_RMW_ZENOH=y       # (default, can be omitted)

# XRCE-DDS (connects to Micro-XRCE-DDS Agent)
CONFIG_NROS=y
CONFIG_NROS_RMW_XRCE=y
CONFIG_NROS_XRCE_AGENT_ADDR="192.0.2.2"
CONFIG_NROS_XRCE_AGENT_PORT=2018
```

**API selection** in `prj.conf`:
```ini
CONFIG_NROS_RUST_API=y         # Rust API (default) — uses rust_cargo_application()
CONFIG_NROS_C_API=y            # C API — links libnros_c.a, uses nros_generate_interfaces()
```

Zenoh requires `CONFIG_POSIX_API=y` and elevated mutex counts. XRCE requires `CONFIG_NET_SOCKETS=y`. See existing examples in `examples/zephyr/` for complete `prj.conf` templates.
