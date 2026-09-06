---
id: 1111
title: "The generated cmake root never sets `NROS_INTERFACE_SEARCH_PATH`, so a C++ node consuming its own workspace's msg package fails `find_package` with a raw cmake error"
status: resolved
type: bug
area: cli, tooling
severity: medium
found: 2026-09-06
related: [0862, 1107, 1108]
---

# The builder knows the answer and does not say it

A workspace with an in-workspace interface package consumed by a C++ node
cannot be built by `nros build` on a clean shell:

```
CMake Error at src/heartbeat_pkg/CMakeLists.txt:9 (find_package):
  By not providing "Findspe_msgs.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "spe_msgs",
  but CMake did not find one.
```

Export `NROS_INTERFACE_SEARCH_PATH=<ws>/src` and the same command succeeds:

```
-- nros: find_package(spe_msgs) -> .../src/spe_msgs
[100%] Built target linux_entry
```

## Why

An interface package is deliberately NOT a subdir of the generated root —
`builder/cmake_root.rs` says so at the discovery loop (issue 0862): its
CMakeLists is verbatim upstream ROS, nano-ros never configures it, and
`nros sync` routes `rosidl_generate_interfaces` through the codegen pipeline
instead.

So a component's `find_package(<pkg>)` resolves through the compat layer's
auto-emitted Find-stub, which scans `NROS_INTERFACE_SEARCH_PATH`.
`cmake/NanoRosGenerateInterfaces.cmake` documents the shape, including that the
ordering matters:

```cmake
set(NROS_INTERFACE_SEARCH_PATH "${CMAKE_SOURCE_DIR}/src")
# MUST precede `find_package(nano_ros)`
```

The hand-written roots set it. **The generated root did not** — `grep -c
NROS_INTERFACE_SEARCH_PATH` over a generated root returned 0 — and
`nano_ros_workspace()` does not set it either. The only two callers of
`nros_workspace_interfaces()` in the tree are templates with hand-written roots.

## Not "unsupported" — unstated

`nros ws env` exists and prints the export, so a user who knows to run
`eval "$(nros ws env)"` is fine. Two things make it a defect anyway:

1. **The builder already knows the workspace root.** It writes `WORKSPACE_ROOT`
   into the same file three lines later. Leaving the answer to the caller's
   environment is the shape CLAUDE.md rejects for knobs: a value that arrives
   through a dependency edge cannot be poisoned by an ambient variable, and one
   that arrives through the environment can.
2. **The error names neither the cause nor the remedy.** It is cmake's generic
   `find_package` failure. Nothing in it mentions interface packages, the search
   path, or `nros ws env`.

## Fix

The emitter sets it, immediately before `find_package(nano_ros)`, from the
workspace root it already computes. It **prepends** rather than assigns, so a
caller who has sourced `nros ws env` — or who points at an out-of-workspace
package tree — keeps their entry and its precedence, since the search path's
shadowing rule is that earlier roots win.

Guarded by `the_interface_search_path_precedes_find_package_nano_ros` in
`builder/cmake_root.rs`, beside the test that pins the toolchain-before-project
ordering, which is the same class of bug — a value set after the thing that
reads it is a value nobody used. Mutation-tested both ways: moving the block
after `find_package(nano_ros)` fails it, and assigning instead of prepending
fails it.

## How it was found

Building a first out-of-tree consumer
([`orin-spe-heartbeat`](https://github.com/jerry73204/orin-spe-heartbeat)),
whose C++ node publishes its own workspace's message type. Every in-tree
workspace with that shape (`examples/workspaces/features`) is exercised through
fixture rows that carry a `codegen_out` step, so the gap did not surface there.
