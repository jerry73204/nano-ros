---
id: 1136
title: "`LAUNCH_ARGS` was removed from `nano_ros_entry` while the CLI still emits it — every `host-tests` integration run has failed for at least 30 runs"
status: open
type: bug
area: build, cli, testing
severity: high
found: 2026-09-06
related: [1127, 0433]
---

# The one lane that could produce a live-peer verdict has not finished in 30 runs

`host-tests.yml`'s `nros-tests integration (host)` job fails at
**Build workspace fixtures**, every time. Measured 2026-09-06 over the last 30
runs of that workflow: **0 success, 10 failure, 18 cancelled, 2 in flight.** The
last five failures all die at the same step.

```
CMake Error at cmake/NanoRosEntry.cmake:426 (add_executable):
  Cannot find source file:

    LAUNCH_ARGS
Call Stack (most recent call first):
  cmake/NanoRosVerbs.cmake:192 (nano_ros_entry)

Error: configure failed: `cmake -S build/posix-zenoh-native -B build/posix-zenoh-native/cmake -DNROS_RMW=zenoh`
error: recipe `build-workspace-fixtures` failed on line 234 with exit code 1
```

So `just ci tier1` never runs, and the interop cells it would carry never
produce a result. This is the mechanism behind issue 1127.

## Cause

Phase-405 W1 dropped `LAUNCH_ARGS` from `nano_ros_entry`'s
`cmake_parse_arguments` keyword list. `cmake/NanoRosEntry.cmake:131` now reads:

```cmake
"NAME;BOARD;LAUNCH;LANG;HOST;BRINGUP;PANIC"
```

The CLI never stopped emitting it —
`packages/cli/nros-cli-core/src/builder/cmake_root.rs:347`:

```rust
out.push_str(&format!("    LAUNCH_ARGS {k}={v}\n"));
```

— and `cmake_root.rs:580` is a unit test asserting the emission
(`body.contains("LAUNCH_ARGS host=robot1")`), so the producer side is green
while the consumer rejects it. Unknown keyword lands in
`_NRA_UNPARSED_ARGUMENTS`, which flows into `add_executable`, which reports it
as a missing **source file**.

## The removal's own reasoning names the mistake

`cmake/NanoRosEntry.cmake:111`:

> phase-405 W1 — LOCATOR, ARGS and LAUNCH_ARGS are GONE. Each had zero
> authored users in the tree (**generated CMakeLists excluded, since those are
> tool output rather than a caller's choice**), and each carried live-looking
> code that could never run.

The exclusion is the defect. The generator *is* a caller — the only one that
runs in CI — so the survey that justified the removal excluded exactly the
users that existed. CLAUDE.md's rule is "fix the CLASS, then prove the sweep";
here the sweep was scoped so that it could not see them.

The removal also predicted the failure mode and got it half right: "Dropping
them from the parse turns such a call into an UNPARSED_ARGUMENTS error rather
than a line that silently does nothing." It does error — as
`add_executable: Cannot find source file: LAUNCH_ARGS`, which names neither the
keyword, nor that it was retired, nor which generator emitted it.

## Not cosmetic — the argument is load-bearing

`examples/workspaces/c/src/demo_bringup/system.toml:83`:

> `args` carries over from the entry each image was derived from. It is how an
> image selects a MACHINE — `native_entry_robot1` and `_robot2` differ ONLY in
> `LAUNCH_ARGS host=`, so an image without it resolves the whole system instead
> of one host and the two would be the same program.

So "stop emitting it" is not the fix: it would make two distinct images
identical, silently. Either restore the keyword and wire it to the launch
resolution, or give the CLI the replacement spelling — and the multi-host
workspaces need a test that the two images differ.

Affected generated roots (`LAUNCH_ARGS host=robot{1,2}`):
`examples/workspaces/{c,cpp,mixed}/build/posix-zenoh-native/CMakeLists.txt`,
and the same gap is noted for `workspaces/rust` (phase-383 W9.a).

## Why no gate caught it

`just check fast` is green: this is a cmake *configure* of a generated
workspace, which no fast gate performs. The producer's unit test asserts the
string it emits, never that a consumer accepts it. Nothing compares the
keywords the CLI emits against the keywords `nano_ros_entry` parses — and that
comparison is a cheap static gate someone should write, since both sides are
literals in the tree.

## Fix, in order

1. Decide restore-the-keyword vs rename-in-both-places. Restoring is smaller
   and keeps the generated output stable.
2. Whichever way, make the CLI's emitted keyword set and `nano_ros_entry`'s
   parsed keyword set agree, and gate that agreement.
3. Assert the multi-host images differ, so a future removal cannot make them
   identical without a red test.
4. Re-run `host-tests.yml` and confirm the integration job reaches `just ci
   tier1` — that is what unblocks issue 1127's verification work.
