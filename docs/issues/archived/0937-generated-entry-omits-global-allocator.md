---
id: 937
title: "The NuttX Entry reached the tree's only `#[global_allocator]` through the
  selection facade, and the facade is emitted only if its directory happens to exist"
status: resolved
type: bug
area: codegen
related: [0594, 0616]
---

## Correction to the first version of this issue

The first filing blamed `builder/entry.rs:344` for naming only `platform_feature`
and prescribed a new board-descriptor field so the Entry would name
`global-allocator` itself. **That diagnosis was wrong**, and it was wrong in the
way that matters: it named a plausible cause without measuring the graph, so the
prescribed fix would have papered over the real mechanism while leaving it live
for every other board.

What is true: no generated Entry names `global-allocator`. What is false: that
this is why nuttx failed. Boards do not rely on the Entry for it — they enable it
on their own `nros-platform` dependency, and cargo feature unification carries it
to the Entry. Five of six bare-metal/RTOS boards do exactly that.

## The actual mechanism

`nros-board-nuttx-qemu` was the one board that did **not**:

```
nros-board-mps2-an385                 features = ["platform-mps2-an385", "global-allocator", "critical-section"]
nros-board-mps2-an385-freertos        features = ["platform-freertos", "global-allocator"]
nros-board-mps3-an536-freertos        features = ["platform-freertos", "global-allocator"]
nros-board-s32z270-freertos           features = ["platform-freertos", "global-allocator"]
nros-board-threadx-qemu-riscv64       features = ["platform-threadx", "global-allocator"]
nros-board-nuttx-qemu                 (none)
```

It reached the allocator by a longer route — `nros-board-nuttx-qemu/default =
["image-runtime"]` → `nros-board-nuttx/image-runtime =
["nros-platform/global-allocator"]`. But the generated Entry depends on the board
crate with `default-features = false`, so that route runs **only** through the
generated selection facade, which re-enables `image-runtime` explicitly.

And the facade is optional (`cmd/build.rs:927`):

```rust
let facade_dir = {
    let d = root.join("generated/nros-selection")
               .join(crate::builder::entry::package_name(image_id));
    d.is_dir().then_some(d)          // absent -> None, silently
};
```

A bare directory probe. When it misses, `builder/entry.rs:287` emits the Entry
without the facade and says nothing — dropping the RMW, the ROS edition, the
capability features and, for NuttX alone, the allocator.

## Why it looked impossible

In the same nightly job, in the same cargo run order:

```
7765  Compiling nuttx_entry_nros_selection  (examples/workspaces/rust/generated/...)
7766  Compiling nuttx_entry                 (examples/workspaces/rust/...)          -> OK
7949  Compiling nuttx_entry                 (examples/workspaces/realtime-rust/...) -> no global memory allocator
```

One selection-crate compile against two Entry compiles. `workspaces/rust` had a
facade; `realtime-rust` did not, though `nros sync` ran for both. Identical Entry
manifests on a developer host, because there the facade exists and the failure
cannot reproduce.

## Verified by measurement, not by rebuild

The graph was measured with `cargo tree -e features -i nros-platform` in
`examples/workspaces/realtime-rust/build/nuttx-zenoh/nuttx_entry`, removing the
facade dependency to reproduce the CI condition. No NuttX SDK needed:

| board fix | facade | `global-allocator` in the graph |
| --- | --- | --- |
| no | present | 1 — via `image-runtime` (why a dev host always passes) |
| no | **absent** | **0** — the CI failure |
| yes | absent | 1 — via the board crate |
| yes | present | 1 |

## Fixed

1. `nros-board-nuttx-qemu` names `global-allocator` on its own `nros-platform`
   dependency, unconditionally, as its five siblings do. The board is now
   self-sufficient and does not depend on the facade for a lang item.
2. A missing facade now WARNS, naming what the Entry is being built without and
   pointing at `nros sync`. It stays a warning rather than an error because an
   unsynced workspace is a state `nros build` is documented to tolerate — what it
   may not do is tolerate it without saying so. That half is the class fix: every
   other board still loses its RMW, edition and capability features to the same
   silent branch, and those failures will be no easier to read than this one.

Still open, deliberately not guessed at here: **why** `nros sync` produced a
facade for `examples/workspaces/rust` and not for `examples/workspaces/realtime-rust`
in the same job. The two builds also resolve different board tokens for the same
platform — `nuttx-qemu-arm` against `nuttx` — which is the next thread to pull.
The warning above is what makes that thread visible the next time it happens.
