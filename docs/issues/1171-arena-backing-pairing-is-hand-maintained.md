---
id: 1171
title: "The picolibc arena is lowered by a hand-copied `EXECUTOR_BACKING` size, so an
  executor knob change silently re-creates the double reservation it removed"
status: open
type: tech-debt
area: [core, embedded, build]
related: [1145, 0163, 0460, phase-392]
---

## What

Issue 1145 paired the two halves of phase-392 W6 on Zephyr: the executor
backing became a `.bss` static, so `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` no
longer has to hold it, and the leaf conf was lowered by the measured size.

The number in the conf is a **literal, copied from `nm` output**:

```
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=961320   # 1048576 - 87256
```

`EXECUTOR_BACKING`'s size is `ExecutorSizing::DEFAULT.u64_len()`, which is
derived from the executor knobs (`MAX_CBS`, `ARENA_SIZE`, `RX_BUF_SIZE`, …).
Move any of those and the subtrahend is wrong, in whichever direction:

* the backing GROWS -> the arena was lowered by too little -> the double
  reservation is partly back, silently, and only `mem-report` would notice;
* the backing SHRINKS -> the arena was lowered by too much relative to the
  saving available, which costs nothing but records a number that no longer
  means what its comment says.

Nothing checks it. The conf is a Kconfig fragment; the size is a Rust const in
another crate; and the only thing that has ever related them is a person with
`nm` output in front of them.

## Why it was not solved in 1145

The obvious fix — have the build derive the arena from the backing — crosses a
one-way boundary. `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` is resolved during the
Zephyr **Kconfig** stage, which completes before the Rust crate that defines the
size is compiled. Issue 0460 is the same wall from the other side (a knob
reaching the C lane and not the Rust one), and its answer was to make the Rust
side READ the dotconfig, which is the direction that works.

So the candidate shapes, none of them free:

1. **A post-link gate.** Read both symbols from the built ELF and assert
   `malloc_arena` is not larger than the pre-W6 arena minus `EXECUTOR_BACKING`.
   Needs a built image, so it belongs to a lane that builds Zephyr — which is
   currently no merge-gating lane (see 1158, 1167).
2. **Publish the size from `nros-node/build.rs`** into a file the conf
   generator reads, and emit the fragment rather than authoring it. Moves the
   arithmetic to one place but adds a generated conf, and the leaf confs are
   authored files today.
3. **Declare it and stop deriving.** Give the backing a Kconfig symbol whose
   default the crate is REQUIRED to match (the `-1` sentinel pattern
   `nros_cargo_build.cmake` already uses), so both sides read one number.

Shape 3 is the one that matches how every other knob in this tree behaves, and
it is the one that would let the arena default be written as an expression.

## Scope

Only `examples/zephyr/rust/talker/prj-zenoh.conf` carries the lowered number
today — that is the one leaf 1145 measured and ran. **Every other Zephyr Rust
leaf still reserves twice**, and each needs its own measurement because the
backing is knob-derived per image:

```
examples/zephyr/rust/{listener,service-client,service-server,...}/prj-*.conf
```

Doing them by hand multiplies this drift by the number of leaves, which is the
argument for fixing the mechanism before finishing the sweep.

## Not this issue

Whether the arena should be 1 MiB at all. It is generous, and nothing here
measured what the backend actually needs — 1145 lowered it by exactly the
amount W6 stopped drawing, and claimed nothing else.
