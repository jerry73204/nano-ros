---
id: 1182
title: "The two nuttx-ffi leaf locks have drifted, and `check-leaf-lockfiles` cannot
  see it because no lane checks out the submodule they patch"
status: open
type: bug
area: build
related: [0359, 0466, 1171]
---

## What

`packages/boards/nros-board-nuttx-qemu/nros-nuttx-ffi` and its `-riscv-ffi`
sibling fail `cargo metadata --locked`:

```
$ cd packages/boards/nros-board-nuttx-qemu/nros-nuttx-ffi
$ cargo metadata --locked --format-version 1 >/dev/null
warning: patch `libc v0.2.183 (<repo>/third-party/nuttx/libc)` was not used in
         the crate graph
error: cannot update the lock file … because --locked was passed to prevent this
```

`check-leaf-lockfiles` classifies both as **lock drift not covered by the
baseline** and exits 1.

## Why nobody has seen it

The precondition is a SUBMODULE, and no merge-gating lane checks it out.

Both leaves' own `.cargo/config.toml` patches `libc` at
`third-party/nuttx/libc`. Without that submodule the patch target does not
exist, `cargo metadata` fails on "unable to update `third-party/nuttx/libc`",
and `check-leaf-lockfiles` classifies the leaf as UNSYNCED. Which of its two
unsynced branches then runs depends on an unrelated file:

* **no central `nros-patch.toml`** (CI, a fresh clone) — SKIP with a warning.
  Issue 0466 made that deliberate: `check-fast` is specified buildless and
  source-free, so it may not demand a sync it is not allowed to perform.
* **`nros-patch.toml` present** (any synced dev tree) — hard FAIL saying "cannot
  resolve on a SYNCED tree … their patch tables or `generated/` trees are
  wrong", which is a **misdiagnosis**: the patch tables are fine and a submodule
  is simply absent. Measured here — the message named the patch tables, and
  `git submodule update --init third-party/nuttx/libc` is what fixed it.

Check out the submodule and the leaf finally resolves — and reports drift. So
neither the real state of these two locks, nor the actual reason they are
unreadable, has ever reached a lane in either direction.

## Not caused by the branch that found it

Found while landing issue 1171 (executor backing / picolibc arena pairing). That
branch modifies no `Cargo.toml`, no `Cargo.lock`, and no `.cargo/config.toml`
anywhere in either leaf's graph — `git status` shows the leaves untouched — so
the resolution inputs are byte-identical to `main`. It only made the gate REACH
them, by checking out `third-party/nuttx/libc` for an unrelated Zephyr build.
Verified both ways: with the submodule present the drift is reported whether or
not `nros-patch.toml` exists, and removing the submodule restores the SKIP.

## What to do

1. Refresh the two locks the sanctioned way (`just lock-update "" "" <leaf-dir>`,
   then READ the diff — a lock refresh that adds registry packages is a
   dependency change, per issue 0359). Understand the "patch `libc` was not used
   in the crate graph" warning first: a patch that goes unused is either a stale
   patch row or a dependency that stopped being reached, and the fix differs.
2. Decide how these leaves get COVERED. A gate whose verdict depends on whether
   the person running it happens to have one submodule is not gating anything;
   it is sampling. Either a merge-gating lane checks out
   `third-party/nuttx/libc`, or the skip is recorded as a known, NAMED gap
   rather than a warning that scrolls past.
3. Separate "the tree is unsynced" from "a submodule this leaf patches is not
   checked out" in the gate's message. They have different fixes, and today the
   second is reported as the first, in the loudest possible way.
