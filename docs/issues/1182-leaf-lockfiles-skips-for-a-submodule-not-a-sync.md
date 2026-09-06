---
id: 1182
title: "`check-leaf-lockfiles` skips two leaves for a SUBMODULE, not for a sync —
  and on a synced tree it says so by blaming the patch tables"
status: open
type: bug
area: build
related: [1178, 0359, 0466, 1171]
---

## What this adds to 1178

Issue 1178 (resolved) regenerated the two `nros-board-nuttx-qemu` FFI leaf locks
and named the visibility gap it left behind: *"the gate still cannot answer for
these two leaves in CI. Making it able to is a lane question (what a
merge-gating job would have to sync), not a lockfile one."*

This is that lane question, plus a correction to its premise. **The precondition
is not a sync. It is one submodule.**

## Measured

Both leaves' own `.cargo/config.toml` patches `libc` at
`third-party/nuttx/libc`. With that submodule absent, `cargo metadata` fails
before it can say anything about the lock:

```
error: failed to load source for dependency `libc`
Caused by: unable to update <repo>/third-party/nuttx/libc
Caused by: failed to read <repo>/third-party/nuttx/libc/Cargo.toml
```

`check-leaf-lockfiles` classifies that as UNSYNCED, and **which of its two
unsynced branches runs then depends on an unrelated file**:

* **no central `nros-patch.toml`** — SKIP with a warning. This is CI and a fresh
  clone, and it is deliberate (issue 0466: `check-fast` is specified buildless
  and source-free, so it may not demand a sync it is not allowed to perform).
* **`nros-patch.toml` present** — hard FAIL, saying

  > cannot resolve on a SYNCED tree … these leaves are genuinely unresolvable.
  > Re-run `nros sync`; if they persist, their patch tables or `generated/`
  > trees are wrong.

  That is a **misdiagnosis**. The patch tables are correct and the `generated/`
  trees are irrelevant; a submodule is simply not checked out, and `nros sync`
  will never fix it. Measured on a tree with the central patch and without the
  submodule; `git submodule update --init third-party/nuttx/libc` is what
  cleared it, and de-initialising it restored the skip — with the patch file
  present or absent, so the submodule is the variable and the sync is not.

Two consequences, and the second is the expensive one:

1. No merge-gating lane covers these two leaves, which is 1178's point.
2. The one environment that DOES reach them reports the wrong cause, in the
   loudest available form, and points at a command that cannot help.

## What to do

* Separate "the tree is unsynced" from "a submodule this leaf patches is not
  checked out". They have different fixes and today the second is reported as
  the first. The gate already knows the difference — the cargo error names the
  path.
* Then decide the coverage: either a merge-gating lane checks out
  `third-party/nuttx/libc` (it is small), or the skip becomes a NAMED, recorded
  gap rather than a warning that scrolls past. A gate whose verdict depends on
  whether the person running it happens to have one submodule is sampling, not
  gating.

## Provenance

Found while landing issue 1171 (executor backing / picolibc arena pairing),
which reached these leaves only by checking out `third-party/nuttx/libc` for an
unrelated Zephyr build. The lock DRIFT seen at that moment is issue 1178's and
was fixed on `main` at `5cafd395c` while this was being written; what is left is
the reporting and the coverage.
