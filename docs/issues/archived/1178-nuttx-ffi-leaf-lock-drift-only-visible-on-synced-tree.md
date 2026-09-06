---
id: 1178
title: "Two nuttx-ffi leaf locks missed `nros-macros`'s new path dep, and
  `check-leaf-lockfiles` can only see it on a SYNCED tree — so no CI job can"
status: resolved
type: bug
area: tooling, ci
related: [0359, 0378, 0463]
---

`nros-macros` gained an in-repo path dep, `nros-entry-lower`. Two leaf locks
were not regenerated with it:

    packages/boards/nros-board-nuttx-qemu/nros-nuttx-ffi/Cargo.lock
    packages/boards/nros-board-nuttx-qemu/nros-nuttx-riscv-ffi/Cargo.lock

so each pinned nothing for that subtree and resolved fresh on every build —
issue 0359's class.

## Why it landed on `main` green

`check-leaf-lockfiles` **skips these two leaves unless the tree is synced**,
and says so:

    Every OTHER leaf lock was still checked. Run this on a synced tree
    to cover these two as well.

Every CI worktree is unsynced, so the gate reports OK there and can only ever
fail on a developer's populated tree. A gate that skips exactly where it runs
is the visibility half of issues 0981/1001 in a third place: the skip is
honest, correctly reported, and still means no merge-gating job can catch this
class.

The commit-date evidence points the wrong way and should not be trusted here —
the LOCKS are newer than the manifests (2026-09-05 vs 2026-08-18), because what
changed was a transitive in-repo dep, not the leaf's own manifest.

## Fixed

`just lock-update "" "" <leaf-dir>` on both. The diff is 8 lines each, all
in-repo path packages, no registry additions — a refresh, not a dependency
change, which is the review this recipe asks for.

Left open by this fix: the gate still cannot answer for these two leaves in
CI. Making it able to is a lane question (what a merge-gating job would have to
sync), not a lockfile one.
