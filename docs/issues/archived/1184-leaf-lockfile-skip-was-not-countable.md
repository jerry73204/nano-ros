---
id: 1184
title: "`check-leaf-lockfiles` skips the two unsynced leaves and exits 0, so CI — which is always unsynced — never checked them and both locks drifted"
status: resolved
type: bug
area: ci, build
severity: medium
found: 2026-09-07
resolved: 2026-09-07
related: [0378, 0359, 0584, 1161]
---

# The branch CI always takes was the one that counted nothing

On a clean `origin/main` worktree the gate is green:

```
$ just check leaf-lockfiles
check-leaf-lockfiles: SKIP 2 leaf crate(s) — tree not synced
       packages/boards/nros-board-nuttx-qemu/nros-nuttx-ffi
       packages/boards/nros-board-nuttx-qemu/nros-nuttx-riscv-ffi
rc=0
```

On a synced tree, the same commit:

```
ERROR: leaf Cargo.lock drift in crate(s) not covered by the baseline:
       packages/boards/nros-board-nuttx-qemu/nros-nuttx-ffi
       packages/boards/nros-board-nuttx-qemu/nros-nuttx-riscv-ffi
```

Both leaves had genuinely drifted — `nros-macros` gained `nros-entry-lower`, and
neither lock was regenerated. The gate exists to catch exactly that (issue 0359:
a lock that pins nothing resolves fresh on every build), and it could not,
because **every CI checkout is unsynced**: `nros sync` writes `nros-patch.toml`
and the per-leaf `generated/` trees, and neither is committed. So the skip branch
is the branch CI always takes, and it printed to stderr and returned 0.

## Why this is the 0584 shape

`check-fast` reports `N gate(s) ran, M SKIPPED` from the `nros_check_skip`
ledger. This skip was never in it, so the summary said nothing, and a green lane
over two unchecked crates was character-for-character a green lane over two
checked ones. That is the distinction issue 0584 made countable for tests and
1161 is still open about for capabilities — here it is one gate over, in a lane
that runs on every push.

## Fixed

The skip is now `nros_check_skip leaf-lockfiles "tree not synced — N leaf
crate(s) NOT checked: …"`, so it lands in the ledger and in the lane summary.

**Deliberately still a SKIP, not a failure.** These leaves genuinely cannot
resolve without sync (issue 0378) — their `.cargo/config.toml` includes a
central patch table and redirects message crates at a `generated/` tree that is
produced, not committed — so failing closed would break the book's own install
flow. What changes is that "not checked" now survives into the summary instead of
being a line on stderr nobody tallies.

The two drifted locks were regenerated the sanctioned way (`just lock-update ""
"" <leaf>`) and reviewed: the diff adds `nros-entry-lower`, an in-repo path
dependency with no registry source, to each — a refresh, not a dependency
change.

## Verified both directions

| | |
| --- | --- |
| synced tree | gate PASSES, skip branch not taken |
| unsynced worktree | `[SKIPPED] leaf-lockfiles: tree not synced — 2 leaf crate(s) NOT checked: …`, rc 0 |

## Worth noting for whoever fixes 0378

The honest end state is that CI syncs, and then this skip never fires and the
gate covers every leaf on every push. The ledger entry is the interim: it makes
the hole visible without pretending it is closed.
