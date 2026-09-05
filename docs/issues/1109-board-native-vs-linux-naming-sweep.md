---
id: 1109
title: "77 `board = \"native\"` selectors predate the native/linux split — some are true, some are claims the board cannot make"
status: open
type: tech-debt
area: docs, tooling
severity: low
found: 2026-09-06
related: [1107, 1108]
---

# Two words, one selector, and nobody has said which is meant where

CLAUDE.md's naming section settles what the three words mean:

- **native** — ROLE: this is the HOST build, not a cross build.
- **posix** — REACH: works on any POSIX-compliant system.
- **linux** — REACH: works only on Linux.

`packages/boards/linux/nros-board.toml` declares `names = ["native", "linux"]`,
and the lookup accepts either (`names.iter().any(|n| n == board)`, in
`cmd/setup.rs:410` and `orchestration/board_descriptor.rs:713`). So both spell
the same board and nothing is broken.

What is unsettled is which word each CALL SITE means. Measured 2026-09-06:

| | count |
|---|---|
| `board = "native"` in a `system.toml` | 77, across 18 files |
| `[image.native]` image names | 18 |
| `board = "linux"` anywhere | **0** |

Zero. The `linux` spelling the board offers has no users in the tree, so every
site is inheriting the older word rather than choosing it.

## Why it is worth a sweep rather than a rename

The two are not synonyms and a blanket substitution would be wrong. A site that
means "build this for whatever host I am on" is correctly `native`. A site that
means "this runs on Linux" should say `linux` — and that is a real claim about
this board, because `nros-board-linux::apply_tier_affinity` calls
`sched_setaffinity` with `cpu_set_t`/`CPU_SET` ungated by `cfg(target_os)`, and
libc defines those for linux/android/freebsd/dragonfly/fuchsia/cygwin and NOT
apple. The crate does not build on macOS.

So the sweep is per-site judgement, not `sed`. The first consumer to write the
`linux` spelling is out-of-tree
([`orin-spe-heartbeat`](https://github.com/jerry73204/orin-spe-heartbeat)),
where the image is a Linux host build and says so.

## Not proposed here

Deleting `native` from the board's `names`. It is the honest word for a host
build and several sites mean exactly that; the defect is that no site has had to
choose.

## Shape of a fix

Read the 18 files, decide per image whether it means the ROLE or the REACH, and
write the word that is true. A gate is probably not worth it — "which did the
author mean" is not statically recoverable — but a note in the naming section
saying the two are both live and must be chosen deliberately would stop the next
site from inheriting.
