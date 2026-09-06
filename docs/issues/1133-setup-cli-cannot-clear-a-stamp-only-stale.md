---
id: 1133
title: "`just setup-cli` reports success while leaving the CLI STALE, when the
  only change was a file cargo does not build"
status: open
type: bug
area: build
related: [issue-0466, issue-0627, phase-375]
---

## Problem

`source_stamp.rs` hashes **all of `packages/cli`** plus the dirs named in
`packages/cli/cli-source-dirs.txt`. Cargo builds a subset of that: adding a file
under `packages/cli/nros-cli-core/tests/` changes the stamp the tree computes
and gives cargo nothing to rebuild, so `just setup-cli` finds the binary up to
date, prints `built: …`, and the baked stamp does not move.

Every consumer that checks freshness then fails, and the failure names the
correct symptom with no reachable fix:

```
in-tree nros CLI is STALE — its sources changed since it was built
  (source stamp bd8518b66c382aa4 != c2cd815459187af8)
Rebuild it (not auto-done — compiling at build/test time is forbidden):
    ./scripts/bootstrap.sh   (contributors: just setup-cli)
```

Running exactly that changes nothing, because there is nothing to compile.

Measured 2026-09-06 on `feat/board-process-w6`: adding
`packages/cli/nros-cli-core/tests/board_new_scaffold.rs` put
`check-declared-qos-header` (and every other gate resolving the CLI) into a red
state that three consecutive `just setup-cli` runs did not clear. `touch
packages/cli/nros-cli-core/src/lib.rs && just setup-cli` cleared it on the first
try.

## Why it is the interesting shape

This is the repo's recurring class one turn further in: a mechanism that is
correct, and a repair command for it that cannot reach every state the mechanism
can enter. The staleness check is right — the stamp really did change — and the
instruction it prints is right for the common case. What is missing is that
`setup-cli` treats "cargo had nothing to do" as "the artifact is current",
which is true of the BINARY and false of the STAMP.

It also makes a test file feel like it broke the build, which is the worst
possible time for a contributor to meet the CLI freshness machinery.

## Suggested fix

`setup-cli` should compare the built binary's baked stamp against the tree's
current stamp AFTER cargo returns, and force a rebuild (`touch` a source file,
or `cargo build --offline` with the stamp as an input) when they differ. That
keeps one definition of fresh instead of two.

A narrower alternative — excluding `tests/` from the stamp closure — is
tempting and wrong: `cli-source-dirs.txt`'s own header records that the previous
textual approximation of the closure was wrong in BOTH directions (23 dirs where
cargo resolves 8), and shrinking the closure by hand is how that happened.

## Workaround

```sh
touch packages/cli/nros-cli-core/src/lib.rs && just setup-cli
```
