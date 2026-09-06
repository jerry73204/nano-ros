---
id: 999
title: "`preflight::check` read `config/rust-targets.txt` from the WORKSPACE root, so every build-std target was reported as an uninstalled rustup one"
status: resolved
type: bug
area: [cli, ci]
severity: medium
related: [1039, 1017]
resolved_in: "c003ae608 (2026-09-03)"
---

# Filed late: the id was reserved, the fix landed, the file never got written

The id `0999` is reserved (`refs/issue-ids/0999`) and the fix is on `main`, but
no `docs/issues/0999-*.md` was ever committed. Four references in
[phase-416](../../roadmap/phase-416-tier2-lane-and-single-spelling.md) point at
it, so a reader following them arrives nowhere. This is that file, written from
the fix commit and from the phase doc rather than from memory.

Its sibling `#0998` has no file either — same cause, same week (`bed98e8ef`
deletes the duplicated sertype copy). It is not reconstructed here because
nothing cites it.

## What was wrong

`preflight::check` receives the **workspace** root — the user's colcon tree —
while `config/rust-targets.txt` lives in the nano-ros **checkout**. The read
therefore found no file, fell back to `Provisioning::Rustup`, and every
build-std target went on being reported as an uninstalled rustup target:

```
- Rust target `armv7a-nuttx-eabihf` (board `nuttx`)
    run: rustup target add armv7a-nuttx-eabihf
```

A build-std target has nothing to install, so the advice was not merely noisy —
it was impossible to act on, and it failed the nightly `nuttx` cell.

The classifier was right. The root handed to it was wrong.

## The fix

`c003ae608` — `check` takes the checkout as well; `cmd/build.rs` already had it
two stages up as `nano_ros_root`. With no checkout to read, the conservative old
behaviour stands rather than a silent pass: preflight must not invent a pass for
a target it cannot classify.

## Why this issue is one of phase-416's own subjects

Two of that phase's recurring shapes happened to the fix itself, which is why
the phase doc cites this issue four times.

**The first fix did nothing, and was "verified".** It was checked with
`nros build --dry-run` in `examples/workspaces/c`, which resolved
`demo_bringup:nuttx` without complaint — and passed for an unrelated reason. The
commit message states the lesson better than a summary can: *a passing COMMAND
is not a passing ASSERTION*. The unit test written alongside covered
`target_provisioning` in isolation, which was the half that was already correct.

**The test could not fail.** Phase-416 records that `#0999`'s own test *"searched
ancestors for the file it was meant to prove was findable"* — so whatever root it
was given, it found the file, and the assertion was vacuous. Four gates written
during that campaign had this shape; it is the same class as issue 0196.

The landed fix adds the test that would have caught it: `check()` itself, with a
build-std board and the real checkout, asserting no `rustup target add` is
reported. Mutation-checked by reinstating the bug, which reds it with the exact
message the nightly printed.

## Acceptance

* [x] `preflight::check` resolves `config/rust-targets.txt` from the checkout.
* [x] A build-std target reports nothing to install.
* [x] The test fails when the bug is reinstated (mutation-checked in
      `c003ae608`).
* [x] This file exists, so phase-416's four references resolve.
