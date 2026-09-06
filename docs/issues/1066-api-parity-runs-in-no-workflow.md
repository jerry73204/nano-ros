---
id: 1066
title: "`check-api-parity` is run by NO workflow on any event, and cannot run in
  the CI container because the image ships no `clang`"
status: open
type: bug
area: ci
related: [issue-1059, phase-379, phase-417, rfc-0089]
---

## Problem

`grep -rn "api-parity" .github/workflows/` returns **nothing**. It is step 4 of
`just ci gate` and a documented part of `ci-l1`, but no `pull_request`, no
`merge_group`, no `schedule` and no `workflow_dispatch` job invokes it.

So the parity ledger — 2158 rows whose whole purpose is to fail when our user
API drifts from rclc/rclcpp/rclrs — is checked only by whoever runs the full
local tier. That is how phase-421's `nros_node_get_serialization_format` (C) and
`Node::serialization_format` (C++) landed UNLEDGERED and stayed that way
(issue 1059).

## Two causes, both measured

**1. The image has no `clang`.** `scripts/api_parity/extract_cxx.py` extracts our
C and C++ surface by running

```
clang -Xclang -ast-dump=json …
```

`grep -n clang ci/docker/ci-base/Dockerfile` matched **nothing** before this
filing. The image installs `clang-format` separately and pinned
(`just setup-clang-format`), which is a different binary — an easy thing to
mistake for coverage.

**2. `cargo +nightly` names a toolchain the image does not have.**
`extract_rust.py` ran a bare `cargo +nightly doc`. The container installs
`nightly-2026-04-11` and creates no `nightly` alias, so there the bare form
either fails or silently fetches a *different*, unpinned nightly. rustdoc's JSON
is an UNSTABLE format, so "some other nightly" is not a harmless substitution:
its schema is exactly what the extractor parses.

Both were invisible locally because a developer box has clang and usually has a
`nightly` toolchain. Same shape as the rest of issue 1059's family — the local
environment answering a question the CI environment would answer differently.

## Cost, so the lane decision is not guessed

Measured in a pristine worktree, this machine:

| | |
| --- | --- |
| `api-parity` | **29.3 s** |
| leaves a `target/` dir | **no** — rustdoc writes into a temp `CARGO_TARGET_DIR` |

It resolves no fixture, needs no SDK, no QEMU and no ROS install (the rclcpp
side is the recorded surface under `docs/reference/api-surface/`). By the
`check-lane-contracts` rule it is a legitimate merge-gating lane.

## Fix

Cause 2 is fixed: `extract_rust.py` now reads the pin from
`tools/rust-toolchain.toml` instead of spelling `+nightly`.

Cause 1 is fixed in `ci/docker/ci-base/Dockerfile` (adds `clang`), but the image
publishes on **push to main**, so the gate step cannot be added in the same
commit — it would be red for a reason that says nothing about API parity.

**Remaining work: once `ghcr.io/newslabntu/nano-ros-ci:humble` has been rebuilt
with `clang`, add the `just check api-parity` step to `gate.yml`'s `check` job**
on `pull_request` + `merge_group`, beside `node-std-tests`. The placeholder
comment marking the spot is already in the file.
