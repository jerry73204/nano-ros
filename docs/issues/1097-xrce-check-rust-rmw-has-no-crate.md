---
id: 1097
title: "`just xrce check-rust-rmw` runs `cargo check` in a directory that has held no `Cargo.toml` since phase-140"
status: open
type: tech-debt
area: rmw, build
severity: low
found: 2026-09-05
related: [phase-420, phase-140]
---

# A recipe that cannot work, on no lane that runs

`just/xrce.just`:

```just
# Phase 121.7.i — `cargo check` the out-of-workspace nros-rmw-xrce Rust
# crate so CI catches drift after platform-API or trait-surface changes.
# Workspace `cargo check` skips it (it's workspace-excluded for xrce-sys
# linking reasons), so this recipe is the only place it gets compiled.
[group("debug")]
check-rust-rmw:
    cd packages/rmw/xrce/nros-rmw-xrce
    cargo check --quiet
```

`packages/rmw/xrce/nros-rmw-xrce/` is a **CMake project**. It ships
`CMakeLists.txt`, `include/`, `src/*.c`, `tests/` and `package.xml`, and no
`Cargo.toml` — phase-140 deleted the install rules and the Rust side moved to
`nros-rmw-xrce-cffi`. So the recipe fails with `could not find 'Cargo.toml'`,
and the comment above it is wrong twice over: there is no "out-of-workspace
nros-rmw-xrce Rust crate", and this is not "the only place it gets compiled".

It is not on `check`, `ci` or any CI lane — it is `[group("debug")]` and named
by nothing — which is why a recipe that cannot succeed has sat here for
phases. That is the point of filing it rather than leaving it: a broken recipe
nobody runs reads, to the next person, as coverage that exists.

**Found by** phase-420 W9 step 4, while enumerating who compiles the vendored
XRCE trees. Not fixed there: outside that change's scope, and the fix is a
judgement call rather than a mechanical one.

## What the fix probably is

Delete it. The intent it names — "catch drift after platform-API or
trait-surface changes" — is already served, and better, by two things that did
not exist in phase 121:

- `just check rmw-xrce` compiles this project's C (issue 0787) and runs its two
  CTests; since phase-420 W9 step 4 those link the cargo lane's archive, so they
  exercise what images ship;
- `nros-rmw-xrce-cffi` is the actual Rust crate and is built by
  `cargo build -p nros-rmw-xrce-cffi`, which that same recipe now runs.

If instead someone wants the Rust crate `cargo check`ed by name, the recipe
should say `cargo check -p nros-rmw-xrce-cffi` from the workspace root and drop
the `cd`.

## Class

Third in a small family this directory keeps producing: a statement about the
build that nothing verifies. The other two were the source-list mirror
(issue 1068) and the version restatement (issue 1069), both of which had a
correct-looking comment asserting an invariant that had stopped holding. This
one is a whole recipe in that shape.
