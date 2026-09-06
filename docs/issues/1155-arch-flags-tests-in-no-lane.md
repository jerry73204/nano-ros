---
id: 1155
title: "`nros-board-common`'s `arch_flags` tests run in no lane — `test-unit` activates no features and the module is behind `build-helpers`"
status: open
type: bug
area: testing, build
severity: low
found: 2026-09-06
related: [0652, phase-418]
---

# Six tests that only run when someone asks for them by name

Measured on this checkout:

```
$ cargo nextest list -p nros-board-common                      | grep -c arch_flags
0
$ cargo nextest list -p nros-board-common --features build-helpers | grep -c arch_flags
6
```

`nros-board-common` is `#![cfg_attr(not(feature = "build-helpers"), no_std)]`,
and `arch_flags` is part of the host-only set behind that feature. `just
test-unit` runs `cargo nextest run --workspace …`, and its own comment records
the reason this matters:

> `cargo nextest run --workspace` activates no features.

It says so about `nros-rmw-*-cffi`, which are excluded and covered elsewhere by
per-feature invocations. `nros-board-common` is neither excluded nor covered
that way, so its feature-gated tests are simply absent from every lane.

## What is in them

`arch_flags` is the resolver `[arch.*]` profiles go through —
`cflags_for_target`, `arch_matches`, and the `target_match` / `target_exclude`
substring rules. phase-418 item 418.3 added three tests there pinning that
`armv7r-none-eabi` resolves to the Cortex-R5 profile and `armv7r-none-eabihf`
resolves to nothing, which is the ABI-mismatch guard for the SPE work. They
pass, and nothing runs them.

The load-bearing coverage today is `check-arch-profile-resolution`, a gate over
the manifests. That is a different assertion — it checks the DATA is coherent,
not that the resolver reads it the way the tests say.

## Shape of a fix

Mirror what the cffi crates already do: a per-feature invocation, so
`nros-board-common --features build-helpers` runs somewhere on the unit lane.
Worth checking at the same time whether any OTHER crate in the workspace has
feature-gated tests in the same position — the answer to "which crates have
tests that `--workspace` with no features cannot see" is not recorded anywhere,
and issue 0652 found four such targets the last time someone asked.
