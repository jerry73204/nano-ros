---
id: 1160
title: "37 precondition-guard sites outside `nros-tests/tests/` were never swept, and `check-test-precondition-guards` does not reach them"
status: open
type: tech-debt
area: testing
severity: medium
found: 2026-09-06
related: [1135]
---

# The sweep had a boundary, and the boundary was the scope of one issue

Issue 1135 fixed four `params.rs` tests that reported PASS on an unmet
precondition, then swept `packages/testing/nros-tests/tests/` for the same
shape: 52 raw early-return lines across 16 files, of which seven helpers were
the defect and two were **live** — `cli_bringup_nuttx.rs` (green on a host with
NuttX and no built CLI) and `staticlib_duplicate_symbols.rs` (no `nm` ⇒ both
symbol assertions skipped, PASS, in a test *named* for the symbol shape).

That sweep stopped at one directory because that is where the reported bug was.
**37 sites of the same shape live outside it and are unreviewed:**

| file | sites |
| --- | --- |
| `packages/cli/rosidl-codegen/tests/parity_test.rs` | 18 |
| `packages/cli/rosidl-codegen/tests/compilation_test.rs` | 4 |
| `packages/cli/rosidl-codegen/tests/comparison_test.rs` | 3 |
| `packages/cli/rosidl-codegen/tests/codegen_golden.rs` | 2 |
| `packages/cli/rosidl-codegen/tests/parity_helpers.rs` | 1 |
| `packages/cli/cargo-nano-ros/tests/integration_tests.rs` | 3 |
| `packages/cli/nros-cli-core/tests/build_verb_pipeline.rs` | 2 |
| `packages/cli/rosidl-bindgen/tests/edition_hash_oracle.rs` | 1 |
| `packages/rmw/cyclonedds/nros-rmw-cyclonedds/tests/bare_metal_link.rs` | 3 |

These are counts of the SHAPE, not of confirmed defects — 1135's own sweep found
that most raw early returns are legitimate (a success-path exit, a return inside
a walk helper, a returned value the caller needs). Someone has to read each one.

The codegen parity tests are the ones to read first: 18 of the 37, and their
subject is whether generated code matches upstream — a family where "the tool
was not available so we did not compare" reporting PASS is the same lie 1135
found in `staticlib_duplicate_symbols.rs`.

## The gate does not reach them either

`check-test-precondition-guards` (added with 1135) rejects a
`require_*`/`ensure_*`/`need_*`/`maybe_*` helper in a test file that returns
`bool` or `Option<()>`. Its scope is `packages/testing/nros-tests/tests/`.
Widening it to `packages/**/tests/` is the obvious move and should be measured
first: the rule needs no allowlist in its current scope, and that property is
what makes it worth having.

Note the deliberate exclusion that should stay: library probes under
`nros-tests/src/` are composed, and each call site writes its own message, so a
`bool` return is correct there.

## Why the shape matters more than the count

A guard that returns early reports PASS. `check-no-vacuous-tests` cannot see it
— that gate keys on a body whose only effects are prints, and these bodies do
plenty on the passing path. So the failure is invisible in exactly the case it
is supposed to report: the environment is fine and the thing under test is not.
