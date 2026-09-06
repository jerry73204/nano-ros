---
id: 1160
title: "37 precondition-guard sites outside `nros-tests/tests/` were never swept, and `check-test-precondition-guards` does not reach them"
status: resolved
type: tech-debt
area: testing
severity: medium
found: 2026-09-06
resolved: 2026-09-06
related: [0693, 1135]
---

# The sweep had a boundary, and the boundary was the scope of one issue

Issue 1135 fixed four `params.rs` tests that reported PASS on an unmet
precondition, then swept `packages/testing/nros-tests/tests/` for the same
shape: 52 raw early-return lines across 16 files, of which seven helpers were
the defect and two were **live** — `cli_bringup_nuttx.rs` (green on a host with
NuttX and no built CLI) and `staticlib_duplicate_symbols.rs` (no `nm` ⇒ both
symbol assertions skipped, PASS, in a test *named* for the symbol shape).

That sweep stopped at one directory because that is where the reported bug was.
37 sites of the same shape lived outside it, unreviewed. This issue read all of
them.

## Outcome

**Every site classified. Four defects fixed, in three files. Two were live.**

The measurement of "37" was of a shape, and reading them turned it into 49
`return` statements across the nine files — the extra 12 are `return`s inside
walk/parse helpers and closures, which 1135's own sweep had already found to be
the dominant legitimate form. The full per-site classification is in the commit
message for `fix(#1160)`; the summary:

| file | sites | verdict |
| --- | --- | --- |
| `rosidl-codegen/tests/parity_test.rs` | 18 | **fixed** — 9 two-guard pairs → 9 one-line `ros_input`/`ros_input_dir` |
| `rosidl-codegen/tests/comparison_test.rs` | 3 | **fixed (LIVE)** — a parse failure was reported as `[NO-ROS]` + PASS |
| `rosidl-codegen/tests/compilation_test.rs` | 4 | **fixed** — a dead `cargo_available()` guard, plus a clippy probe that was wrong twice |
| `rosidl-codegen/tests/parity_helpers.rs` | 5 | left (probe internals) + the new seam lives here |
| `rosidl-codegen/tests/codegen_golden.rs` | 6 | left — a record-mode exit and a success-path exit |
| `cargo-nano-ros/tests/integration_tests.rs` | 4 | left — `AMENT_PREFIX_PATH` is a clean environment fact, documented under 0693 |
| `nros-cli-core/tests/build_verb_pipeline.rs` | 2 | left — each `return` follows an `assert!` pinning the other branch |
| `rosidl-bindgen/tests/edition_hash_oracle.rs` | 2 | left — optional Tier-B1 docker oracle, uncconflated |
| `nros-rmw-cyclonedds/tests/bare_metal_link.rs` | 5 | **fixed (LIVE)** — `nm` absent switched off the audit and reported PASS |

### The two live ones

* **`comparison_test.rs` ×3.** `read_and_parse_ros_message` returned
  `Result<_, String>` and folded three states into one `Err`: no ROS install,
  the file unreadable, and **our parser rejected it**. All three answered
  `note_no_ros(&format!("comparison_test ({e})")); return Ok(())`. A
  `parse_message` failure on `std_msgs/Bool.msg` is the loudest signal
  `rosidl-parser` can produce, and it arrived as a green with `[NO-ROS]` in the
  caption — the `staticlib_duplicate_symbols` lie in the family where it is
  least visible.
* **`bare_metal_link.rs::bare_metal_no_alloc_symbols`.** `nm` absent ⇒
  `eprintln!("[SKIPPED] …"); return` ⇒ PASS, with the alloc-symbol audit (the
  whole substance of the test, and its name) not run. Measured on a PATH with
  cargo and rustup and no `nm`: `1 passed` before, `FAILED` after.

### And the one that reads as a mitigation but is not

`note_no_ros`'s `eprintln!` — 0693's answer to "there is no runtime skip in
`packages/cli`" — is **invisible on the lane that runs these suites**. libtest
captures the output of a PASSING test, and `check-cli-tests` runs
`cargo test --manifest-path packages/cli/Cargo.toml --workspace --quiet`. The
marker is worth keeping for a human with `--nocapture`; it is not a safety net,
and the doc comments now say so. What replaces it is structural: every state
that is not literally "this host has no ROS 2 install" now FAILS.

## The gate did not need widening — it was never narrow

This issue proposed widening `check-test-precondition-guards` from
`packages/testing/nros-tests/tests/` to `packages/**/tests/`, and asked for the
measurement first. The measurement says there is nothing to widen:
`tracked_test_files()` globs `*/tests/*.rs` across the repository and reads
**292 files in 34 directories, 118 of them outside `packages/testing/`**, green
with no allowlist. A `require_planted_probe() -> bool` dropped into
`packages/cli/rosidl-codegen/tests/parity_test.rs` and into
`packages/rmw/cyclonedds/…/tests/bare_metal_link.rs` was rejected in both
places. The issue's premise came from the docstring's prose ("the test-local
guard that composes them, which is what this gate scopes to"), which is about
guards, not directories.

What DID land is a coverage floor, because "OK (0 test files)" and "OK, all 169
of them under nros-tests" are exactly what a gate that has silently narrowed
would print: `assert_scope_is_the_whole_tree()` fails when the scan stops
reaching test files outside `packages/testing/`, and the OK line now reports the
directory spread instead of a bare count. Proven by narrowing the glob and
watching it go red.

The deliberate exclusion stands: library probes under `nros-tests/src/` are
composed and each call site writes its own message, so a `bool` return is
correct there.

## Adjacent, not fixed here

Two things this sweep walked past on purpose, both a different class:

* `parity_test.rs`'s three `test_parse_all_*` collect parse/generate failures
  and then print them under `// Don't panic - just report the failures`. That is
  a test reporting PASS over MEASURED failures — not a precondition guard, and
  turning it red needs a decision about which parser limitations are acceptable.
* `compilation_test.rs` and `bare_metal_link.rs` run `cargo check` / `cargo
  build` at test runtime, which CLAUDE.md forbids ("No compilation inside
  tests"). Pre-existing, and a fixture-stage migration, not a guard fix.
