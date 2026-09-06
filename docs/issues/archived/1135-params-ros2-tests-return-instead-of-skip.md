---
id: 1135
title: "four `params.rs` ROS 2 tests report PASS when the nano-ros node is not discoverable — a bare `return`, not `skip!`"
status: resolved
type: bug
area: testing, rmw
severity: high
found: 2026-09-06
related: [1127, 0445, 0584, 1129, 0196]
---

# The precondition they exist to check is the one they swallow

`packages/testing/nros-tests/tests/params.rs` — `test_ros2_param_list`,
`test_ros2_param_get`, `test_ros2_param_set`, `test_ros2_param_describe`, at
lines 189, 232, 276 and 323 — each open with:

```rust
if !require_node_discoverable(&locator) {
    return;
}
```

A bare `return` from a `#[test]` is a **PASS**. `require_node_discoverable`
polls `ros2 node list` three times for `/demo/talker` and prints
`"Skipping test: nros node /demo/talker not discoverable…"` on failure — but
printing is all it does, so nextest records four green tests.

CLAUDE.md's rule is explicit: *tests must fail on unmet preconditions;* bare
`eprintln!` + `return` reports PASS — never. The correct spelling
(`nros_tests::skip!`) is already used in this same file, at line 514.

## Why this one is worse than the general class

The swallowed condition is not an absent toolchain. It is **"ROS 2 is up, the
router is up, the fixture is built, and our node is not in the graph"** — which
is a *delivery failure*, the exact thing these four tests exist to detect. On
any host where the feature is broken but the environment is fine, all four go
green.

Under `--failure-output never` (what `just native test-ros2-params` passes) the
`eprintln!` is not even shown, so the run is silent as well as green.

`check-no-vacuous-tests` cannot see it: that gate keys on a test body whose only
effects are PRINTS, and these bodies do plenty on the passing path. The gate is
correct; the shape is different.

## Fix (PR #613)

**The second option, at the source.** `require_node_discoverable` returns `()`
and its failure path is `nros_tests::skip_class!(resource, …)`; the four call
sites are a bare `require_node_discoverable(&locator);`. A `require_*` that
hands back a `bool` hands the caller the verdict, and four callers dropped it
identically — with no value to inspect there is nothing for a fifth call site to
drop.

`skip_class!(resource, …)` rather than plain `skip!`: plain reads as
`capability` ("this HOST cannot run it"), which is FALSE here. Every cheap
precondition has already passed, so the host can run this test — a peer just
never appeared. `resource` is the class for that, and it makes the count
readable in a sweep summary.

## The sweep

Scope `packages/testing/nros-tests/tests/` — 52 raw early-return lines across 12
files. Two commands, both re-runnable:

```sh
# every guard-shaped helper that hands back a verdict rather than keeping it
grep -rn --include='*.rs' -E \
  '^\s*fn [a-z_0-9]*(require|ensure|need|maybe)[a-z_0-9]*\s*\([^)]*\)\s*->\s*(bool|Option<\(\)>)' \
  packages/ --exclude-dir=third-party | grep '/tests/'
# every bare early return at a test body's own level (closures/helpers excluded)
python3 scripts/check-test-precondition-guards.py
```

### Fixed — the same defect

| Site | Live? | What it did |
| --- | --- | --- |
| `params.rs` ×4 | yes | the reported bug |
| `cli_bringup_nuttx.rs` | yes | `require_nuttx_setup` skipped on NuttX and arm-gcc but `return None`d on a missing `nros` CLI, and the caller turned that into a bare `return`. Green on a host with NuttX and no built CLI |
| `staticlib_duplicate_symbols.rs` | yes, partial | no `nm` on PATH turned off BOTH symbol assertions and returned. The test is NAMED for the symbol shape, so the green claimed coverage nobody measured |
| `native_api.rs` ×19 (`require_native_env`) | no — dead branch | both arms already `skip!`, so `false` is unreachable. Kept as the live defect's SHAPE and a standing invitation to add a third arm that returns `false` |
| `rtos_e2e.rs` ×3 (`maybe_skip` → `require_cell_runnable`) | no — dead branch | `skip_reason` returns `None` for every combination today. Its doc claimed the printed `[SKIP]` + `return` was "equivalent to the `#[ignore]` attribute" — it is not: `#[ignore]` is not a pass |
| `esp32_emulator.rs`, `freertos_qemu.rs`, `threadx_riscv64_qemu.rs` | no — wrong message | callers skipped CORRECTLY but with `"<guard> check failed"`, while the real reason was `eprintln!`ed into the stream `--failure-output never` discards. Same invisibility as this issue, one step short of the same PASS |
| `exec_depend_drift_check.rs`, `workspace_lints_check.rs` | no | `-> Option<()>` shims (a bool wearing a hat) over `nros_tests::require_nros_cli` |

### Left alone — legitimate control flow

* `actions.rs:46`, `services.rs:80` — the `return` fires when the readiness
  marker IS found; the not-found path below skips (process alive) or panics
  (process exited). A success-path exit, not a swallowed precondition.
* `cmake_platform_matrix.rs` ×3 — the `return`s are `require_codegen_or_skip`'s
  success path; its tail is `skip!`.
* `realtime_tiers_e2e.rs:697,722` and `rtos_e2e.rs`'s `ensure_ready` — end of a
  branch that has already asserted.
* `example_shape.rs` ×8, `cpp_api_drift.rs`, `examples_fixture_coverage.rs`,
  `legacy_files_forbidden.rs`, `no_local_axis_tables.rs`,
  `zephyr_prjconf_requirements.rs` ×2 — `return` inside a walk/parse helper or a
  filtering closure, never a test body.
* `cyclonedds_descriptors.rs` — `require_preconditions() -> Option<(PathBuf,
  PathBuf)>` returns a VALUE the caller needs; `let … else { skip!() }` is the
  correct spelling and stays.

### Not fixed here

* `matrix_fixture_coverage.rs:243,373` — `let Ok(justfile) = … else { return; }`
  ("packaged crate — not a failure") is the same defect: a probe-driven PASS.
  Another session owned that file during phase-433 W2.
* **37 further sites** carry a bare early return at a test body's own level
  outside this directory: `packages/cli/rosidl-codegen/tests/parity_test.rs` ×18,
  `compilation_test.rs` ×4, `comparison_test.rs` ×3, `codegen_golden.rs` ×2,
  `parity_helpers.rs`; `packages/cli/cargo-nano-ros/tests/integration_tests.rs`
  ×3; `packages/cli/nros-cli-core/tests/build_verb_pipeline.rs` ×2;
  `rosidl-bindgen/tests/edition_hash_oracle.rs`; and
  `packages/rmw/cyclonedds/nros-rmw-cyclonedds/tests/bare_metal_link.rs` ×3.
  Unreviewed — each needs the same defect-vs-control-flow call made by reading.

## The class question — answered: gate the SIGNATURE, not the return

`check-test-precondition-guards`
(`scripts/check-test-precondition-guards.py`, recipe in `just/check/gates.just`,
fast lane, self-tests its classifier on every run, no allowlist).

**Rule:** a `require_*` / `ensure_*` / `need_*` / `maybe_*` helper defined in a
test file must not return `bool` or `Option<()>`.

The obvious rule — "a `#[test]` that returns early on a probe result" — is not
statically decidable, because the defect and its legitimate twin are the SAME
SYNTAX:

```rust
if !require_node_discoverable(&locator) { return; }              // defect
if server.wait_for_output_pattern(MARKER, T).is_ok() { return; } // fine
```

Only the meaning of `<cond>` separates them. The decidable substitute — "no bare
`return;` in a test body" — is a different rule: measured across the tree it
flags 40 sites, most of them the legitimate success-path form, so landing it
buys an authored 40-entry allowlist. That is the map shape this repo already
records drifting (the rmw parity map read `("gap", "no vtable slot")` for 28
slots W4 had landed). A prototype of it also mis-attributed a `return` in a
HELPER to a test on its first run, because the helper's new doc comment contains
the literal `#[test]` — evidence that the return shape wants a parser, not a
line scanner.

The signature is both decidable and load-bearing: every one of the seven helpers
in the sweep above would have been caught, and the fixed tree needs no
exceptions. Deliberately out of scope are the library probes in
`nros-tests/src/` (`require_zenohd`, `require_ros2`, `is_*_available`) — they
are composed, and each call site writes its own message, so the verdict belongs
to the test-local guard that composes them.

`check-no-vacuous-tests`'s docstring now carries a "the sibling shape this gate
does NOT catch" section pointing at it, so the next reader asking why the
print-only gate misses the return form finds the argument rather than guessing.

## Also corrected in the same PR (phase-433 W2 pre-flight, each verified first)

* `params.rs` hardcoded `"humble"` in **7** places where
  `nros_tests::ros2::DEFAULT_ROS_DISTRO` is meant.
* `xrce_ros2_interop.rs`'s header said the tests "are diagnostic/informational —
  they report interop status but do not hard-fail the test suite". Untrue: all
  seven cases end in an `assert!` and the only non-failing exits are explicit
  `skip!`s. A stale note like that invites the next reader to dismiss a red there
  as noise — the same false-PASS reading from the other end.
* Its `test_ros2_action_xrce_client` skip blamed `action_tutorials_py`. The peer
  is `Ros2DdsProcess::action_server_fibonacci_with_domain`, an inline rclpy
  script whose only import is `example_interfaces.action.Fibonacci` — chosen,
  per that helper's own doc comment, *precisely so the cell does not depend on
  `action_tutorials_py`*. The old message sent the reader to install something
  that would not have helped.

## Verification

`cargo nextest run -p nros-tests --test params` on a host with no ROS: 8
`[SKIPPED:capability]`, never a pass. The four fixed tests skip at the `zenohd`
guard BEFORE reaching the discovery probe, so the new skip cannot be exercised
there — the structural proof is that the helper no longer returns a value a
caller could drop.

`just check fast` green (240 gates). Clippy `-D warnings` and
`cargo +nightly fmt --check` clean on `nros-tests`.

## How it was found

The phase-433 W2 pre-flight (PR #592), which asked of every live-peer test
binary "would a skip be visible?" before spending
box time running them. `params` was the one binary in the set that could not
produce a verdict — found by reading, not by any gate, which is why the durable
half of this issue is the gate above.
