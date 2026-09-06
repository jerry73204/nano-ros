---
id: 1176
title: "three `parity_test.rs` tests MEASURE parse failures and then pass — and the report they print is captured by libtest, so nobody sees it"
status: open
type: bug
area: testing, codegen
severity: high
found: 2026-09-06
related: [0693, 1135, 1160]
---

# Not an unmet precondition — a measured failure, reported as success

`packages/cli/rosidl-codegen/tests/parity_test.rs`, three tests
(`test_parse_all_*`, at lines 254, 311 and 368). Each walks a ROS share root,
parses every `.msg`/`.srv`/`.action`, collects what failed — and then:

```rust
if !failures.is_empty() {
    eprintln!(
        "Failed to process {} out of {} std_msgs ({}% success rate):",
        failures.len(), count, (count - failures.len()) * 100 / count
    );
    for failure in &failures {
        eprintln!("  {}", failure);
    }
    // Don't panic - just report the failures
    eprintln!("Note: Some failures expected due to parser limitations (default values, etc.)");
}
```

This is a different and worse class than issues 1135 and 1160, which were both
*unmet preconditions* reported as PASS. Here the precondition was MET, the work
ran, the failures are real and counted, and the verdict is still green.

## And the report is invisible on the lane that runs it

libtest captures the output of a **passing** test, and `check-cli-tests` runs
`cargo test --manifest-path packages/cli/Cargo.toml --workspace --quiet` — no
`--nocapture`. So the `eprintln!` block reaches nobody. Measured on a ROS-less
host during issue 1160's sweep: `parity_test` reported `16 passed` in 0.00 s
with no `[NO-ROS]` or failure line anywhere in the output.

Same root shape as the mitigation issue 0693 recorded for these suites — an
`eprintln!` standing in for a skip — and the same reason it does not work.

## Why the comment is not a defence

"Some failures expected due to parser limitations (default values, etc.)" may
well be true. But an expected-failure set that is neither enumerated nor bounded
is indistinguishable from a regression:

* a parser change that breaks 40 more messages passes identically;
* a parser change that FIXES all of them also passes identically, so nobody
  learns the limitation is gone;
* the percentage in the message is computed and then discarded.

## Fix — the shape, not the threshold

Make the tolerated set explicit, so a change to it is a diff someone reviews:

1. **An enumerated expected-failure list** (per message path, with the reason —
   "default values unsupported"), asserted exactly: an unexpected failure fails
   the test, and an entry that now PARSES also fails, so the list cannot rot.
   That is the ratchet idiom this repo already uses elsewhere.
2. Failing that, a **bounded** assertion — `failures.len() <= N` with `N`
   committed and a comment naming what is in it. Weaker, because it cannot tell
   you the set changed identity at constant size, but still a verdict.

Prefer (1). Either way the numbers belong in an `assert!` message, where a
failing test prints them, rather than in an `eprintln!` on the passing path.

## The adjacent item, recorded separately

The same files (`compilation_test.rs`, and `bare_metal_link.rs` in
`nros-rmw-cyclonedds`) **compile at test runtime**, against CLAUDE.md's "No
compilation inside tests — compile in the build stage, the test consumes the
prebuilt fixture". That is its own class and its own decision; noted here only
so the next reader of these files is not surprised by it.

## Provenance

Found by issue 1160's sweep of precondition guards in `packages/*/tests/`,
which fixed four real PASS-on-failure defects and deliberately left this one as
a different class.
