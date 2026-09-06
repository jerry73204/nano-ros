---
id: 1124
title: "Twelve tests turn a fixture-resolver `Err` into a `skip!`, so a lane gap reports as coverage"
status: open
area: testing
severity: medium
related: [0584, 1112, 0196]
---

## What

Twelve sites across five test files answer a failed fixture resolution with
`nros_tests::skip!` instead of failing:

```rust
Err(e) => nros_tests::skip!("<name> fixture not prebuilt ({e})"),
```

| file | sites |
| --- | ---: |
| `declarative_bridge_zenoh_to_cyclonedds.rs` | 4 |
| `bridge_zenoh_to_cyclonedds.rs` | 3 |
| `declarative_bridge_zenoh_to_xrce.rs` | 3 |
| `bridge_mixed_rmw.rs` | 1 |
| `zephyr.rs` | 1 |

Issue 0584's rule: an absent **in-lane** fixture is a hard failure, not a skip —
a gated run has already asserted the lane's fixtures are built, so "not prebuilt"
means the LANE is wrong. Reporting that as a skip reports a gap as coverage.

## Why it is filed rather than swept

Issue 1112 fixed the thirteenth site (`esp32_emulator.rs`, `int32-sink`) and
deliberately stopped there. The rule is about an **in-lane** fixture, and a
bridge test whose fixture is genuinely out of the current lane SHOULD skip.
Converting all twelve blind would turn correct skips into hard failures across
several lanes — the 0196 shape, a gate wider than its rule.

What made the esp32 one answerable was independent evidence: `_check-skip-budget`
named it on a lane that had already asserted its fixtures were built, and the
same file's two sibling tests already `.expect()`ed their peers. Each of the
twelve wants that same evidence.

## What answering it looks like, per site

1. Which lane(s) run this test? (`matrix::CELLS` / `interop::CELLS`, and the
   `NROS_TEST_COORDS` narrowing.)
2. Does the fixture have a manifest row in that lane's build set?
   `nros_fixture_row_artifact_dir_by_id` and `fixtures-manifest.py coords` answer
   it; issue 1112's fix is the worked example.
3. **In lane** ⇒ `.expect(...)`, and add the row to the lane's build if it is
   missing. **Out of lane** ⇒ the skip is correct; say so in a comment naming the
   lane, so the next reader does not re-litigate it.

The likely split is not uniform: the bridge tests need a PEER built for another
RMW, which is exactly the case a lane may legitimately exclude.

## Why it matters

A skip is invisible in a green run. Issue 1112's site hid a real lane gap for as
long as it existed, while `_check-skip-budget` complained on every run and the
complaint had nowhere to land — it can see that something laundered an `Err`, but
not which site did it.
