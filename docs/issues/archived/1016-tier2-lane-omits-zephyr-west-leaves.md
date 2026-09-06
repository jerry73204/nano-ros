---
id: 1016
title: "`lane=tier2` does not build the zephyr rust/c west leaves, so their cells
  report as failures that are really skips"
status: resolved
type: bug
area: testing, zephyr, ci
severity: medium
related: [issue-0968, issue-0828, issue-0445]
found: 2026-09-03
---

## Measured

After a clean `just build-test-fixtures lane=tier2` (module verdict
`== zephyr == OK`), running the nine `example_e2e` zephyr XRCE cells:

```
Summary [295.800s] 9 tests run: 0 passed, 9 failed, 45 skipped
```

Six of those nine had not run at all. They panicked in `resolve_example`
(`zephyr.rs:143`) with:

```
[SKIPPED] zephyr/c/listener xrce image not prebuilt or stale …
  BuildFailed("Zephyr fixture is STALE — a source is newer than the built binary:
    binary: …/build-c-listener-xrce/zephyr/zephyr.exe
    newer:  …/examples/zephyr/c/listener
    probe:  examined 0 input(s); …
    NOT RUN: 3th consecutive stale verdict for this fixture")
```

Bare `cargo nextest` counts a `nros_tests::skip!` panic as a FAILURE (CLAUDE.md
says so), so the summary line for "six were never built" is character-for-
character the same as the summary line for "six ran and failed".

`just build zephyr` (the west-leaf build) does cover them; after it, the same
nine ran with **zero** skips.

## Why it matters more than an inconvenient skip

This is the tier-2 half of issue 0828's class. That one was the build omitting
rows the run would NOT skip, so a stale fixture passed the freshness gate. This
is the run reaching for rows the tier-2 build never made, and the resulting
message being indistinguishable from a result.

It cost a full wrong reading in issue 0968: the six were reported as failures,
reasoned about as failures, and only the `zephyr.rs:143` frame in the panic text
distinguished them. A reader who trusts the `9 failed` count — which is what a
CI summary shows — gets six phantom results.

Note also `probe: examined 0 input(s)` with the leaf DIRECTORY as the "newer"
input. Whether that probe is right is a separate question (issue 0445's family),
but a probe that examines nothing and still returns STALE is worth a look while
here.

## Two candidate fixes, and they are not equivalent

1. **Make the lane build what the lane runs** — the phase-340 W3 property, that
   build-set and run-set are one predicate on one coordinate file. If the west
   leaves cannot be attributed to a tier-2 coordinate they should be in the run
   set at every lane (the `row_artifact_root()` fail-closed rule), which is what
   already happens for other unattributable rows.
2. **Make an unbuilt fixture distinguishable from a failed one** at the summary
   level, not only in the panic text.

Both are worth doing; (2) is what stops the next wrong reading even when (1)
regresses.

## Acceptance

* [ ] `lane=tier2` either builds the zephyr rust/c west leaves or the run does
      not attempt them.
* [ ] A run whose cells were never built cannot report a count identical to one
      whose cells ran and failed.

## Resolved 2026-09-06 — and the title is half wrong, which is worth saying first

`lane=tier2` **does** build the zephyr west leaves its cover names, and the run
**does** skip the ones it does not. Both halves were measured on this tree:

```text
$ ./target/debug/lane-coords tier2 | grep zephyr
zephyr,cpp,xrce
zephyr-cortex-m,c,zenoh

$ fixtures-manifest.py west-leaves --coords-from <tier2 coords>   # 7 of 71
build-cpp-{talker,listener,service-server,service-client,action-server,action-client}-xrce
build-cortex-m-c-talker-zenoh
```

So the lane's zephyr coordinate is `zephyr,cpp,xrce`. The six cells this issue
watched go STALE — `zephyr/{rust,c}/*` at `xrce` — are **out of the lane by
design**, and the three that ran are the cpp ones, which is exactly the trio
issue 0968 still lists (`case_{21,24,27}_xrce_cpp_*`). The lane and the run
agree about all nine.

What did not agree was the RUN this issue measured with. The narrowing lives in
`NROS_TEST_COORDS`, and the sanctioned harness REFUSES the pairing this issue
performed:

```text
$ NROS_FIXTURE_STAMP=<lane=tier2 stamp> nros_fixtures_stamp_require tier2   # no NROS_TEST_COORDS
ERROR: lane 'tier2' is coordinate-scoped, but NROS_TEST_COORDS is unset
       or empty — so the RUN would resolve every coordinate while this
       preflight accepts a build of only tier2's.
EXIT=1
```

A bare `cargo nextest -E '…'` goes around that preflight, and CLAUDE.md already
records the second half of what happened next: bare nextest counts a
`nros_tests::skip!` panic as a FAILURE. `0 passed, 9 failed` is what those two
facts produce together, and neither is a lane defect.

## The class defect that IS here, found by auditing the same code

The west table is **entirely outside** the issue-0828 invariant, and its
skippability rests on a lookup that fails open.

* `row_artifact_root()` returns `""` for a `builder = "west"` row (west writes
  into the Zephyr build root, not under the row's `dir`), so
  `row_is_lane_skippable()` calls **every** west row skippable and
  `lane_build_covers_run::every_unskippable_row_is_in_its_lane_build` has
  nothing to compare. 58 rows + 13 workspace entries, unchecked.
* What actually enforces the skip is
  `fixtures::lane::require_west_leaf_in_lane(build_name, …)`, which looks the
  leaf up **by build-dir NAME** in `west-leaves` and, on a name it cannot find,
  returns `Ok` — "run it". Correct as a default, and a hole with no floor: a
  name the manifest does not model is never emitted by `west-leaves` (so no
  lane builds it) and never skippable by the run (so every lane demands it).
  That is 0828's shape, and its verdict is the STALE message this issue is
  about.
* **18 such names existed**, measured: the `zephyr-dds-*-a9` arms of
  `decode_alias` → `build-{rust,c,cpp}-{6 roles}-cyclonedds-a9`. No test named
  one, no `[[fixture]]` row modelled one, no recipe built one. They were latent
  only because nothing called them.
* And the leaf's COORDINATE was derived twice — `row_coord()` on the build side,
  a board-and-label reconstruction (`mps2_an385` → `zephyr-cortex-m`, `default`
  → `zenoh`) on the run side. They agreed when measured, and nothing held them
  there. This is CLAUDE.md's issue-1025 rule one table over: one formula is not
  enough when the INPUTS are derived twice.

## What changed

1. **`west-leaves` emits the row's `row_coord()`** as a 16th column;
   `fixtures::lane::parse_west_leaf` READS it. The board/label reconstruction is
   deleted. `zephyr-fixture-leaves.sh` names the new column so `read` cannot let
   it be absorbed by `nros_image` (measured: `image=''  coord=zephyr,c,xrce`).
2. **The 18 `-a9` alias arms are deleted.**
3. **`scripts/check-west-leaf-vocabulary.py`** (fast line, `just check
   west-leaf-vocabulary`): every west build-dir name the harness can produce —
   `decode_alias` arms plus every literal/const handed to
   `require_west_leaf_in_lane` — must be modelled by `west-leaves`. It refuses
   to run vacuously (a harvest below 30 names, or zero call sites, is a hard
   error, since a regex that stopped matching would otherwise report a clean
   tree forever) and carries a `--selftest`.
4. **`lane_build_covers_run::every_west_leaf_the_run_can_name_is_built_or_skippable`**
   — the 0828 invariant, stated for the west table, by MEMBERSHIP and by name.
   It also asserts each lane actually narrows (`built < modelled`), so it cannot
   pass for the uninteresting reason that everything was built.

## Measurement, both directions

| check | mutation | result |
| --- | --- | --- |
| `check-west-leaf-vocabulary` | re-add one `-a9` arm | FAIL, naming `build-c-talker-cyclonedds-a9`; restore → `56 resolver build-dir name(s), all modelled by 71 west leaves` |
| `lane::tests::a_west_leaf_takes_its_coordinate_from_the_manifest_not_from_its_board` | revert `parse_west_leaf` to the board-derived form | FAIL: `left: ("zephyr-cortex-m","rust","zenoh")  right: ("zephyr","rust","xrce")`; restore → 11 passed |
| `every_west_leaf_the_run_can_name_is_built_or_skippable` | re-add one `-a9` arm | FAIL for tier1/tier2/tier2-nightly, naming the leaf; restore → ok |

`cargo test -p nros-tests --test lane_build_covers_run -- --test-threads=1`:
9 passed / 1 failed BEFORE these changes and 10 passed / 1 failed after — the
same one, `a_native_build_satisfies_the_tier1_run`, which fails identically on a
pristine `origin/main` worktree and is **not** touched by anything here. (Run in
parallel, that file also loses two more to interference: several cases share one
`.fixtures-built-tier2` stamp path under `tmp/`. Also pre-existing.)

No fixture was built for this: every measurement above is the manifest, the
lane selector and the shell, and the acceptance for a lane's CONTENT is still a
build, not a gate.

## Acceptance

* [x] `lane=tier2` either builds the zephyr rust/c west leaves or the run does
      not attempt them — it does not attempt them, measured above, and that
      property is now checked by name for every lane rather than resting on a
      fail-open lookup.
* [~] A run whose cells were never built cannot report a count identical to one
      whose cells ran and failed. **Under the harness this already holds**:
      `just test-all`'s junit rewrite reports a `skip!` as skipped, and an
      out-of-lane leaf panics through `skip_class!(lane, …)` naming its
      coordinate. Under a bare `cargo nextest` it cannot — nextest has no skip
      verdict — which is CLAUDE.md's documented "bare `cargo nextest` counts
      `nros_tests::skip!` panics as FAILURES", not a property of this repo's
      lane machinery. Nothing is changed for that case, deliberately.

The `probe: examined 0 input(s)` note in the report above is untouched and
belongs to issue 0445's family.
