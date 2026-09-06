---
id: 1124
title: "Twelve tests turn a fixture-resolver `Err` into a `skip!`, so a lane gap reports as coverage"
status: resolved
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

## Answered — all twelve, and the split is NOT the one this issue predicted

The prediction above was "a bridge test whose fixture is genuinely out of the
current lane SHOULD skip". Measured, that case cannot reach these sites at all,
so all twelve convert and none is left as a correct skip.

### The measurement that settles it

**Out-of-lane never returns `Err`.** `require_coord_in_lane` (cargo rows, keyed
on `row.coord`), `require_in_lane` (path-attributed rows) and
`require_workspace_in_lane` (workspace rows, keyed on `id`) run INSIDE the
resolver and BEFORE the existence check, and they PANIC `[SKIPPED:lane]` rather
than returning:

```
packages/testing/nros-tests/src/fixtures/binaries/mod.rs:517  require_prebuilt_row_binary
packages/testing/nros-tests/src/fixtures/binaries/mod.rs:530  require_prebuilt_binary
packages/testing/nros-tests/src/fixtures/binaries/mod.rs:1890 require_prebuilt_workspace_binary
packages/testing/nros-tests/src/lib.rs:123                    skip_class! => panic!("[SKIPPED:{}] …")
```

So the `Err` these twelve sites caught was never "the lane deliberately did not
build it". It is a missing, stale, or failed-to-build IN-LANE fixture — 0584's
hard failure — and `check-skip-budget`'s `FIXTURE_RE` (`not prebuilt`) already
fails the whole run for any of them. Converting does not create a red; it moves
an existing, site-less red onto the site.

### Coordinates and lane build sets (measured)

`cargo run -p nros-tests --bin lane-coords -- <lane>` for the coordinate sets;
`fixtures-manifest.py {coords,list,list-workspaces} --coords-from <file>` for the
build sets.

| fixture (manifest row) | coord | tier1 | tier2 | nightly |
| --- | --- | :-: | :-: | :-: |
| `header-chatter-talker` | `linux,rust,zenoh` | build | build | build |
| `int32-sink-zenoh` | `linux,rust,zenoh` | build | build | build |
| `int32-sink-xrce` | `linux,rust,xrce` | build | not selected | build |
| `int32-sink-cyclonedds` | `linux,rust,cyclonedds` | build | not selected | build |
| `bins/bridge-zenoh-to-cyclonedds-fwd` | `linux,rust,cyclonedds` | build | not selected | build |
| `bins/bridge-zenoh-to-xrce-fwd` | `linux,rust,xrce` | build | not selected | build |
| `workspace-rust-native-bridge` | `linux,rust,cyclonedds` | build | not selected | build |
| `workspace-rust-native-bridge-xrce` | `linux,rust,xrce` | build | not selected | build |
| `examples/native/c/listener` (`build-cyclonedds`) | `linux,c,cyclonedds` | build | build | build |

tier 2's linux coordinates are exactly `linux,{c,cpp,rust}` × one rmw each
(`c,cyclonedds` / `cpp,zenoh` / `rust,zenoh`), so every "not selected" cell above
lane-skips in the RESOLVER. No row had to be added to a lane's build: every one
is already in the build set of every lane that selects it. (`int32-sink`'s three
rows also appear in tier 2's `--coords-from` build set, because they share an
artifact root and issue 0828 made that fail closed — the RUN still skips them by
coordinate, which is the conservative direction.)

### Per-site verdicts — 12 of 12 IN LANE, all converted

| # | file:site | resolver | fixture coord | verdict |
| --: | --- | --- | --- | --- |
| 1 | `declarative_bridge_zenoh_to_cyclonedds.rs` bridge entry (nano-listener test) | `build_native_workspace_rust_bridge_entry` | `linux,rust,cyclonedds` | in lane -> `.expect` |
| 2 | same, talker | `build_native_talker_header` | `linux,rust,zenoh` | in lane -> `.expect` |
| 3 | same, bridge entry (nested-header test) | `build_native_workspace_rust_bridge_entry` | `linux,rust,cyclonedds` | in lane -> `.expect` |
| 4 | same, talker | `build_native_talker_header` | `linux,rust,zenoh` | in lane -> `.expect` |
| 5 | `bridge_zenoh_to_cyclonedds.rs` base e2e | `build_bridge_zenoh_to_cyclonedds_fwd` | `linux,rust,cyclonedds` | in lane -> `.expect` |
| 6 | same, nano-listener | `build_bridge_zenoh_to_cyclonedds_fwd` | `linux,rust,cyclonedds` | in lane -> `.expect` |
| 7 | same, ros2 | `build_bridge_zenoh_to_cyclonedds_fwd` | `linux,rust,cyclonedds` | in lane -> `.expect` |
| 8 | `declarative_bridge_zenoh_to_xrce.rs` bridge entry | `build_native_workspace_rust_bridge_xrce_entry` | `linux,rust,xrce` | in lane -> `.expect` |
| 9 | same, xrce sink | `build_int32_sink_rmw(Xrce)` | `linux,rust,xrce` | in lane -> `.expect` |
| 10 | same, talker | `build_native_talker_header` | `linux,rust,zenoh` | in lane -> `.expect` |
| 11 | `bridge_mixed_rmw.rs` bridge bin | `build_bridge_zenoh_to_xrce_fwd` | `linux,rust,xrce` | in lane -> `.expect` |
| 12 | `zephyr.rs` workspace-entry e2e sink | `build_int32_sink` | `linux,rust,zenoh` | in lane -> `.expect` |

Two SIBLING sites in the same files used `unwrap_or_else(|e| skip!(…))` rather
than the `match` shape this issue grepped for, so they were not in the twelve.
They are the same laundering and converted with them, because leaving them is
exactly the "one file, two spellings" 1112 called out:

* `declarative_bridge_zenoh_to_cyclonedds::nano_cyclone_listener` —
  `build_int32_sink_rmw(Cyclonedds)`, `linux,rust,cyclonedds`.
* `bridge_zenoh_to_cyclonedds::nano_cyclone_listener` —
  `build_native_c_example_rmw("listener", …, Cyclonedds)`, `linux,c,cyclonedds`.

### The one that was actually live in a GATED run

Nine of the twelve are unreachable under a lane gate already:
`require_prebuilt_binary_checks` PANICS for a missing in-lane fixture whenever
`gate_promised_fixtures()` (`NROS_TEST_SCOPE` / `NROS_TEST_COORDS`) holds, so
the site's `Err` arm cannot fire there. **`require_prebuilt_workspace_binary`
has no such panic** — it returns `Err` for missing OR stale in every run — so
sites 1, 3 and 8 (the two declarative bridge `native_entry` fixtures) were
laundering a gated-run `Err` into a skip for real, which is 0584's shape exactly
and the same class as 0411's note four lines below that function.

The doc comment on `build_native_workspace_rust_bridge_entry` claiming callers
"skip cleanly" when cyclonedds is unprovisioned was also stale:
`workspace-fixtures-build.sh` fails LOUD (`return 2`) for a `NROS_RMW=cyclonedds`
row on linux (issue 0120), so a successful build always produces the fixture.

### Sweep — the twelve are gone, and the CLASS is much bigger than twelve

```sh
grep -rn 'skip!' packages/testing/nros-tests/tests/ | grep -iE 'not prebuilt|not built'
```

Zero hits remain in the five files this issue named. But the same sweep is how
this issue's scope turns out to have been the SPELLING, not the defect:

* `not prebuilt` — 4 further sites, in `native_api.rs` (1) and
  `roundtrip_xprocess_e2e.rs` (3). Same words, different files, so the issue's
  five-file table missed them.
* `not built` — **54 sites across 43 files** (`entry_e2e`, `params`,
  `safety_e2e`, `native_example_{pubsub,reqresp}_e2e`, `multihost_e2e`,
  `roundtrip_xprocess_e2e`, `interop_e2e`, `qos_*`, `realtime_*`, …), almost all
  `.unwrap_or_else(|e| skip!("… fixture not built: {e}"))`.

That second set is WORSE than the twelve in one specific way, and it is worth
recording precisely: `check-skip-budget`'s `FIXTURE_RE` is
`not prebuilt|fixture binary MISSING`, so **the `not built` spelling does not
match it**. The twelve were at least loud somewhere; those 54 launder an `Err`
into a skip AND evade the gate that exists to catch exactly that. Filed
separately (issue 1129) rather than swept here, because each one wants the same
per-site evidence this issue demanded and the answer is not uniform — several
of them are behind `NROS_FIXTURES_OPTIONAL` or a west/SDK precondition.

Deliberately NOT converted here, and both stay correct:

* `zephyr.rs`'s `get_prebuilt_zephyr_example` skip — a west leaf behind
  `require_west_leaf_in_lane` + `require_zephyr()`, not one of the twelve, with a
  genuine "no Zephyr SDK on this host" story.
* the `NROS_FIXTURES_OPTIONAL`-guarded arms in `zenoh_archive_symbols.rs`,
  `zenoh_header_parity.rs` and `zpico_build_matrix.rs` — the documented
  light-tier opt-out, which the resolver itself honours one frame up.
