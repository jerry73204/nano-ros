//! phase-340 W3 — a lane's fixture BUILD and its test RUN must select the same
//! rows.
//!
//! # Why this gate is the whole design
//!
//! Issue 0482's lesson is not "tier 2 needed an `all` build". It is that a
//! fixture row's coordinate was computed in TWO places with two answers, and the
//! two derivations drifted silently until 67 of 240 rows sat in no lane at all.
//! phase-340 W3 narrows the tier-2 RUN so its own build suffices — which
//! introduces a second opportunity for exactly that drift, between:
//!
//! * the BUILD's selection — `fixtures-manifest.py list --coords-from F`, whose
//!   predicate is `row_coord(row) ∈ F`;
//! * the RUN's selection — `nros_tests::fixtures::lane::is_in_lane`, whose
//!   predicate must be the same one.
//!
//! So the gate is set equality, over the real manifest, for several coordinate
//! subsets. Anything that makes one side see a row the other does not — a new
//! default, a filter that reads a raw key, a row kind one side forgets — fails
//! here rather than as N missing fixtures in a lane three hours in.
//!
//! Nothing here builds a fixture or compiles anything: it reads the manifest and
//! runs the same Python filter the build runs.

use std::{collections::BTreeSet, path::PathBuf, process::Command};

use nros_tests::fixtures::{
    groups,
    lane::{self, Coord, Row},
};

fn tmpdir() -> PathBuf {
    // `$project/tmp/` (gitignored), not /tmp — CLAUDE.md.
    let d = nros_tests::project_root()
        .join("tmp")
        .join(format!("lane-run-narrowing-{}", std::process::id()));
    std::fs::create_dir_all(&d).expect("create tmp dir");
    d
}

fn manifest_py() -> PathBuf {
    nros_tests::project_root().join("scripts/build/fixtures-manifest.py")
}

/// Run `fixtures-manifest.py <subcommand> --coords-from <file>` and return the
/// FIRST field of each record — the `dir` for `[[fixture]]` rows (both the cargo
/// and the cmake record shapes start with it) and the `id` for workspace rows.
fn build_side_selection(subcommand: &str, coord_file: &std::path::Path) -> Vec<String> {
    let root = nros_tests::project_root();
    let out = Command::new("python3")
        .arg(manifest_py())
        .arg(subcommand)
        .arg("--coords-from")
        .arg(coord_file)
        .current_dir(&root)
        .output()
        .unwrap_or_else(|e| panic!("run fixtures-manifest.py {subcommand}: {e}"));
    assert!(
        out.status.success(),
        "fixtures-manifest.py {subcommand} --coords-from failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter(|l| !l.is_empty())
        .map(|l| l.split('\x1f').next().unwrap_or_default().to_string())
        .collect()
}

fn write_coords(name: &str, coords: &BTreeSet<Coord>) -> PathBuf {
    let path = tmpdir().join(name);
    let mut body = String::new();
    for (p, l, r) in coords {
        body.push_str(&format!("{p},{l},{r}\n"));
    }
    std::fs::write(&path, body).expect("write coord file");
    path
}

fn all_coords() -> Vec<Coord> {
    let mut v: Vec<Coord> = lane::manifest_rows()
        .iter()
        .map(|r| r.coord.clone())
        .collect();
    v.sort();
    v.dedup();
    v
}

/// The subsets this gate exercises. Deterministic and derived from the manifest
/// itself, so it keeps working as the matrix moves — and deliberately including
/// a single-coordinate subset, which is where an off-by-one in either predicate
/// shows up most sharply.
fn subsets() -> Vec<(String, BTreeSet<Coord>)> {
    let all = all_coords();
    assert!(all.len() > 3, "manifest reported {} coordinates", all.len());
    let take = |name: &str, step: usize, offset: usize| {
        let s: BTreeSet<Coord> = all
            .iter()
            .enumerate()
            .filter(|(i, _)| i % step == offset)
            .map(|(_, c)| c.clone())
            .collect();
        (name.to_string(), s)
    };
    vec![
        ("all".to_string(), all.iter().cloned().collect()),
        take("every-2nd", 2, 0),
        take("every-3rd-offset-1", 3, 1),
        (
            "single".to_string(),
            std::iter::once(all[0].clone()).collect(),
        ),
    ]
}

/// The rows a coordinate-narrowed RUN would actually execute.
///
/// Issue 0922 — this is `in lane` UNION `the resolver cannot skip it`, not
/// `in lane` alone. An out-of-lane row still runs when the resolver cannot
/// attribute an artifact back to it: `attribute_path` returns `None` on an
/// ambiguous root and the resolve fails CLOSED.
///
/// Modelling the run as pure coordinate membership made this read as a
/// build-vs-run check while it was really build-vs-a-model-of-the-run. The two
/// agreed only while the BUILD also ignored fail-closed rows — which was issue
/// 0828 itself. Fixing 0828 moved the build onto the real predicate and left
/// this side behind, so the test went red against a build that had just become
/// correct (139 built vs 92 modelled).
///
/// The skippability here is derived from the PRODUCTION attribution functions,
/// never from a column the build also reads. A cross-check whose two sides
/// consume one computation cannot catch a bug in that computation — a
/// `lane_skippable` column was tried here first and passed the mutation it
/// existed to catch, because mutating the shared function moved both sides
/// together. Independence is the point; this is the `check-rmw-api-parity` /
/// `check-rmw-abi-shape` pairing, not a second spelling of one rule.
fn run_side(kind: &str, coords: &BTreeSet<Coord>, key: fn(&Row) -> String) -> Vec<String> {
    let rows = lane::manifest_rows();
    rows.iter()
        .filter(|r| {
            if r.kind != kind {
                return false;
            }
            if lane::is_in_lane(r, coords) {
                return true;
            }
            // Out of lane — so it runs only if the resolver fails closed on it.
            // Ask the same function the resolver asks, by the route this row's
            // table actually uses.
            if kind == "workspace_fixture" {
                // Attributed by `id`, which is unique per row and gated as
                // such, so a workspace row is always attributable and always
                // skippable — however many rows share its artifact root, and
                // 66 of them do.
                lane::attribute_workspace_id(&r.id).is_none()
            } else {
                let probe = std::path::PathBuf::from(&r.artifact_root).join("probe-binary");
                lane::attribute_path_in(rows, &probe).is_none()
            }
        })
        .map(key)
        .collect()
}

/// **The invariant.** For every coordinate subset, the rows the BUILD selects
/// and the rows the RUN does not skip are the same rows.
#[test]
fn build_and_run_select_the_same_fixture_rows() {
    for (name, coords) in subsets() {
        let file = write_coords(&format!("fixture-{name}.txt"), &coords);

        let mut built = build_side_selection("list", &file);
        let mut run = run_side("fixture", &coords, |r| r.dir.clone());
        built.sort();
        run.sort();

        assert_eq!(
            built.len(),
            run.len(),
            "[{name}] the build selects {} `[[fixture]]` row(s) and the run would \
             execute {}. Whichever side is larger is resolving fixtures the other \
             never produced — issue 0482 with the two halves swapped.",
            built.len(),
            run.len()
        );
        assert_eq!(
            built, run,
            "[{name}] build-side and run-side selections differ in CONTENT even \
             though they agree in count; the two predicates have drifted apart"
        );
        let _ = std::fs::remove_file(&file);
    }
}

/// Same invariant for the other coordinate-bearing table. Checking one of two is
/// the issue-0196 shape — a gate narrower than the rule it enforces — and
/// `workspace_fixture` rows are 93 of the 333 buildable rows.
#[test]
fn build_and_run_select_the_same_workspace_rows() {
    for (name, coords) in subsets() {
        let file = write_coords(&format!("workspace-{name}.txt"), &coords);

        let mut built = build_side_selection("list-workspaces", &file);
        let mut run = run_side("workspace_fixture", &coords, |r| r.id.clone());
        built.sort();
        run.sort();

        assert_eq!(
            built, run,
            "[{name}] the build and the run disagree about which \
             `[[workspace_fixture]]` rows this lane contains"
        );
        let _ = std::fs::remove_file(&file);
    }
}

/// Every row the build OMITS must be attributable back from its artifacts, or
/// the run cannot skip it and the lane hard-fails on a fixture it deliberately
/// did not build.
///
/// This is the totality claim the whole design rests on, and it is checked in
/// the direction that matters: not "attribution mostly works" but "there is no
/// row whose omission the run would be unable to notice".
#[test]
fn every_omittable_row_is_attributable_from_its_artifacts() {
    let root = nros_tests::project_root();
    let mut blind = Vec::new();
    for row in lane::manifest_rows() {
        match row.kind.as_str() {
            // Attributed by path — but only a SOLE-row leaf can be. Issue 0517
            // step 3 gave every row of a multi-row leaf the same
            // `<dir>/target`, so `attribute_path` reports those ambiguous and
            // declines (fail closed, never a guess). Those rows reach the lane
            // by COORDINATE instead: the resolver selects the row by
            // configuration and calls `require_coord_in_lane` directly, so what
            // this gate must check for them is that the groups table carries the
            // row with a coordinate to ask about. Checking only the path arm
            // would quietly stop covering them.
            "fixture" => {
                if groups::leaf_is_multi_row(&row.dir) {
                    let known = groups::manifest_rows()
                        .iter()
                        .any(|g| g.dir == row.dir && g.coord == row.coord);
                    if !known {
                        blind.push(format!(
                            "{} (multi-row leaf, coordinate {} absent from the groups \
                             export, so the coord route cannot place it)",
                            row.label(),
                            row.coord_str()
                        ));
                    }
                    continue;
                }
                let probe = root.join(&row.artifact_root).join("bin");
                match lane::attribute_path(&probe) {
                    Some(got) if got.artifact_root == row.artifact_root => {}
                    _ => blind.push(format!(
                        "{} (artifact_root {})",
                        row.label(),
                        row.artifact_root
                    )),
                }
            }
            // Attributed by id.
            "workspace_fixture" => {
                if lane::attribute_workspace_id(&row.id).is_none() {
                    blind.push(format!("{} (workspace id)", row.label()));
                }
            }
            other => panic!("unknown manifest row kind {other:?} — teach this gate about it"),
        }
    }
    assert!(
        blind.is_empty(),
        "{} row(s) can be omitted by a lane build but not recognised as \
         out-of-lane by the run, so the lane would fail on a fixture it \
         deliberately did not build:\n  {}",
        blind.len(),
        blind.join("\n  ")
    );
}

/// The west family's half of the same invariant — phase-350 W1.b.
///
/// `lane::manifest_rows()` is the PATH-attribution table and deliberately omits
/// `builder = "west"` rows: their artifacts land in the Zephyr workspace, not
/// under the row's `dir`, so `attribute_path` can never answer for them. They
/// reach the lane by COORDINATE, keyed on the build-dir name — so what must be
/// checked for them is what the multi-row arm above checks: that the coordinate
/// is PLACEABLE, i.e. present in the export the resolver queries.
///
/// Without this, W1.b's build-side narrowing would be a one-sided change: the
/// zephyr lane omits leaves outside its coordinates, and a run that could not
/// place them would fail on a fixture it deliberately did not build.
#[test]
fn every_west_leaf_is_placeable_by_coordinate() {
    let leaves = lane::west_leaves();
    assert!(
        !leaves.is_empty(),
        "the west-leaves export is empty — the zephyr lane has no rows to narrow on"
    );

    let mut unplaceable = Vec::new();
    for leaf in leaves {
        // issue 1016 — a `leaves.iter().any(|l| l.build_name == leaf.build_name)`
        // check stood here, over `leaf ∈ leaves`. Always true, and it read as
        // the membership assertion this file is about. The direction that can
        // actually fail is the OTHER one — every name the RESOLVERS can produce
        // is in this export, because `require_west_leaf_in_lane` fails open on
        // a miss — and it is asserted in
        // `lane_build_covers_run::every_west_leaf_the_run_can_name_is_built_or_skippable`.
        // What remains here is what the export itself can get wrong: an
        // incomplete coordinate, and a build-dir name claimed by two leaves.
        let (p, l, r) = &leaf.coord;
        if p.is_empty() || l.is_empty() || r.is_empty() {
            unplaceable.push(format!(
                "{} (incomplete coordinate {p},{l},{r})",
                leaf.build_name
            ));
        }
    }
    assert!(
        unplaceable.is_empty(),
        "{} west leaf/leaves cannot be placed in a lane by coordinate, so a \
         coordinate-scoped run could not skip them:\n  {}",
        unplaceable.len(),
        unplaceable.join("\n  ")
    );

    // Build names are the KEY, so they must be unique — two leaves sharing one
    // would make the lookup answer for the wrong coordinate.
    let mut seen = std::collections::BTreeMap::new();
    for leaf in leaves {
        if let Some(prev) = seen.insert(&leaf.build_name, &leaf.dir) {
            panic!(
                "two west leaves share the build dir {}: {} and {} — the \
                 coordinate lookup keys on it",
                leaf.build_name, prev, leaf.dir
            );
        }
    }
}

/// Fail closed. A path under no manifest artifact root must never be skipped.
///
/// This is the issue-0445 guard: the run must not be able to turn "never built"
/// into "skipped" for the families it cannot attribute (the Zephyr west leaves,
/// the shared `build/cargo-fixtures` dirs, the compile-check lane). Those are
/// built module-level rather than by coordinate, so nothing is missing and a
/// hard failure remains the right answer.
#[test]
fn an_unattributable_artifact_is_never_skipped() {
    let coords: BTreeSet<Coord> = std::iter::once(all_coords()[0].clone()).collect();
    for p in [
        "build/zephyr-workspace-builds/build-ws-c-entry-zenoh/zephyr/zephyr.exe",
        "build/cargo-fixtures/qemu-arm-baremetal/thumbv7m-none-eabi/p/bin",
        "build/cmake-fixtures/some-id/bin",
        "/somewhere/entirely/else/bin",
    ] {
        let path = if p.starts_with('/') {
            PathBuf::from(p)
        } else {
            nros_tests::project_root().join(p)
        };
        assert!(
            lane::attribute_path(&path).is_none(),
            "{p} attributed to a manifest row; the run could skip a fixture whose \
             absence is a real failure"
        );
    }
    // …and the in-lane direction of the same rule: a row the lane DOES contain
    // must never be skipped, whatever state its artifacts are in. Without this,
    // "refuse everything" would satisfy the gate above.
    let in_lane: Vec<&Row> = lane::manifest_rows()
        .iter()
        .filter(|r| lane::is_in_lane(r, &coords))
        .collect();
    assert!(
        !in_lane.is_empty(),
        "no row is in the single-coordinate lane; the subset is degenerate"
    );
    for row in in_lane {
        assert!(
            lane::is_in_lane(row, &coords),
            "{} is in lane and must never be skipped",
            row.label()
        );
    }
}

/// The skip DECISION, in both directions, for both row kinds.
///
/// `is_in_lane` is the predicate; this is the thing the resolver actually calls,
/// and the two could disagree (an inverted condition, a `None` that means "skip"
/// instead of "do not attribute"). Checked against a lane built from real rows
/// so the fixture and workspace arms both have a populated in- and out-of-lane
/// side — a one-sided test is how issue 0442's per-arm subsets happened.
#[test]
fn the_skip_decision_fires_only_outside_the_lane() {
    let root = nros_tests::project_root();
    // A lane holding one coordinate of EACH row kind, so all four arms below are
    // populated. Derived rather than written down — the first version used the
    // manifest's first coordinate, which no `[[fixture]]` row occupies, and the
    // vacuity check at the bottom is what caught it.
    let coords: BTreeSet<Coord> = ["fixture", "workspace_fixture"]
        .iter()
        .map(|kind| {
            lane::manifest_rows()
                .iter()
                .find(|r| r.kind == *kind)
                .unwrap_or_else(|| panic!("manifest has no {kind} row"))
                .coord
                .clone()
        })
        .collect();

    let mut checked = (0, 0, 0, 0);
    for row in lane::manifest_rows() {
        let inside = lane::is_in_lane(row, &coords);
        match row.kind.as_str() {
            "fixture" => {
                // Multi-row leaves take the COORDINATE route (issue 0517 step 3
                // — see `every_omittable_row_is_attributable_from_its_artifacts`);
                // the path route cannot answer for them, so asking it here would
                // read a by-design `None` as "the run would fail on a fixture the
                // build omitted".
                let got = if groups::leaf_is_multi_row(&row.dir) {
                    lane::skip_reason_for_coord(&row.coord, row.label(), &coords)
                } else {
                    let probe = root.join(&row.artifact_root).join("bin");
                    lane::skip_reason_for_path(&probe, &coords)
                };
                if inside {
                    assert!(
                        got.is_none(),
                        "{} is IN lane and must not be skipped — a lane that skips \
                         its own coordinates would report green having run nothing",
                        row.label()
                    );
                    checked.0 += 1;
                } else {
                    let reason = got.unwrap_or_else(|| {
                        panic!(
                            "{} is OUT of lane and its artifacts were not skipped; \
                             the run would fail on a fixture the build omitted",
                            row.label()
                        )
                    });
                    assert!(
                        reason.contains("out of lane") && reason.contains(&row.coord_str()),
                        "the skip must name itself and the coordinate, not read as \
                         a staleness verdict (issue 0445): {reason}"
                    );
                    checked.1 += 1;
                }
            }
            "workspace_fixture" => {
                let got = lane::skip_reason_for_workspace_id(&row.id, &coords);
                if inside {
                    assert!(got.is_none(), "{} is IN lane", row.label());
                    checked.2 += 1;
                } else {
                    assert!(got.is_some(), "{} is OUT of lane", row.label());
                    checked.3 += 1;
                }
            }
            other => panic!("unknown row kind {other:?}"),
        }
    }
    // Every arm must have been exercised, or an assertion above is vacuous.
    assert!(
        checked.0 > 0 && checked.1 > 0 && checked.2 > 0 && checked.3 > 0,
        "an arm of this gate saw no rows (fixture in/out {}/{}, workspace in/out \
         {}/{}); the chosen lane is degenerate and the assertions are vacuous",
        checked.0,
        checked.1,
        checked.2,
        checked.3
    );
}

/// The run's coordinate for an `rmw`-less row must be `row_coord`'s answer
/// (`zenoh`), not a bare `None` — the exact drift that put 67 rows in no lane.
///
/// Asserted on the RUN side specifically: the build side is already gated by
/// `lane_build_covers_run::every_fixture_row_is_reachable_through_the_coordinate_filter`,
/// and this is the new consumer that could grow a second default.
#[test]
fn the_run_reads_an_rmw_less_row_at_the_documented_default() {
    let rows = lane::manifest_rows();
    assert!(
        rows.iter().all(|r| !r.coord.2.is_empty()),
        "a row reached the run with an empty rmw — `(platform, lang, None)` is a \
         coordinate no lane-coords file can spell, so the row would be invisible \
         to the run exactly as it was to the build (issue 0482)"
    );
    // The default is `zenoh`, and it must be a coordinate rows actually occupy —
    // a default nothing lands on would make this assertion vacuous.
    assert!(
        rows.iter().any(|r| r.coord.2 == "zenoh"),
        "no row resolves to the documented default rmw"
    );
}
