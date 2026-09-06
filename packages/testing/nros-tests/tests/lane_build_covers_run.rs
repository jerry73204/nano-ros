//! Issue 0482 — a lane's fixture BUILD must cover the tests that lane RUNS.
//!
//! # The defect
//!
//! `just build-test-fixtures lane=tier2` succeeded, wrote its stamp, and
//! satisfied `_require-fixtures`. `just ci-matrix` then produced ~231 failures,
//! nearly all "Test fixture is STALE" / not-found, because `ci-matrix` invokes
//! `test-all` with no `NROS_TEST_SCOPE` — the whole suite runs, and 34 of the 47
//! fixture coordinates had never been built.
//!
//! Two questions were being answered from one lane name:
//!
//! * which fixtures must be **FRESH** — the lane's cell cover, legitimately
//!   narrow; that narrowing is tier 2's entire saving;
//! * which fixtures must **EXIST** — a property of the RUN.
//!
//! `nros_lane_build_lane` is the mapping from the first to the second.
//! `CiLane::run_scope` is its declaration, next to the cell selection. This file
//! is the gate that keeps them equal AND pins the concrete regression, because a
//! consistency check alone would pass just as happily if both sides said
//! "tier 2 builds tier 2".
//!
//! # Why it drives the shell rather than re-implementing it
//!
//! `_require-fixtures` runs `scripts/build/fixture-lane.sh`, so that is the
//! thing whose behaviour matters. Every assertion below execs the real function
//! against a temporary stamp (`NROS_FIXTURE_STAMP` is overridable for exactly
//! this). Pure bash on both sides — nothing here compiles a fixture or a binary.

use std::{collections::BTreeSet, path::PathBuf, process::Command};

/// Run a snippet with `fixture-lane.sh` sourced. Returns (exit code, stdout+stderr).
fn lane_sh(snippet: &str, stamp: Option<&str>) -> (i32, String) {
    lane_sh_env(snippet, stamp, &[])
}

/// [`lane_sh`] with extra environment. phase-340 W3 made
/// `nros_fixtures_stamp_require` also check that a narrow build is paired with a
/// narrowed RUN (`NROS_TEST_COORDS`), so tests that exercise the coverage logic
/// have to supply it — and the tests that exercise the PAIRING requirement
/// deliberately do not.
fn lane_sh_env(snippet: &str, stamp: Option<&str>, env: &[(&str, &str)]) -> (i32, String) {
    let root = nros_tests::project_root();
    let mut cmd = Command::new("bash");
    cmd.arg("-c")
        .arg(format!(
            "set -u; source scripts/build/fixture-lane.sh; {snippet}"
        ))
        .current_dir(&root);
    // The child must not INHERIT the lane environment. `ci-matrix` exports
    // NROS_TEST_COORDS (that is the whole point of W3), so a test asserting
    // "unset is refused" passed standalone and FAILED inside a real run — the
    // child saw the ambient value and the guard correctly declined to refuse.
    // A guard that reads green in isolation while the hole is open in situ is
    // worse than no guard, so the helper now OWNS these variables: cleared
    // first, then set only by an explicit `env` entry below.
    for k in [
        "NROS_TEST_COORDS",
        "NROS_FIXTURE_LANE",
        "NROS_FIXTURE_STAMP",
    ] {
        cmd.env_remove(k);
    }
    if let Some(s) = stamp {
        cmd.env("NROS_FIXTURE_STAMP", s);
    }
    for (k, v) in env {
        cmd.env(k, v);
    }
    let out = cmd.output().expect("run fixture-lane.sh");
    let mut text = String::from_utf8_lossy(&out.stdout).into_owned();
    text.push_str(&String::from_utf8_lossy(&out.stderr));
    (out.status.code().unwrap_or(-1), text)
}

/// The `lane-coords` binary, PREBUILT — never `cargo run`.
///
/// Issue 0523 part B. This helper used to call `nros_lane_coords_file <lane>`,
/// whose body is `cargo run -q -p nros-tests --bin lane-coords`. That is correct
/// inside a BUILD (`build-test-fixtures`, `ci-matrix`) and wrong inside a test:
/// `lane-coords` lives in `nros-tests`, so ANY edit to this crate invalidates it
/// and the next run recompiles the whole package before writing a byte — the
/// function's own comment says "COMPILES for seconds-to-minutes". Three cases
/// here then blew the 60 s per-test timeout, which is how they read as hangs.
///
/// CLAUDE.md: no compilation inside tests. The binary is a build-stage artifact;
/// this consumes it and skips (loudly, with the remedy) when it is absent,
/// rather than building it here.
fn lane_coords_bin() -> PathBuf {
    let target = nros_tests::project_root().join("target");
    // NEWEST, not a preferred profile. Picking `nros-fast-release` first looked
    // tidy and selected an ELEVEN-DAY-OLD artifact that answered `tier2` with 12
    // coordinates where the current sources say 13 — a museum binary, and the
    // comparison against it failed with a drift report that blamed the guard.
    //
    // Freshness needs no check of its own: `lane-coords` is a bin of THIS
    // package, so `cargo nextest run -p nros-tests` rebuilds it in the build
    // phase before any test runs (verified — an artifact backdated to 2020 came
    // back stamped today during the run). That is the whole point of consuming
    // it rather than shelling out to `cargo run`: the compile still happens, it
    // just happens where it is not on a 60-second clock.
    let newest = ["nros-fast-release", "debug", "release"]
        .iter()
        .map(|p| target.join(p).join("lane-coords"))
        .filter(|p| p.is_file())
        .filter_map(|p| p.metadata().and_then(|m| m.modified()).ok().map(|t| (t, p)))
        .max_by_key(|(t, _)| *t);
    match newest {
        Some((_, bin)) => bin,
        None => nros_tests::skip!(
            "lane-coords not prebuilt under {}/{{nros-fast-release,debug,release}} — \
             run `just build`. This test must NOT compile it (issue 0523).",
            target.display()
        ),
    }
}

/// The lane's coordinate set, written where the caller can read it — the same
/// content `just ci-matrix` exports as `NROS_TEST_COORDS`.
///
/// Produced by RUNNING the prebuilt selector, so a test still cannot pass
/// against a coordinate set the recipe would never produce; only the compile
/// moved out.
fn lane_coords_file(lane: &str) -> String {
    let out = Command::new(lane_coords_bin())
        .arg(lane)
        .current_dir(nros_tests::project_root())
        .output()
        .expect("run prebuilt lane-coords");
    assert!(
        out.status.success(),
        "lane-coords {lane} failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let body = String::from_utf8_lossy(&out.stdout).into_owned();
    assert!(
        !body.trim().is_empty(),
        "lane '{lane}' selected zero coordinates — the same refusal \
         `nros_lane_coords_file` makes"
    );
    let path = tmpdir().join(format!("lane-coords-{lane}.txt"));
    std::fs::write(&path, body).expect("write lane coords");
    path.display().to_string()
}

/// Write a stamp in the format `nros_fixtures_stamp_write` produces.
fn write_stamp(dir: &std::path::Path, lane: &str, coords: &[&str]) -> PathBuf {
    let path = dir.join(format!(".fixtures-built-{lane}"));
    let mut body = String::from("# nano-ros fixture build stamp (test)\n");
    body.push_str("built_at=2026-08-07T00:00:00Z\n");
    body.push_str(&format!("lane={lane}\n"));
    for c in coords {
        body.push_str(&format!("coord={c}\n"));
    }
    std::fs::write(&path, body).expect("write stamp");
    path
}

fn tmpdir() -> PathBuf {
    // `$project/tmp/` (gitignored), not /tmp — CLAUDE.md.
    let d = nros_tests::project_root()
        .join("tmp")
        .join(format!("lane-build-covers-run-{}", std::process::id()));
    std::fs::create_dir_all(&d).expect("create tmp dir");
    d
}

/// The two spellings of the mapping must agree, for every lane, in both
/// directions of the vocabulary.
///
/// The shell is the runtime implementation (a preflight must not compile
/// anything to answer this); `CiLane::run_scope` is the declaration that sits
/// next to the cell cover it has to stay consistent with. Nothing else binds
/// them, and an unbound pair is how `fixtures-manifest.py` and
/// `matrix_fixture_coverage.rs` came to hold two different coordinates for the
/// same fixture row.
#[test]
fn shell_build_lane_matches_the_rust_declaration() {
    use nros_tests::ci_lane::{ALL, CiLane};

    for lane in ALL {
        let token = lane.lane_token();
        let (code, out) = lane_sh(&format!("nros_lane_build_lane {token}"), None);
        assert_eq!(code, 0, "nros_lane_build_lane {token} failed:\n{out}");
        assert_eq!(
            out.trim(),
            lane.build_lane(),
            "fixture-lane.sh and CiLane::build_lane disagree about what a {token} RUN \
             needs built. They are two spellings of ONE fact; fix both."
        );
        // The token itself is part of the contract: `_NROS_LANES` must know the
        // name Rust spells the lane with, or every lookup above answers about a
        // different lane.
        let (code, out) = lane_sh(&format!("nros_lane_validate {token}"), None);
        assert_eq!(
            code, 0,
            "fixture-lane.sh does not know the lane token CiLane::lane_token \
             emits ({token}):\n{out}"
        );
    }
    // Keep the type in use so a removal of `CiLane` here fails to compile
    // rather than silently dropping the binding.
    let _: CiLane = CiLane::Tier2;

    // The module-level lanes are their own build lane, in both places by
    // construction — asserted so a future edit cannot quietly make `native`
    // require `all` and re-price tier 1.
    for token in ["all", "native"] {
        let (code, out) = lane_sh(&format!("nros_lane_build_lane {token}"), None);
        assert_eq!(code, 0, "nros_lane_build_lane {token} failed:\n{out}");
        assert_eq!(out.trim(), token, "{token} must be its own build lane");
    }
}

/// An unknown lane must be REFUSED, not defaulted.
///
/// The two silent readings are both wrong in dangerous ways: an empty answer
/// makes the coverage check compare against a nameless file (`sort` reads stdin
/// — the preflight HANGS), and defaulting to `all` launders a requirement nobody
/// declared.
#[test]
fn an_undeclared_lane_is_refused() {
    let (code, out) = lane_sh("nros_lane_build_lane not-a-lane", None);
    assert_ne!(code, 0, "an unknown lane must fail, got:\n{out}");
}

/// **The regression, in the only form it can still take.**
///
/// Issue 0482's defect was that a `lane=tier2` build satisfied a tier-2
/// preflight while `ci-matrix` ran the WHOLE suite — preflight green, then ~231
/// STALE failures on coordinates the lane never built. 0482 closed that by
/// refusing the narrow build. phase-340 W3 closes it the other way instead, by
/// narrowing the RUN, so the narrow build is now ACCEPTED.
///
/// Which means the acceptance on its own is no longer evidence of anything: it
/// is correct only while the run really is narrowed. So this pins the two
/// together — the preflight accepts `lane=tier2`, AND tier 2 declares a
/// coordinate-scoped run whose build lane is itself. Assert only the first and
/// a future edit reverting `run_scope` to `All` would leave this test green over
/// exactly the original bug.
#[test]
fn a_tier2_build_satisfies_the_tier2_run_because_that_run_is_narrowed() {
    use nros_tests::ci_lane::{CiLane, RunScope};

    assert_eq!(
        CiLane::Tier2.run_scope(),
        RunScope::LaneCoords,
        "tier 2 must narrow its RUN to its own coordinates; if it stops doing \
         that, its build lane must go back to `all` (issue 0482) — accepting a \
         `lane=tier2` build for an unnarrowed run is the original defect"
    );
    assert_eq!(CiLane::Tier2.build_lane(), "tier2");

    let dir = tmpdir();
    let coords = lane_coords_file("tier2");
    // The stamp must carry the lane's REAL coordinates now that the preflight
    // diffs them against the run's — a hand-picked pair would fail on coverage
    // for reasons unrelated to what this test is about.
    let coord_lines = std::fs::read_to_string(&coords).expect("read tier2 coords");
    let coord_refs: Vec<&str> = coord_lines.lines().filter(|l| !l.is_empty()).collect();
    let stamp = write_stamp(&dir, "tier2", &coord_refs);
    let (code, out) = lane_sh_env(
        "nros_fixtures_stamp_require tier2",
        Some(stamp.to_str().unwrap()),
        &[("NROS_TEST_COORDS", coords.as_str())],
    );
    assert_eq!(
        code, 0,
        "a lane=tier2 fixture build must now satisfy the tier-2 preflight — \
         phase-340 W3 narrows the tier-2 RUN to the same coordinates, which is \
         what makes the middle rung of the ladder affordable:\n{out}"
    );
    let _ = std::fs::remove_file(&stamp);
}

/// A `lane=tier1` build must still NOT satisfy a tier-2 run.
///
/// The affordability fix must not degenerate into "any stamp will do". Tier 1's
/// cover (10 of 47 coordinates) is a strict subset of tier 2's (13), so the
/// coordinate diff has to report the difference and refuse.
#[test]
fn a_narrower_lane_build_still_does_not_satisfy_a_wider_run() {
    let dir = tmpdir();
    let stamp = write_stamp(&dir, "tier1", &["linux,rust,zenoh"]);
    let (code, out) = lane_sh_env(
        "nros_fixtures_stamp_require tier2 < /dev/null",
        Some(stamp.to_str().unwrap()),
        // Correctly narrowed run — so this exercises the COVERAGE diff, not the
        // pairing check.
        &[("NROS_TEST_COORDS", lane_coords_file("tier2").as_str())],
    );
    assert_ne!(
        code, 0,
        "a lane=tier1 build satisfied a tier-2 run; the coverage diff has \
         stopped diffing:\n{out}"
    );
    assert!(
        out.contains("build-test-fixtures"),
        "the refusal must name the build that would fix it:\n{out}"
    );
    let _ = std::fs::remove_file(&stamp);
}

/// The other direction: a full build satisfies every lane. Without this the
/// previous test could be "passed" by refusing everything, which would make the
/// preflight unusable rather than correct.
#[test]
fn an_all_build_satisfies_every_lane() {
    let dir = tmpdir();
    let stamp = write_stamp(&dir, "all", &[]);
    for lane in ["all", "native", "tier1", "tier2", "tier2-nightly"] {
        let (code, out) = lane_sh(
            &format!("nros_fixtures_stamp_require {lane}"),
            Some(stamp.to_str().unwrap()),
        );
        assert_eq!(
            code, 0,
            "a full fixture build must satisfy lane {lane}:\n{out}"
        );
    }
    let _ = std::fs::remove_file(&stamp);
}

/// Tier 1 keeps its saving: a `native` build still satisfies the tier-1 run.
///
/// The fix must not re-price the one lane that was already honest — if
/// `run_scope` ever said tier 1 runs everything, `just ci` would start demanding
/// a tier-3 build and the ladder would collapse to one rung.
#[test]
fn a_native_build_satisfies_the_tier1_run() {
    let dir = tmpdir();
    let stamp = write_stamp(&dir, "native", &[]);
    for lane in ["native", "tier1"] {
        let (code, out) = lane_sh(
            &format!("nros_fixtures_stamp_require {lane}"),
            Some(stamp.to_str().unwrap()),
        );
        assert_eq!(
            code, 0,
            "a lane=native build must satisfy the {lane} run — tier 1 narrows its \
             run to host binaries, which is exactly what that build produces:\n{out}"
        );
    }
    let _ = std::fs::remove_file(&stamp);
}

/// Every buildable fixture row must be REACHABLE through the coordinate filter
/// a lane build uses.
///
/// This is the other half of issue 0482 and the half that had actually rotted.
/// `rmw` is optional on `[[fixture]]`, and `fixtures-manifest.py` compared the
/// raw key against the lane's triples while `matrix_fixture_coverage.rs` applied
/// a `zenoh` default. 67 of 240 buildable rows therefore sat at the coordinate
/// `(platform, lang, None)` — a triple no `lane-coords` file can even spell — so
/// no coordinate-scoped lane selected them, and the STALENESS GATE, which runs
/// through the same filter, could not report the omission either. Tier 2
/// selected 46 rows where it should have selected 109.
///
/// The check is a round trip: hand the filter the coordinates the manifest
/// itself reports, and every row must come back. Nothing about it depends on
/// which lanes exist, so it keeps holding as the matrix moves — and it fails
/// loudly the moment a row's coordinate stops being expressible, which is the
/// only shape this defect has.
#[test]
fn every_fixture_row_is_reachable_through_the_coordinate_filter() {
    let root = nros_tests::project_root();
    let manifest = root.join("scripts/build/fixtures-manifest.py");

    let coords_out = Command::new("python3")
        .arg(&manifest)
        .arg("coords")
        .current_dir(&root)
        .output()
        .expect("run fixtures-manifest.py coords");
    assert!(
        coords_out.status.success(),
        "fixtures-manifest.py coords failed: {}",
        String::from_utf8_lossy(&coords_out.stderr)
    );
    let coords_text = String::from_utf8(coords_out.stdout).expect("utf-8");

    // Distinct coordinates, and how many `[[fixture]]` rows sit on them.
    let mut triples: BTreeSet<String> = BTreeSet::new();
    let mut plain_rows = 0usize;
    for line in coords_text.lines().filter(|l| !l.is_empty()) {
        let f: Vec<&str> = line.split('\x1f').collect();
        assert_eq!(f.len(), 8, "unexpected coords record: {line:?}");
        triples.insert(format!("{},{},{}", f[1], f[2], f[3]));
        if f[0] == "fixture" {
            plain_rows += 1;
        }
    }
    assert!(
        plain_rows > 0,
        "manifest reported no buildable fixture rows"
    );

    let dir = tmpdir();
    let coord_file = dir.join("all-coords.txt");
    let mut body = String::new();
    for t in &triples {
        body.push_str(t);
        body.push('\n');
    }
    std::fs::write(&coord_file, body).expect("write coord file");

    // Count per BUILDER, because "reachable through the coordinate filter" is a
    // question about a row's OWN build lane. phase-350 W1 gave the zephyr west
    // leaves rows, and a west row must stay invisible to the cargo/cmake lane —
    // handing one to `fixtures-build.sh` would cargo-build a Zephyr app, and to
    // the staleness probe would have it watch a `target/` nothing writes. So
    // `list` excludes them unless asked by name, and the zephyr lane asks.
    //
    // Summing the two lists keeps the invariant exactly as strong: a row
    // reachable through NEITHER still fails, which is the case issue 0482 is
    // about. What it stops asserting is that one list sees every builder.
    let list_with = |extra: &[&str]| -> usize {
        let mut cmd = Command::new("python3");
        cmd.arg(&manifest)
            .arg("list")
            .arg("--coords-from")
            .arg(&coord_file);
        for a in extra {
            cmd.arg(a);
        }
        let out = cmd
            .current_dir(&root)
            .output()
            .expect("run fixtures-manifest.py list --coords-from");
        assert!(
            out.status.success(),
            "list --coords-from {extra:?} failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter(|l| !l.is_empty())
            .count()
    };
    let selected = list_with(&[]) + list_with(&["--builder", "west"]);

    assert_eq!(
        selected, plain_rows,
        "the coordinate filter selected {selected} of {plain_rows} buildable \
         [[fixture]] rows when handed EVERY coordinate the manifest reports. The \
         missing rows sit at a coordinate no lane can express, so no \
         coordinate-scoped build will ever produce them and the staleness gate — \
         which uses this same filter — cannot report them missing (issue 0482)."
    );
    let _ = std::fs::remove_file(&coord_file);
}

/// A coordinate-scoped build must be REFUSED for a module-level requirement —
/// and must refuse it by failing, not by hanging.
///
/// `nros_lane_build_lane tier1` is `native`, which has no coordinate file. The
/// coverage diff below it does `comm -23 <(sort -u "$want_file") …`; handed an
/// empty `$want_file`, `sort` reads STDIN and the preflight blocks forever. A
/// preflight that hangs is worse than one that is wrong, because nothing reports
/// it.
#[test]
fn a_coordinate_scoped_build_is_refused_for_a_module_lane() {
    let dir = tmpdir();
    let stamp = write_stamp(&dir, "tier2", &["linux,rust,zenoh"]);
    let (code, out) = lane_sh(
        // `< /dev/null` so a regression HANGS nothing: if the guard is removed,
        // `sort` gets EOF immediately and the test fails on the exit code
        // instead of blocking the suite.
        "nros_fixtures_stamp_require tier1 < /dev/null",
        Some(stamp.to_str().unwrap()),
    );
    assert_ne!(
        code, 0,
        "a coordinate-scoped build must not satisfy the module-level `native` \
         requirement — a coordinate cover is a strict subset of a module's \
         rows:\n{out}"
    );
    let _ = std::fs::remove_file(&stamp);
}

/// **phase-340 W3's own 0482 guard.** A narrow build must be refused when the
/// RUN is not narrowed to match.
///
/// The recipes export `NROS_TEST_COORDS`, and `ci_lane::tests::
/// recipes_run_the_scope_their_lane_declares` gates that they do. But
/// `NROS_FIXTURE_LANE=tier2 just test-all` typed by hand reaches the SAME
/// acceptance with no narrowing — a narrow stamp accepted for a run that
/// resolves all 333 rows, which is issue 0482 verbatim. Gated where the
/// acceptance is granted, not only where it is configured.
#[test]
fn a_narrow_build_is_refused_when_the_run_is_not_narrowed() {
    let dir = tmpdir();
    let coords = lane_coords_file("tier2");
    let coord_lines = std::fs::read_to_string(&coords).expect("read tier2 coords");
    let coord_refs: Vec<&str> = coord_lines.lines().filter(|l| !l.is_empty()).collect();
    let stamp = write_stamp(&dir, "tier2", &coord_refs);

    // Assert the PRECONDITION, so this can never pass vacuously again: the
    // child must really see NROS_TEST_COORDS absent. Without this the test is
    // only as true as the ambient environment happens to be.
    let (pre_code, pre_out) = lane_sh(
        "if [ -n \"${NROS_TEST_COORDS:-}\" ]; then echo LEAKED; exit 9; fi; echo ABSENT",
        Some(stamp.to_str().unwrap()),
    );
    assert_eq!(
        pre_code, 0,
        "precondition: NROS_TEST_COORDS leaked into the child, so the refusal \
         below would not be testing anything:\n{pre_out}"
    );

    let (code, out) = lane_sh(
        "nros_fixtures_stamp_require tier2 < /dev/null",
        Some(stamp.to_str().unwrap()),
    );
    assert_ne!(
        code, 0,
        "a lane=tier2 stamp was accepted with NROS_TEST_COORDS unset — the run \
         would resolve every coordinate against a build of 13 of them:\n{out}"
    );
    assert!(
        out.contains("NROS_TEST_COORDS"),
        "the refusal must name the missing narrowing:\n{out}"
    );

    // A coordinate file that is not this lane's is refused too: accepting any
    // file would let the build's acceptance and the run's narrowing come from
    // two different places, which is issue 0443's shape with new names.
    let other = dir.join("wrong-coords.txt");
    std::fs::write(&other, "linux,rust,zenoh\n").expect("write coords");
    let (code, out) = lane_sh_env(
        "nros_fixtures_stamp_require tier2 < /dev/null",
        Some(stamp.to_str().unwrap()),
        &[("NROS_TEST_COORDS", other.to_str().unwrap())],
    );
    assert_ne!(code, 0, "a foreign coordinate file was accepted:\n{out}");

    // …and a full build still needs no narrowing at all, or scoping only the
    // freshness gate on top of `lane=all` would stop working.
    let full = write_stamp(&dir, "all", &[]);
    let (code, out) = lane_sh(
        "nros_fixtures_stamp_require tier2",
        Some(full.to_str().unwrap()),
    );
    assert_eq!(
        code, 0,
        "`NROS_FIXTURE_LANE=tier2` on top of a full build must still be allowed \
         — every fixture exists, so an unnarrowed run is fine:\n{out}"
    );

    let _ = std::fs::remove_file(&stamp);
    let _ = std::fs::remove_file(&other);
    let _ = std::fs::remove_file(&full);
}

/// The same property for the WEST table — issue 1016.
///
/// `every_unskippable_row_is_in_its_lane_build` below cannot see west leaves and
/// never could: `row_artifact_root` returns `""` for them (west writes into the
/// Zephyr build root, not under the row's `dir`), so `row_is_lane_skippable`
/// calls every one of them skippable and the check has nothing to compare. Their
/// skippability is real, but it is enforced somewhere else entirely — in
/// `fixtures::lane::require_west_leaf_in_lane`, by looking the leaf's BUILD-DIR
/// NAME up in `fixtures-manifest.py west-leaves`.
///
/// That lookup FAILS OPEN. A name the manifest does not model is therefore:
///
///   * never emitted by `west-leaves`, so no lane's build produces it; and
///   * never skippable by the run, because the lookup misses.
///
/// Which is exactly 0828's shape one table over, and it arrives as
/// `BuildFailed("Zephyr fixture is STALE …")` — indistinguishable from a cell
/// that ran and failed. Issue 0968 reasoned about six such cells as failures.
///
/// MEMBERSHIP, and named: for every build-dir name the run can produce, either
/// the lane BUILDS it or the manifest models it at a coordinate outside the
/// lane (so the run skips it). "Counts agree" would pass with the property
/// broken, which is the mistake 0828's first test made.
#[test]
fn every_west_leaf_the_run_can_name_is_built_or_skippable() {
    let repo = nros_tests::project_root();

    // The run's vocabulary, harvested by the gate that owns that harvest — one
    // spelling, so this test cannot drift from what `just check fast` enforces.
    let vocab = Command::new("python3")
        .arg("-c")
        .arg(
            r#"
import importlib.util, sys
spec = importlib.util.spec_from_file_location("g", "scripts/check-west-leaf-vocabulary.py")
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
names = g.resolver_concrete_names()
print("\n".join(sorted(names)))
"#,
        )
        .current_dir(&repo)
        .output()
        .expect("harvest the resolver's west build-dir vocabulary");
    assert!(
        vocab.status.success(),
        "vocabulary harvest failed:\n{}",
        String::from_utf8_lossy(&vocab.stderr)
    );
    let vocab: BTreeSet<String> = String::from_utf8_lossy(&vocab.stdout)
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(str::to_string)
        .collect();
    // A harvest that silently returned nothing would make every assertion below
    // vacuously true — the failure mode a source-scraping check has.
    assert!(
        vocab.len() >= 30,
        "harvested only {} west build-dir name(s) from the resolvers; the \
         harvest has broken and this test is checking nothing",
        vocab.len()
    );

    let west_build_names = |coords: Option<&str>| -> BTreeSet<String> {
        let mut cmd = Command::new("python3");
        cmd.arg("scripts/build/fixtures-manifest.py")
            .arg("west-leaves");
        if let Some(c) = coords {
            cmd.arg("--coords-from").arg(c);
        }
        let out = cmd.current_dir(&repo).output().expect("run west-leaves");
        assert!(
            out.status.success(),
            "west-leaves failed:\n{}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter(|l| !l.is_empty())
            .map(|l| l.split('\x1f').nth(6).unwrap_or_default().to_string())
            .collect()
    };

    let modelled = west_build_names(None);
    assert!(
        !modelled.is_empty(),
        "the manifest models no west leaves at all — `west-leaves` has broken"
    );

    for lane in ["tier1", "tier2", "tier2-nightly"] {
        let coords = lane_coords_file(lane);
        let built = west_build_names(Some(&coords));
        let stranded: Vec<&String> = vocab
            .iter()
            .filter(|name| !built.contains(*name) && !modelled.contains(*name))
            .collect();
        assert!(
            stranded.is_empty(),
            "{lane}: {} west build-dir name(s) the run can resolve are neither \
             built by this lane nor modelled by the manifest, so \
             `require_west_leaf_in_lane` fails open on them and the run demands \
             a leaf no lane produces (issue 1016). Add the `[[fixture]] builder \
             = \"west\"` row, or delete the resolver name:\n  {}",
            stranded.len(),
            stranded
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join("\n  ")
        );
        // …and the lane must actually narrow, or the assertion above would hold
        // for the uninteresting reason that everything is built.
        assert!(
            built.len() < modelled.len(),
            "{lane}: the west lane built {} of {} leaves — it is not narrowing, \
             so this test proves nothing about skippability",
            built.len(),
            modelled.len()
        );
    }

    // …and the harvest this test and `just check fast` share must itself be
    // able to fail. A source-scraping check whose regex stopped matching prints
    // the same clean line forever — and the arm regex DID miss a mutation once
    // (an `rmw` bound of `[a-z]+` against a `"cyclonedds-a9"` arm).
    let selftest = Command::new("python3")
        .arg("scripts/check-west-leaf-vocabulary.py")
        .arg("--selftest")
        .current_dir(&repo)
        .output()
        .expect("run check-west-leaf-vocabulary.py --selftest");
    assert!(
        selftest.status.success(),
        "check-west-leaf-vocabulary.py --selftest failed — the harvest above is \
         not evidence:\n{}{}",
        String::from_utf8_lossy(&selftest.stdout),
        String::from_utf8_lossy(&selftest.stderr)
    );
}

/// A row the RUN cannot skip must be in the lane's BUILD — issue 0828.
///
/// `--coords-from <lane>` narrows the build to a lane's cell cover, on the
/// premise that the run skips anything outside it at fixture-resolution time.
/// That premise holds only for rows the resolver can ATTRIBUTE: a row whose
/// artifact root is shared by siblings is ambiguous, and `row_artifact_root`'s
/// own contract is to treat an ambiguous match as "not attributable" — fail
/// closed, never skip. Such a row runs at every lane.
///
/// Before the fix, `lane=tier2` built 114 rows and ran 142; the 28-row gap was
/// invisible on any machine carrying `lane=all` residue — green there, ~190
/// stale-fixture failures on a machine that had only ever run tier 2.
///
/// Checks MEMBERSHIP, not counts. The first version of this test asserted
/// `built >= unskippable`, which is 114 >= 28 — true with the fix reverted, so
/// it passed the mutation it existed to catch.
#[test]
fn every_unskippable_row_is_in_its_lane_build() {
    let repo = nros_tests::project_root();
    for lane in ["tier1", "tier2", "tier2-nightly"] {
        let coords = lane_coords_file(lane);
        // The whole comparison happens in the manifest's own module, so this
        // test cannot drift from the predicate it checks.
        let probe = Command::new("python3")
            .arg("-c")
            .arg(
                r#"
import importlib.util, sys, subprocess
spec = importlib.util.spec_from_file_location("fm", "scripts/build/fixtures-manifest.py")
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
rows = m.load("examples/fixtures.toml")
built = subprocess.run(
    [sys.executable, "scripts/build/fixtures-manifest.py", "list", "--coords-from", sys.argv[1]],
    capture_output=True, text=True, check=True).stdout
built_dirs = {l.split("\x1f")[0] for l in built.splitlines() if l.strip()}
missing = [
    (e.get("id") or e.get("dir"))
    for e in rows
    if not m.row_is_lane_skippable(e, rows) and (e.get("dir") or "").rstrip("/") not in built_dirs
]
print(len(missing))
for x in missing[:8]:
    print(x)
"#,
            )
            .arg(&coords)
            .current_dir(&repo)
            .output()
            .expect("probe the manifest");
        assert!(
            probe.status.success(),
            "{lane}: probe failed:\n{}",
            String::from_utf8_lossy(&probe.stderr)
        );
        let text = String::from_utf8_lossy(&probe.stdout);
        let mut lines = text.lines();
        let missing: usize = lines.next().unwrap_or("0").trim().parse().unwrap_or(0);
        assert_eq!(
            missing,
            0,
            "{lane}: {missing} row(s) cannot be skipped by any run yet are absent \
             from this lane's build — each will fail on staleness the lane never \
             promised (issue 0828). First few:\n  {}",
            lines.collect::<Vec<_>>().join("\n  ")
        );
    }
}
