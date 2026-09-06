//! Coordinate-scoped test RUN narrowing — phase-340 W3, closing issue 0482.
//!
//! # The problem this solves
//!
//! A CI lane answers two questions with different answers (issue 0482): which
//! fixtures must be **FRESH** (its cell cover — narrow, and the whole saving)
//! and which must **EXIST** (a property of the RUN). Tier 2 was honest about
//! that only by giving up: `ci-matrix` executed the entire suite, so every
//! coordinate's fixtures had to exist, so the middle rung of the ladder cost
//! the top rung's fixture build. CLAUDE.md's own argument says an unaffordable
//! instruction gets followed selectively, "which is worse than a smaller
//! instruction followed honestly".
//!
//! Making the run narrow needs a test→coordinate mapping. Issue 0482 recorded
//! two candidate designs and why neither worked:
//!
//! * **`scripts/test/lane-filter.sh`** selects by platform-family TOKEN. Tier 2
//!   is 1-wise over platform, so every platform appears in it and platform-level
//!   filtering excludes nothing; the saving is in lang × rmw *within* a
//!   platform, which test names do not encode (issue 0357 settled that binary /
//!   test-name filtering cannot express this).
//! * **the fixture resolver** is where the test↔fixture binding physically
//!   exists — but it "identifies fixtures by PATH across ~30 hand-written
//!   functions, with no link back to the `fixtures.toml` row".
//!
//! # What changed
//!
//! The second objection is about the ~30 functions, and it dissolves once the
//! link is DERIVED instead of hand-written. Every one of those functions
//! computes a path under the manifest row's own artifact root — which is not a
//! coincidence but a requirement, because that is where the build writes. So
//! `fixtures-manifest.py` exports `row_artifact_root()` beside `row_coord()`
//! and this module inverts it: **resolved path → manifest row → coordinate**.
//! No per-resolver edit, no second list to drift (the #328 shape), and the
//! coordinate is `row_coord`'s — the same function `matches_filters` uses to
//! decide whether to BUILD the row.
//!
//! Measured over the manifest as shipped: all 240 buildable `[[fixture]]` rows
//! have DISTINCT artifact roots, so the inversion is exact, not heuristic.
//! [`crate::fixtures::lane`]'s round-trip gate (`tests/lane_run_narrowing.rs`)
//! asserts it stays that way.
//!
//! Workspace rows are attributed by `id` instead: both workspace resolvers take
//! the `fixture_id` already, and several rows legitimately share one `dir`.
//!
//! # One computation, stated precisely
//!
//! ```text
//! BUILD  skips row R  ⟺  row_coord(R) ∉ lane_coords     (fixtures-manifest.py --coords-from)
//! RUN    skips row R  ⟺  row_coord(R) ∉ lane_coords     (this module)
//! ```
//!
//! Same predicate, same `row_coord`, same coordinate file. That is the
//! non-negotiable issue 0482 leaves behind: build-set and run-set must derive
//! from ONE computation, because two derivations of one coordinate drift
//! silently — which is exactly how 67 of 240 rows ended up in no lane at all.
//!
//! # Why this cannot launder "never built" into "skipped" (issue 0445)
//!
//! The skip is keyed on **"this row's coordinate is outside the lane"** and
//! never on "the artifact is missing":
//!
//! * A coordinate INSIDE the lane never skips. A missing or stale in-lane
//!   fixture fails exactly as hard as it did before — [`require_in_lane`] runs
//!   before the existence check and returns `Ok` for it.
//! * A path that attributes to NO row never skips (fail closed). Families built
//!   module-level rather than by coordinate — the compile-check lane — keep
//!   today's hard failure, which is correct: their build is not narrowed
//!   either, so nothing is missing.
//!
//!   The Zephyr west leaves were in that list and no longer belong there
//!   (issue 0713). They are not unattributable in principle — they have
//!   manifest rows — only unattributable BY PATH, because west writes into the
//!   Zephyr build root rather than under `row_artifact_root`. And the premise
//!   stopped holding when phase-350 W1.b narrowed the zephyr build by
//!   coordinate: their build IS narrowed, so a fail-closed resolve fails on an
//!   image the lane deliberately never built. They take the coordinate route
//!   instead, via [`require_west_leaf_in_lane`] keyed on the build-dir name
//!   both halves already agree on.
//! * With `NROS_TEST_COORDS` unset there is no narrowing at all and this module
//!   does no work (it does not even read the manifest).
//! * A `NROS_TEST_COORDS` that names a missing or empty file is a HARD ERROR,
//!   not "no narrowing" — an empty selection would silently skip the whole
//!   suite and report green. Same refusal `_coords_for` makes in
//!   `fixtures-manifest.py` and `nros_lane_coords_file` makes in
//!   `fixture-lane.sh`.

use std::{
    collections::BTreeSet,
    path::{Path, PathBuf},
    sync::OnceLock,
};

use crate::{TestResult, project_root};

/// The env var naming the coordinate file that scopes this RUN.
///
/// Set by `just ci-matrix` / `ci-matrix-nightly` to the SAME
/// `nros_lane_coords_file` output that scoped the lane's fixture build and its
/// staleness gate. Deliberately a distinct name from the build-side
/// `NROS_FIXTURE_COORDS`: that one is exported into fixture BUILD scripts and a
/// test process must not inherit a build's narrowing by accident. They are
/// bound by `CiLane::run_scope`, not by sharing a spelling.
pub const RUN_COORDS_ENV: &str = "NROS_TEST_COORDS";

/// A fixture coordinate, in the spelling `examples/fixtures.toml` uses.
pub type Coord = (String, String, String);

/// One buildable row of a coordinate-bearing manifest table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    /// `fixture` or `workspace_fixture`.
    pub kind: String,
    pub coord: Coord,
    pub dir: String,
    pub id: String,
    /// Repo-relative dir the row's artifacts land in (`row_artifact_root`).
    pub artifact_root: String,
}

impl Row {
    /// `platform,lang,rmw` — the line format of a lane-coords file.
    pub fn coord_str(&self) -> String {
        format!("{},{},{}", self.coord.0, self.coord.1, self.coord.2)
    }

    /// The most useful name for a human: workspace rows are known by `id`,
    /// plain rows by `dir`.
    pub fn label(&self) -> &str {
        if self.id.is_empty() {
            &self.dir
        } else {
            &self.id
        }
    }
}

static ROWS: OnceLock<Vec<Row>> = OnceLock::new();
static RUN_COORDS: OnceLock<Option<BTreeSet<Coord>>> = OnceLock::new();

/// Every buildable row of both coordinate-bearing tables, from
/// `fixtures-manifest.py coords`.
///
/// Shelling into the manifest reader rather than parsing `fixtures.toml` here
/// is the point: `row_coord` and `row_artifact_root` stay the single
/// computations. `nros-tests` already shells into this script for
/// `current_workspace_fixture_record`, so the precedent and the cost are known.
/// Read at most once per test process, and only when a lane actually narrows
/// the run.
pub fn manifest_rows() -> &'static [Row] {
    ROWS.get_or_init(|| {
        let root = project_root();
        let out = std::process::Command::new("python3")
            .arg(root.join("scripts/build/fixtures-manifest.py"))
            .arg("coords")
            .current_dir(&root)
            .output();
        let out = match out {
            Ok(o) if o.status.success() => o,
            // A manifest we cannot read must not silently mean "attribute
            // nothing", because that reads as "nothing is out of lane" and the
            // run then hard-fails on fixtures the lane deliberately skipped —
            // the confusing half of 0482, back again. Panic naming the cause.
            Ok(o) => panic!(
                "fixtures-manifest.py coords failed ({}): {}",
                o.status,
                String::from_utf8_lossy(&o.stderr)
            ),
            Err(e) => panic!("could not run fixtures-manifest.py coords: {e}"),
        };
        let text = String::from_utf8_lossy(&out.stdout).into_owned();
        let mut rows = Vec::new();
        for line in text.lines().filter(|l| !l.is_empty()) {
            let f: Vec<&str> = line.split('\x1f').collect();
            assert_eq!(
                f.len(),
                8,
                "unexpected `coords` record shape (expected 8 \\x1f-separated \
                 fields: kind, platform, lang, rmw, dir, id, artifact_root, \
                 builder): {line:?}"
            );
            // phase-350 W1 — `west` rows are OUT of the coordinate-narrowed run
            // set, because they are out of the coordinate-narrowed BUILD set:
            // `just zephyr build-fixtures` selects leaves module-level, not by
            // coordinate (the `NROS_FIXTURE_COORDS` plumbing stops at the
            // platform fan-out). This module's whole contract is that the two
            // sides skip the SAME rows, so a row the build cannot omit must not
            // be one the run does.
            //
            // phase-350 W1.b flips them in: once the zephyr lane narrows by
            // coordinate, dropping this filter makes
            // `every_omittable_row_is_attributable_from_its_artifacts` demand a
            // real `row_artifact_root` for them — which is W1.d, and is the
            // right ordering pressure rather than an accident.
            if f[7] == "west" {
                continue;
            }
            rows.push(Row {
                kind: f[0].to_string(),
                coord: (f[1].to_string(), f[2].to_string(), f[3].to_string()),
                dir: f[4].to_string(),
                id: f[5].to_string(),
                artifact_root: f[6].to_string(),
            });
        }
        rows
    })
}

/// One `builder = "west"` leaf: its west build-dir NAME and its coordinate.
///
/// phase-350 W1.b. West leaves reach the lane by COORDINATE, never by path —
/// the same route issue 0517's multi-row leaves take, and for a stronger
/// reason: their artifacts do not land under the row's `dir` at all, but in the
/// Zephyr WORKSPACE, whose root is a host fact no manifest can name. So the key
/// is the build-dir name, which the BUILD and the RUN both already have.
#[derive(Debug, Clone)]
pub struct WestLeaf {
    pub build_name: String,
    pub coord: Coord,
    pub dir: String,
}

static WEST_LEAVES: std::sync::OnceLock<Vec<WestLeaf>> = std::sync::OnceLock::new();

/// Every west leaf, from `fixtures-manifest.py west-leaves`.
pub fn west_leaves() -> &'static [WestLeaf] {
    WEST_LEAVES.get_or_init(|| {
        let root = project_root();
        let out = std::process::Command::new("python3")
            .arg(root.join("scripts/build/fixtures-manifest.py"))
            .arg("west-leaves")
            .current_dir(&root)
            .output();
        let out = match out {
            Ok(o) if o.status.success() => o,
            Ok(o) => panic!(
                "fixtures-manifest.py west-leaves failed ({}): {}",
                o.status,
                String::from_utf8_lossy(&o.stderr)
            ),
            Err(e) => panic!("could not run fixtures-manifest.py west-leaves: {e}"),
        };
        let text = String::from_utf8_lossy(&out.stdout).into_owned();
        text.lines()
            .filter(|l| !l.is_empty())
            .map(parse_west_leaf)
            .collect()
    })
}

/// One `west-leaves` record → a [`WestLeaf`].
///
/// Split out of [`west_leaves`] so the property that matters can be tested on
/// an input the shipped manifest does not contain — a record whose BOARD and
/// whose COORDINATE disagree. Any implementation that derives the coordinate
/// from the board answers differently on such a record, which is the whole
/// point: this side is the RUN's skip predicate and `--coords-from` is the
/// BUILD's, and the two must be one computation.
fn parse_west_leaf(line: &str) -> WestLeaf {
    let f: Vec<&str> = line.split('\x1f').collect();
    // 16 since issue 1016 appended `coord` (phase-383 W9.b appended `ws_dir`
    // and `nros_image` before it). Kept as an exact count rather than a lower
    // bound: this assertion is the only thing that notices when the emitter and
    // its readers drift, and it is what caught both additions.
    assert_eq!(
        f.len(),
        16,
        "unexpected `west-leaves` record shape (expected 16 \\x1f-separated \
         fields): {line:?}"
    );
    // board, lang, lang_tag, rmw, role, dir, build_name, id, …, coord
    //
    // issue 1016 — the coordinate is READ, not re-derived. This used to rebuild
    // the triple from `board` and the rmw LABEL with two local special cases
    // (`mps2_an385` → `zephyr-cortex-m`, so the witness leaves do not share
    // their native_sim siblings' coordinate; `default` → `zenoh`, which is how
    // logging-smoke spells its absent rmw). Both were faithful mirrors of
    // `row_coord` and nothing held them there — a drift would be a leaf the
    // lane omits and the run demands, i.e. issue 0828's shape for the west
    // table, arriving as a STALE verdict nobody can distinguish from a failure.
    let coord: Vec<&str> = f[15].split(',').collect();
    assert_eq!(
        coord.len(),
        3,
        "west leaf {:?}: `coord` must be `platform,lang,rmw`, got {:?}",
        f[6],
        f[15]
    );
    WestLeaf {
        build_name: f[6].to_string(),
        coord: (
            coord[0].to_string(),
            coord[1].to_string(),
            coord[2].to_string(),
        ),
        dir: f[5].to_string(),
    }
}

/// The west build-directory name an image was linked in, i.e. the `<name>` of
/// `<zephyr-build-root>/<name>/zephyr/zephyr.{exe,elf}`.
///
/// That shape is not a convention this function imposes — it is where west
/// writes, so every zephyr resolver already spells it, and it is the same
/// `build_name` `fixtures-manifest.py west-leaves` keys its coordinate on.
/// `None` for a path of any other shape, which [`require_west_leaf_in_lane`]
/// treats as fail-closed: run it.
///
/// Lives HERE rather than in `fixtures::binaries` (issue 1016) because both of
/// its consumers ask a lane question — "which coordinate is this image?" — and
/// a second spelling of the shape is how the two answers would drift.
pub fn west_build_name(zephyr_exe: &Path) -> Option<&str> {
    let dir = zephyr_exe.parent()?;
    if dir.file_name()? != "zephyr" {
        return None;
    }
    dir.parent()?.file_name()?.to_str()
}

/// Skip when the west leaf built into `build_name` is outside this run's lane.
///
/// phase-350 W1.b closed the loop the build side had opened: the zephyr lane
/// now narrows by coordinate, so a lane build legitimately omits leaves — and
/// without this the run would resolve one of those and fail on a fixture it had
/// deliberately not built. That asymmetry is what `lane_run_narrowing` gates
/// against for every other family.
///
/// Unknown `build_name` is NOT a skip: an unrecognised leaf is a resolver the
/// manifest does not model, and guessing "out of lane" there would silently
/// stop running it. Fail-closed means run it.
///
/// Failing open is the right default and it is also a hole with no floor
/// (issue 1016). A build-dir name absent from the manifest is, by construction,
/// emitted by NO lane's build and skippable by NO lane's run, so its verdict is
/// always a missing-or-stale message — which reads character-for-character like
/// a cell that ran and failed. The floor is `check-west-leaf-vocabulary.py`.
pub fn require_west_leaf_in_lane(build_name: &str, label: &str) -> TestResult<()> {
    if run_coords().is_none() {
        return Ok(());
    }
    let Some(leaf) = west_leaves().iter().find(|l| l.build_name == build_name) else {
        return Ok(());
    };
    require_coord_in_lane(&leaf.coord, label)
}

/// Parse a lane-coords file body. Same format and same refusals as
/// `_coords_for` in `fixtures-manifest.py` — one `platform,lang,rmw` per line,
/// `#` comments allowed, and an EMPTY selection is refused rather than treated
/// as "select nothing", which would report a green suite that ran no fixture at
/// all.
///
/// Split out of [`run_coords`] so the refusals are testable without an
/// environment variable and a `OnceLock` that has already latched.
pub fn parse_coords(text: &str, label: &str) -> BTreeSet<Coord> {
    let coords: BTreeSet<Coord> = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(|l| {
            let p: Vec<&str> = l.split(',').map(str::trim).collect();
            assert_eq!(
                p.len(),
                3,
                "{label}: expected `platform,lang,rmw`, got {l:?}"
            );
            (p[0].to_string(), p[1].to_string(), p[2].to_string())
        })
        .collect();
    assert!(
        !coords.is_empty(),
        "{label}: no coordinates — refusing to narrow the run to nothing (every \
         fixture would report as out-of-lane and the suite would report green \
         having run nothing)"
    );
    coords
}

/// The coordinate set this RUN is scoped to, or `None` when it is not scoped.
///
/// Panics on a `NROS_TEST_COORDS` that names nothing readable or selects
/// nothing — see the module docs.
pub fn run_coords() -> Option<&'static BTreeSet<Coord>> {
    RUN_COORDS
        .get_or_init(|| {
            let raw = std::env::var_os(RUN_COORDS_ENV)?;
            let raw = PathBuf::from(raw);
            if raw.as_os_str().is_empty() {
                return None;
            }
            let path = if raw.is_absolute() {
                raw
            } else {
                project_root().join(raw)
            };
            let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
                panic!(
                    "{RUN_COORDS_ENV}={} is unreadable ({e}). It must name the \
                     lane-coords file that scoped this run's fixture build; an \
                     unreadable one would silently un-narrow the run.",
                    path.display()
                )
            });
            Some(parse_coords(&text, &path.display().to_string()))
        })
        .as_ref()
}

/// The manifest row a resolved fixture artifact belongs to, or `None`.
///
/// Matches the LONGEST `artifact_root` that is a path-component prefix of
/// `path`. Component-wise on purpose: a textual prefix would let
/// `…/talker/target` claim `…/talker/target-xrce`, which is a DIFFERENT row at
/// a different coordinate.
///
/// `None` for anything outside the manifest's artifact roots — the shared
/// `build/cargo-fixtures/<platform>` dirs (phase-226.D), the Zephyr west build
/// roots, the compile-check lane. Callers must treat `None` as "do not skip".
///
/// Also `None` when the longest match is AMBIGUOUS — see [`attribute_path_in`].
pub fn attribute_path(path: &Path) -> Option<&'static Row> {
    let root = project_root();
    let rel = path.strip_prefix(&root).unwrap_or(path);
    attribute_path_in(manifest_rows(), rel)
}

/// [`attribute_path`] over an explicit row set, so the rule can be tested on
/// inputs the shipped manifest does not contain.
///
/// # Ambiguity is `None`, not "the first one"
///
/// issue 0517 — the longest-match rule assumed at most one row per artifact
/// root, which holds today: measured over the manifest, 0 `kind = "fixture"`
/// roots are shared by more than one row and 0 map to more than one coordinate.
/// It holds because the authored `target_dir` column separates the variants, so
/// `(dir, target_dir)` is effectively the row id — and phase-340 W2.d wants that
/// column gone, which would collapse 52 roots onto shared prefixes.
///
/// Picking the first of several equally-long matches would then attribute an
/// artifact to a row that did not produce it, and the consequence is not a
/// missing file: the run would skip or run the wrong cell against a real binary
/// built with different features. So a tie between rows at DIFFERENT
/// coordinates fails closed, and the caller's existing contract for `None` —
/// never skip — makes that loud rather than silent.
///
/// A tie between rows at the SAME coordinate is not ambiguous for this
/// question: the answer this function exists to give is the coordinate.
/// (`[[workspace_fixture]]` rows are excluded before the tie can arise; several
/// of them legitimately share one build dir and are resolved by `id` instead —
/// see [`attribute_workspace_id`].)
pub fn attribute_path_in<'a>(rows: &'a [Row], rel: &Path) -> Option<&'a Row> {
    let mut best: Option<&'a Row> = None;
    let mut ambiguous = false;
    for row in rows {
        if row.kind != "fixture" || row.artifact_root.is_empty() {
            continue;
        }
        if !path_under(rel, Path::new(&row.artifact_root)) {
            continue;
        }
        match best {
            Some(b) if row.artifact_root.len() > b.artifact_root.len() => {
                best = Some(row);
                ambiguous = false;
            }
            Some(b) if row.artifact_root.len() == b.artifact_root.len() => {
                ambiguous |= row.coord != b.coord;
            }
            Some(_) => {}
            None => best = Some(row),
        }
    }
    if ambiguous { None } else { best }
}

/// Is `path` inside `dir`, comparing whole path COMPONENTS?
///
/// A textual `starts_with` would say `examples/x/target` contains
/// `examples/x/target-zenoh/bin` — a different row at a different coordinate, or
/// (when the variant row is `skip_build` and therefore absent from the manifest)
/// no row at all. Either way a textual rule attributes an artifact to a row that
/// did not produce it, and the run then skips or runs the wrong cell.
///
/// Split out and given synthetic inputs in the tests below on purpose: against
/// the real manifest the longest-match rule happens to hide the difference, so a
/// test that only exercised `attribute_path` would pass under both rules and
/// gate nothing.
///
/// `pub(crate)` for [`crate::fixtures::groups`], which inverts `artifact_root`
/// for a different question (which shared cargo group did the build redirect
/// this row into?) and must answer containment identically — the two
/// inversions disagreeing is the same defect in two places.
pub(crate) fn path_under(path: &Path, dir: &Path) -> bool {
    let p: Vec<_> = path.components().collect();
    let d: Vec<_> = dir.components().collect();
    p.len() >= d.len() && p[..d.len()] == d[..]
}

/// The `[[workspace_fixture]]` row with this `id`, or `None`.
pub fn attribute_workspace_id(fixture_id: &str) -> Option<&'static Row> {
    manifest_rows()
        .iter()
        .find(|r| r.kind == "workspace_fixture" && r.id == fixture_id)
}

/// Whether `row` is inside `coords` — THE run-side selection predicate.
///
/// Exposed (and taking the coordinate set explicitly) so a gate can compare it
/// against `fixtures-manifest.py list --coords-from`, which is the build-side
/// selection, without an environment variable or a fixture. That comparison is
/// the invariant this module exists for; see `tests/lane_run_narrowing.rs`.
pub fn is_in_lane(row: &Row, coords: &BTreeSet<Coord>) -> bool {
    coords.contains(&row.coord)
}

/// Skip when a coordinate the caller ALREADY resolved is outside this run's
/// lane — issue 0517 step 1.
///
/// The row-keyed twin of [`require_in_lane`]. A resolver that selected its row
/// by configuration knows the coordinate outright, and making it hand back a
/// path so `attribute_path` can re-derive that same row is what forces a
/// per-variant LEAF directory to exist at all. The bytes have not lived in one
/// since phase-340 B3 — all 124 cargo rows build into
/// `build/cargo-fixtures/<slug>` — so the leaf path was serving purely as a key,
/// and this is that key without the path.
///
/// `label` is for the message only; pass the row's dir.
pub fn require_coord_in_lane(coord: &Coord, label: &str) -> TestResult<()> {
    let Some(coords) = run_coords() else {
        return Ok(());
    };
    if let Some(reason) = skip_reason_for_coord(coord, label, coords) {
        crate::skip_class!(lane, "{reason}");
    }
    Ok(())
}

/// [`skip_reason_for_path`] for a coordinate the caller already resolved — the
/// decision behind [`require_coord_in_lane`], minus the environment.
///
/// Split out for the reason the other two are: it is the ONLY way to exercise
/// this arm in both directions without latching a process-wide `OnceLock`. The
/// arm needs its own coverage because it is the route every MULTI-row leaf now
/// takes — issue 0517 step 3 made those leaves unattributable by path, so
/// `skip_reason_for_path` cannot answer for them and a lane gate written only
/// against paths silently stops covering them (which is how the same step left
/// `bins/int32-sink`'s three rows unskippable).
pub fn skip_reason_for_coord(
    coord: &Coord,
    label: &str,
    coords: &BTreeSet<Coord>,
) -> Option<String> {
    (!coords.contains(coord)).then(|| out_of_lane_coord(label, coord))
}

fn out_of_lane_coord(label: &str, coord: &Coord) -> String {
    format!(
        "out of lane: {} is at coordinate {},{},{}, which this run's lane does not \
         select, so `just build-test-fixtures lane=<this lane>` deliberately did \
         not build it.\n  \
         This is NOT a staleness or missing-fixture verdict: an in-lane fixture \
         that is absent or stale still fails hard.\n  \
         Run the full ladder (`just ci-full`) or unset {RUN_COORDS_ENV} to \
         execute this coordinate.",
        label, coord.0, coord.1, coord.2,
    )
}

fn out_of_lane(row: &Row) -> String {
    // Deliberately NOT phrased as (or routed through) a staleness verdict.
    // Issue 0445's lesson is that an absorbing message must account for itself:
    // this one names the row, its coordinate, and the fact that the build was
    // told to omit it — so a reader can tell "the lane excluded this" from "the
    // build was supposed to produce this and did not".
    format!(
        "out of lane: {} is at coordinate {}, which this run's lane does not \
         select, so `just build-test-fixtures lane=<this lane>` deliberately did \
         not build it.\n  \
         This is NOT a staleness or missing-fixture verdict: an in-lane fixture \
         that is absent or stale still fails hard.\n  \
         Run the full ladder (`just ci-full`) or unset {RUN_COORDS_ENV} to \
         execute this coordinate.",
        row.label(),
        row.coord_str(),
    )
}

/// Skip when `binary_path` belongs to a manifest row outside this run's lane.
///
/// Called at the fixture-resolution chokepoint, BEFORE the existence check, so
/// a coordinate the lane never built reports as skipped rather than as a
/// missing binary. Returns `Ok(())` — i.e. carry on and apply every existing
/// check — when the run is not narrowed, or the path attributes to no row, or
/// the row is in lane.
pub fn require_in_lane(binary_path: &Path) -> TestResult<()> {
    let Some(coords) = run_coords() else {
        return Ok(());
    };
    if let Some(reason) = skip_reason_for_path(binary_path, coords) {
        crate::skip_class!(lane, "{reason}");
    }
    Ok(())
}

/// [`require_in_lane`] for a `[[workspace_fixture]]`, attributed by `id`.
pub fn require_workspace_in_lane(fixture_id: &str) -> TestResult<()> {
    let Some(coords) = run_coords() else {
        return Ok(());
    };
    if let Some(reason) = skip_reason_for_workspace_id(fixture_id, coords) {
        crate::skip_class!(lane, "{reason}");
    }
    Ok(())
}

/// The whole decision, minus the environment: `Some(reason)` means skip.
///
/// Taking `coords` as an argument (rather than reading [`run_coords`]) is what
/// makes the decision testable in BOTH directions without an env var and a
/// `OnceLock` that latches for the life of the process. The `require_*`
/// wrappers add nothing but the lookup and the panic.
pub fn skip_reason_for_path(binary_path: &Path, coords: &BTreeSet<Coord>) -> Option<String> {
    let row = attribute_path(binary_path)?;
    (!is_in_lane(row, coords)).then(|| out_of_lane(row))
}

/// [`skip_reason_for_path`] for a `[[workspace_fixture]]` id.
pub fn skip_reason_for_workspace_id(fixture_id: &str, coords: &BTreeSet<Coord>) -> Option<String> {
    let row = attribute_workspace_id(fixture_id)?;
    (!is_in_lane(row, coords)).then(|| out_of_lane(row))
}

// ── What the BUILD recorded, as opposed to what the RUN was told (issue 1016) ─
//
// Everything above answers "is this coordinate in the RUN's lane?", read from
// `NROS_TEST_COORDS`. That is one half of a pair, and the other half is written
// down too: `nros_fixtures_stamp_write` records `lane=` plus one `coord=` line
// per coordinate the fixture BUILD was scoped to.
//
// Nothing in a test process read it, and that is the gap issue 1016 walked
// into. A `just build-test-fixtures lane=tier2` followed by a bare `cargo
// nextest` — which is exactly what CLAUDE.md's own triage advice tells you to
// type when re-running a red cell SOLO — has a narrowed BUILD and an
// un-narrowed RUN. `_require-fixtures` refuses that pairing, but only for
// `just test-all`; a bare nextest never reaches it. So every coordinate the
// lane deliberately skipped resolves, misses or reads stale, and reports a
// verdict about the ARTIFACT for a fact about the LANE.
//
// This is read ONLY on the failure path (see `require_prebuilt_binary_checks`
// and `staleness::stale_error`). It can therefore never turn a fixture that IS
// present and fresh into a skip — the issue-0445 hazard, and the reason it is
// not simply folded into `run_coords`.

/// The lane and coordinate set the last fixture BUILD recorded, or `None`.
///
/// `None` when there is no stamp, when it is a pre-0393 bare timestamp, or when
/// the build was not coordinate-scoped (`lane=all`, `lane=native` — the latter
/// is module-level and writes no `coord=` lines). "Cannot tell" is `None`, never
/// "nothing was built": absence of evidence must not become evidence.
pub fn recorded_build() -> Option<&'static (String, BTreeSet<Coord>)> {
    static STAMP: OnceLock<Option<(String, BTreeSet<Coord>)>> = OnceLock::new();
    STAMP
        .get_or_init(|| {
            let path = match std::env::var_os("NROS_FIXTURE_STAMP") {
                Some(p) => PathBuf::from(p),
                None => project_root().join("target/nextest/.fixtures-built"),
            };
            let path = if path.is_absolute() {
                path
            } else {
                project_root().join(path)
            };
            parse_stamp(&std::fs::read_to_string(path).ok()?)
        })
        .as_ref()
}

/// Parse a `.fixtures-built` stamp body. Split out so both directions are
/// testable without a build.
pub fn parse_stamp(text: &str) -> Option<(String, BTreeSet<Coord>)> {
    let mut lane = None;
    let mut coords = BTreeSet::new();
    for line in text.lines().map(str::trim) {
        if let Some(v) = line.strip_prefix("lane=") {
            lane = Some(v.to_string());
        } else if let Some(v) = line.strip_prefix("coord=") {
            let p: Vec<&str> = v.split(',').map(str::trim).collect();
            if p.len() == 3 {
                coords.insert((p[0].to_string(), p[1].to_string(), p[2].to_string()));
            }
        }
    }
    // No `coord=` lines means the build was not coordinate-scoped, so it makes
    // no claim this function can act on.
    (!coords.is_empty()).then(|| (lane.unwrap_or_else(|| "?".to_string()), coords))
}

/// Why the RECORDED fixture build never produced `binary_path`, or `None`.
///
/// Answers only for an artifact this module can place on a coordinate: a west
/// image by its build-dir name, anything else by [`attribute_path`]. An
/// unplaceable path gets `None` — fail closed, exactly as everywhere else here.
pub fn recorded_build_omits(binary_path: &Path) -> Option<String> {
    let (lane, built) = recorded_build()?;
    let coord = coord_of_artifact(binary_path)?;
    if built.contains(&coord) {
        return None;
    }
    Some(format!(
        "the recorded fixture build did NOT build this coordinate: the stamp says \
         lane={lane} ({} coordinate(s)) and this artifact is at {},{},{}.\n  \
         So this is a LANE fact, not a regression and not a staleness result — \
         nothing here ran, and whatever is on disk (if anything) is left over \
         from an earlier, wider build.\n  \
         Narrow the run to match (`just ci matrix`, which exports \
         {RUN_COORDS_ENV}), or widen the build (`just build-test-fixtures`).",
        built.len(),
        coord.0,
        coord.1,
        coord.2,
    ))
}

/// The coordinate an artifact sits at, by whichever route can place it.
fn coord_of_artifact(binary_path: &Path) -> Option<Coord> {
    if let Some(name) = west_build_name(binary_path)
        && let Some(leaf) = west_leaves().iter().find(|l| l.build_name == name)
    {
        return Some(leaf.coord.clone());
    }
    attribute_path(binary_path).map(|r| r.coord.clone())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sole_row_leaf_attributes_to_itself() {
        // The round trip, narrowed to what path attribution is still FOR
        // (issue 0517 step 3). Until the column was deleted every row had a
        // distinct artifact root, so any row's own root round-tripped. That
        // property came from `target_dir`, which was a directory invented to
        // carry a row's identity — and identity now travels as a selector or an
        // id, not as a path.
        //
        // A leaf with ONE row still round-trips, and must: the ~24 resolvers
        // that still spell a leaf `target/` inline all name such leaves, and
        // this is what keeps their lane narrowing working.
        let root = project_root();
        let mut per_dir: std::collections::HashMap<&str, usize> = Default::default();
        for row in manifest_rows().iter().filter(|r| r.kind == "fixture") {
            *per_dir.entry(row.dir.as_str()).or_default() += 1;
        }
        let mut wrong = Vec::new();
        for row in manifest_rows().iter().filter(|r| r.kind == "fixture") {
            if per_dir[row.dir.as_str()] > 1 {
                continue;
            }
            let probe = root.join(&row.artifact_root).join("some/binary");
            match attribute_path(&probe) {
                Some(got) if got.artifact_root == row.artifact_root => {}
                Some(got) => wrong.push(format!("{} -> {}", row.label(), got.label())),
                None => wrong.push(format!("{} -> <unattributable>", row.label())),
            }
        }
        assert!(
            wrong.is_empty(),
            "{} sole-row leaf/leaves do not attribute back to themselves:\n  {}",
            wrong.len(),
            wrong.join("\n  ")
        );
    }

    #[test]
    fn a_multi_row_leaf_is_not_attributable_by_path_and_that_is_the_point() {
        // The other half. Several rows of one leaf now share `<dir>/target`, so
        // a path cannot say which — and `attribute_path` fails closed rather
        // than guessing (phase A). Callers for these leaves go through
        // `groups::select_row` and ask the lane about the row's COORDINATE, so
        // no path is presented in the first place.
        //
        // If this ever starts passing, someone has re-introduced a per-variant
        // directory, and the question to ask is what it is carrying that the
        // selector does not.
        let root = project_root();
        let mut per_dir: std::collections::HashMap<&str, Vec<&Row>> = Default::default();
        for row in manifest_rows().iter().filter(|r| r.kind == "fixture") {
            per_dir.entry(row.dir.as_str()).or_default().push(row);
        }
        let multi: Vec<_> = per_dir.values().filter(|v| v.len() > 1).collect();
        assert!(
            !multi.is_empty(),
            "expected leaves with several rows (examples/native/rust/talker has five)"
        );
        for rows in multi {
            let roots: std::collections::BTreeSet<_> =
                rows.iter().map(|r| r.artifact_root.as_str()).collect();
            if roots.len() > 1 {
                continue; // a leaf whose rows still differ by dir (cmake rows)
            }
            let probe = root.join(rows[0].artifact_root.clone()).join("some/binary");
            let coords: std::collections::BTreeSet<_> = rows.iter().map(|r| &r.coord).collect();
            if coords.len() == 1 {
                continue; // same coordinate: not ambiguous for this question
            }
            assert!(
                attribute_path(&probe).is_none(),
                "{} has {} rows at one root and still attributed to one of them",
                rows[0].dir,
                rows.len()
            );
        }
    }

    #[test]
    fn a_path_outside_every_artifact_root_is_not_attributable() {
        // Fail closed. These are the families built module-level rather than by
        // coordinate; attributing them would be guessing, and guessing here
        // turns "never built" into "skipped" (issue 0445).
        for p in [
            "build/zephyr-workspace-builds/build-ws-c-entry-zenoh/zephyr/zephyr.exe",
            "build/cargo-fixtures/qemu-arm-baremetal/thumbv7m-none-eabi/x/y",
            "build/cmake-fixtures/some-id/bin",
            "packages/testing/nros-smoke/target/x/y",
        ] {
            assert!(
                attribute_path(&project_root().join(p)).is_none(),
                "{p} must not attribute to a manifest row"
            );
        }
    }

    #[test]
    fn containment_is_component_wise_not_textual() {
        // The falsifiable form: synthetic paths where the two rules DISAGREE.
        // Against the real manifest they do not, because longest-match happens
        // to pick the right row anyway — so a test driven by `attribute_path`
        // alone would pass under a textual rule and gate nothing.
        let t = Path::new("examples/x/target");
        assert!(path_under(Path::new("examples/x/target/rel/bin"), t));
        assert!(
            !path_under(Path::new("examples/x/target-zenoh/rel/bin"), t),
            "`target` must not swallow `target-zenoh`: that is a different row at \
             a different coordinate (and if its row is skip_build, no row at all)"
        );
        assert!(!path_under(Path::new("examples/xy/target/bin"), t));
        assert!(path_under(t, t), "a dir contains itself");
        assert!(!path_under(Path::new("examples/x"), t));
    }

    #[test]
    fn artifact_roots_are_pairwise_unnested() {
        // No row's artifact root may sit INSIDE another's. Nesting is still
        // forbidden after issue 0517 step 3, and for the original reason: a
        // nested root makes longest-match pick a row that did not produce the
        // artifact, silently.
        //
        // SHARING a root is now allowed and expected — several rows of one leaf
        // all answer `<dir>/target` since the `target_dir` column was deleted.
        // Those are told apart by selector, and `attribute_path` fails closed on
        // them (see `a_multi_row_leaf_is_not_attributable_by_path_and_that_is_
        // the_point`). Equality is therefore skipped here; containment is not.
        let roots: Vec<&Row> = manifest_rows()
            .iter()
            .filter(|r| r.kind == "fixture")
            .collect();
        let mut bad = Vec::new();
        for a in &roots {
            for b in &roots {
                if std::ptr::eq(*a, *b) {
                    continue;
                }
                if a.artifact_root == b.artifact_root {
                    continue;
                }
                if path_under(Path::new(&a.artifact_root), Path::new(&b.artifact_root)) {
                    bad.push(format!(
                        "{} ({}) is inside {} ({})",
                        a.label(),
                        a.artifact_root,
                        b.label(),
                        b.artifact_root
                    ));
                }
            }
        }
        assert!(
            bad.is_empty(),
            "{} nested artifact root(s):\n  {}",
            bad.len(),
            bad.join("\n  ")
        );
    }

    #[test]
    fn two_coordinates_at_one_artifact_root_are_not_attributable() {
        // issue 0517 — the twin of `groups::two_groups_at_one_artifact_root_do_
        // not_redirect`, and the same synthetic-input argument: today 0
        // `kind = "fixture"` roots are shared by more than one row (the
        // `artifact_roots_are_pairwise_unnested` gate above holds it), so the
        // real manifest cannot exercise the tie.
        //
        // What a wrong answer costs here is not a missing file. `attribute_path`
        // decides whether a cell is IN LANE, so the wrong row's coordinate either
        // skips a test that should have run — a false green, issue 0445's shape —
        // or hard-fails one the lane deliberately never built.
        let row = |root: &str, rmw: &str| Row {
            kind: "fixture".into(),
            coord: ("linux".into(), "rust".into(), rmw.into()),
            dir: "examples/native/rust/talker".into(),
            id: String::new(),
            artifact_root: root.into(),
        };
        let bin = Path::new("examples/native/rust/talker/target/x/talker");

        let tie = vec![
            row("examples/native/rust/talker/target", "zenoh"),
            row("examples/native/rust/talker/target", "xrce"),
        ];
        assert!(
            attribute_path_in(&tie, bin).is_none(),
            "two coordinates at one root must be unattributable; the caller's \
             contract for None is `do not skip`, so this fails loudly"
        );

        // Same root, SAME coordinate: not ambiguous. The coordinate IS the
        // answer, and both rows give it.
        let same = vec![
            row("examples/native/rust/talker/target", "zenoh"),
            row("examples/native/rust/talker/target", "zenoh"),
        ];
        assert_eq!(
            attribute_path_in(&same, bin).map(|r| r.coord_str()),
            Some("linux,rust,zenoh".to_string())
        );

        // A longer match still wins outright, and clears an earlier tie.
        let mut deeper = tie.clone();
        deeper.push(row("examples/native/rust/talker/target/x", "cyclonedds"));
        assert_eq!(
            attribute_path_in(&deeper, bin).map(|r| r.coord_str()),
            Some("linux,rust,cyclonedds".to_string()),
            "the deeper root is unambiguous — a tie among SHORTER roots must not \
             make longest-match give up"
        );

        // A workspace row sharing the root is not a tie: those are attributed by
        // `id` and skipped here. Three real rows do share a root today
        // (examples/workspaces/{mixed,features,safety}) and must stay harmless.
        let mut ws = vec![row("examples/native/rust/talker/target", "zenoh")];
        ws.push(Row {
            kind: "workspace_fixture".into(),
            ..row("examples/native/rust/talker/target", "xrce")
        });
        assert_eq!(
            attribute_path_in(&ws, bin).map(|r| r.coord_str()),
            Some("linux,rust,zenoh".to_string())
        );
    }

    #[test]
    #[should_panic(expected = "refusing to narrow the run to nothing")]
    fn an_empty_coordinate_file_is_refused_not_read_as_no_narrowing() {
        // The dangerous silent reading: an empty file means "nothing is in
        // lane", so EVERY fixture skips and the suite reports green having run
        // nothing. `_coords_for` in fixtures-manifest.py refuses the same thing
        // for the same reason.
        parse_coords("\n# only a comment\n\n", "<test>");
    }

    #[test]
    fn a_coordinate_file_parses_to_row_coord_spelling() {
        let coords = parse_coords("linux,rust,zenoh\n# c\n nuttx,c,zenoh \n", "<test>");
        assert!(coords.contains(&("linux".to_string(), "rust".to_string(), "zenoh".to_string())));
        assert!(coords.contains(&("nuttx".to_string(), "c".to_string(), "zenoh".to_string())));
        assert_eq!(coords.len(), 2);
    }

    /// A west leaf's coordinate is READ from the record, never re-derived — issue 1016.
    ///
    /// The record below is deliberately self-contradictory: its BOARD is
    /// `mps2_an385` and its rmw LABEL is `default`, the two inputs the old
    /// derivation special-cased, while its `coord` column says
    /// `zephyr,rust,xrce`. Anything that computes the triple from board+label
    /// answers `("zephyr-cortex-m", "rust", "zenoh")` here; only reading the
    /// column gives the manifest's answer.
    ///
    /// That is the property, not a formatting detail: this side is the RUN's
    /// skip predicate and `fixtures-manifest.py --coords-from` is the BUILD's.
    /// A leaf the two disagree about is one the lane omits and the run demands,
    /// which surfaces as `BuildFailed("Zephyr fixture is STALE …")` — the
    /// verdict issue 0968 read six times as a failure.
    #[test]
    fn a_west_leaf_takes_its_coordinate_from_the_manifest_not_from_its_board() {
        let mut f = vec![""; 16];
        f[0] = "mps2_an385"; // board — the old platform special case
        f[1] = "rust";
        f[3] = "default"; // rmw LABEL — the old rmw special case
        f[5] = "examples/zephyr/rust/talker";
        f[6] = "build-rust-talker-xrce";
        f[15] = "zephyr,rust,xrce"; // …and what `row_coord` actually says
        let leaf = parse_west_leaf(&f.join("\x1f"));
        assert_eq!(
            leaf.coord,
            ("zephyr".to_string(), "rust".to_string(), "xrce".to_string()),
            "the coordinate must come from the record's `coord` column; a \
             board-derived answer would be (zephyr-cortex-m, rust, zenoh)"
        );
        assert_eq!(leaf.build_name, "build-rust-talker-xrce");
        assert_eq!(leaf.dir, "examples/zephyr/rust/talker");
    }

    #[test]
    #[should_panic(expected = "expected 16")]
    fn a_west_leaf_record_of_the_wrong_width_is_refused() {
        // The emitter and this reader drifting apart silently would shift every
        // column; both prior appends were caught by this assertion.
        parse_west_leaf(&vec![""; 15].join("\x1f"));
    }

    #[test]
    fn an_unnarrowed_run_reads_no_manifest_and_skips_nothing() {
        // The zero-cost property: with no lane coords the resolver must not
        // even consult the manifest, and must never skip.
        if std::env::var_os(RUN_COORDS_ENV).is_some() {
            return; // this process IS narrowed; the property is untestable here
        }
        assert!(run_coords().is_none());
        assert!(
            require_in_lane(&project_root().join("examples/native/rust/talker/target/x/talker"))
                .is_ok()
        );
        assert!(require_workspace_in_lane("workspace-rust-native").is_ok());
    }

    // ── issue 1016 — what the BUILD recorded ────────────────────────────────

    #[test]
    fn a_stamp_without_coordinates_makes_no_claim() {
        // `lane=all` and `lane=native` write no `coord=` lines. Reading either
        // as "nothing was built" would turn every fixture into a lane skip.
        assert!(parse_stamp("lane=all\nbuilt_at=x\n").is_none());
        assert!(parse_stamp("").is_none());
        // A pre-0393 bare timestamp: no `lane=`, no coordinates.
        assert!(parse_stamp("2026-09-06T00:00:00Z\n").is_none());
    }

    #[test]
    fn a_coordinate_scoped_stamp_reads_back_its_lane_and_cover() {
        let (lane, coords) =
            parse_stamp("# comment\nlane=tier2\ncoord=zephyr,cpp,xrce\ncoord=linux,rust,zenoh\n")
                .expect("a stamp with coordinates makes a claim");
        assert_eq!(lane, "tier2");
        assert_eq!(coords.len(), 2);
        assert!(coords.contains(&("zephyr".to_string(), "cpp".to_string(), "xrce".to_string())));
    }

    /// The issue-1016 reading, on the exact coordinates it was measured at.
    ///
    /// `lane-coords tier2` selects `zephyr,cpp,xrce` and nothing else under
    /// `zephyr`, so `build-c-listener-xrce` is a leaf that lane deliberately
    /// does not build — and the verdict the reporter saw said only "STALE".
    #[test]
    fn a_west_image_outside_the_recorded_cover_is_named_as_such() {
        let root = project_root();
        let in_cover = root.join("zephyr-workspace/build-cpp-listener-xrce/zephyr/zephyr.exe");
        let out_of_cover = root.join("zephyr-workspace/build-c-listener-xrce/zephyr/zephyr.exe");

        let (_, built) = parse_stamp("lane=tier2\ncoord=zephyr,cpp,xrce\n").expect("stamp");
        let placed = |p: &Path| coord_of_artifact(p).expect("a west image is placeable");
        assert!(
            built.contains(&placed(&in_cover)),
            "the cover's own coordinate must read as built"
        );
        assert!(
            !built.contains(&placed(&out_of_cover)),
            "zephyr,c,xrce is outside tier 2's cover — that is the measurement \
             issue 1016 rests on"
        );
    }

    #[test]
    fn an_unplaceable_path_makes_no_claim_either() {
        // Fail closed, like every other arm here: a path no table can place
        // must never be reported as "the lane did not build it".
        assert!(coord_of_artifact(Path::new("/nowhere/at/all/bin")).is_none());
    }
}
