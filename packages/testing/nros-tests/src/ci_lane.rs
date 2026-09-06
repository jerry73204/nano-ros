//! CI lane selection — RFC-0061 / phase-318 W3.
//!
//! Which cells a given CI lane runs, **computed from [`crate::matrix::CELLS`]**
//! rather than listed. A hand-maintained list is the thing this replaces: adding
//! a platform to the matrix must extend the lanes without a second edit, or the
//! lanes silently skip it (audit E5, issue 0341).
//!
//! Naming: [`crate::matrix::Tier`] already means Runtime / BuildOnly / CarveOut —
//! a property of a CELL. These are lanes a RUN selects, so they are [`CiLane`].
//!
//! # Strength per axis, not per lane
//!
//! Uniform t-wise is the wrong frame, because the axes select different kinds of
//! thing and so fail in different shapes (RFC-0061 §Selection strategy):
//!
//! | axis | selects | strength |
//! | --- | --- | --- |
//! | `workload` | which core CODE PATH runs | 1-wise — pairing it is waste |
//! | `lang` × `rmw` | which ABI SEAM PAIR meets | pairwise |
//! | `platform` | toolchain + libc + linker | pairwise with `lang` |
//! | `kind` | entry vs carrier WIRING | pairwise with `platform` |
//!
//! Derived from defects, not theory: the sizes/`_opaque` class (0268, 0245) and
//! the freestanding-header class (0332) are platform × lang; the vtable/transport
//! ABI class (0331) is rmw × lang; entry/carrier wiring (0097, 0263) is
//! platform × kind. Nothing in that catalogue needs workload × platform — an
//! action-path bug fails on every platform, which is why [`CiLane::Tier1`] covers
//! every workload once on native and the tier-2 lanes do not re-cover it.
//!
//! # Why tier 2 splits (2026-07-30)
//!
//! Cost is [`coords`], not cell count: cells share fixtures, and it is FIXTURES
//! that take hours to build. Measured 2026-07-30 against 182 runtime cells and
//! 48 coordinates; re-measured 2026-08-20 after phase-370 added the
//! `freertos-posix` board (two cells, its own platform token, so its own
//! coordinates):
//!
//! | lane | selection | cells | coords | cost |
//! | --- | --- | --- | --- | --- |
//! | [`CiLane::Tier1`] | host-exec, 1-wise p,w,k + pairwise l × r | 17 | 12 | 26 % |
//! | [`CiLane::Tier2`] | 1-wise p, l, r, k | 13 | 13 | 26 % |
//! | [`CiLane::Tier2Nightly`] | pairwise p × l × r × k | 36 | 36 | 72 % |
//! | tier 3 | everything | 194 | 50 | 100 % |
//!
//! **These numbers are GATED, not transcribed** — `documented_lane_table_is_live`
//! recomputes them and fails if this table drifts (phase-342 W3). They had:
//! the table said 11/12 for tier 2 and 37/33 for nightly while the code selected
//! 12/13 and 35/35, and three more spellings elsewhere disagreed with both.
//! RFC-0061 had already amended itself once over exactly this — it quoted tier 2
//! at "~20 % of a full sweep" counting CELLS when the cost unit is COORDINATES,
//! where the same cover is 70 %. A stale cost estimate is how the wrong tier gets
//! chosen.
//!
//! The pairwise cover reduces cells by 80 % and fixtures by only 30 %, and the
//! floor is structural: pairwise(platform × lang) needs one fixture per pair and
//! there are 29 declared pairs, so it cannot be tuned away. A middle tier costing
//! 70 % of the sweep is one nobody runs — the failure mode RFC-0061 exists to fix
//! — and pairwise(platform × lang) is also exactly the interaction the
//! 0268 / 0245 / 0332 class lives in, so it cannot simply be dropped either.
//!
//! Hence two lanes: [`CiLane::Tier2`] is 1-wise and affordable per change,
//! [`CiLane::Tier2Nightly`] is the pairwise cover on a nightly cadence. Cheap
//! enough to run, thorough enough to catch the class — a day later rather than
//! pre-merge. The nightly lane keeps rmw and kind in the pairing: over
//! pairwise(platform × lang) alone it costs ~4 more coordinates, and a lane that
//! is not on the critical path should not trade coverage that cheap.
//!
//! Note the ladder is not monotone in CELLS — tier 1 picks 17 and tier 2 picks 11
//! — because tier 1's cells are all native and a native fixture is nearly free.
//! Cell count is the wrong unit; `the_ladder_is_monotone_in_fixture_cost` asserts
//! the ordering in coordinates so that confusion cannot come back.
//!
//! # Why a set cover and not a covering array
//!
//! The axes are not independent — an RMW is not available on every platform — so
//! "pairwise" here means: choose a minimum subset of DECLARED cells such that
//! every (axis_i = a, axis_j = b) pair occurring in any declared cell occurs in
//! the chosen subset. Greedy is adequate (within a `ln n` factor; the input is a
//! couple of hundred cells), and ties break on the cell's debug rendering so the
//! chosen set is deterministic for a fixed table.

use crate::matrix::{Cell, PlatformId};

/// Every Runtime cell the tiers select over — baked cells (`matrix::CELLS`) AND
/// interop/bridge cells (`crate::interop::CELLS`), issue 0352 / phase-324.
///
/// `Kind::Interop` and `Kind::Bridge` are values of the `kind` axis the tiers
/// cover (tier 1 is 1-wise over kind), so moving the interop/bridge cells into
/// their own list must not remove them from the selection pool — or tier 1 would
/// silently stop covering the interop/bridge wiring it always did.
fn all_runtime_cells() -> impl Iterator<Item = &'static Cell> {
    crate::matrix::runtime_cells().chain(crate::interop::runtime_cells().map(|ic| &ic.cell))
}
use std::collections::BTreeSet;

/// A CI lane. See RFC-0061 for the ladder (tier 0 runs no cells at all, tier 3
/// runs everything, so neither needs a computed selection).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CiLane {
    /// Native only: every core code path once, and every language against every
    /// RMW. What `just ci` should mean — minutes, on the host, where a failure is
    /// cheap to debug.
    Tier1,
    /// 1-wise over platform × lang × rmw × kind: every declared value of each
    /// axis appears at least once, no pairing. What `just ci-matrix` runs — the
    /// per-change gate that has to be affordable enough to actually get run.
    Tier2,
    /// Pairwise over platform × lang × rmw × kind. What `just ci-matrix-nightly`
    /// runs — the interaction coverage tier 2 gives up in exchange for being
    /// cheap. See [`ALL`] for why the split exists.
    Tier2Nightly,
}

/// Every computed lane, so tests and tooling iterate rather than enumerate.
pub const ALL: [CiLane; 3] = [CiLane::Tier1, CiLane::Tier2, CiLane::Tier2Nightly];

/// How wide a lane's TEST RUN is.
///
/// Deliberately a separate type from [`CiLane`], because it answers a different
/// question. See [`CiLane::run_scope`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RunScope {
    /// The recipe filters the run to host binaries by NAME
    /// (`scripts/test/lane-filter.sh native`, via `NROS_TEST_SCOPE=native`).
    Native,
    /// The recipe runs the whole suite — no filtering of any kind.
    All,
    /// The recipe runs every test binary, but the fixture RESOLVER skips any
    /// fixture whose manifest row sits outside [`coords`] — phase-340 W3.
    ///
    /// Name-based filtering cannot express this (issue 0357 / 0482): tier 2 is
    /// 1-wise over platform, so every platform is in the lane and a
    /// platform-token filter excludes nothing, while the actual saving is in
    /// lang × rmw *within* a platform, which test names do not encode. The
    /// selection therefore happens where the test↔fixture binding physically
    /// exists — see `crate::fixtures::lane`, which derives it from the same
    /// `row_coord` the BUILD filters on.
    LaneCoords,
}

impl RunScope {
    /// The `NROS_TEST_SCOPE` token. `lane-filter.sh` takes exactly two values;
    /// [`RunScope::LaneCoords`] does not filter by NAME at all, so it maps to
    /// `all` here and does its narrowing at resolution time instead.
    pub fn test_scope(self) -> &'static str {
        match self {
            RunScope::Native => "native",
            RunScope::All | RunScope::LaneCoords => "all",
        }
    }
}

impl CiLane {
    /// The lane token `just build-test-fixtures lane=…` and
    /// `scripts/build/fixture-lane.sh` spell this lane with.
    pub fn lane_token(self) -> &'static str {
        match self {
            CiLane::Tier1 => "tier1",
            CiLane::Tier2 => "tier2",
            CiLane::Tier2Nightly => "tier2-nightly",
        }
    }

    /// The FIXTURE BUILD LANE that covers this lane's run — the `lane=` a
    /// `build-test-fixtures` must have been given for the run's fixtures to
    /// exist.
    ///
    /// Derived from [`CiLane::run_scope`], never declared twice:
    ///
    /// * [`RunScope::Native`] — a module-level build of every host row, which is
    ///   a strict superset of any coordinate cover of the host.
    /// * [`RunScope::All`] — every coordinate is resolved, so nothing short of
    ///   `all` covers it.
    /// * [`RunScope::LaneCoords`] — the run resolves exactly this lane's
    ///   coordinates, so this lane's own build covers it. That equality is the
    ///   whole of phase-340 W3: build-set and run-set are one computation.
    ///
    /// `nros_lane_build_lane` in `scripts/build/fixture-lane.sh` is the runtime
    /// implementation; `tests/lane_build_covers_run.rs` asserts the two agree
    /// for every lane.
    pub fn build_lane(self) -> &'static str {
        match self.run_scope() {
            RunScope::Native => "native",
            RunScope::All => "all",
            RunScope::LaneCoords => self.lane_token(),
        }
    }
}

impl CiLane {
    /// Which TESTS this lane's recipe executes — and therefore, through
    /// [`RunScope::build_lane`], which fixtures must EXIST for it.
    ///
    /// # Why this is not `coords(lane)` (issue 0482)
    ///
    /// Two different questions get asked of a lane, and they have different
    /// answers:
    ///
    /// * **which fixtures must be FRESH** — [`coords`], the lane's own cell
    ///   selection. Legitimately narrow; this is the whole tier-2 saving.
    /// * **which fixtures must EXIST** — a property of the RUN. `ci-matrix`
    ///   invokes `test-all` with no `NROS_TEST_SCOPE`, so every test binary
    ///   executes and every coordinate's fixtures are resolved.
    ///
    /// They used to be answered from the one lane name as if they were the same
    /// question: `_require-fixtures` was handed `NROS_FIXTURE_LANE=tier2` and
    /// asked "does the stamp cover tier 2?", to which a `lane=tier2` build is a
    /// perfectly good answer. The preflight passed and the RUN then discovered,
    /// one test at a time, that 34 of 47 coordinates had never been built —
    /// ~231 failures after a build that reported success.
    ///
    /// The justfile already said the build had to be `all`; a comment is not a
    /// gate. Declaring it here makes `_require-fixtures` derive the requirement
    /// from the same place the recipe derives its run narrowing, so the two
    /// cannot drift, and narrowing a lane's run is ONE edit here that both
    /// consumers follow.
    ///
    /// # phase-340 W3 — tier 2 became cheap as well as honest
    ///
    /// This used to read `Tier2 | Tier2Nightly => RunScope::All`, with a comment
    /// saying a coordinate-granular test selection was inexpressible. It is not:
    /// the fixture RESOLVER can attribute a resolved artifact back to its
    /// manifest row (`crate::fixtures::lane`), because the row's artifact root
    /// is where the build wrote it. So these lanes narrow at resolution time
    /// and their own build now covers their run.
    pub fn run_scope(self) -> RunScope {
        match self {
            // Host-only by construction (`tier1_is_native_only`), and `just ci`
            // filters the run to match. Kept NAME-based: tier 1's build lane is
            // `native`, a module-level superset of `coords(Tier1)`, so a
            // coordinate filter would skip host fixtures that were built and
            // buy nothing.
            // phase-395 W19 — COORDINATES, not names.
            //
            // This was `Native`, a name-based filter, and that is the coarse end
            // of "scope precisely rather than run-and-skip": it selects by test
            // NAME, so a test it cannot name is either dragged in or excluded
            // wholesale. It excluded `zephyr` and `threadx` outright — the very
            // platforms tier 1 promises — and no token spelling fixes that,
            // because `zephyr` also matches `zephyr_cortex_m_qemu`.
            //
            // Coordinate scoping is what tier 2 already does since phase-340 W3,
            // and it makes the selection exact: the resolver deselects by
            // coordinate, so the lane runs its cover and nothing else. It also
            // makes the BUILD and the RUN the same question again, which is why
            // `nros_lane_build_lane` can now map tier 1 to itself.
            CiLane::Tier1 => RunScope::LaneCoords,
            // Every test binary still executes; the resolver skips fixtures
            // outside `coords(lane)` — exactly the rows `--coords-from` told the
            // build to omit.
            CiLane::Tier2 | CiLane::Tier2Nightly => RunScope::LaneCoords,
        }
    }
}

/// The axes, addressed positionally so requirements can be built generically.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Axis {
    Platform,
    Lang,
    Rmw,
    Workload,
    Kind,
}

fn value(c: &Cell, a: Axis) -> String {
    match a {
        Axis::Platform => format!("{:?}", c.platform),
        Axis::Lang => format!("{:?}", c.lang),
        Axis::Rmw => format!("{:?}", c.rmw),
        Axis::Workload => format!("{:?}", c.workload),
        Axis::Kind => format!("{:?}", c.kind),
    }
}

/// One thing a lane must cover: a single axis value, or a pair of them.
type Req = String;

fn singles(c: &Cell, axes: &[Axis]) -> Vec<Req> {
    axes.iter()
        .map(|&a| format!("1|{a:?}={}", value(c, a)))
        .collect()
}

fn pairs(c: &Cell, axes: &[Axis]) -> Vec<Req> {
    let mut out = Vec::new();
    for (i, &a) in axes.iter().enumerate() {
        for &b in &axes[i + 1..] {
            out.push(format!("2|{a:?}={}|{b:?}={}", value(c, a), value(c, b)));
        }
    }
    out
}

fn spec(lane: CiLane) -> (Vec<Axis>, Vec<Axis>) {
    match lane {
        // 1-wise(platform, workload, kind) + pairwise(lang × rmw).
        //
        // `Platform` is what makes the pool above matter: the greedy walk is
        // driven by REQUIREMENTS, not candidates, so a platform satisfying no
        // requirement is never chosen. Measured — without it the cover stayed
        // 10 cells and Linux-only however wide the pool got; with it, 12 cells
        // covering all three tier-1 platforms.
        CiLane::Tier1 => (
            vec![Axis::Platform, Axis::Workload, Axis::Kind],
            vec![Axis::Lang, Axis::Rmw],
        ),
        // 1-wise(platform, lang, rmw, kind) — every declared value once, no
        // pairing. 14 of 50 coordinates (gated by `documented_lane_table_is_live`).
        CiLane::Tier2 => (
            vec![Axis::Platform, Axis::Lang, Axis::Rmw, Axis::Kind],
            vec![],
        ),
        // pairwise(platform × lang × rmw × kind); workload deliberately absent —
        // tier 1 already ran every workload, and workload selects
        // platform-independent logic, so repeating it costs cells and buys nothing.
        CiLane::Tier2Nightly => (
            vec![],
            vec![Axis::Platform, Axis::Lang, Axis::Rmw, Axis::Kind],
        ),
    }
}

fn pool(lane: CiLane) -> Vec<&'static Cell> {
    match lane {
        // phase-395 W19 — the tier-1 PLATFORMS, not just the host.
        //
        // `board-support.toml` is the SSoT for which platforms a tier promises,
        // and this list is asserted equal to it by
        // `tier1_pool_matches_board_registry` below. Hardcoding it WITHOUT that
        // test is what made three places disagree: the registry promised three
        // platforms, this pool named one, and `lane-filter.sh native` excluded
        // the other two outright.
        CiLane::Tier1 => all_runtime_cells()
            .filter(|c| {
                matches!(
                    c.platform,
                    PlatformId::Linux | PlatformId::ZephyrNativeSim | PlatformId::ThreadxLinux
                )
            })
            .collect(),
        CiLane::Tier2 | CiLane::Tier2Nightly => all_runtime_cells().collect(),
    }
}

fn reqs_of(c: &Cell, lane: CiLane) -> BTreeSet<Req> {
    let (s, p) = spec(lane);
    singles(c, &s).into_iter().chain(pairs(c, &p)).collect()
}

/// The cells this lane runs.
///
/// Deterministic for a fixed [`crate::matrix::CELLS`]: greedy set cover with a
/// lexicographic tie-break on the cell's debug rendering. Adding a cell can still
/// reshuffle the chosen set — that is inherent to greedy cover, and why the
/// selection is recomputed rather than committed.
pub fn cells(lane: CiLane) -> Vec<&'static Cell> {
    let candidates = pool(lane);
    let universe: BTreeSet<Req> = candidates.iter().flat_map(|c| reqs_of(c, lane)).collect();

    let mut covered: BTreeSet<Req> = BTreeSet::new();
    let mut chosen: Vec<&'static Cell> = Vec::new();
    let mut remaining: Vec<&'static Cell> = candidates;

    while covered != universe {
        // Most new requirements wins; ties break on the debug rendering so the
        // result does not depend on iteration order or hash seeds.
        let best = remaining
            .iter()
            .enumerate()
            .max_by_key(|(_, c)| {
                let gain = reqs_of(c, lane).difference(&covered).count();
                (gain, std::cmp::Reverse(format!("{c:?}")))
            })
            .map(|(i, _)| i);

        let Some(i) = best else { break };
        let cell = remaining.remove(i);
        if reqs_of(cell, lane).is_subset(&covered) {
            break; // nothing left can add coverage
        }
        covered.extend(reqs_of(cell, lane));
        chosen.push(cell);
    }
    chosen
}

/// The distinct `platform,lang,rmw` FIXTURE coordinates this lane needs, in the
/// spelling `examples/fixtures.toml` uses.
///
/// This — not [`cells`]`.len()` — is what a lane costs, because cells share
/// fixtures and a fixture build is the expensive part. Consumed by the
/// `lane-coords` binary, `fixtures-manifest.py --coords-from` and
/// `NROS_FIXTURE_COORDS`, so a lane's build, its staleness gate and its test
/// selection all derive from one computation.
pub fn coords(lane: CiLane) -> BTreeSet<String> {
    cells(lane)
        .iter()
        .flat_map(|c| {
            let (lang, rmw) = (
                format!("{:?}", c.lang).to_lowercase(),
                format!("{:?}", c.rmw).to_lowercase(),
            );
            c.platform
                .fixture_tokens()
                .iter()
                .map(move |p| format!("{p},{lang},{rmw}"))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    /// The text of one `just` recipe: its header line plus every indented or blank
    /// line under it (issue 1057).
    ///
    /// Stops at the first column-0 non-blank line — including a comment, which is
    /// where the NEXT recipe's doc block starts. Running past it would let a
    /// neighbouring recipe's prose satisfy or break an assertion, and the tier
    /// recipes document each other constantly.
    fn recipe_body(text: &str, name: &str) -> Option<String> {
        let mut lines = text.lines().skip_while(|l| {
            !(l.starts_with(name) && l[name.len()..].starts_with([':', ' ']) && l.contains(':'))
        });
        let header = lines.next()?;
        let mut body = String::from(header);
        for l in lines {
            if !l.is_empty() && !l.starts_with([' ', '\t']) {
                break;
            }
            body.push('\n');
            body.push_str(l);
        }
        Some(body)
    }

    /// Private recipes a body delegates to, e.g. `just ci::_matrix-run` (issue 1057).
    ///
    /// Only `_`-prefixed names: a tier delegating to another PUBLIC tier is a
    /// different relationship (the ladder, checked elsewhere), while a private
    /// delegate is an implementation split of the same recipe and carries the same
    /// obligations.
    fn delegated_recipes(body: &str) -> Vec<String> {
        let mut out = Vec::new();
        for (_, rest) in body
            .match_indices("just ")
            .map(|(i, m)| (i, &body[i + m.len()..]))
        {
            let token: String = rest
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | ':'))
                .collect();
            // MODULE-QUALIFIED private only (`just ci::_matrix-run`). An
            // UNQUALIFIED private delegate (`just _lane-gate`) is a recipe in
            // the ROOT justfile, which the caller has not loaded — it could
            // never resolve here, and accepting it is what forced the lookup to
            // tolerate a miss. Nothing is lost: the one such delegate in the
            // tier recipes is `_lane-gate`, a fixture preflight that exports no
            // `NROS_TEST_*` at all.
            let Some((_, name)) = token.split_once("::") else {
                continue;
            };
            if name.starts_with('_') && !out.iter().any(|n| n == name) {
                out.push(name.to_string());
            }
        }
        out
    }

    fn axis_values(cells: &[&'static Cell], a: Axis) -> BTreeSet<String> {
        cells.iter().map(|c| value(c, a)).collect()
    }

    #[test]
    fn lanes_are_deterministic() {
        for lane in ALL {
            let a: Vec<_> = cells(lane).iter().map(|c| format!("{c:?}")).collect();
            let b: Vec<_> = cells(lane).iter().map(|c| format!("{c:?}")).collect();
            assert_eq!(a, b, "{lane:?} selection must not vary between calls");
        }
    }

    #[test]
    fn tier1_covers_only_host_executable_platforms() {
        // Tier 1 promises runtime evidence EVERY MERGE, and only a
        // host-executable platform can afford that cadence — a cross-run
        // platform's evidence can only ever be nightly, which is tier 2's
        // promise. `check-board-tiers` enforces the same rule on the registry;
        // this is the lane's half.
        for c in cells(CiLane::Tier1) {
            assert!(
                matches!(
                    c.platform,
                    PlatformId::Linux | PlatformId::ZephyrNativeSim | PlatformId::ThreadxLinux
                ),
                "tier 1 runs host-executable platforms only, got {c:?}"
            );
        }
    }

    /// The pool above is a hardcoded list, and `board-support.toml` is the SSoT.
    /// Hardcoding WITHOUT this assertion is exactly how the two drifted apart:
    /// the registry promised three tier-1 platforms while the pool named one,
    /// and nothing noticed because nothing compared them.
    #[test]
    fn tier1_pool_matches_board_registry() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root");
        let out = std::process::Command::new("python3")
            .arg(root.join("scripts/check-board-tiers.py"))
            .arg("--print-tiers")
            .current_dir(root)
            .output();
        let Ok(out) = out else {
            crate::skip!("python3 unavailable — cannot read the board registry");
        };
        if !out.status.success() {
            crate::skip!("check-board-tiers --print-tiers failed; the registry could not be read");
        }
        let text = String::from_utf8_lossy(&out.stdout);
        let mut registry: Vec<&str> = text
            .lines()
            .filter_map(|l| {
                let mut f = l.split('\t');
                let plat = f.next()?;
                let tier = f.next()?;
                (tier == "1").then_some(plat)
            })
            .collect();
        registry.sort_unstable();

        let mut pooled: Vec<String> = pool(CiLane::Tier1)
            .iter()
            .map(|c| format!("{:?}", c.platform))
            .collect();
        pooled.sort_unstable();
        pooled.dedup();

        assert_eq!(
            pooled, registry,
            "the tier-1 POOL and packages/boards/board-support.toml disagree.\n\
             The registry is the source of truth for which platforms a tier \
             promises; update `pool(CiLane::Tier1)` to match, or change the tier \
             in the registry deliberately."
        );
    }

    /// Issue 0393 — `build-test-fixtures-leaves` narrows its fan-out by FILTERING
    /// a hardcoded, deliberately-ordered platform list (zephyr first and solo,
    /// because it wants the whole job budget). Order is a scheduling property, so
    /// the list stays; but a module that a lane can select and the list does not
    /// name would be filtered out silently — the lane would build less than it
    /// gates, and the run would fail "Binary not found" with no clue why.
    ///
    /// So: the justfile's list must be a SUPERSET of every module the matrix can
    /// produce. Adding a platform to `matrix::CELLS` without adding it there
    /// fails here instead of in a sweep three hours later.
    #[test]
    fn build_fanout_names_every_module_the_matrix_can_select() {
        // This one DOES read the root justfile: the fan-out loop lives there,
        // not in a module.
        let justfile = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../justfile");
        let Ok(text) = std::fs::read_to_string(justfile) else {
            return; // out-of-tree checkout; nothing to gate
        };
        // The canonical ordered list — the one the make graph filters.
        let line = text
            .lines()
            .map(str::trim)
            .find(|l| l.starts_with("for platform in zephyr native qemu"))
            .unwrap_or_else(|| {
                panic!(
                    "justfile: canonical fixture platform list not found — if it was \
                     renamed or reordered, update this gate rather than deleting it"
                )
            });
        let listed: BTreeSet<&str> = line
            .trim_start_matches("for platform in")
            .trim_end_matches("; do")
            .split_whitespace()
            .collect();

        let needed: BTreeSet<&str> = PlatformId::ALL.iter().map(|p| p.just_module()).collect();
        let missing: Vec<&&str> = needed.difference(&listed).collect();
        assert!(
            missing.is_empty(),
            "justfile `build-test-fixtures-leaves` does not name {missing:?}; a lane \
             selecting that module would build nothing for it.\n  listed:  {listed:?}\n  \
             matrix:  {needed:?}"
        );
    }

    /// Issue 0482 — the justfile recipe must set the `NROS_TEST_SCOPE` its lane
    /// declares, and must not build a NARROWER fixture lane than that run needs.
    ///
    /// The defect: `ci-matrix` runs the whole suite (no `NROS_TEST_SCOPE`) while
    /// `_require-fixtures` was asked only "does the stamp cover tier 2?". A
    /// `just build-test-fixtures lane=tier2` therefore satisfied the preflight
    /// and the run mass-failed on fixtures the lane had never built.
    /// `run_scope()` is now the one declaration both sides read; this asserts
    /// the recipes agree with it, because the recipes are where the lane
    /// actually reaches a runner.
    ///
    /// Deliberately checked against the RECIPE TEXT rather than trusting the
    /// declaration: the declaration is only worth something if the thing that
    /// runs obeys it, and `ci_tier_ladder_matches_justfile_recipes` set the
    /// precedent that the justfile is a checked consumer of this ladder.
    /// issue 1057 — the extraction itself, because its blind spot is what went
    /// wrong.
    ///
    /// `recipes_run_the_scope_their_lane_declares` read only the NAMED recipe.
    /// When phase-413 turned the tiers into dispatchers, the exports it asserts
    /// on moved into a private delegate and the assertion started measuring a
    /// `case` statement. It went red on main and stayed there, invisible to every
    /// pull request because `check-fast` runs no unit tests.
    ///
    /// So the delegation-following is itself covered, on a synthetic justfile
    /// rather than the real one: a test whose only evidence is the tree it runs
    /// in goes green the moment that tree changes shape again.
    #[test]
    fn a_dispatching_recipe_is_read_through_to_its_private_delegate() {
        let text = "\
tier depth=\"run\":
    #!/usr/bin/env bash
    case \"{{ depth }}\" in
        run)   just ci::_tier-run ;;
        build) just ci::_tier-build ;;
    esac

[private]
_tier-run:
    #!/usr/bin/env bash
    NROS_TEST_COORDS=\"$coords\" just check

[private]
_tier-build:
    #!/usr/bin/env bash
    echo build
";
        let body = recipe_body(text, "tier").expect("the dispatcher");
        assert!(
            !body.contains("NROS_TEST_COORDS="),
            "the dispatcher's own body must NOT carry the export, or this fixture \
             is not reproducing the shape that broke"
        );

        let delegates = delegated_recipes(&body);
        assert_eq!(
            delegates,
            vec!["_tier-run".to_string(), "_tier-build".to_string()],
            "both private delegates must be found, and only the private ones"
        );

        // MODULE-QUALIFIED private only. `just check` is another lane with its
        // own contract; `just _lane-gate` is a ROOT recipe that this text does
        // not contain, and accepting it is what forced the lookup to tolerate a
        // miss — which in turn let a renamed delegate go unnoticed.
        assert_eq!(
            delegated_recipes(
                "tier:\n    just _lane-gate tier2\n    just check\n    just ci::_inner\n"
            ),
            vec!["_inner".to_string()],
            "an unqualified private delegate and a public one must both be skipped"
        );

        // The other half of that rule: a QUALIFIED delegate must resolve. The
        // caller panics when this returns None, and this is the shape it panics
        // on — a dispatcher naming a recipe that is not in its module.
        assert!(
            recipe_body(text, "_tier-renamed").is_none(),
            "a delegate that does not exist must not silently resolve to something"
        );

        let inner = recipe_body(text, "_tier-run").expect("the delegate");
        assert!(
            inner.contains("NROS_TEST_COORDS="),
            "reading through to the delegate is the whole point; got:\n{inner}"
        );
        assert!(
            !inner.contains("echo build"),
            "a recipe body must stop at the next column-0 line, or one delegate's \
             text satisfies an assertion about another"
        );
    }

    #[test]
    fn recipes_run_the_scope_their_lane_declares() {
        use crate::buckets::CiTier;

        // No root-justfile read: `justfile_source()` resolves the file per tier
        // (module recipes live in `just/<mod>.just`) and returns None
        // out-of-tree, which is the guard this used to do by hand.

        for lane in ALL {
            let tier = CiTier::of_lane(lane);
            let recipe = tier.just_recipe();
            // Same resolver as ci_tier_ladder_matches_justfile_recipes — the
            // tiers are module recipes since phase-399 and this test used to
            // look only at column 0 of the root justfile.
            let Some((text, name)) = tier.justfile_source() else {
                continue;
            };
            let Some(mut body) = recipe_body(&text, name) else {
                panic!(
                    "justfile has no `{recipe}` recipe (gated by ci_tier_ladder_matches_justfile_recipes)"
                )
            };

            // issue 1057 — follow a DISPATCHING recipe to the body that runs.
            //
            // phase-413 gave the tiers a `depth` argument, so `matrix` became a
            // `case` that delegates to the private `_matrix-run` /
            // `_matrix-build`, and the exports this test asserts on moved with
            // it. Reading only the named recipe then measured a dispatcher and
            // reported the lane as narrowing nothing — a RED that no pull
            // request can see, because `check-fast` runs no unit tests, and that
            // therefore fails in the merge queue instead.
            //
            // One level deep, deliberately: a dispatcher that delegates to a
            // dispatcher is a shape nothing here has and one this assertion
            // should not quietly accept.
            for delegate in delegated_recipes(&body) {
                // A named delegate that does not resolve is a FAILURE, not a
                // skip (issue 0196's rule). Tolerating a miss leaves exactly the
                // recurrence path this test was just fixed for: rename
                // `_matrix-run` without updating the dispatcher and the
                // assertion silently reads the shorter body again, reporting
                // green on a lane that narrows nothing.
                let inner = recipe_body(&text, &delegate).unwrap_or_else(|| {
                    panic!(
                        "`just {recipe}` dispatches to `{delegate}`, which does not exist in \
                         the same justfile module. Either the delegate was renamed and the \
                         dispatcher not updated, or this test is reading the wrong file — \
                         both are the defect, not a reason to read less:\n{body}"
                    )
                });
                {
                    body.push('\n');
                    body.push_str(&inner);
                }
            }

            let scope = lane.run_scope();
            match scope {
                RunScope::Native => assert!(
                    body.contains("NROS_TEST_SCOPE=native"),
                    "{lane:?} declares RunScope::Native but `just {recipe}` does not set \
                     NROS_TEST_SCOPE=native — the run would execute every platform's tests \
                     on fixtures the lane never built:\n{body}"
                ),
                RunScope::All => assert!(
                    !body.contains("NROS_TEST_SCOPE="),
                    "{lane:?} declares RunScope::All but `just {recipe}` narrows the run with \
                     NROS_TEST_SCOPE — narrow the declaration too, or the required fixture \
                     build (`{}`) is computed for a run that no longer happens:\n{body}",
                    lane.build_lane()
                ),
                // phase-340 W3 — the narrowing is not name-based, so the recipe
                // must hand the RESOLVER the lane's coordinate file. Without it
                // the run silently resolves every coordinate while
                // `_require-fixtures` accepts the lane's own (narrow) build:
                // issue 0482's ~231 STALE failures, with the two sides swapped.
                RunScope::LaneCoords => assert!(
                    body.contains(&format!("{}=", crate::fixtures::lane::RUN_COORDS_ENV)),
                    "{lane:?} declares RunScope::LaneCoords but `just {recipe}` never exports \
                     {} — the run would resolve every coordinate while the preflight accepts a \
                     `lane={}` build:\n{body}",
                    crate::fixtures::lane::RUN_COORDS_ENV,
                    lane.build_lane()
                ),
            }

            // The other half: whatever fixture lane the recipe names must be one
            // `nros_fixtures_stamp_require` will map to a covering build. The
            // recipe may legitimately name the CELL lane (that is what scopes
            // the freshness gate); what it must never do is name a lane the
            // stamp check would then accept a narrower build for. That mapping
            // lives in `fixture-lane.sh` and is exercised by
            // `tests/lane_build_covers_run.rs`; here we only pin that the
            // recipe declares SOME lane, since an unset one silently means
            // `all` and would hide a narrowing.
            assert!(
                body.contains("NROS_FIXTURE_LANE="),
                "`just {recipe}` sets no NROS_FIXTURE_LANE; the fixture gates would \
                 silently fall back to `all` and the lane's scope would be invisible \
                 to them (issue 0443's shape):\n{body}"
            );
        }
    }

    /// The regression this whole module exists for: adding a platform (or RMW, or
    /// language) to the matrix must extend the lane, not be silently skipped.
    #[test]
    fn lanes_touch_every_declared_value_of_every_axis_they_cover() {
        for lane in ALL {
            let chosen = cells(lane);
            let available = pool(lane);
            let (s, p) = spec(lane);
            for a in s.into_iter().chain(p) {
                let want = axis_values(&available, a);
                let got = axis_values(&chosen, a);
                assert_eq!(
                    want,
                    got,
                    "{lane:?} misses {a:?} values {:?}",
                    want.difference(&got).collect::<Vec<_>>()
                );
            }
        }
    }

    #[test]
    fn nightly_covers_every_declared_pair_it_claims_to() {
        let lane = CiLane::Tier2Nightly;
        let (_, p) = spec(lane);
        let want: BTreeSet<Req> = pool(lane).iter().flat_map(|c| pairs(c, &p)).collect();
        let got: BTreeSet<Req> = cells(lane).iter().flat_map(|c| pairs(c, &p)).collect();
        assert_eq!(
            want,
            got,
            "uncovered pairs: {:?}",
            want.difference(&got).take(5).collect::<Vec<_>>()
        );
    }

    /// A lane that selected everything would pass every other test here while
    /// defeating the point, so bound the ladder.
    ///
    /// Measured in COORDINATES, not cells. In cells the ladder is not even
    /// monotone — tier 1 picks 17 cells and 1-wise tier 2 picks 11 — because tier
    /// 1's cells are all native and a native fixture is nearly free. Cell count is
    /// the wrong unit for cost; asserting on it would encode the very confusion
    /// that put "tier 2 is 20 % of the sweep" in RFC-0061.
    /// phase-342 W3 — the module's cost table is DOCUMENTATION OF A COMPUTATION,
    /// so recompute it and fail when they part company.
    ///
    /// Four places carried this arithmetic by hand and three had drifted: the
    /// table above said tier 2 = 11 cells / 12 coords and nightly = 37 / 33,
    /// `justfile` said "12 of 47" and "33 of 47", and the code selected 12 / 13
    /// and 35 / 35. Nobody was wrong on purpose — cells move whenever
    /// `matrix::CELLS` does, and prose does not recompute.
    ///
    /// The counts are asserted, the percentages are not: those are a judgement
    /// about cost that rounds, and pinning them would make this gate fire on
    /// noise. Coordinates are the unit (see `the_ladder_is_monotone_in_fixture_cost`).
    #[test]
    fn documented_lane_table_is_live() {
        let total_coords = {
            let all: BTreeSet<String> = all_runtime_cells()
                .flat_map(|c| {
                    let (lang, rmw) = (
                        format!("{:?}", c.lang).to_lowercase(),
                        format!("{:?}", c.rmw).to_lowercase(),
                    );
                    c.platform
                        .fixture_tokens()
                        .iter()
                        .map(|p| format!("{p},{lang},{rmw}"))
                        .collect::<Vec<_>>()
                })
                .collect();
            all.len()
        };

        // (lane, cells, coords) exactly as the module docs above state them.
        let documented = [
            (CiLane::Tier1, 17, 12),
            (CiLane::Tier2, 13, 13),
            (CiLane::Tier2Nightly, 36, 36),
        ];
        for (lane, want_cells, want_coords) in documented {
            assert_eq!(
                cells(lane).len(),
                want_cells,
                "{lane:?}: the ci_lane module table says {want_cells} cells; recomputed \
                 {}. Update the table (and the justfile comments that quote it).",
                cells(lane).len()
            );
            assert_eq!(
                coords(lane).len(),
                want_coords,
                "{lane:?}: the ci_lane module table says {want_coords} coords; \
                 recomputed {}. Update the table (and the justfile comments).",
                coords(lane).len()
            );
        }
        assert_eq!(
            total_coords, 50,
            "the table's tier-3 denominator (50 coordinates) is stale; recomputed \
             {total_coords}"
        );
    }

    #[test]
    fn the_ladder_is_monotone_in_fixture_cost() {
        let all: BTreeSet<String> = all_runtime_cells()
            .flat_map(|c| {
                let (lang, rmw) = (
                    format!("{:?}", c.lang).to_lowercase(),
                    format!("{:?}", c.rmw).to_lowercase(),
                );
                c.platform
                    .fixture_tokens()
                    .iter()
                    .map(move |p| format!("{p},{lang},{rmw}"))
            })
            .collect();
        let t1 = coords(CiLane::Tier1).len();
        let t2 = coords(CiLane::Tier2).len();
        let tn = coords(CiLane::Tier2Nightly).len();

        assert!(t1 > 0 && t2 > 0 && tn > 0, "empty lane: {t1}/{t2}/{tn}");
        assert!(
            t1 <= t2 && t2 < tn && tn < all.len(),
            "ladder must cost strictly more at each rung: \
             tier1={t1} tier2={t2} nightly={tn} all={}",
            all.len()
        );
        // The point of splitting tier 2: the per-change gate has to be cheap
        // enough that it gets run. If it ever creeps past a third of the sweep it
        // has stopped being a middle tier.
        assert!(
            t2 * 3 <= all.len(),
            "tier 2 ({t2}) must stay under a third of the sweep ({}) — \
             it is the lane that runs on every core change",
            all.len()
        );
    }

    /// Tier 2 gives up pairing to stay cheap; the nightly lane is where that
    /// coverage comes back. If they ever computed the same set, the split would be
    /// pure cost with no benefit.
    #[test]
    fn nightly_is_strictly_more_than_the_gate() {
        let gate = coords(CiLane::Tier2);
        let nightly = coords(CiLane::Tier2Nightly);
        assert!(
            gate.len() < nightly.len(),
            "nightly ({}) must cover more than the gate ({})",
            nightly.len(),
            gate.len()
        );
    }

    // Whether every coordinate a lane selects actually HAS a fixture row is not
    // asserted here on purpose: `tests/matrix_fixture_coverage.rs` already owns
    // that question in both directions, together with the exemption list naming
    // each cell built outside the manifest (Fvp's `just zephyr build-fvp-*`, the
    // zephyr west leaves, NuttxRiscv examples…). A second gate here would need a
    // second copy of that list, and a copy that drifts narrower is the "gates
    // narrower than the rule they enforce" defect (issue 0196).

    /// `scripts/test/lane-filter.sh` derives its exclusion tokens from
    /// `PlatformId`. If a platform is added whose family name the script cannot
    /// produce, the native lane would silently RUN that platform's binaries — the
    /// rot this whole module exists to prevent (audit E5 / issue 0341).
    #[test]
    fn lane_filter_tokens_cover_every_non_native_platform() {
        let out = std::process::Command::new("bash")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../../scripts/test/lane-filter.sh"
            ))
            .arg("native")
            .output();
        let Ok(out) = out else { return }; // script unavailable (packaged crate) — not a failure
        if !out.status.success() {
            panic!(
                "lane-filter.sh native failed: {}",
                String::from_utf8_lossy(&out.stderr)
            );
        }
        // Two views on purpose: the family/binary checks are spelling-agnostic,
        // but the CAPITALISED assertion below is precisely about case — matching
        // it against a lowercased copy would make it unfalsifiable.
        let raw = String::from_utf8_lossy(&out.stdout).to_string();
        let filter = raw.to_lowercase();
        for c in all_runtime_cells() {
            if matches!(c.platform, PlatformId::Linux) {
                continue;
            }
            // Same rule the script applies: the leading CamelCase word.
            // `Esp32Qemu` -> `esp32` (NOT `esp` — stopping at the digit would
            // look for a token the script never emits).
            let debug = format!("{:?}", c.platform);
            let mut family = String::new();
            for (i, ch) in debug.chars().enumerate() {
                if i > 0 && ch.is_ascii_uppercase() {
                    break;
                }
                family.push(ch.to_ascii_lowercase());
            }
            assert!(
                filter.contains(&format!("binary(~{family})")),
                "native lane would still run {:?} binaries — lane-filter.sh emitted:\n{filter}",
                c.platform
            );

            // Issue 0357 — binary exclusion alone is not coverage. The matrix
            // consumers put EVERY platform's cases in one generically-named
            // binary (`rtos_e2e` is entirely cross-platform and matches no token
            // at all), so a lane filtered only by binary name still ran 53
            // cross-platform tests on a host with none of their fixtures.
            assert!(
                filter.contains(&format!("test(~{family})")),
                "no TEST-level exclusion for {:?}; a cross-platform case inside a \
                 generically-named binary would still run — lane-filter.sh emitted:\n{filter}",
                c.platform
            );

            // nextest's `~` is a case-SENSITIVE substring match and the harnesses
            // disagree on spelling: rstest emits `platform_1_Platform__Freertos`,
            // hand-rolled matrices emit `case_05_zephyr_rust`. One spelling
            // silently covers half the suite.
            let mut cap = family.clone();
            cap[..1].make_ascii_uppercase();
            assert!(
                raw.contains(&format!("test(~{cap})")),
                "no CAPITALISED test exclusion for {:?} — rstest case names would \
                 survive (nextest `~` is case-sensitive):\n{raw}",
                c.platform
            );
        }
        // The host lane must not exclude ITSELF. This assertion used to read
        // `!filter.contains("binary(~native)")` — a hardcoded literal, and a
        // gate narrower than the rule it enforces (the issue-0196 class) in two
        // ways at once:
        //
        //   1. It named `native`. phase-337 W8.b renamed the host variant to
        //      `Linux`, `lane-filter.sh`'s `grep -v '^Native$'` stopped matching,
        //      and `Linux` survived into the exclusion set — while this assertion
        //      kept passing, because the string it looked for was gone too.
        //   2. It checked only `binary(...)`. The damage is done by the TEST
        //      form: `not test(~Linux)` filters out every
        //      `platform_N_Platform__Linux` rstest case, i.e. the entire host
        //      matrix, and the run reports a fast green.
        //
        // Both fixed by deriving the host family from `PlatformId` the same way
        // the script does, and by checking every spelling the script emits.
        let host_family = {
            let debug = format!("{:?}", PlatformId::Linux);
            let mut f = String::new();
            for (i, ch) in debug.chars().enumerate() {
                if i > 0 && ch.is_ascii_uppercase() {
                    break;
                }
                f.push(ch.to_ascii_lowercase());
            }
            f
        };
        let mut host_cap = host_family.clone();
        host_cap[..1].make_ascii_uppercase();
        for form in [host_family.as_str(), host_cap.as_str()] {
            for kind in ["binary", "test"] {
                assert!(
                    !raw.contains(&format!("{kind}(~{form})")),
                    "the host lane excludes ITSELF via {kind}(~{form}) — every host \
                     case would be filtered out and the lane would report a fast \
                     green. `lane-filter.sh` must skip the `{:?}` variant:\n{raw}",
                    PlatformId::Linux
                );
            }
        }

        // The unit-test exemption. Without it the lane drops host-only tests
        // whose names merely mention a platform — `board::tier::tests::
        // threadx_inverts_scale`, `qemu::tests::test_parse_results`, and
        // `zephyr::tests::content_aware_staleness_ignores_mtime_only_bumps`, a
        // phase-318 test. Those need no fixture and no toolchain, so excluding
        // them trades one coverage hole for another.
        assert!(
            filter.contains("test(~tests::)"),
            "the unit-test exemption is missing; host-only `mod tests` cases that \
             mention a platform would be excluded from tier 1:\n{filter}"
        );
    }

    /// The filter's lines are ANDed by the caller. Emitting them as separate
    /// nextest `-E` flags instead UNIONS them, and `not A or not B` is a
    /// tautology that selects everything — a filter that silently does nothing.
    /// `just test-all` joins with " and " into a single `-E`; this pins the
    /// grouped line's shape so a future edit cannot produce an expression that
    /// only composes correctly under OR.
    #[test]
    fn lane_filter_test_exclusions_are_one_grouped_conjunction() {
        let out = std::process::Command::new("bash")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../../scripts/test/lane-filter.sh"
            ))
            .arg("native")
            .output();
        let Ok(out) = out else { return };
        if !out.status.success() {
            panic!(
                "lane-filter.sh native failed: {}",
                String::from_utf8_lossy(&out.stderr)
            );
        }
        let text = String::from_utf8_lossy(&out.stdout);
        let grouped: Vec<&str> = text
            .lines()
            .filter(|l| l.starts_with("(test(~tests::)"))
            .collect();
        assert_eq!(
            grouped.len(),
            1,
            "expected exactly one grouped test-exclusion line, got {}:\n{text}",
            grouped.len()
        );
        let g = grouped[0];
        assert!(
            g.ends_with(')') && g.contains(" or (") && g.contains(" and not test(~"),
            "the grouped line must be `(exemption or (not … and not …))`; got:\n{g}"
        );
        // Every other line is a standalone binary exclusion.
        for l in text.lines().filter(|l| !l.starts_with("(test(")) {
            assert!(
                l.starts_with("not binary(~"),
                "unexpected filter line (the caller ANDs these): {l}"
            );
        }
    }
}
