//! RFC-0051 / phase-295 W1.c — matrix ⊆⊇ fixtures.toml cross-check.
//!
//! Forward (ASSERTED): every `Tier::Runtime` cell of the declared matrix
//! whose platform bakes fixtures has at least one matching
//! `[[fixture]]` / `[[workspace_fixture]]` row in `examples/fixtures.toml`
//! for its (platform, lang, rmw) coordinate — a Runtime cell nothing
//! builds is a lie in the table.
//!
//! Reverse (REPORTED, flips to an assert at phase-295 W3-end): every
//! fixture row's (platform, lang, rmw) maps onto SOME cell coordinate —
//! rows outside the matrix are either debt the table must model or
//! orphans to delete. Reported (not asserted) while W3 migrates the
//! long tail; the report keeps the count visible in every run's output.
//!
//! Sibling of `examples_fixture_coverage.rs` (which checks example DIRS
//! have fixture rows); this file checks the MATRIX against the rows.

use std::collections::BTreeSet;

use nros_tests::{
    interop::{self, NO_TEST},
    matrix::{CELLS, Kind, Lang, PlatformId, Rmw, TestCell, Tier},
};

/// fixtures.toml `platform` strings → matrix platform.
///
/// Delegates to the SSoT on `PlatformId` (phase-318 W4.d). It used to be spelled
/// out here, and `ci_lane::coords` then grew a second, disagreeing copy of the
/// forward direction — `qemu-esp32-baremetal` attributed to `QemuBaremetal`
/// rather than `Esp32Qemu`. One home, two directions, a round-trip test.
fn platform_from_str(s: &str) -> Option<PlatformId> {
    PlatformId::from_fixture_token(s)
}

fn lang_from_str(s: &str) -> Option<Lang> {
    Some(match s {
        "rust" => Lang::Rust,
        "c" => Lang::C,
        "cpp" => Lang::Cpp,
        "mixed" => Lang::Mixed,
        _ => return None,
    })
}

fn rmw_from_str(s: &str) -> Option<Rmw> {
    Some(match s {
        "zenoh" => Rmw::Zenoh,
        "cyclonedds" => Rmw::Cyclonedds,
        "xrce" => Rmw::Xrce,
        _ => return None,
    })
}

/// (platform_idx, lang_idx, rmw_idx, is_workspace) coordinate key.
type Coord = (u16, u16, u16, bool);

/// Coordinates present in fixtures.toml + rows whose strings didn't map.
///
/// # Why this shells out instead of parsing the TOML (issue 0482)
///
/// A row's coordinate is `(platform, lang, rmw)`, `rmw` is optional, and this
/// function used to resolve the omission itself:
/// `get("rmw").unwrap_or("zenoh")`, commented "fixtures.toml convention". The
/// LANE side — `fixtures-manifest.py::matches_filters`, which every
/// coordinate-scoped fixture build and the whole staleness gate go through —
/// compared a bare `None` instead. So the two sides of the same question had two
/// answers: this gate reported 67 rows modeled and green while no lane could
/// select any of them. `just build-test-fixtures lane=tier2` produced a stamp
/// the preflight accepted and a `just ci-matrix` that mass-failed STALE on
/// fixtures its own lane had never built.
///
/// The fix is not to copy the default into the second reader — that is the
/// second spelling CLAUDE.md's fix-the-class rule forbids. `row_coord()` in
/// `fixtures-manifest.py` is now the ONE computation and `coords` exports it, so
/// this gate audits the coordinates the BUILD actually uses. If they ever part
/// company again there is nowhere for the disagreement to live.
///
/// One deliberate consequence: `coords` omits `skip_build` rows, which the old
/// in-Rust parser counted. That is the point — a coordinate no build produces
/// cannot satisfy `every_runtime_cell_has_a_fixture_row`, which claims the cell
/// is BUILT. (There are none today; the semantics matter for the next one.)
fn fixture_coords() -> (BTreeSet<Coord>, Vec<String>) {
    let root = nros_tests::project_root();
    let out = std::process::Command::new("python3")
        .arg(root.join("scripts/build/fixtures-manifest.py"))
        .arg("coords")
        .current_dir(&root)
        .output()
        .expect("run fixtures-manifest.py coords");
    assert!(
        out.status.success(),
        "fixtures-manifest.py coords failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8(out.stdout).expect("coords output is utf-8");

    let mut coords = BTreeSet::new();
    let mut unmapped = Vec::new();
    for line in stdout.lines().filter(|l| !l.is_empty()) {
        // <kind>\x1f<platform>\x1f<lang>\x1f<rmw>\x1f<dir>\x1f<id>\x1f<artifact_root>
        // (`id`/`artifact_root` are phase-340 W3's run-narrowing columns; this
        // gate does not read them, but it pins the SHAPE so a column added or
        // dropped fails here rather than silently shifting `dir`.)
        let f: Vec<&str> = line.split('\x1f').collect();
        assert_eq!(
            f.len(),
            8,
            "unexpected `coords` record shape (expected 8 \\x1f-separated fields): {line:?}"
        );
        let (table, p, l, r, dir) = (f[0], f[1], f[2], f[3], f[4]);
        let is_ws = table == "workspace_fixture";
        match (platform_from_str(p), lang_from_str(l), rmw_from_str(r)) {
            (Some(p), Some(l), Some(r)) => {
                coords.insert((p.index(), l.port_index(), r.index(), is_ws));
            }
            // Unlike the old parser, a row with an unreadable platform/lang is
            // REPORTED rather than skipped: `continue`-ing past it is how a row
            // with no coordinate stayed invisible to this gate as well as to
            // every lane.
            _ => unmapped.push(format!("{table}: platform={p} lang={l} rmw={r} dir={dir}")),
        }
    }
    (coords, unmapped)
}

/// Forward: every baked Runtime cell has a fixture row at its coordinate.
#[test]
fn every_runtime_cell_has_a_fixture_row() {
    let (coords, _) = fixture_coords();
    let mut missing = Vec::new();
    for c in CELLS {
        if !matches!(c.tier, Tier::Runtime) {
            continue;
        }
        // Cells built OUTSIDE fixtures.toml — each exemption names its
        // real build channel (the W4 goal is shrinking this list by
        // folding the lanes into fixtures.toml or a sibling manifest):
        // - Native: `just build-test-fixtures` native family + ephemeral
        //   isolation.
        // - Interop/Bridge cells are NO LONGER in `matrix::CELLS` (issue 0352 /
        //   phase-324): they live in `interop::CELLS` and are gated by
        //   `interop_bindings_*` below, not by a fixtures.toml coordinate.
        // - ZephyrNativeSim examples + non-rust workspaces: the west
        //   leaves lane (scripts/build/zephyr-fixture-leaves.sh — its own
        //   staleness sig, fixtures.toml `skip_probe` note).
        // - NuttxRiscv examples: `just nuttx build-riscv-*` recipes.
        // - ThreadxRiscv64 cyclone: `just threadx_riscv64 build-fixtures`
        //   deploy-overlay lane (#214).
        // phase-337 W2.e — `ZephyrQemuCortexM` joins with NO kind qualifier: it
        // has only Example cells and every one of them is a west build, so the
        // native_sim split (examples + non-rust workspaces) has nothing to
        // distinguish here. Adds zero `fixtures.toml` rows.
        let west_lane_zephyr = matches!(c.platform, PlatformId::ZephyrQemuCortexM)
            || (matches!(c.platform, PlatformId::ZephyrNativeSim)
                && (matches!(c.kind, Kind::Example)
                    || (matches!(c.kind, Kind::Workspace) && !matches!(c.lang, Lang::Rust))));
        if matches!(c.platform, PlatformId::Linux)
            || west_lane_zephyr
            || (matches!(c.platform, PlatformId::NuttxRiscv) && matches!(c.kind, Kind::Example))
            || (matches!(c.platform, PlatformId::ThreadxRiscv64)
                && matches!(c.rmw, Rmw::Cyclonedds))
        {
            continue;
        }
        let is_ws = matches!(c.kind, Kind::Workspace);
        let key = (
            c.platform.index(),
            c.lang.port_index(),
            c.rmw.index(),
            is_ws,
        );
        if !coords.contains(&key) {
            missing.push(format!("{c:?}"));
        }
    }
    assert!(
        missing.is_empty(),
        "Runtime cells with NO fixtures.toml row at their (platform, lang, rmw) \
         coordinate — either the table lies or the fixture is missing:\n{}",
        missing.join("\n")
    );
}

/// Reverse (ASSERTED since W3-end): every fixture coordinate maps onto a
/// matrix cell — an unmodeled row is either debt the table must model or
/// an orphan to delete.
#[test]
fn fixture_rows_all_modeled_by_matrix() {
    let (coords, unmapped) = fixture_coords();
    let cell_keys: BTreeSet<_> = CELLS
        .iter()
        .map(|c| {
            (
                c.platform.index(),
                c.lang.port_index(),
                c.rmw.index(),
                matches!(c.kind, Kind::Workspace),
            )
        })
        .collect();
    let orphans: Vec<_> = coords.difference(&cell_keys).collect();
    assert!(
        orphans.is_empty() && unmapped.is_empty(),
        "fixtures.toml coordinates outside the matrix (model them or delete the rows):\n\
         orphan (platform_idx, lang_idx, rmw_idx, is_ws): {orphans:?}\nunmapped rows: {unmapped:?}"
    );
}

/// `PlatformId::fixture_tokens` and `PlatformId::from_fixture_token` must be
/// inverse, and the token space must not collide.
///
/// A collision would silently reattribute a fixture family to the wrong platform
/// — which is exactly the bug that motivated consolidating the mapping — and a
/// non-inverse pair makes coordinate selection and coverage checking disagree
/// about what a lane covers.
#[test]
fn fixture_token_mapping_round_trips() {
    let mut seen: BTreeSet<&str> = BTreeSet::new();
    for &p in PlatformId::ALL {
        let tokens = p.fixture_tokens();
        assert!(!tokens.is_empty(), "{p:?} declares no fixture token");
        for &t in tokens {
            assert!(
                seen.insert(t),
                "token {t:?} claimed by more than one platform"
            );
            assert_eq!(
                PlatformId::from_fixture_token(t),
                Some(p),
                "{t:?} does not map back to {p:?}"
            );
        }
    }
}

/// Every `PlatformId::just_module` must name a module the justfile actually
/// declares. A stale name does not fail loudly — a CI job runs `just <gone> build`
/// and errors with "unknown recipe", which reads as infrastructure noise rather
/// than as the matrix having drifted.
#[test]
fn every_just_module_is_declared_by_the_justfile() {
    let Ok(justfile) = std::fs::read_to_string(nros_tests::project_root().join("justfile")) else {
        return; // packaged crate — not a failure
    };
    for &p in PlatformId::ALL {
        let m = p.just_module();
        assert!(
            justfile.contains(&format!("\nmod {m} ")),
            "justfile declares no `mod {m}` (for {p:?}) — a CI job dispatching on it \
             would fail as `unknown recipe`, not as a matrix gap"
        );
    }
}

// ============================================================================
// Issue #405 / phase-337 W3.f — PRODUCIBILITY.
//
// The issue-0196 rule, applied to the fixture BUILD: what a lane's gate demands
// must be producible by the recipes that lane actually runs.
//
// Two layers narrow a lane and they derive from one `lane-coords` computation,
// so they cannot select different SETS — but below the module level they stopped
// agreeing. `lane-coords --modules` maps `nuttx-riscv,c,zenoh` to the `nuttx`
// module (correctly: `PlatformId::just_module` says `nuttx` owns both NuttX
// witnesses), while `just nuttx build-fixtures` built only the arm side. The
// riscv fixtures lived in `build-riscv-*` recipes nothing on the lane's path
// invoked, so `just build-test-fixtures lane=tier2` finished green and
// `_lane-gate` then failed on a fixture no builder in that run could create.
//
// This gate reads the recipe graph the lane really walks — `build-fixtures` plus
// everything reachable from it through `just` dependencies and `just <module>
// <recipe>` calls in recipe BODIES (comments stripped: a `# … just nuttx
// build-riscv-c …` note is documentation, not an edge) — and asserts every
// fixture token the module owns is passed to one of the two fixture builders
// somewhere in it.
// ============================================================================

/// A parsed `just/<module>.just`: recipe name -> (dependencies, body lines).
fn parse_just_module(text: &str) -> std::collections::BTreeMap<String, (Vec<String>, String)> {
    let mut out = std::collections::BTreeMap::new();
    let mut current: Option<String> = None;
    for raw in text.lines() {
        let is_body = raw.starts_with(' ') || raw.starts_with('\t') || raw.trim().is_empty();
        if is_body {
            if let Some(name) = &current {
                // Full-line comments are prose about other recipes as often as
                // about this one; treating them as edges made this gate pass on
                // the very tree that motivated it.
                if !raw.trim_start().starts_with('#') {
                    let e: &mut (Vec<String>, String) = out.get_mut(name).unwrap();
                    e.1.push_str(raw);
                    e.1.push('\n');
                }
            }
            continue;
        }
        if raw.starts_with('#') || raw.starts_with('[') || raw.starts_with('@') {
            continue; // attribute or comment; keep the current recipe context out of it
        }
        // `name [params…]: [deps…]` — but not `NAME := value`.
        let Some(colon) = raw.find(':') else {
            current = None;
            continue;
        };
        if raw[colon..].starts_with(":=") {
            current = None;
            continue;
        }
        let head = &raw[..colon];
        let Some(name) = head.split_whitespace().next() else {
            current = None;
            continue;
        };
        if !name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
        {
            current = None;
            continue;
        }
        let deps: Vec<String> = raw[colon + 1..]
            .split_whitespace()
            .map(|d| d.trim_matches(|c| c == '(' || c == ')').to_string())
            .collect();
        out.insert(name.to_string(), (deps, String::new()));
        current = Some(name.to_string());
    }
    out
}

/// Fixture platform tokens produced by the recipe graph rooted at
/// `build-fixtures` in `just/<file>` for module `module`.
fn producible_tokens(module: &str, file: &std::path::Path) -> BTreeSet<String> {
    let text = std::fs::read_to_string(file).unwrap_or_default();
    let recipes = parse_just_module(&text);
    let mut seen: BTreeSet<String> = BTreeSet::new();
    let mut stack = vec!["build-fixtures".to_string()];
    let mut tokens = BTreeSet::new();
    while let Some(name) = stack.pop() {
        if !seen.insert(name.clone()) {
            continue;
        }
        let Some((deps, body)) = recipes.get(&name) else {
            continue;
        };
        stack.extend(deps.iter().cloned());
        for line in body.lines() {
            // `[workspace-]fixtures-build.sh <platform> …`
            for marker in ["fixtures-build.sh "] {
                let mut rest = line;
                while let Some(i) = rest.find(marker) {
                    rest = &rest[i + marker.len()..];
                    if let Some(tok) = rest.split_whitespace().next() {
                        tokens.insert(tok.to_string());
                    }
                }
            }
            // `just <module> <recipe>` — an edge to a sibling recipe.
            let call = format!("just {module} ");
            if let Some(i) = line.find(&call)
                && let Some(next) = line[i + call.len()..].split_whitespace().next()
            {
                stack.push(next.to_string());
            }
        }
    }
    tokens
}

#[test]
fn every_fixture_token_is_producible_by_the_module_that_owns_it() {
    let Ok(justfile) = std::fs::read_to_string(nros_tests::project_root().join("justfile")) else {
        return; // packaged crate — not a failure
    };
    // module -> just/<file>, read from the `mod` lines rather than hardcoded, so
    // a renamed module file fails as a missing edge, not as a silent pass.
    let mut files = std::collections::BTreeMap::new();
    for line in justfile.lines() {
        if let Some(rest) = line.strip_prefix("mod ") {
            let mut it = rest.split('\'');
            let name = it.next().unwrap_or("").trim().to_string();
            if let Some(path) = it.next() {
                files.insert(name, path.to_string());
            }
        }
    }

    let mut missing = Vec::new();
    for &p in PlatformId::ALL {
        // Platforms whose fixtures are NOT produced through the two manifest
        // builders. Same exemption set as `every_runtime_cell_has_a_fixture_row`
        // above, and for the same reason — each names its real build channel:
        //   - ZephyrNativeSim / Fvp / ZephyrQemuCortexM: the west leaves lane
        //     (`scripts/build/zephyr-fixture-leaves.sh`, `just zephyr build-fvp-*`),
        //     which carries its own staleness signature and no fixtures.toml row.
        //   - Px4: a CarveOut on every cell; no runner builds SITL, so no recipe
        //     can produce it and demanding one would be the inverse lie.
        if matches!(
            p,
            PlatformId::ZephyrNativeSim
                | PlatformId::Fvp
                | PlatformId::Px4
                | PlatformId::ZephyrQemuCortexM
        ) {
            continue;
        }
        let module = p.just_module();
        let Some(file) = files.get(module) else {
            missing.push(format!(
                "{p:?}: justfile declares no `mod {module}` — nothing can build it"
            ));
            continue;
        };
        let produced = producible_tokens(module, &nros_tests::project_root().join(file));
        for &token in p.fixture_tokens() {
            if !produced.contains(token) {
                missing.push(format!(
                    "{p:?}: `just {module} build-fixtures` cannot produce fixture platform \
                     `{token}` — the recipe graph rooted there builds {produced:?}. A lane that \
                     selects this coordinate schedules that module and nothing else, so the \
                     build finishes green and the staleness gate then fails on a fixture no \
                     recipe in the run could create (issue #405)"
                ));
            }
        }
    }
    assert!(
        missing.is_empty(),
        "lane coordinates that no recipe on the lane's own path can produce:\n{}",
        missing.join("\n")
    );
}

// ============================================================================
// Issue 0352 / phase-324 — interop/bridge cell↔test binding gates.
//
// `matrix::CELLS` is baked-only; interop/bridge cells live in `interop::CELLS`
// with a peer + direction + build channel + test. These four gates enforce the
// correspondence the fixtures.toml coordinate cannot (there is no baked fixture
// row for an interop cell — its nano side comes off the west leaves / native
// example lane and its peer is ephemeral). Together they make the issue-0341
// defect-2 drift class — a cell whose declared (platform, rmw) disagrees with
// what its test builds/runs — a gate failure rather than a silent pass.
// ============================================================================

/// G1 — test coverage. Every Runtime interop/bridge cell names a test binary
/// whose SOURCE FILE exists (`tests/<test>.rs`); every carved-out cell names no
/// test. A cell pointed at a test file that does not exist fails here.
///
/// It does NOT establish that anything is AIMED at the binary, and this doc
/// comment claimed it did ("a Runtime cell nothing runs … fails here") until
/// phase-433 W4 measured it: 9 of the 17 Runtime cells named a binary that no
/// `just` recipe and no workflow names, and all five gates here were green.
/// (They are still swept by root `just test-all`, which filters only by lane
/// coordinate — but a sweep cannot be aimed at one cell or bring up its peer,
/// and a `skip!` inside one is rewritten to `<skipped>`.) That half is
/// `check-interop-cell-runners` (issue 1127), which has to live outside this
/// file — the answer is in `just/**/*.just` and `.github/workflows/`, not in
/// the table.
#[test]
fn interop_bindings_g1_every_runtime_cell_names_a_real_test() {
    let tests_dir = nros_tests::project_root().join("packages/testing/nros-tests/tests");
    let mut bad = Vec::new();
    for c in interop::CELLS {
        match c.cell.tier {
            Tier::Runtime => {
                if c.test == NO_TEST {
                    bad.push(format!("{}: Runtime but carries NO_TEST", c.id));
                    continue;
                }
                if !tests_dir.join(format!("{}.rs", c.test)).is_file() {
                    bad.push(format!(
                        "{}: names test `{}` — no such tests/{}.rs",
                        c.id, c.test, c.test
                    ));
                }
            }
            Tier::CarveOut(_) | Tier::BuildOnly(_) => {
                if c.test != NO_TEST {
                    bad.push(format!(
                        "{}: non-Runtime cell must carry NO_TEST, has `{}`",
                        c.id, c.test
                    ));
                }
            }
        }
    }
    assert!(
        bad.is_empty(),
        "interop G1 (test coverage) violations:\n{}",
        bad.join("\n")
    );
}

/// G2 — build-coord match. Every interop cell's build channel can actually build
/// its platform, and the channel's `just` module matches the platform's. A cell
/// pointed at a channel that cannot produce its coordinate — e.g. the zephyr QoS
/// cell mis-declared to build via `NativeFixtures` (the shape issue 0341 defect 2
/// took) — fails here.
#[test]
fn interop_bindings_g2_build_channel_matches_platform() {
    let mut bad = Vec::new();
    for c in interop::CELLS {
        if !c.build.builds_platform(c.cell.platform) {
            bad.push(format!(
                "{}: build channel {:?} cannot build platform {:?}",
                c.id, c.build, c.cell.platform
            ));
        }
        if c.build.just_module() != c.cell.platform.just_module() {
            bad.push(format!(
                "{}: build channel module {:?} ≠ platform module {:?}",
                c.id,
                c.build.just_module(),
                c.cell.platform.just_module()
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "interop G2 (build-coord) violations:\n{}",
        bad.join("\n")
    );
}

/// G3 — tier correspondence. Each cell's build recipe is `just <module> …` for
/// the same module the channel declares, so the recipe a tier runs to BUILD the
/// nano side names the module that owns it. (The runtime trigger — that the tier
/// running the test also runs this build — is enforced by the recipe living in
/// that module's fixture build, cross-ref phase-319 presence→truth.)
#[test]
fn interop_bindings_g3_build_recipe_names_its_module() {
    let mut bad = Vec::new();
    for c in interop::CELLS {
        let prefix = format!("just {} ", c.build.just_module());
        if !c.build.build_recipe().starts_with(&prefix) {
            bad.push(format!(
                "{}: build recipe {:?} does not start with {:?}",
                c.id,
                c.build.build_recipe(),
                prefix
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "interop G3 (tier/recipe) violations:\n{}",
        bad.join("\n")
    );
}

/// G4 — peer declaration. Every cell's peer is internally consistent with its
/// cell: a single-RMW peer's rmw matches the cell, a bridge's ingress matches the
/// cell's rmw and differs from its egress. This is the check that catches an rmw
/// drift — the zephyr QoS cell reverted to Cyclonedds while its peer stays a
/// zenoh `RosEdition` fails here.
#[test]
fn interop_bindings_g4_peer_consistent_with_cell() {
    let mut bad = Vec::new();
    for c in interop::CELLS {
        if !c.peer.consistent_with(c.cell()) {
            bad.push(format!(
                "{}: peer {:?} inconsistent with cell rmw {:?}",
                c.id, c.peer, c.cell.rmw
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "interop G4 (peer-decl) violations:\n{}",
        bad.join("\n")
    );
}

/// G5 — lane narrowability. Every interop cell's coordinate must be produced by
/// at least one `fixtures.toml` row.
///
/// # What this protects (issue 0770)
///
/// A lane asks two different questions — which fixtures must be FRESH (its
/// coordinate cover) and which must EXIST (a property of the run, issue 0482).
/// They stay consistent only because the fixture resolver can NARROW: an
/// out-of-lane coordinate reports `[SKIPPED:lane]` instead of being demanded
/// fresh from a build that was told not to produce it.
///
/// That narrowing is keyed on a manifest ROW. `require_prebuilt_row_binary`
/// calls `require_coord_in_lane(&row.coord, …)`, and the path-keyed sibling
/// attributes an artifact back to a row before deciding. Either way, **a
/// coordinate no row produces cannot be narrowed** — `attribute_path` returns
/// `None`, whose documented contract is "never skip", so such a cell is
/// silently required in EVERY lane while no lane's build cover necessarily
/// contains it. The failure then surfaces far away, as a stale-fixture verdict
/// that reads like a build-system defect (issue 0445's absorbing message) and
/// gets re-diagnosed once per sweep.
///
/// Interop cells are the ones at risk, because by RFC-0051 they deliberately
/// have no `[[fixture]]` row of their OWN — the peer is ephemeral and the nano
/// side is a plain example. That is fine precisely as long as the nano side's
/// coordinate is still manifest-backed, which is what this asserts.
///
/// # Why it compares tokens through the shared mappings
///
/// `PlatformId::fixture_tokens()` is a SLICE, and `Rmw` deliberately has no
/// blessed `Display` (the same value is spelled `cyclone` by native consumers
/// and `cyclonedds` in the manifest). So this maps forward through
/// `fixture_tokens()` and backward through this file's own `rmw_from_str`,
/// rather than lowercasing an enum name — an ad-hoc normalisation of exactly
/// that kind reported two false violations while this gate was being written
/// (`ZephyrNativeSim` → `zephyrnativesim`, which matches nothing).
///
/// # Two exemptions, both load-bearing
///
/// **`workspace_fixture` rows count.** They carry a coordinate and narrow
/// through `attribute_workspace_id` / `require_workspace_in_lane` rather than
/// by path, which is a different mechanism but the same guarantee. Counting
/// only `kind = "fixture"` falsely flagged `zephyr-qos-rust-zenoh`, whose
/// coordinate is backed by seven workspace rows and zero plain ones.
///
/// **`Tier::CarveOut` cells are skipped.** A carve-out records a lane that
/// deliberately does not exist, so demanding a fixture row for it would be
/// demanding the thing the carve-out exists to say we do not build. That is
/// what `zephyr-qos-cpp-cyclone-CARVED` is, and its recorded reason says so.
///
/// Note what is NOT exempted: `builder = "west"` rows are absent from
/// `manifest_rows()` because west leaves are built module-level and are
/// deliberately unattributable. A Runtime cell that could only be backed by a
/// west row would therefore still fail here — correctly, because nothing could
/// narrow it.
#[test]
fn interop_bindings_g5_cells_are_lane_narrowable() {
    let rows = nros_tests::fixtures::lane::manifest_rows();
    // Precondition, not decoration: an empty manifest would make every cell
    // below "unbacked" and this test would fail for the wrong reason — or, if
    // the loop were inverted, pass having checked nothing.
    assert!(
        rows.iter().any(|r| r.kind == "fixture"),
        "no fixture rows parsed from the manifest — the check below would be vacuous"
    );

    let mut bad = Vec::new();
    for c in interop::CELLS {
        // A carve-out is a recorded absence of a lane; it has no fixture on
        // purpose. See the doc comment.
        if matches!(c.cell.tier, Tier::CarveOut(_)) {
            continue;
        }
        let platform_tokens = c.cell.platform.fixture_tokens();
        let lang = c.cell.lang.as_str();
        let backed = rows.iter().any(|r| {
            (r.kind == "fixture" || r.kind == "workspace_fixture")
                && platform_tokens.contains(&r.coord.0.as_str())
                && r.coord.1 == lang
                && rmw_from_str(&r.coord.2) == Some(c.cell.rmw)
        });
        if !backed {
            bad.push(format!(
                "{}: coordinate {:?}/{}/{:?} is produced by NO fixtures.toml row, \
                 so no lane can narrow it — it will be required fresh in every \
                 lane whose build may not cover it",
                c.id, platform_tokens, lang, c.cell.rmw
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "interop G5 (lane-narrowability) violations:\n{}",
        bad.join("\n")
    );
}
