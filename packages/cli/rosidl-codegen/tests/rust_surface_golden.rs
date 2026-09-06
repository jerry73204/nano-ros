//! Golden test for the `rmw` and idiomatic Rust surfaces — phase-432 W2.5a.
//!
//! # Why this file exists
//!
//! `codegen_golden.rs` is the guard for a codegen change, and it is a good one —
//! but it renders `emit_corpus()`, which covers the `nros`, `c` and `cpp`
//! surfaces and NOT the `rmw` or idiomatic ones. Measured while collapsing the
//! per-surface field views onto the lowered IR: two of the five message surfaces
//! could be rewritten with no golden asserting a single byte of their output.
//!
//! The fingerprint hashes the `packs/rmw/*.jinja` and `packs/rust/*.jinja` TEXT,
//! so a template edit there does move it. What nothing covered is the Rust side
//! — the view builders in `generator/{msg,srv,action}.rs`, which is exactly what
//! W2.5a rewrites.
//!
//! # Why it is NOT wired into `emit_corpus`
//!
//! `emit_corpus` feeds [`rosidl_codegen::codegen_fingerprint`], which is a
//! FIXTURE STALENESS signal (RFC-0061 / phase-318): adding rows to it marks
//! every built workspace fixture in the tree stale. That is the right trade for
//! a surface whose bytes reach a fixture; `generate_message_package` and its
//! siblings have no caller outside this crate's own tests and benches today, so
//! paying a tree-wide rebuild to cover them would be a poor one. Keeping the
//! corpus here means this test can guard the rewrite without moving the
//! fingerprint.
//!
//! If the rmw/idiomatic surfaces ever gain a production caller, fold this corpus
//! into `emit_corpus` and delete this file — one golden per surface set, not two.
//!
//! # One thing the recorded bytes show that is NOT a spelling to preserve
//!
//! The corpus package is named `fingerprint-corpus`, and the Rust surfaces
//! interpolate it into `extern "C"` symbol names verbatim — so the golden
//! contains `fingerprint-corpus__msg__Capped__init`, which is not a Rust
//! identifier. The C and nros surfaces run the same name through
//! `to_c_package_name` and the Rust ones do not. That is a real, pre-existing
//! gap and it is recorded here rather than hidden, because a golden's job is to
//! say what the tool emits. Fixing it SHOULD fail this test.
//!
//! # Updating
//!
//! An intentional change SHOULD fail this test. Re-record with:
//!
//! ```console
//! $ NROS_UPDATE_GOLDEN=1 cargo test -p rosidl-codegen --test rust_surface_golden
//! ```
//!
//! then READ the diff. A byte moving here means a Rust-layer spelling changed.

use rosidl_codegen::{generate_action_package, generate_message_package, generate_service_package};
use rosidl_parser::{parse_action, parse_message, parse_service};
use std::{
    collections::{BTreeMap, HashSet},
    fs,
    path::{Path, PathBuf},
};

const CORPUS: &str = "fingerprint-corpus";

const SHAPES_MSG: &str = include_str!("fixtures/fingerprint-corpus/msg/Shapes.msg");
const NESTED_MSG: &str = include_str!("fixtures/fingerprint-corpus/msg/Nested.msg");
const BOUNDED_MSG: &str = include_str!("fixtures/fingerprint-corpus/msg/Bounded.msg");
const CAPPED_MSG: &str = include_str!("fixtures/fingerprint-corpus/msg/Capped.msg");
const PROBE_SRV: &str = include_str!("fixtures/fingerprint-corpus/srv/Probe.srv");
const PROBE_ACTION: &str = include_str!("fixtures/fingerprint-corpus/action/Probe.action");

/// Every byte the `rmw` and idiomatic Rust surfaces emit for the shared corpus,
/// keyed by a stable relative path.
///
/// Reuses `fingerprint-corpus`'s `.msg`/`.srv`/`.action` deliberately: one set of
/// shapes, so a shape added for the C side is covered on the Rust side too and
/// nobody has to remember to add it twice.
fn emit() -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    let deps: HashSet<String> = HashSet::new();

    let mut put = |name: &str, res: Result<String, String>| {
        // A build whose emitters REJECT a shape is a different tool than one
        // that accepts it — record the rejection as content rather than
        // failing, so the diff shows it (same rule as `emit_corpus`).
        let (tag, body) = match res {
            Ok(s) => ("ok", s),
            Err(e) => ("err", e),
        };
        out.insert(name.to_string(), format!("// emit:{tag}\n{body}"));
    };

    for (name, src) in [
        ("Shapes", SHAPES_MSG),
        ("Nested", NESTED_MSG),
        ("Bounded", BOUNDED_MSG),
        ("Capped", CAPPED_MSG),
    ] {
        let Ok(m) = parse_message(src) else {
            put(&format!("{name}.parse"), Err("parse-error".into()));
            continue;
        };
        let g = generate_message_package(CORPUS, name, &m, &deps);
        put(
            &format!("{name}.rmw.rs"),
            g.as_ref()
                .map(|g| g.message_rmw.clone())
                .map_err(ToString::to_string),
        );
        put(
            &format!("{name}.idiomatic.rs"),
            g.as_ref()
                .map(|g| g.message_idiomatic.clone())
                .map_err(ToString::to_string),
        );
    }

    match parse_service(PROBE_SRV) {
        Ok(sv) => {
            let g = generate_service_package(CORPUS, "Probe", &sv, &deps);
            put(
                "Probe.srv.rmw.rs",
                g.as_ref()
                    .map(|g| g.service_rmw.clone())
                    .map_err(ToString::to_string),
            );
            put(
                "Probe.srv.idiomatic.rs",
                g.as_ref()
                    .map(|g| g.service_idiomatic.clone())
                    .map_err(ToString::to_string),
            );
        }
        Err(_) => put("Probe.srv.parse", Err("parse-error".into())),
    }

    match parse_action(PROBE_ACTION) {
        Ok(ac) => {
            let g = generate_action_package(CORPUS, "Probe", &ac, &deps);
            put(
                "Probe.action.rmw.rs",
                g.as_ref()
                    .map(|g| g.action_rmw.clone())
                    .map_err(ToString::to_string),
            );
            put(
                "Probe.action.idiomatic.rs",
                g.as_ref()
                    .map(|g| g.action_idiomatic.clone())
                    .map_err(ToString::to_string),
            );
        }
        Err(_) => put("Probe.action.parse", Err("parse-error".into())),
    }

    out
}

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/rust-surface-golden")
}

fn update_requested() -> bool {
    std::env::var("NROS_UPDATE_GOLDEN").is_ok_and(|v| v != "0")
}

fn walk(dir: &Path) -> std::io::Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    for e in fs::read_dir(dir)? {
        let p = e?.path();
        if p.is_dir() {
            out.extend(walk(&p)?);
        } else {
            out.push(p);
        }
    }
    Ok(out)
}

#[test]
fn the_rust_surfaces_match_their_committed_golden() {
    let emitted = emit();
    let dir = golden_dir();

    if update_requested() {
        let _ = fs::remove_dir_all(&dir);
        for (rel, body) in &emitted {
            let path = dir.join(rel);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(&path, body).unwrap();
        }
        eprintln!(
            "re-recorded {} golden files under {}",
            emitted.len(),
            dir.display()
        );
        return;
    }

    assert!(
        dir.is_dir(),
        "golden dir {} is missing — record it with \
         NROS_UPDATE_GOLDEN=1 cargo test -p rosidl-codegen --test rust_surface_golden",
        dir.display()
    );

    let mut problems = Vec::new();
    for (rel, body) in &emitted {
        match fs::read_to_string(dir.join(rel)) {
            Err(_) => problems.push(format!("  + {rel}   (new output, no golden)")),
            Ok(on_disk) if &on_disk != body => {
                problems.push(format!("  ~ {rel}   ({})", first_diff(&on_disk, body)));
            }
            Ok(_) => {}
        }
    }
    // A golden nothing emits any more is stale coverage.
    for p in walk(&dir).unwrap_or_default() {
        let rel = p
            .strip_prefix(&dir)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/");
        if !emitted.contains_key(&rel) {
            problems.push(format!("  - {rel}   (golden nothing emits)"));
        }
    }

    assert!(
        problems.is_empty(),
        "the rmw / idiomatic Rust surfaces do not match their committed golden:\n{}\n\n\
         If the change is intended, re-record with \
         NROS_UPDATE_GOLDEN=1 cargo test -p rosidl-codegen --test rust_surface_golden \
         and READ the diff.",
        problems.join("\n")
    );
}

/// The first differing line, so the failure names a place rather than dumping
/// two files.
fn first_diff(expected: &str, actual: &str) -> String {
    for (i, (e, a)) in expected.lines().zip(actual.lines()).enumerate() {
        if e != a {
            return format!("line {}: expected {e:?}, got {a:?}", i + 1);
        }
    }
    format!(
        "same prefix, different length: {} vs {} lines",
        expected.lines().count(),
        actual.lines().count()
    )
}

/// The golden is only a guard if the corpus reaches BOTH surfaces. An empty or
/// half-emitted map would pass the test above for the wrong reason.
#[test]
fn the_corpus_reaches_both_rust_surfaces() {
    let emitted = emit();
    let rmw = emitted.keys().filter(|k| k.contains(".rmw.")).count();
    let idiomatic = emitted.keys().filter(|k| k.contains(".idiomatic.")).count();
    assert_eq!(rmw, 6, "expected 4 msg + 1 srv + 1 action rmw renders");
    assert_eq!(idiomatic, 6, "expected 4 msg + 1 srv + 1 action idiomatic");
    assert!(
        emitted.values().all(|b| b.starts_with("// emit:ok\n")),
        "a corpus shape the Rust surfaces reject would freeze its error text as \
         the golden — fix the emitter or drop the shape, do not record the error"
    );
}
