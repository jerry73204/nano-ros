//! The Rust-entry parity corpus, CLI side (phase-432 W2.4).
//!
//! There are two producers of a Rust entry and there always will be: the
//! `nros::main!()` proc-macro renders with `quote!` at compile time, and this
//! crate renders with `minijinja`. Rendering the macro through the pack was
//! considered and rejected (RFC-0091 §7) — it compiles `minijinja` into every
//! user's entry build, which is the dependency weight that created this
//! duplication once already (issue 0083).
//!
//! So the two renderers stay and the FACTS are shared: both consume
//! [`nros_entry_lower::LoweredEntry`]. This module renders the shared corpus
//! at `packages/cli/nros-entry-lower/testdata/parity/*.json` and byte-pins the
//! result; `nros-macros`' own `entry_parity` test renders the SAME corpus with
//! `quote!` and compares against these files, token for token.
//!
//! That is the diff `emit_rust.rs` has claimed to exist for as long as it has
//! existed. It did not, and building it immediately found a real divergence:
//! the CLI rendered a QoS override's codes as `1, 2, 5` where `quote!`
//! interpolating a `u8` emits `Literal::u8_suffixed`, i.e. `1u8, 2u8, 5u32`.
//! Semantically identical, textually not, and nothing compared them.
//!
//! Regenerate deliberately, and READ THE DIFF — every byte reaches a compiler:
//!
//! ```text
//! NROS_UPDATE_GOLDEN=1 cargo test -p nros-cli-core --lib codegen::entry::parity
//! ```
//!
//! Adding a corpus case makes the MACRO side red until these are regenerated.
//! That is intended: a case nobody rendered on both sides is not coverage.

use std::path::{Path, PathBuf};

use nros_entry_lower::LoweredEntry;

/// The corpus lives in `nros-entry-lower`, the crate BOTH producers depend on.
///
/// Not in either renderer's own tree, and not authored twice: a corpus each
/// side writes for itself can drift into comparing different inputs while
/// staying green, which is the exact defect class this wave exists to remove.
pub(crate) fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../nros-entry-lower/testdata/parity")
}

/// Every corpus case, by name, sorted.
///
/// Enumerated from the DIRECTORY rather than from a list, so adding a `.json`
/// file is the whole edit on this side. Both producers enumerate the same way,
/// so neither can silently cover a subset.
pub(crate) fn cases() -> Vec<(String, LoweredEntry)> {
    let dir = corpus_dir();
    let mut names: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("read parity corpus dir {}: {e}", dir.display()))
        .map(|e| e.expect("read corpus entry").path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    names.sort();

    // A corpus that shrank to nothing renders nothing and compares nothing,
    // which reads exactly like a pass. It must be an error instead.
    assert!(
        names.len() >= 2,
        "the parity corpus at {} holds {} case(s) — at least the bare and rich \
         cases must be present, or this harness compares almost nothing",
        dir.display(),
        names.len()
    );

    names
        .into_iter()
        .map(|p| {
            let name = p
                .file_stem()
                .expect("corpus file has a stem")
                .to_string_lossy()
                .into_owned();
            let text =
                std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()));
            let entry: LoweredEntry = serde_json::from_str(&text)
                .unwrap_or_else(|e| panic!("parse {} as a LoweredEntry: {e}", p.display()));
            (name, entry)
        })
        .collect()
}

/// Where this side's rendering is pinned. Beside the other entry goldens, and
/// with the same `.golden` suffix for the same reason: `cargo fmt` walked the
/// crate and REFORMATTED a golden that sat under `src/` with a bare `.rs`
/// extension, silently rewriting the bytes the harness exists to hold still.
fn golden_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("testdata/entry")
        .join(format!("parity_{name}.rs.golden"))
}

#[test]
fn every_parity_case_matches_its_golden() {
    let update = std::env::var_os("NROS_UPDATE_GOLDEN").is_some();
    let mut drift: Vec<String> = Vec::new();
    let mut checked = 0usize;

    for (name, entry) in cases() {
        let got = super::emit_rust::emit_lowered(&entry);
        let path = golden_path(&name);

        if update {
            std::fs::write(&path, &got).expect("write parity golden");
            continue;
        }

        let want = match std::fs::read_to_string(&path) {
            Ok(w) => w,
            Err(_) => {
                drift.push(format!(
                    "  {name}: no golden at {} — a corpus case was added \
                     without recording what it renders to",
                    path.display()
                ));
                continue;
            }
        };
        checked += 1;
        if got != want {
            drift.push(format!(
                "  {name}: {} is stale\n--- want\n{want}\n--- got\n{got}",
                path.display()
            ));
        }
    }

    if update {
        return;
    }

    assert!(
        drift.is_empty(),
        "parity goldens drifted:\n{}",
        drift.join("\n")
    );
    assert!(
        checked >= 2,
        "only {checked} parity golden(s) compared — the harness must not pass \
         on an empty corpus"
    );
}

/// The corpus must exercise the reset, or the parity gate cannot see the one
/// defect it is most likely to meet.
///
/// A producer that emits a per-node assignment only when it has a value
/// produces working-looking output for every node that HAS values, and leaks
/// the previous node's state into every node that does not. So a corpus of
/// rich cases alone would compare the two renderings on exactly the inputs
/// where they cannot differ.
#[test]
fn the_corpus_covers_a_node_with_nothing_set() {
    let bare = cases().into_iter().flat_map(|(_, e)| e.nodes).any(|n| {
        n.params.is_empty()
            && n.remaps.is_empty()
            && n.qos_overrides.is_empty()
            && n.identity.is_none()
    });
    assert!(
        bare,
        "no corpus case carries a node with no params, no remaps, no QoS \
         overrides and no identity — the RESET is then never compared"
    );
}

/// And it must exercise a package name that is not already an identifier:
/// `sanitize_pkg` is now one function, and this is what proves both producers
/// reach it rather than each carrying the copy they used to.
#[test]
fn the_corpus_covers_a_package_name_needing_sanitisation() {
    let dir = corpus_dir();
    let raw: String = std::fs::read_dir(&dir)
        .expect("read corpus dir")
        .map(|e| std::fs::read_to_string(e.expect("entry").path()).unwrap_or_default())
        .collect();
    assert!(
        raw.contains("talker-pkg"),
        "no corpus case names a package that is not a valid identifier — the \
         one thing `sanitize_pkg` exists for is then untested across the two \
         producers"
    );
}
