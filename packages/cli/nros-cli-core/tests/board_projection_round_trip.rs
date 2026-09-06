//! The projection carries every fact the descriptor states, and nothing else.
//!
//! RFC-0064 R5 D4. This REPLACES `phase215_f_manifest_drift.rs`, which compared
//! a board's `Cargo.toml` face against its `board.cmake` face — and checked
//! ZERO boards, for two independent reasons:
//!
//! * it walked top-level `packages/boards/nros-board-*/` dirs needing BOTH
//!   files, and the only `board.cmake` in the tree sat one level deeper, in a
//!   bundle; and
//! * no board in the tree carried `[package.metadata.nros.board]` at all, so
//!   even a fixed glob would have taken its `continue`.
//!
//! The shape of that failure is the reason this file is written the way it is.
//! A gate that compares two things has to decide what to do when one is
//! missing, and "skip" is the answer that makes it vacuous. Here there is one
//! authored file and a mechanical projection of it, so the question does not
//! arise: every assertion below is about a board that EXISTS, and the count
//! assertion at the end refuses to pass on an empty set.

use nros_cli_core::orchestration::{
    board_descriptor::BoardCatalog,
    board_projection::{flatten_board_id, project},
};

fn repo_root() -> std::path::PathBuf {
    // CARGO_MANIFEST_DIR is `<repo>/packages/cli/nros-cli-core`, so the repo
    // root is three levels up.
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("repo root")
        .to_path_buf()
}

/// Parse `set(VAR "value")` lines back into pairs.
///
/// Deliberately a real parse of the emitted text rather than a second call into
/// the projector: the thing under test is what CMAKE will read, and a test that
/// re-derives the values instead of reading them proves only that a function is
/// deterministic.
fn parse_projection(text: &str) -> Vec<(String, String)> {
    text.lines()
        .filter_map(|l| {
            let rest = l.strip_prefix("set(")?.strip_suffix(')')?;
            let (var, value) = rest.split_once(" \"")?;
            Some((var.to_string(), value.trim_end_matches('"').to_string()))
        })
        .collect()
}

#[test]
fn every_zephyr_board_projects_its_descriptor_exactly() {
    let root = repo_root();
    let catalog = BoardCatalog::load(&root).expect("load board catalog");
    let mut projected = 0;

    for board in catalog.descriptors() {
        let Some(zephyr) = board.zephyr.as_ref() else {
            continue;
        };
        let descriptor = board.source.as_deref().expect("descriptor path recorded");
        let board_dir = root.join(descriptor).parent().unwrap().to_path_buf();
        let module_dir = board_dir.join("build/rust-support");

        let text = project(board, &board_dir, &module_dir).expect("project");
        let vars: std::collections::BTreeMap<String, String> =
            parse_projection(&text).into_iter().collect();

        // --- every AUTHORED fact reaches cmake -----------------------------
        assert_eq!(
            vars.get("NROS_BOARD_ZEPHYR_ID").map(String::as_str),
            Some(zephyr.west_board.as_str()),
            "{descriptor}: the board id must reach cmake"
        );
        for (field, var) in [
            (&zephyr.sdk_abi, "NROS_BOARD_TOOLCHAIN"),
            (&zephyr.default_rmw, "NROS_BOARD_DEFAULT_RMW"),
            (&zephyr.default_transport, "NROS_BOARD_DEFAULT_TRANSPORT"),
            (&zephyr.runner, "NROS_BOARD_RUNNER"),
        ] {
            assert_eq!(
                vars.get(var).map(String::as_str),
                field.as_deref(),
                "{descriptor}: {var} must match the descriptor, including being \
                 ABSENT when the descriptor says nothing — an empty string would \
                 override Zephyr's own default with nothing"
            );
        }

        // --- every DERIVED path is derived, and exists ---------------------
        let flat = flatten_board_id(&zephyr.west_board);
        for (var, rel) in [
            ("NROS_BOARD_PRJ_CONF", "prj.conf".to_string()),
            ("NROS_BOARD_BOARD_CONF", format!("boards/{flat}.conf")),
            ("NROS_BOARD_BOARD_OVERLAY", format!("boards/{flat}.overlay")),
        ] {
            let expected = board_dir.join(&rel);
            match vars.get(var) {
                Some(got) => {
                    assert_eq!(
                        got,
                        &expected.display().to_string(),
                        "{descriptor}: {var} must be the derived path"
                    );
                    assert!(
                        expected.is_file(),
                        "{descriptor}: {var} points at {} which does not exist",
                        expected.display()
                    );
                }
                // Absent is correct when the file is absent: these are
                // present-if-exists, which is what lets a board that needs no
                // overlay say nothing rather than name an empty file.
                None => assert!(
                    !expected.is_file(),
                    "{descriptor}: {} exists but {var} was not emitted",
                    expected.display()
                ),
            }
        }

        // --- provisioning, and the one derived flag ------------------------
        let rust_targets = board
            .provisioning
            .as_ref()
            .map(|p| p.rust_targets.clone())
            .unwrap_or_default();
        if board.provisioning.is_some() {
            assert_eq!(
                vars.get("NROS_BOARD_REQUIRES_RUST").map(String::as_str),
                Some(if rust_targets.is_empty() { "n" } else { "y" }),
                "{descriptor}: REQUIRES_RUST is DERIVED from rust_targets being \
                 non-empty. It used to be a second authored field stating whether \
                 the first was empty — one fact with two homes, which can disagree \
                 with itself"
            );
        }
        assert_eq!(
            vars.contains_key("NROS_BOARD_RUST_SUPPORT_MODULE"),
            !rust_targets.is_empty(),
            "{descriptor}: the Rust-support module is referenced exactly when the \
             board asks for Rust"
        );

        // --- nothing else -------------------------------------------------
        // The projection is a projection: a variable cmake reads that no
        // descriptor field produced is a fact with no author.
        const KNOWN: &[&str] = &[
            "NROS_BOARD_ZEPHYR_ID",
            "NROS_BOARD_TOOLCHAIN",
            "NROS_BOARD_DEFAULT_RMW",
            "NROS_BOARD_DEFAULT_TRANSPORT",
            "NROS_BOARD_RUNNER",
            "NROS_BOARD_PRJ_CONF",
            "NROS_BOARD_BOARD_CONF",
            "NROS_BOARD_BOARD_OVERLAY",
            "NROS_BOARD_ZEPHYR_LINE",
            "NROS_BOARD_RMW_SOURCE",
            "NROS_BOARD_GATED_PKGS",
            "NROS_BOARD_REQUIRES_RUST",
            "NROS_BOARD_RUST_TARGETS",
            "NROS_BOARD_RUST_SUPPORT_MODULE",
        ];
        for var in vars.keys() {
            assert!(KNOWN.contains(&var.as_str()), "{descriptor}: unknown {var}");
        }

        projected += 1;
    }

    // The failure mode this whole file exists to avoid. `phase215_f` passed
    // green while examining nothing.
    assert!(
        projected > 0,
        "no board carries a [board.zephyr] block — refusing to pass on an empty \
         set, which is exactly how the drift gate this replaces stayed green \
         while checking zero boards"
    );
}

#[test]
fn a_board_with_no_zephyr_block_is_refused_with_a_reason() {
    let root = repo_root();
    let catalog = BoardCatalog::load(&root).expect("load board catalog");
    let non_zephyr = catalog
        .descriptors()
        .iter()
        .find(|d| d.zephyr.is_none())
        .expect("some board is not a Zephyr board");

    let err = project(non_zephyr, &root, &root).expect_err("must refuse");
    let msg = err.to_string();
    assert!(
        msg.contains("[board.zephyr]") && msg.contains("west_board"),
        "the refusal must say what is missing and how to add it; got: {msg}"
    );
}
