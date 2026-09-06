//! Phase 219.A — the CLI's Rust entry renderer, and the proc-macro's mirror.
//!
//! The canonical Rust entry is the `nros::main!()` proc-macro
//! (`packages/core/nros-macros/src/main_macro.rs`), which keeps
//! `proc_macro::Span` for diagnostics a CLI shell-out cannot match. This
//! renderer is its SECOND rendering of the same facts, and phase-432 W2.4
//! finally made that pay: both now render
//! [`nros_entry_lower::LoweredEntry`], and a parity corpus
//! (`packages/cli/nros-entry-lower/testdata/parity/`) is rendered by each and
//! compared. That comparison is the "byte-level diff against the proc-macro
//! output" this file's doc comment promised from 2024 and never had — RFC-0091
//! §7, and the reason the file is still here rather than deleted.
//!
//! **The renderer that stays is not the same thing as the VERB that went.**
//! `nros codegen entry --lang rust` is retired (`cmd/codegen.rs`): nothing
//! invoked it — `nano_ros_entry` passes `--lang` as `c` or `cpp` only, and a
//! Rust entry reaches the proc-macro through `rust_cargo_application()` — and
//! what it produced was a strictly POORER entry than the macro's, missing
//! tiers, lifecycle, param services and executor sizing. A user who found the
//! flag got an entry that silently ignored half their `system.toml`. What
//! survives is the renderer, its goldens and the parity gate.
//!
//! What is rendered here and what is not: every fact arrives computed from
//! `nros-entry-lower` and `nros_orchestration_ir`. What stays in this file is
//! SPELLING — quoting, and the syntax of a Rust tuple — because quoting is a
//! correctness property and a template that got it wrong would fail silently
//! (RFC-0091 §6b).

use nros_entry_lower::{LoweredEntry, LoweredNode};

use super::Plan;

/// The whole TU, as the template sees it.
///
/// Issue 1102 — every field is ALREADY CORRECT: `board_path` came from
/// `nros_orchestration_ir`, and every literal is already escaped. The template
/// places them; it does not compute them.
#[derive(serde::Serialize)]
struct RustEntryView {
    bringup: String,
    launch: String,
    board: String,
    board_path: &'static str,
    /// Raw-string literals, already quoted by `quote_str`.
    depfiles: Vec<String>,
    nodes: Vec<RustNodeView>,
}

/// One launch node's runtime state.
///
/// The three list fields are pre-joined literal text rather than lists,
/// because their ELEMENTS are Rust syntax (`("a", "b")`, `("t", 1, 2, 3)`)
/// assembled from escaped literals. Handing the template a list would make it
/// responsible for composing that syntax, which is the half that must stay in
/// Rust.
#[derive(serde::Serialize)]
struct RustNodeView {
    pkg: String,
    params: String,
    remaps: String,
    qos_overrides: String,
    identity: String,
}

/// Emit a Rust `main.rs` body for the given plan.
///
/// Output mirrors the proc-macro's `OwnedSpin` framework branch — RTIC and
/// Embassy stay proc-macro-only, since they need `proc_macro::Span` for the
/// `custom_tasks` splice. The body installs both a hosted `fn main()` and an
/// embedded `#[unsafe(no_mangle)] extern "C" fn main()` so the same TU works
/// for native + bare-metal targets.
pub fn emit(plan: &Plan) -> String {
    emit_lowered(&lower(plan))
}

/// Stage 2 — the CLI's `Plan` becomes the facts both Rust producers render.
///
/// This is the half the proc-macro cannot share: it has no `Plan` (RFC-0091
/// §4 — `Plan` is the CLI's own projection of its input, and lives across
/// `cmd/`, `builder/` and `codegen/`). What IS shared is the OUTPUT type, so
/// the two converge here rather than at the renderer.
pub fn lower(plan: &Plan) -> LoweredEntry {
    LoweredEntry {
        bringup: plan.bringup.clone(),
        launch: plan.launch_file.display().to_string(),
        board: plan.board.clone(),
        // include_bytes! tracking — same rebuild-correctness workaround the
        // proc-macro uses. A path that does not exist is skipped, exactly as
        // the proc-macro does: `include_bytes!` on a missing path is a hard
        // compile error, and the pkg-index walk can name a synthesised dir.
        depfiles: plan
            .depfile_paths
            .iter()
            .filter(|d| d.exists())
            .map(|d| d.display().to_string())
            .collect(),
        nodes: plan.nodes.iter().map(lower_node).collect(),
    }
}

/// Stage 3 — render the lowered entry as Rust.
///
/// Public because the parity harness renders the shared corpus through it
/// directly; the corpus is `LoweredEntry` values, which is the point.
pub fn emit_lowered(entry: &LoweredEntry) -> String {
    let view = RustEntryView {
        bringup: entry.bringup.clone(),
        launch: entry.launch.clone(),
        board: entry.board.clone(),
        board_path: board_path_for(&entry.board).unwrap_or("::nros_board_linux::LinuxBoard"),
        depfiles: entry.depfiles.iter().map(|d| quote_str(d)).collect(),
        nodes: entry.nodes.iter().map(node_view).collect(),
    };

    // A render failure is a bug in a template compiled INTO this binary, so it
    // cannot be handled meaningfully at a call site that only has a plan.
    crate::codegen::entry::render::render("rust_entry.rs.jinja", &view)
        .expect("bundled rust entry template must render")
}

/// A plan node's per-node runtime bake, as neutral facts (issue 0302).
///
/// Four features arrived over four phases — params (264 W4a), identity
/// (268 W1), remaps (305 W3 / issue 0255), QoS overrides (issue #52) — and
/// each wired the proc-macro while leaving this emitter behind, so a CLI-baked
/// entry ran every node with default parameters, no remaps, its own hardcoded
/// name and no QoS overrides. From the same plan. That is the drift the shared
/// [`LoweredNode`] and the parity corpus exist to make impossible: a fifth
/// feature now cannot reach one producer without the other going red.
fn lower_node(n: &super::PlanNode) -> LoweredNode {
    LoweredNode {
        pkg: n.pkg.clone(),
        params: n.params.clone(),
        remaps: n.remaps.clone(),
        // The plan carries LOWERED codes: `nros_orchestration_ir::qos_override`
        // already rejected anything unusable (issue 0303), so nothing is
        // decoded or silently dropped here.
        qos_overrides: n
            .qos_overrides
            .iter()
            .map(|o| nros_entry_lower::QosOverride {
                topic: o.topic.clone(),
                role: o.role,
                policy: o.policy,
                value: o.value,
            })
            .collect(),
        // A namespace without a name is not an identity: the proc-macro keys
        // the override on the name, so `None` here means "keep the node's own".
        identity: n.name.as_ref().map(|name| {
            nros_entry_lower::NodeIdentity::new(name, n.namespace.as_deref().unwrap_or(""))
        }),
    }
}

/// Spell one lowered node as Rust.
///
/// EVERY field is written unconditionally, including the empty case. That
/// reset discipline is the macro's and it is load-bearing: `runtime` is reused
/// across nodes, so a node with no params must clear the previous node's
/// rather than inherit them.
fn node_view(n: &LoweredNode) -> RustNodeView {
    let pairs = |items: &[(String, String)]| -> String {
        items
            .iter()
            .map(|(a, b)| format!("({}, {})", lit_str(a), lit_str(b)))
            .collect::<Vec<_>>()
            .join(", ")
    };

    // SUFFIXED literals (`1u8`), because `quote!` interpolating a `u8` emits
    // `Literal::u8_suffixed` and the proc-macro therefore always has. The two
    // renderings differed here for as long as both existed — semantically
    // identical, textually not — and nothing compared them, which is precisely
    // what the parity corpus was built to find.
    let qos_overrides = n
        .qos_overrides
        .iter()
        .map(|o| {
            format!(
                "({}, {}u8, {}u8, {}u32)",
                lit_str(&o.topic),
                o.role,
                o.policy,
                o.value
            )
        })
        .collect::<Vec<_>>()
        .join(", ");

    let identity = match n.identity_pair() {
        Some((name, namespace)) => format!(
            "::core::option::Option::Some(({}, {}))",
            lit_str(name),
            lit_str(namespace)
        ),
        None => "::core::option::Option::None".to_string(),
    };

    RustNodeView {
        pkg: n.ident(),
        params: pairs(&n.params),
        remaps: pairs(&n.remaps),
        qos_overrides,
        identity,
    }
}

/// Board key → Rust ZST path.
///
/// Delegates to [`nros_orchestration_ir::board_path_for`], the single source
/// of truth shared with the `nros::main!()` proc-macro. Any board added to
/// the IR crate is automatically available here with no extra edit.
fn board_path_for(board: &str) -> Option<&'static str> {
    nros_orchestration_ir::board_path_for(board)
}

/// Quote a string into a valid Rust string literal (raw form when
/// possible so backslashes in path components survive on Windows
/// hosts).
/// Quote a value as a PLAIN Rust string literal, escaping as needed.
///
/// The `nros::main!` proc-macro emits these through `LitStr`, i.e. plain
/// quoted form. This emitter exists to be byte-diffable against that output
/// (issue 0302), so it matches rather than using the raw-string form
/// [`quote_str`] uses for paths.
fn lit_str(s: &str) -> String {
    let escaped = s.replace('\\', "\\\\").replace('"', "\\\"");
    format!("\"{escaped}\"")
}

fn quote_str(s: &str) -> String {
    // Pick a raw-string hash count that doesn't collide with the
    // string's own quote sequences. For paths the input is overwhelm-
    // ingly free of `"#` runs, so a single `#` works.
    let mut hashes = 1usize;
    loop {
        let needle = format!("\"{}", "#".repeat(hashes));
        if !s.contains(&needle) {
            break;
        }
        hashes += 1;
    }
    let hs = "#".repeat(hashes);
    format!("r{hs}\"{s}\"{hs}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::entry::PlanNode;
    use std::path::PathBuf;

    /// Issue 0302 — every per-node field the `nros::main!` proc-macro sets must
    /// be baked here too, INCLUDING the empty case.
    ///
    /// The reset is the point: `runtime` is reused across nodes, so a node with
    /// no params must clear the previous node's rather than inherit them.
    /// Emitting the four assignments only when non-empty would leak state
    /// between nodes and pass a naive "does it contain the value" test.
    #[test]
    fn every_node_gets_the_full_runtime_state_reset() {
        let mut plan = fixture_plan(&[("talker_pkg", "talker"), ("listener_pkg", "listener")]);
        plan.nodes[0].params = vec![("rate".into(), "25".into())];
        plan.nodes[0].remaps = vec![("chatter".into(), "/ns/chatter".into())];
        plan.nodes[0].name = Some("talker".into());
        plan.nodes[0].namespace = Some("/ns".into());
        // node 1 deliberately left bare — it must still be RESET.

        let out = emit(&plan);

        assert!(
            out.contains(r#"runtime.params = &[("rate", "25")];"#),
            "{out}"
        );
        assert!(
            out.contains(r#"runtime.remaps = &[("chatter", "/ns/chatter")];"#),
            "{out}"
        );
        assert!(
            out.contains(
                r#"runtime.node_identity = ::core::option::Option::Some(("talker", "/ns"));"#
            ),
            "{out}"
        );

        // Both nodes reset all four; the bare one gets empties, not omissions.
        assert_eq!(out.matches("runtime.params = &[").count(), 2, "{out}");
        assert_eq!(out.matches("runtime.remaps = &[").count(), 2, "{out}");
        assert_eq!(
            out.matches("runtime.qos_overrides = &[").count(),
            2,
            "{out}"
        );
        assert_eq!(out.matches("runtime.node_identity = ").count(), 2, "{out}");
        assert!(
            out.contains("runtime.params = &[];"),
            "bare node must reset:\n{out}"
        );
        assert!(
            out.contains("runtime.node_identity = ::core::option::Option::None;"),
            "a node with no launch name must clear the previous identity:\n{out}"
        );
    }

    /// The state must be written BEFORE the register call it configures —
    /// after it would configure the next node, or nothing.
    #[test]
    fn state_is_emitted_before_the_register_call() {
        let mut plan = fixture_plan(&[("talker_pkg", "talker")]);
        plan.nodes[0].params = vec![("rate".into(), "25".into())];
        let out = emit(&plan);

        let params_at = out.find("runtime.params").expect("params emitted");
        let register_at = out
            .find("::talker_pkg::register")
            .expect("register emitted");
        assert!(
            params_at < register_at,
            "params must precede the register call:\n{out}"
        );
    }

    fn fixture_plan(nodes: &[(&str, &str)]) -> Plan {
        Plan {
            board: "native".into(),
            nodes: nodes
                .iter()
                .map(|(pkg, exec)| PlanNode {
                    pkg: (*pkg).into(),
                    exec: (*exec).into(),
                    name: None,
                    namespace: None,
                    class_name: None,
                    class_header: None,
                    lang: None,
                    shape: None,
                    qos_overrides: Vec::new(),
                    params: Vec::new(),
                    remaps: Vec::new(),
                    callback_groups: Vec::new(),
                    sched_context: None,
                    group_tiers: std::collections::BTreeMap::new(),
                })
                .collect(),
            depfile_paths: Vec::new(),
            bringup: "demo_bringup".into(),
            launch_file: PathBuf::from("/tmp/system.launch.xml"),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers: Default::default(),
            node_overrides: Vec::new(),
            resolved_tiers: None,
        }
    }

    #[test]
    fn emit_two_node_plan_contains_register_calls() {
        let plan = fixture_plan(&[("talker_pkg", "talker"), ("listener_pkg", "listener")]);
        let src = emit(&plan);
        assert!(src.contains("::talker_pkg::register(runtime)?;"));
        assert!(src.contains("::listener_pkg::register(runtime)?;"));
        assert!(src.contains("LinuxBoard"));
        // Both a hosted main and an embedded main.
        assert!(src.contains("#[cfg(not(target_os = \"none\"))]"));
        assert!(src.contains("#[cfg(target_os = \"none\")]"));
    }

    #[test]
    fn dash_pkg_names_are_sanitised() {
        let plan = fixture_plan(&[("talker-pkg", "talker")]);
        let src = emit(&plan);
        assert!(src.contains("::talker_pkg::register(runtime)?;"));
    }

    #[test]
    fn freertos_board_maps_to_correct_zst() {
        let mut plan = fixture_plan(&[("talker_pkg", "talker")]);
        plan.board = "freertos".into();
        let src = emit(&plan);
        assert!(src.contains("::nros_board_mps2_an385_freertos::Mps2An385"));
    }

    #[test]
    fn quote_str_handles_simple_paths() {
        let q = quote_str("/abs/path.xml");
        assert!(q.starts_with("r#\""));
        assert!(q.ends_with("\"#"));
        assert!(q.contains("/abs/path.xml"));
    }
}
