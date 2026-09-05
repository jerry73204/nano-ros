//! Issue 1102 — a committed record of what every entry emitter actually emits.
//!
//! The entry emitters build target-language source by appending strings, so
//! the generated text exists nowhere you can read it: answering "what C++ does
//! a two-component zenoh image get?" means executing ~130 append sites in your
//! head. Issue 1003 is what that cost — a missing session name lived for three
//! months beside a CORRECT sibling producer, because neither spelling was a
//! document anyone could put side by side.
//!
//! These goldens are that document. They are also the equivalence harness for
//! moving the emitters onto templates: the conversion is only safe if the
//! bytes do not move, and this is what proves it. It must therefore be
//! committed and passing BEFORE the first emitter changes — a harness written
//! afterwards can only confirm whatever the new code happens to do.
//!
//! Regenerate deliberately:
//!
//! ```text
//! NROS_UPDATE_GOLDEN=1 cargo test -p nros-cli-core --lib codegen::entry::golden
//! ```
//!
//! A diff here is never noise. Every byte reaches a compiler on a target.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use super::{Plan, PlanNode};

/// Which emitter a case runs through.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Lang {
    Cpp,
    C,
    Rust,
}

impl Lang {
    fn ext(self) -> &'static str {
        match self {
            Lang::Cpp => "cpp",
            Lang::C => "c",
            Lang::Rust => "rs",
        }
    }
}

fn node(pkg: &str, exec: &str, class: Option<(&str, &str)>) -> PlanNode {
    PlanNode {
        pkg: pkg.into(),
        exec: exec.into(),
        name: None,
        namespace: None,
        class_name: class.map(|(c, _)| c.into()),
        class_header: class.map(|(_, h)| h.into()),
        lang: None,
        shape: None,
        qos_overrides: Vec::new(),
        params: Vec::new(),
        remaps: Vec::new(),
        callback_groups: Vec::new(),
        sched_context: None,
        group_tiers: BTreeMap::new(),
    }
}

/// A plan with no filesystem dependence.
///
/// `depfile_paths` is deliberately EMPTY: `emit_rust` skips a tracked path that
/// does not exist, so a non-empty list would make the golden depend on what is
/// on the machine running the test.
fn plan(board: &str, nodes: Vec<PlanNode>) -> Plan {
    Plan {
        board: board.into(),
        nodes,
        depfile_paths: vec![],
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

/// A C component: `emit_c` refuses any node whose `lang` is not `c`, so the
/// C rows must say so or their goldens record only that refusal.
fn c_node(pkg: &str, exec: &str) -> PlanNode {
    let mut n = node(pkg, exec, None);
    n.lang = Some("c".into());
    n
}

fn typed_node(pkg: &str, exec: &str) -> PlanNode {
    let class = format!("{pkg}::{}", exec.to_uppercase());
    let header = format!("{pkg}/{exec}.hpp");
    node(pkg, exec, Some((&class, &header)))
}

/// A two-tier C plan — the `run_tiers` path, which no other row reaches.
///
/// A tier's members are `(node_name, callback_group)`, and `emit_c` filters
/// nodes into tiers BY NODE NAME, so the names here must match the execs.
fn c_tiered_plan(board: &str) -> Plan {
    use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
    let tier = |name: &str, priority: i64, period: u64, member: &str, grp: &str| ResolvedTier {
        name: name.into(),
        priority,
        stack_bytes: None,
        spin_period_us: Some(period),
        preempt_threshold: None,
        time_slice_us: None,
        sched_class: None,
        class: None,
        period_us: None,
        budget_us: None,
        deadline_us: None,
        deadline_policy: None,
        core: None,
        members: vec![(member.into(), grp.into())],
    };
    let mut ctrl = c_node("ctrl_pkg", "ctrl");
    ctrl.callback_groups = vec!["ctrl_grp".into()];
    ctrl.sched_context = Some(0);
    let mut telem = c_node("telem_pkg", "telem");
    telem.callback_groups = vec!["telem_grp".into()];
    telem.sched_context = Some(1);
    let mut p = plan(board, vec![ctrl, telem]);
    p.resolved_tiers = Some(ResolvedTierTable {
        tiers: vec![
            tier("high", 80, 10_000, "ctrl", "ctrl_grp"),
            tier("low", 10, 100_000, "telem", "telem_grp"),
        ],
    });
    p
}

/// The matrix. One row per (board family x entry shape) that reaches a
/// compiler, because the emitter is what every C/C++ image boots through.
fn cases() -> Vec<(&'static str, Plan, Lang)> {
    let boards = ["native", "zephyr", "nuttx", "freertos", "threadx"];
    let mut out: Vec<(&'static str, Plan, Lang)> = Vec::new();

    // Keep the names 'static without a macro: one arm per board.
    for board in boards {
        let one = plan(board, vec![typed_node("talker_pkg", "talker")]);
        let two = plan(
            board,
            vec![
                typed_node("talker_pkg", "talker"),
                typed_node("listener_pkg", "listener"),
            ],
        );
        let (n1, n2, nc) = match board {
            "native" => ("cpp_native_one", "cpp_native_two", "c_native_one"),
            "zephyr" => ("cpp_zephyr_one", "cpp_zephyr_two", "c_zephyr_one"),
            "nuttx" => ("cpp_nuttx_one", "cpp_nuttx_two", "c_nuttx_one"),
            "freertos" => ("cpp_freertos_one", "cpp_freertos_two", "c_freertos_one"),
            _ => ("cpp_threadx_one", "cpp_threadx_two", "c_threadx_one"),
        };
        out.push((n1, one, Lang::Cpp));
        out.push((n2, two, Lang::Cpp));
        out.push((
            nc,
            plan(board, vec![c_node("c_talker_pkg", "talker")]),
            Lang::C,
        ));
    }

    // The C rows above are one node with nothing set, which reaches neither
    // the per-package dedup, the multi-node storage, nor the `run_tiers` path
    // — most of `emit_c`. These three do.
    let mut c_rich = c_node("c_talker_pkg", "talker");
    c_rich.name = Some("renamed_talker".into());
    c_rich.namespace = Some("/demo".into());
    c_rich.params = vec![("rate_hz".into(), "10".into())];
    c_rich.remaps = vec![("chatter".into(), "/demo/chatter".into())];
    out.push((
        "c_native_rich",
        plan(
            "native",
            // The same package TWICE plus a second one: the forward-declaration
            // loop dedups by package, so a repeat must not emit a second
            // `extern` while it still gets its own node storage.
            vec![
                c_rich,
                c_node("c_talker_pkg", "talker2"),
                c_node("c_listener_pkg", "listener"),
            ],
        ),
        Lang::C,
    ));

    out.push(("c_native_tiers", c_tiered_plan("native"), Lang::C));
    out.push(("c_nuttx_tiers", c_tiered_plan("nuttx"), Lang::C));

    out.push((
        "rust_native_one",
        plan("native", vec![node("talker_pkg", "talker", None)]),
        Lang::Rust,
    ));

    // A bare node leaves every per-node list empty and takes the `None` arm of
    // the identity branch, so it exercises almost none of the per-node
    // rendering. This row carries params, remaps, a QoS override, a name and a
    // namespace across TWO nodes — including one that keeps its own identity,
    // so both arms appear in one golden. Quoting is the part that must stay in
    // Rust, so a value with a quote and a backslash is in here on purpose.
    let mut rich = node("talker_pkg", "talker", None);
    rich.name = Some("renamed_talker".into());
    rich.namespace = Some("/demo".into());
    rich.params = vec![
        ("rate_hz".into(), "10".into()),
        ("greeting".into(), "he said \"hi\" \\ bye".into()),
    ];
    rich.remaps = vec![("chatter".into(), "/demo/chatter".into())];
    rich.qos_overrides = vec![super::QoSOverrideSpec {
        topic: "/demo/chatter".into(),
        role: 1,
        policy: 2,
        value: 5,
    }];
    out.push((
        "rust_native_rich",
        plan("native", vec![rich, node("listener_pkg", "listener", None)]),
        Lang::Rust,
    ));
    out
}

fn render(p: &Plan, lang: Lang) -> Result<String, String> {
    match lang {
        Lang::Cpp => super::emit_cpp::emit_typed(p),
        Lang::C => super::emit_c::emit_typed(p),
        Lang::Rust => Ok(super::emit_rust::emit(p)),
    }
}

/// Where the goldens live.
///
/// OUTSIDE `src/`, and every file carries a `.golden` suffix, because a golden
/// is data and must never be treated as source. `cargo fmt` walks the crate
/// and REFORMATTED `rust_native_one.rs` when these sat under `src/` with a
/// bare `.rs` extension — silently rewriting the very bytes this harness
/// exists to hold still. `check-cli-fmt` caught it; the suffix means no
/// formatter, linter or module scan can reach them again.
fn testdata_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("testdata/entry")
}

#[test]
fn every_emitter_matches_its_golden() {
    let dir = testdata_dir();
    let update = std::env::var_os("NROS_UPDATE_GOLDEN").is_some();
    if update {
        std::fs::create_dir_all(&dir).expect("create testdata dir");
    }

    let mut drift: Vec<String> = Vec::new();
    let mut checked = 0usize;

    for (name, p, lang) in cases() {
        // An emitter that REFUSES is a fact about the plan worth recording, so
        // an error is captured as the golden rather than failing the harness:
        // silently skipping it would let a refusal appear or vanish unnoticed.
        let got = match render(&p, lang) {
            Ok(s) => s,
            Err(e) => format!("EMITTER REFUSED: {e}\n"),
        };
        let path = dir.join(format!("{name}.{}.golden", lang.ext()));

        if update {
            std::fs::write(&path, &got).expect("write golden");
            continue;
        }

        let want = match std::fs::read_to_string(&path) {
            Ok(w) => w,
            Err(_) => {
                drift.push(format!(
                    "  {name}: no golden at {} — a case was added without \
recording what it emits",
                    path.display()
                ));
                continue;
            }
        };
        checked += 1;
        if got != want {
            let first = got
                .lines()
                .zip(want.lines())
                .enumerate()
                .find(|(_, (g, w))| g != w)
                .map(|(i, (g, w))| format!("line {}:\n    was:  {w}\n    now:  {g}", i + 1))
                .unwrap_or_else(|| {
                    format!(
                        "length {} -> {} (one is a prefix of the other)",
                        want.len(),
                        got.len()
                    )
                });
            drift.push(format!("  {name}: {first}"));
        }
    }

    assert!(
        drift.is_empty(),
        "generated entry source changed for {} case(s):\n{}\n\n\
         Every byte here reaches a compiler on a target. If the change is \
         intended, regenerate and READ the diff:\n\
         \x20   NROS_UPDATE_GOLDEN=1 cargo test -p nros-cli-core --lib \
codegen::entry::golden\n",
        drift.len(),
        drift.join("\n")
    );

    assert!(
        checked > 0 || update,
        "the golden harness compared NOTHING — it would pass whatever the \
emitters did"
    );
}
