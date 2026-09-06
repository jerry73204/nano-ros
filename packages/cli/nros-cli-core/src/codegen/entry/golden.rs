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
    /// `emit_typed_probe` — the same TU with the recording tail. Its own
    /// variant because the probe is a SECOND entry point into `emit_cpp`
    /// whose tail nothing else reaches: it returns before the boot config and
    /// the board wrapper, so a corpus without it leaves the early-return
    /// unguarded.
    CppProbe,
    C,
    Rust,
}

impl Lang {
    fn ext(self) -> &'static str {
        match self {
            Lang::Cpp | Lang::CppProbe => "cpp",
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

/// A `lang == "c"` node inside a C++ entry.
///
/// `emit_cpp` validates `class_name` for EVERY node, C ones included, before
/// it looks at `lang` — so a C node in a C++ plan still carries one even
/// though nothing emits it. Recording that is the point: the requirement is
/// load-bearing and reads like an oversight.
fn cpp_c_node(pkg: &str, exec: &str) -> PlanNode {
    let mut n = typed_node(pkg, exec);
    n.lang = Some("c".into());
    n
}

/// A `lang == "rust"` node: installed through `__nros_component_<pkg>_install`
/// on the shared executor, with no entry-created `::nros::Node`.
fn cpp_rust_node(pkg: &str, exec: &str) -> PlanNode {
    let mut n = typed_node(pkg, exec);
    n.lang = Some("rust".into());
    n
}

/// An rclcpp-shape node (RFC-0044): the component OWNS its node, so the entry
/// gives it an aligned arena slot and placement-news it.
fn cpp_rclcpp_node(pkg: &str, exec: &str) -> PlanNode {
    let mut n = typed_node(pkg, exec);
    n.shape = Some("rclcpp".into());
    n
}

/// A two-tier C++ plan — the `run_tiers` path.
///
/// Tier members are matched BY NODE NAME against each node's name-or-exec, so
/// the member strings must equal the execs.
fn cpp_tiered_plan(board: &str) -> Plan {
    let mut ctrl = typed_node("ctrl_pkg", "ctrl");
    ctrl.callback_groups = vec!["ctrl_grp".into()];
    ctrl.sched_context = Some(0);
    let mut telem = typed_node("telem_pkg", "telem");
    telem.callback_groups = vec!["telem_grp".into()];
    telem.sched_context = Some(1);
    let mut p = plan(board, vec![ctrl, telem]);
    p.resolved_tiers = Some(nros_orchestration_ir::ResolvedTierTable {
        tiers: vec![
            tier_spec("high", 80, 10_000, vec![("ctrl", "ctrl_grp")]),
            tier_spec("low", 10, 100_000, vec![("telem", "telem_grp")]),
        ],
    });
    p
}

/// One resolved tier. Shared by the C++ tiered plans below; the C ones keep
/// their own closure because they set a different subset.
fn tier_spec(
    name: &str,
    priority: i64,
    period: u64,
    members: Vec<(&str, &str)>,
) -> nros_orchestration_ir::ResolvedTier {
    nros_orchestration_ir::ResolvedTier {
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
        members: members
            .into_iter()
            .map(|(n, g)| (n.to_string(), g.to_string()))
            .collect(),
    }
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

    // param-services and lifecycle close a setup function with a fixed block
    // that no other row emits. Both paths carry it — the single setup fn, and
    // TIER 0 only, because they are process facts rather than per-tier ones.
    // `emit_c`'s QoS block reached NO golden: `c_native_rich` carries params
    // and remaps but no override, so the one C statement that differs from the
    // C++ spelling of the same data — a free function on the node's ADDRESS
    // where C++ calls a method on the node — was recorded nowhere. Two
    // overrides, so the array's row separator is covered too.
    let mut c_qos = c_node("c_talker_pkg", "talker");
    c_qos.qos_overrides = vec![
        super::QoSOverrideSpec {
            topic: "/demo/chatter".into(),
            role: 1,
            policy: 2,
            value: 5,
        },
        super::QoSOverrideSpec {
            topic: "/demo/status".into(),
            role: 0,
            policy: 1,
            value: 2,
        },
    ];
    out.push(("c_native_qos", plan("native", vec![c_qos]), Lang::C));

    let mut c_svc = plan("native", vec![c_node("c_talker_pkg", "talker")]);
    c_svc.param_services = true;
    c_svc.lifecycle = Some("active".into());
    out.push(("c_native_services", c_svc, Lang::C));

    let mut c_tier_svc = c_tiered_plan("native");
    c_tier_svc.param_services = true;
    c_tier_svc.lifecycle = Some("configure".into());
    out.push(("c_native_tiers_services", c_tier_svc, Lang::C));

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

    // ------------------------------------------------------------------
    // The C++ rows above are one or two bare `configure`-shape nodes on the
    // single-executor path. That is a small fraction of `emit_cpp`: it reaches
    // neither `run_tiers`, nor the sched-context wiring, nor three of the four
    // node shapes, nor the service trailer, nor the probe tail. The rows below
    // are the rest of the emitter, added BEFORE it moves onto a template —
    // a harness written afterwards can only confirm whatever the new code
    // happens to do.
    // ------------------------------------------------------------------

    // Per-node rendering: name + namespace override, params (one carrying a
    // quote AND a backslash, because quoting is the half that must stay in
    // Rust), a remap, and a QoS override — beside a node that keeps its own
    // identity, so both arms of every per-node branch appear in one golden.
    let mut cpp_rich = typed_node("talker_pkg", "talker");
    cpp_rich.name = Some("renamed_talker".into());
    cpp_rich.namespace = Some("/demo".into());
    cpp_rich.params = vec![
        ("rate_hz".into(), "10".into()),
        ("greeting".into(), "he said \"hi\" \\ bye".into()),
    ];
    cpp_rich.remaps = vec![("chatter".into(), "/demo/chatter".into())];
    cpp_rich.qos_overrides = vec![super::QoSOverrideSpec {
        topic: "/demo/chatter".into(),
        role: 1,
        policy: 2,
        value: 5,
    }];
    out.push((
        "cpp_native_rich",
        plan(
            "native",
            vec![cpp_rich, typed_node("listener_pkg", "listener")],
        ),
        Lang::Cpp,
    ));

    // All four node shapes in one TU, with each dedup loop given a repeat:
    // the same C package twice, the same Rust package twice, and two nodes
    // sharing one C++ header. The forward-declaration and include loops dedup
    // by PACKAGE and by HEADER while storage is per NODE, so a repeat must
    // emit one `extern` / one `#include` and still get its own slot.
    let mut dup_header = typed_node("talker_pkg", "talker2");
    dup_header.class_header = Some("talker_pkg/talker.hpp".into());
    out.push((
        "cpp_native_shapes",
        plan(
            "native",
            vec![
                typed_node("talker_pkg", "talker"),
                dup_header,
                cpp_c_node("c_pkg", "c_one"),
                cpp_c_node("c_pkg", "c_two"),
                cpp_rust_node("rust_pkg", "rust_one"),
                cpp_rust_node("rust_pkg", "rust_two"),
                cpp_rclcpp_node("rclcpp_pkg", "rcl_one"),
            ],
        ),
        Lang::Cpp,
    ));

    // param-services + lifecycle close a setup function with a block no other
    // C++ row emits, on BOTH paths — and on the tiered path only tier 0 gets
    // it, because they are process facts rather than per-tier ones.
    let mut cpp_svc = plan("native", vec![typed_node("talker_pkg", "talker")]);
    cpp_svc.param_services = true;
    cpp_svc.lifecycle = Some("active".into());
    out.push(("cpp_native_services", cpp_svc, Lang::Cpp));

    let mut cpp_tier_svc = cpp_tiered_plan("native");
    cpp_tier_svc.param_services = true;
    cpp_tier_svc.lifecycle = Some("configure".into());
    out.push(("cpp_native_tiers_services", cpp_tier_svc, Lang::Cpp));

    // `run_tiers` is reached on native and on the three embedded boards that
    // declare it. ThreadX is deliberately absent here and present below: it
    // has no `run_tiers`, so a tiered ThreadX plan takes the single-executor
    // sched-context path instead, and that divergence is the thing worth
    // pinning.
    out.push(("cpp_native_tiers", cpp_tiered_plan("native"), Lang::Cpp));
    out.push(("cpp_zephyr_tiers", cpp_tiered_plan("zephyr"), Lang::Cpp));
    out.push(("cpp_nuttx_tiers", cpp_tiered_plan("nuttx"), Lang::Cpp));
    out.push(("cpp_freertos_tiers", cpp_tiered_plan("freertos"), Lang::Cpp));

    // ThreadX: tiers declared, `run_tiers` unavailable — the sched-context
    // wiring (`create_sched_context_from_policy`, `bind_node_name_sched`,
    // `bind_group_sched`) that no other row emits.
    out.push(("cpp_threadx_tiers", cpp_tiered_plan("threadx"), Lang::Cpp));

    // RFC-0047 sub-node split: one node with callback groups on two tiers.
    // `run_tiers` cannot express it (its per-tier setups construct whole
    // nodes), so the plan falls back to the sched-context path ON NATIVE —
    // the one row where a native tiered plan does NOT take `run_tiers`.
    let mut split = cpp_tiered_plan("native");
    split.nodes[0].group_tiers = BTreeMap::from([
        ("ctrl_grp".to_string(), "high".to_string()),
        ("telem_grp".to_string(), "low".to_string()),
    ]);
    if let Some(t) = split.resolved_tiers.as_mut() {
        t.tiers[1].members.push(("ctrl".into(), "telem_grp".into()));
    }
    out.push(("cpp_native_group_split", split, Lang::Cpp));

    // The metadata probe: the same setup body with the recording tail, which
    // returns BEFORE the boot config and the board wrapper.
    out.push((
        "cpp_native_probe",
        plan("native", vec![typed_node("talker_pkg", "talker")]),
        Lang::CppProbe,
    ));

    // #0266 — `time_slice_us` has no C++ tier-ABI field, so the emitter
    // REFUSES rather than dropping a declared value. The harness records a
    // refusal as its golden, so this row pins the refusal itself.
    let mut ts = cpp_tiered_plan("native");
    if let Some(t) = ts.resolved_tiers.as_mut() {
        t.tiers[0].time_slice_us = Some(2_000);
    }
    out.push(("cpp_native_time_slice_refused", ts, Lang::Cpp));

    out
}

fn render(p: &Plan, lang: Lang) -> Result<String, String> {
    match lang {
        Lang::Cpp => super::emit_cpp::emit_typed(p),
        // A fixed, absolute-looking `out_path`: the probe bakes it into the TU
        // as a literal, so a real temp path would make the golden depend on
        // the machine that ran the test.
        Lang::CppProbe => super::emit_cpp::emit_typed_probe(
            p,
            &super::emit_cpp::ProbeExport {
                package: "demo_bringup".into(),
                component: "talker".into(),
                executable: "talker".into(),
                language: "cpp".into(),
                out_path: "/build/nros/metadata/talker.json".into(),
            },
        ),
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
