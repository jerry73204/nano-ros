//! C Entry-pkg TU emitter (typed, RFC-0043 / phase-257).
//!
//! Maps a [`Plan`] (see `super::mod`) onto a generated typed `main.c`: per launch
//! node it creates a `nros_cpp_node_t` (`nros_cpp_node_create`) and calls the
//! node's `NROS_C_COMPONENT` factory/configure seam
//! (`__nros_c_component_<pkg>_{create,configure}`) on the real executor; `main`
//! drives `nros_board_native_run_components`. The C counterpart of
//! [`super::emit_cpp::emit_typed`]. The legacy non-typed emitter (the
//! `EntryNodeRuntime` interpreter via `nros_board_native_run`) was retired in
//! phase-257 Stage-3.

use std::fmt::Write;

use super::{Plan, emit_boot_config_static, sanitize_pkg};

/// Phase 257 (W0-A) — a `lang == "c"` node is a `NROS_C_COMPONENT` typed
/// component the C Entry installs via its `__nros_c_component_<pkg>_*` seam.
fn is_c_node(n: &super::PlanNode) -> bool {
    n.lang.as_deref() == Some("c")
}

/// Phase 305 W3 (issue 0255) — bake one `nros_cpp_declare_remap` call per
/// launch `<remap>` pair, scoped to the node identity the entry creates the
/// node with (name + `"/"` namespace, matching `nros_cpp_node_create` above).
/// Must run BEFORE the component's configure registers entities. Shared by
/// the C and C++ typed emitters (the non-drift rule).
pub(super) fn emit_declare_remaps(
    out: &mut String,
    n: &super::PlanNode,
    indent: &str,
    exec_expr: &str,
) {
    let node_name = n.name.as_deref().unwrap_or(&n.exec);
    let name_lit = node_name.replace('\\', "\\\\").replace('"', "\\\"");
    for (from, to) in &n.remaps {
        let f = from.replace('\\', "\\\\").replace('"', "\\\"");
        let t = to.replace('\\', "\\\\").replace('"', "\\\"");
        let _ = writeln!(out, "{indent}{{");
        let _ = writeln!(
            out,
            "{indent}    nros_cpp_ret_t rrc = nros_cpp_declare_remap({exec_expr}, \"{name_lit}\", \"/\", \"{f}\", \"{t}\");"
        );
        let _ = writeln!(
            out,
            "{indent}    if (rrc != NROS_CPP_RET_OK) return (int32_t)rrc;"
        );
        let _ = writeln!(out, "{indent}}}");
    }
}

/// Issue 0745 — bake one `nros_cpp_declare_param` call per launch param,
/// BEFORE the component's construction/configure: an rclcpp-shape ctor reads
/// `declare_parameter` initials immediately (the 0255 remap rule, one row
/// over). Seeding is initial VALUES and is independent of whether the
/// param-SERVICES surface (`nros_cpp_register_parameter_services`) is
/// enabled — the old emission sat inside the `plan.param_services` gate and
/// AFTER construction, so a plan without param services silently dropped
/// every launch param (measured on the ASI consumer: `ctrl_period` and
/// `control_output` both fell back to compiled defaults). Shared by the C
/// and C++ typed emitters (the non-drift rule).
pub(super) fn emit_declare_params(
    out: &mut String,
    n: &super::PlanNode,
    indent: &str,
    exec_expr: &str,
) {
    for (k, v) in &n.params {
        let k_esc = k.replace('\\', "\\\\").replace('"', "\\\"");
        let v_esc = v.replace('\\', "\\\\").replace('"', "\\\"");
        let _ = writeln!(out, "{indent}{{");
        let _ = writeln!(
            out,
            "{indent}    nros_cpp_ret_t prc = nros_cpp_declare_param({exec_expr}, \"{k_esc}\", \"{v_esc}\");"
        );
        let _ = writeln!(
            out,
            "{indent}    if (prc != NROS_CPP_RET_OK) return (int32_t)prc;"
        );
        let _ = writeln!(out, "{indent}}}");
    }
}

/// Issue #52 — bake the node's QoS-override table + the
/// `nros_cpp_node_set_qos_overrides` call. Must run BEFORE the component's
/// configure registers entities, so `create_publisher`/`create_subscription`
/// fold the matching overrides in.
///
/// C entries create nodes through the same `nros_cpp_*` FFI as C++ ones, so
/// this is the C++ emitter's table with a function call instead of a method
/// call. The `(role, policy, value)` lowering is shared
/// (`nros_orchestration_ir::qos_override`, shared with the proc-macro) — two
/// spellings of the same codes is exactly the drift this codebase keeps paying
/// for.
pub(super) fn emit_qos_overrides(out: &mut String, n: &super::PlanNode, i: usize, indent: &str) {
    // Issue 0303 — the plan carries CODES; nothing to decode or skip here.
    if n.qos_overrides.is_empty() {
        return;
    }
    let _ = writeln!(
        out,
        "{indent}static const nros_cpp_qos_override_t __nros_qos_{i}[] = {{"
    );
    for o in &n.qos_overrides {
        let (role, policy, value) = (o.role, o.policy, o.value);
        let topic = o.topic.replace('\\', "\\\\").replace('"', "\\\"");
        let _ = writeln!(
            out,
            "{indent}    {{ \"{topic}\", {role}, {policy}, {value} }},"
        );
    }
    let _ = writeln!(out, "{indent}}};");
    let _ = writeln!(out, "{indent}{{");
    let _ = writeln!(
        out,
        "{indent}    nros_cpp_ret_t qrc = nros_cpp_node_set_qos_overrides(&__nros_node_{i}, __nros_qos_{i}, {});",
        n.qos_overrides.len()
    );
    let _ = writeln!(
        out,
        "{indent}    if (qrc != NROS_CPP_RET_OK) return (int32_t)qrc;"
    );
    let _ = writeln!(out, "{indent}}}");
}

/// Phase 257 (W0-A, RFC-0043) — emit the **typed** C Entry TU: route each launch
/// node to the real executor via its `NROS_C_COMPONENT` factory/configure seam,
/// driven by the C-ABI `nros_board_native_run_components`. The C counterpart of
/// [`super::emit_cpp::emit_typed`]; pure-C workspaces only (every node is a C
/// component — a non-C node has no C seam and is rejected).
/// The whole TU, as the template sees it.
///
/// Issue 1102 — every field is ALREADY CORRECT. `name_lit` is escaped,
/// `decls` is the pre-rendered remap/param/QoS block for one node,
/// `spec_rows` are formatted `nros_native_tier_spec_t` initialisers, and
/// `boot_config` is the shared `emit_boot_config_static` output. The template
/// lays them out; it computes nothing.
#[derive(serde::Serialize)]
struct CEntryView {
    bringup: String,
    launch: String,
    board: String,
    /// Deduped package symbols — the forward declarations are per PACKAGE
    /// while node storage is per NODE, so these two lists differ in length
    /// whenever a launch runs the same package twice.
    externs: Vec<String>,
    node_indices: Vec<usize>,
    tiers: Option<CTiersView>,
    /// Single-executor path only; empty when `tiers` is set.
    setup_nodes: Vec<CNodeView>,
    /// The param-services / lifecycle block that closes the single setup fn,
    /// rendered from `c_service_trailer.c.jinja`.
    trailer: String,
    boot_config: String,
}

#[derive(serde::Serialize)]
struct CTiersView {
    n: usize,
    setups: Vec<CTierSetupView>,
    groups_arrays: Vec<String>,
    spec_rows: Vec<String>,
}

#[derive(serde::Serialize)]
struct CTierSetupView {
    index: usize,
    nodes: Vec<CNodeView>,
    /// Only tier 0 registers param services and lifecycle — they are process
    /// facts, not per-tier ones.
    trailer: String,
}

#[derive(serde::Serialize)]
struct CNodeView {
    index: usize,
    pkg: String,
    name_lit: String,
    decls: String,
}

/// Escape a value into a C string literal body.
///
/// Stays in Rust deliberately: a template that got this wrong would emit
/// source that fails to compile, or worse, compiles with the wrong string.
fn c_str_lit(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// The remap / param / QoS declarations for one node, at the given indent.
///
/// Returned as pre-rendered text because its ELEMENTS are C statements built
/// from escaped literals — assembling those is the half that must not move
/// into a template.
fn node_decls(n: &super::PlanNode, i: usize, indent: &str) -> String {
    let mut d = String::new();
    emit_declare_remaps(&mut d, n, indent, "executor");
    emit_declare_params(&mut d, n, indent, "executor");
    emit_qos_overrides(&mut d, n, i, indent);
    // The template writes the surrounding lines, so an empty block must
    // contribute nothing at all rather than a blank line.
    if d.is_empty() {
        d
    } else {
        format!("\n{}", d.trim_end_matches('\n'))
    }
}

fn node_view(n: &super::PlanNode, i: usize, indent: &str) -> CNodeView {
    CNodeView {
        index: i,
        pkg: sanitize_pkg(&n.pkg),
        name_lit: c_str_lit(n.name.as_deref().unwrap_or(&n.exec)),
        decls: node_decls(n, i, indent),
    }
}

/// The block that closes a setup function, from its own template.
///
/// One definition rendered twice (the single setup fn, and tier 0), rather
/// than the same C text written twice.
fn setup_trailer(plan: &Plan) -> Result<String, String> {
    #[derive(serde::Serialize)]
    struct TrailerView {
        param_services: bool,
        lifecycle_code: Option<u8>,
    }
    // The template file ends with a newline and the environment keeps trailing
    // newlines (the entry TUs need theirs), so an EMPTY trailer would render as
    // a stray blank line inside a C function. The caller writes the lines
    // around this block, so it must contribute exactly nothing when neither
    // service is declared.
    let rendered = crate::codegen::entry::render::render(
        "c_service_trailer.c.jinja",
        &TrailerView {
            param_services: plan.param_services,
            // "none" | "configure" | anything else (i.e. "active").
            lifecycle_code: plan.lifecycle.as_deref().map(|a| match a {
                "none" => 0u8,
                "configure" => 1,
                _ => 2,
            }),
        },
    )?;
    Ok(rendered.trim_end_matches('\n').to_string())
}

pub fn emit_typed(plan: &Plan) -> Result<String, String> {
    for n in &plan.nodes {
        if !is_c_node(n) {
            return Err(format!(
                "typed C entry emit: node pkg `{}` exec `{}` is lang `{}`, not `c` — \
                 a `--lang c` typed Entry supports C components only (use `--lang cpp` for \
                 a mixed C/C++/Rust workspace)",
                n.pkg,
                n.exec,
                n.lang.as_deref().unwrap_or("<unset>")
            ));
        }
    }

    // Forward declarations are per PACKAGE, storage is per NODE.
    let mut externs: Vec<String> = Vec::new();
    for n in &plan.nodes {
        let pkg = sanitize_pkg(&n.pkg);
        if !externs.contains(&pkg) {
            externs.push(pkg);
        }
    }

    let use_tiers = plan
        .resolved_tiers
        .as_ref()
        .is_some_and(|t| !t.tiers.is_empty());
    // A group-split node needs per-group sched contexts (the tier path can only
    // construct whole nodes); keep the sched-context path for such plans. See
    // emit_cpp.rs for the full rationale.
    let has_group_split = plan
        .resolved_tiers
        .as_ref()
        .is_some_and(|t| t.has_group_split_node());
    let use_run_tiers = use_tiers && !has_group_split;

    let mut boot_config = String::new();
    emit_boot_config_static(&mut boot_config, plan)?;

    let tiers_view = if use_run_tiers {
        let tiers = plan.resolved_tiers.as_ref().unwrap();
        // #0266 — the ThreadX round-robin time slice has a per-thread consumer
        // only on the Rust `nros::main!` ThreadX arm today. The C tier ABI
        // (`nros_native_tier_spec_t`) carries no time-slice field, so rather
        // than silently drop a declared value on a C bake, fail loud.
        if let Some(t) = tiers.tiers.iter().find(|t| t.time_slice_us.is_some()) {
            return Err(format!(
                "tier '{}': time_slice_us is not yet supported on the C/C++ codegen \
                 path (#0266) — declare it only on a Rust `nros::main!` ThreadX entry, \
                 or file the C consumer",
                t.name
            ));
        }

        // node_name → tier_index for per-tier filtering.
        let node_to_tier: std::collections::HashMap<String, usize> = tiers
            .tiers
            .iter()
            .enumerate()
            .flat_map(|(ti, t)| t.members.iter().map(move |(node, _)| (node.clone(), ti)))
            .collect();

        let tier0_trailer = setup_trailer(plan)?;
        let setups: Vec<CTierSetupView> = tiers
            .tiers
            .iter()
            .enumerate()
            .map(|(ti, _)| CTierSetupView {
                index: ti,
                nodes: plan
                    .nodes
                    .iter()
                    .enumerate()
                    .filter(|(_, n)| {
                        node_to_tier
                            .get(n.name.as_deref().unwrap_or(&n.exec))
                            .copied()
                            == Some(ti)
                    })
                    .map(|(i, n)| node_view(n, i, "        "))
                    .collect(),
                trailer: if ti == 0 {
                    tier0_trailer.clone()
                } else {
                    String::new()
                },
            })
            .collect();

        // Per-tier callback groups, deduped ACROSS tiers: a group named by two
        // tiers belongs to the first, and the arrays are what the runtime uses
        // to route a callback.
        let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
        let mut groups_per_tier: Vec<Vec<String>> = Vec::new();
        for t in &tiers.tiers {
            let mut g = Vec::new();
            for (_, grp) in &t.members {
                if !grp.is_empty() && seen.insert(grp.clone()) {
                    g.push(grp.clone());
                }
            }
            groups_per_tier.push(g);
        }

        let groups_arrays: Vec<String> = groups_per_tier
            .iter()
            .enumerate()
            .filter(|(_, g)| !g.is_empty())
            .map(|(ti, g)| {
                let items: String = g
                    .iter()
                    .map(|x| format!("\"{}\", ", c_str_lit(x)))
                    .collect();
                format!("static const char* __nros_tier_{ti}_groups[] = {{{items}}};")
            })
            .collect();

        let c_lit = |s: Option<&str>| match s {
            Some(v) => format!("\"{}\"", c_str_lit(v)),
            None => "NULL".to_string(),
        };
        let spec_rows: Vec<String> = tiers
            .tiers
            .iter()
            .enumerate()
            .map(|(ti, tier)| {
                let groups = &groups_per_tier[ti];
                let (groups_expr, n_groups_val) = if groups.is_empty() {
                    ("NULL".to_string(), 0usize)
                } else {
                    (format!("__nros_tier_{ti}_groups"), groups.len())
                };
                // RFC-0052 W2 — stack_bytes/core/preempt_threshold propagate;
                // phase-296 W5.7 — the generic real-time policy rides the spec
                // (see emit_cpp for the field semantics). NULL/0 = unset.
                format!(
                    "    {{ \"{name}\", {groups_expr}, {n_groups_val}u, \
{priority}LL, {stack}u, {spin}ull, \
&__nros_entry_setup_tier_{ti}, {core}u, {preempt}LL, \
{class}, {period}ull, {budget}ull, {deadline}ull, \
{dpolicy} }},",
                    name = c_str_lit(&tier.name),
                    priority = tier.priority,
                    stack = tier.stack_bytes.unwrap_or(0),
                    spin = tier.spin_period_us.unwrap_or(0),
                    core = tier.core.map(|c| c + 1).unwrap_or(0),
                    preempt = tier.preempt_threshold.unwrap_or(-1),
                    class = c_lit(tier.class.as_deref()),
                    period = tier.period_us.unwrap_or(0),
                    budget = tier.budget_us.unwrap_or(0),
                    deadline = tier.deadline_us.unwrap_or(0),
                    dpolicy = c_lit(tier.deadline_policy.as_deref()),
                )
            })
            .collect();

        Some(CTiersView {
            n: tiers.tiers.len(),
            setups,
            groups_arrays,
            spec_rows,
        })
    } else {
        None
    };

    let view = CEntryView {
        bringup: plan.bringup.clone(),
        launch: plan.launch_file.display().to_string(),
        board: plan.board.clone(),
        externs,
        node_indices: (0..plan.nodes.len()).collect(),
        setup_nodes: if tiers_view.is_some() {
            Vec::new()
        } else {
            plan.nodes
                .iter()
                .enumerate()
                .map(|(i, n)| node_view(n, i, "        "))
                .collect()
        },
        trailer: if tiers_view.is_some() {
            String::new()
        } else {
            setup_trailer(plan)?
        },
        tiers: tiers_view,
        boot_config,
    };

    crate::codegen::entry::render::render("c_entry.c.jinja", &view)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::entry::PlanNode;
    use std::path::PathBuf;

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
                    lang: Some("c".into()),
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

    /// Issue #52 — a C node carrying qos_overrides emits the static
    /// `nros_cpp_qos_override_t[]` table + the `nros_cpp_node_set_qos_overrides`
    /// call, BEFORE the component's configure registers entities. Until
    /// phase-296's residue sweep the C emitter dropped overrides entirely, so a
    /// model configured QoS on a C++ image and silently did not on a C one.
    #[test]
    fn typed_emit_bakes_qos_overrides_before_configure() {
        let mut plan = fixture_plan(&[("talker_pkg", "talker")]);
        // Built through the shared lowering, so the test cannot assert codes
        // the real bake would never produce.
        plan.nodes[0].qos_overrides = nros_orchestration_ir::qos_override::lower_all([
            (
                "qos_overrides./chatter.publisher.reliability",
                "best_effort",
            ),
            ("qos_overrides./chatter.subscription.depth", "7"),
        ])
        .expect("fixture overrides lower");
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(
            src.contains("static const nros_cpp_qos_override_t __nros_qos_0[] = {"),
            "{src}"
        );
        // (topic, role=0 publisher, policy=0 reliability, value=0 best_effort)
        assert!(src.contains("{ \"/chatter\", 0, 0, 0 },"), "{src}");
        // (topic, role=1 subscription, policy=3 depth, value=7)
        assert!(src.contains("{ \"/chatter\", 1, 3, 7 },"), "{src}");
        assert!(
            src.contains("nros_cpp_node_set_qos_overrides(&__nros_node_0, __nros_qos_0, 2)"),
            "{src}"
        );
        let set_at = src.find("nros_cpp_node_set_qos_overrides").unwrap();
        // The CALL site, not the forward declaration at the top of the TU.
        let cfg_at = src.find("crc = __nros_c_component").unwrap();
        assert!(
            set_at < cfg_at,
            "overrides must be installed before configure"
        );
    }

    /// A C node with no overrides emits no table and no call.
    #[test]
    fn typed_emit_no_qos_overrides_no_table() {
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(!src.contains("nros_cpp_qos_override_t"));
        assert!(!src.contains("set_qos_overrides"));
    }

    #[test]
    fn typed_emit_creates_node_and_configures_component() {
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        // factory + configure seam forward-decls.
        assert!(src.contains("extern void* __nros_c_component_talker_pkg_create(void);"));
        assert!(src.contains(
            "extern int32_t __nros_c_component_talker_pkg_configure(const nros_cpp_node_t* node, void* executor, void* self);"
        ));
        // setup creates the node + configures on the executor.
        assert!(src.contains("nros_cpp_node_create(executor, \"talker\", \"/\", &__nros_node_0)"));
        assert!(
            src.contains("__nros_c_component_talker_pkg_configure(&__nros_node_0, executor, self)")
        );
        // main drives the real-executor lifecycle via the named runner.
        assert!(src.contains("nros_board_native_run_components_named(nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"));
        // single-node: boot config has the node name baked in.
        assert!(src.contains("NROS_BOOT_SET_NODE_NAME"));
        assert!(src.contains(".node_name  = \"talker\""));
    }

    #[test]
    fn typed_emit_preserves_launch_order() {
        let plan = fixture_plan(&[("talker_pkg", "talker"), ("listener_pkg", "listener")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        let pos_talker = src.find("__nros_c_component_talker_pkg_create").unwrap();
        let pos_listener = src.find("__nros_c_component_listener_pkg_create").unwrap();
        assert!(pos_talker < pos_listener);
    }

    #[test]
    fn typed_emit_dashed_pkg_names_sanitised() {
        let plan = fixture_plan(&[("dashy-pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(src.contains("__nros_c_component_dashy_pkg_create"));
        assert!(!src.contains("dashy-pkg_create"));
    }

    #[test]
    fn typed_emit_rejects_non_c_node() {
        let mut plan = fixture_plan(&[("talker_pkg", "talker")]);
        plan.nodes[0].lang = Some("cpp".into());
        let err = emit_typed(&plan).unwrap_err();
        assert!(err.contains("not `c`"), "{err}");
        assert!(err.contains("talker_pkg"), "{err}");
    }

    #[test]
    fn typed_emit_has_required_includes() {
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(src.contains("#include <nros/boot_config.h>"));
        assert!(src.contains("#include <nros/main.h>"));
        assert!(src.contains("#include <nros/nros_cpp_ffi.h>"));
        assert!(!src.contains("#include <nros/component.h>"));
        assert!(src.contains("int main(int argc, char** argv)"));
    }

    #[test]
    fn typed_emit_multi_node_boot_config_unset() {
        // Multi-node entry → all-unset boot config (node name not baked).
        let plan = fixture_plan(&[("talker_pkg", "talker"), ("listener_pkg", "listener")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(src.contains("NROS_BOOT_CONFIG"));
        // Multi-node: set_flags must be 0 (not NROS_BOOT_SET_NODE_NAME).
        assert!(src.contains(".set_flags  = 0,"));
        assert!(!src.contains("NROS_BOOT_SET_NODE_NAME"));
    }

    // Phase 305 W3 (issue 0255) — launch `<remap>` rules bake as per-pair
    // `nros_cpp_declare_remap` calls BEFORE the component configure (entities
    // register during configure, so the rules must already be in the table).
    #[test]
    fn typed_emit_remaps_declared_before_configure() {
        let mut plan = fixture_plan(&[("talker_pkg", "talker")]);
        plan.nodes[0].remaps = vec![("~/out".into(), "/wire".into())];
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(
            src.contains(
                "nros_cpp_declare_remap(executor, \"talker\", \"/\", \"~/out\", \"/wire\")"
            ),
            "expected declare_remap call; src:\n{src}"
        );
        // Error-propagation guard.
        assert!(src.contains("if (rrc != NROS_CPP_RET_OK) return (int32_t)rrc"));
        // Compare against the configure CALL (the extern forward-decl sits at
        // the top of the TU before everything).
        let remap_at = src.find("nros_cpp_declare_remap").unwrap();
        let cfg_at = src
            .find("__nros_c_component_talker_pkg_configure(&__nros_node_0")
            .unwrap();
        assert!(remap_at < cfg_at, "remap decl must precede configure");
    }

    #[test]
    fn typed_emit_no_remaps_no_declare_calls() {
        // Guard: remap-free plans produce byte-identical output (no declare block).
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(!src.contains("nros_cpp_declare_remap"));
    }

    #[test]
    fn typed_emit_param_services_block_present_when_enabled() {
        // Phase 269 W1 — when param_services is true, the post-configure block emits
        // nros_cpp_register_parameter_services + nros_cpp_declare_param per node param.
        let mut plan = fixture_plan(&[("param_talker_pkg", "param_talker")]);
        plan.param_services = true;
        plan.nodes[0].params = vec![("publish_period_ms".into(), "250".into())];
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(src.contains("nros_cpp_register_parameter_services(executor)"));
        assert!(src.contains("nros_cpp_declare_param(executor, \"publish_period_ms\", \"250\")"));
        // must appear after the configure loop, before return 0
        let reg_at = src.find("nros_cpp_register_parameter_services").unwrap();
        let ret_at = src.rfind("return 0;").unwrap();
        assert!(reg_at < ret_at, "param block must precede return 0");
    }

    #[test]
    fn typed_emit_param_services_registration_is_non_fatal_both_paths() {
        // Issue 0745 — registration builds six service servers and fails outright on
        // an RMW without service-server support (cyclonedds today). Launch params are
        // seeded pre-construction by `emit_declare_params`, so a failed registration
        // must cost the runtime get/set RPC and nothing else. The C++ emitter moved to
        // the non-fatal shape with 0745; BOTH C paths kept the pre-0745 block, which
        // returned the rc and aborted boot. Covers the flat AND the tiered emitter,
        // because the tiered site is a separate copy of the same block.
        for (label, mut plan) in [
            (
                "flat",
                fixture_plan(&[("param_talker_pkg", "param_talker")]),
            ),
            ("tiered", fixture_plan_with_tiers()),
        ] {
            plan.param_services = true;
            plan.nodes[0].params = vec![("publish_period_ms".into(), "250".into())];
            let src = emit_typed(&plan).expect("typed C emit ok");

            assert!(
                src.contains("(void)nros_cpp_register_parameter_services(executor)"),
                "{label}: registration must be non-fatal; got:\n{src}"
            );
            assert!(
                !src.contains("ps_ret"),
                "{label}: no rc check on registration; got:\n{src}"
            );

            // Seeding precedes registration, and happens exactly once per param — the
            // old block re-declared every param AFTER construction (0745 defect 2).
            let seed = "nros_cpp_declare_param(executor, \"publish_period_ms\", \"250\")";
            let seed_at = src
                .find(seed)
                .unwrap_or_else(|| panic!("{label}: no seed; got:\n{src}"));
            let reg_at = src.find("nros_cpp_register_parameter_services").unwrap();
            assert!(seed_at < reg_at, "{label}: params seed BEFORE registration");
            assert_eq!(
                src.matches(seed).count(),
                1,
                "{label}: one seed per param; got:\n{src}"
            );
        }
    }

    #[test]
    fn typed_emit_param_services_absent_when_disabled() {
        // Guard: non-param plans produce byte-identical output (no param block).
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(!src.contains("nros_cpp_register_parameter_services"));
        assert!(!src.contains("nros_cpp_declare_param"));
    }

    #[test]
    fn typed_emit_lifecycle_active_emits_autostart_block() {
        // Phase 269 W2 — lifecycle = Some("active") → nros_cpp_lifecycle_autostart(executor, 2u)
        // in the post-configure block, AFTER any param block, BEFORE return 0.
        let mut plan = fixture_plan(&[("lifecycle_talker_pkg", "lifecycle_talker")]);
        plan.lifecycle = Some("active".into());
        let src = emit_typed(&plan).expect("typed C lifecycle emit ok");
        // autostart call with code 2 (active = configure + activate)
        assert!(
            src.contains("nros_cpp_lifecycle_autostart(executor, 2u)"),
            "expected nros_cpp_lifecycle_autostart(executor, 2u) in:\n{src}"
        );
        // error-propagation guard
        assert!(src.contains("if (lc_ret != NROS_CPP_RET_OK) return (int32_t)lc_ret"));
        // AFTER configure loop (all node configure calls), BEFORE return 0
        let autostart_at = src.find("nros_cpp_lifecycle_autostart").unwrap();
        let ret_at = src.rfind("return 0;").unwrap();
        assert!(
            autostart_at < ret_at,
            "lifecycle block must precede return 0"
        );
        // configure loop precedes the lifecycle block
        let cfg_at = src
            .find("__nros_c_component_lifecycle_talker_pkg_configure")
            .unwrap();
        assert!(
            cfg_at < autostart_at,
            "lifecycle block must follow configure loop"
        );
    }

    #[test]
    fn typed_emit_lifecycle_configure_emits_code_1() {
        let mut plan = fixture_plan(&[("lc_pkg", "lc")]);
        plan.lifecycle = Some("configure".into());
        let src = emit_typed(&plan).expect("typed C lifecycle configure emit ok");
        assert!(
            src.contains("nros_cpp_lifecycle_autostart(executor, 1u)"),
            "expected autostart_code 1 for 'configure'; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_lifecycle_none_emits_code_0() {
        let mut plan = fixture_plan(&[("lc_pkg", "lc")]);
        plan.lifecycle = Some("none".into());
        let src = emit_typed(&plan).expect("typed C lifecycle none emit ok");
        assert!(
            src.contains("nros_cpp_lifecycle_autostart(executor, 0u)"),
            "expected autostart_code 0 for 'none'; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_lifecycle_absent_when_disabled() {
        // Guard: lifecycle = None → byte-identical output (no lifecycle block).
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C emit ok");
        assert!(
            !src.contains("nros_cpp_lifecycle_autostart"),
            "lifecycle block must be absent when lifecycle = None"
        );
        assert!(
            !src.contains("nros_cpp_register_lifecycle_services"),
            "lifecycle block must be absent when lifecycle = None"
        );
    }

    #[test]
    fn typed_emit_lifecycle_after_param_block() {
        // Phase 269 W2 — when both param_services and lifecycle are set, the lifecycle
        // block must appear AFTER the param block (same order as the Rust macro: params → lifecycle).
        let mut plan = fixture_plan(&[("talker_pkg", "talker")]);
        plan.param_services = true;
        plan.nodes[0].params = vec![("foo".into(), "bar".into())];
        plan.lifecycle = Some("active".into());
        let src = emit_typed(&plan).expect("typed C combined emit ok");
        let param_at = src.find("nros_cpp_register_parameter_services").unwrap();
        let lc_at = src.find("nros_cpp_lifecycle_autostart").unwrap();
        assert!(
            param_at < lc_at,
            "lifecycle block must follow param-services block"
        );
    }

    // -------------------------------------------------------------------------
    // Phase 269 (W4) — sched-context wiring tests
    // -------------------------------------------------------------------------

    fn fixture_plan_with_tiers() -> Plan {
        use crate::codegen::entry::PlanNode;
        use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
        use std::path::PathBuf;
        // Two-tier plan: high (idx 0) and low (idx 1), each with one C node.
        let high_tier = ResolvedTier {
            name: "high".into(),
            priority: 80,
            stack_bytes: None,
            spin_period_us: Some(10_000),
            preempt_threshold: None,
            time_slice_us: None,
            sched_class: None,
            class: None,
            period_us: None,
            budget_us: None,
            deadline_us: None,
            deadline_policy: None,
            core: None,
            members: vec![("ctrl".into(), "ctrl_grp".into())],
        };
        let low_tier = ResolvedTier {
            name: "low".into(),
            priority: 10,
            stack_bytes: None,
            spin_period_us: Some(100_000),
            preempt_threshold: None,
            time_slice_us: None,
            sched_class: None,
            class: None,
            period_us: None,
            budget_us: None,
            deadline_us: None,
            deadline_policy: None,
            core: None,
            members: vec![("telem".into(), "telem_grp".into())],
        };
        Plan {
            board: "native".into(),
            nodes: vec![
                PlanNode {
                    pkg: "ctrl_pkg".into(),
                    exec: "ctrl".into(),
                    name: Some("ctrl".into()),
                    namespace: None,
                    class_name: None,
                    class_header: None,
                    lang: Some("c".into()),
                    shape: None,
                    qos_overrides: Vec::new(),
                    params: Vec::new(),
                    remaps: Vec::new(),
                    callback_groups: vec!["ctrl_grp".into()],
                    sched_context: Some(0), // high tier
                    group_tiers: std::collections::BTreeMap::new(),
                },
                PlanNode {
                    pkg: "telem_pkg".into(),
                    exec: "telem".into(),
                    name: Some("telem".into()),
                    namespace: None,
                    class_name: None,
                    class_header: None,
                    lang: Some("c".into()),
                    shape: None,
                    qos_overrides: Vec::new(),
                    params: Vec::new(),
                    remaps: Vec::new(),
                    callback_groups: vec!["telem_grp".into()],
                    sched_context: Some(1), // low tier
                    group_tiers: std::collections::BTreeMap::new(),
                },
            ],
            depfile_paths: Vec::new(),
            bringup: "demo_bringup".into(),
            launch_file: PathBuf::from("/tmp/system.launch.xml"),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers: Default::default(),
            node_overrides: Vec::new(),
            resolved_tiers: Some(ResolvedTierTable {
                tiers: vec![high_tier, low_tier],
            }),
        }
    }

    #[test]
    fn typed_emit_tiers_uses_run_tiers_path() {
        // Phase 274.W2 — multi-tier C plan emits per-tier setup functions + run_tiers
        // call instead of the old sched-context wiring.
        let plan = fixture_plan_with_tiers();
        let src = emit_typed(&plan).expect("typed C tier emit ok");

        // Per-tier setup functions present.
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_0(void* executor)"),
            "expected tier 0 setup fn; got:\n{src}"
        );
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_1(void* executor)"),
            "expected tier 1 setup fn; got:\n{src}"
        );
        // Each setup fn creates only its tier's nodes (ctrl in tier 0, telem in tier 1).
        assert!(
            src.contains("nros_cpp_node_create(executor, \"ctrl\", \"/\", &__nros_node_0)"),
            "ctrl node in tier-0 setup; src:\n{src}"
        );
        assert!(
            src.contains("nros_cpp_node_create(executor, \"telem\", \"/\", &__nros_node_1)"),
            "telem node in tier-1 setup; src:\n{src}"
        );
        // Tier spec table emitted.
        assert!(
            src.contains("static const nros_native_tier_spec_t __nros_tiers[2]"),
            "expected 2-element tier spec table; src:\n{src}"
        );
        assert!(
            src.contains("\"high\""),
            "high tier name in spec table; src:\n{src}"
        );
        assert!(
            src.contains("\"low\""),
            "low tier name in spec table; src:\n{src}"
        );
        // Priority baked in as LL literals.
        assert!(src.contains("80LL"), "high priority 80LL; src:\n{src}");
        assert!(src.contains("10LL"), "low priority 10LL; src:\n{src}");
        // main calls run_tiers, not run_components.
        assert!(
            src.contains("nros_board_native_run_tiers("),
            "main must call nros_board_native_run_tiers; src:\n{src}"
        );
        assert!(
            !src.contains("return nros_board_native_run_components"),
            "must NOT call run_components in multi-tier; src:\n{src}"
        );
        // Old sched-context wiring must NOT appear.
        assert!(
            !src.contains("__nros_sc_ids"),
            "no-sched-ctx: sc_ids array must not appear; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_create_sched_context"),
            "no-sched-ctx: create_sched_context must not appear; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_bind_node_name_sched"),
            "no-sched-ctx: bind_node_name_sched must not appear; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_bind_group_sched"),
            "no-sched-ctx: bind_group_sched must not appear; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_no_tiers_uses_plain_node_create() {
        // Guard: empty resolved_tiers keeps byte-identical plain create (no seed, no sched).
        let plan = fixture_plan(&[("talker_pkg", "talker")]);
        let src = emit_typed(&plan).expect("typed C no-tier emit ok");
        // Must NOT contain sched block artifacts.
        assert!(
            !src.contains("__nros_sc_ids"),
            "no-tier plan must not emit sc_ids array"
        );
        assert!(
            !src.contains("nros_cpp_create_sched_context"),
            "no-tier plan must not emit sched_context_create"
        );
        assert!(
            !src.contains("nros_cpp_bind_node_name_sched"),
            "no-tier plan must not emit bind_node_name_sched"
        );
        assert!(
            !src.contains("nros_cpp_bind_group_sched"),
            "no-tier plan must not emit bind_group_sched"
        );
        assert!(
            !src.contains("nros_cpp_node_create_ex"),
            "no-tier plan must use plain nros_cpp_node_create"
        );
        assert!(
            src.contains("nros_cpp_node_create(executor, \"talker\", \"/\", &__nros_node_0)"),
            "no-tier plan must use plain nros_cpp_node_create"
        );
    }
}
