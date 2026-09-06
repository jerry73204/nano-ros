//! Phase 219.A — Entry-pkg codegen shared module.
//!
//! Lifts the pkg-index walk + launch.xml parse + register-call
//! resolution out of the Rust `nros::main!()` proc-macro (which lives
//! in `packages/core/nros-macros/src/main_macro.rs`) into one place
//! every front-end can call. The three emitters
//! ([`emit_rust`], [`emit_cpp`], [`emit_c`]) consume a single in-memory
//! [`Plan`] IR so the per-language differences stay surface-level.
//!
//! Surface (per phase doc §3.2):
//!
//! ```ignore
//! use nros_cli_core::codegen::entry::{Lang, Plan, plan_from_launch};
//!
//! let plan = plan_from_launch(PlanInput {
//!     workspace: ws.as_path(),
//!     launch_spec: "demo_bringup:system.launch.xml",
//!     board: Some("native".into()),
//!     arg_overrides: vec![],
//! })?;
//! let src = match lang {
//!     Lang::Rust => emit_rust::emit(&plan),               // register-based
//!     Lang::Cpp  => emit_cpp::emit_typed(&plan)?,         // typed (RFC-0043)
//!     Lang::C    => emit_c::emit_typed(&plan)?,           // typed (phase-257)
//! };
//! ```
//!
//! Errors carry enough context that the CLI verb's `eyre::Result`
//! wrapper passes them through verbatim.

use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    path::{Path, PathBuf},
};

use eyre::{Result, WrapErr, bail};
use nros_orchestration_ir::{
    CallbackGroupDecl, DEFAULT_TIER, ResolvedTierTable, TierResolveError, resolve_tiers,
};

use crate::orchestration::cargo_metadata_schema::{NodeOverride, TierDef};

pub mod emit_c;
pub mod emit_cpp;
pub mod emit_rust;
#[cfg(test)]
mod golden;
pub mod metadata;
mod render;

/// The entry emitters' target language.
///
/// phase-432 W2.1 — an ALIAS now, not a second declaration. This enum and
/// `orchestration::ComponentLanguage` were the same three variants with no
/// relationship between them, so a language added to one was invisible to the
/// other; issue #1062 is that already shipped. `Lang::parse` moved to
/// `nros_lang::Language::parse` with it, aliases (`c++`, `cxx`) intact.
pub type Lang = nros_lang::Language;

/// Resolved plan handed to one of the three emitters.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Board key from [`PlanInput::board`]; `"native"` by default.
    pub board: String,
    /// Per-node entries in launch-file order (top-scope first, then
    /// each `<group>`'s children). Duplicates per `(pkg, exec)` are
    /// preserved — multiple instances of the same Node pkg may occur
    /// in a single launch.
    pub nodes: Vec<PlanNode>,
    /// Absolute paths of every file the plan read. Caller emits these
    /// as `cargo:rerun-if-changed=` / `--depfile` entries for build-
    /// system rebuild correctness.
    pub depfile_paths: Vec<PathBuf>,
    /// Bringup-pkg name from the launch spec (`"demo_bringup"` for
    /// `"demo_bringup:system.launch.xml"`). Surfaced so emitters can
    /// thread it into the generated banner.
    pub bringup: String,
    /// Resolved launch-file path; surfaces in the generated header
    /// banner.
    pub launch_file: PathBuf,
    /// Boot lifecycle autostart from `system.toml [lifecycle].autostart`
    /// (`"none"` | `"configure"` | `"active"`). `None` ⇒ no `[lifecycle]` block. (#117)
    pub lifecycle: Option<String>,
    /// `[param_services]` (or `features=["param_services"]`) enabled — register the
    /// ROS 2 parameter services. (#116)
    pub param_services: bool,
    /// `[safety]` (or `features=["safety"]`) enabled. (#118)
    pub safety: Option<bool>,
    /// Raw `[tiers.*]` for the W4 tier resolver. Empty ⇒ single-tier. (#119)
    pub tiers: BTreeMap<String, TierDef>,
    /// Raw `[[node_overrides]]` for the W4 tier resolver. (#119)
    pub node_overrides: Vec<NodeOverride>,
    /// Phase 269 (W4) — resolved tier table (populated by [`resolve_plan_sched`]).
    /// `None` until the caller invokes the resolver. Emitters check this to gate
    /// sched-context wiring: `None` or `is_single_tier()` → byte-identical output.
    pub resolved_tiers: Option<ResolvedTierTable>,
}

// phase-326 (issue 0364): `Plan::for_host` / `Plan::hosts` / `PlanNode.host`
// are gone. They partitioned a multi-host launch at BAKE time from the
// `<node machine="…">` attribute — ROS 1 roslaunch syntax ROS 2 rejects.
// Multi-host is now a resolve-time launch argument (`host:=<id>` + `if=`
// conditions), so a per-host SystemModel already contains only that host's
// nodes and the bake needs no partition step.

/// Issue #52 / 0303 — one lowered QoS override on a plan node.
///
/// The lowering (parameter key/value strings → `(topic, role, policy, value)`
/// codes) lives in `nros_orchestration_ir::qos_override`, shared with the
/// `nros::main!` proc-macro, and REJECTS what it cannot lower. A plan therefore
/// never carries an override the emitters would silently drop.
pub type QoSOverrideSpec = nros_orchestration_ir::qos_override::LoweredOverride;

/// One Node-pkg invocation in launch order.
///
/// `pkg` is the cargo-style pkg name (sanitised via [`sanitize_pkg`]
/// for symbol-name use). `exec` and `name` come straight from the
/// launch XML; today only `pkg` drives codegen (the per-pkg mangled
/// register symbol is keyed on it), but the `exec` / `name` fields
/// stay on the IR for `<param>` / `<remap>` routing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlanNode {
    pub pkg: String,
    pub exec: String,
    pub name: Option<String>,
    pub namespace: Option<String>,
    /// Phase 240.2 (RFC-0043) — fully-qualified C++ component class
    /// (`"talker_pkg::Talker"`), from `nano_ros_node_register(CLASS …)` via the
    /// cmake metadata. Required by the **typed** entry emitter (`emit_cpp_typed`)
    /// which constructs the class; `None` for the legacy register-symbol path.
    pub class_name: Option<String>,
    /// Phase 240.2 — the component class header to `#include`
    /// (`"talker_pkg/Talker.hpp"`). Paired with `class_name`.
    pub class_header: Option<String>,
    /// Phase 240.4 (RFC-0043) — component implementation language from the
    /// cmake metadata (`"c"` / `"cpp"` / `"rust"`). `None` for the launch-only
    /// legacy path. The **typed** entry emitter branches on it: a `"c"` node is
    /// constructed via its C-ABI factory + `configure(node_handle, self)` seam
    /// (`NROS_C_COMPONENT`), a `"cpp"` node via its C++ class + `configure(node)`.
    pub lang: Option<String>,
    /// Phase 242.4 (RFC-0044) — component *shape* from the cmake metadata:
    /// `"rclcpp"` (IS-A-node, ctor-wired — construct-with-handle) or `"configure"`
    /// (RFC-0043 default-construct + `configure(Node&)`). `None` ⇒ `"configure"`
    /// (back-compat). The **typed** entry emitter branches construct on it: an
    /// `"rclcpp"` C++ node is placement-new'd with the executor handle *after*
    /// `nros::init` (the ctor owns the node); a `"configure"` node keeps the
    /// 240.x static-construct-then-`configure(node)` path.
    pub shape: Option<String>,
    /// Phase 211.H (issue #52) — per-topic QoS overrides decomposed from this
    /// node's `qos_overrides.<topic>.<role>.<policy>` launch params. Empty when
    /// none. The typed C++ entry emitter bakes them into a
    /// `node.set_qos_overrides(...)` call before `configure(node)`.
    pub qos_overrides: Vec<QoSOverrideSpec>,
    /// Launch `<param name= value=>` initials (NON-qos params; qos ones go to
    /// `qos_overrides`). Preserved in launch-file order. (#116)
    pub params: Vec<(String, String)>,
    /// Phase 305 W3 (issue 0255) — launch `<remap from= to=/>` rules `(from, to)`,
    /// in launch declaration order (node-level rules first, then group-level —
    /// first match wins at runtime). The typed emitters bake one
    /// `nros_cpp_declare_remap` call per pair so entity registration resolves
    /// `~`/relative names + substitutes matches (executor-side table).
    pub remaps: Vec<(String, String)>,
    /// Per-component callback-group names (from cmake metadata). Empty until the
    /// W4 cmake surface lands. (#119)
    pub callback_groups: Vec<String>,
    /// Resolved sched-context/tier index. `None` until W4 resolves tiers. (#119)
    pub sched_context: Option<u8>,
    /// Phase 273 (RFC-0047 W2) — group → tier bindings from `system.toml
    /// [[component]].group_tiers`. Populated by `plan_from_launch` when the
    /// matching `[[component]]` carries `group_tiers`. Used by `resolve_plan_sched`
    /// to assign each callback group's tier directly, without needing
    /// `[[node_overrides]]`.
    pub group_tiers: BTreeMap<String, String>,
}

impl PlanNode {
    // Phase 258 (Track 2, follow-up) — `register_symbol()` (the dead
    // `__nros_component_<pkg>_register` mangled-symbol string) is gone. The
    // post-257 entries link `__nros_component_<pkg>_install`, not `_register`;
    // nothing consumed this string.

    /// Cmake target name for the static lib the Node pkg's
    /// `nano_ros_node_register()` produces:
    /// `<pkg>_<exec>_component`. The Entry pkg's auto-link
    /// (`nano_ros_entry(... LAUNCH …)`) consumes this string.
    pub fn cmake_link_target(&self) -> String {
        format!("{}_{}_component", sanitize_pkg(&self.pkg), self.exec)
    }
}

/// One tier, in neutral terms — the row BOTH entry packs render.
///
/// Every field is a VALUE, and the strings are RAW: the pack quotes them
/// through its own escaping filter, and decides for itself that an empty
/// `groups` means `NULL` (C) or `nullptr` (C++) rather than an array symbol.
/// Shared rather than declared twice because a tier spelled two ways is the
/// defect this phase exists to remove — the same reason W1.2's gate compares
/// the two message packs one layer down.
///
/// The ENCODINGS here are ABI facts, not spellings: `core_plus1` is the core
/// index plus one with 0 meaning unpinned, and `preempt_threshold` is -1 when
/// unset. Those belong to the lowering; how they are laid out belongs to the
/// pack.
#[derive(serde::Serialize)]
pub(crate) struct TierView {
    pub index: usize,
    pub name: String,
    pub groups: Vec<String>,
    pub priority: i64,
    pub stack_bytes: u64,
    pub spin_period_us: u64,
    /// 0 = unpinned; otherwise the core index PLUS ONE.
    pub core_plus1: u32,
    /// -1 = unset.
    pub preempt_threshold: i64,
    /// `None` = unset; the pack decides what that is spelled.
    pub class: Option<String>,
    pub period_us: u64,
    pub budget_us: u64,
    pub deadline_us: u64,
    pub deadline_policy: Option<String>,
}

/// Build the shared tier rows.
///
/// `groups_per_tier` is a PARAMETER, and that is a defect being carried rather
/// than a design: the two emitters derive a tier's callback groups
/// differently. `emit_c` dedups ACROSS tiers (a group named by two tiers
/// belongs to the first) and drops empty names; `emit_cpp` dedups WITHIN each
/// tier and keeps empty ones, so the same plan yields a group listed under two
/// tiers in one language and one tier in the other, and a `""` entry in the
/// C++ array. One fact, two authored spellings — issue 1172. Reconciling it
/// moves goldens, so it is its own change; sharing the ROW is what makes the
/// divergence visible at all.
pub(crate) fn tier_views(
    tiers: &nros_orchestration_ir::ResolvedTierTable,
    groups_per_tier: Vec<Vec<String>>,
) -> Vec<TierView> {
    tiers
        .tiers
        .iter()
        .enumerate()
        .map(|(ti, tier)| TierView {
            index: ti,
            name: tier.name.clone(),
            groups: groups_per_tier[ti].clone(),
            priority: tier.priority,
            stack_bytes: tier.stack_bytes.unwrap_or(0) as u64,
            spin_period_us: tier.spin_period_us.unwrap_or(0),
            core_plus1: tier.core.map(|c| c + 1).unwrap_or(0),
            preempt_threshold: tier.preempt_threshold.unwrap_or(-1),
            class: tier.class.clone(),
            period_us: tier.period_us.unwrap_or(0),
            budget_us: tier.budget_us.unwrap_or(0),
            deadline_us: tier.deadline_us.unwrap_or(0),
            deadline_policy: tier.deadline_policy.clone(),
        })
        .collect()
}

/// Phase 266 (W5b/W6) — emit the `NROS_BOOT_CONFIG` static blob (C/C++ shared helper).
///
/// For a **single-node** plan the blob bakes the launch node name into
/// `.node_name` with `NROS_BOOT_SET_NODE_NAME` set; a post-link tool (or the
/// runner's inline call to `nros_boot_config_node_name`) can read it back.
/// For a **multi-node** plan (or when the single node has no resolvable name)
/// all fields are zero / unset — `nros_boot_config_node_name` returns NULL and
/// the runner falls back to the unified `"node"` default.
///
/// # Errors
///
/// Returns `Err` if the resolved node name exceeds 63 bytes (the
/// `nros_baked_boot_config.node_name` C field is `char node_name[64]`, which
/// must hold the string **and** a NUL terminator). The caller receives a clear
/// diagnostic rather than a confusing C-compiler array-initialiser error.
///
/// Callers must have already emitted `#include <nros/boot_config.h>`.
pub fn emit_boot_config_static(out: &mut String, plan: &Plan) -> Result<(), String> {
    use std::fmt::Write as _;
    // Escape a literal for embedding in a C string — backslash and quote.
    fn escape_c(raw: &str) -> String {
        raw.replace('\\', "\\\\").replace('"', "\\\"")
    }

    // issue 0794 — this used to set NROS_BOOT_SET_NODE_NAME and nothing else,
    // while the blob defines four fields and the reader
    // (`nros-node/src/executor/types.rs`) branches on all four. So a launch file
    // that declared a namespace produced an image that came up at `/`, silently:
    // the field, the bit, the packer and the reader all existed and worked, and
    // only the producer was missing. RFC-0046 makes launch authoritative for node
    // identity, and it was authoritative for the name alone.
    //
    // `rmw` (layout version 2, issue 1050 defect (3)) is emitted EMPTY with its
    // bit clear, for the same reason as `domain_id`/`locator`: it is a property
    // of the IMAGE, not of a node, and `Plan` carries no such field. A C/C++
    // image gets its selector from the `NROS_ENTRY_RMW` compile definition the
    // entry gate bakes — the second delivery path issue 0794 records above. It
    // is spelled here rather than left to the designated-initializer zero fill
    // so the field is visible in the generated code.
    //
    // `domain_id` and `locator` STAY hardcoded, and that is a scope statement
    // rather than an oversight: neither exists anywhere in `Plan` — `domain_id`
    // appears exactly once in this whole crate, as the literal below. Both are
    // properties of the IMAGE rather than of a node, so wiring them means
    // deciding where they come from first (see the issue).
    let mut flags: Vec<&'static str> = Vec::new();
    let mut node_name = String::new();
    let mut namespace = String::new();
    if plan.nodes.len() == 1 {
        let n = &plan.nodes[0];
        let raw = n.name.as_deref().unwrap_or(&n.exec);
        // Guard: the C field is `char node_name[64]` — 63 usable bytes + NUL.
        if raw.len() > 63 {
            return Err(format!(
                "node name '{}' is {} bytes; the .nros_boot_config node_name field holds at most 63 bytes + NUL",
                raw,
                raw.len(),
            ));
        }
        node_name = escape_c(raw);
        flags.push("NROS_BOOT_SET_NODE_NAME");

        if let Some(ns) = n.namespace.as_deref() {
            // `char namespace_[64]` — same 63-byte budget as the name.
            if ns.len() > 63 {
                return Err(format!(
                    "node namespace '{}' is {} bytes; the .nros_boot_config namespace_ field holds at most 63 bytes + NUL",
                    ns,
                    ns.len(),
                ));
            }
            namespace = escape_c(ns);
            flags.push("NROS_BOOT_SET_NAMESPACE");
        }
    }
    let set_flags = if flags.is_empty() {
        "0".to_string()
    } else {
        flags.join(" | ")
    };
    let _ = writeln!(
        out,
        "/* Phase 266 (RFC-0045) — baked boot config: post-link readable + session name. */\n\
         static const struct nros_baked_boot_config NROS_BOOT_CONFIG\n\
         #if defined(__GNUC__) || defined(__clang__)\n\
             __attribute__((section(\".nros_boot_config\"), used))\n\
         #endif\n\
             = {{\n\
             .magic      = NROS_BOOT_CONFIG_MAGIC,\n\
             .version    = NROS_BOOT_CONFIG_VERSION,\n\
             .set_flags  = {set_flags},\n\
             .domain_id  = 0,\n\
             .node_name  = \"{node_name}\",\n\
             .locator    = \"\",\n\
             .namespace_ = \"{namespace}\",\n\
             .rmw        = \"\",\n\
         }};",
    );
    Ok(())
}

/// Sanitise a pkg name into a valid identifier (`-` → `_`).
///
/// Mirrors the rule the Rust `nros::node!()` macro and the cmake fn
/// `nano_ros_node_register()` already apply (see
/// `packages/core/nros-macros/src/main_macro.rs::pkg_to_crate_ident`).
pub fn sanitize_pkg(pkg: &str) -> String {
    let mut out = String::with_capacity(pkg.len());
    for c in pkg.chars() {
        if c.is_ascii_alphanumeric() || c == '_' {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    out
}

/// Issue #52 — decompose `qos_overrides.<topic>.<role>.<policy>` parameters
/// into the baked override table, sorted for deterministic emission.
///
/// Issue 0303 — this REJECTS an unusable override (unknown role or policy,
/// unparseable value) instead of filtering it away. Silence is the wrong
/// failure mode for QoS: the image would run different delivery semantics than
/// the model declares, invisibly.
fn qos_overrides_from_params(params: &[(String, String)]) -> Result<Vec<QoSOverrideSpec>> {
    nros_orchestration_ir::qos_override::lower_all(
        params.iter().map(|(k, v)| (k.as_str(), v.as_str())),
    )
    .map_err(|e| eyre::eyre!("{e}"))
}

/// The complement of [`qos_overrides_from_params`]: everything that is NOT a
/// QoS override, which is what gets baked as declared parameters.
fn non_qos_params(params: &[(String, String)]) -> Vec<(String, String)> {
    params
        .iter()
        .filter(|(name, _)| !nros_orchestration_ir::qos_override::is_qos_override(name))
        .cloned()
        .collect()
}

/// R1-N2 (RFC-0052 / phase-296 W4.1) — build a [`Plan`] from a resolved
/// SystemModel instead of parsing a launch file. The model is the
/// canonical artifact: structure supplies the node list (params included
/// — the embedded image has no record.json), execution supplies tiers,
/// group bindings, capability features, and per-node deploy facts.
///
/// Board slice: with deploy entries present, `board == "native"/"posix"`
/// keeps `linux`-targeted nodes and any other board key keeps
/// `mcu:<that board>` nodes; a model WITHOUT deploy entries deploys every
/// node (single-image case). An empty slice is a hard error — a bake for
/// a board no node targets is a placement bug, not an empty entry.
pub fn plan_from_model(model_path: &Path, board: Option<String>) -> Result<Plan> {
    use crate::orchestration::model_ingest;
    use ros_launch_manifest_model::Target;

    let model = model_ingest::load_model(model_path)?;
    let board = board.unwrap_or_else(|| "native".to_string());
    let target_rtos = board_to_rtos(&board).to_string();

    // phase-315 / issue 0288 — does the model place ANYTHING on this board?
    //
    // `execution.deploy` is a PARTITION (node -> one target). That is right for
    // machines and wrong for board build-variants, where every board runs every
    // node and the map cannot say so. A board the map never mentions is
    // therefore not "a board with nothing on it" — it is a board the model has
    // no opinion about, which is exactly what the `(None, _) => true` arm below
    // already says per-NODE, lifted to the board level.
    //
    // This is the SECOND copy of this rule: `nros-macros`'s `main_macro.rs` has
    // the same filter for the Rust path and was fixed first. Fixing one and not
    // the other is why `examples/workspaces/c`'s freertos entry still failed
    // with `places no nodes on board mps2-an385-freertos` after the Rust side
    // was green. Two entry emitters, one rule — see issue 0358 for the class.
    let board_mentioned = model.execution.deploy.values().any(|d| match &d.target {
        Some(Target::Linux) => matches!(board.as_str(), "native" | "posix"),
        Some(Target::Mcu { board: b }) => {
            b == &board
                || matches!(
                    d.extra.get("kind"),
                    Some(ros_launch_manifest_model::ExtraValue::Str(k)) if k == &board
                )
        }
        None => false,
    });

    let keep = |fqn: &str| -> bool {
        let Some(dep) = model.execution.deploy.get(fqn) else {
            return model.execution.deploy.is_empty();
        };
        if !board_mentioned {
            return true;
        }
        match (&dep.target, board.as_str()) {
            // Board-agnostic (multi-board system, issue 0356): no board
            // named — this entry's board decides.
            (None, _) => true,
            (Some(Target::Linux), "native" | "posix") => true,
            // Exact board key, or the deploy's platform kind (the
            // integrator's `[deploy.<t>] kind = "zephyr"`, carried in
            // `extra.kind`) — the entry codegen key is the board FAMILY
            // ("zephyr") while deploys name the concrete board
            // ("fvp-aemv8r-smp"), so both spellings must slice.
            (Some(Target::Mcu { board: b }), key) => {
                b == key
                    || matches!(
                        dep.extra.get("kind"),
                        Some(ros_launch_manifest_model::ExtraValue::Str(k)) if k == key
                    )
            }
            _ => false,
        }
    };

    let mut nodes: Vec<PlanNode> = Vec::new();
    for (fqn, inst) in &model.structure.nodes {
        if !keep(fqn) {
            continue;
        }
        let bare = fqn.rsplit('/').next().unwrap_or(fqn).to_string();
        let namespace = {
            let ns = &fqn[..fqn.len() - bare.len()];
            let ns = ns.trim_end_matches('/');
            if ns.is_empty() {
                "/".to_string()
            } else {
                ns.to_string()
            }
        };
        let exec = inst
            .exec
            .clone()
            .or_else(|| {
                // Library-component node: the plugin (class) names it; the
                // typed emitters resolve the real class/header from cmake
                // metadata by (pkg, exec/name) as usual.
                inst.plugin
                    .as_deref()
                    .map(|p| p.rsplit("::").next().unwrap_or(p).to_string())
            })
            .ok_or_else(|| eyre::eyre!("model node '{fqn}' has neither exec nor plugin"))?;
        // #276 (model path) — project `params_files` YAML under the inline
        // `<param>` values. The launch path has its own projection
        // (`resolve_node_params`), but every live bake now goes through the
        // MODEL, so the projection has to exist here too.
        let resolved: Vec<(String, String)> = inst
            .resolved_params(fqn)
            .iter()
            .map(|(k, v)| (k.clone(), v.to_bake_string()))
            .collect();
        let qos_overrides =
            qos_overrides_from_params(&resolved).wrap_err_with(|| format!("node `{fqn}`"))?;
        let params = non_qos_params(&resolved);
        // Group→tier from the model's resolved bindings (`<fqn>/<group>`).
        let mut group_tiers: BTreeMap<String, String> = BTreeMap::new();
        let prefix = format!("{fqn}/");
        for (key, tier) in &model.execution.bindings {
            if let Some(group) = key.strip_prefix(&prefix) {
                group_tiers.insert(group.to_string(), tier.clone());
            }
        }
        nodes.push(PlanNode {
            pkg: inst.pkg.clone().unwrap_or_default(),
            exec,
            name: Some(bare),
            namespace: Some(namespace),
            class_name: None,
            class_header: None,
            lang: None,
            shape: None,
            qos_overrides,
            params,
            remaps: inst
                .remaps
                .iter()
                .map(|r| (r.from.clone(), r.to.clone()))
                .collect(),
            callback_groups: Vec::new(),
            sched_context: None,
            group_tiers,
        });
    }
    if nodes.is_empty() {
        bail!(
            "SystemModel `{}` places no nodes on board `{board}` — check \
             execution.deploy targets",
            model_path.display()
        );
    }

    let tiers: BTreeMap<String, TierDef> = model
        .execution
        .tiers
        .iter()
        .map(|(name, t)| {
            (
                name.clone(),
                crate::orchestration::model_ingest::tier_from_model(t, &target_rtos),
            )
        })
        .collect();

    let lifecycle = model
        .structure
        .nodes
        .values()
        .find_map(|n| n.lifecycle_autostart)
        .map(|a| {
            match a {
                ros_launch_manifest_model::Autostart::None => "none",
                ros_launch_manifest_model::Autostart::Configure => "configure",
                ros_launch_manifest_model::Autostart::Active => "active",
            }
            .to_string()
        });
    let features = &model.execution.features;
    let param_services = features.iter().any(|f| f == "param_services");
    let safety = features.iter().any(|f| f == "safety").then_some(true);

    Ok(Plan {
        board,
        nodes,
        depfile_paths: vec![model_path.to_path_buf()],
        bringup: "system-model".to_string(),
        launch_file: model_path.to_path_buf(),
        lifecycle,
        param_services,
        safety,
        tiers,
        node_overrides: Vec::new(),
        resolved_tiers: None,
    })
}

/// Phase 269 (W4) — derive the RTOS key recognised by [`resolve_tiers`] from a
/// board deploy key. The mapping mirrors the `rtos_spec` function in
/// `nros-orchestration-ir` (`"posix" | "native"`, `"freertos"`, …).
pub fn board_to_rtos(board: &str) -> &str {
    match board {
        "native" | "posix" => "posix",
        b if b.starts_with("freertos") || b.contains("freertos") => "freertos",
        b if b.starts_with("zephyr") || b.contains("zephyr") => "zephyr",
        b if b.starts_with("nuttx") || b.contains("nuttx") => "nuttx",
        b if b.starts_with("threadx") || b.contains("threadx") => "threadx",
        _ => "posix",
    }
}

/// Phase 269 (W4) — resolve `[tiers.*]` + `[[node_overrides]]` + per-node
/// `callback_groups` (from cmake metadata) into a [`ResolvedTierTable`] and
/// stamp each [`PlanNode::sched_context`] with its 0-based tier index
/// (highest-priority-first order from the resolver).
///
/// Must be called AFTER [`metadata::enrich_plan`] so that
/// [`PlanNode::callback_groups`] is populated. The caller supplies the RTOS
/// key (use [`board_to_rtos`] to derive it from `plan.board`).
///
/// No-op (returns `Ok(())`) when both `plan.tiers` and `plan.node_overrides`
/// are empty and no node declares callback groups — the guard keeps
/// single-tier entries byte-identical.
pub fn resolve_plan_sched(plan: &mut Plan, target_rtos: &str) -> Result<()> {
    let has_groups = plan
        .nodes
        .iter()
        .any(|n| !n.callback_groups.is_empty() || !n.group_tiers.is_empty());
    if plan.tiers.is_empty() && plan.node_overrides.is_empty() && !has_groups {
        return Ok(());
    }

    // Component instance names from the launch (match [[node_overrides]].name).
    let component_names: BTreeSet<&str> = plan
        .nodes
        .iter()
        .map(|n| n.name.as_deref().unwrap_or(n.exec.as_str()))
        .collect();

    // Per-node callback group declarations. Phase 273 (W2): when the node carries
    // `group_tiers` from system.toml [[component]], use those tiers directly instead
    // of defaulting to "default" (which required [[node_overrides]] to reassign).
    // Fallback: group ID with DEFAULT_TIER (old path, [[node_overrides]] still work).
    // If callback_groups is empty but group_tiers is set, synthesize from group_tiers.
    let mut callback_groups_map: BTreeMap<String, Vec<CallbackGroupDecl>> = BTreeMap::new();
    for n in &plan.nodes {
        let node_name = n.name.as_deref().unwrap_or(n.exec.as_str()).to_string();
        let decls: Vec<CallbackGroupDecl> = if !n.callback_groups.is_empty() {
            // cmake-declared groups (via enrich_plan): look up tier from group_tiers.
            n.callback_groups
                .iter()
                .map(|g| {
                    let tier = n
                        .group_tiers
                        .get(g)
                        .map(|t| t.as_str())
                        .unwrap_or(DEFAULT_TIER);
                    CallbackGroupDecl {
                        id: g.clone(),
                        r#type: "MutuallyExclusive".to_string(),
                        tier: tier.to_string(),
                    }
                })
                .collect()
        } else if !n.group_tiers.is_empty() {
            // No cmake callback_groups yet; synthesize from system.toml group_tiers.
            n.group_tiers
                .iter()
                .map(|(id, tier)| CallbackGroupDecl {
                    id: id.clone(),
                    r#type: "MutuallyExclusive".to_string(),
                    tier: tier.clone(),
                })
                .collect()
        } else {
            Vec::new()
        };
        if !decls.is_empty() {
            callback_groups_map.insert(node_name, decls);
        }
    }

    let table = resolve_tiers(
        &plan.tiers,
        &plan.node_overrides,
        &component_names,
        &callback_groups_map,
        target_rtos,
    )
    .map_err(|e: TierResolveError| eyre::eyre!("tier resolution failed: {e}"))?;

    // Stamp PlanNode.sched_context with the 0-based index into the ordered tier list
    // (highest-priority-first). Nodes not assigned to any tier keep `sched_context = None`.
    let mut tier_map: HashMap<String, u8> = HashMap::new();
    for (i, tier) in table.tiers.iter().enumerate() {
        for (node_name, _group) in &tier.members {
            tier_map.insert(node_name.clone(), i as u8);
        }
    }
    for n in &mut plan.nodes {
        let node_name = n.name.as_deref().unwrap_or(n.exec.as_str());
        if let Some(&idx) = tier_map.get(node_name) {
            n.sched_context = Some(idx);
        }
    }

    plan.resolved_tiers = Some(table);
    Ok(())
}

/// Write the depfile in GNU-make `target: dep1 dep2 …` form so cmake's
/// `CMAKE_CONFIGURE_DEPENDS` / make-style consumers ingest it directly.
///
/// The "target" line is the generated TU path (or whatever the caller
/// supplies); each dep is one absolute path on its own continuation
/// line so newline-quoting + path escaping stays trivial.
pub fn write_depfile(target: &Path, deps: &[PathBuf], depfile: &Path) -> Result<()> {
    let mut out = String::new();
    out.push_str(&escape_make(&target.display().to_string()));
    out.push(':');
    for dep in deps {
        out.push_str(" \\\n    ");
        out.push_str(&escape_make(&dep.display().to_string()));
    }
    out.push('\n');
    if let Some(parent) = depfile.parent() {
        std::fs::create_dir_all(parent)
            .wrap_err_with(|| format!("create depfile parent `{}`", parent.display()))?;
    }
    std::fs::write(depfile, out)
        .wrap_err_with(|| format!("write depfile `{}`", depfile.display()))?;
    Ok(())
}

/// Make-format target/dep escape: spaces → `\ `, `#` → `\#`.
fn escape_make(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            ' ' => out.push_str("\\ "),
            '#' => out.push_str("\\#"),
            other => out.push(other),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sanitize_pkg_dash_to_underscore() {
        assert_eq!(sanitize_pkg("talker-pkg"), "talker_pkg");
        assert_eq!(sanitize_pkg("a.b/c"), "a_b_c");
        assert_eq!(sanitize_pkg("plain_pkg"), "plain_pkg");
    }

    #[test]
    fn cmake_link_target_uses_mangled_pkg() {
        let n = PlanNode {
            pkg: "talker-pkg".into(),
            exec: "talker".into(),
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
            group_tiers: BTreeMap::new(),
        };
        assert_eq!(n.cmake_link_target(), "talker_pkg_talker_component");
    }

    /// Issue 0276 + phase-54 — a model's `params_files` project into
    /// `PlanNode.params` at codegen time, and `qos_overrides.*` split out into
    /// typed [`QoSOverrideSpec`]s instead of being baked as parameters.
    ///
    /// The resolution algorithm itself lives in the shared `model` crate; this
    /// asserts the WIRING, not a second copy. The param file writes the node
    /// block ABOVE the wildcard on purpose: section precedence is by
    /// specificity, not file order, so this fails if that ordering is lost.
    #[test]
    fn model_params_project_with_specificity_and_qos_split() {
        use tempfile::TempDir;
        let tmp = TempDir::new().unwrap();
        let model = tmp.path().join("system_model.yaml");
        std::fs::write(
            &model,
            r#"
meta:
  version: 1
structure:
  scopes:
    /: {}
  nodes:
    /talker:
      scope: /
      pkg: talker_pkg
      exec: talker
      node_name: talker
      params:
        rate: "50"
        qos_overrides./chatter.publisher.reliability: best_effort
      params_files:
        - "talker:\n  ros__parameters:\n    use_sim_time: true\n    limits:\n      max_accel: 1.5\n/**:\n  ros__parameters:\n    use_sim_time: false\n    rate: 10\nother_node:\n  ros__parameters:\n    rate: 999\n"
execution: {}
contracts: {}
"#,
        )
        .unwrap();

        let plan = plan_from_model(&model, Some("native".into())).expect("plan from model");
        let node = &plan.nodes[0];
        let params: BTreeMap<_, _> = node.params.iter().cloned().collect();

        // The node block sets it true and the wildcard false, with the node
        // block written FIRST — specificity decides, not file order.
        assert_eq!(params.get("use_sim_time").map(String::as_str), Some("true"));
        // Nested map flattened to a dotted key; a float keeps its ".0".
        assert_eq!(
            params.get("limits.max_accel").map(String::as_str),
            Some("1.5")
        );
        // Inline model params outrank the file.
        assert_eq!(params.get("rate").map(String::as_str), Some("50"));
        // A section naming a different node contributes nothing.
        assert!(!params.values().any(|v| v == "999"));
        // QoS travels typed, never as a declared parameter.
        assert!(
            !params.keys().any(|k| k.starts_with("qos_overrides.")),
            "qos override leaked into params: {params:?}"
        );
        // Lowered to codes at plan time (issue 0303): publisher(0),
        // reliability(0), best_effort(0).
        assert_eq!(node.qos_overrides.len(), 1);
        assert_eq!(node.qos_overrides[0].topic, "/chatter");
        assert_eq!(
            node.qos_overrides[0].role,
            nros_orchestration_ir::qos_override::role::PUBLISHER
        );
        assert_eq!(
            node.qos_overrides[0].policy,
            nros_orchestration_ir::qos_override::policy::RELIABILITY
        );
        assert_eq!(node.qos_overrides[0].value, 0);
    }

    /// #236 / issue 0356 — a board-agnostic deploy (`target: None`, a
    /// multi-board system's placement) keeps the node on every board's
    /// slice. An explicit `target: linux` still rejects non-native boards.
    #[test]
    fn model_unplaced_target_is_board_agnostic() {
        use ros_launch_manifest_model::{Deploy, Target};
        let unplaced = Deploy::default();
        assert_eq!(unplaced.target, None);
        let placed_linux = Deploy {
            target: Some(Target::Linux),
            ..Default::default()
        };

        // Mirror the `keep` board match used by plan_from_model + the macro.
        let board_ok = |dep: &Deploy, key: &str| -> bool {
            match (&dep.target, key) {
                (None, _) => true,
                (Some(Target::Linux), "native" | "posix") => true,
                (Some(Target::Mcu { board: b }), key) => b == key,
                _ => false,
            }
        };
        for board in ["native", "posix", "zephyr", "freertos"] {
            assert!(board_ok(&unplaced, board), "unplaced must keep on {board}");
        }
        assert!(board_ok(&placed_linux, "native"));
        assert!(
            !board_ok(&placed_linux, "zephyr"),
            "explicit linux placement must still reject a zephyr board"
        );
    }

    /// Phase 211.H (issue #52) — `qos_overrides.<topic>.<role>.<policy>` params
    /// decompose into sorted `QoSOverrideSpec`s; non-matching params are ignored.
    #[test]
    fn qos_overrides_decompose_from_params() {
        let params = vec![
            (
                "qos_overrides./chatter.publisher.reliability".to_string(),
                "best_effort".to_string(),
            ),
            ("use_sim_time".to_string(), "true".to_string()),
            (
                "qos_overrides./chatter.subscription.durability".to_string(),
                "transient_local".to_string(),
            ),
        ];
        use nros_orchestration_ir::qos_override::{policy, role};
        let got = qos_overrides_from_params(&params).expect("lower");
        assert_eq!(got.len(), 2);
        // sorted (topic, role, policy): publisher before subscription.
        assert_eq!(got[0].role, role::PUBLISHER);
        assert_eq!(got[0].policy, policy::RELIABILITY);
        assert_eq!(got[0].topic, "/chatter");
        assert_eq!(got[0].value, 0); // best_effort
        assert_eq!(got[1].role, role::SUBSCRIPTION);
        assert_eq!(got[1].policy, policy::DURABILITY);
        assert_eq!(got[1].value, 1); // transient_local
    }

    /// Issue 0303 — an override the bake cannot lower FAILS the codegen; it is
    /// not filtered away. Silence would ship different delivery semantics than
    /// the model declares, with nothing to read.
    #[test]
    fn an_unusable_qos_override_fails_codegen() {
        let params = vec![(
            // `pub` instead of `publisher` — the typo that used to vanish.
            "qos_overrides./chatter.pub.reliability".to_string(),
            "reliable".to_string(),
        )];
        let err = qos_overrides_from_params(&params).expect_err("must reject");
        let msg = err.to_string();
        assert!(
            msg.contains("qos_overrides./chatter.pub.reliability"),
            "{msg}"
        );
        assert!(msg.contains("publisher"), "{msg}");
    }

    #[test]
    fn lang_parse() {
        assert_eq!(Lang::parse("rust").unwrap(), Lang::Rust);
        assert_eq!(Lang::parse("cpp").unwrap(), Lang::Cpp);
        assert_eq!(Lang::parse("c++").unwrap(), Lang::Cpp);
        assert_eq!(Lang::parse("c").unwrap(), Lang::C);
        assert!(Lang::parse("python").is_err());
    }

    #[test]
    fn write_depfile_emits_make_format() {
        use tempfile::TempDir;
        let tmp = TempDir::new().unwrap();
        let target = tmp.path().join("gen.cpp");
        let dep_a = tmp.path().join("a.xml");
        let dep_b = tmp.path().join("dir with space/b.xml");
        let depfile = tmp.path().join("gen.d");
        write_depfile(&target, &[dep_a.clone(), dep_b.clone()], &depfile).unwrap();
        let body = std::fs::read_to_string(&depfile).unwrap();
        assert!(body.starts_with(&format!("{}:", target.display())));
        assert!(body.contains(&format!("{}", dep_a.display())));
        // Space in dep_b should be escaped.
        assert!(body.contains("dir\\ with\\ space"));
    }

    /// Helper that builds a single-node [`Plan`] with the given node name.
    fn single_node_plan(name: &str) -> Plan {
        Plan {
            board: "native".into(),
            nodes: vec![PlanNode {
                pkg: "test_pkg".into(),
                exec: name.into(),
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
                group_tiers: BTreeMap::new(),
            }],
            depfile_paths: vec![],
            bringup: "demo".into(),
            launch_file: std::path::PathBuf::from("/tmp/x.launch.xml"),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers: Default::default(),
            node_overrides: Vec::new(),
            resolved_tiers: None,
        }
    }

    /// Phase 266 (W6) — a node name of exactly 63 bytes must succeed (fits in
    /// `char node_name[64]` with one byte left for the NUL terminator).
    #[test]
    fn boot_config_node_name_63_bytes_ok() {
        let name = "a".repeat(63);
        assert_eq!(name.len(), 63);
        let plan = single_node_plan(&name);
        let mut out = String::new();
        assert!(
            emit_boot_config_static(&mut out, &plan).is_ok(),
            "63-byte name should be accepted"
        );
        assert!(
            out.contains(&name),
            "emitted output must contain the node name"
        );
    }

    /// Phase 266 (W6) — a node name of 64 bytes must be rejected with a clear
    /// error message (the C field only holds 63 usable bytes + NUL).
    #[test]
    fn boot_config_node_name_64_bytes_err() {
        let name = "b".repeat(64);
        assert_eq!(name.len(), 64);
        let plan = single_node_plan(&name);
        let mut out = String::new();
        let err =
            emit_boot_config_static(&mut out, &plan).expect_err("64-byte name must be rejected");
        assert!(
            err.contains("64 bytes"),
            "error should mention the byte count; got: {err}"
        );
        assert!(
            err.contains("node_name"),
            "error should mention the field name; got: {err}"
        );
        assert!(
            err.contains("63 bytes"),
            "error should mention the 63-byte limit; got: {err}"
        );
    }

    /// Phase 269 (W0) — non-QoS params are baked into PlanNode.params; qos_overrides.* params
    /// go to qos_overrides, not to params.
    #[test]
    fn non_qos_params_split_from_qos_params() {
        let params = vec![
            ("p".to_string(), "v".to_string()),
            (
                "qos_overrides./chatter.publisher.reliability".to_string(),
                "best_effort".to_string(),
            ),
            ("count".to_string(), "42".to_string()),
        ];
        let non_qos = non_qos_params(&params);
        assert_eq!(non_qos.len(), 2);
        assert_eq!(non_qos[0], ("p".into(), "v".into()));
        assert_eq!(non_qos[1], ("count".into(), "42".into()));
    }

    /// Phase 269 (W4) — resolve_plan_sched assigns PlanNode.sched_context from tiers +
    /// node_overrides + callback_groups. A 2-tier plan with node overrides yields
    /// correct sched_context indices (highest-priority-first: high=0, low=1).
    #[test]
    fn resolve_plan_sched_stamps_sched_context_indices() {
        use crate::orchestration::cargo_metadata_schema::{
            CallbackGroupOverride, NodeOverride, TierDef, TierRtosSpec,
        };
        use std::path::PathBuf;

        let high_tier = TierDef {
            spin_period_us: Some(10_000),
            posix: Some(TierRtosSpec {
                priority: 80,
                stack_bytes: None,
                preempt_threshold: None,
                time_slice_us: None,
                sched_class: None,
                core: None,
                deadline_us: None,
                budget_us: None,
                period_us: None,
            }),
            ..Default::default()
        };
        let low_tier = TierDef {
            spin_period_us: Some(100_000),
            posix: Some(TierRtosSpec {
                priority: 10,
                stack_bytes: None,
                preempt_threshold: None,
                time_slice_us: None,
                sched_class: None,
                core: None,
                deadline_us: None,
                budget_us: None,
                period_us: None,
            }),
            ..Default::default()
        };
        let mut tiers = std::collections::BTreeMap::new();
        tiers.insert("high".to_string(), high_tier);
        tiers.insert("low".to_string(), low_tier);

        let node_overrides = vec![
            NodeOverride {
                name: "ctrl".to_string(),
                callback_groups: vec![CallbackGroupOverride {
                    id: "ctrl_grp".to_string(),
                    tier: "high".to_string(),
                }],
            },
            NodeOverride {
                name: "telem".to_string(),
                callback_groups: vec![CallbackGroupOverride {
                    id: "telem_grp".to_string(),
                    tier: "low".to_string(),
                }],
            },
        ];

        let mut plan = Plan {
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
                    sched_context: None,
                    group_tiers: BTreeMap::new(),
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
                    sched_context: None,
                    group_tiers: BTreeMap::new(),
                },
            ],
            depfile_paths: vec![],
            bringup: "demo".into(),
            launch_file: PathBuf::from("/tmp/x.launch.xml"),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers,
            node_overrides,
            resolved_tiers: None,
        };

        resolve_plan_sched(&mut plan, "posix").expect("resolve_plan_sched should succeed");

        // resolved_tiers populated.
        assert!(plan.resolved_tiers.is_some(), "resolved_tiers must be set");
        let table = plan.resolved_tiers.as_ref().unwrap();
        assert!(
            !table.is_single_tier(),
            "two-tier plan must not be single-tier"
        );
        // highest-priority-first: high (80) = idx 0, low (10) = idx 1.
        assert_eq!(table.tiers[0].name, "high");
        assert_eq!(table.tiers[1].name, "low");
        // PlanNode.sched_context stamped correctly.
        assert_eq!(
            plan.nodes[0].sched_context,
            Some(0),
            "ctrl (high tier) must get sched_context=0"
        );
        assert_eq!(
            plan.nodes[1].sched_context,
            Some(1),
            "telem (low tier) must get sched_context=1"
        );
    }

    /// Phase 273 (W2) — resolve_plan_sched uses PlanNode.group_tiers to assign
    /// callback-group tiers directly from system.toml, without needing
    /// [[node_overrides]]. A 2-tier plan with group_tiers set on each node
    /// yields the same sched_context stamping as the node_overrides path.
    #[test]
    fn resolve_plan_sched_uses_group_tiers_directly() {
        use crate::orchestration::cargo_metadata_schema::{TierDef, TierRtosSpec};
        use std::path::PathBuf;

        let mut tiers = BTreeMap::new();
        tiers.insert(
            "high".to_string(),
            TierDef {
                spin_period_us: Some(10_000),
                posix: Some(TierRtosSpec {
                    priority: 80,
                    stack_bytes: None,
                    preempt_threshold: None,
                    time_slice_us: None,
                    sched_class: None,
                    core: None,
                    deadline_us: None,
                    budget_us: None,
                    period_us: None,
                }),
                ..Default::default()
            },
        );
        tiers.insert(
            "low".to_string(),
            TierDef {
                spin_period_us: Some(100_000),
                posix: Some(TierRtosSpec {
                    priority: 10,
                    stack_bytes: None,
                    preempt_threshold: None,
                    time_slice_us: None,
                    sched_class: None,
                    core: None,
                    deadline_us: None,
                    budget_us: None,
                    period_us: None,
                }),
                ..Default::default()
            },
        );

        let mut ctrl_gt = BTreeMap::new();
        ctrl_gt.insert("ctrl_grp".to_string(), "high".to_string());
        let mut telem_gt = BTreeMap::new();
        telem_gt.insert("telem_grp".to_string(), "low".to_string());

        let mut plan = Plan {
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
                    sched_context: None,
                    group_tiers: ctrl_gt,
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
                    sched_context: None,
                    group_tiers: telem_gt,
                },
            ],
            depfile_paths: vec![],
            bringup: "demo".into(),
            launch_file: PathBuf::from("/tmp/x.launch.xml"),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers,
            node_overrides: Vec::new(), // no [[node_overrides]] needed
            resolved_tiers: None,
        };

        resolve_plan_sched(&mut plan, "posix").expect("resolve_plan_sched should succeed");

        let table = plan
            .resolved_tiers
            .as_ref()
            .expect("resolved_tiers must be set");
        assert!(!table.is_single_tier());
        assert_eq!(table.tiers[0].name, "high");
        assert_eq!(table.tiers[1].name, "low");
        // group_tiers drives the tier assignment directly.
        assert_eq!(
            plan.nodes[0].sched_context,
            Some(0),
            "ctrl (high) must get sched_context=0"
        );
        assert_eq!(
            plan.nodes[1].sched_context,
            Some(1),
            "telem (low) must get sched_context=1"
        );
        // Group members must appear in the tier table.
        assert!(
            table.tiers[0]
                .members
                .contains(&("ctrl".to_string(), "ctrl_grp".to_string())),
            "ctrl/ctrl_grp must be a high-tier member"
        );
        assert!(
            table.tiers[1]
                .members
                .contains(&("telem".to_string(), "telem_grp".to_string())),
            "telem/telem_grp must be a low-tier member"
        );
    }

    /// Phase 269 (W4) — resolve_plan_sched on a plan with no tiers, no overrides,
    /// and no callback groups is a no-op (resolved_tiers stays None).
    /// issue 0794 — the emitter set `NROS_BOOT_SET_NODE_NAME` and nothing else,
    /// so a launch-declared namespace never reached the image while the field,
    /// the bit, the packer and the READER all existed and worked. This asserts
    /// the producer half, in both directions.
    #[test]
    fn a_launch_declared_namespace_reaches_the_baked_boot_config() {
        use std::path::PathBuf;
        fn plan_with(namespace: Option<&str>) -> Plan {
            Plan {
                board: "native".into(),
                nodes: vec![PlanNode {
                    pkg: "talker_pkg".into(),
                    exec: "talker".into(),
                    name: Some("talker".into()),
                    namespace: namespace.map(str::to_string),
                    class_name: None,
                    class_header: None,
                    lang: Some("c".into()),
                    shape: None,
                    qos_overrides: Vec::new(),
                    params: Vec::new(),
                    remaps: Vec::new(),
                    callback_groups: Vec::new(),
                    sched_context: None,
                    group_tiers: Default::default(),
                }],
                depfile_paths: vec![],
                bringup: "demo".into(),
                launch_file: PathBuf::from("/tmp/x.launch.xml"),
                lifecycle: None,
                param_services: false,
                safety: None,
                tiers: Default::default(),
                node_overrides: Vec::new(),
                resolved_tiers: None,
            }
        }

        let mut with_ns = String::new();
        emit_boot_config_static(&mut with_ns, &plan_with(Some("/robot1"))).unwrap();
        assert!(
            with_ns.contains(".namespace_ = \"/robot1\""),
            "the namespace must be baked, not dropped:\n{with_ns}"
        );
        assert!(
            with_ns.contains("NROS_BOOT_SET_NAMESPACE"),
            "the reader branches on the BIT, so baking the field without it \
             changes nothing:\n{with_ns}"
        );

        // The other direction: no namespace declared means the bit stays clear,
        // so the reader falls through to the next rung of RFC-0045's ladder
        // rather than reading an empty string as "configured to root".
        let mut without = String::new();
        emit_boot_config_static(&mut without, &plan_with(None)).unwrap();
        assert!(
            !without.contains("NROS_BOOT_SET_NAMESPACE"),
            "an undeclared namespace must not set the bit:\n{without}"
        );
        assert!(without.contains("NROS_BOOT_SET_NODE_NAME"));
    }

    #[test]
    fn resolve_plan_sched_no_tiers_is_noop() {
        use std::path::PathBuf;
        let mut plan = Plan {
            board: "native".into(),
            nodes: vec![PlanNode {
                pkg: "talker_pkg".into(),
                exec: "talker".into(),
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
                group_tiers: BTreeMap::new(),
            }],
            depfile_paths: vec![],
            bringup: "demo".into(),
            launch_file: PathBuf::from("/tmp/x.launch.xml"),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers: Default::default(),
            node_overrides: Vec::new(),
            resolved_tiers: None,
        };
        resolve_plan_sched(&mut plan, "posix").expect("no-op should succeed");
        assert!(
            plan.resolved_tiers.is_none(),
            "no-tier plan must leave resolved_tiers as None"
        );
        assert!(
            plan.nodes[0].sched_context.is_none(),
            "no-tier plan must leave sched_context as None"
        );
    }
}
