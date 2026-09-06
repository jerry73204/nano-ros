//! Phase 219.B — C++ Entry-pkg TU emitter.
//!
//! Maps a [`Plan`] (see `super::mod`) onto the canonical generated
//! `main.cpp` shape from `docs/roadmap/archived/phase-219-cpp-entry-pkg.md` §3.3.
//!
//! The TU pulls in `<nros/main.hpp>` (the Phase 219.E header that
//! defines the `NROS_MAIN` declarative marker), declares one
//! `extern "C" int32_t __nros_component_<pkg>_register(...)` per
//! launch-XML node, then invokes them in launch order from inside a
//! lambda passed to `nros::board::LinuxBoard::run(...)`.
//!
//! Today the Native-board entry boils down to:
//!
//! - `nros::init()` (no-arg, reads `$NROS_LOCATOR` / `$ROS_DOMAIN_ID`).
//! - Call each Node-pkg register fn in turn — they describe their
//!   `<node>` / `<entity>` set against the supplied `NodeContext`.
//! - `nros::spin()` until `nros::ok()` flips false.
//! - `nros::shutdown()`.
//!
//! The thin `nros::board::<Board>::run(lambda)` adapter shipped by
//! `packages/api/nros-cpp/include/nros/main.hpp` owns the
//! init/spin/shutdown ritual so the generated TU stays one declarative
//! lambda. Phase 235.B added the embedded `ZephyrBoard` sibling: a
//! non-`native` board key (e.g. `"zephyr"`, derived by `nano_ros_entry`
//! from the Phase 215 `NROS_BOARD_RUNNER`) emits
//! `nros::board::ZephyrBoard::run(...)`, which owns the Zephyr + Cyclone
//! `init → network-wait → register → spin → shutdown` lifecycle.

use super::{
    BootConfigView, DeclsView, Plan, QosRowView, boot_config_view, decls_view, qos_views,
    sanitize_pkg,
};

/// The C++ board class an entry calls.
///
/// phase-432 W2.2 — this is a RENDERING of the board family, not a second
/// table. The nineteen board keys collapse onto five families in
/// `nros_entry_lower::board_family`, which is the neutral fact; a `::nros::board::`
/// path is C++'s spelling of it and belongs here, in the emitter, rather than
/// in the lowering. RFC-0091 §8b found the first draft leaking exactly this
/// string into the IR, where a pure-C or Zig pack could not use it.
fn board_cpp_path(board: &str) -> &'static str {
    match nros_entry_lower::board_family(board) {
        nros_entry_lower::BoardFamily::Native => "::nros::board::LinuxBoard",
        nros_entry_lower::BoardFamily::Zephyr => "::nros::board::ZephyrBoard",
        nros_entry_lower::BoardFamily::Nuttx => "::nros::board::NuttxBoard",
        nros_entry_lower::BoardFamily::Freertos => "::nros::board::FreertosBoard",
        nros_entry_lower::BoardFamily::Threadx => "::nros::board::ThreadxBoard",
    }
}

/// The boot wrapper a generated entry gets.
///
/// phase-432 W2.2 — re-exported from `nros-entry-lower`, which is where the
/// derivation lives now. It was computed here from a board-string match, and
/// the proc-macro computed the same thing a third time; a crate the macro can
/// afford is what lets both stop.
pub(crate) use nros_entry_lower::BootShape;

/// The boot shape for a board key.
pub(crate) fn boot_shape(board: &str) -> BootShape {
    nros_entry_lower::board_family(board).boot_shape()
}

pub(crate) fn board_is_embedded(board: &str) -> bool {
    nros_entry_lower::board_family(board).is_embedded()
}

/// phase-263 C2d — Zephyr is the exception among embedded boards: the Zephyr kernel
/// calls the application's `main()` DIRECTLY (there is no nano-ros `startup.c` owning
/// `main`), so a Zephyr LAUNCH entry emits a plain `int main(void)` driving
/// `ZephyrBoard::run_components` — NOT the `nros_app_main` + `NROS_APP_MAIN_REGISTER_VOID`
/// shape the FreeRTOS/NuttX/ThreadX startup paths require. The entry TU is added to the
/// Zephyr `app` target (`nano_ros_entry` zephyr branch), and the connect locator threads
/// in via the compile-time `CONFIG_NROS_ZENOH_LOCATOR` Kconfig (read through the
/// `NROS_ENTRY_LOCATOR` default in `<nros/main.hpp>`), not a baked `-D`.
pub(crate) fn board_is_zephyr(board: &str) -> bool {
    nros_entry_lower::board_family(board) == nros_entry_lower::BoardFamily::Zephyr
}

/// Phase 274.W3 — FreeRTOS embedded boards support `run_tiers` (one RTOS task per tier
/// over one shared session). Unlike the other embedded boards (Zephyr, NuttX, ThreadX)
/// which keep the single-executor sched-context path (W2), `FreertosBoard` has a C
/// `nros_board_freertos_run_tiers` implementation (nros-board-freertos) that mirrors
/// the Rust `run_tiers_entry`. The generated entry emits `nros_app_main` +
/// `NROS_APP_MAIN_REGISTER_VOID`, calling `FreertosBoard::run_tiers` (RFC-0015 §5).
pub(crate) fn board_is_freertos_embedded(board: &str) -> bool {
    nros_entry_lower::board_family(board) == nros_entry_lower::BoardFamily::Freertos
}

/// phase-281 W3 (nuttx) — NuttX embedded boards support `run_tiers` (one pthread
/// per tier over one shared session). Like `FreertosBoard`, `NuttxBoard` has a C
/// `nros_board_nuttx_run_tiers` implementation (nros-board-nuttx-qemu) that
/// mirrors the Rust `run_tiers_entry`; NuttX being POSIX, each non-boot tier is a
/// `pthread` (SCHED_FIFO at the tier's raw priority). The generated entry uses
/// the `nros_app_main` + `NROS_APP_MAIN_REGISTER_VOID` shape (the NuttX startup
/// path calls `app_main`, like FreeRTOS — NOT Zephyr's `main(void)`), calling
/// `NuttxBoard::run_tiers` (RFC-0015 §5).
pub(crate) fn board_is_nuttx(board: &str) -> bool {
    nros_entry_lower::board_family(board) == nros_entry_lower::BoardFamily::Nuttx
}

/// Phase 240.2 (RFC-0043) — **typed** entry emitter. Routes each launch node to
/// the REAL executor via its component object, instead of the legacy type-erased
/// `__nros_component_<pkg>_register` call into the synthesizing
/// `EntryNodeRuntime`. Per node: `#include` the component header, declare static
/// component + node storage (outlives the spin loop — the executor holds
/// `&component` as the dispatch context; no heap), construct the node + call
/// `component.configure(node)` (binds the real callbacks). `main` hands the setup
/// fn to `Board::run_components` (init → setup → `spin_once` loop → shutdown).
///
/// Each node is routed by its `lang` (Phase 240.4): a **C++** node needs
/// `class_name` + `class_header` (construct the class, call `configure(node)`);
/// a **C** node (`lang == "c"`) needs only its pkg — it is built via the C-ABI
/// factory + configure seam `__nros_c_component_<pkg>_{create,configure}`
/// (`NROS_C_COMPONENT`), to which the entry hands the node's `ffi_handle()`.
/// Returns an error naming the offending pkg on a missing requirement.
/// phase-308 W1 — what the generated TU does after `__nros_entry_setup` has
/// constructed and configured every node.
///
/// The setup body is identical for a real entry and for a metadata probe: both
/// need the same per-node includes, static storage, construction and
/// `configure` call, across all three component shapes (C++ `configure`, the C
/// ABI seam, rclcpp construct-with-handle). Only the tail differs — an entry
/// spins, a probe records and exits.
///
/// Splitting here rather than writing a second emitter is deliberate: a
/// parallel emitter would fork three shape-handlers, and the count those
/// probes produce is exactly what must not drift between them.
pub enum EntryTail<'a> {
    /// Hand the setup fn to `Board::run_components` (init → setup → spin →
    /// shutdown). The shipping entry.
    Board,
    /// Open a session against the recording RMW backend, run setup once, dump
    /// the recorded metadata, exit. Never spins.
    MetadataProbe(&'a ProbeExport),
}

/// phase-308 W1 — identity the probe stamps into the sidecar it writes.
pub struct ProbeExport {
    pub package: String,
    pub component: String,
    pub executable: String,
    /// `"c"` or `"cpp"` — the sidecar's `language` field.
    pub language: String,
    /// Absolute path the probe writes the sidecar to.
    pub out_path: String,
}

pub fn emit_typed(plan: &Plan) -> Result<String, String> {
    emit_typed_with_tail(plan, &EntryTail::Board)
}

/// phase-308 W1 — the metadata probe: the same TU an entry would be, with a
/// recording tail.
pub fn emit_typed_probe(plan: &Plan, export: &ProbeExport) -> Result<String, String> {
    emit_typed_with_tail(plan, &EntryTail::MetadataProbe(export))
}

/// The whole TU, as the template sees it.
///
/// Issue 1102 / phase-432 W2.3 — every field is ALREADY CORRECT. Board paths
/// come from `nros_entry_lower`, tier rows are VALUES rather than initialiser
/// text, and raw strings stay raw so the pack quotes them with its own filter
/// (RFC-0091 §8b). Three fields are still pre-rendered and say why below.
#[derive(serde::Serialize)]
struct CppEntryView {
    bringup: String,
    launch: String,
    board: String,
    /// phase-263 C2 — embedded boots through the board's `startup.c`; Zephyr
    /// is exempt because its kernel calls `main()` directly.
    app_main_include: bool,
    /// RFC-0044 — pulled in only when an rclcpp-shape node is present.
    rclcpp_include: bool,
    /// Unique C++ component headers, first-seen order. Deduped by HEADER while
    /// storage is per NODE, so this list and `storage` differ in length
    /// whenever two nodes share a header.
    headers: Vec<String>,
    /// Unique sanitized package symbols for the C factory/configure seam, and
    /// for the Rust install seam. Deduped by PACKAGE, same reason.
    c_pkgs: Vec<String>,
    rust_pkgs: Vec<String>,
    storage: Vec<CppStorageView>,
    tiers: Option<CppTiersView>,
    /// Single-executor path only, and only when the plan declares tiers this
    /// board cannot run as tasks.
    sched: Option<CppSchedView>,
    /// Single-executor path only; empty when `tiers` is set.
    setup_nodes: Vec<CppNodeView>,
    /// The param-services / lifecycle block that closes the single setup fn,
    /// rendered from `cpp_service_trailer.cpp.jinja`.
    trailer: String,
    /// phase-308 — set for a metadata probe, which returns before the boot
    /// config and the board wrapper.
    probe: Option<CppProbeView>,
    /// `None` for a probe, which returns before the blob. Rendered by the
    /// shared `boot_config.c.jinja` — the same partial the C pack includes.
    boot_config: Option<BootConfigView>,
    boot: Option<CppBootView>,
}

/// One node's static storage. A Rust node has none — it self-creates its node
/// on the shared executor — so it contributes no row at all rather than an
/// empty one.
#[derive(serde::Serialize)]
struct CppStorageView {
    index: usize,
    /// `"rclcpp"` gets an aligned arena slot; anything else gets a
    /// `::nros::Node`, plus a component object when `class` is set (a C node
    /// keeps its state in its own TU, so it has none).
    shape: &'static str,
    class: Option<String>,
}

#[derive(serde::Serialize)]
struct CppTiersView {
    n: usize,
    setups: Vec<CppTierSetupView>,
    /// One row per tier, STRUCTURED. The same neutral rows the C pack renders
    /// as `nros_native_tier_spec_t` — shared so the two packs cannot spell one
    /// tier two ways (that is what W1.2's gate exists to catch one layer down).
    tiers: Vec<super::TierView>,
}

#[derive(serde::Serialize)]
struct CppTierSetupView {
    index: usize,
    name: String,
    nodes: Vec<CppNodeView>,
    /// Only tier 0 registers param services and lifecycle.
    trailer: String,
}

/// The sched-context wiring a tiered plan emits when it cannot use
/// `run_tiers`: an embedded board without one (ThreadX), or an RFC-0047
/// group-split node, which per-tier setup functions cannot express.
#[derive(serde::Serialize)]
struct CppSchedView {
    n: usize,
    contexts: Vec<CppSchedContextView>,
    node_binds: Vec<CppNodeBindView>,
    group_binds: Vec<CppGroupBindView>,
}

#[derive(serde::Serialize)]
struct CppSchedContextView {
    index: usize,
    /// `None` = unset; the pack decides that is `nullptr`.
    class: Option<String>,
    period_us: u64,
    budget_us: u64,
    deadline_us: u64,
    deadline_policy: Option<String>,
    os_pri: u8,
}

#[derive(serde::Serialize)]
struct CppNodeBindView {
    name: String,
    namespace: String,
    sched_context: u8,
}

#[derive(serde::Serialize)]
struct CppGroupBindView {
    name: String,
    namespace: String,
    group: String,
    tier_index: usize,
}

#[derive(serde::Serialize)]
struct CppNodeView {
    index: usize,
    /// RAW. The pack quotes it with `c_str`.
    name: String,
    /// `"c"` | `"rust"` | `"rclcpp"` | `"configure"`.
    shape: &'static str,
    pkg: String,
    class: Option<String>,
    /// Whether this body renders inside a per-tier setup function, which is
    /// what selects the executor expression. A fact about WHERE it renders,
    /// not about the node.
    tiered: bool,
    /// The remap + param calls, rendered by the SHARED
    /// `declare_calls.c.jinja` — the same partial the C pack includes.
    decls: DeclsView,
    /// The QoS overrides. NOT shared: this pack calls a method on the node
    /// where C calls a free function on its address.
    qos: Vec<QosRowView>,
}

#[derive(serde::Serialize)]
struct CppProbeView {
    package: String,
    component: String,
    executable: String,
    language: String,
    out_path: String,
}

#[derive(serde::Serialize)]
struct CppBootView {
    /// `"kernel"` | `"app"` | `"host"` — `BootShape`'s single derivation.
    shape: &'static str,
    board_path: &'static str,
    tiers: bool,
    n_tiers: usize,
}

fn boot_shape_str(shape: BootShape) -> &'static str {
    match shape {
        BootShape::Kernel => "kernel",
        BootShape::App => "app",
        BootShape::Host => "host",
    }
}

/// Which of the four construction shapes a node takes.
fn node_shape(n: &super::PlanNode) -> &'static str {
    if is_c_node(n) {
        "c"
    } else if is_rust_node(n) {
        "rust"
    } else if is_rclcpp_node(n) {
        "rclcpp"
    } else {
        "configure"
    }
}

fn node_view(n: &super::PlanNode, i: usize, tiered: bool) -> CppNodeView {
    let exec_expr = if tiered {
        "executor"
    } else {
        "::nros::global_handle()"
    };
    CppNodeView {
        index: i,
        name: n.name.as_deref().unwrap_or(&n.exec).to_string(),
        shape: node_shape(n),
        pkg: sanitize_pkg(&n.pkg),
        class: n.class_name.clone(),
        tiered,
        decls: decls_view(n, exec_expr),
        qos: qos_views(n),
    }
}

/// The block that closes a setup function, from its own template.
///
/// One definition rendered at most twice (the single setup fn, or tier 0),
/// rather than the same C++ text written twice.
fn setup_trailer(plan: &Plan, tiered: bool) -> Result<String, String> {
    #[derive(serde::Serialize)]
    struct TrailerView {
        param_services: bool,
        lifecycle_code: Option<u8>,
        tiered: bool,
    }
    super::render::render(
        "cpp_service_trailer.cpp.jinja",
        &TrailerView {
            param_services: plan.param_services,
            // "none" | "configure" | anything else (i.e. "active").
            lifecycle_code: plan.lifecycle.as_deref().map(|a| match a {
                "none" => 0u8,
                "configure" => 1,
                _ => 2,
            }),
            tiered,
        },
    )
    .map(|s| s.trim_end_matches('\n').to_string())
}

pub fn emit_typed_with_tail(plan: &Plan, tail: &EntryTail<'_>) -> Result<String, String> {
    for n in &plan.nodes {
        if n.class_name.is_none() {
            return Err(format!(
                "typed entry emit: node pkg `{}` exec `{}` is missing class_name (cmake metadata)",
                n.pkg, n.exec
            ));
        }
        if !is_c_node(n) && !is_rust_node(n) && n.class_header.is_none() {
            return Err(format!(
                "typed entry emit: C++ node pkg `{}` exec `{}` is missing class_header \
                 — the typed Entry needs the component's class header (cmake metadata)",
                n.pkg, n.exec
            ));
        }
    }

    // One `#include` per unique C++ component header (first-seen order). C and
    // Rust nodes carry none — their seams are `extern "C"` declarations.
    let mut headers: Vec<String> = Vec::new();
    for n in &plan.nodes {
        if is_c_node(n) || is_rust_node(n) {
            continue;
        }
        let h = n.class_header.as_deref().unwrap().to_string();
        if !headers.contains(&h) {
            headers.push(h);
        }
    }

    let mut c_pkgs: Vec<String> = Vec::new();
    let mut rust_pkgs: Vec<String> = Vec::new();
    for n in &plan.nodes {
        let pkg = sanitize_pkg(&n.pkg);
        if is_c_node(n) {
            if !c_pkgs.contains(&pkg) {
                c_pkgs.push(pkg);
            }
        } else if is_rust_node(n) && !rust_pkgs.contains(&pkg) {
            rust_pkgs.push(pkg);
        }
    }

    // Static per-node storage. Shape-branched (RFC-0044): an rclcpp component
    // OWNS its node, so it gets an arena slot and no `::nros::Node`; a C node
    // keeps its state in its own TU, so it gets no component object; a Rust
    // node self-creates and gets nothing at all.
    let storage: Vec<CppStorageView> = plan
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, n)| !is_rust_node(n))
        .map(|(i, n)| CppStorageView {
            index: i,
            shape: node_shape(n),
            class: if is_c_node(n) {
                None
            } else {
                n.class_name.clone()
            },
        })
        .collect();

    // Phase 269 (W4) / 272 (W2) — sched-context wiring guard.
    let use_tiers = plan
        .resolved_tiers
        .as_ref()
        .is_some_and(|t| !t.is_single_tier());
    // Phase 274.W2/W3, phase-281 W3a/W3 — native and the three embedded boards
    // that declare `run_tiers` take the per-tier-thread path. ThreadX has no
    // `run_tiers`, so it keeps the single-executor sched-context path.
    //
    // RFC-0047 follow-up: a node whose callback groups map to MORE THAN ONE
    // tier cannot be expressed by `run_tiers` — its per-tier setup functions
    // construct whole NODES, so a group-split node silently landed on
    // whichever tier iterated last and BOTH its timers ran at that cadence.
    // Such plans keep the sched-context path, which expresses the split.
    let has_group_split = plan
        .resolved_tiers
        .as_ref()
        .is_some_and(|t| t.has_group_split_node());
    // phase-308 W1 — a metadata probe always takes the single-setup shape.
    // Tiers would be worse than irrelevant: `create_entity` early-returns for
    // entities whose callback group is inactive on the running tier, so a
    // per-tier probe would UNDER-count exactly what the sidecar exists to
    // count.
    let use_run_tiers = !matches!(tail, EntryTail::MetadataProbe(_))
        && use_tiers
        && !has_group_split
        && (!board_is_embedded(&plan.board)
            || board_is_freertos_embedded(&plan.board)
            || board_is_zephyr(&plan.board)
            || board_is_nuttx(&plan.board));

    let mut tiers_view: Option<CppTiersView> = None;
    let mut sched_view: Option<CppSchedView> = None;
    let mut setup_nodes: Vec<CppNodeView> = Vec::new();
    let mut trailer = String::new();

    if use_run_tiers {
        let tiers = plan.resolved_tiers.as_ref().unwrap();

        // #0266 — `time_slice_us` has a per-thread consumer only on the Rust
        // ThreadX arm today; the C++ tier ABI carries no field. Fail loud
        // rather than silently drop a declared value on a C++ bake.
        if let Some(t) = tiers.tiers.iter().find(|t| t.time_slice_us.is_some()) {
            return Err(format!(
                "tier '{}': time_slice_us is not yet supported on the C/C++ codegen \
                 path (#0266) — declare it only on a Rust `nros::main!` ThreadX entry, \
                 or file the C consumer",
                t.name
            ));
        }

        // node_name → tier index, for per-tier node filtering.
        let node_to_tier: std::collections::HashMap<&str, usize> = tiers
            .tiers
            .iter()
            .enumerate()
            .flat_map(|(ti, tier)| {
                tier.members
                    .iter()
                    .map(move |(node_name, _group)| (node_name.as_str(), ti))
            })
            .collect();

        let tier0_trailer = setup_trailer(plan, true)?;
        let setups = tiers
            .tiers
            .iter()
            .enumerate()
            .map(|(ti, tier)| CppTierSetupView {
                index: ti,
                name: tier.name.clone(),
                nodes: plan
                    .nodes
                    .iter()
                    .enumerate()
                    .filter(|(_, n)| {
                        let node_name = n.name.as_deref().unwrap_or(&n.exec);
                        node_to_tier.get(node_name).copied() == Some(ti)
                    })
                    .map(|(i, n)| node_view(n, i, true))
                    .collect(),
                trailer: if ti == 0 {
                    tier0_trailer.clone()
                } else {
                    String::new()
                },
            })
            .collect();

        // Per-tier callback groups, deduped WITHIN each tier and keeping an
        // empty name. `emit_c` dedups ACROSS tiers and drops empties — issue
        // 1172. Preserved verbatim here because W2.3 is byte-for-byte;
        // reconciling the two moves goldens and is its own change.
        let groups_per_tier: Vec<Vec<String>> = tiers
            .tiers
            .iter()
            .map(|tier| {
                let mut seen = std::collections::BTreeSet::new();
                tier.members
                    .iter()
                    .filter_map(|(_, g)| {
                        if seen.insert(g.clone()) {
                            Some(g.clone())
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .collect();

        tiers_view = Some(CppTiersView {
            n: tiers.tiers.len(),
            setups,
            tiers: super::tier_views(tiers, groups_per_tier),
        });
    } else {
        if use_tiers {
            let tiers = plan.resolved_tiers.as_ref().unwrap();
            // RFC-0052 (common backend) — the tier's RTOS-agnostic policy goes
            // through `nros_cpp_create_sched_context_from_policy`, whose body
            // calls `SchedContext::from_tier_policy` — the SAME lowering the
            // Rust runtime's `apply_tier_sched_policy` uses. RAW tier fields
            // only, so a `real_time` tier lowers to the identical Sporadic SC
            // on every language and the mapping cannot drift between codegen
            // paths.
            let contexts = tiers
                .tiers
                .iter()
                .enumerate()
                .map(|(ti, tier)| CppSchedContextView {
                    index: ti,
                    class: tier.class.clone(),
                    period_us: tier.period_us.unwrap_or(0),
                    budget_us: tier.budget_us.unwrap_or(0),
                    deadline_us: tier.deadline_us.unwrap_or(0),
                    deadline_policy: tier.deadline_policy.clone(),
                    os_pri: tier.priority.clamp(0, 255) as u8,
                })
                .collect();

            let node_ns = |name: &str| -> String {
                plan.nodes
                    .iter()
                    .find(|n| n.name.as_deref().unwrap_or(&n.exec) == name)
                    .and_then(|n| n.namespace.as_deref())
                    .unwrap_or("/")
                    .to_string()
            };

            let node_binds = plan
                .nodes
                .iter()
                .filter_map(|n| {
                    n.sched_context.map(|sc| CppNodeBindView {
                        name: n.name.as_deref().unwrap_or(&n.exec).to_string(),
                        namespace: n.namespace.as_deref().unwrap_or("/").to_string(),
                        sched_context: sc,
                    })
                })
                .collect();

            let group_binds = tiers
                .tiers
                .iter()
                .enumerate()
                .flat_map(|(ti, tier)| {
                    tier.members
                        .iter()
                        .map(move |(node_name, group)| (ti, node_name.clone(), group.clone()))
                })
                .map(|(ti, node_name, group)| CppGroupBindView {
                    namespace: node_ns(&node_name),
                    name: node_name,
                    group,
                    tier_index: ti,
                })
                .collect();

            sched_view = Some(CppSchedView {
                n: tiers.tiers.len(),
                contexts,
                node_binds,
                group_binds,
            });
        }

        setup_nodes = plan
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| node_view(n, i, false))
            .collect();
        trailer = setup_trailer(plan, false)?;
    }

    let probe = match tail {
        EntryTail::MetadataProbe(e) => Some(CppProbeView {
            package: e.package.clone(),
            component: e.component.clone(),
            executable: e.executable.clone(),
            language: e.language.clone(),
            out_path: e.out_path.clone(),
        }),
        EntryTail::Board => None,
    };

    // A probe returns before the boot config and the board wrapper: it opens a
    // session against the recording backend, runs setup once, and dumps.
    let (boot_config, boot) = if probe.is_some() {
        (None, None)
    } else {
        (
            Some(boot_config_view(plan)?),
            Some(CppBootView {
                shape: boot_shape_str(boot_shape(&plan.board)),
                board_path: board_cpp_path(&plan.board),
                tiers: use_run_tiers,
                n_tiers: plan
                    .resolved_tiers
                    .as_ref()
                    .map(|t| t.tiers.len())
                    .unwrap_or(0),
            }),
        )
    };

    super::render::render(
        "cpp_entry.cpp.jinja",
        &CppEntryView {
            bringup: plan.bringup.clone(),
            launch: plan.launch_file.display().to_string(),
            board: plan.board.clone(),
            app_main_include: board_is_embedded(&plan.board) && !board_is_zephyr(&plan.board),
            rclcpp_include: plan.nodes.iter().any(is_rclcpp_node),
            headers,
            c_pkgs,
            rust_pkgs,
            storage,
            tiers: tiers_view,
            sched: sched_view,
            setup_nodes,
            trailer,
            probe,
            boot_config,
            boot,
        },
    )
}

fn is_c_node(n: &super::PlanNode) -> bool {
    n.lang.as_deref() == Some("c")
}

/// Phase 257 (W0-B) — a `lang == "rust"` node is installed via the uniform
/// `__nros_component_<pkg>_install` seam onto the shared executor; it self-creates
/// its node (no entry-created `::nros::Node`, no C++ class, no qos-override — D7
/// Option C).
fn is_rust_node(n: &super::PlanNode) -> bool {
    n.lang.as_deref() == Some("rust")
}

/// Phase 242.4 (RFC-0044) — an rclcpp-shape (IS-A-node, construct-with-handle)
/// C++ component: `shape == "rclcpp"` AND not a C node. Everything else (incl.
/// `shape == None` / `"configure"`) keeps the 240.x `configure(Node&)` path.
fn is_rclcpp_node(n: &super::PlanNode) -> bool {
    !is_c_node(n) && n.shape.as_deref() == Some("rclcpp")
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

    /// Typed-emit fixture: each tuple is `(pkg, exec, name, class, header)`.
    /// Defaults to the `configure(Node&)` shape (240.x); use
    /// [`fixture_plan_rclcpp`] for the construct-with-handle shape.
    fn fixture_plan_typed(nodes: &[(&str, &str, &str, &str, &str)]) -> Plan {
        Plan {
            board: "native".into(),
            nodes: nodes
                .iter()
                .map(|(pkg, exec, name, class, header)| PlanNode {
                    pkg: (*pkg).into(),
                    exec: (*exec).into(),
                    name: Some((*name).into()),
                    namespace: None,
                    class_name: Some((*class).into()),
                    class_header: Some((*header).into()),
                    lang: Some("cpp".into()),
                    shape: Some("configure".into()),
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

    /// Phase 242.4 — rclcpp-shape typed fixture: same tuple as
    /// [`fixture_plan_typed`] but `shape == "rclcpp"` (construct-with-handle).
    fn fixture_plan_rclcpp(nodes: &[(&str, &str, &str, &str, &str)]) -> Plan {
        let mut plan = fixture_plan_typed(nodes);
        for n in &mut plan.nodes {
            n.shape = Some("rclcpp".into());
        }
        plan
    }

    /// phase-308 W1 — the probe is the SAME TU an entry would be, minus the
    /// board and plus a dump. That is the point: the per-node construction and
    /// `configure` calls come from one emitter, so the entity count a probe
    /// records cannot drift from what the real entry registers.
    #[test]
    fn metadata_probe_reuses_the_setup_body_and_swaps_the_tail() {
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let export = ProbeExport {
            package: "talker_pkg".into(),
            component: "talker".into(),
            executable: "talker".into(),
            language: "cpp".into(),
            out_path: "/ws/src/talker_pkg/metadata/talker.json".into(),
        };
        let src = emit_typed_probe(&plan, &export).expect("probe emit ok");

        // Same setup body as the entry: header, construction, configure.
        assert!(src.contains("#include \"talker_pkg/Talker.hpp\""), "{src}");
        assert!(src.contains("__nros_entry_setup"), "{src}");
        assert!(src.contains(".configure("), "{src}");

        // Probe tail: dump, with the identity the sidecar is stamped with.
        assert!(
            src.contains("nros_cpp_metadata_dump(\"talker_pkg\", \"talker\", \"talker\", \"cpp\""),
            "{src}"
        );
        assert!(
            src.contains("/ws/src/talker_pkg/metadata/talker.json"),
            "{src}"
        );
        // Explicit registration is what pulls the backend object out of the
        // static archive; with no reference the linker omits it entirely.
        assert!(src.contains("nros_rmw_metadata_register()"), "{src}");

        // NOT an entry: no board CALL, no spin, no boot-config blob. Match the
        // call form — the generated file's header comment mentions
        // `Board::run_components` prose, which a bare substring test flags.
        assert!(
            !src.contains("::run_components("),
            "probe must not spin:\n{src}"
        );
        assert!(!src.contains("NROS_BOOT_CONFIG"), "{src}");
        assert!(
            !src.contains("run_tiers"),
            "probe records every tier:\n{src}"
        );
    }

    #[test]
    fn typed_emit_includes_headers_constructs_and_runs_components() {
        let plan = fixture_plan_typed(&[
            (
                "talker_pkg",
                "talker",
                "talker",
                "talker_pkg::Talker",
                "talker_pkg/Talker.hpp",
            ),
            (
                "listener_pkg",
                "listener",
                "listener",
                "listener_pkg::Listener",
                "listener_pkg/Listener.hpp",
            ),
        ]);
        let src = emit_typed(&plan).expect("typed emit ok");
        // headers included (including boot_config.h for the node-name blob)
        assert!(src.contains("#include <nros/boot_config.h>"));
        assert!(src.contains("#include \"talker_pkg/Talker.hpp\""));
        assert!(src.contains("#include \"listener_pkg/Listener.hpp\""));
        assert!(src.contains("#include <nros/component.hpp>"));
        // static component + node storage
        assert!(src.contains("static ::nros::Node __nros_node_0;"));
        assert!(src.contains("static ::talker_pkg::Talker __nros_comp_0;"));
        assert!(src.contains("static ::listener_pkg::Listener __nros_comp_1;"));
        // setup constructs the node + configures the component
        assert!(src.contains("::nros::create_node(__nros_node_0, \"talker\")"));
        assert!(src.contains("__nros_comp_0.configure(__nros_node_0)"));
        assert!(src.contains("__nros_comp_1.configure(__nros_node_1)"));
        // routes to the real executor via the named overload (phase 266)
        assert!(src.contains(
            "::nros::board::LinuxBoard::run_components(nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
        ));
        assert!(!src.contains("__nros_component_"));
        assert!(!src.contains("NodeContext"));
        // configure shape: no construct-with-handle artifacts.
        assert!(!src.contains("global_handle()"));
        assert!(!src.contains("__nros_comp_buf_"));
        // multi-node: boot config must be all-unset (no single node name baked)
        assert!(src.contains(".set_flags  = 0,"));
        assert!(!src.contains("NROS_BOOT_SET_NODE_NAME"));
    }

    // Phase 305 W3 (issue 0255) — launch `<remap>` rules bake as per-pair
    // `nros_cpp_declare_remap` calls BEFORE construction/configure (rclcpp
    // ctors register entities immediately; configure-shape registers there).
    #[test]
    fn typed_emit_remaps_declared_before_configure() {
        let mut plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        plan.nodes[0].remaps = vec![("chatter".into(), "chatter_remapped".into())];
        let src = emit_typed(&plan).expect("typed emit ok");
        assert!(
            src.contains(
                "nros_cpp_declare_remap(::nros::global_handle(), \"talker\", \"/\", \"chatter\", \"chatter_remapped\")"
            ),
            "expected declare_remap call; src:\n{src}"
        );
        let remap_at = src.find("nros_cpp_declare_remap").unwrap();
        let cfg_at = src.find(".configure(__nros_node_0)").unwrap();
        assert!(remap_at < cfg_at, "remap decl must precede configure");
    }

    #[test]
    fn typed_emit_no_remaps_no_declare_calls() {
        // Guard: remap-free plans produce byte-identical output.
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let src = emit_typed(&plan).expect("typed emit ok");
        assert!(!src.contains("nros_cpp_declare_remap"));
    }

    /// Phase 211.H (issue #52) — a configure-shape node carrying qos_overrides
    /// emits the static `nros_cpp_qos_override_t[]` table + a `set_qos_overrides`
    /// call BEFORE `configure`, with the role/policy/value mapped to C-ABI codes.
    #[test]
    fn typed_emit_bakes_qos_overrides_before_configure() {
        let mut plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        // Built through the shared lowering, so the test cannot assert codes
        // the real bake would never produce.
        plan.nodes[0].qos_overrides = nros_orchestration_ir::qos_override::lower_all([
            (
                "qos_overrides./chatter.publisher.reliability",
                "best_effort",
            ),
            (
                "qos_overrides./chatter.subscription.durability",
                "transient_local",
            ),
        ])
        .expect("fixture overrides lower");
        let src = emit_typed(&plan).expect("typed emit ok");

        // Static table with the two overrides, C-ABI codes:
        //   publisher(0)/reliability(0)/best_effort(0); subscription(1)/durability(1)/transient_local(1)
        assert!(src.contains("static const ::nros_cpp_qos_override_t __nros_qos_0[] = {"));
        assert!(src.contains("{ \"/chatter\", 0, 0, 0 }"));
        assert!(src.contains("{ \"/chatter\", 1, 1, 1 }"));
        // Installed on the node, and BEFORE configure.
        assert!(src.contains("__nros_node_0.set_qos_overrides(__nros_qos_0, 2)"));
        let set_at = src.find("set_qos_overrides").unwrap();
        let cfg_at = src.find("__nros_comp_0.configure(__nros_node_0)").unwrap();
        assert!(set_at < cfg_at, "set_qos_overrides must precede configure");
    }

    /// A node with no qos_overrides emits no table / set call.
    #[test]
    fn typed_emit_no_qos_overrides_no_table() {
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let src = emit_typed(&plan).expect("typed emit ok");
        assert!(!src.contains("nros_cpp_qos_override_t"));
        assert!(!src.contains("set_qos_overrides"));
    }

    #[test]
    fn typed_emit_rclcpp_shape_constructs_with_handle() {
        // Phase 242.4 (RFC-0044) — an rclcpp-shape component OWNS its node: the
        // entry placement-news it with the executor handle *after* init, then
        // checks ok(); there is no separate `create_node` / `configure`.
        let plan = fixture_plan_rclcpp(&[(
            "ctrl_pkg",
            "controller",
            "controller",
            "ctrl_pkg::Controller",
            "ctrl_pkg/Controller.hpp",
        )]);
        let src = emit_typed(&plan).expect("rclcpp emit ok");
        // construct-with-handle headers + arena slot
        assert!(src.contains("#include <nros/component_node.hpp>"));
        assert!(src.contains("#include \"ctrl_pkg/Controller.hpp\""));
        assert!(src.contains(
            "alignas(::ctrl_pkg::Controller) static unsigned char __nros_comp_buf_0[sizeof(::ctrl_pkg::Controller)];"
        ));
        assert!(src.contains("static ::ctrl_pkg::Controller* __nros_comp_0 = nullptr;"));
        // setup: handle → placement-new → ok() check naming the node
        assert!(src.contains("::nros::NodeHandle __h(::nros::global_handle());"));
        assert!(
            src.contains("__nros_comp_0 = new (__nros_comp_buf_0) ::ctrl_pkg::Controller(__h);")
        );
        assert!(src.contains("if (!__nros_comp_0->ok()) {"));
        assert!(src.contains("report_component_failure(\"controller\""));
        // The rclcpp shape does NOT default-construct a Node or call configure.
        assert!(!src.contains("static ::nros::Node __nros_node_0;"));
        assert!(!src.contains("__nros_comp_0.configure"));
        assert!(!src.contains("create_node(__nros_node_0"));
        // still routes to the real executor via the named overload (phase 266)
        assert!(src.contains(
            "::nros::board::LinuxBoard::run_components(nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
        ));
    }

    #[test]
    fn typed_emit_mixed_rclcpp_and_configure_shapes() {
        // One rclcpp node + one configure node in the same entry: each constructs
        // its own way; the includes carry both seams.
        let mut plan = fixture_plan_typed(&[
            (
                "ctrl_pkg",
                "controller",
                "controller",
                "ctrl_pkg::Controller",
                "ctrl_pkg/Controller.hpp",
            ),
            (
                "legacy_pkg",
                "legacy",
                "legacy",
                "legacy_pkg::Legacy",
                "legacy_pkg/Legacy.hpp",
            ),
        ]);
        plan.nodes[0].shape = Some("rclcpp".into());
        // plan.nodes[1] stays "configure".
        let src = emit_typed(&plan).expect("mixed emit ok");
        // node 0 = rclcpp: arena slot + handle construct, no Node/configure.
        assert!(src.contains("static ::ctrl_pkg::Controller* __nros_comp_0 = nullptr;"));
        assert!(
            src.contains("__nros_comp_0 = new (__nros_comp_buf_0) ::ctrl_pkg::Controller(__h);")
        );
        assert!(!src.contains("static ::nros::Node __nros_node_0;"));
        // node 1 = configure: Node + configure, no arena slot.
        assert!(src.contains("static ::nros::Node __nros_node_1;"));
        assert!(src.contains("static ::legacy_pkg::Legacy __nros_comp_1;"));
        assert!(src.contains("__nros_comp_1.configure(__nros_node_1)"));
        assert!(!src.contains("__nros_comp_buf_1"));
        // rclcpp include present because at least one rclcpp node exists.
        assert!(src.contains("#include <nros/component_node.hpp>"));
    }

    #[test]
    fn typed_emit_duplicate_pkg_makes_two_instances_one_include() {
        // Two `<node>` rows of the same pkg → two component objects, one include.
        let plan = fixture_plan_typed(&[
            ("twin_pkg", "a", "a", "twin_pkg::Twin", "twin_pkg/Twin.hpp"),
            ("twin_pkg", "b", "b", "twin_pkg::Twin", "twin_pkg/Twin.hpp"),
        ]);
        let src = emit_typed(&plan).expect("typed emit ok");
        assert_eq!(src.matches("#include \"twin_pkg/Twin.hpp\"").count(), 1);
        assert!(src.contains("static ::twin_pkg::Twin __nros_comp_0;"));
        assert!(src.contains("static ::twin_pkg::Twin __nros_comp_1;"));
        assert!(src.contains("::nros::create_node(__nros_node_0, \"a\")"));
        assert!(src.contains("::nros::create_node(__nros_node_1, \"b\")"));
    }

    #[test]
    fn typed_emit_c_node_uses_factory_configure_seam() {
        // A `lang == "c"` node routes through the C-ABI factory + configure seam
        // (no C++ class, no header include); the entry hands it `ffi_handle()`.
        let mut plan = fixture_plan_typed(&[(
            "sensor_pkg",
            "sensor",
            "sensor",
            "sensor_pkg::Sensor",
            "sensor_pkg/Sensor.hpp",
        )]);
        plan.nodes[0].lang = Some("c".into());
        let src = emit_typed(&plan).expect("typed emit ok");
        // extern "C" factory + configure decls, mangled on pkg.
        assert!(src.contains("void* __nros_c_component_sensor_pkg_create(void);"));
        assert!(src.contains(
            "int32_t __nros_c_component_sensor_pkg_configure(const ::nros_cpp_node_t* node, void* executor, void* self);"
        ));
        // setup uses create() + configure(ffi_handle, executor_handle, self) — not a C++ class.
        assert!(src.contains("void* self = __nros_c_component_sensor_pkg_create();"));
        assert!(src.contains(
            "__nros_c_component_sensor_pkg_configure(__nros_node_0.ffi_handle(), __nros_node_0.executor_handle(), self)"
        ));
        // No C++ class storage / header / .configure for the C node.
        assert!(!src.contains("static ::sensor_pkg::Sensor"));
        assert!(!src.contains("#include \"sensor_pkg/Sensor.hpp\""));
        assert!(!src.contains("__nros_comp_0.configure"));
        // Still routes to the real executor via the named overload (phase 266).
        assert!(src.contains(
            "::nros::board::LinuxBoard::run_components(nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
        ));
    }

    #[test]
    fn typed_emit_mixed_c_and_cpp_nodes() {
        let mut plan = fixture_plan_typed(&[
            (
                "talker_pkg",
                "talker",
                "talker",
                "talker_pkg::Talker",
                "talker_pkg/Talker.hpp",
            ),
            (
                "sensor_pkg",
                "sensor",
                "sensor",
                "sensor_pkg::Sensor",
                "sensor_pkg/Sensor.hpp",
            ),
        ]);
        plan.nodes[1].lang = Some("c".into()); // sensor is C
        let src = emit_typed(&plan).expect("typed emit ok");
        // C++ node: header + class + .configure.
        assert!(src.contains("#include \"talker_pkg/Talker.hpp\""));
        assert!(src.contains("static ::talker_pkg::Talker __nros_comp_0;"));
        assert!(src.contains("__nros_comp_0.configure(__nros_node_0)"));
        // C node: factory seam, no header/class.
        assert!(src.contains("void* self = __nros_c_component_sensor_pkg_create();"));
        assert!(!src.contains("static ::sensor_pkg::Sensor"));
    }

    #[test]
    fn typed_emit_nuttx_board_uses_nuttxboard_run_components() {
        // Phase 266: embedded boards use the 3-arg (locator, session_name, setup) overload.
        let mut plan = fixture_plan_typed(&[("t_pkg", "t", "t", "t_pkg::T", "t_pkg/T.hpp")]);
        plan.board = "nuttx".into();
        let src = emit_typed(&plan).expect("typed emit ok");
        assert!(src.contains(
            "::nros::board::NuttxBoard::run_components(NROS_ENTRY_LOCATOR, nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
        ));
    }

    #[test]
    fn typed_emit_threadx_board_uses_threadxboard_run_components() {
        // Phase 246 — the ThreadX family keys (host sim + bare-metal riscv64) all
        // route the typed entry to the `ThreadxBoard` adapter's `run_components`.
        // Phase 266: uses the 3-arg (locator, session_name, setup) named overload.
        for key in [
            "threadx",
            "threadx-linux",
            "threadx-qemu-riscv64",
            "qemu-riscv64-threadx",
        ] {
            let mut plan = fixture_plan_typed(&[("t_pkg", "t", "t", "t_pkg::T", "t_pkg/T.hpp")]);
            plan.board = key.into();
            let src = emit_typed(&plan).expect("typed emit ok");
            assert!(
                src.contains(
                    "::nros::board::ThreadxBoard::run_components(NROS_ENTRY_LOCATOR, nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
                ),
                "board key {key} must map to ThreadxBoard::run_components with named overload"
            );
        }
    }

    #[test]
    fn typed_emit_native_single_node_bakes_name_in_boot_config() {
        // Phase 266 — single-node native entry: boot config carries the node name.
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let src = emit_typed(&plan).expect("typed emit ok");
        assert!(src.contains("#include <nros/boot_config.h>"));
        assert!(src.contains("NROS_BOOT_SET_NODE_NAME"));
        assert!(src.contains(".node_name  = \"talker\""));
        assert!(src.contains(
            "::nros::board::LinuxBoard::run_components(nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
        ));
    }

    #[test]
    fn typed_emit_errors_when_class_missing() {
        let plan = fixture_plan(&[("talker_pkg", "talker")]); // class_name None
        let err = emit_typed(&plan).unwrap_err();
        assert!(err.contains("missing class_name"), "{err}");
        assert!(err.contains("talker_pkg"), "{err}");
    }

    #[test]
    fn typed_emit_param_services_block_present_when_enabled() {
        // Phase 269 W1 (amended by issue 0745) — param SEEDING now emits per
        // node BEFORE construction (emit_declare_params, the 0255 remap rule:
        // an rclcpp ctor reads declare_parameter initials immediately);
        // param_services gates only the runtime get/set surface.
        let mut plan = fixture_plan_typed(&[(
            "param_talker_pkg",
            "param_talker",
            "param_talker",
            "param_talker_pkg::ParamTalker",
            "param_talker_pkg/ParamTalker.hpp",
        )]);
        plan.param_services = true;
        plan.nodes[0].params = vec![("publish_period_ms".into(), "250".into())];
        let src = emit_typed(&plan).expect("typed cpp emit ok");
        assert!(src.contains("nros_cpp_register_parameter_services(__exec)"));
        assert!(src.contains(
            "nros_cpp_declare_param(::nros::global_handle(), \"publish_period_ms\", \"250\")"
        ));
        // issue 0745 — seeding precedes construction.
        let seed_at = src.find("nros_cpp_declare_param").unwrap();
        let construct_at = src
            .find(".configure(")
            .or_else(|| src.find("new ("))
            .unwrap();
        assert!(
            seed_at < construct_at,
            "param seeding must precede construction"
        );
        // must appear after configure, before return 0
        let reg_at = src.find("nros_cpp_register_parameter_services").unwrap();
        let ret_at = src.rfind("return 0;").unwrap();
        assert!(reg_at < ret_at, "param block must precede return 0");
        // confirms executor handle fetched from global
        assert!(src.contains("::nros::global_handle()"));
    }

    #[test]
    fn typed_emit_param_services_absent_when_disabled() {
        // Guard: non-param plans produce byte-identical output (no param block).
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let src = emit_typed(&plan).expect("typed cpp emit ok");
        assert!(!src.contains("nros_cpp_register_parameter_services"));
        assert!(!src.contains("nros_cpp_declare_param"));
    }

    #[test]
    fn typed_emit_lifecycle_active_emits_autostart_block() {
        // Phase 269 W2 — lifecycle = Some("active") → nros_cpp_lifecycle_autostart(__exec, 2u)
        // in the post-configure block, AFTER any param block, BEFORE return 0.
        let mut plan = fixture_plan_typed(&[(
            "lifecycle_talker_pkg",
            "lifecycle_talker",
            "lifecycle_talker",
            "lifecycle_talker_pkg::LifecycleTalker",
            "lifecycle_talker_pkg/LifecycleTalker.hpp",
        )]);
        plan.lifecycle = Some("active".into());
        let src = emit_typed(&plan).expect("typed cpp lifecycle emit ok");
        // autostart call with code 2 (active = configure + activate)
        assert!(
            src.contains("nros_cpp_lifecycle_autostart(__exec, 2u)"),
            "expected nros_cpp_lifecycle_autostart(__exec, 2u) in:\n{src}"
        );
        // executor handle from global_handle
        assert!(src.contains("::nros::global_handle()"));
        // AFTER configure loop (configure call or C factory), BEFORE return 0
        let autostart_at = src.find("nros_cpp_lifecycle_autostart").unwrap();
        let ret_at = src.rfind("return 0;").unwrap();
        assert!(
            autostart_at < ret_at,
            "lifecycle block must precede return 0"
        );
        // configure call precedes the lifecycle block
        let cfg_at = src.find("__nros_comp_0.configure(__nros_node_0)").unwrap();
        assert!(
            cfg_at < autostart_at,
            "lifecycle block must follow configure call"
        );
    }

    #[test]
    fn typed_emit_lifecycle_configure_emits_code_1() {
        let mut plan = fixture_plan_typed(&[("lc_pkg", "lc", "lc", "lc_pkg::Lc", "lc_pkg/Lc.hpp")]);
        plan.lifecycle = Some("configure".into());
        let src = emit_typed(&plan).expect("typed cpp lifecycle configure emit ok");
        assert!(
            src.contains("nros_cpp_lifecycle_autostart(__exec, 1u)"),
            "expected autostart_code 1 for 'configure'; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_lifecycle_none_emits_code_0() {
        let mut plan = fixture_plan_typed(&[("lc_pkg", "lc", "lc", "lc_pkg::Lc", "lc_pkg/Lc.hpp")]);
        plan.lifecycle = Some("none".into());
        let src = emit_typed(&plan).expect("typed cpp lifecycle none emit ok");
        assert!(
            src.contains("nros_cpp_lifecycle_autostart(__exec, 0u)"),
            "expected autostart_code 0 for 'none'; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_lifecycle_absent_when_disabled() {
        // Guard: lifecycle = None → byte-identical output (no lifecycle block).
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let src = emit_typed(&plan).expect("typed cpp emit ok");
        assert!(
            !src.contains("nros_cpp_lifecycle_autostart"),
            "lifecycle block must be absent when lifecycle = None"
        );
    }

    #[test]
    fn typed_emit_lifecycle_after_param_block() {
        // Phase 269 W2 — when both param_services and lifecycle are set, the lifecycle
        // block must appear AFTER the param block (same order as the Rust macro: params → lifecycle).
        let mut plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        plan.param_services = true;
        plan.nodes[0].params = vec![("foo".into(), "bar".into())];
        plan.lifecycle = Some("active".into());
        let src = emit_typed(&plan).expect("typed cpp combined emit ok");
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
        use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
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
        let mut plan = fixture_plan_typed(&[
            (
                "ctrl_pkg",
                "ctrl",
                "ctrl",
                "ctrl_pkg::Ctrl",
                "ctrl_pkg/Ctrl.hpp",
            ),
            (
                "telem_pkg",
                "telem",
                "telem",
                "telem_pkg::Telem",
                "telem_pkg/Telem.hpp",
            ),
        ]);
        plan.nodes[0].callback_groups = vec!["ctrl_grp".into()];
        plan.nodes[0].sched_context = Some(0);
        plan.nodes[1].callback_groups = vec!["telem_grp".into()];
        plan.nodes[1].sched_context = Some(1);
        plan.resolved_tiers = Some(ResolvedTierTable {
            tiers: vec![high_tier, low_tier],
        });
        plan
    }

    #[test]
    fn typed_emit_group_split_node_falls_back_to_sched_context_path() {
        // Phase 282 follow-up (RFC-0047) — ONE node with callback groups on TWO
        // tiers (`group_tiers = { ctrl = "high", telem = "low" }`) cannot use
        // run_tiers: per-tier setup fns construct whole nodes, so the node
        // landed on the last tier and both timers ran at that cadence
        // (regression caught by realtime_subnode_cpp_e2e: ctrl=6 telem=5).
        // Such plans must keep the single-executor sched-context path.
        use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
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
            members: vec![("sub_node".into(), "ctrl".into())],
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
            members: vec![("sub_node".into(), "telem".into())],
        };
        let mut plan = fixture_plan_typed(&[(
            "subnode_pkg",
            "sub_node",
            "sub_node",
            "subnode_pkg::SubNode",
            "subnode_pkg/SubNode.hpp",
        )]);
        plan.nodes[0].callback_groups = vec!["ctrl".into(), "telem".into()];
        plan.resolved_tiers = Some(ResolvedTierTable {
            tiers: vec![high_tier, low_tier],
        });
        let src = emit_typed(&plan).expect("typed cpp group-split emit ok");

        // Sched-context path: per-group seeding present, run_tiers absent.
        assert!(
            src.contains("nros_cpp_bind_group_sched"),
            "group-split node must seed bind_group_sched; src:\n{src}"
        );
        assert!(
            src.contains("\"ctrl\"") && src.contains("\"telem\""),
            "both groups must be seeded; src:\n{src}"
        );
        assert!(
            !src.contains("__nros_entry_setup_tier_0"),
            "group-split plan must NOT use the run_tiers path; src:\n{src}"
        );
        assert!(
            !src.contains("run_tiers("),
            "group-split plan must NOT call run_tiers; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_tiers_native_uses_run_tiers_path() {
        // Phase 274.W2 — native board + multi-tier emits per-tier setup functions +
        // run_tiers call instead of the old sched-context wiring.
        let plan = fixture_plan_with_tiers();
        let src = emit_typed(&plan).expect("typed cpp tier emit ok");

        // Per-tier setup functions emitted.
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_0(void* executor)"),
            "expected tier-0 setup fn; got:\n{src}"
        );
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_1(void* executor)"),
            "expected tier-1 setup fn; got:\n{src}"
        );
        // Each setup fn creates only its tier's nodes via create_node_on.
        assert!(
            src.contains("::nros::create_node_on(__nros_node_0, executor, \"ctrl\")"),
            "ctrl node must use create_node_on in tier-0 setup; src:\n{src}"
        );
        assert!(
            src.contains("::nros::create_node_on(__nros_node_1, executor, \"telem\")"),
            "telem node must use create_node_on in tier-1 setup; src:\n{src}"
        );
        // NativeTierSpec array emitted.
        assert!(
            src.contains("static const ::nros::board::NativeTierSpec __nros_tiers[2]"),
            "expected 2-element NativeTierSpec array; src:\n{src}"
        );
        assert!(
            src.contains("\"high\""),
            "high tier name in spec table; src:\n{src}"
        );
        assert!(
            src.contains("\"low\""),
            "low tier name in spec table; src:\n{src}"
        );
        assert!(src.contains("80LL"), "high priority 80LL; src:\n{src}");
        assert!(src.contains("10LL"), "low priority 10LL; src:\n{src}");
        // main calls run_tiers.
        assert!(
            src.contains("::nros::board::LinuxBoard::run_tiers("),
            "main must call LinuxBoard::run_tiers; src:\n{src}"
        );
        // Old sched-context wiring must NOT appear in the run_tiers path.
        assert!(
            !src.contains("__nros_sc_ids"),
            "run_tiers path must not emit sc_ids; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_create_sched_context"),
            "run_tiers path must not emit create_sched_context; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_bind_node_name_sched"),
            "run_tiers path must not emit bind_node_name_sched; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_tiers_embedded_uses_sched_context_path() {
        // Phase 272/273 (W2) — a sched-context embedded board (ThreadX) + multi-tier
        // still uses sched-context wiring (bind_node_name_sched + bind_group_sched) because
        // run_tiers is limited to native + FreeRTOS + Zephyr + NuttX (phase-281 W3/W3a).
        // ThreadX keeps board_is_embedded=true && !run_tiers → the single-executor
        // sched-context path.
        use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
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
        let mut plan = fixture_plan_typed(&[
            (
                "ctrl_pkg",
                "ctrl",
                "ctrl",
                "ctrl_pkg::Ctrl",
                "ctrl_pkg/Ctrl.hpp",
            ),
            (
                "telem_pkg",
                "telem",
                "telem",
                "telem_pkg::Telem",
                "telem_pkg/Telem.hpp",
            ),
        ]);
        // Sched-context embedded board (ThreadX) → sched-context path (NOT run_tiers).
        plan.board = "threadx".into();
        plan.nodes[0].callback_groups = vec!["ctrl_grp".into()];
        plan.nodes[0].sched_context = Some(0);
        plan.nodes[1].callback_groups = vec!["telem_grp".into()];
        plan.nodes[1].sched_context = Some(1);
        plan.resolved_tiers = Some(ResolvedTierTable {
            tiers: vec![high_tier, low_tier],
        });
        let src = emit_typed(&plan).expect("typed cpp embedded tier emit ok");
        // Sched-context IDs array declared.
        assert!(
            src.contains("uint8_t __nros_sc_ids[2] = {0};"),
            "embedded tier must emit sc_ids array; got:\n{src}"
        );
        // High tier: no RT class → Fifo SC via the common-backend call, carrying
        // only os_pri=80 (nullptr/0 = absent policy). RFC-0052: the codegen
        // forwards RAW tier fields; the lowering lives in the FFI backend.
        assert!(
            src.contains(
                "nros_cpp_create_sched_context_from_policy(__exec, nullptr, 0ull, 0ull, 0ull, nullptr, 80u, &__nros_sc_ids[0])"
            ),
            "expected tier 0 from_policy call (Fifo, os_pri=80); got:\n{src}"
        );
        // Bind seeds for each tiered node.
        assert!(
            src.contains(
                "nros_cpp_bind_node_name_sched(__exec, \"ctrl\", \"/\", __nros_sc_ids[0])"
            ),
            "ctrl must be seeded; src:\n{src}"
        );
        assert!(
            src.contains(
                "nros_cpp_bind_node_name_sched(__exec, \"telem\", \"/\", __nros_sc_ids[1])"
            ),
            "telem must be seeded; src:\n{src}"
        );
        // run_tiers must NOT be called (embedded boards use single executor).
        assert!(
            !src.contains("LinuxBoard::run_tiers"),
            "embedded board must not emit run_tiers; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_single_executor_forwards_real_time_tier_to_backend() {
        // Phase 297 W1 / RFC-0052 (common backend) — the single-executor
        // sched-context path (ThreadX + group-split) forwards a `real_time`
        // tier's RAW class/budget/period/deadline to
        // `nros_cpp_create_sched_context_from_policy`, whose backend
        // (`SchedContext::from_tier_policy`) does the class→Sporadic lowering —
        // the SAME one the Rust runtime uses. The codegen re-derives nothing.
        use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
        let rt_tier = ResolvedTier {
            name: "control".into(),
            priority: 90,
            stack_bytes: None,
            spin_period_us: Some(5_000),
            preempt_threshold: None,
            time_slice_us: None,
            sched_class: None,
            class: Some("real_time".into()),
            period_us: Some(20_000),
            budget_us: Some(3_000),
            deadline_us: Some(15_000),
            deadline_policy: Some("fault".into()),
            core: None,
            members: vec![("ctrl".into(), "ctrl_grp".into())],
        };
        let mut plan = fixture_plan_typed(&[(
            "ctrl_pkg",
            "ctrl",
            "ctrl",
            "ctrl_pkg::Ctrl",
            "ctrl_pkg/Ctrl.hpp",
        )]);
        plan.board = "threadx".into();
        plan.nodes[0].callback_groups = vec!["ctrl_grp".into()];
        plan.nodes[0].sched_context = Some(0);
        plan.resolved_tiers = Some(ResolvedTierTable {
            tiers: vec![rt_tier],
        });
        let src = emit_typed(&plan).expect("typed cpp real_time tier emit ok");
        // RFC-0052 common backend: the codegen forwards the RAW tier fields to
        // `nros_cpp_create_sched_context_from_policy`; the class→Sporadic +
        // budget/period lowering happens in the FFI backend
        // (`SchedContext::from_tier_policy`), unit-tested in nros-node. The
        // codegen must NOT re-derive the mapping (no `__sc.class_ = ...`).
        assert!(
            src.contains(
                "nros_cpp_create_sched_context_from_policy(__exec, \"real_time\", 20000ull, 3000ull, 15000ull, \"fault\", 90u, &__nros_sc_ids[0])"
            ),
            "real_time tier must forward raw fields to the backend; got:\n{src}"
        );
        assert!(
            !src.contains("__sc.class_"),
            "codegen must not re-derive the class mapping (common backend); got:\n{src}"
        );
    }

    #[test]
    fn typed_emit_tiers_freertos_embedded_uses_run_tiers_path() {
        // Phase 274.W3 — FreeRTOS embedded board + multi-tier emits per-tier setup
        // functions + FreertosBoard::run_tiers via nros_app_main +
        // NROS_APP_MAIN_REGISTER_VOID (NOT the sched-context path, NOT int main).
        let mut plan = fixture_plan_with_tiers();
        plan.board = "freertos".into(); // FreertosBoard

        let src = emit_typed(&plan).expect("typed cpp freertos tier emit ok");

        // Per-tier setup functions emitted.
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_0(void* executor)"),
            "expected tier-0 setup fn; got:\n{src}"
        );
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_1(void* executor)"),
            "expected tier-1 setup fn; got:\n{src}"
        );
        // NativeTierSpec array emitted.
        assert!(
            src.contains("static const ::nros::board::NativeTierSpec __nros_tiers[2]"),
            "expected 2-element NativeTierSpec array; src:\n{src}"
        );
        // FreertosBoard::run_tiers called (not LinuxBoard).
        assert!(
            src.contains("::nros::board::FreertosBoard::run_tiers("),
            "nros_app_main must call FreertosBoard::run_tiers; src:\n{src}"
        );
        // FreeRTOS embedded entry point: nros_app_main + NROS_APP_MAIN_REGISTER_VOID.
        assert!(
            src.contains("extern \"C\" int nros_app_main("),
            "FreeRTOS run_tiers must emit nros_app_main; src:\n{src}"
        );
        assert!(
            src.contains("NROS_APP_MAIN_REGISTER_VOID()"),
            "FreeRTOS run_tiers must emit NROS_APP_MAIN_REGISTER_VOID; src:\n{src}"
        );
        // NOT int main (that's native).
        assert!(
            !src.contains("int main("),
            "FreeRTOS run_tiers must NOT emit int main; src:\n{src}"
        );
        // Old sched-context wiring must NOT appear.
        assert!(
            !src.contains("__nros_sc_ids"),
            "FreeRTOS run_tiers path must not emit sc_ids; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_create_sched_context"),
            "FreeRTOS run_tiers path must not emit create_sched_context; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_tiers_zephyr_embedded_uses_run_tiers_path() {
        // phase-281 W3a — Zephyr embedded board + multi-tier emits per-tier setup
        // functions + ZephyrBoard::run_tiers via a plain `int main(void)` (the Zephyr
        // kernel calls main directly — NO nros_app_main, NO sched-context path).
        let mut plan = fixture_plan_with_tiers();
        plan.board = "zephyr".into(); // ZephyrBoard

        let src = emit_typed(&plan).expect("typed cpp zephyr tier emit ok");

        // Per-tier setup functions emitted.
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_0(void* executor)"),
            "expected tier-0 setup fn; got:\n{src}"
        );
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_1(void* executor)"),
            "expected tier-1 setup fn; got:\n{src}"
        );
        // NativeTierSpec array emitted.
        assert!(
            src.contains("static const ::nros::board::NativeTierSpec __nros_tiers[2]"),
            "expected 2-element NativeTierSpec array; src:\n{src}"
        );
        // ZephyrBoard::run_tiers called (not LinuxBoard / FreertosBoard).
        assert!(
            src.contains("::nros::board::ZephyrBoard::run_tiers("),
            "main must call ZephyrBoard::run_tiers; src:\n{src}"
        );
        // Zephyr entry point: plain int main(void), kernel calls it directly.
        assert!(
            src.contains("int main(void) {"),
            "Zephyr run_tiers must emit int main(void); src:\n{src}"
        );
        // NOT the FreeRTOS/startup.c app_main shape.
        assert!(
            !src.contains("nros_app_main"),
            "Zephyr run_tiers must NOT emit nros_app_main; src:\n{src}"
        );
        assert!(
            !src.contains("NROS_APP_MAIN_REGISTER_VOID"),
            "Zephyr run_tiers must NOT emit NROS_APP_MAIN_REGISTER_VOID; src:\n{src}"
        );
        // Old sched-context wiring must NOT appear (this is the run_tiers path).
        assert!(
            !src.contains("__nros_sc_ids"),
            "Zephyr run_tiers path must not emit sc_ids; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_create_sched_context"),
            "Zephyr run_tiers path must not emit create_sched_context; src:\n{src}"
        );
        // run_tiers path must not CALL run_components (the string appears once in the
        // file-header doc comment, so assert on the call form specifically).
        assert!(
            !src.contains("ZephyrBoard::run_components"),
            "Zephyr run_tiers path must not call ZephyrBoard::run_components; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_tiers_nuttx_embedded_uses_run_tiers_path() {
        // phase-281 W3 (nuttx) — NuttX embedded board + multi-tier emits per-tier
        // setup functions + NuttxBoard::run_tiers via nros_app_main +
        // NROS_APP_MAIN_REGISTER_VOID (the NuttX startup path calls app_main, like
        // FreeRTOS — NOT Zephyr's int main(void), NOT the sched-context path).
        let mut plan = fixture_plan_with_tiers();
        plan.board = "nuttx".into(); // NuttxBoard

        let src = emit_typed(&plan).expect("typed cpp nuttx tier emit ok");

        // Per-tier setup functions emitted.
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_0(void* executor)"),
            "expected tier-0 setup fn; got:\n{src}"
        );
        assert!(
            src.contains("static int32_t __nros_entry_setup_tier_1(void* executor)"),
            "expected tier-1 setup fn; got:\n{src}"
        );
        // NativeTierSpec array emitted.
        assert!(
            src.contains("static const ::nros::board::NativeTierSpec __nros_tiers[2]"),
            "expected 2-element NativeTierSpec array; src:\n{src}"
        );
        // NuttxBoard::run_tiers called (not LinuxBoard / FreertosBoard / ZephyrBoard).
        assert!(
            src.contains("::nros::board::NuttxBoard::run_tiers("),
            "nros_app_main must call NuttxBoard::run_tiers; src:\n{src}"
        );
        // NuttX embedded entry point: nros_app_main + NROS_APP_MAIN_REGISTER_VOID
        // (the app_main startup shape, shared with FreeRTOS).
        assert!(
            src.contains("extern \"C\" int nros_app_main("),
            "NuttX run_tiers must emit nros_app_main; src:\n{src}"
        );
        assert!(
            src.contains("NROS_APP_MAIN_REGISTER_VOID()"),
            "NuttX run_tiers must emit NROS_APP_MAIN_REGISTER_VOID; src:\n{src}"
        );
        // NOT int main (that's native) and NOT the Zephyr int main(void).
        assert!(
            !src.contains("int main("),
            "NuttX run_tiers must NOT emit int main; src:\n{src}"
        );
        // Old sched-context wiring must NOT appear (this is the run_tiers path).
        assert!(
            !src.contains("__nros_sc_ids"),
            "NuttX run_tiers path must not emit sc_ids; src:\n{src}"
        );
        assert!(
            !src.contains("nros_cpp_create_sched_context"),
            "NuttX run_tiers path must not emit create_sched_context; src:\n{src}"
        );
    }

    #[test]
    fn typed_emit_tiers_rclcpp_embedded_node_is_seeded() {
        // Phase 272 (W2) — rclcpp-shape tiered node on an embedded board IS seeded
        // via bind_node_name_sched (the #124 dissolve). Native boards use run_tiers
        // instead; this test covers the embedded (sched-context) path.
        use nros_orchestration_ir::{ResolvedTier, ResolvedTierTable};
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
        let mut plan = fixture_plan_rclcpp(&[(
            "ctrl_pkg",
            "ctrl",
            "ctrl",
            "ctrl_pkg::Ctrl",
            "ctrl_pkg/Ctrl.hpp",
        )]);
        // ThreadX — a sched-context embedded board (phase-281 W3a moved Zephyr and
        // W3(nuttx) moved NuttX onto the run_tiers path, so this seeding proof now
        // uses a board that still schedules via the single-executor sched-context wiring).
        plan.board = "threadx".into();
        plan.nodes[0].callback_groups = vec!["ctrl_grp".into()];
        plan.nodes[0].sched_context = Some(0);
        plan.resolved_tiers = Some(ResolvedTierTable {
            tiers: vec![high_tier],
        });
        let src = emit_typed(&plan).expect("rclcpp embedded tier emit ok");
        // rclcpp-shape node MUST be seeded (the #124 proof, embedded path).
        assert!(
            src.contains(
                "nros_cpp_bind_node_name_sched(__exec, \"ctrl\", \"/\", __nros_sc_ids[0])"
            ),
            "rclcpp-shape tiered node must be seeded via bind_node_name_sched; src:\n{src}"
        );
        // rclcpp construction path unchanged (placement-new with handle).
        assert!(
            src.contains("__nros_comp_0 = new (__nros_comp_buf_0) ::ctrl_pkg::Ctrl(__h);"),
            "rclcpp node still constructs via placement-new"
        );
        // Seed precedes construction.
        let seed_at = src
            .find("nros_cpp_bind_node_name_sched(__exec, \"ctrl\"")
            .unwrap();
        let ctor_at = src.find("new (__nros_comp_buf_0)").unwrap();
        assert!(seed_at < ctor_at, "seed must precede rclcpp construction");
    }

    #[test]
    fn typed_emit_no_tiers_uses_plain_create_node() {
        // Guard: empty resolved_tiers keeps byte-identical plain create (no seed, no sched).
        let plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);
        let src = emit_typed(&plan).expect("typed cpp no-tier emit ok");
        assert!(
            !src.contains("__nros_sc_ids"),
            "no-tier plan must not emit sc_ids"
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
            src.contains("::nros::create_node(__nros_node_0, \"talker\")"),
            "no-tier plan must use plain create_node"
        );
        assert!(
            !src.contains(".sched("),
            "no-tier plan must not use NodeBuilder sched"
        );
    }

    // ---------------------------------------------------------------
    // Issue 1003 — the boot wrapper has ONE derivation
    // ---------------------------------------------------------------

    /// The wrapper each board family gets. Written as a table because the bug
    /// this replaces was a board missing from one of two hand-written branch
    /// chains: a table makes an omission visible as a missing row.
    #[test]
    fn every_board_family_derives_its_boot_shape_once() {
        for (board, want) in [
            ("native", BootShape::Host),
            ("posix", BootShape::Host),
            ("zephyr", BootShape::Kernel),
            ("nuttx", BootShape::App),
            ("freertos", BootShape::App),
            ("threadx", BootShape::App),
            ("threadx-linux", BootShape::App),
        ] {
            assert_eq!(
                boot_shape(board),
                want,
                "board '{board}' derived the wrong boot shape"
            );
        }
    }

    /// ThreadX's `startup.c` owns `main`, so its entry must be `nros_app_main`
    /// — a host `int main` would be a second `main` in an image whose board
    /// already defines one.
    ///
    /// ThreadX is the one board the two former branch chains disagreed about,
    /// and it is kept out of the per-tier chain by `use_run_tiers`. Pinning it
    /// here means the shared derivation is right about it on its own terms,
    /// rather than by an argument about a condition elsewhere.
    #[test]
    fn threadx_is_not_treated_as_a_host_board() {
        assert_ne!(
            boot_shape("threadx"),
            BootShape::Host,
            "ThreadX boots through the board's startup.c, so a host `int main` \
would collide with the one the board defines"
        );
        assert_eq!(boot_shape("threadx"), boot_shape("nuttx"));
    }

    /// The boards that DO have `run_tiers` still emit it, each in its own
    /// wrapper — the consolidation must not have narrowed what works.
    #[test]
    fn boards_with_run_tiers_still_emit_their_own_wrapper() {
        for (board, wrapper) in [
            ("native", "int main(int /*argc*/, char** /*argv*/) {"),
            ("zephyr", "int main(void) {"),
            (
                "nuttx",
                "extern \"C\" int nros_app_main(int /*argc*/, char** /*argv*/) {",
            ),
            (
                "freertos",
                "extern \"C\" int nros_app_main(int /*argc*/, char** /*argv*/) {",
            ),
        ] {
            let mut plan = fixture_plan_with_tiers();
            plan.board = board.into();
            let src =
                emit_typed(&plan).unwrap_or_else(|e| panic!("{board} multi-tier emit failed: {e}"));
            assert!(
                src.contains("::run_tiers("),
                "{board} must still call run_tiers"
            );
            assert!(
                src.contains(wrapper),
                "{board} must be wrapped in `{wrapper}`"
            );
        }
    }

    /// Only the host board resolves its locator at runtime, so it is the one
    /// that passes none. Both halves of that rule now come from one place.
    #[test]
    fn only_the_host_entry_omits_the_locator_argument() {
        let mut plan = fixture_plan_typed(&[(
            "talker_pkg",
            "talker",
            "talker",
            "talker_pkg::Talker",
            "talker_pkg/Talker.hpp",
        )]);

        plan.board = "native".into();
        let host = emit_typed(&plan).expect("native emit ok");
        assert!(
            host.contains("run_components(nros_boot_config_node_name("),
            "the host entry passes no locator: {host}"
        );

        plan.board = "nuttx".into();
        let embedded = emit_typed(&plan).expect("nuttx emit ok");
        assert!(
            embedded.contains("run_components(NROS_ENTRY_LOCATOR, nros_boot_config_node_name("),
            "an embedded entry passes NROS_ENTRY_LOCATOR: {embedded}"
        );
    }
}
