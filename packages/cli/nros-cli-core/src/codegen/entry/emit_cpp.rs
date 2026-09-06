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

use std::fmt::Write;

use super::{
    Plan, QoSOverrideSpec, emit_boot_config_static,
    emit_c::{emit_declare_params, emit_declare_remaps},
    sanitize_pkg,
};

/// Emit a `static const nros_cpp_qos_override_t __nros_qos_<i>[] = {…};` + the
/// `__nros_node_<i>.set_qos_overrides(…)` call for node `i`. No-op when the node
/// has no (recognised) overrides.
fn emit_qos_overrides(out: &mut String, i: usize, overrides: &[QoSOverrideSpec]) {
    // Issue 0303 — the plan already carries CODES: the lowering (and its
    // rejection of anything unusable) happened in
    // `nros_orchestration_ir::qos_override`, so there is nothing to decode or
    // silently skip here.
    if overrides.is_empty() {
        return;
    }
    let _ = writeln!(
        out,
        "        static const ::nros_cpp_qos_override_t __nros_qos_{i}[] = {{"
    );
    for o in overrides {
        let topic = o.topic.replace('\\', "\\\\").replace('"', "\\\"");
        let (role, policy, value) = (o.role, o.policy, o.value);
        let _ = writeln!(
            out,
            "            {{ \"{topic}\", {role}, {policy}, {value} }},"
        );
    }
    out.push_str("        };\n");
    let _ = writeln!(
        out,
        "        __nros_node_{i}.set_qos_overrides(__nros_qos_{i}, {});",
        overrides.len()
    );
}

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

/// Wrap `call` — the board call whose value the entry returns — in the board's
/// boot shape. The `static_cast<int>` belongs to the Zephyr shape only; the
/// other two return the board's `int32_t` directly.
fn emit_boot_wrapper(out: &mut String, shape: BootShape, call: &str) {
    match shape {
        BootShape::Kernel => {
            out.push_str("int main(void) {\n");
            let _ = writeln!(out, "    return static_cast<int>({call});");
            out.push_str("}\n");
        }
        BootShape::App => {
            out.push_str("extern \"C\" int nros_app_main(int /*argc*/, char** /*argv*/) {\n");
            let _ = writeln!(out, "    return {call};");
            out.push_str("}\n\n");
            out.push_str("NROS_APP_MAIN_REGISTER_VOID();\n");
        }
        BootShape::Host => {
            out.push_str("int main(int /*argc*/, char** /*argv*/) {\n");
            let _ = writeln!(out, "    return {call};");
            out.push_str("}\n");
        }
    }
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

    let mut out = String::new();
    let _ = writeln!(
        out,
        "// Generated by `nros codegen entry --lang cpp` (typed — RFC-0043)\n\
         //   bringup = {bringup}\n\
         //   launch  = {launch}\n\
         //   board   = {board}\n\
         //\n\
         // DO NOT EDIT — regenerated at configure time. Routes each launch node\n\
         // to the real executor via its component object (no synthesizing\n\
         // interpreter); `Board::run_components` owns init/spin/shutdown.",
        bringup = plan.bringup,
        launch = plan.launch_file.display(),
        board = plan.board,
    );
    out.push('\n');

    out.push_str("#include <nros/boot_config.h>\n");
    out.push_str("#include <nros/component.hpp>\n");
    out.push_str("#include <nros/main.hpp>\n");
    out.push_str("#include <nros/nros.hpp>\n");
    // phase-263 C2 — embedded boots through the board's startup.c via `app_main`. Zephyr
    // is exempt (the kernel calls `main()` directly — see `board_is_zephyr`).
    if board_is_embedded(&plan.board) && !board_is_zephyr(&plan.board) {
        out.push_str("#include <nros/app_main.h>\n");
    }
    // Phase 242.4 (RFC-0044) — `nros::ComponentNode` / `NodeHandle` /
    // `detail::report_component_failure` for any rclcpp-shape (construct-with-
    // handle) node. Pulled in only when one is present.
    if plan.nodes.iter().any(is_rclcpp_node) {
        out.push_str("#include <new> // placement-new into the component arena slot\n");
        out.push_str("#include <nros/component_node.hpp>\n");
    }
    out.push('\n');

    // One `#include` per unique C++ component header (first-seen order). C nodes
    // carry no header — their factory/configure are extern "C" decls below.
    let mut seen_headers: Vec<&str> = Vec::new();
    for n in &plan.nodes {
        // C nodes carry no header; Rust nodes (phase-257) self-create with no C++
        // class — both skip the include (their seams are extern "C" decls below).
        if is_c_node(n) || is_rust_node(n) {
            continue;
        }
        let h = n.class_header.as_deref().unwrap();
        if !seen_headers.contains(&h) {
            let _ = writeln!(out, "#include \"{h}\"");
            seen_headers.push(h);
        }
    }

    // Forward-declare the C-ABI factory + configure for each unique C pkg.
    let mut seen_c_pkgs: Vec<String> = Vec::new();
    let mut wrote_extern = false;
    for n in &plan.nodes {
        if !is_c_node(n) {
            continue;
        }
        let pkg = sanitize_pkg(&n.pkg);
        if seen_c_pkgs.contains(&pkg) {
            continue;
        }
        if !wrote_extern {
            out.push_str(
                "\n// C component factory + configure seam (NROS_C_COMPONENT); the\n\
                 // node's `ffi_handle()` is handed to the C `configure` as an opaque\n\
                 // `nros_cpp_node_t*` — the C side registers real callbacks on it.\n",
            );
            out.push_str("extern \"C\" {\n");
            wrote_extern = true;
        }
        let _ = writeln!(out, "    void* __nros_c_component_{pkg}_create(void);");
        let _ = writeln!(
            out,
            "    int32_t __nros_c_component_{pkg}_configure(const ::nros_cpp_node_t* node, void* executor, void* self);"
        );
        seen_c_pkgs.push(pkg);
    }
    if wrote_extern {
        out.push_str("}\n");
    }
    out.push('\n');

    // Static per-node storage — one node per launch `<node>` row. Shape-branched
    // (Phase 242.4):
    //  - configure (240.x) C++ node: a static `Node` + a static component object
    //    (default-constructed before init, then `configure(node)` in setup).
    //  - C node: only the static `Node` (its state lives in its own TU — the
    //    factory returns `&static_instance`).
    //  - rclcpp (RFC-0044) C++ node: NO separate `Node` — the component OWNS its
    //    node, constructed from the executor handle. An aligned arena slot holds
    //    the component; it is placement-new'd in setup *after* `nros::init`.
    // Phase 257 (W0-B) — forward-declare the uniform install seam for each unique
    // Rust pkg. The Rust node self-creates its node on the shared executor; the entry
    // hands it `::nros::global_handle()` (= `*mut Executor`).
    let mut seen_rust_pkgs: Vec<String> = Vec::new();
    let mut wrote_rust_extern = false;
    for n in &plan.nodes {
        if !is_rust_node(n) {
            continue;
        }
        let pkg = sanitize_pkg(&n.pkg);
        if seen_rust_pkgs.contains(&pkg) {
            continue;
        }
        if !wrote_rust_extern {
            out.push_str(
                "\n// Rust component install seam (nros::node!); the Rust node\n\
                 // self-creates its node on the shared executor handle (phase-257).\n",
            );
            out.push_str("extern \"C\" {\n");
            wrote_rust_extern = true;
        }
        let _ = writeln!(
            out,
            "    int32_t __nros_component_{pkg}_install(const void* node, void* executor, void* self);"
        );
        seen_rust_pkgs.push(pkg);
    }
    if wrote_rust_extern {
        out.push_str("}\n");
    }
    out.push('\n');

    out.push_str("// Static per-node storage (outlives the spin loop; no heap).\n");
    for (i, n) in plan.nodes.iter().enumerate() {
        if is_rust_node(n) {
            // Phase 257 (W0-B) — Rust node self-creates its node + owns its state on
            // the shared executor (D7 Option C); no entry-side `Node`/component object.
            continue;
        }
        if is_rclcpp_node(n) {
            let cls = n.class_name.as_deref().unwrap();
            let _ = writeln!(
                out,
                "alignas(::{cls}) static unsigned char __nros_comp_buf_{i}[sizeof(::{cls})];"
            );
            let _ = writeln!(out, "static ::{cls}* __nros_comp_{i} = nullptr;");
        } else {
            let _ = writeln!(out, "static ::nros::Node __nros_node_{i};");
            if !is_c_node(n) {
                let cls = n.class_name.as_deref().unwrap();
                let _ = writeln!(out, "static ::{cls} __nros_comp_{i};");
            }
        }
    }
    out.push('\n');

    // Phase 269 (W4) / 272 (W2) — sched-context wiring guard.
    let use_tiers = plan
        .resolved_tiers
        .as_ref()
        .is_some_and(|t| !t.is_single_tier());
    // Phase 274.W2 — multi-tier native → per-tier threads (run_tiers).
    // Phase 274.W3 — FreeRTOS embedded also uses run_tiers (per-RTOS tasks).
    // phase-281 W3a — Zephyr embedded also uses run_tiers (one k_thread per tier
    // over one shared session, via `nros_board_zephyr_run_tiers`).
    // phase-281 W3 (nuttx) — NuttX embedded also uses run_tiers (one pthread per
    // tier over one shared session, via `nros_board_nuttx_run_tiers`). The
    // remaining embedded board (ThreadX) keeps the single-executor sched-context path.
    // Phase 282 follow-up (RFC-0047) — a node whose callback groups map to
    // MORE THAN ONE tier (`group_tiers = { ctrl = "high", telem = "low" }`)
    // cannot be expressed by run_tiers: its per-tier setup fns construct whole
    // NODES, so a group-split node silently landed on whichever tier iterated
    // last and BOTH its timers ran at that tier's cadence. Such plans keep the
    // single-executor sched-context path (`bind_group_sched` seeds each group
    // to its tier's sched context), which expresses the split correctly.
    let has_group_split = plan
        .resolved_tiers
        .as_ref()
        .is_some_and(|t| t.has_group_split_node());
    // phase-308 W1 — a metadata probe always takes the single-setup shape.
    // Tiers would be worse than irrelevant here: `create_entity` early-returns
    // for entities whose callback group is inactive on the running tier, so a
    // per-tier probe would UNDER-count exactly the entities the sidecar exists
    // to count. Recording everything once is the correct probe semantics.
    let use_run_tiers = !matches!(tail, EntryTail::MetadataProbe(_))
        && use_tiers
        && !has_group_split
        && (!board_is_embedded(&plan.board)
            || board_is_freertos_embedded(&plan.board)
            || board_is_zephyr(&plan.board)
            || board_is_nuttx(&plan.board));

    if use_run_tiers {
        // ----------------------------------------------------------------
        // Phase 274.W2 — per-tier setup functions + run_tiers entry point.
        // ----------------------------------------------------------------
        let tiers = plan.resolved_tiers.as_ref().unwrap();

        // node_name → tier_index for per-tier node filtering.
        let node_to_tier: std::collections::HashMap<String, usize> = tiers
            .tiers
            .iter()
            .enumerate()
            .flat_map(|(ti, tier)| {
                tier.members
                    .iter()
                    .map(move |(node_name, _group)| (node_name.clone(), ti))
            })
            .collect();

        // Emit one setup function per tier (only creates THIS tier's nodes).
        for (ti, tier) in tiers.tiers.iter().enumerate() {
            let _ = writeln!(
                out,
                "/* Phase 274.W2 — tier[{ti}] ({name}) setup: creates only this tier's nodes. */",
                name = tier.name
            );
            let _ = writeln!(
                out,
                "static int32_t __nros_entry_setup_tier_{ti}(void* executor) {{"
            );
            out.push_str(
                "    if (executor == nullptr) \
                 return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
            );

            // Issue 0745 — seeds (emit_declare_params) run BEFORE each
            // construction; the executor lazily creates the parameter STORE
            // on first declare. Service registration stays post-construction
            // below (service servers need live entities) and PRESERVES the
            // seeded store.

            for (i, n) in plan.nodes.iter().enumerate() {
                let node_name = n.name.as_deref().unwrap_or(&n.exec);
                // Only emit nodes pinned to this tier.
                if node_to_tier.get(node_name).copied() != Some(ti) {
                    continue;
                }
                let name_lit = node_name.replace('\\', "\\\\").replace('"', "\\\"");
                let _ = writeln!(out, "    {{");
                // Phase 305 W3 (issue 0255) — remap rules BEFORE construction:
                // an rclcpp-shape ctor registers entities immediately.
                emit_declare_remaps(&mut out, n, "        ", "executor");
                emit_declare_params(&mut out, n, "        ", "executor");
                if is_rust_node(n) {
                    // Rust node: install onto the tier's explicit executor handle.
                    let pkg = sanitize_pkg(&n.pkg);
                    let _ = writeln!(
                        out,
                        "        int32_t crc = __nros_component_{pkg}_install(nullptr, executor, nullptr);"
                    );
                    out.push_str("        if (crc != 0) return crc;\n");
                } else if is_rclcpp_node(n) {
                    // rclcpp shape: construct with the tier's explicit executor handle.
                    let cls = n.class_name.as_deref().unwrap();
                    out.push_str("        ::nros::NodeHandle __h(executor);\n");
                    out.push_str(
                        "        if (!__h.valid()) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
                    );
                    let _ = writeln!(
                        out,
                        "        __nros_comp_{i} = new (__nros_comp_buf_{i}) ::{cls}(__h);"
                    );
                    let _ = writeln!(out, "        if (!__nros_comp_{i}->ok()) {{");
                    let _ = writeln!(
                        out,
                        "            ::nros::detail::report_component_failure(\"{name_lit}\", __nros_comp_{i}->error_what(), __nros_comp_{i}->error_code());"
                    );
                    let _ = writeln!(out, "            return __nros_comp_{i}->error_code();");
                    out.push_str("        }\n");
                } else {
                    // Configure-shape (C++ or C): create on the tier's executor.
                    let _ = writeln!(
                        out,
                        "        ::nros::Result r = ::nros::create_node_on(__nros_node_{i}, executor, \"{name_lit}\");"
                    );
                    out.push_str("        if (!r.ok()) return static_cast<int32_t>(r.raw());\n");
                    emit_qos_overrides(&mut out, i, &n.qos_overrides);
                    if is_c_node(n) {
                        let pkg = sanitize_pkg(&n.pkg);
                        let _ = writeln!(
                            out,
                            "        void* self = __nros_c_component_{pkg}_create();"
                        );
                        let _ = writeln!(
                            out,
                            "        int32_t crc = __nros_c_component_{pkg}_configure(__nros_node_{i}.ffi_handle(), __nros_node_{i}.executor_handle(), self);"
                        );
                        out.push_str("        if (crc != 0) return crc;\n");
                    } else {
                        let _ = writeln!(
                            out,
                            "        r = __nros_comp_{i}.configure(__nros_node_{i});"
                        );
                        out.push_str(
                            "        if (!r.ok()) return static_cast<int32_t>(r.raw());\n",
                        );
                    }
                }
                out.push_str("    }\n");
            }

            // Params + lifecycle go in tier[0] (the boot/owning executor).
            if ti == 0 {
                if plan.param_services {
                    out.push_str(
                        "    /* Phase 269 (W1) — param-services: register the runtime get/set surface\n     * (seeding: emit_declare_params, pre-construction — issue 0745). */\n",
                    );
                    out.push_str("    {\n");
                    out.push_str(
                        "        /* Non-fatal (issue 0745): on RMWs without service-server support\n         * (e.g. cyclonedds today) registration fails — the runtime get/set RPC\n         * is unavailable, but the SEEDED store above already carries the launch\n         * initials, so boot proceeds. */\n",
                    );
                    out.push_str("        (void)nros_cpp_register_parameter_services(executor);\n");
                    out.push_str("    }\n");
                }
                if let Some(autostart) = &plan.lifecycle {
                    let autostart_code: u8 = match autostart.as_str() {
                        "none" => 0,
                        "configure" => 1,
                        _ => 2,
                    };
                    out.push_str(
                        "    /* Phase 269 (W2) — lifecycle-services: register + autostart. */\n",
                    );
                    out.push_str("    {\n");
                    let _ = writeln!(
                        out,
                        "        nros_cpp_lifecycle_autostart(executor, {autostart_code}u);"
                    );
                    out.push_str("    }\n");
                }
            }

            out.push_str("    return 0;\n}\n\n");
        }

        // Emit per-tier groups string arrays.
        // Groups are derived from tier.members (unique callback-group IDs, stable order).
        out.push_str("/* Phase 274.W2 — per-tier groups arrays + tier spec table. */\n");
        let tier_groups_vecs: Vec<Vec<String>> = tiers
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
        for (ti, groups) in tier_groups_vecs.iter().enumerate() {
            if !groups.is_empty() {
                let _ = write!(out, "static const char* __nros_tier_{ti}_groups[] = {{");
                for g in groups {
                    let g_lit = g.replace('\\', "\\\\").replace('"', "\\\"");
                    let _ = write!(out, "\"{g_lit}\", ");
                }
                out.push_str("};\n");
            }
        }

        // Emit the NativeTierSpec array (highest-priority-first; resolver produces this order).
        let n_tiers = tiers.tiers.len();
        // #0266 — time_slice_us has a per-thread consumer only on the Rust
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
        let _ = writeln!(
            out,
            "static const ::nros::board::NativeTierSpec __nros_tiers[{n_tiers}] = {{"
        );
        for (ti, tier) in tiers.tiers.iter().enumerate() {
            let name_lit = tier.name.replace('\\', "\\\\").replace('"', "\\\"");
            let priority = tier.priority;
            let spin_period_us = tier.spin_period_us.unwrap_or(0);
            // RFC-0052 W2 — stack_bytes now propagates (the pre-W2 literal
            // hardcoded 0, so [tiers.*.freertos].stack_bytes never reached
            // the task-create call); core rides as core+1 (0 = unpinned),
            // preempt_threshold as -1 = unset.
            let stack_bytes = tier.stack_bytes.unwrap_or(0);
            let core_plus1 = tier.core.map(|c| c + 1).unwrap_or(0);
            let preempt = tier.preempt_threshold.unwrap_or(-1);
            // phase-296 W5.7 — the generic real-time policy rides the spec so
            // kernel-native consumers (Zephyr EDF) can self-apply it on the
            // tier thread; NULL/0 = unset.
            let c_lit = |s: Option<&str>| match s {
                Some(v) => format!("\"{}\"", v.replace('\\', "\\\\").replace('"', "\\\"")),
                None => "nullptr".to_string(),
            };
            let tier_class = c_lit(tier.class.as_deref());
            let dpolicy = c_lit(tier.deadline_policy.as_deref());
            let period_us = tier.period_us.unwrap_or(0);
            let budget_us = tier.budget_us.unwrap_or(0);
            let deadline_us = tier.deadline_us.unwrap_or(0);
            let groups = &tier_groups_vecs[ti];
            let (groups_expr, n_groups_val) = if groups.is_empty() {
                ("nullptr".to_string(), 0usize)
            } else {
                (format!("__nros_tier_{ti}_groups"), groups.len())
            };
            let _ = writeln!(
                out,
                "    {{ \"{name_lit}\", {groups_expr}, {n_groups_val}u, \
                 {priority}LL, {stack_bytes}u, {spin_period_us}ull, \
                 &__nros_entry_setup_tier_{ti}, {core_plus1}u, {preempt}LL, \
                 {tier_class}, {period_us}ull, {budget_us}ull, {deadline_us}ull, \
                 {dpolicy} }},"
            );
        }
        out.push_str("};\n\n");

        // Phase 266 — bake the boot config.
        emit_boot_config_static(&mut out, plan)?;
        out.push('\n');

        // Phase 274.W2/W3 / phase-281 W3a / W3(nuttx) — entry point: Zephyr →
        // plain `int main(void)` (the kernel calls main directly); FreeRTOS +
        // NuttX embedded → nros_app_main + NROS_APP_MAIN_REGISTER_VOID (startup
        // path calls app_main); native → int main(argc, argv).
        let board = board_cpp_path(&plan.board);
        // phase-281 W3a (Zephyr, kernel calls `main(void)` directly) /
        // phase-274 W3 (FreeRTOS) / phase-281 W3 (NuttX, `startup.c` owns
        // `main` and dispatches to `app_main`) / native. The call is the SAME
        // for every board that has `run_tiers`; only the wrapper differs, and
        // that is `boot_shape`'s single derivation.
        let call = format!(
            "{board}::run_tiers(nros_boot_config_node_name(&NROS_BOOT_CONFIG), \
__nros_tiers, {n_tiers}u)"
        );
        emit_boot_wrapper(&mut out, boot_shape(&plan.board), &call);
    } else {
        // ----------------------------------------------------------------
        // Single-executor path (single-tier OR embedded multi-tier with
        // sched-context scheduling). Byte-identical output for single-tier.
        // ----------------------------------------------------------------
        out.push_str("static int32_t __nros_entry_setup() {\n");

        // Phase 269 (W4) / 272 (W2) — sched-context wiring for embedded multi-tier.
        if use_tiers {
            let tiers = plan.resolved_tiers.as_ref().unwrap();
            let n_tiers = tiers.tiers.len();
            out.push_str(
                "    /* Phase 269 (W4) — sched-context wiring (multi-tier scheduling). */\n",
            );
            let _ = writeln!(out, "    uint8_t __nros_sc_ids[{n_tiers}] = {{0}};");
            out.push_str("    {\n");
            out.push_str("        void* __exec = ::nros::global_handle();\n");
            out.push_str(
                "        if (__exec == nullptr) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
            );
            for (ti, tier) in tiers.tiers.iter().enumerate() {
                let os_pri = (tier.priority.clamp(0, 255)) as u8;
                // RFC-0052 (common backend) — route the tier's RTOS-agnostic
                // policy through `nros_cpp_create_sched_context_from_policy`,
                // whose body calls `SchedContext::from_tier_policy` — the SAME
                // lowering the Rust runtime's `apply_tier_sched_policy` uses. The
                // C++ entry forwards RAW tier fields only (no class/budget/period
                // mapping here), so a `real_time` tier lowers to the identical
                // Sporadic SC on every language and the mapping can never drift
                // between codegen paths. `nullptr` / `0` mean "absent"; a tier
                // with no RT class yields a `Fifo` SC carrying just `os_pri`.
                let c_str = |s: Option<&str>| match s {
                    Some(v) => {
                        format!("\"{}\"", v.replace('\\', "\\\\").replace('"', "\\\""))
                    }
                    None => "nullptr".to_string(),
                };
                let class_arg = c_str(tier.class.as_deref());
                let dpolicy_arg = c_str(tier.deadline_policy.as_deref());
                let period_us = tier.period_us.unwrap_or(0);
                let budget_us = tier.budget_us.unwrap_or(0);
                let deadline_us = tier.deadline_us.unwrap_or(0);
                out.push_str("        {\n");
                let _ = writeln!(
                    out,
                    "            nros_cpp_ret_t __scr{ti} = nros_cpp_create_sched_context_from_policy(__exec, {class_arg}, {period_us}ull, {budget_us}ull, {deadline_us}ull, {dpolicy_arg}, {os_pri}u, &__nros_sc_ids[{ti}]);"
                );
                let _ = writeln!(
                    out,
                    "            if (__scr{ti} != NROS_CPP_RET_OK) return static_cast<int32_t>(__scr{ti});"
                );
                out.push_str("        }\n");
            }
            out.push_str("    }\n");
            out.push_str(
                "    /* Phase 272 (W2) — seed node-name → sched-context table (RFC-0047). */\n",
            );
            out.push_str("    {\n");
            out.push_str("        void* __exec = ::nros::global_handle();\n");
            out.push_str(
                "        if (__exec == nullptr) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
            );
            for n in &plan.nodes {
                if let Some(sc_idx) = n.sched_context {
                    let node_name = n.name.as_deref().unwrap_or(&n.exec);
                    let name_lit = node_name.replace('\\', "\\\\").replace('"', "\\\"");
                    let ns_lit = n
                        .namespace
                        .as_deref()
                        .unwrap_or("/")
                        .replace('\\', "\\\\")
                        .replace('"', "\\\"");
                    let _ = writeln!(
                        out,
                        "        nros_cpp_bind_node_name_sched(__exec, \"{name_lit}\", \"{ns_lit}\", __nros_sc_ids[{sc_idx}]);"
                    );
                }
            }
            out.push_str("    }\n");
            out.push_str(
                "    /* Phase 273 (W2) — seed group → sched-context table (RFC-0047). */\n",
            );
            out.push_str("    {\n");
            out.push_str("        void* __exec = ::nros::global_handle();\n");
            out.push_str(
                "        if (__exec == nullptr) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
            );
            let node_ns: Vec<(String, String)> = plan
                .nodes
                .iter()
                .map(|n| {
                    let name = n.name.as_deref().unwrap_or(n.exec.as_str()).to_string();
                    let ns = n
                        .namespace
                        .as_deref()
                        .unwrap_or("/")
                        .replace('\\', "\\\\")
                        .replace('"', "\\\"");
                    (name, ns)
                })
                .collect();
            for (ti, tier) in tiers.tiers.iter().enumerate() {
                for (node_name, group) in &tier.members {
                    let name_lit = node_name.replace('\\', "\\\\").replace('"', "\\\"");
                    let group_lit = group.replace('\\', "\\\\").replace('"', "\\\"");
                    let ns_lit = node_ns
                        .iter()
                        .find(|(n, _)| n == node_name)
                        .map(|(_, ns)| ns.as_str())
                        .unwrap_or("/");
                    let _ = writeln!(
                        out,
                        "        nros_cpp_bind_group_sched(__exec, \"{name_lit}\", \"{ns_lit}\", \"{group_lit}\", __nros_sc_ids[{ti}]);"
                    );
                }
            }
            out.push_str("    }\n");
        }

        // Issue 0745 — per-node seeds (emit_declare_params below) run before
        // each construction; the executor lazily creates the parameter store
        // on first declare. Service registration stays post-construction.
        for (i, n) in plan.nodes.iter().enumerate() {
            let node_name = n.name.as_deref().unwrap_or(&n.exec);
            let name_lit = node_name.replace('\\', "\\\\").replace('"', "\\\"");
            let _ = writeln!(out, "    {{");
            // Phase 305 W3 (issue 0255) — remap rules BEFORE construction (rclcpp
            // ctors register entities immediately). Global executor handle here.
            emit_declare_remaps(&mut out, n, "        ", "::nros::global_handle()");
            emit_declare_params(&mut out, n, "        ", "::nros::global_handle()");
            if is_rust_node(n) {
                // Phase 257 (W0-B) — Rust node on global executor.
                let pkg = sanitize_pkg(&n.pkg);
                out.push_str("        void* __exec = ::nros::global_handle();\n");
                out.push_str(
                    "        if (__exec == nullptr) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
                );
                let _ = writeln!(
                    out,
                    "        int32_t crc = __nros_component_{pkg}_install(nullptr, __exec, nullptr);"
                );
                out.push_str("        if (crc != 0) return crc;\n");
            } else if is_rclcpp_node(n) {
                // rclcpp shape (RFC-0044): placement-new with global executor handle.
                let cls = n.class_name.as_deref().unwrap();
                out.push_str("        ::nros::NodeHandle __h(::nros::global_handle());\n");
                out.push_str(
                    "        if (!__h.valid()) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
                );
                let _ = writeln!(
                    out,
                    "        __nros_comp_{i} = new (__nros_comp_buf_{i}) ::{cls}(__h);"
                );
                let _ = writeln!(out, "        if (!__nros_comp_{i}->ok()) {{");
                let _ = writeln!(
                    out,
                    "            ::nros::detail::report_component_failure(\"{name_lit}\", __nros_comp_{i}->error_what(), __nros_comp_{i}->error_code());"
                );
                let _ = writeln!(out, "            return __nros_comp_{i}->error_code();");
                out.push_str("        }\n");
            } else {
                // Configure-shape (C++ or C) nodes: use global create_node.
                let _ = writeln!(
                    out,
                    "        ::nros::Result r = ::nros::create_node(__nros_node_{i}, \"{name_lit}\");"
                );
                out.push_str("        if (!r.ok()) return static_cast<int32_t>(r.raw());\n");
                emit_qos_overrides(&mut out, i, &n.qos_overrides);
                if is_c_node(n) {
                    let pkg = sanitize_pkg(&n.pkg);
                    let _ = writeln!(
                        out,
                        "        void* self = __nros_c_component_{pkg}_create();"
                    );
                    let _ = writeln!(
                        out,
                        "        int32_t crc = __nros_c_component_{pkg}_configure(__nros_node_{i}.ffi_handle(), __nros_node_{i}.executor_handle(), self);"
                    );
                    out.push_str("        if (crc != 0) return crc;\n");
                } else {
                    let _ = writeln!(
                        out,
                        "        r = __nros_comp_{i}.configure(__nros_node_{i});"
                    );
                    out.push_str("        if (!r.ok()) return static_cast<int32_t>(r.raw());\n");
                }
            }
            out.push_str("    }\n");
        }
        if plan.param_services {
            out.push_str(
                "    /* Phase 269 (W1) — param-services: register the runtime get/set surface\n     * (seeding: emit_declare_params, pre-construction — issue 0745). */\n",
            );
            out.push_str("    {\n");
            out.push_str("        void* __exec = ::nros::global_handle();\n");
            out.push_str(
                "        if (__exec == nullptr) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
            );
            out.push_str(
                "        /* Non-fatal (issue 0745): on RMWs without service-server support\n         * registration fails — the seeded store already carries the launch\n         * initials, so boot proceeds without the get/set RPC. */\n",
            );
            out.push_str("        (void)nros_cpp_register_parameter_services(__exec);\n");
            out.push_str("    }\n");
        }
        if let Some(autostart) = &plan.lifecycle {
            let autostart_code: u8 = match autostart.as_str() {
                "none" => 0,
                "configure" => 1,
                _ => 2,
            };
            out.push_str("    /* Phase 269 (W2) — lifecycle-services: register + autostart. */\n");
            out.push_str("    {\n");
            out.push_str("        void* __exec = ::nros::global_handle();\n");
            out.push_str(
                "        if (__exec == nullptr) return static_cast<int32_t>(::nros::ErrorCode::NotInitialized);\n",
            );
            let _ = writeln!(
                out,
                "        nros_cpp_lifecycle_autostart(__exec, {autostart_code}u);"
            );
            out.push_str("    }\n");
        }
        out.push_str("    return 0;\n}\n\n");

        // phase-308 W1 — the probe never boots a board: it opens a session
        // against the recording backend, runs setup once, and dumps.
        if let EntryTail::MetadataProbe(export) = tail {
            emit_metadata_probe_main(&mut out, export);
            return Ok(out);
        }

        // Phase 266 (W6) — bake the boot config blob.
        emit_boot_config_static(&mut out, plan)?;
        out.push('\n');

        let board = board_cpp_path(&plan.board);
        // phase-263 C2d (Zephyr: kernel calls `main(void)`) / C2 (embedded:
        // `startup.c` calls `app_main`) / native. Only the host board resolves
        // its locator at runtime, so it is the one that takes no locator
        // argument; the wrapper itself comes from `boot_shape`.
        let shape = boot_shape(&plan.board);
        let call = match shape {
            BootShape::Host => format!(
                "{board}::run_components(nros_boot_config_node_name(&NROS_BOOT_CONFIG), \
&__nros_entry_setup)"
            ),
            _ => format!(
                "{board}::run_components(NROS_ENTRY_LOCATOR, \
nros_boot_config_node_name(&NROS_BOOT_CONFIG), &__nros_entry_setup)"
            ),
        };
        emit_boot_wrapper(&mut out, shape, &call);
    }

    Ok(out)
}

/// A `lang == "c"` node is built via the C factory/configure seam (no C++ class).
/// phase-308 W1 — the probe's `main`.
///
/// `nros::init` opens against whatever `$NROS_RMW` selects; the driver sets
/// `NROS_RMW=metadata`, so every publisher / subscription / service / client
/// created during `configure` is RECORDED rather than transported. Timers and
/// guard conditions never reach the RMW and are captured by the `nros-cpp`
/// hooks instead.
///
/// No spin loop: a probe runs the declaration path and exits. A non-zero
/// return is a real failure the driver surfaces — recording NOTHING is an
/// error, not an empty sidecar.
fn emit_metadata_probe_main(out: &mut String, export: &ProbeExport) {
    out.push_str(
        "// phase-308 — metadata probe. Records what this component DECLARES;\n\
         // opens no transport and never spins.\n\
         extern \"C\" int nros_cpp_metadata_dump(const char*, const char*, const char*,\n\
         \x20                                    const char*, const char*);\n\
         // Registering the recording backend EXPLICITLY, rather than relying on\n\
         // its `.init_array` ctor. A static archive only contributes objects\n\
         // that resolve an undefined symbol, so with no reference the linker\n\
         // never pulled the backend into the executable — it was present in\n\
         // libnros_cpp.a and absent from the binary, and `NROS_RMW=metadata`\n\
         // resolved to an unknown backend. This reference is what pulls it in.\n\
         extern \"C\" int nros_rmw_metadata_register(void);\n\n\
         int main(int /*argc*/, char** /*argv*/) {\n\
         \x20   (void)nros_rmw_metadata_register();\n\
         \x20   ::nros::Result r = ::nros::init(nullptr, 0, \"nros_metadata_probe\");\n\
         \x20   if (!r.ok()) return 1;\n\
         \x20   int32_t rc = __nros_entry_setup();\n\
         \x20   if (rc != 0) return rc;\n",
    );
    let _ = writeln!(
        out,
        "    return nros_cpp_metadata_dump({pkg:?}, {comp:?}, {exe:?}, {lang:?}, {out_path:?});\n\
         }}",
        pkg = export.package,
        comp = export.component,
        exe = export.executable,
        lang = export.language,
        out_path = export.out_path,
    );
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
