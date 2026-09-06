//! `nros config show --system <pkg>` — print the resolved effective config for a
//! bringup system (its typed `system.toml`) with per-value provenance.
//!
//! Phase 256 W9: the legacy `config.toml` reader (`--config <path>` on `show` /
//! `check`) is removed — `config.toml` is retired (RFC-0004 §8) and 0 examples
//! ship one. Embedded runtime config lives in `[package.metadata.nros.deploy.<t>]`.

use crate::orchestration::{
    cargo_metadata_schema::SystemToml, nros_config::NrosConfig, params::load_sourced_toml_values,
};
use clap::{Args as ClapArgs, Subcommand};
use eyre::{Result, WrapErr, eyre};
use std::path::{Path, PathBuf};

#[derive(Debug, Subcommand)]
pub enum Args {
    /// Print the resolved effective config for a bringup system (from its typed
    /// `system.toml`) with per-value provenance.
    Show(ShowArgs),
    /// phase-290 (RFC-0049) — print the resolved BUILD-time knob ladder for a
    /// platform: every knob's final value plus the rung that set it
    /// (builtin / platform / board / env).
    Explain(ExplainArgs),
}

#[derive(Debug, ClapArgs)]
pub struct ShowArgs {
    /// Phase 256 — the bringup pkg to resolve. Omit (or pass with no value) to
    /// default to the workspace's `default_system` (or the sole bringup).
    #[arg(long = "system", num_args = 0..=1, default_missing_value = "")]
    pub system: Option<String>,

    /// Output shape. `human` (default) is the provenance report; `cmake` emits
    /// a single `set(NANO_ROS_FEATURES ...)` line for `include()` from
    /// `nano_ros_workspace()` BEFORE nano-ros is imported (phase-323 W1).
    ///
    /// The cmake form exists so the C/C++ build reaches the capability list
    /// through `SystemToml::capability_enabled` — the same accessor the Rust
    /// facade uses — instead of re-deriving it by parsing TOML in cmake, which
    /// would be the second source this phase exists to remove.
    #[arg(long = "format", value_parser = ["human", "cmake"], default_value = "human")]
    pub format: String,

    /// Workspace root for resolution (default: cwd).
    #[arg(long)]
    pub workspace: Option<PathBuf>,
}

pub fn run(args: Args) -> Result<()> {
    match args {
        Args::Show(args) => show(args),
        Args::Explain(args) => explain(args),
    }
}

#[derive(Debug, ClapArgs)]
pub struct ExplainArgs {
    /// Platform name (a directory under the platforms root, e.g. `zephyr`,
    /// `bare-metal`, `freertos-lwip`).
    #[arg(long)]
    pub platform: String,

    /// Optional board package `nros-board.toml` supplying `[knobs]` deltas
    /// (the ladder's board rung). Explicit path; registry-name resolution is
    /// a follow-up.
    #[arg(long)]
    pub board_toml: Option<PathBuf>,

    /// Platforms root (default: `$NROS_PLATFORMS_DIR`, else
    /// `<repo>/config` located by walking up from cwd).
    #[arg(long)]
    pub platforms_dir: Option<PathBuf>,
}

/// phase-290 (RFC-0049) — the porter's debugging surface: every knob, its
/// final value, and WHICH ladder rung set it. Reads the same loader +
/// resolver the build scripts use (`nros_board_common::platform_config`),
/// including live env overrides, so the printout matches what the next
/// build will bake.
fn explain(args: ExplainArgs) -> Result<()> {
    use nros_board_common::platform_config::{BoardKnobsFile, PlatformsTree};

    // phase-400 W1 — a search path. An explicit --platforms-dir still pins one
    // root, because that is what it is for; otherwise the path is
    // $NROS_PLATFORMS_DIR (if set), then packages/platform, then config/.
    let search: Vec<PathBuf> = match args.platforms_dir {
        Some(d) => vec![d],
        None => {
            let repo = find_repo_root()?;
            PlatformsTree::default_search_path(
                &repo,
                std::env::var("NROS_PLATFORMS_DIR").ok().as_deref(),
            )
        }
    };
    let root = search
        .first()
        .cloned()
        .unwrap_or_else(|| PathBuf::from("config"));
    let tree = PlatformsTree::load_search_path(&search).map_err(|e| {
        eyre!(
            "load platforms from {}: {e}",
            search
                .iter()
                .map(|p| p.display().to_string())
                .collect::<Vec<_>>()
                .join(":")
        )
    })?;

    let board = match &args.board_toml {
        Some(p) => Some(BoardKnobsFile::load(p).map_err(|e| eyre!("{}: {e}", p.display()))?),
        None => None,
    };

    let env_get = |name: &str| std::env::var(name).ok().filter(|v| !v.is_empty());
    let mut tx = tree
        .resolve_tx(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.zenoh.tx),
            &env_get,
        )
        .map_err(|e| eyre!("{e}"))?;
    let warnings = tree
        .capability_check(&args.platform, &mut tx)
        .map_err(|e| eyre!("{e}"))?;

    println!(
        "platform: {}   (platforms root: {})",
        args.platform,
        root.display()
    );
    if let Some(p) = &args.board_toml {
        println!("board:    {}", p.display());
    }
    let caps = tree
        .capabilities(&args.platform)
        .map_err(|e| eyre!("{e}"))?;
    if !caps.is_empty() {
        let caps_str: Vec<String> = caps.iter().map(|(k, v)| format!("{k}={v}")).collect();
        println!("capabilities: {}", caps_str.join(", "));
    }
    println!();
    println!("{:<24} {:<10} set by", "knob", "value");
    println!("{:<24} {:<10} ------", "----", "-----");
    println!(
        "{:<24} {:<10} {}",
        "zenoh.tx.batch",
        tx.batch.value,
        tx.batch.source.as_str()
    );
    println!(
        "{:<24} {:<10} {}",
        "zenoh.tx.split_lock",
        tx.split_lock.value,
        tx.split_lock.source.as_str()
    );
    println!(
        "{:<24} {:<10} {}",
        "zenoh.tx.flush_ms",
        tx.flush_ms.value,
        tx.flush_ms.source.as_str()
    );

    // phase-400 W3/W4 — the transport tenant and everything it implies.
    //
    // Printed with provenance because an opaque layered merge is the failure
    // mode of every layered-config system RFC-0049 surveyed: a resolver that
    // cannot say WHY a knob holds its value cannot be debugged without
    // bisecting fragments.
    let transport = tree
        .resolve_transport(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.transport),
            &env_get,
        )
        .map_err(|e| eyre!("{e}"))?;
    println!(
        "{:<24} {:<10} {}",
        "transport.kind",
        transport.kind.value,
        transport.kind.source.as_str()
    );
    if let Some(ep) = &transport.endpoint.value {
        println!(
            "{:<24} {:<10} {}",
            "transport.endpoint",
            ep,
            transport.endpoint.source.as_str()
        );
    }
    for i in &transport.implied {
        match i.overridden_by {
            None => println!("{:<24} {:<10} implied by {}", i.knob, i.value, i.rule),
            Some(src) => println!(
                "{:<24} {:<10} {} — OVERRIDES implication {}={} from {}",
                i.knob,
                !i.value,
                src.as_str(),
                i.knob,
                i.value,
                i.rule
            ),
        }
    }

    // phase-400 W6 — the executor sizing tenant. Defaults mirror
    // nros-node/build.rs, which stays the authority on how the arena is derived
    // from them; this only adds the platform and board rungs those knobs never
    // had, and makes them visible.
    let exec_defaults: &[(&str, usize)] = &[
        ("max_cbs", 4),
        ("max_sc", 8),
        ("max_nodes", 4),
        ("max_shutdown_cbs", 2),
        ("subscription_buffer_size", 1024),
        ("param_service_buffer_size", 4096),
    ];
    let resolved = tree
        .resolve_executor(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.executor),
            &env_get,
            exec_defaults,
        )
        .map_err(|e| eyre!("{e}"))?;

    // The two knobs whose defaults are DERIVED, not constant — which is why
    // they were missing from the table above and therefore invisible to
    // `explain`, in a report whose whole job is to say where a value came from.
    //
    // Neither derivation is duplicated here. `action_clients` defaults to the
    // resolved `max_cbs` (build.rs then clamps to it, so the default IS the
    // clamp), and `arena_size` defaults to `0`, which is the documented Kconfig
    // sentinel for "derive it" — build.rs stays the one place that knows the
    // formula, and this prints `derived` rather than a number it did not
    // compute.
    let max_cbs = resolved
        .iter()
        .find(|(n, _)| *n == "max_cbs")
        .map_or(4, |(_, r)| r.value);
    let derived_defaults: &[(&str, usize)] = &[("action_clients", max_cbs), ("arena_size", 0)];
    let derived = tree
        .resolve_executor(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.executor),
            &env_get,
            derived_defaults,
        )
        .map_err(|e| eyre!("{e}"))?;

    for (name, r) in resolved.iter().chain(derived.iter()) {
        let shown = if *name == "arena_size" && r.value == 0 {
            "derived".to_string()
        } else {
            r.value.to_string()
        };
        println!(
            "{:<34} {:<10} {}  [{}]",
            format!("executor.{name}"),
            shown,
            r.source.as_str(),
            r.env_key
        );
    }

    // phase-400 W6 — the component-runtime tenant. Defaults mirror
    // `packages/api/nros/build.rs`, which stays the authority on them.
    let runtime_defaults: &[(&str, usize)] = &[
        ("max_components", 4),
        ("component_slot_bytes", 512),
        ("max_class_instances", 2),
        ("max_cell_entities", 8),
    ];
    for (name, r) in tree
        .resolve_runtime(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.runtime),
            &env_get,
            runtime_defaults,
        )
        .map_err(|e| eyre!("{e}"))?
    {
        println!(
            "{:<34} {:<10} {}  [{}]",
            format!("runtime.{name}"),
            r.value,
            r.source.as_str(),
            r.env_key
        );
    }

    // phase-400 W6 — the smoltcp net tenant. Defaults mirror
    // `packages/drivers/net/nros-smoltcp/build.rs`, which stays the authority.
    // `max_udp_sockets` shows 1 here: its builtin is FEATURE-derived (4 with
    // `rtps`), and a cargo feature is not a fact this command can see.
    let net_defaults: &[(&str, usize)] = &[
        ("max_sockets", 1),
        ("max_udp_sockets", 1),
        ("buffer_size", 2048),
        ("connect_timeout_ms", 30_000),
        ("socket_timeout_ms", 10_000),
    ];
    for (name, r) in tree
        .resolve_net(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.net),
            &env_get,
            net_defaults,
        )
        .map_err(|e| eyre!("{e}"))?
    {
        println!(
            "{:<34} {:<10} {}  [{}]",
            format!("net.{name}"),
            r.value,
            r.source.as_str(),
            r.env_key
        );
    }

    // phase-400 W6 — the RMW static-pool tenant. Defaults mirror
    // `packages/rmw/cffi/build.rs`, which stays the authority on them.
    // `NROS_RMW_SUBSCRIBER_SLOTS` is absent on purpose: phase-412 W1 derives it.
    let rmw_defaults: &[(&str, usize)] = &[
        ("max_backends", 8),
        ("max_nodes", 4),
        ("message_info_slots", 64),
    ];
    for (name, r) in tree
        .resolve_rmw(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.rmw),
            &env_get,
            rmw_defaults,
        )
        .map_err(|e| eyre!("{e}"))?
    {
        println!(
            "{:<34} {:<10} {}  [{}]",
            format!("rmw.{name}"),
            r.value,
            r.source.as_str(),
            r.env_key
        );
    }

    // phase-400 W6 — the parameter-storage tenant. Defaults mirror
    // `nros-params/build.rs`, which stays the authority on them.
    let param_defaults: &[(&str, usize)] = &[
        ("max_parameters", 32),
        ("max_param_name_len", 64),
        ("max_string_value_len", 256),
        ("max_array_len", 32),
        ("max_byte_array_len", 256),
    ];
    for (name, r) in tree
        .resolve_params(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.params),
            &env_get,
            param_defaults,
        )
        .map_err(|e| eyre!("{e}"))?
    {
        println!(
            "{:<34} {:<10} {}  [{}]",
            format!("params.{name}"),
            r.value,
            r.source.as_str(),
            r.env_key
        );
    }

    // phase-400 W6 — the platform memory tenant. Defaults mirror the board
    // crate that sizes each: FreeRTOS heap_4 `ucHeap` at the `rmw-zenoh` 2 MiB,
    // and the `nros_app` task stack. Both in BYTES, which is the ladder's unit;
    // their env front-ends are KiB and convert at the rung.
    let mem_defaults: &[(&str, usize)] =
        &[("heap_bytes", 2048 * 1024), ("app_stack_bytes", 128 * 1024)];
    for (name, r) in tree
        .resolve_memory(
            &args.platform,
            board.as_ref().map(|b| &b.knobs.memory),
            &env_get,
            mem_defaults,
        )
        .map_err(|e| eyre!("{e}"))?
    {
        println!(
            "{:<34} {:<10} {}  [{}]",
            format!("memory.{name}"),
            r.value,
            r.source.as_str(),
            r.env_key
        );
    }

    for w in tree
        .transport_warnings(&args.platform, &transport)
        .map_err(|e| eyre!("{e}"))?
    {
        println!("warning: {w}");
    }

    for w in warnings {
        println!("warning: {w}");
    }
    Ok(())
}

/// phase-400 W1 — the repo root itself, for building a platform SEARCH PATH.
/// Returns the ROOT, not a platforms directory: the platform path now has more
/// than one entry and the caller assembles it via `default_search_path`.
fn find_repo_root() -> Result<PathBuf> {
    let mut dir = std::env::current_dir().wrap_err("resolve cwd")?;
    loop {
        if dir.join("nros-sdk-index.toml").exists() {
            return Ok(dir);
        }
        if !dir.pop() {
            return Err(eyre!(
                "not inside a nano-ros checkout (no nros-sdk-index.toml sentinel) — \
                 pass --platforms-dir or set NROS_PLATFORMS_DIR"
            ));
        }
    }
}

// Phase 256 W9 — the legacy `config.toml` reader (`--config <path>` on `show`/`check`)
// is removed: `config.toml` is retired (RFC-0004 §8) and 0 examples ship one. `nros
// config show` is now the resolved-`system.toml` view only.
fn show(args: ShowArgs) -> Result<()> {
    let workspace = match args.workspace {
        Some(w) => w,
        None => std::env::current_dir().wrap_err("resolve cwd")?,
    };
    let system = args.system.as_deref().unwrap_or("");
    if args.format == "cmake" {
        print!("{}", render_features_cmake(&workspace, system)?);
        return Ok(());
    }
    print!("{}", render_resolved(&workspace, system)?);
    Ok(())
}

/// Resolve which bringup a command means: explicit `--system <name>`, else the
/// workspace's `default_system`, else the sole bringup.
///
/// Factored out for phase-323 W1 rather than copied: the cmake emitter and the
/// human report must agree on WHICH system they are describing, and a second
/// copy of this ladder is how they would quietly stop agreeing.
fn resolve_bringup<'a>(
    cfg: &'a NrosConfig,
    system: &str,
    workspace: &Path,
) -> Result<&'a crate::orchestration::nros_config::BringupPackageEntry> {
    if !system.is_empty() {
        return cfg
            .bringup_packages
            .get(system)
            .ok_or_else(|| eyre!("no bringup pkg named '{system}' in {}", workspace.display()));
    }
    if let Some(default) = cfg.workspace_metadata.default_system.as_deref() {
        return cfg
            .bringup_packages
            .get(default)
            .ok_or_else(|| eyre!("default_system '{default}' is not a bringup pkg"));
    }
    if cfg.bringup_packages.len() == 1 {
        return Ok(cfg.bringup_packages.values().next().unwrap());
    }
    Err(eyre!(
        "no `--system <pkg>` given and the workspace has {} bringup pkgs (set \
         [workspace.metadata.nros].default_system or pass --system)",
        cfg.bringup_packages.len()
    ))
}

/// phase-323 W1 — emit the declared capability axes as a cmake list.
///
/// `nano_ros_workspace()` includes this BEFORE `add_subdirectory(<nano-ros>)`,
/// so `NANO_ROS_FEATURES` is populated where `nros-c` / `nros-cpp` read it
/// (`set(_caps ${NANO_ROS_FEATURES})`). Without it the workspace cache reads
/// `NANO_ROS_FEATURES:STRING=` and NO declared capability reaches the C/C++
/// build — issue 0353.
///
/// Goes through `capability_enabled`, which honours both the generic
/// `[system].features` list and the deprecated typed blocks. Emitting the
/// DECLARED names (not the cargo features) keeps the cmake side speaking the
/// same vocabulary as `nros_feature_set`'s `CAPABILITIES` argument.
fn render_features_cmake(workspace: &Path, system: &str) -> Result<String> {
    let cfg = NrosConfig::from_workspace(workspace)
        .wrap_err_with(|| format!("load workspace at {}", workspace.display()))?;
    let entry = resolve_bringup(&cfg, system, workspace)?;
    let sys = &entry.system;
    let declared: Vec<&str> = cargo_nano_ros::capability_resolver::CAPABILITIES
        .iter()
        .filter(|c| sys.capability_enabled(c.declared))
        .map(|c| c.declared)
        .collect();
    Ok(format!(
        "# GENERATED by `nros config show --format cmake` — do not edit.\n\
         # Declared capability axes of system `{}` ({}).\n\
         set(NANO_ROS_FEATURES \"{}\" CACHE STRING \"nano-ros capability axes\" FORCE)\n",
        sys.system.name,
        entry.system_toml_path.display(),
        declared.join(";"),
    ))
}

/// Phase 256 Wave 6 — print the resolved effective config for a bringup system
/// with per-value provenance. The typed `system.toml` is the SSoT both codegen
/// paths read; values resolve from it (provenance `system.toml [section]`) with
/// the built-in default as the floor. Any legacy per-package `nros.toml` overlay
/// is surfaced as DEPRECATED (provenance = the overlay file, via the Wave-0
/// `last_block_source` primitive) so the migration target is visible.
fn render_resolved(workspace: &Path, system: &str) -> Result<String> {
    use std::fmt::Write;
    let cfg = NrosConfig::from_workspace(workspace)
        .wrap_err_with(|| format!("load workspace at {}", workspace.display()))?;

    // Resolve the bringup: explicit `--system <name>`, else `default_system`,
    // else the sole bringup.
    let entry = resolve_bringup(&cfg, system, workspace)?;

    let sys = &entry.system;
    let mut out = String::new();
    let _ = writeln!(
        out,
        "# Resolved config for system '{}' (bringup pkg: {})",
        sys.system.name, entry.name
    );
    let _ = writeln!(out, "# source: {}", entry.system_toml_path.display());
    let _ = writeln!(out);
    let _ = writeln!(out, "[system]");
    line(
        &mut out,
        "rmw",
        &resolved_rmw_display(sys),
        "system.toml [system]",
    );
    line(
        &mut out,
        "domain_id",
        &sys.system.domain_id.to_string(),
        "system.toml [system]",
    );
    if let Some(loc) = &sys.system.locator {
        line(&mut out, "locator", loc, "system.toml [system]");
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "[capabilities]");
    // Report the EFFECTIVE state, via `capability_enabled` — the same accessor
    // the bake and the phase-315 facade consult.
    //
    // This used to read only the typed `[safety]` / `[param_services]` /
    // `[lifecycle]` blocks, so a system declaring the equivalent generic form
    //
    //     [system]
    //     features = ["safety"]
    //
    // was reported as `safety = (absent)  # default` while every consumer built
    // it as ENABLED. An audit tool that contradicts the build is worse than no
    // audit tool: it was used to conclude that ws-safety-rust declared nothing,
    // which briefly looked like phase-315 W2 had dropped a capability.
    //
    // Two sources for one axis is precisely the shape issue 0311 / phase-314
    // spent their length collapsing; this was the reporting path's copy of it.
    for (name, detail) in [
        (
            "safety",
            sys.safety
                .as_ref()
                .map(|s| format!("enabled={} crc={}", s.enabled, s.crc)),
        ),
        (
            "param_services",
            sys.param_services
                .as_ref()
                .map(|p| format!("enabled={}", p.enabled)),
        ),
        (
            "lifecycle",
            sys.lifecycle.as_ref().map(|l| l.autostart.clone()),
        ),
    ] {
        let typed = detail.is_some();
        let via_features = sys.system.features.iter().any(|f| f == name);
        let enabled = sys.capability_enabled(name);
        // Show the typed block's detail when there is one; the generic form
        // carries no detail beyond "on", and a disabled typed block that the
        // features list re-enables must still read as enabled.
        let value = match (&detail, enabled) {
            (Some(d), _) => d.clone(),
            (None, true) => "enabled=true".to_string(),
            (None, false) => "(absent)".to_string(),
        };
        let source = match (typed, via_features) {
            (true, true) => "system.toml [<axis>] + [system] features",
            (true, false) => "system.toml [<axis>]",
            (false, true) => "system.toml [system] features",
            (false, false) => "default",
        };
        line(&mut out, name, &value, source);
    }

    // Legacy overlay audit — a per-package `nros.toml` sitting next to the bringup
    // `system.toml` is the deprecated action-at-a-distance path (RFC-0004 §3.1).
    // Name which blocks it still carries (Wave-0 `last_block_source`).
    let overlay = entry
        .system_toml_path
        .parent()
        .map(|dir| dir.join("nros.toml"))
        .filter(|p| p.is_file());
    if let Some(overlay_path) = overlay {
        let sourced = load_sourced_toml_values(std::slice::from_ref(&overlay_path))?;
        let blocks = [
            "build",
            "lifecycle",
            "param_services",
            "safety",
            "scheduling",
        ];
        let present: Vec<&str> = blocks
            .iter()
            .filter(|b| crate::orchestration::params::last_block_source(&sourced, b).is_some())
            .copied()
            .collect();
        if !present.is_empty() {
            let _ = writeln!(out);
            let _ = writeln!(
                out,
                "# IGNORED overlay (phase-256): {} declares [{}]",
                overlay_path.display(),
                present.join("], [")
            );
            let _ = writeln!(
                out,
                "#   the nros.toml overlay is retired (unread); declare these in the bringup \
                 system.toml and delete the file (RFC-0004 §3.1)."
            );
        }
    }
    Ok(out)
}

/// `[system].rmw`, showing the `zenoh` default when the field is empty.
fn resolved_rmw_display(sys: &SystemToml) -> String {
    if sys.system.rmw.is_empty() {
        "zenoh (default)".to_string()
    } else {
        sys.system.rmw.clone()
    }
}

fn line(out: &mut String, key: &str, value: &str, source: &str) {
    use std::fmt::Write;
    let _ = writeln!(out, "{key:<18} = {value:<28} # {source}");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// phase-315 — a capability declared via the GENERIC `[system] features`
    /// list must report as enabled, not `(absent)`.
    ///
    /// `render_resolved` used to read only the typed `[safety]` /
    /// `[param_services]` / `[lifecycle]` blocks, so a system using the
    /// equivalent generic form was reported absent while every consumer built it
    /// as enabled — the audit tool contradicting the build. The bake has had
    /// `system_config_h_features_list_equivalent_to_typed_blocks` for exactly
    /// this equivalence since phase-261; the reporting path had no such test,
    /// which is why it drifted.
    #[test]
    fn render_resolved_honours_generic_features_list() {
        let dir = tempfile::tempdir().unwrap();
        let bringup = dir.path().join("demo_bringup");
        fs::create_dir_all(&bringup).unwrap();
        fs::write(
            bringup.join("package.xml"),
            r#"<package format="3"><name>demo_bringup</name><version>0.1.0</version></package>"#,
        )
        .unwrap();
        fs::write(
            bringup.join("system.toml"),
            "[system]\nname=\"demo\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             features=[\"safety\",\"param_services\"]\n",
        )
        .unwrap();

        let out = render_resolved(dir.path(), "demo_bringup").unwrap();
        for cap in ["safety", "param_services"] {
            let row = out
                .lines()
                .find(|l| l.trim_start().starts_with(cap))
                .unwrap_or_else(|| panic!("no `{cap}` row in:\n{out}"));
            assert!(
                row.contains("enabled=true"),
                "`{cap}` declared via [system] features but reported absent: {row}"
            );
            assert!(
                row.contains("features"),
                "`{cap}` provenance should name the generic list: {row}"
            );
        }
        // Undeclared axes stay absent — the fix must not turn everything on.
        let lifecycle = out
            .lines()
            .find(|l| l.trim_start().starts_with("lifecycle"))
            .unwrap();
        assert!(lifecycle.contains("(absent)"), "{lifecycle}");
    }

    /// Phase 256 Wave 6 — `render_resolved` prints the resolved SSoT config from
    /// the typed `system.toml` with per-value provenance, and flags a sibling
    /// `nros.toml` legacy overlay (Wave-0 `last_block_source`). Uses a Path-A
    /// bringup (package.xml + system.toml, no Cargo.toml) so no `cargo metadata`
    /// runs in-test.
    #[test]
    fn render_resolved_shows_provenance_and_flags_legacy_overlay() {
        let dir = tempfile::tempdir().unwrap();
        let bringup = dir.path().join("demo_bringup");
        fs::create_dir_all(&bringup).unwrap();
        fs::write(
            bringup.join("package.xml"),
            r#"<package format="3"><name>demo_bringup</name><version>0.1.0</version></package>"#,
        )
        .unwrap();
        fs::write(
            bringup.join("system.toml"),
            "[system]\nname=\"demo\"\nrmw=\"cyclonedds\"\ndomain_id=5\n\
             [safety]\ncrc=true\n[lifecycle]\nautostart=\"active\"\n",
        )
        .unwrap();
        // A legacy overlay still carrying a [build] block — the migration target.
        fs::write(bringup.join("nros.toml"), "[build]\nprofile=\"release\"\n").unwrap();

        let out = render_resolved(dir.path(), "demo_bringup").unwrap();
        assert!(out.contains("system 'demo'"), "{out}");
        assert!(out.contains("rmw") && out.contains("cyclonedds"), "{out}");
        assert!(out.contains("domain_id") && out.contains('5'), "{out}");
        assert!(out.contains("safety") && out.contains("crc=true"), "{out}");
        assert!(out.contains("lifecycle") && out.contains("active"), "{out}");
        assert!(
            out.contains("param_services") && out.contains("(absent)"),
            "{out}"
        );
        // The ignored overlay is named with the block it carries.
        assert!(
            out.contains("IGNORED overlay") && out.contains("[build]"),
            "must flag the legacy nros.toml [build] overlay: {out}"
        );
    }

    /// No `--system` match → a clear error, not a panic.
    #[test]
    fn render_resolved_errors_on_unknown_system() {
        let dir = tempfile::tempdir().unwrap();
        let err = render_resolved(dir.path(), "nope").unwrap_err().to_string();
        assert!(err.contains("nope"), "{err}");
    }
}
