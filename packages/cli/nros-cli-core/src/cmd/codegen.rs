//! `nros codegen` — build-tool-facing C/C++ binding generation.
//!
//! Phase 195.A: folds the former standalone `nros-codegen` binary
//! (`nros-codegen-c`) into the canonical `nros` CLI. Same engine
//! (`cargo_nano_ros`), same call shape, so the cmake / build.rs consumers
//! only change the program name (`nros-codegen …` → `nros codegen …`):
//!
//!   nros codegen --args-file <path> [--language c|cpp] [--verbose]
//!   nros codegen resolve-deps --package-xml <path> --output-cmake <path> [--verbose]
//!
//! Distinct from `nros generate` (the user-facing, `package.xml`-driven surface):
//! this is the JSON-`--args-file` contract the build system already speaks.

use clap::{Args as ClapArgs, Subcommand};
use eyre::{Result, WrapErr, bail, eyre};
use std::path::PathBuf;

use crate::{
    abi_guard::{self, Verb},
    codegen::entry as entry_codegen,
};

#[derive(Debug, ClapArgs)]
pub struct Args {
    #[command(subcommand)]
    pub command: Option<Sub>,

    /// Path to the JSON arguments file (default generate mode)
    #[arg(long)]
    pub args_file: Option<PathBuf>,

    /// Target language: "c" (default) or "cpp"
    #[arg(long, default_value = "c")]
    pub language: String,

    /// Verbose output
    #[arg(long)]
    pub verbose: bool,
}

#[derive(Debug, Subcommand)]
pub enum Sub {
    /// Resolve interface dependencies from package.xml → a CMake script
    ResolveDeps {
        /// Path to package.xml
        #[arg(long)]
        package_xml: PathBuf,

        /// Path to output .cmake file
        #[arg(long)]
        output_cmake: PathBuf,

        /// Verbose output
        #[arg(long)]
        verbose: bool,
    },

    /// Phase 212.K.4 — emit per-example Cyclone-DDS topic descriptors.
    ///
    /// Synthesises Cyclone-shaped IDL from one or more `.msg` sources,
    /// drives the host `idlc` to produce `<pkg>_<Msg>.{c,h}` pairs, and
    /// writes a `register.{c,h}` + JSON manifest the consumer build
    /// script feeds into `cc::Build`.
    #[command(name = "cyclonedds-descriptors")]
    CycloneddsDescriptors(super::codegen_cyclonedds_descriptors::Args),

    /// Phase 219.A/B/C — Entry-pkg TU codegen.
    ///
    /// Walks the workspace pkg-index, parses a bringup pkg's
    /// launch.xml, and emits a `main` TU (Rust, C++ or C) that
    /// invokes each `<node pkg=…>`'s mangled register fn in launch
    /// order. The cmake fn `nano_ros_entry(LAUNCH "…")` shells this
    /// subcommand at configure time; the Rust `nros::main!()` proc-
    /// macro is the in-process equivalent for cargo workspaces.
    Entry(EntryArgs),

    /// phase-432 W2.6 — the entry TU for ONE registered node.
    ///
    /// `nano_ros_node_register()` builds an image from a single
    /// component and has no launch tree, so it cannot pass a
    /// `--model`. It passes the node's facts instead, and Rust
    /// synthesises the one-node plan (see
    /// [`entry_codegen::registered_node`]).
    ///
    /// This exists so the verb stops rendering its OWN copy of the
    /// entry TU. It used to `configure_file()` one of six
    /// `cmake/templates/*_entry_main*.cpp.in` — a second producer of
    /// one artifact, which is how issue 1003's missing session name
    /// lived for three months beside a correct sibling.
    #[command(name = "entry-node")]
    EntryNode(EntryNodeArgs),

    /// phase-432 W3.2 — report which entry pack renders a given (language,
    /// board), and the two facts CMake needs BEFORE codegen runs: the
    /// generated TU's extension, and whether a C-family compiler builds it.
    ///
    /// This exists so `NanoRosEntry.cmake` stops re-deriving them. It used to
    /// key on `NANO_ROS_PLATFORM != "posix"` while the dispatch keyed on the
    /// BOARD, and `board_family()` answers `native` for any key it does not
    /// know — so an unlearned board is embedded to one and native to the
    /// other, and the CLI writes a C TU into a file CMake named `.cpp`.
    #[command(name = "entry-pack")]
    EntryPack(EntryPackArgs),
}

/// phase-432 W3.2 — the query CMake asks before it names an output path.
#[derive(Debug, ClapArgs)]
pub struct EntryPackArgs {
    /// The ENTRY's language: `c`, `cpp` or `rust`.
    #[arg(long)]
    pub lang: String,
    /// The board key, as the plan spells it. Decides whether a C entry routes
    /// to the C++ pack.
    #[arg(long)]
    pub board: String,
    /// Emit the answer as JSON instead of `key=value` lines.
    #[arg(long)]
    pub json: bool,
}

/// phase-432 W2.6 — one registered node's facts, as `nano_ros_node_register()`
/// knows them. Each flag was a `@VAR@` substitution in a retired template.
#[derive(Debug, ClapArgs)]
pub struct EntryNodeArgs {
    /// `c` or `cpp` — the COMPONENT's language. The emitted TU is C++
    /// either way; a C component is reached through its
    /// `NROS_C_COMPONENT` factory/configure seam.
    #[arg(long, value_name = "LANG")]
    pub lang: String,

    /// Board key: `native`, `zephyr`, `nuttx`, `threadx`, `freertos`.
    /// Selects the board class and the boot shape.
    #[arg(long, value_name = "KEY", default_value = "native")]
    pub board: String,

    /// The node's own name — and therefore the SESSION name. Issue 1003
    /// is what happens when this does not reach the emitted call.
    #[arg(long, value_name = "NAME")]
    pub node_name: String,

    /// Sanitized package symbol; the infix of
    /// `__nros_c_component_<pkg>_{create,configure}`. Must match the
    /// `-DNROS_PKG_NAME=` the component TU was compiled with.
    #[arg(long, value_name = "SYM")]
    pub pkg_sym: String,

    /// Fully-qualified component class (C++ only).
    #[arg(long, value_name = "NS::CLASS")]
    pub class: Option<String>,

    /// Header to `#include` for `--class` (C++ only).
    #[arg(long, value_name = "PATH")]
    pub header: Option<String>,

    /// `rclcpp` (the component IS-A node) or `configure` (C++ only).
    #[arg(long, value_name = "SHAPE")]
    pub shape: Option<String>,

    /// Where to write the generated TU.
    #[arg(long, value_name = "PATH")]
    pub out: PathBuf,
}

#[derive(Debug, ClapArgs)]
pub struct EntryArgs {
    /// Target language for the emitted TU.
    #[arg(long, value_name = "LANG")]
    pub lang: String,

    /// Workspace root — the directory holding `src/<pkg>/package.xml`.
    /// Typically the dir containing the workspace-root `CMakeLists.txt`
    /// or `Cargo.toml`.
    #[arg(long)]
    pub workspace: PathBuf,

    /// `"<bringup_pkg>"` or `"<bringup_pkg>:<file>.launch.xml"`.
    /// Omit when `--model` is given.
    #[arg(long, conflicts_with = "model")]
    pub launch: Option<String>,

    /// R1-N2 (RFC-0052 W4.1) — build the entry plan from a resolved
    /// SystemModel instead of a launch file (canonical path).
    #[arg(long, value_name = "system_model.yaml")]
    pub model: Option<PathBuf>,

    /// Board key (`native`, `freertos`, …). Defaults to `native` — the
    /// only Entry-pkg target the C/C++ surface supports today
    /// (Phase 212.L.2).
    #[arg(long)]
    pub board: Option<String>,

    /// Launch-arg overrides — `k=v[,k=v]…`. Forwarded to the parser.
    #[arg(long, value_name = "K=V[,K=V]…")]
    pub args: Option<String>,

    /// Output path for the emitted TU.
    #[arg(long)]
    pub out: PathBuf,

    /// Phase 240.2b (RFC-0043) — emit the **typed** C++ Entry: route each
    /// launch node to the real executor via its component object (construct
    /// `class` + call `configure(node)`), instead of the legacy type-erased
    /// `__nros_component_<pkg>_register` call into the synthesizing interpreter.
    /// Requires `--metadata` (the component class/header source). C++ only.
    #[arg(long)]
    pub typed: bool,

    /// Phase 240.2b — path to the cmake-emitted `nros-metadata.json` whose
    /// `components[]` carry each node's C++ `class` + `class_header`. Required
    /// by `--typed`; ignored otherwise.
    #[arg(long)]
    pub metadata: Option<PathBuf>,

    /// REMOVED (phase-326 / issue 0364). `--host` partitioned the bake by the
    /// ROS 1-ism `<node machine="…">`; multi-host now partitions at RESOLVE
    /// time (`host:=<id>` launch argument + `if=` conditions), so pass the
    /// per-host model via `--model` instead. Kept only to fail loud with
    /// guidance (the `--args` / phase-296 R-code.1 precedent).
    #[arg(long, hide = true)]
    pub host: Option<String>,

    /// Optional `.d`-style depfile path. Populated with every file the
    /// CLI read; consumed by cmake `CMAKE_CONFIGURE_DEPENDS` /
    /// build.rs `cargo:rerun-if-changed=` plumbing.
    #[arg(long)]
    pub depfile: Option<PathBuf>,

    /// Phase 219.J — emit a sidecar `.cmake` file declaring the
    /// `target_link_libraries(<exe> PRIVATE <pkg>_<exec>_component)`
    /// calls the cmake fn `include()`s after codegen. When supplied,
    /// the named `<exe>` target receives a PRIVATE link to every
    /// Node-pkg static lib the launch XML pulls in. Path = sidecar
    /// `.cmake` output.
    #[arg(long, value_name = "EXE_TARGET=PATH", value_parser = parse_link_libs)]
    pub emit_link_libs: Option<(String, PathBuf)>,
}

fn parse_link_libs(s: &str) -> std::result::Result<(String, PathBuf), String> {
    let (lhs, rhs) = s
        .split_once('=')
        .ok_or_else(|| format!("expected `<exe_target>=<sidecar_path>`, got `{s}`"))?;
    if lhs.is_empty() || rhs.is_empty() {
        return Err(format!(
            "expected non-empty `<exe_target>=<sidecar_path>`, got `{s}`"
        ));
    }
    Ok((lhs.to_string(), PathBuf::from(rhs)))
}

pub fn run(args: Args) -> Result<()> {
    match args.command {
        Some(Sub::ResolveDeps {
            package_xml,
            output_cmake,
            verbose,
        }) => {
            // Phase 218.E — ABI version guard. package.xml anchors the
            // consumer workspace; guard walks up to find Cargo.lock.
            abi_guard::check_workspace(&package_xml, Verb::Codegen)?;
            cargo_nano_ros::resolve_deps_from_package_xml(cargo_nano_ros::ResolveDepsConfig {
                package_xml,
                output_cmake,
                verbose,
            })
            .map_err(|e| eyre!("{e:#}"))
        }
        Some(Sub::CycloneddsDescriptors(sub_args)) => {
            super::codegen_cyclonedds_descriptors::run(sub_args)
        }
        Some(Sub::Entry(sub_args)) => run_entry(sub_args),
        Some(Sub::EntryNode(sub_args)) => run_entry_node(sub_args),
        Some(Sub::EntryPack(sub_args)) => run_entry_pack(sub_args),
        None => {
            let Some(args_file) = args.args_file else {
                bail!("nros codegen: --args-file is required (or use a subcommand)");
            };
            // Phase 218.E — ABI version guard. args_file lives inside the
            // consumer's CMake build dir; walking up finds the workspace
            // Cargo.lock.
            abi_guard::check_workspace(&args_file, Verb::Codegen)?;
            match args.language.as_str() {
                "c" => cargo_nano_ros::generate_c_from_args_file(cargo_nano_ros::GenerateCConfig {
                    args_file,
                    verbose: args.verbose,
                })
                .map_err(|e| eyre!("{e:#}")),
                "cpp" => {
                    cargo_nano_ros::generate_cpp_from_args_file(cargo_nano_ros::GenerateCppConfig {
                        args_file,
                        verbose: args.verbose,
                    })
                    .map_err(|e| eyre!("{e:#}"))
                }
                other => {
                    bail!("nros codegen: unsupported language '{other}' (expected 'c' or 'cpp')")
                }
            }
        }
    }
}

/// `nros codegen entry --lang {rust|c|cpp}` — Phase 219.A/B/C.
fn run_entry(args: EntryArgs) -> Result<()> {
    let lang = entry_codegen::Lang::parse(&args.lang)?;

    // R-code.1 — `--args` was a launch-arm concept (launch-time `<arg>`
    // substitution); a resolved model is early-bound, so overrides here can
    // only mean the user wanted a different resolve. Fail loud.
    if args.args.is_some() {
        bail!(
            "codegen entry: `--args` was removed with the launch bake (phase-296 \
             R4) — a SystemModel is early-bound; re-run `nros-launch-resolve` \
             with the desired `KEY:=VALUE` bindings instead"
        );
    }

    let mut plan = if let Some(model_path) = &args.model {
        entry_codegen::plan_from_model(model_path, args.board.clone())?
    } else if args.launch.is_some() {
        // phase-296 R-code.1 — the launch-XML entry bake is REMOVED. The
        // canonical input is a play_launch-resolved SystemModel.
        // phase-336 W7 — the example path was `<bringup>/config/…`, the
        // committed location `check-no-tracked-models` now rejects. The model
        // is a build artifact; point at where `nros sync` writes it.
        bail!(
            "codegen entry: `--launch` was removed (phase-296 R4) — generate a \
             SystemModel and pass `--model`:\n  nros sync            \
             # writes <ws>/build/nros/models/<bringup>/system_model.yaml\n  \
             nros codegen entry --model \
             <ws>/build/nros/models/<bringup>/system_model.yaml …"
        );
    } else {
        bail!("codegen entry: pass --model <system_model.yaml>");
    };

    // phase-326 (issue 0364) — the bake-time host partition is REMOVED with
    // `<node machine=>` (ROS 1 syntax). A resolved model is early-bound, so
    // `--host` here could only mean the user wanted a different resolve —
    // same reasoning as `--args` above.
    if let Some(host) = args.host.as_deref() {
        bail!(
            "codegen entry: `--host` was removed (phase-326 / issue 0364) — \
             multi-host partitions at RESOLVE time now. Resolve a per-host \
             SystemModel and pass it via `--model`:\n  nros-launch-resolve \
             <bringup>/launch/<file>.launch.xml --system <bringup>/system.toml \
             host:={host} -o <bringup>/config/<file>_{host}_model.yaml\n  \
             nros codegen entry --model <bringup>/config/<file>_{host}_model.yaml …"
        );
    }

    let src = if args.typed {
        if lang != entry_codegen::Lang::Cpp && lang != entry_codegen::Lang::C {
            bail!(
                "--typed supports --lang cpp or c (got --lang {})",
                args.lang
            );
        }
        let Some(meta_path) = args.metadata.as_ref() else {
            bail!("--typed requires --metadata <nros-metadata.json>");
        };
        let index = entry_codegen::metadata::ComponentIndex::load(meta_path)?;
        entry_codegen::metadata::enrich_plan(&mut plan, &index)?;
        // Phase 269 (W4) — resolve tiers + stamp PlanNode.sched_context after
        // enrich_plan has populated PlanNode.callback_groups from cmake metadata.
        let target_rtos = entry_codegen::board_to_rtos(&plan.board).to_string();
        entry_codegen::resolve_plan_sched(&mut plan, &target_rtos)?;
        match lang {
            // phase-263 C2 (issue 0097) — the C emitter is native-only (it emits a
            // pure-`.c` TU calling the C `nros_board_native_run_components`). The
            // embedded board runners are C++ only (`ThreadxBoard::run_components`, …), so
            // an embedded C entry routes through the C++ emitter (which produces a `.cpp`
            // TU that invokes each C node via its `extern "C"` `__nros_c_component_*` seam
            // — exactly the single-node `threadx_entry_main_c_typed.cpp.in` shape). The
            // cmake side (`nano_ros_entry`) gives the `.out` a `.cpp` extension + links
            // `NanoRosCpp` for an embedded C entry.
            entry_codegen::Lang::C if !entry_codegen::emit_cpp::board_is_embedded(&plan.board) => {
                entry_codegen::emit_c::emit_typed(&plan).map_err(|e| eyre!("{e}"))?
            }
            // phase-432 W3.1 — SAY that the routing fired.
            //
            // The phase doc asked for a configure-time FATAL_ERROR on ThreadX,
            // on the grounds that "a ThreadX C entry silently becomes a `.cpp`
            // with no diagnostic, which is worse than a refusal". Half of that
            // is right and half is not: the silence was the defect, but the
            // routing WORKS — the C++ board runner drives the entry and reaches
            // each C node through its `extern "C"` seam — so refusing would
            // break something that ships. ThreadX is not special here either;
            // every embedded board routes the same way.
            //
            // So: a line on stderr, not a refusal. The author of a `--lang c`
            // entry learns their TU is C++ and why, and nothing that worked
            // stops working.
            entry_codegen::Lang::C => {
                eprintln!(
                    "nros codegen entry: board `{}` has no C-ABI `run_components`, so this \
                     `--lang c` entry is rendered by the C++ pack (a `.cpp` TU that drives \
                     the C++ board runner and calls each C node through its `extern \"C\"` \
                     seam). phase-432 W3.1 is the item that would give this board a C \
                     runner; `nros codegen entry-pack --lang c --board {}` reports the \
                     routing.",
                    plan.board, plan.board,
                );
                entry_codegen::emit_cpp::emit_typed(&plan).map_err(|e| eyre!("{e}"))?
            }
            // C++ entries, and embedded C entries (routed here for the C++ board runner).
            _ => entry_codegen::emit_cpp::emit_typed(&plan).map_err(|e| eyre!("{e}"))?,
        }
    } else {
        match lang {
            // phase-432 W2.4 (RFC-0091 §7) — the Rust entry VERB is retired.
            //
            // Measured before removing it: nothing invoked it. `nano_ros_entry`
            // rejects any LANG but `cpp`/`c` (`NanoRosEntry.cmake`), a Rust
            // entry reaches the `nros::main!()` proc-macro through
            // `rust_cargo_application()`, and `builder/entry.rs` emits an entry
            // PACKAGE whose body is `nros::main!(...)` — it delegates rather
            // than restating. The only caller was the golden harness.
            //
            // Retiring it is not tidiness. What this verb produced was a
            // STRICTLY POORER entry than the macro's: it renders the
            // `OwnedSpin` register path and nothing else, so a plan declaring
            // tiers, `[lifecycle]`, `[param_services]` or per-entry executor
            // sizing compiled, linked, booted — and ignored all four. That is
            // archived issue 0302's shape (four features that reached one
            // producer and not the other) still live for the four the CLI verb
            // never gained, and a user who found the flag had no way to see it.
            //
            // The RENDERER stays: `emit_rust` is the second rendering the
            // parity corpus compares the proc-macro against, which is the diff
            // this file's sibling promised for two years and never had.
            entry_codegen::Lang::Rust => bail!(
                "--lang rust entry is retired (phase-432 W2.4): a Rust entry is \
                 emitted by the `nros::main!()` proc-macro at compile time. \
                 Scaffold one with `nros new`, or let `nano_ros_entry` drive \
                 `rust_cargo_application()`; this verb rendered the register \
                 path only and silently dropped tiers, lifecycle and param \
                 services."
            ),
            // Phase 257 (Stage-3) — the non-typed C/C++ entry (the synthesizing
            // `EntryNodeRuntime` interpreter) is retired; every C/C++ entry is now
            // typed (`--typed`, real executor).
            entry_codegen::Lang::Cpp | entry_codegen::Lang::C => bail!(
                "non-typed --lang {} entry is retired (phase-257): pass `TYPED` to \
                 nano_ros_entry (→ `--typed`) for the real-executor entry",
                args.lang
            ),
        }
    };

    write_generated_tu(&args.out, &src)?;

    if let Some(depfile) = args.depfile.as_ref() {
        entry_codegen::write_depfile(&args.out, &plan.depfile_paths, depfile)?;
    }

    if let Some((exe_target, sidecar)) = args.emit_link_libs.as_ref() {
        write_link_libs_sidecar(exe_target, &plan, sidecar)?;
    }

    Ok(())
}

/// Write-if-changed for a generated TU.
///
/// Only touch `out` when the contents actually change, so cmake's mtime-based
/// dependency tracking does not spuriously rebuild every downstream target on a
/// re-configure. Shared by both entry verbs — the property is about the build
/// graph, not about which producer wrote the bytes, and a second spelling of it
/// is how one of them ends up merely truncating the file.
fn write_generated_tu(out: &std::path::Path, src: &str) -> Result<()> {
    use std::fs;

    let existing = fs::read_to_string(out).ok();
    if existing.as_deref() != Some(src) {
        if let Some(parent) = out.parent() {
            fs::create_dir_all(parent)
                .wrap_err_with(|| format!("create parent `{}`", parent.display()))?;
        }
        fs::write(out, src).wrap_err_with(|| format!("write generated TU `{}`", out.display()))?;
    }
    Ok(())
}

/// `nros codegen entry-pack` — phase-432 W3.2.
///
/// Prints `key=value` lines a CMake `execute_process` can read back with
/// `string(REGEX MATCH)`, or JSON on `--json`. The key=value form is the
/// default because CMake is the caller this exists for and it has no JSON
/// parser; JSON is there so a human or a test can read it without a second
/// spelling of the format.
fn run_entry_pack(args: EntryPackArgs) -> Result<()> {
    // The language is validated by the ONE parser, not by a string test here:
    // a second spelling of "which languages exist" is exactly the drift this
    // whole phase removes.
    let language =
        nros_lang::Language::parse(&args.lang).map_err(|e| eyre!("codegen entry-pack: {e}"))?;
    let info = entry_codegen::pack::entry_pack_for(language, &args.board)
        .map_err(|e| eyre!("codegen entry-pack: {e}"))?;
    if args.json {
        println!("{}", serde_json::to_string(&info)?);
    } else {
        println!("pack={}", info.pack);
        println!("extension={}", info.extension);
        println!("c_family={}", if info.c_family { 1 } else { 0 });
        println!("routed={}", if info.routed { 1 } else { 0 });
    }
    Ok(())
}

/// `nros codegen entry-node` — phase-432 W2.6.
///
/// The `nano_ros_node_register()` path. It renders through the SAME templates
/// as `nros build`; the only difference is where the one-node plan comes from.
fn run_entry_node(args: EntryNodeArgs) -> Result<()> {
    use entry_codegen::registered_node::RegisteredNode;

    // Validate the language against the one parser, rather than testing the
    // string here — a second spelling of "which languages exist" is exactly
    // what phase-432 is deleting.
    match entry_codegen::Lang::parse(&args.lang)? {
        entry_codegen::Lang::C | entry_codegen::Lang::Cpp => {}
        entry_codegen::Lang::Rust => bail!(
            "codegen entry-node: --lang rust is not this verb's shape — a Rust \
             component registers through `nano_ros_node_register(LANGUAGE RUST)` \
             against its Cargo.toml and boots via `nros::main!`, which is the \
             in-process emitter"
        ),
    }

    // A C++ component needs its class and header, and a C one must not carry
    // them: `class_name` is what selects the component-object shape in the
    // emitter, so a C node that supplied one would silently emit a C++ body
    // against a struct that has no `configure` member. Refuse rather than emit.
    let is_cpp = args.lang != "c";
    if is_cpp && (args.class.is_none() || args.header.is_none()) {
        bail!(
            "codegen entry-node --lang {}: --class and --header are required \
             (the entry constructs the component object by name)",
            args.lang
        );
    }
    if !is_cpp && (args.class.is_some() || args.header.is_some()) {
        bail!(
            "codegen entry-node --lang c: --class/--header do not apply — a C \
             component is reached through its `NROS_C_COMPONENT` \
             factory/configure seam, keyed on --pkg-sym"
        );
    }

    let node = RegisteredNode {
        board: args.board,
        node_name: args.node_name,
        pkg_sym: args.pkg_sym,
        language: args.lang,
        class: args.class,
        header: args.header,
        shape: args.shape,
    };

    let src = node.emit().map_err(|e| eyre!("{e}"))?;
    write_generated_tu(&args.out, &src)
}

/// Phase 219.J — emit the `target_link_libraries` sidecar the cmake
/// fn `include()`s after running codegen. Filters to one
/// `<pkg>_<exec>_component` per unique entry; the cmake target name
/// matches what `nano_ros_node_register()` produces.
fn write_link_libs_sidecar(
    exe_target: &str,
    plan: &entry_codegen::Plan,
    sidecar: &PathBuf,
) -> Result<()> {
    use std::fmt::Write;
    let mut out = String::new();
    let _ = writeln!(
        out,
        "# Generated by `nros codegen entry --emit-link-libs`\n\
         # Source plan: bringup={bringup}, launch={launch}.\n\
         # Phase 219.J: closes workflow-review Gap 4 (Entry pkg auto-links\n\
         # the Node-pkg static libs the launch XML pulled in).",
        bringup = plan.bringup,
        launch = plan.launch_file.display(),
    );
    out.push_str(&format!("target_link_libraries({exe_target} PRIVATE"));
    let mut seen: Vec<String> = Vec::new();
    for n in &plan.nodes {
        let target = n.cmake_link_target();
        if !seen.contains(&target) {
            out.push_str(&format!("\n    {target}"));
            seen.push(target);
        }
    }
    out.push_str(")\n");
    if let Some(parent) = sidecar.parent() {
        std::fs::create_dir_all(parent)
            .wrap_err_with(|| format!("create sidecar parent `{}`", parent.display()))?;
    }
    std::fs::write(sidecar, out)
        .wrap_err_with(|| format!("write link-libs sidecar `{}`", sidecar.display()))?;
    Ok(())
}
