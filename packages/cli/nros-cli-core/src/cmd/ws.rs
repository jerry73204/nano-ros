//! `nros ws …` — workspace-level msg-pkg surface.
//!
//! Phase 210.B.3 + 210.D.1 (locked design). Subcommands:
//!
//! * `env` — print shell export for `NROS_INTERFACE_SEARCH_PATH`.
//! * `list` / `status` / `clean` / `doctor` — workspace msg-pkg utilities.
//!
//! The codegen + `[patch.crates-io]` writer — once a `sync` subcommand here —
//! was promoted to the top-level `nros sync` (phase-265 W5); its implementation
//! is [`run_sync`], still in this module, dispatched from `Cmd::Sync`.
//!
//! **Dual-mode (`cargo`-style):** every subcommand works on BOTH layouts —
//! a multi-pkg colcon workspace (`<root>/src/<pkg>/package.xml`) AND a
//! single standalone pkg (`<root>/package.xml`). Detection runs at command
//! time:
//!
//!   * **colcon-mode** iff `<root>/src/` exists AND at least one
//!     immediate subdir contains `package.xml`.
//!   * **single-pkg mode** iff `<root>/package.xml` exists and the colcon
//!     check fails.
//!
//! Mirrors `cargo build` which works at either a workspace root or a
//! standalone pkg dir without special arg.
//!
//! See `docs/roadmap/phase-210-ros-convention-codegen.md` for the
//! full design (patch authority detection, colcon-shape build dir,
//! the chicken-egg motivation for a pre-cargo sync step).

use crate::atomic_file::atomic_write;
use cargo_nano_ros::provider_scan;
use clap::{Args as ClapArgs, Subcommand, ValueEnum};
use eyre::{Result, WrapErr, bail, eyre};
use rosidl_bindgen::ament::Package;
use rosidl_codegen::RosEdition;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeSet, HashMap, HashSet},
    path::{Path, PathBuf},
};

#[derive(Debug, ClapArgs)]
pub struct Args {
    #[command(subcommand)]
    pub command: Sub,
}

#[derive(Debug, Subcommand)]
pub enum Sub {
    /// Print shell export adding <dir> (default `./src`) to
    /// `NROS_INTERFACE_SEARCH_PATH`. `eval "$(nros ws env)"`.
    Env(EnvArgs),

    /// List discovered msg + rust-consumer pkgs in the workspace (or
    /// single pkg). Prints kind, name, dir per row. (Phase 210.F.3.)
    List(ListArgs),

    /// Freshness check — non-fatal sibling of `sync --check`. Prints a
    /// one-line summary of `n up-to-date / n stale / n missing`.
    Status(StatusArgs),

    /// Remove `generated/` + the auto-managed
    /// `[patch.crates-io]` block from each Rust consumer's patch authority
    /// Cargo.toml. Leaves user-written sections alone.
    Clean(CleanArgs),

    /// Lint workspace pkgs: warn on missing `<member_of_group>
    /// rosidl_interface_packages</member_of_group>` markers, malformed
    /// `package.xml`, stale patch blocks. Mirrors the sync detection.
    Doctor(DoctorArgs),

    /// Write the central `<checkout>/nros-patch.toml` and nothing else
    /// (issue 1038).
    ///
    /// 47 tracked leaf `.cargo/config.toml` files reach the universal-trio
    /// patches through one `include = ["…/nros-patch.toml"]` line, and a
    /// missing `include` target is a HARD cargo error during MANIFEST PARSE —
    /// four frames deep, naming neither the leaf nor `nros sync`. Until now the
    /// only thing that wrote that file was a full `nros sync`, which needs a
    /// workspace; a lane building only STANDALONE leaves has no workspace to
    /// sync, so it got the file by side effect from whichever neighbouring lane
    /// happened to build workspace fixtures first. `threadx-riscv64` is the one
    /// lane with no such neighbour, which is why it is the one that failed.
    ///
    /// This is the same writer `sync` calls, not a second spelling of the
    /// content: the patch table must agree with sync's byte for byte or the
    /// leaves resolve against different crates.
    ///
    /// Hidden: a build-system seam, like `model-dims`.
    #[command(name = "central-patch", hide = true)]
    CentralPatch(CentralPatchArgs),

    /// Print the `execution.tiers` dim keys a committed SystemModel declares,
    /// one per line, sorted (issue 0380).
    ///
    /// Hidden: a build-system seam, same role as `codegen-fingerprint`. It
    /// exists so `scripts/check-model-dims.sh` can ASK for the dim set instead
    /// of re-parsing YAML in shell — the extraction that the sync-time guard
    /// uses stays the only implementation. Reads a file; needs no ROS.
    #[command(name = "model-dims", hide = true)]
    ModelDims(ModelDimsArgs),

    /// phase-341 W4 — verify every leaf's committed
    /// `.cargo/nros-board.toml` matches a fresh render of its board
    /// descriptor. Exits non-zero, naming each disagreement.
    ///
    /// Hidden, same seam role as `model-dims`: it exists so a gate can ASK
    /// rather than re-implement the renderer in shell. The predecessor
    /// (`check-board-cargo-config-applied`) grepped the leaf for a
    /// REPRESENTATIVE arg, which caught a lost group but not a lost argument;
    /// this compares exactly, because the file is now generated.
    #[command(name = "check-board-projections", hide = true)]
    CheckBoardProjections(CheckBoardProjectionsArgs),

    /// phase-351 W5 — print one deploy's resolved board FACTS + SITE config as
    /// `KEY=VALUE` lines, for whoever is about to invoke cargo.
    ///
    /// The delivery seam: cargo config is discovered from the invocation CWD
    /// upward and corrosion runs cargo from the workspace root, so a leaf's own
    /// `.cargo/config.toml` never reaches a workspace member. The process
    /// environment does, and its owner is the invoker (cmake, `just`, west).
    #[command(name = "board-facts")]
    BoardFacts(crate::cmd::board_facts::BoardFactsArgs),

    /// phase-392 W5.b/W5.c — print the ENTITY figures a backend sizes its
    /// tables from (`NROS_DECLARED_SERVICE_SERVERS`,
    /// `NROS_DECLARED_INFRA_QUERYABLES`), as `KEY=VALUE` lines, for whoever is
    /// about to invoke cargo.
    ///
    /// Same delivery seam and same reason as `board-facts`: the process
    /// environment is the only carrier that reaches a workspace member's cargo
    /// invocation (issue 0460). Different QUESTION — this one is answered by
    /// the resolved SystemModel, not by the board.
    #[command(name = "entity-facts")]
    EntityFacts(crate::cmd::entity_facts::EntityFactsArgs),

    /// phase-403 W9 (issue 0965) — compose this image's
    /// `nano_ros_node_register(... ENTITIES …)` declarations into the ENTITY
    /// inventory, and derive `NROS_EXECUTOR_MAX_CBS` from it.
    ///
    /// The sibling question to `entity-facts`, asked of a source that can
    /// answer it. `entity-facts` reads the resolved SystemModel and abstains on
    /// every one in this tree, because a launch file names a node and never
    /// says what that node wires; the register call does.
    ///
    /// Prints the env transport (`KEY=VALUE`) on stdout — the same carrier and
    /// the same reason as `board-facts` / `entity-facts` — and optionally
    /// writes the canonical JSON and the `include()`able CMake projection.
    ///
    /// REFUSES, printing nothing, when any component in the image declared no
    /// entities: a partial inventory derives a slot count smaller than the
    /// image needs, and a short `MAX_CBS` fails entity creation at boot.
    #[command(name = "entity-inventory")]
    EntityInventory(crate::cmd::entity_inventory::EntityInventoryArgs),

    /// phase-348 W1 — list packages that announce a provision
    /// (`<export><nano_ros_provides kind="rmw" name="zenoh"/></export>`),
    /// across the search path.
    ///
    /// phase-420 W6 — the search path is an ordered list of roots, and this is
    /// where you read it: the nano-ros tree, this workspace, then
    /// `[workspace] package_paths` from `<ws>/nros.toml`, then
    /// `NROS_PACKAGE_PATH`. Each root prints with its origin, a MISSING marker
    /// when it is not a directory, and each provision prints what it shadows.
    ///
    /// Source-time discovery: nano-ros builds per-target static objects for
    /// RTOS targets with no dynamic linking, so there is no install step for
    /// an ament index to live in. The scan reads only `package.xml`; a
    /// provider's descriptor is read only when it is the one selected.
    ///
    /// Visible rather than hidden, unlike the gate seams above: "which
    /// backends can I pick, and where did they come from" is a question a user
    /// asks, and it is the answer to a shadowed-provider surprise.
    #[command(name = "providers")]
    Providers(ProvidersArgs),

    /// phase-348 W4 — print the workspace's packages in dependency order, one
    /// per line, so a build configures each after everything it depends on.
    ///
    /// Derived from `package.xml`'s existing `<depend>` tags, not a second
    /// dependency declaration: the file is already parsed by the discovery
    /// scan, and a second source of the same fact is the defect the provider
    /// index guards against, one level up.
    ///
    /// Ties break by name, so the order is reproducible across machines. A
    /// dependency cycle is an error naming every package on it.
    #[command(name = "order")]
    Order(OrderArgs),
}

#[derive(Debug, ClapArgs)]
pub struct OrderArgs {
    /// Workspace to scan. Defaults to the cwd. Scanned as a single root — this
    /// is about build order WITHIN one workspace, so the nano-ros tree is not
    /// part of it.
    #[arg(long)]
    pub workspace: Option<PathBuf>,

    /// Print `name<TAB>dir` instead of a path per line.
    #[arg(long)]
    pub lines: bool,

    /// Restrict the output to these package dirs (relative to the workspace),
    /// preserving dependency order. The cmake seam: a workspace still chooses
    /// its SUBDIRS *set* — platform filtering is a selection, not a dependency
    /// — and asks only for that set to be ORDERED.
    #[arg(long = "subdir", value_name = "DIR")]
    pub subdirs: Vec<PathBuf>,
}

#[derive(Debug, ClapArgs)]
pub struct ProvidersArgs {
    /// Workspace to scan as the second search root. Defaults to the cwd.
    ///
    /// Also the directory `<ws>/nros.toml` is read from, and the directory a
    /// relative `package_paths` / `NROS_PACKAGE_PATH` entry resolves against.
    #[arg(long)]
    pub workspace: Option<PathBuf>,

    /// nano-ros source tree to scan as the FIRST search root. Defaults to the
    /// monorepo containing the workspace, else the one containing this `nros`
    /// binary.
    #[arg(long)]
    pub nano_ros_root: Option<PathBuf>,

    /// Only list provisions of this kind (`rmw`, `board`, `platform`, …).
    #[arg(long)]
    pub kind: Option<String>,

    /// Emit JSON rather than a table.
    #[arg(long)]
    pub json: bool,

    /// Emit one TAB-separated row per provision:
    /// `kind<TAB>name<TAB>package<TAB>root_index<TAB>dir`.
    ///
    /// The cmake seam — cmake has no JSON parser, so it ASKS for a shape it can
    /// read with `string(REPLACE)`. Same role as `ws model-dims`: there is never
    /// a second parser of the index to drift from this one.
    #[arg(long, conflicts_with = "json")]
    pub lines: bool,

    /// Scan, then write the result to this path as a reusable index (W3).
    #[arg(long, value_name = "PATH")]
    pub write_index: Option<PathBuf>,

    /// Read this index instead of scanning. Errors if it is missing, of an
    /// unknown version, or was built for a different search path — an index for
    /// other roots is WRONG rather than stale, and serving it would answer a
    /// question nobody asked.
    #[arg(long, value_name = "PATH", conflicts_with = "write_index")]
    pub index: Option<PathBuf>,

    /// Rescan and compare against this index; exit non-zero listing every
    /// difference. The safety net a file watch cannot provide: a package.xml
    /// that did not exist when the index was written is in nobody's watch list.
    #[arg(long, value_name = "PATH", conflicts_with_all = ["index", "write_index"])]
    pub check_index: Option<PathBuf>,

    /// phase-348 W5 — resolve one `<kind>:<name>` and report the winner plus
    /// everything it shadows. Exits non-zero on ambiguity or an unknown name.
    ///
    /// This is the answer to "why is my patched backend not being used": a
    /// later search root overlays an earlier one, and the loser is named rather
    /// than silently dropped.
    #[arg(long, value_name = "KIND:NAME")]
    pub resolve: Option<String>,
}

#[derive(Debug, ClapArgs)]
pub struct CheckBoardProjectionsArgs {
    /// Workspace or single-package dir to scan. Defaults to the cwd.
    #[arg(default_value = ".")]
    pub path: PathBuf,
    /// Rewrite the projections instead of reporting them (phase-351 W3).
    ///
    /// Same writer `nros sync` uses — this is sync's board-projection pass on
    /// its own, for when a DESCRIPTOR changed and the 59 committed projections
    /// have to follow without paying for message codegen in every leaf.
    #[arg(long)]
    pub write: bool,
}

#[derive(Debug, ClapArgs)]
pub struct ModelDimsArgs {
    /// Path to a committed `system_model.yaml`.
    pub model: PathBuf,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum Shell {
    /// POSIX-shell `export VAR=…` (bash/zsh/sh).
    Posix,
    /// Fish-shell `set -gx VAR …`.
    Fish,
}

#[derive(Debug, ClapArgs)]
pub struct EnvArgs {
    /// Workspace root containing pkg subdirs with `package.xml`. Defaults
    /// to `./src` (the colcon-standard layout).
    pub workspace: Option<PathBuf>,

    /// Output shell flavour.
    #[arg(long, value_enum, default_value = "posix")]
    pub shell: Shell,
}

#[derive(Debug, ClapArgs)]
pub struct SyncArgs {
    /// Workspace root (the dir containing `src/`). Defaults to cwd.
    pub workspace: Option<PathBuf>,

    /// Output dir for generated msg crates (Phase 212 convention is `generated/`).
    #[arg(long, default_value = "generated")]
    pub build_dir: PathBuf,

    /// ROS 2 edition (`humble` | `iron` | `jazzy`). When omitted,
    /// inherits `[system].ros_edition` from a `system.toml` at the workspace root
    /// (RFC-0056 W2b auto-lowering), else `humble`.
    #[arg(long)]
    pub ros_edition: Option<String>,

    /// Don't write — just print what would happen.
    #[arg(long)]
    pub dry_run: bool,

    /// Exit non-zero if any patch block is missing or stale (CI hook;
    /// also used by `nros ws status`).
    #[arg(long)]
    pub check: bool,

    /// Verbose codegen output.
    #[arg(short, long)]
    pub verbose: bool,

    /// nano-ros source tree to scan as the FIRST provider search root.
    ///
    /// Same flag and same meaning as `nros ws providers --nano-ros-root`, so the
    /// index `sync` writes is the one that command would read — see
    /// [`provider_search_path`], which exists because two spellings of the root
    /// list make every read "built for other roots".
    #[arg(long)]
    pub nano_ros_root: Option<PathBuf>,

    /// Provider search roots, REPLACING the default (nano-ros tree, then this
    /// workspace) — colcon's `--base-paths`, and issue 0646.
    ///
    /// The default puts the whole nano-ros checkout in front of every
    /// workspace, and it is rescanned per sync: measured on
    /// `examples/workspaces/mixed`, 1570 of the 1590 directories visited are the
    /// underlay, and `regenerate-bindings.sh` runs 22 syncs. A caller that
    /// already knows the underlay has not moved — a build script looping over
    /// workspaces — can say so here rather than pay for the rediscovery each
    /// time.
    ///
    /// Repeatable; order is search order, as with colcon. Naming a root that
    /// holds no provider is not an error (the same is true of the default
    /// workspace root when someone builds nano-ros on its own), but dropping a
    /// root that DOES hold one makes its boards unresolvable — so this is an
    /// override for a caller that knows the tree, not a tuning knob.
    #[arg(long = "base-paths", value_name = "PATH", num_args = 1..)]
    pub base_paths: Vec<PathBuf>,

    /// Skip the provider index entirely (`<ws>/build/nros/providers.json`).
    ///
    /// The index is a CACHE for later commands, not an input to this sync, so a
    /// caller that will not read it can skip the scan outright. Same shape as
    /// `--no-metadata` above: name the expensive optional step and let the
    /// caller decline it.
    #[arg(long)]
    pub no_provider_index: bool,

    /// phase-330 W4 (RFC-0063) — write resolved SystemModels HERE instead of
    /// into each bringup's `config/`, making them build output rather than
    /// committed source. Consumers find them through the same search order
    /// (`nros_orchestration_ir::model_location`): export `NROS_MODEL_DIR` to
    /// the same path for the build.
    #[arg(long)]
    pub model_dir: Option<PathBuf>,

    /// phase-307 W2 — skip the source-metadata refresh. The refresh compiles a
    /// host probe per Node pkg, which is the slow part of a cold sync; skipping
    /// it leaves any existing sidecars untouched and makes bakes fall back to
    /// the SystemModel's entity lower bound.
    #[arg(long)]
    pub no_metadata: bool,

    /// Path to the nano-ros source tree. Accepted for back-compat but
    /// currently a NO-OP since post-212 alignment: the canonical 212
    /// shape carries nros-* runtime crates as path-deps in the user's
    /// own `[dependencies]`, so duplicating them in the patch block
    /// triggers cargo's "patch unused" warnings. Falls back to the env
    /// var `NROS_REPO_DIR` (cmake-side contract) when the flag is
    /// omitted.
    #[arg(long)]
    pub nano_ros_path: Option<PathBuf>,
}

#[derive(Debug, ClapArgs)]
pub struct ListArgs {
    /// Workspace root (cwd or first ancestor containing `src/`). Defaults
    /// to cwd.
    pub workspace: Option<PathBuf>,
}

#[derive(Debug, ClapArgs)]
pub struct StatusArgs {
    pub workspace: Option<PathBuf>,
    #[arg(long, default_value = "generated")]
    pub build_dir: PathBuf,
}

#[derive(Debug, ClapArgs)]
pub struct CleanArgs {
    pub workspace: Option<PathBuf>,
    #[arg(long, default_value = "generated")]
    pub build_dir: PathBuf,
    /// Don't write — just print what would be removed.
    #[arg(long)]
    pub dry_run: bool,
}

#[derive(Debug, ClapArgs)]
pub struct DoctorArgs {
    pub workspace: Option<PathBuf>,
    #[arg(long, default_value = "generated")]
    pub build_dir: PathBuf,
}

pub fn run(args: Args) -> Result<()> {
    match args.command {
        Sub::Env(a) => run_env(a),
        Sub::List(a) => run_list(a),
        Sub::Status(a) => run_status(a),
        Sub::Clean(a) => run_clean(a),
        Sub::Doctor(a) => run_doctor(a),
        Sub::ModelDims(a) => run_model_dims(a),
        Sub::CentralPatch(a) => run_central_patch(a),
        Sub::CheckBoardProjections(a) => run_check_board_projections(a),
        Sub::BoardFacts(a) => crate::cmd::board_facts::run(a),
        Sub::EntityFacts(a) => crate::cmd::entity_facts::run(a),
        Sub::EntityInventory(a) => crate::cmd::entity_inventory::run(a),
        Sub::Providers(a) => run_providers(a),
        Sub::Order(a) => run_order(a),
    }
}

// =============================================================================
// `nros ws order` — phase-348 W4
// =============================================================================

fn run_order(args: OrderArgs) -> Result<()> {
    use cargo_nano_ros::provider_scan;

    let workspace = match args.workspace {
        Some(w) => w,
        None => std::env::current_dir().wrap_err("resolving cwd as the workspace root")?,
    };
    let workspace = workspace
        .canonicalize()
        .unwrap_or_else(|_| workspace.clone());

    let (pkgs, scan) = provider_scan::scan_workspace_packages(&workspace)?;
    for e in &scan.errors {
        eprintln!("warning: {}: {}", e.path.display(), e.message);
    }
    if pkgs.is_empty() {
        bail!(
            "no package.xml under {} — `ws order` orders the packages of ONE \
             workspace, so an empty result means the path is wrong rather than \
             that the workspace has no order",
            workspace.display()
        );
    }

    // The requested subdirs, resolved, ARE the caller's preferred order — a
    // workspace's authored SUBDIRS list, which already works. Preferring it on
    // ties means the sort only ever moves what a declared dependency requires
    // moving, so a workspace whose entries declare no `<exec_depend>` keeps
    // building exactly as before rather than being reshuffled by name.
    let preference: Vec<PathBuf> = args
        .subdirs
        .iter()
        .map(|s| {
            let p = if s.is_absolute() {
                s.clone()
            } else {
                workspace.join(s)
            };
            p.canonicalize().unwrap_or(p)
        })
        .collect();

    let ordered =
        provider_scan::topological_order_with_priority(&pkgs, &preference).map_err(|cycle| {
            eyre::eyre!(
                "dependency cycle among workspace packages: {cycle}\n  \
             every package listed is on the cycle or behind it; \
             `<depend>` in these package.xml files describes a loop, and no \
             build order satisfies it"
            )
        })?;

    // A requested subdir set is FILTERED from the full order, never ordered on
    // its own: package B may sit between two requested packages without being
    // requested itself, and dropping it before the sort would lose the edge
    // that orders them.
    // (as the caller spelled it, resolved for matching). The ORIGINAL spelling
    // is echoed back, never the resolved one: `canonicalize()` follows
    // symlinks, and this repo's own worktree is commonly reached through one
    // (the home-directory alias the top-level CLAUDE.md warns about). Handing
    // cmake a re-spelled absolute path makes `add_subdirectory()` stop
    // recognising it as part of the source tree — the caller gets its own list
    // back, permuted.
    let wanted: Option<Vec<(PathBuf, PathBuf)>> = if args.subdirs.is_empty() {
        None
    } else {
        Some(
            args.subdirs
                .iter()
                .map(|s| {
                    let p = if s.is_absolute() {
                        s.clone()
                    } else {
                        workspace.join(s)
                    };
                    (s.clone(), p.canonicalize().unwrap_or(p))
                })
                .collect(),
        )
    };

    let mut emitted = 0usize;
    for p in &ordered {
        let shown = match &wanted {
            Some(w) => match w.iter().find(|(_, resolved)| *resolved == p.dir) {
                Some((original, _)) => original.clone(),
                None => continue,
            },
            None => p.dir.clone(),
        };
        emitted += 1;
        if args.lines {
            println!("{}\t{}", p.name, shown.display());
        } else {
            println!("{}", shown.display());
        }
    }

    // A requested subdir that matched nothing is a typo or a moved package, and
    // silently emitting a shorter list would drop it from the build.
    if let Some(w) = &wanted
        && emitted != w.len()
    {
        let missing: Vec<String> = w
            .iter()
            .filter(|(_, resolved)| !ordered.iter().any(|p| p.dir == *resolved))
            .map(|(original, _)| original.display().to_string())
            .collect();
        bail!(
            "requested subdir(s) with no package.xml under {}: {}",
            workspace.display(),
            missing.join(", ")
        );
    }
    Ok(())
}

// =============================================================================
// `nros ws providers` — phase-348 W1 (listing) / W3 (the index)
// =============================================================================

/// The provider index a workspace's build tree caches, `<ws>/build/nros/`.
///
/// Same root as the SystemModels (`build/nros/models/`), for the same reason:
/// it is a build artifact, never committed, and regenerable from source at any
/// time.
pub fn provider_index_path(ws_root: &Path) -> PathBuf {
    ws_root.join("build").join("nros").join("providers.json")
}

/// Resolve the search path the way `nros ws providers` does, so the index sync
/// writes is the index that command would read. One implementation, or the two
/// disagree about roots and every read is rejected as "built for other roots".
///
/// phase-420 W6 — this is now four sources, not two: the nano-ros tree, the
/// workspace, `[workspace] package_paths` from `<ws>/nros.toml`, and
/// `NROS_PACKAGE_PATH`. Composition and precedence live in
/// [`provider_scan::build_search_path`]; the only thing done HERE is reading
/// the two ambient sources, so that function stays a pure one and every test of
/// it can name its inputs.
pub fn provider_search_path(workspace: &Path) -> Result<provider_scan::SearchPath> {
    let nano_ros_root = crate::abi_guard::find_monorepo_root(workspace).or_else(|| {
        let exe = std::env::current_exe().ok()?;
        let exe = exe.canonicalize().unwrap_or(exe);
        crate::abi_guard::find_monorepo_root(&exe)
    });
    provider_search_path_with(nano_ros_root.as_deref(), workspace)
}

/// [`provider_search_path`] with the nano-ros root already decided — the shape
/// `--nano-ros-root` and `nros sync` need, and the one place the config file and
/// the environment are read.
fn provider_search_path_with(
    nano_ros_root: Option<&Path>,
    workspace: &Path,
) -> Result<provider_scan::SearchPath> {
    let package_paths = crate::orchestration::nros_config::workspace_package_paths(workspace)
        .wrap_err_with(|| {
            format!(
                "reading [workspace] package_paths from {}",
                workspace.join("nros.toml").display()
            )
        })?;
    let env = std::env::var(provider_scan::PACKAGE_PATH_ENV).ok();
    Ok(provider_scan::build_search_path(
        nano_ros_root,
        workspace,
        &package_paths,
        env.as_deref(),
    ))
}

/// Print every configured root that is not a directory, once, on stderr.
///
/// RFC-0087 D6's decided middle: a missing root is neither a silent skip nor
/// fatal. Fatal is wrong because the default path's own workspace entry is
/// legitimately absent and a porter's `NROS_PACKAGE_PATH` may name a tree that
/// exists on only some machines — making it fatal would refuse `nros sync` on
/// this monorepo. Silent is wrong because nobody types a path they did not mean
/// to exist, so its absence is a typo or a moved tree, and that is precisely the
/// "why is my provider not found" hour this wave exists to end.
///
/// The default roots are exempt (see `RootOrigin::warns_when_missing`), so this
/// prints nothing at all on an unconfigured workspace.
fn warn_missing_roots(path: &provider_scan::SearchPath) {
    for root in path.missing() {
        match &root.as_written {
            Some(written) => eprintln!(
                "warning: provider search root {} ({}, written `{written}`) is not a \
                 directory — nothing was scanned there",
                root.path.display(),
                root.origin.label(),
            ),
            None => eprintln!(
                "warning: provider search root {} ({}) is not a directory — nothing \
                 was scanned there",
                root.path.display(),
                root.origin.label(),
            ),
        }
    }
}

/// The provider search roots for one `nros sync`, honouring the colcon-style
/// scope flags (issue 0646).
///
/// Precedence, most explicit first:
///
/// 1. `--base-paths` — REPLACES the search path outright, like colcon's;
/// 2. `--nano-ros-root` — replaces only the underlay root, keeping the
///    workspace, and matches `nros ws providers`' flag of the same name;
/// 3. the default — `provider_search_path`, the one implementation shared with
///    that command.
fn provider_roots_for_sync(
    ws_root: &Path,
    base_paths: &[PathBuf],
    nano_ros_root: Option<&Path>,
) -> Result<provider_scan::SearchPath> {
    if !base_paths.is_empty() {
        // `--base-paths` REPLACES, so nothing else contributes — not the
        // config file, not the environment. That is the deliberate difference
        // from `NROS_PACKAGE_PATH`, which only ever appends: a flag is typed
        // per invocation, an exported variable is not. See
        // `provider_scan::build_search_path`.
        return Ok(provider_scan::SearchPath::from_base_paths(base_paths));
    }
    match nano_ros_root {
        Some(root) => provider_search_path_with(Some(root), ws_root),
        None => provider_search_path(ws_root),
    }
}

/// Refresh `<ws>/build/nros/providers.json` over an explicit root list. Warns
/// rather than failing — see the call site in `run_sync`.
fn write_provider_index_with(
    ws_root: &Path,
    search_path: &provider_scan::SearchPath,
    verbose: bool,
) {
    use cargo_nano_ros::provider_scan;

    warn_missing_roots(search_path);
    let roots = search_path.paths();
    let path = provider_index_path(ws_root);
    let scan = match provider_scan::scan_roots(&roots) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("sync: warning: provider scan failed, index not written: {e:#}");
            return;
        }
    };
    for err in &scan.errors {
        eprintln!(
            "sync: warning: {}: {} (not indexed)",
            err.path.display(),
            err.message
        );
    }
    let index = provider_scan::ProviderIndex::from_scan(&roots, &scan);
    match index.write(&path) {
        Ok(()) => {
            if verbose {
                println!(
                    "sync: provider index {} ({} provider(s), {} package(s) scanned)",
                    path.display(),
                    index.providers.len(),
                    index.inputs.len()
                );
            }
        }
        Err(e) => eprintln!("sync: warning: could not write {}: {e:#}", path.display()),
    }
}

fn run_providers(args: ProvidersArgs) -> Result<()> {
    use cargo_nano_ros::provider_scan;

    let workspace = match args.workspace {
        Some(w) => w,
        None => std::env::current_dir().wrap_err("resolving cwd as the workspace root")?,
    };
    let workspace = workspace
        .canonicalize()
        .unwrap_or_else(|_| workspace.clone());
    // Root 0 is the nano-ros tree, and an OUT-OF-TREE workspace is not inside
    // it — walking up from the workspace alone finds nothing, which would drop
    // every in-tree backend from the search path and make the scan useless for
    // exactly the user this feature is for. `provider_search_path` owns that
    // fallback, and `nros sync` calls the SAME function: an index written
    // against a different root list is rejected on read, so two spellings of
    // "which roots" would make every cached read fail.
    let search_path = match args.nano_ros_root {
        Some(r) => {
            let r = r.canonicalize().unwrap_or(r);
            provider_search_path_with(Some(&r), &workspace)?
        }
        None => provider_search_path(&workspace)?,
    };
    // phase-420 W6 — before anything is reported ABOUT the roots, report the
    // roots that are not there. A configured root that silently contributes
    // nothing is indistinguishable in every listing below from one that
    // genuinely holds no providers.
    warn_missing_roots(&search_path);
    let roots = search_path.paths();

    // --check-index rescans and compares; it never prints a listing, because
    // its answer is "current" or "here is exactly what moved".
    if let Some(path) = &args.check_index {
        let index = provider_scan::ProviderIndex::read(path)?;
        if !index.is_valid_for(&roots) {
            bail!(
                "provider index {} was built for roots {:?}, but this invocation \
                 searches {:?} — regenerate it rather than comparing against it",
                path.display(),
                index.roots,
                roots
            );
        }
        let fresh = provider_scan::scan_roots(&roots)?;
        let diff = provider_scan::diff_index(&index, &fresh);
        if diff.is_empty() {
            println!(
                "provider index {} is current ({} provider(s), {} package(s) scanned)",
                path.display(),
                fresh.providers.len(),
                fresh.packages_seen()
            );
            return Ok(());
        }
        eprintln!("provider index {} is STALE:", path.display());
        for p in &diff.added_inputs {
            eprintln!("  new package.xml:     {}", p.display());
        }
        for p in &diff.removed_inputs {
            eprintln!("  removed package.xml: {}", p.display());
        }
        for c in &diff.changed_provisions {
            eprintln!("  provision {c}");
        }
        bail!("re-run `nros sync` (or reconfigure) to refresh the provider index");
    }

    // An index read replaces the scan entirely — that is the point of having
    // one. Reconstructed into a ScanResult so every output path below has a
    // single shape to render, rather than a scanned branch and an indexed
    // branch that can disagree about formatting.
    let result = match &args.index {
        Some(path) => {
            let index = provider_scan::ProviderIndex::read(path)?;
            if !index.is_valid_for(&roots) {
                bail!(
                    "provider index {} was built for roots {:?}, not {:?} — \
                     an index for other roots is wrong, not merely stale",
                    path.display(),
                    index.roots,
                    roots
                );
            }
            provider_scan::ScanResult {
                providers: index.providers,
                errors: Vec::new(),
                inputs: index.inputs,
            }
        }
        None => provider_scan::scan_roots(&roots)?,
    };

    // Report failures on stderr whatever the format: a package.xml that could
    // not be parsed is the reason a provider the user expected is missing, and
    // burying that in a JSON field nobody prints is how "it just isn't found"
    // becomes an hour of debugging.
    for e in &result.errors {
        eprintln!("warning: {}: {}", e.path.display(), e.message);
    }

    if let Some(path) = &args.write_index {
        let index = provider_scan::ProviderIndex::from_scan(&roots, &result);
        index.write(path)?;
        if !args.lines && !args.json {
            println!(
                "wrote provider index {} ({} provider(s), {} input(s))",
                path.display(),
                index.providers.len(),
                index.inputs.len()
            );
            return Ok(());
        }
    }

    if let Some(spec) = &args.resolve {
        let (kind, name) = spec.split_once(':').ok_or_else(|| {
            eyre::eyre!("--resolve takes KIND:NAME (e.g. `rmw:zenoh`), got {spec:?}")
        })?;
        let r =
            provider_scan::resolve_unique(&result, kind, name).map_err(|e| eyre::eyre!("{e}"))?;
        // The root's ORIGIN, not only its index: "root[3] won" is unactionable
        // until you know whether root 3 came from the repository's `nros.toml`
        // or from a variable exported in this shell — those have different
        // fixes, and only one of them is visible in a diff.
        let origin = |i: usize| {
            search_path
                .roots
                .get(i)
                .map(|r| r.origin.label())
                .unwrap_or("?")
        };
        println!(
            "{kind}:{name} -> {}  [{}]  root[{}] ({})",
            r.winner.dir.display(),
            r.winner.package,
            r.winner.root_index,
            origin(r.winner.root_index),
        );
        // Shadowing is a legitimate workflow, so it is reported rather than
        // rejected — but never silently, because a user's overlay quietly
        // losing is the failure that costs an afternoon.
        for s in &r.shadowed {
            println!(
                "  shadows  {}  [{}]  root[{}] ({})",
                s.dir.display(),
                s.package,
                s.root_index,
                origin(s.root_index),
            );
        }
        return Ok(());
    }

    if args.lines {
        // Unfiltered by --kind on purpose: cmake filters the rows it wants, and
        // a `--lines --kind rmw` that silently dropped board rows would make a
        // caller's "no board providers" indistinguishable from "I asked wrong".
        print!(
            "{}",
            provider_scan::ProviderIndex::from_scan(&roots, &result).to_lines()
        );
        return Ok(());
    }

    let kind = args.kind.as_deref();
    let matching = |p: &&provider_scan::ProviderPackage| match kind {
        Some(k) => p.provides.iter().any(|pr| pr.kind == k),
        None => true,
    };

    // phase-420 W6 — computed over the WHOLE scan, then filtered by `--kind`
    // when rendering. Computing it over the filtered set instead would let
    // `--kind rmw` report a board collision as if it were not there, which is
    // the "a narrower question got a different answer" shape `--lines` already
    // documents one branch up.
    let shadowed: Vec<provider_scan::Shadowing> = provider_scan::shadowing(&result)
        .into_iter()
        .filter(|s| kind.is_none_or(|k| s.kind == k))
        .collect();

    if args.json {
        let rows: Vec<serde_json::Value> = result
            .providers
            .iter()
            .filter(matching)
            .map(|p| {
                serde_json::json!({
                    "package": p.package,
                    "dir": p.dir,
                    "root": roots[p.root_index],
                    "provides": p.provides.iter()
                        .filter(|pr| kind.is_none_or(|k| pr.kind == k))
                        .map(|pr| serde_json::json!({"kind": pr.kind, "name": pr.name}))
                        .collect::<Vec<_>>(),
                })
            })
            .collect();
        println!(
            "{}",
            serde_json::to_string_pretty(&serde_json::json!({
                // `roots` keeps its shape — a flat array of paths, indexable by
                // the `root_index` every other surface prints. The provenance
                // is ADDED beside it rather than folded into it, so a consumer
                // reading `roots[i]` today still reads a path tomorrow.
                "roots": roots,
                "search_path": search_path.roots.iter().map(|r| serde_json::json!({
                    "path": r.path,
                    "origin": r.origin.label(),
                    "exists": r.exists,
                    "as_written": r.as_written,
                })).collect::<Vec<_>>(),
                "packages_seen": result.packages_seen(),
                "providers": rows,
                "shadowing": shadowed.iter().map(|s| serde_json::json!({
                    "kind": s.kind,
                    "name": s.name,
                    "same_root_tie": s.same_root_tie,
                    "winner": {"package": s.winner.package, "dir": s.winner.dir,
                               "root": s.winner.root_index},
                    "shadowed": s.shadowed.iter().map(|p| serde_json::json!({
                        "package": p.package, "dir": p.dir, "root": p.root_index,
                    })).collect::<Vec<_>>(),
                })).collect::<Vec<_>>(),
            }))?
        );
        return Ok(());
    }

    print!(
        "{}",
        render_provider_listing(&search_path, &result, kind, &shadowed)
    );
    Ok(())
}

/// The human-readable `nros ws providers` table.
///
/// Split out from [`run_providers`] so the shadowing report is TESTABLE. It is
/// the surface RFC-0087 D6 specifies ("prints each package's kind, the root it
/// came from, and what it hid"), and a report nothing asserts on is a report
/// that quietly stops reporting — which is the failure this whole wave is
/// about.
fn render_provider_listing(
    search_path: &provider_scan::SearchPath,
    result: &provider_scan::ScanResult,
    kind: Option<&str>,
    shadowed: &[provider_scan::Shadowing],
) -> String {
    use std::fmt::Write as _;

    let matching = |p: &&provider_scan::ProviderPackage| match kind {
        Some(k) => p.provides.iter().any(|pr| pr.kind == k),
        None => true,
    };
    let mut out = String::new();

    for (i, root) in search_path.roots.iter().enumerate() {
        let _ = writeln!(
            out,
            "root[{i}] {path}  ({origin}){missing}",
            path = root.path.display(),
            origin = root.origin.label(),
            missing = if root.exists {
                ""
            } else {
                "  MISSING — nothing scanned"
            },
        );
    }

    let mut listed = 0usize;
    for p in result.providers.iter().filter(matching) {
        for pr in p
            .provides
            .iter()
            .filter(|pr| kind.is_none_or(|k| pr.kind == k))
        {
            // RFC-0087 D6 — "each package's kind, the root it came from, and
            // what it hid". The first two were already here; the third is what
            // separates a search PATH from a pile, and it is printed on the row
            // the reader is already looking at rather than in a section they
            // would have to know to scroll to.
            let contest = shadowed
                .iter()
                .find(|s| s.kind == pr.kind && s.name == pr.name);
            let _ = writeln!(
                out,
                "  {kind:<9} {name:<18} {pkg:<22} root[{root}] {dir}{note}",
                kind = pr.kind,
                name = pr.name,
                pkg = p.package,
                root = p.root_index,
                dir = p.dir.display(),
                // Named on the loser's own row too: a reader scanning the flat
                // list must not take a row that will never be selected for one
                // that will. A TIE marks every row it involves, the
                // highest-precedence one included — there is no winner among
                // them, so singling one out would name a selection that will
                // not happen.
                note = match contest {
                    Some(s) if s.same_root_tie => "  (AMBIGUOUS — see below)".to_string(),
                    Some(s) if s.winner.dir != p.dir =>
                        format!("  (shadowed by root[{}])", s.winner.root_index),
                    _ => String::new(),
                },
            );
            // What this one hid, on the winner's row.
            if let Some(s) = contest.filter(|s| s.winner.dir == p.dir && !s.same_root_tie) {
                for hidden in &s.shadowed {
                    let _ = writeln!(
                        out,
                        "      shadows  {pkg:<22} root[{root}] {dir}",
                        pkg = hidden.package,
                        root = hidden.root_index,
                        dir = hidden.dir.display(),
                    );
                }
            }
            listed += 1;
        }
    }

    // The denominator matters: "0 providers" from a scan that saw 200 packages
    // is a migration in progress, while "0 providers" from one that saw 0 is a
    // search path pointing somewhere wrong. Without this they look identical.
    let _ = writeln!(
        out,
        "{listed} provision(s) from {} package(s) with an export, {} package(s) scanned",
        result.providers.iter().filter(matching).count(),
        result.packages_seen(),
    );

    // A count, not a repetition of the rows: the rows above already name every
    // loser, and the number exists so a reader who was not looking for
    // shadowing still learns that some happened.
    let (ties, overlays): (Vec<_>, Vec<_>) = shadowed.iter().partition(|s| s.same_root_tie);
    if !overlays.is_empty() {
        let _ = writeln!(
            out,
            "{n} contested provision(s), {hidden} package(s) shadowed: a LATER search \
             root overlays an earlier one, and every loser is named on the winner's row.",
            n = overlays.len(),
            hidden = overlays.iter().map(|s| s.shadowed.len()).sum::<usize>()
        );
    }
    // Ambiguity is not shadowing and must not be counted as it: there is no
    // winner to name, and a build that asks for one of these by name gets
    // `ResolveError::Ambiguous` rather than a provider.
    //
    // Reported as a FACT, not as a defect. Some families are legitimately
    // claimed twice in one root and separated by their descriptor — `board`
    // `threadx` is claimed by `nros-board-threadx-linux` and
    // `nros-board-threadx-qemu-riscv64`, and `provider_scan::candidates`
    // exists for exactly that (phase-348 W2's finding: a flat "two packages,
    // one name is an error" rule would reject a shipping arrangement). So this
    // says what is true of a by-NAME lookup and does not prescribe a rename
    // that would be wrong advice for half the cases it prints on.
    for s in ties {
        let _ = writeln!(
            out,
            "AMBIGUOUS {kind}:{name} — claimed by {n} packages in root[{root}], which \
             has no internal precedence, so `--resolve {kind}:{name}` refuses. A caller \
             with its own discriminator (a board descriptor's `target_contains`) still \
             resolves it; anything else needs one of them renamed or moved to another root.",
            kind = s.kind,
            name = s.name,
            n = s.shadowed.len() + 1,
            root = s.winner.root_index,
        );
    }
    out
}

// =============================================================================
// `nros ws env`
// =============================================================================

fn run_env(args: EnvArgs) -> Result<()> {
    let abs = resolve_env_root(args.workspace.as_deref())?;
    let abs_s = abs.display().to_string();
    match args.shell {
        Shell::Posix => {
            println!(
                "export NROS_INTERFACE_SEARCH_PATH=\"{abs_s}:${{NROS_INTERFACE_SEARCH_PATH:-}}\""
            );
        }
        Shell::Fish => {
            println!("set -gx NROS_INTERFACE_SEARCH_PATH \"{abs_s}\" $NROS_INTERFACE_SEARCH_PATH");
        }
    }
    Ok(())
}

/// Resolve the dir the cmake-side smart Find-stub will scan as a
/// `NROS_INTERFACE_SEARCH_PATH` entry. Mirrors `sync`'s dual-mode
/// detection so a `cd <my_pkg> && eval "$(nros ws env)"` from inside a
/// standalone pkg works the same as one run at a colcon workspace root.
///
/// Resolution order:
///   1. Explicit path arg → use it.
///   2. `<cwd>/src/<sub>/package.xml` exists → use `<cwd>/src`.
///   3. `<cwd>/package.xml` exists → use `<cwd>/..` (so smart Find-stub
///      finds `<parent>/<my_pkg>/package.xml` from there).
///   4. Fallback → `<cwd>/src` (legacy default; may not exist).
fn resolve_env_root(arg: Option<&Path>) -> Result<PathBuf> {
    if let Some(p) = arg {
        return std::fs::canonicalize(p).map_err(|e| eyre!("ws env: {}: {e}", p.display()));
    }
    let cwd = std::env::current_dir()?;
    let src = cwd.join("src");
    if src.is_dir() && has_pkg_subdir(&src) {
        return std::fs::canonicalize(&src).map_err(|e| eyre!("ws env: {}: {e}", src.display()));
    }
    if cwd.join("package.xml").is_file() {
        let parent = cwd.parent().ok_or_else(|| {
            eyre!(
                "ws env: cwd {} is a standalone pkg but has no parent",
                cwd.display()
            )
        })?;
        return std::fs::canonicalize(parent)
            .map_err(|e| eyre!("ws env: {}: {e}", parent.display()));
    }
    // Fallback — caller might not be in a pkg/workspace dir. Use ./src
    // and surface the error from canonicalize if it doesn't exist.
    std::fs::canonicalize(&src).map_err(|e| {
        eyre!(
            "ws env: {}: {e}\n\
                            (no `src/<pkg>/package.xml` colcon layout and no `package.xml` \
                            at cwd — pass an explicit path arg)",
            src.display()
        )
    })
}

// =============================================================================
// `nros sync` — pre-cargo codegen + patch-table writer
// =============================================================================

/// Scanned workspace pkg.
#[derive(Debug, Clone)]
struct WsPkg {
    name: String,
    dir: PathBuf,
    manifest: PathBuf,
    /// True iff msg pkg (member_of_group=rosidl_interface_packages OR
    /// msg/srv/action dirs).
    is_msg_pkg: bool,
    /// True iff `Cargo.toml` at root.
    is_rust_pkg: bool,
    /// Pkg names declared in `<*depend>` tags (filtered for ROS-meta).
    deps: Vec<String>,
    /// Phase 212.M-F.21 — `false` for path-dep targets imported into
    /// `scan` purely so their `<*depend>` rows can be unioned into the
    /// consumer's dep set. These pkgs are NOT cargo-build entry points
    /// and must not become `[patch.crates-io]` authorities. `true` for
    /// the originally-requested single-pkg dir or every workspace-mode
    /// scan hit.
    is_patch_consumer: bool,
}

impl WsPkg {
    /// True iff this pkg needs a `[patch.crates-io]` authority — a Rust pkg
    /// that builds against the generated msg crates / nros-* runtime via
    /// cargo, so cargo must resolve those path-patches from its authority.
    ///
    /// Phase-265 W5b: a pkg that BOTH defines msgs (`is_msg_pkg`, e.g. an
    /// inline `msg/` dir) AND carries a hand `Cargo.toml` is still a consumer
    /// — `native/custom-msg`, `zephyr .../talker-aemv8r`. The old filter
    /// excluded `is_msg_pkg`, silently dropping these ("no Rust consumer
    /// pkgs"). Pure interface packages never carry a *source* `Cargo.toml`
    /// (the crate is generated into `generated/`), so `is_rust_pkg` already
    /// excludes them without the `!is_msg_pkg` guard. `is_patch_consumer`
    /// still excludes path-dep import targets (the Entry→Component walk).
    fn needs_patch_authority(&self) -> bool {
        self.is_rust_pkg && self.is_patch_consumer
    }
}

/// phase-267 W1c/C3e — generate `<bringup>/nros-bridge.toml` for every bringup
/// whose `system.toml` declares a `[[bridge]]`. Plans the bringup (resolving each
/// bridge topic NAME to its ROS type from the node pkgs' synthetic `publishes`
/// metadata — pre-build, no sidecar), then renders the runtime bridge config the
/// entry's `nros_bridge::run_from_config` consumes. No bridge ⇒ no file written
/// (and a stale one is removed). Non-bridge workspaces never plan here.
/// R-code UX — materialize each bringup's SystemModel as part of `nros
/// sync`, so the user's canonical flow (sync → west/cargo/cmake) never
/// hand-runs the resolver. For every pkg with a `launch/` dir: resolve
/// `config/system_model.yaml` when it is missing or older than any input
/// (launch XMLs, system.toml). When the helper is absent, a model that needs no
/// refresh is used as-is; a model that DOES need one is a hard error, never a
/// silent staleness.
/// Multi-launch bringups also refresh per-launch `config/<name>_model.yaml`
/// siblings that were previously committed (variant models stay opt-in:
/// only refreshed, never created, for non-default launches).
///
/// Issue 0320 — content-addressed staleness. A committed model records a
/// `sha256` for every input under `meta.inputs`. This re-hashes each recorded
/// input against the file on disk and returns `Some(reason)` when the recorded
/// provenance no longer holds: a non-portable absolute path (which regenerates
/// the machine-specific legacy models on any checkout), a recorded input that
/// no longer exists, or a hash that has changed (an input the mtime gate does
/// not watch — a sibling include or the `--sched` platform file). `None` means
/// the model's provenance is intact. A model that cannot be parsed returns
/// `None` so the caller falls back to the mtime gate rather than force-churning.
///
/// Relative paths resolve against `bringup_dir` (the package root), matching
/// how the resolver strips the launch file's grandparent as the base and how
/// `main_macro` re-joins them.
fn model_provenance_stale(model_path: &Path, bringup_dir: &Path) -> Option<String> {
    let raw = std::fs::read_to_string(model_path).ok()?;
    let model = ros_launch_manifest_model::SystemModel::from_yaml_str(&raw).ok()?;
    for input in &model.meta.inputs {
        let recorded = Path::new(&input.path);
        if recorded.is_absolute() {
            return Some(format!("non-portable absolute input path `{}`", input.path));
        }
        let resolved = bringup_dir.join(recorded);
        let Ok(bytes) = std::fs::read(&resolved) else {
            return Some(format!("recorded input missing `{}`", input.path));
        };
        let digest = format!("{:x}", Sha256::digest(&bytes));
        if digest != input.sha256 {
            return Some(format!("input hash changed `{}`", input.path));
        }
    }
    // Issue 0427 — the resolver identity is a freshness input too. A resolver fix
    // (node ordering, params, remaps, tiers) changes the OUTPUT for byte-identical
    // inputs, so a model produced by a DIFFERENT resolver pin is stale even when
    // every input hash matches. nano-ros stamps `meta.resolver.version` with
    // `NROS_PLAY_LAUNCH_SHA` at resolve time (`stamp_resolver_pin`) — the pin
    // `verify_resolver_pin` already agrees on — because the resolver's own
    // self-version is unreliable (it wrote `0.1.0` while the tool was v0.1.4).
    // Skip when our pin is unverifiable, matching `verify_resolver_pin`.
    let ours = env!("NROS_PLAY_LAUNCH_SHA");
    if ours != "unknown" {
        match model.meta.resolver.as_ref().map(|r| r.version.as_str()) {
            Some(v) if v == ours => {}
            Some(v) => {
                return Some(format!(
                    "resolver pin changed (model `{}` ≠ ours `{}`)",
                    &v[..v.len().min(12)],
                    &ours[..ours.len().min(12)]
                ));
            }
            None => return Some("no resolver pin recorded".into()),
        }
    }
    None
}

/// Issue 0427 — record the resolver PIN (`NROS_PLAY_LAUNCH_SHA`) into a freshly
/// resolved model's `meta.resolver`, so [`model_provenance_stale`] treats a
/// resolver change as staleness. Overwrites the resolver's own unreliable
/// self-version (it stamped `0.1.0` at v0.1.4). Called on the STAGED file before
/// it is promoted, so a mid-resolve failure leaves no half-stamped model.
fn stamp_resolver_pin(staged: &Path) -> Result<()> {
    let raw = std::fs::read_to_string(staged)
        .wrap_err_with(|| format!("read staged model {}", staged.display()))?;
    let mut model = ros_launch_manifest_model::SystemModel::from_yaml_str(&raw)
        .map_err(|e| eyre::eyre!("parse staged model {}: {e}", staged.display()))?;
    model.meta.resolver = Some(ros_launch_manifest_model::ResolverInfo {
        tool: "nros-launch-resolve".into(),
        version: env!("NROS_PLAY_LAUNCH_SHA").into(),
    });
    let yaml = model
        .to_yaml_string()
        .map_err(|e| eyre::eyre!("serialize staged model {}: {e}", staged.display()))?;
    std::fs::write(staged, yaml)
        .wrap_err_with(|| format!("write staged model {}", staged.display()))?;
    Ok(())
}

/// phase-326 (issue 0364) — the exact launch-argument binding a committed
/// model was resolved from (`meta.args`). Re-resolving MUST replay it: the
/// binding reaches the parser, where `<arg>` defaults and `if=`/`unless=`
/// conditions evaluate, so a per-host variant model
/// (`multihost_robot1_model.yaml`, resolved with `host:=robot1`) re-resolved
/// without its binding would silently become the default configuration.
/// Unparsable/missing model ⇒ empty binding (the plain resolve).
/// phase-330 W4.0 — file names referenced by `<include file="…">` in a launch
/// file.
///
/// A targeted scan, not a full parse: `parse_launch_file` resolves
/// substitutions and needs a `PkgIndex`, and all this decision needs is "is
/// this launch file pulled in by another one". Only the file NAME is compared,
/// so a `$(find-pkg-share …)` prefix does not defeat it.
fn launch_include_names(path: &Path) -> Vec<String> {
    let Ok(raw) = std::fs::read(path) else {
        return Vec::new();
    };
    let mut reader = quick_xml::Reader::from_reader(raw.as_slice());
    let mut buf = Vec::new();
    let mut out = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(quick_xml::events::Event::Eof) | Err(_) => break,
            Ok(quick_xml::events::Event::Start(e) | quick_xml::events::Event::Empty(e))
                if e.name().as_ref() == b"include" =>
            {
                for attr in e.attributes().flatten() {
                    if attr.key.as_ref() == b"file"
                        && let Ok(v) = attr.unescape_value()
                        && let Some(n) = Path::new(v.as_ref()).file_name().and_then(|s| s.to_str())
                    {
                        out.push(n.to_string());
                    }
                }
            }
            _ => {}
        }
        buf.clear();
    }
    out
}

/// phase-330 W4.0 — a `[[model]]` declaration in a bringup's `system.toml`.
///
/// Binding variants exist because a launch file takes ARGUMENTS
/// (`multihost.launch.xml host:=robot1`). Nothing in the launch tree records
/// which bindings matter: today that fact lives in the committed model's
/// `meta.args`, i.e. in the artifact W4.a wants to delete. Declaring it here
/// moves the fact into the INPUTS, which is RFC-0063's whole thesis applied one
/// level deeper.
///
/// ```toml
/// [[model]]
/// launch = "multihost.launch.xml"
/// out    = "multihost_robot1_model.yaml"
/// args   = { host = "robot1" }
/// ```
#[derive(Debug, Clone)]
pub struct ModelDecl {
    pub launch: String,
    pub out: String,
    pub args: Vec<(String, String)>,
}

/// Read `[[model]]` declarations from a bringup's `system.toml`. A malformed or
/// absent table yields none — the derived defaults still apply.
/// issue 0409 direction 3 — a resolved model must actually CARRY the params its
/// inputs declared.
///
/// The stale-resolver failure was silent because nothing checked the output: a
/// binary predating `apply_params_to_nodes` writes a well-formed model with
/// every `[[component]].params` / `params_files` quietly absent.
///
/// The check has to separate two absences that look identical in the file:
///
/// - **dropped** — the launch tree HAS the node, so the projection had a target
///   and produced nothing. That is the data loss.
/// - **unbound** — the component declares params but this variant's launch tree
///   contains no matching node. Legitimate (a component absent from one
///   variant), and `features/`'s bringups do it today because W2b prefixed the
///   component names while the launch files kept the bare ones.
///
/// A correct resolver already reports the second case in `meta.diagnostics`
/// ("declares params but has no matching launch node"), so its DIAGNOSTIC is
/// the evidence that it considered the declaration at all. A stale one emits
/// neither params nor diagnostics — which is exactly what this rejects.
///
/// Matching is by PACKAGE, not by name: the node records `pkg`, and the
/// component/node name mismatch above would make a name comparison report
/// dropped params for every correctly-unbound component.
fn verify_params_projected(model_path: &Path, system_toml: &Path) -> Result<()> {
    if !system_toml.is_file() {
        return Ok(());
    }
    let Ok(raw) = std::fs::read_to_string(system_toml) else {
        return Ok(());
    };
    let Ok(sys) = toml::from_str::<toml::Value>(&raw) else {
        return Ok(());
    };
    // (component name, pkg) for every component declaring params.
    let declaring: Vec<(String, String)> = sys
        .get("component")
        .and_then(|c| c.as_array())
        .map(|arr| {
            arr.iter()
                .filter(|c| {
                    c.get("params")
                        .and_then(|p| p.as_table())
                        .is_some_and(|t| !t.is_empty())
                        || c.get("params_files")
                            .and_then(|p| p.as_array())
                            .is_some_and(|a| !a.is_empty())
                })
                .filter_map(|c| {
                    Some((
                        c.get("name")?.as_str()?.to_string(),
                        c.get("pkg")?.as_str()?.to_string(),
                    ))
                })
                .collect()
        })
        .unwrap_or_default();
    if declaring.is_empty() {
        return Ok(());
    }

    let Ok(text) = std::fs::read_to_string(model_path) else {
        return Ok(());
    };
    let Ok(model) = serde_yaml_ng::from_str::<serde_yaml_ng::Value>(&text) else {
        return Ok(());
    };

    let diagnostics: String = model
        .get("meta")
        .and_then(|m| m.get("diagnostics"))
        .and_then(|d| d.as_sequence())
        .map(|seq| {
            seq.iter()
                .filter_map(|d| d.as_str())
                .collect::<Vec<_>>()
                .join("\n")
        })
        .unwrap_or_default();

    let nodes = model
        .get("structure")
        .and_then(|s| s.get("nodes"))
        .and_then(|n| n.as_mapping());

    let mut dropped: Vec<String> = Vec::new();
    for (name, pkg) in &declaring {
        // A node from this component's package, if the variant instantiates one.
        let node = nodes.and_then(|m| {
            m.iter()
                .find(|(_, v)| v.get("pkg").and_then(|p| p.as_str()) == Some(pkg.as_str()))
        });
        match node {
            Some((key, value)) => {
                let has_params = value
                    .get("params")
                    .and_then(|p| p.as_mapping())
                    .is_some_and(|m| !m.is_empty());
                if !has_params {
                    dropped.push(format!(
                        "  `{name}` (pkg {pkg}) → node {} has no params",
                        key.as_str().unwrap_or("?")
                    ));
                }
            }
            None if diagnostics.contains(name.as_str()) => {} // reported unbound: legitimate
            None => dropped.push(format!(
                "  `{name}` (pkg {pkg}) → no node from this pkg, and no diagnostic explaining why"
            )),
        }
    }

    if dropped.is_empty() {
        return Ok(());
    }
    bail!(
        "the resolver did not project these declared params (issue 0409):\n{}\n\n\
         A resolver predating `apply_params_to_nodes` (rlm v0.1.1) writes a well-formed\n\
         model with every `params` / `params_files` silently absent. Rebuild it:\n\
         \n    ./scripts/bootstrap.sh      (contributors: just setup-launch-resolve)\n\
         \n\
         If a component is deliberately absent from this variant, the resolver says so in\n\
         `meta.diagnostics` and this check accepts it — an empty diagnostics list next to a\n\
         declaring component is the signal that nothing looked at the declaration.",
        dropped.join("\n")
    )
}

fn system_toml_model_decls(system_toml: &Path) -> Vec<ModelDecl> {
    let Ok(raw) = std::fs::read_to_string(system_toml) else {
        return Vec::new();
    };
    let Ok(val) = toml::from_str::<toml::Value>(&raw) else {
        return Vec::new();
    };
    let Some(arr) = val.get("model").and_then(|m| m.as_array()) else {
        return Vec::new();
    };
    arr.iter()
        .filter_map(|e| {
            let launch = e.get("launch")?.as_str()?.to_string();
            let out = e.get("out")?.as_str()?.to_string();
            let args = e
                .get("args")
                .and_then(|a| a.as_table())
                .map(|t| {
                    t.iter()
                        .filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string())))
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default();
            Some(ModelDecl { launch, out, args })
        })
        .collect()
}

fn model_recorded_args(model_path: &Path) -> Vec<(String, String)> {
    let Ok(raw) = std::fs::read_to_string(model_path) else {
        return Vec::new();
    };
    let Ok(model) = ros_launch_manifest_model::SystemModel::from_yaml_str(&raw) else {
        return Vec::new();
    };
    model.meta.args.into_iter().collect()
}

/// phase-330 / issue 0392 C — a throwaway ament prefix over the workspace's own
/// packages, so `$(find-pkg-share <pkg>)` resolves WITHOUT an install step.
///
/// A nav2-style launch file includes its sibling by
/// `$(find-pkg-share secondary_node)/launch/…`, which the resolver answers
/// through `AMENT_PREFIX_PATH` — i.e. from INSTALLED packages. The fixture's
/// packages exist only as sources, so the resolve died with "Package
/// 'secondary_node' not found. Ensure the package is installed and sourced."
///
/// The planner used to synthesise exactly this (the fixture's launch file still
/// carries a comment describing it), but that path went with the launch-XML
/// parser in phase-296 R4 and nothing replaced it for the resolver. This
/// restores it: `<tmp>/share/<pkg>` symlinks to each package's source dir,
/// PREPENDED to any existing `AMENT_PREFIX_PATH` so a real ROS install is still
/// found for everything else.
///
/// The directory lives as long as the returned handle; the caller holds it for
/// the duration of the sync.
#[cfg(unix)]
fn synth_ament_prefix(scan: &[WsPkg]) -> Option<(tempfile::TempDir, std::ffi::OsString)> {
    let dir = tempfile::TempDir::new().ok()?;
    let share = dir.path().join("share");
    std::fs::create_dir_all(&share).ok()?;
    let mut linked = 0usize;
    for pkg in scan {
        let dest = share.join(&pkg.name);
        if dest.exists() {
            continue;
        }
        if std::os::unix::fs::symlink(&pkg.dir, &dest).is_ok() {
            linked += 1;
        }
    }
    if linked == 0 {
        return None;
    }
    let mut value = std::ffi::OsString::from(dir.path());
    if let Some(existing) = std::env::var_os("AMENT_PREFIX_PATH")
        && !existing.is_empty()
    {
        value.push(":");
        value.push(existing);
    }
    Some((dir, value))
}

#[cfg(not(unix))]
fn synth_ament_prefix(_scan: &[WsPkg]) -> Option<(tempfile::TempDir, std::ffi::OsString)> {
    None
}

fn resolve_system_models(scan: &[WsPkg], verbose: bool, model_dir: Option<&Path>) -> Result<()> {
    // Issue 0285 — resolve the helper by ABSOLUTE PATH, never through PATH.
    //
    // This used to run `play_launch` by bare name. `play_launch` is also an
    // unrelated ROS 2 record/replay tool, so on a host that had that one the
    // wrong binary won and every `nros sync` died with "unrecognized
    // subcommand 'resolve'", taking the whole fixture build with it. Probing
    // the capability instead of the name made that degrade rather than fail,
    // but it could not make the RIGHT tool findable.
    //
    // Now we ship our own `nros-launch-resolve`, built from the pinned
    // play_launch submodule and versioned with this CLI, and look for it next
    // to the running `nros` binary. Nothing on PATH can shadow it — and,
    // equally deliberate, we never put it ON PATH, so we cannot shadow a
    // user's real `play_launch` either.
    //
    // Absent used to be a DEGRADE: warn once, use whatever models are committed,
    // carry on. That is wrong, and it cost a full fixture sweep to notice. The
    // helper is a setup step (`just setup-launch-resolve`, now reached from
    // `just setup` and `just build-test-fixtures`), so absent means the tree is
    // mis-provisioned — not that the user chose to skip refreshing.
    //
    // Worse, the degrade was silent in the only way that matters: it printed
    // per-workspace noise whether or not anything was actually stale, so the
    // message carried no signal, and a genuinely stale model sailed through it
    // into a build. Museum SystemModels are exactly the fixture-mtime treadmill
    // this repo keeps getting bitten by.
    //
    // The rule is now: refresh, or fail. Absence is only tolerated when nothing
    // needs refreshing, in which case it is not a degrade at all and says
    // nothing. `blocked` collects everything that DID need the helper so one
    // error names them all rather than the user fixing them one run at a time.
    let play_launch = launch_resolver_path();
    // issue 0409 — verify the resolver BEFORE trusting its output. A binary
    // compiled from a different play_launch checkout does not fail; it produces
    // a model that is missing DATA (one predating rlm v0.1.1 drops every
    // `[[component]].params` / `params_files` projection, silently, exit 0).
    // The CLI refuses a stale SELF for the same reason (0363/0197); the
    // resolver had no such guard.
    if let Some(pl) = &play_launch {
        verify_resolver_pin(pl)?;
    }
    // Held for the whole function: dropping the TempDir removes the symlinks.
    let ament = synth_ament_prefix(scan);
    // phase-330 / issue 0392 C — includes are collected WORKSPACE-WIDE, not per
    // package. A nav2-style bringup includes its fragment from another package
    // (`$(find-pkg-share secondary_node)/launch/secondary.launch.xml`), so a
    // scan limited to sibling launch files does not see it — and the fragment's
    // own package then looks like a bringup whose single launch file is its
    // default, earning a spurious `system_model.yaml` that bakes a fragment as
    // if it were a system.
    let workspace_included: std::collections::HashSet<String> = scan
        .iter()
        .flat_map(|p| {
            let dir = p.dir.join("launch");
            let mut names = Vec::new();
            if let Ok(rd) = std::fs::read_dir(&dir) {
                for e in rd.flatten() {
                    let path = e.path();
                    if path
                        .file_name()
                        .and_then(|n| n.to_str())
                        .is_some_and(|n| n.ends_with(".launch.xml"))
                    {
                        names.extend(launch_include_names(&path));
                    }
                }
            }
            names
        })
        .collect();
    let mut blocked: Vec<String> = Vec::new();
    for pkg in scan {
        let launch_dir = pkg.dir.join("launch");
        if !launch_dir.is_dir() {
            continue;
        }
        let cfg_dir = pkg.dir.join("config");
        let system_toml = pkg.dir.join("system.toml");
        // Input mtime horizon: launch XMLs + system.toml.
        let mut newest_input: Option<std::time::SystemTime> = None;
        let mut launches: Vec<std::path::PathBuf> = Vec::new();
        if let Ok(rd) = std::fs::read_dir(&launch_dir) {
            for e in rd.flatten() {
                let p = e.path();
                if p.extension().and_then(|s| s.to_str()) == Some("xml") {
                    if let Ok(md) = p.metadata()
                        && let Ok(mt) = md.modified()
                    {
                        newest_input = Some(newest_input.map_or(mt, |c| c.max(mt)));
                    }
                    launches.push(p);
                }
            }
        }
        if launches.is_empty() {
            continue;
        }
        if let Ok(md) = system_toml.metadata()
            && let Ok(mt) = md.modified()
        {
            newest_input = Some(newest_input.map_or(mt, |c| c.max(mt)));
        }
        let stale = |model: &std::path::Path| -> bool {
            match (model.metadata().and_then(|m| m.modified()), newest_input) {
                (Ok(mm), Some(ni)) => mm < ni,
                (Err(_), _) => true,
                _ => false,
            }
        };
        // Targets: the default model always; committed variant models refresh.
        let mut targets: Vec<(std::path::PathBuf, std::path::PathBuf)> = Vec::new();
        let default_launch = {
            // `[system] default_launch` else system.launch.xml else the single file.
            let named = std::fs::read_to_string(&system_toml)
                .ok()
                .and_then(|raw| toml::from_str::<toml::Value>(&raw).ok())
                .and_then(|v| {
                    v.get("system")?
                        .get("default_launch")?
                        .as_str()
                        .map(|s| launch_dir.join(s))
                });
            named
                .filter(|p| p.is_file())
                .or_else(|| {
                    let sys = launch_dir.join("system.launch.xml");
                    sys.is_file().then_some(sys)
                })
                .or_else(|| (launches.len() == 1).then(|| launches[0].clone()))
        };
        // …and the same applies to the DEFAULT: if a package's only launch file
        // is an include fragment, it has no system to resolve at all.
        let default_launch = default_launch.filter(|dl| {
            dl.file_name()
                .and_then(|n| n.to_str())
                .is_none_or(|n| !workspace_included.contains(n))
        });
        if let Some(dl) = &default_launch {
            targets.push((dl.clone(), cfg_dir.join("system_model.yaml")));
        }
        // phase-330 W4.0 — DERIVE the plain variants from the launch tree
        // instead of from the committed `config/` scan below. Until this, the
        // committed `*_model.yaml` files were not merely the artifact, they
        // were the DECLARATION of which variants exist, so W4.a could not
        // delete them without silently stopping variant regeneration.
        //
        // The rule: every launch file that is not the default and is not
        // INCLUDED by another launch file is an entry, and gets
        // `<stem>_model.yaml`. Includes are pulled in by their parent's
        // resolve, so resolving them separately would bake a fragment as if it
        // were a system (`launch`'s `sensors.launch.xml` is exactly
        // that case).
        //
        // Binding variants (`<stem>_<binding>_model.yaml`) are NOT derivable —
        // they come from launch ARGUMENTS (`host:=robot1`) that only the
        // committed model's `meta.args` records — so they stay declarative and
        // are read from `[[model]]` below.
        let included: std::collections::HashSet<String> = launches
            .iter()
            .flat_map(|lf| launch_include_names(lf))
            .chain(workspace_included.iter().cloned())
            .collect();
        // A launch file with `[[model]]` declarations is fully described by
        // them: `multihost.launch.xml` exists only to be resolved as
        // `host:=robot1` and `host:=robot2`, and its unbound resolve is not a
        // system anyone deploys — the `all` default leaves nodes that the
        // deploy blocks then fail to place. So declarations REPLACE derivation
        // for their launch file rather than adding to it.
        let declared_launches: std::collections::HashSet<String> =
            system_toml_model_decls(&system_toml)
                .into_iter()
                .map(|d| d.launch)
                .collect();
        for lf in &launches {
            if Some(lf) == default_launch.as_ref() {
                continue;
            }
            let Some(fname) = lf.file_name().and_then(|s| s.to_str()) else {
                continue;
            };
            if included.contains(fname) || declared_launches.contains(fname) {
                continue;
            }
            let stem = fname.trim_end_matches(".launch.xml");
            if stem.is_empty() {
                continue;
            }
            let out = cfg_dir.join(format!("{stem}_model.yaml"));
            if !targets.iter().any(|(_, m)| *m == out) {
                targets.push((lf.clone(), out));
            }
        }
        // phase-330 W4.0 — `[[model]]` declarations: the binding variants,
        // moved OUT of the committed filenames and into `system.toml` so the
        // inputs carry them. `launch` + `out` + `args`.
        for decl in system_toml_model_decls(&system_toml) {
            let lf = launch_dir.join(&decl.launch);
            if !lf.is_file() {
                continue;
            }
            let out = cfg_dir.join(&decl.out);
            if !targets.iter().any(|(_, m)| *m == out) {
                targets.push((lf, out));
            }
        }
        // Committed variant models stay opt-in: only refreshed, never
        // created. Two spellings per launch `<stem>.launch.xml`:
        //   * `<stem>_model.yaml` — the plain resolve;
        //   * `<stem>_<variant>_model.yaml` — phase-326 (issue 0364): a
        //     resolve with launch-argument bindings, recorded in the model's
        //     own `meta.args` and replayed on refresh (e.g.
        //     `multihost_robot1_model.yaml` from `host:=robot1`).
        // A variant filename is claimed by the LONGEST matching launch stem,
        // so `multihost_extra_model.yaml` belongs to
        // `multihost_extra.launch.xml`, not `multihost.launch.xml`, when
        // both launch files exist.
        let stems: Vec<(String, &std::path::PathBuf)> = launches
            .iter()
            .filter_map(|lf| {
                let stem = lf
                    .file_name()
                    .and_then(|s| s.to_str())?
                    .trim_end_matches(".launch.xml")
                    .to_string();
                (!stem.is_empty()).then_some((stem, lf))
            })
            .collect();
        if let Ok(rd) = std::fs::read_dir(&cfg_dir) {
            let mut variants: Vec<std::path::PathBuf> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| {
                    p.file_name()
                        .and_then(|s| s.to_str())
                        .is_some_and(|n| n.ends_with("_model.yaml") && n != "system_model.yaml")
                })
                .collect();
            variants.sort();
            for variant in variants {
                let name = variant.file_name().and_then(|s| s.to_str()).unwrap_or("");
                let claimed = stems
                    .iter()
                    .filter(|(stem, _)| {
                        name == format!("{stem}_model.yaml")
                            || name.starts_with(&format!("{stem}_"))
                    })
                    .max_by_key(|(stem, _)| stem.len());
                if let Some((stem, lf)) = claimed {
                    // The default launch's plain resolve is
                    // `system_model.yaml`, already targeted above — its
                    // `<stem>_model.yaml` sibling would duplicate it.
                    // Binding variants (`<stem>_<v>_model.yaml`) refresh
                    // even for the default launch.
                    if Some(*lf) == default_launch.as_ref() && name == format!("{stem}_model.yaml")
                    {
                        continue;
                    }
                    targets.push(((*lf).clone(), variant));
                }
            }
        }
        // phase-330 W4 — redirect outputs to the build location when asked. The
        // target LIST is still derived from the committed `config/` scan: which
        // variants exist is declared by those files today, which is the second
        // prerequisite W4 still owes (see the phase doc). Only the destination
        // moves here.
        let targets: Vec<(std::path::PathBuf, std::path::PathBuf)> = match model_dir {
            None => targets,
            Some(dir) => targets
                .into_iter()
                .map(|(launch, model)| {
                    let name = model
                        .file_name()
                        .map(std::path::PathBuf::from)
                        .unwrap_or_else(|| std::path::PathBuf::from("system_model.yaml"));
                    // NAMESPACE BY BRINGUP. A flat output dir collides the
                    // moment a workspace has two bringups: `ws-lifecycle-cpp`
                    // has `demo_bringup` and `managed_bringup`, both producing
                    // `system_model.yaml`, and the second silently overwrote the
                    // first. It presented as "regeneration dropped a node",
                    // which is exactly the kind of false loss that would have
                    // made W4.a look unsafe for the wrong reason.
                    // Namespace by the bringup DIR name, not pkg.name — the
                    // consumer ladder (model_location) only knows the dir, and
                    // the two differ for standalone self-bringups
                    // (`talker/` vs pkg `threadx_linux_rs_talker`).
                    (
                        launch,
                        dir.join(
                            pkg.dir
                                .file_name()
                                .map(std::path::PathBuf::from)
                                .unwrap_or_else(|| std::path::PathBuf::from(&pkg.name)),
                        )
                        .join(name),
                    )
                })
                .collect(),
        };
        for (launch, model) in targets {
            // Issue 0320 — staleness is BOTH mtime AND content-addressed. The
            // mtime gate watches only `launch/*.xml` + `system.toml`, but a
            // committed model's `meta.inputs` hashes more (sibling includes, the
            // `--sched` file) and can carry a non-portable absolute path from the
            // machine that generated it. Re-hashing the recorded inputs catches
            // both the wider input set (issue 0196 class) and the 43 legacy
            // absolute-path models, which are otherwise never mtime-stale.
            let provenance = model
                .exists()
                .then(|| model_provenance_stale(&model, &pkg.dir))
                .flatten();
            if !stale(&model) && provenance.is_none() {
                continue;
            }
            let Some(pl) = &play_launch else {
                // Reached only when this model is stale or missing — `stale()`
                // already let the current ones through above.
                blocked.push(format!(
                    "  {} — {} is {} ({})",
                    pkg.name,
                    model.strip_prefix(&pkg.dir).unwrap_or(&model).display(),
                    match (&provenance, model.exists()) {
                        (Some(why), _) => why.as_str(),
                        (None, true) => "older than its inputs",
                        (None, false) => "missing",
                    },
                    launch
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("launch"),
                ));
                continue;
            };
            let dest_dir = model.parent().unwrap_or(&cfg_dir).to_path_buf();
            std::fs::create_dir_all(&dest_dir)
                .wrap_err_with(|| format!("sync: create {}", dest_dir.display()))?;
            let mut cmd = std::process::Command::new(pl);
            if let Some((_, prefix)) = &ament {
                cmd.env("AMENT_PREFIX_PATH", prefix);
            }
            cmd.arg(&launch);
            // phase-326 (issue 0364) — replay the exact binding the committed
            // model records, so a variant model refreshes as ITSELF rather
            // than as the default configuration.
            // phase-330 W4.0 — a `[[model]]` declaration is authoritative; the
            // committed model's own `meta.args` is the FALLBACK, kept so a
            // tree with no declarations still refreshes its variants exactly as
            // before. Once W4.a deletes the committed copies, only the
            // declaration remains — which is the point.
            let declared_args: Vec<(String, String)> = model
                .file_name()
                .and_then(|s| s.to_str())
                .and_then(|name| {
                    system_toml_model_decls(&system_toml)
                        .into_iter()
                        .find(|d| d.out == name)
                        .map(|d| d.args)
                })
                .unwrap_or_default();
            let args = if declared_args.is_empty() {
                model_recorded_args(&model)
            } else {
                declared_args
            };
            for (k, v) in args {
                cmd.arg(format!("{k}:={v}"));
            }
            // Issue 0320 — state the bringup package root explicitly so
            // `meta.inputs[].path` are recorded relative to it structurally,
            // rather than the resolver inferring it as the launch file's
            // grandparent (which emits absolute paths for a non-standard layout).
            cmd.arg("--bringup-root").arg(&pkg.dir);
            if system_toml.is_file() {
                cmd.arg("--system").arg(&system_toml);
            }
            // phase-330 W5.a — the issue-0380 dim-loss refusal is RETIRED: it
            // protected a COMMITTED model from destructive regeneration, and
            // committed models no longer exist (W4.a). Dims live in the
            // resolver INPUTS (system.toml), so a re-resolve cannot lose them;
            // the staged write remains for atomicity only.
            let staged = model.with_extension("yaml.resolving");
            cmd.arg("-o").arg(&staged);
            let out = cmd
                .output()
                .wrap_err_with(|| format!("sync: spawn nros-launch-resolve for {}", pkg.name))?;
            if !out.status.success() {
                let _ = std::fs::remove_file(&staged);
                eyre::bail!(
                    "sync: nros-launch-resolve failed for `{}` ({}):\n{}",
                    pkg.name,
                    launch.display(),
                    String::from_utf8_lossy(&out.stderr),
                );
            }
            // issue 0409 direction 3 — assert the resolver actually PERFORMED the
            // params projection before promoting the staged file. Everything
            // upstream of here verifies the TOOL; this verifies the OUTPUT, which
            // is the property that was silently lost.
            verify_params_projected(&staged, &system_toml).wrap_err_with(|| {
                format!("sync: resolved model for `{}` is missing data", pkg.name)
            })?;
            // Issue 0427 — stamp the resolver pin so a later resolver change makes
            // this model stale (`model_provenance_stale`) instead of reading fresh
            // forever on unchanged inputs.
            stamp_resolver_pin(&staged)
                .wrap_err_with(|| format!("sync: stamp resolver pin for `{}`", pkg.name))?;
            std::fs::rename(&staged, &model)
                .wrap_err_with(|| format!("sync: commit resolved model {}", model.display()))?;
            if verbose {
                println!("sync: resolved {}", model.display());
            } else {
                println!(
                    "sync: resolved {} → {}",
                    launch
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("launch"),
                    model.strip_prefix(&pkg.dir).unwrap_or(&model).display()
                );
            }
        }
    }
    if !blocked.is_empty() {
        eyre::bail!(
            "sync: {} SystemModel(s) need resolving but `nros-launch-resolve` \
             is not next to the `nros` binary:\n{}\n\n\
             Build it:  ./scripts/bootstrap.sh   (contributors: just setup-launch-resolve)\n\
             (If the submodule is missing:  git submodule update --init \
             packages/cli/third-party/play_launch)\n\n\
             Refusing to continue with stale models — a museum SystemModel builds \
             clean and then places nodes wrong at runtime.",
            blocked.len(),
            blocked.join("\n"),
        );
    }
    Ok(())
}

/// Locate `nros-launch-resolve` (issue 0285).
///
/// Mirrors `nros_cli_bin()` in `scripts/build/cargo.sh` — the repo's existing
/// SSoT for finding the CLI — with ONE deliberate omission: no `$PATH` step.
/// A PATH lookup is precisely the bug this fixes, since an unrelated ROS 2
/// `play_launch` won that race. Resolution order:
///
///   1. `$NROS_LAUNCH_RESOLVE` — explicit override, the twin of `$NROS_CLI`
///      (packaging, CI, and tests that ship the helper elsewhere);
///   2. a sibling of the running `nros` — the installed layout;
///   3. `$NROS_REPO_DIR/packages/cli/nros-launch-resolve/target/release/…`,
///      then the same path derived by walking up from the running binary —
///      the per-checkout build, which `cargo.sh` also prefers so each worktree
///      carries its own tools with no cross-tree skew.
///
/// issue 0409 — refuse a `nros-launch-resolve` built from a different
/// `play_launch` checkout than this `nros`.
///
/// Both binaries stamp the submodule commit they compiled in (`build.rs`), so a
/// mismatch means the resolver's vendored layer-2 differs from the one this CLI
/// was built against — and that difference shows up as MISSING CONTENT in
/// generated models, not as an error.
///
/// Unverifiable is not the same as wrong. When either side stamped `unknown`
/// (a tarball build, a vendored drop with no git) or the binary does not
/// support `--version` (an older resolver predating this check), the pin cannot
/// be compared and the run proceeds: refusing there would break legitimate
/// installs to catch a dev-tree hazard.
fn verify_resolver_pin(resolver: &std::path::Path) -> Result<()> {
    const OURS: &str = env!("NROS_PLAY_LAUNCH_SHA");
    // The `--version` probe runs even when OURS is `unknown`. It answers TWO
    // questions and only one of them is about the pin: the other is "can this
    // binary run at all", which matters on every build.
    let Ok(out) = std::process::Command::new(resolver)
        .arg("--version")
        .output()
    else {
        return Ok(()); // could not spawn; the caller's own not-found check reports that
    };
    if !out.status.success() {
        // A resolver that CANNOT LOAD is not a resolver that predates
        // `--version`, and treating them alike is why a missing interpreter
        // surfaced as a raw loader message N packages later:
        //
        //   sync: nros-launch-resolve failed for `pkg` (bringup.launch.py):
        //   …: error while loading shared libraries: libpython3.10.so.1.0:
        //   cannot open shared object file: No such file or directory
        //
        // The resolver embeds CPython (pyo3) because `.launch.py` files ARE
        // Python and ROS compatibility requires executing them, so `libpython`
        // is a hard DT_NEEDED — the loader fails before `main`, and the binary
        // itself can never report it. This is the one place that can.
        let stderr = String::from_utf8_lossy(&out.stderr);
        if stderr.contains("error while loading shared libraries") {
            let missing = stderr
                .split("error while loading shared libraries: ")
                .nth(1)
                .and_then(|s| s.split(':').next())
                .unwrap_or("libpython")
                .trim()
                .to_string();
            let host = std::process::Command::new("python3")
                .arg("-c")
                .arg("import sys;print('%d.%d' % sys.version_info[:2])")
                .output()
                .ok()
                .filter(|o| o.status.success())
                .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
                .filter(|s| !s.is_empty());
            bail!(
                "sync: `{}` cannot start — it needs `{}`, which this host does not have.\n\
                 \n\
                 nano-ros supports Python launch files because they are part of the ROS 2\n\
                 standard, so the resolver embeds CPython and links one specific\n\
                 `libpython`. That link is resolved by the loader BEFORE the program runs,\n\
                 which is why the message above comes from `ld.so` and not from us.\n\
                 \n\
                 {}\n\
                 Fix it either way:\n\
                 \n    just setup-launch-resolve     # rebuild against THIS host's interpreter\n\
                 \n  or install the interpreter it was built for ({}).\n\
                 \n\
                 XML and YAML launch files need no interpreter; only `.launch.py` does.",
                resolver.display(),
                missing,
                match &host {
                    Some(v) => format!(
                        "This host's `python3` is {v}; the resolver was built against a different one."
                    ),
                    None => "No `python3` was found on this host at all.".to_string(),
                },
                missing,
            );
        }
        return Ok(()); // predates `--version`; nothing to compare
    }
    if OURS == "unknown" {
        return Ok(());
    }
    let text = String::from_utf8_lossy(&out.stdout);
    let Some(theirs) = text
        .split_once("play_launch ")
        .map(|(_, rest)| rest.trim_end_matches([')', '\n', ' ']).trim())
    else {
        return Ok(());
    };
    if theirs == "unknown" || theirs == OURS {
        return Ok(());
    }
    bail!(
        "sync: `{}` was built from play_launch {} but this `nros` was built from {}.\n\
         \n\
         A resolver from a different layer-2 checkout does not fail — it writes models that are\n\
         MISSING DATA (one predating rlm v0.1.1 drops every `params` / `params_files`\n\
         projection, silently). Rebuild it so both agree:\n\
         \n    ./scripts/bootstrap.sh      (contributors: just setup-launch-resolve)\n\
         \n(issue 0409)",
        resolver.display(),
        &theirs[..theirs.len().min(12)],
        &OURS[..OURS.len().min(12)],
    )
}

/// The helper is its OWN cargo workspace, so its binary is under
/// `nros-launch-resolve/target/release/`, not beside `nros` in
/// `packages/cli/target/release/`.
fn launch_resolver_path() -> Option<std::path::PathBuf> {
    resolver_beside(&std::env::current_exe().ok()?)
}

/// The lookup itself, parameterised on the `nros` binary's own path so it can
/// be tested without spawning anything.
///
/// Two locations, both derived from `exe` — never `$PATH`:
/// 1. a sibling (installed layout: `nros` and the helper side by side);
/// 2. `../../nros-launch-resolve/target/release/` (in-tree: the helper is its
///    own cargo workspace, so it does NOT land in `packages/cli/target/`).
const LAUNCH_RESOLVER: &str = "nros-launch-resolve";

fn resolver_beside(exe: &std::path::Path) -> Option<std::path::PathBuf> {
    resolver_from(
        exe,
        std::env::var_os("NROS_LAUNCH_RESOLVE").map(std::path::PathBuf::from),
        std::env::var_os("NROS_REPO_DIR").map(std::path::PathBuf::from),
    )
}

/// The search itself, pure in its inputs.
///
/// Taking the two env values as arguments rather than reading them keeps this
/// hermetic: the tests below would otherwise race each other through the
/// process-wide environment, and would also see a real `$NROS_REPO_DIR` from
/// the developer's shell.
fn resolver_from(
    exe: &std::path::Path,
    explicit: Option<std::path::PathBuf>,
    repo_dir: Option<std::path::PathBuf>,
) -> Option<std::path::PathBuf> {
    // 1. Explicit override, mirroring `$NROS_CLI`. A non-existent override
    //    falls through rather than failing: the caller degrades to the
    //    committed model, and a stale env var must not be harder to diagnose
    //    than a missing tool.
    if let Some(p) = explicit
        && p.is_file()
    {
        return Some(p);
    }

    let dir = exe.parent()?;

    // 2. Installed layout — beside the CLI we shipped.
    let sibling = dir.join(LAUNCH_RESOLVER);
    if sibling.is_file() {
        return Some(sibling);
    }

    // 3. Per-checkout build, preferred by cargo.sh for the same reason: each
    //    worktree carries its own tools, with no cross-tree skew.
    let in_checkout = |root: &std::path::Path| {
        root.join("packages")
            .join("cli")
            .join(LAUNCH_RESOLVER)
            .join("target")
            .join("release")
            .join(LAUNCH_RESOLVER)
    };
    if let Some(root) = repo_dir {
        let p = in_checkout(&root);
        if p.is_file() {
            return Some(p);
        }
    }
    // `dir` is <repo>/packages/cli/target/release, so <repo> is four
    // ancestors up (target, cli, packages, repo).
    dir.ancestors()
        .nth(4)
        .map(in_checkout)
        .filter(|p| p.is_file())
}

/// phase-315 W1 — write one selection facade per ENTRY package.
///
/// The bringup owns the declaration, the entry consumes it, and the two are
/// different packages, so this needs both: it finds the workspace's
/// `system.toml` (the bringup) and then every package carrying
/// `[package.metadata.nros.entry]`.
///
/// A workspace with no `system.toml` has nothing to derive from and is left
/// alone — that is the STANDALONE shape, where the build command is the
/// selector (`cargo build --features …`, the twin of C++'s `-DNANO_ROS_RMW=…`)
/// and a facade would have no input. See phase-315 W3.
fn generate_facade_crates(
    ws_root: &std::path::Path,
    scan: &[WsPkg],
    build_root: &std::path::Path,
    verbose: bool,
) -> Result<()> {
    // The bringup: the package that declares the system. More than one is a
    // multi-system workspace, which the facade shape does not yet model — say
    // so rather than silently picking the first.
    let bringups: Vec<&WsPkg> = scan
        .iter()
        .filter(|p| p.dir.join("system.toml").is_file())
        .collect();
    let bringup = match bringups.as_slice() {
        [] => return Ok(()),
        [one] => *one,
        many => {
            eprintln!(
                "sync: {} bringups declare a system ({}); selection facades \
                 are not generated for multi-system workspaces (phase-315 W1 \
                 models one declaration per workspace). Entry manifests keep \
                 their hand-written features.",
                many.len(),
                many.iter()
                    .map(|p| p.name.as_str())
                    .collect::<Vec<_>>()
                    .join(", "),
            );
            return Ok(());
        }
    };

    let system_toml = bringup.dir.join("system.toml");
    let raw = std::fs::read_to_string(&system_toml)
        .wrap_err_with(|| format!("sync: read {}", system_toml.display()))?;
    let sys: crate::orchestration::cargo_metadata_schema::SystemToml =
        toml::from_str(&raw).wrap_err_with(|| format!("sync: parse {}", system_toml.display()))?;

    // Entry packages come from CARGO's member list, not from `scan`.
    //
    // `scan` is ament-driven: a package enters it by having a `package.xml`.
    // Nine workspace entries do not have one — they are cargo workspace members
    // and nothing else, which is legal (the workspace ROOT is their patch
    // authority, so the rest of sync works on them). Keying facade generation
    // off `scan` silently skipped exactly those nine, and the skip was
    // invisible: sync succeeded, and the entries kept their hand-written
    // features, which is the state that looks correct.
    //
    // Cargo's `members` list is the truth for "what is in this workspace" here,
    // because the facade's whole mechanism is cargo feature unification.
    let mut candidates: Vec<(String, PathBuf)> = scan
        .iter()
        .filter(|p| p.is_rust_pkg)
        .map(|p| (p.name.clone(), p.dir.clone()))
        .collect();
    for dir in cargo_workspace_members(ws_root) {
        if !candidates.iter().any(|(_, d)| *d == dir) {
            let name = dir
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or_default()
                .to_string();
            candidates.push((name, dir));
        }
    }

    let facade_root = build_root.join("nros-selection");
    for (pkg_name, pkg_dir) in &candidates {
        // The CARGO manifest — NOT `WsPkg::manifest`, which is the ament
        // `package.xml`, and which the cargo-only members do not have at all.
        let cargo_toml = pkg_dir.join("Cargo.toml");
        if !cargo_toml.is_file() {
            continue;
        }
        let Some(f) = crate::orchestration::facade::write_facade(
            pkg_name,
            pkg_dir,
            &cargo_toml,
            &sys,
            &facade_root,
        )
        .wrap_err_with(|| format!("sync: facade for {pkg_name}"))?
        else {
            continue;
        };
        if f.changed || verbose {
            println!(
                "sync: selection facade {} → nros[{}] {}",
                f.entry,
                f.nros_features.join(", "),
                if f.board_features.is_empty() {
                    String::new()
                } else {
                    format!("board[{}]", f.board_features.join(", "))
                },
            );
        }
    }
    Ok(())
}

/// Cargo workspace members of `ws_root`, as absolute directories.
///
/// Deliberately simple: `members` entries are literal relative paths in every
/// nano-ros example workspace. Glob members (`src/*`) are expanded, since cargo
/// allows them and one of these workspaces could grow one; anything else is
/// skipped rather than guessed at.
fn cargo_workspace_members(ws_root: &std::path::Path) -> Vec<PathBuf> {
    let Ok(raw) = std::fs::read_to_string(ws_root.join("Cargo.toml")) else {
        return Vec::new();
    };
    let Ok(v) = toml::from_str::<toml::Value>(&raw) else {
        return Vec::new();
    };
    let Some(members) = v
        .get("workspace")
        .and_then(|w| w.get("members"))
        .and_then(|m| m.as_array())
    else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for m in members.iter().filter_map(|m| m.as_str()) {
        if let Some(prefix) = m.strip_suffix("/*") {
            if let Ok(rd) = std::fs::read_dir(ws_root.join(prefix)) {
                out.extend(rd.flatten().map(|e| e.path()).filter(|p| p.is_dir()));
            }
        } else {
            let p = ws_root.join(m);
            if p.is_dir() {
                out.push(p);
            }
        }
    }
    out
}

fn generate_bridge_configs(
    ws_root: &std::path::Path,
    scan: &[WsPkg],
    build_root: &std::path::Path,
    verbose: bool,
) -> Result<()> {
    for pkg in scan {
        let system_toml = pkg.dir.join("system.toml");
        if !system_toml.is_file() {
            continue;
        }
        let raw = std::fs::read_to_string(&system_toml).unwrap_or_default();
        let has_bridge = toml::from_str::<toml::Value>(&raw)
            .ok()
            .and_then(|v| {
                v.get("bridge")
                    .and_then(|b| b.as_array())
                    .map(|a| !a.is_empty())
            })
            .unwrap_or(false);
        let dest = pkg.dir.join("nros-bridge.toml");
        if !has_bridge {
            continue;
        }

        // R-code — plan from the bringup's committed SystemModel when
        // present (every in-tree bridge workspace has one); launch-synth
        // resolution survives only for modelless bringups. Both temp guards
        // (synth XML / synthesized record) live in `_guards` through
        // plan_system.
        // phase-330 W4.0b — bridge bringups resolve through the shared order too.
        let model_path = crate::orchestration::model_location::resolve_model_path(
            &pkg.dir,
            "config/system_model.yaml",
        );
        let mut _guard_record = None;
        let (plan_launch_file, plan_record_file) = if model_path.exists() {
            let model = crate::orchestration::model_ingest::load_model(&model_path)?;
            let record = crate::orchestration::model_ingest::plan_record_from_model(&model);
            let tmp = tempfile::NamedTempFile::new()?;
            std::fs::write(tmp.path(), serde_json::to_string_pretty(&record)?)?;
            let rec_path = tmp.path().to_path_buf();
            _guard_record = Some(tmp);
            (model_path.clone(), Some(rec_path))
        } else {
            // R-code.1 — the launch-synth fallback is deleted; a bridge
            // bringup declares system semantics, so it must resolve a model.
            eyre::bail!(
                "sync: bridge bringup `{}` has no committed SystemModel \
                 (config/system_model.yaml) — the launch-synth fallback was \
                 removed (phase-296 R4); resolve one with `play_launch \
                 resolve … --system {}/system.toml`",
                pkg.name,
                pkg.dir.display()
            );
        };
        let output = crate::orchestration::planner::plan_system(
            crate::orchestration::planner::PlanOptions {
                system_pkg: pkg.name.clone(),
                workspace_root: ws_root.to_path_buf(),
                launch_file: plan_launch_file,
                record_file: plan_record_file,
                out_root: build_root.join(&pkg.name).join("nros-bridge-plan"),
                metadata_files: Vec::new(),
                manifest_files: Vec::new(),
                launch_args: Vec::new(),
                rmw: None,
                image: None,
                // Issue 0951 — this bridge plan runs inside the user's
                // workspace, not a nano-ros checkout, so the resolver's own
                // ladder (NROS_REPO_DIR, then an autodetect walk) is the right
                // answer rather than a path invented here.
                nano_ros_path: None,
            },
        )
        .wrap_err_with(|| format!("sync: plan bridge bringup {}", pkg.name))?;

        let plan_json = std::fs::read_to_string(&output.plan_path)?;
        let plan: crate::orchestration::plan::NrosPlan = serde_json::from_str(&plan_json)
            .wrap_err_with(|| format!("sync: parse plan for bridge bringup {}", pkg.name))?;

        match crate::orchestration::bridge_gen::render_bridge_runtime_config(&plan, ws_root) {
            Some(cfg) => {
                // Issue 0498 — sync-owned and read by the runtime, so atomic.
                atomic_write(&dest, &cfg)
                    .wrap_err_with(|| format!("sync: write {}", dest.display()))?;
                if verbose {
                    println!("sync: wrote {}", dest.display());
                }
            }
            // A `[[bridge]]` whose plan carried no resolvable bridge — drop any
            // stale file so the entry doesn't boot an outdated config.
            None => {
                let _ = std::fs::remove_file(&dest);
            }
        }
    }
    Ok(())
}

/// Join `rel` onto `base`, folding `.` and `..` textually.
///
/// `Path::join` + `is_file()` would walk the real filesystem, which fails when
/// an intermediate component does not exist yet. Nothing here touches disk.
fn lexically_join(base: &Path, rel: &Path) -> PathBuf {
    let mut out = base.to_path_buf();
    for part in rel.components() {
        match part {
            std::path::Component::ParentDir => {
                out.pop();
            }
            std::path::Component::CurDir => {}
            other => out.push(other.as_os_str()),
        }
    }
    out
}

pub fn run_sync(args: SyncArgs) -> Result<()> {
    // Captured before `args.workspace` is moved below; the scope flags are read
    // near the END of sync, and a partial move would otherwise force the whole
    // struct to be cloned.
    let base_paths = args.base_paths.clone();
    let nano_ros_root = args.nano_ros_root.clone();
    // phase-308 W1 — captured up front: `args.workspace` is moved just below,
    // and the C/C++-only early return still needs the probe's nano-ros path.
    let nano_ros_for_probes = nano_ros_path_for(&args);
    let ws_root: PathBuf = match args.workspace {
        Some(p) => std::fs::canonicalize(&p).wrap_err_with(|| format!("sync: {}", p.display()))?,
        None => std::env::current_dir()?,
    };
    // phase-429 W2 — the codegen version guard, before the first write. `sync`
    // emits generated msg crates and rewrites every consumer's
    // `.cargo/config.toml`; a mismatched emitter's output is what the user then
    // has to unpick, so the refusal belongs here rather than after.
    //
    // Anchor: a nano-ros checkout named on the COMMAND LINE wins (it IS the
    // runtime this workspace links), then the workspace itself, which the
    // resolver walks up from — the arm that reaches a C/C++-only workspace
    // with no `Cargo.lock`. Deliberately NOT `nano_ros_path_for`, whose
    // `NROS_REPO_DIR` fallback is ambient after `source activate.sh`: that
    // would measure every workspace against whichever checkout the shell was
    // opened in. `runtime_root` still consults the env, one rung lower, for a
    // consumer that is inside no tree at all.
    {
        let anchor = nano_ros_root
            .clone()
            .or_else(|| args.nano_ros_path.clone())
            .unwrap_or_else(|| ws_root.clone());
        crate::abi_guard::check_workspace(&anchor, crate::abi_guard::Verb::Sync)?;
    }
    // Two layouts supported:
    //  * `src/`-based: workspace root has src/, src/<pkg>/ subdirs (colcon
    //    standard).
    //  * Single-pkg: workspace root IS the pkg dir (package.xml at root).
    //    Common for ported standalone examples (`examples/native/rust/talker`).
    // Heuristic: colcon-style layout iff `src/` exists AND has at least one
    // immediate subdir with `package.xml`. Falls through to single-pkg mode
    // when the workspace root itself carries `package.xml` (the standalone
    // example shape; `src/` may exist as the cargo source dir).
    let colcon_layout = ws_root.join("src").is_dir() && has_pkg_subdir(&ws_root.join("src"));
    let single_pkg_mode = !colcon_layout && ws_root.join("package.xml").is_file();
    let src_root = if colcon_layout {
        ws_root.join("src")
    } else if single_pkg_mode {
        ws_root.clone()
    } else {
        bail!(
            "sync: no `src/<pkg>/package.xml` and no `package.xml` at root \
             under {} — expected colcon-style workspace or single-pkg dir",
            ws_root.display()
        );
    };
    let build_root = if args.build_dir.is_absolute() {
        args.build_dir.clone()
    } else {
        ws_root.join(&args.build_dir)
    };

    let mut scan = Vec::new();
    if single_pkg_mode {
        scan_one_pkg_dir(&src_root, &mut scan)?;
    } else {
        scan_workspace(&src_root, &mut scan)?;
    }
    if scan.is_empty() {
        println!("sync: no pkgs under {}", src_root.display());
        return Ok(());
    }
    // Phase 212.M-F.21 — Rust consumer's transitive msg deps via path-deps.
    // The pkg.xml `<*depend>` tags drive AMENT codegen + patch table,
    // but Entry pkgs typically don't list msg deps directly — they
    // inherit them through a path-dep on a Component pkg. Walk each
    // Rust consumer's `Cargo.toml [dependencies]`, resolve path-deps
    // against the scan, and union the dependent pkg's `deps` in. The
    // patch authority for the Entry pkg then carries every msg patch
    // the transitive build needs.
    augment_rust_consumer_deps_via_path_deps(&mut scan)?;
    let msg_pkgs: Vec<&WsPkg> = scan.iter().filter(|p| p.is_msg_pkg).collect();
    let topo = topo_sort_msg_pkgs(&msg_pkgs)?;

    if args.verbose || args.dry_run {
        println!(
            "sync: scanned {} pkgs ({} msg, {} rust) under {}",
            scan.len(),
            msg_pkgs.len(),
            scan.iter().filter(|p| p.is_rust_pkg).count(),
            src_root.display()
        );
        println!("sync: topo order: {topo:?}");
    }

    if args.check {
        return check_freshness(&ws_root, &build_root, &scan, &topo);
    }

    if args.dry_run {
        for name in &topo {
            let pkg = scan.iter().find(|p| &p.name == name).unwrap();
            let out = build_root.join(name);
            println!(
                "sync: WOULD codegen {} from {} → {}",
                name,
                pkg.manifest.display(),
                out.display()
            );
        }
        return Ok(());
    }

    // Captured before `args` is partially moved below (the nano_ros_path take).
    let no_metadata = args.no_metadata;
    let verbose = args.verbose;

    let edition = resolve_sync_edition(args.ros_edition.as_deref(), &ws_root)?;

    // Track every pkg we generate so a later iteration (or AMENT-dep walk)
    // skips already-emitted ones. Keyed by pkg name.
    let mut emitted: HashSet<String> = HashSet::new();

    for name in &topo {
        let pkg = scan.iter().find(|p| &p.name == name).unwrap();
        // First materialize any AMENT-resolved cross-deps so the workspace
        // pkg's deps closure exists in build/ too. Skips workspace deps
        // (those are handled by topo order itself).
        codegen_ament_deps_for(
            &pkg.deps,
            &scan,
            &build_root,
            edition,
            &mut emitted,
            args.verbose,
        )?;
        // Now generate the workspace pkg itself directly from its dir.
        if !emitted.contains(name) {
            codegen_workspace_pkg(pkg, &build_root, edition, args.verbose)?;
            emitted.insert(name.clone());
        }
    }
    // Also generate AMENT deps for every Rust consumer (pkg.xml deps).
    let rust_consumers: Vec<&WsPkg> = scan.iter().filter(|p| p.needs_patch_authority()).collect();
    for c in &rust_consumers {
        codegen_ament_deps_for(
            &c.deps,
            &scan,
            &build_root,
            edition,
            &mut emitted,
            args.verbose,
        )?;
    }

    // phase-267 W1c/C3e — for each bringup declaring a `[[bridge]]`, plan it
    // (topic names→types resolve from the node pkgs' synthetic `publishes`
    // metadata, no build) and write `<bringup>/nros-bridge.toml` — the file the
    // entry's `nros_bridge::run_from_config` consumes at runtime.
    // R-code UX — resolve/refresh each bringup's committed SystemModel first
    // (the canonical input; bridge planning below consumes it).
    // phase-330 W4.0b — ONE knob for both halves. Consumers already read
    // `NROS_MODEL_DIR` (W3.b's search order), so sync honours the same env when
    // `--model-dir` is absent. Symmetric by construction: the variable that
    // says where models are READ is the variable that says where they are
    // WRITTEN. The alternative was threading `--model-dir` through 15 `just`
    // call sites, i.e. a second spelling of the same fact in 15 places.
    let model_dir = args.model_dir.clone().or_else(|| {
        std::env::var_os("NROS_MODEL_DIR")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    });
    // phase-330 W4.a — the DEFAULT flips to the workspace build root: the
    // model is a build artifact (maintainer decision, W7), so with neither
    // `--model-dir` nor `NROS_MODEL_DIR` given, sync writes
    // `<ws>/build/nros/models/<bringup>/<model>` — the same location the
    // consumer ladder's workspace-build-root rung reads. Committed
    // `config/*.yaml` are deleted; nothing writes into the source tree.
    let model_dir = model_dir.or_else(|| Some(ws_root.join("build").join("nros").join("models")));
    resolve_system_models(&scan, args.verbose, model_dir.as_deref())?;
    generate_bridge_configs(&ws_root, &scan, &build_root, args.verbose)?;
    generate_facade_crates(&ws_root, &scan, &build_root, args.verbose)?;

    // phase-348 W3 — the provider index, beside the models under
    // `<ws>/build/nros/`. Written HERE rather than after the Rust-consumer
    // work below, because that block returns early for a C/C++-only workspace:
    // placing it later would silently skip the index for exactly the
    // workspaces that have no cargo path to fall back on.
    //
    // A failure to write is a WARNING, not fatal. The index is a cache —
    // everything in it is rederivable by rescanning — so an unwritable build
    // dir must not take down a sync that otherwise succeeded.
    if args.no_provider_index {
        if args.verbose {
            println!("sync: provider index skipped (--no-provider-index)");
        }
    } else {
        write_provider_index_with(
            &ws_root,
            &provider_roots_for_sync(&ws_root, &base_paths, nano_ros_root.as_deref())?,
            args.verbose,
        );
    }

    if rust_consumers.is_empty() {
        println!("sync: no Rust consumer pkgs — patch tables not written.");
        // phase-308 W1 — a C/C++-only workspace has no Rust consumers, but it
        // DOES have components to probe. Returning here skipped the metadata
        // refresh entirely for exactly the workspaces the C/C++ producer
        // exists to serve.
        refresh_source_metadata(&ws_root, nano_ros_for_probes, no_metadata, verbose)?;
        println!("sync: done.");
        return Ok(());
    }

    // Group consumers by patch authority. Cargo workspace covers many
    // consumers via one umbrella; standalone pkgs are their own authority.
    let all_emitted: Vec<String> = {
        let mut v: Vec<String> = emitted.iter().cloned().collect();
        v.sort();
        v
    };
    let mut authority_to_pkgs: HashMap<PathBuf, Vec<String>> = HashMap::new();
    // phase-327 W5 — the deps each authority's consumers still DECLARE, so
    // the writer can tell a legitimately-removed dep from one this run
    // failed to generate (the narrowing guard in `write_patch_block`).
    let mut authority_to_requested: HashMap<PathBuf, HashSet<String>> = HashMap::new();
    let mut authority_to_leaves: HashMap<PathBuf, Vec<PathBuf>> = HashMap::new();
    for c in &rust_consumers {
        let authority = find_patch_authority(&c.dir, &ws_root)?;
        // phase-333 W1 — only deps this consumer declares BY REGISTRY NAME can be
        // stranded by a narrower patch block; a path dep resolves without one.
        // `c.deps` comes from package.xml `<depend>` rows, which name every
        // message package the leaf uses whether or not its Cargo.toml resolves
        // them via the registry, so intersect the two.
        let registry_named: HashSet<String> = std::fs::read_to_string(c.dir.join("Cargo.toml"))
            .map(|body| registry_style_dep_names(&body).into_iter().collect())
            .unwrap_or_default();
        authority_to_requested
            .entry(authority.clone())
            .or_default()
            .extend(
                c.deps
                    .iter()
                    .filter(|d| registry_named.contains(*d))
                    .cloned(),
            );
        // Workspace mode keeps the locked shared-root topology (`3f07dd9f7`):
        // every consumer's authority carries the full emitted set. Single-pkg
        // mode is dependency-aware — only the msg crates this consumer
        // transitively depends on (its `<depend>` closure), so a node's
        // unconsumed self-codegen crate never lands a broken patch entry.
        let pkgs_for: Vec<String> = if single_pkg_mode {
            emitted_msg_dep_closure(&c.deps, &all_emitted, &build_root)
        } else {
            all_emitted.clone()
        };
        authority_to_pkgs
            .entry(authority.clone())
            .or_default()
            .extend(pkgs_for);
        // phase-351 W3 — which leaves answer to this authority, so its
        // `[patch.crates-io]` can carry the rows their BOARDS declare. A
        // workspace's authority is its ROOT, and one workspace can hold entries
        // for several boards, so this is a set union, not a single board.
        authority_to_leaves
            .entry(authority)
            .or_default()
            .push(c.dir.clone());
    }
    let nano_ros_path = args
        .nano_ros_path
        .or_else(|| std::env::var_os("NROS_REPO_DIR").map(PathBuf::from))
        .or_else(|| autodetect_nano_ros_path(&ws_root));

    // Phase 220.E — collect the union of `nros-*` (+ `nros` + `cyclonedds-sys`)
    // registry-style deps across every Rust consumer pointing at this
    // authority. Each authority gets a single patch block; if any
    // consumer references `nros-rmw-zenoh = "*"`, the authority's
    // block must carry the matching path entry — otherwise cargo
    // can't resolve the dep at all (it'll search crates.io and fail).
    let mut authority_to_extra: HashMap<PathBuf, Vec<String>> = HashMap::new();
    for c in &rust_consumers {
        let authority = find_patch_authority(&c.dir, &ws_root)?;
        let cargo_toml = c.dir.join("Cargo.toml");
        let extras = match std::fs::read_to_string(&cargo_toml) {
            Ok(body) => extract_consumer_registry_nros_deps(&body),
            Err(_) => Vec::new(),
        };
        authority_to_extra
            .entry(authority)
            .or_default()
            .extend(extras);
    }

    // Phase 287 W9 (option E) — regenerate the central patch file once per
    // sync; every authority's config then reaches the universal-trio patches
    // through a single `include` line instead of per-leaf relative paths.
    let central_patch: Option<PathBuf> = match nano_ros_path.as_deref() {
        Some(nrp) => Some(write_central_patch_file(nrp)?),
        None => None,
    };
    // #272 — the central patch is reached via an `include = [...]` config key
    // that was NIGHTLY-ONLY before cargo 1.93 (stabilized there). On an older
    // build toolchain cargo silently drops the include and the build dies with
    // an unexplained `no matching package named 'nros'`. Warn LOUDLY once when
    // the workspace's effective cargo predates 1.93 — the reachability check
    // above only catches a missing FILE, not a cargo too old to read it.
    if central_patch.is_some() {
        warn_if_cargo_predates_config_include(&ws_root);
    }

    for (authority, pkgs) in authority_to_pkgs {
        let mut unique = pkgs;
        unique.sort();
        unique.dedup();
        let mut extras = authority_to_extra.remove(&authority).unwrap_or_default();
        extras.sort();
        extras.dedup();
        let requested = authority_to_requested
            .remove(&authority)
            .unwrap_or_default();
        let leaf_dirs = authority_to_leaves.remove(&authority).unwrap_or_default();
        write_patch_block(
            &authority,
            &build_root,
            &unique,
            nano_ros_path.as_deref(),
            &extras,
            central_patch.as_deref(),
            &requested,
            &leaf_dirs,
        )?;
    }

    // phase-341 W2 — project each Entry leaf's board `cargo_config` into its
    // `.cargo/nros-board.toml`. AFTER the patch pass: both writers maintain the
    // same `include` array, each evicting only its OWN entry by basename, and
    // running last means this one reads (and re-adds onto) what the patch pass
    // just wrote rather than racing it.
    project_board_configs(&rust_consumers, nano_ros_path.as_deref(), verbose)?;

    refresh_source_metadata(&ws_root, nano_ros_path.clone(), no_metadata, verbose)?;

    println!("sync: done.");
    Ok(())
}

/// phase-308 W1 — resolve the nano-ros checkout the metadata probes build
/// against. Mirrors the resolution the patch-table path already does.
fn nano_ros_path_for(args: &SyncArgs) -> Option<PathBuf> {
    args.nano_ros_path
        .clone()
        .or_else(|| std::env::var_os("NROS_REPO_DIR").map(PathBuf::from))
}
/// phase-307 W2 — the producer trigger.
///
/// Runs LAST, and the order is load-bearing: the metadata harness compiles the
/// Node pkg for real, so the generated interface crates must already exist
/// (codegen, above) and the `[patch.crates-io]` tables that redirect
/// `example_interfaces = "*"` at them must already be written (immediately
/// above). Running this any earlier fails to resolve the interface deps.
fn refresh_source_metadata(
    ws_root: &Path,
    nano_ros_path: Option<PathBuf>,
    no_metadata: bool,
    verbose: bool,
) -> Result<()> {
    if no_metadata {
        return Ok(());
    }
    let report = crate::orchestration::metadata_refresh::refresh_stale_sidecars(
        ws_root,
        nano_ros_path.as_deref(),
        verbose,
    )?;
    if report.total() == 0 && report.unsupported.is_empty() {
        return Ok(());
    }
    println!(
        "sync: source metadata — {} rebuilt, {} already current",
        report.rebuilt.len(),
        report.fresh.len()
    );
    // Never silent: a component with no producer is a component whose entity
    // count a bake cannot know, and that is exactly how issue 0257's executor
    // ran out of callback slots at boot.
    for what in &report.unsupported {
        println!("sync: source metadata — no producer for {what}");
    }
    Ok(())
}

fn parse_edition(s: &str) -> Result<RosEdition> {
    RosEdition::parse(s)
        .ok_or_else(|| eyre::eyre!("sync: unknown ROS edition '{s}' (humble | iron | jazzy)"))
}

/// Resolve the codegen edition (phase-304 W2b): an explicit `--ros-edition`
/// wins; otherwise auto-lower `[system].ros_edition` from a `system.toml` at the
/// workspace root (declare once, RFC-0056); neither → humble (byte-identical).
/// The baked type_hash then matches the runtime `ros-<edition>` keyexpr feature.
fn resolve_sync_edition(cli: Option<&str>, ws_root: &Path) -> Result<RosEdition> {
    if let Some(s) = cli {
        return parse_edition(s);
    }
    let sys_toml = ws_root.join("system.toml");
    if sys_toml.is_file() {
        let raw = std::fs::read_to_string(&sys_toml)
            .wrap_err_with(|| format!("sync: read {}", sys_toml.display()))?;
        let sys: crate::orchestration::cargo_metadata_schema::SystemToml =
            toml::from_str(&raw).wrap_err_with(|| format!("sync: parse {}", sys_toml.display()))?;
        return sys
            .system
            .ros_edition()
            .wrap_err_with(|| format!("sync: [system].ros_edition in {}", sys_toml.display()));
    }
    Ok(RosEdition::Humble)
}

// The interface index (ament, when a ROS 2 env is sourced, merged over the
// bundled share dirs at packages/cli/interfaces/) — loaded once per process.
// Used both to codegen AMENT dep pkgs and, on Iron+, to resolve cross-package
// nested `.msg` types for the REP-2011 type hash.
fn interface_index() -> Option<&'static rosidl_bindgen::ament::AmentIndex> {
    static AMENT_INDEX: std::sync::OnceLock<Option<rosidl_bindgen::ament::AmentIndex>> =
        std::sync::OnceLock::new();
    AMENT_INDEX
        .get_or_init(|| cargo_nano_ros::load_index_with_fallback(false).ok())
        .as_ref()
}

// A cross-package `.msg` resolver over the interface index (RIHS01 type-hash
// DAG closure). `generate_package` resolves same-package nested types itself;
// this covers `std_msgs` / `builtin_interfaces` / etc. Consulted only on Iron+
// (Humble emits a placeholder hash and never calls it).
fn ament_msg_resolver() -> impl Fn(&str) -> Option<rosidl_parser::Message> {
    move |fqn: &str| {
        let idx = interface_index()?;
        let mut parts = fqn.split('/');
        let pkg = parts.next()?;
        let name = parts.next_back()?;
        let package = idx.packages().get(pkg)?;
        let content = std::fs::read_to_string(package.get_message_path(name)).ok()?;
        rosidl_parser::parse_message(&content).ok()
    }
}

// Generate the workspace pkg directly (using its dir as a synthetic share_dir
// — `Package::from_share_dir` reads `package.xml` + scans msg/srv/action).
fn codegen_workspace_pkg(
    pkg: &WsPkg,
    build_root: &Path,
    edition: RosEdition,
    verbose: bool,
) -> Result<()> {
    let out_dir = build_root;
    std::fs::create_dir_all(out_dir)
        .wrap_err_with(|| format!("sync: mkdir {}", out_dir.display()))?;
    if verbose {
        println!(
            "sync: codegen workspace pkg {} → {}",
            pkg.name,
            out_dir.display()
        );
    } else {
        println!("sync: codegen {}", pkg.name);
    }
    let package = Package::from_share_dir(pkg.dir.clone())
        .wrap_err_with(|| format!("sync: read pkg {}", pkg.dir.display()))?;
    // Per-field capacity config (RFC-0033), discovered from the pkg source dir.
    let resolver = rosidl_codegen::CapacityResolver::discover(&pkg.dir, None)?;
    resolver.report_deprecations();
    let msg_resolve = ament_msg_resolver();
    rosidl_bindgen::generator::generate_package(
        &package,
        out_dir,
        edition,
        &resolver,
        &msg_resolve,
    )
    .wrap_err_with(|| format!("sync: generate_package failed for {}", pkg.name))?;
    // Codegen emits <out_dir>/<pkg>/{Cargo.toml,src/} with sibling `path =
    // "../<dep>"` deps. We keep that flat layout (no extra `rust/`
    // nesting) so the relative paths between generated crates resolve
    // correctly without a rewrite pass. Our `nros_generator_rs` prefix
    // already namespaces by language — the extra `rust/` colcon adds is
    // there to coexist with `<pkg>/c/`, `<pkg>/cpp/`, etc. inside the
    // same generator's output, which we don't have.
    Ok(())
}

// Resolve AMENT-side deps (the per-pkg.xml `<depend>` tags not in workspace)
// and codegen each via Package::from_share_dir over its AMENT share path.
fn codegen_ament_deps_for(
    deps: &[String],
    scan: &[WsPkg],
    build_root: &Path,
    edition: RosEdition,
    emitted: &mut HashSet<String>,
    verbose: bool,
) -> Result<()> {
    // Pre-load the interface index once per invocation: the ament index
    // (when a ROS 2 env is sourced) merged over the bundled share dirs at
    // packages/cli/interfaces/ — so a host WITHOUT ROS 2 still resolves
    // std_msgs/builtin_interfaces instead of letting cargo fall through to
    // crates.io's yanked ROS crates (#204 probe finding).
    let Some(idx) = interface_index() else {
        return Ok(());
    };

    let in_workspace: HashSet<&str> = scan.iter().map(|p| p.name.as_str()).collect();
    let mut to_resolve: Vec<String> = deps
        .iter()
        .filter(|d| !in_workspace.contains(d.as_str()))
        .cloned()
        .collect();

    while let Some(dep) = to_resolve.pop() {
        if emitted.contains(&dep) {
            continue;
        }
        let Some(amented) = idx.packages().get(&dep).cloned() else {
            // AMENT doesn't know — silently skip (smart-stub semantics).
            continue;
        };
        // Codegen the AMENT pkg.
        let out_dir = build_root;
        std::fs::create_dir_all(out_dir)?;
        if verbose {
            println!(
                "sync: codegen AMENT pkg {} → {}",
                amented.name,
                out_dir.display()
            );
        } else {
            println!("sync: codegen {}", amented.name);
        }
        let resolver = rosidl_codegen::CapacityResolver::discover(&amented.share_dir, None)?;
        resolver.report_deprecations();
        let msg_resolve = ament_msg_resolver();
        rosidl_bindgen::generator::generate_package(
            &amented,
            out_dir,
            edition,
            &resolver,
            &msg_resolve,
        )
        .wrap_err_with(|| format!("sync: generate_package failed for {}", amented.name))?;
        emitted.insert(amented.name.clone());
        // Queue this pkg's own deps (parse its package.xml).
        let pxml = amented.share_dir.join("package.xml");
        if pxml.is_file() {
            let body = std::fs::read_to_string(&pxml).unwrap_or_default();
            for d in extract_pkg_deps(&body) {
                if !in_workspace.contains(d.as_str()) && !emitted.contains(&d) {
                    to_resolve.push(d);
                }
            }
        }
    }
    Ok(())
}

// --- Scan ----------------------------------------------------------------------

fn has_pkg_subdir(dir: &Path) -> bool {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return false;
    };
    for e in entries.flatten() {
        if let Ok(t) = e.file_type()
            && t.is_dir()
            && e.path().join("package.xml").is_file()
        {
            return true;
        }
    }
    false
}

fn scan_one_pkg_dir(pkg_dir: &Path, out: &mut Vec<WsPkg>) -> Result<()> {
    scan_one_pkg_dir_inner(pkg_dir, out, true)
}

fn scan_one_pkg_dir_inner(
    pkg_dir: &Path,
    out: &mut Vec<WsPkg>,
    is_patch_consumer: bool,
) -> Result<()> {
    let manifest = pkg_dir.join("package.xml");
    let body = std::fs::read_to_string(&manifest)?;
    let Some(name) = extract_pkg_name(&body) else {
        bail!(
            "sync: single-pkg mode: package.xml at {} has no <name>",
            manifest.display()
        );
    };
    let is_msg_pkg = crate::interface_package::is_interface_package(pkg_dir, &body);
    let is_rust_pkg = pkg_dir.join("Cargo.toml").is_file();
    let deps = extract_pkg_deps(&body);
    // Phase 212.M-F.21 — when single-pkg mode lands on an Entry pkg
    // (or any Rust consumer that path-deps on a sibling Component pkg),
    // walk those path-deps + add the targets as siblings in `out` so
    // `augment_rust_consumer_deps_via_path_deps` can union their msg
    // `<*depend>` rows. Without this, single-pkg mode's `scan` only
    // contains the Entry pkg itself + the transitive walk has no msg
    // pkgs to discover. Imports are flagged `is_patch_consumer=false` —
    // cargo only respects `[patch.crates-io]` from the pkg it invokes,
    // so writing patches into a path-dep target is dead weight (and the
    // wrong-direction relative paths corrupt the target's manifest).
    if is_rust_pkg && let Ok(cargo_body) = std::fs::read_to_string(pkg_dir.join("Cargo.toml")) {
        for path in extract_cargo_path_deps(&cargo_body) {
            let target = pkg_dir.join(&path);
            if target.join("package.xml").is_file()
                && std::fs::canonicalize(&target).ok() != std::fs::canonicalize(pkg_dir).ok()
            {
                scan_one_pkg_dir_inner(&target, out, false)?;
            }
        }
    }
    out.push(WsPkg {
        name,
        dir: pkg_dir.to_path_buf(),
        manifest,
        is_msg_pkg,
        is_rust_pkg,
        deps,
        is_patch_consumer,
    });
    Ok(())
}

fn scan_workspace(src_root: &Path, out: &mut Vec<WsPkg>) -> Result<()> {
    for entry in std::fs::read_dir(src_root)? {
        let entry = entry?;
        if !entry.file_type()?.is_dir() {
            continue;
        }
        let dir = entry.path();
        let manifest = dir.join("package.xml");
        if !manifest.is_file() {
            continue;
        }
        let body = std::fs::read_to_string(&manifest)?;
        let Some(name) = extract_pkg_name(&body) else {
            continue;
        };
        let is_msg_pkg = crate::interface_package::is_interface_package(&dir, &body);
        let is_rust_pkg = dir.join("Cargo.toml").is_file();
        let deps = extract_pkg_deps(&body);
        out.push(WsPkg {
            name,
            dir,
            manifest,
            is_msg_pkg,
            is_rust_pkg,
            deps,
            is_patch_consumer: true,
        });
    }
    Ok(())
}

fn extract_pkg_name(body: &str) -> Option<String> {
    let start = body.find("<name>")? + "<name>".len();
    let end = body[start..].find("</name>")? + start;
    Some(body[start..end].trim().to_string())
}

/// Phase 212.M-F.21 — walk each Rust consumer's `Cargo.toml [dependencies]`
/// + sibling `[dev-dependencies]` / `[build-dependencies]` tables for
///   `path = "..."` entries that resolve (by directory) to another `WsPkg`
///   in `scan`. For each such hit, union the target pkg's `deps` into the
///   consumer's `deps`. Idempotent — re-running deduplicates.
///
/// Concretely unblocks the Entry-pkg → Component-pkg path: the Entry
/// pkg's `package.xml` typically has no `<depend>` rows but its
/// `Cargo.toml` carries `freertos_rs_talker = { path = "../talker" }`.
/// The Component pkg's `package.xml` lists `<depend>std_msgs</depend>`
/// etc. — those msg deps need to land in the Entry pkg's patch table
/// (the patch authority cargo invokes).
fn augment_rust_consumer_deps_via_path_deps(scan: &mut [WsPkg]) -> Result<()> {
    // Index by canonical directory so we can resolve path-dep targets.
    let dir_to_pkg: std::collections::HashMap<PathBuf, usize> = scan
        .iter()
        .enumerate()
        .filter_map(|(i, p)| std::fs::canonicalize(&p.dir).ok().map(|d| (d, i)))
        .collect();

    // Snapshot pre-augmentation deps so transitivity is single-hop per pass.
    // (Multi-hop chains converge after a small fixed number of passes; we
    // keep it deterministic + bounded.)
    for _ in 0..4 {
        let snapshot: Vec<Vec<String>> = scan.iter().map(|p| p.deps.clone()).collect();
        let mut changed = false;
        for (i, pkg) in scan.iter_mut().enumerate() {
            if !pkg.is_rust_pkg {
                continue;
            }
            let cargo_toml = pkg.dir.join("Cargo.toml");
            let Ok(body) = std::fs::read_to_string(&cargo_toml) else {
                continue;
            };
            for path in extract_cargo_path_deps(&body) {
                let target = pkg.dir.join(&path);
                let Ok(canon) = std::fs::canonicalize(&target) else {
                    continue;
                };
                let Some(&j) = dir_to_pkg.get(&canon) else {
                    continue;
                };
                if i == j {
                    continue;
                }
                let target_deps = &snapshot[j];
                for d in target_deps {
                    if !pkg.deps.contains(d) {
                        pkg.deps.push(d.clone());
                        changed = true;
                    }
                }
            }
            pkg.deps.sort();
            pkg.deps.dedup();
        }
        if !changed {
            break;
        }
    }
    Ok(())
}

/// Extract `path = "<rel>"` values from `[dependencies]` /
/// `[dev-dependencies]` / `[build-dependencies]` tables. Loose TOML
/// scanner — handles single-line `pkg = { path = "..." }` form which
/// is the convention across nano-ros fixtures. Multi-line tables are
/// rare in fixture Cargo.tomls and skipped silently.
fn extract_cargo_path_deps(body: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut in_deps = false;
    for line in body.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('[') && trimmed.ends_with(']') {
            in_deps = matches!(
                trimmed,
                "[dependencies]" | "[dev-dependencies]" | "[build-dependencies]"
            );
            continue;
        }
        if !in_deps {
            continue;
        }
        // Match `<name> = { path = "<rel>", ... }` form.
        let Some(eq) = trimmed.find('=') else {
            continue;
        };
        let rhs = trimmed[eq + 1..].trim_start();
        if !rhs.starts_with('{') {
            continue;
        }
        if let Some(p) = rhs.find("path") {
            let after = &rhs[p + 4..];
            let after = after.trim_start().trim_start_matches('=').trim_start();
            if let Some(rest) = after.strip_prefix('"')
                && let Some(end) = rest.find('"')
            {
                out.push(rest[..end].to_string());
            }
        }
    }
    out
}

/// Phase 212.M-F.21 — walk up from `ws_root` looking for a nano-ros
/// source tree (marker: `packages/core/nros-core/Cargo.toml`). Used as
/// a fallback when neither `--nros-repo` nor `NROS_REPO_DIR` is set.
/// In-tree fixtures + examples sit several levels below the nano-ros
/// root, so this turns the most common "I forgot to set NROS_REPO_DIR"
/// case into a no-op — patches still flow.
pub(crate) fn autodetect_nano_ros_path(ws_root: &Path) -> Option<PathBuf> {
    let mut cur: Option<&Path> = Some(ws_root);
    while let Some(p) = cur {
        if p.join("packages/core/nros-core/Cargo.toml").is_file() {
            return Some(p.to_path_buf());
        }
        cur = p.parent();
    }
    None
}

fn extract_pkg_deps(body: &str) -> Vec<String> {
    let mut deps = Vec::new();
    for tag in &[
        "<depend>",
        "<build_depend>",
        "<exec_depend>",
        "<run_depend>",
        "<build_export_depend>",
    ] {
        let close = tag.replace("<", "</");
        let mut cursor = 0;
        while let Some(rel) = body[cursor..].find(tag) {
            let start = cursor + rel + tag.len();
            let Some(rel_close) = body[start..].find(close.as_str()) else {
                break;
            };
            let end = start + rel_close;
            let name = body[start..end].trim().to_string();
            if !name.is_empty() && !is_ros_meta_pkg(&name) {
                deps.push(name);
            }
            cursor = end;
        }
    }
    deps.sort();
    deps.dedup();
    deps
}

fn is_ros_meta_pkg(name: &str) -> bool {
    name.starts_with("rosidl")
        || name.starts_with("ament")
        || name == "rclcpp"
        || name == "rclpy"
        || name.starts_with("rcl")
        || name.starts_with("rmw")
        || name.starts_with("launch")
        || name == "catkin"
}

fn topo_sort_msg_pkgs(pkgs: &[&WsPkg]) -> Result<Vec<String>> {
    let names: std::collections::HashSet<&str> = pkgs.iter().map(|p| p.name.as_str()).collect();
    let mut remaining: Vec<&&WsPkg> = pkgs.iter().collect();
    let mut emitted: Vec<String> = Vec::new();
    while !remaining.is_empty() {
        let pick_idx = remaining.iter().position(|p| {
            p.deps
                .iter()
                .filter(|d| names.contains(d.as_str()))
                .all(|d| emitted.contains(d))
        });
        match pick_idx {
            Some(idx) => emitted.push(remaining.remove(idx).name.clone()),
            None => {
                let names: Vec<&str> = remaining.iter().map(|p| p.name.as_str()).collect();
                bail!("sync: dependency cycle (or missing dep) among {names:?}");
            }
        }
    }
    Ok(emitted)
}

// --- Patch authority -----------------------------------------------------------

fn find_patch_authority(start: &Path, ws_root: &Path) -> Result<PathBuf> {
    let mut cur = start.to_path_buf();
    loop {
        let cargo = cur.join("Cargo.toml");
        if cargo.is_file() {
            let body = std::fs::read_to_string(&cargo)?;
            if has_workspace_table(&body) {
                return Ok(cargo);
            }
        }
        if cur == *ws_root {
            return Ok(start.join("Cargo.toml"));
        }
        match cur.parent() {
            Some(p) => cur = p.to_path_buf(),
            None => return Ok(start.join("Cargo.toml")),
        }
    }
}

fn has_workspace_table(body: &str) -> bool {
    body.lines().any(|l| {
        let t = l.trim();
        t == "[workspace]" || t.starts_with("[workspace]")
    })
}

// --- Patch block writer --------------------------------------------------------

const BEGIN: &str = "# === BEGIN nros-managed [patch.crates-io] ===";
const END: &str = "# === END nros-managed [patch.crates-io] ===";

/// Phase 287 W9 (option E) — the crates whose `[patch.crates-io]` entries move
/// to the sync-generated CENTRAL `<checkout>/nros-patch.toml` (reached from each
/// leaf via one `include = [...]` line) instead of being emitted per-leaf.
///
/// Membership rule: a crate may live centrally only if it is registry-named in
/// EVERY Rust consumer's dependency graph — cargo warns "patch … was not used in
/// the crate graph" for a patch entry whose crate the graph never resolves
/// registry-style, and the central file is shared by all leaves. That limits the
/// set to the universal trio: `nros` (named by every consumer Cargo.toml) and
/// `nros-core`/`nros-serdes` (the hardcoded base of the managed set, named by
/// every generated msg crate). RMW crates are NOT universal (verified 2026-07-14:
/// a freertos entry's slim graph lacks `nros-rmw-cyclonedds-sys`/`-xrce-cffi`
/// and warns); board/driver/PAC crates even less so. Those stay per-leaf.
const CENTRAL_PATCH_CRATES: &[&str] = &["nros", "nros-core", "nros-serdes"];

/// File name of the central patch file at the nano-ros checkout root.
const CENTRAL_PATCH_FILE: &str = "nros-patch.toml";

/// Phase 287 W9 (option E) — write `<nano_ros_path>/nros-patch.toml`: the
/// central `[patch.crates-io]` for [`CENTRAL_PATCH_CRATES`], ABSOLUTE paths so
/// one generated file serves every leaf regardless of depth. Idempotent
/// (skip-write when content is unchanged, so repeated syncs don't churn the
/// mtime); atomic temp + rename otherwise.
/// #272 — the minor version of cargo that stabilized the `include` config key.
/// Before this, `include` is `-Z config-include` (nightly-only) and stable cargo
/// silently ignores it — dropping the central `[patch.crates-io]` and failing the
/// build with `no matching package named 'nros'`.
const CONFIG_INCLUDE_STABLE_MINOR: u64 = 93;

/// Parse the minor version out of `cargo --version` output
/// (`"cargo 1.96.0 (abc 2026-..)"` → `Some(96)`). `None` when the shape is
/// unrecognised (a custom/edge build) — the caller then stays quiet rather than
/// warn on a version it cannot read.
fn parse_cargo_minor(version_line: &str) -> Option<u64> {
    let ver = version_line.split_whitespace().nth(1)?; // "1.96.0"
    let mut parts = ver.split('.');
    let _major = parts.next()?;
    parts.next()?.parse::<u64>().ok()
}

/// Warn once if the workspace's effective cargo predates the `include` config-key
/// stabilization (1.93), so an external consumer on an old pinned toolchain gets
/// a clear diagnostic instead of a silent patch drop (#272). Best-effort: any
/// failure to run/parse `cargo --version` stays silent (never blocks sync).
/// Does the `cargo --version` warning apply to this workspace? — phase-367 W2.
///
/// Extracted so both arms are testable: a predicate embedded in a
/// side-effect-only function can only be verified by watching for a warning
/// that may legitimately not appear, which is no verification at all.
///
/// `false` for a workspace inside the nano-ros checkout, `true` outside it.
fn cargo_version_warning_applies(ws_root: &Path) -> bool {
    crate::abi_guard::find_monorepo_root(ws_root).is_none()
}

fn warn_if_cargo_predates_config_include(ws_root: &Path) {
    // phase-367 W2 — this warning is for an EXTERNAL consumer on an old pinned
    // toolchain (#272), and it is the single most expensive thing a warm
    // `nros sync` does.
    //
    // Measured on `examples/workspaces/mixed`: 0.160 s with the probe,
    // 0.093 s with it failing fast — **42 % of the run, to decide whether to
    // print a warning**. The cost is not `cargo --version` itself but the
    // `scripts/bin/cargo` PATH shim it goes through, which fans out into
    // `env` -> `bash` -> `dirname` -> `grep` -> the real cargo -> `rustc -vV`.
    // That is also why `strace -c` misattributes it: the weight lands on the
    // children, not on this process's `execve`.
    //
    // An IN-REPO workspace cannot be the case the warning is for: the toolchain
    // is pinned by the checkout, and the build that follows uses the same
    // shim-wrapped cargo this repo ships. `find_monorepo_root` walks up for
    // `packages/core/nros-core/Cargo.toml`, so `Some` means "inside the
    // nano-ros tree" — exactly the population #272 excluded.
    //
    // A build runs ~101 syncs (issue 0649's census), so this is ~6.8 s per
    // fixture build spent deciding not to print anything.
    if !cargo_version_warning_applies(ws_root) {
        return;
    }
    // Run in the workspace root so a `rust-toolchain.toml` there selects the
    // SAME cargo the build will use, not whatever invoked `nros`.
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    let Ok(out) = std::process::Command::new(cargo)
        .arg("--version")
        .current_dir(ws_root)
        .output()
    else {
        return;
    };
    if !out.status.success() {
        return;
    }
    let line = String::from_utf8_lossy(&out.stdout);
    let Some(minor) = parse_cargo_minor(&line) else {
        return;
    };
    if minor < CONFIG_INCLUDE_STABLE_MINOR {
        eprintln!(
            "warning: this workspace's cargo ({}) predates the `include` config-key \
             stabilization (cargo 1.{CONFIG_INCLUDE_STABLE_MINOR}). nros sync writes the \
             nros/nros-core/nros-serdes [patch.crates-io] rows into a central file reached \
             via `include = [\"…/nros-patch.toml\"]`, which stable cargo < 1.{CONFIG_INCLUDE_STABLE_MINOR} \
             SILENTLY IGNORES — the build will then fail `no matching package named 'nros'`. \
             Upgrade to cargo >= 1.{CONFIG_INCLUDE_STABLE_MINOR}, or add those three `path = ` rows \
             to `[patch.crates-io]` by hand.",
            line.trim(),
        );
    }
}

#[derive(Debug, ClapArgs)]
pub struct CentralPatchArgs {
    /// nano-ros checkout to write into. Defaults to `NROS_REPO_DIR`, then cwd.
    #[arg(long)]
    pub nano_ros_path: Option<PathBuf>,
}

/// issue 1038 — write the central patch file, with no workspace and no codegen.
fn run_central_patch(args: CentralPatchArgs) -> Result<()> {
    let root = match args.nano_ros_path {
        Some(p) => p,
        None => match std::env::var_os("NROS_REPO_DIR") {
            Some(v) => PathBuf::from(v),
            None => std::env::current_dir()?,
        },
    };
    let dst = write_central_patch_file(&root)?;
    println!("{}", dst.display());
    Ok(())
}

fn write_central_patch_file(nano_ros_path: &Path) -> Result<PathBuf> {
    let nrp = nano_ros_path
        .canonicalize()
        .unwrap_or_else(|_| nano_ros_path.to_path_buf());
    let mut body = String::from(
        "# Generated by `nros sync` — do not edit, do not commit (gitignored).\n\
         # Phase 287 W9 (RFC-0048 option E): the central [patch.crates-io] every\n\
         # Rust leaf reaches via one `include = [...]` line in .cargo/config.toml.\n\
         # Absolute paths: re-run `nros sync` after moving the checkout, AND\n\
         # after any of the patched crates moves WITHIN it (phase-321 W2.e\n\
         # relocated `nros` to packages/api/; every stale copy of this file then\n\
         # failed cargo with \"failed to load source for dependency `nros`\").\n\
         [patch.crates-io]\n",
    );
    for name in CENTRAL_PATCH_CRATES {
        let sub = nros_crate_subpath(name).expect("central crates are in the lookup table");
        let crate_root = nrp.join(&sub);
        if !crate_root.join("Cargo.toml").is_file() {
            continue;
        }
        body.push_str(&format!(
            "{name} = {{ path = \"{}\" }}\n",
            crate_root.display()
        ));
    }
    let dst = nrp.join(CENTRAL_PATCH_FILE);
    // issue 0562 — the shared helper is both halves (skip-if-identical +
    // atomic rename); this used to be a third private copy.
    crate::atomic_file::atomic_write(&dst, &body)?;
    Ok(dst)
}

/// phase-327 W5 (issue 0368 F4 / the issue-0363 shape, leaf-local) — the
/// generated-crate entries a rewrite would DROP even though some consumer
/// still declares the dependency.
///
/// A crate got into the managed block because a previous sync generated it;
/// for it to leave, either the workspace genuinely dropped the dep (fine —
/// `requested` no longer names it) or THIS run failed to resolve it (no ROS
/// env and not in the bundled interfaces). The second case used to write a
/// narrower table that fails NOWHERE at sync time: the dropped entry
/// resolves from crates.io (yanked, unrelated crates) at the next build.
/// Observed live in issue 0368: a ROS-less host's sync silently removed
/// `example_interfaces`/`action_msgs`/`unique_identifier_msgs` from a
/// TRACKED `.cargo/config.toml`.
///
/// Line-based on the writer's own `# nros-managed` decor, scoped to entries
/// whose path points into the generated tree — runtime crates have their own
/// dead-path guard in [`render_managed_entries`].
fn narrowed_generated_entries(
    existing_body: &str,
    new_names: &HashSet<&str>,
    requested: &HashSet<String>,
) -> Vec<String> {
    let mut narrowed = Vec::new();
    for line in existing_body.lines() {
        let Some(rest) = line.trim_end().strip_suffix("# nros-managed") else {
            continue;
        };
        let Some((name, spec)) = rest.split_once('=') else {
            continue;
        };
        let name = name.trim();
        if !spec.contains("generated/") {
            continue;
        }
        if requested.contains(name) && !new_names.contains(name) {
            narrowed.push(name.to_string());
        }
    }
    narrowed.sort();
    narrowed.dedup();
    narrowed
}

/// Every `execution.tiers.<tier>[.<scope>].<dim>` leaf a model declares.
///
/// Issue 0380 — the committed SystemModel is the SSoT for scheduling dims the
/// resolver's inputs cannot express (`zephyr.deadline_us`,
/// `nuttx.budget_us`/`period_us`, `threadx.preempt_threshold`/`time_slice_us`,
/// per-platform `core` pins). `system.toml` deliberately does not carry them.
///
/// So a re-resolve cannot reproduce them, and two regeneration commits
/// (`07650d0a1`, `6071bd150`) deleted the models and re-resolved — stripping 17
/// dims. Nothing failed at sync time; it surfaced a tier later as ~17 realtime
/// e2e tests reporting the RFC-0052 fail-loud violation they exist to catch.
///
/// Comparing key SETS, not values: a value that legitimately changes is a
/// re-resolve doing its job, whereas a key that DISAPPEARS is content the
/// inputs could never have produced.
fn execution_tier_dims(yaml: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let Ok(doc) = serde_yaml_ng::from_str::<serde_yaml_ng::Value>(yaml) else {
        // Unparseable: report nothing rather than guess. The caller treats an
        // empty prior set as "nothing to lose", which is the safe direction —
        // a guard that fires on a file it cannot read is a guard people disable.
        return out;
    };
    let Some(tiers) = doc.get("execution").and_then(|e| e.get("tiers")) else {
        return out;
    };
    let serde_yaml_ng::Value::Mapping(tiers) = tiers else {
        return out;
    };
    for (tier, body) in tiers {
        let Some(tier) = tier.as_str() else { continue };
        let serde_yaml_ng::Value::Mapping(body) = body else {
            continue;
        };
        for (k, v) in body {
            let Some(k) = k.as_str() else { continue };
            match v {
                // A platform scope (`zephyr:`, `nuttx:`, …) — descend one level
                // so `high.zephyr.deadline_us` is distinguishable from a
                // generic `high.deadline_us`. They lower differently, which is
                // the entire point of the scoped tables.
                serde_yaml_ng::Value::Mapping(scope) => {
                    for (dim, _) in scope {
                        if let Some(dim) = dim.as_str() {
                            out.insert(format!("{tier}.{k}.{dim}"));
                        }
                    }
                }
                _ => {
                    out.insert(format!("{tier}.{k}"));
                }
            }
        }
    }
    out
}

/// `nros ws model-dims <model.yaml>` — issue 0380's read-only door onto
/// [`execution_tier_dims`], so the gate and the sync-time guard cannot disagree
/// about what a "dim" is.
/// phase-341 W4 — `nros ws check-board-projections`.
///
/// Scans the workspace exactly as `sync` does, then asks
/// [`project_board_configs_with`] in CHECK mode whether every committed
/// projection still matches a fresh render of its descriptor. Writes nothing.
fn run_check_board_projections(args: CheckBoardProjectionsArgs) -> Result<()> {
    let ws_root = args
        .path
        .canonicalize()
        .wrap_err_with(|| format!("check-board-projections: {}", args.path.display()))?;
    // Mirror `run_sync`'s dispatch exactly. Scanning a single leaf dir with
    // `scan_workspace` finds NOTHING, and a check that inspects nothing passes —
    // which is how the first cut of this gate reported OK on 0 leaves while a
    // deliberately corrupted projection sat next to it.
    // `has_pkg_subdir` is load-bearing, not decoration: a standalone leaf has a
    // `src/` too (the CARGO source dir), so `src/.is_dir()` alone routes it down
    // the colcon branch, scans the wrong directory and finds no packages. Sync's
    // own comment warns of exactly this; the first cut of this gate ignored it
    // and reported OK on zero leaves.
    let colcon_layout = ws_root.join("src").is_dir() && has_pkg_subdir(&ws_root.join("src"));
    let single_pkg_mode = !colcon_layout && ws_root.join("package.xml").is_file();
    let mut scan: Vec<WsPkg> = Vec::new();
    if single_pkg_mode {
        scan_one_pkg_dir(&ws_root, &mut scan)?;
    } else if colcon_layout {
        scan_workspace(&ws_root.join("src"), &mut scan)?;
    } else {
        scan_workspace(&ws_root, &mut scan)?;
    }
    let leaves: Vec<&WsPkg> = scan.iter().filter(|p| p.needs_patch_authority()).collect();
    if leaves.is_empty() {
        // Never "OK, nothing to do": a gate that inspects nothing must say so,
        // or a mis-pointed path reads as a pass.
        return Err(eyre!(
            "check-board-projections: no patch-authority leaf found under {} —              refusing to report OK on a tree it did not understand",
            ws_root.display()
        ));
    }
    let nano_ros_path = std::env::var_os("NROS_REPO_DIR")
        .map(PathBuf::from)
        .or_else(|| autodetect_nano_ros_path(&ws_root));

    if args.write {
        // phase-351 W3 — sync's board-projection pass, alone. Writing with the
        // SAME function the check calls is the point: a separate "regenerate"
        // path would be a second spelling of the render, which is the drift
        // phase-341 removed.
        project_board_configs(&leaves, nano_ros_path.as_deref(), false)?;
        return Ok(());
    }

    let complaints = project_board_configs_with(&leaves, nano_ros_path.as_deref(), false, true)?;
    if complaints.is_empty() {
        println!(
            "check-board-projections: OK ({} leaf/leaves match their descriptor)",
            leaves.len()
        );
        return Ok(());
    }
    for c in &complaints {
        eprintln!("check-board-projections: {c}");
    }
    eprintln!();
    eprintln!("  `.cargo/nros-board.toml` is GENERATED from the board's `cargo_config`");
    eprintln!("  (RFC-0032 third leg, phase-341). Do not hand-edit it — change the");
    eprintln!("  descriptor and re-run `nros sync`.");
    Err(eyre!(
        "{} board projection(s) disagree with their descriptor",
        complaints.len()
    ))
}

fn run_model_dims(args: ModelDimsArgs) -> Result<()> {
    let raw = std::fs::read_to_string(&args.model)
        .wrap_err_with(|| format!("model-dims: read {}", args.model.display()))?;
    for dim in execution_tier_dims(&raw) {
        println!("{dim}");
    }
    Ok(())
}

/// The `[patch.crates-io]` rows this leaf's board descriptor declares, resolved
/// against the leaf (phase-351 W3).
///
/// Conservative in exactly the way [`project_board_configs_with`] is: no
/// checkout, no catalog, an unresolvable `deploy`, or an out-of-tree leaf all
/// mean NO rows rather than a guess. A wrong row here would be written into ~700
/// leaf configs (issues 0457 / 0463), so silence is the safe direction — the
/// leaf then resolves exactly as it did before this wave.
/// The rows are resolved against the AUTHORITY directory, not the leaf: that is
/// the `.cargo/config.toml` they land in, and cargo resolves a config path
/// against the parent of `.cargo/`. For a workspace that is the ROOT, which is
/// also why this takes every leaf answering to one authority — one workspace can
/// hold entries for several boards, and the root config carries their union.
fn board_declared_patch_rows(
    authority_dir: &Path,
    leaf_dirs: &[PathBuf],
    nano_ros_path: Option<&Path>,
) -> Result<Vec<(String, String)>> {
    use crate::orchestration::board_descriptor::{BoardCatalog, DeployResolution};

    let Some(nrp) = nano_ros_path else {
        return Ok(Vec::new());
    };
    let nrp_c = nrp.canonicalize().unwrap_or_else(|_| nrp.to_path_buf());
    let Some(prefix) = leaf_to_root_prefix(authority_dir, &nrp_c) else {
        return Ok(Vec::new()); // out-of-tree consumer (#272)
    };
    let Ok(catalog) = BoardCatalog::load(&nrp_c) else {
        return Ok(Vec::new());
    };
    let mut out: Vec<(String, String)> = Vec::new();
    for leaf in leaf_dirs {
        let Ok(manifest) = std::fs::read_to_string(leaf.join("Cargo.toml")) else {
            continue;
        };
        let Some(deploy) = entry_deploy_key(&manifest) else {
            continue; // not an Entry pkg — no board to inherit from
        };
        let DeployResolution::Board(descriptor) = catalog.resolve_deploy(&deploy) else {
            continue; // unknown or ambiguous — the projection pass says so
        };
        let Some(raw) = descriptor.cargo_config.as_deref() else {
            continue;
        };
        for row in board_patch_rows(raw, &prefix)? {
            if !out.contains(&row) {
                out.push(row);
            }
        }
    }
    out.sort();
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn write_patch_block(
    authority: &Path,
    build_root: &Path,
    pkgs: &[String],
    nano_ros_path: Option<&Path>,
    extra_runtime_crates: &[String],
    requested_central_patch: Option<&Path>,
    requested: &HashSet<String>,
    leaf_dirs: &[PathBuf],
) -> Result<()> {
    let central_patch = requested_central_patch;
    let authority_dir = authority.parent().unwrap();
    let mut entries = render_managed_entries(
        authority,
        build_root,
        pkgs,
        nano_ros_path,
        extra_runtime_crates,
        requested,
    )?;
    // phase-351 W3 — rows the leaf's BOARD declares (`nros-board-nuttx-qemu`'s
    // NuttX-patched `libc` for `-Z build-std`). They are in-repo relative paths,
    // so they stay INLINE in the tracked config like every other in-repo row
    // (issue 0463's split by ORIGIN) — the sidecar is only for `generated/`.
    entries.extend(board_declared_patch_rows(
        authority_dir,
        leaf_dirs,
        nano_ros_path,
    )?);
    // #272 — how this leaf reaches the central trio (`nros`/`nros-core`/
    // `nros-serdes`) depends on whether it lives INSIDE the nano-ros checkout:
    //
    // - IN-TREE example leaf: its `.cargo/config.toml` is COMMITTED, so it uses
    //   the relative `include = ["…/nros-patch.toml"]` line (a host-absolute path
    //   would break every other checkout). The reachability bail below turns the
    //   include's silent-drop failure modes into a loud error.
    // - OUT-OF-TREE consumer (colcon / autoware_sentinel): NOT committed, and the
    //   `include` has three fragile preconditions (cargo ≥ 1.93, a correct
    //   relative path, a present central file) — tripping any one fails the build
    //   with an unexplained `no matching package named 'nros'`. So inline the trio
    //   with ABSOLUTE paths directly and skip the include entirely — the failure
    //   class cannot occur.
    let external = match nano_ros_path {
        Some(nrp) => {
            let nrp_c = nrp.canonicalize().unwrap_or_else(|_| nrp.to_path_buf());
            let auth_c = authority_dir
                .canonicalize()
                .unwrap_or_else(|_| authority_dir.to_path_buf());
            !auth_c.starts_with(&nrp_c)
        }
        None => false,
    };

    let include_rel: Option<String> = if external {
        // Replace render's RELATIVE trio rows (nros-core/nros-serdes) with the
        // full ABSOLUTE trio (incl. `nros`, which render never emits). No include.
        if let Some(nrp) = nano_ros_path {
            let nrp_c = nrp.canonicalize().unwrap_or_else(|_| nrp.to_path_buf());
            entries.retain(|(name, _)| !CENTRAL_PATCH_CRATES.contains(&name.as_str()));
            for name in CENTRAL_PATCH_CRATES {
                let Some(sub) = nros_crate_subpath(name) else {
                    continue;
                };
                let crate_root = nrp_c.join(&sub);
                if crate_root.join("Cargo.toml").is_file() {
                    entries.push((name.to_string(), crate_root.display().to_string()));
                }
            }
        }
        None
    } else {
        // In-tree: crates served by the central file drop out of the per-leaf
        // emit; the relative `include` line carries them instead.
        central_patch.map(|cp| {
            entries.retain(|(name, _)| !CENTRAL_PATCH_CRATES.contains(&name.as_str()));
            // Cargo resolves a relative `include` against the INCLUDING file's
            // directory (`<authority_dir>/.cargo/`).
            let cfg_dir = authority_dir.join(".cargo");
            pathdiff::diff_paths(cp, &cfg_dir)
                .unwrap_or_else(|| cp.to_path_buf())
                .display()
                .to_string()
        })
    };

    // 1) Write the managed [patch.crates-io] into `<authority_dir>/.cargo/config.toml`
    //    (phase-265: never the consumer Cargo.toml). Format-preserving toml_edit DOM.
    // #272 — fail loud when the include target is unreachable from this leaf.
    //
    // Issue 0463 corrected the reason. #272 recorded it as "cargo would
    // silently drop the patch and the build would die with an unexplained
    // `no matching package named 'nros'`". Measured on cargo 1.97.1, a missing
    // include is not dropped: it is a HARD error, raised while parsing the
    // manifest, so `cargo metadata` fails alongside the build. The check below
    // is still the right one — it is the message, not the behaviour, that was
    // wrong — but it now earns its keep by naming sync instead of pre-empting
    // a silent-resolution failure that does not occur.
    if let Some(inc) = include_rel.as_deref() {
        let target = std::path::Path::new(inc);
        let resolved = if target.is_absolute() {
            target.to_path_buf()
        } else {
            // Resolve the `..` segments LEXICALLY. The include is relative to
            // the config file, i.e. `<authority>/.cargo/`, and on a first sync
            // that directory does not exist yet — so a filesystem-walking
            // `is_file()` on the unnormalised path fails through the missing
            // component and reports a perfectly readable central patch file as
            // unreachable. phase-307 hit exactly that adding a new example
            // workspace: `nros sync` refused a workspace whose only sin was
            // being new.
            lexically_join(&authority_dir.join(".cargo"), target)
        };
        if !resolved.is_file() {
            bail!(
                "sync: central patch file `{}` is not readable from `{}` — \
                 cargo raises a missing `include` as a hard error while parsing \
                 the manifest, so every leaf reaching it becomes unreadable (not \
                 merely unbuildable). Re-run `nros sync` from the nano-ros \
                 checkout (the file is gitignored + regenerated).",
                resolved.display(),
                authority_dir.display(),
            );
        }
    }
    // phase-327 W5 — refuse to NARROW an existing managed block: a
    // still-requested generated crate missing from the new entry set means
    // THIS run failed to resolve it (no ROS env, not in the bundled
    // interfaces), not that the workspace dropped the dep.
    let cfg_path = authority_dir.join(".cargo/config.toml");
    if let Ok(existing) = std::fs::read_to_string(&cfg_path) {
        let new_names: HashSet<&str> = entries.iter().map(|(n, _)| n.as_str()).collect();
        let narrowed = narrowed_generated_entries(&existing, &new_names, requested);
        if !narrowed.is_empty() {
            bail!(
                "sync: refusing to write {} — it would DROP {} still-declared \
                 generated interface crate(s): {}.\n\
                 \x20 The interface index could not resolve them this run (no ROS 2 \
                 environment and not in the bundled set at packages/cli/interfaces/). \
                 A narrower [patch.crates-io] fails nowhere at sync time and resolves \
                 those deps from crates.io at the next build (issue 0368 F4 / the \
                 issue-0363 shape). Fix: source a ROS 2 env, vendor the package into \
                 packages/cli/interfaces/, or remove the dependency — then re-run \
                 `nros sync`.",
                cfg_path.display(),
                narrowed.len(),
                narrowed.join(", "),
            );
        }
    }
    // Issue 0457 — in-tree leaves get the gitignored sidecar; an out-of-tree
    // consumer keeps everything inline (#272: no `include` outside this checkout).
    write_patch_config(authority_dir, &entries, include_rel.as_deref(), !external)?;

    // 2) Migrate: vacate any legacy nros-managed `[patch.crates-io]` block from the
    //    consumer Cargo.toml (one-time; the patch now lives in config.toml). User
    //    patch rows + the rest of the manifest are preserved. Atomic temp + rename
    //    (the parallel-RMW-variant race the splice writer guarded still applies).
    let body = std::fs::read_to_string(authority)
        .wrap_err_with(|| format!("sync: read {}", authority.display()))?;
    let migrated = strip_managed_patch_from_cargo(&body);
    if migrated != body {
        // issue 0562 — the fifth private copy of temp+rename; the shared helper
        // does the `migrated != body` check itself, but keep the explicit
        // guard so the one-time migration stays readable at the call site.
        crate::atomic_file::atomic_write(authority, &migrated)?;
    }

    println!(
        "sync: wrote [patch.crates-io] → {}",
        authority_dir.join(".cargo/config.toml").display()
    );
    Ok(())
}

/// Phase 265 (W3) — migrate a consumer Cargo.toml off the legacy nros-managed
/// `[patch.crates-io]` block (now that patches live in `.cargo/config.toml`).
/// Text-level (NOT toml_edit) so the rest of the hand-authored manifest is byte-
/// preserved: (1) remove every `BEGIN…END` managed region; (2) if a now-empty
/// `[patch.crates-io]` header remains (nothing but blanks until the next section /
/// EOF), drop the header + its trailing blanks too. User patch rows are kept.
fn strip_managed_patch_from_cargo(body: &str) -> String {
    let stripped = strip_managed_block(body);
    drop_empty_patch_crates_io_header(&stripped)
}

/// Remove a `[patch.crates-io]` (bare or quoted) header that has no entries before
/// the next `[section]` / EOF — only blank lines. Leaves a populated table intact.
fn drop_empty_patch_crates_io_header(body: &str) -> String {
    let lines: Vec<&str> = body.lines().collect();
    let mut out: Vec<&str> = Vec::with_capacity(lines.len());
    let mut i = 0usize;
    while i < lines.len() {
        if is_patch_crates_io_header(lines[i]) {
            // Look ahead: is the table body empty (only blanks) until the next
            // section header / EOF?
            let mut j = i + 1;
            let mut empty = true;
            while j < lines.len() {
                let t = lines[j].trim();
                if t.is_empty() {
                    j += 1;
                    continue;
                }
                // Next table header → table ended; anything else → non-empty.
                empty = t.starts_with('[');
                break;
            }
            if empty {
                // Skip the header + the run of blank lines after it; also drop one
                // trailing blank separator already in `out` for a minimal diff.
                if out.last().map(|l| l.trim().is_empty()).unwrap_or(false) {
                    out.pop();
                }
                i += 1;
                while i < lines.len() && lines[i].trim().is_empty() {
                    i += 1;
                }
                continue;
            }
        }
        out.push(lines[i]);
        i += 1;
    }
    let mut s = out.join("\n");
    if body.ends_with('\n') && !s.ends_with('\n') {
        s.push('\n');
    }
    s
}

/// Phase 220.E — static lookup of every nano-ros runtime crate the
/// `nros sync` writer knows how to emit a `[patch.crates-io]` path entry
/// for. Mirrors the workspace layout under `<NROS_REPO_DIR>/packages/`.
///
/// If a consumer references an `nros-*` crate not in this table, the
/// writer logs a warning + skips (so a third-party `nros-foo` extension
/// doesn't break sync — the user can hand-patch outside the managed
/// region).
///
/// Order here doesn't matter; the emission pass dedupes + sorts
/// alphabetically for diff-stable output.
const fn nros_crate_path_lookup() -> &'static [(&'static str, &'static str)] {
    &[
        // Core runtime
        ("nros", "packages/api/nros"),
        ("nros-core", "packages/core/nros-core"),
        ("nros-serdes", "packages/core/nros-serdes"),
        ("nros-platform", "packages/platform/nros-platform"),
        ("nros-platform-api", "packages/platform/nros-platform-api"),
        ("nros-platform-cffi", "packages/platform/nros-platform-cffi"),
        ("nros-node", "packages/core/nros-node"),
        ("nros-rmw", "packages/core/nros-rmw"),
        ("nros-rmw-cffi", "packages/rmw/cffi"),
        ("nros-log", "packages/core/nros-log"),
        ("nros-macros", "packages/core/nros-macros"),
        ("nros-params", "packages/core/nros-params"),
        // Phase 277 W6 — crates the standalone examples reference registry-style
        // after the path-dep flip but that don't ride the `nros-board-*` generic
        // fallback (a support crate under core/, a driver, and a board PAC whose
        // package name has no `nros-` prefix).
        (
            "nros-platform-critical-section",
            "packages/platform/nros-platform-critical-section",
        ),
        // phase-291 (#211) — the zephyr-leaf build.rs bake helper; a
        // [build-dependencies] row in every zephyr rust example / ws entry.
        ("nros-zephyr-build", "packages/tooling/nros-zephyr-build"),
        (
            "nros-transport-callbacks",
            "packages/rmw/transport-callbacks",
        ),
        ("mps2-an385-pac", "packages/boards/mps2-an385-pac"),
        // RMW backends
        ("nros-rmw-zenoh", "packages/rmw/zenoh/nros-rmw-zenoh"),
        (
            "nros-rmw-zenoh-staticlib",
            "packages/rmw/zenoh/nros-rmw-zenoh-staticlib",
        ),
        (
            "nros-rmw-cyclonedds",
            "packages/rmw/cyclonedds/nros-rmw-cyclonedds",
        ),
        (
            "nros-rmw-cyclonedds-sys",
            "packages/rmw/cyclonedds/nros-rmw-cyclonedds-sys",
        ),
        ("nros-rmw-xrce-cffi", "packages/rmw/xrce/nros-rmw-xrce-cffi"),
        (
            "nros-rmw-xrce-cffi-staticlib",
            "packages/rmw/xrce/nros-rmw-xrce-cffi-staticlib",
        ),
        // Transport / SDKs that consumers regularly reference as `version = "*"`
        ("cyclonedds-sys", "packages/rmw/cyclonedds/cyclonedds-sys"),
    ]
}

/// Phase 220.E — scan a consumer `Cargo.toml` body for `nros-*`,
/// `nros`, or `cyclonedds-sys` deps declared registry-style (`version =
/// "*"` or bare `"*"`). Returns crate names sorted + deduped.
///
/// Walks `[dependencies]`, `[dev-dependencies]`, `[build-dependencies]`,
/// and any `[target.<cfg>.dependencies]`-shaped table. Loose TOML scanner
/// matching the existing `extract_cargo_path_deps` style — handles the
/// single-line `name = { version = "*", ... }` form which is the only
/// shape current nano-ros examples use.
///
/// Path-style deps (`path = "..."`) are intentionally skipped — the
/// user already pinned a concrete location, no patch needed.
fn extract_consumer_registry_nros_deps(body: &str) -> Vec<String> {
    use toml_edit::{DocumentMut, Item, Value};

    // Phase 265 (W2) — toml_edit DOM walk. The inline `name = { version = … }`
    // and explicit `[dependencies.<name>]` (dotted) forms collapse to the SAME
    // DOM shape (a table-like dep item), so issue #94 case B disappears. A
    // malformed manifest (won't parse) yields no extras — same as the old loose
    // scanner finding nothing.
    let doc: DocumentMut = match body.parse() {
        Ok(d) => d,
        Err(_) => return Vec::new(),
    };

    // Registry-style iff a version is declared (bare string, or a table — inline
    // or dotted — carrying a `version` key). A path-only table (canonical 212
    // path-dep) is skipped; a `version` + `path` table counts (the version
    // registers the dep in the crates.io namespace `[patch.crates-io]` operates on).
    fn is_registry_style(item: &Item) -> bool {
        match item {
            Item::Value(Value::String(_)) => true,
            Item::Value(Value::InlineTable(t)) => t.contains_key("version"),
            Item::Table(t) => t.contains_key("version"),
            _ => false,
        }
    }
    fn scan_deps(deps: Option<&Item>, out: &mut Vec<String>) {
        let Some(tbl) = deps.and_then(|i| i.as_table_like()) else {
            return;
        };
        for (name, item) in tbl.iter() {
            if is_managed_runtime_crate_name(name) && is_registry_style(item) {
                out.push(name.to_string());
            }
        }
    }

    let mut out: Vec<String> = Vec::new();
    let root = doc.as_table();
    for kind in ["dependencies", "dev-dependencies", "build-dependencies"] {
        scan_deps(root.get(kind), &mut out);
    }
    // `[target.<cfg>.<kind>]` tables.
    if let Some(target) = root.get("target").and_then(|i| i.as_table_like()) {
        for (_cfg, cfg_item) in target.iter() {
            if let Some(cfg_tbl) = cfg_item.as_table_like() {
                for kind in ["dependencies", "dev-dependencies", "build-dependencies"] {
                    scan_deps(cfg_tbl.get(kind), &mut out);
                }
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// True iff `name` is a crate the patch-block writer knows a workspace
/// path for. Restricts the 220.E extension surface to vetted names.
/// Names this manifest declares REGISTRY-style (a `version` key), regardless of
/// whether they are nros-managed. Used by the narrowing guard: only a dep the
/// consumer resolves by registry name can fall through to crates.io if its
/// `[patch.crates-io]` entry disappears. A PATH dep (RFC-0067 D1, phase-333 W1)
/// cannot — it names a directory, so dropping its patch is intentional and safe.
///
/// Without this distinction the guard reads `package.xml`'s `<depend>` rows —
/// which still list `std_msgs` because the leaf genuinely depends on those
/// messages — and blocks the very narrowing phase-333 performs.
fn registry_style_dep_names(body: &str) -> Vec<String> {
    use toml_edit::{DocumentMut, Item, Value};
    let Ok(doc) = body.parse::<DocumentMut>() else {
        return Vec::new();
    };
    // NOTE: deliberately different from `collect_extra_patch_names`'s local
    // `is_registry_style` — here `path` + `version` must NOT count: cargo
    // resolves such a dep from the path (never the registry), so a narrower
    // patch table cannot strand it. It is the RFC-0067 recommended spelling
    // (`path = "…", version = "0.0.0"`), and counting it made the narrowing
    // guard refuse legitimate narrowing for path-migrated local interface
    // pkgs (issue 0395).
    fn is_registry_style(item: &Item) -> bool {
        match item {
            Item::Value(Value::String(_)) => true,
            Item::Value(Value::InlineTable(t)) => {
                t.contains_key("version") && !t.contains_key("path")
            }
            Item::Table(t) => t.contains_key("version") && !t.contains_key("path"),
            _ => false,
        }
    }
    let mut out = Vec::new();
    let root = doc.as_table();
    for kind in ["dependencies", "dev-dependencies", "build-dependencies"] {
        if let Some(tbl) = root.get(kind).and_then(|i| i.as_table_like()) {
            for (name, item) in tbl.iter() {
                if is_registry_style(item) {
                    out.push(name.to_string());
                }
            }
        }
    }
    out
}

fn is_managed_runtime_crate_name(name: &str) -> bool {
    nros_crate_path_lookup().iter().any(|(n, _)| *n == name)
        // RFC-0040 D-Q3 — board crates are managed too (a scaffolded embedded
        // project deps `nros-board-<x> = "*"`). Their path is derived uniformly
        // (`packages/boards/<name>`), not enumerated in the static table.
        || name.starts_with("nros-board-")
}

/// RFC-0040 D-Q3 — map a managed crate name to its `<NROS_REPO_DIR>`-relative
/// subpath. Core/RMW crates come from the static [`nros_crate_path_lookup`]
/// table; board crates follow the uniform `packages/boards/<name>` convention,
/// so any current or future `nros-board-*` resolves without a table entry.
fn nros_crate_subpath(name: &str) -> Option<String> {
    if let Some((_, p)) = nros_crate_path_lookup().iter().find(|(n, _)| *n == name) {
        Some((*p).to_string())
    } else if name.starts_with("nros-board-") {
        Some(format!("packages/boards/{name}"))
    } else {
        None
    }
}

/// Crate names in a generated msg crate's `[dependencies]` /
/// `[build-dependencies]` / `[dev-dependencies]` tables (registry + path).
/// Used to walk the emitted msg-crate dep graph. toml_edit, like W2.
fn cargo_dependency_names(cargo_body: &str) -> Vec<String> {
    let Ok(doc) = cargo_body.parse::<toml_edit::DocumentMut>() else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for table in ["dependencies", "build-dependencies", "dev-dependencies"] {
        if let Some(t) = doc.get(table).and_then(|i| i.as_table_like()) {
            for (k, _) in t.iter() {
                out.push(k.to_string());
            }
        }
    }
    out
}

/// Phase-265 W5b — the transitive closure of `seeds` over the emitted msg-crate
/// dependency graph, intersected with `emitted`. A standalone consumer's patch
/// should carry only the generated msg crates it actually depends on (its
/// `package.xml` `<depend>` rows + their transitive msg deps) — NOT every crate
/// the sync run emitted. This excludes a node's own auto-generated self-crate
/// when nothing consumes it (e.g. `native/custom-msg` hand-codes its msgs inline
/// and uses `std_msgs`; its `msg/` dir still triggers self-codegen, but the
/// unconsumed self-crate must not land a broken `[patch.crates-io]` path entry).
fn emitted_msg_dep_closure(seeds: &[String], emitted: &[String], build_root: &Path) -> Vec<String> {
    let set: HashSet<&str> = emitted.iter().map(String::as_str).collect();
    let mut result: HashSet<String> = HashSet::new();
    let mut stack: Vec<String> = seeds
        .iter()
        .filter(|s| set.contains(s.as_str()))
        .cloned()
        .collect();
    while let Some(c) = stack.pop() {
        if !result.insert(c.clone()) {
            continue;
        }
        if let Ok(body) = std::fs::read_to_string(build_root.join(&c).join("Cargo.toml")) {
            for d in cargo_dependency_names(&body) {
                if set.contains(d.as_str()) && !result.contains(&d) {
                    stack.push(d);
                }
            }
        }
    }
    let mut v: Vec<String> = result.into_iter().collect();
    v.sort();
    v
}

/// Phase 265 (issue 0094) — the managed `(crate_name, relative_path)` patch entries
/// for a consumer authority, in emit order (generated msg crates first, then the
/// deduped + alphabetised runtime crates). Single source of the managed-set + path
/// policy behind the toml_edit `write_patch_config`. Paths are relative to the
/// authority's directory.
fn render_managed_entries(
    authority: &Path,
    build_root: &Path,
    pkgs: &[String],
    nano_ros_path: Option<&Path>,
    extra_runtime_crates: &[String],
    registry_named: &HashSet<String>,
) -> Result<Vec<(String, String)>> {
    let authority_dir = authority.parent().unwrap();
    let mut out: Vec<(String, String)> = Vec::new();
    // issue 0363 — crates whose lookup-table path is dead. Collected rather than
    // failing on the first, so one run reports every stale mapping.
    let mut stale_paths: Vec<(String, String)> = Vec::new();

    // 1) Generated msg crates: a patch entry ONLY for names a consumer still
    //    declares registry-style (RFC-0067 D1, phase-333 W1).
    //
    // A PATH dep (`std_msgs = { path = "generated/std_msgs" }`) needs no patch —
    // it never consults a registry, from any cwd — and leaving one would make
    // cargo warn about an unused patch. Emitting nothing at all was wrong in the
    // other direction, and broke `examples/workspaces/features`: `custom_msgs` is
    // a WORKSPACE-LOCAL msg package whose consumers still say `custom_msgs = "*"`,
    // because the phase-333 sweep converted a fixed list of standard ROS names
    // and cannot know a user's own package names. It is also the copy-out
    // contract (RFC-0026): a user's project declares registry-style and relies on
    // this block, and `nros sync` must keep working for them.
    //
    // So the emitted set follows the CONSUMER's spelling: registry name in, patch
    // out; path dep in, nothing. Self-healing — a leaf converted to path deps
    // simply stops receiving an entry on its next sync.
    for pkg in pkgs {
        if !registry_named.contains(pkg) {
            continue;
        }
        let crate_root = build_root.join(pkg);
        let rel = pathdiff::diff_paths(&crate_root, authority_dir).unwrap_or(crate_root);
        out.push((pkg.clone(), rel.display().to_string()));
    }

    if let Some(nrp) = nano_ros_path {
        let mut wanted: Vec<String> = vec!["nros-core".to_string(), "nros-serdes".to_string()];
        // Phase 244 E3 — scan each generated pkg's Cargo.toml for registry-style
        // runtime deps the consumer never names directly.
        let mut gen_extras: Vec<String> = Vec::new();
        for pkg in pkgs {
            if let Ok(gen_body) = std::fs::read_to_string(build_root.join(pkg).join("Cargo.toml")) {
                gen_extras.extend(extract_consumer_registry_nros_deps(&gen_body));
            }
        }
        for extra in extra_runtime_crates.iter().chain(gen_extras.iter()) {
            if nros_crate_subpath(extra).is_some() {
                if !wanted.iter().any(|w| w == extra) {
                    wanted.push(extra.clone());
                }
            } else {
                eprintln!(
                    "sync: unknown runtime crate `{extra}` referenced as registry dep; \
                     no path mapping in the nros lookup table — skipping patch entry."
                );
            }
        }
        wanted.sort();
        wanted.dedup();
        for cname in &wanted {
            let sub = nros_crate_subpath(cname).expect("cname is a managed crate; subpath exists");
            let crate_root = nrp.join(&sub);
            if !crate_root.join("Cargo.toml").is_file() {
                // issue 0363 — a crate that IS in the lookup table but whose path
                // does not exist means the TABLE is stale (a package moved), not
                // that the crate is optional. Every one of the 23 table paths is
                // an in-repo directory with a TRACKED Cargo.toml, so this can only
                // be staleness.
                //
                // Silently `continue`-ing here is how a stale `nros` binary emitted
                // a patch table missing `nros-zephyr-build` after phase-321 moved
                // it out of packages/core/: the dropped dependency then resolves
                // from crates.io instead of the checkout, which fails NOWHERE and
                // silently builds against the wrong source.
                //
                // Note the asymmetry this removes: an UNKNOWN crate name already
                // warned loudly a few lines above; a known crate with a dead path
                // was the quiet one.
                eprintln!(
                    "sync: ERROR — managed crate `{cname}` maps to `{sub}`, which \
                     does not exist under {}.\n\
                     \x20 The nros lookup table is stale for this crate (a package \
                     moved?). Refusing to emit a patch table that silently omits it; \
                     rebuild the CLI (`./scripts/bootstrap.sh`, contributors `just setup-cli`) or fix nros_crate_path_lookup.",
                    nrp.display()
                );
                stale_paths.push((cname.clone(), sub.clone()));
                continue;
            }
            let rel = pathdiff::diff_paths(&crate_root, authority_dir).unwrap_or(crate_root);
            out.push((cname.clone(), rel.display().to_string()));
        }
    }
    if !stale_paths.is_empty() {
        // Hard stop. The alternative is writing a table we KNOW is incomplete,
        // and an incomplete [patch.crates-io] is worse than a stale one: a stale
        // path fails loudly at build, a missing entry resolves from crates.io.
        let list = stale_paths
            .iter()
            .map(|(n, s)| format!("  {n} -> {s}"))
            .collect::<Vec<_>>()
            .join("\n");
        eyre::bail!(
            "sync: {} managed crate(s) have a dead path in the nros lookup table:\n{list}\n\
             Refusing to write an incomplete [patch.crates-io] — a missing entry \
             resolves that dependency from crates.io instead of this checkout, which \
             fails nowhere. Rebuild the CLI (`./scripts/bootstrap.sh`, contributors `just setup-cli`); if that does not help, \
             nros_crate_path_lookup is stale.",
            stale_paths.len()
        );
    }
    Ok(out)
}

/// Phase 265 (issue 0094) — decor suffix tagging a sync-owned `[patch.crates-io]`
/// entry in a `.cargo/config.toml`. Distinguishes managed entries from user keys
/// (a hand `libc` patch, etc.) so re-sync evicts only its own.
const NROS_MANAGED_TAG: &str = "nros-managed";

/// True if a `[patch.crates-io]` value carries the `# nros-managed` decor marker.
fn item_is_nros_managed(item: &toml_edit::Item) -> bool {
    item.as_value()
        .and_then(|v| v.decor().suffix())
        .and_then(|s| s.as_str())
        .map(|s| s.contains(NROS_MANAGED_TAG))
        .unwrap_or(false)
}

/// Phase 265 (issue 0094) — write the managed `[patch.crates-io]` entries into
/// `<authority_dir>/.cargo/config.toml` via a format-preserving `toml_edit` DOM
/// (replacing the line-based `Cargo.toml` splice). Each managed entry is tagged
/// with a `# nros-managed` decor suffix; on re-sync only tagged keys are evicted,
/// so user content (a hand `libc` patch, `[target]`/`[env]` sections) is preserved.
/// Atomic temp + rename. Creates `.cargo/config.toml` if absent; removes an emptied
/// `[patch.crates-io]` / `[patch]` table.
fn write_patch_config(
    authority_dir: &Path,
    managed: &[(String, String)],
    include_rel: Option<&str>,
    sidecar: bool,
) -> Result<()> {
    let cfg_dir = authority_dir.join(".cargo");
    let cfg = cfg_dir.join("config.toml");
    let text = std::fs::read_to_string(&cfg).unwrap_or_default();

    // Issue 0827 — the derived pool budgets, computed BEFORE the config is
    // rendered because the render needs to know whether the sidecar will exist.
    // Deciding by "am I about to write it" rather than by guessing is what keeps
    // the file and its `include` entry in step (issue 0463).
    //
    // Only for an in-tree leaf: an out-of-tree consumer gets no `include` at all
    // (#272), so a sidecar it could not reference would be a file nothing reads.
    let derived_env: Option<String> = if sidecar {
        render_leaf_env_sidecar(authority_dir)
    } else {
        None
    };
    let out = render_patch_config_with(&text, managed, include_rel, sidecar, derived_env.is_some())
        .wrap_err_with(|| format!("sync: edit {}", cfg.display()))?;

    // Atomic write (create `.cargo/` first).
    std::fs::create_dir_all(&cfg_dir)
        .wrap_err_with(|| format!("sync: mkdir {}", cfg_dir.display()))?;

    // Issue 0457 — the managed `[patch.crates-io]` block lives in its OWN
    // gitignored file, not in `config.toml`.
    //
    // The entries name generated message crates, whose existence and contents
    // come from the USER's ament install; the set also shifts as a leaf's
    // dependency graph resolves. Writing them into a TRACKED `config.toml`
    // made every sync dirty the worktree (the same `nros-log` row appearing
    // and disappearing across runs), and it committed host-derived paths —
    // the rule that already keeps `generated/` and `nros-patch.toml` out of
    // git. But the rest of the file (a `[build] target`, a QEMU `runner`,
    // linker rustflags) genuinely cannot be regenerated and MUST survive a
    // clone, so the file itself cannot simply be gitignored.
    //
    // Splitting on that seam gives both: `config.toml` keeps only authored
    // content plus an `include`, and everything sync owns sits beside it in a
    // file git never sees.
    // Issue 0463 — only the `generated/` rows are host-specific, so only they
    // go to the sidecar. Everything else is an in-repo relative path and stays
    // in the tracked config, where a clone can use it without running sync.
    let generated: Vec<(String, String)> = managed
        .iter()
        .filter(|(_, rel)| is_generated_path(rel))
        .cloned()
        .collect();
    let managed_path = cfg_dir.join(MANAGED_PATCH_FILE);
    if generated.is_empty() || !sidecar {
        // Nothing host-specific — leave no stale file behind.
        let _ = std::fs::remove_file(&managed_path);
    } else {
        // Same temp+rename as every other sync-owned file — one spelling,
        // in [`atomic_write`] (byte-identical temp names to what this wrote
        // inline before phase-341).
        atomic_write(&managed_path, &render_managed_patch_file(&generated))?;
    }

    // Issue 0827 — same temp+rename and the same "no stale file" rule as the
    // patch sidecar above.
    let env_path = cfg_dir.join(MANAGED_ENV_FILE);
    match &derived_env {
        Some(body) => atomic_write(&env_path, body)?,
        None => {
            let _ = std::fs::remove_file(&env_path);
        }
    }

    atomic_write(&cfg, &out)?;
    Ok(())
}

/// Issue 0827 — the `[env]` body for this leaf, or `None` when there is nothing
/// to state.
///
/// `None` on every path that is not a confident derivation, and the cases are
/// deliberately different from each other:
///
/// * no `metadata/` directory, or no probeable component in it — nothing ran, so
///   there is nothing to say;
/// * the inventory REFUSES (`Derivation::Refused`) — it says why, and a refusal
///   is not a budget;
/// * a parse failure — reported, and then treated as "no sidecar", because a
///   budget derived from a half-read probe can be SHORT, and short halts the
///   board. The leaf keeps the crate defaults, which are large rather than wrong.
fn render_leaf_env_sidecar(leaf: &Path) -> Option<String> {
    use crate::{
        entity_inventory::Derivation,
        leaf_entity_env::{inventory_for_leaf, render_env_sidecar},
    };

    let (inv, unprobeable) = match inventory_for_leaf(leaf) {
        Ok(v) => v,
        Err(e) => {
            eprintln!(
                "sync: {}: cannot derive pool budgets ({e}); leaving the crate defaults in place",
                leaf.display()
            );
            return None;
        }
    };
    if inv.is_empty() {
        // An unprobeable component is why a leaf can have `metadata/` and still
        // get no budget. Say so once rather than leaving it to be inferred from
        // an absent file (issue 1061).
        if !unprobeable.is_empty() {
            eprintln!(
                "sync: {}: {} component(s) are un-probeable, so pool budgets stay at the crate \
                 defaults (issue 1061): {}",
                leaf.display(),
                unprobeable.len(),
                unprobeable.join(", ")
            );
        }
        return None;
    }
    match inv.derive() {
        Derivation::Derived(knobs) => {
            // Issue 1125 — the payload classes need a SECOND inventory (the
            // per-type bounds `nros sync` has already written into
            // `generated/`), so they are computed here and refuse
            // independently: an entity budget can derive while a subscribed
            // type carries no bound, and the reverse cannot happen.
            let payload = crate::leaf_payload_classes::payload_classes_for_leaf(leaf, &inv);
            if let crate::leaf_payload_classes::PayloadClasses::Refused { reason } = &payload {
                eprintln!(
                    "sync: {}: payload classes not derived, so `LARGE_PAYLOADS` keeps the \
                     crate default (issue 1125): {reason}",
                    leaf.display()
                );
            }
            Some(render_env_sidecar(&knobs, &payload, &inv.source))
        }
        other => {
            eprintln!(
                "sync: {}: pool budgets not derived ({}); crate defaults stay",
                leaf.display(),
                other.tag()
            );
            None
        }
    }
}

/// Issue 0457 — basename of the per-leaf gitignored file holding sync's managed
/// `[patch.crates-io]` block. Sibling of `config.toml` inside `.cargo/`, reached
/// by an `include` entry that `render_patch_config` maintains.
const MANAGED_PATCH_FILE: &str = "nros-managed-patch.toml";

/// Issue 0827 — basename of the per-leaf gitignored file holding sync's derived
/// pool budgets as `[env]`.
///
/// A second sidecar rather than a section in the patch one, because the two
/// answer different questions and empty independently: a leaf can have a
/// generated message dep and no probeable component, or the reverse. Sharing a
/// file would make each one's presence depend on the other's.
///
/// Same appear-and-disappear-together rule as [`MANAGED_PATCH_FILE`], and for
/// the same reason (issue 0463): a missing `include` target is a HARD cargo
/// error during manifest parse, so the entry exists exactly when the file does.
const MANAGED_ENV_FILE: &str = "nros-managed-env.toml";

/// Render the standalone managed-patch file: a header saying who owns it and a
/// single `[patch.crates-io]` table of the managed entries, alphabetised.
///
/// No `# nros-managed` decor markers here — the whole FILE is managed, so the
/// per-key tag that distinguished sync's rows from user rows inside a shared
/// table has nothing left to distinguish.
/// Issue 0463 — a managed row is host-specific iff its path points into a
/// `generated/` tree: those crates are produced per host by `nros sync` from the
/// consumer's ament install. Every other row is an in-repo crate at a relative
/// path that is the same in any checkout.
fn is_generated_path(rel: &str) -> bool {
    rel.split(['/', '\\']).any(|seg| seg == "generated")
}

fn render_managed_patch_file(managed: &[(String, String)]) -> String {
    let mut sorted: Vec<(String, String)> = managed.to_vec();
    sorted.sort_by(|a, b| a.0.cmp(&b.0));
    sorted.dedup_by(|a, b| a.0 == b.0);

    let mut out = String::new();
    out.push_str(
        "# Generated by `nros sync` — do not edit, do not commit (gitignored).\n\
         #\n\
         # Issue 0457: these entries point at `generated/` message crates built from\n\
         # the USER's ament install, so they are host-specific and their membership\n\
         # moves as the leaf's dependency graph resolves. They live here rather than\n\
         # in `.cargo/config.toml` so that a TRACKED config.toml — which carries the\n\
         # authored `[build]` / `runner` / rustflags a clone cannot regenerate —\n\
         # never churns and never commits a host-derived path.\n\
         #\n\
         # Recreate with: nros sync\n\
         \n\
         [patch.crates-io]\n",
    );
    for (name, rel) in &sorted {
        out.push_str(&format!("{name} = {{ path = \"{rel}\" }}\n"));
    }
    out
}

// --- Board `cargo_config` projection (phase-341 W2) ----------------------------
//
// A board's `cargo_config` in `nros-board.toml` is the SSoT for the leaf's
// `[build] target` / `[unstable] build-std` / `[target.<triple>]` link group
// (RFC-0032's "third leg"). Until now each leaf carried a HAND-COPIED mirror of
// it, which is the mirror-drift class: issue 0440 lost the NuttX kernel-archive
// group in a package collapse and produced ~3680 undefined references, visible
// only at link time on one platform.
//
// Sync projects the descriptor into `<leaf>/.cargo/nros-board.toml` instead,
// reached by a third `include` entry.
//
// COMMITTED, not gitignored — unlike the `nros-managed-patch.toml` sidecar
// (issue 0457), whose rows name `generated/` trees built from the USER's ament
// install and are therefore host-derived. A board's `cargo_config` is a fixed
// string in a committed descriptor, identical in every checkout, and gitignoring
// its projection would mean a fresh clone could not LINK an embedded leaf until
// sync ran. Same shape as `packages/core/*/src/generated.rs`: generate, commit,
// gate (phase-341 W4).

/// Basename of the generated projection, sibling of `config.toml` inside
/// `.cargo/` and therefore reachable by the bare name from an `include`.
const BOARD_CONFIG_FILE: &str = "nros-board.toml";

/// `[package.metadata.nros.entry] deploy` — the board this leaf deploys to.
/// `None` when the leaf is not an Entry pkg (a node/library crate), when the key
/// is absent, or when the manifest does not parse (every other reader of that
/// file will produce a better error than this one could).
fn entry_deploy_key(cargo_toml_body: &str) -> Option<String> {
    let v: toml::Value = toml::from_str(cargo_toml_body).ok()?;
    let deploy = v
        .get("package")?
        .get("metadata")?
        .get("nros")?
        .get("entry")?
        .get("deploy")?
        .as_str()?
        .trim();
    (!deploy.is_empty()).then(|| deploy.to_string())
}

/// The leaf-relative prefix that reaches the nano-ros root — `"../"` per path
/// component, or `""` for a leaf that IS the root.
///
/// This is what a `${workspace}` placeholder resolves to. Cargo resolves both a
/// config `[patch]` path and an `[env]` `relative = true` value against the
/// parent of the `.cargo/` directory — the leaf dir — so one prefix serves both,
/// and the result is identical in every checkout (which is what makes it
/// committable). `None` for a leaf outside the checkout, which #272 already
/// declines to write into.
fn leaf_to_root_prefix(leaf_dir: &Path, nano_ros_root: &Path) -> Option<String> {
    let leaf = leaf_dir
        .canonicalize()
        .unwrap_or_else(|_| leaf_dir.to_path_buf());
    let root = nano_ros_root
        .canonicalize()
        .unwrap_or_else(|_| nano_ros_root.to_path_buf());
    let depth = leaf.strip_prefix(&root).ok()?.components().count();
    Some("../".repeat(depth))
}

/// A descriptor's `cargo_config`, rendered for THIS leaf: `${workspace}`
/// resolved leaf-relative, `[patch]` removed.
///
/// phase-351 W3. This replaces a filter that *withheld* every top-level key
/// carrying `${workspace}` and named it in the header — a filter with no
/// destination. What it withheld was not host-specific at all: every such path
/// points INSIDE the repo (`third-party/nuttx/libc`, a board's own `config/`),
/// so it is the same in every checkout the moment it is written relative to the
/// leaf rather than absolute. That is issue 0463's rule, which the withholding
/// predated: rows split by ORIGIN — in-repo relative rows are committable, only
/// host-derived ones are not.
///
/// `[env]` values become `{ value = "<rel>", relative = true }`, because a bare
/// `[env]` string is passed through verbatim and a relative path would then be
/// read against the process CWD, which is not the leaf for a cmake/corrosion
/// build. `force` is preserved where the descriptor set it.
///
/// **`[patch]` is removed, not rendered.** Not because it cannot be expressed —
/// [`board_patch_rows`] expresses it — but because it cannot be *delivered
/// here*: the leaf's own `config.toml` always carries `[patch.crates-io]` (sync
/// writes the board-crate rows into it), and a projected `[patch.crates-io]`
/// would collide with it in `board_projection_conflicts`, which drops the
/// `include` for the WHOLE file. So patch rows ride sync's managed inline set
/// instead — same file the leaf already has, same `# nros-managed` decor.
fn project_board_config(raw: &str, leaf_prefix: &str) -> Result<String> {
    use toml_edit::{Item, Value};

    fn resolve(v: &mut Value, prefix: &str, in_env: bool) {
        match v {
            Value::String(s) => {
                if !s.value().contains("${workspace}") {
                    return;
                }
                let resolved = s.value().replace("${workspace}/", prefix);
                if in_env {
                    let mut t = toml_edit::InlineTable::new();
                    t.insert("value", resolved.into());
                    t.insert("relative", true.into());
                    *v = Value::InlineTable(t);
                } else {
                    *v = resolved.into();
                }
            }
            Value::Array(a) => a.iter_mut().for_each(|e| resolve(e, prefix, false)),
            Value::InlineTable(t) => {
                // An `[env]` row is already a table when it carries `force`; the
                // placeholder lives in its `value`, and `relative` has to join it.
                let is_env_row = in_env && t.contains_key("value");
                let mut needs_relative = false;
                for (k, inner) in t.iter_mut() {
                    if let Value::String(s) = inner
                        && s.value().contains("${workspace}")
                    {
                        *inner = s.value().replace("${workspace}/", prefix).into();
                        needs_relative |= is_env_row && k == "value";
                    } else {
                        resolve(inner, prefix, false);
                    }
                }
                if needs_relative {
                    t.insert("relative", true.into());
                    // Normalise the whole inline table: inserting after a value
                    // that already carries trailing decor renders `force = true
                    // , relative = true`, and this file is committed.
                    t.fmt();
                }
            }
            _ => {}
        }
    }

    fn walk(item: &mut Item, prefix: &str, in_env: bool) {
        match item {
            Item::Value(v) => resolve(v, prefix, in_env),
            Item::Table(t) => t.iter_mut().for_each(|(_, i)| walk(i, prefix, in_env)),
            Item::ArrayOfTables(a) => a
                .iter_mut()
                .for_each(|t| t.iter_mut().for_each(|(_, i)| walk(i, prefix, in_env))),
            Item::None => {}
        }
    }

    let mut doc: toml_edit::DocumentMut = raw
        .parse()
        .wrap_err("parse a board descriptor's `cargo_config`")?;
    doc.as_table_mut().remove("patch");
    for (key, item) in doc.as_table_mut().iter_mut() {
        walk(item, leaf_prefix, key == "env");
    }
    Ok(doc.to_string())
}

/// The `[patch.crates-io]` rows a board descriptor declares, as
/// `(crate, leaf-relative path)` — sync's managed-entry shape.
///
/// phase-351 W3. `nros-board-nuttx-qemu` declares the NuttX-patched `libc`
/// fork that `-Z build-std` needs, and until now that declaration reached a
/// leaf by THREE separate spellings: the descriptor row (withheld, so inert),
/// a hand-authored row in each of the twelve NuttX leaf configs, and
/// `scripts/build/nuttx-libc-patch.sh`, a shell re-append written against
/// "nros 0.3.7" whose header says it exists "until the upstream CLI bug is
/// fixed" — the CLI has been in-tree since phase 218. Delivering the row from
/// the descriptor retires the other two.
fn board_patch_rows(cargo_config: &str, leaf_prefix: &str) -> Result<Vec<(String, String)>> {
    let doc: toml::Value = toml::from_str(cargo_config)
        .wrap_err("parse a board descriptor's `cargo_config` for [patch] rows")?;
    let Some(patch) = doc.get("patch").and_then(|p| p.as_table()) else {
        return Ok(Vec::new());
    };
    let mut out = Vec::new();
    for (_registry, rows) in patch {
        let Some(rows) = rows.as_table() else {
            continue;
        };
        for (name, spec) in rows {
            let Some(path) = spec.get("path").and_then(|p| p.as_str()) else {
                continue;
            };
            out.push((name.clone(), path.replace("${workspace}/", leaf_prefix)));
        }
    }
    out.sort();
    Ok(out)
}

/// The DO-NOT-EDIT projection: header naming the descriptor it came from, then
/// the rendered `cargo_config`.
fn render_board_config(deploy: &str, descriptor: &str, body: &str) -> String {
    let mut out = format!(
        "# GENERATED by `nros sync` — DO NOT EDIT.\n\
         #\n\
         # Projection of the board `cargo_config` for `deploy = \"{deploy}\"`.\n\
         # SSoT: {descriptor}\n\
         #\n\
         # This file IS committed (unlike sync's `nros-managed-patch.toml`): its\n\
         # content is a fixed string in a committed descriptor, identical in every\n\
         # checkout, and a fresh clone must be able to LINK this leaf before any\n\
         # sync has run. Edit the descriptor and re-run `nros sync`; editing here\n\
         # is drift, which phase-341 exists to make uncommittable (issue 0440).\n\
         #\n\
         # RFC-0032 \"third leg\" / docs/roadmap/phase-341-*.\n\
         #\n\
         # phase-351 W3 — `${{workspace}}` paths are rendered RELATIVE to this leaf\n\
         # (they point inside the repo, so they are the same in every checkout).\n\
         # `[patch]` rows are NOT here: they ride sync's `# nros-managed` entries in\n\
         # the sibling `config.toml`, because a `[patch.crates-io]` in this file\n\
         # would collide with the one that already lives there.\n"
    );
    out.push('\n');
    out.push_str(body.trim_end_matches('\n'));
    out.push('\n');
    out
}

/// Every config key in `text`, to a depth of two (`build.target`,
/// `target.thumbv7m-none-eabi`, `env`), as a set.
///
/// Depth two is the level at which cargo's config merge stops being harmless:
/// two files declaring `[target.<triple>] rustflags` have their arrays JOINED,
/// not overridden, so an included projection that repeats a leaf's still-present
/// mirror would hand the linker `-Tlink.x -Tlink.x`. Depth one would be
/// needlessly coarse (a leaf `[build] target` beside a projection `[build]
/// rustflags` merges fine); depth three would be too fine (that array join is
/// exactly the case we must refuse).
fn config_keys_depth2(text: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let Ok(toml::Value::Table(top)) = toml::from_str::<toml::Value>(text) else {
        return out;
    };
    for (k, v) in top {
        match v {
            toml::Value::Table(sub) => {
                for (sk, _) in sub {
                    out.insert(format!("{k}.{sk}"));
                }
            }
            _ => {
                out.insert(k);
            }
        }
    }
    out
}

/// Keys the leaf's tracked `config.toml` and the projection BOTH declare.
///
/// Non-empty ⇒ this leaf still carries its hand-mirrored block, so the `include`
/// must NOT be added yet: the projection is written (so W3 and the W4
/// regeneration gate have something to compare) but nothing reads it, and the
/// leaf keeps linking exactly as it does today. W3's migration is then a pure
/// DELETION — drop the mirrored table, re-run sync, and the include appears.
fn board_projection_conflicts(existing_cfg: &str, projection_body: &str) -> Vec<String> {
    let leaf = config_keys_depth2(existing_cfg);
    let board = config_keys_depth2(projection_body);
    leaf.intersection(&board).cloned().collect()
}

/// Add (or evict) the projection's `include` entry, with the same
/// evict-then-re-add discipline [`render_patch_config_with`] uses for the
/// central + sidecar entries: our entry is recognised by basename, removed
/// unconditionally, and re-added only when the file it names is one we just
/// wrote. Issue 0463 — a missing include target is a HARD error during MANIFEST
/// PARSE, so an entry pointing at a file no generator wrote does not degrade the
/// leaf, it makes the leaf unreadable.
fn render_board_include(existing: &str, present: bool) -> Result<String> {
    use toml_edit::{DocumentMut, value};

    let mut doc: DocumentMut = existing.parse().wrap_err("parse .cargo/config.toml")?;
    {
        // Same rule as `render_patch_config_with`: decide first, and touch the
        // document only when the membership actually changes. An unconditional
        // evict-then-re-add is not formatting-neutral (toml_edit carries
        // per-element decor), so it rewrote every tracked leaf config with a
        // one-space difference and left the tree permanently dirty after a
        // sync. It also bumped the mtime, re-staling fixtures keyed on the leaf.
        let current: Vec<String> = doc
            .as_table()
            .get("include")
            .and_then(|i| i.as_value())
            .and_then(|v| v.as_array())
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_string))
                    .collect()
            })
            .unwrap_or_default();
        let mut desired: Vec<String> = current
            .iter()
            .filter(|s| !s.ends_with(BOARD_CONFIG_FILE))
            .cloned()
            .collect();
        if present {
            desired.push(BOARD_CONFIG_FILE.to_string());
        }

        if current != desired {
            let inc_item = doc
                .as_table_mut()
                .entry("include")
                .or_insert_with(|| value(toml_edit::Array::new()));
            let arr = inc_item
                .as_value_mut()
                .and_then(|v| v.as_array_mut())
                .ok_or_else(|| eyre!("sync: `include` is not an array"))?;
            arr.retain(|v| {
                v.as_str()
                    .map(|s| !s.ends_with(BOARD_CONFIG_FILE))
                    .unwrap_or(true)
            });
            if present {
                arr.push(BOARD_CONFIG_FILE);
            }
            if arr.is_empty() {
                doc.as_table_mut().remove("include");
            }
        }
    }
    Ok(doc.to_string())
}

/// What the projection pass did to one leaf — for the summary sync prints.
#[derive(Debug, PartialEq, Eq)]
enum BoardProjection {
    /// Written, and the leaf's `include` now reaches it.
    Included,
    /// Written, but the leaf still carries a conflicting hand-mirrored block,
    /// so no `include` was added (phase-341 W3 removes the mirror).
    ShadowedByMirror(Vec<String>),
    /// Nothing to project — the board declares no `cargo_config` (or only a
    /// `[patch]`, which sync delivers inline instead). Any prior projection +
    /// include is removed.
    NoBoardConfig,
}

/// Write (or remove) one leaf's `.cargo/nros-board.toml` and maintain its
/// `include` entry.
///
/// Ordering is the 0463 invariant: the file is written BEFORE the include that
/// names it, and the include is dropped BEFORE the file it names.
///
/// Idempotent — both files are compared before writing, so a re-sync that
/// changes nothing does not touch an mtime (every prebuilt fixture keyed on
/// these inputs would otherwise read STALE).
fn write_board_projection(
    leaf_dir: &Path,
    nano_ros_root: &Path,
    deploy: &str,
    descriptor: &crate::orchestration::board_descriptor::BoardDescriptor,
) -> Result<BoardProjection> {
    let cfg_dir = leaf_dir.join(".cargo");
    let cfg = cfg_dir.join("config.toml");
    let dst = cfg_dir.join(BOARD_CONFIG_FILE);
    let existing_cfg = std::fs::read_to_string(&cfg).unwrap_or_default();

    // An error here means the descriptor's `cargo_config` is not valid TOML — a
    // hard failure, because every other consumer of that string is about to hit
    // the same thing and this is the only place that can name the board.
    let projected = render_board_projection_body(leaf_dir, nano_ros_root, descriptor)
        .wrap_err_with(|| {
            format!(
                "sync: board `{}` ({})",
                descriptor.names.join("/"),
                descriptor.source.as_deref().unwrap_or("nros-board.toml"),
            )
        })?;

    // "Nothing to project" is measured in KEYS, not bytes: a body left holding
    // only a stray comment is still nothing a leaf can inherit.
    if config_keys_depth2(&projected).is_empty() {
        // Drop our include FIRST (so nothing ever names a file that is gone),
        // then the file.
        if !existing_cfg.is_empty() {
            let out = render_board_include(&existing_cfg, false)
                .wrap_err_with(|| format!("sync: edit {}", cfg.display()))?;
            if out != existing_cfg {
                atomic_write(&cfg, &out)?;
            }
        }
        if dst.exists() {
            std::fs::remove_file(&dst)
                .wrap_err_with(|| format!("sync: remove {}", dst.display()))?;
        }
        return Ok(BoardProjection::NoBoardConfig);
    }

    let body = render_board_config(
        deploy,
        descriptor.source.as_deref().unwrap_or("nros-board.toml"),
        &projected,
    );
    if std::fs::read_to_string(&dst).ok().as_deref() != Some(body.as_str()) {
        std::fs::create_dir_all(&cfg_dir)
            .wrap_err_with(|| format!("sync: mkdir {}", cfg_dir.display()))?;
        atomic_write(&dst, &body)?;
    }

    // Conflicts are computed against what was PROJECTED, not the raw descriptor:
    // `[patch]` never reaches the projection (sync delivers those rows inline),
    // so the leaf's own `[patch.crates-io]` is not a duplicate of anything here.
    let conflicts = board_projection_conflicts(&existing_cfg, &projected);
    let out = render_board_include(&existing_cfg, conflicts.is_empty())
        .wrap_err_with(|| format!("sync: edit {}", cfg.display()))?;
    if out != existing_cfg {
        std::fs::create_dir_all(&cfg_dir)
            .wrap_err_with(|| format!("sync: mkdir {}", cfg_dir.display()))?;
        atomic_write(&cfg, &out)?;
    }
    Ok(if conflicts.is_empty() {
        BoardProjection::Included
    } else {
        BoardProjection::ShadowedByMirror(conflicts)
    })
}

/// The projected body for one leaf.
///
/// ONE renderer, shared by the writer and the checker. They used to compute
/// this separately, which is a second spelling of "what a fresh render is" —
/// and the checker exists precisely to compare against a fresh render.
fn render_board_projection_body(
    leaf_dir: &Path,
    nano_ros_root: &Path,
    descriptor: &crate::orchestration::board_descriptor::BoardDescriptor,
) -> Result<String> {
    let prefix = leaf_to_root_prefix(leaf_dir, nano_ros_root).unwrap_or_default();
    let projected = match descriptor.cargo_config.as_deref() {
        Some(raw) => project_board_config(raw, &prefix)?,
        None => String::new(),
    };
    // phase-351 W6 — the `[env] NROS_BOARD_TOML` row phase-349 W2.0 wrote here
    // is GONE. It pointed at the descriptor so a build script could read the
    // board rung, and it could only ever work for a STANDALONE leaf: cargo
    // discovers config from the invocation CWD upward, and corrosion runs cargo
    // from `workspace_toml_dir`, so no workspace member ever read it. W5 moved
    // delivery to the invoker (`nros ws board-facts`, one resolution for every
    // lane), which reaches both.
    Ok(projected)
}

/// phase-341 W4 — the regeneration check that REPLACES
/// `check-board-cargo-config-applied`.
///
/// That gate existed because the leaf mirrored the descriptor by hand, so it
/// asked "does the leaf still carry a REPRESENTATIVE arg from its board?" —
/// deliberately loose, catching a lost GROUP but not a lost argument. Once the
/// block is a projection the question changes: not "did a human copy enough of
/// it" but "is the committed file what the descriptor renders to". That is an
/// exact comparison, and it makes drift uncommittable rather than detectable.
///
/// Shares `render_board_config` with [`write_board_projection`] — checking with
/// a second implementation of the renderer is how the two spellings drift, which
/// is the failure this whole phase exists to remove.
///
/// Returns one human-readable complaint per leaf that disagrees.
fn check_board_projection(
    leaf_dir: &Path,
    nano_ros_root: &Path,
    deploy: &str,
    descriptor: &crate::orchestration::board_descriptor::BoardDescriptor,
) -> Result<Option<String>> {
    let cfg_dir = leaf_dir.join(".cargo");
    let dst = cfg_dir.join(BOARD_CONFIG_FILE);
    let projected = render_board_projection_body(leaf_dir, nano_ros_root, descriptor)?;

    let on_disk = std::fs::read_to_string(&dst).ok();
    if config_keys_depth2(&projected).is_empty() {
        return Ok(on_disk.map(|_| {
            format!(
                "{}: has a projection, but board `{}` projects nothing — stale file",
                dst.display(),
                deploy
            )
        }));
    }

    let expected = render_board_config(
        deploy,
        descriptor.source.as_deref().unwrap_or("nros-board.toml"),
        &projected,
    );
    match on_disk {
        None => Ok(Some(format!(
            "{}: MISSING — board `{}` projects a config but the leaf has none",
            dst.display(),
            deploy
        ))),
        Some(body) if body != expected => Ok(Some(format!(
            "{}: STALE — does not match a fresh render of {}",
            dst.display(),
            descriptor.source.as_deref().unwrap_or("nros-board.toml"),
        ))),
        Some(_) => Ok(None),
    }
}

/// phase-341 W2 — project every Entry leaf's board `cargo_config` into
/// `<leaf>/.cargo/nros-board.toml`.
///
/// Deliberately CONSERVATIVE, because the failure it guards against is invisible
/// until link time (issue 0440) and this writer runs over ~700 leaf configs
/// (issues 0457 / 0463 are what happens when one is wrong):
///
/// * no nano-ros checkout, or no readable board catalog ⇒ touch NOTHING. The
///   projection is committed, so a clone already has it; skipping is always safe,
///   whereas "clean up what I cannot see" would delete a committed file.
/// * a `deploy` no descriptor claims, or one claimed by several ⇒ touch NOTHING
///   for that leaf and say so once at the end.
/// * out-of-tree consumers ⇒ skipped, like #272 skips the `include` for them.
fn project_board_configs(
    leaves: &[&WsPkg],
    nano_ros_path: Option<&Path>,
    verbose: bool,
) -> Result<()> {
    project_board_configs_with(leaves, nano_ros_path, verbose, false).map(|_| ())
}

/// [`project_board_configs`] with the write/check split made explicit.
/// `check` writes NOTHING and returns the leaves whose committed projection
/// disagrees with a fresh render (phase-341 W4).
fn project_board_configs_with(
    leaves: &[&WsPkg],
    nano_ros_path: Option<&Path>,
    verbose: bool,
    check: bool,
) -> Result<Vec<String>> {
    use crate::orchestration::board_descriptor::{BoardCatalog, DeployResolution};

    let mut complaints: Vec<String> = Vec::new();
    let Some(nrp) = nano_ros_path else {
        return Ok(complaints);
    };
    let nrp_c = nrp.canonicalize().unwrap_or_else(|_| nrp.to_path_buf());
    let catalog = match BoardCatalog::load(&nrp_c) {
        Ok(c) => c,
        Err(e) => {
            // Not fatal: a consumer whose NROS_REPO_DIR has no `packages/boards`
            // simply has no board knowledge to project. Never silent, because a
            // leaf that expected a projection will otherwise fail much later.
            println!("sync: board configs not projected (no board catalog: {e})");
            return Ok(complaints);
        }
    };

    let mut included = 0usize;
    let mut shadowed = 0usize;
    let mut unresolved: BTreeSet<String> = BTreeSet::new();
    for leaf in leaves {
        // #272 — an out-of-tree consumer's leaf is not ours to write generated,
        // committed content into.
        let leaf_c = leaf.dir.canonicalize().unwrap_or_else(|_| leaf.dir.clone());
        if !leaf_c.starts_with(&nrp_c) {
            continue;
        }
        let Ok(manifest) = std::fs::read_to_string(leaf.dir.join("Cargo.toml")) else {
            continue;
        };
        let Some(deploy) = entry_deploy_key(&manifest) else {
            continue; // not an Entry pkg — no board to inherit from
        };
        let descriptor = match catalog.resolve_deploy(&deploy) {
            DeployResolution::Board(d) => d,
            DeployResolution::Unknown => {
                unresolved.insert(format!("{deploy} (no board descriptor claims it)"));
                continue;
            }
            DeployResolution::Ambiguous(cands) => {
                unresolved.insert(format!("{deploy} (claimed by {})", cands.join(", ")));
                continue;
            }
        };
        if check {
            if let Some(c) = check_board_projection(&leaf.dir, &nrp_c, &deploy, descriptor)? {
                complaints.push(c);
            }
            continue;
        }
        match write_board_projection(&leaf.dir, &nrp_c, &deploy, descriptor)? {
            BoardProjection::Included => {
                included += 1;
                if verbose {
                    println!(
                        "sync: board config → {}",
                        leaf.dir.join(".cargo").join(BOARD_CONFIG_FILE).display()
                    );
                }
            }
            BoardProjection::ShadowedByMirror(keys) => {
                shadowed += 1;
                if verbose {
                    println!(
                        "sync: board config written but not included for {} — its \
                         tracked config still declares {} (phase-341 W3 removes the mirror)",
                        leaf.dir.display(),
                        keys.join(", ")
                    );
                }
            }
            BoardProjection::NoBoardConfig => {}
        }
    }
    if included + shadowed > 0 {
        println!(
            "sync: board configs — {included} leaf/leaves include \
             .cargo/{BOARD_CONFIG_FILE}, {shadowed} still governed by their own \
             [target.*] block (phase-341 W3)"
        );
    }
    if !unresolved.is_empty() {
        // Loud but not fatal: these leaves keep the hand-mirrored block they
        // have today, which is exactly the pre-phase-341 status quo.
        println!(
            "sync: board configs — {} deploy key(s) resolve to no single board \
             descriptor, so no projection was written for them: {}",
            unresolved.len(),
            unresolved.iter().cloned().collect::<Vec<_>>().join("; ")
        );
    }
    Ok(complaints)
}

/// Pure DOM transform behind [`write_patch_config`]: given the existing
/// `.cargo/config.toml` text (empty string if absent) + the managed entries, return
/// the rewritten text with `[patch.crates-io]`'s nros-managed keys replaced. Format-
/// preserving (`toml_edit`); user keys + `[target]`/`[env]` untouched. No fs — pure +
/// unit-testable.
fn render_patch_config(
    existing: &str,
    managed: &[(String, String)],
    include_rel: Option<&str>,
) -> Result<String> {
    render_patch_config_with(existing, managed, include_rel, true, false)
}

/// [`render_patch_config`] with the sidecar split made explicit.
///
/// Issue 0457 × #272 — `sidecar` is false for an OUT-OF-TREE consumer, which
/// must not gain an `include` at all: outside this checkout the key has three
/// fragile preconditions (cargo ≥ 1.93, a correct relative path, a present
/// target) and tripping any one drops the patch SILENTLY. Those consumers keep
/// the managed entries inline in their own `config.toml`, which is theirs to
/// commit or ignore. In-tree leaves take the sidecar, because there the file is
/// ours and the churn it causes is ours to remove.
fn render_patch_config_with(
    existing: &str,
    managed: &[(String, String)],
    include_rel: Option<&str>,
    sidecar: bool,
    // Issue 0827 — whether this leaf gets a derived `[env]` sidecar. Same
    // appear-and-disappear-together rule as the patch one: the caller decides by
    // whether it is about to WRITE the file, never by guessing.
    want_env: bool,
) -> Result<String> {
    use toml_edit::{DocumentMut, Item, Table, Value, value};

    let mut doc: DocumentMut = existing.parse().wrap_err("parse .cargo/config.toml")?;

    // W9 option E — manage the top-level `include = [...]` array. Our entry is
    // recognised by its `nros-patch.toml` basename (evicted + re-added each
    // sync so a checkout-depth change re-points it); user include entries are
    // preserved, and an array left empty is removed. toml_edit keeps root
    // scalar keys ahead of tables when rendering, so the key lands in a valid
    // position even in a config that already carries [patch]/[env] tables.
    {
        // Decide BEFORE touching the document, and touch nothing when the
        // membership is already what it should be.
        //
        // `retain` + `insert(0, …)` is not formatting-neutral: toml_edit keeps
        // per-element decor, and evicting the first element hands its position
        // to a survivor that carries its own leading space. So a leaf committed
        // as `["../nros-patch.toml", "nros-board.toml"]` came back as
        // `[ "../nros-patch.toml", "nros-board.toml"]` — same membership, one
        // space, every tracked leaf config permanently dirty after any sync.
        // Measured on 20 leaves; it also blocks a rebase and invites `git add
        // -u` to commit sync output, which CLAUDE.md calls out by name.
        //
        // The sibling test `config_writer_quoted_user_header_no_duplicate`
        // already worked the decor rule out, but pinned idempotence FROM the
        // spaced form — feeding tight input with a survivor, which is what the
        // committed files actually are, was the uncovered case.
        //
        // Skipping the write also spares the mtime: an identical rewrite still
        // re-stales every fixture keyed on this leaf.
        let current: Vec<String> = doc
            .as_table()
            .get("include")
            .and_then(|i| i.as_value())
            .and_then(|v| v.as_array())
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_string))
                    .collect()
            })
            .unwrap_or_default();
        let survivors: Vec<String> = current
            .iter()
            .filter(|s| {
                !s.ends_with(CENTRAL_PATCH_FILE)
                    && !s.ends_with(MANAGED_PATCH_FILE)
                    && !s.ends_with(MANAGED_ENV_FILE)
            })
            .cloned()
            .collect();
        let mut desired: Vec<String> = Vec::new();
        if let Some(rel) = include_rel {
            desired.push(rel.to_string());
        }
        desired.extend(survivors.iter().cloned());
        let want_sidecar = sidecar && managed.iter().any(|(_, rel)| is_generated_path(rel));
        if want_sidecar {
            desired.push(MANAGED_PATCH_FILE.to_string());
        }
        if want_env {
            desired.push(MANAGED_ENV_FILE.to_string());
        }

        if current != desired {
            let inc_item = doc
                .as_table_mut()
                .entry("include")
                .or_insert_with(|| value(toml_edit::Array::new()));
            let arr = inc_item
                .as_value_mut()
                .and_then(|v| v.as_array_mut())
                .ok_or_else(|| eyre!("sync: `include` is not an array"))?;
            arr.retain(|v| {
                v.as_str()
                    .map(|s| {
                        !s.ends_with(CENTRAL_PATCH_FILE)
                            && !s.ends_with(MANAGED_PATCH_FILE)
                            && !s.ends_with(MANAGED_ENV_FILE)
                    })
                    .unwrap_or(true)
            });
            if let Some(rel) = include_rel {
                arr.insert(0, rel);
            }
            // Issue 0457 — point at the sibling managed-patch file when there is
            // one. Relative to the INCLUDING file's directory, which is `.cargo/`
            // itself, so the bare basename is the whole path. Re-added each sync,
            // and dropped when the managed set empties, so a leaf that loses its
            // last generated dep does not keep an include to a deleted file.
            //
            // Issue 0463 — that last clause is load-bearing in a way 0457 did not
            // realise. It justified itself with "cargo ignores a missing include
            // SILENTLY"; on cargo 1.97.1 a missing include is a HARD error during
            // manifest parse, so an orphaned entry does not degrade the leaf, it
            // makes the leaf unreadable.
            //
            // The entry is therefore written ONLY when this leaf actually has a
            // host-specific row to put in the sidecar. Rows that are in-repo
            // (relative paths, identical in every checkout) stay inline in the
            // tracked config above, so a leaf with no `generated/` dep gets no
            // sidecar and no include at all, and resolves in a fresh clone with no
            // sync. What remains behind sync is exactly what sync alone can
            // produce — the ament-derived crates, which must not be in git.
            // `_require-leaf-includes` covers the leaves that do need it, saying
            // "run `nros sync`" before cargo says anything at all.
            if want_sidecar {
                arr.push(MANAGED_PATCH_FILE);
            }
            // Issue 0827 — the derived `[env]` sidecar, on the same terms.
            if want_env {
                arr.push(MANAGED_ENV_FILE);
            }
            if arr.is_empty() {
                doc.as_table_mut().remove("include");
            }
        }
    }

    // Ensure [patch] then [patch.crates-io] tables exist.
    let patch_item = doc
        .as_table_mut()
        .entry("patch")
        .or_insert_with(|| Item::Table(Table::new()));
    let patch_tbl = patch_item
        .as_table_mut()
        .ok_or_else(|| eyre!("sync: [patch] is not a table"))?;
    patch_tbl.set_implicit(true);
    let cio_item = patch_tbl
        .entry("crates-io")
        .or_insert_with(|| Item::Table(Table::new()));
    let cio = cio_item
        .as_table_mut()
        .ok_or_else(|| eyre!("sync: [patch.crates-io] is not a table"))?;

    // Evict prior nros-managed keys (preserve user keys + their decor).
    //
    // Issue 0457 — this is now a MIGRATION as well as a refresh: the managed
    // set is written to the sibling `nros-managed-patch.toml`, so any tagged
    // key still sitting in `config.toml` was put there by an older sync and
    // must go, or the leaf would carry the same patch twice (and the tracked
    // file would keep churning, which is the whole point of the split).
    let stale: Vec<String> = cio
        .iter()
        .filter(|(_, v)| item_is_nros_managed(v))
        .map(|(k, _)| k.to_string())
        .collect();
    for k in stale {
        cio.remove(&k);
    }

    // Which rows live HERE, in the tracked config.
    //
    // Issue 0463 — the split is by ORIGIN, not by "sync wrote it". 0457 moved
    // the whole managed set to the sidecar, which stranded every leaf: the set
    // is mostly IN-REPO crates (`nros-log`, a board crate, `mps2-an385-pac` —
    // relative paths that are identical in every checkout), and a clone needs
    // those to resolve at all. Measured across the tree: 183 in-repo rows vs 88
    // `generated/` rows.
    //
    // Only the `generated/` rows are host-specific: those crates are built by
    // `nros sync` from the USER's ament install, so a committed row would name
    // a path no clone has — the rule that already keeps `generated/` itself and
    // the leaf `Cargo.lock` out of git. Those go to the sidecar; everything
    // else stays inline and tracked, stable across syncs.
    //
    // An out-of-tree consumer keeps EVERYTHING inline: #272 gives it no
    // `include`, and its config is its own to commit or ignore.
    let inline: Vec<(String, String)> = if sidecar {
        managed
            .iter()
            .filter(|(_, rel)| !is_generated_path(rel))
            .cloned()
            .collect()
    } else {
        managed.to_vec()
    };
    {
        let mut sorted = inline;
        sorted.sort_by(|a, b| a.0.cmp(&b.0));
        sorted.dedup_by(|a, b| a.0 == b.0);
        for (name, rel) in &sorted {
            let mut it = toml_edit::InlineTable::new();
            it.insert("path", Value::from(rel.as_str()));
            let mut item = value(Value::InlineTable(it));
            if let Some(v) = item.as_value_mut() {
                v.decor_mut().set_suffix(format!("  # {NROS_MANAGED_TAG}"));
            }
            cio.insert(name, item);
        }
    }

    // Drop emptied tables so an empty managed set leaves no bare header (0094 F).
    if cio.is_empty() {
        patch_tbl.remove("crates-io");
    }
    if patch_tbl.is_empty() {
        doc.as_table_mut().remove("patch");
    }

    Ok(doc.to_string())
}

/// Remove EVERY contiguous BEGIN..END region from `body` (including both
/// marker lines). Returns `body` unchanged if no markers found.
///
/// Issue #94 case C — a prior crash or concurrent writer can leave more
/// than one managed block; strip them all so the next sync self-heals
/// instead of indefinitely carrying a stale duplicate.
fn strip_managed_block(body: &str) -> String {
    let mut out = body.to_string();
    while let Some(next) = strip_first_managed_block(&out) {
        out = next;
    }
    out
}

/// Remove the FIRST BEGIN..END region from `body`. Returns `None` when no
/// complete region is present (no BEGIN, or BEGIN without a following END).
fn strip_first_managed_block(body: &str) -> Option<String> {
    let begin_idx = body.find(BEGIN)?;
    let after_begin = begin_idx + BEGIN.len();
    let end_rel = body[after_begin..].find(END)?;
    let end_idx = after_begin + end_rel;
    let end_line_end = end_idx + END.len();
    // Consume the newline after END if present.
    let tail_start = if body[end_line_end..].starts_with('\n') {
        end_line_end + 1
    } else {
        end_line_end
    };
    let mut out = String::new();
    out.push_str(&body[..begin_idx]);
    // Drop a single trailing blank line above BEGIN if it was emitted as
    // a separator by a previous sync (keeps diffs minimal across re-runs).
    if out.ends_with("\n\n") {
        out.pop();
    }
    out.push_str(&body[tail_start..]);
    Some(out)
}

/// True iff `line` is a `[patch.crates-io]` table header, tolerating the
/// TOML-equivalent quoted form `[patch."crates-io"]` (or single-quoted) and
/// any trailing inline comment. Issue #94 case A — cargo/toml_edit and hand
/// edits both occur, and the bare-`starts_with` match missed the quoted form,
/// causing a duplicate header to be emitted (which cargo rejects).
fn is_patch_crates_io_header(line: &str) -> bool {
    let t = line.trim_start();
    let Some(rest) = t.strip_prefix('[') else {
        return false;
    };
    let Some(close) = rest.find(']') else {
        return false;
    };
    let inner = &rest[..close];
    let segs: Vec<&str> = inner.split('.').collect();
    segs.len() == 2
        && strip_toml_key_quotes(segs[0].trim()) == "patch"
        && strip_toml_key_quotes(segs[1].trim()) == "crates-io"
}

/// Strip surrounding quotes from a TOML bare key wrapped in `"..."` or
/// `'...'`. Bare keys pass through unchanged.
fn strip_toml_key_quotes(key: &str) -> &str {
    let trimmed = key.trim();
    if trimmed.len() >= 2 {
        let bytes = trimmed.as_bytes();
        let first = bytes[0];
        let last = bytes[trimmed.len() - 1];
        if (first == b'"' && last == b'"') || (first == b'\'' && last == b'\'') {
            return &trimmed[1..trimmed.len() - 1];
        }
    }
    trimmed
}

// --- Check / freshness ---------------------------------------------------------

fn check_freshness(
    ws_root: &Path,
    build_root: &Path,
    scan: &[WsPkg],
    topo: &[String],
) -> Result<()> {
    let mut stale = false;
    for name in topo {
        let pkg = scan.iter().find(|p| &p.name == name).unwrap();
        let crate_root = build_root.join(name);
        let cargo = crate_root.join("Cargo.toml");
        if !cargo.is_file() {
            eprintln!(
                "sync --check: stale: {name} — no Cargo.toml at {}",
                cargo.display()
            );
            stale = true;
            continue;
        }
        let cargo_mt = std::fs::metadata(&cargo)?.modified()?;
        for subdir in &["msg", "srv", "action"] {
            let d = pkg.dir.join(subdir);
            if !d.is_dir() {
                continue;
            }
            for entry in std::fs::read_dir(d)? {
                let entry = entry?;
                if !entry.file_type()?.is_file() {
                    continue;
                }
                let mt = entry.metadata()?.modified()?;
                if mt > cargo_mt {
                    eprintln!(
                        "sync --check: stale: {name} — {} newer than generated crate",
                        entry
                            .path()
                            .strip_prefix(ws_root)
                            .unwrap_or(&entry.path())
                            .display()
                    );
                    stale = true;
                }
            }
        }
    }
    if stale {
        bail!("sync --check: some pkgs stale — run `nros sync` first.");
    }
    println!("sync --check: all good.");
    Ok(())
}

// =============================================================================
// Phase 210.F.3 — `nros ws {list,status,clean,doctor}` sibling subcommands.
// All dual-mode (single-pkg + colcon-style workspace), same detection as sync.
// =============================================================================

/// Run sync's scan+resolve step without codegen — for list/status/clean/
/// doctor. Returns the workspace root + scanned pkgs + the resolved
/// build_root. The optional `build_dir` arg defaults to `<ws_root>/build`.
fn scan_for_query(
    workspace: Option<&Path>,
    build_dir: &Path,
) -> Result<(PathBuf, Vec<WsPkg>, PathBuf)> {
    let ws_root: PathBuf = match workspace {
        Some(p) => std::fs::canonicalize(p).wrap_err_with(|| format!("ws: {}", p.display()))?,
        None => std::env::current_dir()?,
    };
    let colcon_layout = ws_root.join("src").is_dir() && has_pkg_subdir(&ws_root.join("src"));
    let single_pkg_mode = !colcon_layout && ws_root.join("package.xml").is_file();
    let src_root = if colcon_layout {
        ws_root.join("src")
    } else if single_pkg_mode {
        ws_root.clone()
    } else {
        bail!(
            "ws: no `src/<pkg>/package.xml` and no `package.xml` at root \
             under {} — expected colcon-style workspace or single-pkg dir",
            ws_root.display()
        );
    };
    let mut scan = Vec::new();
    if single_pkg_mode {
        scan_one_pkg_dir(&src_root, &mut scan)?;
    } else {
        scan_workspace(&src_root, &mut scan)?;
    }
    let build_root = if build_dir.is_absolute() {
        build_dir.to_path_buf()
    } else {
        ws_root.join(build_dir)
    };
    Ok((ws_root, scan, build_root))
}

// --- list ---------------------------------------------------------------------

fn run_list(args: ListArgs) -> Result<()> {
    // build_dir doesn't matter for list; use the default for the scan
    // helper's signature.
    let (ws_root, scan, _build_root) =
        scan_for_query(args.workspace.as_deref(), Path::new("build"))?;
    if scan.is_empty() {
        println!("ws list: no pkgs found.");
        return Ok(());
    }
    println!("ws list ({}):", ws_root.display());
    let mut kinds = (0usize, 0usize); // (msg, rust)
    for p in &scan {
        let kind = match (p.is_msg_pkg, p.is_rust_pkg) {
            (true, true) => "msg+rust",
            (true, false) => "msg",
            (false, true) => "rust",
            (false, false) => "other",
        };
        if p.is_msg_pkg {
            kinds.0 += 1;
        }
        if p.needs_patch_authority() {
            kinds.1 += 1;
        }
        println!(
            "  {kind:9}  {:24}  {}",
            p.name,
            p.dir.strip_prefix(&ws_root).unwrap_or(&p.dir).display()
        );
    }
    println!("ws list: {} msg, {} rust consumer", kinds.0, kinds.1);
    Ok(())
}

// --- status -------------------------------------------------------------------

fn run_status(args: StatusArgs) -> Result<()> {
    let (ws_root, scan, build_root) = scan_for_query(args.workspace.as_deref(), &args.build_dir)?;
    let msg_pkgs: Vec<&WsPkg> = scan.iter().filter(|p| p.is_msg_pkg).collect();
    if msg_pkgs.is_empty() {
        println!("ws status: no msg pkgs.");
        return Ok(());
    }
    let mut up_to_date = 0;
    let mut stale = 0;
    let mut missing = 0;
    for pkg in &msg_pkgs {
        let crate_root = build_root.join(&pkg.name);
        let cargo = crate_root.join("Cargo.toml");
        if !cargo.is_file() {
            missing += 1;
            continue;
        }
        let cargo_mt = match std::fs::metadata(&cargo).and_then(|m| m.modified()) {
            Ok(t) => t,
            Err(_) => {
                missing += 1;
                continue;
            }
        };
        let mut pkg_stale = false;
        for subdir in &["msg", "srv", "action"] {
            let d = pkg.dir.join(subdir);
            if !d.is_dir() {
                continue;
            }
            for e in std::fs::read_dir(d)?.flatten() {
                if e.file_type().map(|t| t.is_file()).unwrap_or(false)
                    && let Ok(mt) = e.metadata().and_then(|m| m.modified())
                    && mt > cargo_mt
                {
                    pkg_stale = true;
                    break;
                }
            }
            if pkg_stale {
                break;
            }
        }
        if pkg_stale {
            stale += 1;
        } else {
            up_to_date += 1;
        }
    }
    let _ = ws_root;
    println!(
        "ws status: {up_to_date} up-to-date, {stale} stale, {missing} missing \
         (of {} msg pkgs)",
        msg_pkgs.len()
    );
    Ok(())
}

// --- clean --------------------------------------------------------------------

fn run_clean(args: CleanArgs) -> Result<()> {
    let (ws_root, scan, build_root) = scan_for_query(args.workspace.as_deref(), &args.build_dir)?;
    let gen_dir = build_root;
    if gen_dir.is_dir() {
        if args.dry_run {
            println!("ws clean: WOULD rm -rf {}", gen_dir.display());
        } else {
            std::fs::remove_dir_all(&gen_dir)
                .wrap_err_with(|| format!("ws clean: rm {}", gen_dir.display()))?;
            println!("ws clean: removed {}", gen_dir.display());
        }
    } else {
        println!("ws clean: {} not present, skip", gen_dir.display());
    }
    // Phase 265 — strip the auto-managed `[patch.crates-io]` entries from every Rust
    // consumer's patch-authority `.cargo/config.toml` (the patch now lives there, not
    // the Cargo.toml). User keys (a hand `libc` patch) + `[target]`/`[env]` are kept.
    let rust_consumers: Vec<&WsPkg> = scan.iter().filter(|p| p.is_rust_pkg).collect();
    let mut authorities: std::collections::HashSet<PathBuf> = std::collections::HashSet::new();
    for c in &rust_consumers {
        if let Ok(a) = find_patch_authority(&c.dir, &ws_root) {
            authorities.insert(a);
        }
    }
    for authority in authorities {
        let cfg = authority
            .parent()
            .unwrap_or(&authority)
            .join(".cargo/config.toml");
        let body = match std::fs::read_to_string(&cfg) {
            Ok(b) => b,
            Err(_) => continue,
        };
        if !body.contains(NROS_MANAGED_TAG) {
            continue;
        }
        if args.dry_run {
            println!(
                "ws clean: WOULD strip managed patches from {}",
                cfg.display()
            );
            continue;
        }
        // Re-render with an empty managed set + no include → evicts every
        // nros-managed key AND the W9 central-patch include entry; drops an
        // emptied table/array; preserves user content.
        let cleaned = render_patch_config(&body, &[], None)
            .wrap_err_with(|| format!("ws clean: edit {}", cfg.display()))?;
        // Issue 0498 — cargo may be reading this config; a truncated
        // `.cargo/config.toml` is a manifest-parse error four frames deep.
        atomic_write(&cfg, &cleaned)
            .wrap_err_with(|| format!("ws clean: write {}", cfg.display()))?;
        println!("ws clean: stripped managed patches from {}", cfg.display());
    }
    Ok(())
}

// --- doctor -------------------------------------------------------------------

fn run_doctor(args: DoctorArgs) -> Result<()> {
    let (ws_root, scan, build_root) = scan_for_query(args.workspace.as_deref(), &args.build_dir)?;
    let mut warnings = 0;
    println!("ws doctor ({})", ws_root.display());
    for pkg in &scan {
        // (a) package.xml well-formed?
        let body = match std::fs::read_to_string(&pkg.manifest) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("  ✗ {}: package.xml read error: {e}", pkg.name);
                warnings += 1;
                continue;
            }
        };
        // (b) msg pkg without member_of_group=rosidl_interface_packages
        let has_iface_group = body.contains("rosidl_interface_packages");
        let has_msg_dirs = pkg.dir.join("msg").is_dir()
            || pkg.dir.join("srv").is_dir()
            || pkg.dir.join("action").is_dir();
        if has_msg_dirs && !has_iface_group {
            eprintln!(
                "  ⚠ {}: has msg/srv/action dirs but pkg.xml lacks \
                 <member_of_group>rosidl_interface_packages</member_of_group> \
                 — upstream colcon won't classify it as an interface pkg",
                pkg.name
            );
            warnings += 1;
        }
        // (c) rust consumer: is the patch authority config sane?
        if pkg.needs_patch_authority() {
            match find_patch_authority(&pkg.dir, &ws_root) {
                Ok(a) => {
                    let cfg = a
                        .parent()
                        .map(|d| d.join(".cargo/config.toml"))
                        .unwrap_or_default();
                    let body = std::fs::read_to_string(&cfg).unwrap_or_default();
                    if !body.contains(NROS_MANAGED_TAG) {
                        eprintln!(
                            "  ⚠ {}: no nros-managed [patch.crates-io] entries in \
                             patch authority config ({}). Run `nros sync`.",
                            pkg.name,
                            cfg.display()
                        );
                        warnings += 1;
                    }
                }
                Err(e) => {
                    eprintln!("  ⚠ {}: patch authority resolve failed: {e}", pkg.name);
                    warnings += 1;
                }
            }
        }
    }
    // (d) stale msg pkgs (same logic as status).
    let _ = build_root;
    if warnings == 0 {
        println!("ws doctor: no issues.");
    } else {
        println!("ws doctor: {warnings} warning(s).");
    }
    Ok(())
}
// =============================================================================
// Phase 210.D.1 regression tests — `[patch.crates-io]` dedup writer.
// =============================================================================

#[cfg(test)]
mod config_include_version_tests {
    use super::*;

    /// phase-367 W2 — the probe is skipped in-repo and KEPT for the external
    /// consumer it exists for (#272). Both arms, because skipping one is how a
    /// diagnostic gets deleted by an optimisation.
    #[test]
    fn cargo_version_warning_applies_only_outside_the_monorepo() {
        let repo = crate::abi_guard::find_monorepo_root(Path::new(env!("CARGO_MANIFEST_DIR")))
            .expect("these tests run inside the nano-ros checkout");
        assert!(
            !cargo_version_warning_applies(&repo),
            "an in-repo workspace pins its toolchain; the probe is 42% of a sync"
        );
        assert!(
            !cargo_version_warning_applies(&repo.join("examples/workspaces/mixed")),
            "a workspace nested in the checkout is still in-repo"
        );

        let outside = tempfile::tempdir().expect("tempdir");
        assert!(
            cargo_version_warning_applies(outside.path()),
            "an out-of-tree consumer is exactly who #272's warning is for — \
             skipping it there would delete the diagnostic"
        );
    }

    #[test]
    fn parses_minor_from_cargo_version_line() {
        assert_eq!(parse_cargo_minor("cargo 1.96.0 (abc 2026-01-01)"), Some(96));
        assert_eq!(parse_cargo_minor("cargo 1.93.0"), Some(93));
        assert_eq!(
            parse_cargo_minor("cargo 1.90.1 (deadbeef 2025-06-01)"),
            Some(90)
        );
    }

    #[test]
    fn unrecognised_version_line_parses_to_none() {
        assert_eq!(parse_cargo_minor(""), None);
        assert_eq!(parse_cargo_minor("cargo"), None);
        assert_eq!(parse_cargo_minor("cargo weird-build"), None);
    }

    #[test]
    fn stable_boundary_is_1_93() {
        // The warn gate: < 93 warns, >= 93 stays quiet. Lock the boundary so a
        // refactor can't silently move it off the actual stabilization release.
        assert_eq!(CONFIG_INCLUDE_STABLE_MINOR, 93);
        assert!(parse_cargo_minor("cargo 1.92.0").unwrap() < CONFIG_INCLUDE_STABLE_MINOR);
        assert!(parse_cargo_minor("cargo 1.93.0").unwrap() >= CONFIG_INCLUDE_STABLE_MINOR);
    }
}

#[cfg(test)]
mod launch_resolver_tests {
    use super::*;

    fn touch(path: &std::path::Path) {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, "#!/bin/sh\n").unwrap();
    }

    /// Installed layout: the helper sits beside the `nros` binary.
    #[test]
    fn finds_the_helper_beside_the_nros_binary() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("bin").join("nros");
        touch(&exe);
        let helper = tmp.path().join("bin").join(LAUNCH_RESOLVER);
        touch(&helper);

        assert_eq!(resolver_from(&exe, None, None), Some(helper));
    }

    /// Per-checkout layout: `nros` is at `packages/cli/target/release/nros`,
    /// but the helper is its OWN cargo workspace, so it lands under
    /// `packages/cli/nros-launch-resolve/target/release/` instead. Found via
    /// `$NROS_REPO_DIR` and via the walk-up, matching `nros_cli_bin()`.
    #[test]
    fn finds_the_in_tree_helper_by_repo_dir_and_by_walk_up() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let exe = root
            .join("packages")
            .join("cli")
            .join("target")
            .join("release")
            .join("nros");
        touch(&exe);
        let helper = root
            .join("packages")
            .join("cli")
            .join(LAUNCH_RESOLVER)
            .join("target")
            .join("release")
            .join(LAUNCH_RESOLVER);
        touch(&helper);

        assert_eq!(
            resolver_from(&exe, None, Some(root.to_path_buf())),
            Some(helper.clone()),
            "$NROS_REPO_DIR should locate the per-checkout helper"
        );
        assert_eq!(
            resolver_from(&exe, None, None),
            Some(helper),
            "and the walk-up should find it without the env var"
        );
    }

    /// `$NROS_LAUNCH_RESOLVE` wins, mirroring `$NROS_CLI`; a non-existent
    /// override falls through instead of failing.
    #[test]
    fn env_override_wins_and_a_bad_one_falls_through() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("bin").join("nros");
        touch(&exe);
        let sibling = tmp.path().join("bin").join(LAUNCH_RESOLVER);
        touch(&sibling);
        let packaged = tmp.path().join("packaged").join(LAUNCH_RESOLVER);
        touch(&packaged);

        assert_eq!(
            resolver_from(&exe, Some(packaged.clone()), None),
            Some(packaged),
            "the override must win over the sibling"
        );
        assert_eq!(
            resolver_from(&exe, Some(tmp.path().join("nope")), None),
            Some(sibling),
            "a bad override must fall through to the normal search"
        );
    }

    /// Issue 0285, the property the whole fix exists for: resolution NEVER
    /// consults `$PATH`. A helper reachable only through PATH must not be
    /// found — that is exactly how an unrelated `play_launch` hijacked this
    /// call and took every platform's fixture build down with it.
    #[test]
    fn a_helper_only_on_path_is_not_found() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("bin").join("nros");
        touch(&exe);
        // Exists, but somewhere only PATH would reach.
        touch(
            &tmp.path()
                .join("usr")
                .join("local")
                .join("bin")
                .join(LAUNCH_RESOLVER),
        );

        assert_eq!(
            resolver_from(&exe, None, None),
            None,
            "a helper reachable only via PATH must NOT be used"
        );
    }

    /// No helper anywhere is a clean `None`, so the caller degrades to the
    /// committed model rather than failing the build.
    #[test]
    fn absent_helper_is_none() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("bin").join("nros");
        touch(&exe);
        assert_eq!(resolver_from(&exe, None, None), None);
    }
}

#[cfg(test)]
mod patch_block_tests {
    use super::*;

    /// `strip_managed_block` is a no-op when no BEGIN marker is present.
    #[test]
    fn strip_managed_block_noop_without_markers() {
        let body = "[package]\nname = \"x\"\n";
        assert_eq!(strip_managed_block(body), body);
    }

    fn wspkg(name: &str, is_msg: bool, is_rust: bool, is_consumer: bool) -> WsPkg {
        WsPkg {
            name: name.to_string(),
            dir: PathBuf::from(format!("/ws/{name}")),
            manifest: PathBuf::from(format!("/ws/{name}/package.xml")),
            is_msg_pkg: is_msg,
            is_rust_pkg: is_rust,
            deps: Vec::new(),
            is_patch_consumer: is_consumer,
        }
    }

    /// Phase-265 W5b — a Rust node that ALSO defines msgs (inline `msg/` dir,
    /// e.g. `native/custom-msg`) is still a patch consumer; the old
    /// `!is_msg_pkg` guard wrongly dropped it ("no Rust consumer pkgs").
    #[test]
    fn node_with_msg_dir_is_a_patch_consumer() {
        // is_rust + is_msg + consumer → needs an authority (the fix).
        assert!(wspkg("custom_msg", true, true, true).needs_patch_authority());
        // pure interface pkg (no source Cargo.toml) → excluded by is_rust.
        assert!(!wspkg("std_msgs", true, false, true).needs_patch_authority());
        // plain rust consumer → included.
        assert!(wspkg("talker", false, true, true).needs_patch_authority());
        // path-dep import target (Entry→Component walk) → not an authority.
        assert!(!wspkg("component", false, true, false).needs_patch_authority());
    }

    /// `cargo_dependency_names` collects keys across the three dep tables.
    #[test]
    fn cargo_dependency_names_spans_all_dep_tables() {
        let body = r#"
[dependencies]
std_msgs = "*"
nros = { path = "../nros" }
[build-dependencies]
cc = "1"
[dev-dependencies]
proptest = "1"
"#;
        let mut got = cargo_dependency_names(body);
        got.sort();
        assert_eq!(got, vec!["cc", "nros", "proptest", "std_msgs"]);
    }

    /// The closure keeps only seeds reachable through the emitted graph and
    /// drops a node's unconsumed self-crate. Hermetic: writes a tiny
    /// `generated/<crate>/Cargo.toml` graph under a temp build root.
    #[test]
    fn closure_excludes_unconsumed_self_crate() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        // std_msgs depends on builtin_interfaces; the self-crate depends on
        // nothing emitted and is referenced by no one.
        for (c, deps) in [
            ("std_msgs", "builtin_interfaces = \"*\"\n"),
            ("builtin_interfaces", ""),
            ("native_rs_custom_msg", ""),
        ] {
            let dir = root.join(c);
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(
                dir.join("Cargo.toml"),
                format!("[package]\nname = \"{c}\"\n\n[dependencies]\n{deps}"),
            )
            .unwrap();
        }
        let emitted = vec![
            "builtin_interfaces".to_string(),
            "native_rs_custom_msg".to_string(),
            "std_msgs".to_string(),
        ];
        // Seed with the consumer's `<depend>` (std_msgs only).
        let got = emitted_msg_dep_closure(&["std_msgs".to_string()], &emitted, root);
        assert_eq!(
            got,
            vec!["builtin_interfaces".to_string(), "std_msgs".to_string()],
            "closure must reach builtin_interfaces but exclude the unconsumed self-crate"
        );
    }

    /// Phase 220.E — consumer Cargo.toml scanner finds every
    /// `nros-*` / `nros` / `cyclonedds-sys` dep with a registry-style
    /// version (`"*"` or `{ version = "*", ... }`), even when other
    /// shapes appear in the same `[dependencies]` table. Path-style
    /// deps (no `version` key) are excluded.
    #[test]
    fn extract_consumer_registry_deps_basic() {
        let body = r#"
[package]
name = "demo"

[dependencies]
zephyr = "0.1"
log = "0.4"
nros = { version = "*", default-features = false }
nros-rmw-zenoh = { version = "*", optional = true }
nros-rmw-cyclonedds-sys = { path = "../foo/nros-rmw-cyclonedds-sys" }
std_msgs = { version = "*", default-features = false }
"#;
        let got = extract_consumer_registry_nros_deps(body);
        // nros + nros-rmw-zenoh recognized (registry).
        // nros-rmw-cyclonedds-sys EXCLUDED (path-only, no version key).
        // zephyr/log/std_msgs ignored (not in lookup table).
        assert_eq!(got, vec!["nros".to_string(), "nros-rmw-zenoh".to_string()]);
    }

    /// Bare-string version form `name = "*"` recognized.
    #[test]
    fn extract_consumer_registry_deps_bare_version() {
        let body = r#"
[dependencies]
nros-core = "*"
nros-serdes = "0.4"
"#;
        let got = extract_consumer_registry_nros_deps(body);
        assert_eq!(
            got,
            vec!["nros-core".to_string(), "nros-serdes".to_string()]
        );
    }

    /// Both `version` AND `path` is treated as registry-style (cargo
    /// workspace shape — version key wins for `[patch.crates-io]`
    /// matching purposes).
    #[test]
    fn extract_consumer_registry_deps_version_plus_path() {
        let body = r#"
[dependencies]
nros = { version = "0.4", path = "../api/nros" }
"#;
        let got = extract_consumer_registry_nros_deps(body);
        assert_eq!(got, vec!["nros".to_string()]);
    }

    /// Target-cfg-scoped `[target.<cfg>.dependencies]` tables are
    /// walked too — common shape for platform-specific deps.
    #[test]
    fn extract_consumer_registry_deps_target_cfg() {
        let body = r#"
[dependencies]
log = "0.4"

[target.'cfg(target_os = "linux")'.dependencies]
nros-rmw-zenoh = { version = "*" }
"#;
        let got = extract_consumer_registry_nros_deps(body);
        assert_eq!(got, vec!["nros-rmw-zenoh".to_string()]);
    }

    /// `cyclonedds-sys` lives under `packages/rmw/cyclonedds/` and is intentionally
    /// in the lookup table — it's the most common non-`nros-*`-prefixed
    /// runtime crate consumers reference registry-style.
    #[test]
    fn extract_consumer_registry_deps_cyclonedds_sys() {
        let body = r#"
[dependencies]
cyclonedds-sys = { version = "*" }
nros-foo-extension = { version = "*" }
"#;
        let got = extract_consumer_registry_nros_deps(body);
        // `cyclonedds-sys` in lookup, `nros-foo-extension` is not.
        assert_eq!(got, vec!["cyclonedds-sys".to_string()]);
    }

    /// Path-only deps (the canonical Phase 212 shape) produce an empty
    /// scan — no patch entries needed since cargo resolves them directly.
    #[test]
    fn extract_consumer_registry_deps_path_only_empty() {
        let body = r#"
[dependencies]
nros = { path = "../../../packages/api/nros" }
nros-rmw-zenoh = { path = "../../../packages/rmw/zenoh/nros-rmw-zenoh" }
"#;
        let got = extract_consumer_registry_nros_deps(body);
        assert!(got.is_empty(), "expected no registry deps, got: {got:?}");
    }

    /// The lookup table covers every name the Phase 220 brief enumerates.
    #[test]
    fn lookup_table_covers_phase_220_e_minimum_set() {
        let must_have = [
            "nros",
            "nros-core",
            "nros-serdes",
            "nros-platform",
            "nros-platform-cffi",
            "nros-node",
            "nros-rmw",
            "nros-rmw-cffi",
            "nros-log",
            "nros-macros",
            "nros-rmw-zenoh",
            "nros-rmw-cyclonedds-sys",
            "nros-rmw-xrce-cffi",
            "cyclonedds-sys",
        ];
        for name in &must_have {
            assert!(
                is_managed_runtime_crate_name(name),
                "lookup table missing `{name}`"
            );
        }
    }

    /// Phase 277 W2.e — board crates (`packages/boards/*`) must resolve to a
    /// `[patch.crates-io]` path so `nros sync` can rewrite a scaffolded
    /// project's registry-style `nros-board-<x> = "*"` dep (W6 flips example
    /// board deps to registry-style). Board crates are NOT enumerated in the
    /// static [`nros_crate_path_lookup`] table — `is_managed_runtime_crate_name`
    /// / `nros_crate_subpath` recognize any `nros-board-`-prefixed name
    /// generically (RFC-0040 D-Q3) and derive `packages/boards/<name>`
    /// uniformly, since every current board crate's Cargo package name
    /// equals its directory name under `packages/boards/`. This test locks
    /// that resolution in for the concrete crates phase-277 cares about
    /// (verified against each crate's actual `Cargo.toml` `name =` field).
    #[test]
    fn board_crates_resolve_via_generic_fallback() {
        let boards = [
            "nros-board-linux",
            "nros-board-freertos",
            "nros-board-mps2-an385-freertos",
            "nros-board-threadx",
            "nros-board-threadx-qemu-riscv64",
            "nros-board-mps2-an385",
        ];
        for name in &boards {
            assert!(
                is_managed_runtime_crate_name(name),
                "board crate `{name}` not recognized as managed"
            );
            assert_eq!(
                nros_crate_subpath(name),
                Some(format!("packages/boards/{name}")),
                "board crate `{name}` resolved to an unexpected subpath"
            );
        }
    }

    /// Phase 277 W6 — the standalone-example manifest flip (path-dep →
    /// `version = "*"`) references three nros-owned crates that neither the
    /// pre-W6 static table nor the `nros-board-*` generic fallback covered:
    /// the critical-section support crate, the custom-transport driver crate,
    /// and the MPS2 PAC (a `packages/boards/` crate WITHOUT the `nros-board-`
    /// name prefix). Lock their table entries in so `nros sync` emits patch
    /// rows for them instead of the "unknown runtime crate" skip warning.
    #[test]
    fn lookup_table_covers_w6_example_flip_extras() {
        let extras = [
            (
                "nros-platform-critical-section",
                "packages/platform/nros-platform-critical-section",
            ),
            (
                "nros-transport-callbacks",
                "packages/rmw/transport-callbacks",
            ),
            ("mps2-an385-pac", "packages/boards/mps2-an385-pac"),
        ];
        for (name, subpath) in &extras {
            assert!(
                is_managed_runtime_crate_name(name),
                "lookup table missing `{name}`"
            );
            assert_eq!(
                nros_crate_subpath(name),
                Some((*subpath).to_string()),
                "`{name}` resolved to an unexpected subpath"
            );
        }
    }

    /// Issue #94 case B — explicit dependency-table form
    /// `[dependencies.<name>]` (and target-scoped variants) must be scanned:
    /// a `version`-carrying entry needs a `[patch.crates-io]` path, a
    /// path-only entry does not.
    #[test]
    fn extract_consumer_registry_deps_explicit_table_form() {
        let body = r#"
[dependencies]
log = "0.4"

[dependencies.nros]
version = "*"
default-features = false

[dependencies.nros-rmw-zenoh]
path = "../rmw/zenoh/nros-rmw-zenoh"

[target.'cfg(target_os = "linux")'.dependencies.nros-core]
version = "*"
"#;
        let got = extract_consumer_registry_nros_deps(body);
        // nros + nros-core carry a version → registry → patched.
        // nros-rmw-zenoh is path-only → skipped.
        assert_eq!(got, vec!["nros".to_string(), "nros-core".to_string()]);
    }

    /// Issue #94 case C — `strip_managed_block` removes EVERY managed
    /// region, not just the first, so a doubled block (from a prior crash
    /// or concurrent writer) is self-healed on the next sync.
    #[test]
    fn strip_managed_block_removes_all_blocks() {
        let body = format!(
            "[package]\nname = \"x\"\n\n{BEGIN}\nnros-core = {{ path = \"a\" }}\n{END}\n\n\
             {BEGIN}\nnros-serdes = {{ path = \"b\" }}\n{END}\n"
        );
        let out = strip_managed_block(&body);
        assert!(!out.contains(BEGIN), "leftover BEGIN marker:\n{out}");
        assert!(!out.contains(END), "leftover END marker:\n{out}");
        assert!(out.contains("name = \"x\""), "package head lost:\n{out}");
    }

    // --- phase-265: render_patch_config (.cargo/config.toml, toml_edit) ---

    fn mng(items: &[(&str, &str)]) -> Vec<(String, String)> {
        items
            .iter()
            .map(|(n, p)| (n.to_string(), p.to_string()))
            .collect()
    }

    /// Issue 0457 — the managed set lands in the SIDECAR file, and `config.toml`
    /// gains only the `include` that reaches it. An empty config must not come
    /// back carrying a `[patch.crates-io]` table at all: that table is what used
    /// to make a tracked config churn on every sync.
    #[test]
    fn config_writer_puts_managed_set_in_sidecar_not_config() {
        let managed = mng(&[
            ("nros-core", "../nros-core"),
            ("std_msgs", "generated/std_msgs"),
        ]);
        let out = render_patch_config("", &managed, None).unwrap();
        let doc: toml_edit::DocumentMut = out.parse().unwrap();
        // Issue 0463 — the IN-REPO row stays here (a clone needs it to resolve);
        // only the `generated/` row, which is built per host, goes to the sidecar.
        let cio_here = doc["patch"]["crates-io"].as_table().unwrap();
        assert!(
            cio_here.get("nros-core").is_some(),
            "in-repo row must stay in config.toml:\n{out}"
        );
        assert!(
            cio_here.get("std_msgs").is_none(),
            "generated row must not stay in config.toml:\n{out}"
        );
        let inc: Vec<&str> = doc["include"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|v| v.as_str())
            .collect();
        assert_eq!(inc, vec![MANAGED_PATCH_FILE], "include not wired:\n{out}");

        // The sidecar holds the entries, alphabetised, with no per-key tag
        // (the whole file is managed, so there is nothing to distinguish).
        let generated: Vec<(String, String)> = managed
            .iter()
            .filter(|(_, r)| is_generated_path(r))
            .cloned()
            .collect();
        let side = render_managed_patch_file(&generated);
        let sdoc: toml_edit::DocumentMut = side.parse().unwrap();
        let cio = sdoc["patch"]["crates-io"].as_table().unwrap();
        assert_eq!(
            cio.get("std_msgs").unwrap()["path"].as_str(),
            Some("generated/std_msgs")
        );
        let keys: Vec<&str> = cio.iter().map(|(k, _)| k).collect();
        assert_eq!(
            keys,
            vec!["std_msgs"],
            "sidecar holds only generated:\n{side}"
        );
    }

    /// Issue 0457 — an older sync wrote its managed keys straight into
    /// `config.toml`. Re-syncing must EVICT them (they now live in the sidecar)
    /// while leaving user keys alone, or the leaf carries each patch twice.
    #[test]
    fn config_writer_migrates_managed_keys_out_of_config() {
        let existing = "\
[patch.crates-io]
libc = { path = \"../../third-party/libc\" }
std_msgs = { path = \"generated/std_msgs\" }  # nros-managed
";
        let out = render_patch_config(existing, &mng(&[("std_msgs", "generated/std_msgs")]), None)
            .unwrap();
        let doc: toml_edit::DocumentMut = out.parse().unwrap();
        let cio = doc["patch"]["crates-io"].as_table().unwrap();
        assert!(
            cio.get("std_msgs").is_none(),
            "managed key not migrated out of config.toml:\n{out}"
        );
        assert_eq!(
            cio.get("libc").unwrap()["path"].as_str(),
            Some("../../third-party/libc"),
            "user key lost:\n{out}"
        );
    }

    /// Issue 0457 — no managed entries ⇒ no include to a file sync did not
    /// write. cargo HARD-ERRORS on a missing `include` (issue 0463), so a
    /// dangling one makes the leaf unparseable, not merely unpatched.
    #[test]
    fn config_writer_drops_sidecar_include_when_managed_set_empties() {
        let existing = format!("include = [\"{MANAGED_PATCH_FILE}\"]\n");
        let out = render_patch_config(&existing, &[], None).unwrap();
        let doc: toml_edit::DocumentMut = out.parse().unwrap();
        assert!(
            doc.get("include").is_none(),
            "stale sidecar include kept:\n{out}"
        );
    }

    /// Build a fake nano-ros checkout with the trio crate manifests present.
    #[cfg(test)]
    fn fake_checkout() -> tempfile::TempDir {
        let nrp = tempfile::tempdir().unwrap();
        for name in CENTRAL_PATCH_CRATES {
            let d = nrp
                .path()
                .join(nros_crate_subpath(name).expect("trio in lookup table"));
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(
                d.join("Cargo.toml"),
                format!("[package]\nname = \"{name}\"\n"),
            )
            .unwrap();
        }
        nrp
    }

    /// #272 — an OUT-OF-TREE consumer inlines the trio with ABSOLUTE paths and
    /// emits NO `include` line (which would otherwise silently drop on cargo
    /// < 1.93 / a wrong relative path / a missing central file).
    #[test]
    fn external_consumer_inlines_absolute_trio_no_include() {
        let nrp = fake_checkout();
        let central = write_central_patch_file(nrp.path()).unwrap();

        let ext = tempfile::tempdir().unwrap();
        let authority = ext.path().join("Cargo.toml");
        std::fs::write(&authority, "[package]\nname = \"consumer\"\n").unwrap();
        let build_root = ext.path().join("build");
        std::fs::create_dir_all(&build_root).unwrap();

        write_patch_block(
            &authority,
            &build_root,
            &[],
            Some(nrp.path()),
            &[],
            Some(&central),
            &HashSet::new(),
            &[],
        )
        .unwrap();

        let cfg = std::fs::read_to_string(ext.path().join(".cargo/config.toml")).unwrap();
        assert!(
            !cfg.contains("include"),
            "external must not use `include`:\n{cfg}"
        );
        let doc: toml_edit::DocumentMut = cfg.parse().unwrap();
        let cio = doc["patch"]["crates-io"].as_table().unwrap();
        for name in CENTRAL_PATCH_CRATES {
            let p = cio
                .get(name)
                .unwrap_or_else(|| panic!("trio `{name}` not inlined:\n{cfg}"))["path"]
                .as_str()
                .unwrap();
            assert!(
                std::path::Path::new(p).is_absolute(),
                "`{name}` path not absolute: {p}"
            );
        }
    }

    /// #272 — an IN-TREE example leaf (under the checkout) keeps the relative
    /// `include` line and does NOT inline the trio (its config is committed).
    #[test]
    fn in_tree_leaf_uses_relative_include() {
        let nrp = fake_checkout();
        let central = write_central_patch_file(nrp.path()).unwrap();

        let authority = nrp.path().join("examples/foo/Cargo.toml");
        std::fs::create_dir_all(authority.parent().unwrap()).unwrap();
        std::fs::write(&authority, "[package]\nname = \"foo\"\n").unwrap();
        let build_root = nrp.path().join("build");
        std::fs::create_dir_all(&build_root).unwrap();

        write_patch_block(
            &authority,
            &build_root,
            &[],
            Some(nrp.path()),
            &[],
            Some(&central),
            &HashSet::new(),
            &[],
        )
        .unwrap();

        let cfg =
            std::fs::read_to_string(nrp.path().join("examples/foo/.cargo/config.toml")).unwrap();
        assert!(
            cfg.contains("include"),
            "in-tree leaf must use `include`:\n{cfg}"
        );
        let doc: toml_edit::DocumentMut = cfg.parse().unwrap();
        if let Some(cio) = doc
            .get("patch")
            .and_then(|p| p.get("crates-io"))
            .and_then(|c| c.as_table())
        {
            assert!(
                cio.get("nros").is_none(),
                "trio must be served by the include, not inlined:\n{cfg}"
            );
        }
    }

    #[test]
    fn config_writer_preserves_user_keys_and_sections() {
        // A hand `libc` patch + a [target] section must survive; libc stays UNtagged.
        let existing = "\
[target.thumbv7m-none-eabi]\n\
runner = \"qemu\"\n\n\
[patch.crates-io]\n\
libc = { path = \"../../third-party/nuttx/libc\" }\n";
        let out =
            render_patch_config(existing, &mng(&[("nros-core", "../nros-core")]), None).unwrap();
        let doc: toml_edit::DocumentMut = out.parse().unwrap();
        assert!(doc.get("target").is_some(), "[target] lost:\n{out}");
        let cio = doc["patch"]["crates-io"].as_table().unwrap();
        assert!(cio.get("libc").is_some(), "user libc patch evicted:\n{out}");
        assert!(
            !item_is_nros_managed(cio.get("libc").unwrap()),
            "user libc wrongly tagged:\n{out}"
        );
        // Issue 0463 — an IN-REPO managed entry belongs here, tagged.
        assert!(
            item_is_nros_managed(cio.get("nros-core").expect("in-repo row kept")),
            "in-repo managed entry missing or untagged:\n{out}"
        );
    }

    #[test]
    fn config_writer_evicts_only_managed_on_resync() {
        // First sync, then re-sync with a DIFFERENT managed set: old managed keys gone,
        // a new one present, user key untouched.
        let existing = "[patch.crates-io]\nlibc = { path = \"x\" }\n";
        let s1 = render_patch_config(
            existing,
            &mng(&[
                ("std_msgs", "generated/std_msgs"),
                ("nros-core", "../nros-core"),
            ]),
            None,
        )
        .unwrap();
        // re-sync: std_msgs dropped (no longer generated), nros-serdes added.
        let s2 = render_patch_config(
            &s1,
            &mng(&[
                ("nros-core", "../nros-core"),
                ("nros-serdes", "../nros-serdes"),
            ]),
            None,
        )
        .unwrap();
        let doc: toml_edit::DocumentMut = s2.parse().unwrap();
        let cio = doc["patch"]["crates-io"].as_table().unwrap();
        // Issue 0457 — config.toml keeps ONLY the user key across re-syncs; the
        // managed set (old and new alike) lives in the sidecar, which is why a
        // tracked config.toml no longer churns when the set moves.
        assert!(
            cio.get("std_msgs").is_none(),
            "stale managed std_msgs not evicted:\n{s2}"
        );
        assert!(
            cio.get("nros-serdes").is_some(),
            "in-repo managed entry must stay in config.toml:\n{s2}"
        );
        assert!(
            cio.get("libc").is_some(),
            "user libc lost on re-sync:\n{s2}"
        );
        // The host-specific part is what the sidecar carries.
        let side = render_managed_patch_file(&mng(&[("std_msgs", "generated/std_msgs")]));
        assert!(side.contains("std_msgs"), "generated row missing:\n{side}");
    }

    #[test]
    fn config_writer_idempotent() {
        let existing = "[patch.crates-io]\nlibc = { path = \"x\" }\n";
        let m = mng(&[
            ("nros-core", "../nros-core"),
            ("std_msgs", "generated/std_msgs"),
        ]);
        let a = render_patch_config(existing, &m, None).unwrap();
        let b = render_patch_config(&a, &m, None).unwrap();
        assert_eq!(a, b, "re-render not idempotent:\n--a--\n{a}\n--b--\n{b}");
    }

    #[test]
    fn config_writer_include_added_evicted_and_user_preserved() {
        // W9 option E — fresh config: include lands as a top-level array ahead
        // of the [patch.crates-io] table.
        let out = render_patch_config(
            "",
            &mng(&[("std_msgs", "generated/std_msgs")]),
            Some("../../../../nros-patch.toml"),
        )
        .unwrap();
        // Issue 0457 — the array now carries BOTH the central file and the
        // per-leaf sidecar; assert entry-wise rather than on the whole literal.
        let inc: Vec<String> = out.parse::<toml_edit::DocumentMut>().unwrap()["include"]
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|v| v.as_str().map(str::to_string))
            .collect();
        assert_eq!(
            inc,
            vec!["../../../../nros-patch.toml", MANAGED_PATCH_FILE],
            "include missing or misordered:\n{out}"
        );
        assert!(out.parse::<toml_edit::DocumentMut>().is_ok());

        // Re-sync at a different depth re-points OUR entry, preserves a user one.
        let with_user = out.replace(
            "\"../../../../nros-patch.toml\",",
            "\"../../../../nros-patch.toml\", \"user.toml\",",
        );
        let repointed = render_patch_config(
            &with_user,
            &mng(&[("std_msgs", "generated/std_msgs")]),
            Some("../../nros-patch.toml"),
        )
        .unwrap();
        assert!(repointed.contains("\"../../nros-patch.toml\""));
        assert!(
            !repointed.contains("../../../../nros-patch.toml"),
            "stale include entry not evicted:\n{repointed}"
        );
        assert!(
            repointed.contains("\"user.toml\""),
            "user include entry lost:\n{repointed}"
        );

        // ws clean shape (no include) drops OUR entry; array with only user
        // entries survives; array left empty is removed entirely.
        let cleaned = render_patch_config(&repointed, &[], None).unwrap();
        assert!(!cleaned.contains("nros-patch.toml"));
        assert!(cleaned.contains("\"user.toml\""));
        let ours_only = render_patch_config("", &[], Some("../nros-patch.toml")).unwrap();
        let emptied = render_patch_config(&ours_only, &[], None).unwrap();
        assert!(
            !emptied.contains("include"),
            "emptied include array not removed:\n{emptied}"
        );
    }

    #[test]
    fn config_writer_empty_managed_removes_table() {
        // No managed entries + no user keys → [patch.crates-io] (and [patch]) removed (0094 F).
        let out = render_patch_config(
            "[patch.crates-io]\nnros-core = { path = \"x\" }  # nros-managed\n",
            &[],
            None,
        )
        .unwrap();
        assert!(
            !out.contains("[patch"),
            "empty managed left a patch table:\n{out}"
        );
    }

    #[test]
    fn migrate_strips_managed_block_and_empty_header() {
        // In-tree example shape: [patch.crates-io] holds ONLY the managed BEGIN/END
        // block → migration removes the block AND the now-empty header.
        let body = format!(
            "[package]\nname = \"x\"\n\n[dependencies]\nnros = \"*\"\n\n[patch.crates-io]\n{BEGIN}\n\
             # banner\nnros-core = {{ path = \"a\" }}\n{END}\n"
        );
        let out = strip_managed_patch_from_cargo(&body);
        assert!(
            !out.contains("[patch.crates-io]"),
            "empty patch header left:\n{out}"
        );
        assert!(
            !out.contains(BEGIN) && !out.contains(END),
            "markers left:\n{out}"
        );
        assert!(
            out.contains("name = \"x\"") && out.contains("nros = \"*\""),
            "manifest body lost:\n{out}"
        );
    }

    #[test]
    fn migrate_keeps_user_patch_rows() {
        // A user (non-managed) patch row alongside the managed block: keep the row +
        // header, drop only the managed block.
        let body = format!(
            "[package]\nname = \"x\"\n\n[patch.crates-io]\nlibc = {{ path = \"z\" }}\n{BEGIN}\n\
             nros-core = {{ path = \"a\" }}\n{END}\n"
        );
        let out = strip_managed_patch_from_cargo(&body);
        assert!(
            out.contains("[patch.crates-io]"),
            "header wrongly dropped (had user row):\n{out}"
        );
        assert!(
            out.contains("libc = { path = \"z\" }"),
            "user row lost:\n{out}"
        );
        assert!(!out.contains(BEGIN), "managed block left:\n{out}");
    }

    #[test]
    fn config_writer_quoted_user_header_no_duplicate() {
        // Pre-existing quoted [patch."crates-io"] + user key → still ONE table via DOM
        // (0094 A immune by construction). Issue 0457: the managed entry goes to the
        // sidecar, so what this now guards is that the quoted user table survives
        // intact and un-duplicated — an out-of-tree consumer still merges in place.
        let existing = "[patch.\"crates-io\"]\nlibc = { path = \"x\" }\n";
        let out =
            render_patch_config(existing, &mng(&[("nros-core", "../nros-core")]), None).unwrap();
        let doc: toml_edit::DocumentMut = out.parse().unwrap(); // parses = no duplicate table
        let cio = doc["patch"]["crates-io"].as_table().unwrap();
        assert!(cio.get("libc").is_some(), "user key lost:\n{out}");
        assert!(
            cio.get("nros-core").is_some(),
            "in-repo managed entry must stay in config.toml:\n{out}"
        );

        // A leaf's `include` spelling no longer changes on sync AT ALL.
        //
        // It used to, and the mechanism was understood: `retain` evicts the
        // sync-managed entries and `insert(0, …)` puts the central one back;
        // toml_edit carries per-element decor, so whether the spacing survived
        // depended on whether anything survived retain. What that analysis
        // pinned was idempotence from each resulting state — not the states the
        // COMMITTED files are in. Tight input with a survivor was the uncovered
        // case, and it is what every tracked leaf actually holds, so a sync
        // rewrote 20 configs with a one-space difference: a permanently dirty
        // tree, a blocked rebase, and an invitation for `git add -u` to commit
        // sync output.
        //
        // The rule now is membership, not decor: if the array already says what
        // it should say, the document is not touched and the bytes are
        // identical. Both spellings are therefore stable, and the old
        // tight/spaced distinction is moot.
        let spaced_single = "include = [ \"../nros-patch.toml\"]\n";
        let only_managed = render_patch_config_with(
            spaced_single,
            &mng(&[]),
            Some("../nros-patch.toml"),
            false,
            false,
        )
        .unwrap();
        assert!(
            only_managed.starts_with(spaced_single.trim_end()),
            "membership unchanged: sync must not renormalise the spelling:\n{only_managed}"
        );

        let spaced_pair = "include = [ \"../nros-patch.toml\", \"nros-board.toml\"]\n";
        let with_survivor = render_patch_config_with(
            spaced_pair,
            &mng(&[]),
            Some("../nros-patch.toml"),
            false,
            false,
        )
        .unwrap();
        assert!(
            with_survivor.starts_with(spaced_pair.trim_end()),
            "a survivor keeps the array's decor, so this leaf never churns:\n{with_survivor}"
        );

        // The case the spacing analysis above did NOT cover, and the one every
        // committed leaf actually is: TIGHT input with a survivor. Feeding the
        // spaced form back proves idempotence from the spaced state only —
        // tight -> spaced is a one-way trip that happens exactly once per
        // clone, and it left 20 tracked configs permanently dirty after any
        // sync (blocking a rebase, and inviting `git add -u` to commit sync
        // output). Membership is unchanged here, so the bytes must be too.
        let tight = "include = [\"../nros-patch.toml\", \"nros-board.toml\"]\n";
        let unchanged =
            render_patch_config_with(tight, &mng(&[]), Some("../nros-patch.toml"), false, false)
                .unwrap();
        assert!(
            unchanged.starts_with(tight.trim_end()),
            "membership unchanged, so the array must be byte-identical:\n  was: {tight}  now: {unchanged}"
        );

        // And the board renderer, same rule, same shape.
        let board_same = render_board_include(tight, true).unwrap();
        assert_eq!(
            board_same, tight,
            "board include already present: nothing may be rewritten"
        );

        // A REAL change still renders (guard must not freeze the array).
        let board_added =
            render_board_include("include = [\"../nros-patch.toml\"]\n", true).unwrap();
        assert!(
            board_added.contains("nros-board.toml"),
            "a genuinely missing entry must still be added:\n{board_added}"
        );
        let board_dropped = render_board_include(tight, false).unwrap();
        assert!(
            !board_dropped.contains("nros-board.toml"),
            "eviction must still work:\n{board_dropped}"
        );

        // Same input, out-of-tree: merged in place, into the SAME quoted table.
        let ext = render_patch_config_with(
            existing,
            &mng(&[("nros-core", "../nros-core")]),
            None,
            false,
            false,
        )
        .unwrap();
        let edoc: toml_edit::DocumentMut = ext.parse().unwrap();
        let ecio = edoc["patch"]["crates-io"].as_table().unwrap();
        assert!(
            ecio.get("libc").is_some() && ecio.get("nros-core").is_some(),
            "external merge failed:\n{ext}"
        );
    }
}

/// phase-341 W2 — the board `cargo_config` projection.
#[cfg(test)]
mod board_projection_tests {
    use super::*;
    use crate::orchestration::board_descriptor::{
        BoardDescriptor, EntryKind, LinkKind, PlatformKind, Toolchain,
    };

    const NUTTX_BODY: &str = "\
[build]
target = \"armv7a-nuttx-eabihf\"

[target.armv7a-nuttx-eabihf]
linker = \"arm-none-eabi-gcc\"
rustflags = [
    \"-C\", \"link-arg=-Tdramboot.ld\",
]
";

    fn board(cargo_config: Option<&str>) -> BoardDescriptor {
        BoardDescriptor {
            names: vec!["nuttx".into()],
            west_board: None,
            platform: PlatformKind::Nuttx,
            target: None,
            toolchain: Toolchain::Nightly,
            platform_feature: "platform-nuttx".into(),
            local_aliases: vec![],
            link_kind_stated: Some(LinkKind::NuttxStaging),
            entry_kind: EntryKind::BoardRun,
            supported_netstacks: Vec::new(),
            chip: None,
            board_crate: Some("nros-board-nuttx-qemu".into()),
            crate_path: None,
            board_features: vec![],
            priority_plan: None,
            cargo_config: cargo_config.map(str::to_string),
            entry: None,
            disambiguate_by_target: None,
            capabilities: None,
            cmake: None,
            source: Some("packages/boards/nros-board-nuttx-qemu/nros-board.toml".into()),
            zephyr: None,
            provisioning: None,
        }
    }

    fn leaf(cfg: Option<&str>) -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        if let Some(body) = cfg {
            std::fs::create_dir_all(dir.path().join(".cargo")).unwrap();
            std::fs::write(dir.path().join(".cargo/config.toml"), body).unwrap();
        }
        dir
    }

    fn read_cfg(dir: &Path) -> String {
        std::fs::read_to_string(dir.join(".cargo/config.toml")).unwrap_or_default()
    }

    fn includes(cfg: &str) -> Vec<String> {
        let doc: toml_edit::DocumentMut = cfg.parse().expect("config parses");
        doc.get("include")
            .and_then(|i| i.as_array())
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str())
                    .map(String::from)
                    .collect()
            })
            .unwrap_or_default()
    }

    #[test]
    fn entry_deploy_key_reads_the_entry_table() {
        let manifest =
            "[package]\nname = \"x\"\n\n[package.metadata.nros.entry]\ndeploy = \"nuttx\"\n";
        assert_eq!(entry_deploy_key(manifest).as_deref(), Some("nuttx"));
        // A node pkg with no entry table is not an Entry leaf.
        assert_eq!(
            entry_deploy_key(
                "[package]\nname = \"x\"\n\n[package.metadata.nros.node]\nname = \"t\"\n"
            ),
            None
        );
        // An empty key is not a board (`entry-deploy-missing` is nros check's job).
        assert_eq!(
            entry_deploy_key("[package.metadata.nros.entry]\ndeploy = \"\"\n"),
            None
        );
        // Unparseable manifest — every other reader gives a better error.
        assert_eq!(entry_deploy_key("not ] toml ["), None);
    }

    /// The generated file must say who owns it and which descriptor it came
    /// from — a projection whose header names the wrong file sends the next
    /// reader to edit the wrong SSoT.
    #[test]
    fn projection_carries_a_do_not_edit_header_naming_its_descriptor() {
        let out = render_board_config("nuttx", "packages/boards/b/nros-board.toml", NUTTX_BODY);
        assert!(
            out.starts_with("# GENERATED by `nros sync` — DO NOT EDIT."),
            "{out}"
        );
        assert!(out.contains("packages/boards/b/nros-board.toml"), "{out}");
        assert!(out.contains("deploy = \"nuttx\""), "{out}");
        assert!(out.contains("link-arg=-Tdramboot.ld"), "{out}");
        // Still valid cargo config after the header.
        let doc: toml_edit::DocumentMut = out.parse().expect("projection is valid TOML");
        assert!(doc.get("build").is_some());
    }

    /// phase-351 W3 — the destination the withholding filter never had.
    ///
    /// The real NuttX descriptor shape: `[patch.crates-io] libc = { path =
    /// "${workspace}/third-party/nuttx/libc" }`. That path points INSIDE the
    /// repo, so relative-to-the-leaf it is identical in every checkout — the
    /// row is deliverable, it just cannot ride the PROJECTION (the leaf's own
    /// `config.toml` already owns `[patch.crates-io]`). So the body drops it and
    /// [`board_patch_rows`] hands it to sync's managed inline set.
    #[test]
    fn workspace_patch_row_is_delivered_relative_not_withheld() {
        let raw = format!(
            "{NUTTX_BODY}\n[patch.crates-io]\nlibc = {{ path = \"${{workspace}}/third-party/nuttx/libc\" }}\n"
        );
        let body = project_board_config(&raw, "../../../../").unwrap();
        assert!(
            !body.contains("${workspace}") && !body.contains("[patch"),
            "patch must not ride the projection:\n{body}"
        );
        assert!(
            body.contains("link-arg=-Tdramboot.ld"),
            "the rest must survive:\n{body}"
        );

        let rows = board_patch_rows(&raw, "../../../../").unwrap();
        assert_eq!(
            rows,
            vec![(
                "libc".to_string(),
                "../../../../third-party/nuttx/libc".to_string()
            )],
            "the row is DELIVERED, leaf-relative — not dropped"
        );

        let out = render_board_config("nuttx", "b/nros-board.toml", &body);
        assert!(
            !out.contains("WITHHELD"),
            "nothing is withheld any more:\n{out}"
        );
    }

    /// An `[env]` path must gain `relative = true`, or cargo passes the string
    /// through verbatim and the build resolves it against the process CWD — not
    /// the leaf, for anything cmake/corrosion launches.
    #[test]
    fn workspace_env_row_becomes_relative() {
        let raw = "[env]\nTHREADX_CONFIG_DIR = { value = \"${workspace}/packages/boards/b/config\", force = true }\nTHREADX_PORT = { value = \"risc-v64/gnu\", force = true }\n";
        let body = project_board_config(raw, "../../../../").unwrap();
        assert!(
            body.contains("value = \"../../../../packages/boards/b/config\""),
            "path not made leaf-relative:\n{body}"
        );
        assert!(body.contains("relative = true"), "{body}");
        assert!(body.contains("force = true"), "force must survive:\n{body}");
        // The placeholder-free sibling used to be collateral damage: the filter
        // withheld the WHOLE `[env]` table because one of its rows had a path.
        assert!(
            body.contains("THREADX_PORT")
                && !body
                    .contains("THREADX_PORT = { value = \"risc-v64/gnu\", force = true, relative"),
            "a row with no path must be untouched:\n{body}"
        );
    }

    /// A board whose `cargo_config` is ONLY a `[patch]` table now projects
    /// NOTHING — phase-351 W6.
    ///
    /// W2.0 made this case project an `[env] NROS_BOARD_TOML` row so a build
    /// script could find the board rung. W5 moved that delivery to the INVOKER
    /// (`nros ws board-facts`), which reaches workspace members too — the row
    /// never could, because corrosion runs cargo from the workspace root. With
    /// the row gone, a body holding only rows sync delivers inline has nothing
    /// left to project, which is the pre-W2.0 contract restored.
    #[test]
    fn patch_only_board_config_projects_nothing() {
        let dir = leaf(Some("[env]\nCC = \"gcc\"\n"));
        let outcome = write_board_projection(
            dir.path(),
            dir.path(),
            "nuttx",
            &board(Some(
                "[patch.crates-io]\nlibc = { path = \"${workspace}/x\" }\n",
            )),
        )
        .unwrap();
        assert_eq!(
            outcome,
            BoardProjection::NoBoardConfig,
            "a body of only inline-delivered rows projects nothing (W6)"
        );
        assert!(!dir.path().join(".cargo").join(BOARD_CONFIG_FILE).exists());
    }

    /// A descriptor whose `cargo_config` is not valid TOML fails loudly, naming
    /// the board — the projection writer is the only place that knows which
    /// descriptor the string came from.
    #[test]
    fn unparseable_board_config_fails_naming_the_board() {
        let dir = leaf(Some(""));
        let err = write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some("[build\n")))
            .expect_err("invalid TOML must not be written");
        let msg = format!("{err:#}");
        assert!(msg.contains("nuttx"), "{msg}");
        assert!(!dir.path().join(".cargo").join(BOARD_CONFIG_FILE).exists());
    }

    /// W2 lands the generator ALONGSIDE the hand-mirrored blocks. While a leaf
    /// still declares the same keys, adding the `include` would make cargo JOIN
    /// the two rustflags arrays and hand the linker `-Tdramboot.ld` twice — so
    /// the file is written and the include is withheld.
    #[test]
    fn mirror_still_present_blocks_the_include() {
        let dir = leaf(Some(NUTTX_BODY));
        let outcome =
            write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some(NUTTX_BODY)))
                .unwrap();
        assert!(
            matches!(outcome, BoardProjection::ShadowedByMirror(_)),
            "{outcome:?}"
        );
        assert!(
            dir.path().join(".cargo/nros-board.toml").is_file(),
            "projection must still be written — W3 and the W4 gate compare it"
        );
        assert!(
            includes(&read_cfg(dir.path())).is_empty(),
            "no include while the mirror governs:\n{}",
            read_cfg(dir.path())
        );
    }

    /// W3's migration step is a pure DELETION: drop the mirrored table, re-run
    /// sync, and the include appears.
    #[test]
    fn removing_the_mirror_turns_the_include_on() {
        // Authored remainder only — no key the board also declares.
        let dir = leaf(Some("[env]\nCC = \"arm-none-eabi-gcc\"\n"));
        let outcome =
            write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some(NUTTX_BODY)))
                .unwrap();
        assert_eq!(outcome, BoardProjection::Included);
        let cfg = read_cfg(dir.path());
        assert_eq!(includes(&cfg), vec![BOARD_CONFIG_FILE.to_string()], "{cfg}");
        assert!(
            cfg.contains("CC ="),
            "authored remainder must survive:\n{cfg}"
        );
        // Issue 0463 — the include names a file that exists, always.
        assert!(dir.path().join(".cargo").join(BOARD_CONFIG_FILE).is_file());
    }

    /// A `[build] target` in the leaf beside a `[build] rustflags` in the board
    /// is not a conflict (cargo merges distinct keys fine); the SAME key is.
    #[test]
    fn conflicts_are_computed_at_key_depth_two() {
        assert!(
            board_projection_conflicts("[build]\nrustflags = []\n", "[build]\ntarget = \"x\"\n")
                .is_empty()
        );
        assert_eq!(
            board_projection_conflicts("[build]\ntarget = \"x\"\n", "[build]\ntarget = \"x\"\n"),
            vec!["build.target".to_string()]
        );
        // Different triples do not collide; the same one does.
        assert!(
            board_projection_conflicts(
                "[target.riscv32imc-unknown-none-elf]\nrustflags = []\n",
                NUTTX_BODY
            )
            .is_empty()
        );
        assert_eq!(
            board_projection_conflicts(
                "[target.armv7a-nuttx-eabihf]\nlinker = \"x\"\n",
                NUTTX_BODY
            ),
            vec!["target.armv7a-nuttx-eabihf".to_string()]
        );
    }

    /// A board that loses its `cargo_config` now takes its projection AND its
    /// include with it — phase-351 W6.
    ///
    /// W2.0 kept the file alive for the `[env] NROS_BOARD_TOML` row; with that
    /// row delivered by the invoker instead, there is nothing left to keep.
    /// Issue 0463's invariant is what this really pins: the include must never
    /// name a file that is gone, and dropping BOTH satisfies it exactly as
    /// keeping both did.
    #[test]
    fn board_without_cargo_config_drops_projection_and_include() {
        let dir = leaf(Some("[env]\nCC = \"gcc\"\n"));
        write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some(NUTTX_BODY))).unwrap();
        assert_eq!(includes(&read_cfg(dir.path())).len(), 1);

        // The board loses its cargo_config (or the leaf re-deploys to a board
        // that never had one).
        let outcome =
            write_board_projection(dir.path(), dir.path(), "posix", &board(None)).unwrap();
        assert_eq!(outcome, BoardProjection::NoBoardConfig);
        assert!(
            !dir.path().join(".cargo").join(BOARD_CONFIG_FILE).exists(),
            "the projection must be removed, not left stale"
        );
        assert_eq!(
            includes(&read_cfg(dir.path())).len(),
            0,
            "no include may name the file just deleted (issue 0463):\n{}",
            read_cfg(dir.path())
        );
        assert!(
            read_cfg(dir.path()).contains("CC ="),
            "authored content lost"
        );
    }

    /// Both writers maintain the same `include` array. Neither may evict the
    /// other's entry, and a full sync must converge to the same file.
    #[test]
    fn board_and_patch_includes_coexist() {
        let dir = leaf(Some(""));
        write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some(NUTTX_BODY))).unwrap();
        let after_board = read_cfg(dir.path());
        // The patch pass runs over the same file next sync.
        let after_patch = render_patch_config_with(
            &after_board,
            &[("std_msgs".into(), "generated/std_msgs".into())],
            Some("../../nros-patch.toml"),
            true,
            false,
        )
        .unwrap();
        let inc = includes(&after_patch);
        assert!(
            inc.iter().any(|e| e.ends_with(BOARD_CONFIG_FILE)),
            "patch pass evicted the board include: {inc:?}"
        );
        assert!(inc.iter().any(|e| e.ends_with(CENTRAL_PATCH_FILE)));
        assert!(inc.iter().any(|e| e.ends_with(MANAGED_PATCH_FILE)));
        // …and the board pass, running last, re-adds exactly one entry.
        let converged = render_board_include(&after_patch, true).unwrap();
        assert_eq!(
            includes(&converged)
                .iter()
                .filter(|e| e.ends_with(BOARD_CONFIG_FILE))
                .count(),
            1,
            "duplicate board include:\n{converged}"
        );
    }

    /// Re-syncing must not touch either file when nothing changed: a rewritten
    /// mtime restales every prebuilt fixture keyed on these inputs.
    #[test]
    fn resync_is_idempotent_on_disk() {
        let dir = leaf(Some("[env]\nCC = \"gcc\"\n"));
        write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some(NUTTX_BODY))).unwrap();
        let proj = dir.path().join(".cargo").join(BOARD_CONFIG_FILE);
        let cfg = dir.path().join(".cargo/config.toml");
        let (m1, m2) = (
            std::fs::metadata(&proj).unwrap().modified().unwrap(),
            std::fs::metadata(&cfg).unwrap().modified().unwrap(),
        );
        write_board_projection(dir.path(), dir.path(), "nuttx", &board(Some(NUTTX_BODY))).unwrap();
        assert_eq!(std::fs::metadata(&proj).unwrap().modified().unwrap(), m1);
        assert_eq!(std::fs::metadata(&cfg).unwrap().modified().unwrap(), m2);
    }

    /// A placeholder inside a COMMENT is prose, not a path. The rewrite walks
    /// the DOM, so a comment keeps its text verbatim — including the literal
    /// `${workspace}`, which is what the comment is explaining.
    #[test]
    fn a_placeholder_in_a_comment_is_left_alone() {
        let raw = "# see ${workspace}/third-party for the script\n[build]\ntarget = \"x\"\n";
        let body = project_board_config(raw, "../../").unwrap();
        assert!(body.contains("# see ${workspace}/third-party"), "{body}");
        assert!(body.contains("target = \"x\""), "{body}");
    }
}

#[cfg(test)]
mod provenance_tests {
    // Issue 0320 — content-addressed staleness for committed SystemModels.
    use super::*;

    fn sha(bytes: &[u8]) -> String {
        format!("{:x}", Sha256::digest(bytes))
    }

    fn write_model(dir: &Path, inputs: Vec<(String, String)>) -> PathBuf {
        write_model_with_pin(dir, inputs, env!("NROS_PLAY_LAUNCH_SHA"))
    }

    fn write_model_with_pin(dir: &Path, inputs: Vec<(String, String)>, pin: &str) -> PathBuf {
        let mut m = ros_launch_manifest_model::SystemModel::default();
        m.meta.version = ros_launch_manifest_model::SCHEMA_VERSION;
        m.meta.inputs = inputs
            .into_iter()
            .map(|(path, sha256)| ros_launch_manifest_model::InputHash { path, sha256 })
            .collect();
        // Issue 0427 — stamp the resolver pin the same way `stamp_resolver_pin` does.
        m.meta.resolver = Some(ros_launch_manifest_model::ResolverInfo {
            tool: "nros-launch-resolve".into(),
            version: pin.into(),
        });
        let p = dir.join("system_model.yaml");
        std::fs::write(&p, serde_yaml_ng::to_string(&m).unwrap()).unwrap();
        p
    }

    #[test]
    fn intact_provenance_is_not_stale() {
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path();
        let content = b"[system]\n";
        std::fs::write(bringup.join("system.toml"), content).unwrap();
        let model = write_model(bringup, vec![("system.toml".into(), sha(content))]);
        assert_eq!(model_provenance_stale(&model, bringup), None);
    }

    #[test]
    fn changed_hash_is_stale() {
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path();
        std::fs::write(bringup.join("system.toml"), b"new\n").unwrap();
        let model = write_model(bringup, vec![("system.toml".into(), sha(b"old\n"))]);
        assert!(
            model_provenance_stale(&model, bringup)
                .unwrap()
                .contains("hash changed")
        );
    }

    /// The 43 legacy models: an absolute path is non-portable and must
    /// regenerate even when the file it points at still exists and matches.
    #[test]
    fn absolute_path_is_stale_even_when_file_matches() {
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path();
        let abs = bringup.join("system.toml");
        std::fs::write(&abs, b"x\n").unwrap();
        let model = write_model(bringup, vec![(abs.display().to_string(), sha(b"x\n"))]);
        assert!(
            model_provenance_stale(&model, bringup)
                .unwrap()
                .contains("absolute")
        );
    }

    #[test]
    fn missing_input_is_stale() {
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path();
        let model = write_model(bringup, vec![("gone.toml".into(), sha(b"x"))]);
        assert!(
            model_provenance_stale(&model, bringup)
                .unwrap()
                .contains("missing")
        );
    }

    /// Issue 0427 — a model whose inputs are byte-identical but was produced by a
    /// DIFFERENT resolver pin is stale, so a resolver fix reaches existing models.
    #[test]
    fn resolver_pin_change_is_stale() {
        // Skip when our own pin is unverifiable — the check itself is disabled then.
        if env!("NROS_PLAY_LAUNCH_SHA") == "unknown" {
            return;
        }
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path();
        let content = b"[system]\n";
        std::fs::write(bringup.join("system.toml"), content).unwrap();
        // Same inputs + hash, but a stale resolver pin.
        let model = write_model_with_pin(
            bringup,
            vec![("system.toml".into(), sha(content))],
            "deadbeefdeadbeef",
        );
        assert!(
            model_provenance_stale(&model, bringup)
                .unwrap()
                .contains("resolver pin changed"),
            "a model with a foreign resolver pin must be stale"
        );
    }

    /// Issue 0427 — a model with NO recorded resolver pin (pre-fix / legacy) is
    /// stale, so it re-resolves and gains the pin.
    #[test]
    fn missing_resolver_pin_is_stale() {
        if env!("NROS_PLAY_LAUNCH_SHA") == "unknown" {
            return;
        }
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path();
        let content = b"[system]\n";
        std::fs::write(bringup.join("system.toml"), content).unwrap();
        // Build a model with inputs but NO resolver stamp.
        let mut m = ros_launch_manifest_model::SystemModel::default();
        m.meta.version = ros_launch_manifest_model::SCHEMA_VERSION;
        m.meta.inputs = vec![ros_launch_manifest_model::InputHash {
            path: "system.toml".into(),
            sha256: sha(content),
        }];
        let model = bringup.join("system_model.yaml");
        std::fs::write(&model, serde_yaml_ng::to_string(&m).unwrap()).unwrap();
        assert!(
            model_provenance_stale(&model, bringup)
                .unwrap()
                .contains("no resolver pin"),
            "a model with no resolver pin must be stale"
        );
    }

    /// phase-327 W5 (issue 0368 F4) — the narrowing guard's decision table.
    /// A still-requested generated crate missing from the new entry set is a
    /// failed generation (refuse); a no-longer-requested one is a removed
    /// dependency (allow); runtime crates and user rows are out of scope.
    #[test]
    fn narrowing_guard_distinguishes_failed_generation_from_removed_dep() {
        let existing = r#"
[patch.crates-io]
libc = { path = "../../../third-party/nuttx/libc" }
example_interfaces = { path = "generated/example_interfaces" }  # nros-managed
action_msgs = { path = "generated/action_msgs" }  # nros-managed
std_msgs = { path = "generated/std_msgs" }  # nros-managed
nros-zephyr-build = { path = "../../packages/tooling/nros-zephyr-build" }  # nros-managed
"#;
        let requested: HashSet<String> = ["example_interfaces", "std_msgs", "rclcpp"]
            .iter()
            .map(|s| s.to_string())
            .collect();

        // This run only produced std_msgs: example_interfaces (requested!)
        // would be dropped -> narrowing. action_msgs is absent from
        // `requested` -> legitimately removed, not flagged. The runtime
        // crate row (non-generated path) is never in scope, and the user's
        // own libc row (no decor) is invisible to the guard.
        let new_names: HashSet<&str> = ["std_msgs"].into_iter().collect();
        assert_eq!(
            narrowed_generated_entries(existing, &new_names, &requested),
            vec!["example_interfaces".to_string()]
        );

        // Full regeneration -> nothing narrowed.
        let full: HashSet<&str> = ["std_msgs", "example_interfaces"].into_iter().collect();
        assert!(narrowed_generated_entries(existing, &full, &requested).is_empty());

        // No existing managed block (fresh leaf) -> nothing to narrow.
        assert!(narrowed_generated_entries("", &new_names, &requested).is_empty());
    }
}

#[cfg(test)]
mod params_projection_tests {
    use super::*;

    /// Write a `system.toml` + model pair into a scratch dir and run the check.
    fn check(system_toml: &str, model_yaml: &str) -> Result<()> {
        let dir = tempfile::TempDir::new().unwrap();
        let sys = dir.path().join("system.toml");
        let model = dir.path().join("system_model.yaml");
        std::fs::write(&sys, system_toml).unwrap();
        std::fs::write(&model, model_yaml).unwrap();
        verify_params_projected(&model, &sys)
    }

    const DECLARING: &str = r#"
[[component]]
pkg = "c_param_talker_pkg"
class = "c_param_talker_pkg::Talker"
name = "c_params_param_talker"
params = { "an_int" = 42 }
"#;

    /// The projection happened: the node from that pkg carries params.
    #[test]
    fn projected_params_pass() {
        let model = r#"
meta:
  version: 1
structure:
  nodes:
    /param_talker:
      pkg: c_param_talker_pkg
      params:
        an_int: 42
"#;
        assert!(check(DECLARING, model).is_ok());
    }

    /// issue 0409 — the node EXISTS and has no params. The projection had a
    /// target and produced nothing: that is the silent data loss.
    #[test]
    fn dropped_params_are_rejected() {
        let model = r#"
meta:
  version: 1
structure:
  nodes:
    /param_talker:
      pkg: c_param_talker_pkg
"#;
        let err = check(DECLARING, model).unwrap_err().to_string();
        assert!(err.contains("did not project"), "unexpected error: {err}");
        assert!(
            err.contains("c_params_param_talker"),
            "must name the component: {err}"
        );
        assert!(
            err.contains("setup-launch-resolve"),
            "must give the remedy: {err}"
        );
    }

    /// A component absent from THIS variant is legitimate — and a correct
    /// resolver says so. The diagnostic is the evidence it looked.
    #[test]
    fn unbound_component_with_a_diagnostic_passes() {
        let model = r#"
meta:
  version: 1
  diagnostics:
  - "system config: [[component]] 'c_params_param_talker' declares params but has no matching launch node (absent in this variant?)"
structure:
  nodes:
    /other:
      pkg: some_other_pkg
"#;
        assert!(check(DECLARING, model).is_ok());
    }

    /// No node AND no diagnostic: nothing considered the declaration. That is
    /// the stale-resolver signature — it emits neither params nor diagnostics.
    #[test]
    fn unbound_without_a_diagnostic_is_rejected() {
        let model = r#"
meta:
  version: 1
structure:
  nodes:
    /other:
      pkg: some_other_pkg
"#;
        let err = check(DECLARING, model).unwrap_err().to_string();
        assert!(
            err.contains("no diagnostic explaining why"),
            "unexpected error: {err}"
        );
    }

    /// A bringup that declares no params is not this check's business.
    #[test]
    fn no_declarations_is_a_no_op() {
        let sys = r#"
[[component]]
pkg = "plain_pkg"
class = "plain_pkg::Talker"
name = "plain"
"#;
        assert!(check(sys, "meta:\n  version: 1\n").is_ok());
    }
}

// =============================================================================
// The search path and its shadowing report — phase-420 W6 (RFC-0087 D6)
// =============================================================================

#[cfg(test)]
mod search_path_tests {
    use super::*;
    use std::fs;

    fn provider_xml(name: &str, kind: &str, provides: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.0</version>
  <export>
    <nano_ros_provides kind="{kind}" name="{provides}"/>
  </export>
</package>"#
        )
    }

    fn write_provider(dir: &Path, name: &str, kind: &str, provides: &str) {
        fs::create_dir_all(dir).unwrap();
        fs::write(dir.join("package.xml"), provider_xml(name, kind, provides)).unwrap();
    }

    /// The whole path assembled the way the COMMAND assembles it, from a real
    /// `nros.toml` on disk: a third root, neither the nano-ros tree nor the
    /// workspace, is searched and its provider is selected by name.
    ///
    /// This is the wave's acceptance criterion, and the thing
    /// `serdes_resolver::tests::a_provider_in_the_user_workspace_reaches_the_default_search_path`
    /// deliberately could not reach — it proved the TWO-root case and said so.
    #[test]
    fn a_provider_in_a_third_configured_root_is_selected_by_name() {
        let tmp = tempfile::tempdir().unwrap();
        let nano_ros = tmp.path().join("nano-ros");
        let workspace = tmp.path().join("robot_ws");
        let third = tmp.path().join("elsewhere/nros-packages");
        fs::create_dir_all(&nano_ros).unwrap();
        fs::create_dir_all(&workspace).unwrap();
        write_provider(&third.join("acme_rmw"), "acme_rmw", "rmw", "acme");
        fs::write(
            workspace.join("nros.toml"),
            format!("[workspace]\npackage_paths = [\"{}\"]\n", third.display()),
        )
        .unwrap();

        let path = provider_search_path_with(Some(&nano_ros), &workspace)
            .expect("the search path assembles");
        assert_eq!(
            path.roots.iter().map(|r| r.origin).collect::<Vec<_>>(),
            vec![
                provider_scan::RootOrigin::NanoRosTree,
                provider_scan::RootOrigin::Workspace,
                provider_scan::RootOrigin::PackagePaths,
            ],
            "{:?}",
            path.roots,
        );

        let scan = provider_scan::scan_roots(&path.paths()).unwrap();
        let r = provider_scan::resolve_unique(&scan, "rmw", "acme")
            .expect("the out-of-repo provider resolves by name");
        assert_eq!(r.winner.package, "acme_rmw");
        assert_eq!(r.winner.root_index, 2);
    }

    /// Order decides a cross-root collision, end to end through the config
    /// file: the LATER root wins, and the loser is still in the scan so the
    /// report can name it.
    #[test]
    fn order_decides_a_cross_root_collision_through_the_config_file() {
        let tmp = tempfile::tempdir().unwrap();
        let workspace = tmp.path().join("ws");
        let first = tmp.path().join("first");
        let second = tmp.path().join("second");
        fs::create_dir_all(&workspace).unwrap();
        write_provider(&first.join("p"), "from_first", "serdes", "flatbuf");
        write_provider(&second.join("p"), "from_second", "serdes", "flatbuf");
        fs::write(
            workspace.join("nros.toml"),
            format!(
                "[workspace]\npackage_paths = [\"{}\", \"{}\"]\n",
                first.display(),
                second.display()
            ),
        )
        .unwrap();

        let path = provider_search_path_with(None, &workspace).unwrap();
        let scan = provider_scan::scan_roots(&path.paths()).unwrap();
        let r = provider_scan::resolve_unique(&scan, "serdes", "flatbuf").unwrap();
        assert_eq!(r.winner.package, "from_second", "the later root wins");
        assert_eq!(
            r.shadowed
                .iter()
                .map(|p| p.package.as_str())
                .collect::<Vec<_>>(),
            vec!["from_first"],
        );
    }

    /// The report NAMES the hidden provider — on the winner's row, and again on
    /// the loser's own row so a reader scanning the flat list is not misled
    /// into using one that will never be selected.
    #[test]
    fn the_listing_names_the_provider_that_was_hidden() {
        let tmp = tempfile::tempdir().unwrap();
        let under = tmp.path().join("underlay");
        let over = tmp.path().join("overlay");
        write_provider(&under.join("shipped"), "shipped_zenoh", "rmw", "zenoh");
        write_provider(&over.join("patched"), "patched_zenoh", "rmw", "zenoh");

        let path = provider_scan::SearchPath::from_base_paths(&[under.clone(), over.clone()]);
        let scan = provider_scan::scan_roots(&path.paths()).unwrap();
        let shadowed = provider_scan::shadowing(&scan);
        let text = render_provider_listing(&path, &scan, None, &shadowed);

        assert!(
            text.contains("shadows  shipped_zenoh"),
            "the winner's row must say what it hid:\n{text}"
        );
        assert!(
            text.contains("shadowed by root[1]"),
            "and the loser's own row must say it lost, and to which root:\n{text}"
        );
        assert!(
            text.contains("1 contested provision(s), 1 package(s) shadowed"),
            "a reader who was not looking for shadowing still learns it happened:\n{text}"
        );
        assert!(
            !text.contains("AMBIGUOUS"),
            "a cross-root collision RESOLVES; only a same-root one is ambiguous:\n{text}"
        );
    }

    /// A same-root collision is reported as AMBIGUOUS rather than counted as
    /// shadowing: there is no winner to name, and `resolve_unique` refuses.
    #[test]
    fn a_same_root_collision_is_listed_as_ambiguous() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("one_root");
        write_provider(&root.join("a"), "acme_a", "rmw", "acme");
        write_provider(&root.join("b"), "acme_b", "rmw", "acme");

        let path = provider_scan::SearchPath::from_base_paths(&[root]);
        let scan = provider_scan::scan_roots(&path.paths()).unwrap();
        let shadowed = provider_scan::shadowing(&scan);
        let text = render_provider_listing(&path, &scan, None, &shadowed);

        assert!(text.contains("AMBIGUOUS rmw:acme"), "{text}");
        assert_eq!(
            text.matches("(AMBIGUOUS — see below)").count(),
            2,
            "BOTH rows of a tie are marked — there is no winner among them, so \
             singling one out would name a selection that will not happen:\n{text}"
        );
        assert!(
            !text.contains("Rename one"),
            "the message must not prescribe a rename: `board` `threadx` is \
             legitimately claimed twice in one root and separated by its \
             descriptor (phase-348 W2):\n{text}"
        );
        assert!(
            !text.contains("contested provision(s)"),
            "ambiguity is not shadowing — counting it as such would promise a \
             winner the build will refuse:\n{text}"
        );
    }

    /// A root that does not exist is REPORTED and not fatal. The listing marks
    /// it MISSING in place, keeping its index, so the reader can see that the
    /// root they configured contributed nothing — as distinct from being a root
    /// that holds no providers.
    #[test]
    fn a_nonexistent_configured_root_is_marked_missing_and_is_not_fatal() {
        let tmp = tempfile::tempdir().unwrap();
        let workspace = tmp.path().join("ws");
        let real = tmp.path().join("real");
        fs::create_dir_all(&workspace).unwrap();
        write_provider(&real.join("p"), "real_rmw", "rmw", "acme");
        fs::write(
            workspace.join("nros.toml"),
            format!(
                "[workspace]\npackage_paths = [\"{}/gone\", \"{}\"]\n",
                tmp.path().display(),
                real.display()
            ),
        )
        .unwrap();

        let path = provider_search_path_with(None, &workspace)
            .expect("a missing root must not fail the assembly");
        assert_eq!(path.missing().len(), 1, "{:?}", path.roots);

        let scan = provider_scan::scan_roots(&path.paths())
            .expect("a missing root must not fail the scan");
        assert!(
            provider_scan::resolve_unique(&scan, "rmw", "acme").is_ok(),
            "the roots that ARE there still answer"
        );

        let text = render_provider_listing(&path, &scan, None, &provider_scan::shadowing(&scan));
        assert!(
            text.contains("MISSING — nothing scanned"),
            "a configured root that contributed nothing must say so:\n{text}"
        );
    }

    // ---- issue 0827: the derived `[env]` sidecar ----------------------------

    /// The include entry appears ONLY when the file will be written. A missing
    /// include target is a hard cargo error during manifest parse (issue 0463),
    /// so "wanted" and "written" are one decision, not two.
    #[test]
    fn env_sidecar_include_is_added_only_when_wanted() {
        let with = render_patch_config_with("", &[], None, true, true).unwrap();
        assert!(
            with.contains(MANAGED_ENV_FILE),
            "include missing the env sidecar:\n{with}"
        );
        let without = render_patch_config_with("", &[], None, true, false).unwrap();
        assert!(
            !without.contains(MANAGED_ENV_FILE),
            "include names a file that will not be written:\n{without}"
        );
    }

    /// And it is EVICTED when no longer wanted, so a leaf that loses its
    /// probeable component does not keep an include to a deleted file.
    #[test]
    fn env_sidecar_include_is_evicted_when_no_longer_wanted() {
        let existing = format!("include = [\"{MANAGED_ENV_FILE}\"]\n");
        let out = render_patch_config_with(&existing, &[], None, true, false).unwrap();
        assert!(
            !out.contains(MANAGED_ENV_FILE),
            "stale include survived:\n{out}"
        );
    }

    /// The two sidecars are independent: a leaf may want either, both or
    /// neither, which is why they are separate files.
    #[test]
    fn the_two_sidecars_do_not_depend_on_each_other() {
        // Built inline rather than via the `mng` helper: these tests live in a
        // different test module, and reaching across for a two-line helper
        // would couple them for no benefit.
        let generated: Vec<(String, String)> =
            vec![("std_msgs".to_string(), "generated/std_msgs".to_string())];
        let both = render_patch_config_with("", &generated, None, true, true).unwrap();
        assert!(
            both.contains(MANAGED_PATCH_FILE) && both.contains(MANAGED_ENV_FILE),
            "{both}"
        );

        let env_only = render_patch_config_with("", &[], None, true, true).unwrap();
        assert!(!env_only.contains(MANAGED_PATCH_FILE), "{env_only}");
        assert!(env_only.contains(MANAGED_ENV_FILE), "{env_only}");

        let patch_only = render_patch_config_with("", &generated, None, true, false).unwrap();
        assert!(patch_only.contains(MANAGED_PATCH_FILE), "{patch_only}");
        assert!(!patch_only.contains(MANAGED_ENV_FILE), "{patch_only}");
    }

    /// An OUT-OF-TREE consumer gets no include at all (#272), so it must not
    /// gain an env one either.
    #[test]
    fn an_external_consumer_gets_no_env_include() {
        let out = render_patch_config_with("", &[], None, false, false).unwrap();
        assert!(!out.contains(MANAGED_ENV_FILE), "{out}");
    }
}
