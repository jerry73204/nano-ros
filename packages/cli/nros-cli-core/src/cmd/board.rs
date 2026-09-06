//! `nros board` — board crate introspection.
//!
//! * `list` — Phase 111.A.8: enumerate every `nros-board-*` crate under
//!   `<workspace>/packages/boards/`.
//! * `info <name>` — Phase 215.C.3: print the side-by-side `Cargo.toml` +
//!   `board.cmake` views of a board crate's manifest, optionally erroring
//!   when the two faces drift (the Phase 215.F audit hook).

use clap::{Args as ClapArgs, Subcommand};
use eyre::{Result, WrapErr, eyre};
use serde::Serialize;
use std::{
    fs,
    path::{Path, PathBuf},
};

use crate::orchestration::{
    board_descriptor::{BoardCatalog, BoardDescriptor},
    board_projection,
};

#[derive(Debug, Subcommand)]
pub enum Args {
    /// List every supported board crate
    List(ListArgs),
    /// Print a board's resolved descriptor as JSON.
    Info(InfoArgs),
    /// Project a board descriptor into cmake variables (RFC-0064 R5 D4).
    ///
    /// Written per configure into the build directory by
    /// `nano_ros_use_board()`; never committed. The descriptor is the single
    /// authored source, and this is the mechanical projection of it.
    CmakeVars(CmakeVarsArgs),
}

#[derive(Debug, ClapArgs)]
pub struct ListArgs {
    /// Path to the nano-ros workspace root (auto-detected by walking
    /// upward from cwd if omitted)
    #[arg(long)]
    pub workspace: Option<PathBuf>,
}

#[derive(Debug, ClapArgs)]
pub struct InfoArgs {
    /// Board key. Resolves to a board CRATE
    /// (`packages/boards/nros-board-<name>/`) or, since phase-337 W9.a, to a
    /// conf BUNDLE under a family crate
    /// (`packages/boards/nros-board-<family>/boards/<name>/`) — e.g.
    /// `fvp-aemv8r-smp` is `nros-board-zephyr/boards/fvp-aemv8r-smp/`.
    pub name: String,
    /// Path to the nano-ros workspace root (auto-detected by walking
    /// upward from cwd if omitted). May also be set via
    /// `NROS_WORKSPACE_ROOT`.
    #[arg(long)]
    pub workspace: Option<PathBuf>,
}

#[derive(Debug, ClapArgs)]
pub struct CmakeVarsArgs {
    /// Board key, as `nano_ros_use_board(<key>)` spells it.
    pub name: String,
    /// Path to the nano-ros workspace root (auto-detected by walking upward
    /// from cwd if omitted).
    #[arg(long)]
    pub workspace: Option<PathBuf>,
    /// Where to write the projection. Its directory also receives the
    /// generated Rust-support Kconfig module when the board needs one.
    #[arg(long)]
    pub out: PathBuf,
}

pub fn run(args: Args) -> Result<()> {
    match args {
        Args::List(args) => list(args),
        Args::Info(args) => info(args),
        Args::CmakeVars(args) => cmake_vars(args),
    }
}

fn list(args: ListArgs) -> Result<()> {
    let root = match args.workspace {
        Some(p) => p,
        None => find_workspace_root()?,
    };
    let boards_dir = root.join("packages").join("boards");
    if !boards_dir.is_dir() {
        return Err(eyre!(
            "no `packages/boards/` directory under {}",
            root.display()
        ));
    }

    let mut entries: Vec<BoardEntry> = Vec::new();
    for entry in fs::read_dir(&boards_dir)
        .wrap_err_with(|| format!("failed to read {}", boards_dir.display()))?
    {
        let entry = entry?;
        let path = entry.path();
        let cargo_toml = path.join("Cargo.toml");
        if !cargo_toml.is_file() {
            continue;
        }
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        if !name.starts_with("nros-board-") {
            continue;
        }
        match read_board(&cargo_toml) {
            Ok(b) => entries.push(b),
            Err(e) => eprintln!("warning: skipping {}: {e}", name),
        }
    }
    entries.sort_by(|a, b| a.name.cmp(&b.name));

    if entries.is_empty() {
        println!("No board crates found under {}", boards_dir.display());
        return Ok(());
    }

    let name_w = entries
        .iter()
        .map(|e| e.name.len())
        .max()
        .unwrap_or(4)
        .max(4);
    println!("{:<name_w$}  description", "name", name_w = name_w);
    println!(
        "{:<name_w$}  {}",
        "-".repeat(name_w),
        "-".repeat(60),
        name_w = name_w
    );
    for b in entries {
        println!("{:<name_w$}  {}", b.name, b.description, name_w = name_w);
    }
    Ok(())
}

struct BoardEntry {
    name: String,
    description: String,
}

fn read_board(cargo_toml: &Path) -> Result<BoardEntry> {
    let raw = fs::read_to_string(cargo_toml)?;
    let doc: toml_edit::DocumentMut = raw.parse()?;
    let pkg = doc
        .get("package")
        .and_then(|p| p.as_table())
        .ok_or_else(|| eyre!("no [package] table in {}", cargo_toml.display()))?;
    let name = pkg
        .get("name")
        .and_then(|n| n.as_str())
        .ok_or_else(|| eyre!("no [package].name in {}", cargo_toml.display()))?
        .to_string();
    let description = pkg
        .get("description")
        .and_then(|d| d.as_str())
        .unwrap_or("")
        .to_string();
    Ok(BoardEntry { name, description })
}

// -----------------------------------------------------------------------
// Phase 215.C.3 — `nros board info <name>`
// -----------------------------------------------------------------------

/// JSON envelope produced by `nros board info`.
///
/// RFC-0064 R5 D4 removed the second face. This used to print a Cargo.toml
/// view beside a `board.cmake` view with a computed `drift` list; there is now
/// one authored file, so there is nothing to print twice and nothing to drift.
#[derive(Debug, Serialize)]
struct BoardInfo {
    name: String,
    board_dir: PathBuf,
    descriptor: PathBuf,
    /// The projection this board would produce for a Zephyr build, or `null`
    /// for a board with no `[board.zephyr]` block. Printed because "what does
    /// cmake actually see?" is the question `board info` was invented to
    /// answer, and it is now derivable rather than authored.
    cmake_vars: Option<String>,
}

/// Resolve `name` to its descriptor through the board CATALOG.
///
/// Not by a path convention. `locate_board_crate` below still exists for the
/// consumers that need a directory, but resolution by NAME must go through the
/// catalog, or `nros board info <alias>` and a build of the same alias answer
/// differently — which is what a board resolving only as an alias on another
/// board's descriptor used to guarantee.
fn resolve(root: &Path, name: &str) -> Result<(BoardDescriptor, PathBuf)> {
    let catalog = BoardCatalog::load(root).map_err(|e| eyre!("load board catalog: {e}"))?;
    let board = catalog
        .descriptors()
        .iter()
        .find(|d| d.names.iter().any(|n| n == name))
        .ok_or_else(|| {
            let mut known: Vec<&str> = catalog
                .descriptors()
                .iter()
                .flat_map(|d| d.names.iter().map(String::as_str))
                .collect();
            known.sort_unstable();
            known.dedup();
            eyre!(
                "no board named `{name}`.\n  Known: {}\n  \
                 For an out-of-tree board, put its package directory on \
                 $NROS_EXTRA_BOARD_PATH.",
                known.join(", ")
            )
        })?
        .clone();
    let descriptor = board
        .source
        .as_deref()
        .map(|rel| {
            let p = Path::new(rel);
            if p.is_absolute() {
                p.to_path_buf()
            } else {
                root.join(rel)
            }
        })
        .ok_or_else(|| eyre!("board `{name}` has no recorded descriptor path"))?;
    let dir = descriptor
        .parent()
        .ok_or_else(|| eyre!("descriptor {} has no parent", descriptor.display()))?
        .to_path_buf();
    Ok((board, dir))
}

fn info(args: InfoArgs) -> Result<()> {
    let root = match args.workspace {
        Some(p) => p,
        None => find_workspace_root()?,
    };
    let (board, board_dir) = resolve(&root, &args.name)?;
    // A module dir is needed to project, but `info` writes nothing — name the
    // path the build WOULD use rather than inventing a second spelling.
    let module_dir = board_dir.join("<build-dir>/nros-board-rust-support");
    let cmake_vars = board_projection::project(&board, &board_dir, &module_dir).ok();

    let info = BoardInfo {
        name: args.name.clone(),
        descriptor: board_dir.join("nros-board.toml"),
        board_dir,
        cmake_vars,
    };
    let json = serde_json::to_string_pretty(&info).wrap_err("serialise BoardInfo as JSON")?;
    println!("{json}");
    Ok(())
}

fn cmake_vars(args: CmakeVarsArgs) -> Result<()> {
    let root = match args.workspace {
        Some(p) => p,
        None => find_workspace_root()?,
    };
    let (board, board_dir) = resolve(&root, &args.name)?;

    let out_dir = args
        .out
        .parent()
        .ok_or_else(|| eyre!("--out {} has no parent directory", args.out.display()))?;
    fs::create_dir_all(out_dir).wrap_err_with(|| format!("create {}", out_dir.display()))?;

    let module_dir = out_dir.join(format!("nros-board-{}-rust-support", args.name));
    let text = board_projection::project(&board, &board_dir, &module_dir)?;

    // Generate the Rust-support module only for a board that asked for Rust —
    // the projection references it under exactly the same condition, so the two
    // read the same field rather than agreeing by accident.
    let wants_rust = board
        .provisioning
        .as_ref()
        .is_some_and(|p| !p.rust_targets.is_empty());
    if wants_rust && let Some(z) = board.zephyr.as_ref() {
        board_projection::write_rust_support_module(&module_dir, &args.name, &z.west_board)?;
    }

    fs::write(&args.out, &text).wrap_err_with(|| format!("write {}", args.out.display()))?;
    Ok(())
}

/// Walk upward from cwd until a directory containing `packages/boards/`
/// is found. The `NROS_WORKSPACE_ROOT` env var, when set, short-
/// circuits the walk (matches `nros_build::pkg_index::detect_workspace_root`).
pub(crate) fn find_workspace_root() -> Result<PathBuf> {
    if let Some(override_) = std::env::var_os("NROS_WORKSPACE_ROOT") {
        let p = PathBuf::from(override_);
        if !p.exists() {
            return Err(eyre!(
                "NROS_WORKSPACE_ROOT=`{}` does not exist on disk",
                p.display()
            ));
        }
        return Ok(p);
    }
    let cwd = std::env::current_dir()?;
    let mut cur: &Path = &cwd;
    loop {
        if cur.join("packages").join("boards").is_dir() {
            return Ok(cur.to_path_buf());
        }
        match cur.parent() {
            Some(p) => cur = p,
            None => {
                return Err(eyre!(
                    "could not auto-detect nano-ros workspace root from {}; \
                     pass --workspace <path> explicitly",
                    cwd.display()
                ));
            }
        }
    }
}
