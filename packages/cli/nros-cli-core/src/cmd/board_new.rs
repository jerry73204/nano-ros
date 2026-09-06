//! `nros board new <name>` — scaffold a board package.
//!
//! phase-375 W2/W9. Onboarding a board used to be a scavenger hunt, and the
//! phase's own measurement is why this exists: `s32z270` landed RED on five
//! gates (weak symbols, board tiers, leaf lock, provider announcements, matrix
//! orphan) and `freertos-posix` on two plus a lane-table cascade. Each gate was
//! correct. The cost was discovering them serially, on main, by whoever noticed.
//!
//! So the scaffold's acceptance is not "it writes some files" — it is that a
//! board created by it is green on `just check fast` before its first commit.
//!
//! W2 was written before there WAS one board shape to scaffold; W6–W8 made one
//! (a `package.xml` announcement beside an `nros-board.toml`), which is why
//! this lands after them rather than before.
//!
//! `--out-of-tree <dir>` writes the same package somewhere else and prints the
//! one line that makes it resolvable. That is the whole difference between our
//! board and a user's, and keeping it to one flag is the point of RFC-0064 R5
//! D1.

use std::path::{Path, PathBuf};

use clap::Args as ClapArgs;
use eyre::{Result, WrapErr, bail};

#[derive(Debug, ClapArgs)]
pub struct NewArgs {
    /// Board key, kebab-case. Becomes the directory name, the canonical
    /// `names` entry and the announcement.
    pub name: String,
    /// Platform this board runs on (`zephyr`, `freertos`, `nuttx`, `threadx`,
    /// `bare-metal`, `posix`, `esp32`).
    #[arg(long)]
    pub platform: String,
    /// Zephyr board id (`<board>/<soc>/<variant>`). Required for, and only
    /// meaningful on, `--platform zephyr`.
    #[arg(long)]
    pub west_board: Option<String>,
    /// Write the package outside the nano-ros tree. The board is then reached
    /// through `$NROS_EXTRA_BOARD_PATH`, which this prints.
    #[arg(long)]
    pub out_of_tree: Option<PathBuf>,
    /// nano-ros workspace root (auto-detected from cwd if omitted).
    #[arg(long)]
    pub workspace: Option<PathBuf>,
    /// Print what would be written and write nothing.
    #[arg(long)]
    pub dry_run: bool,
}

/// Board key → the package name a `package.xml` declares.
fn package_name(board: &str) -> String {
    format!("nros_board_{}", board.replace(['-', '.'], "_"))
}

pub fn run(args: NewArgs) -> Result<()> {
    if args.name.is_empty()
        || !args
            .name
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
    {
        bail!(
            "board key `{}` must be kebab-case ([a-z0-9-]). It becomes a \
             directory name, a cargo feature and a cmake value, and a key that \
             is legal in one of those and not the others fails late.",
            args.name
        );
    }
    if args.platform == "zephyr" && args.west_board.is_none() {
        bail!(
            "--platform zephyr needs --west-board <board>/<soc>/<variant>.\n  \
             It is the one irreducible fact about a Zephyr board: the per-board \
             conf and overlay filenames derive from it, and so does the \
             Rust-support Kconfig symbol."
        );
    }
    if args.platform != "zephyr" && args.west_board.is_some() {
        bail!("--west-board is only meaningful with --platform zephyr");
    }

    let root = match &args.workspace {
        Some(p) => p.clone(),
        None => crate::cmd::board::find_workspace_root()?,
    };
    let dir = match &args.out_of_tree {
        Some(base) => base.join(format!("nros-board-{}", args.name)),
        None => root
            .join("packages")
            .join("boards")
            .join(format!("nros-board-{}", args.name)),
    };
    if dir.exists() {
        bail!("{} already exists", dir.display());
    }

    let descriptor = render_descriptor(&args);
    let package_xml = render_package_xml(&args);

    if args.dry_run {
        println!(
            "would write {}/nros-board.toml:\n{descriptor}",
            dir.display()
        );
        println!("would write {}/package.xml:\n{package_xml}", dir.display());
        print_next_steps(&args, &dir);
        return Ok(());
    }

    std::fs::create_dir_all(&dir).wrap_err_with(|| format!("create {}", dir.display()))?;
    std::fs::write(dir.join("nros-board.toml"), &descriptor)?;
    std::fs::write(dir.join("package.xml"), &package_xml)?;
    if args.platform == "zephyr" {
        // An empty `prj.conf` is written on purpose: the projection picks it up
        // present-if-exists, so an absent one is indistinguishable from a board
        // whose author has not got to it yet.
        std::fs::write(
            dir.join("prj.conf"),
            "# Base Kconfig for this board, layered ahead of the consumer's own\n\
             # prj.conf by `nano_ros_use_board()`.\n",
        )?;
    }

    if args.out_of_tree.is_none() {
        append_registry_row(&root, &args)?;
    }

    println!("wrote {}", dir.display());
    print_next_steps(&args, &dir);
    Ok(())
}

fn render_descriptor(args: &NewArgs) -> String {
    let mut out = format!(
        "# Board descriptor for `{name}`.\n\
         #\n\
         # A board is a PACKAGE (RFC-0064 R5 D1): this file says what the board\n\
         # lowers to, and the `package.xml` beside it announces that the board\n\
         # exists. Both are required — a descriptor with no announcement is\n\
         # refused, because it would resolve here and be invisible to every\n\
         # consumer that reaches providers through `provider_scan`.\n\
         \n\
         [[board]]\n\
         names = [\"{name}\"]\n\
         platform = \"{platform}\"\n\
         toolchain = \"stable\"\n\
         platform_feature = \"platform-{platform}\"\n",
        name = args.name,
        platform = args.platform,
    );
    out.push_str(
        "link_kind = \"none\"\n\
         # `hosted-main` when the host OS owns `main`, `zephyr-staticlib` on\n\
         # Zephyr, `board-run` when the board crate's `run()` is the entry.\n\
         # NOT derivable from `platform`: freertos carries both hosted and bare\n\
         # boards.\n",
    );
    out.push_str(&format!(
        "entry_kind = \"{}\"\n",
        if args.platform == "zephyr" {
            "zephyr-staticlib"
        } else if args.platform == "posix" {
            "hosted-main"
        } else {
            "board-run"
        }
    ));
    out.push_str(
        "# An EMPTY list is a statement, not an omission: a deploy naming a\n\
         # netstack this board does not support is refused (phase-351 W4).\n\
         supported_netstacks = []\n",
    );
    if let Some(wb) = &args.west_board {
        out.push_str(&format!(
            "\n[board.zephyr]\n\
             west_board = \"{wb}\"\n\
             # Derived, so absent on purpose: prj.conf and\n\
             # `boards/<west_board with / and @ as _>.{{conf,overlay}}`,\n\
             # present-if-exists. State `runner` only if it differs from what\n\
             # Zephyr's own board definition selects.\n"
        ));
    }
    out.push_str(
        "\n[board.capabilities]\n\
         heap = true\n\
         atomics = true\n\
         threads = true\n",
    );
    out
}

fn render_package_xml(args: &NewArgs) -> String {
    format!(
        "<?xml version=\"1.0\"?>\n\
         <package format=\"3\">\n  \
         <name>{pkg}</name>\n  \
         <version>0.0.0</version>\n  \
         <description>\n    \
         Board support for `{name}` ({platform}).\n  \
         </description>\n  \
         <maintainer email=\"dev@example.com\">Developer</maintainer>\n  \
         <license>Apache-2.0</license>\n\
         \n  \
         <export>\n    \
         <build_type>{build_type}</build_type>\n    \
         <!--\n      \
         Provision, NOT consumption: this package IS a board. Must equal the\n      \
         descriptor's `names`, canonical first — `check-provider-announcements`\n      \
         compares them, because two spellings of one fact drift.\n    \
         -->\n    \
         <nano_ros_provides kind=\"board\" name=\"{name}\"/>\n  \
         </export>\n\
         </package>\n",
        pkg = package_name(&args.name),
        name = args.name,
        platform = args.platform,
        build_type = if args.platform == "zephyr" {
            "nros_cmake"
        } else {
            "nros_cargo"
        },
    )
}

/// Append the `board-support.toml` row, at `tier = "scaffold"`.
///
/// Scaffold is the honest starting tier and `check-board-tiers` demands nothing
/// of it. A tier with a promise needs a named maintainer (phase-375 W1: at
/// least 1 for tier 3, 2 for tier 2, 3 for tier 1), and a scaffold that claimed
/// one would be
/// exactly the unearned promise that gate exists to refuse.
fn append_registry_row(root: &Path, args: &NewArgs) -> Result<()> {
    let path = root.join("packages/boards/board-support.toml");
    let mut body =
        std::fs::read_to_string(&path).wrap_err_with(|| format!("read {}", path.display()))?;
    let row = format!(
        "\n[[board]]\ncrate = \"nros-board-{}\"\ntier = \"scaffold\"\nmaintainers = []\n",
        args.name
    );
    // Before the `# ===== infra` section, which is not boards.
    match body.find("# ===== infra") {
        Some(i) => body.insert_str(i, row.trim_start_matches('\n')),
        None => body.push_str(&row),
    }
    std::fs::write(&path, body)?;
    Ok(())
}

fn print_next_steps(args: &NewArgs, dir: &Path) {
    println!("\nnext:");
    if let Some(base) = &args.out_of_tree {
        println!(
            "  export NROS_EXTRA_BOARD_PATH={}   # how this board is reached",
            base.display()
        );
    } else {
        println!("  packages/boards/board-support.toml  # row added at tier = \"scaffold\"");
    }
    if args.platform == "zephyr" {
        let flat: String = args
            .west_board
            .as_deref()
            .unwrap_or("")
            .chars()
            .map(|c| if matches!(c, '/' | '@' | '.') { '_' } else { c })
            .collect();
        println!(
            "  {}/boards/{flat}.conf      # optional, picked up if present",
            dir.display()
        );
        println!(
            "  {}/boards/{flat}.overlay   # optional, picked up if present",
            dir.display()
        );
        println!(
            "  nano_ros_use_board({})  # in the consumer's CMakeLists",
            args.name
        );
    }
    println!("  just check fast   # the scaffold's acceptance: green before the first commit");
}
