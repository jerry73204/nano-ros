//! `nros sdk-front` — phase-431 W3: point `$NROS_HOME/bin/<name>` at the NEWEST
//! installed version of a tool.
//!
//! `nros setup --tool <name>` already does this as part of an install, so this
//! verb exists for the one caller that cannot go through `nros setup`:
//! `scripts/bootstrap.sh` (W4), which downloads and unpacks the CLI itself —
//! there is no `nros` to run `nros setup --tool nros` with until it has. Rather
//! than let bootstrap re-implement "and then link the newest one" in shell,
//! it unpacks into the store and asks the binary it just placed.
//!
//! One implementation, two callers. A second spelling of this rule in shell is
//! how the two would come to disagree about which version `nros` means.
//!
//! It is deliberately NOT a general "put this on PATH" verb: what gets fronted
//! is `[tool.<name>] front = [...]` in the index, so the decision stays data.

use std::path::PathBuf;

use clap::Parser;
use eyre::{Result, bail};

use crate::orchestration::{sdk_index::SdkIndex, sdk_store};

#[derive(Debug, Parser)]
pub struct Args {
    /// Tool name, as spelled by `[tool.<name>]` in the SDK index.
    pub tool: String,

    /// Path to the SDK index.
    #[arg(long, default_value = "nros-sdk-index.toml")]
    pub index: PathBuf,

    /// Prefix-relative paths to front, instead of reading the index's `front`.
    ///
    /// For `scripts/install.sh`, which has a binary and no index: a user
    /// installing `nros` outside a checkout has no `nros-sdk-index.toml` until
    /// this very install finishes. The alternative was for the installer to
    /// write the symlink itself, which would be a SECOND answer to "which
    /// version does `nros` mean" — living in shell, in the one place that
    /// cannot see the store accumulate. This keeps `front_newest` the only
    /// answer and lets the caller supply the one input it is missing.
    #[arg(long = "front", value_name = "REL")]
    pub front: Vec<String>,
}

pub fn run(args: Args) -> Result<()> {
    let front = if args.front.is_empty() {
        let index = SdkIndex::load(&args.index)?;
        let Some(tool) = index.tool.get(&args.tool) else {
            let known: Vec<&str> = index.tool.keys().map(String::as_str).collect();
            bail!(
                "no `[tool.{}]` in {} — the index pins: {}",
                args.tool,
                args.index.display(),
                known.join(", ")
            );
        };
        tool.front.clone()
    } else {
        args.front.clone()
    };

    if front.is_empty() {
        bail!(
            "`[tool.{}]` declares no `front`, so nothing about it belongs in {}. \
             Resolve it with `nros sdk-path {}` instead — the store is on PATH \
             only for the names in scripts/sdk-path-tools.txt, which is for \
             binaries something we do not control invokes by bare name.",
            args.tool,
            sdk_store::front_dir().display(),
            args.tool
        );
    }

    let root = sdk_store::store_root();
    let linked = sdk_store::front_newest(&root, &args.tool, &front)?;
    if linked.is_empty() {
        // Not an error at the library layer (an install may simply not have
        // happened yet), but here the user ASKED, so say what is missing.
        bail!(
            "no installed version of {} under {} — nothing to point at.\n\
             Provision it:  nros setup --tool {}",
            args.tool,
            root.join(&args.tool).display(),
            args.tool
        );
    }
    for link in linked {
        println!("{}", link.display());
    }
    Ok(())
}
