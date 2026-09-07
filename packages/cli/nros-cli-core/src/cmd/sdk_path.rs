//! `nros sdk-path` — phase-365 W2: print the CONSTRUCTED install path of a
//! provisioned tool.
//!
//! nano-ros decides where a provisioned tool goes: `nros setup` writes
//! `<store>/<tool>/<version>` because `nros-sdk-index.toml` named that version.
//! The layout is our own OUTPUT, so a consumer constructs the path from the same
//! two inputs — it does not search the store for it.
//!
//! This is the bridge for the three non-Rust mechanisms (cmake, `just`, shell),
//! exactly as `nros model-path` is the bridge for the SystemModel rule: the
//! derivation lives ONCE, in `sdk_store::tool_dir`, and everyone else asks.
//!
//! Why asking beats searching, measured (issue 0625): a search can return
//! something we did not install (the legacy unversioned `corrosion/{lib,share}`
//! prefix), something a DIFFERENT project installed — the store is shared while
//! the pin is per-project — or nothing. On 2026-08-16, in a tree pinning
//! `corrosion 0.6.1-nros1`, one `lane=all` configure produced 155 resolutions of
//! 0.5.1 against 28 of 0.6.1, with the search's own ordering verified correct.
//!
//! `--require` is the difference between "where would it be" and "it is there":
//! a miss FAILS with the provisioning command instead of falling back to another
//! version, because a silent substitution is the bug this phase removes.

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

    /// Fail unless the directory exists — for a consumer about to use it.
    #[arg(long)]
    pub require: bool,
}

pub fn run(args: Args) -> Result<()> {
    let index = SdkIndex::load(&crate::cmd::setup::resolve_index(&args.index))?;

    let Some(dir) = sdk_store::tool_dir(&index, &args.tool) else {
        // Name what IS pinned: a typo and an unprovisioned tool look identical
        // otherwise, and the caller is usually a cmake file that cannot explore.
        let known: Vec<&str> = index.tool.keys().map(String::as_str).collect();
        bail!(
            "no `[tool.{}]` in {} — the index pins: {}",
            args.tool,
            args.index.display(),
            known.join(", ")
        );
    };

    // Issue 0628 — answer with the layout that is actually PRESENT.
    //
    // `tool_dir` is where an installer WRITES; a consumer needs somewhere it can
    // read. Two provisioning paths wrote two shapes historically, so a host
    // provisioned before `install-corrosion` learned to ask this command has the
    // pinned version under the flat prefix. Constructing only the versioned one
    // made that host look unprovisioned, and cmake fetched from the network
    // while printing advice to install what was already installed.
    //
    // Still construction: two candidates from the same inputs, no enumeration.
    let usable = sdk_store::tool_dir_usable(&index, &args.tool);

    if args.require && usable.is_none() {
        let version = index
            .tool
            .get(&args.tool)
            .map(|t| t.version.as_str())
            .unwrap_or("?");
        // Name every path tried. "Not installed" and "installed where I did not
        // look" printed the same line before, and they need different fixes.
        let tried: Vec<String> = sdk_store::tool_dir_candidates(&index, &args.tool)
            .iter()
            .map(|p| format!("  {}", p.display()))
            .collect();
        bail!(
            "{} is pinned to {} but no install was found. Looked at:\n{}\n\
             Provision it:  nros setup --tool {}\n\
             (NOT substituting another version on purpose: the store is shared \
             between projects while the pin is per-project, so a substitution \
             silently gives this build another project's toolchain — issue 0625.)",
            args.tool,
            version,
            tried.join("\n"),
            args.tool
        );
    }

    // Without `--require` this still prints the CANONICAL path when nothing is
    // installed: the caller asked where the tool goes, and an empty answer is
    // harder to act on than a path that does not exist yet.
    println!("{}", usable.unwrap_or(dir).display());
    Ok(())
}
