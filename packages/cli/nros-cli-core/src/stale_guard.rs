//! Issue 0363 B — the in-tree CLI refuses to run stale.
//!
//! A staleness guard already existed and was good: `scripts/build/cargo.sh`
//! walks `git ls-files packages/cli` for any source newer than the binary and
//! refuses. But it lives in the shell function `nros_cli_bin()`, so it only
//! covers callers that go through `just` — while `activate.sh` puts the raw
//! binary on `PATH`, so a bare `nros sync` never reaches it.
//!
//! That is the whole defect. `nros sync` is the command CLAUDE.md and
//! `nros-patch.toml`'s own header tell you to run to recover, and it was
//! precisely the invocation the protection did not cover. Same shape as issue
//! 0354 (a validator whose callers exclude the case it exists for), with a
//! worse payload: phase-321 moved packages, the stale binary's hardcoded
//! crate→path table still named the old locations, and the emitted
//! `[patch.crates-io]` table DROPPED `nros-zephyr-build` without a word. A
//! dropped patch entry does not fail — the dependency quietly resolves from
//! crates.io instead of the checkout.
//!
//! So the check moves to where it cannot be bypassed by invocation style: the
//! binary checks itself.

use std::path::{Path, PathBuf};

use crate::source_stamp;

/// Stamp of the sources this binary was compiled from, embedded by `build.rs`.
///
/// `"unknown"` when the build happened outside a git checkout (tarball,
/// vendored copy). That is a skip, not a failure: without the tree there is
/// nothing to be stale RELATIVE TO, and guessing would break every packaged
/// install.
const BUILT_STAMP: &str = env!("NROS_CLI_SOURCE_STAMP");

/// Commands that consume the crate→path table or emit generated artifacts.
///
/// Deliberately NOT every command. `nros --version` / `completions` / `doctor`
/// must keep working on a stale binary — `doctor` especially, since diagnosing
/// a broken checkout is exactly when you have one.
fn command_is_guarded(name: &str) -> bool {
    matches!(
        name,
        "sync" | "plan" | "ws" | "codegen" | "codegen-system" | "generate-rust" | "setup"
    )
}

/// Refuse to run when this binary is older than the sources it was built from,
/// or when it is a FOREIGN binary being run against a checkout.
///
/// Two questions, and phase-431 W1 added the second because shipping a prebuilt
/// `nros` inverts what the first one's exemption means.
///
/// 1. **Is this binary stale relative to the checkout it lives in?** Keyed on
///    the binary's own path. Unchanged.
/// 2. **Is a binary from somewhere else being run against a checkout?** Keyed on
///    the WORKSPACE. Until there was a release to install, the only non-checkout
///    `nros` was a deliberate experiment, so exempting it was right. Once
///    `~/.nros/bin/nros` exists on developer machines, that exemption becomes a
///    hole: a PATH accident silently disables the freshness check inside a
///    checkout, and the binary emits with whatever ITS emitters were.
///
/// RFC-0090's codegen version does not cover case 2. It catches an INCOMPATIBLE
/// emitter; a release at the same version whose emitters have merely MOVED is a
/// freshness question, and the fingerprint that answers it is consulted by the
/// fixture stamps, not here.
///
/// An installed copy run against a user's own project is still exempt, which is
/// the case the original exemption existed to protect.
pub fn refuse_if_stale(command_name: &str) -> Result<(), String> {
    if std::env::var_os("NROS_SKIP_STALE_CHECK").is_some() {
        return Ok(());
    }
    if !command_is_guarded(command_name) {
        return Ok(());
    }
    let Ok(exe) = std::env::current_exe() else {
        return Ok(());
    };
    // The workspace is the cwd. Passed in rather than read inside, so the
    // decision is a pure function of two paths and its tests need no
    // `set_current_dir` — a process-global that leaks between parallel tests
    // (issue 1101 is that hazard, one crate over).
    if let Ok(cwd) = std::env::current_dir() {
        refuse_if_foreign_to_workspace(&exe, &cwd)?;
    }
    let Some(root) = checkout_root_of(&exe) else {
        return Ok(());
    };
    if BUILT_STAMP == "unknown" {
        return Ok(());
    }
    // No stamp computable now (git absent / not a checkout) — skip rather than
    // guess. Same reasoning as `BUILT_STAMP == "unknown"`.
    let Some(current) = source_stamp::source_stamp(&root) else {
        return Ok(());
    };
    if current == BUILT_STAMP {
        return Ok(());
    }
    // Name the files actually being edited. The mtime predicate could only
    // report whichever tracked file sorted first, which was frequently not the
    // one the developer had touched.
    let dirty = source_stamp::modified_cli_files(&root);
    let detail = if dirty.is_empty() {
        "  (no uncommitted CLI edits — the checkout moved, e.g. a branch switch)".to_string()
    } else {
        let mut s = String::from("  uncommitted CLI edits:\n");
        for f in dirty.iter().take(3) {
            s.push_str(&format!("    {f}\n"));
        }
        if dirty.len() > 3 {
            s.push_str(&format!("    … and {} more\n", dirty.len() - 3));
        }
        s.trim_end().to_string()
    };
    Err(format!(
        "in-tree nros CLI is STALE — its sources changed since it was built\n\
         (source stamp {BUILT_STAMP} != {current}) for '{}',\n\
         whose checkout is '{}'.\n\
         {detail}\n\
         A stale CLI silently breaks workspace planning + codegen: its hardcoded\n\
         crate→path table can name locations that no longer exist, and a dropped\n\
         [patch.crates-io] entry resolves from crates.io instead of this checkout\n\
         WITHOUT failing (issues 0363, 0197).\n\
         Rebuild it (not auto-done — compiling at build/test time is forbidden):\n\
         \x20   ./scripts/bootstrap.sh      (contributors: just setup-cli)\n\
         Override for a deliberate experiment: NROS_SKIP_STALE_CHECK=1",
        exe.display(),
        // Issue 1133 — name the CHECKOUT, not only the binary. With several
        // worktrees in play the first question is "is this even my tree?", and
        // the exe path answers it only if you already know where each tree's
        // target dir lives. A `PATH`-resolved binary from ANOTHER checkout is
        // the common cause, and no number of rebuilds here can refresh it.
        root.display()
    ))
}

/// phase-431 W1 — refuse a binary that does not belong to the checkout it is
/// being run against.
///
/// The workspace decides, not the binary: if the current directory sits inside a
/// nano-ros checkout AND that checkout can build a CLI, then the running `nros`
/// must be that checkout's own build. Anything else is a shadow — a released
/// binary on `PATH`, another checkout's build, a stray copy — and it emits with
/// emitters nobody in this tree can see.
///
/// Deliberately silent in three cases, each of which would otherwise break a
/// legitimate flow:
///
/// * the cwd is not in a checkout — a user's own project, which is exactly what
///   a released binary is FOR;
/// * the checkout carries no `packages/cli` sources — nothing to be foreign to;
/// * `current_exe` or the cwd cannot be resolved — skip rather than guess, the
///   same rule the stamp comparison already follows.
fn refuse_if_foreign_to_workspace(exe: &Path, workspace: &Path) -> Result<(), String> {
    let Some(ws_root) = crate::abi_guard::find_monorepo_root(workspace) else {
        return Ok(());
    };
    // A checkout with no CLI sources cannot expect one to be built from it.
    if !ws_root.join("packages/cli/Cargo.toml").is_file() {
        return Ok(());
    }
    let exe_real = exe.canonicalize().unwrap_or_else(|_| exe.to_path_buf());
    let expected_dir = ws_root.join("packages/cli/target");
    let expected_real = expected_dir
        .canonicalize()
        .unwrap_or_else(|_| expected_dir.clone());
    if exe_real.starts_with(&expected_real) {
        return Ok(());
    }
    Err(format!(
        "this `nros` does not belong to the checkout it is being run against.\n\
         \x20   running: {}\n\
         \x20   checkout: {}\n\
         A binary from outside this tree emits with ITS OWN codegen, which may\n\
         differ from this checkout's while carrying the same codegen version —\n\
         the version catches an incompatible emitter, not one that merely moved\n\
         (RFC-0090, phase-431 W1). Build and use this checkout's own CLI:\n\
         \x20   ./scripts/bootstrap.sh      (contributors: just setup-cli)\n\
         \x20   source ./activate.sh\n\
         Override for a deliberate experiment: NROS_SKIP_STALE_CHECK=1",
        exe_real.display(),
        ws_root.display(),
    ))
}

/// `<root>` when `exe` is `<root>/packages/cli/target/**/nros`, else `None`.
fn checkout_root_of(exe: &Path) -> Option<PathBuf> {
    let mut dir = exe.parent()?;
    // walk up looking for the `packages/cli/target` shape
    while let Some(parent) = dir.parent() {
        if dir.file_name().is_some_and(|n| n == "target")
            && parent.file_name().is_some_and(|n| n == "cli")
            && parent
                .parent()?
                .file_name()
                .is_some_and(|n| n == "packages")
        {
            return parent.parent()?.parent().map(Path::to_path_buf);
        }
        dir = parent;
    }
    None
}

/// Report freshness without refusing anything — backs `nros source-stamp`.
///
/// Returns `(built, current)`. Equal means fresh; `None` means the question
/// does not apply here (no stamp, or not a per-checkout binary).
pub fn stamp_pair() -> Option<(String, String)> {
    if BUILT_STAMP == "unknown" {
        return None;
    }
    let exe = std::env::current_exe().ok()?;
    let root = checkout_root_of(&exe)?;
    let current = source_stamp::source_stamp(&root)?;
    Some((BUILT_STAMP.to_string(), current))
}

#[cfg(test)]
mod foreign_binary_tests {
    use super::*;
    use std::fs;

    /// A checkout is "a tree with `packages/core/nros-core/Cargo.toml`" — the
    /// marker `abi_guard::find_monorepo_root` already uses — plus CLI sources.
    fn fake_checkout(root: &Path) {
        fs::create_dir_all(root.join("packages/core/nros-core")).unwrap();
        fs::write(root.join("packages/core/nros-core/Cargo.toml"), "").unwrap();
        fs::create_dir_all(root.join("packages/cli/target/release")).unwrap();
        fs::write(root.join("packages/cli/Cargo.toml"), "").unwrap();
    }

    fn stray(tmp: &Path) -> PathBuf {
        let p = tmp.join("elsewhere/nros");
        fs::create_dir_all(p.parent().unwrap()).unwrap();
        fs::write(&p, "").unwrap();
        p
    }

    /// The whole point of phase-431 W1: a binary from outside the tree is
    /// refused when it is run AGAINST that tree.
    #[test]
    fn a_foreign_binary_is_refused_against_a_checkout() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("checkout");
        fake_checkout(&root);
        let err = refuse_if_foreign_to_workspace(&stray(tmp.path()), &root)
            .expect_err("a binary outside the checkout must be refused");
        assert!(
            err.contains("does not belong to the checkout"),
            "the message must say WHICH problem this is: {err}"
        );
        // phase-368 / `check-emitter-just-spelling`: a user-reachable message
        // that names a `just` recipe must name the USER spelling beside it.
        // This message becomes MORE user-reachable once a binary ships — a
        // user who wanders into a checkout is exactly who sees it.
        assert!(
            err.contains("./scripts/bootstrap.sh") && err.contains("just setup-cli"),
            "the remedy must carry both spellings, user first: {err}"
        );
    }

    /// The negative control, and the case the original exemption existed to
    /// protect: a released binary building a USER's project must not be
    /// refused. Without this, shipping would break every out-of-tree consumer.
    #[test]
    fn a_foreign_binary_is_fine_outside_any_checkout() {
        let tmp = tempfile::tempdir().unwrap();
        let project = tmp.path().join("user-project");
        fs::create_dir_all(&project).unwrap();
        assert!(refuse_if_foreign_to_workspace(&stray(tmp.path()), &project).is_ok());
    }

    /// The checkout's own build is what the tree wants, so it passes this check
    /// and goes on to the staleness comparison.
    #[test]
    fn the_checkouts_own_binary_passes() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("checkout");
        fake_checkout(&root);
        let own = root.join("packages/cli/target/release/nros");
        fs::write(&own, "").unwrap();
        assert!(refuse_if_foreign_to_workspace(&own, &root).is_ok());
    }

    /// A tree with the runtime marker but no CLI sources cannot expect a CLI to
    /// be built from it, so there is nothing to be foreign to. Without this arm
    /// a consumer vendoring `packages/core` would be refused.
    #[test]
    fn a_tree_with_no_cli_sources_is_not_a_checkout_for_this_purpose() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("vendored");
        fs::create_dir_all(root.join("packages/core/nros-core")).unwrap();
        fs::write(root.join("packages/core/nros-core/Cargo.toml"), "").unwrap();
        assert!(refuse_if_foreign_to_workspace(&stray(tmp.path()), &root).is_ok());
    }

    /// A subdirectory of the checkout is still the checkout — the walk-up is
    /// what makes this usable from anywhere in the tree.
    #[test]
    fn a_subdirectory_of_the_checkout_still_counts() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("checkout");
        fake_checkout(&root);
        let deep = root.join("examples/native/rust/talker");
        fs::create_dir_all(&deep).unwrap();
        assert!(refuse_if_foreign_to_workspace(&stray(tmp.path()), &deep).is_err());
    }
}
