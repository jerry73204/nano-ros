//! phase-330 W3.b (RFC-0063) — where a consumer looks for the SystemModel.
//!
//! The model is becoming a BUILD ARTIFACT. Until W4 deletes the committed
//! copies, both locations exist, so every consumer needs the same search order
//! — and there are three consumers that each derived the path independently:
//!
//!   * `nros-macros` — `pkg_index.resolve_pkg(bringup).join("config/system_model.yaml")`
//!   * `nros-build`  — `bringup_dir.join("config/system_model.yaml")`
//!   * `cmake/NanoRosEntry.cmake` — a `MODEL <path>` argument with the same default
//!
//! Three copies of a path policy is how the two `TierRtosSpec` mirrors drifted
//! earlier in this phase, so the policy lives here once and the consumers call
//! it.
//!
//! # The order, and why
//!
//! 1. `$NROS_MODEL_DIR` — a SHARED build root. RFC-0065's builder sets this
//!    when several entry crates use one bringup and should share a single
//!    regeneration. Highest priority because it is an explicit instruction.
//! 2. `$OUT_DIR/nros/` — the per-crate cargo build output. This is the W3.a
//!    decision's default for the cargo path: it exists for a standalone
//!    copy-out example exactly as it does inside a workspace, which is what
//!    lets W3.c need no special case.
//! 3. `<bringup>/<model_rel>` — the committed source copy. The legacy location,
//!    still authoritative until W4.
//!
//! The fallback is what makes this landable on its own: with nothing
//! generating into (1) or (2), every consumer resolves exactly as before.

use std::path::{Path, PathBuf};

/// Candidate model paths for `bringup_dir`/`model_rel`, most-preferred first.
///
/// `model_rel` is the bringup-relative path (`config/system_model.yaml`, or a
/// variant like `config/talker_model.yaml`). Build-output candidates use only
/// its FILE NAME: a build directory is already scoped to one build, so
/// re-nesting `config/` there would add a level that means nothing.
pub fn model_search_paths(bringup_dir: &Path, model_rel: &str) -> Vec<PathBuf> {
    let name = Path::new(model_rel)
        .file_name()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("system_model.yaml"));
    // The bringup's own directory name namespaces the build-output copies. A
    // flat layout collides whenever a workspace has two bringups (both emit
    // `system_model.yaml`), and the loser vanishes silently.
    let bringup = bringup_dir.file_name().map(PathBuf::from);
    let mut out = Vec::new();
    if let Some(dir) = std::env::var_os("NROS_MODEL_DIR") {
        let dir = Path::new(&dir);
        if let Some(b) = &bringup {
            out.push(dir.join(b).join(&name));
        }
        out.push(dir.join(&name));
    }
    if let Some(dir) = std::env::var_os("OUT_DIR") {
        let dir = Path::new(&dir).join("nros");
        // Keyed on the bringup's PATH, not its name — issue 0825. `$OUT_DIR`
        // is per-crate for a build script but SHARED by every test binary of
        // a package, and `demo_bringup` is the name nearly every fixture uses,
        // so a bare name makes one test's model shadow another's. Same hash
        // the cache fallback already uses, for the same reason.
        out.push(dir.join(build_scoped_dir(bringup_dir)).join(&name));
        out.push(dir.join(&name));
    }
    // phase-330 W4/W7 — the WORKSPACE build root, where `nros sync` writes by
    // default once the committed copies are gone: `<ws>/build/nros/models/
    // <bringup>/<name>`. Derived from the conventional `<ws>/src/<bringup>`
    // layout (bringup_dir/../../build) so NO env wiring is needed for the
    // in-tree flows (west entry crates have neither OUT_DIR nor
    // NROS_MODEL_DIR at macro expansion). A bringup outside that layout
    // simply contributes no candidate here.
    if let (Some(b), Some(ws_root)) = (&bringup, bringup_dir.parent().and_then(|src| src.parent()))
    {
        out.push(
            ws_root
                .join("build")
                .join("nros")
                .join("models")
                .join(b)
                .join(&name),
        );
    }
    // Standalone copy-out / single-pkg self-bringups (W3.c): the bringup dir
    // IS the workspace root (`nros sync` run inside it writes
    // `<bringup>/build/nros/models/<bringup>/…`), so the ws-layout rung above
    // misses by two levels. Same namespacing, rooted at the bringup itself.
    if let Some(b) = &bringup {
        out.push(
            bringup_dir
                .join("build")
                .join("nros")
                .join("models")
                .join(b)
                .join(&name),
        );
    }
    out.push(bringup_dir.join(model_rel));
    out
}

/// The model path a consumer should read.
///
/// Returns the first candidate that exists. When none does, returns the LAST
/// candidate — the legacy committed location — purely so the error names a
/// stable path.
///
/// phase-336 W7: that is a naming convenience, NOT advice. Since W4 the model
/// is a build artifact and `check-no-tracked-models` rejects a committed one,
/// so a caller reporting "not found" must tell the user to GENERATE it
/// (`nros sync`), never to create the path this returns. The three CLI messages
/// that got that backwards are fixed; keep new ones consistent.
pub fn resolve_model_path(bringup_dir: &Path, model_rel: &str) -> PathBuf {
    let candidates = model_search_paths(bringup_dir, model_rel);
    for c in &candidates {
        if c.is_file() {
            return c.clone();
        }
    }
    candidates
        .last()
        .cloned()
        .unwrap_or_else(|| bringup_dir.join(model_rel))
}

/// phase-330 W7 — map the INPUT coordinates of a system to the model's
/// bringup-relative path.
///
/// The user-facing input is `(bringup, launch file, launch args)`; the model
/// is a build artifact nobody names. This is the ONE mapping from the former
/// to the latter, shared by `nros::main!(launch = …)`, `nros-build` and (via
/// the CLI) `nano_ros_entry(LAUNCH …)` — the same one-home rule as
/// [`model_search_paths`] above.
///
/// Rules (phase-330 W4.0 derive-plus-declare):
/// 1. `args` non-empty → a `[[model]]` declaration in `system.toml` matching
///    `(launch, args)` MUST exist (declarations are the SSoT for which
///    bindings exist); its `out` names the model.
/// 2. A declaration matching `(launch, no args)` also wins when present.
/// 3. No declaration: the DEFAULT launch (explicit `[system] default_launch`,
///    or the conventional `system.launch.xml`) maps to
///    `config/system_model.yaml`; any other launch maps to
///    `config/<stem>_model.yaml` (stem = file name minus `.launch.xml`).
///
/// Returns the bringup-relative path (`config/<name>`), which
/// [`resolve_model_path`] then locates (build dir first, committed fallback
/// until W4.a deletes it). Errors are strings naming what the USER must fix.
pub fn launch_to_model_rel(
    bringup_dir: &Path,
    launch_file: Option<&str>,
    args: &[(String, String)],
) -> Result<String, String> {
    let system_toml = bringup_dir.join("system.toml");
    let doc: Option<toml::Table> = std::fs::read_to_string(&system_toml)
        .ok()
        .and_then(|raw| raw.parse::<toml::Table>().ok());

    let default_launch = doc
        .as_ref()
        .and_then(|d| d.get("system"))
        .and_then(|s| s.get("default_launch"))
        .and_then(|v| v.as_str())
        .unwrap_or("system.launch.xml")
        .to_string();
    let launch = launch_file.unwrap_or(&default_launch);

    // [[model]] declarations for this launch file.
    let decls: Vec<(Vec<(String, String)>, String)> = doc
        .as_ref()
        .and_then(|d| d.get("model"))
        .and_then(|m| m.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|e| {
                    let decl_launch = e.get("launch")?.as_str()?;
                    if decl_launch != launch {
                        return None;
                    }
                    let out = e.get("out")?.as_str()?.to_string();
                    let mut decl_args: Vec<(String, String)> = e
                        .get("args")
                        .and_then(|a| a.as_table())
                        .map(|t| {
                            t.iter()
                                .filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string())))
                                .collect()
                        })
                        .unwrap_or_default();
                    decl_args.sort();
                    Some((decl_args, out))
                })
                .collect()
        })
        .unwrap_or_default();

    let mut want: Vec<(String, String)> = args.to_vec();
    want.sort();
    if let Some((_, out)) = decls.iter().find(|(a, _)| *a == want) {
        let out_rel = if out.contains('/') {
            out.clone()
        } else {
            format!("config/{out}")
        };
        return Ok(out_rel);
    }
    if !want.is_empty() {
        let known: Vec<String> = decls
            .iter()
            .map(|(a, out)| format!("{out} (args {a:?})"))
            .collect();
        return Err(format!(
            "no `[[model]]` declaration in `{}` matches launch `{launch}` with args {want:?} — \
             binding variants must be DECLARED (phase-330 W4.0). Declared for this launch: [{}]",
            system_toml.display(),
            known.join(", "),
        ));
    }

    // Derive rule.
    if launch == default_launch {
        return Ok("config/system_model.yaml".to_string());
    }
    let stem = launch
        .strip_suffix(".launch.xml")
        .or_else(|| launch.strip_suffix(".xml"))
        .unwrap_or(launch);
    let stem = stem.rsplit('/').next().unwrap_or(stem);
    Ok(format!("config/{stem}_model.yaml"))
}

/// The bringup's default launch file (`[system] default_launch`, or the
/// conventional `system.launch.xml`).
pub fn default_launch(bringup_dir: &Path) -> String {
    std::fs::read_to_string(bringup_dir.join("system.toml"))
        .ok()
        .and_then(|raw| raw.parse::<toml::Table>().ok())
        .as_ref()
        .and_then(|d| d.get("system"))
        .and_then(|s| s.get("default_launch"))
        .and_then(|v| v.as_str())
        .unwrap_or("system.launch.xml")
        .to_string()
}

/// The INVERSE of [`launch_to_model_rel`]: given a model's bringup-relative
/// path, the `(launch file, args)` that produce it.
///
/// A consumer addressed by model path (`nros::main!(model = "…")`) still has to
/// be able to RESOLVE that model from inputs when the artifact is absent, and
/// to do that it needs the inputs back. Declarations are checked first — they
/// are the SSoT for arg-bound variants — then the derive rule is reversed.
///
/// `None` means the path does not correspond to any launch this bringup has,
/// which is a user error worth reporting with the model name in it.
pub fn model_rel_to_inputs(
    bringup_dir: &Path,
    model_rel: &str,
) -> Option<(String, Vec<(String, String)>)> {
    let name = Path::new(model_rel)
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or(model_rel);

    let doc: Option<toml::Table> = std::fs::read_to_string(bringup_dir.join("system.toml"))
        .ok()
        .and_then(|raw| raw.parse::<toml::Table>().ok());

    // 1. A `[[model]]` declaration whose `out` is this file.
    if let Some(arr) = doc
        .as_ref()
        .and_then(|d| d.get("model"))
        .and_then(|m| m.as_array())
    {
        for e in arr {
            let out = e.get("out").and_then(|v| v.as_str()).unwrap_or_default();
            let out_name = Path::new(out)
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or(out);
            if out_name != name {
                continue;
            }
            let launch = e.get("launch").and_then(|v| v.as_str())?.to_string();
            let args: Vec<(String, String)> = e
                .get("args")
                .and_then(|a| a.as_table())
                .map(|t| {
                    t.iter()
                        .filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string())))
                        .collect()
                })
                .unwrap_or_default();
            return Some((launch, args));
        }
    }

    // 2. Reverse the derive rule.
    if name == "system_model.yaml" {
        return Some((default_launch(bringup_dir), Vec::new()));
    }
    let stem = name.strip_suffix("_model.yaml")?;
    Some((format!("{stem}.launch.xml"), Vec::new()))
}

/// The `nros-launch-resolve` helper, by ABSOLUTE path.
///
/// Never `$PATH` (issue 0285): a stale `~/.nros/bin` copy silently shadows the
/// in-tree one and resolves with an older schema. The order is explicit
/// override, then this checkout, then the installed store.
pub fn launch_resolver_bin() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("NROS_LAUNCH_RESOLVE") {
        let p = PathBuf::from(p);
        if p.is_file() {
            return Some(p);
        }
    }
    if let Some(repo) = std::env::var_os("NROS_REPO_DIR") {
        let p = PathBuf::from(repo)
            .join("packages/cli/nros-launch-resolve/target/release/nros-launch-resolve");
        if p.is_file() {
            return Some(p);
        }
    }
    let home = std::env::var_os("NROS_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".nros")))?;
    let p = home.join("bin/nros-launch-resolve");
    p.is_file().then_some(p)
}

/// Where a consumer may WRITE a model it had to resolve itself.
///
/// `$NROS_MODEL_DIR` and `$OUT_DIR` are the build-provided homes. Neither is
/// guaranteed: a proc-macro only sees `OUT_DIR` when the crate has a build
/// script, and nano-ros entry crates deliberately have none — so a plain
/// `cargo build` of an entry has nowhere the build system chose. The last
/// resort is a per-user cache keyed by the bringup's absolute path, which is
/// stable across builds and shared between crates that name the same bringup.
fn model_write_dir(bringup_dir: &Path) -> PathBuf {
    let bringup_name = bringup_dir
        .file_name()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("bringup"));
    if let Some(dir) = std::env::var_os("NROS_MODEL_DIR") {
        return Path::new(&dir).join(&bringup_name);
    }
    if let Some(dir) = std::env::var_os("OUT_DIR") {
        // Issue 0825 — the same path-keyed name the SEARCH side uses. The two
        // must move together: a writer and a reader disagreeing about this
        // directory is the shadowing bug with the roles swapped.
        return Path::new(&dir)
            .join("nros")
            .join(build_scoped_dir(bringup_dir));
    }
    let base = std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|p| PathBuf::from(p).join(".cache")))
        .unwrap_or_else(std::env::temp_dir);
    base.join("nano-ros/models")
        .join(build_scoped_dir(bringup_dir))
}

/// Directory name that identifies a bringup inside a SHARED build output tree.
///
/// `<fnv1a(abs path)>-<dir name>`. The name alone is not an identifier: every
/// workspace in this repository calls its bringup `demo_bringup`, so a
/// name-keyed directory under `$OUT_DIR` — which cargo shares across every
/// test binary of a package — lets one test's model be read as another's
/// (issue 0825: a model with no `execution.tiers` silently erased tiers three
/// sibling tests had authored). The hash keeps the human-readable half for
/// anyone reading the directory, and makes collisions require an actual path
/// collision.
///
/// **Both the reader and the writer must call this.** They are the only two
/// places that spell the directory, and they are in this file for that reason.
fn build_scoped_dir(bringup_dir: &Path) -> String {
    let name = bringup_dir
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "bringup".to_string());
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bringup_dir.to_string_lossy().as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    format!("{h:016x}-{name}")
}

/// Locate the model for `(bringup, model_rel)`, RESOLVING it from the bringup's
/// inputs when no build has produced one.
///
/// phase-330 made the SystemModel a build artifact; this closes the gap that
/// left (issue 0414). A consumer that only ever LOOKED for the artifact fails
/// with "SystemModel not found" whenever it runs without a build system that
/// ran the resolver first — which is what a plain `cargo build` of an entry
/// crate is. The inputs are always present; the artifact is derived from them,
/// so derive it.
///
/// Returns the model path and the input files a caller should register as
/// build dependencies (`system.toml` + the launch file), so a change to either
/// re-runs the consumer — the property that made touching an input a no-op
/// before.
pub fn ensure_model(
    bringup_dir: &Path,
    model_rel: &str,
) -> Result<(PathBuf, Vec<PathBuf>), String> {
    let system_toml = bringup_dir.join("system.toml");
    let (launch_file, args) = model_rel_to_inputs(bringup_dir, model_rel).ok_or_else(|| {
        format!(
            "`{model_rel}` names no launch of bringup `{}` — expected \
             `config/system_model.yaml` (the default launch), `config/<stem>_model.yaml` \
             (for `<stem>.launch.xml`), or a `[[model]] out = …` declaration in {}",
            bringup_dir.display(),
            system_toml.display(),
        )
    })?;
    let launch_path = bringup_dir.join("launch").join(&launch_file);
    let inputs = vec![system_toml.clone(), launch_path.clone()];

    // A model a build already produced wins: cmake / nros-build resolve into
    // NROS_MODEL_DIR or OUT_DIR before invoking cargo, and re-resolving here
    // would duplicate that work — and could disagree with it.
    let found = resolve_model_path(bringup_dir, model_rel);
    if found.is_file() {
        // Issue 0495 — a BUILD-PRODUCED model is itself a rebuild dependency.
        //
        // This branch does not own the file: some other producer wrote it and
        // may rewrite it with different content without touching `system.toml`
        // or the launch XML (a `nros sync` on a newer CLI, a different
        // resolver, an expert `MODEL` override). A consumer that tracked only
        // the inputs then compiles against a model it cannot notice changing —
        // the museum-binary shape, and what made `rebuilds_on_model_touch`
        // fail: the macro READ this path and never registered it, while the
        // module's own contract is "every file the macro read".
        //
        // Deliberately NOT symmetric with the self-resolved branch below, which
        // must keep excluding its artifact — see the comment there.
        let mut inputs = inputs;
        inputs.push(found.clone());
        return Ok((found, inputs));
    }

    if !launch_path.is_file() {
        return Err(format!(
            "cannot resolve `{model_rel}`: its launch file `{}` does not exist",
            launch_path.display()
        ));
    }
    let resolver = launch_resolver_bin().ok_or_else(|| {
        "cannot resolve the SystemModel: `nros-launch-resolve` not found. Build it with \
         `./scripts/bootstrap.sh` (contributors: `just setup-launch-resolve`), or point \
         $NROS_LAUNCH_RESOLVE at one. (Never resolved \
         through $PATH — a stale copy there resolves with an older schema, issue 0285.)"
            .to_string()
    })?;

    let out_dir = model_write_dir(bringup_dir);
    std::fs::create_dir_all(&out_dir).map_err(|e| format!("create {}: {e}", out_dir.display()))?;
    let out = out_dir.join(
        Path::new(model_rel)
            .file_name()
            .unwrap_or_else(|| std::ffi::OsStr::new("system_model.yaml")),
    );

    // A model this consumer resolved EARLIER is reusable while it is newer than
    // every input. Without this check the macro re-resolves on every
    // invocation, rewrites the file, and the fresh mtime makes cargo consider
    // the entry dirty forever — an unconditional rebuild that looks like a
    // caching bug in cargo. (Observed exactly that: three consecutive
    // `cargo check`s each re-checked the entry with nothing edited.)
    if is_fresh(&out, &inputs) {
        return Ok((out, inputs));
    }

    let mut cmd = std::process::Command::new(&resolver);
    cmd.arg(&launch_path);
    for (k, v) in &args {
        cmd.arg(format!("{k}:={v}"));
    }
    cmd.arg("--bringup-root")
        .arg(bringup_dir)
        .arg("--system")
        .arg(&system_toml)
        .arg("-o")
        .arg(&out);
    let output = cmd
        .output()
        .map_err(|e| format!("spawn {}: {e}", resolver.display()))?;
    if !output.status.success() {
        return Err(format!(
            "nros-launch-resolve failed for `{}`:\n{}",
            launch_path.display(),
            String::from_utf8_lossy(&output.stderr).trim(),
        ));
    }
    // The SELF-RESOLVED model is deliberately NOT added to `inputs`: the caller
    // registers those as build dependencies, and an artifact this function may
    // rewrite is exactly the wrong thing to depend on — that is the perpetual
    // rebuild the `is_fresh` check above exists to prevent. The INPUTS are what
    // decide whether the consumer is stale.
    //
    // Issue 0495 — the BUILD-PRODUCED branch above IS tracked, and the asymmetry
    // is the point: there this function is a reader of someone else's artifact,
    // here it is the writer. Only a writer can loop on its own output.
    Ok((out, inputs))
}

/// True when `path` exists and is at least as new as every file in `inputs`.
fn is_fresh(path: &Path, inputs: &[PathBuf]) -> bool {
    let Ok(meta) = std::fs::metadata(path) else {
        return false;
    };
    let Ok(when) = meta.modified() else {
        return false;
    };
    inputs.iter().all(|i| {
        std::fs::metadata(i)
            .and_then(|m| m.modified())
            .map(|t| t <= when)
            .unwrap_or(true)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // Env is process-global, so these run under one lock and restore what they
    // set — a leaked NROS_MODEL_DIR would silently reorder every later test.
    fn with_env<T>(pairs: &[(&str, Option<&str>)], f: impl FnOnce() -> T) -> T {
        use std::sync::{Mutex, OnceLock};
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        // `unwrap_or_else(into_inner)`, not `unwrap()`: this guards env, not
        // data, so a panicking test leaves nothing inconsistent — but poisoning
        // makes every LATER test fail with `PoisonError` instead of its own
        // assertion. One real failure read as five (phase-336 W7).
        let _g = LOCK
            .get_or_init(|| Mutex::new(()))
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let saved: Vec<(String, Option<std::ffi::OsString>)> = pairs
            .iter()
            .map(|(k, _)| ((*k).to_string(), std::env::var_os(k)))
            .collect();
        for (k, v) in pairs {
            match v {
                Some(v) => unsafe { std::env::set_var(k, v) },
                None => unsafe { std::env::remove_var(k) },
            }
        }
        let out = f();
        for (k, v) in saved {
            match v {
                Some(v) => unsafe { std::env::set_var(&k, v) },
                None => unsafe { std::env::remove_var(&k) },
            }
        }
        out
    }

    #[test]
    fn committed_location_is_the_fallback() {
        with_env(&[("NROS_MODEL_DIR", None), ("OUT_DIR", None)], || {
            let p = resolve_model_path(
                Path::new("/ws/src/demo_bringup"),
                "config/system_model.yaml",
            );
            assert_eq!(
                p,
                PathBuf::from("/ws/src/demo_bringup/config/system_model.yaml")
            );
        });
    }

    #[test]
    fn build_output_outranks_committed_and_drops_the_config_level() {
        with_env(
            &[("NROS_MODEL_DIR", None), ("OUT_DIR", Some("/build/out"))],
            || {
                let c = model_search_paths(
                    Path::new("/ws/src/demo_bringup"),
                    "config/system_model.yaml",
                );
                // Issue 0825 — the rung is keyed on the bringup's PATH, so the
                // directory carries a hash. Asserted through the same function
                // the production code calls, because the POINT is that the two
                // sides agree; a literal here would let them drift apart while
                // this test stayed green.
                assert_eq!(
                    c[0],
                    PathBuf::from("/build/out/nros")
                        .join(build_scoped_dir(Path::new("/ws/src/demo_bringup")))
                        .join("system_model.yaml")
                );
                assert!(
                    c[0].to_string_lossy().contains("demo_bringup"),
                    "the readable half survives: {}",
                    c[0].display()
                );
                assert_eq!(c[1], PathBuf::from("/build/out/nros/system_model.yaml"));
                // phase-330 W4/W7 inserted the two build-root rungs BETWEEN the
                // OUT_DIR candidates and the committed path, and this positional
                // test was not moved with them — it asserted the committed path
                // at `c[2]` and had been red ever since. Asserting the whole
                // ORDER rather than three indices is the point of the test, so
                // it now names every rung; a new one inserted in the middle
                // fails here loudly instead of shifting an index nobody checks.
                assert_eq!(
                    c[2],
                    PathBuf::from("/ws/build/nros/models/demo_bringup/system_model.yaml"),
                    "the workspace build root (<ws>/build/nros/models/<bringup>/) \
                     must outrank the committed config path"
                );
                assert_eq!(
                    c[3],
                    PathBuf::from(
                        "/ws/src/demo_bringup/build/nros/models/demo_bringup/system_model.yaml"
                    ),
                    "the standalone/self-bringup build root must also outrank the \
                     committed config path"
                );
                assert_eq!(
                    c[4],
                    PathBuf::from("/ws/src/demo_bringup/config/system_model.yaml"),
                    "the committed config path stays LAST — the model is a build \
                     artifact (phase-330 W4), so every build output outranks it"
                );
            },
        );
    }

    #[test]
    fn two_bringups_of_the_same_name_do_not_share_an_out_dir_slot() {
        // Issue 0825. `$OUT_DIR` is per-crate for a build script but SHARED by
        // every test binary of a package, and every workspace here names its
        // bringup `demo_bringup`. When the rung was keyed on that name, one
        // test's model was read as another's: the reader got a model with no
        // `execution.tiers`, and `apply_model_execution` erased the tiers the
        // fixture had authored. Three sibling tests failed on a file they never
        // touched.
        with_env(
            &[("NROS_MODEL_DIR", None), ("OUT_DIR", Some("/build/out"))],
            || {
                let a = model_search_paths(
                    Path::new("/ws-a/src/demo_bringup"),
                    "config/system_model.yaml",
                );
                let b = model_search_paths(
                    Path::new("/ws-b/src/demo_bringup"),
                    "config/system_model.yaml",
                );
                assert_ne!(
                    a[0], b[0],
                    "same NAME, different PATH — these must not resolve to one file"
                );
            },
        );
    }

    #[test]
    fn the_writer_and_the_reader_agree_on_the_out_dir_slot() {
        // The other half of 0825: a fix applied to only one side turns a
        // shadowing bug into a never-found bug. This is the shape CLAUDE.md
        // names — one helper, not a second spelling.
        with_env(
            &[("NROS_MODEL_DIR", None), ("OUT_DIR", Some("/build/out"))],
            || {
                let bringup = Path::new("/ws/src/demo_bringup");
                let written = model_write_dir(bringup).join("system_model.yaml");
                let searched = model_search_paths(bringup, "config/system_model.yaml");
                assert_eq!(
                    written, searched[0],
                    "what the writer writes must be what the reader reads first"
                );
            },
        );
    }

    #[test]
    fn shared_model_dir_outranks_out_dir() {
        with_env(
            &[
                ("NROS_MODEL_DIR", Some("/build/nros")),
                ("OUT_DIR", Some("/build/out")),
            ],
            || {
                let c = model_search_paths(Path::new("/ws/b"), "config/system_model.yaml");
                assert_eq!(c[0], PathBuf::from("/build/nros/b/system_model.yaml"));
                assert_eq!(c[1], PathBuf::from("/build/nros/system_model.yaml"));
            },
        );
    }

    #[test]
    fn variant_models_keep_their_own_name() {
        with_env(&[("NROS_MODEL_DIR", Some("/b")), ("OUT_DIR", None)], || {
            let c = model_search_paths(Path::new("/ws/b"), "config/talker_model.yaml");
            assert_eq!(c[0], PathBuf::from("/b/b/talker_model.yaml"));
        });
    }

    /// The in-tree bringups whose `[image.*]` tables carry `args`.
    fn arg_bound_bringups() -> Vec<PathBuf> {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        ["c", "mixed"]
            .iter()
            .map(|ws| {
                repo.join("examples/workspaces")
                    .join(ws)
                    .join("src/demo_bringup")
            })
            .collect()
    }

    /// Two images of one multi-host system must not resolve to ONE model.
    ///
    /// Issue 1136. `native_robot1` and `native_robot2` differ in exactly one
    /// thing — `args = { host = … }`, emitted as `LAUNCH_ARGS host=…` — and
    /// that single difference is what picks the per-host `[[model]]` variant.
    /// Drop it anywhere along the chain and both images resolve the whole
    /// system: two binaries, one program, no error. phase-405 W1 dropped it at
    /// the cmake end, where the loss was loud (a missing "source file"); this
    /// asserts the half where it would be SILENT.
    #[test]
    fn per_host_images_resolve_to_distinct_models() {
        let mut bound = 0;
        for bringup in arg_bound_bringups() {
            let raw = std::fs::read_to_string(bringup.join("system.toml"))
                .unwrap_or_else(|e| panic!("{}: {e}", bringup.display()));
            let doc: toml::Table = raw.parse().expect("system.toml parses");
            let images = doc
                .get("image")
                .and_then(|i| i.as_table())
                .expect("[image.*] tables");

            let mut seen: Vec<(String, String)> = Vec::new();
            for (name, img) in images {
                let Some(args) = img.get("args").and_then(|a| a.as_table()) else {
                    continue;
                };
                bound += 1;
                let launch = img.get("launch").and_then(|v| v.as_str());
                let pairs: Vec<(String, String)> = args
                    .iter()
                    .filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string())))
                    .collect();
                let rel = launch_to_model_rel(&bringup, launch, &pairs)
                    .unwrap_or_else(|e| panic!("{name}: {e}"));

                // The binding must CHANGE the answer, not merely be accepted.
                let unbound = launch_to_model_rel(&bringup, launch, &[])
                    .unwrap_or_else(|e| panic!("{name} unbound: {e}"));
                assert_ne!(
                    rel,
                    unbound,
                    "{}: image `{name}` binds {pairs:?} and still resolves the \
                     unbound model — the binding reaches nothing",
                    bringup.display(),
                );
                if let Some((other, _)) = seen.iter().find(|(_, r)| *r == rel) {
                    panic!(
                        "{}: images `{other}` and `{name}` both resolve `{rel}` — \
                         two per-host images are one program",
                        bringup.display(),
                    );
                }
                seen.push((name.clone(), rel));
            }
        }
        // A sweep over nothing passes. Both workspaces declare a robot1/robot2
        // pair, so anything under four means the tables moved and this test
        // stopped asking the question.
        assert!(
            bound >= 4,
            "expected >= 4 arg-bound images in-tree, found {bound}"
        );
    }

    #[test]
    fn missing_everywhere_reports_the_committed_path() {
        with_env(
            &[
                ("NROS_MODEL_DIR", Some("/nope")),
                ("OUT_DIR", Some("/nope2")),
            ],
            || {
                let p = resolve_model_path(Path::new("/ws/b"), "config/system_model.yaml");
                assert_eq!(p, PathBuf::from("/ws/b/config/system_model.yaml"));
            },
        );
    }
}
