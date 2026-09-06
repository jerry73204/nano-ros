//! Phase 187.3 — the install store, provenance, lockfile, and fetch/source-build
//! engine behind `nros setup`.
//!
//! A tool always lands at the same versioned prefix
//! `$NROS_HOME/sdk/<tool>/<version>/`, whether fetched (prebuilt `dist`) or
//! source-built — downstream resolves the prefix, provenance-agnostic
//! (`.nros-provenance` records which). The store is shared across workspaces;
//! `nros-sdk.lock` pins what's installed. See
//! `docs/design/0014-nros-setup-toolchain-management.md`.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    process::Command,
};

use eyre::{Result, WrapErr, bail, eyre};
use serde::{Deserialize, Serialize};

use super::sdk_index::{SourcePackage, SourceProvision, ToolPackage};

/// The lockfile name (written in cwd by `nros setup` / auto-setup — pins the
/// installed toolset for the workspace it's run in). Single source for the name.
pub const LOCK_FILE: &str = "nros-sdk.lock";

/// The shared SDK store root: `$NROS_HOME/sdk`, else `~/.nros/sdk`, else
/// `./.nros/sdk`.
pub fn store_root() -> PathBuf {
    if let Some(h) = std::env::var_os("NROS_HOME") {
        return PathBuf::from(h).join("sdk");
    }
    if let Some(h) = std::env::var_os("HOME") {
        return PathBuf::from(h).join(".nros").join("sdk");
    }
    PathBuf::from(".nros").join("sdk")
}

/// The versioned install prefix for a tool — identical for prebuilt + source.
pub fn tool_prefix(root: &Path, tool: &str, version: &str) -> PathBuf {
    root.join(tool).join(version)
}

/// The pinned install prefix for a tool — phase-365 W1, the ONE constructor.
///
/// nano-ros decides where a provisioned tool goes: `nros setup` writes
/// `<store>/<tool>/<version>` because the index named that version. The layout
/// is our own OUTPUT, not a fact about the environment, so a consumer must
/// CONSTRUCT the path from the same two inputs that produced it rather than
/// search the store for it.
///
/// A search can return something we did not install (the legacy unversioned
/// `corrosion/{lib,share}` prefix), something a DIFFERENT project installed (a
/// newer version, since the store is shared and the pin is per-project), or
/// nothing — three wrong answers to a question with a known right one. Measured
/// on 2026-08-16 in a tree pinning `corrosion 0.6.1-nros1`: 155 resolutions of
/// 0.5.1 against 28 of 0.6.1, with the search's own ordering verified correct
/// (issue 0625).
///
/// This is the same rule CLAUDE.md already states one layer up for SystemModels
/// — locate through `nros_orchestration_ir::model_location`, never a
/// hand-derived path.
///
/// Returns `None` when the index has no such tool; the caller reports it with
/// the provisioning command, and must NEVER substitute another version.
pub fn tool_dir(index: &super::sdk_index::SdkIndex, tool: &str) -> Option<PathBuf> {
    let version = &index.tool.get(tool)?.version;
    Some(tool_prefix(&store_root(), tool, version))
}

/// The pinned prefix if it exists, else the LEGACY FLAT prefix if that does —
/// issue 0628.
///
/// [`tool_dir`] answers "where does this tool go", which is the right question
/// for an installer and the wrong one for a consumer, because two provisioning
/// paths have historically written two shapes:
///
/// ```text
/// nros setup --tool corrosion       ->  <store>/<tool>/<version>/   (versioned)
/// just workspace install-corrosion  ->  <store>/<tool>/             (flat, legacy)
/// ```
///
/// `install-corrosion` now resolves through `nros sdk-path` and writes the
/// versioned shape, so flat is only ever residue on a host provisioned before
/// that. But residue is what people HAVE: phase-365 replaced a newest-first
/// prefix SEARCH with a single constructed path, and on a flat-only host the
/// constructed path does not exist, `find_package` is never called, and the
/// configure silently fetches Corrosion from the network while advising you to
/// install the copy you already have.
///
/// This stays construction, not search — the phase-365 thesis is intact. It
/// builds TWO candidates from the same two inputs and takes the first that
/// resolves; it never enumerates the store, so it cannot return a version
/// nobody pinned (issue 0500) nor a sibling project's install.
///
/// Returns `None` when neither resolves, so a caller can say which paths it
/// looked at rather than reporting "not installed".
pub fn tool_dir_usable(index: &super::sdk_index::SdkIndex, tool: &str) -> Option<PathBuf> {
    tool_dir_candidates(index, tool)
        .into_iter()
        .find(|cand| cand.is_dir())
}

/// The candidates [`tool_dir_usable`] tries, in order — exposed so an error
/// message can NAME them. "Installed where I did not look" and "not installed"
/// are different problems and used to print the same line.
pub fn tool_dir_candidates(index: &super::sdk_index::SdkIndex, tool: &str) -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Some(versioned) = tool_dir(index, tool) {
        out.push(versioned);
    }
    // The flat prefix is only meaningful when something was installed INTO it.
    // `<store>/<tool>` always exists once any version was installed under it, so
    // requiring an install marker is what keeps a versioned-only store from
    // resolving to its own parent — which would hand `find_package` a directory
    // holding `0.6.1-nros1/` and nothing it can use.
    let flat = store_root().join(tool);
    if flat.join(".installed-version").is_file() || flat.join(".nros-provenance").is_file() {
        out.push(flat);
    }
    out
}

/// The user-facing bin dir: `$NROS_HOME/bin`, else `~/.nros/bin`, else
/// `./.nros/bin` — the sibling of [`store_root`], resolved the same way.
///
/// This is the ONE directory a user puts on PATH. The store itself never is:
/// `activate.sh` only adds a store `bin/` for the tools named in
/// `scripts/sdk-path-tools.txt`, which exists for binaries something we do not
/// control invokes by bare name.
pub fn front_dir() -> PathBuf {
    if let Some(h) = std::env::var_os("NROS_HOME") {
        return PathBuf::from(h).join("bin");
    }
    if let Some(h) = std::env::var_os("HOME") {
        return PathBuf::from(h).join(".nros").join("bin");
    }
    PathBuf::from(".nros").join("bin")
}

/// Order two store version strings — digit runs numerically, the rest as text.
///
/// `sort -Vr` is what the store's other consumers use and it is not usable
/// here. Issue 0625: a flat `corrosion/` prefix put `lib/` and `share/` where
/// versions go, and a pure-alpha name sorts AHEAD of numeric ones under `-V`,
/// so `lib` won 155 of 183 resolutions in a single configure. The filter in
/// [`installed_versions`] is the real defence; this is the second one, and it
/// also gets `nros1` vs `nros10` right, which a plain string compare does not.
fn version_key(v: &str) -> Vec<(u64, String)> {
    let mut out = Vec::new();
    let mut rest = v;
    while !rest.is_empty() {
        let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        rest = &rest[digits.len()..];
        let text: String = rest.chars().take_while(|c| !c.is_ascii_digit()).collect();
        rest = &rest[text.len()..];
        out.push((digits.parse::<u64>().unwrap_or(0), text));
    }
    out
}

/// Every version of `tool` actually installed under `root`, newest LAST.
///
/// "Installed" means the directory carries a provenance marker — the file
/// [`execute`] writes on success. That is what keeps a partial unpack, a
/// legacy flat prefix's `lib/`, and a directory somebody created by hand out
/// of the answer. A name that does not begin with a digit is skipped for the
/// same reason: a version starts with a number, and the alternative is issue
/// 0625 again.
///
/// This ENUMERATES the store, which the pinned-resolution path is forbidden to
/// do (`tool_dir`, phase-365). The two are asking different questions: that one
/// means "where does the version I pinned live" and has one right answer to
/// construct; this one means "what is the newest thing here", which nothing but
/// the store knows.
pub fn installed_versions(root: &Path, tool: &str) -> Vec<String> {
    let mut found: Vec<String> = Vec::new();
    let Ok(entries) = std::fs::read_dir(root.join(tool)) else {
        return found;
    };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if !name.starts_with(|c: char| c.is_ascii_digit()) {
            continue;
        }
        if Provenance::read(&entry.path()).is_none() {
            continue;
        }
        found.push(name);
    }
    found.sort_by_key(|v| version_key(v));
    found
}

/// The newest installed version of `tool`, if any.
pub fn newest_installed(root: &Path, tool: &str) -> Option<String> {
    installed_versions(root, tool).pop()
}

/// Point `<front_dir>/<name>` at the NEWEST installed version of `tool`, for
/// each `bin/<name>` the index's `front` list names. Returns what was linked.
///
/// Versions accumulate — nothing prunes the store (issue 0500) — and the user
/// is promised exactly one command: `nros` is whichever version is newest, not
/// whichever was installed last. So this recomputes from the store rather than
/// linking the prefix it was just handed; installing an older version on
/// purpose must not silently downgrade the command.
///
/// A symlink, not a copy: the store is the single artifact and `readlink -f`
/// then answers "which version am I running" without a provenance lookup.
pub fn front_newest(root: &Path, tool: &str, front: &[String]) -> Result<Vec<PathBuf>> {
    front_newest_into(root, &front_dir(), tool, front)
}

/// [`front_newest`] with the destination passed in.
///
/// The public wrapper reads `$NROS_HOME`; this takes both directories as
/// parameters so the decision is a pure function of them and its tests need no
/// `set_var` — a process-global that leaks between parallel tests (issue 1101),
/// and the same refactor phase-431 W1 made for the workspace guard.
pub fn front_newest_into(
    root: &Path,
    dir: &Path,
    tool: &str,
    front: &[String],
) -> Result<Vec<PathBuf>> {
    if front.is_empty() {
        return Ok(Vec::new());
    }
    let Some(version) = newest_installed(root, tool) else {
        return Ok(Vec::new());
    };
    let prefix = tool_prefix(root, tool, &version);
    std::fs::create_dir_all(dir).wrap_err_with(|| format!("create {}", dir.display()))?;
    let mut linked = Vec::new();
    for rel in front {
        let target = prefix.join(rel);
        if !target.is_file() {
            bail!(
                "{tool} {version} declares `front = [\"{rel}\"]` in the index, but \
                 {} does not exist. The install is incomplete, or the index names \
                 the wrong path.",
                target.display()
            );
        }
        let Some(name) = Path::new(rel).file_name() else {
            bail!("{tool}: `front` entry {rel:?} has no file name");
        };
        let link = dir.join(name);
        // Replace rather than fail: this runs on every install, and the whole
        // point is that the link MOVES to the newest version.
        match std::fs::symlink_metadata(&link) {
            Ok(_) => {
                std::fs::remove_file(&link)
                    .wrap_err_with(|| format!("replace {}", link.display()))?;
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                return Err(e).wrap_err_with(|| format!("stat {}", link.display()));
            }
        }
        std::os::unix::fs::symlink(&target, &link)
            .wrap_err_with(|| format!("link {} -> {}", link.display(), target.display()))?;
        linked.push(link);
    }
    Ok(linked)
}

/// How a tool was installed; persisted to `<prefix>/.nros-provenance`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ProvenanceKind {
    Prebuilt,
    Source,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Provenance {
    pub kind: ProvenanceKind,
    pub version: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sha256: Option<String>,
}

impl Provenance {
    fn marker(prefix: &Path) -> PathBuf {
        prefix.join(".nros-provenance")
    }

    /// Read the provenance marker, if the prefix is populated.
    pub fn read(prefix: &Path) -> Option<Self> {
        let raw = std::fs::read_to_string(Self::marker(prefix)).ok()?;
        toml::from_str(&raw).ok()
    }

    pub fn write(&self, prefix: &Path) -> Result<()> {
        std::fs::create_dir_all(prefix)
            .wrap_err_with(|| format!("create prefix {}", prefix.display()))?;
        std::fs::write(Self::marker(prefix), toml::to_string(self)?)
            .wrap_err("write .nros-provenance")?;
        Ok(())
    }
}

/// `nros-sdk.lock` — the exact installed toolset, for reproducibility.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SdkLock {
    #[serde(default)]
    pub tool: BTreeMap<String, LockEntry>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LockEntry {
    pub version: String,
    pub provenance: ProvenanceKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sha256: Option<String>,
}

impl SdkLock {
    /// Load the lockfile; a missing file is an empty lock (not an error).
    pub fn load(path: &Path) -> Result<Self> {
        match std::fs::read_to_string(path) {
            Ok(raw) => toml::from_str(&raw).wrap_err("invalid nros-sdk.lock"),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Self::default()),
            Err(e) => Err(e).wrap_err_with(|| format!("read {}", path.display())),
        }
    }

    pub fn record(&mut self, tool: &str, p: &Provenance) {
        self.tool.insert(
            tool.to_string(),
            LockEntry {
                version: p.version.clone(),
                provenance: p.kind,
                sha256: p.sha256.clone(),
            },
        );
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        std::fs::write(path, toml::to_string_pretty(self)?)
            .wrap_err_with(|| format!("write {}", path.display()))
    }
}

/// Where a source recipe's tree comes from.
///
/// Two modes because git is not always a way to GET a buildable tree: GNU
/// make's git tree ships no `configure`, only a `bootstrap` that wants
/// autotools and gnulib. Everything after the fetch — `configure`, `install`,
/// the toolchain choice — is identical between them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SourceFetch {
    Git { git: String, git_ref: String },
    Tarball { url: String, sha256: String },
}

/// The decided install action for a tool on a host.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InstallAction {
    /// Already at the prefix (idempotent skip).
    Present,
    /// Fetch + verify + unpack the prebuilt artifact.
    Prebuilt {
        url: String,
        sha256: String,
        /// Optional unpack override — see `DistArtifact::install`. `None` keeps
        /// the mirror-shaped default (`tar -xf` into the prefix).
        install: Option<String>,
    },
    /// Build from source into the same prefix.
    Source {
        /// Where the tree comes from. Exactly one variant, enforced by
        /// `SdkIndex::validate`.
        fetch: SourceFetch,
        configure: Option<String>,
        install: Option<String>,
        /// Issue 0374 d4 — build with the checkout's own `rust-toolchain.toml`
        /// rather than the workspace channel.
        respect_toolchain: bool,
    },
    /// No prebuilt for this host and no source recipe.
    Unavailable,
}

/// Decide how to install `tool` on `host`, given the prefix's current state.
/// Pure — does no I/O beyond reading the provenance marker.
pub fn plan_install(tool: &ToolPackage, host: &str, prefix: &Path) -> InstallAction {
    if Provenance::read(prefix).is_some() {
        return InstallAction::Present;
    }
    if let Some(d) = tool.dist_for(host) {
        return InstallAction::Prebuilt {
            url: d.url.clone(),
            sha256: d.sha256.clone(),
            install: d.install.clone(),
        };
    }
    if let Some(s) = &tool.source {
        // `validate` has already rejected neither-and-both, so this only has to
        // pick. Preferring `git` when present keeps the pre-tarball behaviour
        // byte-identical for every existing recipe.
        let fetch = match (&s.git, &s.url) {
            (Some(git), _) => SourceFetch::Git {
                git: git.clone(),
                git_ref: s.git_ref.clone().unwrap_or_default(),
            },
            (None, Some(url)) => SourceFetch::Tarball {
                url: url.clone(),
                sha256: s.sha256.clone().unwrap_or_default(),
            },
            (None, None) => return InstallAction::Unavailable,
        };
        return InstallAction::Source {
            fetch,
            configure: s.configure.clone(),
            install: s.install.clone(),
            respect_toolchain: s.respect_toolchain,
        };
    }
    InstallAction::Unavailable
}

/// Execute an install action into `prefix`, returning the recorded provenance.
/// Side-effecting (shells out to curl / sha256sum / tar / git); real-run only.
/// Install `tool` at `prefix`, then front whatever the index's `front` list
/// names (phase-431 W3).
///
/// `front` is a parameter of the INSTALL rather than a separate call, because
/// three call sites reach this function and a fourth spelling of "and then link
/// it" is how one of them ends up not doing it. Fronting is a no-op for the
/// tools that declare nothing, which is all of them but `nros`.
pub fn execute(
    action: &InstallAction,
    tool: &str,
    version: &str,
    prefix: &Path,
    front: &[String],
) -> Result<Provenance> {
    let provenance = execute_install(action, tool, version, prefix)?;
    for link in front_newest(&store_root(), tool, front)? {
        eprintln!("    → {} (newest installed)", link.display());
    }
    Ok(provenance)
}

fn execute_install(
    action: &InstallAction,
    tool: &str,
    version: &str,
    prefix: &Path,
) -> Result<Provenance> {
    match action {
        InstallAction::Present => Provenance::read(prefix)
            .ok_or_else(|| eyre!("{tool}: present but no provenance marker")),
        InstallAction::Prebuilt {
            url,
            sha256,
            install,
        } => {
            // The prebuilt dists are `.tar.zst`; `tar -xf` shells out to the
            // external `zstd` binary to decompress. A host without it (stock
            // Ubuntu 22.04 ships no `zstd`) otherwise fails DEEP inside tar with
            // a cryptic "zstd: Cannot exec", after a download and behind a bare
            // "unpack prebuilt archive". Probe BEFORE downloading and fail with
            // the package name for the detected manager (issue 0385).
            if (url.ends_with(".zst") || url.ends_with(".tzst")) && !command_on_path("zstd") {
                let remedy = crate::cmd::setup::detect_package_manager()
                    .map(|m| crate::cmd::setup::native_install_command(m, &["zstd".to_string()]))
                    .unwrap_or_else(|| "install `zstd` with your package manager".to_string());
                eyre::bail!(
                    "this prebuilt dist is zstd-compressed, but the `zstd` binary \
                     is not on PATH — `tar` cannot unpack a `.tar.zst` without it. \
                     Install it:\n  {remedy}\n(or run `nros setup --system`)"
                );
            }
            std::fs::create_dir_all(prefix)
                .wrap_err_with(|| format!("create {}", prefix.display()))?;
            let archive = prefix.with_extension("download");
            sh(
                &[
                    "curl",
                    "-L",
                    "--fail",
                    "--silent",
                    "--show-error",
                    "-o",
                    &archive.to_string_lossy(),
                    url,
                ],
                None,
            )
            .wrap_err_with(|| format!("download {url}"))?;
            verify_sha256(&archive, sha256)?;
            // Default: the mirror shape (a prefix-rooted tar). An UPSTREAM
            // asset often is not that — ninja publishes a zip holding a bare
            // binary — so a dist may carry its own unpack step. Same free-form
            // shell as a source recipe's `install`, for the same reason.
            match install {
                None => {
                    sh(
                        &[
                            "tar",
                            "-xf",
                            &archive.to_string_lossy(),
                            "-C",
                            &prefix.to_string_lossy(),
                        ],
                        None,
                    )
                    .wrap_err("unpack prebuilt archive")?;
                }
                Some(cmd) => {
                    let cmd = cmd
                        .replace("{archive}", &archive.to_string_lossy())
                        .replace("{prefix}", &prefix.to_string_lossy());
                    sh(&["sh", "-c", &cmd], None)
                        .wrap_err("unpack prebuilt archive (dist install step)")?;
                }
            }
            let _ = std::fs::remove_file(&archive);
            let p = Provenance {
                kind: ProvenanceKind::Prebuilt,
                version: version.to_string(),
                sha256: Some(sha256.clone()),
            };
            p.write(prefix)?;
            Ok(p)
        }
        InstallAction::Source {
            fetch,
            configure,
            install,
            respect_toolchain,
        } => {
            // Issue 0374 d4 — build with the workspace's pinned channel unless
            // the recipe opts out. `None` (unreadable pin, or opted out) keeps
            // the old behaviour: rustup resolves from the checkout, which may
            // download a second toolchain.
            let toolchain: Option<String> = if *respect_toolchain {
                None
            } else {
                std::env::var("NROS_REPO_DIR")
                    .ok()
                    .map(PathBuf::from)
                    .as_deref()
                    .and_then(workspace_rust_channel)
            };
            let toolchain = toolchain.as_deref();
            let src = prefix.with_extension("src");
            let _ = std::fs::remove_dir_all(&src);
            let src_str = src.to_string_lossy();
            match fetch {
                SourceFetch::Git { git, git_ref } => {
                    // `git_ref` may be a raw SHA (index tools with no upstream tag pin
                    // the commit), and `git clone --depth 1 --branch <sha>` is rejected
                    // ("Remote branch <sha> not found"). Mirror the `[source.*]` shallow
                    // path: init + fetch-by-ref at depth 1 (works for sha/tag/branch via
                    // the server's reachable-SHA support) + detached checkout.
                    sh(&["git", "init", "-q", &src_str], None)
                        .wrap_err_with(|| format!("git init {src_str} ({tool})"))?;
                    sh(
                        &["git", "-C", &src_str, "remote", "add", "origin", git],
                        None,
                    )
                    .wrap_err_with(|| format!("git remote add ({tool})"))?;
                    sh(
                        &[
                            "git", "-C", &src_str, "fetch", "-q", "--depth", "1", "origin", git_ref,
                        ],
                        None,
                    )
                    .wrap_err_with(|| format!("git fetch --depth 1 {git_ref} ({tool})"))?;
                    sh(
                        &["git", "-C", &src_str, "checkout", "-q", "FETCH_HEAD"],
                        None,
                    )
                    .wrap_err_with(|| format!("git checkout FETCH_HEAD ({tool})"))?;
                }
                SourceFetch::Tarball { url, sha256 } => {
                    // `--strip-components=1` so `configure`/`install` see the
                    // project root exactly as the git path leaves it: every step
                    // after the fetch is then mode-independent, which is the
                    // point of putting both behind one enum.
                    std::fs::create_dir_all(&src)
                        .wrap_err_with(|| format!("create {src_str} ({tool})"))?;
                    let archive = prefix.with_extension("src.download");
                    sh(
                        &[
                            "curl",
                            "-L",
                            "--fail",
                            "--silent",
                            "--show-error",
                            "-o",
                            &archive.to_string_lossy(),
                            url,
                        ],
                        None,
                    )
                    .wrap_err_with(|| format!("download {url} ({tool})"))?;
                    verify_sha256(&archive, sha256)?;
                    sh(
                        &[
                            "tar",
                            "-xf",
                            &archive.to_string_lossy(),
                            "-C",
                            &src_str,
                            "--strip-components=1",
                        ],
                        None,
                    )
                    .wrap_err_with(|| format!("unpack source tarball ({tool})"))?;
                    let _ = std::fs::remove_file(&archive);
                }
            }
            let prefix_abs = prefix.to_string_lossy().to_string();
            if let Some(cfg) = configure {
                sh_with_toolchain(
                    &["sh", "-c", &cfg.replace("{prefix}", &prefix_abs)],
                    Some(&src),
                    toolchain,
                )
                .wrap_err("configure step")?;
            }
            if let Some(inst) = install {
                if let Some(tc) = toolchain {
                    eprintln!(
                        "nros setup: building {tool} with the workspace Rust channel \
                         ({tc}) rather than the checkout's own pin — set \
                         `respect_toolchain = true` on this recipe if it needs its own."
                    );
                }
                sh_with_toolchain(
                    &["sh", "-c", &inst.replace("{prefix}", &prefix_abs)],
                    Some(&src),
                    toolchain,
                )
                .wrap_err("install step")?;
            }
            let p = Provenance {
                kind: ProvenanceKind::Source,
                version: version.to_string(),
                sha256: None,
            };
            p.write(prefix)?;
            Ok(p)
        }
        InstallAction::Unavailable => {
            bail!("{tool} {version}: no prebuilt for this host and no source recipe in the index")
        }
    }
}

/// Verify `path`'s sha256 equals `expected` (shells out to `sha256sum`, falling
/// back to `shasum -a 256` on macOS).
fn verify_sha256(path: &Path, expected: &str) -> Result<()> {
    let out = Command::new("sha256sum")
        .arg(path)
        .output()
        .or_else(|_| {
            Command::new("shasum")
                .args(["-a", "256"])
                .arg(path)
                .output()
        })
        .wrap_err("run sha256sum / shasum")?;
    if !out.status.success() {
        bail!("sha256sum failed for {}", path.display());
    }
    let got = String::from_utf8_lossy(&out.stdout);
    let got = got.split_whitespace().next().unwrap_or("");
    if !got.eq_ignore_ascii_case(expected) {
        bail!(
            "sha256 mismatch for {}: expected {expected}, got {got}",
            path.display()
        );
    }
    Ok(())
}

/// Does a `remote.origin.fetch` refspec set make `<branch>` resolvable?
///
/// A refspec's SOURCE side (before the `:`) is what matters: `refs/heads/*`
/// covers everything, `refs/heads/<branch>` covers exactly one. A leading `+`
/// (force) is not part of the ref pattern.
fn refspecs_cover_branch(specs: &str, branch: &str) -> bool {
    specs.lines().any(|spec| {
        let src = spec
            .trim()
            .trim_start_matches('+')
            .split(':')
            .next()
            .unwrap_or("")
            .trim();
        src == "refs/heads/*" || src == format!("refs/heads/{branch}")
    })
}

/// Issue 0833 follow-on — a submodule that DECLARES a branch must be able to
/// resolve it.
///
/// `git submodule update --depth 1` (the shallow path above, and the default
/// for most `[source.*]` rows) clones single-branch: it writes
/// `remote.origin.fetch = +refs/heads/main:refs/remotes/origin/main` and
/// nothing else. When `.gitmodules` declares a DIFFERENT branch — the normal
/// case for our forks, which keep patches on a named line and `main` clean —
/// that branch never materialises as a remote-tracking ref. The effects are
/// quiet and compounding: `git submodule update --remote` cannot follow the
/// declared line, `origin/<branch>` does not exist so no one can diff the pin
/// against its tip, and CLAUDE.md's "resolve the branch by containment" check
/// (`git merge-base --is-ancestor <pin> <branch>`) fails with *unknown
/// revision* rather than an answer.
///
/// Measured 2026-08-27: zenoh-pico declared `branch = nano-ros` while fetching
/// only `main`, and it was the ONLY one of 20 submodules whose declared branch
/// and refspec disagreed — so the breakage read as "that fork is odd" instead
/// of as a provisioning bug. Nothing in the repo sets that refspec; `--depth 1`
/// does.
///
/// Additive on purpose: the existing spec is kept and the declared branch is
/// added beside it, so the shallow intent survives (this does NOT widen to
/// `refs/heads/*` and start pulling every branch tip). Idempotent — a
/// submodule with no declared branch, or one already covered, is untouched, so
/// re-running `nros setup` repairs a tree provisioned before this landed.
fn ensure_submodule_branch_refspec(workspace: &Path, path: &str, shallow: bool) -> Result<()> {
    let ws = workspace.to_string_lossy();
    // `.gitmodules` is keyed by submodule NAME, which need not equal its path.
    let Ok(entries) = sh_capture(
        &[
            "git",
            "-C",
            &ws,
            "config",
            "-f",
            ".gitmodules",
            "--get-regexp",
            r"^submodule\..*\.path$",
        ],
        None,
    ) else {
        return Ok(());
    };
    let Some(name) = entries.lines().find_map(|line| {
        let (key, value) = line.split_once(char::is_whitespace)?;
        (value.trim() == path).then(|| {
            key.trim_start_matches("submodule.")
                .trim_end_matches(".path")
                .to_string()
        })
    }) else {
        return Ok(());
    };

    // No declared branch => nothing to guarantee (an untouched upstream pin).
    let key = format!("submodule.{name}.branch");
    let Ok(branch) = sh_capture(
        &[
            "git",
            "-C",
            &ws,
            "config",
            "-f",
            ".gitmodules",
            "--get",
            &key,
        ],
        None,
    ) else {
        return Ok(());
    };
    // `.` means "the superproject's branch" — not a name we can add a spec for.
    if branch.is_empty() || branch == "." {
        return Ok(());
    }

    let subdir = workspace.join(path);
    let subdir_s = subdir.to_string_lossy();
    let specs = sh_capture(
        &[
            "git",
            "-C",
            &subdir_s,
            "config",
            "--get-all",
            "remote.origin.fetch",
        ],
        None,
    )
    .unwrap_or_default();
    if refspecs_cover_branch(&specs, &branch) {
        return Ok(());
    }

    let spec = format!("+refs/heads/{branch}:refs/remotes/origin/{branch}");
    sh(
        &[
            "git",
            "-C",
            &subdir_s,
            "config",
            "--add",
            "remote.origin.fetch",
            &spec,
        ],
        None,
    )
    .wrap_err_with(|| format!("add fetch refspec for {path} (branch {branch})"))?;

    // A refspec alone creates no ref. Fetch once, at the same depth the
    // submodule was provisioned with, so `origin/<branch>` actually exists —
    // otherwise this "fix" leaves the same unknown-revision error behind a
    // config that looks correct.
    let mut fetch: Vec<&str> = vec!["git", "-C", &subdir_s, "fetch", "origin"];
    if shallow {
        fetch.push("--depth");
        fetch.push("1");
    }
    fetch.push(&branch);
    if let Err(e) = sh(&fetch, None) {
        // Non-fatal: an offline host still gets the durable config, and the
        // next fetch materialises the ref. Failing provisioning over this
        // would be worse than the state we are repairing.
        eprintln!(
            "nros setup: {path}: added fetch refspec for `{branch}`, but fetching it failed ({e}). It will resolve on the next `git fetch`."
        );
    }
    Ok(())
}

/// Outcome of [`provision_source`] — for the `nros setup` disposition line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SourceDisposition {
    /// Fetched into `dest` (clone or submodule update).
    Provisioned,
    /// `dest` already present — left untouched (idempotent skip).
    AlreadyPresent,
    /// No fetch step declared (version-only `[source.*]`).
    NoFetch,
    /// `--dry-run`: what would have happened.
    Planned,
}

/// Phase 195.B — provision a `[source.*]` package into its workspace-relative
/// `dest` from index data (never a path baked into the binary). Idempotent: an
/// already-present `dest` is left untouched. Two modes (see
/// [`SourcePackage::provision`]):
///
/// Both modes honor the source's `shallow` (default true → `--depth 1`,
/// fetch-by-SHA so a lagging pin is still a real depth-1 checkout) and
/// `recursive` (default true → descend the source's own nested submodules).
///
/// - **Clone:** shallow → `git init` + `git fetch --depth 1 origin <ref>` +
///   `git checkout FETCH_HEAD` (`ref` may be a sha, so `--branch` can't be
///   used); full → `git clone <git> <dest>` + `git checkout <ref>`. Then, if
///   recursive, `git submodule update --init --recursive [--depth 1]`.
/// - **Submodule:** `git -C <workspace> submodule update --init [--recursive]
///   [--depth 1] -- <submodule>` (inherently idempotent; checks out the
///   superproject's recorded commit, kept in lockstep with the index ref via
///   the SSOT rule).
pub fn provision_source(
    name: &str,
    src: &SourcePackage,
    workspace: &Path,
    dry_run: bool,
    shallow_override: Option<bool>,
) -> Result<SourceDisposition> {
    // `--full` / `--shallow` (per-invocation) wins over the index `shallow`.
    let shallow = shallow_override.unwrap_or(src.shallow);
    match src.provision() {
        SourceProvision::None => Ok(SourceDisposition::NoFetch),
        SourceProvision::Submodule => {
            let path = src.submodule.as_deref().expect("submodule mode has a path");
            if dry_run {
                return Ok(SourceDisposition::Planned);
            }
            // Fast path: `git submodule update --init [--recursive] [--depth 1]`.
            // CAVEAT: `--depth 1` shallow-fetches the submodule's BRANCH TIP, not
            // the pinned gitlink SHA — so a pin that lags its tip and isn't an
            // advertised ref (e.g. PX4-Autopilot's 1.15.x sha vs `main`) fails the
            // checkout here. We catch that and fall back to an explicit
            // depth-1 fetch-by-SHA (GitHub serves reachable SHAs), which `git
            // clone --branch` / `submodule update` can't express directly.
            let workspace_s = workspace.to_string_lossy();
            let mut args: Vec<&str> =
                vec!["git", "-C", &workspace_s, "submodule", "update", "--init"];
            if src.recursive {
                args.push("--recursive");
            }
            if shallow {
                args.push("--depth");
                args.push("1");
            }
            args.push("--");
            args.push(path);
            let fast = sh(&args, None);
            if let Err(e) = fast {
                if !shallow {
                    return Err(e)
                        .wrap_err_with(|| format!("git submodule update {path} (source {name})"));
                }
                // By-SHA fallback. `submodule update` already initialised the
                // gitdir + worktree (only its checkout of the lagging pin failed),
                // so fetch the exact gitlink SHA at depth 1 and check it out, then
                // descend if recursive.
                let sha = sh_capture(&["git", "-C", &workspace_s, "ls-tree", "HEAD", path], None)
                    .wrap_err_with(|| format!("read gitlink sha for {path} (source {name})"))?
                    .split_whitespace()
                    .nth(2)
                    .map(str::to_owned)
                    .ok_or_else(|| eyre!("no gitlink sha for {path} (source {name})"))?;
                let subdir = workspace.join(path);
                let subdir_s = subdir.to_string_lossy();
                sh(
                    &[
                        "git", "-C", &subdir_s, "fetch", "--depth", "1", "origin", &sha,
                    ],
                    None,
                )
                .wrap_err_with(|| format!("git fetch --depth 1 {sha} ({path}, source {name})"))?;
                sh(&["git", "-C", &subdir_s, "checkout", "-q", &sha], None)
                    .wrap_err_with(|| format!("git checkout {sha} ({path}, source {name})"))?;
                if src.recursive {
                    sh(
                        &[
                            "git",
                            "-C",
                            &subdir_s,
                            "submodule",
                            "update",
                            "--init",
                            "--recursive",
                            "--depth",
                            "1",
                            "--recommend-shallow",
                        ],
                        None,
                    )
                    .wrap_err_with(|| {
                        format!("git submodule update --recursive ({path}, source {name})")
                    })?;
                }
            }
            // Issue 0833 follow-on — `--depth 1` clones single-branch, so a
            // submodule declaring a non-`main` branch could not resolve it.
            // Runs on every provision, including the already-initialised
            // no-op, which is what makes an older tree self-heal.
            ensure_submodule_branch_refspec(workspace, path, shallow)?;
            Ok(SourceDisposition::Provisioned)
        }
        SourceProvision::Clone => {
            let git = src.git.as_deref().expect("clone mode has a git url");
            let git_ref = src.git_ref.as_deref().expect("clone mode has a ref");
            let dest = src.dest.as_deref().expect("clone mode has a dest");
            let dest_abs = workspace.join(dest);
            // Idempotent: a present, non-empty dest is left as-is (don't clobber
            // a contributor checkout / in-progress work). `nros setup` on a
            // fresh tree provisions; a populated tree is a skip.
            let present = dest_abs
                .read_dir()
                .map(|mut d| d.next().is_some())
                .unwrap_or(false);
            if present {
                return Ok(SourceDisposition::AlreadyPresent);
            }
            if dry_run {
                return Ok(SourceDisposition::Planned);
            }
            if let Some(parent) = dest_abs.parent() {
                std::fs::create_dir_all(parent)
                    .wrap_err_with(|| format!("create source dest parent {}", parent.display()))?;
            }
            let dest_str = dest_abs.to_string_lossy();
            if shallow {
                // Shallow clone of a possibly-SHA `ref`: `git clone --branch`
                // can't take a sha, so init + fetch-by-ref at depth 1 (works for
                // sha/tag/branch via the server's reachable-SHA support) + check
                // out the fetched commit (detached, same as a sha checkout).
                sh(&["git", "init", "-q", &dest_str], None)
                    .wrap_err_with(|| format!("git init {dest_str} (source {name})"))?;
                sh(
                    &["git", "-C", &dest_str, "remote", "add", "origin", git],
                    None,
                )
                .wrap_err_with(|| format!("git remote add (source {name})"))?;
                sh(
                    &[
                        "git", "-C", &dest_str, "fetch", "-q", "--depth", "1", "origin", git_ref,
                    ],
                    None,
                )
                .wrap_err_with(|| format!("git fetch --depth 1 {git_ref} (source {name})"))?;
                sh(
                    &["git", "-C", &dest_str, "checkout", "-q", "FETCH_HEAD"],
                    None,
                )
                .wrap_err_with(|| format!("git checkout FETCH_HEAD (source {name})"))?;
            } else {
                sh(&["git", "clone", git, &dest_str], None)
                    .wrap_err_with(|| format!("git clone {git} (source {name})"))?;
                sh(&["git", "-C", &dest_str, "checkout", git_ref], None)
                    .wrap_err_with(|| format!("git checkout {git_ref} (source {name})"))?;
            }
            if src.recursive {
                // A cloned source may carry its own nested submodules.
                let mut sub: Vec<&str> = vec![
                    "git",
                    "-C",
                    &dest_str,
                    "submodule",
                    "update",
                    "--init",
                    "--recursive",
                ];
                if shallow {
                    sub.push("--depth");
                    sub.push("1");
                }
                sh(&sub, None).wrap_err_with(|| {
                    format!("git submodule update --recursive (source {name})")
                })?;
            }
            Ok(SourceDisposition::Provisioned)
        }
    }
}

/// True when `cmd` is executable on PATH (issue 0385 — probe `zstd` before an
/// unpack that would otherwise fail deep inside `tar`).
fn command_on_path(cmd: &str) -> bool {
    std::process::Command::new(cmd)
        .arg("--version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn sh(args: &[&str], cwd: Option<&Path>) -> Result<()> {
    sh_with_toolchain(args, cwd, None)
}

/// Issue 0374 direction 4 — run a source recipe under a CHOSEN Rust toolchain.
///
/// A source recipe runs with its cwd inside the upstream checkout, and rustup
/// resolves the toolchain from the directory it is invoked in. zenoh 1.7.2
/// carries `rust-toolchain.toml` `channel = "1.85.0"`, so `nros setup native`
/// silently downloads a SECOND toolchain — measured in 0374 as
/// `info: syncing channel updates for '1.85.0-…'` on a host that already had
/// the nano-ros pin, and this host still carries that 1.85.0 as a result.
///
/// Setting `RUSTUP_TOOLCHAIN` overrides the directory's pin for that command
/// only. It is a per-recipe opt-out (`respect_toolchain = true`) because a
/// recipe MAY legitimately need its own pin — a nightly-only crate cannot be
/// built by a stable pin, and forcing one would turn a working recipe into a
/// compile error.
fn sh_with_toolchain(args: &[&str], cwd: Option<&Path>, toolchain: Option<&str>) -> Result<()> {
    let (cmd, rest) = args.split_first().ok_or_else(|| eyre!("empty command"))?;
    let mut c = Command::new(cmd);
    c.args(rest);
    if let Some(d) = cwd {
        c.current_dir(d);
    }
    if let Some(tc) = toolchain {
        c.env("RUSTUP_TOOLCHAIN", tc);
    }
    let status = c.status().wrap_err_with(|| format!("spawn {cmd}"))?;
    if !status.success() {
        bail!("`{}` failed ({status})", args.join(" "));
    }
    Ok(())
}

/// The workspace's pinned Rust channel, read from `rust-toolchain.toml`.
///
/// `None` when it cannot be read or carries no `channel` — in which case source
/// recipes keep their previous behaviour (the checkout's own pin wins), because
/// guessing a toolchain is worse than the extra download this avoids.
pub fn workspace_rust_channel(workspace: &Path) -> Option<String> {
    let raw = std::fs::read_to_string(workspace.join("rust-toolchain.toml")).ok()?;
    let value: toml::Value = toml::from_str(&raw).ok()?;
    value
        .get("toolchain")?
        .get("channel")?
        .as_str()
        .map(str::to_string)
}

/// Run a command and capture trimmed stdout (for reading a gitlink SHA, etc.).
fn sh_capture(args: &[&str], cwd: Option<&Path>) -> Result<String> {
    let (cmd, rest) = args.split_first().ok_or_else(|| eyre!("empty command"))?;
    let mut c = Command::new(cmd);
    c.args(rest);
    if let Some(d) = cwd {
        c.current_dir(d);
    }
    let out = c.output().wrap_err_with(|| format!("spawn {cmd}"))?;
    if !out.status.success() {
        bail!("`{}` failed ({})", args.join(" "), out.status);
    }
    Ok(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

#[cfg(test)]
mod phase365_tool_dir_tests {
    use super::*;
    use crate::orchestration::sdk_index::SdkIndex;

    fn repo_index() -> SdkIndex {
        // The real index, so this test tracks the shipped pins rather than a
        // fixture that can agree with a stale copy of them.
        let idx =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../nros-sdk-index.toml");
        SdkIndex::load(&idx).expect("load nros-sdk-index.toml")
    }

    /// phase-365 W1 — the constructor and the INSTALLER must be one function.
    ///
    /// `nros setup` computes its destination as
    /// `tool_prefix(&store_root(), name, &tool.version)`. If `tool_dir` ever
    /// derives anything else, consumers look where nothing was installed — the
    /// two-spellings failure this phase exists to remove, reintroduced at its
    /// own root.
    #[test]
    fn tool_dir_matches_where_setup_installs_for_every_pinned_tool() {
        let index = repo_index();
        let root = store_root();
        assert!(!index.tool.is_empty(), "index declares no tools");
        for (name, tool) in &index.tool {
            let installed = tool_prefix(&root, name, &tool.version);
            let resolved = tool_dir(&index, name)
                .unwrap_or_else(|| panic!("tool_dir returned None for pinned tool `{name}`"));
            assert_eq!(
                installed,
                resolved,
                "`{name}`: setup installs to {} but tool_dir resolves {}",
                installed.display(),
                resolved.display()
            );
        }
    }

    /// phase-431 W3 — the index's `nros` smoke check asserts the CODEGEN
    /// VERSION, so it must be the version this binary actually emits.
    ///
    /// The entry exists to make a released CLI safe to install, and the property
    /// that makes it safe is what it emits — so `smoke` runs
    /// `bin/nros --codegen-version` rather than `--version`. That number is
    /// AUTHORED in two places once it is written into the index, and an
    /// authored mirror in this tree has drifted every time nothing bound it
    /// (`check-rmw-api-parity`'s map: 25 symbols, reading green). Bound here.
    #[test]
    fn the_nros_smoke_check_expects_the_version_this_binary_emits() {
        let index = repo_index();
        let tool = index
            .tool
            .get("nros")
            .expect("`[tool.nros]` — phase-431 W3 added it; if it went away, so should this test");
        let expected = crate::abi_guard::EMITTED_VERSION.to_string();
        let probe = tool
            .smoke
            .iter()
            .find(|p| p.run.contains("--codegen-version"))
            .expect("`[tool.nros].smoke` must probe `--codegen-version`");
        assert_eq!(
            probe.expect, expected,
            "the index expects `{}` from `{}` while this binary emits `{expected}`",
            probe.expect, probe.run
        );
        // The other half of the promise: one command, newest version.
        assert_eq!(tool.front, vec!["bin/nros".to_string()]);
    }

    /// Issue 0628 — the candidate list is CONSTRUCTED, versioned first.
    ///
    /// Order is the whole contract: a host carrying both shapes must get the
    /// pinned one. Flat is legacy residue, and residue must never outrank a pin.
    #[test]
    fn candidates_are_versioned_then_flat_and_never_enumerate() {
        let index = repo_index();
        let root = store_root();
        let tool = index
            .tool
            .keys()
            .next()
            .expect("index declares no tools")
            .clone();
        let cands = tool_dir_candidates(&index, &tool);
        assert!(!cands.is_empty(), "no candidate for pinned `{tool}`");
        assert_eq!(
            cands[0],
            tool_dir(&index, &tool).unwrap(),
            "the pinned path must be tried FIRST"
        );
        for c in &cands {
            assert!(
                c == &root.join(&tool) || c.starts_with(root.join(&tool)),
                "candidate {} escaped <store>/<tool> — this must not be a search",
                c.display()
            );
        }
        assert!(
            cands.len() <= 2,
            "exactly two shapes exist; {} candidates means something is enumerating",
            cands.len()
        );
    }

    /// A flat prefix with no install marker is NOT a candidate.
    ///
    /// `<store>/<tool>` exists as soon as any version is installed beneath it,
    /// so an unguarded flat candidate would hand `find_package` a directory
    /// containing `0.6.1-nros1/` and nothing it can resolve from — the
    /// pre-0493 failure this must not reintroduce from the other side.
    #[test]
    fn a_bare_parent_directory_is_not_mistaken_for_a_flat_install() {
        let index = repo_index();
        let tool = index.tool.keys().next().unwrap().clone();
        let flat = store_root().join(&tool);
        let marked =
            flat.join(".installed-version").is_file() || flat.join(".nros-provenance").is_file();
        let listed = tool_dir_candidates(&index, &tool).contains(&flat);
        assert_eq!(
            marked,
            listed,
            "`{tool}`: flat prefix {} is {} an install but {} listed",
            flat.display(),
            if marked { "marked as" } else { "NOT" },
            if listed { "IS" } else { "is NOT" }
        );
    }

    /// A miss is `None`, never a fallback. Substituting a different version is
    /// exactly how a store shared between two projects hands the wrong one to
    /// the older checkout (issue 0625).
    #[test]
    fn an_unpinned_tool_resolves_to_nothing() {
        assert!(tool_dir(&repo_index(), "no-such-tool-in-the-index").is_none());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orchestration::sdk_index::SdkIndex;

    fn tmp(tag: &str) -> PathBuf {
        crate::test_support::scratch_path(&format!("store_{tag}"))
    }

    /// A version prefix the store would accept: populated, with the marker
    /// `execute` writes on success.
    fn install_fake(root: &Path, tool: &str, version: &str, rel: &str) -> PathBuf {
        let prefix = tool_prefix(root, tool, version);
        let file = prefix.join(rel);
        std::fs::create_dir_all(file.parent().unwrap()).unwrap();
        std::fs::write(&file, format!("#!/bin/sh\necho {version}\n")).unwrap();
        Provenance {
            kind: ProvenanceKind::Prebuilt,
            version: version.to_string(),
            sha256: None,
        }
        .write(&prefix)
        .unwrap();
        prefix
    }

    /// Issue 0625's shape, in the one place that is ALLOWED to enumerate: a
    /// directory where a version belongs must not be able to win by sorting.
    #[test]
    fn installed_versions_ignores_what_is_not_an_install() {
        let root = tmp("versions");
        std::fs::remove_dir_all(&root).ok();
        install_fake(&root, "nros", "0.5.0-nros1", "bin/nros");
        install_fake(&root, "nros", "0.6.0-nros1", "bin/nros");
        // A pure-alpha sibling — `lib`/`share` under a legacy flat prefix. Under
        // `sort -Vr` this sorts AHEAD of every numeric version, which is how it
        // won 155 of 183 resolutions once.
        std::fs::create_dir_all(root.join("nros").join("lib")).unwrap();
        // A version-shaped directory with no provenance: a partial unpack, or a
        // prefix somebody created by hand. Not an install.
        std::fs::create_dir_all(root.join("nros").join("9.9.9-nros1").join("bin")).unwrap();

        assert_eq!(
            installed_versions(&root, "nros"),
            vec!["0.5.0-nros1".to_string(), "0.6.0-nros1".to_string()]
        );
        assert_eq!(
            newest_installed(&root, "nros").as_deref(),
            Some("0.6.0-nros1")
        );
        std::fs::remove_dir_all(&root).ok();
    }

    /// `nros1` < `nros2` < `nros10` — a string compare gets the last pair
    /// backwards, and the store's own vocabulary reaches double digits
    /// (`qemu 11.0.0-nros6` today).
    #[test]
    fn version_order_is_numeric_not_lexical() {
        let root = tmp("order");
        std::fs::remove_dir_all(&root).ok();
        for v in ["0.5.0-nros2", "0.5.0-nros10", "0.5.0-nros1"] {
            install_fake(&root, "nros", v, "bin/nros");
        }
        assert_eq!(
            newest_installed(&root, "nros").as_deref(),
            Some("0.5.0-nros10")
        );
        assert!("0.5.0-nros10" < "0.5.0-nros2", "a string compare disagrees");
        std::fs::remove_dir_all(&root).ok();
    }

    /// The promise: ONE `nros`, and it is the newest — not the last installed.
    #[test]
    fn front_points_at_the_newest_even_when_an_older_one_is_installed_after() {
        let root = tmp("front_newest");
        let bin = tmp("front_newest_bin");
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&bin).ok();
        let front = vec!["bin/nros".to_string()];

        install_fake(&root, "nros", "0.6.0-nros1", "bin/nros");
        let linked = front_newest_into(&root, &bin, "nros", &front).unwrap();
        assert_eq!(linked, vec![bin.join("nros")]);
        assert_eq!(
            std::fs::read_link(bin.join("nros")).unwrap(),
            tool_prefix(&root, "nros", "0.6.0-nros1").join("bin/nros")
        );

        // Installing an OLDER version afterwards must not downgrade the command.
        install_fake(&root, "nros", "0.5.0-nros1", "bin/nros");
        front_newest_into(&root, &bin, "nros", &front).unwrap();
        assert_eq!(
            std::fs::read_link(bin.join("nros")).unwrap(),
            tool_prefix(&root, "nros", "0.6.0-nros1").join("bin/nros")
        );

        // A NEWER one does move it, and replaces the existing link rather than
        // failing on it — this runs on every install.
        install_fake(&root, "nros", "0.7.0-nros1", "bin/nros");
        front_newest_into(&root, &bin, "nros", &front).unwrap();
        assert_eq!(
            std::fs::read_link(bin.join("nros")).unwrap(),
            tool_prefix(&root, "nros", "0.7.0-nros1").join("bin/nros")
        );
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&bin).ok();
    }

    /// Declaring nothing fronts nothing — every tool but `nros` is this case,
    /// and it must not create `$NROS_HOME/bin` as a side effect.
    #[test]
    fn front_is_a_no_op_for_a_tool_that_declares_none() {
        let root = tmp("front_none");
        let bin = tmp("front_none_bin");
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&bin).ok();
        install_fake(&root, "qemu", "11.0.0-nros6", "bin/qemu-system-arm");
        assert!(
            front_newest_into(&root, &bin, "qemu", &[])
                .unwrap()
                .is_empty()
        );
        assert!(!bin.exists(), "no `front` must not create the bin dir");

        // An UNINSTALLED tool that declares one is also silent -- there is
        // nothing to point at yet, and that is not an error.
        assert!(
            front_newest_into(&root, &bin, "nros", &["bin/nros".to_string()])
                .unwrap()
                .is_empty()
        );
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&bin).ok();
    }

    /// An index that names a path the install does not contain is an INDEX bug,
    /// and it must say so rather than leave a dangling link behind.
    #[test]
    fn front_refuses_a_path_the_install_does_not_have() {
        let root = tmp("front_missing");
        let bin = tmp("front_missing_bin");
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&bin).ok();
        install_fake(&root, "nros", "0.5.0-nros1", "bin/nros");
        let err = front_newest_into(&root, &bin, "nros", &["bin/nrs".to_string()])
            .unwrap_err()
            .to_string();
        assert!(err.contains("bin/nrs"), "{err}");
        assert!(!bin.join("nrs").exists(), "left a link behind");
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&bin).ok();
    }

    /// Issue 0374 d4 — the workspace channel is what source recipes build with,
    /// so reading it must not depend on formatting or on extra keys being absent.
    #[test]
    fn workspace_rust_channel_reads_the_pin() {
        let dir = tmp("channel");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("rust-toolchain.toml"),
            "[toolchain]\nchannel = \"stable\"\ncomponents = [\"clippy\"]\n",
        )
        .unwrap();
        assert_eq!(workspace_rust_channel(&dir).as_deref(), Some("stable"));

        // A pin that names an exact version reads the same way.
        std::fs::write(
            dir.join("rust-toolchain.toml"),
            "[toolchain]\nchannel = \"1.85.0\"\n",
        )
        .unwrap();
        assert_eq!(workspace_rust_channel(&dir).as_deref(), Some("1.85.0"));

        // No file, and a file with no channel, both mean "do not override" —
        // guessing a toolchain is worse than the download this avoids.
        std::fs::remove_file(dir.join("rust-toolchain.toml")).unwrap();
        assert_eq!(workspace_rust_channel(&dir), None);
        std::fs::write(dir.join("rust-toolchain.toml"), "[toolchain]\n").unwrap();
        assert_eq!(workspace_rust_channel(&dir), None);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The opt-out must survive the index → action hop, or a recipe that needs
    /// its own nightly would be built with the workspace channel anyway.
    #[test]
    fn respect_toolchain_reaches_the_install_action() {
        let idx: SdkIndex = toml::from_str(
            r#"
            [tool.plain]
            version = "1"
            [tool.plain.source]
            git = "https://example.invalid/a"
            ref = "v1"
            install = "cargo install --path ."

            [tool.pinned]
            version = "1"
            [tool.pinned.source]
            git = "https://example.invalid/b"
            ref = "v1"
            install = "cargo install --path ."
            respect_toolchain = true
            "#,
        )
        .unwrap();
        let fresh = tmp("respect");
        let _ = std::fs::remove_dir_all(&fresh);
        match plan_install(&idx.tool["plain"], "linux-x86_64", &fresh) {
            InstallAction::Source {
                respect_toolchain, ..
            } => assert!(!respect_toolchain, "default must be false"),
            other => panic!("expected Source, got {other:?}"),
        }
        match plan_install(&idx.tool["pinned"], "linux-x86_64", &fresh) {
            InstallAction::Source {
                respect_toolchain, ..
            } => assert!(respect_toolchain, "the opt-out must reach the action"),
            other => panic!("expected Source, got {other:?}"),
        }
    }

    #[test]
    fn prefix_layout_is_tool_then_version() {
        let p = tool_prefix(Path::new("/store"), "qemu", "11.0-nros1");
        assert_eq!(p, Path::new("/store/qemu/11.0-nros1"));
    }

    #[test]
    fn provenance_roundtrips_and_marks_present() {
        let prefix = tmp("prov");
        assert!(Provenance::read(&prefix).is_none());
        let p = Provenance {
            kind: ProvenanceKind::Prebuilt,
            version: "11.0".into(),
            sha256: Some("abc".into()),
        };
        p.write(&prefix).unwrap();
        assert_eq!(Provenance::read(&prefix).as_ref(), Some(&p));
        std::fs::remove_dir_all(&prefix).ok();
    }

    #[test]
    fn lockfile_records_and_roundtrips() {
        let path = tmp("lock").join("nros-sdk.lock");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        assert!(SdkLock::load(&path).unwrap().tool.is_empty()); // missing ⇒ empty
        let mut lock = SdkLock::default();
        lock.record(
            "qemu",
            &Provenance {
                kind: ProvenanceKind::Source,
                version: "11.0".into(),
                sha256: None,
            },
        );
        lock.save(&path).unwrap();
        let back = SdkLock::load(&path).unwrap();
        assert_eq!(back.tool["qemu"].version, "11.0");
        assert_eq!(back.tool["qemu"].provenance, ProvenanceKind::Source);
        std::fs::remove_dir_all(path.parent().unwrap()).ok();
    }

    /// Verifies SDK planning prefers present tools, then prebuilt, then source, then unavailable.
    #[test]
    fn plan_picks_present_prebuilt_source() {
        let idx = SdkIndex::parse(
            "[tool.qemu]\nversion=\"11.0\"\ndist.linux-x86_64={url=\"u\",sha256=\"h\"}\n\
             [tool.qemu.source]\ngit=\"g\"\nref=\"r\"\n\
             [tool.bare]\nversion=\"1\"\n",
        )
        .unwrap();
        let qemu = &idx.tool["qemu"];
        let bare = &idx.tool["bare"];
        let fresh = tmp("plan-fresh");

        // prebuilt host → Prebuilt; non-prebuilt host with source → Source.
        assert!(matches!(
            plan_install(qemu, "linux-x86_64", &fresh),
            InstallAction::Prebuilt { .. }
        ));
        assert!(matches!(
            plan_install(qemu, "macos-arm64", &fresh),
            InstallAction::Source { .. }
        ));
        // no dist + no source → Unavailable.
        assert_eq!(
            plan_install(bare, "macos-arm64", &fresh),
            InstallAction::Unavailable
        );

        // a populated prefix → Present (idempotent skip).
        let present = tmp("plan-present");
        Provenance {
            kind: ProvenanceKind::Prebuilt,
            version: "11.0".into(),
            sha256: None,
        }
        .write(&present)
        .unwrap();
        assert_eq!(
            plan_install(qemu, "linux-x86_64", &present),
            InstallAction::Present
        );
        std::fs::remove_dir_all(&present).ok();
    }

    /// Issue 0833 follow-on — the refspec predicate behind
    /// `ensure_submodule_branch_refspec`. The bug it exists to prevent was a
    /// single-branch clone (`+refs/heads/main:…`) under a `.gitmodules`
    /// declaring `branch = nano-ros`, so the "narrow spec, different branch"
    /// row is the regression case.
    #[test]
    fn refspec_coverage_is_decided_by_the_source_side() {
        let wide = "+refs/heads/*:refs/remotes/origin/*";
        let only_main = "+refs/heads/main:refs/remotes/origin/main";

        assert!(refspecs_cover_branch(wide, "nano-ros"));
        assert!(refspecs_cover_branch(only_main, "main"));
        // The measured zenoh-pico state.
        assert!(!refspecs_cover_branch(only_main, "nano-ros"));
        assert!(!refspecs_cover_branch("", "nano-ros"));

        // Multiple specs: covered if ANY covers it (what --add produces).
        let repaired = format!("{only_main}\n+refs/heads/nano-ros:refs/remotes/origin/nano-ros");
        assert!(refspecs_cover_branch(&repaired, "nano-ros"));
        assert!(refspecs_cover_branch(&repaired, "main"));
        assert!(!refspecs_cover_branch(&repaired, "some-other-branch"));

        // The DESTINATION side must not decide it: a spec whose source is
        // `main` does not provide `nano-ros` merely by naming it on the right.
        assert!(!refspecs_cover_branch(
            "+refs/heads/main:refs/remotes/origin/nano-ros",
            "nano-ros"
        ));

        // A missing `+` is still a valid refspec.
        assert!(refspecs_cover_branch(
            "refs/heads/nano-ros:refs/remotes/origin/nano-ros",
            "nano-ros"
        ));
        // Prefix collisions are not matches.
        assert!(!refspecs_cover_branch(
            "+refs/heads/nano:refs/remotes/origin/nano",
            "nano-ros"
        ));
    }

    #[test]
    fn provision_source_no_fetch_present_and_dry_run() {
        // Version-only source → no fetch step.
        let none = SourcePackage {
            version: "1".into(),
            ..Default::default()
        };
        assert_eq!(
            provision_source("x", &none, Path::new("/ws"), false, None).unwrap(),
            SourceDisposition::NoFetch
        );

        // Clone mode, dest already populated → AlreadyPresent (no git run).
        let ws = tmp("prov-src");
        let dest_rel = "third-party/lwip";
        let dest_abs = ws.join(dest_rel);
        std::fs::create_dir_all(&dest_abs).unwrap();
        std::fs::write(dest_abs.join("CMakeLists.txt"), "x").unwrap();
        let clone = SourcePackage {
            version: "2.2.0".into(),
            git: Some("https://example/lwip.git".into()),
            git_ref: Some("STABLE-2_2_0".into()),
            dest: Some(dest_rel.into()),
            submodule: None,
            shallow: true,
            recursive: true,
        };
        assert_eq!(
            provision_source("lwip", &clone, &ws, false, None).unwrap(),
            SourceDisposition::AlreadyPresent
        );

        // Clone mode, empty dest, dry-run → Planned (no git run).
        let empty = SourcePackage {
            dest: Some("third-party/empty".into()),
            ..clone.clone()
        };
        assert_eq!(
            provision_source("lwip", &empty, &ws, true, None).unwrap(),
            SourceDisposition::Planned
        );
        std::fs::remove_dir_all(&ws).ok();
    }
}
