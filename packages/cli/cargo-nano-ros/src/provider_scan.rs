//! phase-348 W1 — source-time provider discovery: the scan.
//!
//! RFC-0071 D5. A provider announces itself with a `<nano_ros_provides/>` export
//! in its `package.xml`; this walks a list of workspace roots and reports every
//! package that does.
//!
//! **Source-time, not install-time.** colcon's discovery artifact is the ament
//! index, reached by sourcing `setup.sh` — which exists only *after* an install
//! step. nano-ros builds per-target static objects for RTOS targets that
//! generally have no dynamic linking, so there is no install-and-source stage
//! for an index to live in. Discovery therefore reads the source tree.
//!
//! **One concept, no special cases.** The search path is an ordered list of
//! roots, and the nano-ros tree is simply the FIRST entry — `packages/rmw/*` are
//! not builtins reached by a different code path, they are providers found the
//! way a user's are.
//!
//! **The scan reads only `package.xml`.** The descriptor (`nros-rmw.toml`,
//! `nros-board.toml`, …) is read only for the provider actually selected: one
//! cheap parse per package, one detailed parse per build. That is also why
//! [`ProviderPackage`] carries no descriptor fields — a scan that had to
//! understand every provider family would be a second place to teach about
//! them.
//!
//! Policy lives above this module, deliberately: shadowing between roots is
//! phase-348 W5 and ordering is W4, so the scan reports FACTS (what is where,
//! in search-path order) and takes no decision about ambiguity.
//!
//! Not to be confused with [`crate::package_discovery`], which walks
//! `Cargo.toml` to answer "what cargo packages are here". Same trees, different
//! file, different question — they are not two derivations of one fact, and
//! merging them would couple provider discovery to being written in Rust.

use eyre::{Result, WrapErr, bail};
use serde::{Deserialize, Serialize};
use std::{
    collections::HashSet,
    path::{Path, PathBuf},
};

use crate::package_xml::{PackageXml, Provision};

/// Directory names never descended into. Build output and vendored trees, both
/// of which contain `package.xml` files that are copies or third-party — and
/// `third-party/` alone is large enough to dominate the walk.
const PRUNED_DIRS: &[&str] = &[
    ".git",
    "target",
    "build",
    "install",
    "log",
    "third-party",
    "generated",
    "node_modules",
];

/// Build-root name PREFIXES, pruned like [`PRUNED_DIRS`] (issue 0645).
///
/// An exact-name list misses this tree's actual build roots, because almost
/// none of them are called `build` or `target`: a workspace fixture build
/// writes `build-workspace-fixtures/`, its FreeRTOS sibling
/// `build-workspace-fixtures-freertos/`, and phase-340's shared cargo groups
/// write `target-<coord>/`. Those are exactly the trees this list exists to
/// skip, and the exact match walked straight into them.
///
/// Measured on `examples/workspaces/mixed`: the scan descended 7113
/// directories, 3923 of them build output, paying four `stat`s each
/// (`COLCON_IGNORE`, `AMENT_IGNORE`, `NROS_IGNORE`, `package.xml`). That is
/// 34k `statx` calls with 86 % returning ENOENT — after the issue-0641 fix
/// removed the subprocess cost, this was co-dominant with it.
///
/// A staged copy under a build root DOES contain real `package.xml` files, so
/// the walk found them and they were not junk — they were duplicates of the
/// source tree it had already scanned.
///
/// The cost of the rule: a real package directory may not be named `build-*`
/// or `target-*`. That is already true by convention here (`build/` and
/// `target/` are pruned outright, and `examples/**/target-*/` is globally
/// gitignored), and the alternative — dropping an ignore marker into every
/// build root as it is created — needs every creator to remember, which is the
/// failure mode markers already have.
const PRUNED_DIR_PREFIXES: &[&str] = &["build-", "target-"];

/// Is this directory name build output or a vendored tree?
///
/// `pub` so the BOARD walk shares it rather than growing a second prune list.
/// RFC-0064 R5 D2: boards were found by their own walk with its own rules,
/// which is how a board came to have a descriptor and no announcement.
pub fn is_pruned_dir(name: &str) -> bool {
    PRUNED_DIRS.contains(&name)
        || PRUNED_DIR_PREFIXES
            .iter()
            .any(|prefix| name.starts_with(prefix))
}

/// Marker files that exclude a subtree, honoured as colcon and ament spell them
/// plus our own. Buying the convention: a user who already knows `COLCON_IGNORE`
/// should not have to learn a second spelling to get the same effect.
// issue 0809 — `.nros-ignore` is the spelling that EXISTS. `NROS_IGNORE` was
// this walk's guess at nano-ros's marker; issue 0621 then established
// `.nros-ignore` in `nros-pkg-index` and did not sweep here, so the repo root's
// own `.nros-ignore` — written so a vendored nano-ros stops polluting a
// consumer's package graph — was honoured by one walk and not the other. Both
// spellings are accepted rather than one replaced: `NROS_IGNORE` may exist in a
// consumer tree that predates 0621, and silently dropping support would be the
// same defect pointed the other way.
pub const IGNORE_MARKERS: &[&str] = &[
    "COLCON_IGNORE",
    "AMENT_IGNORE",
    "NROS_IGNORE",
    ".nros-ignore",
];

/// Does `dir` carry a marker opting its subtree out of discovery?
///
/// Shared with the board walk (RFC-0064 R5 D2). Note the caller decides whether
/// the ROOT itself may opt out: `scan_root` checks this only at `depth > 0`,
/// because a root the user named explicitly is a root they meant.
pub fn is_ignored_dir(dir: &Path) -> bool {
    IGNORE_MARKERS.iter().any(|m| dir.join(m).exists())
}

/// A package that announces at least one provision.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProviderPackage {
    /// `<name>` from package.xml.
    pub package: String,
    /// Directory containing the package.xml.
    pub dir: PathBuf,
    /// Which search-path root it was found under, by index. Retained because
    /// shadowing (W5) is decided by root ORDER, and recomputing which root a
    /// path belongs to from the path alone is ambiguous when one root nests
    /// inside another.
    pub root_index: usize,
    /// The provisions it announces.
    pub provides: Vec<Provision>,
    /// `<depend>` entries. Carried because they come free from the same parse
    /// and W4 derives build ORDER from them — re-reading every package.xml to
    /// get them would be a second walk over the same files.
    pub depends: HashSet<String>,
}

impl ProviderPackage {
    /// Where this provider's descriptor would live, given a kind. Not read by
    /// the scan; this is the handoff to selection time.
    pub fn descriptor_path(&self, kind: &str) -> PathBuf {
        self.dir.join(format!("nros-{kind}.toml"))
    }
}

/// A `package.xml` that could not be read or parsed.
///
/// Kept as data rather than aborting the scan: one malformed package.xml
/// somewhere in a large tree must not make every provider undiscoverable. The
/// caller decides — the CLI prints them, and a gate can make them fatal.
#[derive(Debug, Clone)]
pub struct ScanError {
    pub path: PathBuf,
    pub message: String,
}

/// What a scan found.
#[derive(Debug, Clone, Default)]
pub struct ScanResult {
    /// Providers, in search-path order then by path within a root.
    pub providers: Vec<ProviderPackage>,
    /// Unreadable/malformed package.xml files encountered on the way.
    pub errors: Vec<ScanError>,
    /// Every package.xml READ, provider or not, sorted.
    ///
    /// Two jobs. It is the denominator for "the scan looked at N packages and 4
    /// were providers" — without it an empty result cannot be told apart from a
    /// walk that never ran. And it is the cache-invalidation input set (W3):
    /// cmake watches exactly these files, so editing any package.xml — not only
    /// a provider's — re-configures. A non-provider matters because ADDING a
    /// provision to it is precisely the edit that must be noticed.
    pub inputs: Vec<PathBuf>,
}

impl ScanResult {
    /// How many package.xml files the walk read.
    pub fn packages_seen(&self) -> usize {
        self.inputs.len()
    }
}

/// The default search path: the nano-ros tree first, then the user workspace.
///
/// The two-root path — what a caller gets with no configuration at all. It is
/// [`build_search_path`] with an empty `package_paths` and no environment, kept
/// as its own name because most callers have nothing to configure and should
/// not have to say so twice.
///
/// **Both roots live in the user's repo.** That was once the whole story, and
/// the reason given was reproducibility: machine state makes a build
/// irreproducible from the checkout and lets CI diverge from a developer's box.
/// phase-420 W6 widens the path without giving that up — see
/// [`build_search_path`] for how, and for why an environment variable is
/// additive there rather than authoritative.
///
/// When the workspace IS inside the nano-ros tree (building the monorepo's own
/// examples, which is the common case in this repo) the two roots coincide and
/// the second is dropped: scanning one tree twice would report every provider
/// as shadowing itself.
pub fn default_search_path(nano_ros_root: Option<&Path>, workspace: &Path) -> Vec<PathBuf> {
    build_search_path(nano_ros_root, workspace, &[], None).paths()
}

// ===========================================================================
// The search path — phase-420 W6 (RFC-0087 D6)
// ===========================================================================

/// The environment variable that appends roots to the search path.
///
/// colcon's `--base-paths` in source-time form, and named for what it holds
/// rather than for a family: `NROS_RMW_PATH` would need a sibling per kind, and
/// a fifth provider family would then need a fifth variable nobody sets.
pub const PACKAGE_PATH_ENV: &str = "NROS_PACKAGE_PATH";

/// Where a search root came from. Carried because the two questions a user asks
/// about a root — "why is this being scanned" and "why is my provider losing to
/// that one" — are both answered by its origin, and a bare path answers
/// neither.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RootOrigin {
    /// Root 0 — the nano-ros checkout. Not a builtin, merely first.
    NanoRosTree,
    /// The workspace being built.
    Workspace,
    /// `[workspace] package_paths` in the workspace's `nros.toml`.
    PackagePaths,
    /// An entry of [`PACKAGE_PATH_ENV`].
    Env,
    /// `--base-paths`, colcon's flag, which REPLACES the whole path. Its own
    /// origin because a root that arrived this way has no default beneath it:
    /// reporting it as a `Workspace` would imply a nano-ros tree at root 0 that
    /// is not being scanned.
    BasePaths,
}

impl RootOrigin {
    /// How the origin is spelled in `nros ws providers` output — the thing the
    /// user would edit to change it.
    pub fn label(self) -> &'static str {
        match self {
            RootOrigin::NanoRosTree => "nano-ros tree",
            RootOrigin::Workspace => "workspace",
            RootOrigin::PackagePaths => "nros.toml [workspace] package_paths",
            RootOrigin::Env => PACKAGE_PATH_ENV,
            RootOrigin::BasePaths => "--base-paths",
        }
    }

    /// Is a missing root of this origin worth a warning?
    ///
    /// Only for the CONFIGURED origins. The nano-ros tree and the workspace are
    /// legitimately absent — `nros ws providers` in a directory with no
    /// workspace under it is a normal invocation — and warning about them would
    /// print noise on every monorepo build, which is how a warning stops being
    /// read. A root somebody typed is different: nobody types a path they did
    /// not mean to exist, so its absence is a typo or a moved tree, and that is
    /// exactly the "my provider is not found" hour.
    pub fn warns_when_missing(self) -> bool {
        matches!(
            self,
            RootOrigin::PackagePaths | RootOrigin::Env | RootOrigin::BasePaths
        )
    }
}

/// One root on the search path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SearchRoot {
    /// Absolute, `~`-expanded, canonicalized when it exists.
    pub path: PathBuf,
    pub origin: RootOrigin,
    /// Whether `path` is a directory. Recorded rather than filtered — a root
    /// that vanished must still occupy its index, or the numbers in a stored
    /// [`ProviderIndex`] would mean different trees on two machines.
    pub exists: bool,
    /// The entry exactly as authored, when it was not already the final path.
    /// The fix for a typo is an edit to this string, so the message has to be
    /// able to quote it.
    pub as_written: Option<String>,
}

/// An ordered search path, with each root's provenance.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SearchPath {
    pub roots: Vec<SearchRoot>,
}

impl SearchPath {
    /// Just the paths, in order — what [`scan_roots`] and [`ProviderIndex`]
    /// take. Every root is included, missing ones too: see [`SearchRoot::exists`].
    pub fn paths(&self) -> Vec<PathBuf> {
        self.roots.iter().map(|r| r.path.clone()).collect()
    }

    /// The path a `--base-paths` flag names: exactly these roots, in this
    /// order, and nothing else.
    ///
    /// colcon's semantics, and issue 0646's reason for having the flag — a
    /// caller that already knows where the packages are should not pay to
    /// rediscover an underlay per invocation. Nothing else contributes: not the
    /// nano-ros tree, not the config file, not [`PACKAGE_PATH_ENV`]. A flag
    /// that replaced everything EXCEPT the environment would be the hardest
    /// kind of override to reason about.
    pub fn from_base_paths(paths: &[PathBuf]) -> Self {
        let mut out = Self::default();
        for p in paths {
            push_root(&mut out, p.clone(), RootOrigin::BasePaths, None);
        }
        out
    }

    /// The configured roots that are not directories, in order. Empty is the
    /// normal case; anything in it is worth printing.
    pub fn missing(&self) -> Vec<&SearchRoot> {
        self.roots
            .iter()
            .filter(|r| !r.exists && r.origin.warns_when_missing())
            .collect()
    }
}

/// Assemble the ordered search path from every source.
///
/// **Order is precedence, and the order is: nano-ros tree, workspace,
/// `package_paths`, then [`PACKAGE_PATH_ENV`].** [`resolve_unique`] gives the
/// LATER root the win, so this reads underlay → overlay throughout: the
/// nano-ros tree is what everything else overlays, and the environment overlays
/// even the workspace's own committed configuration.
///
/// **The environment APPENDS to the config rather than replacing it, and that
/// is a choice.** colcon's own precedent does not settle it — `COLCON_PREFIX_PATH`
/// has no configuration file competing with it, so it never had to decide. What
/// colcon does have is `--base-paths`, which replaces outright; that is a FLAG,
/// and the distinction is the point. A flag is typed per invocation and its
/// blast radius is one command. An exported variable persists for a shell
/// session and reaches every `nros` and every cmake configure underneath it, so
/// letting it REPLACE a committed `package_paths` would let one developer's
/// shell silently delete a root the repository declares — the same tree
/// building differently on two machines with no diff to look at. That is the
/// "works here, not there" failure this module's doc already names as the
/// expensive kind, and it is the reason `default_search_path` refused an
/// environment variable outright.
///
/// Additive-only answers that objection without giving up the capability. The
/// environment can RAISE a provider's precedence (it lands last, so it wins)
/// but it can never make a configured root disappear; the search path is
/// recorded in the [`ProviderIndex`], so an index built under a different
/// environment is rejected on read rather than served; and every winner is
/// printed with the root it came from. "Replace it all" keeps its verb —
/// `nros sync --base-paths`, which is a flag, exactly like colcon's.
///
/// **Relative entries resolve against the WORKSPACE, never the cwd.** RFC-0087
/// D6's own example is `package_paths = ["src", …]`, which plainly means
/// `<ws>/src`; `nros.toml` sits at the workspace root, so that is the directory
/// the author was writing relative to. A cwd-relative reading would make
/// `nros ws providers` answer differently depending on where it was invoked
/// from — issue 0979's shape one module over, where a search path computed
/// against the cwd produced an empty tree and a message naming the wrong thing.
/// [`PACKAGE_PATH_ENV`] resolves the same way so the two sources cannot mean
/// different things by one string.
///
/// **`~` expands, `~user` does not.** `~` and `~/…` take `$HOME`. `~user` needs
/// a passwd lookup, and left literal it becomes a missing root that says so,
/// which beats resolving to something the author did not name.
///
/// **A repeated root is one root, and the FIRST occurrence keeps the index.**
/// Scanning one tree twice reports every provider in it as shadowing itself —
/// the noise `default_search_path` already avoids for the nested-workspace
/// case. Keeping the first occurrence means adding an entry cannot renumber the
/// roots before it, so `root[0]` is the nano-ros tree in every invocation.
/// Repeating a root therefore cannot raise its precedence; to raise it, remove
/// the earlier entry.
pub fn build_search_path(
    nano_ros_root: Option<&Path>,
    workspace: &Path,
    package_paths: &[String],
    env_value: Option<&str>,
) -> SearchPath {
    let mut out = SearchPath::default();

    if let Some(r) = nano_ros_root {
        push_root(&mut out, r.to_path_buf(), RootOrigin::NanoRosTree, None);
    }

    // The nested case: a workspace INSIDE the nano-ros tree is already covered
    // by root 0. Kept as `starts_with` rather than folded into `push_root`'s
    // equality dedup, because containment and identity are different questions
    // and only the default pair is known to nest.
    let ws = workspace.to_path_buf();
    if !out
        .roots
        .iter()
        .any(|r| ws == r.path || ws.starts_with(&r.path))
    {
        push_root(&mut out, ws, RootOrigin::Workspace, None);
    }

    for entry in package_paths {
        if entry.trim().is_empty() {
            continue;
        }
        let path = resolve_entry(entry, workspace);
        push_root(
            &mut out,
            path,
            RootOrigin::PackagePaths,
            Some(entry.clone()),
        );
    }

    for entry in env_value.into_iter().flat_map(|v| v.split(':')) {
        if entry.trim().is_empty() {
            continue;
        }
        let path = resolve_entry(entry, workspace);
        push_root(&mut out, path, RootOrigin::Env, Some(entry.to_string()));
    }

    out
}

/// Turn one authored entry into an absolute path: `~` expansion, then
/// workspace-relative resolution, then canonicalization if it exists.
///
/// Canonicalization only when the directory is there — there is nothing to
/// canonicalize otherwise, and a missing root must keep the spelling that
/// appears in the warning so the reader recognises what they typed.
fn resolve_entry(entry: &str, workspace: &Path) -> PathBuf {
    let expanded = expand_tilde(entry);
    let abs = if expanded.is_absolute() {
        expanded
    } else {
        workspace.join(expanded)
    };
    abs.canonicalize().unwrap_or(abs)
}

/// `~` / `~/…` against `$HOME`. Anything else, including `~user`, is returned
/// unchanged — see [`build_search_path`].
fn expand_tilde(entry: &str) -> PathBuf {
    let rest = if entry == "~" {
        ""
    } else if let Some(r) = entry.strip_prefix("~/") {
        r
    } else {
        return PathBuf::from(entry);
    };
    match std::env::var_os("HOME") {
        Some(home) if !home.is_empty() => {
            let home = PathBuf::from(home);
            if rest.is_empty() {
                home
            } else {
                home.join(rest)
            }
        }
        // No `$HOME` to expand against. Left literal so it surfaces as a
        // missing root naming `~/...`, rather than silently becoming a
        // relative path under the workspace.
        _ => PathBuf::from(entry),
    }
}

/// Append a root unless an earlier one is the same directory.
fn push_root(out: &mut SearchPath, path: PathBuf, origin: RootOrigin, as_written: Option<String>) {
    if out.roots.iter().any(|r| r.path == path) {
        return;
    }
    let exists = path.is_dir();
    let as_written = as_written.filter(|w| Path::new(w) != path);
    out.roots.push(SearchRoot {
        path,
        origin,
        exists,
        as_written,
    });
}

/// Scan an ordered search path. Earlier roots come first in the result.
pub fn scan_roots(roots: &[PathBuf]) -> Result<ScanResult> {
    let mut out = ScanResult::default();
    for (root_index, root) in roots.iter().enumerate() {
        let one = scan_root(root, root_index)
            .wrap_err_with(|| format!("scanning provider root {}", root.display()))?;
        out.providers.extend(one.providers);
        out.errors.extend(one.errors);
        out.inputs.extend(one.inputs);
    }
    Ok(out)
}

/// Scan a single root. A root that does not exist yields nothing rather than an
/// error: the user-workspace entry of the default search path is legitimately
/// absent when someone builds nano-ros on its own.
pub fn scan_root(root: &Path, root_index: usize) -> Result<ScanResult> {
    let mut out = ScanResult::default();
    walk_packages(root, &mut out, |dir, pkg, out| {
        if !pkg.provides.is_empty() {
            out.providers.push(ProviderPackage {
                package: pkg.name.clone(),
                dir: dir.to_path_buf(),
                root_index,
                provides: pkg.provides.clone(),
                depends: pkg.dependencies.clone(),
            });
        }
    })?;

    // `stack.pop()` makes the walk order depend on read_dir order, which is
    // filesystem-dependent. Sort so the result is reproducible across machines
    // — W5 reports ambiguity by listing paths, and an unstable order would make
    // that message differ between hosts for the same tree.
    out.providers.sort_by(|a, b| a.dir.cmp(&b.dir));
    out.errors.sort_by(|a, b| a.path.cmp(&b.path));
    out.inputs.sort();
    Ok(out)
}

/// The shared tree walk: find every `package.xml` under `root`, parse it once,
/// and hand it to `visit`.
///
/// Factored out because phase-348 has two consumers with different questions —
/// W1/W3 want "who provides what", W4 wants "every package and its depends" —
/// and walking twice would double the I/O while letting the two disagree about
/// which directories are pruned. Parse failures and unreadable directories
/// accumulate in `out.errors` either way.
fn walk_packages(
    root: &Path,
    out: &mut ScanResult,
    mut visit: impl FnMut(&Path, &PackageXml, &mut ScanResult),
) -> Result<()> {
    if !root.is_dir() {
        return Ok(());
    }

    // `(dir, depth)` — depth exists ONLY to exempt the root from the ignore
    // markers, matching `nros_pkg_index::build_pkg_index`, whose walkdir filter
    // returns `true` for `entry.depth() == 0` before any marker is read.
    //
    // issue 1054 — this walk checked the markers on every directory it popped,
    // the root included, so `scan_roots(&[<nano-ros root>])` returned ZERO
    // packages: the repository root carries its own `.nros-ignore` (issue
    // 0621), whose header states the opposite contract in as many words — it
    // "prunes the whole tree from any walk that starts ABOVE it" and "does NOT
    // affect nano-ros's own discovery". `nros-pkg-index` honoured that; issue
    // 0809 taught this walk the `.nros-ignore` SPELLING and did not carry the
    // depth-0 exemption that makes the spelling safe. Two walks over one marker
    // vocabulary must not mean two different things.
    //
    // The rule, stated positively: a marker prunes DESCENDANTS, never the root
    // the caller named. A caller that hands us a directory has already decided
    // to scan it, and a marker inside it cannot overrule that decision — it is
    // addressed to walks from above.
    let mut stack = vec![(root.to_path_buf(), 0usize)];
    while let Some((dir, depth)) = stack.pop() {
        if depth > 0 && IGNORE_MARKERS.iter().any(|m| dir.join(m).exists()) {
            continue;
        }

        let manifest = dir.join("package.xml");
        if manifest.is_file() {
            out.inputs.push(manifest.clone());
            match PackageXml::parse(&manifest) {
                Ok(pkg) => visit(&dir, &pkg, out),
                Err(e) => out.errors.push(ScanError {
                    path: manifest,
                    message: format!("{e:#}"),
                }),
            }
            // Do not descend into a package, as colcon does not. A package's
            // own subdirectories are its sources, and anything package-shaped
            // inside one is a fixture or a copy rather than a sibling provider.
            continue;
        }

        let entries = match std::fs::read_dir(&dir) {
            Ok(e) => e,
            // An unreadable directory is worth reporting but not fatal — a
            // permission-denied subtree must not hide every provider above it.
            Err(e) => {
                out.errors.push(ScanError {
                    path: dir.clone(),
                    message: format!("read_dir: {e}"),
                });
                continue;
            }
        };
        for entry in entries.flatten() {
            let path = entry.path();
            // `file_type()` rather than `is_dir()`: symlinks are not followed,
            // so a link pointing at an ancestor cannot make the walk loop.
            let Ok(ft) = entry.file_type() else { continue };
            if !ft.is_dir() {
                continue;
            }
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name.starts_with('.') || is_pruned_dir(name.as_ref()) {
                continue;
            }
            stack.push((path, depth + 1));
        }
    }
    Ok(())
}

// ===========================================================================
// The package graph — phase-348 W4
// ===========================================================================

/// A package in a workspace, whether or not it provides anything.
///
/// Distinct from [`ProviderPackage`], which is only the ones announcing a
/// provision: build ORDER is a property of every package, and an entry that
/// composes two node packages provides nothing at all while still having to be
/// configured after them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WorkspacePackage {
    pub name: String,
    pub dir: PathBuf,
    /// Every `<depend>` / `<*_depend>` entry, verbatim. Includes names that are
    /// not workspace packages (`std_msgs`); ordering ignores those rather than
    /// failing, since an external dependency imposes no local build order.
    pub depends: HashSet<String>,
}

/// Every package under `root`, in path order.
pub fn scan_workspace_packages(root: &Path) -> Result<(Vec<WorkspacePackage>, ScanResult)> {
    let mut out = ScanResult::default();
    let mut pkgs = Vec::new();
    walk_packages(root, &mut out, |dir, pkg, _out| {
        pkgs.push(WorkspacePackage {
            name: pkg.name.clone(),
            dir: dir.to_path_buf(),
            depends: pkg.dependencies.clone(),
        });
    })?;
    pkgs.sort_by(|a, b| a.dir.cmp(&b.dir));
    out.errors.sort_by(|a, b| a.path.cmp(&b.path));
    out.inputs.sort();
    Ok((pkgs, out))
}

/// A dependency cycle, reported as the names on it.
#[derive(Debug, Clone)]
pub struct DependencyCycle(pub Vec<String>);

impl std::fmt::Display for DependencyCycle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0.join(" -> "))
    }
}

/// Order `pkgs` so every package follows the workspace packages it depends on.
///
/// **Deterministic.** Among packages whose dependencies are all satisfied, the
/// one sorting first by NAME goes next. A topological sort has many valid
/// answers, and an order that varied by filesystem or hash iteration would make
/// a build reproducible only by luck — and would make a diff of the emitted
/// order unreadable.
///
/// **Dependencies outside the workspace are ignored, not errors.** `<depend>`
/// lists message packages (`std_msgs`) and system packages that impose no local
/// build order; treating an unknown name as a missing package would reject
/// every real workspace.
///
/// A cycle is an error naming the packages on it. Emitting a partial order
/// instead would produce a build that fails somewhere downstream with no clue
/// as to why.
pub fn topological_order(
    pkgs: &[WorkspacePackage],
) -> std::result::Result<Vec<WorkspacePackage>, DependencyCycle> {
    topological_order_with_priority(pkgs, &[])
}

/// [`topological_order`], but ties break by a caller-supplied preference first.
///
/// `priority` is a list of package directories in the order the caller already
/// wants — a workspace's authored `SUBDIRS` list. Among packages whose
/// dependencies are all satisfied, the one appearing earliest in `priority`
/// goes next; anything absent from it falls back to name order.
///
/// **This is what makes adopting derived ordering safe.** Every workspace's
/// authored list is already a working order, and most packages have no
/// dependency relation to each other at all — in a workspace where the entry
/// packages declare no `<exec_depend>`, a pure name-sorted topological order is
/// free to interleave an entry between two node packages and break the build,
/// even though it violates no declared constraint. Preferring the authored
/// order means the sort can only ever MOVE a package that a declared dependency
/// requires moving. It fixes what is stated and preserves what is not, so
/// turning it on cannot regress a workspace that has not declared its deps yet.
pub fn topological_order_with_priority(
    pkgs: &[WorkspacePackage],
    priority: &[PathBuf],
) -> std::result::Result<Vec<WorkspacePackage>, DependencyCycle> {
    let rank = |p: &WorkspacePackage| -> (usize, String) {
        match priority.iter().position(|d| *d == p.dir) {
            Some(i) => (i, String::new()),
            None => (usize::MAX, p.name.clone()),
        }
    };
    topo_inner(pkgs, &rank)
}

fn topo_inner(
    pkgs: &[WorkspacePackage],
    rank: &dyn Fn(&WorkspacePackage) -> (usize, String),
) -> std::result::Result<Vec<WorkspacePackage>, DependencyCycle> {
    let local: HashSet<&str> = pkgs.iter().map(|p| p.name.as_str()).collect();

    // name -> the workspace packages it must follow.
    let mut pending: Vec<(&WorkspacePackage, HashSet<&str>)> = pkgs
        .iter()
        .map(|p| {
            let deps: HashSet<&str> = p
                .depends
                .iter()
                .map(String::as_str)
                .filter(|d| local.contains(d) && *d != p.name)
                .collect();
            (p, deps)
        })
        .collect();

    let mut done: HashSet<&str> = HashSet::new();
    let mut order: Vec<WorkspacePackage> = Vec::with_capacity(pkgs.len());

    while !pending.is_empty() {
        let mut ready: Vec<usize> = pending
            .iter()
            .enumerate()
            .filter(|(_, (_, deps))| deps.iter().all(|d| done.contains(d)))
            .map(|(i, _)| i)
            .collect();

        if ready.is_empty() {
            // Everything left is on, or behind, a cycle. Report the remaining
            // names sorted — naming all of them beats naming one arbitrary
            // edge, because the author has to look at the whole knot anyway.
            let mut names: Vec<String> = pending.iter().map(|(p, _)| p.name.clone()).collect();
            names.sort();
            return Err(DependencyCycle(names));
        }

        // Deterministic tie-break: caller preference, then name.
        ready.sort_by_key(|&i| rank(pending[i].0));
        let next = ready[0];
        let (pkg, _) = pending.remove(next);
        done.insert(pkg.name.as_str());
        order.push(pkg.clone());
    }

    Ok(order)
}

// ===========================================================================
// Resolution and shadowing — phase-348 W5
// ===========================================================================

/// Which provider won a `(kind, name)` lookup, and what it shadowed.
///
/// `shadowed` is retained rather than discarded — ESP-IDF keeps both a
/// documented precedence order (`COMPONENT_SOURCE`) and the losing path
/// (`COMPONENT_OVERRIDEN_DIR`) so the winner can reach back into the loser, and
/// notes that last-write-wins with no recorded provenance would be unusable.
/// The same applies here: "why is my patched backend not being used" is
/// answerable only if the loser is still nameable.
#[derive(Debug, Clone)]
pub struct Resolution {
    pub winner: ProviderPackage,
    /// Lower-precedence claimants, nearest-loser first.
    pub shadowed: Vec<ProviderPackage>,
}

impl Resolution {
    pub fn is_shadowing(&self) -> bool {
        !self.shadowed.is_empty()
    }
}

#[derive(Debug, Clone)]
pub enum ResolveError {
    /// Two or more packages in the SAME root claim one name. Unlike shadowing,
    /// there is no precedence to appeal to.
    Ambiguous {
        kind: String,
        name: String,
        root_index: usize,
        candidates: Vec<PathBuf>,
    },
    NotFound {
        kind: String,
        name: String,
        /// Every name of this kind that IS claimed, sorted — an unknown name is
        /// usually a typo, and the list is the fix.
        available: Vec<String>,
    },
}

impl std::fmt::Display for ResolveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ResolveError::Ambiguous {
                kind,
                name,
                root_index,
                candidates,
            } => {
                write!(
                    f,
                    "{kind} `{name}` is claimed by {} packages in the SAME search \
                     root (root[{root_index}]), so there is no precedence between \
                     them:",
                    candidates.len()
                )?;
                for c in candidates {
                    write!(f, "\n  {}", c.display())?;
                }
                write!(
                    f,
                    "\n  Rename one, or — if they are genuinely different targets \
                     that share a name — disambiguate them in their descriptors \
                     and resolve with `candidates()` instead."
                )
            }
            ResolveError::NotFound {
                kind,
                name,
                available,
            } => {
                write!(f, "no {kind} provider named `{name}`.")?;
                if available.is_empty() {
                    write!(f, " No {kind} providers were discovered at all.")
                } else {
                    write!(f, " Available: {}", available.join(", "))
                }
            }
        }
    }
}

impl std::error::Error for ResolveError {}

/// Every provider claiming `(kind, name)`, highest precedence first.
///
/// For callers that have their OWN discriminator and so must see all of them —
/// board resolution is the live case: `threadx` is legitimately claimed by both
/// `nros-board-threadx-linux` and `nros-board-threadx-qemu-riscv64`, separated
/// by the descriptor's `target_contains`. A flat "two packages, one name is an
/// error" rule would reject a shipping arrangement (phase-348 W2 finding).
pub fn candidates<'a>(scan: &'a ScanResult, kind: &str, name: &str) -> Vec<&'a ProviderPackage> {
    let mut hits: Vec<&ProviderPackage> = scan
        .providers
        .iter()
        .filter(|p| {
            p.provides
                .iter()
                .any(|pr| pr.kind == kind && pr.name == name)
        })
        .collect();
    // LATER roots win — see `resolve_unique`.
    hits.sort_by(|a, b| b.root_index.cmp(&a.root_index).then(a.dir.cmp(&b.dir)));
    hits
}

/// Resolve `(kind, name)` to exactly one provider.
///
/// **A LATER search root overlays an earlier one.** The search path is ordered
/// `[nano-ros tree, user workspace]`, so the user's package wins — colcon's
/// overlay-beats-underlay rule, and the whole point of allowing a workspace
/// provider at all (testing a patched backend against the rest of the tree).
///
/// Note this is the OPPOSITE of `AMENT_PREFIX_PATH`, where the overlay is
/// listed first. Ours reads underlay → overlay because the nano-ros tree is
/// root 0 by construction (it is not a special case, merely the first entry),
/// and re-ordering the list to put the user first would make "root[0]" mean
/// different trees in different invocations.
///
/// Ambiguity WITHIN one root is an error: precedence between roots is defined,
/// precedence within a root is not.
pub fn resolve_unique(
    scan: &ScanResult,
    kind: &str,
    name: &str,
) -> std::result::Result<Resolution, ResolveError> {
    let hits = candidates(scan, kind, name);
    let Some(first) = hits.first() else {
        let mut available: Vec<String> = scan
            .providers
            .iter()
            .flat_map(|p| p.provides.iter())
            .filter(|pr| pr.kind == kind)
            .map(|pr| pr.name.clone())
            .collect();
        available.sort();
        available.dedup();
        return Err(ResolveError::NotFound {
            kind: kind.to_string(),
            name: name.to_string(),
            available,
        });
    };

    let top_root = first.root_index;
    let tied: Vec<&ProviderPackage> = hits
        .iter()
        .copied()
        .filter(|p| p.root_index == top_root)
        .collect();
    if tied.len() > 1 {
        return Err(ResolveError::Ambiguous {
            kind: kind.to_string(),
            name: name.to_string(),
            root_index: top_root,
            candidates: tied.iter().map(|p| p.dir.clone()).collect(),
        });
    }

    Ok(Resolution {
        winner: (*first).clone(),
        shadowed: hits.iter().skip(1).map(|p| (*p).clone()).collect(),
    })
}

/// One `(kind, name)` claimed by more than one provider, in precedence order.
///
/// The reporting shape for RFC-0087 D6's "shadowing is reported, never
/// silent". [`resolve_unique`] answers one lookup and either wins or refuses;
/// this enumerates every contested pair in a scan, so a listing can annotate
/// the packages it is already printing instead of the reader having to
/// `--resolve` each name to find out whether it was contested.
#[derive(Debug, Clone)]
pub struct Shadowing {
    pub kind: String,
    pub name: String,
    /// Highest precedence — the provider a build will use, UNLESS
    /// [`Shadowing::same_root_tie`] is set, in which case there is no winner
    /// and [`resolve_unique`] refuses.
    pub winner: ProviderPackage,
    /// The losers, nearest first.
    pub shadowed: Vec<ProviderPackage>,
    /// The top two claimants share a root, so precedence cannot separate them.
    /// Recorded rather than dropped: printing a `winner` for a pair that
    /// `resolve_unique` will refuse would be a listing that disagrees with the
    /// build, which is worse than not listing it.
    pub same_root_tie: bool,
}

/// Every contested `(kind, name)` in a scan, sorted by kind then name.
///
/// Cross-root contention is NORMAL and resolvable — it is what a search path is
/// for, and the loser is named rather than dropped. Same-root contention is not
/// (`ResolveError::Ambiguous`), and is reported here too because a listing that
/// showed only the resolvable half would let the unresolvable one through
/// silently.
pub fn shadowing(scan: &ScanResult) -> Vec<Shadowing> {
    let mut pairs: Vec<(String, String)> = scan
        .providers
        .iter()
        .flat_map(|p| p.provides.iter())
        .map(|pr| (pr.kind.clone(), pr.name.clone()))
        .collect();
    pairs.sort();
    pairs.dedup();

    pairs
        .into_iter()
        .filter_map(|(kind, name)| {
            let hits = candidates(scan, &kind, &name);
            // `candidates` orders by precedence, so `hits[0]` is the winner and
            // the rest are shadowed — the same ordering `resolve_unique` uses,
            // because two orderings of one fact is how a report starts
            // disagreeing with the build.
            let (first, rest) = hits.split_first()?;
            if rest.is_empty() {
                return None;
            }
            let same_root_tie = rest[0].root_index == first.root_index;
            Some(Shadowing {
                kind,
                name,
                winner: (*first).clone(),
                shadowed: rest.iter().map(|p| (*p).clone()).collect(),
                same_root_tie,
            })
        })
        .collect()
}

// ===========================================================================
// The index — phase-348 W3
// ===========================================================================

/// Bumped when the on-disk shape changes. A reader that finds a version it does
/// not know REGENERATES rather than guessing: an index is a cache, and a cache
/// that silently misinterprets an old layout is worse than no cache.
pub const INDEX_VERSION: u32 = 1;

/// A scan result written to disk so it is not recomputed per configure.
///
/// Purely a CACHE — every field is rederivable by rescanning, and nothing may
/// depend on the file existing. That is what makes it safe for cmake to read
/// and for a gate to delete.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProviderIndex {
    pub version: u32,
    /// The search path this was produced from, in order. Recorded because an
    /// index built for a DIFFERENT search path is not stale, it is wrong —
    /// [`ProviderIndex::is_valid_for`] rejects it rather than serving it.
    pub roots: Vec<PathBuf>,
    /// Every package.xml read, sorted. cmake attaches these to
    /// `CMAKE_CONFIGURE_DEPENDS` so editing one re-configures.
    pub inputs: Vec<PathBuf>,
    pub providers: Vec<ProviderPackage>,
}

impl ProviderIndex {
    pub fn from_scan(roots: &[PathBuf], scan: &ScanResult) -> Self {
        Self {
            version: INDEX_VERSION,
            roots: roots.to_vec(),
            inputs: scan.inputs.clone(),
            providers: scan.providers.clone(),
        }
    }

    /// Whether this index answers questions about `roots`.
    pub fn is_valid_for(&self, roots: &[PathBuf]) -> bool {
        self.version == INDEX_VERSION && self.roots == roots
    }

    /// Write atomically — via a temp file in the same directory, then rename.
    ///
    /// A half-written index is a JSON parse error at someone else's configure,
    /// and the two writers here (`nros sync` and a cmake configure) can run
    /// concurrently on one tree. Same reasoning as `check-atomic-sync-writes`
    /// enforces for sync's other outputs.
    pub fn write(&self, path: &Path) -> Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .wrap_err_with(|| format!("creating {}", parent.display()))?;
        }
        let body = serde_json::to_string_pretty(self)? + "\n";
        crate::atomic_file::atomic_write(path, &body)
    }

    pub fn read(path: &Path) -> Result<Self> {
        let body = std::fs::read_to_string(path)
            .wrap_err_with(|| format!("reading provider index {}", path.display()))?;
        let idx: Self = serde_json::from_str(&body)
            .wrap_err_with(|| format!("parsing provider index {}", path.display()))?;
        if idx.version != INDEX_VERSION {
            bail!(
                "provider index {} is version {} but this nros speaks {} — \
                 delete it and re-run `nros sync`",
                path.display(),
                idx.version,
                INDEX_VERSION
            );
        }
        Ok(idx)
    }

    /// One line per provision, `kind\tname\tpackage\troot_index\tdir`.
    ///
    /// The machine-readable seam for cmake, which has no JSON parser. Same role
    /// as `nros ws model-dims`: cmake ASKS rather than re-implementing the read,
    /// so there is never a second parser of this file to drift.
    pub fn to_lines(&self) -> String {
        let mut out = String::new();
        for p in &self.providers {
            for pr in &p.provides {
                out.push_str(&format!(
                    "{}\t{}\t{}\t{}\t{}\n",
                    pr.kind,
                    pr.name,
                    p.package,
                    p.root_index,
                    p.dir.display()
                ));
            }
        }
        out
    }
}

/// How a freshly-scanned tree differs from an index. Empty ⇒ the index is
/// current.
///
/// This exists because watching the recorded `inputs` cannot catch the case
/// that matters most: a package.xml that did not exist when the index was
/// written is in nobody's watch list. That is issue 0196's exact shape — a
/// probe whose inputs never include the thing that breaks it — so the answer is
/// an explicit rescan-and-compare rather than a cleverer file watch.
#[derive(Debug, Default)]
pub struct IndexDiff {
    /// package.xml paths present now, absent from the index.
    pub added_inputs: Vec<PathBuf>,
    /// Recorded in the index, gone from the tree.
    pub removed_inputs: Vec<PathBuf>,
    /// `kind:name -> dir` provisions that appeared, vanished, or moved.
    pub changed_provisions: Vec<String>,
}

impl IndexDiff {
    pub fn is_empty(&self) -> bool {
        self.added_inputs.is_empty()
            && self.removed_inputs.is_empty()
            && self.changed_provisions.is_empty()
    }
}

/// Compare an index against a fresh scan of the same roots.
pub fn diff_index(index: &ProviderIndex, fresh: &ScanResult) -> IndexDiff {
    let old_inputs: HashSet<&PathBuf> = index.inputs.iter().collect();
    let new_inputs: HashSet<&PathBuf> = fresh.inputs.iter().collect();

    let provision_set = |ps: &[ProviderPackage]| -> HashSet<String> {
        ps.iter()
            .flat_map(|p| {
                let dir = p.dir.display().to_string();
                p.provides
                    .iter()
                    .map(move |pr| format!("{}:{} -> {}", pr.kind, pr.name, dir))
            })
            .collect()
    };
    let old_p = provision_set(&index.providers);
    let new_p = provision_set(&fresh.providers);

    let mut changed: Vec<String> = new_p
        .difference(&old_p)
        .map(|s| format!("+ {s}"))
        .chain(old_p.difference(&new_p).map(|s| format!("- {s}")))
        .collect();
    changed.sort();

    let mut added: Vec<PathBuf> = new_inputs
        .difference(&old_inputs)
        .map(|p| (*p).clone())
        .collect();
    let mut removed: Vec<PathBuf> = old_inputs
        .difference(&new_inputs)
        .map(|p| (*p).clone())
        .collect();
    added.sort();
    removed.sort();

    IndexDiff {
        added_inputs: added,
        removed_inputs: removed,
        changed_provisions: changed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn write(path: &Path, body: &str) {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, body).unwrap();
    }

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

    fn plain_xml(name: &str) -> String {
        format!(
            r#"<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <depend>std_msgs</depend>
</package>"#
        )
    }

    /// The W1 acceptance criterion, both halves: a package with the export is
    /// listed, one without is not.
    #[test]
    fn scan_lists_providers_and_only_providers() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/my_rmw/package.xml"),
            &provider_xml("my_rmw", "rmw", "acme"),
        );
        write(&root.join("src/my_node/package.xml"), &plain_xml("my_node"));

        let r = scan_root(root, 0).unwrap();
        assert_eq!(r.packages_seen(), 2, "both packages were looked at");
        assert_eq!(r.providers.len(), 1);
        assert_eq!(r.providers[0].package, "my_rmw");
        assert_eq!(r.providers[0].provides[0].name, "acme");
        assert!(r.errors.is_empty());
    }

    #[test]
    fn search_path_order_is_preserved_and_root_recorded() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        write(
            &a.path().join("packages/rmw/zenoh/package.xml"),
            &provider_xml("nros_rmw_zenoh", "rmw", "zenoh"),
        );
        write(
            &b.path().join("src/patched_zenoh/package.xml"),
            &provider_xml("patched_zenoh", "rmw", "zenoh"),
        );

        let r = scan_roots(&[a.path().to_path_buf(), b.path().to_path_buf()]).unwrap();
        assert_eq!(r.providers.len(), 2, "the scan reports BOTH");
        assert_eq!(r.providers[0].root_index, 0);
        assert_eq!(r.providers[1].root_index, 1);
        // Resolving the collision is W5's job, not the scan's; the scan must
        // not silently drop either one, which is what makes W5 able to warn
        // with both paths.
        assert_eq!(
            r.providers[0].provides[0].name,
            r.providers[1].provides[0].name
        );
    }

    #[test]
    fn ignore_markers_and_pruned_dirs_are_skipped() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/ignored/package.xml"),
            &provider_xml("ignored", "rmw", "nope"),
        );
        write(&root.join("src/ignored/COLCON_IGNORE"), "");
        write(
            &root.join("build/stale/package.xml"),
            &provider_xml("stale", "rmw", "stale"),
        );
        write(
            &root.join("third-party/vendored/package.xml"),
            &provider_xml("vendored", "rmw", "vendored"),
        );
        write(
            &root.join("src/real/package.xml"),
            &provider_xml("real", "rmw", "real"),
        );

        let r = scan_root(root, 0).unwrap();
        let names: Vec<_> = r.providers.iter().map(|p| p.package.as_str()).collect();
        assert_eq!(names, vec!["real"]);
        assert_eq!(r.packages_seen(), 1, "the skipped ones were never parsed");
    }

    /// issue 0809 — the dotted spelling is the one that EXISTS on disk.
    ///
    /// The repo root's own `.nros-ignore` (issue 0621, so a vendored nano-ros
    /// stops polluting a consumer's package graph) was honoured by
    /// `nros-pkg-index` and NOT by this walk, which had guessed `NROS_IGNORE`.
    /// Both are accepted now: a consumer tree predating 0621 may carry the
    /// undotted one, and dropping it would be the same defect reversed.
    #[test]
    fn both_nros_ignore_spellings_are_honoured() {
        for marker in [".nros-ignore", "NROS_IGNORE"] {
            let tmp = tempfile::tempdir().unwrap();
            let root = tmp.path();
            write(
                &root.join("vendored/nano-ros/packages/x/package.xml"),
                &provider_xml("vendored_pkg", "rmw", "nope"),
            );
            write(&root.join("vendored/nano-ros").join(marker), "");
            write(
                &root.join("src/real/package.xml"),
                &provider_xml("real", "rmw", "real"),
            );

            let r = scan_root(root, 0).unwrap();
            let names: Vec<_> = r.providers.iter().map(|p| p.package.as_str()).collect();
            assert_eq!(
                names,
                vec!["real"],
                "`{marker}` must stop the walk descending"
            );
        }
    }

    /// issue 1054 — a marker on the root the caller NAMED does not prune it.
    ///
    /// This is the depth-0 exemption `nros_pkg_index::build_pkg_index` has had
    /// since issue 0621 (`entry.depth() == 0` returns true before any marker is
    /// read), and which issue 0809 did not carry across when it taught this
    /// walk the `.nros-ignore` spelling. The concrete casualty: the nano-ros
    /// repository root carries `.nros-ignore`, so `scan_roots(&[<nano-ros
    /// root>])` — root 0 of `default_search_path` — reported ZERO packages and
    /// therefore zero providers of EVERY family, silently.
    ///
    /// A caller that hands this walk a directory has already decided to scan
    /// it; the marker is addressed to walks that start ABOVE it, which is what
    /// `.nros-ignore`'s own header says in as many words.
    #[test]
    fn the_root_is_scanned_despite_its_own_ignore_marker() {
        for marker in IGNORE_MARKERS {
            let tmp = tempfile::tempdir().unwrap();
            let root = tmp.path();
            write(&root.join(marker), "");
            write(
                &root.join("packages/rmw/zenoh/package.xml"),
                &provider_xml("nros_rmw_zenoh", "rmw", "zenoh"),
            );
            write(&root.join("src/plain/package.xml"), &plain_xml("plain"));

            let r = scan_root(root, 0).unwrap();
            assert_eq!(
                r.packages_seen(),
                2,
                "`{marker}` on the root must not stop the walk that STARTS there"
            );
            let names: Vec<_> = r.providers.iter().map(|p| p.package.as_str()).collect();
            assert_eq!(names, vec!["nros_rmw_zenoh"]);
        }
    }

    /// The other half of the same rule: exempting the root exempts ONLY the
    /// root. A marker one level down still prunes, so the exemption cannot be
    /// mistaken for "the markers are advisory".
    #[test]
    fn a_marker_below_the_root_still_prunes_even_when_the_root_has_one_too() {
        for marker in IGNORE_MARKERS {
            let tmp = tempfile::tempdir().unwrap();
            let root = tmp.path();
            // The root carries a marker AND so does a nested subtree: the first
            // is exempt, the second is not.
            write(&root.join(".nros-ignore"), "");
            write(&root.join("vendored").join(marker), "");
            write(
                &root.join("vendored/nano-ros/packages/rmw/zenoh/package.xml"),
                &provider_xml("vendored_rmw", "rmw", "vendored"),
            );
            write(
                &root.join("src/real/package.xml"),
                &provider_xml("real", "rmw", "real"),
            );

            let r = scan_root(root, 0).unwrap();
            let names: Vec<_> = r.providers.iter().map(|p| p.package.as_str()).collect();
            assert_eq!(
                names,
                vec!["real"],
                "`{marker}` at depth 1 must still prune its subtree"
            );
            assert_eq!(
                r.packages_seen(),
                1,
                "the pruned package.xml was never parsed"
            );
        }
    }

    /// issue 0621, the case the marker file exists FOR: a consumer vendors
    /// nano-ros as a subdirectory and scans their own root. nano-ros's 272+
    /// packages are not part of their graph, and the marker is what keeps them
    /// out. The depth-0 exemption must not weaken this — here the walk starts
    /// ABOVE the marker, which is exactly when it bites.
    #[test]
    fn a_vendored_nano_ros_is_pruned_when_the_walk_starts_above_it() {
        let tmp = tempfile::tempdir().unwrap();
        let consumer = tmp.path();
        write(
            &consumer.join("src/my_app/package.xml"),
            &plain_xml("my_app"),
        );
        // The vendored tree, with its own marker at its root — the shape the
        // real repository has.
        write(&consumer.join("nano-ros/.nros-ignore"), "vendored\n");
        for dup in ["templates/a", "templates/b"] {
            write(
                &consumer
                    .join("nano-ros/examples")
                    .join(dup)
                    .join("src/demo_bringup/package.xml"),
                &plain_xml("demo_bringup"),
            );
        }
        write(
            &consumer.join("nano-ros/packages/rmw/zenoh/package.xml"),
            &provider_xml("nros_rmw_zenoh", "rmw", "zenoh"),
        );

        let (pkgs, scan) = scan_workspace_packages(consumer).unwrap();
        let names: Vec<_> = pkgs.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(
            names,
            vec!["my_app"],
            "the vendored tree must stay out of the consumer's package graph"
        );
        assert_eq!(scan.packages_seen(), 1);

        // …and scanning the vendored tree DIRECTLY still finds everything in
        // it. Same marker, same tree, different starting point: that asymmetry
        // is the whole contract.
        let inner = scan_root(&consumer.join("nano-ros"), 0).unwrap();
        assert_eq!(inner.packages_seen(), 3);
        let inner_names: Vec<_> = inner.providers.iter().map(|p| p.package.as_str()).collect();
        assert_eq!(inner_names, vec!["nros_rmw_zenoh"]);
    }

    /// A malformed package.xml is reported without taking the scan down with
    /// it. The failure mode this prevents: one broken file in a large tree
    /// making every provider in it undiscoverable.
    #[test]
    fn malformed_package_xml_is_reported_not_fatal() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(&root.join("src/broken/package.xml"), "<package><name>");
        write(
            &root.join("src/good/package.xml"),
            &provider_xml("good", "rmw", "good"),
        );

        let r = scan_root(root, 0).unwrap();
        assert_eq!(r.providers.len(), 1, "the good one is still found");
        assert_eq!(r.errors.len(), 1);
        assert!(r.errors[0].path.ends_with("src/broken/package.xml"));
    }

    #[test]
    fn scan_does_not_descend_into_a_package() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/outer/package.xml"),
            &provider_xml("outer", "rmw", "outer"),
        );
        write(
            &root.join("src/outer/test/fixture/package.xml"),
            &provider_xml("fixture", "rmw", "fixture"),
        );

        let r = scan_root(root, 0).unwrap();
        let names: Vec<_> = r.providers.iter().map(|p| p.package.as_str()).collect();
        assert_eq!(names, vec!["outer"], "the nested fixture is not a sibling");
    }

    #[test]
    fn missing_root_is_empty_not_an_error() {
        let tmp = tempfile::tempdir().unwrap();
        let r = scan_root(&tmp.path().join("no-such-workspace"), 1).unwrap();
        assert!(r.providers.is_empty());
        assert_eq!(r.packages_seen(), 0);
    }

    /// `default_search_path` is pure — it compares paths and never touches the
    /// filesystem — so these directories need not exist. They are still built
    /// under a tempdir rather than written as absolute literals, because a
    /// hardcoded home-directory path in a test resolves only on the machine
    /// that wrote it (`check-absolute-paths`, issue 0334 — which reads source
    /// text and so flags such a path in a COMMENT too, deliberately: a doc
    /// example is exactly how the pattern spreads).
    #[test]
    fn search_path_drops_a_workspace_nested_in_the_nano_ros_tree() {
        let tmp = tempfile::tempdir().unwrap();
        let nros = tmp.path().join("nano-ros");
        let nested = nros.join("examples/native");
        let outside = tmp.path().join("my_ws");

        assert_eq!(
            default_search_path(Some(&nros), &nested),
            vec![nros.clone()],
            "a nested workspace would otherwise be scanned twice and every \
             provider would appear to shadow itself"
        );
        assert_eq!(
            default_search_path(Some(&nros), &outside),
            vec![nros.clone(), outside.clone()],
        );
        assert_eq!(
            default_search_path(None, &outside),
            vec![outside],
            "an out-of-tree consumer with no nano-ros source still scans its own \
             workspace",
        );
    }

    // --- resolution and shadowing (W5) --------------------------------------

    /// The workflow W5 exists for: a user's workspace copy overlays the
    /// nano-ros one, and the loser stays nameable so "why is my patched
    /// backend not used" is answerable.
    #[test]
    fn a_later_root_overlays_an_earlier_one_and_the_loser_is_kept() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        write(
            &a.path().join("packages/rmw/zenoh/package.xml"),
            &provider_xml("nros_rmw_zenoh", "rmw", "zenoh"),
        );
        write(
            &b.path().join("src/patched_zenoh/package.xml"),
            &provider_xml("patched_zenoh", "rmw", "zenoh"),
        );

        let scan = scan_roots(&[a.path().to_path_buf(), b.path().to_path_buf()]).unwrap();
        let r = resolve_unique(&scan, "rmw", "zenoh").unwrap();

        assert_eq!(r.winner.package, "patched_zenoh", "the OVERLAY wins");
        assert!(r.is_shadowing());
        assert_eq!(r.shadowed.len(), 1);
        assert_eq!(r.shadowed[0].package, "nros_rmw_zenoh");
    }

    /// Two claimants in ONE root have no precedence to appeal to.
    #[test]
    fn same_root_ambiguity_is_an_error_listing_both() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/one/package.xml"),
            &provider_xml("one", "rmw", "dup"),
        );
        write(
            &root.join("src/two/package.xml"),
            &provider_xml("two", "rmw", "dup"),
        );

        let scan = scan_roots(&[root.to_path_buf()]).unwrap();
        let err = resolve_unique(&scan, "rmw", "dup").unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("SAME search root"), "got: {msg}");
        assert!(
            msg.contains("src/one") && msg.contains("src/two"),
            "got: {msg}"
        );
    }

    /// The W2 finding: a name claimed twice IN ONE ROOT is legitimate when the
    /// caller has its own discriminator (`threadx`, split by
    /// `target_contains`). `candidates()` serves that caller — it must return
    /// both rather than erroring.
    #[test]
    fn candidates_returns_every_claimant_for_a_caller_with_a_discriminator() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/threadx_linux/package.xml"),
            &provider_xml("threadx_linux", "board", "threadx"),
        );
        write(
            &root.join("src/threadx_riscv/package.xml"),
            &provider_xml("threadx_riscv", "board", "threadx"),
        );

        let scan = scan_roots(&[root.to_path_buf()]).unwrap();
        assert_eq!(candidates(&scan, "board", "threadx").len(), 2);
        // …while the unique resolver correctly refuses to pick one.
        assert!(resolve_unique(&scan, "board", "threadx").is_err());
    }

    /// An unknown name is usually a typo, so the error carries the list.
    #[test]
    fn not_found_lists_the_names_that_do_exist() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/a/package.xml"),
            &provider_xml("a", "rmw", "zenoh"),
        );
        let scan = scan_roots(&[root.to_path_buf()]).unwrap();
        let msg = resolve_unique(&scan, "rmw", "zenho")
            .unwrap_err()
            .to_string();
        assert!(msg.contains("zenho"), "names what was asked: {msg}");
        assert!(msg.contains("Available: zenoh"), "got: {msg}");
    }

    #[test]
    fn resolving_without_a_shadow_reports_none() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/only/package.xml"),
            &provider_xml("only", "rmw", "solo"),
        );
        let scan = scan_roots(&[root.to_path_buf()]).unwrap();
        let r = resolve_unique(&scan, "rmw", "solo").unwrap();
        assert!(!r.is_shadowing());
    }

    // --- index (W3) --------------------------------------------------------

    #[test]
    fn index_round_trips_and_matches_a_fresh_scan() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("ws");
        write(
            &root.join("src/a/package.xml"),
            &provider_xml("a", "rmw", "acme"),
        );
        write(&root.join("src/b/package.xml"), &plain_xml("b"));

        let roots = vec![root.clone()];
        let scan = scan_roots(&roots).unwrap();
        let idx = ProviderIndex::from_scan(&roots, &scan);
        let path = tmp.path().join("build/nros/providers.json");
        idx.write(&path).unwrap();

        let back = ProviderIndex::read(&path).unwrap();
        assert!(back.is_valid_for(&roots));
        assert_eq!(back.providers.len(), 1);
        assert_eq!(back.inputs.len(), 2, "non-providers are inputs too");
        assert!(
            diff_index(&back, &scan).is_empty(),
            "no drift against itself"
        );
    }

    /// The case a file watch cannot cover: a package.xml that did not exist
    /// when the index was written is in nobody's watch list (issue 0196's
    /// shape). Only a rescan-and-compare sees it.
    #[test]
    fn diff_detects_a_provider_added_after_the_index_was_written() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("ws");
        write(
            &root.join("src/a/package.xml"),
            &provider_xml("a", "rmw", "acme"),
        );
        let roots = vec![root.clone()];
        let idx = ProviderIndex::from_scan(&roots, &scan_roots(&roots).unwrap());

        write(
            &root.join("src/late/package.xml"),
            &provider_xml("late", "rmw", "latecomer"),
        );
        let fresh = scan_roots(&roots).unwrap();
        let d = diff_index(&idx, &fresh);

        assert!(!d.is_empty());
        assert_eq!(d.added_inputs.len(), 1);
        assert!(d.added_inputs[0].ends_with("src/late/package.xml"));
        assert!(
            d.changed_provisions
                .iter()
                .any(|c| c.contains("+ rmw:latecomer")),
            "got {:?}",
            d.changed_provisions
        );
    }

    #[test]
    fn diff_detects_a_provision_removed_from_an_existing_package() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("ws");
        let pkg = root.join("src/a/package.xml");
        write(&pkg, &provider_xml("a", "rmw", "acme"));
        let roots = vec![root.clone()];
        let idx = ProviderIndex::from_scan(&roots, &scan_roots(&roots).unwrap());

        // The file still exists and is still an input — only its content moved.
        write(&pkg, &plain_xml("a"));
        let d = diff_index(&idx, &scan_roots(&roots).unwrap());

        assert!(d.added_inputs.is_empty() && d.removed_inputs.is_empty());
        assert!(
            d.changed_provisions
                .iter()
                .any(|c| c.starts_with("- rmw:acme")),
            "got {:?}",
            d.changed_provisions
        );
    }

    /// An index built for a different search path is WRONG, not stale.
    #[test]
    fn index_for_other_roots_is_rejected() {
        let tmp = tempfile::tempdir().unwrap();
        let a = tmp.path().join("a");
        let b = tmp.path().join("b");
        fs::create_dir_all(&a).unwrap();
        let one = std::slice::from_ref(&a);
        let idx = ProviderIndex::from_scan(one, &scan_roots(one).unwrap());
        assert!(idx.is_valid_for(one));
        assert!(!idx.is_valid_for(&[a, b]));
    }

    #[test]
    fn unknown_index_version_is_an_error_not_a_guess() {
        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("providers.json");
        fs::write(
            &path,
            r#"{"version":9999,"roots":[],"inputs":[],"providers":[]}"#,
        )
        .unwrap();
        let err = ProviderIndex::read(&path).unwrap_err().to_string();
        assert!(err.contains("version 9999"), "got: {err}");
    }

    #[test]
    fn to_lines_emits_one_row_per_provision() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path().join("ws");
        write(
            &root.join("src/multi/package.xml"),
            r#"<?xml version="1.0"?>
<package format="3">
  <name>multi</name>
  <export>
    <nano_ros_provides kind="rmw" name="one"/>
    <nano_ros_provides kind="board" name="two"/>
  </export>
</package>"#,
        );
        let roots = vec![root];
        let idx = ProviderIndex::from_scan(&roots, &scan_roots(&roots).unwrap());
        let rendered = idx.to_lines();
        let lines: Vec<&str> = rendered.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].starts_with("rmw\tone\tmulti\t0\t"));
        assert!(lines[1].starts_with("board\ttwo\tmulti\t0\t"));
    }

    // --- the package graph (W4) --------------------------------------------

    fn pkg_xml(name: &str, deps: &[&str]) -> String {
        let d: String = deps
            .iter()
            .map(|x| format!("  <exec_depend>{x}</exec_depend>\n"))
            .collect();
        format!(
            "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  <name>{name}</name>\n{d}</package>"
        )
    }

    fn ordered_names(root: &Path) -> Vec<String> {
        let (pkgs, _) = scan_workspace_packages(root).unwrap();
        topological_order(&pkgs)
            .unwrap()
            .into_iter()
            .map(|p| p.name)
            .collect()
    }

    /// The real shape this replaces: an entry composes node packages and must
    /// be configured AFTER them, which every workspace states by hand today as
    /// "node pkgs BEFORE entries" in a `SUBDIRS` list.
    #[test]
    fn an_entry_is_ordered_after_the_nodes_it_composes() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        // Written in an order that would be WRONG if the walk order were kept:
        // `entry` sorts before both node packages by path and by name.
        write(
            &root.join("src/entry/package.xml"),
            &pkg_xml("entry", &["talker_pkg", "listener_pkg"]),
        );
        write(
            &root.join("src/talker_pkg/package.xml"),
            &pkg_xml("talker_pkg", &["std_msgs"]),
        );
        write(
            &root.join("src/listener_pkg/package.xml"),
            &pkg_xml("listener_pkg", &["std_msgs"]),
        );

        let names = ordered_names(root);
        let pos = |n: &str| names.iter().position(|x| x == n).unwrap();
        assert!(pos("talker_pkg") < pos("entry"));
        assert!(pos("listener_pkg") < pos("entry"));
        assert_eq!(
            names,
            vec!["listener_pkg", "talker_pkg", "entry"],
            "ties break by name, so the order is reproducible"
        );
    }

    /// `std_msgs` is not in the workspace. Treating an unknown dependency as a
    /// missing package would reject every real workspace.
    #[test]
    fn dependencies_outside_the_workspace_are_ignored() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/only/package.xml"),
            &pkg_xml("only", &["std_msgs", "rclcpp", "some_system_dep"]),
        );
        assert_eq!(ordered_names(root), vec!["only"]);
    }

    #[test]
    fn a_cycle_is_an_error_naming_every_package_on_it() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(&root.join("src/a/package.xml"), &pkg_xml("a", &["b"]));
        write(&root.join("src/b/package.xml"), &pkg_xml("b", &["c"]));
        write(&root.join("src/c/package.xml"), &pkg_xml("c", &["a"]));
        write(&root.join("src/fine/package.xml"), &pkg_xml("fine", &[]));

        let (pkgs, _) = scan_workspace_packages(root).unwrap();
        let err = topological_order(&pkgs).unwrap_err();
        assert_eq!(err.0, vec!["a", "b", "c"], "names the whole knot");
        assert!(!err.0.contains(&"fine".to_string()));
    }

    /// A package depending on itself is a no-op, not a one-node cycle. Real
    /// package.xml files do this by listing their own name in a group.
    #[test]
    fn a_self_dependency_does_not_deadlock() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/solo/package.xml"),
            &pkg_xml("solo", &["solo"]),
        );
        assert_eq!(ordered_names(root), vec!["solo"]);
    }

    /// Independent packages come out in a stable, name-sorted order rather than
    /// whatever `read_dir` returned — otherwise the emitted order would differ
    /// between machines for one tree.
    #[test]
    fn independent_packages_are_ordered_deterministically() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        for n in ["zulu", "alpha", "mike"] {
            write(&root.join(format!("src/{n}/package.xml")), &pkg_xml(n, &[]));
        }
        assert_eq!(ordered_names(root), vec!["alpha", "mike", "zulu"]);
    }

    /// The safety-net property, and the reason ordering can be turned on
    /// everywhere: where nothing is DECLARED, the caller's authored order
    /// survives untouched.
    ///
    /// Found the hard way — four real workspaces broke when this sorted purely
    /// by name. Their entry packages declare no `<exec_depend>` at all, so a
    /// name-sorted order was free to interleave `native_entry` between
    /// `ctrl_pkg` and `telem_pkg` without violating any stated constraint, and
    /// the entry's codegen then ran before the node metadata it reads existed.
    #[test]
    fn caller_order_wins_ties_so_undeclared_workspaces_are_untouched() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        // Exactly the realtime-c shape: an entry that declares nothing, whose
        // name sorts between the two node packages.
        write(
            &root.join("src/ctrl_pkg/package.xml"),
            &pkg_xml("ctrl_pkg", &[]),
        );
        write(
            &root.join("src/native_entry/package.xml"),
            &pkg_xml("native_entry", &[]),
        );
        write(
            &root.join("src/telem_pkg/package.xml"),
            &pkg_xml("telem_pkg", &[]),
        );

        let (pkgs, _) = scan_workspace_packages(root).unwrap();

        // Name order interleaves the entry — valid topologically, broken in
        // practice.
        let by_name: Vec<String> = topological_order(&pkgs)
            .unwrap()
            .into_iter()
            .map(|p| p.name)
            .collect();
        assert_eq!(by_name, vec!["ctrl_pkg", "native_entry", "telem_pkg"]);

        // The authored order is preserved instead.
        let authored: Vec<PathBuf> = ["ctrl_pkg", "telem_pkg", "native_entry"]
            .iter()
            .map(|n| root.join("src").join(n))
            .collect();
        let got: Vec<String> = topological_order_with_priority(&pkgs, &authored)
            .unwrap()
            .into_iter()
            .map(|p| p.name)
            .collect();
        assert_eq!(got, vec!["ctrl_pkg", "telem_pkg", "native_entry"]);
    }

    /// Preference must not override a DECLARED dependency — otherwise the
    /// safety net would silently do nothing whenever the authored order is
    /// wrong, which is the only case it exists for.
    #[test]
    fn a_declared_dependency_beats_the_caller_order() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/entry/package.xml"),
            &pkg_xml("entry", &["node_pkg"]),
        );
        write(
            &root.join("src/node_pkg/package.xml"),
            &pkg_xml("node_pkg", &[]),
        );

        let (pkgs, _) = scan_workspace_packages(root).unwrap();
        // Author asks for the WRONG order.
        let authored: Vec<PathBuf> = ["entry", "node_pkg"]
            .iter()
            .map(|n| root.join("src").join(n))
            .collect();
        let got: Vec<String> = topological_order_with_priority(&pkgs, &authored)
            .unwrap()
            .into_iter()
            .map(|p| p.name)
            .collect();
        assert_eq!(got, vec!["node_pkg", "entry"], "the declared edge wins");
    }

    /// The acceptance criterion: a workspace whose `src/` holds a PROVIDER and
    /// a CONSUMER of it orders the provider first, with nothing authored.
    #[test]
    fn a_workspace_provider_is_ordered_before_its_consumer() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write(
            &root.join("src/my_backend/package.xml"),
            r#"<?xml version="1.0"?>
<package format="3">
  <name>my_backend</name>
  <export><nano_ros_provides kind="rmw" name="acme"/></export>
</package>"#,
        );
        write(
            &root.join("src/app/package.xml"),
            &pkg_xml("app", &["my_backend"]),
        );

        let names = ordered_names(root);
        assert_eq!(names, vec!["my_backend", "app"]);

        // And the provider is still discoverable as one — ordering did not
        // consume the provision.
        let scan = scan_roots(&[root.to_path_buf()]).unwrap();
        assert_eq!(scan.providers.len(), 1);
        assert_eq!(scan.providers[0].provides[0].name, "acme");
    }

    #[test]
    fn descriptor_path_is_derived_from_kind() {
        let tmp = tempfile::tempdir().unwrap();
        write(
            &tmp.path().join("src/p/package.xml"),
            &provider_xml("p", "rmw", "acme"),
        );
        let r = scan_root(tmp.path(), 0).unwrap();
        assert!(
            r.providers[0]
                .descriptor_path("rmw")
                .ends_with("nros-rmw.toml"),
            "the scan hands selection a path; it does not read it"
        );
    }
    // =======================================================================
    // The search path — phase-420 W6
    // =======================================================================

    /// The two-root default is unchanged by W6: with nothing configured, the
    /// wider builder produces exactly what `default_search_path` always did.
    /// The widening is opt-in, so no existing tree changes shape.
    #[test]
    fn with_nothing_configured_the_path_is_still_the_two_default_roots() {
        let repo = tempfile::tempdir().unwrap();
        let ws = tempfile::tempdir().unwrap();

        let path = build_search_path(Some(repo.path()), ws.path(), &[], None);
        assert_eq!(
            path.paths(),
            default_search_path(Some(repo.path()), ws.path()),
            "`default_search_path` must BE the unconfigured case, not a second \
             implementation of it"
        );
        assert_eq!(
            path.roots.iter().map(|r| r.origin).collect::<Vec<_>>(),
            vec![RootOrigin::NanoRosTree, RootOrigin::Workspace],
        );
        assert!(path.missing().is_empty());
    }

    /// The acceptance criterion, first half: a provider in a THIRD root —
    /// neither the nano-ros tree nor the workspace — is on the search path and
    /// resolves by name.
    #[test]
    fn a_configured_third_root_is_searched_and_its_provider_resolves() {
        let repo = tempfile::tempdir().unwrap();
        let ws = tempfile::tempdir().unwrap();
        let third = tempfile::tempdir().unwrap();
        write(
            &third.path().join("acme_rmw/package.xml"),
            &provider_xml("acme_rmw", "rmw", "acme"),
        );

        let path = build_search_path(
            Some(repo.path()),
            ws.path(),
            &[third.path().display().to_string()],
            None,
        );
        assert_eq!(path.roots.len(), 3, "{:?}", path.roots);
        assert_eq!(path.roots[2].origin, RootOrigin::PackagePaths);

        let scan = scan_roots(&path.paths()).unwrap();
        let r = resolve_unique(&scan, "rmw", "acme").expect("the third root's provider resolves");
        assert_eq!(r.winner.package, "acme_rmw");
        assert_eq!(r.winner.root_index, 2);
        assert!(!r.is_shadowing());
    }

    /// The environment reaches the path too, and it lands AFTER the config —
    /// which under the later-wins rule means it can override a configured
    /// provider without being able to delete the configured ROOT.
    #[test]
    fn the_environment_appends_after_the_config_and_never_replaces_it() {
        let repo = tempfile::tempdir().unwrap();
        let ws = tempfile::tempdir().unwrap();
        let configured = tempfile::tempdir().unwrap();
        let from_env = tempfile::tempdir().unwrap();
        write(
            &configured.path().join("acme/package.xml"),
            &provider_xml("configured_acme", "rmw", "acme"),
        );
        write(
            &from_env.path().join("acme/package.xml"),
            &provider_xml("env_acme", "rmw", "acme"),
        );

        let path = build_search_path(
            Some(repo.path()),
            ws.path(),
            &[configured.path().display().to_string()],
            Some(&from_env.path().display().to_string()),
        );
        assert_eq!(
            path.roots.iter().map(|r| r.origin).collect::<Vec<_>>(),
            vec![
                RootOrigin::NanoRosTree,
                RootOrigin::Workspace,
                RootOrigin::PackagePaths,
                RootOrigin::Env,
            ],
            "the configured root must still be on the path — an exported \
             variable may add precedence, never subtract a root",
        );

        let scan = scan_roots(&path.paths()).unwrap();
        let r = resolve_unique(&scan, "rmw", "acme").unwrap();
        assert_eq!(r.winner.package, "env_acme", "the LATER root wins");
        assert_eq!(
            r.shadowed
                .iter()
                .map(|p| p.package.as_str())
                .collect::<Vec<_>>(),
            vec!["configured_acme"],
            "and the loser is NAMED, not dropped",
        );
    }

    /// Order decides a cross-root collision, and reversing the order reverses
    /// the winner. Asserted both ways round because a test that only ever sees
    /// one order cannot tell precedence from the order two tempdirs happen to
    /// sort in.
    #[test]
    fn search_path_order_decides_a_cross_root_collision() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        write(
            &a.path().join("p/package.xml"),
            &provider_xml("from_a", "serdes", "flatbuf"),
        );
        write(
            &b.path().join("p/package.xml"),
            &provider_xml("from_b", "serdes", "flatbuf"),
        );
        let ws = tempfile::tempdir().unwrap();

        for (first, second, expected) in [
            (a.path(), b.path(), "from_b"),
            (b.path(), a.path(), "from_a"),
        ] {
            let path = build_search_path(
                None,
                ws.path(),
                &[first.display().to_string(), second.display().to_string()],
                None,
            );
            let scan = scan_roots(&path.paths()).unwrap();
            let r = resolve_unique(&scan, "serdes", "flatbuf").unwrap();
            assert_eq!(
                r.winner.package,
                expected,
                "with {} before {}, the later root must win",
                first.display(),
                second.display(),
            );
        }
    }

    /// `shadowing()` is what the listing prints, so it must name the loser and
    /// agree with `resolve_unique` about the winner. Two orderings of one fact
    /// is how a report starts disagreeing with the build.
    #[test]
    fn the_shadowing_report_names_the_hidden_provider() {
        let under = tempfile::tempdir().unwrap();
        let over = tempfile::tempdir().unwrap();
        write(
            &under.path().join("p/package.xml"),
            &provider_xml("shipped", "rmw", "zenoh"),
        );
        write(
            &over.path().join("p/package.xml"),
            &provider_xml("patched", "rmw", "zenoh"),
        );
        // An uncontested provision, to prove the report is about collisions
        // rather than about "more than one provider exists".
        write(
            &under.path().join("q/package.xml"),
            &provider_xml("alone", "board", "mps2"),
        );

        let scan = scan_roots(&[under.path().to_path_buf(), over.path().to_path_buf()]).unwrap();
        let report = shadowing(&scan);
        assert_eq!(report.len(), 1, "only the contested pair: {report:?}");
        let s = &report[0];
        assert_eq!((s.kind.as_str(), s.name.as_str()), ("rmw", "zenoh"));
        assert!(!s.same_root_tie);
        assert_eq!(s.winner.package, "patched");
        assert_eq!(
            s.shadowed
                .iter()
                .map(|p| p.package.as_str())
                .collect::<Vec<_>>(),
            vec!["shipped"],
        );
        assert_eq!(
            s.winner.package,
            resolve_unique(&scan, "rmw", "zenoh")
                .unwrap()
                .winner
                .package,
            "the report and the resolver must agree about the winner",
        );
    }

    /// A collision WITHIN one root has no precedence to appeal to, so the
    /// report says so rather than inventing a winner the resolver will refuse.
    #[test]
    fn a_same_root_collision_is_reported_as_a_tie_not_as_shadowing() {
        let root = tempfile::tempdir().unwrap();
        write(
            &root.path().join("one/package.xml"),
            &provider_xml("one", "rmw", "acme"),
        );
        write(
            &root.path().join("two/package.xml"),
            &provider_xml("two", "rmw", "acme"),
        );

        let scan = scan_root(root.path(), 0).unwrap();
        let report = shadowing(&scan);
        assert_eq!(report.len(), 1);
        assert!(
            report[0].same_root_tie,
            "same root ⇒ no precedence; claiming a winner would disagree with \
             `resolve_unique`, which refuses",
        );
        assert!(matches!(
            resolve_unique(&scan, "rmw", "acme"),
            Err(ResolveError::Ambiguous { .. })
        ));
    }

    /// A root that does not exist is neither fatal nor invisible: it keeps its
    /// index (so the numbers in a stored index mean the same tree everywhere),
    /// contributes nothing, and is reported by `missing()`.
    #[test]
    fn a_nonexistent_configured_root_keeps_its_index_and_is_reported() {
        let repo = tempfile::tempdir().unwrap();
        let ws = tempfile::tempdir().unwrap();
        let real = tempfile::tempdir().unwrap();
        write(
            &real.path().join("p/package.xml"),
            &provider_xml("real", "rmw", "acme"),
        );
        let gone = ws.path().join("definitely/not/here");
        assert!(!gone.exists(), "the premise of the test");

        let path = build_search_path(
            Some(repo.path()),
            ws.path(),
            &[
                gone.display().to_string(),
                real.path().display().to_string(),
            ],
            None,
        );
        assert_eq!(path.roots.len(), 4, "{:?}", path.roots);
        assert!(!path.roots[2].exists);
        assert!(path.roots[3].exists);

        let missing = path.missing();
        assert_eq!(missing.len(), 1, "{missing:?}");
        assert_eq!(missing[0].path, gone);
        assert_eq!(missing[0].origin, RootOrigin::PackagePaths);

        // Not fatal: the scan runs and the root that IS there still answers.
        let scan = scan_roots(&path.paths()).expect("a missing root must not fail the scan");
        let r = resolve_unique(&scan, "rmw", "acme").expect("the real root still resolves");
        assert_eq!(r.winner.root_index, 3, "the missing root kept index 2");
    }

    /// The default roots are exempt from the missing-root report. A workspace
    /// that does not exist yet is a normal invocation, and a warning printed on
    /// every one of them is a warning nobody reads.
    #[test]
    fn a_missing_default_root_is_not_reported() {
        let repo = tempfile::tempdir().unwrap();
        let ws = repo.path().parent().unwrap().join("no-such-workspace");
        assert!(!ws.exists());

        let path = build_search_path(Some(repo.path()), &ws, &[], None);
        assert!(!path.roots[1].exists);
        assert!(
            path.missing().is_empty(),
            "only a root somebody TYPED is worth warning about: {:?}",
            path.missing(),
        );
    }

    /// A relative entry resolves against the WORKSPACE, not the cwd — the
    /// directory `nros.toml` sits in, and RFC-0087 D6's own `["src", …]`
    /// example.
    #[test]
    fn a_relative_entry_resolves_against_the_workspace() {
        let ws = tempfile::tempdir().unwrap();
        write(
            &ws.path().join("vendor/p/package.xml"),
            &provider_xml("vendored", "rmw", "acme"),
        );

        let path = build_search_path(None, ws.path(), &["vendor".to_string()], None);
        let vendor = path
            .roots
            .iter()
            .find(|r| r.origin == RootOrigin::PackagePaths)
            .expect("the configured root");
        assert_eq!(
            vendor.path,
            ws.path().join("vendor").canonicalize().unwrap(),
            "relative to the workspace, and canonicalized because it exists",
        );
        assert_eq!(vendor.as_written.as_deref(), Some("vendor"));
        assert!(vendor.exists);

        let scan = scan_roots(&path.paths()).unwrap();
        assert!(resolve_unique(&scan, "rmw", "acme").is_ok());
    }

    /// `~/…` expands against `$HOME`; `~user` is left literal, so it surfaces
    /// as a missing root naming what was typed rather than resolving to a
    /// directory nobody named.
    #[test]
    fn tilde_expands_for_home_and_not_for_a_named_user() {
        let home = std::env::var_os("HOME")
            .filter(|h| !h.is_empty())
            .expect("this test is about $HOME expansion; without $HOME it proves nothing");
        let home = PathBuf::from(home);

        assert_eq!(expand_tilde("~"), home);
        assert_eq!(expand_tilde("~/nros-packages"), home.join("nros-packages"));
        assert_eq!(
            expand_tilde("~someone/pkgs"),
            PathBuf::from("~someone/pkgs"),
            "`~user` needs a passwd lookup; left literal it fails loudly",
        );
        assert_eq!(expand_tilde("/opt/pkgs"), PathBuf::from("/opt/pkgs"));
        assert_eq!(
            expand_tilde("relative/pkgs"),
            PathBuf::from("relative/pkgs")
        );
    }

    /// A root named twice is one root, and it keeps the index of its FIRST
    /// appearance — so adding an entry never renumbers the roots before it, and
    /// nothing shadows itself.
    #[test]
    fn a_repeated_root_is_collapsed_to_its_first_appearance() {
        let ws = tempfile::tempdir().unwrap();
        let extra = tempfile::tempdir().unwrap();
        write(
            &extra.path().join("p/package.xml"),
            &provider_xml("once", "rmw", "acme"),
        );
        let spelled = extra.path().display().to_string();

        let path = build_search_path(
            None,
            ws.path(),
            std::slice::from_ref(&spelled),
            Some(&format!("{spelled}:{spelled}")),
        );
        assert_eq!(
            path.roots.len(),
            2,
            "workspace + one copy of the repeated root: {:?}",
            path.roots,
        );
        assert_eq!(
            path.roots[1].origin,
            RootOrigin::PackagePaths,
            "the first appearance keeps the slot; repeating cannot promote it",
        );

        let scan = scan_roots(&path.paths()).unwrap();
        let r = resolve_unique(&scan, "rmw", "acme").unwrap();
        assert!(
            !r.is_shadowing(),
            "one tree scanned twice would report every provider as shadowing itself",
        );
        assert!(shadowing(&scan).is_empty());
    }

    /// Empty entries are dropped rather than becoming the workspace root: a
    /// trailing `:` in `NROS_PACKAGE_PATH` is a typo, and resolving it to the
    /// workspace would silently duplicate root 1.
    #[test]
    fn empty_entries_are_dropped() {
        let ws = tempfile::tempdir().unwrap();
        let path = build_search_path(None, ws.path(), &[String::new(), "  ".into()], Some("::"));
        assert_eq!(
            path.roots.iter().map(|r| r.origin).collect::<Vec<_>>(),
            vec![RootOrigin::Workspace],
        );
    }

    /// `--base-paths` REPLACES: no nano-ros tree, no workspace, no config, no
    /// environment. That is colcon's semantics for the flag, and the deliberate
    /// contrast with `NROS_PACKAGE_PATH`, which only ever appends.
    #[test]
    fn base_paths_replace_the_whole_search_path() {
        let a = tempfile::tempdir().unwrap();
        let b = tempfile::tempdir().unwrap();
        let path = SearchPath::from_base_paths(&[a.path().to_path_buf(), b.path().to_path_buf()]);
        assert_eq!(
            path.roots.iter().map(|r| r.origin).collect::<Vec<_>>(),
            vec![RootOrigin::BasePaths, RootOrigin::BasePaths],
        );
        assert_eq!(
            path.paths(),
            vec![a.path().to_path_buf(), b.path().to_path_buf()]
        );
    }
}

#[cfg(test)]
mod pruned_dir_tests {
    use super::is_pruned_dir;

    /// issue 0645 — the exact-name list missed this tree's real build roots.
    #[test]
    fn build_root_prefixes_are_pruned() {
        // The names that actually cost the walk 3923 directories.
        assert!(is_pruned_dir("build-workspace-fixtures"));
        assert!(is_pruned_dir("build-workspace-fixtures-freertos"));
        assert!(is_pruned_dir("target-zenoh-fixture-posix"));
        // Still the plain ones.
        assert!(is_pruned_dir("build"));
        assert!(is_pruned_dir("target"));
        assert!(is_pruned_dir("third-party"));
    }

    /// The prefix must not swallow source packages. `builder_pkg` and
    /// `targeting_pkg` are the shapes a prefix match gets wrong if it is
    /// written as `contains` or without the hyphen.
    #[test]
    fn source_package_names_survive() {
        for name in [
            "builder_pkg",
            "buildings",
            "targeting_pkg",
            "targets",
            "src",
            "talker_pkg",
            "rust_heartbeat_pkg",
        ] {
            assert!(!is_pruned_dir(name), "{name} must not be pruned");
        }
    }
}
