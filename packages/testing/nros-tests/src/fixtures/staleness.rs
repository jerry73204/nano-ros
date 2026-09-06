//! What a freshness probe compared, and how long a coordinate has not run.
//!
//! # The defect this addresses (issue 0445)
//!
//! A staleness verdict is ABSORBING. When a probe calls a fixture stale the
//! fixture is never launched, so the runtime result it would have produced is
//! not merely unknown — it is replaced by an explanation that reads as
//! complete, remedy and all. Issue 0444 (a FreeRTOS Rust image that boots,
//! brings up the network and never reaches app setup) sat behind issue 0442 (a
//! probe exemption applied on one arm and not its sibling) for exactly as long
//! as the cells read STALE, and nothing in the report suggested looking.
//!
//! Two properties fix that without weakening any probe:
//!
//! 1. **Say what was compared.** A probe cannot know it is wrong, but it can
//!    account for its own decisions. `zpico.h` being EXAMINED by one arm and
//!    EXEMPTED by another is visible the moment the verdict prints the counts.
//! 2. **Count how long a coordinate has not run.** "Stale since your last edit"
//!    and "stale for the eleventh run in a row" are different signals and used
//!    to print identically. The second is the state in which defects
//!    accumulate unseen, so it is worth a line of its own.
//!
//! # One spelling of the exemption rule
//!
//! [`exempt_probe_input`] is the single place that decides whether a candidate
//! dependency is an edit event. It exists because the three probe arms
//! (`dep_file_newer_than`, `cmake_dep_info_newer_source`,
//! `newest_source_after`) each carried their OWN subset of the exemptions:
//! cargo-`OUT_DIR` products were skipped by one arm only, and the in-place
//! header list by two of three. That divergence IS issue 0442, and issue 0196
//! says the fix is a shared helper rather than a third copy. `check-staleness-
//! probe-exemptions` keeps them on it.

use std::{
    cell::Cell,
    fs,
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use crate::{TestError, project_root};

/// Headers that are REGENERATED IN PLACE: cbindgen writes them on every build
/// of any feature set, moving the mtime without changing a byte. "Newer than my
/// binary" therefore says nothing about this fixture's inputs — issue #222's
/// cross-family false-stale.
const REGENERATED_INPLACE_HEADERS: &[&str] = &[
    "packages/api/nros-c/include/nros/nros_generated.h",
    "packages/api/nros-cpp/include/nros/nros_cpp_ffi.h",
    "packages/rmw/zenoh/zpico-sys/c/include/zpico.h",
];

/// Why a candidate dependency is not an edit event.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Exemption {
    /// cbindgen output written in place — mtime moves, content does not.
    RegeneratedInPlace,
    /// A cargo `OUT_DIR` product (`…/build/<pkg>-<hash>/out/…`): ANY cargo
    /// invocation in the same target dir reruns build scripts and refreshes
    /// these WITHOUT relinking the binary. A semantic change to one implies an
    /// edited tracked source, and those ARE in the dep graph. Without this,
    /// `just ci` re-staled the very fixtures its own check phase probed fresh.
    CargoOutDir,
}

impl Exemption {
    fn label(self) -> &'static str {
        match self {
            Exemption::RegeneratedInPlace => "regenerated-in-place header",
            Exemption::CargoOutDir => "cargo OUT_DIR product",
        }
    }
}

/// The one exemption rule. Returns `Some(reason)` when `path` must not be read
/// as an edit event, `None` when it is a real input worth comparing.
///
/// Every probe arm calls THIS — never a subset of it. See the module docs.
pub fn exempt_probe_input(path: &Path) -> Option<Exemption> {
    if REGENERATED_INPLACE_HEADERS
        .iter()
        .any(|suffix| path.ends_with(suffix))
    {
        return Some(Exemption::RegeneratedInPlace);
    }
    if is_cargo_out_dir_product(path) {
        return Some(Exemption::CargoOutDir);
    }
    None
}

fn is_cargo_out_dir_product(path: &Path) -> bool {
    let mut saw_out = false;
    for comp in path.components().rev() {
        let c = comp.as_os_str().to_string_lossy();
        if !saw_out {
            if c == "out" {
                saw_out = true;
            }
        } else {
            // the component just above `out/` is `<pkg>-<hash>` inside `build/`
            return path
                .components()
                .any(|p| p.as_os_str().to_string_lossy() == "build");
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Direction 1 — account for what the probe compared.
// ---------------------------------------------------------------------------

thread_local! {
    static EXAMINED: Cell<usize> = const { Cell::new(0) };
    static EXEMPT_INPLACE: Cell<usize> = const { Cell::new(0) };
    static EXEMPT_OUTDIR: Cell<usize> = const { Cell::new(0) };
    /// phase-363 — set when an arm could not obtain the MEASURED input set and
    /// fell back to a hand-authored one.
    static UNMEASURED: Cell<bool> = const { Cell::new(false) };
}

/// Start accounting for one probe run. Called by each `require_*_fresh` entry
/// point before it probes; the counters are per-thread because nextest runs
/// each test in its own process and rstest fixtures resolve on the test thread.
pub fn begin_probe() {
    EXAMINED.with(|c| c.set(0));
    EXEMPT_INPLACE.with(|c| c.set(0));
    EXEMPT_OUTDIR.with(|c| c.set(0));
    UNMEASURED.with(|c| c.set(false));
}

/// Record one candidate the probe looked at, and whether the shared rule
/// exempted it. Returns `true` when the caller should SKIP the candidate — so
/// an arm reads `if note_candidate(&p) { continue; }` and cannot both account
/// and decide differently.
pub fn note_candidate(path: &Path) -> bool {
    EXAMINED.with(|c| c.set(c.get() + 1));
    match exempt_probe_input(path) {
        Some(Exemption::RegeneratedInPlace) => {
            EXEMPT_INPLACE.with(|c| c.set(c.get() + 1));
            true
        }
        Some(Exemption::CargoOutDir) => {
            EXEMPT_OUTDIR.with(|c| c.set(c.get() + 1));
            true
        }
        None => false,
    }
}

/// Record that this probe could not read the input set the BUILD recorded and
/// is comparing a hand-authored one instead (phase-363).
///
/// The fallback is deliberate and fails safe — it is over-broad, so it errs
/// toward a false STALE rather than a museum binary. What is not acceptable is
/// it happening SILENTLY: a probe that quietly stops measuring looks identical
/// to one that measured and found nothing, and CLAUDE.md already tells the
/// reader to believe the `probe:` line over the verdict. So it says so there.
pub fn note_unmeasured_input_set() {
    UNMEASURED.with(|c| c.set(true));
}

/// What the probe compared, in one line.
pub fn probe_accounting() -> String {
    let examined = EXAMINED.with(Cell::get);
    let inplace = EXEMPT_INPLACE.with(Cell::get);
    let outdir = EXEMPT_OUTDIR.with(Cell::get);
    let mut line = format!(
        "examined {examined} input(s); exempted {inplace} {} + {outdir} {}",
        Exemption::RegeneratedInPlace.label(),
        Exemption::CargoOutDir.label(),
    );
    if UNMEASURED.with(Cell::get) {
        line.push_str(
            "; INPUT SET UNMEASURED — no build-script record found, \
             compared a hand-authored path list instead",
        );
    }
    line
}

// ---------------------------------------------------------------------------
// Direction 3 — count how long a coordinate has not produced a runtime result.
// ---------------------------------------------------------------------------

/// How long this coordinate has been reported stale.
#[derive(Clone, Copy, Debug)]
pub struct StaleHistory {
    /// Consecutive resolutions that ended in a staleness verdict, this one
    /// included. `1` means "stale since your last edit".
    pub consecutive: u32,
    /// Unix seconds of the FIRST verdict in the current run of them.
    pub since_epoch: u64,
}

impl StaleHistory {
    /// The line worth printing above a remedy: silent when a cell just went
    /// stale, loud once it has been non-running long enough to hide something.
    pub fn note(&self) -> String {
        if self.consecutive < 2 {
            return String::new();
        }
        let age = now_epoch().saturating_sub(self.since_epoch);
        format!(
            "\n  NOT RUN: {}th consecutive stale verdict for this fixture, first {} ago.\n  \
             This coordinate has produced no runtime result since then — whatever it \n  \
             would have done is being absorbed by this message (issue 0445). If the \n  \
             rebuild does not clear it, suspect the probe before trusting the verdict.",
            self.consecutive,
            human_duration(age),
        )
    }
}

fn now_epoch() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn human_duration(secs: u64) -> String {
    match secs {
        s if s < 90 => format!("{s}s"),
        s if s < 5400 => format!("{}m", s / 60),
        s if s < 172_800 => format!("{}h", s / 3600),
        s => format!("{}d", s / 86_400),
    }
}

/// Where the ledger lives. Under `target/` (gitignored) so it survives a test
/// run but never a clean, and never reaches a commit.
fn ledger_dir() -> PathBuf {
    project_root().join("target/nros-fixture-staleness")
}

/// One file per coordinate, named after the binary path with separators
/// flattened. Per-coordinate files rather than one shared ledger because
/// nextest resolves fixtures from many processes at once and a single file
/// would need a lock for no benefit.
fn ledger_path(binary: &Path) -> PathBuf {
    ledger_dir().join(format!("{}.stale", flatten_path_key(binary)))
}

// ── Content-aware staleness (#147 / phase-286 W2) ───────────────────────────
//
// Moved here from `zephyr.rs` by phase-353 W2 so BOTH staleness arms share one
// spelling. It answers "did the bytes change", which is the only honest answer
// to an mtime that moved on its own: `git pull --rebase`, `git stash push/pop`
// and a branch switch all rewrite tracked files identically, and every prebuilt
// fixture then reads STALE (~28 s per cold Zephyr leaf — issue 0509 direction
// 3, issue 0466).
//
// It lived in the zephyr module and was used by the zephyr arm alone, so the
// dep-info arm kept comparing raw mtimes. That is issue 0442's shape exactly —
// arms of one probe diverging because the rule lived in one of them — and the
// reason this module's header already insists exemptions have ONE spelling.

/// Flatten a candidate path (file or dir) into its source files, applying the
/// same `target`/`build`/`.git` skips as [`path_newer_than`]'s dir recursion.
pub(crate) fn collect_source_files(path: &Path, out: &mut Vec<PathBuf>) {
    let Ok(meta) = path.symlink_metadata() else {
        return;
    };
    if meta.is_file() {
        out.push(path.to_path_buf());
        return;
    }
    if !meta.is_dir() {
        return;
    }
    let Ok(entries) = std::fs::read_dir(path) else {
        return;
    };
    for entry in entries.flatten() {
        let name = entry.file_name();
        if crate::treewalk::is_skipped_dir(name.to_string_lossy().as_ref()) {
            continue;
        }
        collect_source_files(&entry.path(), out);
    }
}

/// Non-cryptographic content hash of a file (SipHash via `DefaultHasher`) — a
/// change detector, not a security primitive. `None` if unreadable.
pub(crate) fn hash_file_content(path: &Path) -> Option<u64> {
    use std::hash::Hasher;
    let bytes = std::fs::read(path).ok()?;
    let mut h = std::collections::hash_map::DefaultHasher::new();
    h.write(&bytes);
    Some(h.finish())
}

/// Content-aware staleness (#147 / phase-286 W2).
///
/// Returns `Some(true)` if a watched source's CONTENT differs from what the
/// current linked binary was built with, `Some(false)` if everything matches
/// (including the case where an mtime moved but the bytes did not), or `None`
/// if the binary itself can't be hashed (caller falls back to the mtime gate).
///
/// Mechanism: a sidecar `<binary_dir>/.nros-srcbaseline` records the binary's
/// content hash plus each watched file's `(mtime, size, content_hash)`. When the
/// binary hash differs from the baseline (a rebuild happened, or first sight),
/// the image IS the fresh truth — re-record the baseline and report not-stale.
/// When the binary is unchanged, only files whose `(mtime, size)` moved are
/// content-hashed; a moved mtime with an unchanged hash is an artifact (not
/// stale), a changed hash or a newly-appearing file is a real edit (stale).
/// Atomically (temp + rename) write the source baseline sidecar.
/// Nanosecond mtime of a path, if it can be read.
fn mtime_nanos(p: &Path) -> Option<u128> {
    Some(
        p.metadata()
            .ok()?
            .modified()
            .ok()?
            .duration_since(std::time::UNIX_EPOCH)
            .ok()?
            .as_nanos(),
    )
}

fn write_srcbaseline(path: &Path, binary_path: &Path, bin_hash: u64, files: &[PathBuf]) {
    let bin_mtime = mtime_nanos(binary_path);
    // issue 0806 — the binary's MTIME rides along with its hash. The hash alone
    // cannot answer "was this rebuilt?" when the build is REPRODUCIBLE: a
    // rebuild that emits identical bytes leaves the hash equal, the baseline
    // unrefreshed, and every source that moved before it permanently "changed".
    // A build OUTPUT is the one artifact whose mtime is trustworthy here —
    // git rewrites source mtimes (the treadmill this whole content-aware path
    // exists for) and never touches build outputs.
    let mut out = match bin_mtime {
        Some(m) => format!("bin {bin_hash} {m}\n"),
        None => format!("bin {bin_hash}\n"),
    };
    for f in files {
        let Ok(meta) = f.metadata() else { continue };
        let Some(mtime) = meta
            .modified()
            .ok()
            .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
            .map(|d| d.as_nanos())
        else {
            continue;
        };
        let Some(h) = hash_file_content(f) else {
            continue;
        };
        out.push_str(&format!("{mtime} {} {h} {}\n", meta.len(), f.display()));
    }
    // Atomic: write a pid-unique temp then rename, so parallel test processes
    // sharing a fixture never read a half-written baseline.
    let tmp = path.with_extension(format!("tmp.{}", std::process::id()));
    if std::fs::write(&tmp, out.as_bytes()).is_ok() {
        let _ = std::fs::rename(&tmp, path);
    }
}

pub(crate) fn candidates_changed_content(
    binary_path: &Path,
    candidates: &[PathBuf],
) -> Option<bool> {
    candidates_changed_content_policy(binary_path, candidates, true)
}

/// [`candidates_changed_content`], with the no-baseline answer named.
///
/// `first_sight_is_fresh = true` is the zephyr arm's long-standing behaviour;
/// `false` keeps the dep-info arm strict, so an artifact whose mtime already
/// looks stale is not forgiven merely for being unrecognised.
pub(crate) fn candidates_changed_content_policy(
    binary_path: &Path,
    candidates: &[PathBuf],
    first_sight_is_fresh: bool,
) -> Option<bool> {
    let bin_hash = hash_file_content(binary_path)?;

    let mut files = Vec::new();
    for c in candidates {
        collect_source_files(c, &mut files);
    }
    files.sort();
    files.dedup();

    // phase-353 W2 — nothing to compare is INCONCLUSIVE, never "unchanged".
    //
    // `collect_source_files` skips `target`/`build`/`.git`, so a candidate that
    // lives under one of those yields an empty set. Answering `Some(false)`
    // there would report the artifact fresh on the strength of having examined
    // nothing — a silent-fresh path, which is the museum-binary shape issue
    // 0196 is about. `None` hands the decision back to the caller, which keeps
    // its own (stricter) mtime verdict.
    if files.is_empty() {
        return None;
    }

    let baseline_path = binary_path.parent()?.join(".nros-srcbaseline");
    let baseline = std::fs::read_to_string(&baseline_path).ok();

    // Parse baseline: first line `bin <hash>`, then `<mtime_nanos> <size> <hash> <path>`.
    let mut stored_bin: Option<u64> = None;
    let mut stored_bin_mtime: Option<u128> = None;
    let mut stored: std::collections::HashMap<PathBuf, (u128, u64, u64)> =
        std::collections::HashMap::new();
    if let Some(text) = &baseline {
        for line in text.lines() {
            if let Some(rest) = line.strip_prefix("bin ") {
                // `bin <hash>` (pre-0806) or `bin <hash> <mtime_nanos>`.
                let mut it = rest.trim().split(' ');
                stored_bin = it.next().and_then(|h| h.parse().ok());
                stored_bin_mtime = it.next().and_then(|m| m.parse().ok());
                continue;
            }
            let mut it = line.splitn(4, ' ');
            let (Some(m), Some(s), Some(h), Some(p)) = (it.next(), it.next(), it.next(), it.next())
            else {
                continue;
            };
            if let (Ok(m), Ok(s), Ok(h)) = (m.parse(), s.parse(), h.parse()) {
                stored.insert(PathBuf::from(p), (m, s, h));
            }
        }
    }

    let file_meta = |p: &Path| -> Option<(u128, u64, u64)> {
        let meta = p.metadata().ok()?;
        let mtime = meta
            .modified()
            .ok()?
            .duration_since(std::time::UNIX_EPOCH)
            .ok()?
            .as_nanos();
        Some((mtime, meta.len(), 0))
    };

    // Binary changed (rebuilt) or no baseline → the image is the fresh truth.
    // Record a full baseline and report not-stale.
    //
    // phase-353 W2 — that "or no baseline" is a POLICY, not a fact, and the two
    // arms need different ones. The zephyr arm has always taken it: a first
    // sight records and reports fresh. The dep-info arm must not, because its
    // mtime gate has ALREADY said something looks newer, and answering "fresh"
    // to an unknown artifact would undo the guard #147 added — an existing test
    // (`a_stale_verdict_reports_its_own_reasoning_and_its_age`) catches exactly
    // that. `first_sight_is_fresh` names the choice at each call site instead of
    // hiding it here.
    // issue 0806 — "was it rebuilt?" is TWO questions, and the hash answers only
    // one. A REPRODUCIBLE build that emits identical bytes leaves the hash
    // equal, so the baseline never refreshes and every source that moved before
    // it stays "changed" forever: the verdict is absorbing, and no number of
    // rebuilds clears it (measured at four consecutive STALE verdicts on
    // `build-cortex-m-c-talker-zenoh`, whose baseline was two rebuilds old).
    //
    // The binary's MTIME closes it. This is the one artifact whose mtime can be
    // trusted: git rewrites SOURCE mtimes — the treadmill this whole
    // content-aware path exists to survive — and never touches build outputs.
    //
    // Only consulted when the baseline actually carries one, so pre-0806
    // baselines keep the hash-only behaviour and self-heal on their next
    // rebuild instead of all going stale at once.
    let bin_mtime = mtime_nanos(binary_path);

    // The decisive case, and the one the baseline cannot express: an artifact
    // NEWER THAN EVERY INPUT is fresh by definition — the build ran after the
    // last source touch, whatever the bytes came out as. Checking this FIRST is
    // what makes a reproducible rebuild able to clear a verdict at all; the
    // content machinery below exists only to adjudicate the opposite case,
    // where a source genuinely looks newer and we must tell a real edit from a
    // git-induced mtime bump.
    //
    // Without it the probe is ABSORBING: hash equal => baseline never
    // refreshed => sources that moved before the last build stay "changed"
    // forever. Measured at five consecutive STALE verdicts on
    // `build-cortex-m-c-talker-zenoh` across two full rebuilds, with nothing
    // under any watched path newer than the image.
    if let Some(bin_m) = bin_mtime
        && files
            .iter()
            .filter_map(|f| mtime_nanos(f))
            .all(|src_m| src_m <= bin_m)
    {
        write_srcbaseline(&baseline_path, binary_path, bin_hash, &files);
        return Some(false);
    }

    let rebuilt = match (stored_bin_mtime, bin_mtime) {
        (Some(stored), Some(now)) => stored_bin != Some(bin_hash) || stored != now,
        _ => stored_bin != Some(bin_hash),
    };
    if rebuilt {
        if first_sight_is_fresh {
            write_srcbaseline(&baseline_path, binary_path, bin_hash, &files);
            return Some(false);
        }
        // A stale verdict verifies NOTHING, so it must not leave a baseline
        // behind: doing so made the very next call compare against bytes that
        // were never blessed and report fresh, turning one real stale verdict
        // into a permanent pass. Caught by
        // `a_stale_verdict_reports_its_own_reasoning_and_its_age`, which probes
        // the same artifact three times.
        return Some(true);
    }

    // Binary unchanged: only content-hash files whose (mtime,size) moved.
    let mut refreshed = false;
    for f in &files {
        let Some((mtime, size, _)) = file_meta(f) else {
            // Unreadable now but tracked before → treat as a change.
            if stored.contains_key(f) {
                return Some(true);
            }
            continue;
        };
        match stored.get(f) {
            Some(&(sm, ss, sh)) => {
                if sm == mtime && ss == size {
                    continue; // unchanged, cheap path
                }
                // (mtime,size) moved — disambiguate by content.
                match hash_file_content(f) {
                    Some(h) if h == sh => refreshed = true, // mtime artifact only
                    Some(_) => return Some(true),           // real content change
                    None => return Some(true),
                }
            }
            // A watched file that did not exist at baseline time → real add.
            None => return Some(true),
        }
    }
    if refreshed {
        write_srcbaseline(&baseline_path, binary_path, bin_hash, &files);
    }
    Some(false)
}

// ---------------------------------------------------------------------------
// The MEASURED input sets, one spelling each.
// ---------------------------------------------------------------------------
//
// phase-395 W10. Both of these were written inline inside a probe arm, with the
// mtime comparison interleaved into the parse. That is fine while the probe is
// the only consumer; it stops being fine the moment a SECOND consumer needs the
// same set, because the second consumer then either re-parses (a second idiom,
// which is issue 0442's shape) or inherits an mtime verdict it does not want.
//
// The shadow cache key ([`crate::fixtures::cache_key`]) is that second
// consumer. It needs the input PATHS and nothing else, so the parse lives here
// and each caller applies its own policy — the same split
// `candidates_changed_content_policy` already makes for `first_sight_is_fresh`.

/// Split a cargo dep-info dependency list (`DEP DEP …`) into paths, honouring
/// make-style backslash escapes (`\ ` for a space in a path, `\\` for a
/// literal backslash). Cargo emits every dependency on the single rule line
/// after `TARGET: `.
pub(crate) fn split_dep_info_line(deps: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut chars = deps.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '\\' => {
                if let Some(&next) = chars.peek() {
                    cur.push(next);
                    chars.next();
                }
            }
            c if c.is_whitespace() => {
                if !cur.is_empty() {
                    out.push(std::mem::take(&mut cur));
                }
            }
            c => cur.push(c),
        }
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

/// Every dependency path a make-style `.d` dep-info file lists, in file order.
///
/// The COMPILER's own record of what it read — authoritative in a way no hand
/// list is. No exemption rule and no mtime comparison is applied here; a caller
/// that wants those applies them itself (the probe calls [`note_candidate`] per
/// path, the cache key applies [`exempt_probe_input`] with a different policy).
/// Missing or unreadable `.d` → an EMPTY vec, which every caller must treat as
/// "nothing was measured" rather than "nothing changed".
pub(crate) fn dep_file_paths(dep_file: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let Ok(content) = fs::read_to_string(dep_file) else {
        return out;
    };
    for line in content.lines() {
        let Some((_target, deps)) = line.split_once(": ") else {
            continue;
        };
        out.extend(split_dep_info_line(deps).into_iter().map(PathBuf::from));
    }
    out
}

/// Every repo-local dependency path ninja recorded for a build dir, in the
/// order `ninja -t deps` prints them.
///
/// ninja folds the compiler's `-MD` lists into the binary `.ninja_deps` log (no
/// text `.d` survives), so this reads them back with a pure QUERY that dumps
/// the log and never builds. Paths outside the repo (system headers) are
/// dropped: they never change and would only add noise.
///
/// Empty when there is no `.ninja_deps`, `ninja` is unavailable, or the query
/// fails — again "nothing was measured", never "nothing changed".
pub(crate) fn ninja_dep_paths(build_dir: &Path) -> Vec<PathBuf> {
    if !build_dir.join(".ninja_deps").exists() {
        return Vec::new();
    }
    let Ok(output) = std::process::Command::new("ninja")
        .arg("-C")
        .arg(build_dir)
        .args(["-t", "deps"])
        .output()
    else {
        return Vec::new();
    };
    if !output.status.success() {
        return Vec::new();
    }
    let root = project_root();
    let text = String::from_utf8_lossy(&output.stdout);
    // Indented lines are dependency paths; unindented lines are the per-object
    // `<obj>: #deps …` headers.
    text.lines()
        .filter_map(|l| l.strip_prefix("    "))
        .map(PathBuf::from)
        .filter(|p| p.starts_with(&root))
        .collect()
}

/// Flatten a repo-relative path into one filesystem-safe component.
///
/// Shared by the staleness ledger and the shadow-cache store so a coordinate
/// has ONE on-disk name in both. Long paths (workspace fixtures nest deeply)
/// would blow past `NAME_MAX`, so they are tail-truncated and disambiguated by
/// a hash of the whole thing.
pub(crate) fn flatten_path_key(binary: &Path) -> String {
    let root = project_root();
    let rel = binary.strip_prefix(&root).unwrap_or(binary);
    let key: String = rel
        .to_string_lossy()
        .chars()
        .map(|c| {
            if c.is_alphanumeric() || c == '.' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if key.len() > 180 {
        format!("{}__{:x}", &key[key.len() - 140..], fnv1a(key.as_bytes()))
    } else {
        key
    }
}

pub(crate) fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Record a staleness verdict for `binary` and return the run it belongs to.
///
/// Best-effort: a ledger that cannot be written degrades to "first verdict",
/// which is exactly the pre-0445 behaviour. A probe must never fail because
/// bookkeeping did.
pub fn record_stale(binary: &Path) -> StaleHistory {
    let path = ledger_path(binary);
    let prev = fs::read_to_string(&path).ok().and_then(|s| parse_entry(&s));
    let entry = match prev {
        Some((n, since)) => StaleHistory {
            consecutive: n.saturating_add(1),
            since_epoch: since,
        },
        None => StaleHistory {
            consecutive: 1,
            since_epoch: now_epoch(),
        },
    };
    let _ = fs::create_dir_all(ledger_dir());
    let _ = fs::write(
        &path,
        format!(
            "{} {} {}\n",
            entry.consecutive,
            entry.since_epoch,
            binary.display()
        ),
    );
    entry
}

/// Whether a FRESH verdict rests on a probe that actually measured something,
/// and the line to print when it does not (issue 1045).
///
/// [`probe_accounting`] is rendered only inside a STALE message, so a degraded
/// probe was invisible in exactly the direction that matters: on the fresh path
/// "examined 0 inputs" and "examined 2286 inputs" read identically, and both
/// read as a pass. That asymmetry is what let issue 1005's symlink defect run
/// across the whole cross-compiled tree — `zpico_recorded_inputs` returned
/// **0 entries** for every FreeRTOS / NuttX / ThreadX fixture, so the probe
/// silently ran the hand-authored bootstrap walk its own doc comment calls
/// unreachable, and every verdict it produced was FRESH.
///
/// Two shapes count as degraded, and they are different failures:
///
/// * `examined == 0` — the probe compared NOTHING. A fresh verdict here is not
///   evidence of anything; it is the absence of a measurement.
/// * `UNMEASURED` — an arm could not read the input set the BUILD recorded and
///   fell back to a hand-authored list. That one fails safe (over-broad, so it
///   errs toward a false STALE), but it still means the verdict describes a
///   different input set than the one that was built.
///
/// Returns `None` when the probe measured normally, so a healthy run stays
/// silent. Split out from the printing so it can be tested without capturing
/// stderr.
pub fn fresh_verdict_warning() -> Option<String> {
    let examined = EXAMINED.with(Cell::get);
    let unmeasured = UNMEASURED.with(Cell::get);
    if examined > 0 && !unmeasured {
        return None;
    }
    let why = if examined == 0 {
        "it compared NOTHING"
    } else {
        "it could not read the input set the build recorded"
    };
    Some(format!(
        "[nros-tests] WARNING: fixture reported FRESH by a DEGRADED probe — {why}. \
         probe: {}. A fresh verdict from this probe is the absence of a measurement, \
         not evidence the binary is current; re-read it before trusting a green \
         (issues 1005, 1045). Set NROS_STRICT_STALENESS_PROBE=1 to make this a failure.",
        probe_accounting()
    ))
}

/// Clear the coordinate's stale run — it resolved fresh, so it is about to
/// produce a runtime result.
///
/// Also the one place a FRESH verdict is announced, so it is where a degraded
/// probe has to say so (issue 1045). Putting it here rather than at the four
/// `require_*_fresh` entry points is deliberate: every one of them ends in this
/// call, and a rule that lives at the choke point cannot be forgotten by the
/// fifth (issue 0196's shape).
pub fn record_fresh(binary: &Path) -> Result<(), String> {
    let _ = fs::remove_file(ledger_path(binary));
    let Some(warning) = fresh_verdict_warning() else {
        return Ok(());
    };
    let detail = format!("{warning}\n  binary: {}", binary.display());
    if strict_probe() {
        return Err(detail);
    }
    eprintln!("{detail}");
    Ok(())
}

/// Whether a degraded probe should FAIL rather than warn.
///
/// Off by default: making it fatal today would break every arm that has no
/// recorded input set, which is a much larger change than making the
/// degradation visible. The knob exists so a lane can opt in once the arms are
/// clean, which is how this stops being permanent.
pub fn strict_probe() -> bool {
    std::env::var_os("NROS_STRICT_STALENESS_PROBE").is_some()
}

fn parse_entry(s: &str) -> Option<(u32, u64)> {
    let mut it = s.split_whitespace();
    let n = it.next()?.parse().ok()?;
    let since = it.next()?.parse().ok()?;
    Some((n, since))
}

/// A coordinate currently carrying a stale run, for reporting.
pub struct NonRunning {
    pub binary: String,
    pub consecutive: u32,
    pub since_epoch: u64,
}

/// Every coordinate the ledger currently reports as non-running, most-stuck
/// first. Backs `just fixture-staleness`.
pub fn non_running() -> Vec<NonRunning> {
    let mut out = Vec::new();
    let Ok(entries) = fs::read_dir(ledger_dir()) else {
        return out;
    };
    for entry in entries.flatten() {
        let Ok(text) = fs::read_to_string(entry.path()) else {
            continue;
        };
        let mut it = text.split_whitespace();
        let (Some(n), Some(since)) = (it.next(), it.next()) else {
            continue;
        };
        let (Ok(consecutive), Ok(since_epoch)) = (n.parse(), since.parse()) else {
            continue;
        };
        out.push(NonRunning {
            binary: it.collect::<Vec<_>>().join(" "),
            consecutive,
            since_epoch,
        });
    }
    out.sort_by(|a, b| {
        b.consecutive
            .cmp(&a.consecutive)
            .then(a.since_epoch.cmp(&b.since_epoch))
    });
    out
}

// ---------------------------------------------------------------------------
// The verdict itself.
// ---------------------------------------------------------------------------

/// Build the one staleness error every probe returns.
///
/// One spelling so the accounting and the not-run counter cannot be present on
/// some arms and missing on others — the same reason the exemption rule is
/// shared. `what` names the artifact ("Test fixture", "Zephyr fixture"),
/// `newer` is the input that tripped it, `remedy` the rebuild command.
///
/// Issue 1016 — a STALE verdict on a coordinate the last fixture BUILD never
/// built is true and misleading: the binary is a leftover from an earlier,
/// wider build and the lane in front of you did not touch it. The verdict now
/// says so when the build's own stamp can prove it. Deliberately a MESSAGE and
/// not a reclassification: `tests/zephyr_leaf_staleness.rs` asserts on the
/// `Err` this returns, and a panic here would break the probe's own regression
/// test — the reading was wrong, not the verdict.
pub fn stale_error(what: &str, binary: &Path, newer: &Path, remedy: &str) -> TestError {
    let history = record_stale(binary);
    let lane_note = match crate::fixtures::lane::recorded_build_omits(binary) {
        Some(reason) => format!("\n  NOT BUILT BY THIS LANE: {reason}"),
        None => String::new(),
    };
    TestError::BuildFailed(format!(
        "{what} is STALE — a source is newer than the built binary:\n  \
         binary: {}\n  newer:  {}\n  probe:  {}{}{}\n\
         {remedy}",
        binary.display(),
        newer.display(),
        probe_accounting(),
        history.note(),
        lane_note,
    ))
}

/// A staleness verdict that is not an mtime comparison (the west-fixture CLI
/// stamp). Same ledger, same not-run counter, no accounting line — nothing was
/// compared.
pub fn stale_error_custom(binary: &Path, body: String) -> TestError {
    let history = record_stale(binary);
    TestError::BuildFailed(format!("{body}{}", history.note()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exemptions_are_one_rule_not_three() {
        let inplace = project_root().join("packages/rmw/zenoh/zpico-sys/c/include/zpico.h");
        assert_eq!(
            exempt_probe_input(&inplace),
            Some(Exemption::RegeneratedInPlace),
            "the header whose one-armed exemption was issue 0442"
        );
        // No profile dir in the literal — the predicate keys on the
        // `build/<pkg>-<hash>/out/` shape, and naming a profile here would be a
        // hardcoded target path the profile table no longer owns.
        let outdir = project_root().join("target/p/build/zpico-sys-abc123/out/zpico.o");
        assert_eq!(exempt_probe_input(&outdir), Some(Exemption::CargoOutDir));
        let real = project_root().join("packages/core/nros-core/src/lib.rs");
        assert_eq!(exempt_probe_input(&real), None);
    }

    #[test]
    fn accounting_reports_what_was_examined_and_exempted() {
        begin_probe();
        assert!(note_candidate(
            &project_root().join("packages/rmw/zenoh/zpico-sys/c/include/zpico.h")
        ));
        assert!(!note_candidate(
            &project_root().join("packages/core/nros-core/src/lib.rs")
        ));
        let line = probe_accounting();
        assert!(line.contains("examined 2"), "{line}");
        assert!(
            line.contains("exempted 1 regenerated-in-place header"),
            "{line}"
        );
    }

    #[test]
    fn a_fresh_resolution_clears_the_not_run_run() {
        let key = project_root().join("target/nros-fixture-staleness-selftest/bin-a");
        let _ = record_fresh(&key);
        assert_eq!(record_stale(&key).consecutive, 1);
        assert_eq!(record_stale(&key).consecutive, 2);
        let second = record_stale(&key);
        assert_eq!(second.consecutive, 3);
        assert!(
            second.note().contains("NOT RUN"),
            "a coordinate stale three runs running must say so: {}",
            second.note()
        );
        let _ = record_fresh(&key);
        assert_eq!(
            record_stale(&key).consecutive,
            1,
            "running the fixture must reset the count, else the counter measures \
             age rather than non-running"
        );
        let _ = record_fresh(&key);
    }

    #[test]
    fn a_single_stale_verdict_stays_quiet() {
        let key = project_root().join("target/nros-fixture-staleness-selftest/bin-b");
        let _ = record_fresh(&key);
        let first = record_stale(&key);
        assert_eq!(first.consecutive, 1);
        assert!(
            first.note().is_empty(),
            "stale-since-your-last-edit is the normal case and must not shout"
        );
        let _ = record_fresh(&key);
    }

    /// issue 1045 — a FRESH verdict from a probe that measured nothing must SAY so.
    ///
    /// The negative control is the point: a healthy probe stays silent, so the
    /// warning cannot become background noise that a reader learns to skip.
    #[test]
    fn a_degraded_probe_announces_itself_on_the_fresh_path() {
        begin_probe();
        assert!(
            fresh_verdict_warning().is_some(),
            "a probe that examined ZERO inputs reported fresh without complaint — \
             that is issue 1005's symlink defect made invisible again"
        );
        assert!(
            fresh_verdict_warning()
                .unwrap()
                .contains("compared NOTHING"),
            "the warning must name WHICH degradation happened"
        );
    }

    #[test]
    fn an_unmeasured_input_set_announces_itself_even_when_candidates_were_seen() {
        begin_probe();
        // A hand-authored fallback DOES examine candidates, so `examined > 0`
        // alone would miss it — the two degradations are different failures.
        let seen = project_root().join("target/nros-1045-probe/some-input.rs");
        note_candidate(&seen);
        note_unmeasured_input_set();
        let warning = fresh_verdict_warning().expect("an unmeasured input set is degraded");
        assert!(
            warning.contains("could not read the input set"),
            "got: {warning}"
        );
        assert!(
            warning.contains("INPUT SET UNMEASURED"),
            "the accounting line must travel with the warning; got: {warning}"
        );
    }

    #[test]
    fn a_measured_probe_says_nothing() {
        begin_probe();
        let seen = project_root().join("target/nros-1045-probe/measured-input.rs");
        note_candidate(&seen);
        assert!(
            fresh_verdict_warning().is_none(),
            "a probe that measured normally must stay silent, or the warning \
             becomes noise and stops being read"
        );
    }
}
