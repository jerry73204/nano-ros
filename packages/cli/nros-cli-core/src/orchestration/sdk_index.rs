//! Phase 187.1 — the SDK package index that `nros setup` reads.
//!
//! `nros-sdk-index.toml` is the versioned manifest of host toolchains/tools.
//! Each `[tool.*]` carries a per-host prebuilt `dist` (GitHub Release asset URL,
//! sha256) **and** a `[tool.*.source]` recipe used when no `dist` matches the
//! host — both install into the same `$NROS_HOME/sdk/<tool>/<version>/` prefix.
//! `[source.*]` packages build with the app (target-compiled, never prebuilt);
//! `[gated.*]` are license-gated (never fetched/built — instruct + env check).
//!
//! This module is the format + loader (the rest of `nros setup` — board
//! resolution, fetch/cache, the CI release gate — is Phase 187.2–187.5). See
//! `docs/design/0014-nros-setup-toolchain-management.md`.

use std::{collections::BTreeMap, path::Path};

use eyre::{Result, WrapErr, bail};
use serde::{Deserialize, Serialize};

/// The whole `nros-sdk-index.toml`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SdkIndex {
    /// Prebuilt host tools (qemu, cross-gcc, zenohd, …), keyed by tool name.
    #[serde(default)]
    pub tool: BTreeMap<String, ToolPackage>,
    /// Source packages built with the app (kernels, small C libs), by name.
    #[serde(default)]
    pub source: BTreeMap<String, SourcePackage>,
    /// License-gated packages (never hosted/built), by name.
    #[serde(default)]
    pub gated: BTreeMap<String, GatedPackage>,
    /// RMW → host package set (Phase 191.6.a). The RMW axis is orthogonal to the
    /// board/platform axis: a board lists only its platform/toolchain packages,
    /// the chosen RMW contributes its host daemon/tool (`zenohd` / `xrce-agent`
    /// / `cyclonedds`). `nros setup <board> --rmw <name>` resolves
    /// `board.packages ∪ rmw.packages` — no `board×rmw` pair enumeration.
    #[serde(default)]
    pub rmw: BTreeMap<String, RmwEntry>,
    /// Board → required package set (Phase 191.1). The board→toolchain SSOT that
    /// ships with the index — replaces board-name keyword guessing in
    /// `resolve_packages`. Keyed by the canonical board id the user passes to
    /// `nros setup <board>`.
    #[serde(default)]
    pub board: BTreeMap<String, BoardEntry>,
    /// Named source groupings not tied to a single board/rmw (Phase 197.2) —
    /// e.g. `[reference.px4]`. Consumed by `tools/setup.sh --with-reference`,
    /// NOT by `nros setup`.
    #[serde(default)]
    pub reference: BTreeMap<String, ReferenceEntry>,
    /// #0390 — the `[source.*]` the REPO's OWN build stage needs, as a UNION
    /// (distinct from the per-board/per-rmw `build_sources`, which cover building
    /// an APP for one target). `just test` links every RMW's `-sys` crate, and
    /// `build-test-fixtures` resolves graphs that path-dep platform sources
    /// (`nuttx-libc`, `px4-rs`) even for a native component — so the contributor
    /// build needs this whole set regardless of which board/rmw was provisioned.
    /// `nros setup --build-sources` provisions them; `--build-sources --check` is
    /// the preflight `just test` / `build-test-fixtures` run before building.
    #[serde(default)]
    pub build_sources: Vec<String>,
    /// phase-327 W1 (RFC-0062) — OS packages by ABSTRACT key, mapped per
    /// package manager. The class `apt-packages` + every module's ad-hoc
    /// prereq probe moves into. `nros setup --system` composes the detected
    /// manager's install command and PRINTS it (`--sudo` to run);
    /// `--system --check` runs the probes (the doctor surface).
    #[serde(default)]
    pub system: BTreeMap<String, SystemDep>,
    /// `[prereq.<key>]` — the prerequisite namespace (RFC-0062, amended
    /// 2026-08-29). Supersedes `[system.*]`, which parses as an alias for
    /// `provider = "system"`. Read the two through [`SdkIndex::prereqs`],
    /// never directly, so a consumer cannot see only half the table.
    #[serde(default)]
    pub prereq: BTreeMap<String, PrereqDep>,
    /// phase-327 W1 — the Rust layer (pinned toolchains, targets, cargo
    /// tools), previously living in `just workspace` recipe bodies.
    #[serde(default)]
    pub rust: RustSection,
    /// phase-327 W1 — pip-installed tools (west, colcon, …), previously
    /// scattered per module.
    #[serde(default)]
    pub python: BTreeMap<String, PythonDep>,
}

/// phase-327 W1 (RFC-0062) — one OS package, declared by abstract key.
///
/// Per-manager mappings are explicit fields (not a flattened map) so
/// `deny_unknown_fields` keeps catching typos; adding a manager is a schema
/// change, which is the point — mappings are reviewed, not guessed.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemDep {
    /// One line of intent — surfaces in the composed plan so the user knows
    /// what they are installing and can prune.
    #[serde(default)]
    pub why: Option<String>,
    #[serde(default)]
    pub apt: Vec<String>,
    #[serde(default)]
    pub dnf: Vec<String>,
    #[serde(default)]
    pub pacman: Vec<String>,
    #[serde(default)]
    pub brew: Vec<String>,
    /// Presence probe. Optional: an entry without one is composed into the
    /// install command but reported `unknown` by `--check`.
    #[serde(default)]
    pub check: Option<CheckProbe>,
}

/// Which mechanism installs a prerequisite.
///
/// All four already existed as separate index classes before this table did —
/// `[system.*]`, `[tool.*].dist`, `[tool.*].source` + `install`, `[source.*]`.
/// What did not exist was one NAME a consumer could write without knowing which
/// of the four answers it. `board.packages` had been doing exactly that
/// informally: its entries resolve across `[tool.*]`, `[source.*]`, `[gated.*]`
/// and `[system.*]` already.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Provider {
    /// An OS package, via the detected package manager. The default, so every
    /// `[system.*]` entry is a valid `[prereq.*]` entry unchanged.
    #[default]
    System,
    /// A prebuilt dist from the store — the existing `[tool.<key>].dist`.
    Sdk,
    /// Built from source into the store — `[tool.<key>].source` + `install`.
    Source,
    /// A checkout in the tree — the existing `[source.<key>]`.
    Submodule,
}

/// `[prereq.<key>]` — one prerequisite, one or more providers.
///
/// The OS-package fields are flattened in rather than nested under a
/// `[prereq.x.system]` sub-table, so an existing `[system.*]` entry is
/// byte-identical as a `[prereq.*]` entry. That is the whole migration for 25
/// of 25 current entries.
/// Who may name a `[prereq.*]` key.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrereqRole {
    /// A package's CONTENT depends on it, so a `package.xml` may name it and
    /// `nros setup --workspace` can discover it. `cmake`, `clang`, a ROS
    /// package the code links.
    Package,
    /// A repo-level recipe needs it, and no package does: `doxygen` for
    /// `just docs-c`, `gnu-parallel` for the gate runner. Reached by running
    /// that recipe's scope, never by declaring a dependency.
    Workspace,
    /// Test/build INFRASTRUCTURE for a target: emulators, cross toolchains,
    /// on-chip debug probes. Comes from WHERE you deploy, not from what your
    /// code needs — `<depend>qemu-system-arm</depend>` is a category error.
    Infra,
    /// A third-party source tree this repo builds (submodule or fetched
    /// source). ROS 2 would express these as `*_vendor` packages.
    Vendor,
    /// Not yet classified. The DEFAULT so the field can land before all 46
    /// entries carry one; `check-prereq-roles` is what forbids it staying.
    #[default]
    Unclassified,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrereqDep {
    #[serde(default)]
    pub why: Option<String>,
    /// WHO may name this key, and therefore how a user reaches it.
    ///
    /// RFC-0062 amendment 3. Measured before adding it: of 46 keys, exactly
    /// THREE are ever named by a `package.xml` (`cargo`, `cmake`, `nros`).
    /// The rest are provisioning facts no package's CONTENT depends on, and
    /// conflating the two is why `nros setup` cannot answer "what does MY
    /// workspace need" — it has no way to tell a dependency from a toolchain.
    #[serde(default)]
    pub role: PrereqRole,
    /// Ordered providers. Empty means "just `provider`", which defaults to
    /// `system` — so the common single-provider entry writes neither field.
    ///
    /// ORDER is preference, and whether it may become a policy-driven chain
    /// (never build from source in CI; prefer the dist offline) is deliberately
    /// still open in RFC-0062 — it interacts with RFC-0065 D14.
    #[serde(default)]
    pub providers: Vec<Provider>,
    #[serde(default)]
    pub provider: Provider,
    #[serde(default)]
    pub apt: Vec<String>,
    #[serde(default)]
    pub dnf: Vec<String>,
    #[serde(default)]
    pub pacman: Vec<String>,
    #[serde(default)]
    pub brew: Vec<String>,
    /// Every soname this entry satisfies, when it satisfies more than the one
    /// `check.sharedlib` probes — issue 0926. `libssl3` provides BOTH
    /// `libssl.so.3` and `libcrypto.so.3`; `libglib2` provides four.
    ///
    /// Exists so `check-dist-runtime-deps` can map a measured soname back to
    /// the key a `system = [..]` must name, WITHOUT keeping a soname table of
    /// its own — a second mapping beside this one is the drift that let
    /// `[tool.qemu] system = ["libslirp"]` stand while 19 sonames went
    /// undeclared. The probe stays `check.sharedlib`: one soname is enough to
    /// decide presence, and probing four would only be slower.
    #[serde(default)]
    pub provides: Vec<String>,
    /// The key in `[tool.*]` / `[source.*]` this resolves through, when the
    /// provider is not `system` and the names differ. Absent ⇒ the prereq key
    /// IS the class key.
    #[serde(default)]
    pub source: Option<String>,
    #[serde(default)]
    pub check: Option<CheckProbe>,
}

impl PrereqDep {
    /// The providers to try, in order. Never empty: an entry that declares
    /// neither field means `system`.
    #[must_use]
    pub fn provider_chain(&self) -> Vec<Provider> {
        if self.providers.is_empty() {
            vec![self.provider]
        } else {
            self.providers.clone()
        }
    }

    #[must_use]
    pub fn packages_for(&self, manager: &str) -> &[String] {
        match manager {
            "apt" => &self.apt,
            "dnf" => &self.dnf,
            "pacman" => &self.pacman,
            "brew" => &self.brew,
            _ => &[],
        }
    }
}

impl From<&SystemDep> for PrereqDep {
    /// `[system.*]` lowered to `provider = "system"` — the alias that makes W1
    /// additive. Every field maps; none is dropped.
    fn from(d: &SystemDep) -> Self {
        Self {
            why: d.why.clone(),
            // A lowered `[system.*]` entry carries no role: the alias exists to
            // make the merge additive, and inventing a classification here
            // would put it somewhere no author chose. `check-prereq-roles`
            // reports it as unclassified, which is the truth.
            role: PrereqRole::Unclassified,
            providers: Vec::new(),
            provider: Provider::System,
            apt: d.apt.clone(),
            dnf: d.dnf.clone(),
            pacman: d.pacman.clone(),
            brew: d.brew.clone(),
            // Empty, and that is not a dropped field: `[system.*]` has no
            // `provides` to map. The legacy shape predates it, and phase-398
            // W4 retired `[system.*]` outright — an entry arriving here is a
            // pre-migration index, which by definition declares no soname
            // beyond its probe.
            provides: Vec::new(),
            source: None,
            check: d.check.clone(),
        }
    }
}

impl SystemDep {
    /// The native package list for `manager` ("apt" | "dnf" | "pacman" |
    /// "brew"), empty when unmapped.
    pub fn packages_for(&self, manager: &str) -> &[String] {
        match manager {
            "apt" => &self.apt,
            "dnf" => &self.dnf,
            "pacman" => &self.pacman,
            "brew" => &self.brew,
            _ => &[],
        }
    }
}

/// A presence probe for a [`SystemDep`] / [`PythonDep`]. At LEAST one field —
/// validated, since serde alone would accept an empty table.
///
/// Issue 0487 — the fields are OR-ed, not exclusive: a package can need two
/// spellings to be found on two distros (libgcrypt ships a `.pc` on Arch and a
/// `libgcrypt-config` script on Ubuntu 22.04, never both), so declaring several
/// is correct usage rather than ambiguity. `336eb1724` relaxed the check and
/// left this sentence saying the opposite.
/// A version requirement on a probed command (phase-404 W1).
///
/// BOTH `min` and `exact` exist, and the pair is the point. A floor is right
/// where newer is fine — `openocd` 0.13 would serve a 0.12 pin. It is WRONG
/// where the pin is an interop contract: our cyclonedds must be the cyclonedds
/// ROS ships (issue 0507), and exceeding that breaks interop exactly as surely
/// as falling short. A single `min` would have quietly licensed the upgrade
/// that issue 0609 measured going wrong for the zenoh router.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VersionConstraint {
    /// Args that make the command print its version. `--version` by default,
    /// which is right for every tool in the index today.
    #[serde(default)]
    pub run: Option<String>,
    /// Lowest acceptable version (inclusive).
    #[serde(default)]
    pub min: Option<String>,
    /// The ONLY acceptable version, compared on the components `exact` names —
    /// `exact = "0.10"` accepts 0.10.5. Use where the pin is a contract.
    #[serde(default)]
    pub exact: Option<String>,
    /// Escape hatch: a literal that must appear immediately BEFORE the version
    /// in the output, when the default "first version-shaped token" is wrong.
    ///
    /// Deliberately not a regex. phase-404 W1 chose a known-shape default over
    /// per-entry patterns because a regex per entry is a second place for the
    /// index to drift; this covers the exception without opening that door.
    #[serde(default)]
    pub after: Option<String>,
}

/// The first version-shaped token in `text`, or the first after `marker`.
///
/// "Version-shaped" is a digit run containing at least one dot, which is what
/// every tool in this index prints: `Open On-Chip Debugger 0.12.0-g9ea7f3d`,
/// `QEMU emulator version 11.0.0`, `arm-none-eabi-gcc (Arm GNU Toolchain
/// 13.2.rel1 ...)`. Trailing non-numeric junk (`-g9ea7f3d`, `.rel1`) is left
/// behind by construction, since the scan stops at the first byte that is
/// neither digit nor dot.
#[must_use]
pub fn extract_version(text: &str, marker: Option<&str>) -> Option<String> {
    let hay = match marker {
        Some(m) => {
            let at = text.find(m)?;
            &text[at + m.len()..]
        }
        None => text,
    };
    let bytes = hay.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if !bytes[i].is_ascii_digit() {
            i += 1;
            continue;
        }
        // A version token must not start mid-number (`arm-13.7` -> `13.7`, but
        // only once the `-` has ended the previous run).
        let start = i;
        let mut j = i;
        while j < bytes.len() && (bytes[j].is_ascii_digit() || bytes[j] == b'.') {
            j += 1;
        }
        let tok = &hay[start..j];
        // Trailing dot is punctuation, not part of the version.
        let tok = tok.trim_end_matches('.');
        if tok.contains('.') {
            return Some(tok.to_string());
        }
        i = j.max(i + 1);
    }
    None
}

/// Compare dotted numeric versions component-wise. Missing components are 0,
/// so `0.12` and `0.12.0` compare equal.
#[must_use]
fn version_cmp(a: &str, b: &str) -> std::cmp::Ordering {
    let parts = |v: &str| -> Vec<u64> {
        v.split('.')
            .map(|p| p.parse::<u64>().unwrap_or(0))
            .collect()
    };
    let (x, y) = (parts(a), parts(b));
    for i in 0..x.len().max(y.len()) {
        let ord = x
            .get(i)
            .copied()
            .unwrap_or(0)
            .cmp(&y.get(i).copied().unwrap_or(0));
        if ord != std::cmp::Ordering::Equal {
            return ord;
        }
    }
    std::cmp::Ordering::Equal
}

/// Does `found` satisfy the constraint? `exact` compares only the components it
/// names, so `exact = "0.10"` accepts `0.10.5` and rejects `0.11.0`.
#[must_use]
pub fn version_satisfies(found: &str, c: &VersionConstraint) -> bool {
    if let Some(want) = &c.exact {
        let depth = want.split('.').count();
        let trimmed: String = found.split('.').take(depth).collect::<Vec<_>>().join(".");
        if version_cmp(&trimmed, want) != std::cmp::Ordering::Equal {
            return false;
        }
    }
    if let Some(floor) = &c.min
        && version_cmp(found, floor) == std::cmp::Ordering::Less
    {
        return false;
    }
    true
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CheckProbe {
    /// `command -v <cmd>` succeeds.
    #[serde(default)]
    pub cmd: Option<String>,
    /// phase-404 W1 — is the found copy NEW ENOUGH? Only meaningful beside
    /// `cmd`, which is what it runs.
    ///
    /// Every other field here answers "is it present". That is the wrong
    /// question the moment a prerequisite can come from more than one provider:
    /// Ubuntu 22.04 has `openocd`, at 0.11.0 against a 0.12.0 pin, so presence
    /// and satisfaction differ by a version nobody could express.
    #[serde(default)]
    pub version: Option<VersionConstraint>,
    /// A shared library the dynamic linker can find (`ldconfig -p` on Linux;
    /// skipped elsewhere — the entry reports `unknown` there).
    ///
    /// Matches by PREFIX, so this answers "is some build of this library
    /// loadable", NOT "is the `-dev` package installed". Those differ: the
    /// runtime package ships `libfoo.so.N`, which a `sharedlib = "libfoo.so"`
    /// probe matches, while the headers and the unversioned symlink come from
    /// `-dev`. Use this when the CONSUMER needs the library at run time — a
    /// `dlopen` (bindgen/libclang) or a runtime dependency (libslirp). When the
    /// consumer compiles against headers, use [`Self::header`]; issue 0603 was
    /// this probe reporting mbedTLS present on a host with only `libmbedtls14`.
    #[serde(default)]
    pub sharedlib: Option<String>,
    /// `pkg-config --exists <name>` succeeds (dev headers).
    ///
    /// The right probe for a `-dev` package that ships a `.pc` — but many do
    /// not (Ubuntu's `libmbedtls-dev` is why `generate_mbedtls_pc_files` has to
    /// fabricate one), so [`Self::header`] covers the rest.
    #[serde(default)]
    pub pkg_config: Option<String>,
    /// A C header exists on the compiler's default search path — the honest
    /// test for a `-dev` package whose consumer `#include`s it (issue 0603).
    ///
    /// Written as the include spelling, not a path: `mbedtls/entropy.h`, so
    /// the probe and the `#include` that needs it are the same string. Checked
    /// against `/usr/include`, `/usr/local/include` and the Debian multiarch
    /// dir; Linux-only, `unknown` elsewhere, same as `sharedlib`.
    ///
    /// Not right for every `-dev` package: a VERSIONED one (`libclang-14-dev`)
    /// installs under `/usr/lib/llvm-14/include`, so a probe here would be a
    /// false negative on a host that genuinely has it.
    #[serde(default)]
    pub header: Option<String>,
    /// phase-398 W2 — the resolved binary EXECUTES.
    ///
    /// Not `cmd`, which is `command_exists`: a PATH lookup, and a store dist is
    /// not on PATH. This is the probe the motivating failure needed — the QEMU
    /// dist's path existed and the dynamic loader was the first thing to
    /// disagree (`libslirp.so.0` missing).
    ///
    /// The value is a command line; a zero exit is PRESENT. `Unknown` where it
    /// cannot answer — a foreign-platform tool cannot be run here, and "cannot
    /// execute" is not "absent".
    #[serde(default)]
    pub runs: Option<String>,
    /// phase-398 W2 — a file that must exist inside the provider's checkout.
    ///
    /// For `source`/`submodule` providers, which have no PATH entry and no
    /// soname: today their presence test is "the directory exists", which is
    /// true of an uninitialised submodule. Relative to the provider's own
    /// `dest`, so the probe does not restate a location the provider already
    /// declares.
    #[serde(default)]
    pub path: Option<String>,
}

impl CheckProbe {
    fn field_count(&self) -> usize {
        [
            self.cmd.is_some(),
            self.sharedlib.is_some(),
            self.pkg_config.is_some(),
            self.header.is_some(),
        ]
        .iter()
        .filter(|b| **b)
        .count()
    }
}

/// phase-327 W1 — the Rust layer.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RustSection {
    /// Pinned toolchains, keyed by a stable alias (`nightly-pinned`).
    #[serde(default)]
    pub toolchain: BTreeMap<String, RustToolchain>,
    /// rustup targets, keyed by a short alias.
    #[serde(default)]
    pub target: BTreeMap<String, RustTarget>,
    /// `cargo install`ed tools, keyed by binary-ish alias.
    #[serde(default, rename = "cargo-tool")]
    pub cargo_tool: BTreeMap<String, RustCargoTool>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RustToolchain {
    /// rustup channel (`nightly-2026-04-11`, `stable`).
    pub channel: String,
    #[serde(default)]
    pub components: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RustTarget {
    pub triple: String,
    /// Alias of the `[rust.toolchain.*]` this target installs under; `None`
    /// = the default toolchain.
    #[serde(default)]
    pub toolchain: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RustCargoTool {
    /// The crates.io crate name (`cargo-nextest`).
    #[serde(rename = "crate")]
    pub crate_name: String,
    #[serde(default)]
    pub version: Option<String>,
    /// `cargo install --locked` (default true).
    #[serde(default = "default_true")]
    pub locked: bool,
    #[serde(default)]
    pub check: Option<CheckProbe>,
}

/// phase-327 W1 — one pip-installed tool.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PythonDep {
    /// The PyPI distribution name.
    pub pip: String,
    #[serde(default)]
    pub version: Option<String>,
    #[serde(default)]
    pub why: Option<String>,
    #[serde(default)]
    pub check: Option<CheckProbe>,
}

/// A named `[reference.*]` source grouping (Phase 197.2).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceEntry {
    /// `[source.*]` names this reference set pulls.
    #[serde(default)]
    pub sources: Vec<String>,
}

/// An RMW's host package set — the orthogonal RMW axis (Phase 191.6.a).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RmwEntry {
    /// The index package names (`[tool]`/`[source]`/`[gated]`) this RMW's host
    /// side needs — e.g. `["zenohd"]`, `["xrce-agent"]`, `["cyclonedds"]`.
    #[serde(default)]
    pub packages: Vec<String>,
    /// `[source.*]` names built with the app for this RMW (Phase 197.2). Consumed
    /// by `tools/setup.sh` (the local dev provisioner), NOT by `nros setup` —
    /// recorded here so the index is the single source manifest.
    #[serde(default)]
    pub build_sources: Vec<String>,
    /// Opt-in dev `[source.*]` (full upstream repos, for hacking on the RMW).
    #[serde(default)]
    pub dev_sources: Vec<String>,
}

/// A prebuilt host tool: a per-host `dist` map + an optional `source` fallback.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolPackage {
    pub version: String,
    /// The exact upstream revision the prebuilt is built/repackaged from (Phase
    /// 191.2) — e.g. ARM `13.2.rel1`, xPack `14.2.0-3`, a fork branch. The SSOT
    /// the build scripts consume (as the `build-tool.yml` `upstream` input)
    /// instead of hardcoding/hand-deriving it. For tools with a `source` recipe
    /// this equals `source.ref`; recorded here too for dist-only tools.
    #[serde(default)]
    pub upstream: Option<String>,
    /// host key (`<os>-<arch>`, e.g. `linux-x86_64`) → prebuilt artifact.
    #[serde(default)]
    pub dist: BTreeMap<String, DistArtifact>,
    /// Build-from-source recipe used when no `dist` matches the host.
    #[serde(default)]
    pub source: Option<ToolSource>,
    /// phase-327 W4 (RFC-0062) — `[system.*]` keys this tool's DIST needs at
    /// RUNTIME (e.g. qemu-nros links `libslirp.so.0` dynamically, which stock
    /// Ubuntu does not ship). `nros setup --tool` checks + names them before
    /// the smoke check can fail with a bare loader error.
    #[serde(default)]
    pub system: Vec<String>,
    /// issue 0929 — commands that must WORK once the dist is installed.
    ///
    /// `system` answers "are the libraries it links present?". That is not the
    /// same question as "does it run", and the gap is not hypothetical:
    /// `arm-none-eabi-gdb` resolves every shared library it names and still
    /// does nothing, because ARM embeds a Python that fails to initialise. So
    /// `--check` reported `[OK]` for a toolchain with a dead debugger — the
    /// exact shape issue 0926 removed for openocd, reappearing one cause over.
    #[serde(default)]
    pub smoke: Vec<SmokeCheck>,
    /// phase-431 W3 — prefix-relative paths (`bin/nros`) that must be reachable
    /// by BARE NAME from `$NROS_HOME/bin`, which is the one directory a user
    /// puts on PATH.
    ///
    /// The store accumulates versions and the link always points at the NEWEST
    /// installed one, so `nros` means one command however many versions are
    /// present. That is the promise; `sdk_store::front_newest` is where it is
    /// kept.
    ///
    /// This is NOT `scripts/sdk-path-tools.txt`. That list puts a store `bin/`
    /// dir on PATH for tools an RTOS `make` or a cmake `find_program` invokes
    /// by bare name, and `nros` must never be on it: it would shadow the
    /// checkout's own CLI in a contributor's shell, which `just doctor` now
    /// fails on (phase-431 W2) and `nros build` refuses (W1).
    #[serde(default)]
    pub front: Vec<String>,
}

/// One "does it actually run?" probe for a `[tool.*]` dist (issue 0929).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmokeCheck {
    /// Argv, relative to the install prefix (`bin/arm-none-eabi-gdb --version`).
    pub run: String,
    /// Substring the command must print. Checked on stdout AND stderr, because
    /// a tool is free to write its banner to either.
    ///
    /// Required, and the reason is the bug that motivated this: gdb EXITS 0
    /// while printing nothing at all, so an exit-status probe calls it healthy.
    /// Asserting on output is what distinguishes "ran" from "started".
    pub expect: String,
}

/// A board's required SDK package set — the board→toolchain SSOT (Phase 191.1).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardEntry {
    /// Target arch family (descriptive: `cortex-m3`, `riscv32`, `x86_64`, …).
    #[serde(default)]
    pub arch: Option<String>,
    /// Platform / RTOS (descriptive: `bare-metal`, `freertos`, `posix`, …).
    #[serde(default)]
    pub platform: Option<String>,
    /// The index package names (`[tool]`/`[source]`/`[gated]`) this board needs.
    /// Explicit — no derivation, no board-name guessing. May be empty (e.g. an
    /// ESP32-C3 board whose riscv32 toolchain is rustup-managed).
    #[serde(default)]
    pub packages: Vec<String>,
    /// `[source.*]` names built with the app for this board (Phase 197.2).
    /// Consumed by `tools/setup.sh`, NOT by `nros setup <board>` (they're
    /// target-compiled with the app, not host tools) — recorded here so the
    /// index is the single source manifest.
    #[serde(default)]
    pub build_sources: Vec<String>,
    /// Opt-in dev `[source.*]` (full upstream repos, for in-tree development).
    #[serde(default)]
    pub dev_sources: Vec<String>,
}

/// A prebuilt artifact for one host.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DistArtifact {
    pub url: String,
    pub sha256: String,
    /// How to turn the downloaded file into a populated prefix, when the
    /// default cannot.
    ///
    /// Default (absent): `tar -xf <archive> -C <prefix>` — right for the
    /// `.tar.zst` assets on the nano-ros-sdk mirror, which are built to unpack
    /// prefix-rooted (`bin/…`).
    ///
    /// It is wrong for an UPSTREAM asset, and that is the case this exists for.
    /// Policy is prebuilt-when-upstream-ships-one, source only as fallback, so
    /// the index has to take assets shaped the way their projects publish them
    /// rather than the way our mirror does. ninja's `ninja-linux.zip` is both
    /// kinds of wrong at once: not a tar, and a bare binary with no `bin/`.
    ///
    /// Free-form shell, like `[tool.*.source].install` — the same reason, that
    /// the unpack step genuinely varies per project and a fixed vocabulary of
    /// archive types would just be a smaller ad-hoc recipe. `{archive}` is the
    /// downloaded file, `{prefix}` the install prefix.
    #[serde(default)]
    pub install: Option<String>,
}

/// The source-build fallback recipe — installs into the same prefix as `dist`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolSource {
    /// Git remote to fetch from. Mutually exclusive with [`url`](Self::url);
    /// exactly one of the two must be set (checked by `validate`).
    #[serde(default)]
    pub git: Option<String>,
    /// Git ref (tag/sha) — pinned in lockstep with the prebuilt `version`.
    #[serde(rename = "ref", default)]
    pub git_ref: Option<String>,
    /// A source TARBALL to fetch instead of cloning.
    ///
    /// Exists because git is not always a way to get a buildable tree. GNU
    /// make's git tree at `4.4.1` ships no `configure` at all — only
    /// `bootstrap`, which wants autoconf/automake/gettext and pulls gnulib —
    /// so the release tarball is the only sane input, and a git-only schema
    /// pushed `make` out of the index into an ad-hoc `just` recipe.
    ///
    /// The archive is fetched, sha256-verified and unpacked into the build
    /// directory with `--strip-components=1`, so `configure`/`install` see the
    /// project root exactly as the git path leaves it. Everything after the
    /// fetch is therefore identical between the two modes.
    #[serde(default)]
    pub url: Option<String>,
    /// Checksum for [`url`](Self::url). Required with it — an unverified
    /// download is a supply-chain hole, and the git path gets its integrity
    /// from the pinned ref.
    #[serde(default)]
    pub sha256: Option<String>,
    /// Configure step; `{prefix}` is substituted with the install prefix.
    #[serde(default)]
    pub configure: Option<String>,
    /// Build + install step.
    #[serde(default)]
    pub install: Option<String>,
    /// Issue 0374 direction 4 — honour the CHECKOUT's own `rust-toolchain.toml`
    /// instead of building with the workspace's pinned channel.
    ///
    /// Default `false`: a recipe whose checkout pins a different Rust version
    /// makes rustup download a whole second toolchain during `nros setup`,
    /// unannounced (zenoh 1.7.2 pins 1.85.0 and does exactly that). Building it
    /// with the channel the workspace already has avoids the download.
    ///
    /// Set `true` for a recipe that genuinely needs its own pin — a
    /// nightly-only crate cannot be built by a stable channel, and forcing one
    /// would turn a working recipe into a compile error.
    #[serde(default)]
    pub respect_toolchain: bool,
}

/// A package compiled with the user's app for their chosen target.
///
/// Phase 195.B — `[source.*]` provisioning is data-driven: `nros setup`
/// fetches the source into [`dest`](Self::dest) from index data, never a
/// hardcoded `third-party/` path. `submodule` is an optional *mode hint*:
/// - **clone mode** (`git` + `ref` + `dest`, no `submodule`): fresh
///   `git clone`@`ref`. Here the two fields ARE the pin — nothing else records
///   it — so they are required in practice even though the type allows `None`.
/// - **submodule mode** (`submodule` + `dest`, and NO `git`/`ref`):
///   `git submodule update --init <submodule>`, then the gitlink sha.
///
/// Issue 0602 — submodule mode carried `git`/`ref` too, described here as
/// recording "the canonical pin (the SSOT — so `.gitmodules` and the index
/// can't drift)". That claim was false in both halves. The provisioning path
/// never reads them: it takes `submodule`, runs `submodule update --init`, and
/// resolves the commit with `ls-tree HEAD <path>`. And `.gitmodules` plus the
/// gitlink already hold that fact AUTHORITATIVELY, with git enforcing them —
/// so the index held a third recording that nothing kept true, and it had
/// drifted on 6 of the 14 entries (threadx named upstream at a commit one
/// behind our fork's).
///
/// They are gone from those entries. Provenance for a submodule is
/// `.gitmodules` + the gitlink, which is the copy that cannot silently disagree
/// with what gets checked out.
///
/// A source with no fetch fields at all has no provisioning step (e.g. a
/// host-built package whose tree already lives in the workspace).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourcePackage {
    pub version: String,
    /// Git URL to clone (clone mode). Mutually exclusive with `submodule`.
    #[serde(default)]
    pub git: Option<String>,
    /// Git ref (tag/branch/sha) to check out — pinned in lockstep with
    /// `version`. Required in clone mode.
    #[serde(default, rename = "ref")]
    pub git_ref: Option<String>,
    /// Workspace-relative destination the source is provisioned into. The
    /// index is the SSOT — never a path baked into the `nros` binary.
    #[serde(default)]
    pub dest: Option<String>,
    /// `.gitmodules` path when the canonical source is a committed submodule;
    /// `nros setup` runs `git submodule update --init <path>` instead of a
    /// fresh clone. `git`/`ref` still record the pin (SSOT) in this mode.
    #[serde(default)]
    pub submodule: Option<String>,
    /// Shallow-fetch the submodule (`--depth 1`). Default `true` — pins lag the
    /// upstream branch tip, and `git submodule update --depth 1` fetches the
    /// pinned SHA directly (fetch-by-SHA), so this is a true depth-1 checkout,
    /// not a deepen-to-reach-pin. Set `shallow = false` for a source whose
    /// upstream rejects reachable-SHA shallow fetches. Submodule mode only.
    #[serde(default = "default_true")]
    pub shallow: bool,
    /// Recurse into the source's own nested submodules (`--recursive`). Default
    /// `true`. Only affects a source that *has* nested submodules; it never
    /// pulls sibling top-level sources (e.g. PX4-Autopilot is a separate
    /// `[source.*]`, not nested in `px4-rs`). Set `recursive = false` to pin a
    /// source to its top tree only. Submodule mode only.
    #[serde(default = "default_true")]
    pub recursive: bool,
}

fn default_true() -> bool {
    true
}

// Hand-rolled so `SourcePackage::default()` matches the serde defaults
// (`shallow`/`recursive` default to `true`); a `#[derive(Default)]` would make
// the bools `false` and silently diverge from a TOML-parsed entry.
impl Default for SourcePackage {
    fn default() -> Self {
        Self {
            version: String::new(),
            git: None,
            git_ref: None,
            dest: None,
            submodule: None,
            shallow: true,
            recursive: true,
        }
    }
}

/// How a [`SourcePackage`] is provisioned (Phase 195.B).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceProvision {
    /// `git clone <git> @ <ref>` into `dest`.
    Clone,
    /// `git submodule update --init <submodule>` (dest is the submodule path).
    Submodule,
    /// No fetch step — the tree already lives in the workspace.
    None,
}

impl SourcePackage {
    /// Which provisioning mode this entry declares (Phase 195.B).
    pub fn provision(&self) -> SourceProvision {
        if self.submodule.is_some() {
            SourceProvision::Submodule
        } else if self.git.is_some() {
            SourceProvision::Clone
        } else {
            SourceProvision::None
        }
    }
}

/// A license-gated package: never fetched or built; `nros setup` instructs the
/// user and `nros doctor` checks the `env` var points at the installed SDK.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GatedPackage {
    pub version: String,
    pub env: String,
    #[serde(default)]
    pub installer: Option<String>,
}

impl SdkIndex {
    /// Where a prereq's `path` probe resolves against, if its provider has a
    /// checkout at all.
    ///
    /// phase-398 W2. Only `submodule`/`source` providers have one: an OS
    /// package has no directory we own, and a `sdk` dist lives in the store
    /// under a version we do not know here. Returning `None` makes the probe
    /// abstain (`Unknown`) rather than test against a guessed root.
    ///
    /// The location comes from the provider's own `dest`, so the prereq entry
    /// never restates a path the `[source.*]` entry already declares — one
    /// fact, one place.
    #[must_use]
    pub fn prereq_checkout_dir(
        &self,
        key: &str,
        dep: &PrereqDep,
        repo_root: &Path,
    ) -> Option<std::path::PathBuf> {
        let wants_checkout = dep
            .provider_chain()
            .iter()
            .any(|p| matches!(p, Provider::Submodule | Provider::Source));
        if !wants_checkout {
            return None;
        }
        let class_key = dep.source.as_deref().unwrap_or(key);
        let dest = self.source.get(class_key)?.dest.as_ref()?;
        Some(repo_root.join(dest))
    }

    /// One warning per `[system.<key>]` still declared — W4's retirement notice.
    ///
    /// RETURNED, not printed, and the suppression flag is a PARAMETER. That is
    /// deliberate and it is the lesson from phase-383 W1.f, which shipped a
    /// correct, well-tested deprecation lint with NO production caller: its
    /// warning reached nobody, and a removal on that basis would have landed on
    /// users who were never told. A warning observable only on stderr under an
    /// ambient env var cannot be tested for REACHABILITY, which is the property
    /// that was missing.
    #[must_use]
    pub fn deprecated_system_table_warnings(&self, suppressed: bool) -> Vec<String> {
        if suppressed || self.system.is_empty() {
            return Vec::new();
        }
        let mut keys: Vec<&String> = self.system.keys().collect();
        keys.sort();
        vec![format!(
            "[system.*] is deprecated and becomes [prereq.*] (RFC-0062, amended \
             2026-08-29): {} entr(ies) still declared — {}. Each is already read \
             as `provider = \"system\"`, so renaming the table header is the \
             whole migration. Removal at the next minor version. Set \
             NROS_SUPPRESS_DEPRECATION=1 to silence.",
            keys.len(),
            keys.iter()
                .take(4)
                .map(|k| k.as_str())
                .collect::<Vec<_>>()
                .join(", ")
                + if keys.len() > 4 { ", …" } else { "" },
        )]
    }

    /// A key declared in BOTH tables — illegal once the merge is complete.
    ///
    /// `genromfs` was in `[tool.*]` and `[system.*]` at once, tied together only
    /// by prose in `why` ("the [tool.genromfs] source recipe is the store
    /// alternative"). `[prereq.*]` expresses that as one entry with an ordered
    /// `providers` list, so the duplication has a replacement and can be
    /// refused rather than tolerated.
    #[must_use]
    pub fn duplicate_prereq_keys(&self) -> Vec<String> {
        let mut out: Vec<String> = self
            .prereq
            .keys()
            .filter(|k| self.system.contains_key(*k))
            .cloned()
            .collect();
        out.sort();
        out
    }

    /// Every prerequisite, `[prereq.*]` and `[system.*]` together.
    ///
    /// THE accessor: no consumer reads either table directly, because one that
    /// reads only `[system.*]` sees a shrinking half of the SSoT while the
    /// migration runs, and one that reads only `[prereq.*]` sees an empty table
    /// today. Both spellings coexist by design until W4 retires the alias.
    ///
    /// `[prereq.*]` WINS on a duplicate key. A key in both is a migration in
    /// progress — the new entry is the intended one — and W4 adds the gate that
    /// makes the overlap illegal once it should no longer exist.
    #[must_use]
    pub fn prereqs(&self) -> BTreeMap<String, PrereqDep> {
        let mut out: BTreeMap<String, PrereqDep> = self
            .system
            .iter()
            .map(|(k, v)| (k.clone(), PrereqDep::from(v)))
            .collect();
        for (k, v) in &self.prereq {
            out.insert(k.clone(), v.clone());
        }
        out
    }

    /// Read, parse, + validate an `nros-sdk-index.toml`.
    pub fn load(path: &Path) -> Result<Self> {
        let raw = std::fs::read_to_string(path)
            .wrap_err_with(|| format!("failed to read SDK index {}", path.display()))?;
        let idx =
            Self::parse(&raw).wrap_err_with(|| format!("invalid SDK index {}", path.display()))?;
        idx.validate()
            .wrap_err_with(|| format!("invalid SDK index {}", path.display()))?;
        Ok(idx)
    }

    /// Parse from a string (schema only — no cross-reference validation, so unit
    /// tests can parse partial fixtures). [`load`] additionally [`validate`]s.
    pub fn parse(raw: &str) -> Result<Self> {
        toml::from_str(raw).wrap_err("invalid nros-sdk-index.toml schema")
    }

    /// Phase 191.4 — every `[board.*].packages` name must be a defined
    /// `[tool]`/`[source]`/`[gated]` package. Phase 191.6.a extends this to
    /// `[rmw.*].packages`. Catches typos/renames that would otherwise silently
    /// skip (a board's/RMW's tool would just not install).
    pub fn validate(&self) -> Result<()> {
        let known = |pkg: &str| {
            self.tool.contains_key(pkg)
                || self.source.contains_key(pkg)
                || self.gated.contains_key(pkg)
        };
        for (board, entry) in &self.board {
            for pkg in &entry.packages {
                if !known(pkg) {
                    bail!(
                        "board '{board}' references undefined package '{pkg}' \
                         (not a [tool]/[source]/[gated] entry)"
                    );
                }
            }
        }
        for (rmw, entry) in &self.rmw {
            for pkg in &entry.packages {
                if !known(pkg) {
                    bail!(
                        "rmw '{rmw}' references undefined package '{pkg}' \
                         (not a [tool]/[source]/[gated] entry)"
                    );
                }
            }
        }
        // phase-327 W1 — the new classes' cross-references and probe shapes.
        // Against the MERGED table: a `[tool.*] system = [..]` reference is
        // satisfied by either spelling while both exist, or the W4 rename of
        // `[system.*]` → `[prereq.*]` would break every such reference at once.
        let prereq_keys = self.prereqs();
        for (name, tool) in &self.tool {
            for key in &tool.system {
                if !prereq_keys.contains_key(key) {
                    bail!(
                        "tool '{name}' declares runtime system dep '{key}' \
                         with no [prereq.{key}] entry"
                    );
                }
            }
            // A source recipe fetches from EXACTLY ONE place. Both fields are
            // `Option` so either mode parses, which means neither and both are
            // now expressible states — and both fail late and confusingly
            // (neither: an empty build dir and a configure that "cannot find
            // ./configure"; both: whichever the executor happens to check
            // first, silently). Reject them here, where the message can say
            // which tool.
            if let Some(src) = &tool.source {
                match (&src.git, &src.url) {
                    (Some(_), Some(_)) => bail!(
                        "[tool.{name}.source] sets both `git` and `url` — a \
                         recipe fetches from exactly one place"
                    ),
                    (None, None) => bail!(
                        "[tool.{name}.source] sets neither `git` nor `url` — \
                         nothing to fetch"
                    ),
                    _ => {}
                }
                if src.git.is_some() && src.git_ref.is_none() {
                    bail!("[tool.{name}.source] has `git` but no `ref` to pin it to");
                }
                // Unverified download = supply-chain hole. The git path gets
                // its integrity from the pinned ref; the url path has only this.
                if src.url.is_some() && src.sha256.is_none() {
                    bail!("[tool.{name}.source] has `url` but no `sha256` to verify it");
                }
            }
        }
        for (key, dep) in &self.system {
            if dep.apt.is_empty()
                && dep.dnf.is_empty()
                && dep.pacman.is_empty()
                && dep.brew.is_empty()
            {
                bail!("[system.{key}] maps to no package manager at all");
            }
            // issue 0487 — AT LEAST one, not exactly one. The single-probe rule
            // assumed every dependency has one right existence test, and
            // libgcrypt refuted it: Arch's libgcrypt 1.12 ships `libgcrypt.pc`
            // and NO `libgcrypt-config`, Ubuntu 22.04's 1.9 ships the script and
            // no `.pc`. Either probe alone is a false negative on one of the two
            // hosts, and a false negative here HARD-BLOCKS `nros setup` while
            // telling the user to sudo-install a package they already have.
            // Probes are OR-ed (see `run_probe`), so declaring both answers
            // "is the dev package installed" on both distros.
            if let Some(check) = &dep.check
                && check.field_count() == 0
            {
                bail!(
                    "[system.{key}].check must set at least one of cmd/sharedlib/pkg_config/header"
                );
            }
        }
        for (alias, target) in &self.rust.target {
            if let Some(tc) = &target.toolchain
                && !self.rust.toolchain.contains_key(tc)
            {
                bail!(
                    "[rust.target.{alias}] references undefined toolchain alias '{tc}' \
                     (not a [rust.toolchain.*] entry)"
                );
            }
        }
        for (alias, tool) in &self.rust.cargo_tool {
            if let Some(check) = &tool.check
                && check.field_count() != 1
            {
                bail!("[rust.cargo-tool.{alias}].check must set exactly one probe field");
            }
        }
        for (alias, py) in &self.python {
            if let Some(check) = &py.check
                && check.field_count() != 1
            {
                bail!("[python.{alias}].check must set exactly one probe field");
            }
        }
        // Phase 195.B — a `[source.*]` provisioning recipe must be coherent so
        // `nros setup` can act on it without guessing. `submodule` mode needs a
        // `dest`; clone mode (a `git` with no `submodule`) needs both `ref` and
        // `dest`. `git`/`ref` may accompany `submodule` (they record the pin).
        for (name, src) in &self.source {
            match src.provision() {
                SourceProvision::Clone => {
                    if src.git_ref.is_none() {
                        bail!("source '{name}' has `git` but no `ref` (clone needs a pinned ref)");
                    }
                    if src.dest.is_none() {
                        bail!("source '{name}' has `git` but no `dest` (where to provision it)");
                    }
                }
                SourceProvision::Submodule => {
                    if src.dest.is_none() {
                        bail!("source '{name}' has `submodule` but no `dest`");
                    }
                }
                SourceProvision::None => {}
            }
        }
        Ok(())
    }
}

impl ToolPackage {
    /// The prebuilt artifact for `host` (e.g. `linux-x86_64`), if one exists.
    pub fn dist_for(&self, host: &str) -> Option<&DistArtifact> {
        self.dist.get(host)
    }

    /// Whether this tool can be installed on `host` — a matching prebuilt, or a
    /// source recipe to fall back to. (`false` ⇒ no prebuilt + no source.)
    pub fn installable_on(&self, host: &str) -> bool {
        self.dist.contains_key(host) || self.source.is_some()
    }
}

/// The current host key (`<os>-<arch>`), matching `dist` map keys.
pub fn host_key() -> String {
    let arch = match std::env::consts::ARCH {
        "aarch64" => "arm64",
        other => other, // x86_64, riscv64, …
    };
    format!("{}-{arch}", std::env::consts::OS) // linux / macos / windows
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
[tool.qemu]
version = "11.0-nros1"
dist.linux-x86_64 = { url = "https://github.com/org/nano-ros-sdk/releases/download/qemu-11.0-nros1/qemu-linux-x86_64.tar.zst", sha256 = "aa" }
dist.macos-arm64  = { url = "https://example/qemu-macos-arm64.tar.zst", sha256 = "bb" }
[tool.qemu.source]
git = "https://github.com/org/qemu"
ref = "v11.0-nros1"
configure = "./configure --prefix={prefix} --target-list=arm-softmmu"
install = "make -j && make install"

[tool.arm-none-eabi-gcc]
version = "13.2"
dist.linux-x86_64 = { url = "https://example/arm-gcc-linux-x86_64.tar.zst", sha256 = "cc" }

[source.freertos-kernel]
version = "10.6.2"

[gated.nv-spe-fsp]
version = "36.3"
env = "NV_SPE_FSP_DIR"
installer = "nvidia-sdk-manager"
"#;

    #[test]
    fn parses_tool_source_and_gated_sections() {
        let idx = SdkIndex::parse(SAMPLE).expect("sample parses");
        assert_eq!(idx.tool.len(), 2);
        assert_eq!(idx.source.len(), 1);
        assert_eq!(idx.gated.len(), 1);

        let qemu = &idx.tool["qemu"];
        assert_eq!(qemu.version, "11.0-nros1");
        assert_eq!(qemu.dist_for("linux-x86_64").unwrap().sha256, "aa");
        assert!(qemu.dist_for("windows-x86_64").is_none());
        let src = qemu.source.as_ref().expect("qemu has a source recipe");
        assert_eq!(src.git_ref.as_deref(), Some("v11.0-nros1")); // the `ref` key
        assert!(src.configure.as_deref().unwrap().contains("{prefix}"));

        assert_eq!(idx.source["freertos-kernel"].version, "10.6.2");
        assert_eq!(idx.gated["nv-spe-fsp"].env, "NV_SPE_FSP_DIR");
    }

    /// A source recipe fetches from exactly one place. Both `git` and `url` are
    /// `Option`, so "neither" and "both" became expressible the moment the
    /// tarball mode landed — and both fail LATE and confusingly if they get
    /// past here (neither: an empty build dir and a configure that cannot find
    /// `./configure`; both: whichever the executor checks first, silently).
    #[test]
    fn tool_source_fetches_from_exactly_one_place() {
        let both = SdkIndex::parse(
            "[tool.t]\nversion=\"1\"\n[tool.t.source]\ngit=\"g\"\nref=\"r\"\n\
             url=\"u\"\nsha256=\"s\"\n",
        )
        .expect("parses");
        let err = both
            .validate()
            .expect_err("both git and url must be rejected");
        assert!(format!("{err}").contains("exactly one place"), "{err}");

        let neither =
            SdkIndex::parse("[tool.t]\nversion=\"1\"\n[tool.t.source]\nconfigure=\"x\"\n")
                .expect("parses");
        let err = neither.validate().expect_err("neither must be rejected");
        assert!(format!("{err}").contains("nothing to fetch"), "{err}");

        // An unverified download is a supply-chain hole; the git path gets its
        // integrity from the pinned ref, the url path has only the checksum.
        let no_sum = SdkIndex::parse("[tool.t]\nversion=\"1\"\n[tool.t.source]\nurl=\"u\"\n")
            .expect("parses");
        let err = no_sum
            .validate()
            .expect_err("url without sha256 must be rejected");
        assert!(format!("{err}").contains("sha256"), "{err}");

        let no_ref = SdkIndex::parse("[tool.t]\nversion=\"1\"\n[tool.t.source]\ngit=\"g\"\n")
            .expect("parses");
        let err = no_ref
            .validate()
            .expect_err("git without ref must be rejected");
        assert!(format!("{err}").contains("no `ref`"), "{err}");

        // And the two legal shapes still pass.
        for ok in [
            "[tool.t]\nversion=\"1\"\n[tool.t.source]\ngit=\"g\"\nref=\"r\"\n",
            "[tool.t]\nversion=\"1\"\n[tool.t.source]\nurl=\"u\"\nsha256=\"s\"\n",
        ] {
            SdkIndex::parse(ok)
                .expect("parses")
                .validate()
                .expect("legal shape");
        }
    }

    #[test]
    fn installable_on_uses_dist_or_source_fallback() {
        let idx = SdkIndex::parse(SAMPLE).unwrap();
        // qemu: prebuilt for linux, source fallback covers any host.
        assert!(idx.tool["qemu"].installable_on("linux-x86_64"));
        assert!(idx.tool["qemu"].installable_on("freebsd-riscv64")); // via source
        // arm-gcc: prebuilt only for linux-x86_64, no source → not installable elsewhere.
        assert!(idx.tool["arm-none-eabi-gcc"].installable_on("linux-x86_64"));
        assert!(!idx.tool["arm-none-eabi-gcc"].installable_on("macos-arm64"));
    }

    #[test]
    fn unknown_field_is_rejected() {
        let bad = "[tool.qemu]\nversion = \"1\"\nbogus = true\n";
        assert!(SdkIndex::parse(bad).is_err());
    }

    #[test]
    fn validate_rejects_board_referencing_undefined_package() {
        // qemu defined; board references it (ok) + a typo'd one (rejected).
        let ok = SdkIndex::parse("[tool.qemu]\nversion=\"1\"\n[board.x]\npackages=[\"qemu\"]\n")
            .unwrap();
        assert!(ok.validate().is_ok());

        let bad = SdkIndex::parse("[tool.qemu]\nversion=\"1\"\n[board.x]\npackages=[\"qemoo\"]\n")
            .unwrap();
        let err = bad.validate().unwrap_err().to_string();
        assert!(err.contains("undefined package 'qemoo'"), "{err}");

        // source + gated names are valid package targets too.
        let src_gated = SdkIndex::parse(
            "[source.lwip]\nversion=\"1\"\n[gated.fvp]\nversion=\"1\"\nenv=\"E\"\n\
             [board.b]\npackages=[\"lwip\",\"fvp\"]\n",
        )
        .unwrap();
        assert!(src_gated.validate().is_ok());
    }

    #[test]
    fn source_provision_modes_parse_and_validate() {
        // Clone mode: git + ref + dest.
        let clone = SdkIndex::parse(
            "[source.lwip]\nversion=\"2.2.0\"\ngit=\"https://example/lwip.git\"\n\
             ref=\"STABLE-2_2_0\"\ndest=\"third-party/freertos/lwip\"\n",
        )
        .unwrap();
        let lwip = &clone.source["lwip"];
        assert_eq!(lwip.provision(), SourceProvision::Clone);
        assert_eq!(lwip.git_ref.as_deref(), Some("STABLE-2_2_0")); // the `ref` key
        assert!(clone.validate().is_ok());

        // Submodule mode: submodule + dest.
        let sm = SdkIndex::parse(
            "[source.threadx]\nversion=\"6.4.1\"\nsubmodule=\"third-party/threadx/kernel\"\n\
             dest=\"third-party/threadx/kernel\"\n",
        )
        .unwrap();
        assert_eq!(sm.source["threadx"].provision(), SourceProvision::Submodule);
        assert!(sm.validate().is_ok());

        // No-fetch mode: version only.
        let none = SdkIndex::parse("[source.x]\nversion=\"1\"\n").unwrap();
        assert_eq!(none.source["x"].provision(), SourceProvision::None);
        assert!(none.validate().is_ok());
    }

    #[test]
    fn source_submodule_with_pin_is_valid_and_submodule_mode() {
        // git/ref accompany submodule (record the pin/SSOT) — valid, and the
        // mode is Submodule (submodule update preferred over clone).
        let sm = SdkIndex::parse(
            "[source.x]\nversion=\"1\"\ngit=\"https://e/x.git\"\nref=\"abc123\"\n\
             dest=\"third-party/x\"\nsubmodule=\"third-party/x\"\n",
        )
        .unwrap();
        assert!(sm.validate().is_ok());
        assert_eq!(sm.source["x"].provision(), SourceProvision::Submodule);
    }

    #[test]
    fn source_provision_incoherence_is_rejected() {
        // git without ref.
        let no_ref = SdkIndex::parse("[source.x]\nversion=\"1\"\ngit=\"u\"\ndest=\"d\"\n").unwrap();
        assert!(
            no_ref
                .validate()
                .unwrap_err()
                .to_string()
                .contains("no `ref`")
        );

        // git without dest.
        let no_dest = SdkIndex::parse("[source.x]\nversion=\"1\"\ngit=\"u\"\nref=\"r\"\n").unwrap();
        assert!(
            no_dest
                .validate()
                .unwrap_err()
                .to_string()
                .contains("no `dest`")
        );
    }

    /// phase-327 W1 (RFC-0062) — the new classes round-trip, cross-refs are
    /// validated, and probe shapes must be unambiguous.
    #[test]
    fn system_rust_python_classes_parse_and_validate() {
        let idx = SdkIndex::parse(
            r#"
[system.libslirp]
why = "runtime dep of the qemu dist"
apt = ["libslirp0"]
dnf = ["libslirp"]
check = { sharedlib = "libslirp.so.0" }

[system.gnu-parallel]
apt = ["parallel"]
brew = ["parallel"]
check = { cmd = "parallel" }

[tool.qemu]
version = "11.0-nros2"
system = ["libslirp"]

[rust.toolchain.nightly-pinned]
channel = "nightly-2026-04-11"
components = ["rustfmt", "clippy"]

[rust.target.riscv32imc]
triple = "riscv32imc-unknown-none-elf"
toolchain = "nightly-pinned"

[rust.cargo-tool.nextest]
crate = "cargo-nextest"
check = { cmd = "cargo-nextest" }

[python.west]
pip = "west"
check = { cmd = "west" }
"#,
        )
        .expect("new classes parse");
        assert!(idx.validate().is_ok());
        assert_eq!(idx.system["libslirp"].packages_for("apt"), ["libslirp0"]);
        assert_eq!(idx.system["libslirp"].packages_for("dnf"), ["libslirp"]);
        assert!(idx.system["libslirp"].packages_for("pacman").is_empty());
        assert_eq!(idx.tool["qemu"].system, ["libslirp"]);
        assert_eq!(idx.rust.cargo_tool["nextest"].crate_name, "cargo-nextest");
        assert!(
            idx.rust.cargo_tool["nextest"].locked,
            "locked defaults true"
        );
        assert_eq!(idx.python["west"].pip, "west");

        // A tool naming an undefined system key is a validation error.
        let dangling = SdkIndex::parse("[tool.qemu]\nversion=\"1\"\nsystem=[\"nope\"]\n").unwrap();
        let err = dangling.validate().unwrap_err().to_string();
        // phase-398 W4 — the message names `[prereq.*]`, the table a reader
        // should add the entry to now. The validator checks the MERGED set, so
        // a `[system.*]` entry still satisfies the reference while the alias
        // lives; only the wording moved.
        assert!(err.contains("no [prereq.nope] entry"), "{err}");

        // A system entry mapping no manager at all is rejected.
        let unmapped = SdkIndex::parse("[system.x]\nwhy=\"w\"\n").unwrap();
        assert!(
            unmapped
                .validate()
                .unwrap_err()
                .to_string()
                .contains("no package manager"),
        );

        // issue 0487 — a MULTI-FIELD `[system.*]` probe is ACCEPTED, and this
        // assertion inverted with it. The old rule was "exactly one"; 0487 made
        // it "at least one" because probes are OR-ed (see `run_probe`), which is
        // what let libgcrypt read as missing on Arch when only one of its
        // spellings matched. libgcrypt genuinely needs both spellings (Arch
        // ships the `.pc` and no `libgcrypt-config`; Ubuntu 22.04 ships the
        // script and no `.pc`), so either probe ALONE is a false negative on one
        // of the two distros. Declaring both is now the answer, not an error.
        let multi_probe = SdkIndex::parse(
            "[system.x]\napt=[\"x\"]\ncheck = { cmd = \"x\", sharedlib = \"libx.so\" }\n",
        )
        .unwrap();
        assert!(
            multi_probe.validate().is_ok(),
            "issue 0487: [system.*].check ORs its probes, so two fields is valid",
        );

        // A `[system.*]` probe declaring NO field at all is still rejected —
        // the rule relaxed to at-least-one, not to zero.
        let no_probe = SdkIndex::parse("[system.x]\napt=[\"x\"]\ncheck = { }\n").unwrap();
        assert!(
            no_probe
                .validate()
                .unwrap_err()
                .to_string()
                .contains("at least one"),
        );

        // ...but the OTHER kinds still take exactly one, and 0487 did not touch
        // them. Kept as a rejection case so relaxing `[system.*]` cannot quietly
        // relax these too — the coverage this test would otherwise have lost.
        let ambiguous_tool = SdkIndex::parse(
            "[rust.cargo-tool.t]\ncrate = \"cargo-x\"\ncheck = { cmd = \"x\", sharedlib = \"libx.so\" }\n",
        )
        .unwrap();
        assert!(
            ambiguous_tool
                .validate()
                .unwrap_err()
                .to_string()
                .contains("exactly one"),
        );

        // A target referencing an undefined toolchain alias is rejected.
        let bad_tc = SdkIndex::parse(
            "[rust.target.t]\ntriple=\"thumbv7m-none-eabi\"\ntoolchain=\"ghost\"\n",
        )
        .unwrap();
        assert!(
            bad_tc
                .validate()
                .unwrap_err()
                .to_string()
                .contains("undefined toolchain alias 'ghost'"),
        );
    }

    #[test]
    fn host_key_is_os_dash_arch() {
        let k = host_key();
        assert!(k.contains('-'), "host key looks like <os>-<arch>: {k}");
        assert!(!k.contains("aarch64"), "arch normalized to arm64: {k}");
    }
}

#[cfg(test)]
mod prereq_tests {
    use super::*;

    fn idx(body: &str) -> SdkIndex {
        toml::from_str(body).expect("parses")
    }

    /// W1 is additive: every existing `[system.*]` entry must reach consumers
    /// through `prereqs()` unchanged, with no field dropped. This is the wave's
    /// acceptance criterion, asserted rather than eyeballed.
    #[test]
    fn a_system_entry_survives_the_alias_intact() {
        let i = idx(r#"
            [system.libslirp]
            why = "runtime dep of the qemu dist"
            apt = ["libslirp0"]
            dnf = ["libslirp"]
            pacman = ["libslirp"]
            brew = ["libslirp"]
            check = { sharedlib = "libslirp.so.0" }
        "#);
        let p = i.prereqs();
        let d = p.get("libslirp").expect("aliased into the prereq table");
        assert_eq!(d.provider, Provider::System, "the default and the alias");
        assert_eq!(d.provider_chain(), vec![Provider::System]);
        assert_eq!(d.why.as_deref(), Some("runtime dep of the qemu dist"));
        assert_eq!(d.packages_for("apt"), ["libslirp0"]);
        assert_eq!(d.packages_for("dnf"), ["libslirp"]);
        assert_eq!(d.packages_for("pacman"), ["libslirp"]);
        assert_eq!(d.packages_for("brew"), ["libslirp"]);
        assert_eq!(
            d.check.as_ref().and_then(|c| c.sharedlib.as_deref()),
            Some("libslirp.so.0"),
            "the probe is the half that makes a miss diagnosable"
        );
        assert!(d.packages_for("zypper").is_empty(), "unmapped manager");
    }

    /// A key in both tables is a migration in progress, and the NEW entry is
    /// the intended one. W4 adds the gate that makes the overlap illegal once
    /// it should no longer exist.
    #[test]
    fn a_prereq_entry_wins_over_the_system_alias() {
        let i = idx(r#"
            [system.genromfs]
            why = "old"
            apt = ["genromfs"]

            [prereq.genromfs]
            why = "new"
            providers = ["system", "source"]
            apt = ["genromfs"]
            source = "genromfs"
        "#);
        let p = i.prereqs();
        assert_eq!(p.len(), 1, "one key, not two");
        let d = &p["genromfs"];
        assert_eq!(d.why.as_deref(), Some("new"));
        assert_eq!(
            d.provider_chain(),
            vec![Provider::System, Provider::Source],
            "ordered preference: prefer the OS package, build if absent"
        );
        assert_eq!(d.source.as_deref(), Some("genromfs"));
    }

    /// The common entry writes neither `provider` nor `providers`.
    #[test]
    fn the_default_provider_is_system_and_the_chain_is_never_empty() {
        let i = idx("[prereq.parallel]\napt = [\"parallel\"]\n");
        let d = &i.prereqs()["parallel"];
        assert_eq!(d.provider, Provider::System);
        assert_eq!(d.provider_chain(), vec![Provider::System]);
    }

    /// The non-system providers parse. They install nothing yet — W2 gives
    /// them probes — but the schema must already carry them, or `[prereq.*]`
    /// is just `[system.*]` renamed.
    #[test]
    fn every_provider_parses() {
        let i = idx(r#"
            [prereq.qemu]
            provider = "sdk"
            [prereq.freertos-kernel]
            provider = "submodule"
            [prereq.sccache]
            provider = "source"
        "#);
        let p = i.prereqs();
        assert_eq!(p["qemu"].provider, Provider::Sdk);
        assert_eq!(p["freertos-kernel"].provider, Provider::Submodule);
        assert_eq!(p["sccache"].provider, Provider::Source);
    }

    /// `deny_unknown_fields` is load-bearing: a mistyped key must be an error,
    /// not a silently ignored declaration — the same reasoning `[image.*]`
    /// gives, and the failure mode this whole RFC exists to delete.
    #[test]
    fn a_mistyped_field_is_an_error() {
        let e = toml::from_str::<SdkIndex>("[prereq.x]\naptt = [\"y\"]\n");
        assert!(e.is_err(), "a typo must not parse as an empty entry");
    }

    // ---- phase-404 W1: version constraints ----

    /// Real `--version` output from the tools this index pins, captured on a
    /// 22.04 host. A synthetic string would prove the parser handles the string
    /// I imagined rather than the ones tools print.
    #[test]
    fn the_version_scan_reads_what_these_tools_actually_print() {
        let cases = [
            (
                "Open On-Chip Debugger 0.12.0-g9ea7f3d (2026-08-30-10:21)",
                "0.12.0",
            ),
            ("QEMU emulator version 11.0.0", "11.0.0"),
            (
                "GNU gdb (Arm GNU Toolchain 13.2.rel1 (Build arm-13.7)) 13.2.90.20231008-git",
                "13.2",
            ),
            (
                "arm-none-eabi-gcc (Arm GNU Toolchain 13.2.rel1 (Build arm-13.7)) 13.2.1 20231009",
                "13.2",
            ),
            ("sccache 0.17.0", "0.17.0"),
        ];
        for (out, want) in cases {
            assert_eq!(
                extract_version(out, None).as_deref(),
                Some(want),
                "parsing {out:?}"
            );
        }
    }

    #[test]
    fn a_version_needs_a_dot_so_a_bare_year_is_not_one() {
        // `genromfs 0.5.2` style is fine; a lone build number must not win.
        assert_eq!(
            extract_version("build 20231009 v1.2", None).as_deref(),
            Some("1.2")
        );
        assert_eq!(extract_version("no version here", None), None);
        // Trailing punctuation is not part of the token.
        assert_eq!(
            extract_version("version 1.2.", None).as_deref(),
            Some("1.2")
        );
    }

    #[test]
    fn the_after_marker_picks_a_later_token() {
        let out = "Foo 1.0 built against Bar 2.5";
        assert_eq!(extract_version(out, None).as_deref(), Some("1.0"));
        assert_eq!(extract_version(out, Some("Bar")).as_deref(), Some("2.5"));
    }

    #[test]
    fn min_is_a_floor_and_missing_components_are_zero() {
        let c = VersionConstraint {
            min: Some("0.12".into()),
            ..Default::default()
        };
        assert!(version_satisfies("0.12.0", &c), "0.12.0 meets a 0.12 floor");
        assert!(version_satisfies("0.13.1", &c));
        assert!(
            !version_satisfies("0.11.0", &c),
            "22.04's openocd must NOT satisfy"
        );
        // Numeric, not lexicographic: the bug a string compare would have.
        let c10 = VersionConstraint {
            min: Some("1.9".into()),
            ..Default::default()
        };
        assert!(version_satisfies("1.10", &c10), "1.10 > 1.9 numerically");
    }

    /// `exact` compares only the components it names, so a pin that is a
    /// CONTRACT (cyclonedds must be what ROS ships — issue 0507) accepts patch
    /// releases of that line and rejects the next minor, which `min` would have
    /// silently allowed.
    #[test]
    fn exact_pins_a_line_and_rejects_the_next_one() {
        let c = VersionConstraint {
            exact: Some("0.10".into()),
            ..Default::default()
        };
        assert!(version_satisfies("0.10.5", &c));
        assert!(version_satisfies("0.10.0", &c));
        assert!(
            !version_satisfies("0.11.0", &c),
            "the next minor breaks the contract"
        );
        assert!(!version_satisfies("0.9.9", &c));
    }

    #[test]
    fn no_constraint_is_satisfied_by_anything() {
        assert!(version_satisfies("0.0.1", &VersionConstraint::default()));
    }
}
