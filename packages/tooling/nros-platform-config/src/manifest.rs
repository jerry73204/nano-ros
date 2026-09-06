//! Manifest parser for `<kernel>_platforms.toml` files.
//!
//! Phase 136.1 / 136.4 landed the loader + per-platform resolver
//! inside `zpico-sys/build/`. Phase 152.5 lifted it into the
//! `nros-board-common` library so the per-kernel generic board
//! crates can share one canonical implementation.
//!
//! Schema carries every per-platform datum the cc-rs collapse
//! needs: SDK env vars (with help text + validation), conditional
//! include paths (interpolated `{env:VAR}` / `{nros}` / `{out}` /
//! `{src}` tokens; `when.target_match` / `when.target_not` /
//! `when.if_env` / `when.capability` gates), extra source files
//! (with `if_env`, `when` and `with_define` modifiers),
//! debug-env-driven defines, conditional defines
//! (`defines_conditional`), and an `[arch.*]` table for reusable
//! target-arch compiler-flag profiles.
//!
//! issue 1143 — `when.capability` is the RFC-0086 fact vocabulary,
//! resolved through the RFC-0049 ladder (platform, then the
//! board's `[board.capabilities]`, board winning). It is the one
//! gate whose answer differs between two BOARDS of one platform,
//! which is what the FreeRTOS family needed and what the
//! target/env gates cannot express.
//!
//! Use from `build.rs`:
//! ```ignore
//! use nros_board_common::manifest::PlatformManifest;
//! let m = PlatformManifest::load("zenoh_platforms.toml".as_ref())?;
//! let resolved = m.for_platform("posix")?;
//! ```

use std::{collections::BTreeMap, fs, path::Path};

use serde::{Deserialize, Deserializer};

/// Accept either a single TOML string (`arch = "cortex-m3"`) or an
/// array (`arch = ["cortex-m3", "riscv32imc"]`). Returns the
/// normalised `Vec<String>`. Phase 148.
fn deserialize_arch_field<'de, D: Deserializer<'de>>(d: D) -> Result<Vec<String>, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum StringOrVec {
        Scalar(String),
        Vec(Vec<String>),
    }
    Ok(match StringOrVec::deserialize(d)? {
        StringOrVec::Scalar(s) => vec![s],
        StringOrVec::Vec(v) => v,
    })
}

/// Top-level manifest: `[platform.<name>]` + `[arch.<name>]`.
#[derive(Debug, Deserialize)]
pub struct PlatformManifest {
    pub platform: BTreeMap<String, PlatformEntry>,
    #[serde(default)]
    pub arch: BTreeMap<String, ArchEntry>,
}

/// One `[platform.<name>]` block.
#[derive(Debug, Default, Deserialize, Clone)]
pub struct PlatformEntry {
    /// Optional parent platform name. Parent fields are merged
    /// before this entry's fields override them.
    #[serde(default)]
    pub inherits: Option<String>,
    /// Preprocessor defines added unconditionally
    /// (`cc::Build::define(name, None)`).
    #[serde(default)]
    pub defines: Vec<String>,
    /// Defines gated by a `when` matcher — the `X` / `X_conditional` pair this
    /// schema already uses for `include_paths` (issue 1143).
    ///
    /// The case that needed it: zenoh-pico picks its `_z_sys_net_socket_t` from
    /// ONE `ZENOH_<platform>` define, and on FreeRTOS the choice is a board
    /// FACT, not a platform one — `freertos/lwip.h` `#include`s
    /// `lwip/sockets.h`, so a board with no lwIP cannot even parse the type
    /// header, let alone link the 63 `lwip_*` symbols it pulls. Nothing in the
    /// schema could branch a define, so the lwIP one was unconditional and
    /// every FreeRTOS board got it.
    #[serde(default)]
    pub defines_conditional: Vec<ConditionalDefine>,
    /// Key=value defines (`cc::Build::define(name, Some(value))`).
    #[serde(default)]
    pub defines_kv: BTreeMap<String, String>,
    /// Defines whose value comes from an env var with a literal
    /// default.
    #[serde(default)]
    pub defines_env: BTreeMap<String, EnvDefault>,
    /// Glob roots under `zenoh-pico/src/` for core protocol /
    /// system-common source selection. Drift gate (136.6)
    /// validates these.
    #[serde(default)]
    pub include: Vec<String>,
    /// Glob roots under `zenoh-pico/src/` to exclude from
    /// `include` matches.
    #[serde(default)]
    pub exclude: Vec<String>,
    /// `cargo:rustc-link-lib=` system libraries (POSIX hosts).
    #[serde(default)]
    pub system_libs: Vec<String>,
    /// Where mbedTLS comes from when `link-tls` is active:
    /// `pkg-config`, `vendored`, or `none`.
    #[serde(default)]
    pub mbedtls: Option<String>,
    /// Per-link-feature overrides declared in the manifest.
    /// `LinkOverride::On(false)` forces off; `Mode("feature")`
    /// defers to `CARGO_FEATURE_LINK_<X>`.
    #[serde(default)]
    pub link: BTreeMap<String, LinkOverride>,
    /// Extra source files (paths interpolated; see `{nros}` /
    /// `{src}` tokens), optionally conditional on env presence
    /// and pulling in additional defines when included.
    #[serde(default)]
    pub extra_sources: Vec<ExtraSource>,
    /// Required env vars + help text + optional sub-dir
    /// validation. Build script panics loudly when absent.
    #[serde(default)]
    pub required_env: Vec<RequiredEnv>,
    /// Unconditional include paths (interpolated). Order matters
    /// — first wins for `#include` resolution.
    #[serde(default)]
    pub include_paths: Vec<String>,
    /// Include paths gated by a `when` matcher (target /
    /// env-presence).
    #[serde(default)]
    pub include_paths_conditional: Vec<ConditionalPath>,
    /// Optional `[arch.*]` profile(s) to apply (cflags + sysroot /
    /// errno-override hooks). Accepts a single arch name (scalar
    /// TOML string) or a list (TOML array); single-arch platforms
    /// stay readable while multi-arch platforms (bare-metal across
    /// cortex-m3 + riscv32imc) declare every arch they support and
    /// let `build.rs::build_zenoh_pico_unified` pick the first one
    /// whose `target_match` matches the build target. See Phase 148.
    #[serde(default, deserialize_with = "deserialize_arch_field")]
    pub arch: Vec<String>,
    /// Cross-compile compile-rs options (opt_level, warnings,
    /// extra cflags).
    #[serde(default)]
    pub compile: CompileSettings,
    /// `cc::Build::pic(bool)` override (NuttX flat builds use
    /// `false`; POSIX leaves the cc-rs default).
    #[serde(default)]
    pub pic: Option<bool>,
    /// Rerun-if-env-changed env vars to register beyond
    /// `required_env`. Set for env-gated debug knobs etc.
    #[serde(default)]
    pub rerun_if_env_changed: Vec<String>,
    /// Who compiles this component's vendored C for this platform.
    /// See [`CompiledBy`] — issue 0534. `None` inherits (and resolves to
    /// [`CompiledBy::Cargo`]); an unset child must NOT downgrade a parent that
    /// set it, which is why this is an `Option` like `pic` and `mbedtls`.
    #[serde(default)]
    pub compiled_by: Option<CompiledBy>,
}

/// Who compiles a `[build.<component>]` block's vendored C sources.
///
/// Issue **0534**. A platform whose own build system compiles the component
/// still needs a `[build.*]` block — the drift gate wants a manifest entry, and
/// the defines/includes document what that external build is expected to use.
/// Zephyr's block said exactly that in a COMMENT ("no cc-rs consumer hits it"),
/// and #529 then made a cc-rs consumer hit it: naming the platform in the
/// resolver is what selects `build_zenoh_pico_unified`, so the vendored
/// `system/zephyr/*.c` got compiled by a build script, where Zephyr's generated
/// `version.h` does not exist. The build died on a missing header nobody had
/// asked for.
///
/// The comment was true and unenforced. This field is the same claim, checked.
#[derive(Debug, Default, Deserialize, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum CompiledBy {
    /// This build script compiles the sources with cc-rs. The default, and
    /// what every platform that has no external build system does.
    #[default]
    Cargo,
    /// The platform's own build system compiles them; cargo must not, and the
    /// block is descriptive only.
    Platform,
}

/// `[arch.<name>]` block — reusable target-arch compiler-flag
/// profile shared across platforms.
#[derive(Debug, Default, Deserialize, Clone)]
pub struct ArchEntry {
    /// Substring that must be in the target triple for the arch
    /// block to apply.
    #[serde(default)]
    pub target_match: Option<String>,
    /// Substring that, if present in the target triple, vetoes
    /// this arch block. Used to disambiguate Cortex-M3 (thumbv7m)
    /// from Cortex-M4 (thumbv7em).
    #[serde(default)]
    pub target_exclude: Option<String>,
    /// Compiler flags appended to `cc::Build`.
    #[serde(default)]
    pub cflags: Vec<String>,
    /// Whether the build should add the picolibc sysroot's
    /// `include/` to the search path (RISC-V bare-metal).
    #[serde(default)]
    pub needs_picolibc: bool,
    /// Whether the build should generate + prepend the
    /// errno-override shadow header (RISC-V picolibc TLS-errno
    /// workaround).
    #[serde(default)]
    pub needs_errno_override: bool,
    /// Whether the build needs `detect_riscv_compiler` cross-cc
    /// probe (cargo doesn't auto-set CC for bare-metal RISC-V).
    #[serde(default)]
    pub needs_riscv_compiler: bool,
}

/// Per-link-feature override declared in `zenoh_platforms.toml`.
/// `bool` collapses to On / Off; a string like `"feature"` defers
/// to the matching `CARGO_FEATURE_*` env var. Distinct from the
/// build-script `LinkPolicy` struct in `build/policy.rs`.
#[derive(Debug, Deserialize, Clone, PartialEq, Eq)]
#[serde(untagged)]
pub enum LinkOverride {
    On(bool),
    Mode(String),
}

/// Env-var-backed define: value comes from `env`, falls back to
/// `default` literal.
#[derive(Debug, Deserialize, Clone)]
pub struct EnvDefault {
    pub env: String,
    pub default: String,
}

/// Extra C source compiled into the zenoh-pico archive.
#[derive(Debug, Deserialize, Clone)]
pub struct ExtraSource {
    /// Interpolated path (`{nros}` / `{src}` / `{out}` /
    /// `{env:VAR}`).
    pub path: String,
    /// If set, only include when the named env var is present.
    #[serde(default)]
    pub if_env: Option<String>,
    /// If set, `cc::Build::define(name, Some(value))` whenever
    /// this source is included.
    #[serde(default)]
    pub with_define: Option<Vec<String>>,
    /// issue 1143 — the same `when` gate `include_paths_conditional` and
    /// `defines_conditional` carry. Composes with `if_env` (both must pass);
    /// an empty matcher is always true, which is every existing row.
    #[serde(default)]
    pub when: WhenMatcher,
}

/// One conditional preprocessor define (issue 1143). Mirrors
/// [`ConditionalPath`], one field over: a NAME instead of a path.
#[derive(Debug, Deserialize, Clone)]
pub struct ConditionalDefine {
    /// Define name, passed to `cc::Build::define(name, None)`.
    pub name: String,
    /// Matcher table; see [`WhenMatcher`].
    pub when: WhenMatcher,
}

/// One required env var.
#[derive(Debug, Deserialize, Clone)]
pub struct RequiredEnv {
    pub name: String,
    pub help: String,
    /// Optional sub-directory that must exist under the env's
    /// value for the build to proceed (loud panic otherwise).
    #[serde(default)]
    pub validate_subdir: Option<String>,
    /// issue 1143 — the same `when` gate the conditional defines, sources and
    /// include paths carry. An SDK is required only where the build uses it:
    /// `LWIP_DIR` is a hard requirement of a FreeRTOS board that HAS lwIP and
    /// meaningless to one that cannot reach a netstack, and without this the
    /// netstack-less board still had to invent a directory to point at before
    /// its build would start.
    #[serde(default)]
    pub when: WhenMatcher,
}

/// One conditional include path.
#[derive(Debug, Deserialize, Clone)]
pub struct ConditionalPath {
    /// Interpolated path.
    pub path: String,
    /// Matcher table; see `WhenMatcher`.
    pub when: WhenMatcher,
}

/// Gate that decides whether a conditional item applies.
/// Forms (`target_match` / `target_not` / `if_env`) compose: each
/// non-`None` field must match for the matcher to return `true`.
#[derive(Debug, Default, Deserialize, Clone)]
pub struct WhenMatcher {
    /// Substring that must appear in the target triple.
    #[serde(default)]
    pub target_match: Option<String>,
    /// Substring that must NOT appear in the target triple.
    /// Special value `"embedded"` means "target_os is one of the
    /// known embedded RTOSes". Build-script consumer expands.
    #[serde(default)]
    pub target_not: Option<String>,
    /// Env var that must be set (any value).
    #[serde(default)]
    pub if_env: Option<String>,
    /// issue 1143 — RFC-0086 `[capabilities]` facts that must be DECLARED with
    /// the given value, resolved through the RFC-0049 ladder (platform, then
    /// the board's own `[capabilities]` rung, board winning).
    ///
    /// **Absent is not false**, the rule `resolve_transport` already applies to
    /// the same table: an UNDECLARED capability matches neither `true` nor
    /// `false`, so a matcher naming one is inert on a platform nobody has
    /// described. That is what keeps `capability = { ip_stack = true }` from
    /// silently dropping a source on a platform that simply says nothing —
    /// the ladder's own answer has to be an answer.
    #[serde(default)]
    pub capability: BTreeMap<String, bool>,
}

/// Optimization level for the cc-rs build. Accepts either a numeric
/// level (`opt_level = 2`) or a string level (`opt_level = "s"` / `"z"`
/// for size), matching `cc::Build::opt_level` / `opt_level_str`.
/// Phase 204.9 — size builds need `-Os`, which the integer form can't
/// express.
#[derive(Debug, Deserialize, Clone)]
#[serde(untagged)]
pub enum OptLevel {
    Num(u32),
    Str(String),
}

/// `cc::Build` compile settings.
#[derive(Debug, Default, Deserialize, Clone)]
pub struct CompileSettings {
    pub opt_level: Option<OptLevel>,
    #[serde(default)]
    pub warnings: Option<bool>,
    #[serde(default)]
    pub cflags: Vec<String>,
}

/// Resolved view of one platform after `inherits` chain merge.
#[derive(Debug, Clone)]
pub struct ResolvedPlatform {
    pub name: String,
    pub defines: Vec<String>,
    /// issue 1143 — `when`-gated defines; see [`ConditionalDefine`].
    pub defines_conditional: Vec<ConditionalDefine>,
    pub defines_kv: BTreeMap<String, String>,
    pub defines_env: BTreeMap<String, EnvDefault>,
    pub include: Vec<String>,
    pub exclude: Vec<String>,
    pub system_libs: Vec<String>,
    pub mbedtls: Option<String>,
    pub link: BTreeMap<String, LinkOverride>,
    pub extra_sources: Vec<ExtraSource>,
    pub required_env: Vec<RequiredEnv>,
    pub include_paths: Vec<String>,
    pub include_paths_conditional: Vec<ConditionalPath>,
    pub arch: Vec<String>,
    pub compile: CompileSettings,
    pub pic: Option<bool>,
    pub rerun_if_env_changed: Vec<String>,
    pub compiled_by: CompiledBy,
}

impl PlatformManifest {
    /// Parse the manifest from a TOML file on disk.
    pub fn load(path: &Path) -> Result<Self, ManifestError> {
        let text = fs::read_to_string(path).map_err(|e| ManifestError::Io {
            path: path.display().to_string(),
            source: e,
        })?;
        Self::parse(&text)
    }

    /// Parse the manifest from an in-memory TOML string.
    pub fn parse(text: &str) -> Result<Self, ManifestError> {
        toml::from_str(text).map_err(ManifestError::Parse)
    }

    /// Resolve one `[platform.<name>]` block, walking the
    /// `inherits` chain. Child fields win when both parent and
    /// child set the same key; list-shaped fields are unioned
    /// (parent first, then child); maps merge per-key with child
    /// override.
    pub fn for_platform(&self, name: &str) -> Result<ResolvedPlatform, ManifestError> {
        let mut seen = std::collections::BTreeSet::new();
        let entry = self.resolve(name, &mut seen)?;
        Ok(ResolvedPlatform {
            name: name.to_string(),
            defines: entry.defines,
            defines_conditional: entry.defines_conditional,
            defines_kv: entry.defines_kv,
            defines_env: entry.defines_env,
            include: entry.include,
            exclude: entry.exclude,
            system_libs: entry.system_libs,
            mbedtls: entry.mbedtls,
            link: entry.link,
            extra_sources: entry.extra_sources,
            required_env: entry.required_env,
            include_paths: entry.include_paths,
            include_paths_conditional: entry.include_paths_conditional,
            arch: entry.arch,
            compile: entry.compile,
            pic: entry.pic,
            rerun_if_env_changed: entry.rerun_if_env_changed,
            compiled_by: entry.compiled_by.unwrap_or_default(),
        })
    }

    /// Look up an `[arch.*]` block by name.
    pub fn arch_for(&self, name: &str) -> Option<&ArchEntry> {
        self.arch.get(name)
    }

    fn resolve(
        &self,
        name: &str,
        seen: &mut std::collections::BTreeSet<String>,
    ) -> Result<PlatformEntry, ManifestError> {
        if !seen.insert(name.to_string()) {
            return Err(ManifestError::InheritCycle(name.to_string()));
        }
        let entry = self
            .platform
            .get(name)
            .ok_or_else(|| ManifestError::UnknownPlatform(name.to_string()))?
            .clone();
        let parent = match entry.inherits.as_deref() {
            Some(parent_name) => Some(self.resolve(parent_name, seen)?),
            None => None,
        };
        Ok(merge(parent, entry))
    }
}

fn merge(parent: Option<PlatformEntry>, mut child: PlatformEntry) -> PlatformEntry {
    let Some(parent) = parent else {
        child.inherits = None;
        return child;
    };

    let mut defines = parent.defines;
    defines.append(&mut child.defines);
    let mut defines_conditional = parent.defines_conditional;
    defines_conditional.append(&mut child.defines_conditional);
    let mut defines_kv = parent.defines_kv;
    defines_kv.extend(std::mem::take(&mut child.defines_kv));
    let mut defines_env = parent.defines_env;
    defines_env.extend(std::mem::take(&mut child.defines_env));
    let mut include = parent.include;
    include.append(&mut child.include);
    let mut exclude = parent.exclude;
    exclude.append(&mut child.exclude);
    let mut system_libs = parent.system_libs;
    system_libs.append(&mut child.system_libs);
    let mbedtls = child.mbedtls.or(parent.mbedtls);
    let mut link = parent.link;
    link.extend(std::mem::take(&mut child.link));
    let mut extra_sources = parent.extra_sources;
    extra_sources.append(&mut child.extra_sources);
    let mut required_env = parent.required_env;
    required_env.append(&mut child.required_env);
    let mut include_paths = parent.include_paths;
    include_paths.append(&mut child.include_paths);
    let mut include_paths_conditional = parent.include_paths_conditional;
    include_paths_conditional.append(&mut child.include_paths_conditional);
    // Child's arch list overrides parent's when non-empty; otherwise
    // inherit. Mirrors the Option<String>.or semantics now extended
    // to multi-arch platforms (Phase 148).
    let arch = if child.arch.is_empty() {
        parent.arch
    } else {
        child.arch
    };
    let compile = CompileSettings {
        opt_level: child.compile.opt_level.or(parent.compile.opt_level),
        warnings: child.compile.warnings.or(parent.compile.warnings),
        cflags: {
            let mut c = parent.compile.cflags;
            c.extend(child.compile.cflags);
            c
        },
    };
    let pic = child.pic.or(parent.pic);
    let compiled_by = child.compiled_by.or(parent.compiled_by);
    let mut rerun_if_env_changed = parent.rerun_if_env_changed;
    rerun_if_env_changed.append(&mut child.rerun_if_env_changed);

    PlatformEntry {
        inherits: None,
        defines,
        defines_conditional,
        defines_kv,
        defines_env,
        include,
        exclude,
        system_libs,
        mbedtls,
        link,
        extra_sources,
        required_env,
        include_paths,
        include_paths_conditional,
        arch,
        compile,
        pic,
        rerun_if_env_changed,
        compiled_by,
    }
}

#[derive(Debug)]
pub enum ManifestError {
    Io {
        path: String,
        source: std::io::Error,
    },
    Parse(toml::de::Error),
    UnknownPlatform(String),
    InheritCycle(String),
}

impl std::fmt::Display for ManifestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io { path, source } => write!(f, "read {path}: {source}"),
            Self::Parse(e) => write!(f, "parse zenoh_platforms.toml: {e}"),
            Self::UnknownPlatform(name) => write!(f, "unknown platform: {name}"),
            Self::InheritCycle(name) => write!(f, "inherits cycle through {name}"),
        }
    }
}

impl std::error::Error for ManifestError {}

// ----------------------------------------------------------------
// Interpolation + matcher (consumed by build.rs's unified driver).
// ----------------------------------------------------------------

/// Tokens available for interpolation in any `path` / `defines_env`
/// field. Build-script populates this context once + threads it
/// through.
pub struct InterpContext<'a> {
    /// `CARGO_MANIFEST_DIR` (`zpico-sys/`).
    pub nros: &'a Path,
    /// `OUT_DIR`.
    pub out: &'a Path,
    /// `zenoh-pico/src` (relative to `nros`).
    pub src: &'a Path,
}

/// Replace every `{nros}` / `{out}` / `{src}` / `{nuttx_include}` / `{env:VAR}`
/// token in `input`. Missing env vars produce `None` so the caller can
/// emit a helpful panic.
///
/// `{nuttx_include}` exists because `{env:NUTTX_DIR}/include` is WRONG and
/// looked right (issue 0551). NuttX is built in place, one checkout serves both
/// arches, and `build-nuttx.sh`'s snapshot short-circuit guarantees the export
/// snapshot rather than the tree — so the tree's `include/` holds whichever arch
/// was configured LAST, and after any `make olddefconfig` holds no generated
/// `nuttx/config.h` at all (that target runs `clean_context`, which deletes it).
/// The whole NuttX fixture lane died on `fatal error: nuttx/config.h: No such
/// file or directory` from exactly that spelling.
///
/// Issue 0525 already made `nros_build_paths::nuttx_include_root` the one
/// sanctioned resolution, and its gate greps Rust and shell. A TOML
/// `{env:NUTTX_DIR}/include` is neither, which is why this site outlived six
/// earlier sweeps. Give the manifest a token that cannot spell it wrongly
/// rather than a second place to remember the rule.
pub fn interpolate(input: &str, ctx: &InterpContext<'_>) -> Result<String, InterpError> {
    let mut out = String::with_capacity(input.len());
    let mut rest = input;
    loop {
        let Some(start) = rest.find('{') else {
            out.push_str(rest);
            return Ok(out);
        };
        out.push_str(&rest[..start]);
        rest = &rest[start + 1..];
        let Some(end) = rest.find('}') else {
            return Err(InterpError::UnterminatedToken(input.to_string()));
        };
        let token = &rest[..end];
        rest = &rest[end + 1..];
        let value: String = if token == "nros" {
            ctx.nros.display().to_string()
        } else if token == "out" {
            ctx.out.display().to_string()
        } else if token == "src" {
            ctx.src.display().to_string()
        } else if token == "nuttx_include" {
            let dir = std::env::var("NUTTX_DIR")
                .map_err(|_| InterpError::MissingEnv("NUTTX_DIR".to_string()))?;
            // `nros_build_paths` directly, not the board crate's re-export:
            // `nuttx_export::include_root` is a one-line delegator to exactly
            // this, and its own comment says the shared SPELLING is the point.
            // Calling the shared one is what lets this module live in a leaf
            // crate (phase-400 W6).
            nros_build_paths::nuttx_include_root(Path::new(&dir))
                .display()
                .to_string()
        } else if let Some(var) = token.strip_prefix("env:") {
            std::env::var(var).map_err(|_| InterpError::MissingEnv(var.to_string()))?
        } else {
            return Err(InterpError::UnknownToken(token.to_string()));
        };
        out.push_str(&value);
    }
}

#[derive(Debug)]
pub enum InterpError {
    UnknownToken(String),
    UnterminatedToken(String),
    MissingEnv(String),
}

impl std::fmt::Display for InterpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownToken(t) => write!(f, "unknown interpolation token `{{{t}}}`"),
            Self::UnterminatedToken(s) => write!(f, "unterminated `{{` in `{s}`"),
            Self::MissingEnv(v) => write!(f, "env var `{v}` not set"),
        }
    }
}

impl std::error::Error for InterpError {}

/// Returns `true` when every populated field in `m` matches the
/// current target / env / capability state. Empty matcher = always true.
/// `target_not == "embedded"` is the special-case "target_os is
/// one of the known RTOSes" gate; build-script supplies the
/// `is_embedded` flag pre-computed.
///
/// `caps` is the RFC-0086 capability map the RFC-0049 ladder resolved for this
/// build (platform rung merged with the board's, board winning) — see
/// `PlatformsTree::capabilities_with_board`. Pass an empty map when no ladder
/// was resolved; a `capability` matcher is then inert, which is the
/// absent-is-not-false rule and the reason an undescribed platform keeps
/// building exactly as before.
pub fn matches(
    m: &WhenMatcher,
    target: &str,
    is_embedded: bool,
    caps: &BTreeMap<String, bool>,
) -> bool {
    for (name, want) in &m.capability {
        if caps.get(name) != Some(want) {
            return false;
        }
    }
    if let Some(needle) = m.target_match.as_deref()
        && !match_target(target, needle)
    {
        return false;
    }
    if let Some(needle) = m.target_not.as_deref() {
        let hit = if needle == "embedded" {
            is_embedded
        } else {
            match_target(target, needle)
        };
        if hit {
            return false;
        }
    }
    if let Some(var) = m.if_env.as_deref()
        && std::env::var(var).is_err()
    {
        return false;
    }
    true
}

/// Match a target-triple needle. Supports trailing `*` glob:
/// `riscv64-*` matches anything starting with `riscv64-`.
fn match_target(target: &str, needle: &str) -> bool {
    if let Some(prefix) = needle.strip_suffix('*') {
        target.starts_with(prefix)
    } else {
        target.contains(needle)
    }
}

// Note: the loader is exercised at build time — `build.rs` parses
// `zenoh_platforms.toml` + resolves every platform on every cargo
// build. A typo, broken `inherits` chain, or shape regression
// surfaces as a build-script panic. There is no separate
// `#[test]`-style suite because `cargo test` doesn't link build
// scripts; the build-time invariant is the real gate.

// …that argument covers the fields a build-script panic would surface. It does
// NOT cover `when.capability` (issue 1143): a matcher that silently ignores the
// capability table does not panic, it compiles the OTHER arm, and the build
// succeeds holding a socket type nobody chose. So this one is pinned here.
#[cfg(test)]
mod capability_matcher_tests {
    use super::*;

    fn caps(pairs: &[(&str, bool)]) -> BTreeMap<String, bool> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    fn matcher(toml_body: &str) -> WhenMatcher {
        toml::from_str(toml_body).expect("matcher")
    }

    #[test]
    fn a_capability_matcher_compares_the_value_not_the_presence() {
        let want_true = matcher("capability = { ip_stack = true }");
        let want_false = matcher("capability = { ip_stack = false }");
        let on = caps(&[("ip_stack", true)]);
        let off = caps(&[("ip_stack", false)]);

        assert!(matches(&want_true, "thumbv7m-none-eabi", true, &on));
        assert!(!matches(&want_true, "thumbv7m-none-eabi", true, &off));
        assert!(matches(&want_false, "thumbv7m-none-eabi", true, &off));
        assert!(!matches(&want_false, "thumbv7m-none-eabi", true, &on));
    }

    /// Absent is not false — the rule `resolve_transport` already applies to
    /// the same table. An undeclared capability matches NEITHER arm, so a
    /// platform nobody has described keeps building exactly as it did.
    #[test]
    fn an_undeclared_capability_matches_neither_value() {
        let empty = BTreeMap::new();
        assert!(!matches(
            &matcher("capability = { ip_stack = true }"),
            "x86_64-unknown-linux-gnu",
            false,
            &empty
        ));
        assert!(!matches(
            &matcher("capability = { ip_stack = false }"),
            "x86_64-unknown-linux-gnu",
            false,
            &empty
        ));
    }

    /// The gates compose: a row is included only when EVERY populated field
    /// agrees. A capability match must not rescue a target mismatch.
    #[test]
    fn capability_composes_with_the_target_gates() {
        let m = matcher("target_match = \"thumbv7m\"\ncapability = { ip_stack = true }");
        let on = caps(&[("ip_stack", true)]);
        assert!(matches(&m, "thumbv7m-none-eabi", true, &on));
        assert!(!matches(&m, "armv7r-none-eabi", true, &on));
        assert!(!matches(
            &m,
            "thumbv7m-none-eabi",
            true,
            &caps(&[("ip_stack", false)])
        ));
    }

    /// Every row that predates the field is an empty matcher, and an empty
    /// matcher is unconditionally true — the whole compatibility claim.
    #[test]
    fn an_empty_matcher_is_still_always_true() {
        assert!(matches(
            &WhenMatcher::default(),
            "x86_64-unknown-linux-gnu",
            false,
            &BTreeMap::new()
        ));
    }

    /// `defines_conditional` and `extra_sources.when` parse, and an
    /// `extra_sources` row written before the field still parses without it.
    #[test]
    fn the_new_schema_rows_parse() {
        let m: PlatformManifest = toml::from_str(
            "[platform.p]\n\
             defines = [\"ZENOH_GENERIC\"]\n\
             defines_conditional = [\n\
               { name = \"ZENOH_ORIN_SPE\", when = { capability = { ip_stack = false } } },\n\
             ]\n\
             extra_sources = [\n\
               { path = \"{src}/a.c\" },\n\
               { path = \"{src}/b.c\", when = { capability = { ip_stack = true } } },\n\
             ]\n",
        )
        .expect("parse");
        let r = m.for_platform("p").expect("resolve");
        assert_eq!(r.defines_conditional.len(), 1);
        assert_eq!(r.defines_conditional[0].name, "ZENOH_ORIN_SPE");
        let no_caps = BTreeMap::new();
        // The legacy row has no `when`, so it survives with no capabilities;
        // the gated one does not.
        assert!(matches(&r.extra_sources[0].when, "t", true, &no_caps));
        assert!(!matches(&r.extra_sources[1].when, "t", true, &no_caps));
    }

    /// The FreeRTOS shape issue 1143 exists for, end to end through the
    /// resolver: one manifest, two capability states, and the three things
    /// that have to move together — the platform DEFINE (and so the socket
    /// type), the netstack TU, and the SDK the netstack needs.
    ///
    /// Pinned here because they are three separate fields and nothing else
    /// makes them agree; a fix that dropped `network.c` and left
    /// `ZENOH_FREERTOS_LWIP` defined would compile `lwip/sockets.h` on a board
    /// with no lwIP, and one that dropped the define and left `LWIP_DIR`
    /// required would refuse to start.
    #[test]
    fn the_freertos_arms_move_together() {
        let m: PlatformManifest = toml::from_str(
            "[platform.freertos]\n\
             defines = [\"ZENOH_GENERIC\"]\n\
             defines_conditional = [\n\
               { name = \"ZENOH_FREERTOS_LWIP\", when = { capability = { ip_stack = true } } },\n\
               { name = \"ZENOH_ORIN_SPE\", when = { capability = { ip_stack = false } } },\n\
             ]\n\
             extra_sources = [\n\
               { path = \"{src}/system/freertos/system.c\" },\n\
               { path = \"{src}/system/freertos/lwip/network.c\", when = { capability = { ip_stack = true } } },\n\
             ]\n\
             required_env = [\n\
               { name = \"FREERTOS_DIR\", help = \"k\" },\n\
               { name = \"LWIP_DIR\", help = \"n\", when = { capability = { ip_stack = true } } },\n\
             ]\n",
        )
        .expect("parse");
        let r = m.for_platform("freertos").expect("resolve");
        let target = "armv7r-none-eabi";

        let view = |caps: &BTreeMap<String, bool>| {
            (
                r.defines_conditional
                    .iter()
                    .filter(|d| matches(&d.when, target, true, caps))
                    .map(|d| d.name.as_str())
                    .collect::<Vec<_>>(),
                r.extra_sources
                    .iter()
                    .filter(|s| matches(&s.when, target, true, caps))
                    .map(|s| s.path.as_str())
                    .collect::<Vec<_>>(),
                r.required_env
                    .iter()
                    .filter(|e| matches(&e.when, target, true, caps))
                    .map(|e| e.name.as_str())
                    .collect::<Vec<_>>(),
            )
        };

        let on = view(&caps(&[("ip_stack", true)]));
        assert_eq!(on.0, vec!["ZENOH_FREERTOS_LWIP"]);
        assert_eq!(
            on.1,
            vec![
                "{src}/system/freertos/system.c",
                "{src}/system/freertos/lwip/network.c"
            ]
        );
        assert_eq!(on.2, vec!["FREERTOS_DIR", "LWIP_DIR"]);

        let off = view(&caps(&[("ip_stack", false)]));
        assert_eq!(off.0, vec!["ZENOH_ORIN_SPE"]);
        assert_eq!(
            off.1,
            vec!["{src}/system/freertos/system.c"],
            "the netstack TU is the 80,846 bytes of bss issue 1143 measured"
        );
        assert_eq!(off.2, vec!["FREERTOS_DIR"]);

        // And the state every build without a board rung is in: neither arm,
        // which is why `ZENOH_GENERIC` alone is not enough and the real block
        // keeps the platform declaring `ip_stack = true`.
        let undeclared = view(&BTreeMap::new());
        assert!(undeclared.0.is_empty());
    }

    /// The `inherits` merge has to carry the new list, parent first — the
    /// shape every other list field in this schema uses.
    #[test]
    fn defines_conditional_merges_down_the_inherits_chain() {
        let m: PlatformManifest = toml::from_str(
            "[platform.base]\n\
             defines_conditional = [{ name = \"A\", when = { capability = { x = true } } }]\n\
             [platform.child]\n\
             inherits = \"base\"\n\
             defines_conditional = [{ name = \"B\", when = { capability = { x = false } } }]\n",
        )
        .expect("parse");
        let r = m.for_platform("child").expect("resolve");
        let names: Vec<_> = r
            .defines_conditional
            .iter()
            .map(|d| d.name.as_str())
            .collect();
        assert_eq!(names, vec!["A", "B"]);
    }
}
