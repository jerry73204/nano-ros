//! Workspace and package discovery for host planning.

use cargo_nano_ros::package_xml::PackageXml;
use eyre::{Result, WrapErr};
use serde::Deserialize;
use serde_json::{Value as JsonValue, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
    sync::{LazyLock, Mutex},
};

use super::{
    cargo_metadata_schema::{ComponentMetadata, PackageMetadataNros},
    config::{ComponentConfig, ComponentLinkage, ComponentMetadataConfig, ComponentOverrides},
    source_metadata::ComponentLanguage,
};

/// Permissive envelope for extracting a `[component]` table out of a package's
/// `nros.toml` (Phase 172 W.1 fold) while ignoring sibling tables
/// (`[workspace]` / `[system]` / `[deploy]` / `[node]` / `[[transport]]`).
/// Unknown keys are ignored on purpose — only `[component]` is read here.
#[derive(Debug, Deserialize)]
struct ComponentEnvelope {
    #[serde(default)]
    component: Option<ComponentConfig>,
}

/// Load a component declaration from a manifest path. Handles two forms:
///
/// - **Folded** (Phase 172 W.1): a `[component]` table inside a package's
///   `nros.toml`. Returns `Ok(None)` when that file carries no `[component]`
///   (it is a workspace-root / direct-mode manifest, not a component).
/// - **Legacy**: the standalone whole-file form (`component_nros.toml` or
///   `nros/components/*.toml`), which is deprecated and warns once per file.
pub fn load_component_config(path: &Path) -> Result<Option<ComponentConfig>> {
    let raw = fs::read_to_string(path)
        .wrap_err_with(|| format!("failed to read component manifest {}", path.display()))?;
    let is_nros_toml = path.file_name().and_then(|name| name.to_str()) == Some("nros.toml");
    if is_nros_toml {
        let envelope: ComponentEnvelope =
            toml::from_str(&raw).wrap_err_with(|| format!("failed to parse {}", path.display()))?;
        Ok(envelope.component)
    } else {
        warn_legacy_component_manifest(path);
        let config: ComponentConfig = toml::from_str(&raw)
            .wrap_err_with(|| format!("failed to parse component manifest {}", path.display()))?;
        Ok(Some(config))
    }
}

/// Emit the `component_nros.toml` deprecation notice at most once per file path
/// for the life of the process (Phase 172 W.1 deprecation window).
fn warn_legacy_component_manifest(path: &Path) {
    static WARNED: LazyLock<Mutex<BTreeSet<PathBuf>>> =
        LazyLock::new(|| Mutex::new(BTreeSet::new()));
    if WARNED.lock().unwrap().insert(path.to_path_buf()) {
        eprintln!(
            "warning: `{}` is a deprecated standalone component manifest; fold it into the \
             package's `nros.toml` as a `[component]` table (Phase 172 W.1). The standalone \
             form still works during the deprecation window.",
            path.display()
        );
    }
}

#[derive(Debug, Clone)]
pub struct Workspace {
    pub root: PathBuf,
    pub packages: Vec<Package>,
}

#[derive(Debug, Clone)]
pub struct Package {
    pub name: String,
    pub root: PathBuf,
    pub package_xml: PathBuf,
    /// Phase 254 — the package's `system.toml` (bringup pkg), the typed
    /// capability/topology SSoT (RFC-0004). `None` for non-bringup packages.
    pub system_toml: Option<PathBuf>,
    pub launch_files: Vec<PathBuf>,
    pub manifest_files: Vec<PathBuf>,
    pub metadata_files: Vec<PathBuf>,
    /// Component-declaration candidates that tie a package to a
    /// `nros::component!` export + its source-metadata path (Phase 126.B.7).
    /// In preference order: the package's folded `nros.toml` `[component]`
    /// table (W.1), the legacy standalone `component_nros.toml`, then any
    /// `nros/components/*.toml`. An `nros.toml` without a `[component]` table
    /// is filtered out at parse time (`load_component_config`).
    pub component_config_files: Vec<PathBuf>,
    /// Phase 212.M-F.17 — summaries derived from the package's
    /// `[package.metadata.nros.{component,components,node,nodes}]` tables
    /// in `Cargo.toml`. Populated at discovery time; one entry per
    /// declared component. Empty when the package has no `Cargo.toml`
    /// or no nros component metadata table.
    pub cargo_component_metadata: Vec<CargoComponentSummary>,
    /// Phase 219.L — summaries derived from `nano_ros_node_register(...)`
    /// calls in the package's `CMakeLists.txt`. Populated at discovery
    /// time; one entry per static call. Empty when the package has no
    /// `CMakeLists.txt` or no `nano_ros_node_register` call. Lets
    /// `nros metadata` / `nros plan` discover pure-C/C++ Node pkgs that
    /// carry only `package.xml` + `CMakeLists.txt` (no `nros.toml`, no
    /// `Cargo.toml`) — closes Phase 219 workflow-review Gap 6.
    pub cmake_component_metadata: Vec<CmakeNodeSummary>,
}

/// Phase 212.M-F.17 — α-bridge between the in-tree
/// `[package.metadata.nros.component]` / `…components.<Name>` Cargo
/// metadata and the planner's source-metadata pipeline.
///
/// The planner's `find_source_metadata` walk currently keys off the
/// `(package, executable)` pair recorded in a `metadata/*.json` sidecar
/// file. The Phase 212 in-tree fixtures dropped those sidecars in favor
/// of `[package.metadata.nros.component]`, leaving `find_source_metadata`
/// blind. `CargoComponentSummary` carries just enough information to
/// synthesise a minimal `JsonArtifact` for the planner's `(package,
/// executable)` match — full entity / param / remap synthesis is
/// intentionally out of scope (runtime `Component::register(ctx)`
/// carries those in the redesign).
/// Phase 219.L — summary derived from a single `nano_ros_node_register(...)`
/// call statically parsed from a package's `CMakeLists.txt`. Carries the
/// minimum information needed for `nros metadata --build` and `nros plan`
/// to dedup + identify C/C++ Node pkgs that don't ship `nros.toml` or
/// `Cargo.toml` component metadata.
///
/// Mirrors [`CargoComponentSummary`] field-for-field where the semantics
/// match (`package` / `component` / `executable` / `class`), so downstream
/// consumers can treat both summary kinds uniformly when the cmake-first
/// path is sufficient.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CmakeNodeSummary {
    /// `package.xml` `<name>` — matches the planner's `(package, executable)`
    /// key shape.
    pub package: String,
    /// `nano_ros_node_register(NAME …)` value. Same as ROS 2 composable
    /// node's "instance name".
    pub component: String,
    /// Defaults to the value of `NAME`. Phase 212.L's `<pkg>_<NAME>_component`
    /// static lib target is what the linked Entry pkg references; the
    /// executable shape here keeps parity with [`CargoComponentSummary`].
    pub executable: String,
    /// `nano_ros_node_register(CLASS <pkg_sym>::<UserClass>)` value.
    pub class: Option<String>,
    /// `nano_ros_node_register(LANGUAGE C|CPP)` value, or a conservative
    /// inference from the class shape for pre-223 callers.
    pub language: ComponentLanguage,
    /// `nano_ros_node_register(DEPLOY <target>[ <target>...])` values.
    pub deploy_targets: Vec<String>,
    /// phase-308 W1 — `HEADER <hdr>` when the verb declares one. The metadata
    /// probe must `#include` the class's header to construct it; with no
    /// explicit value the convention `<pkg>/<Class>.hpp` applies (derived at
    /// the point of use, never guessed past an explicit declaration).
    pub header: Option<String>,
    /// phase-308 W1 — `SHAPE rclcpp|configure`. Decides how the probe drives
    /// the declaration: call `configure(node)` directly, or go through the
    /// rclcpp-compat factory. Defaults to `rclcpp` in the cmake verb.
    pub shape: Option<String>,
    /// phase-308 W1 — the CMake library target the component's sources build
    /// into (`nros_components_register_node(<lib> …)`, or the node name for
    /// the `nano_ros_add_node` spelling).
    ///
    /// The metadata probe must LINK it, not merely `add_subdirectory` the
    /// package: linking is what carries the target's PUBLIC include dirs, so
    /// without it the probe cannot even `#include` the component's own header.
    pub library_target: Option<String>,
    /// Absolute path to the `CMakeLists.txt` the summary was derived from.
    pub manifest_path: PathBuf,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CargoComponentSummary {
    /// Cargo `[package].name` the component belongs to. Used by the
    /// planner's `find_source_metadata` package match.
    pub package: String,
    /// Cargo `[package].name` — the CRATE name, which routinely diverges from
    /// `package` (the `package.xml` `<name>`). phase-307 W1 needs the real
    /// crate name: the metadata harness declares the component as a cargo path
    /// dependency and must `use` it under its rustc-visible name.
    pub crate_name: String,
    /// Short component instance name. Derived per Phase 212.M-F.17:
    /// `metadata.name` when present, else the multi-shape table key,
    /// else the class basename (`talker_pkg::Talker` → `Talker`),
    /// else the package name.
    pub component: String,
    /// Executable name. Defaults to the package name; overridden when a
    /// `[[bin]] name = …` row matches the component name.
    pub executable: String,
    /// `metadata.class` when present (`<pkg-dir>::<UserClass>`). Threaded
    /// through to the synthetic JSON so downstream readers that care
    /// about the class can still find it.
    pub class: Option<String>,
    /// `metadata.default_namespace` when present.
    pub default_namespace: Option<String>,
    /// Absolute path to the `Cargo.toml` the summary was derived from.
    /// Recorded so synthetic JSON artifacts can name a real on-disk path
    /// for diagnostics (matches the file-artifact `path` field shape).
    pub manifest_path: PathBuf,
    /// phase-308 — the package ALSO declares `[package.metadata.nros.entry]`,
    /// i.e. it is a self-contained standalone example (issue 0100) rather than
    /// the canonical platform-agnostic Node pkg.
    ///
    /// Such a package deps its board crate directly, so it cannot be compiled
    /// for the host and cannot be metadata-probed. Recorded here rather than
    /// discovered by the producer, so the producer can report the exclusion
    /// instead of failing the build on ARM inline asm.
    pub deploy_bound: bool,
    /// phase-267 W1c/C3 — topics the node declares it PUBLISHES, carried from
    /// `[package.metadata.nros.node].publishes` into the synthetic metadata so
    /// the planner resolves a `[[bridge]]`'s topic names to types pre-build.
    pub publishes: Vec<super::cargo_metadata_schema::TopicDecl>,
    /// phase-267 W1c/C3 — topics the node declares it SUBSCRIBES to.
    pub subscribes: Vec<super::cargo_metadata_schema::TopicDecl>,
}

impl Workspace {
    pub fn discover(root: &Path) -> Result<Self> {
        let mut packages = Vec::new();
        let root = root.to_path_buf();
        if root.join("package.xml").is_file() {
            packages.push(discover_package(&root)?);
        }
        let src = root.join("src");
        if src.is_dir() {
            for entry in fs::read_dir(&src)? {
                let entry = entry?;
                let path = entry.path();
                if path.join("package.xml").is_file() {
                    packages.push(discover_package(&path)?);
                }
            }
        }
        packages.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(Self { root, packages })
    }

    /// Phase 212.M-F.17 — synthesise `(manifest_path, json_value)` tuples
    /// from every package's `[package.metadata.nros.{component,components,
    /// node,nodes}]` table. The planner appends these to its `JsonArtifact`
    /// list AFTER the sidecar file artifacts so sidecars win the dedup pass
    /// in `schema_components` (back-compat: a package shipping both an
    /// authoritative metadata JSON and a stub component table keeps the
    /// file's richer data on the plan).
    ///
    /// The synthetic JSON carries the minimum keys the planner's
    /// `find_source_metadata` `(package, executable)` walk needs plus
    /// the downstream `schema_components` dedup id (`package` +
    /// `component` + `language`). `class` / `default_namespace` flow
    /// through when present.
    ///
    /// Each tuple's first element is the source `Cargo.toml`; downstream
    /// callers that mint a `JsonArtifact` use it as the artifact `path`
    /// so diagnostics name a real on-disk file.
    pub fn synthetic_metadata_artifacts(&self) -> Vec<(PathBuf, JsonValue)> {
        let mut out = Vec::new();
        for pkg in &self.packages {
            for summary in &pkg.cargo_component_metadata {
                out.push((
                    summary.manifest_path.clone(),
                    summary_to_synthetic_json(summary),
                ));
            }
            for summary in &pkg.cmake_component_metadata {
                out.push((
                    summary.manifest_path.clone(),
                    cmake_summary_to_synthetic_json(summary),
                ));
            }
        }
        out
    }

    pub fn source_metadata_files(&self) -> Vec<PathBuf> {
        unique_paths(
            self.packages
                .iter()
                .flat_map(|pkg| pkg.metadata_files.iter().cloned()),
        )
    }

    pub fn manifest_files(&self) -> Vec<PathBuf> {
        unique_paths(
            self.packages
                .iter()
                .flat_map(|pkg| pkg.manifest_files.iter().cloned()),
        )
    }

    /// Phase 254 — the package's `system.toml` path (the bringup pkg's typed
    /// capability/topology SSoT). `None` if the package has none.
    pub fn package_system_toml(&self, package: &str) -> Option<PathBuf> {
        self.packages
            .iter()
            .find(|pkg| pkg.name == package)
            .and_then(|pkg| pkg.system_toml.clone())
    }

    /// Iterate every component declaration in the workspace — folded
    /// `nros.toml` `[component]` tables (W.1) and legacy standalone
    /// `component_nros.toml` / `nros/components/*.toml` files — as
    /// `(package_root, manifest_path, parsed_config)` tuples, deduped by
    /// `(package, component)`. Used by the metadata command to detect packages
    /// that
    /// declared themselves nros components but lack the
    /// `nros::component!` export (their `[metadata].source_metadata`
    /// path doesn't exist on disk — see Phase 126.B.7 acceptance
    /// criterion).
    pub fn component_declarations(&self) -> Result<Vec<ComponentDeclaration>> {
        let mut out = Vec::new();
        for pkg in &self.packages {
            // Dedup by `(package, component)` within a package, first-wins. The
            // folded `nros.toml` sorts ahead of a legacy `component_nros.toml`,
            // so when both declare the same component the folded form wins and
            // the legacy file is ignored (it still warns once on read).
            // Phase 219.L appends cmake-derived declarations LAST in the same
            // dedup pass, so any `nano_ros_node_register(NAME …)` whose
            // `(package, NAME)` matches an explicit `[component]` table is
            // silently superseded — the explicit metadata wins.
            let mut seen = BTreeSet::new();
            for manifest_path in &pkg.component_config_files {
                // A package `nros.toml` is a candidate only if it actually
                // carries a `[component]` table (W.1 fold); skip it otherwise.
                let Some(config) = load_component_config(manifest_path)? else {
                    continue;
                };
                if !seen.insert((config.package.clone(), config.component.clone())) {
                    continue;
                }
                out.push(ComponentDeclaration {
                    package_root: pkg.root.clone(),
                    manifest_path: manifest_path.clone(),
                    // phase-330 / issue 0392 B — honour the manifest's `class`
                    // instead of always discarding it. This was `None`, which
                    // forced the harness to guess the type from the component
                    // id and made any legacy manifest with a bare instance name
                    // unsyncable.
                    class: config.class.clone(),
                    crate_name: Some(config.linkage.resolved_crate_name(&config.package)),
                    deploy_bound: false,
                    header: None,
                    shape: None,
                    library_target: None,
                    config,
                });
            }
            // Phase 219.L — synthesise declarations for `nano_ros_node_register`
            // calls statically parsed from `CMakeLists.txt` (pure-C/C++ Node
            // pkgs that ship no `nros.toml` / `Cargo.toml` component
            // metadata). Closes Phase 219 workflow-review Gap 6.
            for summary in &pkg.cmake_component_metadata {
                if !seen.insert((summary.package.clone(), summary.component.clone())) {
                    continue;
                }
                out.push(ComponentDeclaration {
                    package_root: pkg.root.clone(),
                    manifest_path: summary.manifest_path.clone(),
                    class: summary.class.clone(),
                    crate_name: None,
                    deploy_bound: false,
                    header: summary.header.clone(),
                    shape: summary.shape.clone(),
                    library_target: summary.library_target.clone(),
                    config: cmake_summary_to_component_config(summary),
                });
            }
            // phase-307 W1 — the canonical shipping Rust shape is a lib-only
            // Node pkg declaring `[package.metadata.nros.node]`, which
            // `discover_cargo_component_metadata` has always PARSED but which
            // never became a declaration. Without this loop `nros metadata
            // --build` has no candidates in any real workspace, which is why
            // no `source-metadata.json` exists anywhere in the tree outside the
            // hand-written test fixtures. Appended last in the same dedup pass
            // as the cmake loop: an explicit `[component]` table still wins.
            for summary in &pkg.cargo_component_metadata {
                if !seen.insert((summary.package.clone(), summary.component.clone())) {
                    continue;
                }
                out.push(ComponentDeclaration {
                    package_root: pkg.root.clone(),
                    manifest_path: summary.manifest_path.clone(),
                    class: summary.class.clone(),
                    crate_name: Some(summary.crate_name.clone()),
                    deploy_bound: summary.deploy_bound,
                    header: None,
                    shape: None,
                    library_target: None,
                    config: cargo_summary_to_component_config(summary),
                });
            }
        }
        Ok(out)
    }
}

/// Phase 219.L — synthesise a [`ComponentConfig`] from a CMake-derived
/// [`CmakeNodeSummary`]. Mirrors the shape `discover_cargo_component_metadata`
/// produces for the Cargo path: minimum keys to identify the component
/// `(package, component, language)` plus an optional class threaded through
/// for downstream consumers.
fn cmake_summary_to_component_config(summary: &CmakeNodeSummary) -> ComponentConfig {
    ComponentConfig {
        version: 1,
        package: summary.package.clone(),
        component: summary.component.clone(),
        class: None,
        language: summary.language,
        linkage: ComponentLinkage::default(),
        metadata: ComponentMetadataConfig {
            source_metadata: format!("metadata/{}.json", summary.component),
            generated_by: Some("nano_ros_node_register".to_string()),
        },
        overrides: ComponentOverrides::default(),
    }
}

/// phase-307 W1 — synthesise a [`ComponentConfig`] from a Cargo-derived
/// [`CargoComponentSummary`], mirroring [`cmake_summary_to_component_config`].
///
/// The sidecar path deliberately matches the cmake lowering's
/// `metadata/<component>.json`: `Package::metadata_files` already collects
/// `metadata/*.json`, so a produced sidecar is discovered by
/// `source_metadata_files()` on the next run with no further wiring.
fn cargo_summary_to_component_config(summary: &CargoComponentSummary) -> ComponentConfig {
    ComponentConfig {
        version: 1,
        package: summary.package.clone(),
        component: summary.component.clone(),
        class: None,
        language: ComponentLanguage::Rust,
        linkage: ComponentLinkage {
            crate_name: Some(summary.crate_name.clone()),
            ..ComponentLinkage::default()
        },
        metadata: ComponentMetadataConfig {
            source_metadata: format!("metadata/{}.json", summary.component),
            generated_by: Some("package.metadata.nros.node".to_string()),
        },
        overrides: ComponentOverrides {
            default_namespace: summary.default_namespace.clone(),
            ..ComponentOverrides::default()
        },
    }
}

/// Phase 219.L / 223 — Best-effort language inference for CMake Node pkgs.
/// `LANGUAGE` is authoritative when present. Older CMakeLists omitted it, so
/// fall back to the historical class-shape heuristic.
///
/// # `rust` belongs here, and leaving it out was not free (issue 0641)
///
/// The doc line above says `LANGUAGE` is authoritative, and for `RUST` it was
/// not: the match knew `c`/`cpp`/`cxx`, so `LANGUAGE RUST` fell through to the
/// `_` arm, met a class containing `::`, and came back **Cpp**. A Rust node
/// package was then handed to the C/C++ metadata probe, which emits a C++ TU
/// that `#include`s a header the package does not have.
///
/// The cost was not a wrong answer but a repeated one. That probe configures
/// and builds a whole CMake project (Corrosion included), fails at
/// `fatal error: rust_heartbeat_pkg/Heartbeat.hpp: No such file`, and the
/// failure is not remembered for a non-`deploy_bound` component — so EVERY
/// `nros sync` of `examples/workspaces/mixed` re-ran it. Measured: 1.2 s warm
/// against ~0.25 s for its sibling workspaces, and 12.5 s whenever the probe
/// tree needed real work.
///
/// An unrecognised value now falls back LOUDLY rather than silently, because a
/// silent fallback is exactly what hid this: the declaration said one thing,
/// the inference did another, and nothing printed.
fn infer_cmake_language(language: Option<&str>, class: Option<&str>) -> ComponentLanguage {
    match language.map(|s| s.to_ascii_lowercase()) {
        Some(lang) if lang == "c" => ComponentLanguage::C,
        Some(lang) if lang == "cpp" || lang == "cxx" => ComponentLanguage::Cpp,
        Some(lang) if lang == "rust" || lang == "rs" => ComponentLanguage::Rust,
        Some(lang) => {
            eprintln!(
                "nros: `LANGUAGE {lang}` is not one of c/cpp/cxx/rust — guessing from the \
                 class shape instead. Fix the declaration; a wrong guess here routes the \
                 component to the wrong metadata probe (issue 0641)."
            );
            infer_language_from_class(class).unwrap_or(ComponentLanguage::Cpp)
        }
        None => infer_language_from_class(class).unwrap_or(ComponentLanguage::Cpp),
    }
}

fn infer_language_from_class(class: Option<&str>) -> Option<ComponentLanguage> {
    let class = class?;
    if class.contains("::") {
        Some(ComponentLanguage::Cpp)
    } else {
        Some(ComponentLanguage::C)
    }
}

/// Parsed component manifest paired with its on-disk location.
#[derive(Debug, Clone)]
pub struct ComponentDeclaration {
    /// Package root the manifest belongs to. `source_metadata` paths
    /// in the manifest resolve relative to this directory.
    pub package_root: PathBuf,
    /// Absolute path to the manifest the declaration came from — a package's
    /// folded `nros.toml` (W.1) or a legacy standalone `component_nros.toml` /
    /// `nros/components/*.toml`.
    pub manifest_path: PathBuf,
    pub config: ComponentConfig,
    /// phase-307 W1 — the registered type's FULLY QUALIFIED path
    /// (`talker_pkg::Talker`), verbatim from the declaring manifest's `class`.
    ///
    /// The metadata harness used to GUESS this as `<crate>::<module>::Component`
    /// from the component id, which the shipping `nros::node!(Class)` shape
    /// (`impl Node for Class`, no `Component` alias, no module segment) never
    /// matches. Carrying the declared class removes the guess. `None` for the
    /// legacy `crate::module` manifests, where the guess IS the convention.
    pub class: Option<String>,
    /// phase-307 W1 — rustc-visible crate name for the harness's path dep.
    /// `None` falls back to the component id's first `::` segment.
    pub crate_name: Option<String>,
    /// phase-308 — the declaring package is bound to a deploy target (it also
    /// declares an Entry, so it deps a board crate) and cannot be compiled for
    /// the host. The metadata producer reports these instead of probing them.
    pub deploy_bound: bool,
    /// phase-308 W1 — header the C/C++ probe must `#include` to construct the
    /// class. `None` for Rust declarations and for CMake ones that declare no
    /// explicit `HEADER`; the convention fallback is derived at the point of
    /// use ([`ComponentDeclaration::probe_header`]).
    pub header: Option<String>,
    /// phase-308 W1 — `rclcpp` | `configure`: how the probe drives the
    /// declaration path. `None` for Rust.
    pub shape: Option<String>,
    /// phase-308 W1 — the CMake library target the probe links. `None` for Rust.
    pub library_target: Option<String>,
}

impl ComponentDeclaration {
    /// phase-308 W1 — the header the probe TU must `#include`.
    ///
    /// An explicit `HEADER <hdr>` on the cmake verb wins. With none, the
    /// in-tree convention is `<pkg>/<Class>.hpp` derived from the declared
    /// class (`talker_pkg::Talker` → `talker_pkg/Talker.hpp`) — which is what
    /// every C/C++ example ships. `None` when there is no class to derive
    /// from: the probe cannot be generated and the caller must say so rather
    /// than guess a path that will fail to compile.
    pub fn probe_header(&self) -> Option<String> {
        if let Some(h) = self.header.as_deref().filter(|h| !h.is_empty()) {
            return Some(h.to_string());
        }
        let class = self.class.as_deref()?;
        let (ns, leaf) = class.rsplit_once("::")?;
        // A nested namespace maps to the outermost segment: the include root is
        // the package dir, not the full path.
        let pkg = ns.split("::").next().unwrap_or(ns);
        Some(format!("{pkg}/{leaf}.hpp"))
    }

    /// phase-308 W1 — how the probe drives the declaration path.
    /// Defaults to `rclcpp`, matching the cmake verb's own default.
    pub fn probe_shape(&self) -> &str {
        self.shape.as_deref().unwrap_or("rclcpp")
    }

    /// Absolute path to the `[metadata].source_metadata` file the
    /// component is expected to emit. Relative paths resolve against
    /// `package_root`.
    pub fn source_metadata_path(&self) -> PathBuf {
        let raw = Path::new(&self.config.metadata.source_metadata);
        if raw.is_absolute() {
            raw.to_path_buf()
        } else {
            self.package_root.join(raw)
        }
    }
}

fn discover_package(root: &Path) -> Result<Package> {
    let package_xml = root.join("package.xml");
    let parsed = PackageXml::parse(&package_xml)
        .wrap_err_with(|| format!("failed to parse {}", package_xml.display()))?;
    // Phase 212.M-F.17 fix: the synth artifact's `package` field must match
    // what `<node pkg="…"/>` in the launch XML references. ROS convention
    // makes `package.xml` `<name>` the canonical pkg key; Cargo.toml
    // `[package].name` is often a *crate* name that diverges (e.g.
    // `talker_pkg` vs `talker_pkg_component`). Drive synthesis from the
    // package.xml name so `find_source_metadata` matches.
    let cargo_component_metadata = discover_cargo_component_metadata(root, &parsed.name)?;
    let cmake_component_metadata = discover_cmake_node_metadata(root, &parsed.name)?;
    Ok(Package {
        name: parsed.name,
        root: root.to_path_buf(),
        package_xml,
        // Phase 254 — the bringup package's `system.toml` (the capability/topology
        // SSoT both codegen paths read).
        system_toml: root
            .join("system.toml")
            .is_file()
            .then(|| root.join("system.toml")),
        launch_files: collect_files(
            root,
            &["launch"],
            &["launch.py", "launch.xml", "launch.yaml", "launch.yml"],
        )?,
        manifest_files: collect_files(
            root,
            &["manifest", "manifests"],
            &["launch.yaml", "launch.yml"],
        )?,
        metadata_files: collect_files(root, &["metadata", "nros", "target/nros"], &["json"])?,
        component_config_files: discover_component_configs(root)?,
        cargo_component_metadata,
        cmake_component_metadata,
    })
}

/// Phase 219.L — statically parse the package's `CMakeLists.txt` for
/// `nano_ros_node_register(NAME … CLASS … SOURCES … DEPLOY …)` calls and
/// synthesise one [`CmakeNodeSummary`] per call. Returns an empty vec
/// when the package has no `CMakeLists.txt` or no
/// `nano_ros_node_register` calls (e.g. an Entry pkg using
/// `nano_ros_entry(...)`, an interface-only pkg, a Bringup pkg, …).
///
/// **Why static parse, not cmake-configure-first.** Phase 219 review
/// Gap 6 noted two options: (a) extend the walker, (b) require a prior
/// `cmake configure` whose `${CMAKE_BINARY_DIR}/nros-metadata.json` the
/// CLI then reads. (a) wins because:
/// - the user's first `nros metadata` / `nros plan` call needs to work
///   without an explicit configure step ("solo planner mode");
/// - the cmake fn args are single-line + keyword form, so the regex
///   walker is small + bounded.
///
/// Parse policy is conservative: parser failures (malformed argument
/// list, missing required keyword) skip the offending call rather than
/// failing the whole discovery — partial information beats none for
/// `nros metadata --build`. A future hardening pass can promote
/// skipped-call diagnostics to warnings.
fn discover_cmake_node_metadata(root: &Path, package_name: &str) -> Result<Vec<CmakeNodeSummary>> {
    let cmakelists = root.join("CMakeLists.txt");
    if !cmakelists.is_file() {
        return Ok(Vec::new());
    }
    let text = fs::read_to_string(&cmakelists)
        .wrap_err_with(|| format!("read {}", cmakelists.display()))?;
    let stripped = strip_cmake_comments(&text);
    let mut out = Vec::new();
    for call in extract_cmake_calls(&stripped, "nano_ros_node_register") {
        let Some(args) = parse_cmake_kwargs(&call) else {
            continue;
        };
        let Some(name) = args.single("NAME") else {
            continue;
        };
        let class = args.single("CLASS");
        let language_kw = args.single("LANGUAGE").or_else(|| args.single("LANG"));
        let language = infer_cmake_language(language_kw.as_deref(), class.as_deref());
        out.push(CmakeNodeSummary {
            package: package_name.to_string(),
            component: name.clone(),
            class,
            language,
            deploy_targets: args.multi("DEPLOY"),
            header: args.single("HEADER"),
            shape: args.single("SHAPE"),
            // The library `nano_ros_node_register` builds is
            // `<PROJECT_NAME>_<NAME>_component` (NanoRosNodeRegister.cmake), NOT the
            // bare node name — and it exists under that name even in EXISTING_TARGET
            // mode, where the verb adds an INTERFACE library aliasing the caller's
            // target. So it is the one spelling that is always linkable. Issue 0939.
            library_target: args
                .single("TARGET")
                .or_else(|| Some(component_library_target(package_name, &name))),
            executable: name,
            manifest_path: cmakelists.clone(),
        });
    }
    // RFC-0048 (phase-287 W6) — the ament-shape spelling of a workspace component:
    // `nano_ros_add_node(<name> CLASS <ns::Class> [TYPED] <sources…> [DEPLOY <t>…])`.
    // Positional name + sources (unlike node_register's all-keyword form), so it needs
    // a dedicated parse. Language is inferred from the source extensions.
    for call in extract_cmake_calls(&stripped, "nano_ros_add_node") {
        let Some(summary) = parse_add_node_call(&call, package_name, &cmakelists) else {
            continue;
        };
        out.push(summary);
    }
    // RFC-0057 (phase-305) — the split spelling:
    //   nano_ros_auto_add_library(<lib> [STATIC] <sources…>)
    //   nros_components_register_node(<lib> PLUGIN <ns::Class> EXECUTABLE <name> …)
    // The register call carries no sources; recover them from the matching
    // auto_add_library call in the same file for language inference.
    let mut lib_sources: std::collections::HashMap<String, Vec<String>> =
        std::collections::HashMap::new();
    for call in extract_cmake_calls(&stripped, "nano_ros_auto_add_library") {
        let toks = tokenize_cmake_body(&call);
        let mut it = toks.into_iter().filter(|t| t != "STATIC" && t != "SHARED");
        if let Some(lib) = it.next() {
            lib_sources.insert(lib, it.collect());
        }
    }
    for call in extract_cmake_calls(&stripped, "nros_components_register_node") {
        let Some(summary) =
            parse_register_node_call(&call, package_name, &cmakelists, &lib_sources)
        else {
            continue;
        };
        out.push(summary);
    }
    Ok(out)
}

/// Parse a `nros_components_register_node(<lib> PLUGIN <ns::Class>
/// EXECUTABLE <name> [HEADER <h>] [SHAPE <s>] [TYPED] [DEPLOY <t>…]
/// [CALLBACK_GROUPS <g>…])` call body (RFC-0057). Sources come from the
/// sibling `nano_ros_auto_add_library` call via `lib_sources`.
fn parse_register_node_call(
    body: &str,
    package_name: &str,
    cmakelists: &Path,
    lib_sources: &std::collections::HashMap<String, Vec<String>>,
) -> Option<CmakeNodeSummary> {
    const KEYWORDS: &[&str] = &[
        "PLUGIN",
        "EXECUTABLE",
        "TYPED",
        "DEPLOY",
        "HEADER",
        "SHAPE",
        "CALLBACK_GROUPS",
    ];
    let tokens = tokenize_cmake_body(body);
    let mut target: Option<String> = None;
    let mut plugin: Option<String> = None;
    let mut executable: Option<String> = None;
    let mut deploy: Vec<String> = Vec::new();
    // phase-308 W1 — captured, not skipped: the probe needs the header to
    // `#include` and the shape to know how to drive the declaration.
    let mut header: Option<String> = None;
    let mut shape: Option<String> = None;
    let mut current: Option<&str> = None;
    for tok in tokens {
        if KEYWORDS.contains(&tok.as_str()) {
            current = match tok.as_str() {
                "PLUGIN" => Some("PLUGIN"),
                "EXECUTABLE" => Some("EXECUTABLE"),
                "DEPLOY" => Some("DEPLOY"),
                "HEADER" => Some("HEADER"),
                "SHAPE" => Some("SHAPE"),
                "CALLBACK_GROUPS" => Some("SKIPN"),
                _ => None, // TYPED — bare flag
            };
            continue;
        }
        match current {
            Some("PLUGIN") => {
                plugin = Some(tok);
                current = None;
            }
            Some("EXECUTABLE") => {
                executable = Some(tok);
                current = None;
            }
            Some("HEADER") => {
                header = Some(tok);
                current = None;
            }
            Some("SHAPE") => {
                shape = Some(tok);
                current = None;
            }
            Some("SKIPN") => {}
            Some("DEPLOY") => deploy.push(tok),
            _ => {
                if target.is_none() {
                    target = Some(tok);
                }
            }
        }
    }
    let executable = executable?;
    let empty: Vec<String> = Vec::new();
    let sources = target
        .as_ref()
        .and_then(|t| lib_sources.get(t))
        .unwrap_or(&empty);
    let language = language_from_sources_or_class(
        sources,
        plugin.as_deref(),
        &format!("{package_name}: nros_components_register_node(EXECUTABLE {executable})"),
    );
    Some(CmakeNodeSummary {
        package: package_name.to_string(),
        component: executable.clone(),
        executable,
        class: plugin,
        language,
        deploy_targets: deploy,
        header,
        shape,
        library_target: target,
        manifest_path: cmakelists.to_path_buf(),
    })
}

/// Parse a `nano_ros_add_node(<name> CLASS <ns::Class> [TYPED] <src…> [DEPLOY <t>…])`
/// call body. The first non-keyword token is the node name; `CLASS` takes one
/// value; `TYPED` is a bare flag; `DEPLOY` takes values; every other bare token is
/// a source. Language is inferred from source extensions, else the CLASS.
/// The CMake library a registered component builds into.
///
/// `NanoRosNodeRegister.cmake` names it `${PROJECT_NAME}_${NAME}_component`, and
/// guarantees a target by that name in both modes: it either builds the library
/// itself, or — under `EXISTING_TARGET` — adds an INTERFACE library of that name
/// linking the caller's target. The bare node name is NOT a target, so a probe
/// that links it fails at `-l<name>`.
///
/// Issue 0939: the probe did link the bare name, so `nros sync` reported "no
/// producer for <pkg>::<node>" for every C/C++ component and then cached that as
/// "probe failed at this source last sync; unchanged" — which is why the
/// underlying linker error stayed invisible across runs.
fn component_library_target(package: &str, name: &str) -> String {
    format!("{package}_{name}_component")
}

fn parse_add_node_call(
    body: &str,
    package_name: &str,
    cmakelists: &Path,
) -> Option<CmakeNodeSummary> {
    // HEADER / SHAPE (one value) + CALLBACK_GROUPS (multi) ride through the
    // verb to `nano_ros_node_register` (287-W6 workspace slice 2) — the parser
    // must consume their VALUES so they are not misread as positional sources.
    const KEYWORDS: &[&str] = &[
        "CLASS",
        "TYPED",
        "DEPLOY",
        "HEADER",
        "SHAPE",
        "CALLBACK_GROUPS",
        "SOURCES",
        "LANGUAGE",
    ];
    let tokens = tokenize_cmake_body(body);
    let mut name: Option<String> = None;
    let mut class: Option<String> = None;
    let mut deploy: Vec<String> = Vec::new();
    let mut sources: Vec<String> = Vec::new();
    let mut header: Option<String> = None;
    let mut shape: Option<String> = None;
    let mut language_kw: Option<String> = None;
    let mut current: Option<&str> = None;
    for tok in tokens {
        if KEYWORDS.contains(&tok.as_str()) {
            current = match tok.as_str() {
                "CLASS" => Some("CLASS"),
                "DEPLOY" => Some("DEPLOY"),
                "HEADER" => Some("HEADER"),
                "SHAPE" => Some("SHAPE"),
                "CALLBACK_GROUPS" => Some("CALLBACK_GROUPS"),
                "SOURCES" => Some("SOURCES"),
                "LANGUAGE" => Some("LANGUAGE"),
                _ => None, // TYPED — bare flag, no value
            };
            continue;
        }
        match current {
            Some("CLASS") => {
                class = Some(tok);
                current = None;
            }
            Some("LANGUAGE") => {
                language_kw = Some(tok);
                current = None;
            }
            Some("HEADER") => {
                header = Some(tok);
                current = None;
            }
            Some("SHAPE") => {
                shape = Some(tok);
                current = None;
            }
            Some("CALLBACK_GROUPS") => {
                // multi-value; swallow until the next keyword
            }
            Some("DEPLOY") => deploy.push(tok),
            Some("SOURCES") => sources.push(tok),
            _ => {
                // A bare token: the first is the node name, the rest are sources.
                if name.is_none() {
                    name = Some(tok);
                } else {
                    sources.push(tok);
                }
            }
        }
    }
    let name = name?;
    // An explicit LANGUAGE is the whole point of the keyword — it is the only
    // thing this scanner and cmake are guaranteed to read the same way.
    //
    // Without one, infer from the source extension and NOT the class: a C node's
    // class still uses `::` (e.g. `c_talker_pkg::Talker`), so class-shape
    // inference mislabels it Cpp. The class is the LAST resort, taken out loud,
    // for sources this scanner cannot read at all (issue 1062).
    let language = match language_kw.as_deref() {
        Some(kw) => infer_cmake_language(Some(kw), class.as_deref()),
        None => language_from_sources_or_class(
            &sources,
            class.as_deref(),
            &format!("{package_name}: nano_ros_add_node({name})"),
        ),
    };
    Some(CmakeNodeSummary {
        package: package_name.to_string(),
        component: name.clone(),
        class,
        language,
        deploy_targets: deploy,
        header,
        shape,
        // `nano_ros_add_node(<name> …)` builds a target of that name.
        // Same naming as node_register: this verb forwards to it. Issue 0939.
        library_target: Some(component_library_target(package_name, &name)),
        executable: name,
        manifest_path: cmakelists.to_path_buf(),
    })
}

/// C++ if any source carries a C++ extension, C if any carries `.c` — and
/// `None` when the list settles nothing.
///
/// # Why `None` exists (issue 1062)
///
/// This used to end in a bare `else C`, which made one answer serve two
/// unrelated questions: "every source is a C file" and "this list told me
/// nothing". The second is common, because the scanner reads `CMakeLists.txt`
/// as TEXT — `SOURCES ${_controller_sources}` is one token that expands to
/// nothing here, though cmake expands it fine at configure time. A C++
/// component then read as C, was probed against the C ABI seam, and the probe
/// failed to link against symbols `NROS_C_COMPONENT` never emitted.
///
/// Returning `None` hands the decision back to the caller, which has the class
/// shape to fall back on and can say out loud that it is guessing.
fn language_from_sources(sources: &[String]) -> Option<ComponentLanguage> {
    let mut saw_c = false;
    for s in sources {
        if s.ends_with(".cpp") || s.ends_with(".cxx") || s.ends_with(".cc") || s.ends_with(".C") {
            return Some(ComponentLanguage::Cpp);
        }
        if s.ends_with(".c") {
            saw_c = true;
        }
    }
    saw_c.then_some(ComponentLanguage::C)
}

/// Source extensions decide; when they cannot, the class shape decides and says
/// so. `decl` names the declaration in the warning so the message points at the
/// line to edit.
///
/// The class fallback is a guess and stays one — a C component whose class
/// carries `::` (`c_talker_pkg::Talker`) guesses Cpp. That is why it prints:
/// the remedy, `LANGUAGE`, travels with the guess. Silence is what let the
/// wrong answer through before (issues 1062, 0641).
fn language_from_sources_or_class(
    sources: &[String],
    class: Option<&str>,
    decl: &str,
) -> ComponentLanguage {
    if let Some(lang) = language_from_sources(sources) {
        return lang;
    }
    // No sources at all is not a mystery to report — there is nothing the
    // author could have written differently. An unreadable list is.
    if !sources.is_empty() {
        eprintln!(
            "nros: {decl}: no source in `{}` carries a C/C++ extension — a cmake \
             variable or generator expression expands at configure time but not \
             for this scanner. Guessing `{}` from the class shape; pass \
             `LANGUAGE C` or `LANGUAGE CPP` to state it (issue 1062).",
            sources.join(" "),
            match infer_language_from_class(class) {
                Some(ComponentLanguage::C) => "c",
                Some(ComponentLanguage::Rust) => "rust",
                _ => "cpp",
            }
        );
    }
    infer_language_from_class(class).unwrap_or(ComponentLanguage::Cpp)
}

/// Strip `#` line comments from a CMake source while preserving line
/// offsets (each comment span replaced by spaces) — keeps any byte-offset
/// based downstream parser stable.
fn strip_cmake_comments(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let mut in_comment = false;
    for ch in src.chars() {
        if in_comment {
            if ch == '\n' {
                in_comment = false;
                out.push(ch);
            } else {
                out.push(' ');
            }
        } else if ch == '#' {
            in_comment = true;
            out.push(' ');
        } else {
            out.push(ch);
        }
    }
    out
}

/// Return every `<fn_name>(...)` call body found in `src`. Handles
/// balanced parentheses inside the body (none of the cmake fns 219.L
/// cares about use nested parens, but the walker stays generic).
fn extract_cmake_calls(src: &str, fn_name: &str) -> Vec<String> {
    let mut out = Vec::new();
    let needle = format!("{fn_name}(");
    let bytes = src.as_bytes();
    let mut i = 0;
    while i + needle.len() <= bytes.len() {
        if !src.is_char_boundary(i) || !src[i..].starts_with(&needle) {
            i += 1;
            continue;
        }
        // Confirm boundary before the call (start of file OR non-identifier).
        // CMake identifiers include `_`, so `my_nano_ros_node_register(`
        // must NOT match the `nano_ros_node_register` needle.
        let prev_is_ident = i > 0 && (bytes[i - 1].is_ascii_alphanumeric() || bytes[i - 1] == b'_');
        if prev_is_ident {
            i += 1;
            continue;
        }
        let body_start = i + needle.len();
        let mut depth = 1usize;
        let mut j = body_start;
        while j < bytes.len() && depth > 0 {
            match bytes[j] {
                b'(' => depth += 1,
                b')' => depth -= 1,
                _ => {}
            }
            if depth == 0 {
                break;
            }
            j += 1;
        }
        if depth == 0 {
            out.push(src[body_start..j].to_string());
            i = j + 1;
        } else {
            // Unterminated call — give up on this match.
            break;
        }
    }
    out
}

/// Minimal CMake keyword-argument parser. Tokenises on whitespace,
/// strips `"`-delimited strings, then collects each known keyword's
/// values until the next keyword. Returns `None` only when the input
/// is empty.
fn parse_cmake_kwargs(body: &str) -> Option<CmakeKwargs> {
    // The valid keyword set for the cmake fns 219.L parses. Conservatively
    // wide to avoid eating later keywords as values.
    const KEYWORDS: &[&str] = &[
        "NAME",
        "CLASS",
        "LANGUAGE",
        "SOURCES",
        "DEPLOY",
        "BOARD",
        "LAUNCH",
        "ARGS",
        "LANG",
        "RMW",
        "DOMAIN_ID",
        "LOCATOR",
        "TARGET",
    ];
    let tokens = tokenize_cmake_body(body);
    if tokens.is_empty() {
        return None;
    }
    let mut map: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut current: Option<String> = None;
    for tok in tokens {
        if KEYWORDS.contains(&tok.as_str()) {
            current = Some(tok.clone());
            map.entry(tok).or_default();
        } else if let Some(key) = current.as_ref() {
            map.entry(key.clone()).or_default().push(tok);
        }
        // Tokens before the first keyword are dropped (CMake fns 219.L
        // cares about don't use positional args).
    }
    Some(CmakeKwargs { map })
}

fn tokenize_cmake_body(body: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut chars = body.chars().peekable();
    while let Some(&ch) = chars.peek() {
        if ch.is_whitespace() {
            chars.next();
            continue;
        }
        if ch == '"' {
            chars.next();
            let mut buf = String::new();
            while let Some(c) = chars.next() {
                if c == '\\' {
                    if let Some(esc) = chars.next() {
                        buf.push(esc);
                    }
                } else if c == '"' {
                    break;
                } else {
                    buf.push(c);
                }
            }
            out.push(buf);
            continue;
        }
        let mut buf = String::new();
        while let Some(&c) = chars.peek() {
            if c.is_whitespace() {
                break;
            }
            buf.push(c);
            chars.next();
        }
        if !buf.is_empty() {
            out.push(buf);
        }
    }
    out
}

#[derive(Debug, Default)]
struct CmakeKwargs {
    map: BTreeMap<String, Vec<String>>,
}

impl CmakeKwargs {
    fn single(&self, key: &str) -> Option<String> {
        self.map.get(key).and_then(|v| v.first().cloned())
    }
    fn multi(&self, key: &str) -> Vec<String> {
        self.map.get(key).cloned().unwrap_or_default()
    }
}

/// Phase 212.M-F.17 — read `<root>/Cargo.toml` and synthesise one
/// [`CargoComponentSummary`] per `[package.metadata.nros.{component,
/// components.<Name>}]` (and the post-N.12 `node` / `nodes` aliases)
/// entry. Returns an empty vec when:
///
/// * the package has no `Cargo.toml` (every embedded / CMake-only
///   package in the in-tree examples / fixtures), OR
/// * the `Cargo.toml` has no `[package.metadata.nros]` table, OR
/// * the table is present but declares only `application` / `entry` /
///   `deploy` (not a component package).
///
/// Parse errors are propagated so a malformed `Cargo.toml` surfaces at
/// discovery time rather than silently dropping the package.
fn discover_cargo_component_metadata(
    root: &Path,
    pkg_xml_name: &str,
) -> Result<Vec<CargoComponentSummary>> {
    let cargo_toml = root.join("Cargo.toml");
    if !cargo_toml.is_file() {
        return Ok(Vec::new());
    }
    let raw = fs::read_to_string(&cargo_toml)
        .wrap_err_with(|| format!("failed to read {}", cargo_toml.display()))?;
    let envelope: CargoManifestEnvelope = toml::from_str(&raw)
        .wrap_err_with(|| format!("failed to parse {} for nros metadata", cargo_toml.display()))?;
    let Some(package) = envelope.package else {
        return Ok(Vec::new());
    };
    // M-F.17 fix: `pkg_name` drives the synthetic artifact's `package`
    // field which must match what `<node pkg="…"/>` references — that's
    // the package.xml `<name>`, NOT the Cargo.toml `[package].name`
    // (which is the crate name, often suffixed `_component` / `_pkg`).
    let pkg_name = pkg_xml_name.to_string();
    // phase-307 W1: the crate name is NOT diagnostics-only any more — the
    // metadata harness path-depends on this crate and names its type through
    // it, so a `talker_pkg` / `talker-pkg-component` divergence is load-bearing.
    let crate_name = package.name.replace('-', "_");
    let Some(metadata) = package.metadata else {
        return Ok(Vec::new());
    };
    let Some(nros) = metadata.nros else {
        return Ok(Vec::new());
    };
    // Mirrors `nros_config::normalise_node_alias`: `node` is the post-N.12
    // canonical spelling, `component` is the deprecated alias. We accept
    // both at discovery time without warning (warnings live in
    // `parse_package_metadata_nros`).
    // phase-308: a package that declares an ENTRY as well as a node is the
    // self-contained standalone shape (issue 0100) — deploy-bound by
    // definition, so not host-probeable.
    //
    // issue 0318 — `[deploy.<target>]` says the same thing, and 27 standalone
    // examples (freertos / nuttx / threadx-linux / zephyr) spell it that way
    // instead of with an `[entry]` table. Keying on `entry` alone let them
    // through to the host probe, which cannot compile them: they dep a board
    // crate directly (`nros-board-mps2-an385-freertos`) or, on zephyr, the
    // `zephyr` crate whose build script needs `DOTCONFIG` from a cmake
    // configure that has not run yet. Both spellings mean deploy-bound.
    //
    // Safe against over-triggering: no colcon-workspace Node pkg (`src/<pkg>`,
    // the probeable shape) carries a `[deploy.*]` table — deploy lives on the
    // Entry pkg there. Verified across the tree when this landed.
    let deploy_bound = nros.deploy_bound();
    let single = nros.node.as_ref().or(nros.component.as_ref());
    let multi: Vec<(String, &ComponentMetadata)> = if !nros.nodes.is_empty() {
        nros.nodes.iter().map(|(k, v)| (k.clone(), v)).collect()
    } else {
        nros.components
            .iter()
            .map(|(k, v)| (k.clone(), v))
            .collect()
    };

    let bins: Vec<String> = envelope
        .bin
        .iter()
        .flat_map(|b| b.iter())
        .filter_map(|b| b.name.clone())
        .collect();

    let mut out = Vec::new();
    if let Some(component) = single {
        out.push(synthesise_summary(
            &pkg_name,
            &crate_name,
            deploy_bound,
            None,
            component,
            &bins,
            &cargo_toml,
        ));
    }
    for (key, component) in multi {
        out.push(synthesise_summary(
            &pkg_name,
            &crate_name,
            deploy_bound,
            Some(&key),
            component,
            &bins,
            &cargo_toml,
        ));
    }
    Ok(out)
}

/// Build a [`CargoComponentSummary`] for one `[component]` / `[node]`
/// (single) or `[components.<Name>]` / `[nodes.<Name>]` (multi) entry.
///
/// Per the Phase 212.M-F.17 task spec:
///
/// * `metadata.name` wins as the component name when present.
/// * Otherwise the multi-shape key wins (`components.Talker` → `Talker`).
/// * Otherwise the class basename wins (`talker_pkg::Talker` → `Talker`).
/// * Otherwise the package name is used as a last-resort fallback.
///
/// `executable` defaults to the package name; if a `[[bin]] name = …`
/// matches the chosen component name, that bin name wins instead.
fn synthesise_summary(
    pkg_name: &str,
    crate_name: &str,
    deploy_bound: bool,
    multi_key: Option<&str>,
    component: &ComponentMetadata,
    bins: &[String],
    manifest_path: &Path,
) -> CargoComponentSummary {
    let component_name = component
        .name
        .clone()
        .or_else(|| multi_key.map(ToString::to_string))
        .or_else(|| {
            component
                .class
                .as_deref()
                .and_then(class_basename)
                .map(ToString::to_string)
        })
        .unwrap_or_else(|| pkg_name.to_string());

    // `[[bin]] name = …` override: when one of the workspace member
    // `[[bin]]` rows happens to share the component name, prefer it
    // (the planner expects `executable` to point at the actual binary
    // a build tree drops on disk).
    // M-F.17 fix: launch XML `<node exec="…"/>` references the component
    // name (e.g. `talker`), not the cargo pkg / crate name (e.g.
    // `talker_pkg_component`). When a `[[bin]] name = component_name`
    // exists, that's the executable — clean Application-pkg case. For a
    // staticlib Component pkg (no `[[bin]]`), the executable IS the
    // component name (it's the symbolic identity the launcher resolves;
    // the actual binary is the Entry pkg that links the component in).
    let executable = if bins.iter().any(|n| n == &component_name) || bins.is_empty() {
        // `[[bin]] name = component_name` (clean Application pkg) OR a
        // staticlib Component pkg with no `[[bin]]` — both cases the
        // executable IS the component name.
        component_name.clone()
    } else {
        pkg_name.to_string()
    };

    CargoComponentSummary {
        package: pkg_name.to_string(),
        crate_name: crate_name.to_string(),
        deploy_bound,
        component: component_name,
        executable,
        class: component.class.clone(),
        default_namespace: component.default_namespace.clone(),
        manifest_path: manifest_path.to_path_buf(),
        publishes: component.publishes.clone(),
        subscribes: component.subscribes.clone(),
    }
}

/// `talker_pkg::Talker` → `Some("Talker")`. Returns `None` when the class
/// string carries no `::` separator (a malformed value the lint catches
/// elsewhere).
fn class_basename(class: &str) -> Option<&str> {
    class.rsplit_once("::").map(|(_, tail)| tail)
}

/// Permissive `Cargo.toml` envelope — only the keys M-F.17 cares about
/// are typed; every sibling table (`[dependencies]`, `[lib]`, …) is
/// ignored. Strictness on the nros tables themselves comes from
/// [`PackageMetadataNros`]'s `deny_unknown_fields`.
#[derive(Debug, Deserialize)]
struct CargoManifestEnvelope {
    #[serde(default)]
    package: Option<CargoPackageEnvelope>,
    #[serde(default)]
    bin: Option<Vec<CargoBinEnvelope>>,
}

#[derive(Debug, Deserialize)]
struct CargoPackageEnvelope {
    name: String,
    #[serde(default)]
    metadata: Option<CargoPackageMetadataEnvelope>,
}

#[derive(Debug, Deserialize)]
struct CargoPackageMetadataEnvelope {
    #[serde(default)]
    nros: Option<PackageMetadataNros>,
}

#[derive(Debug, Deserialize)]
struct CargoBinEnvelope {
    #[serde(default)]
    name: Option<String>,
}

/// Build the synthetic JSON object the planner consumes for one
/// summary. Mirrors the keys [`super::planner::schema_components`]
/// + [`super::planner::find_source_metadata`] read:
///
/// * `package` / `component` / `executable` — `(package, executable)`
///   match + `package::component` dedup id.
/// * `language` — every Cargo-resident component is Rust today; the
///   field is required so `schema_components` doesn't fall through to
///   the `"rust"` literal default.
/// * `synthetic` / `synthetic_source` — provenance markers; downstream
///   `nros check` lints distinguish synthesised entries from
///   authoritative metadata.
fn summary_to_synthetic_json(summary: &CargoComponentSummary) -> JsonValue {
    let mut obj = json!({
        "version": 1,
        "package": summary.package,
        "component": summary.component,
        "executable": summary.executable,
        "language": "rust",
        "synthetic": true,
        "synthetic_source": "cargo_metadata",
    });
    let map = obj.as_object_mut().expect("synthetic JSON is an object");
    if let Some(class) = &summary.class {
        map.insert("class".to_string(), JsonValue::String(class.clone()));
    }
    if let Some(namespace) = &summary.default_namespace {
        map.insert(
            "default_namespace".to_string(),
            JsonValue::String(namespace.clone()),
        );
    }
    // phase-267 W1c/C3 — emit the declared topic endpoints as the flat
    // `publishers` / `subscribers` arrays the planner's `collect_entity_array`
    // reads (same shape as a post-build sidecar). A `[[bridge]]`'s topic NAMES
    // then resolve to types via `resolve_topic_interface` with NO build.
    let topic_entities =
        |decls: &[super::cargo_metadata_schema::TopicDecl], prefix: &str| -> JsonValue {
            let arr: Vec<JsonValue> = decls
                .iter()
                .map(|d| {
                    // ROS type `"pkg/msg/Name"` (or `"pkg/Name"`) → {package, name}:
                    // package = first segment, name = LAST segment (drop the
                    // `msg`/`srv`/`action` namespace middle). `split_once` alone
                    // left `name = "msg/Name"` for the 3-part form → mis-resolve.
                    let segs: Vec<&str> = d.type_name.split('/').collect();
                    let (package, name) = match segs.as_slice() {
                        [pkg, .., n] => (*pkg, *n),
                        [n] => ("", *n),
                        _ => ("", d.type_name.as_str()),
                    };
                    let id = format!(
                        "{prefix}_{}",
                        d.topic.trim_start_matches('/').replace('/', "_")
                    );
                    json!({
                        "id": id,
                        "topic": d.topic,
                        "type": { "package": package, "name": name, "kind": "message" },
                    })
                })
                .collect();
            JsonValue::Array(arr)
        };
    if !summary.publishes.is_empty() {
        map.insert(
            "publishers".to_string(),
            topic_entities(&summary.publishes, "pub"),
        );
    }
    if !summary.subscribes.is_empty() {
        map.insert(
            "subscribers".to_string(),
            topic_entities(&summary.subscribes, "sub"),
        );
    }
    obj
}

fn cmake_summary_to_synthetic_json(summary: &CmakeNodeSummary) -> JsonValue {
    let language = match &summary.language {
        ComponentLanguage::C => "c",
        ComponentLanguage::Cpp => "cpp",
        ComponentLanguage::Rust => "rust",
    };
    let mut obj = json!({
        "version": 1,
        "package": summary.package,
        "component": summary.component,
        "executable": summary.executable,
        "language": language,
        "synthetic": true,
        "synthetic_source": "cmake_node_register",
    });
    if let Some(class) = &summary.class {
        obj.as_object_mut()
            .expect("synthetic JSON is an object")
            .insert("class".to_string(), JsonValue::String(class.clone()));
    }
    obj
}

/// Locate component declaration candidates. Preference order (W.1 fold):
/// the package's `nros.toml` (read for a `[component]` table — the canonical
/// folded form), then the deprecated standalone `component_nros.toml` at the
/// package root, then any `nros/components/*.toml`. Whether a candidate is
/// actually a component is decided at parse time (`load_component_config`
/// returns `None` for an `nros.toml` with no `[component]`).
fn discover_component_configs(root: &Path) -> Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    // W.1 fold: a package `nros.toml` may carry a `[component]` table.
    let folded = root.join("nros.toml");
    if folded.is_file() {
        out.push(folded);
    }
    let primary = root.join("component_nros.toml");
    if primary.is_file() {
        out.push(primary);
    }
    // The multi-component glob is order-independent — sort it for determinism,
    // but keep it *after* the root candidates so the folded `nros.toml` and the
    // legacy `component_nros.toml` retain their preference order.
    let components_dir = root.join("nros").join("components");
    if components_dir.is_dir() {
        let mut globbed = Vec::new();
        for entry in fs::read_dir(&components_dir)? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "toml") {
                globbed.push(path);
            }
        }
        globbed.sort();
        out.extend(globbed);
    }
    Ok(out)
}

fn collect_files(root: &Path, dirs: &[&str], suffixes: &[&str]) -> Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    for dir in dirs {
        let path = root.join(dir);
        if path.is_dir() {
            collect_matching(&path, suffixes, &mut out)?;
        }
    }
    out.sort();
    Ok(out)
}

fn collect_matching(dir: &Path, suffixes: &[&str], out: &mut Vec<PathBuf>) -> Result<()> {
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            collect_matching(&path, suffixes, out)?;
        } else if suffixes.iter().any(|suffix| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.ends_with(suffix))
        }) {
            out.push(path);
        }
    }
    Ok(())
}

pub fn unique_paths<I>(paths: I) -> Vec<PathBuf>
where
    I: IntoIterator<Item = PathBuf>,
{
    let mut seen = BTreeSet::new();
    paths
        .into_iter()
        .filter(|path| seen.insert(path.clone()))
        .collect()
}

#[cfg(test)]
mod probe_target_tests {
    //! phase-308 W1 — what the C/C++ metadata probe needs off a declaration.
    use super::*;

    fn decl(
        class: Option<&str>,
        header: Option<&str>,
        shape: Option<&str>,
    ) -> ComponentDeclaration {
        ComponentDeclaration {
            package_root: PathBuf::from("/ws/src/talker_pkg"),
            manifest_path: PathBuf::from("/ws/src/talker_pkg/CMakeLists.txt"),
            class: class.map(ToString::to_string),
            crate_name: None,
            deploy_bound: false,
            header: header.map(ToString::to_string),
            shape: shape.map(ToString::to_string),
            library_target: None,
            config: ComponentConfig {
                version: 1,
                package: "talker_pkg".into(),
                component: "talker".into(),
                class: None,
                language: ComponentLanguage::Cpp,
                linkage: ComponentLinkage::default(),
                metadata: ComponentMetadataConfig {
                    source_metadata: "metadata/talker.json".into(),
                    generated_by: None,
                },
                overrides: ComponentOverrides::default(),
            },
        }
    }

    /// The in-tree convention every C/C++ example ships.
    #[test]
    fn header_derives_from_the_class_when_undeclared() {
        assert_eq!(
            decl(Some("talker_pkg::Talker"), None, None).probe_header(),
            Some("talker_pkg/Talker.hpp".to_string())
        );
    }

    /// An explicit `HEADER` on the cmake verb always wins — the whole reason
    /// the parser captures it instead of skipping it.
    #[test]
    fn declared_header_wins_over_the_convention() {
        assert_eq!(
            decl(Some("talker_pkg::Talker"), Some("custom/Path.hpp"), None).probe_header(),
            Some("custom/Path.hpp".to_string())
        );
    }

    /// A nested namespace still maps to the package include root.
    #[test]
    fn nested_namespaces_use_the_outermost_segment() {
        assert_eq!(
            decl(Some("my_pkg::inner::Node"), None, None).probe_header(),
            Some("my_pkg/Node.hpp".to_string())
        );
    }

    /// No class ⇒ no derivable header. The caller must report that it cannot
    /// generate a probe, NOT guess a path that fails to compile.
    #[test]
    fn no_class_yields_no_header() {
        assert_eq!(decl(None, None, None).probe_header(), None);
    }

    /// Defaults match the cmake verb's own default.
    #[test]
    fn shape_defaults_to_rclcpp() {
        assert_eq!(decl(Some("p::C"), None, None).probe_shape(), "rclcpp");
        assert_eq!(
            decl(Some("p::C"), None, Some("configure")).probe_shape(),
            "configure"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// issue 0318 — `[deploy.<target>]` marks a package deploy-bound just as
    /// `[entry]` does. Keying only on `entry` let 27 standalone examples
    /// (freertos / nuttx / threadx-linux / zephyr) reach the host metadata
    /// probe, which cannot compile them: they dep a board crate directly, or
    /// on zephyr the `zephyr` crate whose build script needs `DOTCONFIG` from
    /// a cmake configure that has not run yet.
    fn summary_for(manifest: &str) -> Vec<CargoComponentSummary> {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("Cargo.toml"), manifest).unwrap();
        discover_cargo_component_metadata(dir.path(), "demo_pkg").unwrap()
    }

    const NODE_TABLE: &str = r#"
[package]
name = "demo_pkg"
version = "0.1.0"

[package.metadata.nros.node]
class = "demo_pkg::Demo"
name = "demo"
"#;

    #[test]
    fn deploy_table_alone_marks_the_package_deploy_bound() {
        let manifest = format!(
            "{NODE_TABLE}\n[package.metadata.nros.deploy.zephyr]\nboard = \"native_sim/native/64\"\n"
        );
        let got = summary_for(&manifest);
        assert_eq!(got.len(), 1, "expected one component summary");
        assert!(
            got[0].deploy_bound,
            "a package carrying [deploy.<target>] is bound to that target and is \
             NOT host-probeable, exactly like one carrying [entry]"
        );
    }

    /// The over-trigger guard. A colcon-workspace Node pkg declares a node and
    /// NO deploy target (deploy lives on the Entry pkg), so it stays probeable
    /// — losing that would silently drop exact executor sizing, which is the
    /// regression issue 0288 warns about.
    #[test]
    fn a_plain_node_package_stays_probeable() {
        let got = summary_for(NODE_TABLE);
        assert_eq!(got.len(), 1, "expected one component summary");
        assert!(
            !got[0].deploy_bound,
            "a node pkg with no deploy target must remain host-probeable"
        );
    }

    #[test]
    fn entry_table_still_marks_the_package_deploy_bound() {
        let manifest =
            format!("{NODE_TABLE}\n[package.metadata.nros.entry]\ndeploy = \"zephyr\"\n");
        let got = summary_for(&manifest);
        assert_eq!(got.len(), 1, "expected one component summary");
        assert!(
            got[0].deploy_bound,
            "the pre-0318 [entry] behaviour must be preserved"
        );
    }

    /// phase-267 W1c/C3 — a node's declared `publishes` lower into the SYNTHETIC
    /// metadata as a flat `publishers` array in the exact shape the planner's
    /// `collect_entity_array` reads (`{ id, topic, type: {package, name, kind} }`),
    /// so a bridge's topic name resolves to its type with NO build.
    #[test]
    fn synthetic_json_emits_declared_topic_entities() {
        use super::super::cargo_metadata_schema::TopicDecl;
        let summary = CargoComponentSummary {
            package: "talker_pkg".into(),
            crate_name: "talker_pkg".into(),
            deploy_bound: false,
            component: "talker".into(),
            executable: "talker".into(),
            class: Some("talker_pkg::Talker".into()),
            default_namespace: None,
            manifest_path: PathBuf::from("/ws/talker_pkg/Cargo.toml"),
            publishes: vec![TopicDecl {
                topic: "/chatter".into(),
                type_name: "std_msgs/msg/Int32".into(),
            }],
            subscribes: vec![],
        };
        let json = summary_to_synthetic_json(&summary);
        let pubs = json
            .get("publishers")
            .and_then(JsonValue::as_array)
            .expect("publishers array");
        assert_eq!(pubs.len(), 1);
        assert_eq!(pubs[0]["topic"], "/chatter");
        assert_eq!(pubs[0]["type"]["package"], "std_msgs");
        // Issue 0099 — 3-part `pkg/msg/Name` drops the `msg` namespace middle
        // (was wrongly `"msg/Int32"`, which mis-resolved bridge topic types).
        assert_eq!(pubs[0]["type"]["name"], "Int32");
        assert_eq!(pubs[0]["type"]["kind"], "message");
        // No subscribers declared → key omitted (keeps non-declaring nodes
        // byte-identical to pre-C3 synthetic JSON).
        assert!(json.get("subscribers").is_none());
    }

    /// RAII scratch directory under the system temp dir (no `tempfile` dep).
    struct Scratch(PathBuf);
    impl Scratch {
        fn new(tag: &str) -> Self {
            let dir = crate::test_support::scratch_dir(&format!("nros-ws-test-{tag}"));
            Scratch(dir)
        }
        fn write(&self, rel: &str, body: &str) {
            let path = self.0.join(rel);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, body).unwrap();
        }
    }
    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    const PKG_XML: &str = r#"<?xml version="1.0"?>
<package format="3"><name>demo_pkg</name><version>0.0.0</version>
<description>t</description><maintainer email="a@b.c">a</maintainer><license>MIT</license>
</package>"#;

    const COMPONENT_TABLE: &str = r#"
        [component]
        version = 1
        package = "demo_pkg"
        component = "talker"
        language = "rust"
        [component.linkage]
        crate_name = "demo_pkg"
        executable = "talker"
        [component.metadata]
        source_metadata = "target/nros/metadata/talker.json"
    "#;

    // The same declaration in the legacy standalone (whole-file) shape.
    const LEGACY_WHOLE_FILE: &str = r#"
        version = 1
        package = "demo_pkg"
        component = "talker"
        language = "rust"
        [linkage]
        crate_name = "demo_pkg"
        executable = "talker"
        [metadata]
        source_metadata = "target/nros/metadata/talker.json"
    "#;

    #[test]
    fn folds_component_table_in_package_nros_toml() {
        let s = Scratch::new("fold");
        s.write("src/demo_pkg/package.xml", PKG_XML);
        // A package nros.toml carrying [workspace]-unrelated sibling tables plus
        // the folded [component] — sibling tables must be ignored.
        s.write(
            "src/demo_pkg/nros.toml",
            &format!("[[transport]]\nid = \"t\"\nkind = \"udp\"\n{COMPONENT_TABLE}"),
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let decls = ws.component_declarations().unwrap();
        assert_eq!(decls.len(), 1, "folded [component] is discovered");
        assert_eq!(decls[0].config.package, "demo_pkg");
        assert_eq!(decls[0].config.component, "talker");
        assert!(decls[0].manifest_path.ends_with("nros.toml"));
    }

    #[test]
    fn legacy_component_nros_toml_still_discovered() {
        let s = Scratch::new("legacy");
        s.write("src/demo_pkg/package.xml", PKG_XML);
        s.write("src/demo_pkg/component_nros.toml", LEGACY_WHOLE_FILE);

        let decls = Workspace::discover(&s.0)
            .unwrap()
            .component_declarations()
            .unwrap();
        assert_eq!(decls.len(), 1);
        assert_eq!(decls[0].config.component, "talker");
        assert!(decls[0].manifest_path.ends_with("component_nros.toml"));
    }

    #[test]
    fn folded_nros_toml_wins_over_legacy_for_same_component() {
        let s = Scratch::new("both");
        s.write("src/demo_pkg/package.xml", PKG_XML);
        s.write("src/demo_pkg/nros.toml", COMPONENT_TABLE);
        s.write("src/demo_pkg/component_nros.toml", LEGACY_WHOLE_FILE);

        let decls = Workspace::discover(&s.0)
            .unwrap()
            .component_declarations()
            .unwrap();
        assert_eq!(decls.len(), 1, "duplicate (package, component) deduped");
        assert!(
            decls[0].manifest_path.ends_with("nros.toml")
                && !decls[0].manifest_path.ends_with("component_nros.toml"),
            "folded form wins: {}",
            decls[0].manifest_path.display()
        );
    }

    // -----------------------------------------------------------------
    // Phase 212.M-F.17 — synthetic metadata from Cargo.toml
    // -----------------------------------------------------------------

    /// `[package.metadata.nros.component]` single-shape → one summary.
    /// Component name defaults to class basename when `metadata.name`
    /// is absent; executable defaults to package name.
    #[test]
    fn synthetic_metadata_from_single_component_table() {
        let s = Scratch::new("mf17-single");
        s.write(
            "src/talker_pkg/package.xml",
            PKG_XML.replace("demo_pkg", "talker_pkg").as_str(),
        );
        s.write(
            "src/talker_pkg/Cargo.toml",
            r#"
[package]
name = "talker_pkg"
version = "0.1.0"
edition = "2021"

[package.metadata.nros.component]
class = "talker_pkg::Talker"
default_namespace = "/demo"
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let pkg = ws
            .packages
            .iter()
            .find(|p| p.name == "talker_pkg")
            .expect("pkg");
        assert_eq!(pkg.cargo_component_metadata.len(), 1, "one summary");
        let summary = &pkg.cargo_component_metadata[0];
        assert_eq!(summary.package, "talker_pkg");
        // `metadata.name` absent → class basename wins.
        assert_eq!(summary.component, "Talker");
        // No `[[bin]]` row → executable is the component name (the
        // symbolic identity the launch XML `<node exec="…"/>` references).
        // M-F.17 fix-up: previously fell back to pkg_name which broke
        // staticlib Component pkgs whose crate name != component name.
        assert_eq!(summary.executable, "Talker");
        assert_eq!(summary.class.as_deref(), Some("talker_pkg::Talker"));
        assert_eq!(summary.default_namespace.as_deref(), Some("/demo"));
        assert!(summary.manifest_path.ends_with("Cargo.toml"));

        let synth = ws.synthetic_metadata_artifacts();
        assert_eq!(synth.len(), 1);
        let (path, value) = &synth[0];
        assert!(path.ends_with("Cargo.toml"));
        assert_eq!(value["version"], 1);
        assert_eq!(value["package"], "talker_pkg");
        assert_eq!(value["component"], "Talker");
        assert_eq!(value["executable"], "Talker");
        assert_eq!(value["language"], "rust");
        assert_eq!(value["class"], "talker_pkg::Talker");
        assert_eq!(value["default_namespace"], "/demo");
        assert_eq!(value["synthetic"], true);
        assert_eq!(value["synthetic_source"], "cargo_metadata");
    }

    /// `metadata.name` wins over class basename when both are present;
    /// a `[[bin]] name = …` row matching the component name overrides
    /// the package-name executable default.
    #[test]
    fn synthetic_metadata_name_and_bin_override() {
        let s = Scratch::new("mf17-name-bin");
        s.write(
            "src/talker_pkg/package.xml",
            PKG_XML.replace("demo_pkg", "talker_pkg").as_str(),
        );
        s.write(
            "src/talker_pkg/Cargo.toml",
            r#"
[package]
name = "talker_pkg"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "talker"
path = "src/bin/talker.rs"

[package.metadata.nros.component]
class = "talker_pkg::Talker"
name = "talker"
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let pkg = ws
            .packages
            .iter()
            .find(|p| p.name == "talker_pkg")
            .expect("pkg");
        let summary = &pkg.cargo_component_metadata[0];
        // `metadata.name = "talker"` wins.
        assert_eq!(summary.component, "talker");
        // `[[bin]] name = "talker"` matches the component name → wins.
        assert_eq!(summary.executable, "talker");
    }

    /// `[package.metadata.nros.components.<Name>]` multi-shape →
    /// one summary per entry; the table key wins as the component
    /// name when `metadata.name` is absent on the entry.
    #[test]
    fn synthetic_metadata_from_multi_components_table() {
        let s = Scratch::new("mf17-multi");
        s.write(
            "src/multi_pkg/package.xml",
            PKG_XML.replace("demo_pkg", "multi_pkg").as_str(),
        );
        s.write(
            "src/multi_pkg/Cargo.toml",
            r#"
[package]
name = "multi_pkg"
version = "0.1.0"
edition = "2021"

[package.metadata.nros.components.Talker]
class = "multi_pkg::Talker"

[package.metadata.nros.components.Listener]
class = "multi_pkg::Listener"
default_namespace = "/multi"
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let pkg = ws
            .packages
            .iter()
            .find(|p| p.name == "multi_pkg")
            .expect("pkg");
        assert_eq!(pkg.cargo_component_metadata.len(), 2);
        // BTreeMap iteration order is key-sorted: Listener < Talker.
        let listener = &pkg.cargo_component_metadata[0];
        assert_eq!(listener.component, "Listener");
        assert_eq!(listener.default_namespace.as_deref(), Some("/multi"));
        let talker = &pkg.cargo_component_metadata[1];
        assert_eq!(talker.component, "Talker");
        assert!(talker.default_namespace.is_none());

        let synth = ws.synthetic_metadata_artifacts();
        assert_eq!(synth.len(), 2);
    }

    /// Package with no `Cargo.toml` (e.g. a CMake / Zephyr-only
    /// component) → empty summary list, no error.
    #[test]
    fn synthetic_metadata_no_cargo_toml() {
        let s = Scratch::new("mf17-no-cargo");
        s.write(
            "src/cmake_pkg/package.xml",
            PKG_XML.replace("demo_pkg", "cmake_pkg").as_str(),
        );
        // No Cargo.toml.

        let ws = Workspace::discover(&s.0).unwrap();
        let pkg = ws
            .packages
            .iter()
            .find(|p| p.name == "cmake_pkg")
            .expect("pkg");
        assert!(pkg.cargo_component_metadata.is_empty());
        assert!(ws.synthetic_metadata_artifacts().is_empty());
    }

    /// Cargo.toml present but with no `[package.metadata.nros]` table →
    /// empty summary list (e.g. a regular Rust library that happens to
    /// sit next to a `package.xml`).
    #[test]
    fn synthetic_metadata_no_nros_table() {
        let s = Scratch::new("mf17-no-nros");
        s.write(
            "src/plain_pkg/package.xml",
            PKG_XML.replace("demo_pkg", "plain_pkg").as_str(),
        );
        s.write(
            "src/plain_pkg/Cargo.toml",
            r#"
[package]
name = "plain_pkg"
version = "0.1.0"
edition = "2021"

[dependencies]
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let pkg = ws
            .packages
            .iter()
            .find(|p| p.name == "plain_pkg")
            .expect("pkg");
        assert!(pkg.cargo_component_metadata.is_empty());
        assert!(ws.synthetic_metadata_artifacts().is_empty());
    }

    /// `node` (post-N.12 canonical key) is treated the same as
    /// `component` (deprecated alias). The discovery path is
    /// warning-free (the warning lives in `nros_config`).
    #[test]
    fn synthetic_metadata_accepts_node_alias() {
        let s = Scratch::new("mf17-node");
        s.write(
            "src/node_pkg/package.xml",
            PKG_XML.replace("demo_pkg", "node_pkg").as_str(),
        );
        s.write(
            "src/node_pkg/Cargo.toml",
            r#"
[package]
name = "node_pkg"
version = "0.1.0"
edition = "2021"

[package.metadata.nros.node]
class = "node_pkg::Node"
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let pkg = ws
            .packages
            .iter()
            .find(|p| p.name == "node_pkg")
            .expect("pkg");
        assert_eq!(pkg.cargo_component_metadata.len(), 1);
        assert_eq!(pkg.cargo_component_metadata[0].component, "Node");
    }

    #[test]
    fn root_only_nros_toml_is_not_a_component() {
        let s = Scratch::new("rootonly");
        s.write("src/demo_pkg/package.xml", PKG_XML);
        // A workspace-root / direct-mode nros.toml with no [component] table.
        s.write(
            "src/demo_pkg/nros.toml",
            "[workspace]\ndefault = \"x\"\n[node]\nname = \"n\"\n",
        );

        let decls = Workspace::discover(&s.0)
            .unwrap()
            .component_declarations()
            .unwrap();
        assert!(decls.is_empty(), "no [component] table → not a component");
    }

    // -----------------------------------------------------------------
    // Phase 219.L — `nano_ros_node_register` discovery from CMakeLists.
    // -----------------------------------------------------------------

    #[test]
    fn strip_cmake_comments_preserves_lines() {
        let src = "foo # bar\nbaz\n";
        let out = strip_cmake_comments(src);
        assert_eq!(out, "foo      \nbaz\n");
    }

    #[test]
    fn extract_call_finds_register_body() {
        let src = "project(p)\nnano_ros_node_register(\n    NAME talker\n    CLASS p::T)\n";
        let calls = extract_cmake_calls(src, "nano_ros_node_register");
        assert_eq!(calls.len(), 1);
        assert!(calls[0].contains("NAME talker"));
        assert!(calls[0].contains("CLASS p::T"));
    }

    #[test]
    fn extract_call_skips_identifier_prefix() {
        // `my_nano_ros_node_register` must NOT match.
        let src = "my_nano_ros_node_register(...)\nnano_ros_node_register(NAME real)\n";
        let calls = extract_cmake_calls(src, "nano_ros_node_register");
        assert_eq!(calls.len(), 1);
        assert!(calls[0].contains("NAME real"));
    }

    #[test]
    fn parse_kwargs_extracts_name_class_deploy() {
        let body = r#"
    NAME    talker
    CLASS   talker_pkg::Talker
    SOURCES src/Talker.cpp src/Helper.cpp
    DEPLOY  native freertos
"#;
        let kw = parse_cmake_kwargs(body).unwrap();
        assert_eq!(kw.single("NAME").as_deref(), Some("talker"));
        assert_eq!(kw.single("CLASS").as_deref(), Some("talker_pkg::Talker"));
        assert_eq!(kw.multi("DEPLOY"), vec!["native", "freertos"]);
        assert_eq!(
            kw.multi("SOURCES"),
            vec!["src/Talker.cpp", "src/Helper.cpp"]
        );
    }

    #[test]
    fn parse_kwargs_strips_quoted_strings() {
        let body = r#" NAME "my talker" CLASS "ns::Class" "#;
        let kw = parse_cmake_kwargs(body).unwrap();
        assert_eq!(kw.single("NAME").as_deref(), Some("my talker"));
        assert_eq!(kw.single("CLASS").as_deref(), Some("ns::Class"));
    }

    #[test]
    fn cmake_only_node_pkg_yields_component_declaration() {
        let s = Scratch::new("219l");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/CMakeLists.txt",
            r#"project(talker_pkg)
nano_ros_node_register(
    NAME    talker
    CLASS   talker_pkg::Talker
    SOURCES src/Talker.cpp
    DEPLOY  native)
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        assert_eq!(ws.packages.len(), 1);
        assert_eq!(ws.packages[0].cmake_component_metadata.len(), 1);
        let summary = &ws.packages[0].cmake_component_metadata[0];
        assert_eq!(summary.package, "talker_pkg");
        assert_eq!(summary.component, "talker");
        assert_eq!(summary.class.as_deref(), Some("talker_pkg::Talker"));
        assert_eq!(summary.language, ComponentLanguage::Cpp);
        assert_eq!(summary.deploy_targets, vec!["native"]);

        let synth = ws.synthetic_metadata_artifacts();
        assert_eq!(synth.len(), 1);
        assert_eq!(synth[0].1["package"], "talker_pkg");
        assert_eq!(synth[0].1["component"], "talker");
        assert_eq!(synth[0].1["executable"], "talker");
        assert_eq!(synth[0].1["language"], "cpp");
        assert_eq!(synth[0].1["synthetic_source"], "cmake_node_register");

        let decls = ws.component_declarations().unwrap();
        assert_eq!(decls.len(), 1);
        let cfg = &decls[0].config;
        assert_eq!(cfg.package, "talker_pkg");
        assert_eq!(cfg.component, "talker");
        assert_eq!(cfg.language, ComponentLanguage::Cpp);
    }

    /// Verifies CMake declarations are skipped when nros.toml declares the same pair.
    #[test]
    fn cmake_decls_skip_duplicate_pair() {
        // Explicit `nros.toml` `[component]` table wins over cmake parse.
        let s = Scratch::new("219l");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/nros.toml",
            r#"[component]
version = 1
package = "talker_pkg"
component = "talker"
language = "cpp"
[component.metadata]
source_metadata = "metadata/talker.json"
"#,
        );
        s.write(
            "src/talker_pkg/CMakeLists.txt",
            r#"nano_ros_node_register(NAME talker CLASS talker_pkg::Talker DEPLOY native)
"#,
        );
        let ws = Workspace::discover(&s.0).unwrap();
        let decls = ws.component_declarations().unwrap();
        assert_eq!(decls.len(), 1, "explicit nros.toml wins");
        assert_eq!(decls[0].manifest_path.file_name().unwrap(), "nros.toml");
    }

    #[test]
    fn cmake_decl_language_default_is_cpp_for_pkg_class_qualname() {
        let s = Scratch::new("219l");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/CMakeLists.txt",
            "nano_ros_node_register(NAME t CLASS p::C DEPLOY native)\n",
        );
        let decls = Workspace::discover(&s.0)
            .unwrap()
            .component_declarations()
            .unwrap();
        assert_eq!(decls[0].config.language, ComponentLanguage::Cpp);
    }

    #[test]
    fn cmake_decl_language_c_when_class_has_no_qualname() {
        let s = Scratch::new("219l");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/CMakeLists.txt",
            "nano_ros_node_register(NAME t CLASS register_t DEPLOY native)\n",
        );
        let decls = Workspace::discover(&s.0)
            .unwrap()
            .component_declarations()
            .unwrap();
        assert_eq!(decls[0].config.language, ComponentLanguage::C);
    }

    #[test]
    fn cmake_decl_language_c_when_language_keyword_is_c() {
        let s = Scratch::new("223c");
        s.write(
            "src/c_talker_pkg/package.xml",
            "<package format=\"3\"><name>c_talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/c_talker_pkg/CMakeLists.txt",
            r#"nano_ros_node_register(
    NAME talker
    CLASS c_talker_pkg::Talker
    LANGUAGE C
    SOURCES src/Talker.c
    DEPLOY native)
"#,
        );
        let ws = Workspace::discover(&s.0).unwrap();
        let summary = &ws.packages[0].cmake_component_metadata[0];
        assert_eq!(summary.language, ComponentLanguage::C);

        let synth = ws.synthetic_metadata_artifacts();
        assert_eq!(synth.len(), 1);
        assert_eq!(synth[0].1["package"], "c_talker_pkg");
        assert_eq!(synth[0].1["component"], "talker");
        assert_eq!(synth[0].1["executable"], "talker");
        assert_eq!(synth[0].1["language"], "c");
        assert_eq!(synth[0].1["class"], "c_talker_pkg::Talker");
    }

    /// Issue 0099 — a node's declared `publishes` (Cargo `[[package.metadata.
    /// nros.node.publishes]]`) becomes a synthetic `publishers` entity, and a
    /// 3-part ROS type `pkg/msg/Name` splits to `{package: pkg, name: Name}`
    /// (NOT `name: "msg/Name"` — the bug that mis-resolved bridge topics).
    #[test]
    fn synthetic_publisher_type_drops_msg_namespace() {
        let s = Scratch::new("0099pub");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/Cargo.toml",
            r#"[package]
name = "talker_pkg"
version = "0.1.0"
edition = "2024"

[lib]

[package.metadata.nros.node]
class = "talker_pkg::Talker"
name = "talker"

[[package.metadata.nros.node.publishes]]
topic = "/chatter"
type = "std_msgs/msg/Int32"
"#,
        );

        let ws = Workspace::discover(&s.0).unwrap();
        let synth = ws.synthetic_metadata_artifacts();
        assert_eq!(synth.len(), 1);
        let pubs = synth[0].1["publishers"]
            .as_array()
            .expect("publishers array");
        assert_eq!(pubs.len(), 1);
        assert_eq!(pubs[0]["topic"], "/chatter");
        assert_eq!(pubs[0]["type"]["package"], "std_msgs");
        assert_eq!(pubs[0]["type"]["name"], "Int32");
        assert_eq!(pubs[0]["type"]["kind"], "message");
    }

    #[test]
    fn cmake_decl_skipped_when_name_missing() {
        let s = Scratch::new("219l");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/CMakeLists.txt",
            "nano_ros_node_register(CLASS p::T DEPLOY native)\n",
        );
        let ws = Workspace::discover(&s.0).unwrap();
        assert!(ws.packages[0].cmake_component_metadata.is_empty());
    }

    // -----------------------------------------------------------------
    // RFC-0048 (phase-287 W6) — `nano_ros_add_node` (positional) discovery.
    // -----------------------------------------------------------------

    #[test]
    fn parse_add_node_positional_name_class_sources() {
        let body = "talker CLASS c_talker_pkg::Talker TYPED src/Talker.c";
        let s = parse_add_node_call(body, "c_talker_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(s.component, "talker");
        assert_eq!(s.executable, "talker");
        assert_eq!(s.class.as_deref(), Some("c_talker_pkg::Talker"));
        // Language from the `.c` source, NOT the `::`-bearing class.
        assert_eq!(s.language, ComponentLanguage::C);
    }

    #[test]
    fn parse_add_node_cpp_source_and_deploy() {
        let body = "listener CLASS ws::Listener src/Listener.cpp DEPLOY native";
        let s = parse_add_node_call(body, "cpp_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(s.component, "listener");
        assert_eq!(s.language, ComponentLanguage::Cpp);
        assert_eq!(s.deploy_targets, vec!["native".to_string()]);
    }

    #[test]
    fn add_node_unexpandable_sources_fall_back_to_the_class_not_to_c() {
        // Issue 1062, and the exact ASI declaration that found it. cmake expands
        // `${_controller_sources}` to `.cpp` paths before inferring; this scanner
        // reads CMakeLists as text and sees one token with no extension. The old
        // `else C` called that C, and the C++ component was then probed against
        // `__nros_c_component_*` — symbols `NROS_COMPONENT` never emits.
        let body = "controller CLASS controller_pkg::Controller SHAPE rclcpp \
                    SOURCES ${_controller_sources}";
        let s = parse_add_node_call(body, "controller_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(
            s.language,
            ComponentLanguage::Cpp,
            "an unreadable source list is not evidence of C"
        );
    }

    #[test]
    fn add_node_language_keyword_beats_the_source_extensions() {
        // The keyword exists because it is the one thing cmake and this scanner
        // are guaranteed to read identically. It therefore has to win even when
        // the extensions disagree — otherwise it cannot fix the case it is for.
        let body = "shim CLASS shim_pkg::Shim LANGUAGE C SOURCES src/shim.cpp";
        let s = parse_add_node_call(body, "shim_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(s.language, ComponentLanguage::C);
    }

    #[test]
    fn add_node_language_keyword_is_not_swallowed_as_a_source() {
        // Before the keyword existed, `LANGUAGE CPP` was two positional tokens:
        // cmake put them in UNPARSED_ARGUMENTS and appended both to SOURCES.
        // The parser has to consume the value, not file it under sources.
        let body = "listener CLASS ws::Listener LANGUAGE CPP src/Listener.cpp DEPLOY native";
        let s = parse_add_node_call(body, "cpp_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(s.language, ComponentLanguage::Cpp);
        assert_eq!(s.component, "listener");
        assert_eq!(s.deploy_targets, vec!["native".to_string()]);
    }

    #[test]
    fn add_node_literal_c_sources_still_read_as_c() {
        // The fallback must not swallow the case it replaced: a real `.c` list
        // is evidence, and the `::`-bearing class must not override it.
        let body = "talker CLASS c_talker_pkg::Talker TYPED SOURCES src/Talker.c src/util.c";
        let s = parse_add_node_call(body, "c_talker_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(s.language, ComponentLanguage::C);
    }

    #[test]
    fn add_node_library_target_is_the_component_lib_not_the_node_name() {
        // Issue 0939. NanoRosNodeRegister.cmake builds
        // `${PROJECT_NAME}_${NAME}_component`; the bare node name is not a
        // target. Getting this wrong is not a build error here — it surfaces
        // one directory away, as the metadata probe failing to link
        // `-l<name>`, which `nros sync` then reports as "no producer for
        // <pkg>::<node>" and caches as "probe failed last sync".
        let body = "controller CLASS controller_pkg::Controller TYPED src/controller.cpp";
        let s = parse_add_node_call(body, "controller_pkg", Path::new("CMakeLists.txt")).unwrap();
        assert_eq!(
            s.library_target.as_deref(),
            Some("controller_pkg_controller_component"),
            "the probe links this name; the bare node name does not exist as a target"
        );
    }

    #[test]
    fn node_register_library_target_defaults_to_the_component_lib() {
        // Same rule on the all-keyword verb. `TARGET` still wins when given,
        // but nothing in-tree passes it, so the default is what always applies.
        let body = "NAME listener CLASS ws::Listener LANGUAGE cpp SOURCES src/Listener.cpp";
        let calls = parse_cmake_kwargs(body).unwrap();
        let name = calls.single("NAME").unwrap();
        assert_eq!(
            component_library_target("cpp_pkg", &name),
            "cpp_pkg_listener_component"
        );
    }

    #[test]
    fn discover_finds_add_node_from_cmakelists() {
        let s = Scratch::new("addnode");
        s.write(
            "src/talker_pkg/package.xml",
            "<package format=\"3\"><name>talker_pkg</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/talker_pkg/CMakeLists.txt",
            "find_package(nano_ros REQUIRED)\n\
             nano_ros_add_node(talker CLASS talker_pkg::Talker TYPED src/Talker.c)\n",
        );
        let ws = Workspace::discover(&s.0).unwrap();
        let meta = &ws.packages[0].cmake_component_metadata;
        assert_eq!(meta.len(), 1);
        assert_eq!(meta[0].component, "talker");
        assert_eq!(meta[0].class.as_deref(), Some("talker_pkg::Talker"));
    }

    #[test]
    fn discover_finds_rfc0057_split_spelling() {
        // RFC-0057 (phase-305): nano_ros_auto_add_library +
        // nros_components_register_node with a NESTED upstream namespace —
        // the planner must see the component (language from the lib sources).
        let s = Scratch::new("rfc0057");
        s.write(
            "src/autoware_mrm_handler/package.xml",
            "<package format=\"3\"><name>autoware_mrm_handler</name><version>0.1.0</version>\
             <description>t</description><maintainer email=\"x@x\">x</maintainer>\
             <license>Apache-2.0</license></package>",
        );
        s.write(
            "src/autoware_mrm_handler/CMakeLists.txt",
            "find_package(nano_ros REQUIRED)\n\
             nano_ros_auto_add_library(mrm_handler_lib STATIC src/mrm_handler_core.cpp)\n\
             nros_components_register_node(mrm_handler_lib\n\
                 PLUGIN autoware::mrm_handler::MrmHandler\n\
                 EXECUTABLE mrm_handler\n\
                 HEADER autoware/mrm_handler/mrm_handler_core.hpp)\n",
        );
        let ws = Workspace::discover(&s.0).unwrap();
        let meta = &ws.packages[0].cmake_component_metadata;
        assert_eq!(meta.len(), 1);
        assert_eq!(meta[0].component, "mrm_handler");
        assert_eq!(
            meta[0].class.as_deref(),
            Some("autoware::mrm_handler::MrmHandler")
        );
        assert!(matches!(meta[0].language, ComponentLanguage::Cpp));
    }
}
