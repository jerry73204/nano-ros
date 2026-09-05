//! Phase 212.B — `[workspace.metadata.nros]` + `[package.metadata.nros]` +
//! `[package.metadata.ament]` + `<bringup>/system.toml` data contracts.
//!
//! These are the *user-authored* TOML surfaces introduced by Phase 212. They
//! live in standard cargo manifest tables (`[workspace.metadata.…]` /
//! `[package.metadata.…]`) so that cargo treats them as opaque user data and
//! pure-cargo workflows (no CLI build wrapper) keep working. The
//! `<bringup>/system.toml` is the per-system declarative file owned by the
//! `<system>_bringup` package.
//!
//! Vocabulary discipline (per the Phase 212 doc): every field name is a strict
//! subset of names that already appear in `nros-sdk-index.toml`,
//! `app_config.h`, or the existing planner schema. No second TOML dialect.
//!
//! Every struct here uses `#[serde(deny_unknown_fields)]` so typos surface as
//! parse errors at the user's terminal instead of being silently dropped.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

// Phase 228.E — the per-tier schema types live in the shared
// `nros-orchestration-ir` crate (single source of truth, also consumed by the
// `nros::main!()` proc-macro). Re-exported here so the rest of the CLI keeps
// referencing them through `cargo_metadata_schema::` unchanged.
pub use nros_orchestration_ir::{
    CallbackGroupDecl, CallbackGroupOverride, NodeOverride, TierDef, TierRtosSpec,
};

/// RFC-0078 — re-exported so a `system.toml` reader sees one vocabulary.
pub use nros_orchestration_ir::wcet::{BoundaryWcet, WcetError, WcetProfile};

use super::schema::RemapRule;

// ---------------------------------------------------------------------------
// Workspace-root metadata: `[workspace.metadata.nros]`
// ---------------------------------------------------------------------------

/// `[workspace.metadata.nros]` in a workspace-root `Cargo.toml`.
///
/// Thin pointer (see `docs/design/0024-multi-node-workspace-layout.md` §5). The
/// authoritative system spec lives in `<bringup>/system.toml`; this table
/// only disambiguates which bringup the workspace defaults to plus a small
/// set of rarely-used workspace-wide overrides.
///
/// Per the Phase 212.L.7 redesign, `default_system` may name EITHER a
/// bringup package (`<system>_bringup`) OR a Node/Entry pkg that eats its
/// own Entry role (single-pkg `cargo run` dev loop). The launcher walks
/// the workspace and resolves the pointer against either category.
///
/// All fields are optional; an absent `[workspace.metadata.nros]` table
/// parses as `WorkspaceMetadataNros::default()`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorkspaceMetadataNros {
    /// Bringup package name (`<system>_bringup`) OR Entry pkg name
    /// (Phase 212.L.7 self-entry shape). `nros plan` /
    /// `nros codegen-system` with no `--bringup` hint resolves the
    /// system via this pointer.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_system: Option<String>,
    /// Optional workspace-wide RMW override — rare, intended for
    /// `nros plan --override` workflows. Values are `"zenoh"` /
    /// `"xrce"` / `"cyclonedds"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw_override: Option<String>,
    /// Optional workspace-wide `ROS_DOMAIN_ID` override. When present,
    /// `nros plan` / `nros codegen-system` propagate it into the
    /// generated `system_config.h` instead of the per-deploy /
    /// `[system].domain_id` value. Rare — used for one-off bring-ups
    /// against a shared ROS 2 graph.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_id_override: Option<u32>,
}

// ---------------------------------------------------------------------------
// Per-package metadata: `[package.metadata.nros]`
// ---------------------------------------------------------------------------

/// `[package.metadata.nros]` in a component / application package's
/// `Cargo.toml`.
///
/// Three top-level shapes (mutually exclusive):
///
/// * Single-node crate — `[package.metadata.nros.node]` describes the
///   one node the crate exposes. (The Phase 212.N.12 rename made
///   `node` the canonical key; `[package.metadata.nros.component]`
///   remains accepted as a deprecated alias — declaring both is a
///   hard error. See `nros_config::parse_package_metadata_nros`.)
/// * Multi-component crate — `[package.metadata.nros.components.<Name>]`
///   table-of-tables enumerates each.
/// * Application crate — `[package.metadata.nros.application]` describes a
///   native-only application pkg (per Phase 212.L.2).
///
/// Phase 212.L.7 also adds an optional per-target deploy table at
/// `[package.metadata.nros.deploy.<target>]`, used both by application pkgs
/// and by self-bringup component pkgs (component pkg w/ `[deploy.*]` and no
/// sibling bringup acts as its own bringup).
///
/// At most one of `component` / `components` / `application` may be present;
/// the loader validates this after deserialization (a serde untagged enum
/// would lose the precise `deny_unknown_fields` error, so we keep the fields
/// flat and reject conflicts in [`PackageMetadataNros::validate`]).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageMetadataNros {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub component: Option<ComponentMetadata>,
    /// Phase 212.N.12 (in-flight) — `node` is the forward-looking spelling of
    /// the `component` shape. The reader accepts BOTH spellings during the
    /// in-flight rename (Phase 212.B.2 task spec). Mutually exclusive with
    /// `component` / `components` / `nodes` / `application` (validated below).
    /// The shape is identical to [`ComponentMetadata`] so codegen can treat
    /// the two interchangeably until N.12 retires the `component` spelling.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node: Option<ComponentMetadata>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub components: BTreeMap<String, ComponentMetadata>,
    /// Phase 212.N.12 in-flight — `nodes` is the forward-looking spelling
    /// of the `components` (multi-shape) table. Same shape, accepted as an
    /// alias during the rename. Mutually exclusive with `components`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub nodes: BTreeMap<String, ComponentMetadata>,
    /// Phase 212.L.2 — `[package.metadata.nros.application]`. Application
    /// pkgs are native-only orchestration roots; they MUST NOT name an RTOS
    /// in their `deploy = […]` allow-list. (The `nros check` lint enforces
    /// the no-RTOS rule; the schema only accepts the field.)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub application: Option<ApplicationMetadata>,
    /// Phase 212.N.7 — `[package.metadata.nros.entry]`. Entry pkgs declare
    /// which `[deploy.<target>]` block they run on (the firmware bin pulls
    /// in the per-board shim + emits `run_plan(runtime)`). Strict schema —
    /// `deploy = "<board>"` is the only field today.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entry: Option<EntryMetadata>,
    /// Phase 212.L.7 / L.8 — per-target deploy tables, keyed by target name
    /// (`native`, `qemu-mps2-an385`, …). Populates both application pkgs and
    /// self-bringup component pkgs (component pkg w/ `[deploy.*]` and no
    /// sibling bringup eats its own bringup role).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub deploy: BTreeMap<String, DeployTargetMetadata>,
    /// Phase 212.B.2 stub (`[package.metadata.nros.domain]`) — opaque
    /// pass-through during the schema in-flight window. The full typed shape
    /// lands with system.toml's F.4 work. Captured as `toml::Value` so
    /// `deny_unknown_fields` still surfaces typos elsewhere while letting
    /// users author the table.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain: Option<toml::Value>,
    /// Phase 212.B.2 stub (`[package.metadata.nros.bridge]`) — opaque
    /// pass-through, same rationale as `domain`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bridge: Option<toml::Value>,
    /// Phase 212.B.2 stub (`[package.metadata.nros.embedded]`) — opaque
    /// pass-through. Will eventually hold board-specific embedded knobs
    /// (`linker_script` / `stack_size` / …); kept opaque so the reader
    /// doesn't break the moment a board author authors the table.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub embedded: Option<toml::Value>,
    /// Issue #391 / phase-333 — generated message crates carry a constant
    /// cargo `version = "0.0.0"`; the UPSTREAM interface version
    /// (`4.9.2`, …) moved here. Emitted by the msg codegen into every
    /// generated crate's manifest; this reader must accept it or any
    /// workspace containing a generated tree fails metadata parsing
    /// (codegen-system died on `unknown field ament_version` — the emitter
    /// landed without extending this schema).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ament_version: Option<String>,
}

impl PackageMetadataNros {
    /// Is this package bound to a deploy target, i.e. NOT host-compilable?
    ///
    /// issue 0358 — two manifest spellings say this, and every consumer had to
    /// remember both:
    ///
    /// ```toml
    /// [package.metadata.nros.entry]           # 24 standalone examples
    /// deploy = "native"
    ///
    /// [package.metadata.nros.deploy.zephyr]   # 27 standalone examples
    /// board = "native_sim/native/64"
    /// ```
    ///
    /// The source-metadata probe remembered only the first, so 27 packages fell
    /// through to a host build they cannot survive — they dep a board crate
    /// directly, or on zephyr the `zephyr` crate whose build script wants
    /// `DOTCONFIG` from a cmake configure that has not run. That surfaced as
    /// `DOTCONFIG must be set by wrapper` on a Zephyr leaf (issue 0318), which
    /// is several layers from a predicate that forgot half its input.
    ///
    /// Being one function is the point: the next consumer asks the question
    /// instead of re-deriving the answer, and a third spelling is one edit here
    /// rather than a hunt through call sites.
    pub fn deploy_bound(&self) -> bool {
        self.entry.is_some() || !self.deploy.is_empty()
    }

    /// Reject manifests that combine more than one of `component` /
    /// `node` / `components` / `nodes` / `application` — the shapes are
    /// mutually exclusive per the Phase 212.L design doc plus the Phase
    /// 212.N.12 rename (the `node` / `nodes` spellings are aliases of
    /// `component` / `components`, not new categories).
    pub fn validate(&self) -> Result<(), String> {
        let has_component = self.component.is_some();
        let has_node = self.node.is_some();
        let has_components = !self.components.is_empty();
        let has_nodes = !self.nodes.is_empty();
        let has_application = self.application.is_some();
        let count = [
            has_component,
            has_node,
            has_components,
            has_nodes,
            has_application,
        ]
        .into_iter()
        .filter(|b| *b)
        .count();
        if count > 1 {
            return Err("`[package.metadata.nros]` carries more than one of \
                 `component` / `node` / `components` / `nodes` / `application` — pick exactly \
                 one shape (Phase 212.L.2 / L.7; N.12 rename in flight — `node` / \
                 `nodes` are the forward-looking spellings of `component` / `components`)"
                .to_string());
        }
        Ok(())
    }

    /// Phase 212.N.12 in-flight rename — `component` and `node` are aliases.
    /// Returns the present shape, preferring `node` (the forward spelling)
    /// over `component`. Callers reading the per-pkg shape go through this
    /// accessor so the N.12 sweep can later flip the storage field without
    /// touching every read site.
    pub fn node_or_component(&self) -> Option<&ComponentMetadata> {
        self.node.as_ref().or(self.component.as_ref())
    }

    /// Phase 212.N.12 in-flight rename — multi-shape accessor. Returns
    /// `nodes` when present, else `components`. Empty if neither populated.
    pub fn nodes_or_components(&self) -> &BTreeMap<String, ComponentMetadata> {
        if !self.nodes.is_empty() {
            &self.nodes
        } else {
            &self.components
        }
    }

    /// True when this manifest is a *self-bringup-eligible* component or
    /// application pkg (Phase 212.L.7): it declares its component/application
    /// surface AND at least one `[package.metadata.nros.deploy.<target>]`
    /// table. The planner / codegen path treats such a pkg as its own
    /// degenerate 1-component bringup when no sibling bringup pkg points at
    /// it.
    pub fn is_self_bringup_eligible(&self) -> bool {
        let has_role = self.component.is_some()
            || self.node.is_some()
            || !self.components.is_empty()
            || !self.nodes.is_empty()
            || self.application.is_some();
        has_role && !self.deploy.is_empty()
    }
}

/// `[package.metadata.nros.entry]` — Phase 212.N.7.
///
/// Marks an Entry pkg (the firmware bin) so the planner can route it to the
/// right `[deploy.<target>]` block. Today the only field is `deploy =
/// "<board>"` (the deploy-target key in the workspace deploy map). The
/// reader keeps this strict so a typo on `deploy =` surfaces immediately.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EntryMetadata {
    /// Board / deploy-target key (e.g. `"freertos"`, `"zephyr"`).
    pub deploy: String,
    /// phase-271 (issue #110) — per-entry executor callback-table size. The
    /// `nros::main!` macro reads this to open the executor sized to the entry's
    /// OWN declared topology (`Executor::open_sized`) instead of the
    /// workspace-global `NROS_EXECUTOR_MAX_CBS`. Absent → build default. The CLI
    /// only needs to ACCEPT the key here (so `deny_unknown_fields` doesn't reject
    /// the entry); the macro is what consumes it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_callbacks: Option<usize>,
    /// Companion to [`max_callbacks`](Self::max_callbacks) — scheduling-context
    /// slots. Absent / `0` → build default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_sched_contexts: Option<usize>,
    /// Node packages this entry links and boots. Consumed by the `nros::main!`
    /// bake; accepted here so discovery does not reject the entry (issue 0100's
    /// self-contained standalone examples all declare it).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub node_pkgs: Vec<String>,
}

/// `[package.metadata.nros.node]` (single shape, canonical post Phase
/// 212.N.12) — or `[package.metadata.nros.component]` (deprecated alias)
/// — or `[package.metadata.nros.components.<Name>]` (multi shape).
///
/// Pure deployment intent — no build-system knobs (Cargo + CMake own those).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentMetadata {
    /// Phase 212.L.4 — fully-qualified class name (`<pkg-dir>::<UserClass>`).
    /// Lint-enforced by `nros check` to match the host pkg name; codegen
    /// uses it to land at the right Rust type.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub class: Option<String>,
    /// Short component instance name (used as the planner / codegen
    /// instance identifier when the pkg is its own self-bringup, per
    /// Phase 212.L.7).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Default namespace the component is mounted at. Absent → `/`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_namespace: Option<String>,
    /// Phase 216.A.5 — `"inline" | "deferred" | "from_isr"`.
    ///
    /// Accepted here but VALIDATED in `cmd::check_workspace`, which owns the
    /// (framework × strategy) matrix and its own parse of this same table.
    /// Two parsers, one TOML table: this field existing only in the other one
    /// meant `Workspace::discover` returned a hard "unknown field `dispatch`"
    /// error for every standalone example that declares it — 15 of them,
    /// across stm32f4 / esp32 / nuttx — so `nros metadata` and every other
    /// `Workspace::discover` caller simply could not read those packages.
    /// phase-307 W5's coverage gate is what surfaced it. Keep the two parsers
    /// in sync; the gate re-checks the whole tree.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dispatch: Option<String>,
    /// Raw ROS parameter declarations. Values stay as `toml::Value` here so
    /// the planner can do its own type-aware lowering (mirrors the existing
    /// `params::ParameterTable` resolution path).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub parameters: BTreeMap<String, toml::Value>,
    /// `from` → `to` topic / service remaps, mirroring rclpy / rclcpp.
    /// Aliased to [`RemapRule`] (already `{from, to}`-shaped in
    /// `super::schema`) to avoid creating a duplicate type.
    ///
    /// TODO(issue 0255): component-default remaps are parsed but not yet
    /// routed into entry codegen — phase-306 W3 wired the launch/model remap
    /// inputs (`NodeSpec.remaps` / `NodeInstance.remaps`) into
    /// `PlanNode.remaps`; threading THIS metadata source through the planner
    /// into the same field (launch rules win over component defaults) is the
    /// remaining input.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub remaps: Vec<RemapRule>,
    /// Phase 228.A (RFC-0015) — callback groups the node declares (it owns its
    /// callbacks). Each names a symbolic `tier` the system's `[tiers.*]` maps to
    /// an RTOS task/priority. Empty → all callbacks default to the `"default"`
    /// tier (the single-task degenerate case).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub callback_groups: Vec<CallbackGroupDecl>,
    /// phase-267 W1c/C3a — topics this node PUBLISHES, declared by the node
    /// author (`[[package.metadata.nros.node.publishes]] topic=… type=…`). Read
    /// as SYNTHETIC metadata (pre-build, no sidecar) so the planner can resolve a
    /// `[[bridge]]`'s topic NAMES to their ROS types without building. Empty ⇒ the
    /// node declares no publishers in metadata (entity resolution falls back to
    /// the post-build sidecar, as before).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub publishes: Vec<TopicDecl>,
    /// phase-267 W1c/C3a — topics this node SUBSCRIBES to. Same shape + role as
    /// [`publishes`](Self::publishes).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub subscribes: Vec<TopicDecl>,
    /// Issue 1061 — every entity this component creates, in the
    /// `nano_ros_node_register(... ENTITIES ...)` grammar:
    /// `["publisher:std_msgs/msg/String:/chatter", "timer", "sub*2"]`.
    ///
    /// # Why this is not `publishes` + `subscribes`
    ///
    /// Those two answer a DATA-FLOW question for the planner — which topics a
    /// bridge can resolve pre-build. This answers a SLOT-DEMAND question: how
    /// many callback entries, subscriber rings and queryables the image must
    /// reserve. Timers, service servers and actions cost slots and carry no
    /// topic, so they cannot be expressed there, and `MAX_CBS` /
    /// `MAX_QUERYABLES` cannot be derived without them.
    ///
    /// # Why it exists at all
    ///
    /// The metadata PROBE normally answers this by compiling the component for
    /// the host and reading what it declares. A leaf that sets
    /// `[unstable] build-std` for a foreign target, or depends on a board crate
    /// with no host build, cannot be probed — its metadata is
    /// `<component>.json.unprobeable` — and before this it had no way to say
    /// what it creates, so its pools kept the crate defaults. On a board where
    /// `.bss` is subtracted from the stack that is not a footprint nicety
    /// (issue 1052).
    ///
    /// # It does not become a way to disagree with the code
    ///
    /// Where the probe CAN run, `leaf_entity_env::reconcile` compares this list
    /// against it per kind and REFUSES on a mismatch. The declaration is for
    /// leaves with nothing to check it against; it is not an override.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub entities: Vec<String>,
}

/// phase-267 W1c/C3a — one declared topic endpoint in node Cargo metadata:
/// `{ topic = "/chatter", type = "std_msgs/Int32" }`. The `type` is the ROS type
/// name (`<pkg>/<Msg>` or `<pkg>/msg/<Msg>`); the planner resolves a bridge's
/// topic name to it pre-build.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopicDecl {
    pub topic: String,
    #[serde(rename = "type")]
    pub type_name: String,
}

/// `[package.metadata.nros.application]` — Phase 212.L.2.
///
/// Application pkgs are native-only orchestration roots: they wire several
/// component pkgs together but MUST NOT name an RTOS target in their
/// `deploy = […]` allow-list. The allow-list semantics are enforced by
/// `nros check`; the schema only accepts the field.
///
/// The application's per-target deploy block lives on the outer
/// `[package.metadata.nros.deploy.<target>]` (shared with self-bringup
/// component pkgs), not nested under `[application]`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplicationMetadata {
    /// Allow-list of deploy targets — keys into the outer
    /// `[package.metadata.nros.deploy.<target>]` map. Must not include an
    /// RTOS target name (lint-enforced by `nros check`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub deploy: Vec<String>,
    /// Optional short app name; falls back to the Cargo `[package].name`
    /// when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

/// `[package.metadata.nros.deploy.<target>]` — Phase 212.L.8.
///
/// Per-target deploy parameters baked into the bringup tree by `nros
/// codegen-system` (and recorded into `nros-plan.json` by `nros plan`).
/// Used by application pkgs and by self-bringup component pkgs.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeployTargetMetadata {
    /// Board identifier (e.g. `native_sim/native/64`, `mps2-an385`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub board: Option<String>,
    /// RMW backend (`zenoh` / `xrce` / `cyclonedds`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw: Option<String>,
    /// Serialization format, by PROVIDER NAME (phase-421 W4, RFC-0088 D6).
    ///
    /// The Cargo-native deploy block is the one `[deploy.<t>]` nano-ros OWNS.
    /// The `system.toml` one is `ros_launch_manifest_model::DeployBlock`, an
    /// upstream `deny_unknown_fields` struct that declares `rmw` and not
    /// `serdes` — so the system.toml rung of RFC-0088 D6's
    /// `[deploy.<t>] serdes = "…"` needs a spec release, and the live home for
    /// the key is `[image.<id>].serdes` (where `rmw` itself moved, RFC-0065 D6).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub serdes: Option<String>,
    /// Baked ROS_DOMAIN_ID — embedded targets bake at build time.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_id: Option<u32>,
    /// Optional RMW locator URI (e.g. `tcp/127.0.0.1:7447`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
    /// Static network overlay the `nros::main!` bake lowers into a
    /// `DeployOverlay` (`main_macro::DeployOverlayLit`), overlaying the board's
    /// compiled-in `Config::default()`. Dotted IPv4 strings.
    ///
    /// Same two-parser drift as [`ComponentMetadata::dispatch`]: the macro read
    /// these keys while this envelope rejected them, so `Workspace::discover`
    /// hard-failed on every board example that declares a static IP — stm32f4,
    /// esp32, nuttx. phase-307 W5's coverage gate surfaced it. Parsed here for
    /// discovery; the macro remains the consumer.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gateway: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub netmask: Option<String>,
    /// Transport selector (`udp` / `tcp` / `serial`), consumed by the bake.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub transport: Option<String>,
    /// Rust target triple the deploy builds for (`armv7a-nuttx-eabihf`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
}

/// Convenience alias: spec calls these `RemapEntry`. The existing
/// `RemapRule` already has the right shape, so we expose both names.
pub type RemapEntry = RemapRule;

// ---------------------------------------------------------------------------
// Per-package metadata: `[package.metadata.ament]`
// ---------------------------------------------------------------------------

/// `[package.metadata.ament]` — the source of truth for `nros emit
/// package-xml` (Phase 212.G). Mirrors ament/colcon's `package.xml`
/// vocabulary 1-to-1.
///
/// Vocabulary (Phase 212.B.4):
///
/// * `description` / `license` — passthrough to `<description>` /
///   `<license>` in the emitted `package.xml`. When absent the emitter
///   falls back to a synthesised description + `"Apache-2.0"`.
/// * `maintainer = { name, email }` — populates the single
///   `<maintainer email="…">…</maintainer>` row. Multiple maintainers
///   are not modelled yet — ROS allows several `<maintainer>` rows,
///   but every in-tree fixture authors at most one.
/// * `build_depend` / `exec_depend` / `test_depend` /
///   `buildtool_depend` — each row emits a corresponding `<*_depend>`
///   entry. Sorted + deduped at emit time so list ordering doesn't
///   drift between edits.
/// * `build_type` — `<export><build_type>…</build_type></export>`.
///   Defaults differ between component pkgs (`ament_cargo`) and
///   bringup pkgs (`ament_cmake`).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageMetadataAment {
    /// `<description>` body. Absent → emitter synthesises a generic
    /// "`nano-ros component package <name>`" line.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// `<license>` body (SPDX identifier). Absent → `"Apache-2.0"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub license: Option<String>,
    /// `<maintainer email="…">name</maintainer>` row. Absent →
    /// emitter falls back to a placeholder `Developer <dev@example.com>`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub maintainer: Option<AmentMaintainer>,
    /// `<build_depend>` rows.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub build_depend: Vec<String>,
    /// `<buildtool_depend>` rows (e.g. `"ament_cargo"`, `"ament_cmake"`).
    /// Phase 212.B.4: explicit dependency category so users opting in
    /// to colcon interop can author `buildtool_depend = ["ament_cmake"]`
    /// without polluting `<build_depend>`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub buildtool_depend: Vec<String>,
    /// `<exec_depend>` rows.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exec_depend: Vec<String>,
    /// `<test_depend>` rows.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub test_depend: Vec<String>,
    /// `<export><build_type>…</build_type></export>` body. Component
    /// pkgs default to `"ament_cargo"`; bringup pkgs default to
    /// `"ament_cmake"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build_type: Option<String>,
}

/// `maintainer = { name = "…", email = "…" }` — Phase 212.B.4.
///
/// Modelled as a strict struct so a stray `affiliation = "…"` or
/// `github = "…"` typo surfaces as `unknown field` at parse time.
/// Both `name` and `email` are mandatory when the table is present;
/// `package.xml` requires the email attribute and a non-empty body,
/// so we mirror that policy verbatim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AmentMaintainer {
    pub name: String,
    pub email: String,
}

// ---------------------------------------------------------------------------
// Per-bringup file: `<bringup>/system.toml`
// ---------------------------------------------------------------------------

/// `<bringup>/system.toml` — the authoritative system spec.
///
/// Sections (see `docs/design/0025-workspace-layout-by-case.md` Case 3/4 and
/// `0024-multi-node-workspace-layout.md` §4):
///
/// * `[system]` — name, RMW, domain, optional locator.
/// * `[[component]]` — one entry per node/component.
/// * `[deploy.<target>]` — per-target deploy block (`kind = "self" | "qemu"
///   | "flash" | …`).
/// * `[[domain]]` — optional per-system domain routing.
/// * `[[bridge]]` — optional cross-RMW bridges.
/// * `[[model]]` — phase-330 W4.0: a model this bringup produces from a launch
///   file plus argument bindings (see [`SystemModelEntry`]).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemToml {
    pub system: SystemHeader,
    #[serde(default, rename = "component", skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<SystemComponentEntry>,
    /// `[deploy.<target>]` — keyed by target name (e.g. `native`,
    /// `qemu-mps2-an385`, `flash-stm32f4-disco`).
    ///
    /// **PLACEMENT ONLY** since phase-383 W1 (RFC-0065 D6). This is upstream's
    /// [`DeployTarget`] (`ros_launch_manifest_model`'s `DeployBlock`) and it
    /// answers "which nodes run where". What to COMPILE lives in [`Self::image`].
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub deploy: BTreeMap<String, DeployTarget>,
    /// `[host.<name>]` — the machines, and the only thing that partitions
    /// nodes (issue 0951).
    ///
    /// The placement half of `[deploy.*]`, on its own terms. A host says WHERE
    /// a node runs; `[image.*]` says WHAT gets built. `[deploy.*]` conflated
    /// the two, which is why rlm's resolver had to filter `kind = "embedded"`
    /// blocks out of placement to work at all.
    ///
    /// Re-exported from rlm like `DeployTarget`, so there is one definition of
    /// the schema rather than a mirror that drifts — the lesson `DeployBlock`'s
    /// own history records.
    #[serde(default)]
    pub host: BTreeMap<String, HostTarget>,
    /// `[image.<id>]` — the buildable images (RFC-0065 D6, phase-383 W1).
    ///
    /// A NEW nano-ros-owned table, not a rename of [`Self::deploy`]: that one
    /// is upstream's `deny_unknown_fields` type, and the two are independent
    /// (an embedded image is no placement; a machine running stock ROS 2 nodes
    /// is no image). See [`super::image`] for the full argument.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub image: BTreeMap<String, ImageBlock>,
    /// `[image_defaults]` — keys shared by every `[image.<id>]` (RFC-0065 D5.1).
    ///
    /// **Why a separate table rather than a bare `[image]` header.** TOML would
    /// happily accept `[image]` carrying both scalars and `[image.<id>]`
    /// sub-tables, but serde cannot model that alongside `deny_unknown_fields`
    /// — `flatten` and `deny_unknown_fields` do not compose — and dropping the
    /// deny is not on the table: a silently ignored `board` key is a build for
    /// the wrong target that reports success. PlatformIO avoids the same
    /// collision with `[env]` / `[env:NAME]`, where `:` keeps the namespaces
    /// apart; `_defaults` is the same trick in TOML's grammar.
    ///
    /// Note this is only for keys the system header does not already default
    /// (`profile`, `variant`, `conf`, `panic`). `rmw` and `ros_edition` fall
    /// back to [`SystemHeader`] as they always have.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub image_defaults: Option<ImageBlock>,
    /// `[board_config.<board>]` — the SITE half of a board (RFC-0072 §5,
    /// issue 0951).
    ///
    /// Keyed by BOARD, not by image or deploy name, because that is what the
    /// fact is about. It was authored as `[deploy.<name>.nros]` until the 30
    /// blocks in this tree were measured: they held exactly THREE distinct
    /// value-sets over five boards, and the 25 duplicates existed only because
    /// the deploy key was sometimes the friendly name (`freertos`) and
    /// sometimes the board spelling (`mps2-an385-freertos`) — two keys for one
    /// board, which is two places for one fact to drift.
    ///
    /// Keying by board also makes "two blocks disagree about one board"
    /// unrepresentable rather than merely detected: `board_facts` used to
    /// compare candidates and refuse a conflict, and that check now has
    /// nothing to catch.
    ///
    /// The key is matched through the same `BoardCatalog::resolve_deploy`
    /// rule every other board spelling uses (issue 0606), so an alias resolves
    /// here exactly as it does everywhere else.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub board_config: BTreeMap<String, toml::Value>,
    #[serde(default, rename = "domain", skip_serializing_if = "Vec::is_empty")]
    pub domains: Vec<SystemDomainEntry>,
    #[serde(default, rename = "bridge", skip_serializing_if = "Vec::is_empty")]
    pub bridges: Vec<SystemBridgeEntry>,
    /// phase-330 W4.0 (RFC-0063) — `[[model]]`: a SystemModel this bringup
    /// produces from a launch file plus ARGUMENT BINDINGS.
    ///
    /// Plain models need no entry: `nros sync` derives one per launch file that
    /// is neither the default nor `<include>`d by another. Bindings are the
    /// case that cannot be derived — `multihost.launch.xml host:=robot1` is a
    /// different system from `host:=robot2`, and nothing in the launch tree
    /// says which bindings matter. That fact used to live only in the committed
    /// model's own `meta.args`, i.e. inside the artifact the model is supposed
    /// to be regenerated from; declaring it here moves it into the inputs.
    #[serde(default, rename = "model", skip_serializing_if = "Vec::is_empty")]
    pub models: Vec<SystemModelEntry>,
    /// Phase 228.A (RFC-0015 §4.2) — `[tiers.<name>]` priority/scheduling tiers.
    /// The system owner maps the symbolic tier names a node's callback groups
    /// reference to per-RTOS task knobs. Empty → the single-tier degenerate case.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub tiers: BTreeMap<String, TierDef>,
    /// Phase 228.A — `[[node_overrides]]` deployment-time tier reassignment of a
    /// node's callback groups (RFC-0015 §4.2), without touching the node package.
    #[serde(
        default,
        rename = "node_overrides",
        skip_serializing_if = "Vec::is_empty"
    )]
    pub node_overrides: Vec<NodeOverride>,
    /// RFC-0078 — `[wcet]`: declared execution-time bounds, keyed per named
    /// measurement profile, plus which profile each deploy target selects.
    ///
    /// Selection lives HERE rather than in `[deploy.<target>]` because
    /// `DeployTarget` is a re-export of rlm's `DeployBlock` — an upstream type
    /// with `deny_unknown_fields`, so a field added there would have to land in
    /// another repository first. Keying the selection map by target name keeps
    /// "a board selects a profile" true without that dependency.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wcet: Option<SystemWcet>,
    /// Phase 254 — declared capability axes (RFC-0031 §Generalization), the
    /// single typed home read by BOTH codegen paths (the Rust planner + the
    /// C/C++ bake). Supersedes the transitional per-package `nros.toml`
    /// capability overlays. `[safety]` — E2E message-integrity (CRC).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub safety: Option<SystemSafety>,
    /// Phase 254 — `[param_services]`: the external ROS 2 parameter server.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub param_services: Option<SystemParamServices>,
    /// Phase 256 Wave 1 — `[lifecycle]`: the managed-node boot autostart state,
    /// the typed home superseding the per-package `nros.toml` `[lifecycle]`
    /// overlay. Read by the planner (`PlanLifecycle`). Absent ⇒ no lifecycle
    /// block (node stays a plain node).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lifecycle: Option<SystemLifecycle>,
    // `[param_persistence]` has no typed field, so `deny_unknown_fields` REJECTS
    // such a block. Not "until the backends land" — issue 0080 ruled on-device
    // parameter persistence a NON-GOAL (2026-07-10; launch-baked defaults are
    // the model), and phase-359 W10 deleted the dormant seam it was waiting on.
    // The rejection is now permanent rather than provisional.
}

/// `[wcet]` — RFC-0078 declared execution-time bounds.
///
/// Two halves: the profiles themselves, and which one each deploy target
/// selects. A target with no entry gets NO bounds, which is the default and is
/// not an error — absent stays representable all the way up.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemWcet {
    /// `[wcet.profiles.<name>]`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub profiles: BTreeMap<String, WcetProfile>,
    /// `[wcet.select]` — deploy-target name to profile name.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub select: BTreeMap<String, String>,
}

/// Why a `[wcet]` section could not be resolved for a target.
///
/// Both variants are HARD errors rather than a silent `None`. A selection that
/// quietly resolves to nothing puts the scheduling model straight back to
/// counting every boundary as zero — issue 0259 with the evidence removed — and
/// the whole point of RFC-0078 is that absent must be loud.
#[derive(Clone, Debug, PartialEq)]
pub enum WcetSelectionError {
    /// `[wcet.select]` names a profile that `[wcet.profiles]` does not define.
    /// Almost always a typo, and a typo must not read as "this board has no
    /// measurements".
    UnknownProfile { target: String, profile: String },
    /// The selected profile exists and cannot be believed.
    InvalidProfile {
        target: String,
        profile: String,
        errors: Vec<WcetError>,
    },
}

impl std::fmt::Display for WcetSelectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownProfile { target, profile } => write!(
                f,
                "[wcet.select] {target} = \"{profile}\" names no profile under \
                 [wcet.profiles]; a typo here would silently mean \"no bounds for \
                 this board\""
            ),
            Self::InvalidProfile {
                target,
                profile,
                errors,
            } => {
                write!(
                    f,
                    "[wcet.profiles.{profile}] (selected by {target}) cannot be believed:"
                )?;
                for e in errors {
                    write!(f, "\n  - {e}")?;
                }
                Ok(())
            }
        }
    }
}

impl SystemToml {
    /// The measurement profile this deploy target selects, validated.
    ///
    /// `Ok(None)` means no bounds are declared for this target — no `[wcet]`
    /// section, or no `[wcet.select]` entry. That is the default and stays
    /// silent. Everything else is an error, because the alternative is a
    /// declaration that exists and does nothing.
    pub fn wcet_profile_for(
        &self,
        target: &str,
    ) -> Result<Option<&WcetProfile>, WcetSelectionError> {
        let Some(wcet) = self.wcet.as_ref() else {
            return Ok(None);
        };
        let Some(name) = wcet.select.get(target) else {
            return Ok(None);
        };
        let profile =
            wcet.profiles
                .get(name)
                .ok_or_else(|| WcetSelectionError::UnknownProfile {
                    target: target.to_string(),
                    profile: name.clone(),
                })?;
        let errors = profile.validate();
        if !errors.is_empty() {
            return Err(WcetSelectionError::InvalidProfile {
                target: target.to_string(),
                profile: name.clone(),
                errors,
            });
        }
        Ok(Some(profile))
    }
}

/// phase-330 W4.0 — one `[[model]]` entry: which launch file, with which
/// argument bindings, produces which model file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemModelEntry {
    /// Launch file name, relative to the bringup's `launch/`.
    pub launch: String,
    /// Output model file name (`<stem>_<binding>_model.yaml` by convention).
    pub out: String,
    /// Launch argument bindings, passed as `key:=value`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub args: BTreeMap<String, String>,
}

/// Phase 256 Wave 1 — `[lifecycle]` in `system.toml`. `autostart` ∈
/// `none` | `configure` | `active` (the `LifecycleAutostart` plan enum);
/// defaults to `none` (services registered, externally driven).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemLifecycle {
    #[serde(default = "default_autostart")]
    pub autostart: String,
}

fn default_autostart() -> String {
    "none".to_string()
}

/// Phase 254 — `[safety]` in `system.toml`: E2E message-integrity (CRC + sequence
/// gap/dup). Mirrors the `PlanSafety` shape; `enabled = false` opts out even when
/// the block is present.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemSafety {
    #[serde(default = "default_true_cap")]
    pub enabled: bool,
    #[serde(default = "default_true_cap")]
    pub crc: bool,
}

/// Phase 254 — `[param_services]` in `system.toml`: the external parameter server.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemParamServices {
    #[serde(default = "default_true_cap")]
    pub enabled: bool,
}

fn default_true_cap() -> bool {
    true
}

/// Phase 261 W4 — shared capability validation for both codegen paths (planner +
/// bake): reject an unknown `[system].features` entry (typo guard, hard error) and
/// warn once per deprecated typed capability block (`[safety]` / `[param_services]`),
/// steering authors to `features = [...]`.
pub fn validate_and_warn_capabilities(sys: &SystemToml) -> eyre::Result<()> {
    let unknown = sys.unknown_features();
    if !unknown.is_empty() {
        let known: Vec<&str> = cargo_nano_ros::capability_resolver::CAPABILITIES
            .iter()
            .map(|c| c.declared)
            .collect();
        eyre::bail!(
            "unknown capability in [system].features: {} (known axes: {})",
            unknown.join(", "),
            known.join(", ")
        );
    }
    for blk in sys.deprecated_typed_capability_blocks() {
        eprintln!(
            "warning: the typed `[{blk}]` block is deprecated (phase-261); declare \
             `features = [\"{blk}\"]` under `[system]` instead"
        );
    }
    Ok(())
}

impl SystemToml {
    /// Phase 261 — is the declared capability axis (`capability_resolver::Capability
    /// .declared`, e.g. `"safety"` / `"param_services"`) enabled in this system?
    /// Maps the registry's language-neutral axis name onto the typed `[block]
    /// enabled` field, so the bake/generate can drive the C/C++ `#define` + Rust
    /// feature lowering from a registry loop instead of hardcoded per-axis branches.
    /// Unknown axis ⇒ `false`.
    pub fn capability_enabled(&self, declared: &str) -> bool {
        // Phase 261 W4 — the generic `[system].features = [...]` surface enables an
        // axis by declared name, equivalently to the typed `[<name>] enabled = true`
        // block. Either source flips the axis on.
        if self.system.features.iter().any(|f| f == declared) {
            return true;
        }
        match declared {
            "safety" => self.safety.as_ref().is_some_and(|s| s.enabled),
            "param_services" => self.param_services.as_ref().is_some_and(|p| p.enabled),
            // Phase 269 W2 — `[lifecycle]` present and autostart != "none" (or omit check:
            // the section being present is sufficient — even "none" registers the services).
            "lifecycle" => self.lifecycle.is_some(),
            _ => false,
        }
    }

    /// Phase 261 W4 — validate `[system].features`: every entry must name a known
    /// capability axis (`capability_resolver::capability`), else a hard error (typo
    /// guard). Returns the unknown names; empty ⇒ all valid.
    pub fn unknown_features(&self) -> Vec<String> {
        self.system
            .features
            .iter()
            .filter(|f| cargo_nano_ros::capability_resolver::capability(f).is_none())
            .cloned()
            .collect()
    }

    /// Phase 261 W4 — the typed capability blocks (`[safety]`, `[param_services]`)
    /// present in this system. Deprecated in favour of `[system].features = [...]`;
    /// callers warn so authors migrate. Returns the declared axis names.
    pub fn deprecated_typed_capability_blocks(&self) -> Vec<&'static str> {
        let mut blocks = Vec::new();
        if self.safety.is_some() {
            blocks.push("safety");
        }
        if self.param_services.is_some() {
            blocks.push("param_services");
        }
        blocks
    }

    /// Which IMAGE a caller that was given no image resolves values against:
    /// `cli` (`--image`, alias `--target`) → the sole image
    /// [`select_default_images`] picks → the sole DEPRECATED `[deploy.<t>]` →
    /// `None`.
    ///
    /// Phase 256 / issue 0951. It was `resolve_target` until the name was the
    /// last thing here still asserting the retired concept: what it returns is
    /// an image id, `[system].default_target` no longer decides anything, and
    /// `--target` survives only as an alias. A function whose name says
    /// "target" and whose body says "image" is how the two came to be read as
    /// synonyms, which is what issue 0938 cost — `nros build` resolving RMW
    /// from `[image.*]` while `plan` read `[deploy.<t>]`. The tree already
    /// spelled it `selected_image` / `cli_image` in `build.rs` and `image.rs`;
    /// this is the last holdout adopting the vocabulary its callers use.
    ///
    /// The image rung DELEGATES to `select_default_images` rather than
    /// reimplementing "explicit list, else the only one". That function is what
    /// a bare `nros build` already uses to decide which images to build, so
    /// routing this through it means `nros plan` and `nros build` cannot
    /// disagree about which image a bringup means by default — the drift issue
    /// 0938 fixed for `rmw`, applied to selection itself. It also inherits the
    /// rule that a `default_images` entry naming no declared image is an ERROR,
    /// not a silent skip.
    ///
    /// Only a selection of exactly ONE image answers here. Several is not an
    /// ambiguity to break by picking: a bringup that builds eight images has no
    /// single "the" target, and `None` makes the caller fall back to its own
    /// defaults rather than resolve half a workspace against one arbitrary
    /// member.
    ///
    /// `[system].default_target` no longer decides. It is authored NOWHERE in
    /// this tree and named the deploy era's concept; `default_images` is its
    /// replacement. The FIELD still parses — `SystemToml` is
    /// `deny_unknown_fields`, so deleting it would turn an unused key into a
    /// hard parse error for an out-of-tree user — and
    /// [`Self::deprecated_default_target_warning`] is what tells them.
    pub fn resolve_image(&self, cli: Option<&str>) -> Option<String> {
        if let Some(c) = cli {
            return Some(c.to_string());
        }
        if let Ok(super::image::ImageSelection::Images(picked)) =
            super::image::select_default_images(&self.image, &self.system.default_images)
            && let [only] = picked.as_slice()
        {
            return Some(only.clone());
        }
        // The DEPRECATED rung, and the only one left that reads `[deploy.*]`.
        // Unreachable for anything this tree authors — 0951 left zero deploy
        // blocks in any `system.toml`, and a synthesised bringup now projects
        // an image for the same row, so the image rung above answers first
        // (`nros_config.rs`'s synthesis test pins exactly that). It stays for
        // an out-of-tree workspace written before 0951, and retires with the
        // rest of `[deploy.*]` parsing at 0.6.0 — phase-383 W10.b, which is
        // waiting on the version boundary, not on this.
        if self.deploy.len() == 1 {
            return self.deploy.keys().next().cloned();
        }
        None
    }

    /// `default_images` names only declared images, or say which it does not.
    ///
    /// [`Self::resolve_image`] must return an `Option`, so it can only IGNORE
    /// a bad `default_images` — and ignoring it means a typo silently
    /// downgrades a plan to image-agnostic instead of failing. Callers with an
    /// error channel call this first; `select_default_images` is the one
    /// implementation of the rule, so this cannot drift from what
    /// `nros build` enforces.
    pub fn validate_default_images(&self) -> Result<(), String> {
        super::image::select_default_images(&self.image, &self.system.default_images).map(|_| ())
    }

    /// Issue 0951 — `[system].default_target` is set but no longer consulted.
    ///
    /// A key that silently stopped working is worse than one that was deleted,
    /// so the retirement says so out loud. `None` when unset, which is every
    /// bringup in this tree.
    #[must_use]
    pub fn deprecated_default_target_warning(&self) -> Option<String> {
        let t = self.system.default_target.as_deref()?;
        Some(format!(
            "[system].default_target = \"{t}\" is retired and no longer selects \
             anything — it named the `[deploy.*]` era's concept. Use \
             `default_images = [\"{t}\"]`, which is also what a bare `nros build` \
             reads."
        ))
    }

    /// The `[host.<name>]` an image runs on, if it names one.
    ///
    /// Issue 0951 — `domain_id` and `locator` are a MACHINE's runtime facts, so
    /// they live on `[host.*]`. But the id a caller resolves is an IMAGE id, and
    /// the two are not the same name: `examples/workspaces/rust` builds images
    /// `native_robot1` / `native_robot2` for hosts `robot1` / `robot2`. The link
    /// is the image's `args` — the launch-argument bindings that are already
    /// "how an image selects a MACHINE" (see [`super::image::ImageBlock::args`]).
    ///
    /// Matched by VALUE, not by a hardcoded `host` argument name. Every in-tree
    /// binding happens to spell it `host`, but the name belongs to the
    /// workspace's launch file, and a workspace spelling it `machine` would
    /// silently get no host — the silent-skip shape. An argument whose value
    /// names a declared host IS the binding, whatever the argument is called.
    ///
    /// Ambiguity is refused rather than guessed: two arguments naming two
    /// different hosts is not a question this can answer, and picking the first
    /// would depend on map order.
    #[must_use]
    pub fn host_for_image(&self, image_id: &str) -> Option<&HostTarget> {
        let img = self.image_for(image_id)?;
        let mut named = img
            .args
            .values()
            .filter_map(|v| self.host.get_key_value(v.as_str()));
        match (named.next(), named.next()) {
            (Some((_, h)), None) => Some(h),
            _ => None,
        }
    }

    /// Phase 256 Wave 8 / issue 0951 — the ROS domain id for `target`:
    /// the bound `[host.<name>].domain_id`, then the DEPRECATED
    /// `[deploy.<target>].domain_id`, then `[system].domain_id`. The RFC-0004
    /// §3.1 ladder (a CLI flag is a future rung). Both codegen paths resolve
    /// through this.
    ///
    /// The host rung is what makes the deprecation lint's own advice true: it
    /// tells users to move `domain_id` to `[host.*]`, and before this nothing
    /// read it there — `SystemToml::host` had no production reader at all, so
    /// following the advice silently baked the `[system]` default into
    /// firmware.
    pub fn resolved_domain_id(&self, target: Option<&str>) -> u32 {
        target
            .and_then(|t| self.host_for_image(t).and_then(|h| h.domain_id))
            .or_else(|| {
                target
                    .and_then(|t| self.deploy.get(t))
                    .and_then(|dt| dt.domain_id)
            })
            .unwrap_or(self.system.domain_id)
    }

    /// Phase 256 Wave 8 / issue 0951 — the locator for `target`: the bound
    /// `[host.<name>].locator`, then the DEPRECATED `[deploy.<target>].locator`,
    /// then `[system].locator` (`None` when none set it).
    pub fn resolved_locator(&self, target: Option<&str>) -> Option<String> {
        target
            .and_then(|t| self.host_for_image(t).and_then(|h| h.locator.clone()))
            .or_else(|| {
                target
                    .and_then(|t| self.deploy.get(t))
                    .and_then(|dt| dt.locator.clone())
            })
            .or_else(|| self.system.locator.clone())
    }

    /// `[image.<id>].rmw` with `[image_defaults]` as the base — the RFC-0065
    /// answer to "what does this image link".
    ///
    /// Deliberately the same merge `facade::image_rmw` performs, reached from
    /// the id rather than from an entry package name, so `nros plan` and
    /// `nros build` cannot drift apart on the same workspace (issue 0938).
    #[must_use]
    pub fn image_rmw_for(&self, id: &str) -> Option<String> {
        self.image_for(id).and_then(|img| img.rmw)
    }

    /// `[image.<id>]` folded over `[image_defaults]` — the one place that
    /// performs the merge.
    ///
    /// Every per-image reader goes through this rather than reaching into
    /// `self.image` directly, because forgetting the base is a silent wrong
    /// answer: the block parses, the field is `None`, and the caller falls
    /// through to a default the author already overrode workspace-wide.
    /// `image_rmw_for` was the only image reader on this type and it hardcoded
    /// one field; this is that function with the field removed.
    #[must_use]
    pub fn image_for(&self, id: &str) -> Option<ImageBlock> {
        let base = self.image_defaults.clone().unwrap_or_default();
        self.image.get(id).map(|img| img.with_base(&base))
    }

    /// The RMW backend name for `target`: the CLI `--rmw` flag, then
    /// `[image.<target>].rmw` (over `[image_defaults]`), then the DEPRECATED
    /// `[deploy.<target>].rmw`, then `[system].rmw`, then `"zenoh"`.
    ///
    /// Issue 0938 — the image rung is new and is what removes a real duality.
    /// `nros build` has resolved an image's RMW from `[image.*]` since RFC-0065
    /// while this helper — used by `nros plan` and `nros codegen-system`, the
    /// latter baking the answer into `#define NROS_SYSTEM_RMW` — read only
    /// `[deploy.<t>].rmw`. A workspace setting both therefore BUILT one backend
    /// and BAKED another, with no diagnostic. This comment's previous version
    /// promised exactly what was missing: "so a given target gets exactly one
    /// RMW — no duality".
    ///
    /// The deploy rung is kept BELOW the image one rather than deleted, because
    /// deleting it would change behaviour for every workspace that still
    /// carries the field, and the deprecation W1.f promised has not reached its
    /// version boundary. Image-wins is the whole fix: where both exist the
    /// answer now matches `nros build`, and where only deploy exists nothing
    /// changes. It goes when `[deploy.*]` retires (phase-383 W10.b).
    ///
    /// `target` names an image in the RFC-0065 world and a deploy in the
    /// RFC-0031 one; they share a namespace by construction, since an image is
    /// what a deploy target used to be.
    pub fn resolved_rmw(&self, target: Option<&str>, cli: Option<&str>) -> String {
        if let Some(c) = cli {
            return c.to_string();
        }
        if let Some(t) = target
            && let Some(r) = self.image_rmw_for(t)
        {
            return r;
        }
        if let Some(t) = target
            && let Some(dt) = self.deploy.get(t)
            && let Some(r) = &dt.rmw
        {
            return r.clone();
        }
        if self.system.rmw.is_empty() {
            "zenoh".to_string()
        } else {
            self.system.rmw.clone()
        }
    }

    /// The serialization format for `target`, by provider name — phase-421 W4,
    /// RFC-0088 D6.
    ///
    /// **The ladder mirrors [`Self::resolved_rmw`]'s, rung for rung**, because
    /// serdes is the same kind of fact: one declared, language-agnostic value
    /// that decides what gets compiled. CLI flag, then `[image.<target>].serdes`
    /// (folded over `[image_defaults]`), then `[system].serdes`, then `"cdr"`.
    ///
    /// **One rung of `rmw`'s is absent, and that is a finding rather than a
    /// choice.** `rmw` also reads `[deploy.<target>].rmw` — the DEPRECATED rung
    /// (see `image::DEPRECATED_DEPLOY_FIELDS`) — and `[deploy.*]` in
    /// `system.toml` is `ros_launch_manifest_model::system_config::DeployBlock`,
    /// an UPSTREAM struct with `deny_unknown_fields`. `serdes` cannot be added
    /// to it from this repo, so RFC-0088 D6's `[deploy.<t>] serdes = "…"`
    /// spelling needs a `ros-launch-manifest` release before it can parse at
    /// all. Nothing is lost by its absence: `[image.*]` is where every build
    /// field moved (RFC-0065 D6, issue 0951), a deploy rung would be born
    /// deprecated, and the Cargo-native deploy block nano-ros DOES own carries
    /// the key ([`DeployTargetMetadata::serdes`]), projected here as an image.
    ///
    /// The default is `cdr` and not, say, the first provider found: a build
    /// that declares no format must behave exactly as it does today.
    #[must_use]
    pub fn resolved_serdes(&self, target: Option<&str>, cli: Option<&str>) -> String {
        if let Some(c) = cli {
            return c.to_string();
        }
        if let Some(t) = target
            && let Some(s) = self.image_for(t).and_then(|img| img.serdes)
        {
            return s;
        }
        self.system
            .serdes
            .clone()
            .unwrap_or_else(|| cargo_nano_ros::serdes_descriptor::DEFAULT_SERDES_NAME.to_string())
    }

    /// Phase 255 Wave 5 — the multi-RMW link set a single binary needs when it
    /// hosts cross-RMW `[[bridge]]`s. A single binary must link the **union** of
    /// every bridged session's RMW so it can speak both sides. Returned in
    /// declaration order, deduped, and always seeded with the system default
    /// (`resolved_rmw(None, None)`) so the local side links too.
    ///
    /// Each `[[bridge]]` names two endpoints (`from`/`to`). An endpoint is an
    /// `<rmw>:<domain>` session selector (e.g. `cyclone:default`) — the RMW is
    /// the prefix before `:`. A bare endpoint with no `:` is read as a
    /// `[[domain]]` name and resolved to that domain's `rmw`. Unresolvable bare
    /// endpoints are skipped (the `nros check` provenance pass flags them; issue
    /// 0076 §A).
    ///
    /// Empty `[[bridge]]` set ⇒ a single-element vec (just the default) ⇒ the
    /// build is byte-identical to a non-bridged single-RMW system.
    pub fn bridged_rmws(&self) -> Vec<String> {
        let mut set: Vec<String> = vec![self.resolved_rmw(None, None)];
        let domain_rmw = |name: &str| -> Option<String> {
            self.domains
                .iter()
                .find(|d| d.name == name)
                .map(|d| d.rmw.clone())
        };
        for bridge in &self.bridges {
            for endpoint in [&bridge.from, &bridge.to] {
                // `<rmw>:<domain>` → the RMW is the prefix; a bare name is a
                // `[[domain]]` reference resolved to its `rmw`.
                let rmw = match endpoint.split_once(':') {
                    Some((rmw, _domain)) => Some(rmw.to_string()),
                    None => domain_rmw(endpoint),
                };
                if let Some(rmw) = rmw
                    && !rmw.is_empty()
                    && !set.contains(&rmw)
                {
                    set.push(rmw);
                }
            }
        }
        set
    }
}

/// `[system]` table inside `<bringup>/system.toml`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemHeader {
    pub name: String,
    pub rmw: String,
    pub domain_id: u32,
    /// Target ROS 2 edition (`humble` | `iron` | `jazzy`) — the
    /// RFC-0056 axis, declared ONCE here and lowered (phase-304 W2) to the
    /// message-gen `--ros-edition` (baked type hash), the `ros-<edition>` cargo
    /// feature (runtime keyexpr format), and the `generated/<edition>/`
    /// interface dir — so the baked hash and the runtime format can never
    /// disagree. Absent ⇒ `humble` (the current default).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ros_edition: Option<String>,
    /// Optional default locator. Per-deploy blocks can override.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
    /// Optional default launch filename — RELATIVE to `<bringup>/launch/`
    /// (per docs/system-toml-schema-v0.1.md §3.1, design-doc §11.3
    /// 2026-06-03). Names the launch file picked when neither CLI flag nor
    /// macro arg nor per-deploy `launch` override selects one. When absent,
    /// the resolver falls back to the literal `"system.launch.xml"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_launch: Option<String>,
    /// Optional default `[deploy.<target>]` block key — picked by
    /// Entry codegen when the user does not pass `--target`. When absent,
    /// the launcher falls back to `"native"` if that block exists, else
    /// the first deploy entry in declaration / sorted order. Phase 212.J.2.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default_target: Option<String>,
    /// `default_images` — which `[image.<id>]` a bare `nros build` builds
    /// (RFC-0065 D1, phase-383 W1.c).
    ///
    /// Absent, and with more than one image declared, `nros build` LISTS them
    /// and fails rather than guessing. PlatformIO's `default_envs` is the
    /// model; the divergence is deliberate — `pio run` builds every
    /// environment when none is named, and `examples/workspaces/rust` declares
    /// eight images across three cross toolchains, so an accidental bare
    /// invocation would be an expensive way to learn the default.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub default_images: Vec<String>,
    /// System-wide serialization format, by PROVIDER NAME (phase-421 W4,
    /// RFC-0088 D6). Absent ⇒ `cdr`, which is what every image builds today.
    ///
    /// Optional where [`Self::rmw`] is mandatory, deliberately: making it
    /// mandatory would mean editing every `system.toml` in the tree to restate
    /// the value they already have, and RFC-0088's rule is that a build
    /// declaring no format behaves exactly as it does now.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub serdes: Option<String>,
    /// Phase 261 W4 — generic capability axes by declared name, e.g.
    /// `features = ["safety", "param_services"]`. Each entry must resolve via
    /// `capability_resolver::capability(name)` (unknown ⇒ hard error, typo guard);
    /// it lowers identically to the typed `[<name>] enabled = true` block on every
    /// language. The thin user surface that replaces the per-axis typed blocks (now
    /// deprecated). Absent ⇒ empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,
}

impl SystemHeader {
    /// phase-304 W2 (RFC-0056) — resolve the declared `[system].ros_edition`
    /// into a [`RosEdition`]. Absent ⇒ `Humble` (the default). An unknown string
    /// is a HARD error (typo guard), never a silent fallback to humble — a wrong
    /// edition silently bakes the wrong type hash / keyexpr format.
    pub fn ros_edition(&self) -> eyre::Result<rosidl_codegen::RosEdition> {
        match &self.ros_edition {
            // phase-405 W3 — derived, not a second literal.
            None => Ok(rosidl_codegen::RosEdition::default()),
            Some(s) => rosidl_codegen::RosEdition::parse(s).ok_or_else(|| {
                eyre::eyre!(
                    "[system].ros_edition = '{s}' is not a known ROS edition \
                     (humble | iron | jazzy)"
                )
            }),
        }
    }
}

/// `[[component]]` row.
///
/// `Eq` is deliberately absent: `params` holds `toml::Value`, which is only
/// `PartialEq` (floats). Nothing needs `Eq` here — the parent `SystemToml` is
/// `PartialEq` too — and a parameter's value type is worth more than a trait
/// no caller uses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemComponentEntry {
    pub pkg: String,
    pub class: String,
    pub name: String,
    /// Phase 273 (RFC-0047 W2) — callback-group → tier assignment for this component,
    /// keyed by the code-declared group name. Deployment policy (workspace-owned),
    /// replacing the package manifest's `callback_groups` tier binding. A group with
    /// no entry runs on the default tier. Empty ⇒ fall back to the package manifest
    /// `callback_groups` tier (deprecated path — move to `group_tiers`).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub group_tiers: BTreeMap<String, String>,
    /// phase-330 W4 — deployment-time parameter values for this component's
    /// node, equivalent to an inline `<param>` in the launch file.
    ///
    /// nano-ros does not consume these directly: the RESOLVER projects them
    /// onto `structure.nodes` (`apply_params_to_nodes`, rlm v0.1.1). They are
    /// declared here so this parser, which denies unknown fields, does not
    /// reject a bringup that uses them.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub params: BTreeMap<String, toml::Value>,
    /// phase-330 W4 — parameter FILE contents for this component's node, same
    /// shape as a `<param from=…>` launch entry. Resolver-consumed, as `params`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub params_files: Vec<String>,
}

/// `[deploy.<target>]` block.
///
/// Per the F.4 §12 known-gap #2 resolution (path a — relax parser): both
/// `kind` and `target` are OPTIONAL. The deploy block is configuration-
/// by-target — the `<target>` map key (e.g. `native`, `qemu-mps2-an385`,
/// `threadx-linux`, `platformio`) already names the runner, and the
/// runner stage derives sensible defaults for `kind` / `target` from
/// the target name when these fields are absent. Strict
/// `deny_unknown_fields` is preserved — widening the schema, not
/// loosening the policy.
///
/// `nros check` is the place to surface a heads-up when `kind`/`target`
/// is absent AND the runner can't synthesise defaults from the target
/// name; that's a lint, not a parser error.
///
/// **Re-exported, not redeclared** (nano-ros issue 0293). This was a second
/// definition of `ros_launch_manifest_model::system_config::DeployBlock`, and
/// the two drifted: rlm's copy lacked `launch`, so serde silently dropped the
/// key and launch-scoped deploy blocks were counted against every launch file.
/// One schema now; `deny_unknown_fields` lives on it, so the next divergence
/// is a parse error rather than a dropped key.
pub use ros_launch_manifest_model::system_config::DeployBlock as DeployTarget;
/// `[host.<name>]` — a machine (issue 0951). Re-exported for the same reason
/// `DeployTarget` is: the schema has one definition, upstream.
pub use ros_launch_manifest_model::system_config::HostBlock as HostTarget;

pub use super::image::ImageBlock;

/// `[[domain]]` row.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemDomainEntry {
    pub name: String,
    pub rmw: String,
    #[serde(default)]
    pub id: u32,
    /// phase-267 (xrce variant) — explicit locator for this domain's session.
    /// Required for agent-based backends (xrce: the Micro-XRCE-DDS Agent address,
    /// e.g. `udp/127.0.0.1:8888`) which can't be discovered by DDS domain id. For
    /// DDS/multicast backends (cyclonedds) it stays `None` (domain-discovered).
    #[serde(default)]
    pub locator: Option<String>,
}

/// `[[bridge]]` row.
///
/// phase-267 W1c — a bridge forwards declared `topics` (by name; `nros sync`
/// resolves each to its `type_name` + `type_hash` from the publishing
/// component's metadata, so the user never writes the opaque RIHS hash). Empty
/// `topics` ⇒ forward every declared topic (resolve-from-interfaces). Direction
/// is `from`→`to`; `bidirectional` adds the reverse relay (echo-suppressed).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemBridgeEntry {
    pub name: String,
    pub from: String,
    pub to: String,
    /// Forwarded topic NAMES (e.g. `["/chatter"]`). Empty ⇒ forward all declared
    /// topics. Types/hashes are resolved by `nros sync`, not written here.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub topics: Vec<String>,
    /// `true` ⇒ also relay `to`→`from` (echo-suppressed). Default one-way.
    #[serde(default, skip_serializing_if = "is_false")]
    pub bidirectional: bool,
}

/// serde `skip_serializing_if` helper — omit a `false` flag so a one-way bridge
/// round-trips byte-identically to a pre-W1c `[[bridge]]`.
fn is_false(b: &bool) -> bool {
    !*b
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    /// issue 0358 — the two spellings must answer identically.
    ///
    /// They mean the same thing, so a consumer that reaches for either one in
    /// isolation is already wrong. Asserting the EQUIVALENCE rather than each
    /// arm separately is what would have failed when the probe checked only
    /// `entry`, instead of quietly passing because the `entry` arm was fine.
    #[test]
    fn both_deploy_bound_spellings_agree() {
        use super::PackageMetadataNros;

        let via_entry: PackageMetadataNros =
            toml::from_str("[entry]\ndeploy = \"native\"\n").expect("entry form parses");
        let via_deploy: PackageMetadataNros =
            toml::from_str("[deploy.zephyr]\nboard = \"native_sim/native/64\"\n")
                .expect("deploy form parses");
        let neither: PackageMetadataNros =
            toml::from_str("[node]\nclass = \"demo::Talker\"\n").expect("node form parses");

        assert!(via_entry.deploy_bound(), "[entry] means deploy-bound");
        assert!(
            via_deploy.deploy_bound(),
            "[deploy.<target>] means deploy-bound too — this is the half the \
             metadata probe forgot (issue 0318), and 27 packages hard-failed"
        );
        assert_eq!(
            via_entry.deploy_bound(),
            via_deploy.deploy_bound(),
            "the two spellings must not disagree"
        );
        assert!(
            !neither.deploy_bound(),
            "a plain node pkg is host-probeable and must stay so"
        );
    }

    use super::*;

    /// phase-304 W2 — `[system].ros_edition` resolves to a `RosEdition`; absent
    /// ⇒ humble; an unknown string is a HARD error (typo guard).
    #[test]
    fn ros_edition_resolves_with_humble_default_and_typo_guard() {
        use rosidl_codegen::RosEdition;
        let hdr = |ed: &str| {
            toml::from_str::<SystemHeader>(&format!(
                "name = \"t\"\nrmw = \"zenoh\"\ndomain_id = 0\nros_edition = \"{ed}\"\n"
            ))
            .unwrap()
        };
        assert_eq!(hdr("jazzy").ros_edition().unwrap(), RosEdition::Jazzy);
        assert_eq!(hdr("Iron").ros_edition().unwrap(), RosEdition::Iron);
        // Absent → humble default.
        let bare: SystemHeader =
            toml::from_str("name = \"t\"\nrmw = \"zenoh\"\ndomain_id = 0\n").unwrap();
        assert_eq!(bare.ros_edition().unwrap(), RosEdition::Humble);
        // Unknown → loud error, never a silent humble fallback.
        let err = hdr("foxy").ros_edition().unwrap_err().to_string();
        assert!(err.contains("not a known ROS edition"), "got: {err}");
    }

    /// phase-267 W1c/C3a — a node declares its published topics+types in
    /// `[package.metadata.nros.node]`; the planner reads them pre-build to
    /// resolve a bridge's topic names.
    #[test]
    fn node_metadata_parses_publishes_and_subscribes() {
        let raw = r#"
[node]
class = "talker_pkg::Talker"
name = "talker"
[[node.publishes]]
topic = "/chatter"
type = "std_msgs/Int32"
[[node.subscribes]]
topic = "/cmd"
type = "std_msgs/Bool"
"#;
        let m: PackageMetadataNros = toml::from_str(raw).expect("parse node metadata");
        let node = m.node.expect("node table");
        assert_eq!(node.publishes.len(), 1);
        assert_eq!(node.publishes[0].topic, "/chatter");
        assert_eq!(node.publishes[0].type_name, "std_msgs/Int32");
        assert_eq!(node.subscribes[0].topic, "/cmd");
        assert_eq!(node.subscribes[0].type_name, "std_msgs/Bool");

        // Back-compat: a node WITHOUT publishes/subscribes still parses.
        let plain: PackageMetadataNros =
            toml::from_str("[node]\nclass=\"p::T\"\nname=\"t\"\n").expect("plain node");
        assert!(plain.node.unwrap().publishes.is_empty());
    }

    /// Round-trip a `[workspace.metadata.nros]` golden through parse +
    /// serialize + reparse and compare structs.
    #[test]
    fn workspace_metadata_round_trip() {
        let raw = r#"
default_system = "demo_bringup"
rmw_override = "cyclonedds"
"#;
        let v1: WorkspaceMetadataNros = toml::from_str(raw).expect("parse golden");
        assert_eq!(v1.default_system.as_deref(), Some("demo_bringup"));
        assert_eq!(v1.rmw_override.as_deref(), Some("cyclonedds"));

        let reserialized = toml::to_string(&v1).expect("serialize");
        let v2: WorkspaceMetadataNros = toml::from_str(&reserialized).expect("reparse");
        assert_eq!(v1, v2);
    }

    /// Minimal workspace-metadata table (only `default_system`) parses; the
    /// optional `rmw_override` defaults to `None`.
    #[test]
    fn workspace_metadata_minimal_parses() {
        let raw = r#"default_system = "demo_bringup""#;
        let v: WorkspaceMetadataNros = toml::from_str(raw).expect("parse");
        assert_eq!(v.default_system.as_deref(), Some("demo_bringup"));
        assert!(v.rmw_override.is_none());
    }

    /// Empty workspace-metadata is also valid (workspace may declare the
    /// table without populating it yet).
    #[test]
    fn workspace_metadata_empty_parses() {
        let v: WorkspaceMetadataNros = toml::from_str("").expect("parse empty");
        assert_eq!(v, WorkspaceMetadataNros::default());
    }

    /// `[package.metadata.nros.component]` single-shape round-trip.
    #[test]
    fn package_metadata_single_component_round_trip() {
        let raw = r#"
[component]
default_namespace = "/demo"

[component.parameters]
rate_hz = 10
greeting = "hello"

[[component.remaps]]
from = "chatter"
to = "topic/chatter"
"#;
        let v1: PackageMetadataNros = toml::from_str(raw).expect("parse");
        v1.validate().expect("single-shape is valid");
        let component = v1.component.as_ref().expect("component present");
        assert_eq!(component.default_namespace.as_deref(), Some("/demo"));
        assert_eq!(component.parameters.len(), 2);
        assert_eq!(component.remaps.len(), 1);
        assert_eq!(component.remaps[0].from, "chatter");
        assert_eq!(component.remaps[0].to, "topic/chatter");
        assert!(v1.components.is_empty());

        let reserialized = toml::to_string(&v1).expect("serialize");
        let v2: PackageMetadataNros = toml::from_str(&reserialized).expect("reparse");
        assert_eq!(v1, v2);
    }

    /// `[package.metadata.nros.components.<Name>]` multi-shape round-trip.
    #[test]
    fn package_metadata_multi_component_round_trip() {
        let raw = r#"
[components.Talker]
default_namespace = "/demo"

[components.Talker.parameters]
rate_hz = 10

[components.Listener]
default_namespace = "/demo"
"#;
        let v1: PackageMetadataNros = toml::from_str(raw).expect("parse");
        v1.validate().expect("multi-shape is valid");
        assert!(v1.component.is_none());
        assert_eq!(v1.components.len(), 2);
        // BTreeMap ⇒ keys are sorted.
        let names: Vec<&str> = v1.components.keys().map(String::as_str).collect();
        assert_eq!(names, ["Listener", "Talker"]);

        let reserialized = toml::to_string(&v1).expect("serialize");
        let v2: PackageMetadataNros = toml::from_str(&reserialized).expect("reparse");
        assert_eq!(v1, v2);
    }

    /// Declaring both `component` and `components` is a hard error (loader
    /// must call `validate`).
    #[test]
    fn package_metadata_rejects_both_shapes() {
        let raw = r#"
[component]
default_namespace = "/a"

[components.Other]
default_namespace = "/b"
"#;
        let v: PackageMetadataNros = toml::from_str(raw).expect("parse");
        let err = v.validate().expect_err("conflicting shapes must error");
        assert!(
            err.contains("component"),
            "diagnostic mentions field: {err}"
        );
    }

    /// `deny_unknown_fields` rejects typos on the component table.
    #[test]
    fn rejects_unknown_field_in_strict_mode() {
        let raw = r#"
[component]
default_namespace = "/demo"
unknown_typo = true
"#;
        let err =
            toml::from_str::<PackageMetadataNros>(raw).expect_err("unknown field must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("unknown_typo") || msg.contains("unknown field"),
            "diagnostic should name the typo: {msg}"
        );
    }

    /// Same strictness for the workspace table.
    #[test]
    fn rejects_unknown_field_on_workspace_metadata() {
        let raw = r#"
default_system = "demo_bringup"
not_a_field = 42
"#;
        let err = toml::from_str::<WorkspaceMetadataNros>(raw)
            .expect_err("unknown field must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("not_a_field") || msg.contains("unknown field"),
            "diagnostic: {msg}"
        );
    }

    /// Phase 212.B.4 — extended `[package.metadata.ament]` carries
    /// `description` / `maintainer = { name, email }` / `license` /
    /// `buildtool_depend` alongside the dependency lists.
    #[test]
    fn parses_ament_metadata_basic() {
        let raw = r#"
description = "A talker that publishes std_msgs/String at 10 Hz."
license = "Apache-2.0"
maintainer = { name = "Ada Lovelace", email = "ada@example.com" }
buildtool_depend = ["ament_cargo"]
exec_depend = ["std_msgs", "rcl_interfaces"]
build_depend = ["std_msgs"]
test_depend = []
"#;
        let v: PackageMetadataAment = toml::from_str(raw).expect("parse");
        assert_eq!(
            v.description.as_deref(),
            Some("A talker that publishes std_msgs/String at 10 Hz.")
        );
        assert_eq!(v.license.as_deref(), Some("Apache-2.0"));
        let m = v.maintainer.as_ref().expect("maintainer present");
        assert_eq!(m.name, "Ada Lovelace");
        assert_eq!(m.email, "ada@example.com");
        assert_eq!(v.buildtool_depend, vec!["ament_cargo"]);
        assert_eq!(v.exec_depend, vec!["std_msgs", "rcl_interfaces"]);
        assert_eq!(v.build_depend, vec!["std_msgs"]);
        assert!(v.test_depend.is_empty());

        // Round-trip cleanly.
        let reser = toml::to_string(&v).expect("ser");
        let v2: PackageMetadataAment = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// `deny_unknown_fields` rejects typos on the extended ament table.
    #[test]
    fn rejects_unknown_field_in_ament_metadata() {
        let raw = r#"
description = "x"
license = "Apache-2.0"
not_a_field = true
"#;
        let err = toml::from_str::<PackageMetadataAment>(raw)
            .expect_err("unknown field must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("not_a_field") || msg.contains("unknown field"),
            "diagnostic: {msg}"
        );
    }

    /// `maintainer = { name, email }` is strict: a stray
    /// `affiliation = …` field fails at parse time.
    #[test]
    fn rejects_unknown_field_in_ament_maintainer() {
        let raw = r#"
maintainer = { name = "Ada", email = "a@b.c", affiliation = "ACME" }
"#;
        let err = toml::from_str::<PackageMetadataAment>(raw)
            .expect_err("unknown field on maintainer must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("affiliation") || msg.contains("unknown field"),
            "diagnostic: {msg}"
        );
    }

    /// Phase 212.L.7 — `[workspace.metadata.nros] default_system = "..."`
    /// + `rmw_override` + `domain_id_override` round-trips.
    #[test]
    fn loads_workspace_metadata_default_system() {
        let raw = r#"
default_system = "demo_bringup"
rmw_override = "cyclonedds"
domain_id_override = 7
"#;
        let v: WorkspaceMetadataNros = toml::from_str(raw).expect("parse");
        assert_eq!(v.default_system.as_deref(), Some("demo_bringup"));
        assert_eq!(v.rmw_override.as_deref(), Some("cyclonedds"));
        assert_eq!(v.domain_id_override, Some(7));

        let reser = toml::to_string(&v).expect("ser");
        let v2: WorkspaceMetadataNros = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// `[package.metadata.ament]` round-trip (legacy fields only).
    #[test]
    fn ament_metadata_round_trip() {
        let raw = r#"
build_depend = ["rosidl_default_generators"]
exec_depend = ["rosidl_default_runtime", "std_msgs"]
test_depend = ["ament_lint_auto"]
build_type = "ament_cargo"
"#;
        let v1: PackageMetadataAment = toml::from_str(raw).expect("parse");
        assert_eq!(v1.build_depend, vec!["rosidl_default_generators"]);
        assert_eq!(v1.exec_depend, vec!["rosidl_default_runtime", "std_msgs"]);
        assert_eq!(v1.test_depend, vec!["ament_lint_auto"]);
        assert_eq!(v1.build_type.as_deref(), Some("ament_cargo"));

        let reserialized = toml::to_string(&v1).expect("serialize");
        let v2: PackageMetadataAment = toml::from_str(&reserialized).expect("reparse");
        assert_eq!(v1, v2);
    }

    /// Minimal `[package.metadata.ament]` (only `exec_depend`) parses.
    #[test]
    fn ament_metadata_minimal_parses() {
        let raw = r#"exec_depend = ["std_msgs"]"#;
        let v: PackageMetadataAment = toml::from_str(raw).expect("parse");
        assert_eq!(v.exec_depend, vec!["std_msgs"]);
        assert!(v.build_depend.is_empty());
        assert!(v.test_depend.is_empty());
        assert!(v.build_type.is_none());
    }

    /// Full `<bringup>/system.toml` golden round-trip.
    #[test]
    fn system_toml_round_trip() {
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0
locator = "tcp/127.0.0.1:7447"

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::TalkerNode"
name = "talker"

[[component]]
pkg = "listener_pkg"
class = "listener_pkg::ListenerNode"
name = "listener"

[deploy.native]
kind = "self"
target = "x86_64-unknown-linux-gnu"
launch = "launch/system.launch.xml"

[deploy.qemu-mps2-an385]
kind = "qemu"
target = "thumbv7m-none-eabi"
board = "mps2_an385"

[[domain]]
name = "default"
rmw = "zenoh"
id = 0

[[bridge]]
name = "cyclone_to_zenoh"
from = "cyclone:default"
to = "zenoh:default"
"#;
        let v1: SystemToml = toml::from_str(raw).expect("parse system.toml");
        assert_eq!(v1.system.name, "demo");
        assert_eq!(v1.system.rmw, "zenoh");
        assert_eq!(v1.system.domain_id, 0);
        assert_eq!(v1.system.locator.as_deref(), Some("tcp/127.0.0.1:7447"));
        assert_eq!(v1.components.len(), 2);
        assert_eq!(v1.components[0].name, "talker");
        assert_eq!(v1.components[1].name, "listener");
        assert_eq!(v1.deploy.len(), 2);
        let native = v1.deploy.get("native").expect("native deploy present");
        assert_eq!(native.kind.as_deref(), Some("self"));
        assert_eq!(native.launch.as_deref(), Some("launch/system.launch.xml"));
        let qemu = v1
            .deploy
            .get("qemu-mps2-an385")
            .expect("qemu deploy present");
        assert_eq!(qemu.board.as_deref(), Some("mps2_an385"));
        assert_eq!(v1.domains.len(), 1);
        assert_eq!(v1.bridges.len(), 1);

        let reserialized = toml::to_string(&v1).expect("serialize");
        let v2: SystemToml = toml::from_str(&reserialized).expect("reparse");
        assert_eq!(v1, v2);
    }

    /// Phase 255 Wave 5 — `bridged_rmws` returns the union of the system default
    /// plus every cross-RMW `[[bridge]]` endpoint's RMW (the `<rmw>:<domain>`
    /// prefix, or a bare `[[domain]]` name resolved to its rmw). No bridges ⇒
    /// just the default (single-RMW, byte-identical build).
    #[test]
    fn bridged_rmws_unions_bridge_endpoints() {
        // `<rmw>:<domain>` selectors — the prefix is the RMW.
        let prefixed: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [[domain]]\nname=\"default\"\nrmw=\"zenoh\"\nid=0\n\
             [[bridge]]\nname=\"b\"\nfrom=\"cyclonedds:default\"\nto=\"zenoh:default\"\n",
        )
        .unwrap();
        assert_eq!(prefixed.bridged_rmws(), vec!["zenoh", "cyclonedds"]);

        // Bare endpoint names resolve through `[[domain]]`.
        let by_domain: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [[domain]]\nname=\"cloud\"\nrmw=\"cyclonedds\"\nid=1\n\
             [[bridge]]\nname=\"b\"\nfrom=\"cloud\"\nto=\"local\"\n",
        )
        .unwrap();
        // `cloud` → cyclonedds; `local` is undeclared → skipped.
        assert_eq!(by_domain.bridged_rmws(), vec!["zenoh", "cyclonedds"]);

        // No bridges ⇒ just the system default.
        let plain: SystemToml =
            toml::from_str("[system]\nname=\"d\"\nrmw=\"xrce\"\ndomain_id=0\n").unwrap();
        assert_eq!(plain.bridged_rmws(), vec!["xrce"]);
    }

    /// Phase 254 — `[safety]` / `[param_services]` capability axes parse as typed
    /// `system.toml` tables (the single home both codegen paths read), with
    /// defaults + round-trip. Absent ⇒ `None` (byte-identical to pre-254).
    #[test]
    fn parses_system_toml_capability_axes() {
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[safety]
crc = false

[param_services]
"#;
        let v: SystemToml = toml::from_str(raw).expect("parse system.toml with capabilities");
        let safety = v.safety.as_ref().expect("[safety] present");
        assert!(safety.enabled, "enabled defaults true");
        assert!(!safety.crc, "crc = false round-trips");
        let ps = v.param_services.as_ref().expect("[param_services] present");
        assert!(ps.enabled, "enabled defaults true");

        // Round-trip.
        let v2: SystemToml =
            toml::from_str(&toml::to_string(&v).expect("serialize")).expect("reparse");
        assert_eq!(v, v2);

        // Absent → None.
        let bare: SystemToml =
            toml::from_str("[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n").expect("bare");
        assert!(bare.safety.is_none() && bare.param_services.is_none());

        // enabled = false opts out.
        let off: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n[safety]\nenabled=false\n",
        )
        .expect("parse");
        assert!(!off.safety.as_ref().unwrap().enabled);
    }

    /// Phase 255 — `resolved_rmw` applies the RFC-0031 precedence:
    /// `--rmw` > `[deploy.<t>].rmw` > `[system].rmw` > `zenoh`.
    #[test]
    fn resolved_rmw_precedence_ladder() {
        let sys: SystemToml = toml::from_str(
            r#"
[system]
name = "d"
rmw = "zenoh"
domain_id = 0

[deploy.native]
rmw = "cyclonedds"

[deploy.qemu]
kind = "qemu"
"#,
        )
        .unwrap();

        // CLI flag wins over everything.
        assert_eq!(sys.resolved_rmw(Some("native"), Some("xrce")), "xrce");
        // [deploy.<t>].rmw overrides [system].rmw.
        assert_eq!(sys.resolved_rmw(Some("native"), None), "cyclonedds");
        // …but an [image.<t>].rmw outranks it — issue 0938. This is the case
        // that had NO coverage and no diagnostic: the workspace built one
        // backend (`nros build` reads the image) and baked another
        // (`codegen-system` read the deploy).
        let both: SystemToml = toml::from_str(
            r#"
[system]
name = "d"
rmw = "zenoh"
domain_id = 0

[image.gw]
board = "native"
rmw = "xrce"

[deploy.gw]
rmw = "cyclonedds"
"#,
        )
        .unwrap();
        assert_eq!(
            both.resolved_rmw(Some("gw"), None),
            "xrce",
            "the image owns the RMW; a deploy field must not override it"
        );
        // And `[image_defaults]` is the base, exactly as `facade::image_rmw`
        // merges it — one answer for `nros build` and `nros plan`.
        let defaults: SystemToml = toml::from_str(
            r#"
[system]
name = "d"
rmw = "zenoh"
domain_id = 0

[image_defaults]
rmw = "cyclonedds"

[image.gw]
board = "native"
"#,
        )
        .unwrap();
        assert_eq!(defaults.resolved_rmw(Some("gw"), None), "cyclonedds");
        // deploy block without rmw → falls to [system].rmw.
        assert_eq!(sys.resolved_rmw(Some("qemu"), None), "zenoh");
        // unknown / no target → [system].rmw.
        assert_eq!(sys.resolved_rmw(Some("nope"), None), "zenoh");
        assert_eq!(sys.resolved_rmw(None, None), "zenoh");

        // Empty [system].rmw → the built-in "zenoh" default.
        let bare: SystemToml =
            toml::from_str("[system]\nname=\"d\"\nrmw=\"\"\ndomain_id=0\n").unwrap();
        assert_eq!(bare.resolved_rmw(None, None), "zenoh");
    }

    /// phase-421 W4 — `resolved_serdes` mirrors `resolved_rmw`'s ladder:
    /// `--serdes` > `[image.<t>].serdes` (over `[image_defaults]`) >
    /// `[system].serdes` > `cdr`.
    #[test]
    fn resolved_serdes_precedence_ladder() {
        let sys: SystemToml = toml::from_str(
            r#"
[system]
name = "d"
rmw = "zenoh"
domain_id = 0
serdes = "uorb"

[image.gw]
board = "native"
serdes = "flatbuf"

[image.plain]
board = "native"
"#,
        )
        .unwrap();

        assert_eq!(sys.resolved_serdes(Some("gw"), Some("cdr")), "cdr");
        assert_eq!(sys.resolved_serdes(Some("gw"), None), "flatbuf");
        assert_eq!(sys.resolved_serdes(Some("plain"), None), "uorb");
        assert_eq!(sys.resolved_serdes(Some("nope"), None), "uorb");
        assert_eq!(sys.resolved_serdes(None, None), "uorb");

        // `[image_defaults]` is the base, exactly as it is for rmw.
        let defaults: SystemToml = toml::from_str(
            r#"
[system]
name = "d"
rmw = "zenoh"
domain_id = 0

[image_defaults]
serdes = "flatbuf"

[image.gw]
board = "native"
"#,
        )
        .unwrap();
        assert_eq!(defaults.resolved_serdes(Some("gw"), None), "flatbuf");
    }

    /// A system.toml that never mentions a serialization format must resolve to
    /// CDR — RFC-0088's rule that declaring nothing changes nothing.
    #[test]
    fn a_system_declaring_no_serdes_is_cdr_everywhere() {
        let bare: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n[image.gw]\nboard=\"native\"\n",
        )
        .unwrap();
        assert_eq!(bare.resolved_serdes(None, None), "cdr");
        assert_eq!(bare.resolved_serdes(Some("gw"), None), "cdr");
        // And the answer is a name the resolver can actually lower.
        assert!(
            cargo_nano_ros::serdes_resolver::resolve_serdes(&bare.resolved_serdes(None, None))
                .is_ok(),
            "the default must be a provider this checkout ships"
        );
    }

    /// Phase 256 / issue 0951 — `resolve_image`: `--image` → the sole image
    /// `default_images` picks → the sole `[deploy.<t>]` → `None`.
    #[test]
    fn resolve_image_precedence() {
        // CLI flag wins.
        let two: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             default_images=[\"native\"]\n\
             [image.native]\nboard=\"native\"\n[image.qemu]\nboard=\"native\"\n",
        )
        .unwrap();
        assert_eq!(two.resolve_image(Some("qemu")).as_deref(), Some("qemu"));
        // No flag → the one image `default_images` names.
        assert_eq!(two.resolve_image(None).as_deref(), Some("native"));

        // `default_images` naming SEVERAL is not an answer to "which one".
        // A bringup that builds three images has no single "the" target, and
        // picking a member arbitrarily would resolve the whole workspace
        // against it.
        let many: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             default_images=[\"a\",\"b\"]\n\
             [image.a]\nboard=\"native\"\n[image.b]\nboard=\"native\"\n",
        )
        .unwrap();
        assert_eq!(many.resolve_image(None), None);

        // `[system].default_target` is RETIRED — it named the deploy era's
        // concept. The field still parses (deleting it from a
        // `deny_unknown_fields` struct would be a hard error for an
        // out-of-tree user), but it decides nothing, and the warning is what
        // says so.
        let retired: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\ndefault_target=\"a\"\n\
             [image.a]\nboard=\"native\"\n[image.b]\nboard=\"native\"\n",
        )
        .unwrap();
        assert_eq!(retired.resolve_image(None), None, "it must not decide");
        let w = retired
            .deprecated_default_target_warning()
            .expect("set, so warned");
        assert!(w.contains("default_images"), "{w}");
        assert!(
            toml::from_str::<SystemToml>("[system]\nname=\"d\"\nrmw=\"z\"\ndomain_id=0\n")
                .unwrap()
                .deprecated_default_target_warning()
                .is_none(),
            "unset must not warn"
        );

        // Two deploys → ambiguous → None.
        let ambiguous: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [deploy.a]\nkind=\"self\"\n[deploy.b]\nkind=\"self\"\n",
        )
        .unwrap();
        assert_eq!(ambiguous.resolve_image(None), None);

        // Sole deploy → that one (the deprecated rung, still live).
        let sole: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n[deploy.only]\nkind=\"self\"\n",
        )
        .unwrap();
        assert_eq!(sole.resolve_image(None).as_deref(), Some("only"));

        // Issue 0951 — the sole IMAGE resolves the same way, so a workspace
        // that has finished migrating off `[deploy.*]` still has a target.
        // Without this rung, deleting the last deploy block does not fail: the
        // plan quietly falls back to the x86_64 / native / debug defaults.
        let sole_image: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [image.firmware]\nboard=\"mps2-an385-freertos\"\n",
        )
        .unwrap();
        assert_eq!(sole_image.resolve_image(None).as_deref(), Some("firmware"));

        // Two images are as ambiguous as two deploys.
        let two_images: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [image.a]\nboard=\"native\"\n[image.b]\nboard=\"native\"\n",
        )
        .unwrap();
        assert_eq!(two_images.resolve_image(None), None);

        // Mid-migration: one image and one deploy. The IMAGE is the half that
        // survives, so it decides — picking the deploy would resolve to a name
        // that is about to stop existing.
        let both: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [image.firmware]\nboard=\"native\"\n[deploy.legacy]\nkind=\"self\"\n",
        )
        .unwrap();
        assert_eq!(both.resolve_image(None).as_deref(), Some("firmware"));
    }

    /// `image_for` folds `[image_defaults]` under the block. Forgetting the
    /// base is a silent wrong answer — the field reads `None` and the caller
    /// falls through to a default the author already overrode workspace-wide.
    #[test]
    fn image_for_folds_the_defaults_table() {
        let sys: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\n\
             [image_defaults]\nrmw=\"cyclonedds\"\nprofile=\"release\"\n\
             [image.a]\nboard=\"native\"\n\
             [image.b]\nboard=\"native\"\nprofile=\"debug\"\n",
        )
        .unwrap();
        let a = sys.image_for("a").expect("declared");
        assert_eq!(a.rmw.as_deref(), Some("cyclonedds"), "inherited");
        assert_eq!(a.profile.as_deref(), Some("release"), "inherited");
        let b = sys.image_for("b").expect("declared");
        assert_eq!(b.profile.as_deref(), Some("debug"), "the block wins");
        assert_eq!(b.rmw.as_deref(), Some("cyclonedds"), "still inherited");
        assert!(sys.image_for("nope").is_none());
    }

    /// Issue 0951 — `domain_id` / `locator` come from the `[host.*]` the image
    /// is bound to. The deprecation lint has told users to move them there
    /// since the `[deploy.*]` retirement began, and nothing read them there:
    /// following the advice silently baked the `[system]` default instead.
    #[test]
    fn runtime_values_resolve_through_the_bound_host() {
        let sys: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=1\nlocator=\"tcp/sys:7447\"\n\
             [host.robot1]\ndomain_id=7\nlocator=\"tcp/r1:7447\"\n\
             [host.robot2]\n\
             [image.native_robot1]\nboard=\"native\"\nargs={ host = \"robot1\" }\n\
             [image.native_robot2]\nboard=\"native\"\nargs={ host = \"robot2\" }\n\
             [image.plain]\nboard=\"native\"\n",
        )
        .unwrap();

        // The image names its machine through `args`; the machine states the
        // runtime facts.
        assert_eq!(sys.resolved_domain_id(Some("native_robot1")), 7);
        assert_eq!(
            sys.resolved_locator(Some("native_robot1")).as_deref(),
            Some("tcp/r1:7447")
        );
        // A host that overrides nothing falls to `[system]` — not to the other
        // host's values.
        assert_eq!(sys.resolved_domain_id(Some("native_robot2")), 1);
        assert_eq!(
            sys.resolved_locator(Some("native_robot2")).as_deref(),
            Some("tcp/sys:7447")
        );
        // An image bound to no host is a system-level answer.
        assert_eq!(sys.resolved_domain_id(Some("plain")), 1);
        assert_eq!(sys.resolved_domain_id(None), 1);
    }

    /// The binding is matched by VALUE, so the launch argument may be called
    /// anything. Hardcoding `host` would silently place nothing for a workspace
    /// that spells it `machine`.
    #[test]
    fn the_binding_argument_may_be_named_anything() {
        let sys: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=1\n\
             [host.robot1]\ndomain_id=7\n\
             [image.a]\nboard=\"native\"\nargs={ machine = \"robot1\" }\n",
        )
        .unwrap();
        assert_eq!(sys.resolved_domain_id(Some("a")), 7);
    }

    /// Two arguments naming two different hosts is not a question this can
    /// answer, and picking the first would depend on map order.
    #[test]
    fn two_bound_hosts_are_refused_rather_than_guessed() {
        let sys: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=1\n\
             [host.a]\ndomain_id=7\n[host.b]\ndomain_id=9\n\
             [image.i]\nboard=\"native\"\nargs={ x = \"a\", y = \"b\" }\n",
        )
        .unwrap();
        assert!(sys.host_for_image("i").is_none());
        assert_eq!(sys.resolved_domain_id(Some("i")), 1, "falls to [system]");
    }

    /// Phase 256 Wave 8 — `[deploy.<t>].domain_id`/`.locator` override the
    /// `[system]` defaults for that target; absent → the system value.
    #[test]
    fn resolved_domain_and_locator_honour_deploy_override() {
        let sys: SystemToml = toml::from_str(
            "[system]\nname=\"d\"\nrmw=\"zenoh\"\ndomain_id=0\nlocator=\"tcp/sys:7447\"\n\
             [deploy.robot]\nkind=\"flash\"\ndomain_id=7\nlocator=\"tcp/robot:7450\"\n\
             [deploy.native]\nkind=\"self\"\n",
        )
        .unwrap();

        // robot overrides both.
        assert_eq!(sys.resolved_domain_id(Some("robot")), 7);
        assert_eq!(
            sys.resolved_locator(Some("robot")).as_deref(),
            Some("tcp/robot:7450")
        );
        // native overrides neither → [system] defaults.
        assert_eq!(sys.resolved_domain_id(Some("native")), 0);
        assert_eq!(
            sys.resolved_locator(Some("native")).as_deref(),
            Some("tcp/sys:7447")
        );
        // no target → [system] defaults.
        assert_eq!(sys.resolved_domain_id(None), 0);
    }

    /// Minimal `<bringup>/system.toml` — only `[system]` + one
    /// `[[component]]`, optional sections absent.
    #[test]
    fn system_toml_minimal_parses() {
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::TalkerNode"
name = "talker"
"#;
        let v: SystemToml = toml::from_str(raw).expect("parse minimal");
        assert_eq!(v.system.name, "demo");
        assert!(v.system.locator.is_none());
        assert_eq!(v.components.len(), 1);
        assert!(v.deploy.is_empty());
        assert!(v.domains.is_empty());
        assert!(v.bridges.is_empty());
        // Phase 228.A — tier surface defaults empty (backward compat).
        assert!(v.tiers.is_empty());
        assert!(v.node_overrides.is_empty());
    }

    #[test]
    fn system_toml_parses_tiers_and_overrides() {
        // Phase 228.A (RFC-0015 §4.2) — tier + override schema.
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "ctrl_pkg"
class = "ctrl_pkg::Control"
name = "control_node"

[tiers.high]
spin_period_us = 1000
[tiers.high.freertos]
priority = 5
stack_bytes = 8192
[tiers.high.zephyr]
priority = -1
[tiers.high.posix]
priority = 80
sched_class = "SCHED_FIFO"

[tiers.low.freertos]
priority = 1

[[node_overrides]]
name = "control_node"
callback_groups = [{ id = "telemetry", tier = "low" }]
"#;
        let v: SystemToml = toml::from_str(raw).expect("parse tier schema");

        let high = v.tiers.get("high").expect("high tier");
        assert_eq!(high.spin_period_us, Some(1000));
        assert_eq!(high.freertos.as_ref().unwrap().priority, 5);
        assert_eq!(high.freertos.as_ref().unwrap().stack_bytes, Some(8192));
        assert_eq!(high.zephyr.as_ref().unwrap().priority, -1);
        assert_eq!(
            high.posix.as_ref().unwrap().sched_class.as_deref(),
            Some("SCHED_FIFO")
        );
        assert_eq!(
            v.tiers
                .get("low")
                .unwrap()
                .freertos
                .as_ref()
                .unwrap()
                .priority,
            1
        );

        let ov = &v.node_overrides[0];
        assert_eq!(ov.name, "control_node");
        assert_eq!(ov.callback_groups[0].tier, "low");

        // Round-trips through serde.
        let s = toml::to_string(&v).expect("serialize");
        let v2: SystemToml = toml::from_str(&s).expect("reparse");
        assert_eq!(v, v2);
    }

    // -----------------------------------------------------------------
    // Phase 212.L — class / name / application / deploy schema additions
    // -----------------------------------------------------------------

    /// `[package.metadata.nros.component]` round-trip carries the new
    /// `class` + `name` fields.
    #[test]
    fn loads_component_with_class_and_name() {
        let raw = r#"
[component]
class = "alpha_pkg::Node"
name = "alpha"
default_namespace = "/demo"

[component.parameters]
rate_hz = 5
"#;
        let v: PackageMetadataNros = toml::from_str(raw).expect("parse");
        v.validate().expect("single-shape ok");
        let c = v.component.as_ref().expect("component present");
        assert_eq!(c.class.as_deref(), Some("alpha_pkg::Node"));
        assert_eq!(c.name.as_deref(), Some("alpha"));
        assert_eq!(c.default_namespace.as_deref(), Some("/demo"));
        assert_eq!(c.parameters.len(), 1);

        // Round-trip.
        let reser = toml::to_string(&v).expect("ser");
        let v2: PackageMetadataNros = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// `[package.metadata.nros.application]` accepts a `deploy = […]`
    /// allow-list and an optional `name`.
    #[test]
    fn loads_application_with_deploy_targets() {
        let raw = r#"
[application]
name = "demo_app"
deploy = ["native", "qemu-arm-baremetal"]
"#;
        let v: PackageMetadataNros = toml::from_str(raw).expect("parse");
        v.validate().expect("application-shape ok");
        let app = v.application.as_ref().expect("application present");
        assert_eq!(app.name.as_deref(), Some("demo_app"));
        assert_eq!(app.deploy, vec!["native", "qemu-arm-baremetal"]);
        assert!(v.component.is_none());
        assert!(v.components.is_empty());

        let reser = toml::to_string(&v).expect("ser");
        let v2: PackageMetadataNros = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// `[package.metadata.nros.deploy.<target>]` populates the typed
    /// per-target table.
    #[test]
    fn loads_deploy_target_metadata() {
        let raw = r#"
[component]
class = "alpha_pkg::Node"
name = "alpha"

[deploy.native]
board = "native_sim/native/64"
rmw = "zenoh"
domain_id = 7
locator = "tcp/127.0.0.1:7447"

[deploy.qemu-mps2-an385]
board = "mps2-an385"
rmw = "cyclonedds"
"#;
        let v: PackageMetadataNros = toml::from_str(raw).expect("parse");
        v.validate().expect("valid");
        assert!(v.is_self_bringup_eligible());
        assert_eq!(v.deploy.len(), 2);
        let native = v.deploy.get("native").expect("native present");
        assert_eq!(native.board.as_deref(), Some("native_sim/native/64"));
        assert_eq!(native.rmw.as_deref(), Some("zenoh"));
        assert_eq!(native.domain_id, Some(7));
        assert_eq!(native.locator.as_deref(), Some("tcp/127.0.0.1:7447"));
        let qemu = v.deploy.get("qemu-mps2-an385").expect("qemu present");
        assert_eq!(qemu.board.as_deref(), Some("mps2-an385"));
        assert_eq!(qemu.rmw.as_deref(), Some("cyclonedds"));
        assert!(qemu.domain_id.is_none());
        assert!(qemu.locator.is_none());

        let reser = toml::to_string(&v).expect("ser");
        let v2: PackageMetadataNros = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// `deny_unknown_fields` rejects typos on `[component]` w/ the new
    /// `class` + `name` siblings.
    #[test]
    fn rejects_unknown_field_in_component() {
        let raw = r#"
[component]
class = "alpha_pkg::Node"
name = "alpha"
bogus = true
"#;
        let err =
            toml::from_str::<PackageMetadataNros>(raw).expect_err("unknown field must reject");
        let s = err.to_string();
        assert!(s.contains("bogus") || s.contains("unknown field"), "{s}");
    }

    /// `deny_unknown_fields` on `[application]`.
    #[test]
    fn rejects_unknown_field_in_application() {
        let raw = r#"
[application]
deploy = ["native"]
oops = 1
"#;
        let err = toml::from_str::<PackageMetadataNros>(raw)
            .expect_err("unknown field on application must reject");
        let s = err.to_string();
        assert!(s.contains("oops") || s.contains("unknown field"), "{s}");
    }

    /// `deny_unknown_fields` on `[deploy.<target>]`.
    #[test]
    fn rejects_unknown_field_in_deploy_target() {
        let raw = r#"
[component]
class = "alpha_pkg::Node"
name = "alpha"

[deploy.native]
board = "native_sim"
mystery = "no"
"#;
        let err = toml::from_str::<PackageMetadataNros>(raw)
            .expect_err("unknown field on deploy.<target> must reject");
        let s = err.to_string();
        assert!(s.contains("mystery") || s.contains("unknown field"), "{s}");
    }

    /// Component + application in the same pkg is rejected (mutex).
    #[test]
    fn rejects_component_and_application_in_same_pkg() {
        let raw = r#"
[component]
class = "alpha_pkg::Node"
name = "alpha"

[application]
deploy = ["native"]
"#;
        let v: PackageMetadataNros = toml::from_str(raw).expect("parse");
        let err = v.validate().expect_err("mutex must trip");
        assert!(
            err.contains("application") || err.contains("component"),
            "{err}"
        );
    }

    /// `[system].default_launch` is accepted by `SystemHeader` per
    /// docs/system-toml-schema-v0.1.md §3.1 (resolves the 2026-06-03 §11.3
    /// design lock — F.4 §12 known gap #1).
    #[test]
    fn parses_system_toml_with_default_launch() {
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0
default_launch = "talker_only.launch.xml"

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::TalkerNode"
name = "talker"
"#;
        let v: SystemToml = toml::from_str(raw).expect("parse with default_launch");
        assert_eq!(
            v.system.default_launch.as_deref(),
            Some("talker_only.launch.xml")
        );
        // Absence keeps the field None — resolver supplies the literal
        // "system.launch.xml" fallback.
        let raw_minimal = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0
"#;
        let v_min: SystemToml = toml::from_str(raw_minimal).expect("parse minimal");
        assert!(v_min.system.default_launch.is_none());
    }

    /// `[deploy.<target>]` accepts a block with neither `kind` nor `target`
    /// — both fields are optional per F.4 §12 known-gap #2 path (a). The
    /// in-tree `multi_pkg_workspace_threadx` fixture carries such a block;
    /// the runner derives sensible defaults from the `<target>` map key.
    /// (The PlatformIO fixture that also carried one went with the
    /// integration's test surface — issue 0704.)
    #[test]
    fn accepts_deploy_target_without_kind() {
        // Mirrors the `multi_pkg_workspace_threadx` fixture shape.
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::Talker"
name = "talker"

[deploy.threadx-linux]
launch = "launch/system.launch.xml"
"#;
        let v: SystemToml = toml::from_str(raw)
            .expect("deploy block without kind/target must parse (F.4 §12 gap #2)");
        let dt = v
            .deploy
            .get("threadx-linux")
            .expect("threadx-linux deploy present");
        assert!(dt.kind.is_none(), "kind absent when omitted");
        assert!(dt.target.is_none(), "target absent when omitted");
        assert_eq!(dt.launch.as_deref(), Some("launch/system.launch.xml"));
        assert!(dt.board.is_none());
    }

    /// `[deploy.<target>].framework` is accepted (F.4 §12 known gap #3).
    /// PlatformIO carries `framework = "espidf"` / `"arduino"` / … on its
    /// deploy block; the field passes through verbatim for the runner.
    /// The schema case outlives the fixture: PlatformIO's test surface was
    /// removed with the integration (issue 0704), but `framework` is a
    /// pass-through field and this test needs no fixture to pin it.
    #[test]
    fn accepts_platformio_framework_field() {
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0

[[component]]
pkg = "talker_pkg"
class = "talker_pkg::talker"
name = "talker"

[deploy.platformio]
launch = "launch/system.launch.xml"
framework = "espidf"
board = "esp32dev"
"#;
        let v: SystemToml =
            toml::from_str(raw).expect("framework field must parse (F.4 §12 gap #3)");
        let dt = v
            .deploy
            .get("platformio")
            .expect("platformio deploy present");
        assert_eq!(dt.framework.as_deref(), Some("espidf"));
        assert_eq!(dt.board.as_deref(), Some("esp32dev"));
        assert_eq!(dt.launch.as_deref(), Some("launch/system.launch.xml"));
        // Round-trip: serialized form keeps the field.
        let reser = toml::to_string(&v).expect("ser");
        let v2: SystemToml = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// Phase 212.M-F.17 — synthesis subset round-trip. The α-bridge in
    /// `workspace.rs::synthetic_metadata_artifacts` reads `class` /
    /// `name` / `default_namespace` out of `[component]` and the
    /// `[components.<Name>]` table-of-tables, then mints fresh JSON for
    /// the planner. Lock in the shape of those reads here so a future
    /// schema tweak that drops one of them surfaces at the cargo
    /// metadata schema boundary (closest to the user-facing TOML)
    /// instead of as a planner-level mystery.
    #[test]
    fn synthesis_subset_round_trip_single_and_multi() {
        // Single-shape `[component]` carrying every M-F.17 field.
        let raw_single = r#"
[component]
class = "talker_pkg::Talker"
name = "talker"
default_namespace = "/demo"
"#;
        let v: PackageMetadataNros = toml::from_str(raw_single).expect("parse single");
        v.validate().expect("single-shape valid");
        let c = v.component.as_ref().expect("component present");
        assert_eq!(c.class.as_deref(), Some("talker_pkg::Talker"));
        assert_eq!(c.name.as_deref(), Some("talker"));
        assert_eq!(c.default_namespace.as_deref(), Some("/demo"));

        // Multi-shape `[components.<Name>]` — same subset on a per-entry
        // basis. Bridge uses `<Name>` as the component-name fallback
        // when `metadata.name` is absent on the entry.
        let raw_multi = r#"
[components.Talker]
class = "talker_pkg::Talker"
default_namespace = "/demo"

[components.Listener]
class = "listener_pkg::Listener"
"#;
        let v: PackageMetadataNros = toml::from_str(raw_multi).expect("parse multi");
        v.validate().expect("multi-shape valid");
        assert_eq!(v.components.len(), 2);
        let talker = v.components.get("Talker").expect("Talker entry");
        assert_eq!(talker.class.as_deref(), Some("talker_pkg::Talker"));
        assert_eq!(talker.default_namespace.as_deref(), Some("/demo"));
        // Multi-shape entries inherit their component name from the
        // table key when `metadata.name` is absent — the bridge layer
        // checks both, but the schema records only what's authored.
        assert!(talker.name.is_none());

        // Round-trip the multi-shape so a schema edit that breaks
        // serialisation of one of the synth subset fields fails here.
        let reser = toml::to_string(&v).expect("ser");
        let v2: PackageMetadataNros = toml::from_str(&reser).expect("reparse");
        assert_eq!(v, v2);
    }

    /// `deny_unknown_fields` on `[system]` catches typos at the bringup
    /// surface.
    #[test]
    fn system_toml_rejects_unknown_field() {
        let raw = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0
mystery_knob = "no"
"#;
        let err = toml::from_str::<SystemToml>(raw)
            .expect_err("unknown field on [system] must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("mystery_knob") || msg.contains("unknown field"),
            "diagnostic: {msg}"
        );
    }
}

#[cfg(test)]
mod wcet_authoring_tests {
    use super::*;

    fn parse(toml_src: &str) -> SystemToml {
        toml::from_str(toml_src).expect("system.toml must parse")
    }

    const HEADER: &str = r#"
[system]
name = "demo"
rmw = "zenoh"
domain_id = 0
"#;

    fn with_profile(extra: &str) -> String {
        format!(
            r#"{HEADER}
[wcet.profiles.stm32f4-168mhz-release]
cpu = "cortex-m4f"
clock_hz = 168000000
profile = "release"
measured_at_commit = "a1b2c3d4e5f6"
counter_valid = true
source = "nros.wcet.measurements/1"
margin_percent = 20.0

[wcet.profiles.stm32f4-168mhz-release.boundaries]
"/perception/on_scan" = {{ min_observed_cycles = 41120, max_observed_cycles = 68940, iterations = 1000 }}
{extra}
"#
        )
    }

    /// The section parses at all — `SystemToml` carries `deny_unknown_fields`,
    /// so before this field existed a `[wcet]` block was a hard parse error and
    /// the authoring path could not exist.
    #[test]
    fn a_wcet_section_parses_and_resolves_for_the_selecting_target() {
        let sys = parse(&with_profile(
            "\n[wcet.select]\nflash-stm32f4-disco = \"stm32f4-168mhz-release\"",
        ));
        let profile = sys
            .wcet_profile_for("flash-stm32f4-disco")
            .expect("valid selection")
            .expect("a profile is selected");
        assert_eq!(profile.cpu, "cortex-m4f");
        // 68_940 * 1.20 = 82_728 cycles at 168 MHz
        let ms = profile.exec_ms("/perception/on_scan").expect("convertible");
        assert!((ms - 0.492_428_571).abs() < 1e-6, "got {ms}");
    }

    /// A target nobody selected for gets nothing, silently. Absent is the
    /// default and must not be an error.
    #[test]
    fn a_target_with_no_selection_gets_no_bounds_and_no_error() {
        let sys = parse(&with_profile(
            "\n[wcet.select]\nflash-stm32f4-disco = \"stm32f4-168mhz-release\"",
        ));
        assert_eq!(sys.wcet_profile_for("native").unwrap(), None);
    }

    #[test]
    fn no_wcet_section_at_all_is_not_an_error() {
        let sys = parse(HEADER);
        assert_eq!(sys.wcet_profile_for("native").unwrap(), None);
    }

    /// The failure this resolver exists for: a typo must NOT read as "this
    /// board has no measurements".
    #[test]
    fn a_selection_naming_an_unknown_profile_is_a_hard_error() {
        let sys = parse(&with_profile(
            "\n[wcet.select]\nflash-stm32f4-disco = \"stm32f4-168mhz-relase\"",
        ));
        match sys.wcet_profile_for("flash-stm32f4-disco") {
            Err(WcetSelectionError::UnknownProfile { target, profile }) => {
                assert_eq!(target, "flash-stm32f4-disco");
                assert_eq!(profile, "stm32f4-168mhz-relase");
            }
            other => panic!("a typo must be an error, got {other:?}"),
        }
    }

    /// A profile that exists and cannot be believed stops the build rather than
    /// evaporating into `None`.
    #[test]
    fn a_selected_profile_that_fails_validation_is_a_hard_error() {
        let src = with_profile("\n[wcet.select]\nflash-stm32f4-disco = \"stm32f4-168mhz-release\"")
            .replace("counter_valid = true", "counter_valid = false");
        let sys = parse(&src);
        match sys.wcet_profile_for("flash-stm32f4-disco") {
            Err(WcetSelectionError::InvalidProfile { errors, .. }) => {
                assert!(errors.contains(&WcetError::CounterNotValid), "{errors:?}");
            }
            other => panic!("an unbelievable profile must be an error, got {other:?}"),
        }
    }

    /// An observation with no margin and no explicit bound parses, validates,
    /// and yields NO exec_ms — RFC-0078's central refusal, reachable from
    /// authored TOML rather than only from Rust.
    #[test]
    fn an_authored_observation_without_a_bound_yields_no_exec_ms() {
        let src = with_profile("\n[wcet.select]\nflash-stm32f4-disco = \"stm32f4-168mhz-release\"")
            .replace("margin_percent = 20.0\n", "");
        let sys = parse(&src);
        let profile = sys
            .wcet_profile_for("flash-stm32f4-disco")
            .expect("still valid")
            .expect("still selected");
        assert_eq!(
            profile.exec_ms("/perception/on_scan"),
            None,
            "a high-water mark with no declared bound must not reach the scheduler"
        );
    }
}
