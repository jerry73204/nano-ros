use serde::{Deserialize, Serialize};

use super::schema::{
    DeadlinePolicy, EnvDecl, InterfaceRef, ParameterTable, QosProfile, RemapRule, SchedClass,
    SourceLocation,
};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NrosPlan {
    pub version: u32,
    pub system: String,
    pub trace: PlanTrace,
    pub components: Vec<PlanComponent>,
    pub instances: Vec<PlanInstance>,
    pub interfaces: Vec<PlanInterface>,
    pub sched_contexts: Vec<PlanSchedContext>,
    /// Phase 172.B — callback execution chains inferred from the topic
    /// dataflow graph (publisher topic → subscriber callback). Additive; old
    /// plans (v1) omit it and deserialize to an empty vec. Omitted from output
    /// when empty so chain-less plans stay byte-identical to v1.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub callback_chains: Vec<PlanCallbackChain>,
    /// Phase 172.C — callback groups derived from the chains (one
    /// mutually-exclusive group per chain; one reentrant singleton group per
    /// chain-less callback). Additive; old plans omit it and deserialize to
    /// an empty vec. Omitted from output when empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub callback_groups: Vec<PlanCallbackGroup>,
    /// Phase 172.A — managed-lifecycle (REP-2002) spec for the generated
    /// binary's node. Additive; absent ⇒ plain node (pre-172.A). Omitted from
    /// output when absent so non-lifecycle plans stay byte-identical.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lifecycle: Option<PlanLifecycle>,
    /// Phase 250 (Wave 3) — the ROS 2 parameter SERVER (the 6 get/set/list/
    /// describe services) for external query/update, WITHOUT persistence. The
    /// user writes normal `declare_parameter`/`get_parameter` in node source; a
    /// declared `[param_services]` axis lowers this on so the generated entry
    /// carries `nros/param-services` and the runtime registers the services on
    /// the first declared parameter. (It used to be distinguished from a
    /// `param_persistence` sibling; issue 0080 retired that, so the SERVER is
    /// now the only parameter capability a plan carries.)
    /// Additive; absent ⇒ no server, omitted so plans stay byte-identical.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub param_services: Option<PlanParamServices>,
    /// Issue 0259 — what the RFC-0052 realizer concluded while deriving this
    /// schedule, carried INTO the artifact rather than only onto stderr.
    ///
    /// A warning printed at bake time is gone the moment the terminal scrolls;
    /// the plan is the thing a user inspects afterwards with `nros explain`.
    /// Putting the realizer's verdicts here is what makes a derived schedule
    /// auditable — the feasibility check in particular, whose whole value is
    /// that it names the numbers it judged.
    ///
    /// Additive; absent ⇒ omitted, so a plan with no warnings stays
    /// byte-identical to one produced before this field existed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub sched_warnings: Vec<PlanSchedWarning>,
    /// Phase 250 (Wave 1) — E2E message-integrity (CRC + sequence gap/dup)
    /// capability. A declared `[safety]` block lowers this on; the generated
    /// entry then carries the `nros/safety-e2e` umbrella feature so the
    /// capability is compiled in (a *compile* dimension — embedded only pays
    /// the arena/CRC code size when selected). Additive; absent ⇒ no safety,
    /// omitted from output so plans stay byte-identical.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub safety: Option<PlanSafety>,
    /// Phase 172 — in-binary bridges: gateways that forward declared topics
    /// between the sessions in each bridge's `connect` list. The generator
    /// resolves each topic's type from `interfaces` and emits raw sub→pub
    /// forwarding across the sessions. Additive; absent ⇒ no bridges. Omitted
    /// from output when empty so non-bridge plans stay byte-identical.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bridges: Vec<PlanBridge>,
    /// Phase 211.E — `<executable>` declarations from the launch surface here
    /// as non-rmw "spawn" entries the deploy stage runs alongside the rmw
    /// `instances`. The parser records each `<executable>` as a `record.node`
    /// with `package=None`; the planner used to reject those as
    /// `missing-package`, but they're legal in ROS 2 launches (rviz, rosbag,
    /// robot_state_publisher's CLI wrapper, etc.) so the planner now surfaces
    /// them here. Additive: pre-211.E plans don't carry the field;
    /// `serde(default)` keeps them round-tripping unchanged. Omitted from
    /// output when empty so non-executable plans stay byte-identical.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub executables: Vec<PlanExecutable>,
    pub build: PlanBuildOptions,
}

/// One realizer verdict about one node's derived schedule (issue 0259).
///
/// `dim` names the requirement dimension the verdict is about (`deadline`,
/// `budget`, `feasibility`, …) so a reader can tell a weakened guarantee from
/// an impossible declaration. `reason` carries the NUMBERS — a verdict without
/// its inputs is an assertion, and this schema exists so the derivation can be
/// checked rather than believed.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PlanSchedWarning {
    pub node: String,
    pub dim: String,
    pub reason: String,
}

impl NrosPlan {
    /// Phase 261 — is the declared capability axis (`capability_resolver::Capability
    /// .declared`, e.g. `"safety"` / `"param_services"`) enabled in this plan? The
    /// planner only emits a capability block when its axis is enabled, so presence
    /// of the typed field IS the enabled state. Mirrors `SystemToml::capability_enabled`
    /// so `generate` can registry-loop the Rust feature lowering instead of hardcoding
    /// per-axis keys. Unknown axis ⇒ `false`.
    pub fn capability_enabled(&self, declared: &str) -> bool {
        match declared {
            "safety" => self.safety.is_some(),
            "param_services" => self.param_services.is_some(),
            _ => false,
        }
    }
}

/// Phase 172 — a topic-forwarding gateway: relay the `topics` (raw CDR) between
/// the sessions named in `connect` (≥2). Mirrors the root `[[bridge]]`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanBridge {
    pub name: String,
    pub connect: Vec<PlanBridgeEndpoint>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub topics: Vec<String>,
    /// phase-267 W1c — `true` ⇒ relay both directions (echo-suppressed). Default
    /// one-way (`connect[0]`→`connect[1]`). Omitted when false (back-compat).
    #[serde(default, skip_serializing_if = "core::ops::Not::not")]
    pub bidirectional: bool,
}

/// One session a bridge connects (`rmw` + ROS `domain` + optional `locator`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanBridgeEndpoint {
    pub rmw: String,
    #[serde(default)]
    pub domain: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
}

/// Phase 250 (Wave 3) — the declared parameter-server capability. A pure
/// on/off toggle (its presence is the enable signal); no sub-config, since the
/// parameters themselves are declared in normal node source, not config.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanParamServices {}

/// Phase 250 (Wave 1) — declared E2E-safety capability. `crc` selects whether
/// the CRC-32 check is wired in addition to sequence gap/dup tracking (default
/// true). Presence of the plan field is the enable signal; the `crc` flag is
/// carried for the Layer-2 codegen wave (which lowers it into the generated
/// node's `.safety()` subscriptions). Today only the presence matters — it
/// lowers to the `nros/safety-e2e` umbrella feature.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSafety {
    #[serde(default = "default_true")]
    pub crc: bool,
}

fn default_true() -> bool {
    true
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanTrace {
    pub system_config: String,
    pub launch_record: String,
    pub generated_by: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanComponent {
    pub id: String,
    pub package: String,
    pub component: String,
    pub language: String,
    pub source_metadata: String,
    pub component_config: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanInstance {
    pub id: String,
    pub component: String,
    pub package: String,
    pub executable: String,
    pub launch_name: String,
    pub namespace: String,
    /// Phase 211.B — entity kind: `"node"` for a plain `<node>`,
    /// `"container"` for a `<node_container>` (the spawned binary
    /// hosting composable children in-process), or `"composable_node"`
    /// for a `<composable_node>` / `<load_composable_node>` child.
    /// Additive on the schema: defaults to `"node"` so pre-211.B plans
    /// round-trip unchanged.
    #[serde(
        default = "PlanInstance::default_kind",
        skip_serializing_if = "PlanInstance::is_default_kind"
    )]
    pub kind: String,
    /// Phase 211.B — when `kind == "composable_node"`, the id of the
    /// parent container instance (the `<node_container>` that hosts
    /// this composable in-process). `None` for `kind = "node"` and for
    /// `kind = "container"` (the container itself has no parent).
    /// Additive: `#[serde(default)]` so pre-211.B plans round-trip
    /// unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub container_id: Option<String>,
    /// Phase 225.N — optional launch-instance provenance used by generated-ID
    /// planning. The legacy `trace.launch_record_entity` string remains the
    /// compact required breadcrumb; this richer shape lets later planners carry
    /// a launch declaration index and the resolved package/exec/name tuple
    /// without changing existing plan fixtures.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launch_instance: Option<PlanLaunchInstance>,
    pub remaps: Vec<RemapRule>,
    /// Phase 211.E — `<set_env>` / `<env>` declarations the launch attached
    /// to this instance. Empty when nothing is declared. Additive on the
    /// schema: `#[serde(default)]` so existing nros-plan.json files written
    /// before the field was emitted round-trip unchanged.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub env: Vec<EnvDecl>,
    pub nodes: Vec<PlanNode>,
    pub callbacks: Vec<PlanCallback>,
    pub parameters: Vec<PlanParameter>,
    /// Phase 211.H — per-topic QoS overrides lowered from
    /// `qos_overrides.<topic>.<role>.<policy>` launch params. Split out of
    /// `parameters` by the planner. Additive: `#[serde(default)]` +
    /// `skip_serializing_if` so plans written before 211.H round-trip unchanged.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub qos_overrides: Vec<QosOverride>,
    pub sched_bindings: Vec<PlanSchedBinding>,
    pub trace: InstanceTrace,
}

impl PlanInstance {
    fn default_kind() -> String {
        "node".to_string()
    }

    fn is_default_kind(kind: &str) -> bool {
        kind == "node"
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstanceTrace {
    pub launch_record_entity: String,
    pub source_metadata: String,
}

/// Phase 225.N — source launch row that produced a planned instance. All
/// fields are optional because older parser records only expose
/// `InstanceTrace::launch_record_entity`; newer launch parsers can fill a
/// declaration index and raw ROS launch selectors.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanLaunchInstance {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub index: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub record_entity: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub package: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub executable: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launch_name: Option<String>,
}

/// Phase 211.E — a non-rmw command the deploy stage spawns alongside the
/// rmw `instances`. Sourced from `<executable cmd="…">` in the launch
/// (parser writes it as a `record.node` with `package=None`, `cmd` carries
/// the fully-resolved command).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanExecutable {
    /// Synthesized, stable id (`executable.<sanitized-name>.<index>`).
    pub id: String,
    /// Display / record name (the launch's `name=` attribute, or
    /// `"executable"` when unnamed).
    pub name: String,
    /// Resolved namespace (defaults to `/`).
    pub namespace: String,
    /// Full argv as the parser resolved it. `cmd[0]` is the executable
    /// path; `cmd[1..]` are its arguments (post-substitution).
    pub cmd: Vec<String>,
    /// Just the `<arg>` children, separately, in declaration order.
    /// Empty when the launch declared none.
    #[serde(default)]
    pub args: Vec<String>,
    /// `<set_env>` / `<env>` declarations attached to this entry (Phase
    /// 211.E env-propagation). Empty when nothing is declared.
    #[serde(default)]
    pub env: Vec<EnvDecl>,
    pub trace: ExecutableTrace,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExecutableTrace {
    pub launch_record_entity: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanNode {
    pub id: String,
    pub source_node: String,
    /// Phase 225.N — source-authored fallback ROS graph name. Launch `name=`
    /// still overrides it; this field lets the planner/auditor explain when a
    /// launch instance omitted `name=` and fell back to source metadata.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_default_name: Option<String>,
    /// Phase 225.N — declaration-order slot local to the source metadata
    /// artifact. Future generated IDs are assigned from launch instance +
    /// declaration slot, not user-authored stable strings.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    /// Phase 225.N — source declaration location for generated-ID diagnostics.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<SourceLocation>,
    pub resolved_name: String,
    pub namespace: String,
    pub entities: Vec<PlanEntity>,
    /// Phase 172.K.5 — ROS domain this node is bound to (from a root
    /// `[system].[[domain]]` group). `None` ⇒ the system default domain. When
    /// nodes span >1 distinct domain the generator opens a session per domain
    /// (`SESSION_SPECS` + `open_multi`) and routes each node to its slot.
    /// Additive + skip-when-absent so single-domain plans round-trip unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_id: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "role", rename_all = "snake_case", deny_unknown_fields)]
pub enum PlanEntity {
    Publisher {
        id: String,
        source_entity: String,
        resolved_name: String,
        interface: InterfaceRef,
        qos: QosProfile,
        trace: EntityTrace,
    },
    Subscriber {
        id: String,
        source_entity: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        callback: Option<String>,
        resolved_name: String,
        interface: InterfaceRef,
        qos: QosProfile,
        trace: EntityTrace,
    },
    Timer {
        id: String,
        source_entity: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        callback: Option<String>,
        /// Legacy, truncating — kept because consumers read it. The
        /// authoritative period is `period_us` (issue #505).
        period_ms: u64,
        /// Issue #505 — the period at the executor's own resolution. Optional
        /// so a plan written before #505 still deserializes; absent means
        /// "`period_ms` widened", which is what the planner now emits anyway.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        period_us: Option<u64>,
        trace: EntityTrace,
    },
    ServiceServer {
        id: String,
        source_entity: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        callback: Option<String>,
        resolved_name: String,
        interface: InterfaceRef,
        qos: Option<QosProfile>,
        trace: EntityTrace,
    },
    ServiceClient {
        id: String,
        source_entity: String,
        resolved_name: String,
        interface: InterfaceRef,
        qos: Option<QosProfile>,
        trace: EntityTrace,
    },
    ActionServer {
        id: String,
        source_entity: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        callback: Option<String>,
        resolved_name: String,
        interface: InterfaceRef,
        qos: Option<QosProfile>,
        trace: EntityTrace,
    },
    ActionClient {
        id: String,
        source_entity: String,
        resolved_name: String,
        interface: InterfaceRef,
        qos: Option<QosProfile>,
        trace: EntityTrace,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EntityTrace {
    pub source_artifact: SourceLocation,
    pub manifest_endpoint: Option<String>,
    /// Phase 225.N — declaration-order slot local to the source node/component.
    /// Kept in trace so every entity variant gets the generated-ID provenance
    /// without expanding each enum arm.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCallback {
    pub id: String,
    pub source_callback: String,
    /// Phase 225.N — declaration-order slot local to the source metadata
    /// artifact. Future codegen can use this as the stable callback reference
    /// while old plans continue to use `source_callback`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub group: String,
    pub sched_context: String,
    pub source: SourceLocation,
}

/// Phase 172.B — an inferred (or overridden) callback execution chain: an
/// ordered sequence of callbacks where each consumes the topic the previous
/// produced. The head is a chain entry (a timer, or a subscriber whose topic
/// has no in-system publisher); `links` records the producing topic for each
/// edge so the chain is auditable. 172.C derives callback groups from these
/// chains; 172.G assigns tiers per chain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCallbackChain {
    /// Stable chain id, e.g. `chain/<head-callback-id>`.
    pub id: String,
    /// Ordered callback ids from head to tail.
    pub callbacks: Vec<String>,
    /// One entry per edge between consecutive `callbacks`.
    pub links: Vec<PlanChainLink>,
    /// `true` when the planner inferred this chain from the topic graph;
    /// `false` when it came from an explicit `[[chain]]` override.
    pub inferred: bool,
}

/// One dataflow edge in a [`PlanCallbackChain`]: `from` publishes `topic`,
/// which `to` subscribes to.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanChainLink {
    pub from: String,
    pub to: String,
    pub topic: String,
}

/// Phase 172.C — dispatch concurrency class of a [`PlanCallbackGroup`],
/// mirroring rclcpp's two callback-group kinds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CallbackGroupKind {
    /// Members never run concurrently with one another (serialized
    /// dispatch) — the safe default for dataflow-coupled pipeline stages
    /// that may share state.
    MutuallyExclusive,
    /// Members may run concurrently — inferred for callbacks with no
    /// detected dataflow coupling.
    Reentrant,
}

/// Phase 172.C — an inferred (or overridden) callback group. Each callback
/// belongs to exactly one group; the group's [`CallbackGroupKind`] decides
/// whether its members serialize or may run concurrently. Derived from the
/// 172.B callback chains: every chain becomes one mutually-exclusive group
/// (its stages serialize), and every callback outside any chain becomes its
/// own reentrant group (no coupling detected → concurrent-safe). 172.G
/// assigns scheduling tiers on top of this grouping.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCallbackGroup {
    /// Stable group id, e.g. `group/<chain-head>` or `group/<callback-id>`.
    pub id: String,
    /// Serialize vs concurrent dispatch.
    pub kind: CallbackGroupKind,
    /// Callback ids that belong to this group (chain order for chain
    /// groups; a single callback for reentrant groups).
    pub callbacks: Vec<String>,
    /// `true` when the planner inferred this group from the chains;
    /// `false` when it came from an explicit `[[group]]` override.
    pub inferred: bool,
    /// Phase 434 — how a MODEL-derived record settled this group, when one
    /// was consulted: `"default"` (the node declared nothing, so the
    /// contract's default applied) or `"exclusive"` (an explicit
    /// `concurrency:` set named these callbacks). `None` for a chain group
    /// or for any group produced from a legacy record with no
    /// `node_concurrency` key — the planner's own inference is all there is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declared: Option<String>,
}

/// Phase 172.A — boot autostart policy for a managed-lifecycle (REP-2002) node.
/// The generated runtime registers the five `~/change_state` / `~/get_state`
/// services and then drives the node to this state at boot; `ros2 lifecycle`
/// can drive it further at runtime.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LifecycleAutostart {
    /// Register the services but leave the node `Unconfigured` — every
    /// transition is externally driven (`ros2 lifecycle set`).
    None,
    /// Auto-`configure` to `Inactive` at boot.
    Configure,
    /// Auto-`configure` then `activate` to `Active` at boot.
    Active,
}

/// Phase 172.A — managed-lifecycle spec for the generated binary. Its presence
/// marks the binary's node as managed; absence keeps the pre-172.A behaviour
/// (a plain node brought up once at boot). The runtime models one lifecycle
/// state machine per executor, so this is currently system-level; per-instance
/// (multiple managed nodes in one binary) is a deferred runtime extension.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanLifecycle {
    /// State the generated runtime drives the node to at boot.
    pub autostart: LifecycleAutostart,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanParameter {
    pub node: String,
    pub name: String,
    pub value: super::schema::ParameterValue,
    pub source: ParameterSource,
}

/// Phase 211.H — a per-topic QoS override, lowered from a ROS 2
/// `qos_overrides.<topic>.<role>.<policy>` launch parameter. ROS expresses
/// these as specially-named `<param>`s; the planner splits them out of the
/// generic `parameters` table into this typed list so the codegen can emit the
/// matching `QoSProfile` on the publisher/subscriber for `<topic>` instead of
/// the API default. `role` is `"publisher"` / `"subscription"`; `policy` is
/// `reliability` / `durability` / `history` / `depth` (others pass through for
/// forward-compat). One entry per `(topic, role, policy)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QosOverride {
    pub topic: String,
    pub role: String,
    pub policy: String,
    pub value: super::schema::ParameterValue,
    pub source: ParameterSource,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ParameterSource {
    pub kind: ParameterSourceKind,
    pub artifact: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParameterSourceKind {
    SourceDefault,
    ComponentConfig,
    SystemOverlay,
    Launch,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSchedBinding {
    pub callback: String,
    pub context: String,
    pub priority: Option<u8>,
    pub source: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanInterface {
    pub id: String,
    pub interface: InterfaceRef,
    pub used_by: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSchedContext {
    pub id: String,
    pub executor: String,
    /// Schema-level class; generated code maps this to the runtime scheduler class.
    pub class: SchedClass,
    pub priority: Option<u8>,
    pub period_ms: Option<u64>,
    pub budget_ms: Option<u64>,
    pub deadline_ms: Option<u64>,
    pub deadline_policy: DeadlinePolicy,
    pub stack_size: Option<u32>,
    pub core: Option<u32>,
    pub task: Option<String>,
}

/// Phase 173.5 — physical transport a `[[transport]]` entry selects.
/// The kind always comes from `nros.toml`; the per-kind value (ip /
/// baudrate / device) lands wherever that platform's net stack reads it
/// (board `Config` for `NanoRosOwned`, an RTOS config fragment for
/// `RtosOwned` — see [`super::generate`] / Phase 173.7).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TransportKind {
    Ethernet,
    Wifi,
    Serial,
    Can,
}

impl TransportKind {
    /// The board crate Cargo feature that enables this transport.
    pub fn cargo_feature(self) -> &'static str {
        match self {
            TransportKind::Ethernet => "ethernet",
            TransportKind::Wifi => "wifi",
            TransportKind::Serial => "serial",
            TransportKind::Can => "can",
        }
    }
}

/// phase-206 W5 — the one message for a `[[transport]].interfaces` list.
///
/// nano-ros does not model NICs. The upstream `ros-launch-manifest` schema
/// still carries a `TransportBlock.interfaces` field, so the value can still
/// ARRIVE from a `system.toml`; it stops here, loudly, rather than being
/// copied into a plan nothing reads (which is what it did until W5). The
/// caller prefixes the offending transport.
///
/// Shared so the ingest path and any future reader say the same thing —
/// a second spelling of this remedy is how the class comes back.
pub const RETIRED_INTERFACES_REMEDY: &str = "`interfaces` is not a nano-ros \
     setting (retired phase-206 W5): nano-ros does not model NICs. Bind the \
     middleware to an interface in the middleware's own config — CycloneDDS \
     `<General><Interfaces><NetworkInterface name=\"…\"/></Interfaces>` via \
     `CYCLONEDDS_URI`, or a zenoh config's `listen`/`connect` endpoints and \
     `scouting/multicast/interface`. Remove the key from the transport.";

/// Phase 173.5 — one transport⟷RMW binding from `nros.toml`'s
/// `[[transport]]` array. Two or more entries put the build in **bridge
/// mode** (each transport runs its own RMW session;
/// `Executor::open_multi` consumes the resulting `SessionSpec`s).
///
/// `rmw`/`locator` are optional per entry; when absent they fall back to
/// the top-level `build.rmw` / the platform default. The generator —
/// not hand-written code — turns these into the board transport
/// feature(s), the per-transport `Config` values, and the RMW deps.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanTransport {
    pub kind: TransportKind,
    /// Stable transport id used to bind a node/instance to this session
    /// (`SystemComponent.transport`). `None` ⇒ defaults to `rmw` (works when
    /// each transport has a distinct rmw). Phase 172.K.
    #[serde(default)]
    pub id: Option<String>,
    /// IPv4 CIDR (`"10.0.2.50/24"`) or `"dhcp"` — ethernet/wifi only.
    pub ip: Option<String>,
    /// WiFi SSID — wifi only. Phase 172.K.
    #[serde(default)]
    pub ssid: Option<String>,
    /// WiFi password — wifi only. Phase 172.K.
    #[serde(default)]
    pub password: Option<String>,
    /// Ethernet MAC (`"02:00:00:00:00:01"`) — ethernet only. `None` ⇒
    /// the board's fixed/fused MAC. (Phase 172.J — replaces
    /// `config.toml`'s `[network].mac`.)
    #[serde(default)]
    pub mac: Option<String>,
    /// Default IPv4 gateway (`"10.0.2.2"`) — ethernet only. `None` ⇒ a
    /// flat link with no gateway. (Phase 172.J — replaces
    /// `config.toml`'s `[network].gateway`.)
    #[serde(default)]
    pub gateway: Option<String>,
    // phase-206 W5 — there is deliberately no `interfaces` field. nano-ros does
    // not model NICs: see `RETIRED_INTERFACES_REMEDY`.
    /// Device handle (`"UART0"`, `"CAN0"`) — serial / can only.
    pub device: Option<String>,
    /// Line rate (serial baud / CAN bitrate) — serial / can only.
    pub baudrate: Option<u32>,
    /// RMW that rides this transport. `None` ⇒ inherit `build.rmw`.
    pub rmw: Option<String>,
    /// Zenoh/DDS locator seeding this transport's session. `None` ⇒
    /// platform / env default.
    pub locator: Option<String>,
    /// Phase 172 WP-B — ROS domain this transport's session joins. `None` ⇒
    /// the build/system default (0). Lets a bridge open same-rmw sessions on
    /// distinct domains (multi-domain in-binary); the session = (rmw, locator,
    /// domain). Additive; skip-when-absent so single-domain plans round-trip
    /// unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanBuildOptions {
    pub target: String,
    pub board: String,
    pub rmw: String,
    pub profile: String,
    pub features: Vec<String>,
    pub cfg: ParameterTable,
    /// Phase 173.5 — `nros.toml` `[[transport]]` entries. Empty ⇒
    /// zero-config single-transport build (board default transport +
    /// the single linked RMW). Defaulted so pre-173.5 plans parse;
    /// skip-when-empty so the stable pretty fixtures (zero-config
    /// builds) round-trip without an empty `"transports": []`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub transports: Vec<PlanTransport>,
    /// Phase 255 Wave 5 — the extra RMW backends a single binary links to host
    /// cross-RMW `[[bridge]]`s (the union of every bridged `[[domain]]`'s RMW,
    /// from `system.toml`). Empty ⇒ single-RMW build (just `rmw`), byte-identical
    /// to pre-255. Supersedes the `[[transport]].rmw` overlay multi-RMW path —
    /// bridges are topology and belong in `system.toml`, read by both codegen
    /// paths. `rmw_set` unions this with `rmw` / `[[transport]].rmw`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bridged_rmws: Vec<String>,
    /// Phase 204.15 — coherent size/speed *intent* (`size` | `speed` |
    /// `balanced` | `debug`). The generated package build fans it out to RUSTFLAGS (`-C
    /// opt-level/lto/codegen-units/strip`) on top of the cargo profile, so one
    /// knob tunes the Rust layer without per-crate `[profile.*]` edits. `None` ⇒
    /// today's behaviour (profile only). Per-layer overrides + the cc/CMake
    /// fan-out are tracked follow-ups (204.15).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub optimize: Option<String>,
    /// Phase 204.15 (increment 2) — per-layer `[build.cargo]` override table.
    /// Refines the Rust/cargo layer *on top of* the `optimize` baseline
    /// (precedence: `optimize` baseline → `[build.cargo]` field). Lets a build
    /// tune one cargo-profile field without disturbing the coherent intent —
    /// e.g. `optimize = "size"` + `[build.cargo] debug = true, strip = false`
    /// keeps Rust debuginfo while everything else stays size-tuned. `None` ⇒
    /// the `optimize` baseline alone.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cargo: Option<PlanCargoOverrides>,
    /// Phase 204.15 (increment 3) — per-layer `[build.cc]` override for the C/C++
    /// layer. `debug`/`cflags` are exported as `CFLAGS`/`CXXFLAGS` which `cc-rs`
    /// *appends* to its computed flags (every zenoh-pico/XRCE/net.c/lwIP
    /// `cc::Build`, no build.rs edit): `debug = true` adds `-g` without disturbing
    /// the opt level → the C-side of the "debug one layer" case. `opt_level` →
    /// `NROS_CC_OPT` (build scripts that honor it override their hardcoded opt; 204.9).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cc: Option<PlanCcOverrides>,
    /// Phase 195.C — workspace root, populated at generate time (NOT part of
    /// the plan wire format). Lets `profile()` load board descriptors from
    /// `<workspace>/packages/boards/*/nros-board.toml` so the CLI carries no
    /// baked-in board layout. `None` outside a generate run.
    #[serde(skip)]
    pub workspace_root: Option<std::path::PathBuf>,
}

/// Phase 204.15 (increment 2) — `[build.cargo]` per-layer override fields. Each
/// maps to a cargo `[profile.release]` key and, when present, replaces the value
/// the `optimize` intent produced. Values are kept as raw JSON so `opt_level`
/// accepts both `3` (number) and `"z"` (string); `lto`/`strip` accept a bool or
/// string; rendering picks the right TOML literal. Unknown JSON shapes are
/// dropped at render time (never panic).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCargoOverrides {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub opt_level: Option<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lto: Option<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub debug: Option<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub strip: Option<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub codegen_units: Option<serde_json::Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub panic: Option<serde_json::Value>,
}

/// Phase 204.15 (increment 3) — `[build.cc]` per-layer override for the C/C++
/// toolchain. Applied via `CFLAGS`/`CXXFLAGS` env (cc-rs appends) + `NROS_CC_OPT`,
/// so it reaches every `cc::Build` without a build.rs edit.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCcOverrides {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub debug: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub opt_level: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cflags: Vec<String>,
}

impl PlanBuildOptions {
    /// `true` when more than one transport is declared — the build runs
    /// multiple RMW sessions via `Executor::open_multi` (bridge mode).
    pub fn is_bridge(&self) -> bool {
        self.transports.len() > 1
    }

    /// Phase 204.7 — `true` when the declared transports carry **no IP link**
    /// (every transport is serial/CAN, none ethernet/wifi). The generated
    /// `.cargo/config.toml` then bakes `NROS_LINK_IP=0` so the zenoh-pico /
    /// XRCE TCP+UDP link C is dropped (with `--gc-sections`, ~‑33 KB BSS on a
    /// bare-metal serial build). Empty transports ⇒ `false` (zero-config build
    /// keeps the board default IP link).
    pub fn drops_ip_link(&self) -> bool {
        !self.transports.is_empty()
            && self
                .transports
                .iter()
                .all(|t| matches!(t.kind, TransportKind::Serial | TransportKind::Can))
    }

    /// Validate the `[[transport]]` array against per-kind field rules.
    /// Returns the list of human-readable problems (empty ⇒ valid) so
    /// the caller can surface them all at once rather than one per run.
    pub fn validate_transports(&self) -> Vec<String> {
        let mut problems = Vec::new();
        for (i, t) in self.transports.iter().enumerate() {
            let at = format!("transport[{i}] (kind = {:?})", t.kind);
            match t.kind {
                TransportKind::Ethernet => {
                    if t.device.is_some() || t.baudrate.is_some() {
                        problems.push(format!("{at}: `device`/`baudrate` are serial/can-only"));
                    }
                    if t.ssid.is_some() || t.password.is_some() {
                        problems.push(format!("{at}: `ssid`/`password` are wifi-only"));
                    }
                }
                TransportKind::Wifi => {
                    // wifi carries ssid/password (+ optional static ip/gateway);
                    // mac is ethernet-only, device/baudrate are serial/can-only.
                    if t.device.is_some() || t.baudrate.is_some() {
                        problems.push(format!("{at}: `device`/`baudrate` are serial/can-only"));
                    }
                    if t.mac.is_some() {
                        problems.push(format!("{at}: `mac` is ethernet-only"));
                    }
                }
                TransportKind::Serial | TransportKind::Can => {
                    if t.ip.is_some() {
                        problems.push(format!("{at}: `ip` is ethernet/wifi-only"));
                    }
                    if t.mac.is_some() {
                        problems.push(format!("{at}: `mac` is ethernet-only"));
                    }
                    if t.gateway.is_some() {
                        problems.push(format!("{at}: `gateway` is ethernet/wifi-only"));
                    }
                    if t.ssid.is_some() || t.password.is_some() {
                        problems.push(format!("{at}: `ssid`/`password` are wifi-only"));
                    }
                }
            }
        }
        problems
    }
}

#[cfg(test)]
mod transport_tests {
    use super::*;

    fn build_with(transports_json: &str) -> PlanBuildOptions {
        let json = format!(
            r#"{{
                "target": "thumbv7m-none-eabi",
                "board": "baremetal",
                "rmw": "zenoh",
                "profile": "release",
                "features": [],
                "cfg": {{}}{transports_json}
            }}"#
        );
        serde_json::from_str(&json).expect("PlanBuildOptions parses")
    }

    #[test]
    fn pre_173_5_plan_without_transports_parses_to_empty() {
        let build = build_with("");
        assert!(build.transports.is_empty());
        assert!(!build.is_bridge());
        assert!(build.validate_transports().is_empty());
    }

    #[test]
    fn build_cc_override_parses() {
        // Phase 204.15 inc 3 — `[build.cc]` deserializes; absent ⇒ None.
        assert!(build_with("").cc.is_none());
        let json = r#"{
            "target": "x", "board": "native", "rmw": "zenoh", "profile": "release",
            "features": [], "cfg": {},
            "cc": { "debug": true, "opt_level": "s", "cflags": ["-fno-plt"] }
        }"#;
        let b: PlanBuildOptions = serde_json::from_str(json).expect("parses");
        let cc = b.cc.expect("cc present");
        assert_eq!(cc.debug, Some(true));
        assert_eq!(cc.opt_level.as_deref(), Some("s"));
        assert_eq!(cc.cflags, vec!["-fno-plt".to_string()]);
    }

    #[test]
    fn single_ethernet_transport_parses_and_validates() {
        let build = build_with(
            r#",
            "transports": [
                { "kind": "ethernet", "ip": "10.0.2.50/24", "rmw": "zenoh", "locator": "tcp/10.0.2.2:7447" }
            ]"#,
        );
        assert_eq!(build.transports.len(), 1);
        assert!(!build.is_bridge());
        assert_eq!(build.transports[0].kind, TransportKind::Ethernet);
        assert_eq!(build.transports[0].kind.cargo_feature(), "ethernet");
        assert_eq!(build.transports[0].ip.as_deref(), Some("10.0.2.50/24"));
        assert!(build.validate_transports().is_empty());
    }

    #[test]
    fn two_transports_are_bridge_mode() {
        let build = build_with(
            r#",
            "transports": [
                { "kind": "ethernet", "ip": "dhcp", "rmw": "zenoh" },
                { "kind": "serial", "device": "UART0", "baudrate": 115200, "rmw": "cyclonedds" }
            ]"#,
        );
        assert!(build.is_bridge());
        assert_eq!(build.transports[1].kind.cargo_feature(), "serial");
        assert_eq!(build.transports[1].baudrate, Some(115200));
        assert!(build.validate_transports().is_empty());
    }

    #[test]
    fn mismatched_transport_fields_are_reported() {
        // ethernet with a baudrate, serial with an ip — both wrong.
        let build = build_with(
            r#",
            "transports": [
                { "kind": "ethernet", "baudrate": 9600 },
                { "kind": "serial", "ip": "10.0.0.1/24", "device": "UART0" }
            ]"#,
        );
        let problems = build.validate_transports();
        assert_eq!(problems.len(), 2, "both mismatches reported: {problems:?}");
    }

    #[test]
    fn ethernet_mac_and_gateway_parse_and_validate() {
        // Phase 172.J — mac + gateway on an ethernet transport.
        let build = build_with(
            r#",
            "transports": [
                { "kind": "ethernet", "ip": "10.0.2.50/24",
                  "mac": "02:00:00:00:00:01", "gateway": "10.0.2.2" }
            ]"#,
        );
        assert_eq!(
            build.transports[0].mac.as_deref(),
            Some("02:00:00:00:00:01")
        );
        assert_eq!(build.transports[0].gateway.as_deref(), Some("10.0.2.2"));
        assert!(build.validate_transports().is_empty());
    }

    #[test]
    fn mac_and_gateway_are_ethernet_only() {
        // Phase 172.J — serial transport rejects mac + gateway.
        let build = build_with(
            r#",
            "transports": [
                { "kind": "serial", "device": "UART0", "baudrate": 115200,
                  "mac": "02:00:00:00:00:01", "gateway": "10.0.2.2" }
            ]"#,
        );
        let problems = build.validate_transports();
        assert_eq!(
            problems.len(),
            2,
            "mac + gateway both rejected: {problems:?}"
        );
    }

    #[test]
    fn wifi_transport_parses_with_ssid_password_and_id() {
        // Phase 172.K.4 — wifi kind + ssid/password + transport id.
        let build = build_with(
            r#",
            "transports": [
                { "kind": "wifi", "id": "wlan", "ssid": "Net", "password": "pw",
                  "ip": "10.0.0.50/24", "rmw": "zenoh" }
            ]"#,
        );
        assert_eq!(build.transports[0].kind, TransportKind::Wifi);
        assert_eq!(build.transports[0].kind.cargo_feature(), "wifi");
        assert_eq!(build.transports[0].id.as_deref(), Some("wlan"));
        assert_eq!(build.transports[0].ssid.as_deref(), Some("Net"));
        assert!(build.validate_transports().is_empty());
    }

    #[test]
    fn ssid_password_are_wifi_only() {
        // Phase 172.K.4 — ethernet + serial reject ssid/password.
        let build = build_with(
            r#",
            "transports": [
                { "kind": "ethernet", "ssid": "Net" },
                { "kind": "serial", "device": "UART0", "password": "pw" }
            ]"#,
        );
        let problems = build.validate_transports();
        assert_eq!(problems.len(), 2, "ssid + password rejected: {problems:?}");
    }

    /// phase-206 W5 — the NIC vocabulary is gone from the plan schema, and
    /// `deny_unknown_fields` makes a plan that still carries it fail LOUDLY
    /// rather than parsing to a value nothing reads (which is what the field
    /// did for its whole life: parsed, validated, serialized, read by nobody).
    ///
    /// This replaces the three tests that asserted the field round-tripped;
    /// without it, re-adding `interfaces` would look like a green refactor.
    #[test]
    fn a_transport_naming_interfaces_is_rejected() {
        let json = r#"{
            "target": "x", "board": "native", "rmw": "zenoh",
            "profile": "release", "features": [], "cfg": {},
            "transports": [
                { "kind": "ethernet", "ip": "10.0.2.50/24",
                  "interfaces": ["eth0", "eth1"] }
            ]
        }"#;
        let err = serde_json::from_str::<PlanBuildOptions>(json)
            .expect_err("a plan naming `interfaces` must not parse")
            .to_string();
        assert!(err.contains("interfaces"), "{err}");
    }

    #[test]
    fn unknown_transport_kind_is_rejected() {
        let json = r#"{
            "target": "x", "board": "native", "rmw": "zenoh",
            "profile": "release", "features": [], "cfg": {},
            "transports": [ { "kind": "bluetooth" } ]
        }"#;
        assert!(serde_json::from_str::<PlanBuildOptions>(json).is_err());
    }
}
