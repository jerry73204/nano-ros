use serde::{Deserialize, Serialize};

use super::schema::{InterfaceRef, ParameterValue, QosProfile, SourceLocation, SourceName};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceMetadata {
    pub version: u32,
    pub package: String,
    pub component: String,
    pub language: ComponentLanguage,
    pub executable: Option<String>,
    pub exported_symbol: Option<String>,
    pub nodes: Vec<SourceNode>,
    pub callbacks: Vec<SourceCallback>,
    pub parameters: Vec<SourceParameter>,
    pub trace: SourceMetadataTrace,
    /// phase-307 W2 — how this sidecar was produced and from what.
    ///
    /// Absent on a raw harness emission and on the hand-written test fixtures;
    /// the refresh step stamps it. A consumer reads it to tell a sidecar that
    /// describes the current source from museum data.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub provenance: Option<SourceMetadataProvenance>,
}

/// phase-307 W2 — sidecar provenance stamp.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceMetadataProvenance {
    /// Content digest of the component sources the sidecar was derived from.
    /// Content-addressed, NOT mtime-based — see `metadata_refresh`.
    pub inputs_digest: String,
    /// Producer identity + version (`nros 0.5.0`).
    pub generator: String,
}

/// The language a component is written in, as recorded in source metadata.
///
/// phase-432 W2.1 — an ALIAS for the one enumeration. The serde repr is
/// unchanged (`snake_case`: `rust` / `c` / `cpp`), which matters because this
/// field is written to metadata files on users' disks;
/// `nros_lang`'s `serde_repr_is_the_on_disk_contract` pins those spellings.
pub type ComponentLanguage = nros_lang::Language;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceNode {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_default_name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<SourceLocation>,
    pub unresolved_name: SourceName,
    pub namespace: Option<String>,
    pub publishers: Vec<SourcePublisher>,
    pub subscribers: Vec<SourceSubscriber>,
    pub timers: Vec<SourceTimer>,
    pub services: Vec<SourceService>,
    pub actions: Vec<SourceAction>,
    /// issue 0900 — the CLIENT halves, which size the executor arena.
    ///
    /// `#[serde(default)]` because this struct is `deny_unknown_fields` and a
    /// sidecar emitted before these existed must still parse. An empty vector
    /// from an OLD sidecar is indistinguishable from a component that genuinely
    /// has no clients, which is why the consumer keys on the sidecar's presence
    /// rather than on the count being zero — see `entity_facts`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub action_clients: Vec<SourceClient>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub service_clients: Vec<SourceClient>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourcePublisher {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub unresolved_topic: SourceName,
    pub interface: InterfaceRef,
    pub qos: QosProfile,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceSubscriber {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub unresolved_topic: SourceName,
    pub interface: InterfaceRef,
    pub qos: QosProfile,
    pub callback: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub callback_slot: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceTimer {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    /// Truncating. Kept because every committed plan fixture and the planner's
    /// JSON path read it, and #505 deliberately kept emitting it.
    pub period_ms: u64,
    /// Issue #505 — the same period at the executor's own resolution.
    ///
    /// ADDITIVE: `write_timer_json` emits both, so this is `Option` and
    /// defaulted rather than replacing `period_ms`. It is optional for one more
    /// reason — the hand-written fixtures under `tests/fixtures/orchestration/`
    /// predate #505 and legitimately carry only the millisecond field.
    ///
    /// This struct is `deny_unknown_fields`, which is why the addition was not
    /// merely ignored: every source-metadata parse failed outright with
    /// `unknown field 'period_us'` until this line existed (issue 0518).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub period_us: Option<u64>,
    pub callback: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub callback_slot: Option<u32>,
}

impl SourceTimer {
    /// The declared period in MICROSECONDS — `period_us` when the harness
    /// supplied it, else the truncated `period_ms` widened.
    ///
    /// Issue #505 exists because `period_ms` truncates: a declared 33.333 ms
    /// timer reached the runtime as 33 ms (a permanent 1 % rate error) and a
    /// sub-millisecond period reached it as ZERO. Read the period through this
    /// method rather than the field, so the preference is stated once instead
    /// of at each call site — a site that forgets is not a compile error, it is
    /// a silent return of the bug #505 fixed.
    pub fn period_us(&self) -> u64 {
        self.period_us
            .unwrap_or(self.period_ms.saturating_mul(1_000))
    }
}

/// The same precedence for the JSON-map path, where the planner works with
/// `serde_json::Value` rather than [`SourceTimer`]. Kept beside its twin so the
/// two cannot drift.
pub fn timer_period_us_from_json(entity: &serde_json::Value) -> u64 {
    entity
        .get("period_us")
        .and_then(serde_json::Value::as_u64)
        .unwrap_or_else(|| {
            entity
                .get("period_ms")
                .and_then(serde_json::Value::as_u64)
                .unwrap_or(0)
                .saturating_mul(1_000)
        })
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceService {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub unresolved_name: SourceName,
    pub interface: InterfaceRef,
    pub callback: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub callback_slot: Option<u32>,
}

/// issue 0900 — an action or service CLIENT.
///
/// One struct for both: a client registers no callbacks, so unlike the server
/// pair (`SourceService` carries a `callback`, `SourceAction` carries three)
/// they differ in nothing but which array they arrive in.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceClient {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub unresolved_name: SourceName,
    pub interface: InterfaceRef,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceAction {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub unresolved_name: SourceName,
    pub interface: InterfaceRef,
    pub goal_callback: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub goal_callback_slot: Option<u32>,
    pub cancel_callback: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cancel_callback_slot: Option<u32>,
    pub accepted_callback: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub accepted_callback_slot: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceCallback {
    pub id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub kind: CallbackKind,
    pub group: Option<String>,
    pub effects: Vec<CallbackEffect>,
    pub source: SourceLocation,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CallbackKind {
    Timer,
    Subscription,
    Service,
    ActionGoal,
    ActionCancel,
    ActionAccepted,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CallbackEffect {
    pub kind: CallbackEffectKind,
    /// Source entity ID affected by the callback.
    pub entity: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entity_slot: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CallbackEffectKind {
    Publishes,
    ReadsParameter,
    WritesParameter,
    SendsServiceRequest,
    SendsServiceReply,
    SendsActionGoal,
    SendsActionFeedback,
    SendsActionResult,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceParameter {
    pub node: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub declaration_slot: Option<u32>,
    pub name: String,
    pub default: ParameterValue,
    pub read_only: bool,
    pub source: SourceLocation,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceMetadataTrace {
    pub generator: String,
    pub package_manifest: String,
    pub source_artifacts: Vec<String>,
}
