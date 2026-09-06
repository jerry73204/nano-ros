//! Node source metadata recorded without opening middleware.

use crate::{
    ParameterType, QoSProfile,
    heapless::{String, Vec},
};

#[cfg(feature = "alloc")]
use crate::{QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy};
#[cfg(feature = "alloc")]
// phase-359 W8 — `alloc`, not `std`. These three are the file's ONLY reason for
// a gate, and `alloc::{format, string::String, vec::Vec}` are the same types.
// Gating them as `std` did not just mislabel them, it WITHHELD the source
// metadata JSON API from `no_std + alloc` targets that can run it.
use alloc::{format, string::String as StdString, vec::Vec as StdVec};

/// Maximum nodes recorded by the built-in metadata recorder.
pub const DEFAULT_MAX_METADATA_NODES: usize = 8;
/// Maximum entities recorded by the built-in metadata recorder.
pub const DEFAULT_MAX_METADATA_ENTITIES: usize = 32;
/// Maximum callback/effect records kept by the built-in metadata recorder.
pub const DEFAULT_MAX_METADATA_CALLBACKS: usize = 32;
/// Maximum bytes in recorded source names and stable IDs.
pub const METADATA_STRING_CAPACITY: usize = 128;

/// Fixed-capacity string used by component metadata records.
pub type MetadataString = String<METADATA_STRING_CAPACITY>;

/// Declaration-order node slot within one extracted component.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NodeSlot(pub usize);

impl NodeSlot {
    /// Create a node slot from its declaration-order index.
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Declaration-order index.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Declaration-order entity slot within one extracted component.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EntitySlot(pub usize);

impl EntitySlot {
    /// Create an entity slot from its declaration-order index.
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Declaration-order index.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Declaration-order callback slot within one extracted component.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CallbackSlot(pub usize);

impl CallbackSlot {
    /// Create a callback slot from its declaration-order index.
    pub const fn new(index: usize) -> Self {
        Self(index)
    }

    /// Declaration-order index.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Source location attached to callbacks and parameters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceLocationMetadata {
    pub artifact: MetadataString,
    pub line: Option<u32>,
    pub column: Option<u32>,
}

impl SourceLocationMetadata {
    /// Empty source location used when caller data is unavailable.
    pub const fn empty() -> Self {
        Self {
            artifact: MetadataString::new(),
            line: None,
            column: None,
        }
    }

    /// Capture the Rust caller location.
    #[track_caller]
    pub fn caller() -> Result<Self, NodeMetadataError> {
        let location = core::panic::Location::caller();
        Ok(Self {
            // issue 0699 — keep the TAIL, never fail on depth.
            //
            // rustc emits `Location::file()` ABSOLUTE whenever the recording
            // crate is compiled as a path dependency from another directory,
            // which is exactly how the metadata harness builds it. So this
            // field carries the user's home directory, and a workspace nested
            // ~100 chars deep overflowed the 128-byte buffer — `nros sync`
            // failed with `Metadata(NameTooLong)` for no reason but where the
            // user keeps their files.
            //
            // Truncating the FRONT is safe precisely here: the CLI's
            // `metadata_build.rs` already rewrites this to be relative to the
            // component package afterwards, so the prefix is discarded either
            // way and the tail is the part that survives. A path is also
            // diagnostic metadata — losing its head degrades a message,
            // whereas erroring loses the whole register.
            artifact: copy_str_keep_tail(location.file())?,
            line: Some(location.line()),
            column: Some(location.column()),
        })
    }
}

/// Parameter default value recorded for source metadata.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParameterDefault {
    Bool(bool),
    Integer(i64),
    Double(MetadataString),
    String(MetadataString),
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
}

impl ParameterDefault {
    /// Parameter type implied by this default.
    pub const fn parameter_type(&self) -> ParameterType {
        match self {
            Self::Bool(_) => ParameterType::Bool,
            Self::Integer(_) => ParameterType::Integer,
            Self::Double(_) => ParameterType::Double,
            Self::String(_) => ParameterType::String,
            Self::BoolArray => ParameterType::BoolArray,
            Self::IntegerArray => ParameterType::IntegerArray,
            Self::DoubleArray => ParameterType::DoubleArray,
            Self::StringArray => ParameterType::StringArray,
        }
    }

    /// Default JSON-compatible value for a parameter type.
    pub fn for_type(param_type: ParameterType) -> Result<Self, NodeMetadataError> {
        Ok(match param_type {
            ParameterType::Bool => Self::Bool(false),
            ParameterType::Integer => Self::Integer(0),
            ParameterType::Double => Self::Double(copy_str("0.0")?),
            ParameterType::String => Self::String(copy_str("")?),
            ParameterType::BoolArray => Self::BoolArray,
            ParameterType::IntegerArray => Self::IntegerArray,
            ParameterType::DoubleArray => Self::DoubleArray,
            ParameterType::StringArray => Self::StringArray,
            ParameterType::ByteArray | ParameterType::NotSet => Self::Integer(0),
        })
    }
}

/// Unresolved ROS name category as written by component source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceNameKind {
    /// Starts with `/`.
    Absolute,
    /// Starts with `~`.
    Private,
    /// Any other non-empty source name.
    Relative,
}

impl SourceNameKind {
    /// Classify a source name without resolving launch remaps or namespaces.
    pub const fn from_source_name(name: &str) -> Self {
        let bytes = name.as_bytes();
        if bytes.is_empty() {
            Self::Relative
        } else if bytes[0] == b'/' {
            Self::Absolute
        } else if bytes[0] == b'~' {
            Self::Private
        } else {
            Self::Relative
        }
    }
}

/// Phase 305 W3 (issue 0255) — the ONE name-resolution seam: ROS 2 source-name
/// expansion (`~`/relative → FQN, ns=`/` collapse) + launch remap substitution
/// (exact-FQN match, first rule wins; no wildcards). Implemented in
/// `nros_node::names` so both the Rust `ExecutorSink` and the C-ABI
/// executor-side remap table share one lowering; re-exported here next to
/// [`SourceNameKind`] as the source-metadata-level entry point.
pub use nros_node::names::{
    MAX_RESOLVED_NAME_LEN, ResolvedName, expand_name, fully_qualified_name, resolve_name,
};

/// Stable source-level identifier required for component-mode declarations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EntityId<'a>(pub &'a str);

impl<'a> EntityId<'a> {
    /// Create a stable entity identifier.
    pub const fn new(id: &'a str) -> Self {
        Self(id)
    }

    /// Borrow the identifier string.
    pub const fn as_str(self) -> &'a str {
        self.0
    }
}

/// Stable node identifier required for component-mode node declarations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NodeId<'a>(pub &'a str);

impl<'a> NodeId<'a> {
    /// Create a stable node identifier.
    pub const fn new(id: &'a str) -> Self {
        Self(id)
    }

    /// Borrow the identifier string.
    pub const fn as_str(self) -> &'a str {
        self.0
    }
}

/// Stable callback identifier required for component-mode callbacks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CallbackId<'a>(pub &'a str);

impl<'a> CallbackId<'a> {
    /// Create a stable callback identifier.
    pub const fn new(id: &'a str) -> Self {
        Self(id)
    }

    /// Borrow the identifier string.
    pub const fn as_str(self) -> &'a str {
        self.0
    }
}

/// Entity role recorded for source metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    Publisher,
    Subscription,
    Timer,
    ServiceServer,
    ServiceClient,
    ActionServer,
    ActionClient,
    Parameter,
}

/// Optional callback effect relation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CallbackEffectKind {
    Reads,
    Publishes,
    Writes,
}

/// Metadata recorder/runtime error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeMetadataError {
    /// Fixed recorder capacity exhausted.
    Capacity,
    /// Stable ID, node name, namespace, topic, service, action, or parameter name was too long.
    NameTooLong,
    /// Entity references a node ID that has not been declared.
    UnknownNode,
    /// Callback effect references an entity ID that has not been declared.
    UnknownEntity,
    /// Stable ID already exists in the same component metadata.
    DuplicateId,
}

/// Recorded node declaration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeMetadata {
    pub slot: NodeSlot,
    pub id: MetadataString,
    pub source_default_name: MetadataString,
    pub name: MetadataString,
    pub namespace: MetadataString,
    pub domain_id: u32,
}

/// Recorded entity declaration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntityMetadata {
    pub slot: Option<EntitySlot>,
    pub id: MetadataString,
    pub node_slot: Option<NodeSlot>,
    pub node_id: MetadataString,
    pub kind: EntityKind,
    pub source_name: MetadataString,
    pub source_name_kind: SourceNameKind,
    pub type_name: &'static str,
    pub type_hash: &'static str,
    pub qos: QoSProfile,
    pub callback_slot: Option<CallbackSlot>,
    pub callback_id: Option<MetadataString>,
    pub callback_source: SourceLocationMetadata,
    pub callback_group: Option<MetadataString>,
    pub action_cancel_callback_slot: Option<CallbackSlot>,
    pub action_cancel_callback_id: Option<MetadataString>,
    pub action_cancel_source: SourceLocationMetadata,
    pub action_accepted_callback_slot: Option<CallbackSlot>,
    pub action_accepted_callback_id: Option<MetadataString>,
    pub action_accepted_source: SourceLocationMetadata,
    pub period_ms: Option<u64>,
    /// Issue #505 — the timer period at the executor's own resolution.
    /// `period_ms` is kept (and still emitted) for consumers of the
    /// metadata JSON, but it truncates: a declared 33.333 ms timer used
    /// to reach the runtime as 33 ms, a 1% permanent rate error, and a
    /// sub-millisecond period reached it as zero.
    pub period_us: Option<u64>,
    pub parameter_type: Option<ParameterType>,
    pub parameter_default: Option<ParameterDefault>,
    pub parameter_read_only: bool,
    /// Phase 250 (Wave 2b) — a subscription that opted into E2E message-integrity
    /// validation (`.safety()`): the runtime registers it via
    /// `create_generic_subscription_with_integrity` and surfaces
    /// [`IntegrityStatus`](crate::IntegrityStatus) through `CallbackCtx::integrity()`.
    /// Ungated (a plain flag) — only the runtime branch that reads it is gated on
    /// `safety-e2e`, so when the capability is off the flag is simply ignored
    /// (the subscription registers as a basic one). `false` for every other entity.
    pub safety: bool,
    pub source: SourceLocationMetadata,
}

/// Recorded optional callback effect.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CallbackEffectMetadata {
    pub callback_id: MetadataString,
    pub callback_slot: Option<CallbackSlot>,
    pub kind: CallbackEffectKind,
    pub entity_id: MetadataString,
    pub entity_slot: Option<EntitySlot>,
}

/// Source metadata document settings used by the std JSON emitter.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone)]
pub struct SourceMetadataExport<'a> {
    pub package: &'a str,
    pub component: &'a str,
    pub executable: Option<&'a str>,
    pub exported_symbol: Option<&'a str>,
    pub package_manifest: &'a str,
    pub source_artifacts: &'a [&'a str],
    /// phase-308 — the source language the sidecar describes (`"rust"`,
    /// `"c"`, `"cpp"`). It was a hardcoded `"rust"` literal in the serializer
    /// while Rust was the only producer; the C/C++ producer reaches this SAME
    /// serializer, so the value has to come from the caller. The consumption
    /// side keys on `(package, executable)` and never branches on it, but the
    /// schema declares it and a wrong value is a lie in the artifact.
    pub language: &'a str,
}

#[cfg(feature = "alloc")]
impl<'a> SourceMetadataExport<'a> {
    /// Create export settings with ROS package and component names.
    pub const fn new(package: &'a str, component: &'a str) -> Self {
        Self {
            package,
            component,
            executable: None,
            exported_symbol: None,
            package_manifest: "package.xml",
            source_artifacts: &[],
            language: "rust",
        }
    }

    /// Set the source language (`"c"` / `"cpp"`; defaults to `"rust"`).
    pub const fn language(mut self, language: &'a str) -> Self {
        self.language = language;
        self
    }

    /// Set executable name.
    pub const fn executable(mut self, executable: &'a str) -> Self {
        self.executable = Some(executable);
        self
    }

    /// Set exported symbol name.
    pub const fn exported_symbol(mut self, exported_symbol: &'a str) -> Self {
        self.exported_symbol = Some(exported_symbol);
        self
    }

    /// Set package manifest path.
    pub const fn package_manifest(mut self, package_manifest: &'a str) -> Self {
        self.package_manifest = package_manifest;
        self
    }

    /// Set source artifact paths.
    pub const fn source_artifacts(mut self, source_artifacts: &'a [&'a str]) -> Self {
        self.source_artifacts = source_artifacts;
        self
    }
}

/// phase-308 — `pub` for metadata-mode adapters (see
/// [`MetadataRecorder::push_node`]); the capacity error is theirs to surface.
pub fn metadata_string(value: &str) -> Result<MetadataString, NodeMetadataError> {
    copy_str(value)
}

/// `copy_str`, but for a value whose TAIL is the informative end — a source
/// path. Keeps the last whole path components that fit, marked with a leading
/// `…/` so a reader can tell it was cut.
///
/// Issue 0699: the only caller is `SourceLocation::caller()`, whose input is
/// `core::panic::Location::file()`. That is an absolute path in the metadata
/// harness, so its length is set by where the USER put the workspace — not by
/// anything this tree controls, and not something a fixed buffer can bound.
pub(crate) fn copy_str_keep_tail(value: &str) -> Result<MetadataString, NodeMetadataError> {
    if value.len() <= METADATA_STRING_CAPACITY {
        return copy_str(value);
    }
    // Room for the marker, then the longest component-aligned tail that fits.
    const MARK: &str = "…/";
    let budget = METADATA_STRING_CAPACITY - MARK.len();
    let tail = value
        .char_indices()
        .find(|(i, _)| value.len() - i <= budget)
        .map(|(i, _)| &value[i..])
        .unwrap_or("");
    // Prefer a component boundary so the result reads as a path.
    let tail = match tail.find('/') {
        Some(cut) => &tail[cut + 1..],
        None => tail,
    };
    let mut out = MetadataString::new();
    out.push_str(MARK)
        .map_err(|_| NodeMetadataError::NameTooLong)?;
    out.push_str(tail)
        .map_err(|_| NodeMetadataError::NameTooLong)?;
    Ok(out)
}

pub(crate) fn copy_str(value: &str) -> Result<MetadataString, NodeMetadataError> {
    let mut out = MetadataString::new();
    out.push_str(value)
        .map_err(|_| NodeMetadataError::NameTooLong)?;
    Ok(out)
}

/// In-memory metadata sink used by host discovery. It never opens transport.
#[derive(Debug)]
pub struct MetadataRecorder<
    const MAX_NODES: usize = DEFAULT_MAX_METADATA_NODES,
    const MAX_ENTITIES: usize = DEFAULT_MAX_METADATA_ENTITIES,
    const MAX_CALLBACKS: usize = DEFAULT_MAX_METADATA_CALLBACKS,
> {
    nodes: Vec<NodeMetadata, MAX_NODES>,
    entities: Vec<EntityMetadata, MAX_ENTITIES>,
    callback_effects: Vec<CallbackEffectMetadata, MAX_CALLBACKS>,
}

impl<const MAX_NODES: usize, const MAX_ENTITIES: usize, const MAX_CALLBACKS: usize> Default
    for MetadataRecorder<MAX_NODES, MAX_ENTITIES, MAX_CALLBACKS>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_NODES: usize, const MAX_ENTITIES: usize, const MAX_CALLBACKS: usize>
    MetadataRecorder<MAX_NODES, MAX_ENTITIES, MAX_CALLBACKS>
{
    /// Create an empty metadata recorder.
    pub const fn new() -> Self {
        Self {
            nodes: Vec::new(),
            entities: Vec::new(),
            callback_effects: Vec::new(),
        }
    }

    /// Recorded nodes in declaration order.
    pub fn nodes(&self) -> &[NodeMetadata] {
        &self.nodes
    }

    /// Recorded entities in declaration order.
    pub fn entities(&self) -> &[EntityMetadata] {
        &self.entities
    }

    /// Recorded optional callback effects in declaration order.
    pub fn callback_effects(&self) -> &[CallbackEffectMetadata] {
        &self.callback_effects
    }

    /// Emit schema-version-1 source metadata JSON without opening transport.
    #[cfg(feature = "alloc")]
    pub fn to_source_metadata_json(
        &self,
        export: &SourceMetadataExport<'_>,
    ) -> Result<StdString, core::fmt::Error> {
        let mut out = StdString::new();
        self.write_source_metadata_json(export, &mut out)?;
        Ok(out)
    }

    /// Write schema-version-1 source metadata JSON without opening transport.
    #[cfg(feature = "alloc")]
    pub fn write_source_metadata_json(
        &self,
        export: &SourceMetadataExport<'_>,
        out: &mut impl core::fmt::Write,
    ) -> core::fmt::Result {
        write!(out, "{{")?;
        write!(out, "\"version\":1,")?;
        write_json_field(out, "package", export.package)?;
        out.write_char(',')?;
        write_json_field(out, "component", export.component)?;
        out.write_char(',')?;
        write_json_field(out, "language", export.language)?;
        out.write_char(',')?;
        write_json_opt_field(out, "executable", export.executable)?;
        out.write_char(',')?;
        write_json_opt_field(out, "exported_symbol", export.exported_symbol)?;
        out.write_char(',')?;
        self.write_nodes_json(out)?;
        out.write_char(',')?;
        self.write_callbacks_json(out)?;
        out.write_char(',')?;
        self.write_parameters_json(out)?;
        out.write_char(',')?;
        self.write_trace_json(export, out)?;
        write!(out, "}}")
    }

    /// Record a node.
    ///
    /// phase-308 — `pub` because the recorder is the ONE recorder: the Rust
    /// adapter (a `NodeContext` sink) and the C/C++ adapter (a recording RMW
    /// backend + the timer/guard hooks in `nros-cpp`) both feed it, so there is
    /// one definition of what a slot is and one schema emitter. Only the
    /// adapter is per-language. Not a general-purpose API — it exists for
    /// metadata-mode adapters, and misuse produces a sidecar that lies about
    /// what the component declares.
    pub fn push_node(
        &mut self,
        id: NodeId<'_>,
        name: &str,
        namespace: &str,
        domain_id: u32,
    ) -> Result<(), NodeMetadataError> {
        if self.has_node(id.as_str()) {
            return Err(NodeMetadataError::DuplicateId);
        }

        self.nodes
            .push(NodeMetadata {
                slot: NodeSlot::new(self.nodes.len()),
                id: copy_str(id.as_str())?,
                source_default_name: copy_str(name)?,
                name: copy_str(name)?,
                namespace: copy_str(namespace)?,
                domain_id,
            })
            .map_err(|_| NodeMetadataError::Capacity)
    }

    /// Record an entity against an already-recorded node. See [`push_node`]
    /// for why this is public.
    ///
    /// [`push_node`]: Self::push_node
    pub fn push_entity(&mut self, mut entity: EntityMetadata) -> Result<(), NodeMetadataError> {
        if !self.has_node(&entity.node_id) {
            return Err(NodeMetadataError::UnknownNode);
        }
        if self.has_entity(&entity.id) {
            return Err(NodeMetadataError::DuplicateId);
        }

        entity.slot = Some(EntitySlot::new(self.entities.len()));
        entity.node_slot = self.node_slot_for_id(&entity.node_id);
        let mut current_callbacks = Vec::<MetadataString, 3>::new();
        let mut next_callback_slot = self.callback_slot_count();
        entity.callback_slot = entity.callback_id.as_ref().map(|callback_id| {
            self.callback_slot_for_current_entity(
                callback_id.as_str(),
                &mut current_callbacks,
                &mut next_callback_slot,
            )
        });
        entity.action_cancel_callback_slot =
            entity
                .action_cancel_callback_id
                .as_ref()
                .map(|callback_id| {
                    self.callback_slot_for_current_entity(
                        callback_id.as_str(),
                        &mut current_callbacks,
                        &mut next_callback_slot,
                    )
                });
        entity.action_accepted_callback_slot =
            entity
                .action_accepted_callback_id
                .as_ref()
                .map(|callback_id| {
                    self.callback_slot_for_current_entity(
                        callback_id.as_str(),
                        &mut current_callbacks,
                        &mut next_callback_slot,
                    )
                });

        self.entities
            .push(entity)
            .map_err(|_| NodeMetadataError::Capacity)
    }

    pub(crate) fn push_callback_effect(
        &mut self,
        callback_id: CallbackId<'_>,
        kind: CallbackEffectKind,
        entity_id: EntityId<'_>,
    ) -> Result<(), NodeMetadataError> {
        if !self.has_entity(entity_id.as_str()) {
            return Err(NodeMetadataError::UnknownEntity);
        }

        self.callback_effects
            .push(CallbackEffectMetadata {
                callback_id: copy_str(callback_id.as_str())?,
                callback_slot: self.callback_slot_for_id(callback_id.as_str()),
                kind,
                entity_id: copy_str(entity_id.as_str())?,
                entity_slot: self.entity_slot_for_id(entity_id.as_str()),
            })
            .map_err(|_| NodeMetadataError::Capacity)
    }

    pub(crate) fn has_node(&self, id: &str) -> bool {
        self.nodes.iter().any(|node| node.id.as_str() == id)
    }

    pub(crate) fn has_entity(&self, id: &str) -> bool {
        self.entities.iter().any(|entity| entity.id.as_str() == id)
    }

    fn node_slot_for_id(&self, id: &str) -> Option<NodeSlot> {
        self.nodes
            .iter()
            .find(|node| node.id.as_str() == id)
            .map(|node| node.slot)
    }

    fn entity_slot_for_id(&self, id: &str) -> Option<EntitySlot> {
        self.entities
            .iter()
            .find(|entity| entity.id.as_str() == id)
            .and_then(|entity| entity.slot)
    }

    fn callback_slot_for_current_entity(
        &self,
        id: &str,
        current_callbacks: &mut Vec<MetadataString, 3>,
        next_callback_slot: &mut usize,
    ) -> CallbackSlot {
        if let Some(slot) = self.callback_slot_for_id(id) {
            return slot;
        }
        if let Some((index, _)) = current_callbacks
            .iter()
            .enumerate()
            .find(|(_, callback_id)| callback_id.as_str() == id)
        {
            return CallbackSlot::new(self.callback_slot_count() + index);
        }
        let slot = CallbackSlot::new(*next_callback_slot);
        let _ = current_callbacks
            .push(copy_str(id).expect("callback ID already fits metadata string capacity"));
        *next_callback_slot += 1;
        slot
    }

    fn callback_slot_for_id(&self, id: &str) -> Option<CallbackSlot> {
        let mut seen = Vec::<&str, MAX_CALLBACKS>::new();
        for entity in &self.entities {
            for callback_id in entity_callback_ids(entity) {
                let Some(callback_id) = callback_id else {
                    continue;
                };
                let callback_id = callback_id.as_str();
                if seen.contains(&callback_id) {
                    continue;
                }
                if callback_id == id {
                    return Some(CallbackSlot::new(seen.len()));
                }
                let _ = seen.push(callback_id);
            }
        }
        None
    }

    fn callback_slot_count(&self) -> usize {
        let mut seen = Vec::<&str, MAX_CALLBACKS>::new();
        for entity in &self.entities {
            for callback_id in entity_callback_ids(entity) {
                let Some(callback_id) = callback_id else {
                    continue;
                };
                let callback_id = callback_id.as_str();
                if !seen.contains(&callback_id) {
                    let _ = seen.push(callback_id);
                }
            }
        }
        seen.len()
    }

    #[cfg(feature = "alloc")]
    fn write_nodes_json(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(out, "\"nodes\":[")?;
        for (index, node) in self.nodes.iter().enumerate() {
            if index > 0 {
                out.write_char(',')?;
            }
            write!(out, "{{")?;
            write_json_field(out, "id", node.id.as_str())?;
            out.write_char(',')?;
            write!(out, "\"declaration_slot\":{},", node.slot.index())?;
            write_json_field(
                out,
                "source_default_name",
                node.source_default_name.as_str(),
            )?;
            out.write_char(',')?;
            write!(out, "\"unresolved_name\":")?;
            write_source_name(
                out,
                node.name.as_str(),
                SourceNameKind::from_source_name(&node.name),
            )?;
            out.write_char(',')?;
            if node.namespace.as_str() == "/" {
                write!(out, "\"namespace\":null,")?;
            } else {
                write_json_field(out, "namespace", node.namespace.as_str())?;
                out.write_char(',')?;
            }
            self.write_node_entities(out, node.id.as_str())?;
            write!(out, "}}")?;
        }
        write!(out, "]")
    }

    #[cfg(feature = "alloc")]
    fn write_node_entities(
        &self,
        out: &mut impl core::fmt::Write,
        node_id: &str,
    ) -> core::fmt::Result {
        self.write_entity_array(out, "publishers", node_id, EntityKind::Publisher)?;
        out.write_char(',')?;
        self.write_entity_array(out, "subscribers", node_id, EntityKind::Subscription)?;
        out.write_char(',')?;
        self.write_entity_array(out, "timers", node_id, EntityKind::Timer)?;
        out.write_char(',')?;
        self.write_entity_array(out, "services", node_id, EntityKind::ServiceServer)?;
        out.write_char(',')?;
        self.write_entity_array(out, "actions", node_id, EntityKind::ActionServer)?;
        // issue 0900 — the CLIENT halves. `record_entity` has always captured
        // them (`EntityKind::{ActionClient,ServiceClient}`, set in node.rs) and
        // this writer dropped them on the way out, so a sidecar described only
        // what a component SERVES. The executor arena is sized from the client
        // count, which is why the omission had a size cost and not just a
        // descriptive one.
        out.write_char(',')?;
        self.write_entity_array(out, "action_clients", node_id, EntityKind::ActionClient)?;
        out.write_char(',')?;
        self.write_entity_array(out, "service_clients", node_id, EntityKind::ServiceClient)
    }

    #[cfg(feature = "alloc")]
    fn write_entity_array(
        &self,
        out: &mut impl core::fmt::Write,
        field: &str,
        node_id: &str,
        kind: EntityKind,
    ) -> core::fmt::Result {
        write!(out, "\"{}\":[", field)?;
        for (index, entity) in self
            .entities
            .iter()
            .filter(|entity| entity.node_id.as_str() == node_id && entity.kind == kind)
            .enumerate()
        {
            if index > 0 {
                out.write_char(',')?;
            }
            match kind {
                EntityKind::Publisher => write_publisher_json(out, entity)?,
                EntityKind::Subscription => write_subscriber_json(out, entity)?,
                EntityKind::Timer => write_timer_json(out, entity)?,
                EntityKind::ServiceServer => write_service_json(out, entity)?,
                EntityKind::ActionServer => write_action_json(out, entity)?,
                // issue 0900 — a client registers no callbacks, so it needs
                // none of the server writers' callback fields. One writer
                // serves both client kinds.
                EntityKind::ActionClient | EntityKind::ServiceClient => {
                    write_client_json(out, entity)?
                }
                _ => {}
            }
        }
        write!(out, "]")
    }

    #[cfg(feature = "alloc")]
    fn write_callbacks_json(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        let callbacks = self.source_callbacks();
        write!(out, "\"callbacks\":[")?;
        for (index, callback) in callbacks.iter().enumerate() {
            if index > 0 {
                out.write_char(',')?;
            }
            write!(out, "{{")?;
            write_json_field(out, "id", callback.id.as_str())?;
            out.write_char(',')?;
            if let Some(slot) = callback.slot {
                write!(out, "\"declaration_slot\":{},", slot.index())?;
            }
            write_json_field(out, "kind", callback.kind)?;
            out.write_char(',')?;
            if let Some(group) = callback.group.as_ref() {
                write_json_field(out, "group", group)?;
                out.write_char(',')?;
            } else {
                write!(out, "\"group\":null,")?;
            }
            write!(out, "\"effects\":[")?;
            for (effect_index, effect) in self
                .callback_effects
                .iter()
                .filter(|effect| effect.callback_id.as_str() == callback.id)
                .enumerate()
            {
                if effect_index > 0 {
                    out.write_char(',')?;
                }
                write!(out, "{{")?;
                write_json_field(out, "kind", effect_json_kind(effect.kind))?;
                out.write_char(',')?;
                write_json_field(out, "entity", effect.entity_id.as_str())?;
                if let Some(entity_slot) = effect.entity_slot {
                    write!(out, ",\"entity_slot\":{}", entity_slot.index())?;
                }
                write!(out, "}}")?;
            }
            write!(out, "],")?;
            write_source_location(out, &callback.source)?;
            write!(out, "}}")?;
        }
        write!(out, "]")
    }

    #[cfg(feature = "alloc")]
    fn write_parameters_json(&self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(out, "\"parameters\":[")?;
        for (index, entity) in self
            .entities
            .iter()
            .filter(|entity| entity.kind == EntityKind::Parameter)
            .enumerate()
        {
            if index > 0 {
                out.write_char(',')?;
            }
            write!(out, "{{")?;
            write_json_field(out, "node", entity.node_id.as_str())?;
            out.write_char(',')?;
            if let Some(slot) = entity.slot {
                write!(out, "\"declaration_slot\":{},", slot.index())?;
            }
            write_json_field(out, "name", entity.source_name.as_str())?;
            out.write_char(',')?;
            write!(out, "\"default\":")?;
            write_parameter_default(out, entity.parameter_default.as_ref())?;
            out.write_char(',')?;
            write!(out, "\"read_only\":{},", entity.parameter_read_only)?;
            write_source_location(out, &entity.source)?;
            write!(out, "}}")?;
        }
        write!(out, "]")
    }

    #[cfg(feature = "alloc")]
    fn write_trace_json(
        &self,
        export: &SourceMetadataExport<'_>,
        out: &mut impl core::fmt::Write,
    ) -> core::fmt::Result {
        write!(out, "\"trace\":{{")?;
        write_json_field(out, "generator", "nros-metadata-rust")?;
        out.write_char(',')?;
        write_json_field(out, "package_manifest", export.package_manifest)?;
        out.write_char(',')?;
        write!(out, "\"source_artifacts\":[")?;
        for (index, artifact) in export.source_artifacts.iter().enumerate() {
            if index > 0 {
                out.write_char(',')?;
            }
            write_json_string(out, artifact)?;
        }
        write!(out, "]}}")
    }

    #[cfg(feature = "alloc")]
    fn source_callbacks(&self) -> StdVec<SourceCallbackRef> {
        let mut callbacks = StdVec::new();
        for entity in &self.entities {
            let Some(callback_id) = entity.callback_id.as_ref() else {
                continue;
            };
            let kind = match entity.kind {
                EntityKind::Subscription => "subscription",
                EntityKind::Timer => "timer",
                EntityKind::ServiceServer => "service",
                EntityKind::ActionServer => "action_goal",
                _ => continue,
            };
            if !callbacks
                .iter()
                .any(|callback: &SourceCallbackRef| callback.id == callback_id.as_str())
            {
                callbacks.push(SourceCallbackRef {
                    id: callback_id.as_str().into(),
                    slot: entity.callback_slot,
                    kind,
                    source: entity.callback_source.clone(),
                    group: entity
                        .callback_group
                        .as_ref()
                        .map(|group| group.as_str().into()),
                });
            }
            if entity.kind == EntityKind::ActionServer {
                if let Some(cancel_id) = entity.action_cancel_callback_id.as_ref()
                    && !callbacks
                        .iter()
                        .any(|callback: &SourceCallbackRef| callback.id == cancel_id.as_str())
                {
                    callbacks.push(SourceCallbackRef {
                        id: cancel_id.as_str().into(),
                        slot: entity.action_cancel_callback_slot,
                        kind: "action_cancel",
                        source: entity.action_cancel_source.clone(),
                        group: entity
                            .callback_group
                            .as_ref()
                            .map(|group| group.as_str().into()),
                    });
                }
                if let Some(accepted_id) = entity.action_accepted_callback_id.as_ref()
                    && !callbacks
                        .iter()
                        .any(|callback: &SourceCallbackRef| callback.id == accepted_id.as_str())
                {
                    callbacks.push(SourceCallbackRef {
                        id: accepted_id.as_str().into(),
                        slot: entity.action_accepted_callback_slot,
                        kind: "action_accepted",
                        source: entity.action_accepted_source.clone(),
                        group: entity
                            .callback_group
                            .as_ref()
                            .map(|group| group.as_str().into()),
                    });
                }
            }
        }
        callbacks
    }
}

#[cfg(feature = "alloc")]
struct SourceCallbackRef {
    id: StdString,
    slot: Option<CallbackSlot>,
    kind: &'static str,
    source: SourceLocationMetadata,
    group: Option<StdString>,
}

pub(crate) fn entity_callback_ids(entity: &EntityMetadata) -> [Option<&MetadataString>; 3] {
    [
        entity.callback_id.as_ref(),
        entity.action_cancel_callback_id.as_ref(),
        entity.action_accepted_callback_id.as_ref(),
    ]
}

/// Inputs for [`entity_metadata`]. Collapses the seven positional
/// arguments — three of them adjacent `&str` (`source_name` /
/// `type_name` / `type_hash`) that are trivially transposable at a
/// call site — into one named-field struct.
/// phase-308 — `pub` alongside [`MetadataRecorder::push_entity`]: metadata-mode
/// adapters for other languages build entities through this ONE constructor so
/// the defaults (and therefore the recorded shape) cannot diverge per language.
pub struct EntityMetadataSpec<'a> {
    pub id: EntityId<'a>,
    pub node_id: NodeId<'a>,
    pub kind: EntityKind,
    pub source_name: &'a str,
    pub type_name: &'static str,
    pub type_hash: &'static str,
    pub qos: QoSProfile,
}

/// Build an [`EntityMetadata`] from its identifying fields, defaulting the
/// rest. See [`EntityMetadataSpec`] for why this is public.
pub fn entity_metadata(spec: EntityMetadataSpec<'_>) -> Result<EntityMetadata, NodeMetadataError> {
    let EntityMetadataSpec {
        id,
        node_id,
        kind,
        source_name,
        type_name,
        type_hash,
        qos,
    } = spec;
    Ok(EntityMetadata {
        slot: None,
        id: copy_str(id.as_str())?,
        node_slot: None,
        node_id: copy_str(node_id.as_str())?,
        kind,
        source_name: copy_str(source_name)?,
        source_name_kind: SourceNameKind::from_source_name(source_name),
        type_name,
        type_hash,
        qos,
        callback_slot: None,
        callback_id: None,
        callback_source: SourceLocationMetadata::empty(),
        callback_group: None,
        action_cancel_callback_slot: None,
        action_cancel_callback_id: None,
        action_cancel_source: SourceLocationMetadata::empty(),
        action_accepted_callback_slot: None,
        action_accepted_callback_id: None,
        action_accepted_source: SourceLocationMetadata::empty(),
        period_ms: None,
        period_us: None,
        parameter_type: None,
        parameter_default: None,
        parameter_read_only: false,
        safety: false,
        source: SourceLocationMetadata::empty(),
    })
}

#[cfg(feature = "alloc")]
fn write_publisher_json(
    out: &mut impl core::fmt::Write,
    entity: &EntityMetadata,
) -> core::fmt::Result {
    write!(out, "{{")?;
    write_json_field(out, "id", entity.id.as_str())?;
    out.write_char(',')?;
    if let Some(slot) = entity.slot {
        write!(out, "\"declaration_slot\":{},", slot.index())?;
    }
    write!(out, "\"unresolved_topic\":")?;
    write_source_name(out, entity.source_name.as_str(), entity.source_name_kind)?;
    out.write_char(',')?;
    write_interface(out, entity.type_name, "message")?;
    out.write_char(',')?;
    write_qos(out, entity.qos)?;
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_subscriber_json(
    out: &mut impl core::fmt::Write,
    entity: &EntityMetadata,
) -> core::fmt::Result {
    write!(out, "{{")?;
    write_json_field(out, "id", entity.id.as_str())?;
    out.write_char(',')?;
    if let Some(slot) = entity.slot {
        write!(out, "\"declaration_slot\":{},", slot.index())?;
    }
    write!(out, "\"unresolved_topic\":")?;
    write_source_name(out, entity.source_name.as_str(), entity.source_name_kind)?;
    out.write_char(',')?;
    write_interface(out, entity.type_name, "message")?;
    out.write_char(',')?;
    write_qos(out, entity.qos)?;
    out.write_char(',')?;
    write_json_field(
        out,
        "callback",
        entity
            .callback_id
            .as_ref()
            .map(|id| id.as_str())
            .unwrap_or(""),
    )?;
    if let Some(callback_slot) = entity.callback_slot {
        write!(out, ",\"callback_slot\":{}", callback_slot.index())?;
    }
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_timer_json(out: &mut impl core::fmt::Write, entity: &EntityMetadata) -> core::fmt::Result {
    write!(out, "{{")?;
    write_json_field(out, "id", entity.id.as_str())?;
    out.write_char(',')?;
    if let Some(slot) = entity.slot {
        write!(out, "\"declaration_slot\":{},", slot.index())?;
    }
    write!(out, "\"period_ms\":{},", entity.period_ms.unwrap_or(0))?;
    write!(out, "\"period_us\":{},", entity.period_us.unwrap_or(0))?;
    write_json_field(
        out,
        "callback",
        entity
            .callback_id
            .as_ref()
            .map(|id| id.as_str())
            .unwrap_or(""),
    )?;
    if let Some(callback_slot) = entity.callback_slot {
        write!(out, ",\"callback_slot\":{}", callback_slot.index())?;
    }
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_service_json(
    out: &mut impl core::fmt::Write,
    entity: &EntityMetadata,
) -> core::fmt::Result {
    write!(out, "{{")?;
    write_json_field(out, "id", entity.id.as_str())?;
    out.write_char(',')?;
    if let Some(slot) = entity.slot {
        write!(out, "\"declaration_slot\":{},", slot.index())?;
    }
    write!(out, "\"unresolved_name\":")?;
    write_source_name(out, entity.source_name.as_str(), entity.source_name_kind)?;
    out.write_char(',')?;
    write_interface(out, entity.type_name, "service")?;
    out.write_char(',')?;
    write_json_field(
        out,
        "callback",
        entity
            .callback_id
            .as_ref()
            .map(|id| id.as_str())
            .unwrap_or(""),
    )?;
    if let Some(callback_slot) = entity.callback_slot {
        write!(out, ",\"callback_slot\":{}", callback_slot.index())?;
    }
    write!(out, "}}")
}

/// issue 0900 — a client entity: identity, name and interface, no callbacks.
///
/// Deliberately ONE writer for both `ActionClient` and `ServiceClient`. They
/// differ only in the interface KIND word, and the server-side pair is already
/// two near-identical functions; a third and fourth would be the second
/// spelling this repo keeps paying for.
#[cfg(feature = "alloc")]
fn write_client_json(
    out: &mut impl core::fmt::Write,
    entity: &EntityMetadata,
) -> core::fmt::Result {
    let interface_kind = if entity.kind == EntityKind::ActionClient {
        "action"
    } else {
        "service"
    };
    write!(out, "{{")?;
    write_json_field(out, "id", entity.id.as_str())?;
    out.write_char(',')?;
    if let Some(slot) = entity.slot {
        write!(out, "\"declaration_slot\":{},", slot.index())?;
    }
    write!(out, "\"unresolved_name\":")?;
    write_source_name(out, entity.source_name.as_str(), entity.source_name_kind)?;
    out.write_char(',')?;
    write_interface(out, entity.type_name, interface_kind)?;
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_action_json(
    out: &mut impl core::fmt::Write,
    entity: &EntityMetadata,
) -> core::fmt::Result {
    let goal_callback = entity
        .callback_id
        .as_ref()
        .map(|id| id.as_str())
        .unwrap_or("");
    let cancel_callback = entity
        .action_cancel_callback_id
        .as_ref()
        .map(|id| id.as_str())
        .unwrap_or(goal_callback);
    let accepted_callback = entity
        .action_accepted_callback_id
        .as_ref()
        .map(|id| id.as_str())
        .unwrap_or(goal_callback);
    write!(out, "{{")?;
    write_json_field(out, "id", entity.id.as_str())?;
    out.write_char(',')?;
    if let Some(slot) = entity.slot {
        write!(out, "\"declaration_slot\":{},", slot.index())?;
    }
    write!(out, "\"unresolved_name\":")?;
    write_source_name(out, entity.source_name.as_str(), entity.source_name_kind)?;
    out.write_char(',')?;
    write_interface(out, entity.type_name, "action")?;
    out.write_char(',')?;
    write_json_field(out, "goal_callback", goal_callback)?;
    if let Some(callback_slot) = entity.callback_slot {
        write!(out, ",\"goal_callback_slot\":{}", callback_slot.index())?;
    }
    out.write_char(',')?;
    write_json_field(out, "cancel_callback", cancel_callback)?;
    if let Some(callback_slot) = entity.action_cancel_callback_slot {
        write!(out, ",\"cancel_callback_slot\":{}", callback_slot.index())?;
    }
    out.write_char(',')?;
    write_json_field(out, "accepted_callback", accepted_callback)?;
    if let Some(callback_slot) = entity.action_accepted_callback_slot {
        write!(out, ",\"accepted_callback_slot\":{}", callback_slot.index())?;
    }
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_source_name(
    out: &mut impl core::fmt::Write,
    value: &str,
    kind: SourceNameKind,
) -> core::fmt::Result {
    write!(out, "{{")?;
    write_json_field(out, "value", value)?;
    out.write_char(',')?;
    write_json_field(out, "kind", source_name_kind_json(kind))?;
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_interface(
    out: &mut impl core::fmt::Write,
    type_name: &str,
    fallback_kind: &'static str,
) -> core::fmt::Result {
    let interface = parse_interface(type_name, fallback_kind);
    write!(out, "\"interface\":{{")?;
    write_json_field(out, "package", &interface.package)?;
    out.write_char(',')?;
    write_json_field(out, "name", &interface.name)?;
    out.write_char(',')?;
    write_json_field(out, "kind", interface.kind)?;
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_qos(out: &mut impl core::fmt::Write, qos: QoSProfile) -> core::fmt::Result {
    write!(out, "\"qos\":{{")?;
    write_json_field(out, "reliability", reliability_json(qos.reliability))?;
    out.write_char(',')?;
    write_json_field(out, "durability", durability_json(qos.durability))?;
    out.write_char(',')?;
    write_json_field(out, "history", history_json(qos.history))?;
    out.write_char(',')?;
    write!(out, "\"depth\":{},", qos.depth)?;
    write_optional_ms(out, "deadline_ms", qos.deadline_ms)?;
    out.write_char(',')?;
    write_optional_ms(out, "lifespan_ms", qos.lifespan_ms)?;
    out.write_char(',')?;
    write_json_field(out, "liveliness", liveliness_json(qos.liveliness_kind))?;
    out.write_char(',')?;
    write_optional_ms(out, "liveliness_lease_duration_ms", qos.liveliness_lease_ms)?;
    write!(out, ",\"extensions\":{{}}}}")
}

#[cfg(feature = "alloc")]
fn write_source_location(
    out: &mut impl core::fmt::Write,
    source: &SourceLocationMetadata,
) -> core::fmt::Result {
    write!(out, "\"source\":{{")?;
    write_json_field(out, "artifact", source.artifact.as_str())?;
    out.write_char(',')?;
    write!(out, "\"line\":")?;
    write_optional_u32(out, source.line)?;
    out.write_char(',')?;
    write!(out, "\"column\":")?;
    write_optional_u32(out, source.column)?;
    write!(out, "}}")
}

#[cfg(feature = "alloc")]
fn write_parameter_default(
    out: &mut impl core::fmt::Write,
    default: Option<&ParameterDefault>,
) -> core::fmt::Result {
    match default {
        Some(ParameterDefault::Bool(value)) => write!(out, "{}", value),
        Some(ParameterDefault::Integer(value)) => write!(out, "{}", value),
        Some(ParameterDefault::Double(value)) => write!(out, "{}", value.as_str()),
        Some(ParameterDefault::String(value)) => write_json_string(out, value.as_str()),
        Some(ParameterDefault::BoolArray)
        | Some(ParameterDefault::IntegerArray)
        | Some(ParameterDefault::DoubleArray)
        | Some(ParameterDefault::StringArray)
        | None => write!(out, "[]"),
    }
}

#[cfg(feature = "alloc")]
fn write_json_field(out: &mut impl core::fmt::Write, name: &str, value: &str) -> core::fmt::Result {
    write_json_string(out, name)?;
    out.write_char(':')?;
    write_json_string(out, value)
}

#[cfg(feature = "alloc")]
fn write_json_opt_field(
    out: &mut impl core::fmt::Write,
    name: &str,
    value: Option<&str>,
) -> core::fmt::Result {
    write_json_string(out, name)?;
    out.write_char(':')?;
    if let Some(value) = value {
        write_json_string(out, value)
    } else {
        write!(out, "null")
    }
}

#[cfg(feature = "alloc")]
fn write_json_string(out: &mut impl core::fmt::Write, value: &str) -> core::fmt::Result {
    out.write_char('"')?;
    for ch in value.chars() {
        match ch {
            '"' => write!(out, "\\\"")?,
            '\\' => write!(out, "\\\\")?,
            '\n' => write!(out, "\\n")?,
            '\r' => write!(out, "\\r")?,
            '\t' => write!(out, "\\t")?,
            ch if ch.is_control() => write!(out, "\\u{:04x}", ch as u32)?,
            ch => out.write_char(ch)?,
        }
    }
    out.write_char('"')
}

#[cfg(feature = "alloc")]
fn write_optional_ms(out: &mut impl core::fmt::Write, name: &str, value: u32) -> core::fmt::Result {
    write_json_string(out, name)?;
    out.write_char(':')?;
    if value == 0 {
        write!(out, "null")
    } else {
        write!(out, "{}", value)
    }
}

#[cfg(feature = "alloc")]
fn write_optional_u32(out: &mut impl core::fmt::Write, value: Option<u32>) -> core::fmt::Result {
    if let Some(value) = value {
        write!(out, "{}", value)
    } else {
        write!(out, "null")
    }
}

#[cfg(feature = "alloc")]
fn source_name_kind_json(kind: SourceNameKind) -> &'static str {
    match kind {
        SourceNameKind::Absolute => "absolute",
        SourceNameKind::Relative => "relative",
        SourceNameKind::Private => "private",
    }
}

#[cfg(feature = "alloc")]
fn effect_json_kind(kind: CallbackEffectKind) -> &'static str {
    match kind {
        CallbackEffectKind::Publishes => "publishes",
        CallbackEffectKind::Reads => "reads_parameter",
        CallbackEffectKind::Writes => "writes_parameter",
    }
}

// issue 0829 — `"system_default"` is the spelling `liveliness_json` below
// already uses for the same sentinel, and the one the orchestration IR emits
// for an unstated policy (`nros-cli-core/src/orchestration/schema.rs:58-88`,
// `planner.rs:1657-1671`). The metadata reports what the node REQUESTED, so a
// sentinel must survive to the JSON: printing the resolved value here would
// claim the node asked for something it left to the backend.
#[cfg(feature = "alloc")]
fn reliability_json(value: QoSReliabilityPolicy) -> &'static str {
    match value {
        QoSReliabilityPolicy::SystemDefault => "system_default",
        QoSReliabilityPolicy::Reliable => "reliable",
        QoSReliabilityPolicy::BestEffort => "best_effort",
    }
}

#[cfg(feature = "alloc")]
fn durability_json(value: QoSDurabilityPolicy) -> &'static str {
    match value {
        QoSDurabilityPolicy::SystemDefault => "system_default",
        QoSDurabilityPolicy::Volatile => "volatile",
        QoSDurabilityPolicy::TransientLocal => "transient_local",
    }
}

#[cfg(feature = "alloc")]
fn history_json(value: QoSHistoryPolicy) -> &'static str {
    match value {
        QoSHistoryPolicy::SystemDefault => "system_default",
        QoSHistoryPolicy::KeepLast => "keep_last",
        QoSHistoryPolicy::KeepAll => "keep_all",
    }
}

#[cfg(feature = "alloc")]
fn liveliness_json(value: QoSLivelinessPolicy) -> &'static str {
    match value {
        QoSLivelinessPolicy::None => "system_default",
        QoSLivelinessPolicy::Automatic => "automatic",
        QoSLivelinessPolicy::ManualByTopic => "manual_by_topic",
        QoSLivelinessPolicy::ManualByNode => "manual_by_node",
    }
}

#[cfg(feature = "alloc")]
struct ParsedInterface {
    package: StdString,
    name: StdString,
    kind: &'static str,
}

#[cfg(feature = "alloc")]
fn parse_interface(type_name: &str, fallback_kind: &'static str) -> ParsedInterface {
    let parts: StdVec<&str> = type_name.split("::").collect();
    if parts.len() >= 4 {
        let package = parts[0].into();
        let kind = match parts[1] {
            "msg" => "message",
            "srv" => "service",
            "action" => "action",
            _ => fallback_kind,
        };
        let mut type_leaf = parts[3].trim_end_matches('_');
        if type_leaf.is_empty() {
            type_leaf = parts.last().copied().unwrap_or("");
        }
        return ParsedInterface {
            package,
            name: format!("{}/{}", parts[1], type_leaf),
            kind,
        };
    }

    ParsedInterface {
        package: StdString::new(),
        name: type_name.into(),
        kind: fallback_kind,
    }
}

#[cfg(test)]
mod tests {
    /// Issue 0699 — a source path longer than the buffer must RECORD, not fail.
    ///
    /// `nros sync` died with `Metadata(NameTooLong)` purely because the user's
    /// workspace sat ~100 chars deep: rustc emits `Location::file()` absolute
    /// for a path dependency, so this field's length is set by where the user
    /// keeps their files. The tail is what the CLI keeps anyway.
    #[test]
    fn a_path_longer_than_the_buffer_keeps_its_tail() {
        // A literal, not `format!`: this crate is `no_std` and the test cfg has
        // no allocator — the same constraint the buffer itself exists under.
        let deep = concat!(
            "/deep/nested/nested/nested/nested/nested/nested/nested/nested/",
            "nested/nested/nested/nested/nested/nested/nested/nested/nested/",
            "ws/build/nros-metadata/metadata-probe/listener/src/lib.rs"
        );
        assert!(deep.len() > super::METADATA_STRING_CAPACITY);

        let got = super::copy_str_keep_tail(deep).expect("must not error on depth");
        assert!(got.len() <= super::METADATA_STRING_CAPACITY);
        assert!(
            got.ends_with("src/lib.rs"),
            "the informative tail must survive, got {got:?}"
        );
        assert!(got.starts_with('…'), "a cut must be visible, got {got:?}");

        // A path that fits is untouched — no marker, no loss.
        let short = "src/lib.rs";
        assert_eq!(super::copy_str_keep_tail(short).unwrap().as_str(), short);
    }

    use super::*;
    use crate::qos;

    #[test]
    fn source_name_kind_preserves_unresolved_names() {
        assert_eq!(
            SourceNameKind::from_source_name("/scan"),
            SourceNameKind::Absolute
        );
        assert_eq!(
            SourceNameKind::from_source_name("~/scan"),
            SourceNameKind::Private
        );
        assert_eq!(
            SourceNameKind::from_source_name("scan"),
            SourceNameKind::Relative
        );
    }

    // Phase 305 W3 (issue 0255) — the expansion/remap seam as re-exported here.
    // The exhaustive rule matrix lives with the impl (`nros_node::names`);
    // these pin the metadata-level contract each `SourceNameKind` maps to.
    #[test]
    fn expand_name_covers_each_source_name_kind() {
        // Absolute: unchanged.
        assert_eq!(
            expand_name("/scan", "lidar", "/sensing").unwrap().as_str(),
            "/scan"
        );
        // Private: node-FQN prefixed; ns=/ collapses.
        assert_eq!(
            expand_name("~/scan", "lidar", "/sensing").unwrap().as_str(),
            "/sensing/lidar/scan"
        );
        assert_eq!(
            expand_name("~/scan", "lidar", "/").unwrap().as_str(),
            "/lidar/scan"
        );
        // Relative: namespace prefixed; ns=/ collapses.
        assert_eq!(
            expand_name("scan", "lidar", "/sensing").unwrap().as_str(),
            "/sensing/scan"
        );
        assert_eq!(expand_name("scan", "lidar", "/").unwrap().as_str(), "/scan");
    }

    #[test]
    fn resolve_name_substitutes_first_matching_rule() {
        let remaps = [
            ("~/scan", "/points_raw"),
            ("/sensing/lidar/scan", "/ignored"),
        ];
        assert_eq!(
            resolve_name("~/scan", "lidar", "/sensing", remaps)
                .unwrap()
                .as_str(),
            "/points_raw"
        );
        // No match → the expansion stands.
        assert_eq!(
            resolve_name("other", "lidar", "/sensing", [("/x", "/y")])
                .unwrap()
                .as_str(),
            "/sensing/other"
        );
    }

    #[test]
    fn recorder_rejects_duplicate_stable_ids() {
        let mut recorder = MetadataRecorder::<1, 2, 1>::new();
        recorder
            .push_node(NodeId::new("node"), "talker", "/", 0)
            .unwrap();

        let first = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("pub"),
            node_id: NodeId::new("node"),
            kind: EntityKind::Publisher,
            source_name: "chatter",
            type_name: "std_msgs::msg::dds_::String_",
            type_hash: "hash",
            qos: qos::DEFAULT,
        })
        .unwrap();
        recorder.push_entity(first.clone()).unwrap();

        assert_eq!(
            recorder.push_entity(first),
            Err(NodeMetadataError::DuplicateId)
        );
    }

    #[test]
    fn recorder_rejects_duplicate_nodes_and_unknown_node_entities() {
        let mut recorder = MetadataRecorder::<1, 1, 1>::new();
        recorder
            .push_node(NodeId::new("node"), "talker", "/", 0)
            .unwrap();

        assert_eq!(
            recorder.push_node(NodeId::new("node"), "other", "/", 0),
            Err(NodeMetadataError::DuplicateId)
        );

        let entity = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("pub"),
            node_id: NodeId::new("missing_node"),
            kind: EntityKind::Publisher,
            source_name: "chatter",
            type_name: "std_msgs::msg::dds_::String_",
            type_hash: "hash",
            qos: qos::DEFAULT,
        })
        .unwrap();

        assert_eq!(
            recorder.push_entity(entity),
            Err(NodeMetadataError::UnknownNode)
        );
    }

    #[test]
    fn recorder_assigns_slots_and_source_default_names_by_declaration_order() {
        let mut recorder = MetadataRecorder::<2, 4, 3>::new();
        recorder
            .push_node(NodeId::new("node_alpha"), "talker", "/", 0)
            .unwrap();
        recorder
            .push_node(NodeId::new("node_beta"), "listener", "/demo", 42)
            .unwrap();

        assert_eq!(recorder.nodes()[0].slot, NodeSlot::new(0));
        assert_eq!(recorder.nodes()[0].source_default_name.as_str(), "talker");
        assert_eq!(recorder.nodes()[1].slot, NodeSlot::new(1));
        assert_eq!(recorder.nodes()[1].source_default_name.as_str(), "listener");

        recorder
            .push_entity(
                entity_metadata(EntityMetadataSpec {
                    id: EntityId::new("pub_chatter"),
                    node_id: NodeId::new("node_alpha"),
                    kind: EntityKind::Publisher,
                    source_name: "/chatter",
                    type_name: "std_msgs::msg::dds_::String_",
                    type_hash: "hash",
                    qos: qos::DEFAULT,
                })
                .unwrap(),
            )
            .unwrap();
        let mut subscription = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("sub_chatter"),
            node_id: NodeId::new("node_beta"),
            kind: EntityKind::Subscription,
            source_name: "/chatter",
            type_name: "std_msgs::msg::dds_::String_",
            type_hash: "hash",
            qos: qos::DEFAULT,
        })
        .unwrap();
        subscription.callback_id = Some(copy_str("on_message").unwrap());
        recorder.push_entity(subscription).unwrap();
        let mut timer = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("timer_tick"),
            node_id: NodeId::new("node_alpha"),
            kind: EntityKind::Timer,
            source_name: "",
            type_name: "",
            type_hash: "",
            qos: qos::DEFAULT,
        })
        .unwrap();
        timer.callback_id = Some(copy_str("on_tick").unwrap());
        recorder.push_entity(timer).unwrap();

        assert_eq!(recorder.entities()[0].slot, Some(EntitySlot::new(0)));
        assert_eq!(recorder.entities()[0].node_slot, Some(NodeSlot::new(0)));
        assert_eq!(recorder.entities()[0].callback_slot, None);
        assert_eq!(recorder.entities()[1].slot, Some(EntitySlot::new(1)));
        assert_eq!(recorder.entities()[1].node_slot, Some(NodeSlot::new(1)));
        assert_eq!(
            recorder.entities()[1].callback_slot,
            Some(CallbackSlot::new(0))
        );
        assert_eq!(
            recorder.entities()[2].callback_slot,
            Some(CallbackSlot::new(1))
        );

        recorder
            .push_callback_effect(
                CallbackId::new("on_tick"),
                CallbackEffectKind::Publishes,
                EntityId::new("pub_chatter"),
            )
            .unwrap();
        assert_eq!(
            recorder.callback_effects()[0].callback_slot,
            Some(CallbackSlot::new(1))
        );
        assert_eq!(
            recorder.callback_effects()[0].entity_slot,
            Some(EntitySlot::new(0))
        );
    }

    #[test]
    fn recorder_assigns_distinct_callback_slots_within_one_action_entity() {
        let mut recorder = MetadataRecorder::<1, 1, 3>::new();
        recorder
            .push_node(NodeId::new("node"), "action_node", "/", 0)
            .unwrap();
        let mut action = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("act_count"),
            node_id: NodeId::new("node"),
            kind: EntityKind::ActionServer,
            source_name: "/count",
            type_name: "example_interfaces::action::dds_::Fibonacci_",
            type_hash: "hash",
            qos: qos::DEFAULT,
        })
        .unwrap();
        action.callback_id = Some(copy_str("on_goal").unwrap());
        action.action_cancel_callback_id = Some(copy_str("on_cancel").unwrap());
        action.action_accepted_callback_id = Some(copy_str("on_accepted").unwrap());

        recorder.push_entity(action).unwrap();

        assert_eq!(
            recorder.entities()[0].callback_slot,
            Some(CallbackSlot::new(0))
        );
        assert_eq!(
            recorder.entities()[0].action_cancel_callback_slot,
            Some(CallbackSlot::new(1))
        );
        assert_eq!(
            recorder.entities()[0].action_accepted_callback_slot,
            Some(CallbackSlot::new(2))
        );
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn source_metadata_json_uses_agent_a_schema_shape() {
        // 7 entity slots, not 5: issue 0900 added the two CLIENT kinds to this
        // fixture. The recorder is fixed-capacity by design (`no_std`, static
        // storage), so a new entity here is a deliberate widening.
        let mut recorder = MetadataRecorder::<1, 7, 1>::new();
        recorder
            .push_node(NodeId::new("node_talker"), "talker", "/", 0)
            .unwrap();
        recorder
            .push_entity(
                entity_metadata(EntityMetadataSpec {
                    id: EntityId::new("pub_chatter"),
                    node_id: NodeId::new("node_talker"),
                    kind: EntityKind::Publisher,
                    source_name: "chatter",
                    type_name: "std_msgs::msg::dds_::String_",
                    type_hash: "hash",
                    qos: crate::qos::DEFAULT,
                })
                .unwrap(),
            )
            .unwrap();
        let mut timer = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("timer_publish"),
            node_id: NodeId::new("node_talker"),
            kind: EntityKind::Timer,
            source_name: "",
            type_name: "",
            type_hash: "",
            qos: crate::qos::DEFAULT,
        })
        .unwrap();
        timer.callback_id = Some(copy_str("cb_timer").unwrap());
        timer.callback_source = SourceLocationMetadata {
            artifact: copy_str("src/talker.rs").unwrap(),
            line: Some(42),
            column: Some(5),
        };
        timer.period_ms = Some(100);
        recorder.push_entity(timer).unwrap();
        let mut param = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("param_rate"),
            node_id: NodeId::new("node_talker"),
            kind: EntityKind::Parameter,
            source_name: "rate_hz",
            type_name: "",
            type_hash: "",
            qos: crate::qos::DEFAULT,
        })
        .unwrap();
        param.parameter_type = Some(ParameterType::Integer);
        param.parameter_default = Some(ParameterDefault::Integer(10));
        param.source = SourceLocationMetadata {
            artifact: copy_str("src/talker.rs").unwrap(),
            line: Some(25),
            column: Some(9),
        };
        recorder.push_entity(param).unwrap();
        let mut action = entity_metadata(EntityMetadataSpec {
            id: EntityId::new("act_count"),
            node_id: NodeId::new("node_talker"),
            kind: EntityKind::ActionServer,
            source_name: "~/count",
            type_name: "example_interfaces::action::dds_::Fibonacci_",
            type_hash: "hash",
            qos: crate::qos::DEFAULT,
        })
        .unwrap();
        action.callback_id = Some(copy_str("cb_count_goal").unwrap());
        action.callback_source = SourceLocationMetadata {
            artifact: copy_str("src/talker.rs").unwrap(),
            line: Some(90),
            column: Some(5),
        };
        action.action_cancel_callback_id = Some(copy_str("cb_count_cancel").unwrap());
        action.action_cancel_source = SourceLocationMetadata {
            artifact: copy_str("src/talker.rs").unwrap(),
            line: Some(96),
            column: Some(5),
        };
        action.action_accepted_callback_id = Some(copy_str("cb_count_accepted").unwrap());
        action.action_accepted_source = SourceLocationMetadata {
            artifact: copy_str("src/talker.rs").unwrap(),
            line: Some(104),
            column: Some(5),
        };
        recorder.push_entity(action).unwrap();
        recorder
            .push_callback_effect(
                CallbackId::new("cb_timer"),
                CallbackEffectKind::Publishes,
                EntityId::new("pub_chatter"),
            )
            .unwrap();

        // Issue 0900 — one of each client kind, on the node that also SERVES
        // (it owns the action server above), so this would catch a writer that
        // filtered by node rather than by kind.
        recorder
            .push_entity(
                entity_metadata(EntityMetadataSpec {
                    id: EntityId::new("client_fib"),
                    node_id: NodeId::new("node_talker"),
                    kind: EntityKind::ActionClient,
                    source_name: "/fibonacci",
                    type_name: "example_interfaces::action::dds_::Fibonacci_",
                    type_hash: "hash",
                    qos: crate::qos::DEFAULT,
                })
                .unwrap(),
            )
            .unwrap();
        recorder
            .push_entity(
                entity_metadata(EntityMetadataSpec {
                    id: EntityId::new("client_add"),
                    node_id: NodeId::new("node_talker"),
                    kind: EntityKind::ServiceClient,
                    source_name: "/add_two_ints",
                    type_name: "example_interfaces::srv::dds_::AddTwoInts_",
                    type_hash: "hash",
                    qos: crate::qos::DEFAULT,
                })
                .unwrap(),
            )
            .unwrap();

        let json = recorder
            .to_source_metadata_json(
                &SourceMetadataExport::new("demo_nodes_rs", "talker")
                    .executable("talker")
                    .exported_symbol("nros_node_talker")
                    .source_artifacts(&["src/talker.rs"]),
            )
            .unwrap();

        assert!(json.contains("\"version\":1"));
        assert!(json.contains("\"language\":\"rust\""));
        assert!(json.contains("\"unresolved_name\":{\"value\":\"talker\",\"kind\":\"relative\"}"));
        assert!(json.contains(
            "\"interface\":{\"package\":\"std_msgs\",\"name\":\"msg/String\",\"kind\":\"message\"}"
        ));
        assert!(json.contains("\"kind\":\"publishes\",\"entity\":\"pub_chatter\""));
        assert!(
            json.contains("\"source\":{\"artifact\":\"src/talker.rs\",\"line\":42,\"column\":5}")
        );
        assert!(json.contains("\"name\":\"rate_hz\",\"default\":10,\"read_only\":false"));
        assert!(json.contains("\"goal_callback\":\"cb_count_goal\""));
        assert!(json.contains("\"cancel_callback\":\"cb_count_cancel\""));
        assert!(json.contains("\"accepted_callback\":\"cb_count_accepted\""));
        assert!(json.contains("\"kind\":\"action_cancel\""));
        assert!(json.contains("\"kind\":\"action_accepted\""));

        // Issue 0900 — the CLIENT halves must reach the JSON, not just the
        // recorder. `record_entity` always captured them and the writer dropped
        // them, so the sidecar described only what a component SERVES. The
        // executor arena is sized from the client count, so the omission cost
        // bytes on a task stack and not merely description.
        //
        // Asserted on the SERIALISED form rather than on `recorder.entities()`:
        // the defect was entirely in the writer, so a test that stops at the
        // recorder passes against the broken tree — the vacuous shape this repo
        // keeps finding.
        assert!(
            json.contains("\"action_clients\":[{"),
            "an action client must reach the sidecar, got {json}"
        );
        assert!(
            json.contains("\"service_clients\":[{"),
            "a service client must reach the sidecar, got {json}"
        );
        // Scoped to the client ARRAYS, not to the whole document. The first
        // version of these asserts searched all of `json` and passed against a
        // broken emitter: this fixture's action SERVER carries the same
        // Fibonacci interface, so "the string appears somewhere" was true
        // whatever `write_client_json` did. A test that cannot fail is the
        // shape `check-no-vacuous-tests` exists to catch.
        let action_clients = &json[json.find("\"action_clients\":").expect("array present")..];
        let action_clients = &action_clients[..action_clients.find(']').unwrap()];
        let service_clients = &json[json.find("\"service_clients\":").expect("array present")..];
        let service_clients = &service_clients[..service_clients.find(']').unwrap()];

        // The interface must be PARSED, not echoed as a Rust path. `write_
        // client_json` shares `write_interface` with the server writers, and
        // `parse_interface` only splits the 4-segment DDS-mangled form — which
        // is what `A::ACTION_NAME` actually supplies. Pinning the parsed shape
        // is what would catch a client writer that bypassed that helper.
        assert!(
            action_clients.contains(
                "\"interface\":{\"package\":\"example_interfaces\",\"name\":\"action/Fibonacci\",\"kind\":\"action\"}"
            ),
            "action client interface must be parsed, got {action_clients}"
        );
        assert!(
            service_clients.contains(
                "\"interface\":{\"package\":\"example_interfaces\",\"name\":\"srv/AddTwoInts\",\"kind\":\"service\"}"
            ),
            "service client interface must be parsed, got {service_clients}"
        );
        // The KIND word is the only thing `write_client_json` branches on, so
        // the two arrays must not agree on it.
        assert!(
            !action_clients.contains("\"kind\":\"service\"")
                && !service_clients.contains("\"kind\":\"action\""),
            "the client kind word is what distinguishes the arrays"
        );
        // A client registers no callbacks, so it must carry none of the server
        // writers' callback fields.
        assert!(
            !action_clients.contains("callback") && !service_clients.contains("callback"),
            "a client registers no callback; emitting one would misdescribe it"
        );
        assert!(json.contains("\"generator\":\"nros-metadata-rust\""));
    }
}
