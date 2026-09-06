//! phase-308 — the process-global recorder that non-Rust adapters feed.
//!
//! The Rust producer needs no global: `record_node_metadata::<C>` owns a
//! `MetadataRecorder` on the stack and hands it to the component as a
//! `NodeContext` sink. A C++ component cannot be driven that way — its
//! `configure(nros::Node&)` reaches the runtime through the `nros_cpp_*` C ABI,
//! and by then there is no Rust value to thread through. So the C/C++ adapter
//! records into a global instead.
//!
//! **The global is the only thing that is new.** It is the SAME
//! [`MetadataRecorder`] type, filled through the same [`push_node`] /
//! [`push_entity`] calls, and dumped through the same
//! [`to_source_metadata_json`] — which is the point. The recorder, the slot
//! accounting and the schema emitter are one implementation; only the adapter
//! that feeds them is per-language. A second recorder would be a second
//! definition of "what is a callback slot", and the count those sidecars carry
//! is the whole reason they exist (issue 0257).
//!
//! [`push_node`]: MetadataRecorder::push_node
//! [`push_entity`]: MetadataRecorder::push_entity
//! [`to_source_metadata_json`]: MetadataRecorder::to_source_metadata_json
//!
//! # Why a global is safe here
//!
//! This module is compiled only under the `metadata-mode` feature, which exists
//! for the host probe: a single-threaded process that constructs one component,
//! runs its declaration path, writes one file and exits. It never ships in
//! firmware — the feature is not reachable from any board or entry build, and
//! enabling it replaces the RMW with a backend that transports nothing.
//!
//! # Node attribution
//!
//! The RMW seam does NOT carry a node: `create_publisher(session, topic_name,
//! type_name, …)` has no participant argument, because by that layer the
//! entity's owning node is already resolved away. A recording backend alone
//! would therefore produce a sidecar whose entities belong to no node — the
//! total count would still be right, but `nodes[]` would be a fiction for any
//! multi-node component.
//!
//! [`begin_node`] closes that: `nros_cpp_node_create*` is hooked to open a node
//! and make it current, and every entity recorded afterwards attributes to it.
//! `configure()` declares one node's entities at a time, so a cursor is the
//! correct model — not a heuristic.

use alloc::{string::String, vec::Vec};
// issue 0687 follow-up — the PORTABLE mutex, not `std::sync::Mutex`. This was
// the only thing in the file `alloc` could not supply, and it was keeping
// `metadata-mode` a `std` capability for no reason the code supports: the guard
// beside the feature said "writes a file and exits", but the file write is in
// `nros-cpp`'s `metadata_hooks`, not here.
//
// A spin lock is the right shape rather than a compromise: this module is
// compiled only under `metadata-mode`, whose documented contract (above) is a
// SINGLE-THREADED probe process, so the lock exists to make a global `Sync`,
// not to arbitrate contention that happens. The `spin` edge rides the feature,
// so a build without `metadata-mode` — every firmware image — does not get it.
use nros_rmw::sync::Mutex;

use crate::node_metadata::{
    EntityId, EntityKind, EntityMetadataSpec, MetadataRecorder, NodeId, SourceMetadataExport,
    entity_metadata,
};

/// The recorder every non-Rust adapter feeds, plus the current-node cursor.
#[derive(Default)]
struct State {
    recorder: MetadataRecorder,
    /// Node id set by the most recent [`begin_node`]; entities attribute here.
    current_node: Option<String>,
    /// Per-node entity counter, so generated ids are unique and stable.
    seq: usize,
    /// Type names seen so far. The ABI hands them over as borrowed C strings
    /// while `EntityMetadata::type_name` is `&'static str`, so the probe leaks
    /// them deliberately — a process that writes one file and exits.
    leaked: Vec<&'static str>,
}

fn state() -> &'static Mutex<State> {
    static STATE: Mutex<State> = Mutex::new(State {
        recorder: MetadataRecorder::new(),
        current_node: None,
        seq: 0,
        leaked: Vec::new(),
    });
    &STATE
}

fn intern(s: &str) -> &'static str {
    alloc::boxed::Box::leak(String::from(s).into_boxed_str())
}

/// Discard everything recorded so far. For tests; a probe records once.
pub fn reset() {
    state().with(|st| {
        st.recorder = MetadataRecorder::new();
        st.current_node = None;
        st.seq = 0;
    });
}

/// Open a node and make it current. Returns false if the recorder is full or
/// the node is a duplicate — never silently drops, because a dropped node makes
/// every entity after it vanish from the count.
///
/// phase-428 W6 remainder: `#[must_use]` because the doc comment above states
/// the consequence of ignoring the answer and `bool` says nothing at a call
/// site that drops it. RFC-0089 Part I's fourth row — a difference the compiler
/// does not point at must be made loud by other means.
#[must_use]
pub fn begin_node(name: &str, namespace: &str, domain_id: u32) -> bool {
    state().with(|st| {
        let id = String::from(name);
        if st
            .recorder
            .push_node(NodeId::new(&id), name, namespace, domain_id)
            .is_err()
        {
            return false;
        }
        st.current_node = Some(id);
        true
    })
}

/// Record one entity against the current node.
///
/// `callback_id` and `period_ms` are what distinguish a timer from a
/// subscription in the emitted schema; both are `None` for entities that carry
/// neither. Returns false when there is no current node or the recorder is
/// full.
///
/// phase-428 W6 remainder: see [`begin_node`] — a dropped `false` here is an
/// entity missing from the emitted metadata with nothing said about it.
#[must_use]
pub fn record_entity(
    kind: EntityKind,
    source_name: &str,
    type_name: &str,
    callback_id: Option<&str>,
    period_ms: Option<u64>,
) -> bool {
    state().with(|st| {
        let Some(node_id) = st.current_node.clone() else {
            return false;
        };
        let type_name = intern(type_name);
        st.leaked.push(type_name);
        st.seq += 1;
        let id = alloc::format!("{}#{}", source_name, st.seq);
        let spec = EntityMetadataSpec {
            id: EntityId::new(&id),
            node_id: NodeId::new(&node_id),
            kind,
            source_name,
            type_name,
            type_hash: "",
            qos: crate::QoSProfile::default(),
        };
        let Ok(mut entity) = entity_metadata(spec) else {
            return false;
        };
        entity.period_ms = period_ms;
        if let Some(cb) = callback_id {
            entity.callback_id = crate::node_metadata::metadata_string(cb).ok();
        }
        st.recorder.push_entity(entity).is_ok()
    })
}

/// Serialize everything recorded so far, through the one schema emitter.
pub fn to_json(export: &SourceMetadataExport<'_>) -> Result<String, core::fmt::Error> {
    state().with(|st| st.recorder.to_source_metadata_json(export))
}

/// Entities recorded so far — for adapter tests and the probe's own sanity
/// check ("a component that declared nothing is a bug, not an empty sidecar").
pub fn entity_count() -> usize {
    state().with(|st| st.recorder.entities().len())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node_metadata::EntityKind;

    /// The recorder is process-global by design, so tests that touch it cannot
    /// run concurrently under cargo's thread-per-test harness. Serialize them
    /// rather than making the production type carry a test-only mode.
    // `std::sync::Mutex`, NOT the portable one the module now uses: these tests
    // genuinely contend (cargo runs them on a thread each) and hold the lock
    // across a whole test body, which is exactly the case a spin lock is wrong
    // for. Production never contends — see the module doc.
    static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The C++ shape that motivates phase-307 and 308 together: one
    /// subscription the SystemModel can see, plus timers it cannot. Recorded
    /// through the shared recorder, emitted by the shared serializer.
    ///
    /// Serialized with `language: "cpp"` — which was a hardcoded `"rust"`
    /// literal until this phase, because Rust was the only producer.
    #[test]
    fn records_a_cpp_component_through_the_shared_serializer() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset();
        assert!(begin_node("cpp_talker", "/", 0));
        assert!(record_entity(
            EntityKind::Subscription,
            "/chatter",
            "std_msgs/msg/Int32",
            Some("on_chatter"),
            None
        ));
        for i in 0..5 {
            assert!(record_entity(
                EntityKind::Timer,
                &alloc::format!("tick{i}"),
                "",
                Some(&alloc::format!("tick{i}")),
                Some(100)
            ));
        }
        assert_eq!(entity_count(), 6);

        let export = SourceMetadataExport::new("talker_pkg", "cpp_talker")
            .executable("cpp_talker")
            .language("cpp");
        let json = to_json(&export).expect("serialize");
        assert!(json.contains("\"language\":\"cpp\""), "got: {json}");
        assert!(json.contains("\"package\":\"talker_pkg\""), "got: {json}");
        assert!(
            json.contains("\"executable\":\"cpp_talker\""),
            "got: {json}"
        );
        // Five timers, one subscription — the counts the sizing bake reads.
        assert_eq!(json.matches("\"period_ms\"").count(), 5, "got: {json}");
        assert!(json.contains("\"subscribers\":[{"), "got: {json}");
        reset();
    }

    /// issue 0900 — the CLIENT halves reach the sidecar.
    ///
    /// `record_entity` has always accepted `ActionClient`/`ServiceClient`, and
    /// the serializer dropped them: a sidecar described only what a component
    /// SERVES. The executor arena is sized from the client count, so the
    /// omission had a size cost (74,240 vs 16,384 bytes) and not merely a
    /// descriptive one. (phase-392 W6: this used to add "on the task stack",
    /// which was inherited from 0900 and false since phase-271 — the arena is
    /// borrowed from caller-supplied backing, a `.bss` static on every entry
    /// path. The size cost is real; the placement claim was not.)
    #[test]
    fn client_entities_reach_the_sidecar() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset();
        assert!(begin_node("client_node", "/", 0));
        assert!(record_entity(
            EntityKind::ActionClient,
            "/fibonacci",
            "example_interfaces/action/Fibonacci",
            None,
            None
        ));
        assert!(record_entity(
            EntityKind::ServiceClient,
            "/add_two_ints",
            "example_interfaces/srv/AddTwoInts",
            None,
            None
        ));
        let export = SourceMetadataExport::new("client_pkg", "client_node").language("rust");
        let json = to_json(&export).expect("serialize");
        assert!(json.contains("\"action_clients\":[{"), "got: {json}");
        assert!(json.contains("\"service_clients\":[{"), "got: {json}");
        // A client registers no callbacks. The server writers emit a
        // `callback`/`goal_callback` field; the client writer must not, which is
        // why it is a separate function rather than a reused one.
        let tail = &json[json.find("\"action_clients\"").expect("array")..];
        let end = tail.find(']').expect("array end");
        assert!(
            !tail[..end].contains("callback"),
            "a client must carry no callback field: {}",
            &tail[..end]
        );
        reset();
    }

    /// An entity recorded with no open node is a bug in the adapter, and must
    /// be refused rather than silently attributed or dropped — a dropped entity
    /// is an under-sized executor at boot.
    #[test]
    fn entities_without_a_node_are_refused() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset();
        assert!(!record_entity(
            EntityKind::Timer,
            "tick",
            "",
            Some("tick"),
            Some(10)
        ));
        assert_eq!(entity_count(), 0);
        reset();
    }
}
