//! QosListener Node pkg — subscribes `/qos_chatter` with a QoS profile that
//! MATCHES the `rust_qos_talker_pkg` publisher (phase-263 B4).
//!
//! Declared via `create_subscription_for_topic_with_qos` (the declarative
//! QoS-override surface) with the same `reliable() + transient_local() +
//! depth(10)` contract. QoS must match for the endpoints to connect: a default
//! (volatile) subscriber would still connect to a reliable+transient_local
//! publisher, but would NOT replay the pre-join history — matching the profile
//! is what makes TRANSIENT_LOCAL observable. Republishes the running receive
//! count on `/qos_ok` so a cross-process subscriber can watch the QoS-matched
//! delivery path end-to-end (Track D).

#![no_std]

use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};
use std_msgs::msg::Int32;

/// QosListener — counts QoS-matched messages received on `/qos_chatter`.
pub struct QosListener;

impl Node for QosListener {
    const NAME: &'static str = "qos_listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("qos_listener"))?;
        // Same QoS contract as the publisher — re-uses the talker pkg's profile
        // so the two are guaranteed to match.
        let qos = rust_qos_talker_pkg::qos_profile();
        let _sub = node.create_subscription_for_topic_with_qos::<Int32>("/qos_chatter", qos)?;
        let pub_ok = node.create_publisher_for_topic::<Int32>("/qos_ok")?;
        node.callback_for_name("/qos_chatter")
            .publishes_entity(&pub_ok)?;
        Ok(())
    }
}

impl ExecutableNode for QosListener {
    /// Running count of received messages.
    type State = u32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "/qos_chatter" && ctx.message::<Int32>().is_ok() {
            *state = state.wrapping_add(1);
            let msg = Int32 {
                data: *state as i32,
            };
            let _ = ctx.publish_to_topic::<Int32, 8>("/qos_ok", &msg);
        }
    }
}

nros::node!(QosListener);
