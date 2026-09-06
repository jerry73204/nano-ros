//! ReadingListener Node pkg — subscribes the in-workspace `custom_msgs/Reading`
//! on `/reading` and republishes the sequence it saw on `/reading_seq`
//! (phase-263 B6).
//!
//! Consumes the SAME generated `custom_msgs` crate as the talker — proof the
//! in-workspace interface package round-trips through codegen, publish, and
//! deserialize. The republished `/reading_seq` (a stock `std_msgs/Int32`) lets a
//! cross-process subscriber confirm the custom message was decoded correctly
//! (Track D).

#![no_std]

use custom_msgs::msg::Reading;
use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};
use std_msgs::msg::Int32;

/// ReadingListener — echoes the last decoded `Reading.sequence` on `/reading_seq`.
pub struct ReadingListener;

impl Node for ReadingListener {
    const NAME: &'static str = "reading_listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("reading_listener"))?;
        let _sub = node.create_subscription_for_topic::<Reading>("/reading")?;
        let pub_seq = node.create_publisher_for_topic::<Int32>("/reading_seq")?;
        node.callback_for_name("/reading")
            .publishes_entity(&pub_seq)?;
        Ok(())
    }
}

impl ExecutableNode for ReadingListener {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "/reading"
            && let Ok(reading) = ctx.message::<Reading>()
        {
            let msg = Int32 {
                data: reading.sequence,
            };
            let _ = ctx.publish_to_topic::<Int32, 8>("/reading_seq", &msg);
        }
    }
}

nros::node!(ReadingListener);
