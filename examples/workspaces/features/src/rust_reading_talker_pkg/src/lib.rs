//! ReadingTalker Node pkg — publishes the IN-WORKSPACE custom message
//! `custom_msgs/Reading` on `/reading` (phase-263 B6).
//!
//! `custom_msgs` is a real ROS 2 interface package (`package.xml` +
//! `msg/Reading.msg`) that lives inside this workspace. `nros sync` runs the
//! nano-ros codegen pipeline over it and emits a `generated/custom_msgs` Rust
//! crate; this Node pkg depends on `custom_msgs = "*"` (resolved to the
//! generated crate via the auto-managed `[patch.crates-io]` block). The
//! generated `Reading` type implements `RosMessage`, so it flows through the
//! ordinary typed `create_publisher_for_topic` / `publish_to_topic` path — no
//! special handling vs a stock `std_msgs` type. The differentiator: the message
//! schema is YOURS, defined and generated in-tree.

#![no_std]

use custom_msgs::msg::Reading;
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};

/// ReadingTalker — emits a synthetic sensor `Reading` every second.
pub struct ReadingTalker;

impl Node for ReadingTalker {
    const NAME: &'static str = "reading_talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("reading_talker"))?;
        let pub_reading = node.create_publisher_for_topic::<Reading>("/reading")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&pub_reading)?;
        Ok(())
    }
}

impl ExecutableNode for ReadingTalker {
    /// Monotonic sequence number stamped into each Reading.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            let msg = Reading {
                // A ramp so a subscriber sees changing data.
                temperature: 20.0 + f64::from(*state) * 0.5,
                humidity: 50.0,
                sequence: *state,
            };
            let _ = ctx.publish_to_topic::<Reading, 32>("/reading", &msg);
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(ReadingTalker);
