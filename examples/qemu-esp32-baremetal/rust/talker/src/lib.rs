//! ESP32-C3 QEMU Talker — Node pkg (agnostic application logic).
//!
//! Publishes `std_msgs/String` (`Hello World: N`) on `/chatter` once per second.
//!
//! Node pkg shape: `register()` declares the node + publisher + timer;
//! `ExecutableNode::on_callback("on_tick")` runs the timer body (bump
//! counter, publish). The board crate's `BoardEntry` runtime owns
//! `Executor::open`, RMW registration, hardware/transport bring-up and
//! the spin loop — this source carries only the talker behaviour.

#![no_std]

use core::fmt::Write as _;
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::String as StringMsg;

/// Talker component — counter state + chatter publish on every tick.
pub struct Talker;

impl Node for Talker {
    const NAME: &'static str = "talker";

    // phase-391 W5-endgame (issue 0857) — exact bounds: one publisher.
    // The per-class cell is a STATIC sized by these; the knob default
    // (8 slots x ~1.35 KiB) collapsed the esp32 single-image stack.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("talker"))?;
        let pub_chatter = node.create_publisher_for_topic::<StringMsg>("/chatter")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&pub_chatter)?;
        Ok(())
    }
}

impl ExecutableNode for Talker {
    /// Monotonic counter — the next sequence number to publish.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            *state = state.wrapping_add(1);
            let mut msg = StringMsg::default();
            let _ = write!(msg.data, "Hello World: {}", *state);
            let _ = ctx.publish_to_topic::<StringMsg, 64>("/chatter", &msg);
            // Observable per-publish line (routed to the console by the board's
            // log writer) — the e2e harness waits for `Publishing:` to confirm
            // the 1 Hz timer fired + the session published. Mirrors the
            // official ROS 2 demo talker (phase-277 W4).
            nros_board_esp32_qemu::nros_log::log_info!(
                nros_board_esp32_qemu::nros_log::get_logger("talker"),
                "Publishing: '{}'",
                msg.data
            );
        }
    }
}

nros::node!(Talker);
