//! ESP32-C3 QEMU Listener — Node pkg (agnostic application logic).
//!
//! Subscribes to `std_msgs/String` on `/chatter` and tracks the last
//! value seen.
//!
//! Node pkg shape: `register()` declares the node + subscription;
//! `ExecutableNode::on_callback("on_chatter")` runs the subscription
//! body. The board crate's `BoardEntry` runtime owns `Executor::open`,
//! RMW registration, hardware/transport bring-up and the spin loop —
//! this source carries only the listener behaviour.

#![no_std]

use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};
use std_msgs::msg::String as StringMsg;

/// Listener component — last value seen on `/chatter`.
pub struct Listener;

impl Node for Listener {
    const NAME: &'static str = "listener";

    // phase-391 W5-endgame (issue 0857) — exact bounds: one subscription,
    // no publishers/services/actions, so the static cell pays ~nothing.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("listener"))?;
        let _sub =
            node.create_subscription_for_callback_name::<StringMsg>("on_chatter", "/chatter")?;
        nros_board_esp32_qemu::nros_log::log_info!(
            nros_board_esp32_qemu::nros_log::get_logger("listener"),
            "Subscriber created for topic: /chatter"
        );
        Ok(())
    }
}

impl ExecutableNode for Listener {
    /// Number of messages seen on `/chatter`.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_chatter"
            && let Ok(msg) = ctx.message::<StringMsg>()
        {
            *state = state.wrapping_add(1);
            // Observable per-receive line (routed to the console by the
            // board's log writer) — the e2e harness asserts on `I heard:`.
            nros_board_esp32_qemu::nros_log::log_info!(
                nros_board_esp32_qemu::nros_log::get_logger("listener"),
                "I heard: [{}]",
                msg.data
            );
        }
    }
}

nros::node!(Listener);
