//! Listener Node pkg — subscribes to `std_msgs/Int32` on `/chatter`.
//!
//! Board-agnostic Node pkg. The sibling Entry pkg (`robot_entry`)
//! wires it onto a board via the `nros::main!(launch = "demo_bringup:...")`
//! macro, which emits one `listener_pkg::register(runtime)?;` call per
//! matching `<node>` entry in the launch file.
//!
//! `register()` declares the node + a subscription whose `on_message`
//! callback decodes the incoming `std_msgs::msg::Int32` payload.

#![no_std]

use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};
use std_msgs::msg::Int32;

/// Listener — counts the int32 messages seen on `/chatter`.
pub struct Listener;

impl Node for Listener {
    const NAME: &'static str = "listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("listener"))?;
        let sub_chatter =
            node.create_subscription_for_callback_name::<Int32>("on_message", "/chatter")?;
        node.callback_for_name("on_message")
            .reads_entity(&sub_chatter)?;
        Ok(())
    }
}

impl ExecutableNode for Listener {
    /// Count of messages received so far.
    type State = u32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_message" {
            if let Ok(msg) = ctx.message::<Int32>() {
                log::info!("I heard: {}", msg.data);
                *state = state.wrapping_add(1);
            }
        }
    }
}

nros::node!(Listener);
