//! Listener Node pkg — subscribes the relative topic `chatter`, republishes the
//! running receive count on `heard` (phase-263 B5).
//!
//! Relative topic names so the launch `<group ns=…>` / `<remap>` apply. Lives in
//! a separate sub-launch (`sensors.launch.xml`) pulled in via `<include>` — the
//! advanced-launch feature this workspace exists to exercise.

#![no_std]

use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};
use std_msgs::msg::Int32;

pub struct Listener;

impl Node for Listener {
    const NAME: &'static str = "listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("listener"))?;
        let _sub = node.create_subscription_for_topic::<Int32>("chatter")?;
        let pub_heard = node.create_publisher_for_topic::<Int32>("heard")?;
        node.callback_for_name("chatter")
            .publishes_entity(&pub_heard)?;
        Ok(())
    }
}

impl ExecutableNode for Listener {
    type State = u32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "chatter" && ctx.message::<Int32>().is_ok() {
            *state = state.wrapping_add(1);
            let msg = Int32 {
                data: *state as i32,
            };
            let _ = ctx.publish_to_topic::<Int32, 8>("heard", &msg);
        }
    }
}

nros::node!(Listener);
