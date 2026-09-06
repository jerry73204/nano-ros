//! Phase 228.G fixture — control node (tier `high`).
//!
//! Labels its timer with the `ctrl` callback group; `system.toml` maps that
//! group to the `high` tier. `nros::node!` emits the `register` symbol the
//! `nros::main!()` per-tier emit calls. Publishes a monotonic counter on `/ctrl`
//! every 10 ms so a subscriber can observe the high-tier cadence (Track D).

#![no_std]

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::Int32;

pub struct Control;

impl Node for Control {
    const NAME: &'static str = "control_node";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("control_node"))?;
        // issue 0572 — say that this node's register() RAN. A tier that never
        // dispatches and a node that was never registered on it look identical
        // from outside: the topic is simply silent.
        log::info!("Control::register on a tier admitting group `ctrl`");
        node.callback_group("ctrl")?;
        let pub_ctrl = node.create_publisher_for_topic::<Int32>("/ctrl")?;
        let _t = node.create_timer_for_callback_name("on_ctrl", TimerDuration::from_millis(10))?;
        node.callback_for_name("on_ctrl")
            .publishes_entity(&pub_ctrl)?;
        Ok(())
    }
}

impl ExecutableNode for Control {
    /// Monotonic tick counter — the next int32 to publish on `/ctrl`.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_ctrl" {
            let msg = Int32 { data: *state };
            // issue 0572 — the result used to be discarded, which made "the
            // timer never fired" and "every publish failed" the same
            // observation from outside: `/ctrl` silent, guest console empty.
            // The FIRST tick and every error now say so. Only the first, because
            // this tier fires every 10 ms and a per-tick line would swamp the
            // serial console the e2e observers read.
            match ctx.publish_to_topic::<Int32, 8>("/ctrl", &msg) {
                Ok(()) => {
                    if *state == 0 {
                        log::info!("on_ctrl: first publish OK (tier `high` is dispatching)");
                    }
                }
                Err(e) => log::error!("on_ctrl: publish to /ctrl FAILED: {e:?}"),
            }
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(Control);
