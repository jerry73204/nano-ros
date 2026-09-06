//! Phase 228.G fixture — telemetry node (tier `low`).
//!
//! Publishes a monotonic counter on `/telem` every 100 ms so a subscriber can
//! observe the low-tier cadence (Track D) — ~10× slower than the high-tier
//! `/ctrl` node.

#![no_std]

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::Int32;

pub struct Telem;

impl Node for Telem {
    const NAME: &'static str = "telem_node";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("telem_node"))?;
        // issue 0572 — say that this node's register() RAN. A tier that never
        // dispatches and a node that was never registered on it look identical
        // from outside: the topic is simply silent.
        log::info!("Telem::register on a tier admitting group `telem`");
        node.callback_group("telem")?;
        let pub_telem = node.create_publisher_for_topic::<Int32>("/telem")?;
        let _t =
            node.create_timer_for_callback_name("on_telem", TimerDuration::from_millis(100))?;
        node.callback_for_name("on_telem")
            .publishes_entity(&pub_telem)?;
        Ok(())
    }
}

impl ExecutableNode for Telem {
    /// Monotonic tick counter — the next int32 to publish on `/telem`.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_telem" {
            let msg = Int32 { data: *state };
            // issue 0572 — the result used to be discarded, which made "the
            // timer never fired" and "every publish failed" the same
            // observation from outside: `/telem` silent, guest console empty.
            // The FIRST tick and every error now say so. Only the first, because
            // this tier fires every 10 ms and a per-tick line would swamp the
            // serial console the e2e observers read.
            match ctx.publish_to_topic::<Int32, 8>("/telem", &msg) {
                Ok(()) => {
                    if *state == 0 {
                        log::info!("on_telem: first publish OK (tier `low` is dispatching)");
                    }
                }
                Err(e) => log::error!("on_telem: publish to /telem FAILED: {e:?}"),
            }
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(Telem);
