//! Remap-talker Node pkg — publishes `std_msgs/Int32` on the PRIVATE name
//! `~/out` at 4 Hz.
//!
//! phase-306 W3/W4 (issue 0255) — the code never names an absolute topic: it
//! declares (and publishes through) the source-level private name `~/out`.
//! The system model namespaces this node under `/island` and remaps `~/out`
//! to `/remapped_out`; `nros::main!(model = …)` bakes the rules into
//! `runtime.remaps`, and entity creation expands `~/out` against the node
//! identity (`/island/remap_talker/out`) then resolves it through the rules —
//! so the name that hits the wire is `/remapped_out`. Change the model's
//! `remaps` + rebuild → the wire topic moves with no code edit.

#![no_std]

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::Int32;

/// Publish period — 4 Hz keeps the e2e's ≥3-receives budget short.
const PERIOD_MS: u64 = 250;

/// Remap-talker — counter state + private-name publish on every tick.
pub struct RemapTalker;

impl Node for RemapTalker {
    const NAME: &'static str = "remap_talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("remap_talker"))?;
        // PRIVATE source name — the remap seam (not this code) decides the
        // wire name.
        let pub_out = node.create_publisher_for_topic::<Int32>("~/out")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(PERIOD_MS))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&pub_out)?;
        Ok(())
    }
}

impl ExecutableNode for RemapTalker {
    /// Monotonic counter — the next int32 to publish.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            *state = state.wrapping_add(1);
            let msg = Int32 { data: *state };
            // Publish through the SAME source-level name the entity was
            // declared with — entity ids stay source names; only the wire
            // name is remapped.
            let _ = ctx.publish_to_topic::<Int32, 8>("~/out", &msg);
        }
    }
}

nros::node!(RemapTalker);
