//! Six timers on one node — the entity count the SystemModel cannot see.
//!
//! phase-307 W6 lane 2 / issue 0257. Launch wiring has no timer entity, so this
//! node contributes ZERO to `executor_sizing::count_callbacks` while needing
//! SIX callback slots at runtime. The executor's table defaults to four.
//!
//! Sized from the model alone, `nros::main!` emits no sizing at all (a derived
//! value at or below the build default is not worth emitting) and the fifth
//! `create_timer_for_callback_name` returns `Full`. Sized from the
//! `nros sync`-produced source-metadata sidecar — which records timers, because
//! the recorder walks the same `register` this file defines — it opens at eight.
//!
//! Six is chosen, not arbitrary: it is the smallest count that over-runs the
//! four-slot default by enough that a derivation from the model (zero) cannot
//! accidentally cover it.

#![no_std]

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::Int32;

/// One callback slot each. Staggered periods so every timer fires inside a
/// short bounded spin without them all coalescing onto one tick.
const TIMER_PERIODS_MS: [(&str, u64); 6] = [
    ("tick0", 50),
    ("tick1", 60),
    ("tick2", 70),
    ("tick3", 80),
    ("tick4", 90),
    ("tick5", 100),
];

pub struct BurstTalker;

impl Node for BurstTalker {
    const NAME: &'static str = "burst_talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("burst_talker"))?;
        // The one publisher consumes NO callback slot — publishers never do.
        // It exists so the node's model wiring is non-empty while its callback
        // count stays invisible to the model.
        let chatter = node.create_publisher_for_topic::<Int32>("/chatter")?;
        for (name, period_ms) in TIMER_PERIODS_MS {
            node.create_timer_for_callback_name(name, TimerDuration::from_millis(period_ms))?;
            node.callback_for_name(name).publishes_entity(&chatter)?;
        }
        Ok(())
    }
}

impl ExecutableNode for BurstTalker {
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        *state = state.wrapping_add(1);
        let msg = Int32 { data: *state };
        let _ = ctx.publish_to_topic::<Int32, 8>("/chatter", &msg);
        // One line per timer id: the test asserts all six appear, which is only
        // possible if all six callbacks got a slot.
        log::info!("burst tick {}", callback.as_str());
    }
}

nros::node!(BurstTalker);
