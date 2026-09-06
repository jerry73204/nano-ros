//! Param-talker Node pkg — publishes `std_msgs/Int32` on `/chatter`, timer rate
//! configured by a launch parameter.
//!
//! phase-264 W4 (RFC-0004 §10) — the launch `<node>` carries a
//! `<param name="publish_period_ms" value="…"/>` child. `nros::main!`
//! **compile-bakes** that value into the generated entry and seeds it into this
//! node's `NodeContext`; `register()` reads it back with `ctx.param(name)` and drives
//! the timer period from the baked launch value (W4a). `[param_services]` registers the
//! ROS 2 param services + a volatile store seeded from the same initial (W4b), and the
//! callback re-reads the **live** value each tick via `ctx.parameter::<i64>(name)` (W4c)
//! and publishes it. So: change the `<param value=…/>` + rebuild → both the rate (baked)
//! and the published value change; `ros2 param set publish_period_ms N` on the running
//! node → the published value changes live (the baked timer rate does not — it is fixed
//! at register). Reconfigured values are volatile (lost at the next boot).

#![no_std]

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::Int32;

/// Default timer period (ms) when the launch file declares no `publish_period_ms`.
const DEFAULT_PERIOD_MS: u64 = 1000;

/// Param-talker — counter state + chatter publish on every tick.
pub struct ParamTalker;

impl Node for ParamTalker {
    const NAME: &'static str = "param_talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        // W4a — read the launch-baked initial parameter value. `ctx.param` returns the
        // compile-time `<param value=…/>` string (or `None` if undeclared).
        let period_ms: u64 = ctx
            .param("publish_period_ms")
            .and_then(|s| s.parse().ok())
            .unwrap_or(DEFAULT_PERIOD_MS);

        let mut node = ctx.create_node(NodeOptions::new("param_talker"))?;
        let pub_chatter = node.create_publisher_for_topic::<Int32>("/chatter")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(period_ms))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&pub_chatter)?;
        Ok(())
    }
}

impl ExecutableNode for ParamTalker {
    /// Monotonic counter — the next int32 to publish.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            // W4c — re-read the LIVE parameter value each tick. Boots at the baked
            // launch initial; a `ros2 param set publish_period_ms N` changes what we
            // publish here (the baked timer rate stays fixed). `-1` if param services
            // are off / undeclared.
            let live = ctx
                .parameter::<i64>("publish_period_ms")
                .map(|v| v as i32)
                .unwrap_or(-1);
            let msg = Int32 { data: live };
            let _ = ctx.publish_to_topic::<Int32, 8>("/chatter", &msg);
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(ParamTalker);
