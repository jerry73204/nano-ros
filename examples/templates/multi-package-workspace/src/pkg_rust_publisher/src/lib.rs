//! Multi-package-workspace demo — Rust publisher Node pkg (agnostic logic).
//!
//! Publishes `std_msgs/Int32` on `/chatter` once per second. Pairs with
//! `pkg_c_talker` + `pkg_cpp_listener` for a mixed-language demo.
//!
//! Node pkg shape: `register()` declares the node + publisher + timer;
//! `ExecutableNode::on_callback("on_tick")` runs the timer body (bump
//! counter, publish). The board's `BoardEntry` runtime — emitted by the
//! sibling `main.rs` `nros::main!()` — owns `Executor::open`, RMW
//! registration and the spin loop, so this source carries only the
//! publisher behaviour.

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};
use std_msgs::msg::Int32;

/// Publisher component — counter state + chatter publish on every tick.
pub struct Talker;

impl Node for Talker {
    const NAME: &'static str = "talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("talker"))?;
        let pub_chatter = node.create_publisher_for_topic::<Int32>("/chatter")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&pub_chatter)?;
        Ok(())
    }
}

impl ExecutableNode for Talker {
    /// Monotonic counter — the next int32 to publish.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            let msg = Int32 { data: *state };
            let _ = ctx.publish_to_topic::<Int32, 8>("/chatter", &msg);
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(Talker);
