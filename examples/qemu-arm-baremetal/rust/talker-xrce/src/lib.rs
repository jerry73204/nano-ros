//! Declarative XRCE talker node — RMW/platform/transport-agnostic logic
//! (phase-244.D1). Identical application shape to `serial_talker_pkg`; the only
//! difference is the *transport* (XRCE-over-UART), which is a board-build +
//! deploy-overlay concern (the entry builds the board with the `xrce-transport`
//! feature + sets `transport = "xrce"`, so `BoardEntry::setup_transport` installs
//! the XRCE custom-transport vtable before the RMW registers), never node logic.
//! Publishes the `std_msgs/String` demo payload (`Hello World: N`) on `/chatter` once per second.

#![no_std]

use core::fmt::Write as _;
use nros::{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeResult,
    TickCtx, TimerDuration,
};
use std_msgs::msg::String as StringMsg;

pub struct XrceTalker;

impl Node for XrceTalker {
    const NAME: &'static str = "talker_xrce";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(nros::NodeOptions::new("talker_xrce"))?;
        node.create_publisher_for_topic::<StringMsg>("/chatter")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        Ok(())
    }
}

impl ExecutableNode for XrceTalker {
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut i32, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            // Official ROS 2 demo behavior (phase-277 W4): payload
            // "Hello World: N" (N from 1) + the canonical `Publishing:` line.
            *state = state.wrapping_add(1);
            let mut msg = StringMsg::default();
            let _ = write!(msg.data, "Hello World: {}", *state);
            match ctx.publish_to_topic::<StringMsg, 64>("/chatter", &msg) {
                Ok(()) => log::info!("Publishing: '{}'", msg.data),
                Err(e) => log::error!("Publish failed: {:?}", e),
            }
        }
    }

    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {}
}

nros::node!(XrceTalker);
