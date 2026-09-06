//! Declarative serial listener node — RMW/platform/transport-agnostic logic
//! (phase-244.D1). Identical application shape to `listener_pkg`; the only
//! difference between the ethernet and serial listeners is the *transport*,
//! which is a board-build + deploy-overlay concern (the entry pkg builds the
//! board with the `serial` feature + sets a `serial/UART_0#…` locator), never
//! node logic. Declares a `/chatter` subscription bound to `on_message`; each
//! typed `std_msgs/String` delivery logs `I heard: [Hello World: N]`.

#![no_std]

use nros::{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeOptions,
    NodeResult, TickCtx,
};
use std_msgs::msg::String as StringMsg;

pub struct SerialListenerNode;

impl Node for SerialListenerNode {
    const NAME: &'static str = "serial_listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("serial_listener"))?;
        node.create_subscription_for_callback_name::<StringMsg>("on_message", "/chatter")?;
        log::info!("Subscribing to /chatter (std_msgs/String)");
        log::info!("Subscriber declared");
        log::info!("Waiting for messages...");
        Ok(())
    }
}

impl ExecutableNode for SerialListenerNode {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut (), callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_message"
            && let Ok(msg) = ctx.message::<StringMsg>()
        {
            log::info!("I heard: [{}]", msg.data);
        }
    }

    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {}
}

nros::node!(SerialListenerNode);
