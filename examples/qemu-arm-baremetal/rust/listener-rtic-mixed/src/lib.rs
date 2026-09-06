//! Node pkg for the QEMU MPS2-AN385 mixed-priority RTIC listener.
//!
//! Platform/RMW-agnostic application logic (RFC-0024 shape): declares a
//! subscription on `/chatter` bound to the `on_message` callback; each typed
//! `std_msgs/String` delivery logs `I heard: [Hello World: N]`. The boot scaffold
//! (reset → `RticBoardEntry::init_hardware_with_deploy` → executor → dispatch)
//! is owned by `nros::main!()` + `nros-board-mps2-an385` (Phase 244.D1
//! enabler) — none of it appears here.
//!
//! Migration note (Phase 244.D1 Wave B): the old `#[rtic::app]` form split the
//! work across two RTIC tasks at distinct priorities — `net_poll` (priority 1)
//! drove transport I/O while `listen` (priority 2) could preempt it, with the
//! `ffi-sync` feature masking interrupts during FFI calls. The declarative node
//! shape collapses to a single dispatch surface: the executor spin + deferred
//! callback dispatch is owned by the board Entry, so the explicit mixed-priority
//! split does not survive here. The pub/sub behavior and output markers are
//! preserved.

#![no_std]

use nros::{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeOptions,
    NodeResult, TickCtx,
};
use std_msgs::msg::String as StringMsg;

pub struct ListenerNode;

impl Node for ListenerNode {
    const NAME: &'static str = "listener";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("listener"))?;
        node.create_subscription_for_callback_name::<StringMsg>("on_message", "/chatter")?;
        // Preserve the old example's startup marker exactly.
        log::info!("Waiting for messages on /chatter...");
        Ok(())
    }
}

impl ExecutableNode for ListenerNode {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut (), callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_message"
            && let Ok(msg) = ctx.message::<StringMsg>()
        {
            // Preserve the old example's per-message marker exactly.
            log::info!("I heard: [{}]", msg.data);
        }
    }

    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {}
}

nros::node!(ListenerNode);
