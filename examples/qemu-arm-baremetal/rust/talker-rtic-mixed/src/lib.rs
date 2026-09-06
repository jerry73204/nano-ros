//! Declarative mixed-priority RTIC talker node — RMW/platform-agnostic logic.
//!
//! Migrated from the legacy hand-written `#[rtic::app]` + `#[init]`/`#[task]` +
//! manual `Executor` shape (Phase 244.D1). The old example created the
//! publisher in `#[init]`, drove transport I/O from a low-priority `net_poll`
//! task, and published from a *higher*-priority `publish` task on a 1000 ms
//! `Mono::delay` — the `ffi-sync` feature masking interrupts around FFI calls so
//! the preempting publisher could not corrupt zenoh-pico's global state.
//!
//! In the declarative shape there is a single dispatch context, so the explicit
//! two-priority split (low `net_poll` vs high `publish`) does not survive the
//! migration — the boot scaffold owns spinning and the timer fires the publish
//! callback cooperatively. The pub/sub behavior and output markers are
//! preserved: the `std_msgs/String` demo payload (`Hello World: N`) on `/chatter` once per second,
//! logging `Publishing: 'Hello World: N'` (the marker the QEMU E2E asserts).

#![no_std]

use core::fmt::Write as _;
use nros::{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeResult,
    TickCtx, TimerDuration,
};
use std_msgs::msg::String as StringMsg;

pub struct Talker;

impl Node for Talker {
    const NAME: &'static str = "talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 0, 0);

    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(nros::NodeOptions::new("talker"))?;
        node.create_publisher_for_topic::<StringMsg>("/chatter")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        Ok(())
    }
}

impl ExecutableNode for Talker {
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

nros::node!(Talker);
