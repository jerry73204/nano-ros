//! Rust heartbeat Node pkg for the mixed C/C++/Rust workspace.
//!
//! The package intentionally has no message dependency: it proves that a
//! Rust Node pkg can be linked through the CMake Entry path and registered
//! beside C and C++ Node packages.
//!
//! phase-263 C2c — `no_std` so the SAME node compiles for the host (native /
//! threadx-linux) AND for the genuinely-`no_std` embedded Rust targets
//! (`thumbv7m-none-eabi` on FreeRTOS, `x86_64-unknown-none` on Zephyr native_sim).
//! The body uses only the `nros` API + a `u32` counter — no `std` surface — so the
//! `nros` `std` feature was dropped (alloc-only).
#![no_std]

use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
};

pub struct Heartbeat;

impl Node for Heartbeat {
    const NAME: &'static str = "heartbeat";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("heartbeat"))?;
        let timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(500))?;
        node.callback_for_name("on_tick").writes_entity(&timer)?;
        Ok(())
    }
}

impl ExecutableNode for Heartbeat {
    type State = u32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {
        if callback.is_named("on_tick") {
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(Heartbeat);
