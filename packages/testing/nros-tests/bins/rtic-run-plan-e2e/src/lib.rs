//! RTIC `nros::main!()` run-plan E2E node (introduced in phase-216.B).
//!
//! Self-contained standalone fixture (issue 0100): the node logic lives in this
//! crate's `lib.rs`; `main.rs` is the `nros::main!()` RTIC Entry. The synthetic
//! `__nros_e2e` callback (fired by the board's `e2e-synthetic-callback`
//! feature) signals a clean QEMU semihosting exit on the first delivery.

#![no_std]

use nros::{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeResult, TickCtx,
};

pub struct E2eNode;

pub struct E2eState {
    fired: bool,
}

impl Node for E2eNode {
    const NAME: &'static str = "rtic_run_plan_e2e";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(_ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        Ok(())
    }
}

impl ExecutableNode for E2eNode {
    type State = E2eState;

    fn init() -> Self::State {
        E2eState { fired: false }
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "__nros_e2e" && !state.fired {
            state.fired = true;
            nros_board_mps2_an385::exit_success();
        }
    }

    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {}
}

nros::node!(E2eNode);
