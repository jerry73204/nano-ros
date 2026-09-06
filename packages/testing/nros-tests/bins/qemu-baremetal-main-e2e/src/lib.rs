//! Phase 244.D1 — bare-metal `nros::main!()` BoardEntry E2E node.
//!
//! Self-contained standalone fixture (issue 0100): the node logic lives in this
//! crate's `lib.rs` (was a sibling `qemu_baremetal_e2e_pkg`); `main.rs` is the
//! `nros::main!()` Form-1 Entry dispatching to this crate's `register`.
//!
//! Platform/RMW-agnostic application logic (RFC-0024 shape): registers a
//! periodic timer; the first tick signals E2E success via the board's
//! semihosting `exit_success()`. The boot scaffold (reset → `BoardEntry::run`
//! → executor → spin) is owned by `nros::main!()` + `nros-board-mps2-an385`.

#![no_std]

use nros::{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeResult,
    TickCtx, TimerDuration,
};

pub struct E2eNode;

impl Node for E2eNode {
    const NAME: &'static str = "qemu_baremetal_e2e";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(nros::NodeOptions::new("qemu_baremetal_e2e_node"))?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(100))?;
        Ok(())
    }
}

impl ExecutableNode for E2eNode {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut (), callback: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            // Boot → BoardEntry::run → executor → spin → callback all reached:
            // signal a clean QEMU semihosting exit. (When no RMW peer is present
            // the executor never opens; the board's `Executor::open failed:`
            // banner is the alternate boot proof — see the E2E test.)
            nros_board_mps2_an385::exit_success();
        }
    }

    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {}
}

nros::node!(E2eNode);
