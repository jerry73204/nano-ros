//! QEMU MPS2-AN385 RTIC Fibonacci Action Client node logic.
//!
//! Sends an `example_interfaces/Fibonacci` goal on `/fibonacci`. Declarative,
//! platform/RMW-agnostic Node: `register()` declares node + action client;
//! `tick()` issues a one-shot `send_goal` (then stays idempotent);
//! feedback/result callbacks land via `on_callback` once codegen wires the
//! result-future + feedback-stream + `GoalStatusArray` subscribers through to
//! dispatch. The entry crate's `nros::main!()` + the RTIC board
//! (`nros-board-mps2-an385`) own hardware/network bring-up, executor open,
//! RMW registration, and the RTIC dispatch loop. Locator/domain live in the
//! entry's `[package.metadata.nros.deploy.rtic-mps2-an385]` — never here.

#![no_std]

use example_interfaces::action::{Fibonacci, FibonacciFeedback, FibonacciGoal, FibonacciResult};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult, TickCtx,
};

// Diagnostics route through `nros-log`.

/// Fibonacci action client — declares the client, then issues a single goal
/// (`order = 10`) on the first `tick`.
pub struct FibonacciClient;

impl Node for FibonacciClient {
    const NAME: &'static str = "fibonacci_action_client";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 1, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("fibonacci_action_client"))?;
        let _client = node.create_action_client_with_callbacks_for_name::<Fibonacci>(
            "/fibonacci",
            "on_result",
            "on_feedback",
        )?;
        Ok(())
    }
}

pub struct State {
    /// Set once the goal has been sent — keeps `tick` idempotent.
    sent: bool,
}

impl ExecutableNode for FibonacciClient {
    type State = State;

    fn init() -> Self::State {
        State { sent: false }
    }

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        // Feedback / result auto-driven by the executor's action-client seam
        // (Phase 212.M-F.23) and dispatched here by callback name.
        match callback.as_str() {
            "on_feedback" => {
                if let Ok(f) = ctx.message::<FibonacciFeedback>() {
                    log::info!("Next number in sequence received: {:?}", f.sequence);
                }
            }
            "on_result" => {
                if let Ok(r) = ctx.message::<FibonacciResult>() {
                    log::info!("Result received: {:?}", r.sequence);
                }
            }
            _ => {}
        }
    }

    fn tick(state: &mut Self::State, ctx: &mut TickCtx<'_>) {
        if state.sent {
            return;
        }
        let goal = FibonacciGoal { order: 10 };
        // 32 B is more than enough for one `i32` + CDR header.
        if ctx
            .send_goal_for_name::<FibonacciGoal, 32>("/fibonacci", &goal)
            .is_ok()
        {
            log::info!("Sending goal");
            state.sent = true;
            log::info!("Goal accepted by server, waiting for result");
        }
        // On a `Runtime` stub error, `sent` stays false — the next tick retries.
    }
}

nros::node!(FibonacciClient);
