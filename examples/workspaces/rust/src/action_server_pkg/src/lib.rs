//! FibonacciServer Node pkg — serves `example_interfaces/action/Fibonacci` on
//! `/fibonacci`.
//!
//! Board-agnostic Node pkg, declarative shape (phase-263 A4): `register()`
//! declares a node + an action SERVER with goal / cancel / accepted callbacks;
//! `on_callback` runs the goal/cancel *decisions* (`ctx.set_goal_response` /
//! `ctx.set_cancel_response`); `tick()` is the only place the executor is free for
//! action ops, so it walks the active goals, streams `Fibonacci` feedback, and
//! completes each goal with the result. The sibling Entry pkg's macro-generated
//! runtime owns init / executor / spin.
//!
//! Note: the declarative app-node shape does not surface the goal *payload* at
//! tick time (only the goal IDs), so the server emits a fixed-`ORDER` sequence
//! rather than the per-goal requested order — the same shape the embedded
//! `action_server_rtic_pkg` reference uses. The companion `action_client_pkg`
//! requests `order = 10`; this server returns the canonical 0..=10 sequence.

#![no_std]

use example_interfaces::action::{Fibonacci, FibonacciFeedback, FibonacciGoal, FibonacciResult};
use nros::{
    Callback, CallbackCtx, CancelResponse, ExecutableNode, GoalId, GoalResponse, GoalStatus, Node,
    NodeContext, NodeOptions, NodeResult, TickCtx,
};

/// Fixed Fibonacci order — matches the client's requested goal (the goal payload
/// is not surfaced at tick time, so the order is a compile-time constant here).
const ORDER: i32 = 10;

/// Fibonacci action server — accepts non-negative goal orders and completes each
/// accepted goal with the canonical Fibonacci sequence.
pub struct FibonacciServer;

impl Node for FibonacciServer {
    const NAME: &'static str = "fibonacci_server";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 1);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("fibonacci_server"))?;
        let _action = node.create_action_server_for_name_with_callbacks::<Fibonacci>(
            "/fibonacci",
            "on_goal",
            "on_cancel",
            "on_accepted",
        )?;
        Ok(())
    }
}

impl ExecutableNode for FibonacciServer {
    /// Goals completed so far (informational).
    type State = u32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_goal" => {
                let response = match ctx.message::<FibonacciGoal>() {
                    Ok(goal) if goal.order >= 0 => GoalResponse::AcceptAndExecute,
                    _ => GoalResponse::Reject,
                };
                let _ = ctx.set_goal_response(response);
            }
            "on_cancel" => {
                let _ = ctx.set_cancel_response(CancelResponse::Accept);
            }
            "on_accepted" => {
                // Feedback + result are driven from `tick()` — the only place the
                // executor is free for action ops.
            }
            _ => {}
        }
    }

    fn tick(state: &mut Self::State, ctx: &mut TickCtx<'_>) {
        // Snapshot active goals into a fixed-cap stack list so the borrow checker
        // lets us issue mutable executor ops after the visit returns.
        let mut pending: heapless::Vec<GoalId, 4> = heapless::Vec::new();
        ctx.for_each_active_goal_for_name("/fibonacci", &mut |goal_id, status| {
            if matches!(status, GoalStatus::Accepted | GoalStatus::Executing) {
                let _ = pending.push(*goal_id);
            }
        });

        for goal_id in pending {
            let mut seq: heapless::Vec<i32, 64> = heapless::Vec::new();
            for i in 0..=ORDER {
                let next = match i {
                    0 => 0,
                    1 => 1,
                    _ => {
                        let len = seq.len();
                        seq[len - 1] + seq[len - 2]
                    }
                };
                let _ = seq.push(next);
                let feedback = FibonacciFeedback {
                    sequence: seq.clone(),
                };
                let _ = ctx.publish_feedback_for_name::<FibonacciFeedback, 256>(
                    "/fibonacci",
                    &goal_id,
                    &feedback,
                );
            }

            let result = FibonacciResult { sequence: seq };
            let _ = ctx.complete_goal_for_name::<FibonacciResult, 256>(
                "/fibonacci",
                &goal_id,
                GoalStatus::Succeeded,
                &result,
            );
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(FibonacciServer);
