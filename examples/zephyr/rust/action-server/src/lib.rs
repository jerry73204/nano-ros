//! Zephyr Fibonacci action server.
//!
//! Declarative: node + action server with distinct goal / cancel /
//! accepted callbacks. Bodies:
//!  - `on_goal` accepts non-negative orders, rejects otherwise.
//!  - `on_cancel` always accepts.
//!  - `on_accepted` is a no-op (the per-spin work runs in `tick()`).
//!  - `tick()` walks every active goal, publishes feedback, completes.

#![no_std]

mod app_main;

use example_interfaces::action::{Fibonacci, FibonacciFeedback, FibonacciGoal, FibonacciResult};
use nros::{
    Callback, CallbackCtx, CancelResponse, ExecutableNode, GoalResponse, GoalStatus, Node,
    NodeContext, NodeOptions, NodeResult, TickCtx,
};

/// issue 0450 — the largest order this demo will compute.
///
/// Bounds BOTH stack buffers: the sequence is a `heapless::Vec<i32, 64>`, and
/// the CDR-encoded feedback/result must fit the 256-byte buffers below
/// (4-byte length + 4 bytes per element, so 50 elements is 208). The client
/// asks for 10. A larger request is clamped rather than refused — the demo's
/// job is to show streaming feedback, not to police arithmetic.
const MAX_ORDER: i32 = 50;

pub struct FibonacciServer;

impl Node for FibonacciServer {
    const NAME: &'static str = "fibonacci_action_server";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 1);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("fibonacci_action_server"))?;
        let _action = node.create_action_server_for_name_with_callbacks::<Fibonacci>(
            "/fibonacci",
            "on_goal",
            "on_cancel",
            "on_accepted",
        )?;
        // Readiness marker the e2e harness greps before sending a goal.
        log::info!("Waiting for action goals");
        Ok(())
    }
}

impl ExecutableNode for FibonacciServer {
    /// issue 0450 — the order most recently ACCEPTED, so `tick` can compute the
    /// sequence the client actually asked for. `for_each_active_goal_for_name`
    /// surfaces the goal id and status but not the request payload, which is
    /// why the previous body hardcoded its output: it had nothing else to go
    /// on. One slot is enough for the demo (a single goal at a time); a server
    /// handling concurrent goals would key this by `GoalId`.
    type State = i32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_goal" => {
                let order = ctx.message::<FibonacciGoal>().ok().map(|g| g.order);
                if let Some(order) = order {
                    log::info!("Received goal request with order {}", order);
                }
                let accept = matches!(order, Some(o) if o >= 0);
                if accept {
                    // Remember what was requested. Reading the order and then
                    // ignoring it is the part of the old body most likely to
                    // mislead a reader (issue 0450).
                    *_state = order.unwrap_or(0).min(MAX_ORDER);
                }
                let _ = ctx.set_goal_response(if accept {
                    GoalResponse::AcceptAndExecute
                } else {
                    GoalResponse::Reject
                });
            }
            "on_cancel" => {
                let _ = ctx.set_cancel_response(CancelResponse::Accept);
            }
            "on_accepted" => {
                // No imperative work here; the executor drives feedback
                // and result through `tick()` when it is free for action ops.
                log::info!("Executing goal");
            }
            _ => {}
        }
    }

    fn tick(state: &mut Self::State, ctx: &mut TickCtx<'_>) {
        // Collect goal ids first — typed feedback / result calls borrow
        // `ctx` mutably so they can't run inside `visit`.
        let mut goals: nros::heapless::Vec<(nros::GoalId, i32), 4> = nros::heapless::Vec::new();
        ctx.for_each_active_goal_for_name("/fibonacci", &mut |goal_id, _status: GoalStatus| {
            let _ = goals.push((*goal_id, 0));
        });

        for (goal_id, _unused) in goals {
            log::info!("Executing goal");
            // issue 0450 — compute the sequence, do not recite it. `Fibonacci`
            // is the canonical ROS 2 action demo BECAUSE the sequence is built
            // incrementally and each step is streamed as feedback; a server
            // that published a fixed `[0, 1, 1]` whatever the request
            // demonstrated the plumbing while misrepresenting the example.
            let order = *state;
            let mut sequence: nros::heapless::Vec<i32, 64> = nros::heapless::Vec::new();
            for i in 0..=order {
                let next = match i {
                    0 => 0,
                    1 => 1,
                    _ => {
                        let n = sequence.len();
                        sequence[n - 1].saturating_add(sequence[n - 2])
                    }
                };
                if sequence.push(next).is_err() {
                    break;
                }
                let feedback = FibonacciFeedback {
                    sequence: sequence.clone(),
                };
                log::info!("Publish feedback");
                let _ = ctx.publish_feedback_for_name::<FibonacciFeedback, 256>(
                    "/fibonacci",
                    &goal_id,
                    &feedback,
                );
            }

            let result = FibonacciResult { sequence };
            let _ = ctx.complete_goal_for_name::<FibonacciResult, 256>(
                "/fibonacci",
                &goal_id,
                GoalStatus::Succeeded,
                &result,
            );
            log::info!("Goal succeeded");
        }
    }
}

nros::node!(FibonacciServer);
// Issue 0330 — force-link the selected RMW backend into this staticlib. The
// facade macro below used to emit these references itself, which named two
// concrete backends in an RMW-agnostic layer; naming one here is correct,
// because selecting an RMW is exactly what this crate does. Registration is
// still done by `nros_app_register_backends` — this is only a DCE anchor
// (issues 0155 / 0163). cyclonedds needs none: its register entry lives in the
// Zephyr module's C++ lib, which the image already links.
