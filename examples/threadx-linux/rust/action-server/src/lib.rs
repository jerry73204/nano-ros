//! ThreadX Linux Action Server — Node pkg.
//!
//! Serves `example_interfaces/Fibonacci` on `/fibonacci`: accepts goals with a
//! non-negative order, publishes feedback, and completes them from `tick`. The
//! generated runtime owns init / executor / spin.
//!
//! phase-338 W3.e — body converged onto the group-A source.
//!
//! phase-394 / issue 0856 — `tick()` also serves the CANCEL path: a goal in
//! `GoalStatus::Canceling` completes as `Canceled` carrying whatever was
//! computed before the cancel arrived, rather than reporting success it did
//! not achieve. `NROS_FIB_STEP_TICKS` paces the loop at one term per N ticks
//! so a cancel has a window to land in at all — unpaced, order 10 finishes in
//! about four milliseconds and every cancel arrives after the goal is done.

#![no_std]

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

/// issue 0450 — the order most recently ACCEPTED, so `tick` can compute the
/// sequence the client actually asked for. `for_each_active_goal_for_name`
/// surfaces the goal id and status but not the request payload, which is why an
/// earlier body hardcoded its output: it had nothing else to go on. One slot is
/// enough for the demo (a single goal at a time); a server handling concurrent
/// goals would key this by `GoalId`.
pub struct ServerState {
    /// Order of the goal most recently accepted.
    order: i32,
    /// Sequence built so far. Lives across ticks because paced mode emits one
    /// term per step, and because a CANCEL has to return what was computed
    /// before the cancel arrived.
    sequence: nros::heapless::Vec<i32, 64>,
    /// Ticks between terms. 0 = unpaced: the whole sequence in one tick.
    ///
    /// Pacing exists so a cancel has a window to land in. Unpaced, this server
    /// finishes order 10 in about four milliseconds, and ROS 2's own cancel
    /// client cancels three seconds after sending the goal -- so the goal has
    /// always succeeded long before the cancel arrives, and the cancel path
    /// cannot be exercised at all. Set `NROS_FIB_STEP_TICKS` to slow it down.
    step_ticks: u32,
    /// Ticks since the last term was published.
    wait: u32,
}

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
        log::info!("Waiting for action goals");
        Ok(())
    }
}

impl ExecutableNode for FibonacciServer {
    type State = ServerState;

    fn init() -> Self::State {
        // Paced mode is opt-in so the default is byte-identical to before:
        // unpaced, the whole sequence goes out in the tick that sees the goal.
        // Every existing test depends on that.
        //
        // `option_env!`, not `std::env::var`: this crate is `no_std`, so the
        // knob is read at COMPILE time --
        //     NROS_FIB_STEP_TICKS=2000 cargo build --release
        // -- and `parse` on a `&str` is core, not std.
        let step_ticks = match option_env!("NROS_FIB_STEP_TICKS") {
            Some(v) => match v.parse::<u32>() {
                Ok(n) => n,
                Err(_) => 0,
            },
            None => 0,
        };
        ServerState {
            order: 0,
            sequence: nros::heapless::Vec::new(),
            step_ticks,
            wait: 0,
        }
    }

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_goal" => {
                let order = ctx.message::<FibonacciGoal>().map(|g| g.order).ok();
                if let Some(order) = order {
                    log::info!("Received goal request with order {}", order);
                }
                let accept = order.map(|o| o >= 0).unwrap_or(false);
                if accept {
                    // Remember what was requested. Reading the order and then
                    // ignoring it is the part of the old body most likely to
                    // mislead a reader (issue 0450).
                    _state.order = order.unwrap_or(0).min(MAX_ORDER);
                    _state.sequence.clear();
                    _state.wait = 0;
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
                // Per-spin work runs in `tick()` (the only place the
                // executor is free for action ops).
            }
            _ => {}
        }
    }

    fn tick(state: &mut Self::State, ctx: &mut TickCtx<'_>) {
        // Collect goal ids AND statuses first — typed feedback / result calls
        // borrow `ctx` mutably so they cannot run inside `visit`.
        let mut goals: nros::heapless::Vec<(nros::GoalId, GoalStatus), 4> =
            nros::heapless::Vec::new();
        ctx.for_each_active_goal_for_name("/fibonacci", &mut |goal_id, status: GoalStatus| {
            let _ = goals.push((*goal_id, status));
        });

        for (goal_id, status) in goals {
            // A cancel the `on_cancel` callback ACCEPTED shows up here as
            // `Canceling`. Honour it: finish the goal as `Canceled`, returning
            // whatever was computed before it arrived. Accepting a cancel and
            // then reporting `Succeeded` anyway is a lie the client cannot
            // detect, and it is what this server did before.
            if status == GoalStatus::Canceling {
                log::info!("Goal canceled");
                let result = FibonacciResult {
                    sequence: state.sequence.clone(),
                };
                let _ = ctx.complete_goal_for_name::<FibonacciResult, 256>(
                    "/fibonacci",
                    &goal_id,
                    GoalStatus::Canceled,
                    &result,
                );
                state.sequence.clear();
                state.wait = 0;
                continue;
            }

            // issue 0450 — compute the sequence, do not recite it. `Fibonacci`
            // is the canonical ROS 2 action demo BECAUSE the sequence is built
            // incrementally and each step is streamed as feedback; a server
            // that published a fixed `[0, 1, 1]` whatever the request
            // demonstrated the plumbing while misrepresenting the example.
            let order = state.order;

            if state.step_ticks > 0 {
                // Paced: one term per `step_ticks` ticks, so a cancel has a
                // window to land in.
                if state.wait > 0 {
                    state.wait -= 1;
                    continue;
                }
                state.wait = state.step_ticks;
                push_next(&mut state.sequence);
                log::info!("Publish feedback");
                let feedback = FibonacciFeedback {
                    sequence: state.sequence.clone(),
                };
                let _ = ctx.publish_feedback_for_name::<FibonacciFeedback, 256>(
                    "/fibonacci",
                    &goal_id,
                    &feedback,
                );
                if (state.sequence.len() as i32) <= order {
                    continue; // more terms to go; finish on a later tick
                }
            } else {
                // Unpaced (the default): the whole sequence in this tick.
                state.sequence.clear();
                for _ in 0..=order {
                    if !push_next(&mut state.sequence) {
                        break;
                    }
                    log::info!("Publish feedback");
                    let feedback = FibonacciFeedback {
                        sequence: state.sequence.clone(),
                    };
                    let _ = ctx.publish_feedback_for_name::<FibonacciFeedback, 256>(
                        "/fibonacci",
                        &goal_id,
                        &feedback,
                    );
                }
            }

            let result = FibonacciResult {
                sequence: state.sequence.clone(),
            };
            let _ = ctx.complete_goal_for_name::<FibonacciResult, 256>(
                "/fibonacci",
                &goal_id,
                GoalStatus::Succeeded,
                &result,
            );
            state.sequence.clear();
            state.wait = 0;
            log::info!("Goal succeeded");
        }
    }
}

/// Append the next Fibonacci term. Returns false when the buffer is full.
fn push_next(sequence: &mut nros::heapless::Vec<i32, 64>) -> bool {
    let next = match sequence.len() {
        0 => 0,
        1 => 1,
        n => sequence[n - 1].saturating_add(sequence[n - 2]),
    };
    sequence.push(next).is_ok()
}

nros::node!(FibonacciServer);
