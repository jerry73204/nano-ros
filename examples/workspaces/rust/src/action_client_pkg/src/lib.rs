//! FibonacciClient Node pkg — calls the `example_interfaces/action/Fibonacci`
//! server on `/fibonacci` once, then republishes the final Fibonacci number on
//! `/fib_result`.
//!
//! Declarative shape with BOTH dispatch surfaces (phase-263 A4, the first
//! workspace example to exercise the action-CLIENT path):
//!   - `register()` declares an action CLIENT (result + feedback callbacks) and a
//!     `/fib_result` publisher.
//!   - `tick(TickCtx)` sends the goal exactly once (gated by a `sent` flag) — goal
//!     send is a tick-only op, the executor is free there.
//!   - `on_callback("on_feedback")` observes the streamed partial sequence;
//!     `on_callback("on_result")` reads the terminal result and publishes its last
//!     element on `/fib_result`. Publishing (not logging) keeps the result
//!     observable on the wire — the workspace shape inits no log sink yet (A5).

#![no_std]

use example_interfaces::action::{Fibonacci, FibonacciFeedback, FibonacciGoal, FibonacciResult};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult, TickCtx,
};
use std_msgs::msg::Int32;

/// Fibonacci action client — one-shot goal, result republished on `/fib_result`.
pub struct FibonacciClient;

/// `sent` keeps the one-shot goal send idempotent across ticks.
pub struct ClientState {
    sent: bool,
}

impl Node for FibonacciClient {
    const NAME: &'static str = "fibonacci_client";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 0, 1, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("fibonacci_client"))?;
        let _client = node.create_action_client_with_callbacks_for_name::<Fibonacci>(
            "/fibonacci",
            "on_result",
            "on_feedback",
        )?;
        let pub_result = node.create_publisher_for_topic::<Int32>("/fib_result")?;
        node.callback_for_name("on_result")
            .publishes_entity(&pub_result)?;
        Ok(())
    }
}

impl ExecutableNode for FibonacciClient {
    type State = ClientState;

    fn init() -> Self::State {
        ClientState { sent: false }
    }

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        match callback.as_str() {
            "on_feedback" => {
                // Partial sequence streamed mid-goal — observed, nothing to emit.
                let _ = ctx.message::<FibonacciFeedback>();
            }
            "on_result" => {
                if let Ok(result) = ctx.message::<FibonacciResult>() {
                    let last = result.sequence.last().copied().unwrap_or(0);
                    let msg = Int32 { data: last };
                    let _ = ctx.publish_to_topic::<Int32, 8>("/fib_result", &msg);
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
        if ctx
            .send_goal_for_name::<FibonacciGoal, 32>("/fibonacci", &goal)
            .is_ok()
        {
            state.sent = true;
        }
    }
}

nros::node!(FibonacciClient);
