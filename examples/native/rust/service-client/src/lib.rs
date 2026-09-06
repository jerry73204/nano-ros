//! Native AddTwoInts service client — the official ROS 2 demo service client.
//!
//! Issues one request per timer tick until it succeeds, then reports the sum.
//! `main.rs`'s `nros::main!()` and the board own `nros::init`, executor open,
//! RMW registration and the spin loop.
//!
//! phase-338 W3 — was an `[package.metadata.nros.application]` example on the
//! imperative Executor API. Now Node-class like every other platform's copy.
//!
//! Output contract: the group body already emitted the success marker
//! ("Result of add_two_ints: N"); the FAILURE marker was added to every group
//! copy in this wave, because the silent `Err(_) => {}` arm made a client with
//! no server indistinguishable from a healthy one — and `services.rs`'s
//! client-without-server test asserts exactly that marker.

#![no_std]

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult, TickCtx,
    TimerDuration,
};

pub struct AddTwoIntsClient;

impl Node for AddTwoIntsClient {
    const NAME: &'static str = "add_two_ints_client";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 1, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("add_two_ints_client"))?;
        let _client = node.create_service_client_for_name::<AddTwoInts>("/add_two_ints")?;
        let _timer =
            node.create_timer_for_callback_name("issue_call", TimerDuration::from_secs(1))?;
        Ok(())
    }
}

pub struct State {
    /// Set by `on_callback` when the timer fires; drained by `tick`
    /// after dispatching the call.
    pending: bool,
    /// Set once the reply has been logged — the single-shot client
    /// then idles while the runtime keeps spinning.
    done: bool,
}

impl ExecutableNode for AddTwoIntsClient {
    type State = State;

    fn init() -> Self::State {
        State {
            pending: false,
            done: false,
        }
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "issue_call" && !state.done {
            state.pending = true;
        }
    }

    fn tick(state: &mut Self::State, ctx: &mut TickCtx<'_>) {
        if !state.pending || state.done {
            return;
        }
        state.pending = false;
        let req = AddTwoIntsRequest { a: 2, b: 3 };
        // Stack-buf sizes: AddTwoInts request = 2 × i64 + CDR header = 24 B;
        // response = 1 × i64 + header = 16 B. 64 each is generous.
        match ctx
            .call_for_name::<AddTwoIntsRequest, AddTwoIntsResponse, 64, 64>("/add_two_ints", &req)
        {
            Ok(reply) => {
                log::info!("Result of add_two_ints: {}", reply.sum);
                state.done = true;
            }
            Err(e) => {
                // phase-338 W3 — report the failure instead of swallowing it.
                // The silent arm made a client with no server look identical to
                // a healthy one on every platform, and `services.rs`'s
                // client-without-server test asserts exactly this marker. The
                // timer paces the retries, so this logs once per attempt.
                log::info!("Service call failed, retrying: {:?}", e);
            }
        }
    }
}

nros::node!(AddTwoIntsClient);
