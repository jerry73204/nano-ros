//! AddClient Node pkg — calls `example_interfaces/srv/AddTwoInts` on
//! `/add_two_ints` once a second and republishes the returned sum on `/sum`.
//!
//! Declarative shape with BOTH dispatch surfaces (phase-263 A1, the first
//! workspace example to exercise the service-CLIENT path):
//!   - `register()` declares a service CLIENT, a 1 Hz timer, and a `/sum`
//!     publisher.
//!   - `on_callback("on_tick")` (timer) just arms a `pending` flag — the executor
//!     is busy mid-dispatch, so it cannot issue a blocking client call there.
//!   - `tick(TickCtx)` runs *between* callback dispatch, where the executor is
//!     free: it issues the blocking `call_for_name` and publishes the sum. The
//!     flag gates it to the 1 Hz timer cadence (tick itself runs every spin).

#![no_std]

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};
use nros::{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult, TickCtx,
    TimerDuration,
};
use std_msgs::msg::Int32;

/// AddClient — `a` counts up; each armed tick calls `add(a, 1)` + publishes the sum.
pub struct AddClient;

/// Left operand + a "timer fired, please call" flag set by `on_callback` and
/// consumed by `tick`.
pub struct ClientState {
    a: i64,
    pending: bool,
}

impl Node for AddClient {
    const NAME: &'static str = "add_client";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(1, 0, 1, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("add_client"))?;
        let _client = node.create_service_client_for_name::<AddTwoInts>("/add_two_ints")?;
        let pub_sum = node.create_publisher_for_topic::<Int32>("/sum")?;
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        node.callback_for_name("on_tick")
            .publishes_entity(&pub_sum)?;
        Ok(())
    }
}

impl ExecutableNode for AddClient {
    type State = ClientState;

    fn init() -> Self::State {
        ClientState {
            a: 0,
            pending: false,
        }
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_tick" {
            // Arm the call; `tick` issues it when the executor is free.
            state.pending = true;
        }
    }

    fn tick(state: &mut Self::State, ctx: &mut TickCtx<'_>) {
        if !state.pending {
            return;
        }
        state.pending = false;
        let req = AddTwoIntsRequest { a: state.a, b: 1 };
        // 24-byte request (two int64 + header), 16-byte response (one int64).
        if let Ok(resp) = ctx
            .call_for_name::<AddTwoIntsRequest, AddTwoIntsResponse, 24, 16>("/add_two_ints", &req)
        {
            let msg = Int32 {
                data: resp.sum as i32,
            };
            let _ = ctx.publish_to_topic::<Int32, 8>("/sum", &msg);
        }
        state.a = state.a.wrapping_add(1);
    }
}

nros::node!(AddClient);
