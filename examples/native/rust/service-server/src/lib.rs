//! Native AddTwoInts service server — the official ROS 2 demo service.
//!
//! Node pkg shape: `register()` declares the node + service server and logs the
//! readiness marker; `on_callback("handle_add")` answers each request.
//! `main.rs`'s `nros::main!()` and the board own `nros::init`, executor open,
//! RMW registration and the spin loop.
//!
//! phase-338 W3 — was an `[package.metadata.nros.application]` example written
//! against the imperative Executor API. Now Node-class like every other
//! platform's copy, byte-identical to them (the `example_portability` gate
//! asserts it).
//!
//! Output contract preserved: the group body already emitted
//! `SERVICE_SERVER_READY_MARKER` ("Waiting for service requests"), "Incoming
//! request" and the `a: {} b: {}` line. Only the imperative version's
//! "Spin error" is gone, and nothing asserts it — spin belongs to the board in
//! this shape.

#![no_std]

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};
use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};

pub struct AddTwoIntsServer;

impl Node for AddTwoIntsServer {
    const NAME: &'static str = "add_two_ints_server";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 1, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("add_two_ints_server"))?;
        let _srv = node.create_service_server_for_name_with_callback::<AddTwoInts>(
            "/add_two_ints",
            "handle_add",
        )?;
        log::info!("Waiting for service requests");
        Ok(())
    }
}

impl ExecutableNode for AddTwoIntsServer {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "handle_add" {
            if let Ok(req) = ctx.message::<AddTwoIntsRequest>() {
                log::info!("Incoming request");
                log::info!("a: {} b: {}", req.a, req.b);
                let reply = AddTwoIntsResponse { sum: req.a + req.b };
                let _ = ctx.reply::<AddTwoIntsResponse, 64>(&reply);
            }
        }
    }
}

nros::node!(AddTwoIntsServer);
