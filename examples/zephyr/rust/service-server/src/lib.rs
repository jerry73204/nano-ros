//! Zephyr AddTwoInts service server.
//!
//! Declarative: node + service server with a `handle_add` callback.
//! Body: reads the typed request, writes the typed reply through the
//! reply sink. Generated runtime owns init / executor / spin.

#![no_std]

mod app_main;


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
        // Readiness marker the e2e harness greps before driving the client.
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
// Issue 0330 — force-link the selected RMW backend into this staticlib. The
// facade macro below used to emit these references itself, which named two
// concrete backends in an RMW-agnostic layer; naming one here is correct,
// because selecting an RMW is exactly what this crate does. Registration is
// still done by `nros_app_register_backends` — this is only a DCE anchor
// (issues 0155 / 0163). cyclonedds needs none: its register entry lives in the
// Zephyr module's C++ lib, which the image already links.

