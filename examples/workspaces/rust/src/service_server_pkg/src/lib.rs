//! AddServer Node pkg — serves `example_interfaces/srv/AddTwoInts` on
//! `/add_two_ints`.
//!
//! Board-agnostic Node pkg, declarative shape: `register()` declares a service
//! SERVER entity whose callback id equals the service name; the
//! `ExecutableNode::on_callback("/add_two_ints")` body reads the request via
//! `ctx.message::<AddTwoIntsRequest>()`, computes the sum, and writes the reply
//! with `ctx.reply::<AddTwoIntsResponse, N>()`. The sibling Entry pkg's
//! macro-generated runtime owns init / executor / spin.
//!
//! This is the first workspace example to exercise the declarative service-server
//! API (phase-263 A1) — the everyday-ROS RPC pattern in a real Node pkg.

#![no_std]

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};
use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};

/// AddServer — stateless `AddTwoInts` service server.
pub struct AddServer;

impl Node for AddServer {
    const NAME: &'static str = "add_server";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 1, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let mut node = ctx.create_node(NodeOptions::new("add_server"))?;
        // Name doubles as the stable entity id AND the callback id, so the
        // dispatch fires `on_callback("/add_two_ints")`.
        let _svc = node.create_service_server_for_name::<AddTwoInts>("/add_two_ints")?;
        Ok(())
    }
}

impl ExecutableNode for AddServer {
    type State = ();

    fn init() -> Self::State {}

    fn on_callback(_state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "/add_two_ints"
            && let Ok(req) = ctx.message::<AddTwoIntsRequest>()
        {
            let resp = AddTwoIntsResponse { sum: req.a + req.b };
            // 16 bytes covers one int64 + CDR header.
            let _ = ctx.reply::<AddTwoIntsResponse, 16>(&resp);
        }
    }
}

nros::node!(AddServer);
