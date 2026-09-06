//! QEMU MPS2-AN385 RTIC AddTwoInts Service Server node logic.
//!
//! Serves an `example_interfaces/AddTwoInts` service on `/add_two_ints`.
//! Declarative, platform/RMW-agnostic Node: `register()` declares node + service
//! server; `on_callback("on_add")` reads the typed request, sums the two ints,
//! and writes the typed reply. The entry crate's `nros::main!()` + the RTIC board
//! (`nros-board-mps2-an385`) own hardware/network bring-up, executor open,
//! RMW registration, and the RTIC dispatch loop. Locator/domain live in the
//! entry's `[package.metadata.nros.deploy.rtic-mps2-an385]` — never here.

#![no_std]

use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};
use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult};

// Diagnostics route through `nros-log`.

/// AddTwoInts service server — sums the two request ints on every call.
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
            "on_add",
        )?;
        log::info!("Waiting for service requests...");
        Ok(())
    }
}

impl ExecutableNode for AddTwoIntsServer {
    /// Count of handled requests.
    type State = u32;

    fn init() -> Self::State {
        0
    }

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {
        if callback.as_str() == "on_add"
            && let Ok(req) = ctx.message::<AddTwoIntsRequest>()
        {
            log::info!("Incoming request");
            log::info!("a: {} b: {}", req.a, req.b);
            let resp = AddTwoIntsResponse { sum: req.a + req.b };
            let _ = ctx.reply::<AddTwoIntsResponse, 64>(&resp);
            *state = state.wrapping_add(1);
        }
    }
}

nros::node!(AddTwoIntsServer);
