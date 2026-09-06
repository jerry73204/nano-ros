//! ThreadX QEMU RISC-V Service Client — app-node logic.
//!
//! Calls `example_interfaces/AddTwoInts` on `/add_two_ints`. This is an **app
//! node** (it owns the image's entry, via `src/app_main.rs`), not a workspace
//! Node lib — but the *logic* is still platform/RMW-agnostic: `register()`
//! declares node + service client + a 1 Hz driver timer; `on_callback("issue_call")`
//! arms the request, and `tick` dispatches the call — `TickCtx` is the only
//! place `&mut Executor` is free (see `TickCtx` docs). The client sends ONE
//! fixed request (2, 3); the timer retries until the call succeeds (discovery
//! warm-up), then goes quiet. The board (`nros-board-threadx-qemu-riscv64`,
//! `BoardEntry::run`) owns `nros::init`, executor open, RMW registration, and
//! the spin loop. RMW selection (zenoh / cyclonedds) lives in
//! `Cargo.toml [features]`; the locator + domain in
//! `[package.metadata.nros.deploy.threadx-qemu-riscv64]` — never here.

#![no_std]

mod app_main;

// Keep the board crate (panic handler + allocator + critical-section impl)
// linked into the `staticlib`. phase-369 — `app_main!` names it on both RMW
// paths now, so this anchor is belt-and-braces rather than load-bearing
// (issue #205 — the per-example critical-section anchor moved into the board
// crate).

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

// CycloneDDS / CMake firmware path: the C `startup.c::main` calls
// `tx_kernel_enter()` and dispatches to this `app_main` *inside* the ThreadX app
// thread — so the kernel is already running here. `run_app_thread` runs the
// post-kernel body (open executor + `register` + spin); it must NOT re-enter the
// kernel via `BoardEntry::run`. phase-369 — BOTH RMWs build through CMake and
// compile this; the cargo `main.rs` path is gone. It is thin — the board owns
// executor open, RMW registration, and the spin loop; the `nros::node!()`-emitted
// `register` declares the client. No manual `Executor::open` / `register_rmw` /
// spin loop / hardcoded locator in the example.

