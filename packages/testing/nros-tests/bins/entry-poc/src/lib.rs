//! Phase 212.N.9 entry-poc lib — companion to `src/main.rs`.
//!
//! Exposes the `pub fn register(runtime)` symbol the
//! `nros::main!()` no-arg form emits as
//! `::entry_poc::register(runtime)?;`.
//!
//! The body is **intentionally empty** — this POC verifies the
//! macro emit + the Board boot path reach user code without needing
//! a live zenohd / executor. The actual `register()` symbol is
//! defined by the `nros::node!()` macro below; we provide an
//! `EntryPoc` that satisfies the trait but creates no real
//! subscriptions so `cargo build && ./target/debug/entry-poc`
//! exits 0 with no running RMW broker. Production Entry pkgs would
//! declare real nodes here.

#![no_std]

use nros::{Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeResult};

pub struct EntryPoc;

impl Node for EntryPoc {
    const NAME: &'static str = "entry_poc";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(_ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        // No node / publisher / subscription — keeps the POC's
        // exit-0 contract under the no-zenohd CI environment.
        Ok(())
    }
}

impl ExecutableNode for EntryPoc {
    type State = ();
    fn init() -> Self::State {}
    fn on_callback(_state: &mut Self::State, _cb: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {}
}

nros::node!(EntryPoc);
