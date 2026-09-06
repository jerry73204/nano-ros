//! Phase 212.M.5.a.3 fixture — talker component (Rust, no_std).
//!
//! Declares a single node. The BSP baker links this pkg's mangled
//! register symbol (`__nros_component_talker_pkg_register`, emitted by
//! `nros::node!()`) into the firmware and dispatches it from
//! `nros_system_run`. The M.5.a.2 BSP-side `DeclarativeSlot` does
//! not fire timer / subscription callbacks today (M.5.a.4 follow-up)
//! so a typed publisher / timer here would add link-time weight
//! without execution coverage; the node-only declaration exercises
//! the M.5.a.3 link + dispatch contract end-to-end.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct Talker;

impl Node for Talker {
    const NAME: &'static str = "talker";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("talker"))?;
        Ok(())
    }
}

// Phase 212.M.5.a.4 — the macro now emits `_init` / `_dispatch` /
// `_tick` symbols that call into `ExecutableNode`. Declarative
// pkgs satisfy that contract with the no-op blanket impl.
nros::declarative_component!(Talker);
nros::node!(Talker);
