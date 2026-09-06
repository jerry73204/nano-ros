//! Phase 212.N.9 fixture — minimal Node pkg.
//!
//! Implements `nros::Node` for the `Talker` struct and stamps
//! it with `nros::node!()`. The latter emits the
//! `pub fn register(runtime)` symbol the `nros::main!()` macro's
//! emitted body calls.

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

nros::declarative_component!(Talker);
nros::node!(Talker);
