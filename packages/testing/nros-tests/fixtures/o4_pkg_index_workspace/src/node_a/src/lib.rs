//! Phase 212.O.4 fixture — Node pkg `node_a`.
//!
//! `nros::node!()` emits the `pub fn register(runtime)` symbol the
//! `nros::main!()` macro's emitted body dispatches to (one call per
//! `<node pkg="..."/>` entry the workspace pkg-index resolved through
//! demo_bringup/launch/system.launch.xml).

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct NodeA;

impl Node for NodeA {
    const NAME: &'static str = "node_a";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("node_a"))?;
        Ok(())
    }
}

nros::declarative_component!(NodeA);
nros::node!(NodeA);
