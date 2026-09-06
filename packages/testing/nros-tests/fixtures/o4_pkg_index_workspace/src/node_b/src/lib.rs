//! Phase 212.O.4 fixture — Node pkg `node_b`. See sibling `node_a`.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct NodeB;

impl Node for NodeB {
    const NAME: &'static str = "node_b";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("node_b"))?;
        Ok(())
    }
}

nros::declarative_component!(NodeB);
nros::node!(NodeB);
