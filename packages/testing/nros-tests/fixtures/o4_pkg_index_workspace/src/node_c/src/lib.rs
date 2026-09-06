//! Phase 212.O.4 fixture — Node pkg `node_c`. See sibling `node_a`.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct NodeC;

impl Node for NodeC {
    const NAME: &'static str = "node_c";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("node_c"))?;
        Ok(())
    }
}

nros::declarative_component!(NodeC);
nros::node!(NodeC);
