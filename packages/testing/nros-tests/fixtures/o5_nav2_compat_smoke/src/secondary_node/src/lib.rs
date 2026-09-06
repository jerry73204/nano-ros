//! Phase 212.O.5 fixture — secondary Component pkg (Rust, no_std).
//!
//! `nros::node!()` emits the `pub fn register(runtime)` symbol the
//! codegen-emitted `run_plan(runtime)` body invokes when the primary
//! launch's `<include>` resolves this pkg's sibling launch.xml.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct Secondary;

impl Node for Secondary {
    const NAME: &'static str = "secondary";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("secondary"))?;
        Ok(())
    }
}

nros::declarative_component!(Secondary);
nros::node!(Secondary);
