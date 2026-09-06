//! Phase 212.O.5 fixture — primary Component pkg (Rust, no_std).
//!
//! Implements `nros::Node` for the `Primary` struct and stamps it with
//! `nros::node!()`. The latter emits the `pub fn register(runtime)`
//! symbol the codegen-emitted `run_plan(runtime)` body calls.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct Primary;

impl Node for Primary {
    const NAME: &'static str = "primary";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("primary"))?;
        Ok(())
    }
}

nros::declarative_component!(Primary);
nros::node!(Primary);
