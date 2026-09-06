//! Phase 212.O.3 fixture — board-agnostic shared Component.
//!
//! The whole point: this source compiles UNMODIFIED under both
//! the host (`posix_entry`, `deploy = "native"`) and FreeRTOS (`freertos_entry`,
//! `thumbv7m-none-eabi`). Linking the same rlib against two distinct
//! Board impls proves the Phase 212.N.4 codegen emit
//! (`OUT_DIR/run_plan.rs`) is board-agnostic.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct SharedNode;

impl Node for SharedNode {
    const NAME: &'static str = "shared_node";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("shared_node"))?;
        Ok(())
    }
}

// Phase 212.M.5.a.4 — declarative_component + node macros emit the
// register / lifecycle symbols the codegen `run_plan(runtime)` body
// invokes. Same macro expansion under both platform features —
// neither macro reads platform cfg.
nros::declarative_component!(SharedNode);
nros::node!(SharedNode);
