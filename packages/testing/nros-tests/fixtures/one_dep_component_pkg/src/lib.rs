//! Phase 212.M-F.13 path (b) fixture — minimal Node pkg.
//!
//! Declares one node with `nros::Node` and stamps it with the
//! `nros::node!()` macro. The macro expansion references
//! `RuntimeCtx` / `RuntimeError` / the four `Node*Fn` aliases —
//! all of which must resolve through the
//! `nros::__macro_support::nros_platform` re-export, NOT through a
//! direct `nros_platform` crate dep (this pkg's `Cargo.toml` only
//! lists `nros` under `[dependencies]`).
//!
//! `declarative_component!` supplies the blanket
//! `ExecutableNode` impl the `register(runtime)` wrapper needs
//! when no real callback / timer body is in play. Together the two
//! macros exercise the same emit surface that production component
//! pkgs use, so a regression in the re-export path here flags before
//! it hits the freertos / threadx / nuttx Node fixtures.

#![no_std]

use nros::{Node, NodeContext, NodeOptions, NodeResult};

pub struct OneDep;

impl Node for OneDep {
    const NAME: &'static str = "one_dep";

    // issue 0857 — the cell registries this class fills, exactly: (publishers,
    // service servers, service clients, action clients, action servers). Undeclared
    // means `NROS_RUNTIME_MAX_CELL_ENTITIES` per kind, in `.bss`, twice over.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {
        let _node = ctx.create_node(NodeOptions::new("one_dep"))?;
        Ok(())
    }
}

nros::declarative_component!(OneDep);
nros::node!(OneDep);
