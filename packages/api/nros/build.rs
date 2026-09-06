//! Build script for `nros` — phase-391 W5.
//!
//! Reads `NROS_RUNTIME_*` environment variables (or their `CONFIG_*` Kconfig
//! spellings) and generates `nros_runtime_config.rs`, included by
//! `src/config.rs`.
//!
//! # Why a knob and not a const generic
//!
//! `node_runtime` carries nine `extern "C"` sites and backs
//! `__nros_component_<pkg>_install`, the uniform cross-language
//! component-install seam. A const generic would put a type parameter on a type
//! that C and C++ consume; a baked `pub const` is invisible at the ABI. Its
//! twin `node_metadata` has ZERO `extern "C"` sites and uses const generics
//! freely — the tree already draws this line, W5 just states it.
//!
//! # Why `knob_usize` and not `env::var`
//!
//! Issue 0460: a Zephyr RUST image inherits none of the cmake `set(ENV{...})`
//! knob exports, so an `env::var` read compiles the crate default whatever
//! Kconfig said. `knob_usize` falls back to `$DOTCONFIG`.
use std::{env, path::Path};

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    // phase-427 W9 — `Context::baked()` (`src/init.rs`) reads
    // `option_env!("NROS_LOCATOR")` / `option_env!("NROS_DOMAIN_ID")` in THIS
    // crate, so this crate's build has to carry them. A leaf's `cargo:rustc-env`
    // never reaches a dependency's compilation (the reason the leaf bakes for
    // itself), and on Zephyr the values come from Kconfig, not the process
    // environment; the same helper the leaves call re-exports
    // `CONFIG_NROS_ZENOH_LOCATOR` / `CONFIG_NROS_DOMAIN_ID` from `$DOTCONFIG`
    // — the issue-0460 channel the knobs below already rely on. Off Zephyr it
    // is a no-op beyond the `rerun-if-env-changed` lines, which is what makes
    // a process-environment bake (the FreeRTOS / ThreadX fixture lanes export
    // `NROS_DOMAIN_ID`) re-compile this crate rather than reuse a stale object.
    nros_zephyr_build::bake_nros_config();

    // phase-400 W6 — the `[knobs.runtime]` platform/board rungs, resolved once.
    // `None` when no lane named a platform, and the builtins below then stand.
    //
    // phase-391 emits the consts from these numbers and sizes the arena from
    // them; it does not decide their VALUES. This is where a platform or board
    // gets to, without an env var on the shell that happened to run the build.
    let rungs = nros_platform_config::platform_config::BuildRungs::from_build_env()
        .map(|r| r.runtime_rungs())
        .unwrap_or_default();

    // Component pool slots. `register_node::<C>()` is a runtime call so the
    // COUNT is not known at compile time, but the BOUND is — which is all a
    // static pool needs. Mirrors `NROS_EXECUTOR_MAX_NODES` (default 4) one
    // layer down; 4 leaves room for the multi-component entries codegen emits
    // without charging a single-node image for slots it never fills.
    let max_components = env_usize("NROS_RUNTIME_MAX_COMPONENTS", rungs.max_components, 4);

    // Per-slot byte budget for the type-erased `TypedSlot<C>`. A BYTE budget
    // rather than a type, because the pool is heterogeneous (`TypedSlot<C>` is
    // generic over `C`) and the FFI seam cannot name a generic. A component
    // whose slot does not fit is a registration error, not a compile error.
    let component_slot_bytes = env_usize(
        "NROS_RUNTIME_COMPONENT_SLOT_BYTES",
        rungs.component_slot_bytes,
        512,
    );

    // phase-391 W5.3b — instances of ONE component class the macro-emitted
    // per-class store can hold. Multi-instance is real: the launch path bakes
    // one identity per plan node and can name the same class twice. 2 covers
    // the pair case without charging every class for more; install past the
    // cap is a Full-style registration error, not silent reuse.
    let max_class_instances = env_usize(
        "NROS_RUNTIME_MAX_CLASS_INSTANCES",
        rungs.max_class_instances,
        2,
    );

    // phase-391 W5 regression fix — entity-registry slots per COMPONENT CELL.
    // The first heapless conversion borrowed the metadata twin's 32, which is
    // per-PLAN-shaped: it made every cell ~20 KB up front (4 registries x 32 x
    // ~152 B) — 4 components ate ~80 KiB of the 128 KiB bare-metal arena.
    // 8 is per-component-shaped (a component declaring more than 8 entities of
    // ONE KIND is rare and gets a loud registration error + this knob).
    let max_cell_entities = env_usize("NROS_RUNTIME_MAX_CELL_ENTITIES", rungs.max_cell_entities, 8);

    let contents = format!(
        "/// Component pool slots (set via `NROS_RUNTIME_MAX_COMPONENTS`, default 4).\n\
         ///\n\
         /// phase-391 W5. The bound the static component pool is sized to; a\n\
         /// `register_node` past it fails rather than growing a `Vec`.\n\
         pub const MAX_COMPONENTS: usize = {max_components};\n\
         \n\
         /// Per-slot storage for a type-erased `TypedSlot<C>`, in bytes\n\
         /// (set via `NROS_RUNTIME_COMPONENT_SLOT_BYTES`, default 512).\n\
         pub const COMPONENT_SLOT_BYTES: usize = {component_slot_bytes};\n\
         \n\
         /// Instances of one component class the per-class store holds\n\
         /// (set via `NROS_RUNTIME_MAX_CLASS_INSTANCES`, default 2).\n\
         pub const MAX_CLASS_INSTANCES: usize = {max_class_instances};\n\
         \n\
         /// Entity-registry slots per component cell, PER KIND\n\
         /// (set via `NROS_RUNTIME_MAX_CELL_ENTITIES`, default 8).\n\
         pub const MAX_CELL_ENTITIES: usize = {max_cell_entities};\n"
    );
    std::fs::write(Path::new(&out_dir).join("nros_runtime_config.rs"), contents).unwrap();
}

/// phase-400 W6 — `rung` is the platform/board answer from `[knobs.runtime]`,
/// and it sits between the Kconfig rung and the crate builtin. Passing it as
/// `knob_usize`'s default is what places it there: env and `$DOTCONFIG` still
/// win above it, and the builtin applies only where no descriptor spoke.
fn env_usize(name: &str, rung: Option<usize>, default: usize) -> usize {
    nros_zephyr_build::knob_usize(name, &format!("CONFIG_{name}"), rung.unwrap_or(default))
}
