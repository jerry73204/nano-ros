//! Stage 2 of the codegen pipeline (RFC-0068): **Lower**.
//!
//! Takes the neutral [`rosidl_resolve::ResolvedMessage`] plus a `CodegenConfig`
//! (capacity/storage policy — the [`config`] module)
//! (pointer width, enum width, alignment rules) and produces a TARGET-concrete
//! but still LANGUAGE-neutral IR: per-field storage, `repr(C)` layout,
//! plainness, alignment and serialized size. These are the facts a per-language
//! renderer must not recompute.
//!
//! The [`config`] module is the capacity/storage resolver, relocated here from
//! `rosidl-codegen` (phase-335 W1.b) because capacity resolution IS a Lower
//! input. `rosidl-codegen` re-exports it, so existing `rosidl_codegen::config`
//! and `crate::CapacityResolver` paths are unchanged.

pub mod config;
mod lowered;

pub use lowered::{
    CdrOp, FieldShape, LoweredField, LoweredStorage, LoweredType, lower, lower_fields,
};
