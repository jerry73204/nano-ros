//! Generated nros bindings
//!
//! This crate is `no_std` compatible.

#![no_std]
#![allow(dead_code)]
#![allow(clippy::all)]

/// The nros codegen version this crate was emitted at (phase-429 W1).
///
/// The runtime declares the versions it accepts in `nros_core::codegen_version`;
/// the assertion below is what turns a disagreement into a build failure rather
/// than a wrong field offset several frames down at run time.
pub const NROS_EMITTED_CODEGEN_VERSION: u32 = 2;

// Crate scope, deliberately: rustc evaluates a crate-scope `const` item for a
// crate that is merely COMPILED, so `cargo check` reports this. An inline
// `const {}` block inside a generic (RFC-0088's `format_check`) is
// monomorphisation-timed, and `cargo check` never sees it.
const _: () = assert!(
    nros_core::codegen_version::accepts(NROS_EMITTED_CODEGEN_VERSION),
    "this crate's NROS_EMITTED_CODEGEN_VERSION (declared just above) is not \
     accepted by the nros-core it is being compiled against -- compare it with \
     nros_core::codegen_version. Regenerate the generated/ tree with this \
     checkout's codegen: `nros sync` in a consumer workspace, or \
     `just generate-bindings` in-tree."
);

pub mod msg;
