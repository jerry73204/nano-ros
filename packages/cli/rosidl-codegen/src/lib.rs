// Capacity/storage config relocated to the Lower stage (RFC-0068 / phase-335
// W1.b); re-exported so `rosidl_codegen::config::…` paths are unchanged.
pub use rosidl_lower::config;
/// phase-403 W6 — the derived per-type size bound leaves codegen as build
/// metadata, instead of stopping at a `#define` in a generated header.
pub mod bounds;
pub mod codegen_version;
// RFC-0091 §6b / phase-432 W2.5b — a language's Rust surface area: its filters.
pub mod filters;
// RFC-0061 / phase-318 W1 — the tool answers "would I emit different bytes?"
pub mod fingerprint;
pub mod generator;
pub mod idl_generator;
/// phase-304 W1 (RFC-0056) — REP-2011 RIHS01 type-hash engine. Relocated to the
/// `rosidl-resolve` crate (RFC-0068 Stage 1 / phase-335 W1.a); re-exported here
/// so `rosidl_codegen::rihs::…` paths (incl. rosidl-bindgen's) are unchanged.
pub use rosidl_resolve::rihs;

// RFC-0068 Stage 1 IR, re-exported so consumers (rosidl-bindgen) reach it
// through the crate they already depend on (phase-335 W1.c).
pub use rosidl_resolve::{ResolvedAction, ResolvedMessage, ResolvedService};
pub mod render;
pub mod schema_value;
pub mod templates;
pub mod types;
pub mod utils;

pub use bounds::{
    BoundInventory, BoundState, INVENTORY_CMAKE_NAME, INVENTORY_JSON_NAME,
    INVENTORY_SCHEMA_VERSION, TypeBoundEntry,
};
pub use config::{
    CODEGEN_CONFIG_FILENAME, CapacityResolver, ConfigError, FieldKind, FieldStorage, StorageMode,
};
pub use fingerprint::codegen_fingerprint;

/// The CODEGEN VERSION this binary EMITS (`E`) — phase-429 W2.
///
/// Read straight through from the `nros-core` this emitter was compiled
/// against, so "what does this `nros` emit" has exactly one answer and it is
/// baked at build time. Sibling of [`codegen_fingerprint`]: the fingerprint
/// says whether two binaries emit the same BYTES, this says whether emitted
/// bytes and a runtime can be PAIRED.
///
/// Deliberately not a release version — see `nros_core::codegen_version`.
pub const EMITTED_CODEGEN_VERSION: u32 = nros_core::codegen_version::NROS_CODEGEN_VERSION;
pub use generator::{
    GeneratedActionPackage, GeneratedCActionPackage, GeneratedCPackage, GeneratedCServicePackage,
    GeneratedCppActionPackage, GeneratedCppPackage, GeneratedCppServicePackage, GeneratedFfiRs,
    GeneratedNrosActionPackage, GeneratedNrosPackage, GeneratedNrosServicePackage,
    GeneratedPackage, GeneratedServicePackage, GeneratorError, generate_action_package,
    generate_c_action_package, generate_c_message_package, generate_c_message_package_with_lookup,
    generate_c_service_package, generate_cpp_action_package, generate_cpp_message_package,
    generate_cpp_message_package_with_lookup, generate_cpp_service_package,
    generate_message_package, generate_nros_action_package, generate_nros_inline_action,
    generate_nros_inline_message, generate_nros_inline_service, generate_nros_message_package,
    generate_nros_service_package, generate_service_package,
};
pub use idl_generator::{GeneratedIdlCode, extract_annotations, generate_idl_file};
pub use types::{
    C_DEFAULT_SEQUENCE_CAPACITY, C_DEFAULT_STRING_CAPACITY, CPP_DEFAULT_SEQUENCE_CAPACITY,
    CPP_DEFAULT_STRING_CAPACITY, CodegenBackend, DEFAULT_ROS_EDITION, FieldTypeExt, IdlTypeExt,
    NROS_DEFAULT_SEQUENCE_CAPACITY, NROS_DEFAULT_STRING_CAPACITY, NrosCodegenMode, RosEdition,
    c_array_suffix_for_field, c_type_for_constant, c_type_for_field, compute_serialized_size_max,
    cpp_type_for_field, escape_keyword, idl_constant_value_to_rust, nros_type_for_constant,
    nros_type_for_field, nros_type_for_field_with_mode, repr_c_type_for_field, rust_type_for_field,
    rust_type_for_idl, rust_type_for_idl_constant, to_c_package_name,
};

#[cfg(test)]
mod tests {
    use super::*;
    use rosidl_parser::{FieldType, PrimitiveType, parse_message};

    #[test]
    fn test_basic_type_mapping() {
        let field_type = FieldType::Primitive(PrimitiveType::Int32);
        let rust_type = rust_type_for_field(&field_type, false, None);
        assert_eq!(rust_type, "i32");
    }

    #[test]
    fn test_keyword_escaping() {
        assert_eq!(escape_keyword("type"), "type_");
        assert_eq!(escape_keyword("match"), "match_");
        assert_eq!(escape_keyword("normal"), "normal");
    }

    #[test]
    fn test_simple_message_generation() {
        let msg = parse_message("int32 x\nfloat64 y\n").unwrap();
        let result = generate_message_package(
            "test_msgs",
            "TestMessage",
            &msg,
            &std::collections::HashSet::new(),
        );
        assert!(result.is_ok());
    }
}
