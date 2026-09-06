use super::common::{
    GeneratorError, SchemaCaps, build_c_fields, build_idiomatic_fields, build_nros_fields,
    build_nros_message_schema, build_rmw_fields,
};
use crate::{
    config::CapacityResolver,
    templates::{
        BuildRsTemplate, CConstant, CargoNrosTomlTemplate, CargoTomlTemplate, IdiomaticField,
        LibNrosRsTemplate, LibRsTemplate, MessageCHeaderTemplate, MessageCSourceTemplate,
        MessageConstant, MessageIdiomaticTemplate, MessageNrosTemplate, MessageRmwTemplate,
        RmwField,
    },
    types::{
        NrosCodegenMode, c_type_for_constant, constant_value_to_rust, nros_type_for_constant,
        rust_type_for_constant, to_c_package_name,
    },
    utils::{extract_dependencies, needs_big_array, to_snake_case},
};
use rosidl_parser::{FieldType, Message};
use std::collections::HashSet;

pub struct GeneratedPackage {
    pub cargo_toml: String,
    pub build_rs: String,
    pub lib_rs: String,
    pub message_rmw: String,
    pub message_idiomatic: String,
}

/// Generate a complete ROS 2 message package with both RMW and idiomatic layers
pub fn generate_message_package(
    package_name: &str,
    message_name: &str,
    message: &Message,
    all_dependencies: &HashSet<String>,
) -> Result<GeneratedPackage, GeneratorError> {
    // Extract dependencies from this specific message
    let msg_deps = extract_dependencies(message);

    // Combine with externally provided dependencies
    let mut all_deps: Vec<String> = all_dependencies.iter().cloned().collect();
    all_deps.extend(msg_deps);
    all_deps.sort();
    all_deps.dedup();

    // Check if we need serde's big-array feature
    let needs_big_array_feature = needs_big_array(message);

    // Generate Cargo.toml
    let cargo_toml_template = CargoTomlTemplate {
        package_name,
        dependencies: &all_deps,
        needs_big_array: needs_big_array_feature,
    };
    let cargo_toml = crate::render::render("cargo.toml", &cargo_toml_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate build.rs
    let build_rs_template = BuildRsTemplate;
    let build_rs = crate::render::render("build.rs", &build_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate lib.rs
    let lib_rs_template = LibRsTemplate {
        has_messages: true,
        has_services: false,
        has_actions: false,
    };
    let lib_rs = crate::render::render("lib.rs", &lib_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate RMW layer message — projected from the lowered IR (phase-432
    // W2.5a); this used to map over `message.fields` directly.
    let rmw_fields: Vec<RmwField> = build_rmw_fields(package_name, message_name, message);

    let rmw_constants: Vec<MessageConstant> = message
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: rust_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let message_module = &to_snake_case(message_name);

    let message_rmw_template = MessageRmwTemplate {
        package_name,
        message_name,
        message_module,
        fields: rmw_fields,
        constants: rmw_constants,
    };
    // RFC-0068 Stage 3 (phase-335 W3): rmw Rust message from the minijinja pack.
    let message_rmw = crate::render::render("message_rmw.rs", &message_rmw_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate idiomatic layer message — projected from the lowered IR
    // (phase-432 W2.5a); this used to map over `message.fields` directly.
    let idiomatic_fields: Vec<IdiomaticField> =
        build_idiomatic_fields(package_name, message_name, message);

    let idiomatic_constants: Vec<MessageConstant> = message
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: rust_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let message_idiomatic_template = MessageIdiomaticTemplate {
        package_name,
        message_name,
        message_module,
        fields: idiomatic_fields,
        constants: idiomatic_constants,
    };
    let message_idiomatic =
        crate::render::render("message_idiomatic.rs", &message_idiomatic_template)
            .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedPackage {
        cargo_toml,
        build_rs,
        lib_rs,
        message_rmw,
        message_idiomatic,
    })
}

/// Generated nros message package
pub struct GeneratedNrosPackage {
    pub cargo_toml: String,
    pub lib_rs: String,
    pub message_rs: String,
}

/// Generate a nros message package
pub fn generate_nros_message_package(
    package_name: &str,
    message_name: &str,
    message: &Message,
    all_dependencies: &HashSet<String>,
    package_version: &str,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedNrosPackage, GeneratorError> {
    // Extract dependencies from this specific message
    let msg_deps = extract_dependencies(message);

    // Combine with externally provided dependencies
    let mut all_deps: Vec<String> = all_dependencies.iter().cloned().collect();
    all_deps.extend(msg_deps);
    all_deps.sort();
    all_deps.dedup();

    // Generate Cargo.toml
    let cargo_toml_template = CargoNrosTomlTemplate {
        package_name,
        package_version,
        dependencies: &all_deps,
        has_actions: false,
    };
    let cargo_toml = crate::render::render("cargo_nros.toml", &cargo_toml_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate lib.rs
    let lib_rs_template = LibNrosRsTemplate {
        has_messages: true,
        has_services: false,
        has_actions: false,
    };
    let lib_rs = crate::render::render("lib_nros.rs", &lib_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // phase-335 W1.c — storage from the lowered IR (byte-identical), resolved once.
    let fields = build_nros_fields(
        package_name,
        message_name,
        message,
        resolver,
        NrosCodegenMode::Crate,
    )?;

    // Generate constants
    let constants: Vec<MessageConstant> = message
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let stamp_offset = stamp_offset_for(message);

    let has_fields = !fields.is_empty();
    let has_large_array = fields.iter().any(|f| f.is_large_array);
    let has_borrowed = fields.iter().any(|f| f.is_borrowed);
    let schema = build_nros_message_schema(
        package_name,
        message_name,
        &message.fields,
        &SchemaCaps::new(message_name, resolver),
    );
    let message_template = MessageNrosTemplate {
        package_name,
        message_name,
        type_hash,
        stamp_offset,
        fields,
        constants,
        has_fields,
        has_large_array,
        has_borrowed,
        inline_mode: false,
        schema_helper_consts: schema.helper_consts,
        schema_fields_block: schema.fields_block,
        schema_type_name: schema.nros_type_name,
    };
    let message_rs = crate::render::render("message_nros.rs", &message_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedNrosPackage {
        cargo_toml,
        lib_rs,
        message_rs,
    })
}

/// Generate a single message's Rust code in inline mode.
///
/// Unlike `generate_nros_message_package`, this only returns the rendered
/// message code (no Cargo.toml or lib.rs). Cross-package references use
/// `super::super::super::pkg::msg::Type` paths.
pub fn generate_nros_inline_message(
    package_name: &str,
    message_name: &str,
    message: &Message,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<String, GeneratorError> {
    let fields = build_nros_fields(
        package_name,
        message_name,
        message,
        resolver,
        NrosCodegenMode::Inline,
    )?;

    let constants: Vec<MessageConstant> = message
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let has_fields = !fields.is_empty();
    let has_large_array = fields.iter().any(|f| f.is_large_array);
    let has_borrowed = fields.iter().any(|f| f.is_borrowed);
    let schema = build_nros_message_schema(
        package_name,
        message_name,
        &message.fields,
        &SchemaCaps::new(message_name, resolver),
    );

    let template = MessageNrosTemplate {
        package_name,
        message_name,
        stamp_offset: stamp_offset_for(message),
        type_hash,
        fields,
        constants,
        has_fields,
        has_large_array,
        has_borrowed,
        inline_mode: true,
        schema_helper_consts: schema.helper_consts,
        schema_fields_block: schema.fields_block,
        schema_type_name: schema.nros_type_name,
    };

    crate::render::render("message_nros.rs", &template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))
}

/// Generated C message package
pub struct GeneratedCPackage {
    /// Header file content (.h)
    pub header: String,
    /// Source file content (.c)
    pub source: String,
    /// Header filename
    pub header_name: String,
    /// Source filename
    pub source_name: String,
}

/// Generate C code for a message type
/// See [`generate_c_message_package_with_lookup`]; this variant resolves no
/// nested types.
///
/// A message with a nested field therefore reports `Unresolved` and emits NO
/// size constant, which is the honest answer for a caller that cannot supply a
/// resolver — never a guessed bound. Callers that HAVE one (the bindgen path
/// composes same-package + ament-index resolution into `self_resolve`) should
/// use the `_with_lookup` form.
pub fn generate_c_message_package(
    package_name: &str,
    message_name: &str,
    message: &Message,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedCPackage, GeneratorError> {
    generate_c_message_package_with_lookup(
        package_name,
        message_name,
        message,
        type_hash,
        resolver,
        &|_| None,
    )
}

/// Emit the C header + source for one message, resolving nested types through
/// `lookup` so the header can carry the type's serialized-size bound
/// (issue 0896 layer 2).
pub fn generate_c_message_package_with_lookup(
    package_name: &str,
    message_name: &str,
    message: &Message,
    type_hash: &str,
    resolver: &CapacityResolver,
    lookup: &crate::schema_value::MsgLookup<'_>,
) -> Result<GeneratedCPackage, GeneratorError> {
    let c_pkg_name = to_c_package_name(package_name);
    let msg_snake = to_snake_case(message_name);

    // Build struct and guard names
    let struct_name = format!("{}_msg_{}", c_pkg_name, msg_snake);
    let guard_name = format!(
        "{}_MSG_{}_H",
        c_pkg_name.to_uppercase(),
        msg_snake.to_uppercase()
    );
    let constant_prefix = format!(
        "{}_MSG_{}",
        c_pkg_name.to_uppercase(),
        msg_snake.to_uppercase()
    );
    let header_name = format!("{}_msg_{}.h", c_pkg_name, msg_snake);
    let source_name = format!("{}_msg_{}.c", c_pkg_name, msg_snake);

    // Extract dependencies — both cross-package (umbrella includes) and
    // intra-package (per-type includes for types in the same package).
    let mut dependencies = Vec::new();
    let mut type_includes = Vec::new();
    for field in &message.fields {
        let field_type = match &field.field_type {
            FieldType::NamespacedType { .. } => Some(&field.field_type),
            FieldType::Array { element_type, .. }
            | FieldType::Sequence { element_type }
            | FieldType::BoundedSequence { element_type, .. } => {
                if matches!(element_type.as_ref(), FieldType::NamespacedType { .. }) {
                    Some(element_type.as_ref())
                } else {
                    None
                }
            }
            _ => None,
        };
        if let Some(FieldType::NamespacedType { package, name }) = field_type {
            let pkg = package.as_deref().unwrap_or(package_name);
            let dep = to_c_package_name(pkg);
            let header_filename =
                format!("{}_msg_{}.h", to_c_package_name(pkg), to_snake_case(name));
            let type_header = if dep != c_pkg_name {
                // Cross-package: include with subdirectory path
                if !dependencies.contains(&dep) {
                    dependencies.push(dep.clone());
                }
                format!("{}/msg/{}", dep, header_filename)
            } else {
                // Intra-package: include from same msg/ directory
                header_filename
            };
            if !type_includes.contains(&type_header) {
                type_includes.push(type_header);
            }
        }
    }
    dependencies.sort();
    type_includes.sort();

    // Build C fields
    let fields = build_c_fields(Some(package_name), message_name, message, resolver)?;

    // Build C constants
    let constants: Vec<CConstant> = message
        .constants
        .iter()
        .map(|constant| CConstant {
            name: constant.name.clone(),
            c_type: c_type_for_constant(&constant.constant_type),
            value: constant_value_to_rust(&constant.value),
        })
        .collect();

    let has_fields = !fields.is_empty();
    let has_borrowed = fields.iter().any(|f| f.is_borrowed);

    // issue 0896 layer 2 — the type's own size bound, computed with THE size
    // rule (`nros_serdes::size::max_serialized_size`) over a schema built by
    // `schema_value`, never a second walk that adds up field widths here.
    //
    // phase-408 W1 — the derivation itself lives in
    // `generator::common::derive_message_bound`, shared with the C++ pack. It
    // used to be inline here, and copying it into the C++ emitter would have
    // been a second implementation of "how big can this type get" along the
    // LANGUAGE axis, which is the class this campaign keeps finding.
    let bound = crate::generator::common::derive_message_bound(
        package_name,
        message_name,
        message,
        &struct_name,
        resolver,
        lookup,
    )?;

    // Generate header
    let header_template = MessageCHeaderTemplate {
        package_name,
        message_name,
        type_hash,
        guard_name,
        struct_name: struct_name.clone(),
        constant_prefix,
        fields: fields.clone(),
        constants,
        dependencies,
        type_includes,
        has_fields,
        has_borrowed,
        tx_max_serialized_size: bound.tx,
        rx_max_serialized_size: bound.rx,
        unbounded_reason: bound.reason,
        unbounded_token: bound.token,
    };
    // RFC-0068 Stage 3 (phase-335 W2): C message emission renders from the
    // minijinja data pack (packs/c/) instead of the compile-time askama path.
    let header = crate::render::render_c("message.h", &header_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate source
    let source_template = MessageCSourceTemplate {
        package_name,
        message_name,
        type_hash,
        header_name: header_name.clone(),
        struct_name,
        fields,
        has_fields,
        has_borrowed,
    };
    let source = crate::render::render_c("message.c", &source_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedCPackage {
        header,
        source,
        header_name,
        source_name,
    })
}

/// RFC-0052 W3a — static stamp-offset predicate: a message whose FIRST
/// field is `std_msgs/Header` (or a bare `builtin_interfaces/Time`, which
/// includes `Header` itself) carries `stamp.sec` at CDR byte 4 (after the
/// 4-byte encapsulation header; `Time` is `{ i32 sec; u32 nanosec }`,
/// 4-byte aligned, no preceding fields). Everything else: `None`.
fn stamp_offset_for(message: &rosidl_parser::ast::Message) -> Option<usize> {
    use rosidl_parser::ast::FieldType;
    let first = message.fields.first()?;
    match &first.field_type {
        FieldType::NamespacedType { package, name } => {
            let pkg = package.as_deref();
            let is_header = name == "Header" && matches!(pkg, Some("std_msgs") | None);
            let is_time = name == "Time" && matches!(pkg, Some("builtin_interfaces") | None);
            if is_header || is_time { Some(4) } else { None }
        }
        _ => None,
    }
}

#[cfg(test)]
mod stamp_offset_tests {
    use super::*;
    use rosidl_parser::ast::{Field, FieldType, Message};

    fn msg(fields: Vec<Field>) -> Message {
        Message {
            fields,
            constants: vec![],
        }
    }

    fn field(name: &str, field_type: FieldType) -> Field {
        Field {
            name: name.to_string(),
            field_type,
            default_value: None,
        }
    }

    #[test]
    fn header_leading_gets_offset_4() {
        let m = msg(vec![
            field(
                "header",
                FieldType::NamespacedType {
                    package: Some("std_msgs".to_string()),
                    name: "Header".to_string(),
                },
            ),
            field(
                "x",
                FieldType::Primitive(rosidl_parser::ast::PrimitiveType::Float64),
            ),
        ]);
        assert_eq!(stamp_offset_for(&m), Some(4));
    }

    #[test]
    fn time_leading_gets_offset_4_and_others_none() {
        let t = msg(vec![field(
            "stamp",
            FieldType::NamespacedType {
                package: Some("builtin_interfaces".to_string()),
                name: "Time".to_string(),
            },
        )]);
        assert_eq!(stamp_offset_for(&t), Some(4));
        let plain = msg(vec![field("data", FieldType::String)]);
        assert_eq!(stamp_offset_for(&plain), None);
        // Header NOT first — no offset (peek would be wrong).
        let trailing = msg(vec![
            field(
                "x",
                FieldType::Primitive(rosidl_parser::ast::PrimitiveType::Int32),
            ),
            field(
                "header",
                FieldType::NamespacedType {
                    package: Some("std_msgs".to_string()),
                    name: "Header".to_string(),
                },
            ),
        ]);
        assert_eq!(stamp_offset_for(&trailing), None);
    }
}
