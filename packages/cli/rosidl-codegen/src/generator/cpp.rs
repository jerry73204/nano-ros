use super::common::{GeneratorError, build_cpp_ffi_field, build_cpp_field, resolve_cap_override};
use crate::{
    config::CapacityResolver,
    templates::{
        ActionCppHeaderTemplate, CConstant, CppFfiField, CppField, MessageCppExportsTemplate,
        MessageCppHeaderTemplate, MessageCppTypesTemplate, SequenceStructDef,
        ServiceCppHeaderTemplate,
    },
    types::{
        c_type_for_constant, compute_serialized_size_max, constant_value_to_rust, to_c_package_name,
    },
    utils::to_snake_case,
};
use rosidl_parser::{Action, FieldType, Message, Service};

/// One message-like part's split Rust FFI glue (phase-306 W1, issue 0253):
/// the TYPES half (crate-mangled structs + plain field serializers — safe to
/// duplicate across per-package crates) and the EXPORTS half (only the
/// `#[unsafe(no_mangle)]` C-ABI wrappers — included solely by the owning
/// package's crate).
pub struct GeneratedFfiRs {
    /// TYPES half content (`<stem>_types.rs`)
    pub types_rs: String,
    /// EXPORTS half content (`<stem>_exports.rs`)
    pub exports_rs: String,
    /// TYPES half filename
    pub types_rs_name: String,
    /// EXPORTS half filename
    pub exports_rs_name: String,
}

impl GeneratedFfiRs {
    /// The pre-split single-file view (types + exports concatenated) — for
    /// tests / one-off harnesses that compile the glue as one module.
    pub fn combined(&self) -> String {
        format!("{}\n{}", self.types_rs, self.exports_rs)
    }
}

/// Generated C++ message package (header + split FFI Rust glue)
pub struct GeneratedCppPackage {
    /// C++ header content (.hpp)
    pub header: String,
    /// Split Rust FFI glue
    pub ffi: GeneratedFfiRs,
    /// Header filename
    pub header_name: String,
}

/// Generated C++ service package (header + per-part split FFI glue)
pub struct GeneratedCppServicePackage {
    /// C++ header content (.hpp)
    pub header: String,
    /// Header filename
    pub header_name: String,
    /// Split Rust FFI glue for request
    pub request_ffi: GeneratedFfiRs,
    /// Split Rust FFI glue for response
    pub response_ffi: GeneratedFfiRs,
}

/// Generated C++ action package (header + per-part split FFI glue)
pub struct GeneratedCppActionPackage {
    /// C++ header content (.hpp)
    pub header: String,
    /// Header filename
    pub header_name: String,
    /// Split Rust FFI glue for goal
    pub goal_ffi: GeneratedFfiRs,
    /// Split Rust FFI glue for result
    pub result_ffi: GeneratedFfiRs,
    /// Split Rust FFI glue for feedback
    pub feedback_ffi: GeneratedFfiRs,
}

/// Helper: build CppField list and CppFfiField list + sequence structs from message fields.
///
/// `message_name` + `resolver` supply per-field capacity (RFC-0033); the same
/// resolved `cap` feeds both the header type and the FFI repr so the two agree.
type BuiltFields = (Vec<CppField>, Vec<CppFfiField>, Vec<SequenceStructDef>);

fn build_fields(
    fields: &[rosidl_parser::Field],
    struct_name: &str,
    current_package: Option<&str>,
    message_name: &str,
    resolver: &CapacityResolver,
) -> Result<BuiltFields, GeneratorError> {
    let mut cpp_fields = Vec::new();
    let mut ffi_fields = Vec::new();
    let mut seq_structs = Vec::new();

    // phase-432 W2.5a — the lowered IR is the context both builders read: the
    // storage decision (phase-335 W1.c had that much), and now the element-cap
    // fold, the shape predicates and the element classification too.
    let package = current_package.unwrap_or("");
    let lowered = rosidl_lower::lower_fields(package, message_name, fields, resolver);
    super::common::ensure_element_caps_apply(package, message_name, fields, resolver)?;
    for field in &lowered {
        let storage = resolve_cap_override(field, current_package, message_name)?;
        cpp_fields.push(build_cpp_field(field, current_package, storage));
        let (ffi_field, seq_struct) =
            build_cpp_ffi_field(field, struct_name, current_package, storage);
        ffi_fields.push(ffi_field);
        if let Some(ss) = seq_struct {
            seq_structs.push(ss);
        }
    }

    Ok((cpp_fields, ffi_fields, seq_structs))
}

/// Helper: build CConstant list
fn build_constants(constants: &[rosidl_parser::Constant]) -> Vec<CConstant> {
    constants
        .iter()
        .map(|c| CConstant {
            name: c.name.clone(),
            c_type: c_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect()
}

/// Helper: extract unique cross-package dependencies from fields
fn extract_deps(fields: &[rosidl_parser::Field]) -> Vec<String> {
    let mut deps = Vec::new();
    for field in fields {
        collect_field_type_cross_pkg_deps(&field.field_type, &mut deps);
    }
    deps.sort();
    deps
}

fn collect_field_type_cross_pkg_deps(ft: &FieldType, deps: &mut Vec<String>) {
    match ft {
        FieldType::NamespacedType {
            package: Some(pkg), ..
        } => {
            let dep = to_c_package_name(pkg);
            if !deps.contains(&dep) {
                deps.push(dep.clone());
            }
        }
        FieldType::Array { element_type, .. }
        | FieldType::Sequence { element_type }
        | FieldType::BoundedSequence { element_type, .. } => {
            collect_field_type_cross_pkg_deps(element_type, deps);
        }
        _ => {}
    }
}

/// Helper: extract includes for same-package (intra-package) type dependencies.
/// These must be included individually to avoid ordering issues in the umbrella header.
fn extract_intra_package_includes(
    fields: &[rosidl_parser::Field],
    package_name: &str,
) -> Vec<String> {
    let c_pkg = to_c_package_name(package_name);
    let mut includes = Vec::new();
    for field in fields {
        collect_field_type_intra_pkg_includes(
            &field.field_type,
            package_name,
            &c_pkg,
            &mut includes,
        );
    }
    includes.sort();
    includes
}

fn collect_field_type_intra_pkg_includes(
    ft: &FieldType,
    package_name: &str,
    c_pkg: &str,
    includes: &mut Vec<String>,
) {
    match ft {
        // Unqualified type (`Foo[]`) — always same-pkg.
        FieldType::NamespacedType {
            package: None,
            name,
        } => {
            let path = format!("msg/{}_msg_{}.hpp", c_pkg, to_snake_case(name));
            if !includes.contains(&path) {
                includes.push(path);
            }
        }
        // Explicitly-qualified type (`autoware_planning_msgs/Foo[]`) where
        // the package matches our own. ROS `.msg` files often spell intra-
        // package refs with the full package prefix (e.g. Path.msg uses
        // `autoware_planning_msgs/PathPoint[]`), so they need the same
        // include-injection treatment as unqualified types.
        FieldType::NamespacedType {
            package: Some(pkg),
            name,
        } if pkg == package_name => {
            let path = format!("msg/{}_msg_{}.hpp", c_pkg, to_snake_case(name));
            if !includes.contains(&path) {
                includes.push(path);
            }
        }
        FieldType::Array { element_type, .. }
        | FieldType::Sequence { element_type }
        | FieldType::BoundedSequence { element_type, .. } => {
            collect_field_type_intra_pkg_includes(element_type, package_name, c_pkg, includes);
        }
        _ => {}
    }
}

/// Inputs for [`render_ffi_rs`]: the generated-file stem, the names of the
/// generated FFI functions, plus the message's fields and sequence-struct defs.
struct FfiRenderSpec<'a> {
    package_name: &'a str,
    message_name: &'a str,
    /// Filename stem (e.g. `std_msgs_msg_int32`) — the split pair is written
    /// as `<stem>_types.rs` + `<stem>_exports.rs`.
    file_stem: &'a str,
    struct_name: &'a str,
    ffi_publish_fn: &'a str,
    ffi_serialize_fn: &'a str,
    ffi_deserialize_fn: &'a str,
    serialize_fn: &'a str,
    deserialize_fn: &'a str,
    ffi_fields: &'a [CppFfiField],
    seq_structs: &'a [SequenceStructDef],
}

/// Generate the split Rust FFI glue pair for a message-like struct
/// (phase-306 W1, issue 0253): TYPES half + EXPORTS half.
fn render_ffi_rs(spec: FfiRenderSpec<'_>) -> Result<GeneratedFfiRs, GeneratorError> {
    let has_fields = !spec.ffi_fields.is_empty();
    let serialized_size_max = compute_serialized_size_max(spec.ffi_fields);
    let has_heap = spec.ffi_fields.iter().any(|f| f.is_heap);
    let has_heap_string = spec.ffi_fields.iter().any(|f| f.is_heap && f.is_string);
    let has_borrowed = spec.ffi_fields.iter().any(|f| f.is_borrowed);
    let view_repr_struct_name = format!("{}_view", spec.struct_name);
    let deserialize_view_fn = format!("{}_borrowed", spec.deserialize_fn);

    let types_template = MessageCppTypesTemplate {
        package_name: spec.package_name,
        message_name: spec.message_name,
        repr_c_struct_name: spec.struct_name.to_string(),
        serialize_fn: spec.serialize_fn.to_string(),
        deserialize_fn: spec.deserialize_fn.to_string(),
        // issue #201 — same stem as the field-deserializer (`deserialize_…`
        // → `teardown_…`), so nested references resolve uniformly across
        // messages AND service/action request/response parts.
        teardown_fn: spec.deserialize_fn.replacen("deserialize_", "teardown_", 1),
        fields: spec.ffi_fields.to_vec(),
        sequence_structs: spec.seq_structs.to_vec(),
        has_fields,
        has_heap,
        has_heap_string,
        has_borrowed,
        view_repr_struct_name: view_repr_struct_name.clone(),
        deserialize_view_fn: deserialize_view_fn.clone(),
    };

    let exports_template = MessageCppExportsTemplate {
        package_name: spec.package_name,
        message_name: spec.message_name,
        repr_c_struct_name: spec.struct_name.to_string(),
        ffi_publish_fn: spec.ffi_publish_fn.to_string(),
        ffi_serialize_fn: spec.ffi_serialize_fn.to_string(),
        ffi_deserialize_fn: spec.ffi_deserialize_fn.to_string(),
        serialize_fn: spec.serialize_fn.to_string(),
        deserialize_fn: spec.deserialize_fn.to_string(),
        fields: spec.ffi_fields.to_vec(),
        has_fields,
        serialized_size_max,
        has_heap,
        has_borrowed,
        view_repr_struct_name,
        deserialize_view_fn,
        ffi_deserialize_view_fn: format!("{}_borrowed", spec.ffi_deserialize_fn),
    };

    Ok(GeneratedFfiRs {
        types_rs: crate::render::render("message_cpp_types.rs", &types_template)
            .map_err(|e| GeneratorError::RenderError(e.to_string()))?,
        exports_rs: crate::render::render("message_cpp_exports.rs", &exports_template)
            .map_err(|e| GeneratorError::RenderError(e.to_string()))?,
        types_rs_name: format!("{}_types.rs", spec.file_stem),
        exports_rs_name: format!("{}_exports.rs", spec.file_stem),
    })
}

/// Escape `text` for use INSIDE a C++ double-quoted string literal.
///
/// The `static_assert` text interpolates a reason built from `.msg` member
/// names and nested type names — data, not a literal an emitter author wrote —
/// so it goes through here rather than straight into the template. Same
/// reasoning as `BoundInventory::to_cmake`'s quoting, which
/// `bounds::tests::a_reason_cannot_break_out_of_the_cmake_string` pins.
fn cpp_string_literal_body(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    // The message is authored as a wrapped Rust literal, so its line
    // continuations arrive as runs of spaces. Collapse them — a diagnostic is
    // read as one line by whoever hits it.
    for ch in text
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .chars()
    {
        match ch {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            c => out.push(c),
        }
    }
    out
}

/// Generate C++ code for a message type.
///
/// See [`generate_cpp_message_package_with_lookup`]; this variant resolves no
/// nested types, so a message with a nested field reports `Unresolved` and
/// states NO size bound — the honest answer for a caller that cannot supply a
/// resolver, never a guessed one.
pub fn generate_cpp_message_package(
    package_name: &str,
    message_name: &str,
    message: &Message,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedCppPackage, GeneratorError> {
    generate_cpp_message_package_with_lookup(
        package_name,
        message_name,
        message,
        type_hash,
        resolver,
        &|_| None,
    )
}

/// Emit the C++ header + split FFI glue for one message, resolving nested types
/// through `lookup` so the header can carry the type's DERIVED serialized-size
/// bound (issue 0896 layer 2, phase-408 W1) — the sibling of
/// [`crate::generate_c_message_package_with_lookup`], and the same derivation.
pub fn generate_cpp_message_package_with_lookup(
    package_name: &str,
    message_name: &str,
    message: &Message,
    type_hash: &str,
    resolver: &CapacityResolver,
    lookup: &crate::schema_value::MsgLookup<'_>,
) -> Result<GeneratedCppPackage, GeneratorError> {
    let c_pkg_name = to_c_package_name(package_name);
    let msg_snake = to_snake_case(message_name);

    let struct_name = format!("{}_msg_{}_t", c_pkg_name, msg_snake);
    let guard_name = format!(
        "{}_MSG_{}_HPP",
        c_pkg_name.to_uppercase(),
        msg_snake.to_uppercase()
    );
    let ffi_publish_fn = format!("nros_cpp_publish_{}_msg_{}", c_pkg_name, msg_snake);
    let ffi_serialize_fn = format!("nros_cpp_serialize_{}_msg_{}", c_pkg_name, msg_snake);
    let ffi_deserialize_fn = format!("nros_cpp_deserialize_{}_msg_{}", c_pkg_name, msg_snake);
    let serialize_fn = format!("serialize_{}_msg_{}_fields", c_pkg_name, msg_snake);
    let deserialize_fn = format!("deserialize_{}_msg_{}_fields", c_pkg_name, msg_snake);

    let header_name = format!("{}_msg_{}.hpp", c_pkg_name, msg_snake);
    let file_stem = format!("{}_msg_{}", c_pkg_name, msg_snake);

    let (cpp_fields, ffi_fields, seq_structs) = build_fields(
        &message.fields,
        &struct_name,
        Some(package_name),
        message_name,
        resolver,
    )?;
    let constants = build_constants(&message.constants);
    let dependencies = extract_deps(&message.fields);
    let intra_package_includes = extract_intra_package_includes(&message.fields, package_name);
    let has_fields = !cpp_fields.is_empty();
    let has_borrowed = cpp_fields.iter().any(|f| f.is_borrowed);
    let serialized_size_max = compute_serialized_size_max(&ffi_fields);

    // phase-408 W1 — the DERIVED bound, from the same
    // `generator::common::derive_message_bound` the C pack calls. The poison
    // token is built from the C-style flat name (no `_t`), so one type has ONE
    // poison token across both languages.
    let bound = super::common::derive_message_bound(
        package_name,
        message_name,
        message,
        &format!("{c_pkg_name}_msg_{msg_snake}"),
        resolver,
        lookup,
    )?;
    let unbounded_message = bound.reason.as_ref().map(|reason| {
        cpp_string_literal_body(&format!(
            "{package_name}/{message_name} states no serialized-size bound -- {reason}.              A bound must EXIST before a buffer can be sized from it: bound the field in              the .msg (`string<=64`, `int32[<=8]`), or give it an INLINE `cap` in              nros-codegen.toml. To size this subscription by hand instead, pass an              explicit byte count: nros::bind_subscription_sized<M, C, Method>(node, topic,              self, qos, rx_bytes). If the reason says a nested type could NOT BE RESOLVED,              that is a search-path problem and not a property of the message."
        ))
    });

    // Render C++ header
    let header_template = MessageCppHeaderTemplate {
        package_name,
        message_name,
        type_hash,
        guard_name,
        cpp_package: c_pkg_name.clone(),
        ffi_publish_fn: ffi_publish_fn.clone(),
        ffi_serialize_fn: ffi_serialize_fn.clone(),
        ffi_deserialize_fn: ffi_deserialize_fn.clone(),
        fields: cpp_fields,
        constants,
        dependencies,
        intra_package_includes,
        has_fields,
        serialized_size_max,
        has_borrowed,
        ffi_deserialize_view_fn: format!("{}_borrowed", ffi_deserialize_fn),
        tx_max_serialized_size: bound.tx,
        rx_max_serialized_size: bound.rx,
        unbounded_reason: bound.reason,
        unbounded_token: bound.token,
        unbounded_message,
    };
    let header = crate::render::render("message_cpp.hpp", &header_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Render split Rust FFI glue (types + exports)
    let ffi = render_ffi_rs(FfiRenderSpec {
        package_name,
        message_name,
        file_stem: &file_stem,
        struct_name: &struct_name,
        ffi_publish_fn: &ffi_publish_fn,
        ffi_serialize_fn: &ffi_serialize_fn,
        ffi_deserialize_fn: &ffi_deserialize_fn,
        serialize_fn: &serialize_fn,
        deserialize_fn: &deserialize_fn,
        ffi_fields: &ffi_fields,
        seq_structs: &seq_structs,
    })?;

    Ok(GeneratedCppPackage {
        header,
        ffi,
        header_name,
    })
}

/// Generate C++ code for a service type
pub fn generate_cpp_service_package(
    package_name: &str,
    service_name: &str,
    service: &Service,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedCppServicePackage, GeneratorError> {
    let c_pkg_name = to_c_package_name(package_name);
    let srv_snake = to_snake_case(service_name);

    let guard_name = format!(
        "{}_SRV_{}_HPP",
        c_pkg_name.to_uppercase(),
        srv_snake.to_uppercase()
    );
    let header_name = format!("{}_srv_{}.hpp", c_pkg_name, srv_snake);

    // Request
    let req_struct = format!("{}_srv_{}_request_t", c_pkg_name, srv_snake);
    let req_publish_fn = format!("nros_cpp_publish_{}_srv_{}_request", c_pkg_name, srv_snake);
    let req_serialize_fn = format!(
        "nros_cpp_serialize_{}_srv_{}_request",
        c_pkg_name, srv_snake
    );
    let req_deser_fn = format!(
        "nros_cpp_deserialize_{}_srv_{}_request",
        c_pkg_name, srv_snake
    );
    let req_ser_fn = format!("serialize_{}_srv_{}_request_fields", c_pkg_name, srv_snake);
    let req_deser_fn_inner = format!(
        "deserialize_{}_srv_{}_request_fields",
        c_pkg_name, srv_snake
    );

    let (req_cpp_fields, req_ffi_fields, req_seq_structs) = build_fields(
        &service.request.fields,
        &req_struct,
        Some(package_name),
        &format!("{service_name}_Request"),
        resolver,
    )?;
    let req_constants = build_constants(&service.request.constants);
    let req_serialized_size = compute_serialized_size_max(&req_ffi_fields);

    // Response
    let resp_struct = format!("{}_srv_{}_response_t", c_pkg_name, srv_snake);
    let resp_publish_fn = format!("nros_cpp_publish_{}_srv_{}_response", c_pkg_name, srv_snake);
    let resp_serialize_fn = format!(
        "nros_cpp_serialize_{}_srv_{}_response",
        c_pkg_name, srv_snake
    );
    let resp_deser_fn = format!(
        "nros_cpp_deserialize_{}_srv_{}_response",
        c_pkg_name, srv_snake
    );
    let resp_ser_fn = format!("serialize_{}_srv_{}_response_fields", c_pkg_name, srv_snake);
    let resp_deser_fn_inner = format!(
        "deserialize_{}_srv_{}_response_fields",
        c_pkg_name, srv_snake
    );

    let (resp_cpp_fields, resp_ffi_fields, resp_seq_structs) = build_fields(
        &service.response.fields,
        &resp_struct,
        Some(package_name),
        &format!("{service_name}_Response"),
        resolver,
    )?;
    let resp_constants = build_constants(&service.response.constants);
    let resp_serialized_size = compute_serialized_size_max(&resp_ffi_fields);

    let dependencies = {
        let mut deps = extract_deps(&service.request.fields);
        for d in extract_deps(&service.response.fields) {
            if !deps.contains(&d) {
                deps.push(d);
            }
        }
        deps.sort();
        deps
    };
    let intra_package_includes = {
        let mut incs = extract_intra_package_includes(&service.request.fields, package_name);
        for i in extract_intra_package_includes(&service.response.fields, package_name) {
            if !incs.contains(&i) {
                incs.push(i);
            }
        }
        incs.sort();
        incs
    };

    // Render header
    let header_template = ServiceCppHeaderTemplate {
        package_name,
        service_name,
        type_hash,
        guard_name,
        cpp_package: c_pkg_name.clone(),
        request_ffi_publish_fn: req_publish_fn.clone(),
        request_ffi_serialize_fn: req_serialize_fn.clone(),
        request_ffi_deserialize_fn: req_deser_fn.clone(),
        response_ffi_publish_fn: resp_publish_fn.clone(),
        response_ffi_serialize_fn: resp_serialize_fn.clone(),
        response_ffi_deserialize_fn: resp_deser_fn.clone(),
        request_fields: req_cpp_fields,
        request_constants: req_constants,
        response_fields: resp_cpp_fields,
        response_constants: resp_constants,
        dependencies,
        intra_package_includes,
        has_request_fields: !service.request.fields.is_empty(),
        has_response_fields: !service.response.fields.is_empty(),
        request_serialized_size_max: req_serialized_size,
        response_serialized_size_max: resp_serialized_size,
    };
    let header = crate::render::render("service_cpp.hpp", &header_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Render split FFI glue for request and response
    let request_ffi = render_ffi_rs(FfiRenderSpec {
        package_name,
        message_name: &format!("{}Request", service_name),
        file_stem: &format!("{}_srv_{}_request", c_pkg_name, srv_snake),
        struct_name: &req_struct,
        ffi_publish_fn: &req_publish_fn,
        ffi_serialize_fn: &req_serialize_fn,
        ffi_deserialize_fn: &req_deser_fn,
        serialize_fn: &req_ser_fn,
        deserialize_fn: &req_deser_fn_inner,
        ffi_fields: &req_ffi_fields,
        seq_structs: &req_seq_structs,
    })?;

    let response_ffi = render_ffi_rs(FfiRenderSpec {
        package_name,
        message_name: &format!("{}Response", service_name),
        file_stem: &format!("{}_srv_{}_response", c_pkg_name, srv_snake),
        struct_name: &resp_struct,
        ffi_publish_fn: &resp_publish_fn,
        ffi_serialize_fn: &resp_serialize_fn,
        ffi_deserialize_fn: &resp_deser_fn,
        serialize_fn: &resp_ser_fn,
        deserialize_fn: &resp_deser_fn_inner,
        ffi_fields: &resp_ffi_fields,
        seq_structs: &resp_seq_structs,
    })?;

    Ok(GeneratedCppServicePackage {
        header,
        header_name,
        request_ffi,
        response_ffi,
    })
}

/// Generate C++ code for an action type
pub fn generate_cpp_action_package(
    package_name: &str,
    action_name: &str,
    action: &Action,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedCppActionPackage, GeneratorError> {
    let c_pkg_name = to_c_package_name(package_name);
    let act_snake = to_snake_case(action_name);

    let guard_name = format!(
        "{}_ACTION_{}_HPP",
        c_pkg_name.to_uppercase(),
        act_snake.to_uppercase()
    );
    let header_name = format!("{}_action_{}.hpp", c_pkg_name, act_snake);

    // Helper struct for action sub-message parts
    struct ActionPart {
        publish_fn: String,
        serialize_fn: String,
        deser_fn: String,
        ser_fn: String,
        deser_fn_inner: String,
        struct_name: String,
        cpp_fields: Vec<CppField>,
        ffi_fields: Vec<CppFfiField>,
        seq_structs: Vec<SequenceStructDef>,
        constants: Vec<CConstant>,
        size: usize,
    }

    // `part_name` is the C IDENTIFIER half (`goal`), `key_part` the PAYLOAD NAME
    // half (`Goal`). They are two different vocabularies and this closure needs
    // both: the struct is `..._action_probe_goal_t`, while the RFC-0033 capacity
    // key is `<pkg>/<Action>_Goal.<field>` — the spelling `nros-codegen.toml`
    // uses and the spelling the C (`action.rs`) and Rust generators already
    // look up.
    //
    // phase-432 W1.2 — found by `check-repr-memory-agreement` on its first run.
    // This site passed the lowercase `part_name` for BOTH, so every C++ action
    // payload looked up `Probe_goal.waypoints`, matched nothing, and fell back
    // to the built-in capacity: the C pack emitted `int64_t data[8]` for a
    // `cap = 8` field while the C++ pack emitted `FixedSequence<int64_t, 64>`.
    // A miss in a capacity resolver is silent by construction, so nothing said
    // so — the two packs simply described different memory for one lowered
    // field. `generate_cpp_service_package` spells `{service_name}_Request`
    // correctly, which is why services were unaffected.
    let build_part =
        |part_name: &str, key_part: &str, msg: &Message| -> Result<ActionPart, GeneratorError> {
            let struct_name = format!("{}_action_{}_{}_t", c_pkg_name, act_snake, part_name);
            let (cpp_f, ffi_f, seq_s) = build_fields(
                &msg.fields,
                &struct_name,
                Some(package_name),
                &format!("{action_name}_{key_part}"),
                resolver,
            )?;
            let constants = build_constants(&msg.constants);
            let size = compute_serialized_size_max(&ffi_f);
            Ok(ActionPart {
                publish_fn: format!(
                    "nros_cpp_publish_{}_action_{}_{}",
                    c_pkg_name, act_snake, part_name
                ),
                serialize_fn: format!(
                    "nros_cpp_serialize_{}_action_{}_{}",
                    c_pkg_name, act_snake, part_name
                ),
                deser_fn: format!(
                    "nros_cpp_deserialize_{}_action_{}_{}",
                    c_pkg_name, act_snake, part_name
                ),
                ser_fn: format!(
                    "serialize_{}_action_{}_{}_fields",
                    c_pkg_name, act_snake, part_name
                ),
                deser_fn_inner: format!(
                    "deserialize_{}_action_{}_{}_fields",
                    c_pkg_name, act_snake, part_name
                ),
                struct_name,
                cpp_fields: cpp_f,
                ffi_fields: ffi_f,
                seq_structs: seq_s,
                constants,
                size,
            })
        };

    let goal = build_part("goal", "Goal", &action.spec.goal)?;
    let result = build_part("result", "Result", &action.spec.result)?;
    let feedback = build_part("feedback", "Feedback", &action.spec.feedback)?;

    let dependencies = {
        let mut deps = extract_deps(&action.spec.goal.fields);
        for d in extract_deps(&action.spec.result.fields) {
            if !deps.contains(&d) {
                deps.push(d);
            }
        }
        for d in extract_deps(&action.spec.feedback.fields) {
            if !deps.contains(&d) {
                deps.push(d);
            }
        }
        deps.sort();
        deps
    };
    let intra_package_includes = {
        let mut incs = extract_intra_package_includes(&action.spec.goal.fields, package_name);
        for i in extract_intra_package_includes(&action.spec.result.fields, package_name) {
            if !incs.contains(&i) {
                incs.push(i);
            }
        }
        for i in extract_intra_package_includes(&action.spec.feedback.fields, package_name) {
            if !incs.contains(&i) {
                incs.push(i);
            }
        }
        incs.sort();
        incs
    };

    // Render header
    let header_template = ActionCppHeaderTemplate {
        package_name,
        action_name,
        type_hash,
        guard_name,
        cpp_package: c_pkg_name.clone(),
        goal_ffi_publish_fn: goal.publish_fn.clone(),
        goal_ffi_serialize_fn: goal.serialize_fn.clone(),
        goal_ffi_deserialize_fn: goal.deser_fn.clone(),
        result_ffi_publish_fn: result.publish_fn.clone(),
        result_ffi_serialize_fn: result.serialize_fn.clone(),
        result_ffi_deserialize_fn: result.deser_fn.clone(),
        feedback_ffi_publish_fn: feedback.publish_fn.clone(),
        feedback_ffi_serialize_fn: feedback.serialize_fn.clone(),
        feedback_ffi_deserialize_fn: feedback.deser_fn.clone(),
        goal_fields: goal.cpp_fields,
        goal_constants: goal.constants,
        result_fields: result.cpp_fields,
        result_constants: result.constants,
        feedback_fields: feedback.cpp_fields,
        feedback_constants: feedback.constants,
        dependencies,
        intra_package_includes,
        has_goal_fields: !action.spec.goal.fields.is_empty(),
        has_result_fields: !action.spec.result.fields.is_empty(),
        has_feedback_fields: !action.spec.feedback.fields.is_empty(),
        goal_serialized_size_max: goal.size,
        result_serialized_size_max: result.size,
        feedback_serialized_size_max: feedback.size,
    };
    let header = crate::render::render("action_cpp.hpp", &header_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Render split FFI glue for each part
    let goal_ffi = render_ffi_rs(FfiRenderSpec {
        package_name,
        message_name: &format!("{}Goal", action_name),
        file_stem: &format!("{}_action_{}_goal", c_pkg_name, act_snake),
        struct_name: &goal.struct_name,
        ffi_publish_fn: &goal.publish_fn,
        ffi_serialize_fn: &goal.serialize_fn,
        ffi_deserialize_fn: &goal.deser_fn,
        serialize_fn: &goal.ser_fn,
        deserialize_fn: &goal.deser_fn_inner,
        ffi_fields: &goal.ffi_fields,
        seq_structs: &goal.seq_structs,
    })?;

    let result_ffi = render_ffi_rs(FfiRenderSpec {
        package_name,
        message_name: &format!("{}Result", action_name),
        file_stem: &format!("{}_action_{}_result", c_pkg_name, act_snake),
        struct_name: &result.struct_name,
        ffi_publish_fn: &result.publish_fn,
        ffi_serialize_fn: &result.serialize_fn,
        ffi_deserialize_fn: &result.deser_fn,
        serialize_fn: &result.ser_fn,
        deserialize_fn: &result.deser_fn_inner,
        ffi_fields: &result.ffi_fields,
        seq_structs: &result.seq_structs,
    })?;

    let feedback_ffi = render_ffi_rs(FfiRenderSpec {
        package_name,
        message_name: &format!("{}Feedback", action_name),
        file_stem: &format!("{}_action_{}_feedback", c_pkg_name, act_snake),
        struct_name: &feedback.struct_name,
        ffi_publish_fn: &feedback.publish_fn,
        ffi_serialize_fn: &feedback.serialize_fn,
        ffi_deserialize_fn: &feedback.deser_fn,
        serialize_fn: &feedback.ser_fn,
        deserialize_fn: &feedback.deser_fn_inner,
        ffi_fields: &feedback.ffi_fields,
        seq_structs: &feedback.seq_structs,
    })?;

    Ok(GeneratedCppActionPackage {
        header,
        header_name,
        goal_ffi,
        result_ffi,
        feedback_ffi,
    })
}
