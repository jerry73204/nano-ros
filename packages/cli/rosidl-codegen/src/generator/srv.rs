use super::common::{
    GeneratorError, PayloadLang, SchemaCaps, build_c_fields, build_idiomatic_fields,
    build_nros_fields, build_nros_schema_for_struct, build_rmw_fields,
    ensure_supported_storage_for_payload,
};
use crate::{
    config::CapacityResolver,
    templates::{
        BuildRsTemplate, CConstant, CargoNrosTomlTemplate, CargoTomlTemplate, LibNrosRsTemplate,
        LibRsTemplate, MessageConstant, ServiceCHeaderTemplate, ServiceCSourceTemplate,
        ServiceIdiomaticTemplate, ServiceNrosTemplate, ServiceRmwTemplate,
    },
    types::{
        NrosCodegenMode, c_type_for_constant, constant_value_to_rust, nros_type_for_constant,
        rust_type_for_constant, to_c_package_name,
    },
    utils::{extract_dependencies, needs_big_array, to_snake_case},
};
use rosidl_parser::{FieldType, Message, Service};
use std::collections::HashSet;

pub struct GeneratedServicePackage {
    pub cargo_toml: String,
    pub build_rs: String,
    pub lib_rs: String,
    pub service_rmw: String,
    pub service_idiomatic: String,
}

/// Generate a complete ROS 2 service package with both RMW and idiomatic layers
pub fn generate_service_package(
    package_name: &str,
    service_name: &str,
    service: &Service,
    all_dependencies: &HashSet<String>,
) -> Result<GeneratedServicePackage, GeneratorError> {
    // Extract dependencies from request and response
    let mut req_deps = extract_dependencies(&service.request);
    let resp_deps = extract_dependencies(&service.response);
    req_deps.extend(resp_deps);

    // Combine with externally provided dependencies
    let mut all_deps: Vec<String> = all_dependencies.iter().cloned().collect();
    all_deps.extend(req_deps);
    all_deps.sort();
    all_deps.dedup();

    // Check if we need serde's big-array feature
    let needs_big_array_feature =
        needs_big_array(&service.request) || needs_big_array(&service.response);

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
        has_messages: false,
        has_services: true,
        has_actions: false,
    };
    let lib_rs = crate::render::render("lib.rs", &lib_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    let request_member = format!("{service_name}_Request");
    let response_member = format!("{service_name}_Response");
    // phase-432 W2.5a — the two halves are lowered under their rosidl-convention
    // member names (`<Svc>_Request` / `<Svc>_Response`), the same keys the nros
    // and C paths use, so all three name one member the same way.
    let message_to_rmw_fields =
        |member: &str, msg: &Message| build_rmw_fields(package_name, member, msg);

    let message_to_idiomatic_fields =
        |member: &str, msg: &Message| build_idiomatic_fields(package_name, member, msg);

    let message_to_constants = |msg: &Message, _rmw_layer: bool| {
        msg.constants
            .iter()
            .map(|c| MessageConstant {
                name: c.name.clone(),
                rust_type: rust_type_for_constant(&c.constant_type),
                value: constant_value_to_rust(&c.value),
            })
            .collect()
    };

    // Generate RMW layer service
    let service_rmw_template = ServiceRmwTemplate {
        package_name,
        service_name,
        request_fields: message_to_rmw_fields(&request_member, &service.request),
        request_constants: message_to_constants(&service.request, true),
        response_fields: message_to_rmw_fields(&response_member, &service.response),
        response_constants: message_to_constants(&service.response, true),
    };
    // RFC-0068 Stage 3 (phase-335 W3): rmw Rust service from the minijinja pack.
    let service_rmw = crate::render::render("service_rmw.rs", &service_rmw_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate idiomatic layer service
    let service_idiomatic_template = ServiceIdiomaticTemplate {
        package_name,
        service_name,
        request_fields: message_to_idiomatic_fields(&request_member, &service.request),
        request_constants: message_to_constants(&service.request, false),
        response_fields: message_to_idiomatic_fields(&response_member, &service.response),
        response_constants: message_to_constants(&service.response, false),
    };
    let service_idiomatic =
        crate::render::render("service_idiomatic.rs", &service_idiomatic_template)
            .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedServicePackage {
        cargo_toml,
        build_rs,
        lib_rs,
        service_rmw,
        service_idiomatic,
    })
}

/// Generated nros service package
pub struct GeneratedNrosServicePackage {
    pub cargo_toml: String,
    pub lib_rs: String,
    pub service_rs: String,
}

/// Generate a nros service package
#[allow(clippy::too_many_arguments)] // 3 distinct REP-2011 hashes (Req/Resp/Service)
pub fn generate_nros_service_package(
    package_name: &str,
    service_name: &str,
    service: &Service,
    all_dependencies: &HashSet<String>,
    package_version: &str,
    request_type_hash: &str,
    response_type_hash: &str,
    service_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedNrosServicePackage, GeneratorError> {
    // Extract dependencies from request and response
    let mut req_deps = extract_dependencies(&service.request);
    let resp_deps = extract_dependencies(&service.response);
    req_deps.extend(resp_deps);

    // Combine with externally provided dependencies
    let mut all_deps: Vec<String> = all_dependencies.iter().cloned().collect();
    all_deps.extend(req_deps);
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
        has_messages: false,
        has_services: true,
        has_actions: false,
    };
    let lib_rs = crate::render::render("lib_nros.rs", &lib_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    let request_msg = format!("{service_name}_Request");
    let response_msg = format!("{service_name}_Response");
    // Issues 0343/0344 — Rust srv/action share the message deserialize macro
    // (`_nros_field.jinja`) so `heap` works here; `borrowed` still has no view
    // type, and the C emitter has no `_fini` to free heap fields with. The
    // policy table lives on `ensure_supported_storage_for_payload`.
    ensure_supported_storage_for_payload(
        "service",
        PayloadLang::Rust,
        package_name,
        &request_msg,
        &service.request.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "service",
        PayloadLang::Rust,
        package_name,
        &response_msg,
        &service.response.fields,
        resolver,
    )?;

    // Generate request fields
    let request_fields = build_nros_fields(
        package_name,
        &request_msg,
        &service.request,
        resolver,
        NrosCodegenMode::Crate,
    )?;

    let request_constants: Vec<MessageConstant> = service
        .request
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    // Generate response fields
    let response_fields = build_nros_fields(
        package_name,
        &response_msg,
        &service.response,
        resolver,
        NrosCodegenMode::Crate,
    )?;

    let response_constants: Vec<MessageConstant> = service
        .response
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let has_request_fields = !request_fields.is_empty();
    let has_response_fields = !response_fields.is_empty();
    let has_request_large_array = request_fields.iter().any(|f| f.is_large_array);
    let has_response_large_array = response_fields.iter().any(|f| f.is_large_array);
    let req_struct = format!("{}Request", service_name);
    let resp_struct = format!("{}Response", service_name);
    let req_schema = build_nros_schema_for_struct(
        package_name,
        &req_struct,
        &format!("{}/srv/{}_Request", package_name, service_name),
        "REQ_",
        &service.request.fields,
        &SchemaCaps::new(&request_msg, resolver),
    );
    let resp_schema = build_nros_schema_for_struct(
        package_name,
        &resp_struct,
        &format!("{}/srv/{}_Response", package_name, service_name),
        "RESP_",
        &service.response.fields,
        &SchemaCaps::new(&response_msg, resolver),
    );

    let service_template = ServiceNrosTemplate {
        has_borrowed_request: request_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_response: response_fields.iter().any(|f| f.is_borrowed),
        package_name,
        service_name,
        request_type_hash,
        response_type_hash,
        service_hash,
        request_fields,
        request_constants,
        response_fields,
        response_constants,
        has_request_fields,
        has_response_fields,
        has_request_large_array,
        has_response_large_array,
        inline_mode: false,
        req_schema_helper_consts: req_schema.helper_consts,
        req_schema_fields_block: req_schema.fields_block,
        req_schema_type_name: req_schema.nros_type_name,
        resp_schema_helper_consts: resp_schema.helper_consts,
        resp_schema_fields_block: resp_schema.fields_block,
        resp_schema_type_name: resp_schema.nros_type_name,
    };
    let service_rs = crate::render::render("service_nros.rs", &service_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedNrosServicePackage {
        cargo_toml,
        lib_rs,
        service_rs,
    })
}

/// Generate a single service's Rust code in inline mode.
pub fn generate_nros_inline_service(
    package_name: &str,
    service_name: &str,
    service: &Service,
    request_type_hash: &str,
    response_type_hash: &str,
    service_hash: &str,
    resolver: &CapacityResolver,
) -> Result<String, GeneratorError> {
    let mode = NrosCodegenMode::Inline;
    let request_msg = format!("{service_name}_Request");
    let response_msg = format!("{service_name}_Response");
    // Issues 0343/0344 — Rust srv/action share the message deserialize macro
    // (`_nros_field.jinja`) so `heap` works here; `borrowed` still has no view
    // type, and the C emitter has no `_fini` to free heap fields with. The
    // policy table lives on `ensure_supported_storage_for_payload`.
    ensure_supported_storage_for_payload(
        "service",
        PayloadLang::Rust,
        package_name,
        &request_msg,
        &service.request.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "service",
        PayloadLang::Rust,
        package_name,
        &response_msg,
        &service.response.fields,
        resolver,
    )?;

    let request_fields =
        build_nros_fields(package_name, &request_msg, &service.request, resolver, mode)?;

    let request_constants: Vec<MessageConstant> = service
        .request
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let response_fields = build_nros_fields(
        package_name,
        &response_msg,
        &service.response,
        resolver,
        mode,
    )?;

    let response_constants: Vec<MessageConstant> = service
        .response
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let has_request_fields = !request_fields.is_empty();
    let has_response_fields = !response_fields.is_empty();
    let has_request_large_array = request_fields.iter().any(|f| f.is_large_array);
    let has_response_large_array = response_fields.iter().any(|f| f.is_large_array);

    let req_struct = format!("{}Request", service_name);
    let resp_struct = format!("{}Response", service_name);
    let req_schema = build_nros_schema_for_struct(
        package_name,
        &req_struct,
        &format!("{}/srv/{}_Request", package_name, service_name),
        "REQ_",
        &service.request.fields,
        &SchemaCaps::new(&request_msg, resolver),
    );
    let resp_schema = build_nros_schema_for_struct(
        package_name,
        &resp_struct,
        &format!("{}/srv/{}_Response", package_name, service_name),
        "RESP_",
        &service.response.fields,
        &SchemaCaps::new(&response_msg, resolver),
    );

    let template = ServiceNrosTemplate {
        has_borrowed_request: request_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_response: response_fields.iter().any(|f| f.is_borrowed),
        package_name,
        service_name,
        request_type_hash,
        response_type_hash,
        service_hash,
        request_fields,
        request_constants,
        response_fields,
        response_constants,
        has_request_fields,
        has_response_fields,
        has_request_large_array,
        has_response_large_array,
        inline_mode: true,
        req_schema_helper_consts: req_schema.helper_consts,
        req_schema_fields_block: req_schema.fields_block,
        req_schema_type_name: req_schema.nros_type_name,
        resp_schema_helper_consts: resp_schema.helper_consts,
        resp_schema_fields_block: resp_schema.fields_block,
        resp_schema_type_name: resp_schema.nros_type_name,
    };

    crate::render::render("service_nros.rs", &template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))
}

/// Generated C service package
pub struct GeneratedCServicePackage {
    /// Header file content (.h)
    pub header: String,
    /// Source file content (.c)
    pub source: String,
    /// Header filename
    pub header_name: String,
    /// Source filename
    pub source_name: String,
}

/// Generate C code for a service type
pub fn generate_c_service_package(
    package_name: &str,
    service_name: &str,
    service: &Service,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedCServicePackage, GeneratorError> {
    let c_pkg_name = to_c_package_name(package_name);
    let srv_snake = to_snake_case(service_name);

    // Build struct and guard names
    let service_struct_name = format!("{}_srv_{}", c_pkg_name, srv_snake);
    let request_struct_name = format!("{}_srv_{}_request", c_pkg_name, srv_snake);
    let response_struct_name = format!("{}_srv_{}_response", c_pkg_name, srv_snake);
    let guard_name = format!(
        "{}_SRV_{}_H",
        c_pkg_name.to_uppercase(),
        srv_snake.to_uppercase()
    );
    let constant_prefix = format!(
        "{}_SRV_{}",
        c_pkg_name.to_uppercase(),
        srv_snake.to_uppercase()
    );
    let header_name = format!("{}_srv_{}.h", c_pkg_name, srv_snake);
    let source_name = format!("{}_srv_{}.c", c_pkg_name, srv_snake);

    // Extract dependencies from both request and response.
    // Mirrors the logic in msg.rs: produce one `type_includes` entry per
    // referenced struct (same-package OR cross-package) and a separate
    // `dependencies` list of other packages (used elsewhere for metadata).
    let mut dependencies = Vec::new();
    let mut type_includes = Vec::new();
    for field in service
        .request
        .fields
        .iter()
        .chain(service.response.fields.iter())
    {
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
                // Cross-package: include with subdirectory path.
                if !dependencies.contains(&dep) {
                    dependencies.push(dep.clone());
                }
                format!("{}/msg/{}", dep, header_filename)
            } else {
                // Intra-package: the srv .h lives at <pkg>/srv/, so the
                // msg header is one level up in ../msg/.
                format!("../msg/{}", header_filename)
            };
            if !type_includes.contains(&type_header) {
                type_includes.push(type_header);
            }
        }
    }
    dependencies.sort();
    type_includes.sort();

    let request_msg = format!("{service_name}_Request");
    let response_msg = format!("{service_name}_Response");
    // Issues 0343/0344 — Rust srv/action share the message deserialize macro
    // (`_nros_field.jinja`) so `heap` works here; `borrowed` still has no view
    // type, and the C emitter has no `_fini` to free heap fields with. The
    // policy table lives on `ensure_supported_storage_for_payload`.
    ensure_supported_storage_for_payload(
        "service",
        PayloadLang::C,
        package_name,
        &request_msg,
        &service.request.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "service",
        PayloadLang::C,
        package_name,
        &response_msg,
        &service.response.fields,
        resolver,
    )?;

    // Build C fields for request
    let request_fields =
        build_c_fields(Some(package_name), &request_msg, &service.request, resolver)?;

    let request_constants: Vec<CConstant> = service
        .request
        .constants
        .iter()
        .map(|constant| CConstant {
            name: constant.name.clone(),
            c_type: c_type_for_constant(&constant.constant_type),
            value: constant_value_to_rust(&constant.value),
        })
        .collect();

    // Build C fields for response
    let response_fields = build_c_fields(
        Some(package_name),
        &response_msg,
        &service.response,
        resolver,
    )?;

    let response_constants: Vec<CConstant> = service
        .response
        .constants
        .iter()
        .map(|constant| CConstant {
            name: constant.name.clone(),
            c_type: c_type_for_constant(&constant.constant_type),
            value: constant_value_to_rust(&constant.value),
        })
        .collect();

    let has_request_fields = !request_fields.is_empty();
    let has_response_fields = !response_fields.is_empty();

    // Generate header
    let header_template = ServiceCHeaderTemplate {
        has_borrowed_request: request_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_response: response_fields.iter().any(|f| f.is_borrowed),
        package_name,
        service_name,
        type_hash,
        guard_name,
        service_struct_name: service_struct_name.clone(),
        request_struct_name: request_struct_name.clone(),
        response_struct_name: response_struct_name.clone(),
        constant_prefix,
        request_fields: request_fields.clone(),
        request_constants,
        response_fields: response_fields.clone(),
        response_constants,
        dependencies,
        type_includes,
        has_request_fields,
        has_response_fields,
    };
    // RFC-0068 Stage 3 (phase-335 W2): C service from the minijinja pack.
    let header = crate::render::render_c("service.h", &header_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate source
    let source_template = ServiceCSourceTemplate {
        has_borrowed_request: request_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_response: response_fields.iter().any(|f| f.is_borrowed),
        package_name,
        service_name,
        type_hash,
        header_name: header_name.clone(),
        service_struct_name,
        request_struct_name,
        response_struct_name,
        request_fields,
        response_fields,
        has_request_fields,
        has_response_fields,
    };
    let source = crate::render::render_c("service.c", &source_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedCServicePackage {
        header,
        source,
        header_name,
        source_name,
    })
}
