use super::common::{
    GeneratorError, PayloadLang, SchemaCaps, build_action_envelope_schemas, build_c_fields,
    build_idiomatic_fields, build_nros_fields, build_nros_schema_for_struct, build_rmw_fields,
    ensure_supported_storage_for_payload,
};
use crate::{
    config::CapacityResolver,
    templates::{
        ActionCHeaderTemplate, ActionCSourceTemplate, ActionIdiomaticTemplate, ActionNrosTemplate,
        ActionRmwTemplate, BuildRsTemplate, CConstant, CargoNrosTomlTemplate, CargoTomlTemplate,
        LibNrosRsTemplate, LibRsTemplate, MessageConstant,
    },
    types::{
        NrosCodegenMode, c_type_for_constant, constant_value_to_rust, nros_type_for_constant,
        rust_type_for_constant, to_c_package_name,
    },
    utils::{extract_dependencies, needs_big_array, to_snake_case},
};
use rosidl_parser::{Action, FieldType, Message};
use std::collections::HashSet;

pub struct GeneratedActionPackage {
    pub cargo_toml: String,
    pub build_rs: String,
    pub lib_rs: String,
    pub action_rmw: String,
    pub action_idiomatic: String,
}

/// Generate a complete ROS 2 action package with both RMW and idiomatic layers
pub fn generate_action_package(
    package_name: &str,
    action_name: &str,
    action: &Action,
    all_dependencies: &HashSet<String>,
) -> Result<GeneratedActionPackage, GeneratorError> {
    // Extract dependencies from goal, result, and feedback
    let mut goal_deps = extract_dependencies(&action.spec.goal);
    let result_deps = extract_dependencies(&action.spec.result);
    let feedback_deps = extract_dependencies(&action.spec.feedback);
    goal_deps.extend(result_deps);
    goal_deps.extend(feedback_deps);

    // Combine with externally provided dependencies
    let mut all_deps: Vec<String> = all_dependencies.iter().cloned().collect();
    all_deps.extend(goal_deps);
    all_deps.sort();
    all_deps.dedup();

    // Check if we need serde's big-array feature
    let needs_big_array_feature = needs_big_array(&action.spec.goal)
        || needs_big_array(&action.spec.result)
        || needs_big_array(&action.spec.feedback);

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
        has_services: false,
        has_actions: true,
    };
    let lib_rs = crate::render::render("lib.rs", &lib_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // phase-432 W2.5a — each part is lowered under its rosidl-convention member
    // name (`<Action>_Goal` / `_Result` / `_Feedback`), the same keys the nros
    // and C paths use, so all three name one member the same way.
    let goal_member = format!("{action_name}_Goal");
    let result_member = format!("{action_name}_Result");
    let feedback_member = format!("{action_name}_Feedback");
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

    // Generate RMW layer action
    let action_rmw_template = ActionRmwTemplate {
        package_name,
        action_name,
        goal_fields: message_to_rmw_fields(&goal_member, &action.spec.goal),
        goal_constants: message_to_constants(&action.spec.goal, true),
        result_fields: message_to_rmw_fields(&result_member, &action.spec.result),
        result_constants: message_to_constants(&action.spec.result, true),
        feedback_fields: message_to_rmw_fields(&feedback_member, &action.spec.feedback),
        feedback_constants: message_to_constants(&action.spec.feedback, true),
    };
    // RFC-0068 Stage 3 (phase-335 W3): rmw Rust action from the minijinja pack.
    let action_rmw = crate::render::render("action_rmw.rs", &action_rmw_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate idiomatic layer action
    let action_idiomatic_template = ActionIdiomaticTemplate {
        package_name,
        action_name,
        goal_fields: message_to_idiomatic_fields(&goal_member, &action.spec.goal),
        goal_constants: message_to_constants(&action.spec.goal, false),
        result_fields: message_to_idiomatic_fields(&result_member, &action.spec.result),
        result_constants: message_to_constants(&action.spec.result, false),
        feedback_fields: message_to_idiomatic_fields(&feedback_member, &action.spec.feedback),
        feedback_constants: message_to_constants(&action.spec.feedback, false),
    };
    let action_idiomatic = crate::render::render("action_idiomatic.rs", &action_idiomatic_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedActionPackage {
        cargo_toml,
        build_rs,
        lib_rs,
        action_rmw,
        action_idiomatic,
    })
}

/// Generated nros action package
pub struct GeneratedNrosActionPackage {
    pub cargo_toml: String,
    pub lib_rs: String,
    pub action_rs: String,
}

/// Generate a nros action package
pub fn generate_nros_action_package(
    package_name: &str,
    action_name: &str,
    action: &Action,
    all_dependencies: &HashSet<String>,
    package_version: &str,
    hashes: &crate::rihs::ActionTypeHashes,
    resolver: &CapacityResolver,
) -> Result<GeneratedNrosActionPackage, GeneratorError> {
    // Extract dependencies from goal, result, and feedback
    let mut goal_deps = extract_dependencies(&action.spec.goal);
    let result_deps = extract_dependencies(&action.spec.result);
    let feedback_deps = extract_dependencies(&action.spec.feedback);
    goal_deps.extend(result_deps);
    goal_deps.extend(feedback_deps);

    // Combine with externally provided dependencies
    let mut all_deps: Vec<String> = all_dependencies.iter().cloned().collect();
    all_deps.extend(goal_deps);

    // Phase 212.K.7.1.d: action envelope structs reference
    // `unique_identifier_msgs::msg::UUID` (every envelope with a
    // `goal_id`) + `builtin_interfaces::msg::Time` (SendGoal_Response
    // `stamp`). Inject the deps unconditionally so the generated
    // `Cargo.toml` resolves the nested-type `<Pkg::msg::T as Message>`
    // expansions.
    if package_name != "unique_identifier_msgs" {
        all_deps.push("unique_identifier_msgs".to_string());
    }
    if package_name != "builtin_interfaces" {
        all_deps.push("builtin_interfaces".to_string());
    }
    // Phase 244 E3 (RFC-0044) — the generated `impl RosAction::register_protocol_types`
    // names `action_msgs::srv::CancelGoal_{Request,Response}` + `msg::GoalStatusArray`,
    // so the action crate must depend on `action_msgs` (its own envelopes are local,
    // these three are not). `action_msgs` itself has no actions, so no self-dep.
    if package_name != "action_msgs" {
        all_deps.push("action_msgs".to_string());
    }
    all_deps.sort();
    all_deps.dedup();

    // Generate Cargo.toml
    let cargo_toml_template = CargoNrosTomlTemplate {
        package_name,
        package_version,
        dependencies: &all_deps,
        has_actions: true,
    };
    let cargo_toml = crate::render::render("cargo_nros.toml", &cargo_toml_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate lib.rs
    let lib_rs_template = LibNrosRsTemplate {
        has_messages: false,
        has_services: false,
        has_actions: true,
    };
    let lib_rs = crate::render::render("lib_nros.rs", &lib_rs_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    let goal_msg = format!("{action_name}_Goal");
    let result_msg = format!("{action_name}_Result");
    let feedback_msg = format!("{action_name}_Feedback");
    // Issues 0343/0344 — Rust srv/action share the message deserialize macro
    // (`_nros_field.jinja`) so `heap` works here; `borrowed` still has no view
    // type, and the C emitter has no `_fini` to free heap fields with. The
    // policy table lives on `ensure_supported_storage_for_payload`.
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::Rust,
        package_name,
        &goal_msg,
        &action.spec.goal.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::Rust,
        package_name,
        &result_msg,
        &action.spec.result.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::Rust,
        package_name,
        &feedback_msg,
        &action.spec.feedback.fields,
        resolver,
    )?;

    // Generate goal fields
    let goal_fields = build_nros_fields(
        package_name,
        &goal_msg,
        &action.spec.goal,
        resolver,
        NrosCodegenMode::Crate,
    )?;

    let goal_constants: Vec<MessageConstant> = action
        .spec
        .goal
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    // Generate result fields
    let result_fields = build_nros_fields(
        package_name,
        &result_msg,
        &action.spec.result,
        resolver,
        NrosCodegenMode::Crate,
    )?;

    let result_constants: Vec<MessageConstant> = action
        .spec
        .result
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    // Generate feedback fields
    let feedback_fields = build_nros_fields(
        package_name,
        &feedback_msg,
        &action.spec.feedback,
        resolver,
        NrosCodegenMode::Crate,
    )?;

    let feedback_constants: Vec<MessageConstant> = action
        .spec
        .feedback
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let has_goal_fields = !goal_fields.is_empty();
    let has_result_fields = !result_fields.is_empty();
    let has_feedback_fields = !feedback_fields.is_empty();
    let has_goal_large_array = goal_fields.iter().any(|f| f.is_large_array);
    let has_result_large_array = result_fields.iter().any(|f| f.is_large_array);
    let has_feedback_large_array = feedback_fields.iter().any(|f| f.is_large_array);

    let goal_struct = format!("{}Goal", action_name);
    let result_struct = format!("{}Result", action_name);
    let feedback_struct = format!("{}Feedback", action_name);
    let goal_schema = build_nros_schema_for_struct(
        package_name,
        &goal_struct,
        &format!("{}/action/{}_Goal", package_name, action_name),
        "GOAL_",
        &action.spec.goal.fields,
        &SchemaCaps::new(&goal_msg, resolver),
    );
    let result_schema = build_nros_schema_for_struct(
        package_name,
        &result_struct,
        &format!("{}/action/{}_Result", package_name, action_name),
        "RESULT_",
        &action.spec.result.fields,
        &SchemaCaps::new(&result_msg, resolver),
    );
    let feedback_schema = build_nros_schema_for_struct(
        package_name,
        &feedback_struct,
        &format!("{}/action/{}_Feedback", package_name, action_name),
        "FEEDBACK_",
        &action.spec.feedback.fields,
        &SchemaCaps::new(&feedback_msg, resolver),
    );

    let envelopes = build_action_envelope_schemas(package_name, action_name);

    let action_template = ActionNrosTemplate {
        has_borrowed_goal: goal_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_result: result_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_feedback: feedback_fields.iter().any(|f| f.is_borrowed),
        package_name,
        action_name,
        goal_type_hash: &hashes.goal,
        result_type_hash: &hashes.result,
        feedback_type_hash: &hashes.feedback,
        send_goal_request_type_hash: &hashes.send_goal_request,
        send_goal_response_type_hash: &hashes.send_goal_response,
        get_result_request_type_hash: &hashes.get_result_request,
        get_result_response_type_hash: &hashes.get_result_response,
        feedback_message_type_hash: &hashes.feedback_message,
        action_hash: &hashes.action,
        send_goal_service_hash: &hashes.send_goal_service,
        get_result_service_hash: &hashes.get_result_service,
        goal_fields,
        goal_constants,
        result_fields,
        result_constants,
        feedback_fields,
        feedback_constants,
        has_goal_fields,
        has_result_fields,
        has_feedback_fields,
        has_goal_large_array,
        has_result_large_array,
        has_feedback_large_array,
        inline_mode: false,
        goal_schema_helper_consts: goal_schema.helper_consts,
        goal_schema_fields_block: goal_schema.fields_block,
        goal_schema_type_name: goal_schema.nros_type_name,
        result_schema_helper_consts: result_schema.helper_consts,
        result_schema_fields_block: result_schema.fields_block,
        result_schema_type_name: result_schema.nros_type_name,
        feedback_schema_helper_consts: feedback_schema.helper_consts,
        feedback_schema_fields_block: feedback_schema.fields_block,
        feedback_schema_type_name: feedback_schema.nros_type_name,
        send_goal_request_schema_helper_consts: envelopes.send_goal_request.helper_consts,
        send_goal_request_schema_fields_block: envelopes.send_goal_request.fields_block,
        send_goal_request_schema_type_name: envelopes.send_goal_request.nros_type_name,
        send_goal_response_schema_helper_consts: envelopes.send_goal_response.helper_consts,
        send_goal_response_schema_fields_block: envelopes.send_goal_response.fields_block,
        send_goal_response_schema_type_name: envelopes.send_goal_response.nros_type_name,
        get_result_request_schema_helper_consts: envelopes.get_result_request.helper_consts,
        get_result_request_schema_fields_block: envelopes.get_result_request.fields_block,
        get_result_request_schema_type_name: envelopes.get_result_request.nros_type_name,
        get_result_response_schema_helper_consts: envelopes.get_result_response.helper_consts,
        get_result_response_schema_fields_block: envelopes.get_result_response.fields_block,
        get_result_response_schema_type_name: envelopes.get_result_response.nros_type_name,
        feedback_message_schema_helper_consts: envelopes.feedback_message.helper_consts,
        feedback_message_schema_fields_block: envelopes.feedback_message.fields_block,
        feedback_message_schema_type_name: envelopes.feedback_message.nros_type_name,
    };
    let action_rs = crate::render::render("action_nros.rs", &action_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedNrosActionPackage {
        cargo_toml,
        lib_rs,
        action_rs,
    })
}

/// Generate a single action's Rust code in inline mode.
pub fn generate_nros_inline_action(
    package_name: &str,
    action_name: &str,
    action: &Action,
    hashes: &crate::rihs::ActionTypeHashes,
    resolver: &CapacityResolver,
) -> Result<String, GeneratorError> {
    let mode = NrosCodegenMode::Inline;
    let goal_msg = format!("{action_name}_Goal");
    let result_msg = format!("{action_name}_Result");
    let feedback_msg = format!("{action_name}_Feedback");
    // Issues 0343/0344 — Rust srv/action share the message deserialize macro
    // (`_nros_field.jinja`) so `heap` works here; `borrowed` still has no view
    // type, and the C emitter has no `_fini` to free heap fields with. The
    // policy table lives on `ensure_supported_storage_for_payload`.
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::Rust,
        package_name,
        &goal_msg,
        &action.spec.goal.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::Rust,
        package_name,
        &result_msg,
        &action.spec.result.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::Rust,
        package_name,
        &feedback_msg,
        &action.spec.feedback.fields,
        resolver,
    )?;

    let goal_fields =
        build_nros_fields(package_name, &goal_msg, &action.spec.goal, resolver, mode)?;

    let goal_constants: Vec<MessageConstant> = action
        .spec
        .goal
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let result_fields = build_nros_fields(
        package_name,
        &result_msg,
        &action.spec.result,
        resolver,
        mode,
    )?;

    let result_constants: Vec<MessageConstant> = action
        .spec
        .result
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let feedback_fields = build_nros_fields(
        package_name,
        &feedback_msg,
        &action.spec.feedback,
        resolver,
        mode,
    )?;

    let feedback_constants: Vec<MessageConstant> = action
        .spec
        .feedback
        .constants
        .iter()
        .map(|c| MessageConstant {
            name: c.name.clone(),
            rust_type: nros_type_for_constant(&c.constant_type),
            value: constant_value_to_rust(&c.value),
        })
        .collect();

    let has_goal_fields = !goal_fields.is_empty();
    let has_result_fields = !result_fields.is_empty();
    let has_feedback_fields = !feedback_fields.is_empty();
    let has_goal_large_array = goal_fields.iter().any(|f| f.is_large_array);
    let has_result_large_array = result_fields.iter().any(|f| f.is_large_array);
    let has_feedback_large_array = feedback_fields.iter().any(|f| f.is_large_array);

    let goal_struct = format!("{}Goal", action_name);
    let result_struct = format!("{}Result", action_name);
    let feedback_struct = format!("{}Feedback", action_name);
    let goal_schema = build_nros_schema_for_struct(
        package_name,
        &goal_struct,
        &format!("{}/action/{}_Goal", package_name, action_name),
        "GOAL_",
        &action.spec.goal.fields,
        &SchemaCaps::new(&goal_msg, resolver),
    );
    let result_schema = build_nros_schema_for_struct(
        package_name,
        &result_struct,
        &format!("{}/action/{}_Result", package_name, action_name),
        "RESULT_",
        &action.spec.result.fields,
        &SchemaCaps::new(&result_msg, resolver),
    );
    let feedback_schema = build_nros_schema_for_struct(
        package_name,
        &feedback_struct,
        &format!("{}/action/{}_Feedback", package_name, action_name),
        "FEEDBACK_",
        &action.spec.feedback.fields,
        &SchemaCaps::new(&feedback_msg, resolver),
    );

    let envelopes = build_action_envelope_schemas(package_name, action_name);

    let template = ActionNrosTemplate {
        has_borrowed_goal: goal_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_result: result_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_feedback: feedback_fields.iter().any(|f| f.is_borrowed),
        package_name,
        action_name,
        goal_type_hash: &hashes.goal,
        result_type_hash: &hashes.result,
        feedback_type_hash: &hashes.feedback,
        send_goal_request_type_hash: &hashes.send_goal_request,
        send_goal_response_type_hash: &hashes.send_goal_response,
        get_result_request_type_hash: &hashes.get_result_request,
        get_result_response_type_hash: &hashes.get_result_response,
        feedback_message_type_hash: &hashes.feedback_message,
        action_hash: &hashes.action,
        send_goal_service_hash: &hashes.send_goal_service,
        get_result_service_hash: &hashes.get_result_service,
        goal_fields,
        goal_constants,
        result_fields,
        result_constants,
        feedback_fields,
        feedback_constants,
        has_goal_fields,
        has_result_fields,
        has_feedback_fields,
        has_goal_large_array,
        has_result_large_array,
        has_feedback_large_array,
        inline_mode: true,
        goal_schema_helper_consts: goal_schema.helper_consts,
        goal_schema_fields_block: goal_schema.fields_block,
        goal_schema_type_name: goal_schema.nros_type_name,
        result_schema_helper_consts: result_schema.helper_consts,
        result_schema_fields_block: result_schema.fields_block,
        result_schema_type_name: result_schema.nros_type_name,
        feedback_schema_helper_consts: feedback_schema.helper_consts,
        feedback_schema_fields_block: feedback_schema.fields_block,
        feedback_schema_type_name: feedback_schema.nros_type_name,
        send_goal_request_schema_helper_consts: envelopes.send_goal_request.helper_consts,
        send_goal_request_schema_fields_block: envelopes.send_goal_request.fields_block,
        send_goal_request_schema_type_name: envelopes.send_goal_request.nros_type_name,
        send_goal_response_schema_helper_consts: envelopes.send_goal_response.helper_consts,
        send_goal_response_schema_fields_block: envelopes.send_goal_response.fields_block,
        send_goal_response_schema_type_name: envelopes.send_goal_response.nros_type_name,
        get_result_request_schema_helper_consts: envelopes.get_result_request.helper_consts,
        get_result_request_schema_fields_block: envelopes.get_result_request.fields_block,
        get_result_request_schema_type_name: envelopes.get_result_request.nros_type_name,
        get_result_response_schema_helper_consts: envelopes.get_result_response.helper_consts,
        get_result_response_schema_fields_block: envelopes.get_result_response.fields_block,
        get_result_response_schema_type_name: envelopes.get_result_response.nros_type_name,
        feedback_message_schema_helper_consts: envelopes.feedback_message.helper_consts,
        feedback_message_schema_fields_block: envelopes.feedback_message.fields_block,
        feedback_message_schema_type_name: envelopes.feedback_message.nros_type_name,
    };

    crate::render::render("action_nros.rs", &template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))
}

/// Generated C action package
pub struct GeneratedCActionPackage {
    /// Header file content (.h)
    pub header: String,
    /// Source file content (.c)
    pub source: String,
    /// Header filename
    pub header_name: String,
    /// Source filename
    pub source_name: String,
}

/// Generate C code for an action type
pub fn generate_c_action_package(
    package_name: &str,
    action_name: &str,
    action: &Action,
    type_hash: &str,
    resolver: &CapacityResolver,
) -> Result<GeneratedCActionPackage, GeneratorError> {
    let c_pkg_name = to_c_package_name(package_name);
    let action_snake = to_snake_case(action_name);

    // Build struct and guard names
    let action_struct_name = format!("{}_action_{}", c_pkg_name, action_snake);
    let goal_struct_name = format!("{}_action_{}_goal", c_pkg_name, action_snake);
    let result_struct_name = format!("{}_action_{}_result", c_pkg_name, action_snake);
    let feedback_struct_name = format!("{}_action_{}_feedback", c_pkg_name, action_snake);
    let guard_name = format!(
        "{}_ACTION_{}_H",
        c_pkg_name.to_uppercase(),
        action_snake.to_uppercase()
    );
    let constant_prefix = format!(
        "{}_ACTION_{}",
        c_pkg_name.to_uppercase(),
        action_snake.to_uppercase()
    );
    let header_name = format!("{}_action_{}.h", c_pkg_name, action_snake);
    let source_name = format!("{}_action_{}.c", c_pkg_name, action_snake);

    // Extract dependencies from goal, result, and feedback.
    // Same pattern as msg.rs: per-type `type_includes` with correct paths,
    // plus a flat `dependencies` list for metadata.
    let mut dependencies = Vec::new();
    let mut type_includes = Vec::new();
    for field in action
        .spec
        .goal
        .fields
        .iter()
        .chain(action.spec.result.fields.iter())
        .chain(action.spec.feedback.fields.iter())
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
                // Intra-package: the action .h lives at <pkg>/action/, so the
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

    let goal_msg = format!("{action_name}_Goal");
    let result_msg = format!("{action_name}_Result");
    let feedback_msg = format!("{action_name}_Feedback");
    // Issues 0343/0344 — Rust srv/action share the message deserialize macro
    // (`_nros_field.jinja`) so `heap` works here; `borrowed` still has no view
    // type, and the C emitter has no `_fini` to free heap fields with. The
    // policy table lives on `ensure_supported_storage_for_payload`.
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::C,
        package_name,
        &goal_msg,
        &action.spec.goal.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::C,
        package_name,
        &result_msg,
        &action.spec.result.fields,
        resolver,
    )?;
    ensure_supported_storage_for_payload(
        "action",
        PayloadLang::C,
        package_name,
        &feedback_msg,
        &action.spec.feedback.fields,
        resolver,
    )?;

    // Build C fields for goal
    let goal_fields = build_c_fields(Some(package_name), &goal_msg, &action.spec.goal, resolver)?;

    let goal_constants: Vec<CConstant> = action
        .spec
        .goal
        .constants
        .iter()
        .map(|constant| CConstant {
            name: constant.name.clone(),
            c_type: c_type_for_constant(&constant.constant_type),
            value: constant_value_to_rust(&constant.value),
        })
        .collect();

    // Build C fields for result
    let result_fields = build_c_fields(
        Some(package_name),
        &result_msg,
        &action.spec.result,
        resolver,
    )?;

    let result_constants: Vec<CConstant> = action
        .spec
        .result
        .constants
        .iter()
        .map(|constant| CConstant {
            name: constant.name.clone(),
            c_type: c_type_for_constant(&constant.constant_type),
            value: constant_value_to_rust(&constant.value),
        })
        .collect();

    // Build C fields for feedback
    let feedback_fields = build_c_fields(
        Some(package_name),
        &feedback_msg,
        &action.spec.feedback,
        resolver,
    )?;

    let feedback_constants: Vec<CConstant> = action
        .spec
        .feedback
        .constants
        .iter()
        .map(|constant| CConstant {
            name: constant.name.clone(),
            c_type: c_type_for_constant(&constant.constant_type),
            value: constant_value_to_rust(&constant.value),
        })
        .collect();

    let has_goal_fields = !goal_fields.is_empty();
    let has_result_fields = !result_fields.is_empty();
    let has_feedback_fields = !feedback_fields.is_empty();

    // Generate header
    let header_template = ActionCHeaderTemplate {
        has_borrowed_goal: goal_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_result: result_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_feedback: feedback_fields.iter().any(|f| f.is_borrowed),
        package_name,
        action_name,
        type_hash,
        guard_name,
        action_struct_name: action_struct_name.clone(),
        goal_struct_name: goal_struct_name.clone(),
        result_struct_name: result_struct_name.clone(),
        feedback_struct_name: feedback_struct_name.clone(),
        constant_prefix,
        goal_fields: goal_fields.clone(),
        goal_constants,
        result_fields: result_fields.clone(),
        result_constants,
        feedback_fields: feedback_fields.clone(),
        feedback_constants,
        dependencies,
        type_includes,
        has_goal_fields,
        has_result_fields,
        has_feedback_fields,
    };
    // RFC-0068 Stage 3 (phase-335 W2): C action from the minijinja pack.
    let header = crate::render::render_c("action.h", &header_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    // Generate source
    let source_template = ActionCSourceTemplate {
        has_borrowed_goal: goal_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_result: result_fields.iter().any(|f| f.is_borrowed),
        has_borrowed_feedback: feedback_fields.iter().any(|f| f.is_borrowed),
        package_name,
        action_name,
        type_hash,
        header_name: header_name.clone(),
        action_struct_name,
        goal_struct_name,
        result_struct_name,
        feedback_struct_name,
        goal_fields,
        result_fields,
        feedback_fields,
        has_goal_fields,
        has_result_fields,
        has_feedback_fields,
    };
    let source = crate::render::render_c("action.c", &source_template)
        .map_err(|e| GeneratorError::RenderError(e.to_string()))?;

    Ok(GeneratedCActionPackage {
        header,
        source,
        header_name,
        source_name,
    })
}
