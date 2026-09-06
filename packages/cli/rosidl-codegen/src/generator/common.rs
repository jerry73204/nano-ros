use crate::{
    config::{CapacityResolver, FieldKind as CapFieldKind, FieldStorage, StorageMode},
    templates::{
        CField, CppFfiField, CppField, FieldKind, IdiomaticField, NrosField, RmwField,
        SequenceStructDef,
    },
    types::{
        C_DEFAULT_SEQUENCE_CAPACITY, CPP_DEFAULT_SEQUENCE_CAPACITY, CPP_DEFAULT_STRING_CAPACITY,
        NrosCodegenMode, c_cdr_read_method, c_cdr_write_method, c_type_for_field_heap,
        constant_value_to_rust, cpp_type_for_field_heap, escape_keyword, repr_c_type_for_field,
        to_c_package_name,
    },
    utils::to_snake_case,
};
use rosidl_parser::{FieldType, PrimitiveType};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum GeneratorError {
    #[error("Template rendering failed: {0}")]
    RenderError(String),

    #[error("Invalid message structure: {0}")]
    InvalidMessage(String),

    #[error(
        "{package}/{message}.{field}: storage mode '{mode}' is not yet supported \
         (Phase 229 ships 'owned'; 'heap' lands in 229.5, 'borrowed' in 229.6)"
    )]
    UnsupportedStorageMode {
        package: String,
        message: String,
        field: String,
        mode: &'static str,
    },

    #[error(
        "{package}/{message}.{field}: storage mode '{mode}' is not supported on {entity} \
         payloads — only messages implement it today (issue 0343). Set this field to \
         'inline' in nros-codegen.toml, or move the payload into a .msg."
    )]
    UnsupportedStorageModeForPayload {
        entity: String,
        package: String,
        message: String,
        field: String,
        mode: &'static str,
    },

    #[error(
        "{package}/{message}.{field}: `view` mode does not support element \
         type `{element}` — only fixed-width primitive sequences (`uint8[]`, \
         `int8[]`, `bool[]`, `float32[]`, `uint16[]`, …) and strings can be \
         viewed zero-copy. Sequences of strings or nested messages have no \
         fixed-width byte span; use `mode = \"heap\"` or `\"inline\"` for those \
         fields."
    )]
    UnsupportedViewElement {
        package: String,
        message: String,
        field: String,
        element: String,
    },

    /// phase-403 W7 — an `element_cap` that names a field with no element
    /// dimension.
    ///
    /// An ERROR and not a warning, and not silence, because the whole point of
    /// the key is to REPLACE a built-in 256 that nobody chose with a number
    /// somebody did. A key that quietly does nothing leaves the user believing
    /// they bounded a type that codegen still reports unbounded, and sends them
    /// looking at the derivation instead of at their config.
    #[error("codegen config: {details}")]
    ElementCapShape { details: String },

    /// phase-403 W7b (issue 0961) — the derived bound blew the type's stated
    /// `max_serialized` budget.
    ///
    /// A build error and not a warning: the budget is the user saying what
    /// their deployment can afford, and a type that does not fit will fail at
    /// run time as a dropped take with no explanation. Failing here, with the
    /// nesting chain and its factors, is the whole point of the key.
    #[error("{details}")]
    BoundExceedsBudget { details: String },
}

/// Resolve the borrowed-view field type + full `CdrReader` read expression for
/// a `borrowed`-mode field (RFC-0033, Phase 229.6, issue 0007).
///
/// Returns `(borrowed_rust_type, read_expr)`:
/// - **strings** → `&'a str` via `reader.read_string()?`,
/// - **single-byte sequences** (`uint8[]`/`byte[]`/`int8[]`/`bool[]`) → true
///   `&'a [u8]` slices (no alignment concern),
/// - **multi-byte numeric sequences** (`float32[]`, `uint16[]`, …) → an
///   alignment-agnostic `nros_core::LeSliceView<'a, T>` (the alignment guard):
///   borrows the raw LE bytes zero-copy, decodes per element on access, so the
///   receive buffer need not be `T`-aligned.
///
/// Sequences of strings or nested messages are rejected — their elements are
/// not fixed-width byte spans, so they cannot borrow zero-copy.
fn nros_borrowed_view_for_field(
    field_type: &FieldType,
    package_name: &str,
    message_name: &str,
    field_name: &str,
) -> Result<(String, String), GeneratorError> {
    let unsupported = |elem: &str| GeneratorError::UnsupportedViewElement {
        package: package_name.to_string(),
        message: message_name.to_string(),
        field: field_name.to_string(),
        element: elem.to_string(),
    };
    // Multi-byte numeric → alignment-agnostic `LeSliceView<'a, T>`.
    let le_view = |t: &str| {
        (
            format!("nros_core::LeSliceView<'a, {t}>"),
            format!("reader.read_le_slice::<{t}>()?"),
        )
    };
    match field_type {
        FieldType::String | FieldType::WString => {
            Ok(("&'a str".to_string(), "reader.read_string()?".to_string()))
        }
        FieldType::Sequence { element_type } => match element_type.as_ref() {
            FieldType::Primitive(PrimitiveType::UInt8 | PrimitiveType::Byte) => Ok((
                "&'a [u8]".to_string(),
                "reader.read_slice_u8()?".to_string(),
            )),
            FieldType::Primitive(PrimitiveType::Int8) => Ok((
                "&'a [u8]".to_string(),
                "reader.read_slice_i8()?".to_string(),
            )),
            FieldType::Primitive(PrimitiveType::Bool) => Ok((
                "&'a [u8]".to_string(),
                "reader.read_slice_bool()?".to_string(),
            )),
            FieldType::Primitive(PrimitiveType::UInt16) => Ok(le_view("u16")),
            FieldType::Primitive(PrimitiveType::Int16) => Ok(le_view("i16")),
            FieldType::Primitive(PrimitiveType::UInt32) => Ok(le_view("u32")),
            FieldType::Primitive(PrimitiveType::Int32) => Ok(le_view("i32")),
            FieldType::Primitive(PrimitiveType::UInt64) => Ok(le_view("u64")),
            FieldType::Primitive(PrimitiveType::Int64) => Ok(le_view("i64")),
            FieldType::Primitive(PrimitiveType::Float32) => Ok(le_view("f32")),
            FieldType::Primitive(PrimitiveType::Float64) => Ok(le_view("f64")),
            other => Err(unsupported(&format!("{other:?}"))),
        },
        other => Err(unsupported(&format!("{other:?}"))),
    }
}

/// Resolve the borrowed-view **C** field type + the `nros/view.h` reader
/// function for a `borrowed`-mode field (RFC-0033, Phase 235, issue 0021).
///
/// Returns `(borrowed_c_type, borrow_fn)` — both `nros/view.h` symbols. All
/// three readers share the signature
/// `int32_t fn(const uint8_t** ptr, const uint8_t* end, const uint8_t* origin, T* out)`,
/// so the C template calls them uniformly:
/// - **strings** → `nros_view_str_t` / `nros_cdr_borrow_string`,
/// - **single-byte sequences** (`uint8[]`/`byte[]`/`int8[]`/`bool[]`) →
///   `nros_view_bytes_t` / `nros_cdr_borrow_bytes`,
/// - **multi-byte numeric sequences** (`float32[]`, `uint16[]`, …) → the
///   alignment-agnostic `nros_le_slice_view_<t>_t` / `nros_cdr_borrow_le_slice_<t>`.
///
/// Sequences of strings or nested messages are rejected (no fixed-width byte
/// span) — same policy as the Rust generator.
fn c_borrowed_view_for_field(
    field_type: &FieldType,
    package_name: &str,
    message_name: &str,
    field_name: &str,
) -> Result<(String, String), GeneratorError> {
    let unsupported = |elem: &str| GeneratorError::UnsupportedViewElement {
        package: package_name.to_string(),
        message: message_name.to_string(),
        field: field_name.to_string(),
        element: elem.to_string(),
    };
    let le_view = |suffix: &str| {
        (
            format!("nros_le_slice_view_{suffix}_t"),
            format!("nros_cdr_borrow_le_slice_{suffix}"),
        )
    };
    let bytes = || {
        (
            "nros_view_bytes_t".to_string(),
            "nros_cdr_borrow_bytes".to_string(),
        )
    };
    match field_type {
        FieldType::String | FieldType::WString => Ok((
            "nros_view_str_t".to_string(),
            "nros_cdr_borrow_string".to_string(),
        )),
        FieldType::Sequence { element_type } => match element_type.as_ref() {
            FieldType::Primitive(
                PrimitiveType::UInt8
                | PrimitiveType::Byte
                | PrimitiveType::Int8
                | PrimitiveType::Bool,
            ) => Ok(bytes()),
            FieldType::Primitive(PrimitiveType::UInt16) => Ok(le_view("u16")),
            FieldType::Primitive(PrimitiveType::Int16) => Ok(le_view("i16")),
            FieldType::Primitive(PrimitiveType::UInt32) => Ok(le_view("u32")),
            FieldType::Primitive(PrimitiveType::Int32) => Ok(le_view("i32")),
            FieldType::Primitive(PrimitiveType::UInt64) => Ok(le_view("u64")),
            FieldType::Primitive(PrimitiveType::Int64) => Ok(le_view("i64")),
            FieldType::Primitive(PrimitiveType::Float32) => Ok(le_view("f32")),
            FieldType::Primitive(PrimitiveType::Float64) => Ok(le_view("f64")),
            other => Err(unsupported(&format!("{other:?}"))),
        },
        other => Err(unsupported(&format!("{other:?}"))),
    }
}

/// One string shape, as the idiomatic pack's `FieldKind` splits it: narrow or
/// wide, `.msg`-bounded or not.
///
/// The lowered IR deliberately models NONE of this — `FieldShape::Str` and
/// `CdrOp::String` are one variant each, because the C, C++, nros and rmw
/// surfaces all spell a string off `field_type` and none of them branches on
/// the flavour. Adding four variants to the shared IR for one pack's template
/// conditionals would put a fact in every surface's context that four of them
/// ignore, which is the inverse of what W2.5a is for. So this reads the IR's
/// `field_type` and stays a per-surface FILTER — the same shape as `c_type`.
#[derive(Clone, Copy)]
enum StringFlavour {
    Narrow,
    NarrowBounded,
    Wide,
    WideBounded,
}

fn string_flavour(ft: &FieldType) -> Option<StringFlavour> {
    match ft {
        FieldType::String => Some(StringFlavour::Narrow),
        FieldType::BoundedString(_) => Some(StringFlavour::NarrowBounded),
        FieldType::WString => Some(StringFlavour::Wide),
        FieldType::BoundedWString(_) => Some(StringFlavour::WideBounded),
        _ => None,
    }
}

/// The idiomatic Rust pack's exhaustive field classification, projected from
/// the lowered IR (phase-432 W2.5a).
///
/// The STRUCTURAL spine — scalar / string / array / sequence / nested, the
/// array length, and what the element is — comes from `LoweredField::shape` and
/// the element accessors, which is where the IR states those facts. Only the
/// two questions the IR does not model are answered off `field_type`: the
/// string flavour (see [`StringFlavour`]) and whether a sequence's bound came
/// from the `.msg` (`FieldShape::Sequence` covers both `T[]` and `T[<=N]`,
/// because storage is what every other surface asks about and both are
/// `LoweredStorage::Bounded`).
pub(crate) fn determine_field_kind(f: &rosidl_lower::LoweredField) -> FieldKind {
    use rosidl_lower::FieldShape;

    let element = element_type_of(&f.field_type);
    match f.shape {
        FieldShape::Scalar => FieldKind::Primitive,
        FieldShape::Nested => FieldKind::NestedMessage,

        FieldShape::Str => match string_flavour(&f.field_type) {
            Some(StringFlavour::Narrow) => FieldKind::UnboundedString,
            Some(StringFlavour::NarrowBounded) => FieldKind::BoundedString,
            Some(StringFlavour::Wide) => FieldKind::UnboundedWString,
            Some(StringFlavour::WideBounded) => FieldKind::BoundedWString,
            // Unreachable: `FieldShape::Str` is set by exactly those four arms.
            None => FieldKind::UnboundedString,
        },

        // Arrays > 32 elements don't impl Copy/Clone in Rust, so they are their
        // own kind whatever the element is.
        FieldShape::Array { len } if len > 32 => FieldKind::LargeArray,
        FieldShape::Array { .. } => {
            if f.element_is_primitive() {
                FieldKind::PrimitiveArray
            } else {
                match element.and_then(string_flavour) {
                    Some(StringFlavour::Narrow) => FieldKind::UnboundedStringArray,
                    Some(StringFlavour::NarrowBounded) => FieldKind::BoundedStringArray,
                    Some(StringFlavour::Wide) => FieldKind::UnboundedWStringArray,
                    Some(StringFlavour::WideBounded) => FieldKind::BoundedWStringArray,
                    None => FieldKind::NestedMessageArray,
                }
            }
        }

        FieldShape::Sequence => {
            // `T[<=N]` vs `T[]` — the one distinction `FieldShape::Sequence`
            // does not carry.
            let msg_bounded = matches!(f.field_type, FieldType::BoundedSequence { .. });
            if f.element_is_primitive() {
                return if msg_bounded {
                    FieldKind::BoundedPrimitiveSequence
                } else {
                    FieldKind::UnboundedPrimitiveSequence
                };
            }
            match (msg_bounded, element.and_then(string_flavour)) {
                (true, Some(StringFlavour::Narrow)) => FieldKind::BoundedUnboundedStringSequence,
                (true, Some(StringFlavour::NarrowBounded)) => {
                    FieldKind::BoundedBoundedStringSequence
                }
                (true, Some(StringFlavour::Wide)) => FieldKind::BoundedUnboundedWStringSequence,
                (true, Some(StringFlavour::WideBounded)) => {
                    FieldKind::BoundedBoundedWStringSequence
                }
                (true, None) => FieldKind::BoundedNestedMessageSequence,
                (false, Some(StringFlavour::Narrow)) => FieldKind::UnboundedUnboundedStringSequence,
                (false, Some(StringFlavour::NarrowBounded)) => {
                    FieldKind::UnboundedBoundedStringSequence
                }
                (false, Some(StringFlavour::Wide)) => FieldKind::UnboundedUnboundedWStringSequence,
                (false, Some(StringFlavour::WideBounded)) => {
                    FieldKind::UnboundedBoundedWStringSequence
                }
                (false, None) => FieldKind::UnboundedNestedMessageSequence,
            }
        }
    }
}

/// The element of an array or sequence, or `None` for a non-container.
fn element_type_of(ft: &FieldType) -> Option<&FieldType> {
    match ft {
        FieldType::Array { element_type, .. }
        | FieldType::Sequence { element_type }
        | FieldType::BoundedSequence { element_type, .. } => Some(element_type.as_ref()),
        _ => None,
    }
}

/// The Rust CDR method suffix for a lowered scalar op — the nros pack's
/// `reader.read_<x>()` / `writer.write_<x>()` (phase-432 W2.5a).
///
/// A per-language SPELLING of a neutral fact, which is the shape W2.5b names as
/// a language's whole Rust surface area. It replaces `primitive_to_cdr_method`
/// at the nros builder, and the two agree by construction: `CdrOp` is exactly
/// as coarse as this mapping — `Byte`, `Char` and `UInt8` all lower to
/// `CdrOp::U8` and all three spelled `"u8"`.
///
/// `String` and `Nested` are not scalars and reach this only through a caller
/// that ignored `scalar_op` / `element_is_primitive`; they map to the empty
/// string, which renders as no method rather than a wrong one.
pub(super) fn cdr_op_rust_method(op: rosidl_lower::CdrOp) -> String {
    use rosidl_lower::CdrOp;
    match op {
        CdrOp::Bool => "bool",
        CdrOp::U8 => "u8",
        CdrOp::I8 => "i8",
        CdrOp::U16 => "u16",
        CdrOp::I16 => "i16",
        CdrOp::U32 => "u32",
        CdrOp::I32 => "i32",
        CdrOp::U64 => "u64",
        CdrOp::I64 => "i64",
        CdrOp::F32 => "f32",
        CdrOp::F64 => "f64",
        CdrOp::String | CdrOp::Nested => "",
    }
    .to_string()
}

/// Storage-mode policy for a service/action payload struct.
///
/// Issue 0343 discovered that RFC-0033 modes were resolved for srv/action but
/// implemented only in the MESSAGE templates, so a heap-configured `.srv` field
/// got the heap TYPE with an owned serde body — code that does not compile.
/// Issue 0344 then implemented what is implementable, which is not everything:
///
/// | mode | Rust srv/action | C srv/action | C++ srv/action |
/// | --- | --- | --- | --- |
/// | `owned` | yes | yes | yes |
/// | `heap` | **yes** (0344) | **yes** (0345) | n/a (FFI-delegated) |
/// | `borrowed` | rejected — see below | rejected | n/a |
///
/// **C heap (0345).** The premise 0343/0344 recorded — that supporting heap in C
/// would need every consumer taught to free — was wrong: `nros_service_callback_t`
/// hands the callback `request_data`/`request_len`, so nros-c never builds a typed
/// payload struct at all. The CALLER declares it and calls `_deserialize`, exactly
/// as for messages, so the caller `_fini`s it — the RFC-0033 contract, unchanged.
/// The fix was therefore the shared arms (`_c_field.jinja`) plus a generated
/// `_fini` per payload struct, with no framework or ownership change.
///
/// **Why `borrowed` stays rejected everywhere.** `borrowed` works by emitting a
/// `{Msg}View<'a>` / `{Msg}_View` alongside the owned struct; the srv/action
/// templates emit no view type. The mode would therefore silently degrade to
/// `owned` — the field builder keeps the owned container for the publish path —
/// which is a wrong answer rather than an error.
///
/// C++ needs no policy here: its templates delegate serialization across the FFI
/// to the Rust core, so the container type is all that changes.
pub(super) fn ensure_supported_storage_for_payload(
    entity: &str,
    lang: PayloadLang,
    package_name: &str,
    message_name: &str,
    fields: &[rosidl_parser::Field],
    resolver: &CapacityResolver,
) -> Result<(), GeneratorError> {
    for field in fields {
        let Some(kind) = (match &field.field_type {
            FieldType::String | FieldType::WString => Some(CapFieldKind::String),
            FieldType::Sequence { .. } => Some(CapFieldKind::Sequence),
            _ => None,
        }) else {
            continue;
        };
        let storage = resolver.resolve(package_name, message_name, &field.name, kind);
        let rejected = match (storage.mode, lang) {
            (StorageMode::Inline, _) => false,
            // Rust gained heap in 0344 (`_nros_field.jinja`); C gained it in
            // 0345 (`_c_field.jinja` + a generated `_fini` per payload struct).
            (StorageMode::Heap, _) => false,
            // Issue 0346 — srv/action payloads now emit `{Payload}View` /
            // `{Payload}_View` + `deserialize_view`, so borrowed is supported
            // in both languages. Both directions read from a raw buffer (the
            // service callback's `request_data`, the client's `response`), so the
            // view's lifetime story matches a subscription's.
            (StorageMode::View, _) => false,
        };
        if rejected {
            return Err(GeneratorError::UnsupportedStorageModeForPayload {
                entity: entity.to_string(),
                package: package_name.to_string(),
                message: message_name.to_string(),
                field: field.name.clone(),
                mode: storage.mode.as_str(),
            });
        }
    }
    Ok(())
}

/// Which emitter is asking — see [`ensure_supported_storage_for_payload`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PayloadLang {
    Rust,
    C,
}

/// Convert a Message field to NrosField with explicit codegen mode.
///
/// `resolver` supplies the per-field capacity for **unbounded** sequence/string
/// fields (RFC-0033). Bounded fields, arrays, primitives, and nested types are
/// unaffected. A non-`owned` storage mode is rejected in Phase 229 (`heap` and
/// `borrowed` land in 229.5 / 229.6).
/// Build every C field of `msg`, sourcing each field's storage from the lowered
/// IR once (phase-335 W1.c) — no field builder resolves capacity.
pub(super) fn build_c_fields(
    current_package: Option<&str>,
    message: &str,
    msg: &rosidl_parser::Message,
    resolver: &CapacityResolver,
) -> Result<Vec<CField>, GeneratorError> {
    let store = lowered_storages(
        current_package.unwrap_or(""),
        message,
        &msg.fields,
        resolver,
    );
    let package = current_package.unwrap_or("");
    ensure_element_caps_apply(package, message, &msg.fields, resolver)?;
    msg.fields
        .iter()
        .zip(store.iter())
        .map(|(f, s)| {
            build_c_field(
                &f.name,
                &f.field_type,
                current_package,
                message,
                resolver,
                Some(*s),
            )
        })
        .collect()
}

/// phase-403 W7 — reject a `[fields]` `element_cap` that names a field with no
/// element dimension, rather than ignoring it.
///
/// Called from each language's field funnel (`build_c_fields`,
/// `build_nros_fields`, and the C++ builder's loop) because those are the three
/// places a message's field TYPES and the resolver are in scope at once. Every
/// generator entry point reaches one of them, so a misdirected key cannot be
/// generated past in any language.
pub(super) fn ensure_element_caps_apply(
    package: &str,
    message: &str,
    fields: &[rosidl_parser::Field],
    resolver: &CapacityResolver,
) -> Result<(), GeneratorError> {
    let offenders = resolver.element_cap_shape_errors(package, message, fields);
    if offenders.is_empty() {
        return Ok(());
    }
    // Every offender, for the reason `TypeBound::Unbounded` names every
    // unbounded member: fixing a config one build at a time is the loop W0
    // removed.
    Err(GeneratorError::ElementCapShape {
        details: offenders
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join("; "),
    })
}

/// Build one nros field from the lowered IR (phase-432 W2.5a).
///
/// Every fact below was re-derived here from `rosidl_parser` before this wave:
/// the element-cap fold, the two configurable shapes, the storage mode flags,
/// the five shape predicates, the array length, the element classification and
/// both CDR method names. All of them are now READ off `LoweredField`, so the
/// nros surface and the C surface cannot answer the same question differently.
///
/// What is left is the two things a language owes: the identifier SPELLING
/// (`escape_keyword`) and the CDR method NAMES (`cdr_op_rust_method`), which is
/// a `CdrOp` -> Rust suffix mapping and belongs with the other per-language
/// spellings (W2.5b).
pub(super) fn field_to_nros_field_with_mode(
    field: &rosidl_lower::LoweredField,
    package_name: &str,
    message_name: &str,
    mode: NrosCodegenMode,
) -> Result<NrosField, GeneratorError> {
    // phase-403 W7 — the container spells `heapless::Vec<heapless::String<32>, N>`
    // so the element's `String::try_from` ENFORCES the bound the derived number
    // claims. The fold is `LoweredField::storage_type`; it used to be an
    // `element_capped` call this builder made for itself.
    let capped = field.storage_type();
    let field_type: &FieldType = capped.as_ref();

    let mut borrowed_rust_type = String::new();
    let mut borrowed_read_expr = String::new();
    // RFC-0033 `borrowed` (Phase 229.6, issue 0007): the owned `{Msg}` struct
    // keeps a default-capacity owned container for the publish path; the
    // additionally-emitted `{Msg}View<'a>` borrows this field zero-copy.
    if field.is_borrowed() {
        let (bt, expr) =
            nros_borrowed_view_for_field(field_type, package_name, message_name, &field.name)?;
        borrowed_rust_type = bt;
        borrowed_read_expr = expr;
    }

    let array_size = field.array_len();

    Ok(NrosField {
        name: escape_keyword(&field.name),
        field_type: field_type.clone(),
        is_configurable: field.configurable,
        cap: field.configured_cap(),
        mode,
        current_package: package_name.to_string(),
        primitive_method: field
            .scalar_op()
            .map(cdr_op_rust_method)
            .unwrap_or_default(),
        element_primitive_method: field
            .element_op()
            .filter(|_| field.element_is_primitive())
            .map(cdr_op_rust_method)
            .unwrap_or_default(),
        array_size,
        is_primitive: field.is_primitive(),
        is_string: field.is_string(),
        is_array: field.is_array(),
        is_sequence: field.is_sequence(),
        is_nested: field.is_nested(),
        is_primitive_element: field.element_is_primitive(),
        is_string_element: field.element_is_string(),
        is_large_array: array_size > 32,
        is_heap: field.is_heap(),
        is_borrowed: field.is_borrowed(),
        borrowed_rust_type,
        borrowed_read_expr,
    })
}

/// Convert a Message field to NrosField (crate mode).
/// Per-field storage decisions for `msg`, read from the lowered IR (phase-335
/// W1.c). A generator computes this once and hands each builder the matching
/// entry (`Some(store[i])`) so no builder resolves capacity a second time.
/// `lower_fields` calls the same `CapacityResolver` with the same keys, so the
/// result is byte-identical to an inline `resolver.resolve`.
pub(super) fn lowered_storages(
    package: &str,
    message: &str,
    fields: &[rosidl_parser::Field],
    resolver: &CapacityResolver,
) -> Vec<FieldStorage> {
    rosidl_lower::lower_fields(package, message, fields, resolver)
        .iter()
        .map(|lf| lf.storage.as_field_storage())
        .collect()
}

/// The lowered IR for a surface that mirrors ROS's OWN runtime types — the
/// `rmw` and idiomatic Rust packs (phase-432 W2.5a).
///
/// Those two layers spell `rosidl_runtime_rs::Sequence<T>` / `std::vec::Vec<T>`
/// and `String`: containers with no fixed capacity, chosen by ROS rather than by
/// us. Nothing in a nano-ros storage config can change them, which is why
/// `generate_message_package` and its siblings take no [`CapacityResolver`] —
/// so the empty resolver is not a stand-in here, it is the right answer, and
/// naming it once keeps the three entry points from each deciding that
/// separately.
pub(super) fn lowered_ros_abi_fields(
    package: &str,
    message: &str,
    fields: &[rosidl_parser::Field],
) -> Vec<rosidl_lower::LoweredField> {
    rosidl_lower::lower_fields(package, message, fields, &CapacityResolver::empty())
}

/// Build every `rmw` field of a message, projected from the lowered IR
/// (phase-432 W2.5a).
///
/// The `rmw` surface is the thinnest of the five: a name, the type AS PARSED
/// (its containers are ROS's, so no storage decision applies) and the `.msg`
/// default. All three are IR facts, and the only Rust here is the two SPELLINGS
/// — `escape_keyword` and `constant_value_to_rust`.
///
/// It replaces three byte-identical closures in `msg.rs`, `srv.rs` and
/// `action.rs`, each mapping over `rosidl_parser`'s fields. The projection is
/// one fact per line; three copies of it was three places for a fourth
/// surface's habit to be copied from.
pub(super) fn build_rmw_fields(
    package_name: &str,
    message_name: &str,
    msg: &rosidl_parser::Message,
) -> Vec<RmwField> {
    lowered_ros_abi_fields(package_name, message_name, &msg.fields)
        .iter()
        .map(|f| RmwField {
            name: escape_keyword(&f.name),
            field_type: f.field_type.clone(),
            current_package: package_name.to_string(),
            default_value: f
                .default_value
                .as_ref()
                .map(constant_value_to_rust)
                .unwrap_or_default(),
        })
        .collect()
}

/// Build every idiomatic-Rust field of a message, projected from the lowered IR
/// (phase-432 W2.5a).
///
/// The `rmw` surface plus one classification: `determine_field_kind`, which the
/// pack branches on. Like `build_rmw_fields` it replaces three byte-identical
/// closures that each mapped over `rosidl_parser`'s fields.
pub(super) fn build_idiomatic_fields(
    package_name: &str,
    message_name: &str,
    msg: &rosidl_parser::Message,
) -> Vec<IdiomaticField> {
    lowered_ros_abi_fields(package_name, message_name, &msg.fields)
        .iter()
        .map(|f| IdiomaticField {
            name: escape_keyword(&f.name),
            field_type: f.field_type.clone(),
            current_package: package_name.to_string(),
            default_value: f
                .default_value
                .as_ref()
                .map(constant_value_to_rust)
                .unwrap_or_default(),
            kind: determine_field_kind(f),
        })
        .collect()
}

/// Build every nros field of `msg` from the lowered IR (phase-432 W2.5a).
///
/// phase-335 W1.c already sourced the STORAGE decision here; the whole field is
/// sourced here now, so nothing downstream re-reads `rosidl_parser`.
pub(super) fn build_nros_fields(
    package: &str,
    message: &str,
    msg: &rosidl_parser::Message,
    resolver: &CapacityResolver,
    mode: NrosCodegenMode,
) -> Result<Vec<NrosField>, GeneratorError> {
    let lowered = rosidl_lower::lower_fields(package, message, &msg.fields, resolver);
    ensure_element_caps_apply(package, message, &msg.fields, resolver)?;
    lowered
        .iter()
        .map(|f| field_to_nros_field_with_mode(f, package, message, mode))
        .collect()
}

/// Build a CField from a field type.
///
/// `resolver` supplies the per-field capacity for **unbounded** sequence/string
/// fields (RFC-0033). A non-`owned` storage mode is rejected in Phase 229.
pub(super) fn build_c_field(
    name: &str,
    field_type: &FieldType,
    current_package: Option<&str>,
    message_name: &str,
    resolver: &CapacityResolver,
    // phase-335 W1.c — storage from the IR when the caller already lowered it.
    pre_storage: Option<FieldStorage>,
) -> Result<CField, GeneratorError> {
    let escaped_name = escape_keyword(name);

    // phase-403 W7 — fold any configured ELEMENT bound into the shape before
    // anything reads it, so every branch below sees the `string<=N[]` the `.msg`
    // could have spelled. `char name[16][32]` and its `sizeof`-bounded
    // `nros_cdr_read_string` then fall out of the existing bounded-element arms.
    let capped = resolver.element_capped(
        current_package.unwrap_or(""),
        message_name,
        name,
        field_type,
    );
    let field_type: &FieldType = capped.as_ref();

    // Resolve per-field capacity for the two configurable shapes.
    let cap_kind = match field_type {
        FieldType::String | FieldType::WString => Some(CapFieldKind::String),
        FieldType::Sequence { .. } => Some(CapFieldKind::Sequence),
        _ => None,
    };
    let unsupported = |mode: &'static str| GeneratorError::UnsupportedStorageMode {
        package: current_package.unwrap_or("").to_string(),
        message: message_name.to_string(),
        field: name.to_string(),
        mode,
    };
    // (c_type, array_suffix, is_heap, resolved owned sequence capacity).
    let mut is_heap = false;
    let mut is_borrowed = false;
    let mut borrowed_c_type = String::new();
    let mut borrowed_read_fn = String::new();
    let mut owned_seq_cap: Option<usize> = None;
    // phase-335 step 2 — resolve storage for the FLAGS + the capacity the
    // `c_type` / `c_array_suffix` pack filters need; the type STRING is composed
    // in the pack from `field_type` + these facts, not pre-baked here.
    let is_configurable = cap_kind.is_some();
    let mut cap: usize = 0;
    if let Some(kind) = cap_kind {
        let package = current_package.unwrap_or("");
        let storage =
            pre_storage.unwrap_or_else(|| resolver.resolve(package, message_name, name, kind));
        cap = storage.cap;
        match storage.mode {
            StorageMode::Inline => {
                if matches!(field_type, FieldType::Sequence { .. }) {
                    owned_seq_cap = Some(storage.cap);
                }
            }
            StorageMode::Heap => {
                // Heap is only bridgeable for shapes `c_type_for_field_heap`
                // supports — validate here (the filter assumes it holds), reject
                // otherwise, preserving the pre-step-2 error behaviour.
                if c_type_for_field_heap(field_type, current_package).is_none() {
                    return Err(unsupported("heap"));
                }
                is_heap = true;
            }
            // RFC-0033 `borrowed` (Phase 235, issue 0021): the owned `{Msg}`
            // struct keeps its resolved-capacity container for the publish path;
            // the emitted `{Msg}_View` borrows this field zero-copy (see
            // `borrowed_c_type` / `borrowed_read_fn`).
            StorageMode::View => {
                is_borrowed = true;
                let (bt, bfn) = c_borrowed_view_for_field(field_type, package, message_name, name)?;
                borrowed_c_type = bt;
                borrowed_read_fn = bfn;
                if matches!(field_type, FieldType::Sequence { .. }) {
                    owned_seq_cap = Some(storage.cap);
                }
            }
        }
    }

    // Determine type characteristics
    let (is_primitive, primitive_type) = match field_type {
        FieldType::Primitive(prim) => (true, Some(prim)),
        _ => (false, None),
    };

    let is_string = matches!(
        field_type,
        FieldType::String
            | FieldType::BoundedString(_)
            | FieldType::WString
            | FieldType::BoundedWString(_)
    );

    let is_array = matches!(field_type, FieldType::Array { .. });
    let is_sequence = matches!(
        field_type,
        FieldType::Sequence { .. } | FieldType::BoundedSequence { .. }
    );
    let is_nested = matches!(field_type, FieldType::NamespacedType { .. });

    // Get array/sequence info. Owned unbounded sequences use the resolved
    // capacity; heap sequences are unbounded (capacity unused → 0).
    let (array_size, sequence_capacity) = match field_type {
        FieldType::Array { size, .. } => (*size, 0),
        FieldType::Sequence { .. } => (0, owned_seq_cap.unwrap_or(C_DEFAULT_SEQUENCE_CAPACITY)),
        FieldType::BoundedSequence { max_size, .. } => (0, *max_size),
        _ => (0, 0),
    };

    // Get element info for arrays/sequences
    let (is_primitive_element, is_string_element, element_type) = match field_type {
        FieldType::Array { element_type, .. }
        | FieldType::Sequence { element_type }
        | FieldType::BoundedSequence { element_type, .. } => {
            let is_prim = matches!(element_type.as_ref(), FieldType::Primitive(_));
            let is_str = matches!(
                element_type.as_ref(),
                FieldType::String
                    | FieldType::BoundedString(_)
                    | FieldType::WString
                    | FieldType::BoundedWString(_)
            );
            (is_prim, is_str, Some(element_type.as_ref()))
        }
        _ => (false, false, None),
    };

    // Get CDR methods
    let (cdr_write_method, cdr_read_method) = if let Some(prim) = primitive_type {
        (
            c_cdr_write_method(prim).to_string(),
            c_cdr_read_method(prim).to_string(),
        )
    } else {
        (String::new(), String::new())
    };

    let (element_cdr_write_method, element_cdr_read_method) =
        if let Some(FieldType::Primitive(prim)) = element_type {
            (
                c_cdr_write_method(prim).to_string(),
                c_cdr_read_method(prim).to_string(),
            )
        } else {
            (String::new(), String::new())
        };

    // Get nested struct names (use current_package for intra-package references)
    let nested_struct_name = if let FieldType::NamespacedType { package, name } = field_type {
        let pkg = package.as_deref().or(current_package).unwrap_or("");
        format!("{}_msg_{}", to_c_package_name(pkg), to_snake_case(name))
    } else {
        String::new()
    };

    let element_struct_name =
        if let Some(FieldType::NamespacedType { package, name }) = element_type {
            let pkg = package.as_deref().or(current_package).unwrap_or("");
            format!("{}_msg_{}", to_c_package_name(pkg), to_snake_case(name))
        } else {
            String::new()
        };

    Ok(CField {
        name: escaped_name,
        field_type: field_type.clone(),
        is_configurable,
        cap,
        current_package: current_package.unwrap_or("").to_string(),
        cdr_write_method,
        cdr_read_method,
        element_cdr_write_method,
        element_cdr_read_method,
        array_size,
        sequence_capacity,
        nested_struct_name,
        element_struct_name,
        is_primitive,
        is_string,
        is_array,
        is_sequence,
        is_nested,
        is_primitive_element,
        is_string_element,
        is_heap,
        is_borrowed,
        borrowed_c_type,
        borrowed_read_fn,
    })
}

/// Resolved C++ storage for a field (RFC-0033). `Owned(cap)` keeps a
/// fixed-capacity container (`cap` = `Some` only for the configurable
/// string/sequence shapes); `Heap` is an `nros::HeapSequence<T>` primitive
/// sequence.
#[derive(Clone, Copy)]
pub(super) enum CppStorage {
    Owned(Option<usize>),
    Heap,
    /// RFC-0033 `borrowed` (Phase 235): a zero-copy view into the CDR buffer.
    /// Carries the resolved owned capacity too — the owned `{Msg}` struct keeps
    /// a fixed-capacity container for the publish path; only `{Msg}View` borrows.
    Borrowed(CppView, Option<usize>),
}

/// The borrowed C++ view kind for a `mode = "view"` field (Phase 235).
/// Carries `'static` strings so `CppStorage` stays `Copy`.
#[derive(Clone, Copy)]
pub(super) enum CppView {
    /// `nros::StringView` via `CdrReader::read_string`.
    Str,
    /// `nros::Span<cpp>` via `reader` (`read_slice_u8`/`_i8`/`_bool`).
    Bytes {
        cpp: &'static str,
        reader: &'static str,
    },
    /// `nros::LeSpan<cpp>` via `read_le_slice::<suffix>` (alignment-agnostic).
    Le {
        cpp: &'static str,
        suffix: &'static str,
    },
}

impl CppView {
    /// The C++ view type for the header (`{Msg}View`) field.
    pub(super) fn cpp_view_type(self) -> String {
        match self {
            CppView::Str => "nros::StringView".to_string(),
            CppView::Bytes { cpp, .. } => format!("nros::Span<{cpp}>"),
            CppView::Le { cpp, .. } => format!("nros::LeSpan<{cpp}>"),
        }
    }
    /// The `CdrReader` call (no `?`) that borrows this field's bytes.
    pub(super) fn reader_call(self) -> String {
        match self {
            CppView::Str => "read_string()".to_string(),
            CppView::Bytes { reader, .. } => format!("{reader}()"),
            CppView::Le { suffix, .. } => format!("read_le_slice::<{suffix}>()"),
        }
    }
    /// `true` for the `LeSpan` case (the FFI extracts `.as_bytes().as_ptr()`
    /// instead of `.as_ptr()`).
    pub(super) fn is_le(self) -> bool {
        matches!(self, CppView::Le { .. })
    }
}

/// Resolve the borrowed C++ view kind for a `borrowed`-mode field. Strings and
/// byte/numeric sequences borrow; sequences of strings or nested messages are
/// rejected (no fixed-width byte span) — same policy as Rust and C.
fn cpp_borrow_kind(
    field_type: &FieldType,
    package_name: &str,
    message_name: &str,
    field_name: &str,
) -> Result<CppView, GeneratorError> {
    let unsupported = |elem: &str| GeneratorError::UnsupportedViewElement {
        package: package_name.to_string(),
        message: message_name.to_string(),
        field: field_name.to_string(),
        element: elem.to_string(),
    };
    match field_type {
        FieldType::String | FieldType::WString => Ok(CppView::Str),
        FieldType::Sequence { element_type } => match element_type.as_ref() {
            FieldType::Primitive(PrimitiveType::UInt8 | PrimitiveType::Byte) => {
                Ok(CppView::Bytes {
                    cpp: "uint8_t",
                    reader: "read_slice_u8",
                })
            }
            FieldType::Primitive(PrimitiveType::Int8) => Ok(CppView::Bytes {
                cpp: "int8_t",
                reader: "read_slice_i8",
            }),
            FieldType::Primitive(PrimitiveType::Bool) => Ok(CppView::Bytes {
                cpp: "uint8_t",
                reader: "read_slice_bool",
            }),
            FieldType::Primitive(PrimitiveType::UInt16) => Ok(CppView::Le {
                cpp: "uint16_t",
                suffix: "u16",
            }),
            FieldType::Primitive(PrimitiveType::Int16) => Ok(CppView::Le {
                cpp: "int16_t",
                suffix: "i16",
            }),
            FieldType::Primitive(PrimitiveType::UInt32) => Ok(CppView::Le {
                cpp: "uint32_t",
                suffix: "u32",
            }),
            FieldType::Primitive(PrimitiveType::Int32) => Ok(CppView::Le {
                cpp: "int32_t",
                suffix: "i32",
            }),
            FieldType::Primitive(PrimitiveType::UInt64) => Ok(CppView::Le {
                cpp: "uint64_t",
                suffix: "u64",
            }),
            FieldType::Primitive(PrimitiveType::Int64) => Ok(CppView::Le {
                cpp: "int64_t",
                suffix: "i64",
            }),
            FieldType::Primitive(PrimitiveType::Float32) => Ok(CppView::Le {
                cpp: "float",
                suffix: "f32",
            }),
            FieldType::Primitive(PrimitiveType::Float64) => Ok(CppView::Le {
                cpp: "double",
                suffix: "f64",
            }),
            other => Err(unsupported(&format!("{other:?}"))),
        },
        other => Err(unsupported(&format!("{other:?}"))),
    }
}

/// Resolve the per-field storage for a C++ field (RFC-0033). Owned shapes carry
/// their resolved capacity; `heap` is allowed only for primitive sequences
/// (rejected for heap strings / sequences of strings / nested messages, and for
/// `borrowed`). Shared by the C++ header + FFI builders so both agree.
pub(super) fn resolve_cap_override(
    name: &str,
    field_type: &FieldType,
    current_package: Option<&str>,
    message_name: &str,
    resolver: &CapacityResolver,
    // phase-335 W1.c — storage from the IR when the caller already lowered it.
    pre_storage: Option<FieldStorage>,
) -> Result<CppStorage, GeneratorError> {
    let kind = match field_type {
        FieldType::String | FieldType::WString => CapFieldKind::String,
        FieldType::Sequence { .. } => CapFieldKind::Sequence,
        _ => return Ok(CppStorage::Owned(None)),
    };
    let package = current_package.unwrap_or("");
    let unsupported = |mode: &'static str| GeneratorError::UnsupportedStorageMode {
        package: package.to_string(),
        message: message_name.to_string(),
        field: name.to_string(),
        mode,
    };
    let storage =
        pre_storage.unwrap_or_else(|| resolver.resolve(package, message_name, name, kind));
    match storage.mode {
        StorageMode::Inline => Ok(CppStorage::Owned(Some(storage.cap))),
        StorageMode::Heap => {
            // Heap is only bridgeable for primitive sequences (see
            // cpp_type_for_field_heap); reject heap strings / non-primitive seqs.
            if cpp_type_for_field_heap(field_type, current_package).is_some() {
                Ok(CppStorage::Heap)
            } else {
                Err(unsupported("heap"))
            }
        }
        StorageMode::View => Ok(CppStorage::Borrowed(
            cpp_borrow_kind(field_type, package, message_name, name)?,
            Some(storage.cap),
        )),
    }
}

/// Build a CppField for C++ header generation.
///
/// `storage` is the resolved per-field storage (RFC-0033): an owned
/// fixed-capacity container or an `nros::HeapSequence<T>` heap sequence.
pub(super) fn build_cpp_field(
    name: &str,
    field_type: &FieldType,
    current_package: Option<&str>,
    storage: CppStorage,
) -> CppField {
    let escaped_name = escape_keyword(name);
    // phase-335 step 2 — CppField carries the NEUTRAL facts; the `cpp_type` /
    // `cpp_array_suffix` pack filters compose the C++ type string. The borrowed
    // VIEW type (`nros::StringView` / `Span<T>` / `LeSpan<T>`) stays computed here
    // (it derives from the CppViewKind, not `field_type`).
    let cp = current_package.unwrap_or("").to_string();
    match storage {
        CppStorage::Borrowed(b, cap) => CppField {
            name: escaped_name,
            field_type: field_type.clone(),
            is_heap: false,
            cap,
            current_package: cp,
            is_borrowed: true,
            borrowed_cpp_type: b.cpp_view_type(),
        },
        CppStorage::Owned(cap) => CppField {
            name: escaped_name,
            field_type: field_type.clone(),
            is_heap: false,
            cap,
            current_package: cp,
            is_borrowed: false,
            borrowed_cpp_type: String::new(),
        },
        CppStorage::Heap => CppField {
            name: escaped_name,
            field_type: field_type.clone(),
            is_heap: true,
            cap: None,
            current_package: cp,
            is_borrowed: false,
            borrowed_cpp_type: String::new(),
        },
    }
}

/// Build a CppFfiField and optional SequenceStructDef for Rust FFI glue generation
pub(super) fn build_cpp_ffi_field(
    name: &str,
    field_type: &FieldType,
    struct_name: &str,
    current_package: Option<&str>,
    storage: CppStorage,
) -> (CppFfiField, Option<SequenceStructDef>) {
    let escaped_name = escape_keyword(name);
    let is_heap = matches!(storage, CppStorage::Heap);
    let cap_override = match storage {
        CppStorage::Owned(cap) => cap,
        CppStorage::Heap => None,
        CppStorage::Borrowed(_, cap) => cap,
    };
    // RFC-0033 borrowed (Phase 235): the `{Msg}ViewRepr` FFI struct stores this
    // field as `nros_cpp_borrow_t { *const u8, usize }`, filled from the
    // `CdrReader` borrow method; the owned `{Msg}` repr keeps its owned type.
    let (is_borrowed, borrowed_reader_call, borrowed_le) = match storage {
        CppStorage::Borrowed(b, _) => (true, b.reader_call(), b.is_le()),
        _ => (false, String::new(), false),
    };

    // Determine type characteristics
    let (is_primitive, primitive_type) = match field_type {
        FieldType::Primitive(prim) => (true, Some(prim)),
        _ => (false, None),
    };

    let is_string = matches!(
        field_type,
        FieldType::String
            | FieldType::BoundedString(_)
            | FieldType::WString
            | FieldType::BoundedWString(_)
    );

    let is_array = matches!(field_type, FieldType::Array { .. });
    let is_sequence = matches!(
        field_type,
        FieldType::Sequence { .. } | FieldType::BoundedSequence { .. }
    );
    let is_nested = matches!(field_type, FieldType::NamespacedType { .. });

    // Array/sequence size info. Unbounded sequences use the resolved capacity.
    let (array_size, sequence_capacity) = match field_type {
        FieldType::Array { size, .. } => (*size, 0),
        FieldType::Sequence { .. } => (0, cap_override.unwrap_or(CPP_DEFAULT_SEQUENCE_CAPACITY)),
        FieldType::BoundedSequence { max_size, .. } => (0, *max_size),
        _ => (0, 0),
    };

    // Element type info
    let (is_primitive_element, is_string_element, element_type) = match field_type {
        FieldType::Array { element_type, .. }
        | FieldType::Sequence { element_type }
        | FieldType::BoundedSequence { element_type, .. } => {
            let is_prim = matches!(element_type.as_ref(), FieldType::Primitive(_));
            let is_str = matches!(
                element_type.as_ref(),
                FieldType::String
                    | FieldType::BoundedString(_)
                    | FieldType::WString
                    | FieldType::BoundedWString(_)
            );
            (is_prim, is_str, Some(element_type.as_ref()))
        }
        _ => (false, false, None),
    };

    // CDR methods for primitives
    let (cdr_write_method, cdr_read_method) = if let Some(prim) = primitive_type {
        (
            c_cdr_write_method(prim).to_string(),
            c_cdr_read_method(prim).to_string(),
        )
    } else {
        (String::new(), String::new())
    };

    let (element_cdr_write_method, element_cdr_read_method) =
        if let Some(FieldType::Primitive(prim)) = element_type {
            (
                c_cdr_write_method(prim).to_string(),
                c_cdr_read_method(prim).to_string(),
            )
        } else {
            (String::new(), String::new())
        };

    // Nested function names
    let nested_serialize_fn = if let FieldType::NamespacedType { package, name: n } = field_type {
        let pkg = package.as_deref().or(current_package).unwrap_or("unknown");
        format!(
            "serialize_{}_msg_{}_fields",
            to_c_package_name(pkg),
            to_snake_case(n)
        )
    } else {
        String::new()
    };

    let nested_deserialize_fn = if let FieldType::NamespacedType { package, name: n } = field_type {
        let pkg = package.as_deref().or(current_package).unwrap_or("unknown");
        format!(
            "deserialize_{}_msg_{}_fields",
            to_c_package_name(pkg),
            to_snake_case(n)
        )
    } else {
        String::new()
    };

    // issue #201 — recursive heap teardown (the Rust-glue analog of the C
    // `_fini` recursion): frees + nulls a message's heap allocations so the
    // deserializers' error paths can tear down locally-allocated element
    // buffers whose nested elements own heap memory.
    let nested_teardown_fn = if let FieldType::NamespacedType { package, name: n } = field_type {
        let pkg = package.as_deref().or(current_package).unwrap_or("unknown");
        format!(
            "teardown_{}_msg_{}_fields",
            to_c_package_name(pkg),
            to_snake_case(n)
        )
    } else {
        String::new()
    };

    // Element nested function names (for arrays/sequences of nested types)
    let (elem_nested_ser, elem_nested_deser, elem_nested_teardown) =
        if let Some(FieldType::NamespacedType { package, name: n }) = element_type {
            let pkg = package.as_deref().or(current_package).unwrap_or("unknown");
            (
                format!(
                    "serialize_{}_msg_{}_fields",
                    to_c_package_name(pkg),
                    to_snake_case(n)
                ),
                format!(
                    "deserialize_{}_msg_{}_fields",
                    to_c_package_name(pkg),
                    to_snake_case(n)
                ),
                format!(
                    "teardown_{}_msg_{}_fields",
                    to_c_package_name(pkg),
                    to_snake_case(n)
                ),
            )
        } else {
            (String::new(), String::new(), String::new())
        };

    // Compute repr(C) type
    // phase-335 step 2 — `repr_c_type` / `view_repr_type` are composed in the pack
    // by the `cpp_repr_c_type` / `cpp_view_repr_type` filters from the neutral
    // facts below; the builder no longer spells them. `element_repr_type`
    // (`elem_repr_c`) stays here — it also feeds the Rust-side SequenceStructDef.

    // The repr(C) type of a sequence element (Rust mirror). Shared by the
    // SequenceStructDef and — for heap sequences — `element_repr_type` (used for
    // `size_of::<T>()` + the `*mut T` cast in the FFI deserializer).
    let elem_repr_c = match element_type {
        Some(FieldType::Primitive(prim)) => {
            repr_c_type_for_field(&FieldType::Primitive(*prim), current_package)
        }
        Some(FieldType::String) | Some(FieldType::WString) => {
            format!("[u8; {}]", CPP_DEFAULT_STRING_CAPACITY)
        }
        Some(FieldType::BoundedString(sz)) | Some(FieldType::BoundedWString(sz)) => {
            format!("[u8; {sz}]")
        }
        Some(FieldType::NamespacedType { package, name: n }) => {
            // When package is None the element type is from the current package
            let pkg = package.as_deref().or(current_package).unwrap_or("unknown");
            format!("{}_msg_{}_t", to_c_package_name(pkg), to_snake_case(n))
        }
        _ => "u8".to_string(),
    };

    // Build sequence struct def if needed
    let seq_struct = if is_sequence {
        Some(SequenceStructDef {
            struct_name: format!("{}_{}_seq_t", struct_name, to_snake_case(name)),
            element_type: elem_repr_c.clone(),
            capacity: sequence_capacity,
            is_heap,
        })
    } else {
        None
    };

    // For a heap sequence the FFI deserializer needs the element repr type.
    let element_repr_type = if is_sequence {
        elem_repr_c
    } else {
        String::new()
    };

    // Use element nested functions for array/sequence elements
    let final_nested_ser = if is_nested {
        nested_serialize_fn
    } else {
        elem_nested_ser
    };
    let final_nested_deser = if is_nested {
        nested_deserialize_fn
    } else {
        elem_nested_deser
    };
    let final_nested_teardown = if is_nested {
        nested_teardown_fn
    } else {
        elem_nested_teardown
    };

    // String capacity for deserialization (resolved for unbounded strings).
    let string_capacity = match field_type {
        FieldType::String | FieldType::WString => {
            cap_override.unwrap_or(CPP_DEFAULT_STRING_CAPACITY)
        }
        FieldType::BoundedString(sz) | FieldType::BoundedWString(sz) => *sz,
        _ => 0,
    };

    let element_string_capacity = match element_type {
        Some(FieldType::String) | Some(FieldType::WString) => CPP_DEFAULT_STRING_CAPACITY,
        Some(FieldType::BoundedString(sz)) | Some(FieldType::BoundedWString(sz)) => *sz,
        _ => 0,
    };

    let field = CppFfiField {
        name: escaped_name,
        field_type: field_type.clone(),
        struct_name: struct_name.to_string(),
        cap: cap_override,
        current_package: current_package.unwrap_or("").to_string(),
        cdr_write_method,
        cdr_read_method,
        element_cdr_write_method,
        element_cdr_read_method,
        array_size,
        sequence_capacity,
        nested_serialize_fn: final_nested_ser,
        nested_deserialize_fn: final_nested_deser,
        nested_teardown_fn: final_nested_teardown,
        string_capacity,
        element_string_capacity,
        is_primitive,
        is_string,
        is_array,
        is_sequence,
        is_nested,
        is_primitive_element,
        is_string_element,
        is_heap,
        element_repr_type,
        is_borrowed,
        borrowed_reader_call,
        borrowed_le,
    };

    (field, seq_struct)
}

// ============================================================================
// nros-serdes::Message schema builder
// ============================================================================
//
// Emits the `impl ::nros_serdes::Message for <Msg>` block + any helper
// `pub const` items (NestedType + element FieldType statics) so backends
// like `nros-rmw-cyclonedds` (Phase 212.K.7.4-6) can walk the static
// field schema at runtime via `<M as Message>::FIELDS` / `TYPE_NAME`.
//
// Per-field expressions reference helper consts (`FT_<name>`, `NESTED_<name>`)
// rather than inlining `&FieldType::...` literals — `&FieldType::Foo` doesn't
// yield a `&'static FieldType` because the temporary is dropped at end of
// expression. Top-level `pub const` items live for `'static` and provide
// the stable address the recursive variants need.

/// Schema artefacts attached to a generated nros message struct.
///
/// `nros_type_name` is the package-qualified ROS type name (e.g.
/// `"std_msgs/msg/Header"`) used for `Message::TYPE_NAME`.
///
/// `helper_consts` is a (possibly empty) block of `pub const` items that
/// must be emitted in the same module as the message struct so the
/// recursive `FieldType::Array(_, &FT_FOO)` / `FieldType::Nested(&NESTED_FOO)`
/// references resolve to `'static` addresses.
///
/// `fields_block` is the body of the `Message::FIELDS` slice — one
/// `::nros_serdes::Field { … },` per IDL field, in declaration order.
#[derive(Debug, Clone, Default)]
pub struct NrosMessageSchema {
    pub nros_type_name: String,
    pub helper_consts: String,
    pub fields_block: String,
}

/// Build the [`NrosMessageSchema`] for a parsed `.msg` body.
///
/// Uses the standard message convention: struct identifier matches
/// `message_name`, and `TYPE_NAME` is `<pkg>/msg/<MessageName>`.
/// Helper consts are emitted unprefixed (`NESTED_<X>`, `FT_<X>_ELEM`)
/// since a `.msg` package emits a single Message impl per file.
pub fn build_nros_message_schema(
    package_name: &str,
    message_name: &str,
    fields: &[rosidl_parser::Field],
    caps: &SchemaCaps<'_>,
) -> NrosMessageSchema {
    let nros_type_name = format!("{}/msg/{}", package_name, message_name);
    build_nros_schema_for_struct(
        package_name,
        message_name,
        &nros_type_name,
        "",
        fields,
        caps,
    )
}

/// Where the fields of ONE emitted schema read their `nros-codegen.toml` caps.
///
/// # Why the schema needs this at all (phase-403 W0)
///
/// The emitted `Message::FIELDS` is what `M::MAX_SERIALIZED_SIZE_XCDR*` — and
/// therefore `rx_buffer_for!(M)` — is computed from at compile time. It used to
/// render an unbounded `string` as `FieldType::String` whatever the config said,
/// while the STRUCT FIELD beside it was already `heapless::String<cap>`. So the
/// schema described a container the type does not have: a capped field was
/// storage-bounded and schema-unbounded at once, and with an unbounded type now
/// a build error, capping a field fixed the C header and left `rx_buffer_for!`
/// still refusing to compile. The knob the diagnostic names has to work.
///
/// # Why the message name is carried and not derived
///
/// A config key is `<package>/<Message>.<field>`, and `<Message>` is the string
/// the FIELD BUILDERS already resolve storage with — `Probe_Request`, not the
/// Rust ident `ProbeRequest` the schema emitter is otherwise named after. Two
/// spellings of one key is how a cap reaches the container and misses the
/// schema, which is the defect being fixed; carrying the field builders' own
/// string is what keeps the two from drifting.
pub struct SchemaCaps<'a> {
    message: &'a str,
    resolver: &'a CapacityResolver,
}

/// The resolver a schema with no configuration reads: none.
///
/// A `static` and not a per-call `CapacityResolver::empty()` so
/// [`SchemaCaps::unconfigured`] can hand out a borrow.
static NO_CAPS: std::sync::LazyLock<CapacityResolver> =
    std::sync::LazyLock::new(CapacityResolver::empty);

impl<'a> SchemaCaps<'a> {
    /// Read caps for the type the field builders call `message`.
    pub fn new(message: &'a str, resolver: &'a CapacityResolver) -> Self {
        Self { message, resolver }
    }

    /// No configuration: every field keeps the shape its `.msg` gives it.
    ///
    /// This is the honest spelling for a caller that genuinely has no resolver
    /// (the unit tests, and the action ENVELOPE structs, whose fields are
    /// rosidl-generated `uuid`/`status`/`stamp` members that no user config
    /// names). It is byte-identical to the pre-phase-403 emitter.
    pub fn unconfigured() -> SchemaCaps<'static> {
        SchemaCaps {
            message: "",
            resolver: &NO_CAPS,
        }
    }

    /// The bound the config STATES for one field of this schema, or `None`.
    ///
    /// Only the two shapes the resolver is keyed on are asked about, and only at
    /// the field's own level — the same set `field_to_nros_field_with_mode`
    /// treats as configurable, so the schema and the struct field are decided by
    /// one question asked once. A `.msg`-bounded shape is not asked at all: the
    /// interface is authoritative.
    fn declared(&self, package: &str, field: &rosidl_parser::Field) -> Option<usize> {
        let kind = match &field.field_type {
            FieldType::String | FieldType::WString => CapFieldKind::String,
            FieldType::Sequence { .. } => CapFieldKind::Sequence,
            _ => return None,
        };
        self.resolver
            .declared_bound(package, self.message, &field.name, kind)
    }

    /// phase-403 W7 — the field's shape with any configured ELEMENT bound
    /// folded in, so a `string[]` capped `element_cap = 32` renders the
    /// `BoundedString(32)` element the `.msg` spelling `string<=32[]` would
    /// have produced.
    ///
    /// The rewrite has to happen HERE, on the same resolver and the same
    /// `self.message` key the struct field is built from, or the emitted
    /// `Message::FIELDS` and the derived bound in `schema_value` would describe
    /// different types — the exact drift W0 fixed for the `cap` dimension.
    fn element_capped<'t>(
        &self,
        package: &str,
        field: &'t rosidl_parser::Field,
    ) -> std::borrow::Cow<'t, FieldType> {
        self.resolver
            .element_capped(package, self.message, &field.name, &field.field_type)
    }
}

/// Build the [`NrosMessageSchema`] for a Rust struct whose identifier
/// differs from its `Message::TYPE_NAME` payload.
///
/// Used by the service / action emit paths (K.7.1.c) where the Rust
/// struct name is e.g. `AddTwoIntsRequest` but the wire type-name
/// follows rosidl convention (`example_interfaces/srv/AddTwoInts_Request`).
///
/// `struct_name` is the Rust ident referenced by `offset_of!` macros.
/// `nros_type_name` is the ROS-side type name string written into
/// `Message::TYPE_NAME`.
/// `const_prefix` namespaces helper consts (`<prefix>NESTED_<X>`,
/// `<prefix>FT_<X>_ELEM`) so multiple schemas emitted in the same
/// module (service Request + Response, action Goal/Result/Feedback)
/// don't collide on shared field names. Pass `""` for the single-schema
/// `.msg` case.
pub fn build_nros_schema_for_struct(
    package_name: &str,
    struct_name: &str,
    nros_type_name: &str,
    const_prefix: &str,
    fields: &[rosidl_parser::Field],
    caps: &SchemaCaps<'_>,
) -> NrosMessageSchema {
    build_nros_schema_for_struct_with_path(
        package_name,
        struct_name,
        nros_type_name,
        const_prefix,
        fields,
        caps,
        &default_nested_type_path,
    )
}

/// Like [`build_nros_schema_for_struct`] but lets the caller override how
/// a `NamespacedType { package, name }` is rendered as a Rust path. The
/// default ([`default_nested_type_path`]) follows the `.msg` convention
/// (`crate::msg::<X>` / `<pkg>::msg::<X>`). The K.7.1.d action envelope
/// emit path uses a custom resolver to reach the action-self structs
/// (`<Action>Goal/Result/Feedback`, same module — bare ident) and the
/// `unique_identifier_msgs::msg::UUID` / `builtin_interfaces::msg::Time`
/// nested types (default path).
pub fn build_nros_schema_for_struct_with_path(
    package_name: &str,
    struct_name: &str,
    nros_type_name: &str,
    const_prefix: &str,
    fields: &[rosidl_parser::Field],
    caps: &SchemaCaps<'_>,
    nested_path_resolver: &dyn Fn(Option<&str>, &str, &str) -> String,
) -> NrosMessageSchema {
    let mut helper_consts = String::new();
    let mut fields_block = String::new();

    for field in fields {
        // Use the *raw* IDL field name for the schema (matches the .msg
        // source); the rendered struct field still goes through
        // `escape_keyword` to dodge Rust reserved words.
        let raw_name = &field.name;
        let access_name = escape_keyword(raw_name);
        let ty_expr = render_field_type_expr(
            raw_name,
            caps.element_capped(package_name, field).as_ref(),
            caps.declared(package_name, field),
            package_name,
            const_prefix,
            &mut helper_consts,
            nested_path_resolver,
        );
        fields_block.push_str(&format!(
            "        ::nros_serdes::Field {{\n            \
             name: \"{name}\",\n            \
             ty: {ty_expr},\n            \
             offset: ::core::mem::offset_of!({msg}, {access}),\n        }},\n",
            name = raw_name,
            ty_expr = ty_expr,
            msg = struct_name,
            access = access_name,
        ));
    }

    NrosMessageSchema {
        nros_type_name: nros_type_name.to_string(),
        helper_consts,
        fields_block,
    }
}

/// Emit the FieldType expression for a single field. Recursive variants
/// hoist their inner FieldType / NestedType into a module-scoped
/// `pub const`, appended to `helper_consts`, and reference it by name.
///
/// `const_prefix` namespaces the emitted helper-const idents so multiple
/// schemas in the same module don't collide on shared field names.
///
/// `declared` is the bound the codegen config states for THIS field
/// ([`SchemaCaps::declared`]), or `None`. It applies to the field's own
/// top-level shape only — the recursion into an element type passes `None`,
/// because a config key names a field and an element is not one, and because
/// the emitter spells an element string from a built-in default rather than
/// from the config (`nros_type_for_field_with_mode`). Claiming a bound for it
/// would be claiming a number nobody stated.
pub(crate) fn render_field_type_expr(
    field_name: &str,
    field_type: &FieldType,
    declared: Option<usize>,
    package_name: &str,
    const_prefix: &str,
    helper_consts: &mut String,
    nested_path_resolver: &dyn Fn(Option<&str>, &str, &str) -> String,
) -> String {
    match field_type {
        FieldType::Primitive(prim) => primitive_field_type_expr(prim).to_string(),
        FieldType::String => match declared {
            Some(n) => format!("::nros_serdes::FieldType::BoundedString({})", n),
            None => "::nros_serdes::FieldType::String".to_string(),
        },
        FieldType::WString => match declared {
            Some(n) => format!("::nros_serdes::FieldType::BoundedWString({})", n),
            None => "::nros_serdes::FieldType::WString".to_string(),
        },
        FieldType::BoundedString(n) => {
            format!("::nros_serdes::FieldType::BoundedString({})", n)
        }
        FieldType::BoundedWString(n) => {
            format!("::nros_serdes::FieldType::BoundedWString({})", n)
        }
        FieldType::NamespacedType { package, name } => {
            // Emit a NestedType helper const, sourcing TYPE_NAME + FIELDS
            // from the nested type's own Message impl so we never duplicate
            // the package/type-name string.
            let nested_const = format!("{}NESTED_{}", const_prefix, upper_ident(field_name));
            let nested_path = nested_path_resolver(package.as_deref(), name, package_name);
            helper_consts.push_str(&format!(
                "#[allow(non_upper_case_globals)]\n\
                 pub const {nested_const}: ::nros_serdes::NestedType = ::nros_serdes::NestedType {{\n    \
                 type_name: <{nested_path} as ::nros_serdes::Message>::TYPE_NAME,\n    \
                 fields: <{nested_path} as ::nros_serdes::Message>::FIELDS,\n}};\n",
            ));
            format!("::nros_serdes::FieldType::Nested(&{})", nested_const)
        }
        FieldType::Array { element_type, size } => {
            let elem_const = format!("{}FT_{}_ELEM", const_prefix, upper_ident(field_name));
            emit_element_const(
                &elem_const,
                field_name,
                element_type,
                package_name,
                const_prefix,
                helper_consts,
                nested_path_resolver,
            );
            format!("::nros_serdes::FieldType::Array({}, &{})", size, elem_const)
        }
        FieldType::Sequence { element_type } => {
            let elem_const = format!("{}FT_{}_ELEM", const_prefix, upper_ident(field_name));
            emit_element_const(
                &elem_const,
                field_name,
                element_type,
                package_name,
                const_prefix,
                helper_consts,
                nested_path_resolver,
            );
            match declared {
                Some(n) => format!(
                    "::nros_serdes::FieldType::BoundedSequence({}, &{})",
                    n, elem_const
                ),
                None => format!("::nros_serdes::FieldType::Sequence(&{})", elem_const),
            }
        }
        FieldType::BoundedSequence {
            element_type,
            max_size,
        } => {
            let elem_const = format!("{}FT_{}_ELEM", const_prefix, upper_ident(field_name));
            emit_element_const(
                &elem_const,
                field_name,
                element_type,
                package_name,
                const_prefix,
                helper_consts,
                nested_path_resolver,
            );
            format!(
                "::nros_serdes::FieldType::BoundedSequence({}, &{})",
                max_size, elem_const
            )
        }
    }
}

/// Emit a `pub const <ident>: FieldType = <expr>;` for the recursive
/// element of an Array / Sequence / BoundedSequence field.
fn emit_element_const(
    const_ident: &str,
    field_name: &str,
    element_type: &FieldType,
    package_name: &str,
    const_prefix: &str,
    helper_consts: &mut String,
    nested_path_resolver: &dyn Fn(Option<&str>, &str, &str) -> String,
) {
    // The inner expression is rendered with the *parent* field name so any
    // further-nested helpers stay scoped under the same FT_<FIELD>_ prefix.
    let inner_expr = render_field_type_expr(
        field_name,
        element_type,
        // An element is not a field, so no config key names it. See
        // `render_field_type_expr`'s `declared`.
        None,
        package_name,
        const_prefix,
        helper_consts,
        nested_path_resolver,
    );
    helper_consts.push_str(&format!(
        "#[allow(non_upper_case_globals)]\n\
         pub const {ident}: ::nros_serdes::FieldType = {inner};\n",
        ident = const_ident,
        inner = inner_expr,
    ));
}

/// Map an IDL primitive to its `::nros_serdes::FieldType::*` variant.
fn primitive_field_type_expr(prim: &PrimitiveType) -> &'static str {
    match prim {
        PrimitiveType::Bool => "::nros_serdes::FieldType::Bool",
        // ROS IDL `octet` / `byte` / `char` and `uint8` all map to Uint8 on
        // the wire (same single-byte CDR encoding).
        PrimitiveType::Byte | PrimitiveType::Char | PrimitiveType::UInt8 => {
            "::nros_serdes::FieldType::Uint8"
        }
        PrimitiveType::Int8 => "::nros_serdes::FieldType::Int8",
        PrimitiveType::UInt16 => "::nros_serdes::FieldType::Uint16",
        PrimitiveType::Int16 => "::nros_serdes::FieldType::Int16",
        PrimitiveType::UInt32 => "::nros_serdes::FieldType::Uint32",
        PrimitiveType::Int32 => "::nros_serdes::FieldType::Int32",
        PrimitiveType::UInt64 => "::nros_serdes::FieldType::Uint64",
        PrimitiveType::Int64 => "::nros_serdes::FieldType::Int64",
        PrimitiveType::Float32 => "::nros_serdes::FieldType::Float32",
        PrimitiveType::Float64 => "::nros_serdes::FieldType::Float64",
    }
}

/// Render the Rust path to a nested message type. Mirrors the
/// crate-mode rules in `nros_type_for_field_with_mode` for
/// `NamespacedType` so we can hand the type as `<Path as Message>`.
///
/// Default resolver passed to
/// [`build_nros_schema_for_struct_with_path`] — assumes the nested
/// type lives under `<pkg>::msg::<X>` (the `.msg` shape).
pub fn default_nested_type_path(pkg: Option<&str>, name: &str, current_package: &str) -> String {
    match pkg {
        Some(p) if p == current_package => format!("crate::msg::{}", name),
        Some(p) => format!("{}::msg::{}", p, name),
        None => format!("crate::msg::{}", name),
    }
}

// ============================================================================
// Action envelope schemas (Phase 212.K.7.1.d)
// ============================================================================

/// Schemas for the five rosidl action wire-envelope structs of a single
/// action package. Mirrors the upstream `rosidl_generator_cpp` shape:
///
/// - `<A>_SendGoal_Request`   { goal_id: UUID, goal: <A>Goal }
/// - `<A>_SendGoal_Response`  { accepted: bool, stamp: Time }
/// - `<A>_GetResult_Request`  { goal_id: UUID }
/// - `<A>_GetResult_Response` { status: int8, result: <A>Result }
/// - `<A>_FeedbackMessage`    { goal_id: UUID, feedback: <A>Feedback }
///
/// Field order, names, and types match
/// `rosidl_generator_cpp`'s `<action>__struct.hpp` exactly (verified
/// against `example_interfaces/action/Fibonacci`).
#[derive(Debug, Clone)]
pub struct ActionEnvelopeSchemas {
    pub send_goal_request: NrosMessageSchema,
    pub send_goal_response: NrosMessageSchema,
    pub get_result_request: NrosMessageSchema,
    pub get_result_response: NrosMessageSchema,
    pub feedback_message: NrosMessageSchema,
}

/// Build the [`ActionEnvelopeSchemas`] for an action, given the action's
/// host package + name. The five schemas reference both external types
/// (`unique_identifier_msgs::msg::UUID`, `builtin_interfaces::msg::Time`)
/// and action-self types (`<A>Goal/Result/Feedback`, same module — bare
/// idents). The custom path resolver fans them out.
pub fn build_action_envelope_schemas(
    package_name: &str,
    action_name: &str,
) -> ActionEnvelopeSchemas {
    use rosidl_parser::{Field, PrimitiveType};

    let goal_struct = format!("{}Goal", action_name);
    let result_struct = format!("{}Result", action_name);
    let feedback_struct = format!("{}Feedback", action_name);

    // The five envelope structs have no configurable field: every member below
    // is a rosidl-fixed `goal_id`/`accepted`/`stamp`/`status` or a nested
    // reference to the user's own Goal/Result/Feedback, whose caps are read
    // where THAT struct's schema is emitted. So there is nothing here for a
    // config key to name, and saying so is more honest than threading a
    // resolver that can never match.
    let envelope_caps = SchemaCaps::unconfigured();

    // Action-self struct path resolver: when the nested package matches
    // this action's host package AND the struct name matches one of the
    // three user-facing structs, reach it as a bare ident (same module).
    // Everything else falls through to the default `.msg` shape.
    let goal_clone = goal_struct.clone();
    let result_clone = result_struct.clone();
    let feedback_clone = feedback_struct.clone();
    let pkg_clone = package_name.to_string();
    let resolver = move |pkg: Option<&str>, name: &str, current_package: &str| -> String {
        if pkg == Some(pkg_clone.as_str())
            && (name == goal_clone || name == result_clone || name == feedback_clone)
        {
            name.to_string()
        } else {
            default_nested_type_path(pkg, name, current_package)
        }
    };

    let uuid_field = || Field {
        name: "goal_id".to_string(),
        field_type: FieldType::NamespacedType {
            package: Some("unique_identifier_msgs".to_string()),
            name: "UUID".to_string(),
        },
        default_value: None,
    };
    let self_field = |name: &str, struct_name: &str| Field {
        name: name.to_string(),
        field_type: FieldType::NamespacedType {
            package: Some(package_name.to_string()),
            name: struct_name.to_string(),
        },
        default_value: None,
    };

    // SendGoal_Request: goal_id (UUID), goal (<A>Goal)
    let send_goal_request = build_nros_schema_for_struct_with_path(
        package_name,
        &format!("{}_SendGoal_Request", action_name),
        &format!("{}/action/{}_SendGoal_Request", package_name, action_name),
        "SG_REQ_",
        &[uuid_field(), self_field("goal", &goal_struct)],
        &envelope_caps,
        &resolver,
    );

    // SendGoal_Response: accepted (bool), stamp (Time)
    let send_goal_response = build_nros_schema_for_struct_with_path(
        package_name,
        &format!("{}_SendGoal_Response", action_name),
        &format!("{}/action/{}_SendGoal_Response", package_name, action_name),
        "SG_RESP_",
        &[
            Field {
                name: "accepted".to_string(),
                field_type: FieldType::Primitive(PrimitiveType::Bool),
                default_value: None,
            },
            Field {
                name: "stamp".to_string(),
                field_type: FieldType::NamespacedType {
                    package: Some("builtin_interfaces".to_string()),
                    name: "Time".to_string(),
                },
                default_value: None,
            },
        ],
        &envelope_caps,
        &resolver,
    );

    // GetResult_Request: goal_id (UUID)
    let get_result_request = build_nros_schema_for_struct_with_path(
        package_name,
        &format!("{}_GetResult_Request", action_name),
        &format!("{}/action/{}_GetResult_Request", package_name, action_name),
        "GR_REQ_",
        &[uuid_field()],
        &envelope_caps,
        &resolver,
    );

    // GetResult_Response: status (int8), result (<A>Result)
    let get_result_response = build_nros_schema_for_struct_with_path(
        package_name,
        &format!("{}_GetResult_Response", action_name),
        &format!("{}/action/{}_GetResult_Response", package_name, action_name),
        "GR_RESP_",
        &[
            Field {
                name: "status".to_string(),
                field_type: FieldType::Primitive(PrimitiveType::Int8),
                default_value: None,
            },
            self_field("result", &result_struct),
        ],
        &envelope_caps,
        &resolver,
    );

    // FeedbackMessage: goal_id (UUID), feedback (<A>Feedback)
    let feedback_message = build_nros_schema_for_struct_with_path(
        package_name,
        &format!("{}_FeedbackMessage", action_name),
        &format!("{}/action/{}_FeedbackMessage", package_name, action_name),
        "FB_",
        &[uuid_field(), self_field("feedback", &feedback_struct)],
        &envelope_caps,
        &resolver,
    );

    ActionEnvelopeSchemas {
        send_goal_request,
        send_goal_response,
        get_result_request,
        get_result_response,
        feedback_message,
    }
}

/// Turn a field name into an UPPER_SNAKE_CASE identifier fragment for
/// use inside helper-const names (`NESTED_<X>`, `FT_<X>_ELEM`).
fn upper_ident(s: &str) -> String {
    // Strip a trailing `_` first — `escape_keyword` adds one for reserved
    // words, but it's stable to recompute via the raw IDL name. We keep
    // ASCII-safe transforms only; IDL field names are already
    // `[a-z][a-z0-9_]*`.
    s.trim_end_matches('_').to_ascii_uppercase()
}

/// issue 0896 layer 2 / phase-408 W1 — one type's serialized-size bound,
/// derived ONCE and handed to every emitter that states it.
///
/// The number comes from `nros_serdes::size::max_serialized_size` over a schema
/// built by [`crate::schema_value`] — THE size rule, the same function the Rust
/// runtime's `M::MAX_SERIALIZED_SIZE_XCDR*` consts use. It is deliberately NOT
/// [`crate::types::compute_serialized_size_max`], the C++ pack's older in-header
/// `SERIALIZED_SIZE_MAX`: that one ESTIMATES (a flat 512 bytes per nested
/// message, a flat default capacity per string) and always returns a value, so
/// it can neither report "unbounded" nor be relied on to err upwards — a nested
/// type whose own bound exceeds 512 is charged 512, which UNDER-sizes a receive
/// buffer (issue 0964).
///
/// Living here rather than in one emitter is the point. The C header had this
/// derivation inline; giving the C++ pack its own copy would have been a second
/// implementation of "how big can this type get" along the LANGUAGE axis, which
/// is the sizes-header mirror class (0088 → 0114 → 0122 → 0123 → 0245 → 0268)
/// with a new hat. Both packs now call this, so a C header and a C++ header for
/// the same type cannot state different numbers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedBound {
    /// XCDR1 — what this stack WRITES, so a transmit buffer wants exactly it.
    pub tx: Option<usize>,
    /// `max(XCDR1, XCDR2)` — a receive buffer must hold whatever encoding a
    /// peer negotiated (RFC-0055), so it takes the larger of the two.
    pub rx: Option<usize>,
    /// Prose reason, `Some` exactly when there is no bound. Says WHICH of the
    /// two no-bound outcomes happened: "unbounded member" (we looked and no
    /// bound exists — bound the field) versus "could not be resolved" (we could
    /// not look — fix the search path). Collapsing those is the confusion issue
    /// 0896 exists to remove.
    pub reason: Option<String>,
    /// A C identifier naming the type AND the member that costs it the bound,
    /// `Some` exactly when there is no bound. Emitters POISON the size constant
    /// with it, so naming the constant is an error that says what is wrong
    /// rather than "undeclared identifier".
    pub token: Option<String>,
}

/// Derive [`DerivedBound`] for one parsed message.
///
/// `ident_prefix` is the C-style flat type name (`std_msgs_msg_string`) the
/// poison token is built from. The C and the C++ pack pass the SAME prefix, so
/// one type has one poison token in both languages.
///
/// `lookup` resolves nested types. A caller that has no resolver passes
/// `&|_| None` and gets `Unresolved` — never a guessed bound.
pub fn derive_message_bound(
    package_name: &str,
    message_name: &str,
    message: &rosidl_parser::Message,
    ident_prefix: &str,
    caps: &CapacityResolver,
    lookup: &crate::schema_value::MsgLookup<'_>,
) -> Result<DerivedBound, GeneratorError> {
    use crate::schema_value::{TypeBound, bound_message};
    use nros_serdes::cdr::EncodingVersion;

    let fqn = format!("{package_name}/{message_name}");
    let x1 = bound_message(&fqn, message, EncodingVersion::Xcdr1, caps, lookup);
    let x2 = bound_message(&fqn, message, EncodingVersion::Xcdr2, caps, lookup);

    // phase-403 W7b (issue 0961) — a stated `max_serialized` budget is checked
    // HERE, against the same classification the emitted constants come from, so
    // the number in the diagnostic is the number in the header.
    crate::bounds::check_budget(
        &fqn,
        &crate::bounds::BoundState::classify(&x1, &x2),
        &crate::schema_value::chains_for(&fqn, message, caps, lookup),
        caps.max_serialized(package_name, message_name),
    )
    .map_err(|e| GeneratorError::BoundExceedsBudget {
        details: e.to_string(),
    })?;

    // issue 0896 Q2 — the compiler error must name the TYPE and the FIELD, not
    // just say "no bound". A C identifier cannot hold `.` or `(`, so the path is
    // flattened; the prose reason travels beside it for the parts an identifier
    // cannot carry.
    let flatten = |what: &str| -> String {
        what.chars()
            .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
            .collect()
    };
    let unbounded_token = |what: &str| {
        let member = what.split(" (").next().unwrap_or(what);
        format!("NROS_UNBOUNDED__{ident_prefix}__field_{}", flatten(member))
    };
    let unresolved_token = |t: &str| {
        format!(
            "NROS_UNRESOLVED__{ident_prefix}__nested_type_{}",
            flatten(t)
        )
    };

    // phase-403 W6 — the TX/RX classification lives in `bounds::BoundState` so
    // the emitted constants and the exported inventory cannot drift into
    // disagreeing about which encoding feeds which direction. The poison TOKENS
    // stay here: they are identifiers, which only the emitters need.
    Ok(match crate::bounds::BoundState::classify(&x1, &x2) {
        // TX writes XCDR1; RX must hold either encoding, so it takes the max.
        crate::bounds::BoundState::Bounded { tx, rx } => DerivedBound {
            tx: Some(tx),
            rx: Some(rx),
            reason: None,
            token: None,
        },
        crate::bounds::BoundState::Unbounded { reason } => {
            // The prose reason names EVERY offending member (phase-403 W0); the
            // poison TOKEN can only name one, because it is an identifier. The
            // FIRST is the one it names, matching the order the reason lists
            // them in, so the identifier the compiler prints is the first line
            // of the reason beside it.
            let members = match (&x1, &x2) {
                (TypeBound::Unbounded(w), _) | (_, TypeBound::Unbounded(w)) => w.clone(),
                _ => unreachable!("classify only reports Unbounded from an Unbounded input"),
            };
            let first = members
                .first()
                .cloned()
                .unwrap_or_else(|| "unknown".to_string());
            DerivedBound {
                tx: None,
                rx: None,
                reason: Some(reason),
                token: Some(unbounded_token(&first)),
            }
        }
        crate::bounds::BoundState::Unresolved { reason } => {
            let nested = match (&x1, &x2) {
                (TypeBound::Unresolved(t), _) | (_, TypeBound::Unresolved(t)) => t.clone(),
                _ => unreachable!("classify only reports Unresolved from an Unresolved input"),
            };
            DerivedBound {
                tx: None,
                rx: None,
                reason: Some(reason),
                token: Some(unresolved_token(&nested)),
            }
        }
    })
}

#[cfg(test)]
mod schema_tests {
    use super::*;
    use rosidl_parser::{Field, PrimitiveType};

    fn prim_field(name: &str, prim: PrimitiveType) -> Field {
        Field {
            name: name.to_string(),
            field_type: FieldType::Primitive(prim),
            default_value: None,
        }
    }

    fn nested_field(name: &str, pkg: &str, ty: &str) -> Field {
        Field {
            name: name.to_string(),
            field_type: FieldType::NamespacedType {
                package: Some(pkg.to_string()),
                name: ty.to_string(),
            },
            default_value: None,
        }
    }

    #[test]
    fn primitive_only_emits_inline_field_type() {
        let schema = build_nros_message_schema(
            "std_msgs",
            "Int32",
            &[prim_field("data", PrimitiveType::Int32)],
            &SchemaCaps::unconfigured(),
        );
        assert_eq!(schema.nros_type_name, "std_msgs/msg/Int32");
        assert_eq!(schema.helper_consts, "");
        assert!(schema.fields_block.contains("name: \"data\","));
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::Int32,")
        );
        assert!(
            schema
                .fields_block
                .contains("offset: ::core::mem::offset_of!(Int32, data)")
        );
    }

    #[test]
    fn nested_field_emits_nested_helper_const() {
        let schema = build_nros_message_schema(
            "std_msgs",
            "Header",
            &[
                nested_field("stamp", "builtin_interfaces", "Time"),
                Field {
                    name: "frame_id".to_string(),
                    field_type: FieldType::String,
                    default_value: None,
                },
            ],
            &SchemaCaps::unconfigured(),
        );
        assert!(
            schema
                .helper_consts
                .contains("pub const NESTED_STAMP: ::nros_serdes::NestedType")
        );
        assert!(
            schema
                .helper_consts
                .contains("<builtin_interfaces::msg::Time as ::nros_serdes::Message>::TYPE_NAME")
        );
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::Nested(&NESTED_STAMP),")
        );
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::String,")
        );
    }

    #[test]
    fn bounded_sequence_emits_element_const() {
        let schema = build_nros_message_schema(
            "test_msgs",
            "Bounded",
            &[Field {
                name: "items".to_string(),
                field_type: FieldType::BoundedSequence {
                    element_type: Box::new(FieldType::Primitive(PrimitiveType::UInt8)),
                    max_size: 16,
                },
                default_value: None,
            }],
            &SchemaCaps::unconfigured(),
        );
        assert!(
            schema
                .helper_consts
                .contains("pub const FT_ITEMS_ELEM: ::nros_serdes::FieldType")
        );
        assert!(
            schema
                .helper_consts
                .contains("= ::nros_serdes::FieldType::Uint8;")
        );
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::BoundedSequence(16, &FT_ITEMS_ELEM),")
        );
    }

    #[test]
    fn bounded_string_inlines_capacity() {
        let schema = build_nros_message_schema(
            "test_msgs",
            "Strs",
            &[Field {
                name: "label".to_string(),
                field_type: FieldType::BoundedString(32),
                default_value: None,
            }],
            &SchemaCaps::unconfigured(),
        );
        assert!(schema.helper_consts.is_empty());
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::BoundedString(32),")
        );
    }

    #[test]
    fn array_of_nested_emits_chained_consts() {
        let schema = build_nros_message_schema(
            "test_msgs",
            "Mixed",
            &[Field {
                name: "points".to_string(),
                field_type: FieldType::Array {
                    element_type: Box::new(FieldType::NamespacedType {
                        package: Some("geometry_msgs".to_string()),
                        name: "Point".to_string(),
                    }),
                    size: 3,
                },
                default_value: None,
            }],
            &SchemaCaps::unconfigured(),
        );
        // Array hoists FT_POINTS_ELEM; the nested type hoists NESTED_POINTS
        // (named after the parent field, since we scope inner consts under
        // the parent field's name).
        assert!(
            schema
                .helper_consts
                .contains("pub const NESTED_POINTS: ::nros_serdes::NestedType")
        );
        assert!(
            schema
                .helper_consts
                .contains("pub const FT_POINTS_ELEM: ::nros_serdes::FieldType = ::nros_serdes::FieldType::Nested(&NESTED_POINTS);")
        );
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::Array(3, &FT_POINTS_ELEM),")
        );
    }

    #[test]
    fn self_package_nested_uses_crate_path() {
        let schema = build_nros_message_schema(
            "local_msgs",
            "Outer",
            &[nested_field("inner", "local_msgs", "Inner")],
            &SchemaCaps::unconfigured(),
        );
        assert!(
            schema
                .helper_consts
                .contains("<crate::msg::Inner as ::nros_serdes::Message>::TYPE_NAME")
        );
    }

    #[test]
    fn keyword_field_name_escapes_for_offset() {
        // `type` is a Rust keyword and gets a trailing underscore in the
        // host struct field — schema name stays raw, but offset_of!
        // must reference the escaped Rust field.
        let schema = build_nros_message_schema(
            "test_msgs",
            "Sample",
            &[prim_field("type", PrimitiveType::Int32)],
            &SchemaCaps::unconfigured(),
        );
        assert!(schema.fields_block.contains("name: \"type\","));
        assert!(schema.fields_block.contains("offset_of!(Sample, type_)"));
    }

    // ------------------------------------------------------------------
    // K.7.1.c — service Request/Response + action Goal/Result/Feedback
    //
    // These use `build_nros_schema_for_struct` directly to verify the
    // (struct_name, type_name, const_prefix) generalization.
    // ------------------------------------------------------------------

    #[test]
    fn service_request_schema_uses_srv_type_name_and_struct_offset() {
        let schema = build_nros_schema_for_struct(
            "example_interfaces",
            "AddTwoIntsRequest",
            "example_interfaces/srv/AddTwoInts_Request",
            "REQ_",
            &[
                prim_field("a", PrimitiveType::Int64),
                prim_field("b", PrimitiveType::Int64),
            ],
            &SchemaCaps::unconfigured(),
        );
        // Primitive-only schema needs no helper consts.
        assert_eq!(schema.helper_consts, "");
        assert_eq!(
            schema.nros_type_name,
            "example_interfaces/srv/AddTwoInts_Request"
        );
        // Offset must reference the Rust struct ident (AddTwoIntsRequest),
        // not the rosidl wire name.
        assert!(
            schema
                .fields_block
                .contains("offset: ::core::mem::offset_of!(AddTwoIntsRequest, a)")
        );
        assert!(
            schema
                .fields_block
                .contains("offset: ::core::mem::offset_of!(AddTwoIntsRequest, b)")
        );
        assert!(
            schema
                .fields_block
                .contains("ty: ::nros_serdes::FieldType::Int64,")
        );
    }

    #[test]
    fn service_response_schema_distinct_helper_const_prefix() {
        // Same field name on both halves with nested types — the
        // REQ_/RESP_ const prefix is what keeps the module-scope idents
        // distinct.
        let req = build_nros_schema_for_struct(
            "demo",
            "MoveRequest",
            "demo/srv/Move_Request",
            "REQ_",
            &[nested_field("header", "std_msgs", "Header")],
            &SchemaCaps::unconfigured(),
        );
        let resp = build_nros_schema_for_struct(
            "demo",
            "MoveResponse",
            "demo/srv/Move_Response",
            "RESP_",
            &[nested_field("header", "std_msgs", "Header")],
            &SchemaCaps::unconfigured(),
        );
        assert!(req.helper_consts.contains("pub const REQ_NESTED_HEADER:"));
        assert!(resp.helper_consts.contains("pub const RESP_NESTED_HEADER:"));
        // No collision: REQ_ ident never appears in the RESP_ block and vv.
        assert!(!req.helper_consts.contains("RESP_NESTED_HEADER"));
        assert!(!resp.helper_consts.contains("REQ_NESTED_HEADER"));
        assert!(
            req.fields_block
                .contains("ty: ::nros_serdes::FieldType::Nested(&REQ_NESTED_HEADER),")
        );
        assert!(
            resp.fields_block
                .contains("ty: ::nros_serdes::FieldType::Nested(&RESP_NESTED_HEADER),")
        );
    }

    /// Verifies action goal, result, and feedback type names follow ROSIDL convention.
    #[test]
    fn action_types_follow_rosidl_convention() {
        let goal = build_nros_schema_for_struct(
            "example_interfaces",
            "FibonacciGoal",
            "example_interfaces/action/Fibonacci_Goal",
            "GOAL_",
            &[prim_field("order", PrimitiveType::Int32)],
            &SchemaCaps::unconfigured(),
        );
        let result = build_nros_schema_for_struct(
            "example_interfaces",
            "FibonacciResult",
            "example_interfaces/action/Fibonacci_Result",
            "RESULT_",
            &[Field {
                name: "sequence".to_string(),
                field_type: FieldType::Sequence {
                    element_type: Box::new(FieldType::Primitive(PrimitiveType::Int32)),
                },
                default_value: None,
            }],
            &SchemaCaps::unconfigured(),
        );
        let feedback = build_nros_schema_for_struct(
            "example_interfaces",
            "FibonacciFeedback",
            "example_interfaces/action/Fibonacci_Feedback",
            "FEEDBACK_",
            &[Field {
                name: "sequence".to_string(),
                field_type: FieldType::Sequence {
                    element_type: Box::new(FieldType::Primitive(PrimitiveType::Int32)),
                },
                default_value: None,
            }],
            &SchemaCaps::unconfigured(),
        );
        assert_eq!(
            goal.nros_type_name,
            "example_interfaces/action/Fibonacci_Goal"
        );
        assert_eq!(
            result.nros_type_name,
            "example_interfaces/action/Fibonacci_Result"
        );
        assert_eq!(
            feedback.nros_type_name,
            "example_interfaces/action/Fibonacci_Feedback"
        );
        // Result and Feedback share field name `sequence` but the
        // RESULT_/FEEDBACK_ prefix keeps the FT_*_ELEM idents distinct.
        assert!(
            result
                .helper_consts
                .contains("pub const RESULT_FT_SEQUENCE_ELEM:")
        );
        assert!(
            feedback
                .helper_consts
                .contains("pub const FEEDBACK_FT_SEQUENCE_ELEM:")
        );
        assert!(!result.helper_consts.contains("FEEDBACK_FT_SEQUENCE_ELEM"));
        assert!(!feedback.helper_consts.contains("RESULT_FT_SEQUENCE_ELEM"));
        // Offsets reference the Rust struct ident.
        assert!(
            goal.fields_block
                .contains("offset: ::core::mem::offset_of!(FibonacciGoal, order)")
        );
        assert!(
            result
                .fields_block
                .contains("offset: ::core::mem::offset_of!(FibonacciResult, sequence)")
        );
        assert!(
            feedback
                .fields_block
                .contains("offset: ::core::mem::offset_of!(FibonacciFeedback, sequence)")
        );
    }

    // ------------------------------------------------------------------
    // K.7.1.d — action envelope schemas
    // ------------------------------------------------------------------

    #[test]
    fn action_envelope_send_goal_request_shape() {
        let envs = build_action_envelope_schemas("example_interfaces", "Fibonacci");
        let s = &envs.send_goal_request;
        assert_eq!(
            s.nros_type_name,
            "example_interfaces/action/Fibonacci_SendGoal_Request"
        );
        // goal_id resolves to the cross-package UUID type.
        assert!(
            s.helper_consts.contains("pub const SG_REQ_NESTED_GOAL_ID:"),
            "helper_consts = {}",
            s.helper_consts
        );
        assert!(
            s.helper_consts.contains(
                "<unique_identifier_msgs::msg::UUID as ::nros_serdes::Message>::TYPE_NAME"
            )
        );
        // goal resolves to the action-self struct (bare ident — same module).
        assert!(s.helper_consts.contains("pub const SG_REQ_NESTED_GOAL:"));
        assert!(
            s.helper_consts
                .contains("<FibonacciGoal as ::nros_serdes::Message>::TYPE_NAME")
        );
        // Offsets reference the envelope struct name (raw rosidl form).
        assert!(
            s.fields_block
                .contains("offset: ::core::mem::offset_of!(Fibonacci_SendGoal_Request, goal_id)")
        );
        assert!(
            s.fields_block
                .contains("offset: ::core::mem::offset_of!(Fibonacci_SendGoal_Request, goal)")
        );
        // Field order matches upstream: goal_id then goal.
        let goal_id_pos = s.fields_block.find("\"goal_id\"").unwrap();
        let goal_pos = s.fields_block.find("\"goal\"").unwrap();
        assert!(goal_id_pos < goal_pos);
    }

    #[test]
    fn action_envelope_send_goal_response_shape() {
        let envs = build_action_envelope_schemas("example_interfaces", "Fibonacci");
        let s = &envs.send_goal_response;
        assert_eq!(
            s.nros_type_name,
            "example_interfaces/action/Fibonacci_SendGoal_Response"
        );
        // stamp resolves to builtin_interfaces::msg::Time.
        assert!(s.helper_consts.contains("pub const SG_RESP_NESTED_STAMP:"));
        assert!(
            s.helper_consts
                .contains("<builtin_interfaces::msg::Time as ::nros_serdes::Message>::TYPE_NAME")
        );
        // accepted is a primitive (no helper const for that one).
        assert!(s.fields_block.contains("\"accepted\""));
        assert!(s.fields_block.contains("::nros_serdes::FieldType::Bool"));
        assert!(
            s.fields_block
                .contains("ty: ::nros_serdes::FieldType::Nested(&SG_RESP_NESTED_STAMP),")
        );
        // Field order: accepted then stamp (matches upstream Fibonacci_SendGoal_Response_).
        let acc_pos = s.fields_block.find("\"accepted\"").unwrap();
        let stamp_pos = s.fields_block.find("\"stamp\"").unwrap();
        assert!(acc_pos < stamp_pos);
    }

    #[test]
    fn action_envelope_get_result_request_shape() {
        let envs = build_action_envelope_schemas("example_interfaces", "Fibonacci");
        let s = &envs.get_result_request;
        assert_eq!(
            s.nros_type_name,
            "example_interfaces/action/Fibonacci_GetResult_Request"
        );
        // Single field: goal_id.
        assert!(s.helper_consts.contains("pub const GR_REQ_NESTED_GOAL_ID:"));
        assert!(s.fields_block.contains("\"goal_id\""));
        assert!(
            s.fields_block
                .contains("ty: ::nros_serdes::FieldType::Nested(&GR_REQ_NESTED_GOAL_ID),")
        );
        // No second field.
        assert_eq!(s.fields_block.matches("name: ").count(), 1);
    }

    #[test]
    fn action_envelope_get_result_response_shape() {
        let envs = build_action_envelope_schemas("example_interfaces", "Fibonacci");
        let s = &envs.get_result_response;
        assert_eq!(
            s.nros_type_name,
            "example_interfaces/action/Fibonacci_GetResult_Response"
        );
        // Field order: status (Int8) then result (Nested<FibonacciResult>).
        assert!(s.fields_block.contains("\"status\""));
        assert!(s.fields_block.contains("::nros_serdes::FieldType::Int8"));
        assert!(s.helper_consts.contains("pub const GR_RESP_NESTED_RESULT:"));
        assert!(
            s.helper_consts
                .contains("<FibonacciResult as ::nros_serdes::Message>::TYPE_NAME")
        );
        let status_pos = s.fields_block.find("\"status\"").unwrap();
        let result_pos = s.fields_block.find("\"result\"").unwrap();
        assert!(status_pos < result_pos);
    }

    #[test]
    fn action_envelope_feedback_message_shape() {
        let envs = build_action_envelope_schemas("example_interfaces", "Fibonacci");
        let s = &envs.feedback_message;
        assert_eq!(
            s.nros_type_name,
            "example_interfaces/action/Fibonacci_FeedbackMessage"
        );
        assert!(s.helper_consts.contains("pub const FB_NESTED_GOAL_ID:"));
        assert!(s.helper_consts.contains("pub const FB_NESTED_FEEDBACK:"));
        assert!(
            s.helper_consts
                .contains("<FibonacciFeedback as ::nros_serdes::Message>::TYPE_NAME")
        );
        let goal_id_pos = s.fields_block.find("\"goal_id\"").unwrap();
        let feedback_pos = s.fields_block.find("\"feedback\"").unwrap();
        assert!(goal_id_pos < feedback_pos);
    }

    #[test]
    fn action_envelope_prefixes_are_distinct_across_halves() {
        // Every envelope must use its own SG_REQ_ / SG_RESP_ / GR_REQ_
        // / GR_RESP_ / FB_ prefix so the module-scope `pub const`
        // idents don't collide. Shared field name `goal_id` lives on
        // SendGoal_Request, GetResult_Request, and FeedbackMessage —
        // the three matching NESTED_GOAL_ID consts must not clash.
        let envs = build_action_envelope_schemas("example_interfaces", "Fibonacci");
        assert!(envs.send_goal_request.helper_consts.contains("SG_REQ_"));
        assert!(envs.send_goal_response.helper_consts.contains("SG_RESP_"));
        assert!(envs.get_result_request.helper_consts.contains("GR_REQ_"));
        assert!(envs.get_result_response.helper_consts.contains("GR_RESP_"));
        assert!(envs.feedback_message.helper_consts.contains("FB_"));
        // Per-half: the OTHER half's prefix must not leak in.
        assert!(!envs.send_goal_request.helper_consts.contains("GR_REQ_"));
        assert!(!envs.feedback_message.helper_consts.contains("SG_REQ_"));
        assert!(!envs.get_result_request.helper_consts.contains("FB_NESTED"));
    }

    #[test]
    fn empty_request_schema_emits_no_fields_no_helpers() {
        // A trigger-style service has an empty request body.
        let schema = build_nros_schema_for_struct(
            "std_srvs",
            "TriggerRequest",
            "std_srvs/srv/Trigger_Request",
            "REQ_",
            &[],
            &SchemaCaps::unconfigured(),
        );
        assert_eq!(schema.helper_consts, "");
        assert_eq!(schema.fields_block, "");
        assert_eq!(schema.nros_type_name, "std_srvs/srv/Trigger_Request");
    }
}
