//! ROS 2 Parameter Services
//!
//! This module implements the ROS 2 parameter service interface, enabling
//! `ros2 param` CLI tools to interact with nros nodes.
//!
//! # Services
//!
//! The following services are registered under the node's namespace:
//! - `~/get_parameters` - Get parameter values by name
//! - `~/set_parameters` - Set parameter values
//! - `~/list_parameters` - List all parameters
//! - `~/describe_parameters` - Get parameter descriptors
//! - `~/get_parameter_types` - Get parameter types
//! - `~/set_parameters_atomically` - Set multiple parameters atomically
//!
//! # Example
//!
//! ```ignore
//! use nros::prelude::*;
//!
//! let config = ExecutorConfig::from_env().node_name("my_node");
//! let mut executor: Executor = Executor::open(&config)?;
//! let mut node = executor.create_node("my_node")?;
//!
//! // Parameter services can be integrated via the executor
//! // and respond to `ros2 param list /my_node` etc.
//! ```

// Note: Module is already gated by #[cfg(feature = "param-services")] in lib.rs

extern crate alloc;
use alloc::boxed::Box;

use nros_params::{
    ParameterDescriptor as InternalDescriptor, ParameterServer, ParameterType as InternalType,
    ParameterValue as InternalValue, SetParameterResult,
};

pub(crate) use nros_rcl_interfaces::{
    msg::{
        FloatingPointRange, IntegerRange, ParameterDescriptor, ParameterValue, SetParametersResult,
    },
    srv::{
        DescribeParameters, GetParameterTypes, GetParameters, ListParameters, SetParameters,
        SetParametersAtomically,
    },
};

// phase-382 W1' — the request/reply VALUE types are no longer on any live
// path: the six handlers stream, so nothing constructs one at run time. They
// remain here for the round-trip oracle, which builds a request value,
// serializes it, and deserializes the streamed reply back into the generated
// type to prove the hand-written writes have not drifted.
#[cfg(test)]
pub(crate) use nros_rcl_interfaces::srv::{
    DescribeParametersRequest, DescribeParametersResponse, GetParameterTypesRequest,
    GetParameterTypesResponse, GetParametersRequest, GetParametersResponse, ListParametersRequest,
    ListParametersResponse, SetParametersAtomicallyRequest, SetParametersAtomicallyResponse,
    SetParametersRequest, SetParametersResponse,
};

/// Maximum number of parameters in a request/response
pub const MAX_PARAMS_PER_REQUEST: usize = 64;

// ═══════════════════════════════════════════════════════════════════════════
// TYPE CONVERSIONS: Internal ↔ nros-rcl-interfaces
// ═══════════════════════════════════════════════════════════════════════════

/// Convert internal ParameterValue to rcl_interfaces ParameterValue
/// Why a parameter value could not be converted between the wire form and the
/// node's internal form (issue 0323).
///
/// Both directions used to discard every `push`/`push_str` result, so an
/// over-capacity value was silently TRUNCATED and then reported as success: a
/// ROS 2 client believed it had set a long string or a 40-element array while
/// the node held a prefix. An unrecognised `type_` became `NotSet`. Same class
/// as issues 0223/0224 — a swallowed capacity error turning malformed input
/// into a plausible business value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueConversionError {
    /// The value exceeds this node's compile-time capacity
    /// (`NROS_MAX_STRING_VALUE_LEN`, `NROS_MAX_ARRAY_LEN`, …).
    CapacityExceeded,
    /// `type_` is not a known rcl_interfaces `ParameterType` code.
    UnknownType(u8),
}

impl ValueConversionError {
    /// Human-readable reason for a `SetParametersResult`.
    pub fn reason(&self) -> &'static str {
        match self {
            Self::CapacityExceeded => "Value exceeds this node's parameter capacity",
            Self::UnknownType(_) => "Unknown parameter type",
        }
    }
}

pub fn to_rcl_value(value: &InternalValue) -> Result<ParameterValue, ValueConversionError> {
    let mut result = ParameterValue::default();

    match value {
        InternalValue::NotSet => {
            result.type_ = 0; // PARAMETER_NOT_SET
        }
        InternalValue::Bool(v) => {
            result.type_ = 1; // PARAMETER_BOOL
            result.bool_value = *v;
        }
        InternalValue::Integer(v) => {
            result.type_ = 2; // PARAMETER_INTEGER
            result.integer_value = *v;
        }
        InternalValue::Double(v) => {
            result.type_ = 3; // PARAMETER_DOUBLE
            result.double_value = *v;
        }
        InternalValue::String(v) => {
            result.type_ = 4; // PARAMETER_STRING
            result
                .string_value
                .push_str(v.as_str())
                .map_err(|_| ValueConversionError::CapacityExceeded)?;
        }
        InternalValue::ByteArray(v) => {
            result.type_ = 5; // PARAMETER_BYTE_ARRAY
            for &b in v.iter() {
                result
                    .byte_array_value
                    .push(b)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
        }
        InternalValue::BoolArray(v) => {
            result.type_ = 6; // PARAMETER_BOOL_ARRAY
            for &b in v.iter() {
                result
                    .bool_array_value
                    .push(b)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
        }
        InternalValue::IntegerArray(v) => {
            result.type_ = 7; // PARAMETER_INTEGER_ARRAY
            for &i in v.iter() {
                result
                    .integer_array_value
                    .push(i)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
        }
        InternalValue::DoubleArray(v) => {
            result.type_ = 8; // PARAMETER_DOUBLE_ARRAY
            for &d in v.iter() {
                result
                    .double_array_value
                    .push(d)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
        }
        InternalValue::StringArray(v) => {
            result.type_ = 9; // PARAMETER_STRING_ARRAY
            for s in v.iter() {
                let mut hs = heapless::String::new();
                hs.push_str(s.as_str())
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
                result
                    .string_array_value
                    .push(hs)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
        }
    }

    Ok(result)
}

/// Convert rcl_interfaces ParameterValue to internal ParameterValue
pub fn from_rcl_value(value: &ParameterValue) -> Result<InternalValue, ValueConversionError> {
    match value.type_ {
        0 => Ok(InternalValue::NotSet),
        1 => Ok(InternalValue::Bool(value.bool_value)),
        2 => Ok(InternalValue::Integer(value.integer_value)),
        3 => Ok(InternalValue::Double(value.double_value)),
        4 => {
            let mut s = heapless::String::new();
            s.push_str(value.string_value.as_str())
                .map_err(|_| ValueConversionError::CapacityExceeded)?;
            Ok(InternalValue::String(s))
        }
        5 => {
            let mut v = heapless::Vec::new();
            for &b in value.byte_array_value.iter() {
                v.push(b)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
            Ok(InternalValue::ByteArray(v))
        }
        6 => {
            let mut v = heapless::Vec::new();
            for &b in value.bool_array_value.iter() {
                v.push(b)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
            Ok(InternalValue::BoolArray(v))
        }
        7 => {
            let mut v = heapless::Vec::new();
            for &i in value.integer_array_value.iter() {
                v.push(i)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
            Ok(InternalValue::IntegerArray(v))
        }
        8 => {
            let mut v = heapless::Vec::new();
            for &d in value.double_array_value.iter() {
                v.push(d)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
            Ok(InternalValue::DoubleArray(v))
        }
        9 => {
            let mut v = heapless::Vec::new();
            for s in value.string_array_value.iter() {
                let mut hs = heapless::String::new();
                hs.push_str(s.as_str())
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
                v.push(hs)
                    .map_err(|_| ValueConversionError::CapacityExceeded)?;
            }
            Ok(InternalValue::StringArray(v))
        }
        other => Err(ValueConversionError::UnknownType(other)),
    }
}

/// Convert internal ParameterType to u8 type code
pub fn type_to_u8(param_type: InternalType) -> u8 {
    match param_type {
        InternalType::NotSet => 0,
        InternalType::Bool => 1,
        InternalType::Integer => 2,
        InternalType::Double => 3,
        InternalType::String => 4,
        InternalType::ByteArray => 5,
        InternalType::BoolArray => 6,
        InternalType::IntegerArray => 7,
        InternalType::DoubleArray => 8,
        InternalType::StringArray => 9,
    }
}

/// Convert internal ParameterDescriptor to rcl_interfaces ParameterDescriptor
pub fn to_rcl_descriptor(desc: &InternalDescriptor) -> ParameterDescriptor {
    let mut result = ParameterDescriptor::default();

    let _ = result.name.push_str(desc.name.as_str());
    result.type_ = type_to_u8(desc.param_type);
    let _ = result.description.push_str(desc.description.as_str());
    result.read_only = desc.read_only;
    result.dynamic_typing = desc.dynamic_typing;

    // Convert range constraints
    match &desc.range {
        nros_params::ParameterRange::None => {}
        nros_params::ParameterRange::Integer(range) => {
            let ir = IntegerRange {
                from_value: range.min,
                to_value: range.max,
                step: range.step as u64,
            };
            let _ = result.integer_range.push(ir);
        }
        nros_params::ParameterRange::FloatingPoint(range) => {
            let fr = FloatingPointRange {
                from_value: range.min,
                to_value: range.max,
                step: range.step,
            };
            let _ = result.floating_point_range.push(fr);
        }
    }

    result
}

/// Convert SetParameterResult to rcl_interfaces SetParametersResult
/// Build the `SetParametersResult` for a value that could not be converted
/// (issue 0323) — `successful = false` with a reason naming the cause.
pub fn conversion_failure_result(err: ValueConversionError) -> SetParametersResult {
    let mut reason_str = heapless::String::new();
    let _ = reason_str.push_str(err.reason());
    SetParametersResult {
        successful: false,
        reason: reason_str,
    }
}

pub fn to_rcl_set_result(result: SetParameterResult) -> SetParametersResult {
    let mut reason_str = heapless::String::new();
    let _ = reason_str.push_str(set_result_reason(result));
    SetParametersResult {
        successful: result.is_success(),
        reason: reason_str,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// STREAMING SERVICE HANDLERS (phase-382 W1')
// ═══════════════════════════════════════════════════════════════════════════
//
// Each handler reads the request's fields off the wire and writes the reply's
// fields straight back, so NEITHER side is ever materialised as a value. The
// by-value handlers below (now `#[cfg(test)]`) are kept as the oracle the
// round-trip tests compare against.
//
// Why this is byte-identical rather than merely similar: no `rcl_interfaces`
// message uses a DHEADER, so every generated `Serialize` is plain sequential
// CDR and a hand-written field write lands in the same place. The one real
// risk is DRIFT — a regenerated message gaining, losing or reordering a field
// while these writes stay put — which is exactly what
// `*_streams_like_the_oracle` in the test module is for. Read the generated
// impls in `packages/interfaces/rcl-interfaces/generated/` beside any edit
// here.

use nros_core::{CdrReader, CdrWriter, DeserError, EncodingVersion, SerError};
use nros_params::{MAX_STRING_VALUE_LEN, ParameterRange};
use nros_rmw::TransportError;

/// Capacity of every sequence member in the generated `rcl_interfaces` types
/// (`heapless::Vec<_, 64>`).
///
/// The streaming readers enforce it so an over-long sequence fails the request
/// exactly where the generated `Deserialize` would have — `CapacityExceeded`
/// before the handler sees anything, no reply sent. Drift guard:
/// `wire_capacities_match_the_generated_messages`.
const WIRE_SEQ_CAP: usize = 64;

/// Capacity of every string member in the generated `rcl_interfaces` types
/// (`heapless::String<256>`). Same enforcement, same drift guard.
const WIRE_STRING_CAP: usize = 256;

#[inline]
fn ser_failed(_e: SerError) -> TransportError {
    TransportError::SerializationError
}

#[inline]
fn deser_failed(_e: DeserError) -> TransportError {
    TransportError::DeserializationError
}

/// Read a sequence length, refusing one the generated message could not hold.
#[inline]
fn read_wire_seq_len(reader: &mut CdrReader<'_>) -> Result<usize, DeserError> {
    let len = reader.read_sequence_len()?;
    if len > WIRE_SEQ_CAP {
        return Err(DeserError::CapacityExceeded);
    }
    Ok(len)
}

/// Read a string, refusing one the generated message could not hold. Borrows
/// out of the receive buffer — no copy, which is why `GetParameters` needs no
/// scratch at all.
#[inline]
fn read_wire_string<'r>(reader: &mut CdrReader<'r>) -> Result<&'r str, DeserError> {
    let s = reader.read_string()?;
    if s.len() > WIRE_STRING_CAP {
        return Err(DeserError::CapacityExceeded);
    }
    Ok(s)
}

/// The oracle builds its reply strings with `let _ = push_str(...)` into a
/// `heapless::String<256>`, and `push_str` is all-or-nothing — so a value too
/// long for the wire message reaches the wire as the EMPTY string, not as a
/// prefix. Mirror that rather than truncating.
#[inline]
fn fit_or_empty(s: &str) -> &str {
    if s.len() <= WIRE_STRING_CAP { s } else { "" }
}

/// Re-read the request body from the start.
///
/// Two handlers need two passes over the same request (`ListParameters` needs
/// the prefixes for both the count and the write pass;
/// `SetParametersAtomically` must validate everything before applying
/// anything) and `CdrReader` cannot seek. Taking the remaining bytes as a
/// borrowed slice and building a fresh reader over them is exact rather than
/// approximate: CDR alignment is computed as `pos - origin`, and this slice
/// begins precisely at the original reader's `origin`, so every subsequent
/// alignment decision is the same one. The encoding version is carried over
/// for the same reason — under XCDR2 alignment caps at 4.
#[inline]
fn rewind<'r>(body: &'r [u8], version: EncodingVersion) -> Result<CdrReader<'r>, DeserError> {
    match version {
        EncodingVersion::Xcdr1 => Ok(CdrReader::new(body)),
        EncodingVersion::Xcdr2 => CdrReader::new_at_xcdr2(body, 0),
    }
}

/// Whether a stored value fits the wire message that would carry it.
///
/// Mirrors exactly the cases where `to_rcl_value` returns
/// `CapacityExceeded` — the node's own capacities are build-time knobs
/// (`NROS_MAX_BYTE_ARRAY_LEN` defaults to 256 against a wire cap of 64), so
/// this is not hypothetical.
fn value_fits_wire(value: &InternalValue) -> bool {
    match value {
        InternalValue::String(s) => s.len() <= WIRE_STRING_CAP,
        InternalValue::ByteArray(v) => v.len() <= WIRE_SEQ_CAP,
        InternalValue::BoolArray(v) => v.len() <= WIRE_SEQ_CAP,
        InternalValue::IntegerArray(v) => v.len() <= WIRE_SEQ_CAP,
        InternalValue::DoubleArray(v) => v.len() <= WIRE_SEQ_CAP,
        InternalValue::StringArray(v) => {
            v.len() <= WIRE_SEQ_CAP && v.iter().all(|s| s.len() <= WIRE_STRING_CAP)
        }
        InternalValue::NotSet
        | InternalValue::Bool(_)
        | InternalValue::Integer(_)
        | InternalValue::Double(_) => true,
    }
}

/// Write an internal value in the wire form of `rcl_interfaces/msg/ParameterValue`.
///
/// Field-for-field mirror of the generated `Serialize`: all ten members are
/// written unconditionally in declaration order, and only the one selected by
/// `type_` carries data. The caller must have checked [`value_fits_wire`]
/// first — that is `to_rcl_value`'s error channel, which has no equivalent
/// once the bytes are going out.
fn write_parameter_value(
    writer: &mut CdrWriter<'_>,
    value: &InternalValue,
) -> Result<(), SerError> {
    writer.write_u8(type_to_u8(value.param_type()))?;
    writer.write_bool(match value {
        InternalValue::Bool(v) => *v,
        _ => false,
    })?;
    writer.write_i64(match value {
        InternalValue::Integer(v) => *v,
        _ => 0,
    })?;
    writer.write_f64(match value {
        InternalValue::Double(v) => *v,
        _ => 0.0,
    })?;
    writer.write_string(match value {
        InternalValue::String(s) => s.as_str(),
        _ => "",
    })?;

    match value {
        InternalValue::ByteArray(v) => {
            writer.write_sequence_len(v.len())?;
            for b in v.iter() {
                writer.write_u8(*b)?;
            }
        }
        _ => writer.write_sequence_len(0)?,
    }
    match value {
        InternalValue::BoolArray(v) => {
            writer.write_sequence_len(v.len())?;
            for b in v.iter() {
                writer.write_bool(*b)?;
            }
        }
        _ => writer.write_sequence_len(0)?,
    }
    match value {
        InternalValue::IntegerArray(v) => {
            writer.write_sequence_len(v.len())?;
            for i in v.iter() {
                writer.write_i64(*i)?;
            }
        }
        _ => writer.write_sequence_len(0)?,
    }
    match value {
        InternalValue::DoubleArray(v) => {
            writer.write_sequence_len(v.len())?;
            for d in v.iter() {
                writer.write_f64(*d)?;
            }
        }
        _ => writer.write_sequence_len(0)?,
    }
    match value {
        InternalValue::StringArray(v) => {
            writer.write_sequence_len(v.len())?;
            for s in v.iter() {
                writer.write_string(s.as_str())?;
            }
        }
        _ => writer.write_sequence_len(0)?,
    }
    Ok(())
}

/// Read one wire `ParameterValue` straight into the internal form.
///
/// The wire message always carries all ten members, so all ten are CONSUMED
/// whatever `type_` says — a conversion failure must not desynchronise the
/// reader mid-sequence. Only the selected member is kept, which is what holds
/// the peak at one `InternalValue` instead of the 1.19 MB request.
///
/// The outer `Result` is a wire/framing failure (the whole request dies, as it
/// would have during `Deserialize`); the inner one is issue 0323's per-value
/// rejection, which becomes an unsuccessful `SetParametersResult`.
fn read_parameter_value(
    reader: &mut CdrReader<'_>,
) -> Result<Result<InternalValue, ValueConversionError>, DeserError> {
    let type_ = reader.read_u8()?;
    let bool_value = reader.read_bool()?;
    let integer_value = reader.read_i64()?;
    let double_value = reader.read_f64()?;
    let string_value = read_wire_string(reader)?;

    let mut err: Option<ValueConversionError> = None;
    let mut value = match type_ {
        0 => InternalValue::NotSet,
        1 => InternalValue::Bool(bool_value),
        2 => InternalValue::Integer(integer_value),
        3 => InternalValue::Double(double_value),
        4 => match InternalValue::from_string(string_value) {
            Some(v) => v,
            None => {
                err = Some(ValueConversionError::CapacityExceeded);
                InternalValue::NotSet
            }
        },
        5 => InternalValue::ByteArray(heapless::Vec::new()),
        6 => InternalValue::BoolArray(heapless::Vec::new()),
        7 => InternalValue::IntegerArray(heapless::Vec::new()),
        8 => InternalValue::DoubleArray(heapless::Vec::new()),
        9 => InternalValue::StringArray(heapless::Vec::new()),
        other => {
            err = Some(ValueConversionError::UnknownType(other));
            InternalValue::NotSet
        }
    };

    for _ in 0..read_wire_seq_len(reader)? {
        let b = reader.read_u8()?;
        if let InternalValue::ByteArray(v) = &mut value
            && v.push(b).is_err()
        {
            err = err.or(Some(ValueConversionError::CapacityExceeded));
        }
    }
    for _ in 0..read_wire_seq_len(reader)? {
        let b = reader.read_bool()?;
        if let InternalValue::BoolArray(v) = &mut value
            && v.push(b).is_err()
        {
            err = err.or(Some(ValueConversionError::CapacityExceeded));
        }
    }
    for _ in 0..read_wire_seq_len(reader)? {
        let i = reader.read_i64()?;
        if let InternalValue::IntegerArray(v) = &mut value
            && v.push(i).is_err()
        {
            err = err.or(Some(ValueConversionError::CapacityExceeded));
        }
    }
    for _ in 0..read_wire_seq_len(reader)? {
        let d = reader.read_f64()?;
        if let InternalValue::DoubleArray(v) = &mut value
            && v.push(d).is_err()
        {
            err = err.or(Some(ValueConversionError::CapacityExceeded));
        }
    }
    for _ in 0..read_wire_seq_len(reader)? {
        let s = read_wire_string(reader)?;
        if let InternalValue::StringArray(v) = &mut value {
            let mut hs = heapless::String::<MAX_STRING_VALUE_LEN>::new();
            if hs.push_str(s).is_err() || v.push(hs).is_err() {
                err = err.or(Some(ValueConversionError::CapacityExceeded));
            }
        }
    }

    match err {
        Some(e) => Ok(Err(e)),
        None => Ok(Ok(value)),
    }
}

/// Write one `rcl_interfaces/msg/SetParametersResult`.
#[inline]
fn write_set_result(
    writer: &mut CdrWriter<'_>,
    successful: bool,
    reason: &str,
) -> Result<(), SerError> {
    writer.write_bool(successful)?;
    writer.write_string(fit_or_empty(reason))
}

/// The reason string every `SetParametersResult` carries for each outcome —
/// the ONE table, used by the streaming handlers and `to_rcl_set_result`.
#[inline]
fn set_result_reason(result: SetParameterResult) -> &'static str {
    match result {
        SetParameterResult::Success => "",
        SetParameterResult::ReadOnly => "Parameter is read-only",
        SetParameterResult::TypeMismatch => "Type mismatch",
        SetParameterResult::OutOfRange => "Value out of range",
        SetParameterResult::NotFound => "Parameter not found",
        SetParameterResult::StorageFull => "Parameter storage full",
        // Issue 1151 — rclrs's wording, verbatim.
        SetParameterResult::Undeclared => UNDECLARED_REASON,
        SetParameterResult::InvalidRange => "Invalid range",
    }
}

/// What a remote set reports for a name the node never declared (issue
/// 1151). rclrs `parameter.rs` `validate_parameter_setting`, verbatim.
pub const UNDECLARED_REASON: &str =
    "Parameter was not declared and undeclared parameters are not allowed";

/// Apply one already-converted parameter, exactly as `handle_set_parameters` does.
///
/// Issue 1151 — the "declare it if missing" branch that used to live here is
/// gone; `ParameterServer::apply` is the single place that decides, and it
/// refuses an undeclared name unless the server was told to allow it.
#[inline]
fn apply_one(server: &mut ParameterServer, name: &str, value: InternalValue) -> SetParameterResult {
    server.apply(name, value)
}

/// `GetParameters` — no scratch at all.
///
/// Read a name (borrowed out of the receive buffer), look the value up in the
/// store, write its fields. No wire value is ever constructed, in either
/// direction.
pub(crate) fn stream_get_parameters(
    server: &ParameterServer,
    reader: &mut CdrReader<'_>,
    writer: &mut CdrWriter<'_>,
) -> Result<(), TransportError> {
    let count = read_wire_seq_len(reader).map_err(deser_failed)?;
    writer.write_sequence_len(count).map_err(ser_failed)?;

    for _ in 0..count {
        let name = read_wire_string(reader).map_err(deser_failed)?;
        // issue 0323 — an unknown parameter, and a stored value too large for
        // the RESPONSE message, both reply NOT_SET. GetParameters has no
        // per-parameter error channel, so that is the only in-protocol way to
        // avoid handing back a plausible-looking truncation.
        match server.get(name).filter(|v| value_fits_wire(v)) {
            Some(value) => write_parameter_value(writer, value),
            None => write_parameter_value(writer, &InternalValue::NotSet),
        }
        .map_err(ser_failed)?;
    }
    Ok(())
}

/// `SetParameters` — one parameter at a time.
///
/// The reply has exactly one result per requested parameter, so the sequence
/// length is known from the request before any element is written.
pub(crate) fn stream_set_parameters(
    server: &mut ParameterServer,
    reader: &mut CdrReader<'_>,
    writer: &mut CdrWriter<'_>,
) -> Result<(), TransportError> {
    let count = read_wire_seq_len(reader).map_err(deser_failed)?;
    writer.write_sequence_len(count).map_err(ser_failed)?;

    for _ in 0..count {
        let name = read_wire_string(reader).map_err(deser_failed)?;
        // issue 0323 — an over-capacity or unknown-type value is REJECTED, not
        // truncated and then reported as success.
        match read_parameter_value(reader).map_err(deser_failed)? {
            Err(e) => write_set_result(writer, false, e.reason()),
            Ok(value) => {
                let result = apply_one(server, name, value);
                write_set_result(writer, result.is_success(), set_result_reason(result))
            }
        }
        .map_err(ser_failed)?;
    }
    Ok(())
}

/// `SetParametersAtomically` — validate every parameter, then apply every one.
///
/// The only handler that genuinely needs the request twice: nothing may be
/// applied until everything has been checked, and the check needs each value.
/// [`rewind`] gives the second pass without copying anything and without a
/// scratch — peak stays at the one `InternalValue` the pass in flight holds.
pub(crate) fn stream_set_parameters_atomically(
    server: &mut ParameterServer,
    reader: &mut CdrReader<'_>,
    writer: &mut CdrWriter<'_>,
) -> Result<(), TransportError> {
    let version = reader.version();
    // `rewind` is only exact while the slice begins at the reader's alignment
    // origin, i.e. while nothing has been read yet. Breaking that shifts CDR
    // padding with no other symptom, so enforce it rather than document it: a
    // future field read added above this line fails loudly here.
    if !reader.is_at_origin() {
        return Err(deser_failed(DeserError::UnexpectedEof));
    }
    let body = reader
        .read_bytes(reader.remaining())
        .map_err(deser_failed)?;

    // Pass 1 — validate. Deliberately does NOT stop at the first failure the
    // way the by-value handler's `break` does: reading to the end keeps a
    // malformed TAIL failing the request (no reply at all), which is what
    // deserializing the whole message used to guarantee.
    let mut pass = rewind(body, version).map_err(deser_failed)?;
    let count = read_wire_seq_len(&mut pass).map_err(deser_failed)?;
    let mut can_set_all = true;
    for _ in 0..count {
        let name = read_wire_string(&mut pass).map_err(deser_failed)?;
        let Ok(value) = read_parameter_value(&mut pass).map_err(deser_failed)? else {
            // issue 0323 — the truncation used to happen inside this
            // pre-check, so an over-capacity value passed validation and
            // applied as a shortened value with `successful = true`.
            can_set_all = false;
            continue;
        };
        // The same verdict `apply_one` would reach — read-only, type,
        // min/max/step, and issue 1151's undeclared refusal — without
        // applying it. One predicate, so the pre-check cannot pass a value the
        // apply would then refuse.
        if !server.check_apply(name, &value).is_success() {
            can_set_all = false;
        }
    }

    if can_set_all {
        // Pass 2 — apply. Every value converted in pass 1; skip rather than
        // unwrap so a future divergence cannot panic mid-batch.
        let mut pass = rewind(body, version).map_err(deser_failed)?;
        let count = read_wire_seq_len(&mut pass).map_err(deser_failed)?;
        for _ in 0..count {
            let name = read_wire_string(&mut pass).map_err(deser_failed)?;
            let Ok(value) = read_parameter_value(&mut pass).map_err(deser_failed)? else {
                continue;
            };
            let _ = apply_one(server, name, value);
        }
        write_set_result(writer, true, "").map_err(ser_failed)
    } else {
        write_set_result(writer, false, "One or more parameters could not be set")
            .map_err(ser_failed)
    }
}

/// `ListParameters` — the one reply whose sequence length is not the request's.
///
/// Both output sequences are derived from the STORE, so their lengths are not
/// known until it has been walked. The store is cheap to walk twice, so the
/// only scratch is the borrowed prefixes: `&str` into the receive buffer for
/// the request's filters, `&str` into the store for the deduplicated reply
/// prefixes. Bounded by the wire capacity, no copies, no `alloc`.
pub(crate) fn stream_list_parameters<'r>(
    server: &ParameterServer,
    reader: &mut CdrReader<'r>,
    writer: &mut CdrWriter<'_>,
) -> Result<(), TransportError> {
    let prefix_count = read_wire_seq_len(reader).map_err(deser_failed)?;
    let mut filters: heapless::Vec<&'r str, WIRE_SEQ_CAP> = heapless::Vec::new();
    for _ in 0..prefix_count {
        let p = read_wire_string(reader).map_err(deser_failed)?;
        filters
            .push(p)
            .map_err(|_| TransportError::DeserializationError)?;
    }
    let depth = reader.read_u64().map_err(deser_failed)?;

    let selected = |name: &str| -> bool {
        let matches_prefix =
            filters.is_empty() || filters.iter().any(|prefix| name.starts_with(*prefix));
        // depth 0 = unlimited
        matches_prefix && (depth == 0 || name.matches('.').count() as u64 <= depth)
    };

    // Pass 1 over the store: how many names, and which prefixes. Both are
    // capped at the wire capacity, which is what the by-value handler's
    // `let _ = push(..)` into a `heapless::Vec<_, 64>` silently did.
    let mut name_count = 0usize;
    let mut prefixes: heapless::Vec<&str, WIRE_SEQ_CAP> = heapless::Vec::new();
    for param in server.iter() {
        let name = param.name.as_str();
        if !selected(name) {
            continue;
        }
        if name_count < WIRE_SEQ_CAP {
            name_count += 1;
        }
        if let Some(dot) = name.rfind('.') {
            let prefix = &name[..dot];
            if !prefixes.contains(&prefix) {
                let _ = prefixes.push(prefix);
            }
        }
    }

    // ListParametersResponse is a bare ListParametersResult: names, then prefixes.
    writer.write_sequence_len(name_count).map_err(ser_failed)?;
    let mut written = 0usize;
    for param in server.iter() {
        if written == name_count {
            break;
        }
        let name = param.name.as_str();
        if selected(name) {
            writer
                .write_string(fit_or_empty(name))
                .map_err(ser_failed)?;
            written += 1;
        }
    }
    writer
        .write_sequence_len(prefixes.len())
        .map_err(ser_failed)?;
    for prefix in prefixes.iter() {
        writer
            .write_string(fit_or_empty(prefix))
            .map_err(ser_failed)?;
    }
    Ok(())
}

/// Write one `rcl_interfaces/msg/ParameterDescriptor` from the internal form.
fn write_descriptor(writer: &mut CdrWriter<'_>, desc: &InternalDescriptor) -> Result<(), SerError> {
    writer.write_string(fit_or_empty(desc.name.as_str()))?;
    writer.write_u8(type_to_u8(desc.param_type))?;
    writer.write_string(fit_or_empty(desc.description.as_str()))?;
    // `additional_constraints` — the internal descriptor has no such field, so
    // `to_rcl_descriptor` leaves it at the default.
    writer.write_string("")?;
    writer.write_bool(desc.read_only)?;
    writer.write_bool(desc.dynamic_typing)?;
    match &desc.range {
        ParameterRange::FloatingPoint(range) => {
            writer.write_sequence_len(1)?;
            writer.write_f64(range.min)?;
            writer.write_f64(range.max)?;
            writer.write_f64(range.step)?;
        }
        _ => writer.write_sequence_len(0)?,
    }
    match &desc.range {
        ParameterRange::Integer(range) => {
            writer.write_sequence_len(1)?;
            writer.write_i64(range.min)?;
            writer.write_i64(range.max)?;
            writer.write_u64(range.step as u64)?;
        }
        _ => writer.write_sequence_len(0)?,
    }
    Ok(())
}

/// The descriptor for a parameter that has none: name + type, everything else
/// default. `type_ = 0` (NOT_SET) when the name is unknown entirely.
fn write_minimal_descriptor(
    writer: &mut CdrWriter<'_>,
    name: &str,
    type_: u8,
) -> Result<(), SerError> {
    writer.write_string(fit_or_empty(name))?;
    writer.write_u8(type_)?;
    writer.write_string("")?;
    writer.write_string("")?;
    writer.write_bool(false)?;
    writer.write_bool(false)?;
    writer.write_sequence_len(0)?;
    writer.write_sequence_len(0)?;
    Ok(())
}

/// `DescribeParameters` — one descriptor per requested name.
pub(crate) fn stream_describe_parameters(
    server: &ParameterServer,
    reader: &mut CdrReader<'_>,
    writer: &mut CdrWriter<'_>,
) -> Result<(), TransportError> {
    let count = read_wire_seq_len(reader).map_err(deser_failed)?;
    writer.write_sequence_len(count).map_err(ser_failed)?;

    for _ in 0..count {
        let name = read_wire_string(reader).map_err(deser_failed)?;
        if let Some(desc) = server.get_descriptor(name) {
            write_descriptor(writer, desc)
        } else if let Some(param) = server.get_parameter(name) {
            write_minimal_descriptor(writer, name, type_to_u8(param.param_type()))
        } else {
            write_minimal_descriptor(writer, name, 0)
        }
        .map_err(ser_failed)?;
    }
    Ok(())
}

/// `GetParameterTypes` — one type code per requested name.
pub(crate) fn stream_get_parameter_types(
    server: &ParameterServer,
    reader: &mut CdrReader<'_>,
    writer: &mut CdrWriter<'_>,
) -> Result<(), TransportError> {
    let count = read_wire_seq_len(reader).map_err(deser_failed)?;
    writer.write_sequence_len(count).map_err(ser_failed)?;

    for _ in 0..count {
        let name = read_wire_string(reader).map_err(deser_failed)?;
        // NOT_SET for an unknown parameter.
        let code = server.get_type(name).map(type_to_u8).unwrap_or(0);
        writer.write_u8(code).map_err(ser_failed)?;
    }
    Ok(())
}

// ═══════════════════════════════════════════════════════════════════════════
// BY-VALUE SERVICE HANDLERS — TEST ORACLE ONLY (phase-382 W1')
// ═══════════════════════════════════════════════════════════════════════════
//
// These are the handlers that shipped until phase-382 W1'. They are kept, not
// deleted, because they are the ONLY independent statement of what these six
// replies should contain: `*_streams_like_the_oracle` builds a request, runs
// the streaming handler over its bytes, and compares the result against what
// these produce. Hand-written CDR writes drift silently otherwise.
//
// They are not on any live path. Each deserializes its request BY VALUE into a
// stack local — `SetParametersRequest` measures 1,192,968 bytes — and boxes
// its reply, which is the pair of problems W1' removed.

/// Handle GetParameters service request
///
/// Returns `Box<Response>` because the response type contains large heapless arrays
/// (~1MB+ for Vec<ParameterValue, 64>) that would overflow the stack.
/// Test oracle only (phase-382 W1'); the live path is the streaming handler above.
#[cfg(test)]
pub fn handle_get_parameters(
    server: &ParameterServer,
    request: &GetParametersRequest,
) -> Box<GetParametersResponse> {
    let mut response = Box::new(GetParametersResponse::default());

    for name in request.names.iter() {
        // issue 0323 — a stored value that does not fit the RESPONSE message's
        // capacity replies NOT_SET rather than a silently shortened value.
        // GetParameters has no per-parameter error channel, so "not set" is
        // the only in-protocol way to avoid handing back a plausible-looking
        // truncation; a client can tell that apart from a real value, which it
        // could not do before.
        let value = server
            .get(name.as_str())
            .and_then(|v| to_rcl_value(v).ok())
            .unwrap_or_else(|| {
                // Return NOT_SET for unknown parameters
                ParameterValue {
                    type_: 0,
                    ..Default::default()
                }
            });
        let _ = response.values.push(value);
    }

    response
}

/// Handle SetParameters service request
///
/// Returns `Box<Response>` because request/response types contain large heapless arrays.
/// Test oracle only (phase-382 W1'); the live path is the streaming handler above.
#[cfg(test)]
pub fn handle_set_parameters(
    server: &mut ParameterServer,
    request: &SetParametersRequest,
) -> Box<SetParametersResponse> {
    let mut response = Box::new(SetParametersResponse::default());

    for param in request.parameters.iter() {
        // issue 0323 — an over-capacity or unknown-type value is REJECTED
        // here. It used to be truncated and then reported as success, so a
        // client believed it had set a long string or a 40-element array while
        // the node held a prefix.
        let value = match from_rcl_value(&param.value) {
            Ok(v) => v,
            Err(e) => {
                let _ = response.results.push(conversion_failure_result(e));
                continue;
            }
        };
        let result = apply_one(server, param.name.as_str(), value);
        let _ = response.results.push(to_rcl_set_result(result));
    }

    response
}

/// Handle SetParametersAtomically service request
///
/// Returns `Box<Response>` because request/response types contain large heapless arrays.
/// Test oracle only (phase-382 W1'); the live path is the streaming handler above.
#[cfg(test)]
pub fn handle_set_parameters_atomically(
    server: &mut ParameterServer,
    request: &SetParametersAtomicallyRequest,
) -> Box<SetParametersAtomicallyResponse> {
    let mut response = Box::new(SetParametersAtomicallyResponse::default());

    // First, validate all parameters can be set
    let mut can_set_all = true;
    for param in request.parameters.iter() {
        // issue 0323 — the truncation used to happen INSIDE this pre-check, so
        // an over-capacity value passed validation and applied as a shortened
        // value with `successful = true`. It now fails the batch, which is the
        // atomic contract.
        let Ok(value) = from_rcl_value(&param.value) else {
            can_set_all = false;
            break;
        };

        // Check if setting would succeed — the same predicate the apply uses.
        if !server.check_apply(param.name.as_str(), &value).is_success() {
            can_set_all = false;
            break;
        }
    }

    if can_set_all {
        // Set all parameters
        for param in request.parameters.iter() {
            // The pre-check above already proved every value converts; skip
            // rather than unwrap so a future divergence cannot panic mid-batch.
            let Ok(value) = from_rcl_value(&param.value) else {
                continue;
            };
            let _ = apply_one(server, param.name.as_str(), value);
        }
        response.result.successful = true;
    } else {
        response.result.successful = false;
        let _ = response
            .result
            .reason
            .push_str("One or more parameters could not be set");
    }

    response
}

/// Handle ListParameters service request
///
/// Returns `Box<Response>` because response type contains large heapless arrays (~32KB).
/// Test oracle only (phase-382 W1'); the live path is the streaming handler above.
#[cfg(test)]
pub fn handle_list_parameters(
    server: &ParameterServer,
    request: &ListParametersRequest,
) -> Box<ListParametersResponse> {
    let mut response = Box::new(ListParametersResponse::default());

    // Collect parameter names
    let mut names: heapless::Vec<heapless::String<256>, MAX_PARAMS_PER_REQUEST> =
        heapless::Vec::new();
    let mut prefixes: heapless::Vec<heapless::String<256>, MAX_PARAMS_PER_REQUEST> =
        heapless::Vec::new();

    for param in server.iter() {
        let name = param.name.as_str();

        // Check prefix filter
        let matches_prefix = if request.prefixes.is_empty() {
            true
        } else {
            request
                .prefixes
                .iter()
                .any(|prefix| name.starts_with(prefix.as_str()))
        };

        if matches_prefix {
            // Check depth filter (0 = unlimited)
            let should_include = if request.depth == 0 {
                true
            } else {
                // Count dots in the name after the prefix
                let depth = name.matches('.').count() as u64;
                depth <= request.depth
            };

            if should_include {
                let mut n = heapless::String::new();
                let _ = n.push_str(name);
                let _ = names.push(n);

                // Extract prefix (everything up to the last dot)
                if let Some(dot_pos) = name.rfind('.') {
                    let prefix = &name[..dot_pos];
                    let mut p = heapless::String::new();
                    let _ = p.push_str(prefix);
                    // Only add if not already present
                    if !prefixes.iter().any(|existing| existing.as_str() == prefix) {
                        let _ = prefixes.push(p);
                    }
                }
            }
        }
    }

    response.result.names = names;
    response.result.prefixes = prefixes;

    response
}

/// Handle DescribeParameters service request
///
/// Returns `Box<Response>` because response type contains large heapless arrays (~50KB).
/// Test oracle only (phase-382 W1'); the live path is the streaming handler above.
#[cfg(test)]
pub fn handle_describe_parameters(
    server: &ParameterServer,
    request: &DescribeParametersRequest,
) -> Box<DescribeParametersResponse> {
    let mut response = Box::new(DescribeParametersResponse::default());

    for name in request.names.iter() {
        let descriptor = if let Some(desc) = server.get_descriptor(name.as_str()) {
            to_rcl_descriptor(desc)
        } else if let Some(param) = server.get_parameter(name.as_str()) {
            // Create a minimal descriptor from the parameter
            let mut d = ParameterDescriptor::default();
            let _ = d.name.push_str(name.as_str());
            d.type_ = type_to_u8(param.param_type());
            d
        } else {
            // Unknown parameter - return empty descriptor
            let mut d = ParameterDescriptor::default();
            let _ = d.name.push_str(name.as_str());
            d.type_ = 0; // NOT_SET
            d
        };
        let _ = response.descriptors.push(descriptor);
    }

    response
}

/// Handle GetParameterTypes service request
///
/// Returns `Box<Response>` for API consistency with other handlers.
/// Test oracle only (phase-382 W1'); the live path is the streaming handler above.
#[cfg(test)]
pub fn handle_get_parameter_types(
    server: &ParameterServer,
    request: &GetParameterTypesRequest,
) -> Box<GetParameterTypesResponse> {
    let mut response = Box::new(GetParameterTypesResponse::default());

    for name in request.names.iter() {
        let type_code = server.get_type(name.as_str()).map(type_to_u8).unwrap_or(0); // NOT_SET for unknown
        let _ = response.types.push(type_code);
    }

    response
}

// ═══════════════════════════════════════════════════════════════════════════
// PARAMETER SERVICE SERVERS
// ═══════════════════════════════════════════════════════════════════════════

use crate::executor::{EmbeddedServiceServer, NodeError};

// PARAM_SERVICE_BUFFER_SIZE is generated by build.rs from the
// NROS_PARAM_SERVICE_BUFFER_SIZE env var (default 4096).
pub use crate::config::PARAM_SERVICE_BUFFER_SIZE;

/// How many zenoh queryables the ROS parameter services claim on a node.
///
/// Issue 0827 — a service server IS a queryable, so this number is a term in
/// every service-buffer pool the RMW sizes. It had SIX spellings and no
/// definition: the count of `create_param_srv::<_>` statements in
/// `executor/spin.rs`, two doc comments, `nros-zpico-build`'s default-picking
/// comment, the RMW's own exhaustion message, and CLAUDE.md. Two of them had
/// already drifted to the wrong value for the lifecycle sibling.
///
/// Kept BESIDE the code that creates them, not in the RMW: only the runtime
/// knows it creates these on the node's behalf. Deriving the number from the
/// user's entity set is issue 0460's defect — codegen sees the application's
/// services and never these.
///
/// Gated with the same `param-services` feature that compiles the servers, so
/// an image without it contributes zero rather than eleven.
/// `check-infra-queryable-counts` holds it to the number of creation sites.
pub const PARAM_SERVICE_QUERYABLES: usize = 6;

/// Type alias for a parameter service server with standard buffer sizes.
type ParamServer<Svc> =
    EmbeddedServiceServer<Svc, PARAM_SERVICE_BUFFER_SIZE, PARAM_SERVICE_BUFFER_SIZE>;

/// Holds the 6 ROS 2 parameter service servers for a node.
///
/// These servers handle `ros2 param` CLI interactions:
/// - `get_parameters` / `set_parameters` / `set_parameters_atomically`
/// - `list_parameters` / `describe_parameters` / `get_parameter_types`
///
/// Boxed when stored in executor to avoid 48KB+ on the stack
/// (6 servers × 8KB buffers each).
pub struct ParameterServiceServers {
    get_parameters: ParamServer<GetParameters>,
    set_parameters: ParamServer<SetParameters>,
    set_parameters_atomically: ParamServer<SetParametersAtomically>,
    list_parameters: ParamServer<ListParameters>,
    describe_parameters: ParamServer<DescribeParameters>,
    get_parameter_types: ParamServer<GetParameterTypes>,
}

impl ParameterServiceServers {
    /// Create a new set of parameter service servers
    pub(crate) fn new(
        get_parameters: ParamServer<GetParameters>,
        set_parameters: ParamServer<SetParameters>,
        set_parameters_atomically: ParamServer<SetParametersAtomically>,
        list_parameters: ParamServer<ListParameters>,
        describe_parameters: ParamServer<DescribeParameters>,
        get_parameter_types: ParamServer<GetParameterTypes>,
    ) -> Self {
        Self {
            get_parameters,
            set_parameters,
            set_parameters_atomically,
            list_parameters,
            describe_parameters,
            get_parameter_types,
        }
    }

    /// Process all parameter service servers, handling any pending requests.
    ///
    /// Uses split borrows: the `server` parameter provides mutable access to the
    /// `ParameterServer` while `self` provides access to the service servers.
    ///
    /// Returns the number of requests handled.
    pub(crate) fn process(&mut self, server: &mut ParameterServer) -> Result<usize, NodeError> {
        let mut count = 0;

        // phase-382 W1' — every one of these STREAMS. `handle_request_boxed`
        // boxed only the reply and left the request as a by-value stack local,
        // so `ros2 param set` put 1.19 MB on the calling task's stack.
        if self
            .get_parameters
            .handle_request_raw(|reader, writer| stream_get_parameters(server, reader, writer))?
        {
            count += 1;
        }

        if self
            .set_parameters
            .handle_request_raw(|reader, writer| stream_set_parameters(server, reader, writer))?
        {
            count += 1;
        }

        if self
            .set_parameters_atomically
            .handle_request_raw(|reader, writer| {
                stream_set_parameters_atomically(server, reader, writer)
            })?
        {
            count += 1;
        }

        if self
            .list_parameters
            .handle_request_raw(|reader, writer| stream_list_parameters(server, reader, writer))?
        {
            count += 1;
        }

        if self
            .describe_parameters
            .handle_request_raw(|reader, writer| {
                stream_describe_parameters(server, reader, writer)
            })?
        {
            count += 1;
        }

        if self
            .get_parameter_types
            .handle_request_raw(|reader, writer| {
                stream_get_parameter_types(server, reader, writer)
            })?
        {
            count += 1;
        }

        Ok(count)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TYPE-ERASED PARAMETER PROCESSING (for Executor integration)
// ═══════════════════════════════════════════════════════════════════════════

/// Type-erased trait for processing parameter services inside `spin_once()`.
///
/// The concrete `ParameterServiceServers` is stored behind a
/// `Box<dyn ParamServiceProcessor>` so the executor can call `process()`
/// without coupling to the parameter service implementation.
pub(crate) trait ParamServiceProcessor {
    fn process_services(&mut self, server: &mut ParameterServer) -> Result<usize, NodeError>;
}

impl ParamServiceProcessor for ParameterServiceServers {
    fn process_services(&mut self, server: &mut ParameterServer) -> Result<usize, NodeError> {
        self.process(server)
    }
}

/// Holds parameter server state for the executor.
///
/// Stored outside the arena so it doesn't consume `MAX_CBS` slots.
pub(crate) struct ParamState<'s> {
    /// phase-382 W2' — the server BORROWS its slot table now, so `ParamState`
    /// carries the storage's lifetime. `'s` is the executor's own borrow
    /// lifetime, which is what W3' will hand the carved table under.
    pub(crate) server: ParameterServer<'s>,
    /// Issue 0745 — `None` until `register_parameter_services` attaches the
    /// six service servers. The STORE half exists earlier: launch-param
    /// seeding (`declare_parameter`) runs BEFORE any node is constructed —
    /// service servers cannot be created that early, but initial values
    /// must be, or ctor-read parameters silently use compiled defaults.
    pub(crate) services: Option<Box<dyn ParamServiceProcessor>>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nros_rcl_interfaces::msg::Parameter;

    /// A parameter server over freshly leaked heap storage.
    ///
    /// phase-382 W2' — the store is CALLER-OWNED now, so a test that wants a
    /// server has to place its slots somewhere. The factories below return
    /// the server by value, so the slots must outlive the frame that built
    /// them: leak one allocation, exactly as `Executor::leak_parameter_storage`
    /// does until W3' carves the table out of the executor backing.
    /// Initialising through the pointer keeps the 285 KB off the test
    /// thread's stack (issue 0756) rather than merely getting away with it.
    #[inline(never)]
    fn leaked_server() -> ParameterServer<'static> {
        let mut uninit = Box::<nros_params::ParameterStorage>::new_uninit();
        // Safety: `new_uninit` is correctly sized and aligned for this type,
        // and `init_in_place` writes every slot before `assume_init` observes
        // it.
        let table = unsafe {
            nros_params::ParameterStorage::init_in_place(uninit.as_mut_ptr());
            Box::leak(uninit.assume_init()).as_table()
        };
        ParameterServer::new_in(table)
    }

    #[test]
    fn test_value_conversion_bool() {
        let internal = InternalValue::Bool(true);
        let rcl = to_rcl_value(&internal).expect("value fits the wire message");
        assert_eq!(rcl.type_, 1);
        assert!(rcl.bool_value);

        let back = from_rcl_value(&rcl).expect("valid wire value");
        assert_eq!(back.as_bool(), Some(true));
    }

    #[test]
    fn test_value_conversion_integer() {
        let internal = InternalValue::Integer(42);
        let rcl = to_rcl_value(&internal).expect("value fits the wire message");
        assert_eq!(rcl.type_, 2);
        assert_eq!(rcl.integer_value, 42);

        let back = from_rcl_value(&rcl).expect("valid wire value");
        assert_eq!(back.as_integer(), Some(42));
    }

    #[test]
    fn test_value_conversion_double() {
        let internal = InternalValue::Double(1.25);
        let rcl = to_rcl_value(&internal).expect("value fits the wire message");
        assert_eq!(rcl.type_, 3);
        assert!((rcl.double_value - 1.25).abs() < 0.001);

        let back = from_rcl_value(&rcl).expect("valid wire value");
        assert!((back.as_double().unwrap() - 1.25).abs() < 0.001);
    }

    #[test]
    fn test_value_conversion_string() {
        let internal = InternalValue::from_string("hello").unwrap();
        let rcl = to_rcl_value(&internal).expect("value fits the wire message");
        assert_eq!(rcl.type_, 4);
        assert_eq!(rcl.string_value.as_str(), "hello");

        let back = from_rcl_value(&rcl).expect("valid wire value");
        assert_eq!(back.as_string(), Some("hello"));
    }

    #[test]
    fn test_get_parameters_handler() {
        use alloc::boxed::Box;

        let mut server = leaked_server();
        server.declare("speed", InternalValue::Double(1.0));
        server.declare("enabled", InternalValue::Bool(true));

        // Use Box for request due to large heapless::Vec size (~1MB+)
        // Handler returns Box<Response> internally
        let mut request = Box::new(GetParametersRequest::default());
        let mut n1 = heapless::String::new();
        n1.push_str("speed").unwrap();
        let mut n2 = heapless::String::new();
        n2.push_str("enabled").unwrap();
        request.names.push(n1).unwrap();
        request.names.push(n2).unwrap();

        let response = handle_get_parameters(&server, &request);
        assert_eq!(response.values.len(), 2);
        assert_eq!(response.values[0].type_, 3); // DOUBLE
        assert_eq!(response.values[1].type_, 1); // BOOL
    }

    #[test]
    fn test_set_parameters_handler() {
        use alloc::boxed::Box;

        let mut server = leaked_server();
        server.declare("speed", InternalValue::Double(1.0));

        // Use Box for request due to large heapless::Vec size (~1MB+)
        // Handler returns Box<Response> internally
        let mut request = Box::new(SetParametersRequest::default());
        let mut param = Box::new(Parameter::default());
        param.name.push_str("speed").unwrap();
        param.value.type_ = 3; // DOUBLE
        param.value.double_value = 2.5;
        request.parameters.push(*param).unwrap();

        let response = handle_set_parameters(&mut server, &request);
        assert_eq!(response.results.len(), 1);
        assert!(response.results[0].successful);

        assert_eq!(server.get_double("speed"), Some(2.5));
    }

    #[test]
    fn test_list_parameters_handler() {
        use alloc::boxed::Box;

        let mut server = leaked_server();
        server.declare("robot.speed", InternalValue::Double(1.0));
        server.declare("robot.name", InternalValue::from_string("bot1").unwrap());
        server.declare("sensor.range", InternalValue::Double(10.0));

        // Use Box for request due to large heapless::Vec size
        let request = Box::new(ListParametersRequest::default());
        let response = handle_list_parameters(&server, &request);

        assert_eq!(response.result.names.len(), 3);
    }

    #[test]
    fn test_get_parameter_types_handler() {
        use alloc::boxed::Box;

        let mut server = leaked_server();
        server.declare("speed", InternalValue::Double(1.0));
        server.declare("count", InternalValue::Integer(5));

        // Use Box for request due to large heapless::Vec size
        let mut request = Box::new(GetParameterTypesRequest::default());
        let mut n1 = heapless::String::new();
        n1.push_str("speed").unwrap();
        let mut n2 = heapless::String::new();
        n2.push_str("count").unwrap();
        request.names.push(n1).unwrap();
        request.names.push(n2).unwrap();

        let response = handle_get_parameter_types(&server, &request);
        assert_eq!(response.types.len(), 2);
        assert_eq!(response.types[0], 3); // DOUBLE
        assert_eq!(response.types[1], 2); // INTEGER
    }

    /// issue 0323 — a wire array larger than the node's `MAX_ARRAY_LEN` is
    /// REJECTED, not silently shortened. The rcl_interfaces message holds 64
    /// integers while the internal value holds `MAX_ARRAY_LEN` (32 by
    /// default), so a 40-element array — the issue's own example — used to
    /// arrive as a 32-element prefix reported as success.
    #[test]
    fn oversize_integer_array_is_rejected_not_truncated() {
        let mut wire = ParameterValue {
            type_: 7,
            ..Default::default()
        };
        for i in 0..40i64 {
            wire.integer_array_value
                .push(i)
                .expect("wire message holds 64");
        }
        assert!(
            matches!(
                from_rcl_value(&wire),
                Err(ValueConversionError::CapacityExceeded)
            ),
            "a 40-element array must not be accepted as a truncated 32-element one"
        );
    }

    /// An unrecognised `type_` used to become `NotSet` — indistinguishable
    /// from a client deliberately clearing the parameter.
    #[test]
    fn unknown_parameter_type_is_an_error_not_notset() {
        let wire = ParameterValue {
            type_: 99,
            ..Default::default()
        };
        assert!(matches!(
            from_rcl_value(&wire),
            Err(ValueConversionError::UnknownType(99))
        ));
    }

    /// The rejection reaches the wire as `successful = false` with a reason,
    /// which is the half a client actually observes.
    #[test]
    fn conversion_failure_replies_unsuccessful_with_a_reason() {
        let r = conversion_failure_result(ValueConversionError::CapacityExceeded);
        assert!(!r.successful);
        assert!(
            r.reason.as_str().contains("capacity"),
            "reason should name the cause, got {:?}",
            r.reason
        );
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ROUND-TRIP ORACLE (phase-382 W1')
    // ═══════════════════════════════════════════════════════════════════════
    //
    // The streaming handlers write CDR by hand. Nothing in the compiler ties
    // those writes to the generated `Serialize` impls, so the only thing
    // standing between them and silent drift is this: build a request value,
    // serialize it, run the STREAMING handler over those bytes, and compare
    // the result against what the by-value handler produces for the same
    // request — as raw bytes AND as the generated reply type deserialized back
    // out of the stream.
    //
    // Every frame here that materialises a wire value does so in its own
    // `#[inline(never)]` helper: `SetParametersRequest` is 1.19 MB and
    // `GetParametersResponse` is 1.17 MB, so two of them live in one frame is
    // a stack overflow rather than a test failure.

    use alloc::{boxed::Box, vec, vec::Vec as AllocVec};
    use nros_core::{CdrReader, CdrWriter, Deserialize, Serialize};
    use nros_params::{MAX_PARAM_NAME_LEN, MAX_STRING_VALUE_LEN};
    use nros_rmw::TransportError;

    /// Heap scratch for one encoded message. Sized for the largest wire form
    /// these tests build (a 64-slot sequence of fully populated values).
    const WIRE_BUF: usize = 1 << 20;

    #[inline(never)]
    fn encode<M: Serialize>(msg: &M) -> AllocVec<u8> {
        let mut buf = vec![0u8; WIRE_BUF];
        let len = {
            let mut writer = CdrWriter::new_with_header(&mut buf).expect("encapsulation header");
            msg.serialize(&mut writer).expect("message fits WIRE_BUF");
            writer.position()
        };
        buf.truncate(len);
        buf
    }

    /// Deserialize the streamed bytes back into the generated reply type,
    /// landing the value on the HEAP.
    ///
    /// `Box::new(M::deserialize(..))` would not do: it materialises the reply
    /// as a stack temporary — twice, counting the `Result` — and a libtest
    /// thread has 2 MiB. Write it through `new_uninit` instead, and keep this
    /// the only frame that ever holds one. `GetParametersResponse` (1.17 MB)
    /// exceeds even that, because the generated `deserialize` builds the
    /// sequence and then the struct inside its OWN frame; that one reply is
    /// compared element-wise instead — see `assert_get_reply_matches`.
    #[inline(never)]
    fn decode_boxed<M: Deserialize>(bytes: &[u8]) -> Box<M> {
        let mut reader = CdrReader::new_with_header(bytes).expect("encapsulation header");
        let mut slot = Box::<M>::new_uninit();
        match M::deserialize(&mut reader) {
            Ok(value) => {
                slot.write(value);
            }
            Err(e) => panic!("streamed reply is not the generated type: {e:?}"),
        }
        // SAFETY: the `Ok` arm above wrote the value into the slot.
        unsafe { slot.assume_init() }
    }

    /// Run a streaming handler over an encoded request, exactly as
    /// `ServiceTrait::handle_request_raw` does: reader past the request's
    /// header, writer past the reply's.
    #[inline(never)]
    fn run_streaming(
        request: &[u8],
        handler: impl FnOnce(&mut CdrReader<'_>, &mut CdrWriter<'_>) -> Result<(), TransportError>,
    ) -> AllocVec<u8> {
        let mut reply = vec![0u8; WIRE_BUF];
        let len = {
            let mut reader = CdrReader::new_with_header(request).expect("request header");
            let mut writer = CdrWriter::new_with_header(&mut reply).expect("reply header");
            handler(&mut reader, &mut writer).expect("streaming handler");
            writer.position()
        };
        reply.truncate(len);
        reply
    }

    /// Every stored parameter, in slot order, as the wire values a client
    /// would see. Used to prove the two paths leave the STORE in the same
    /// state, not just the wire.
    #[inline(never)]
    fn store_digest(server: &ParameterServer) -> AllocVec<(alloc::string::String, ParameterValue)> {
        server
            .iter()
            .map(|p| {
                (
                    alloc::string::String::from(p.name.as_str()),
                    to_rcl_value(&p.value).expect("test values fit the wire"),
                )
            })
            .collect()
    }

    fn wire_string(s: &str) -> heapless::String<256> {
        heapless::String::try_from(s).expect("test string fits the wire message")
    }

    /// A string at the capacity bound of BOTH the store and the wire message.
    /// With the default knobs these are the same 256; a build that raises
    /// `NROS_MAX_STRING_VALUE_LEN` above the wire's 256 keeps this at the
    /// smaller bound, which is the one a round trip can survive.
    fn bound_string() -> alloc::string::String {
        let n = core::cmp::min(MAX_STRING_VALUE_LEN, 256);
        core::iter::repeat_n('x', n).collect()
    }

    fn wire_value_of(type_: u8) -> ParameterValue {
        ParameterValue {
            type_,
            ..Default::default()
        }
    }

    /// One wire value per `ParameterType`, all ten, plus the two rejection
    /// shapes issue 0323 is about.
    #[inline(never)]
    fn every_wire_value() -> AllocVec<(&'static str, ParameterValue)> {
        let mut out = AllocVec::new();

        out.push(("v_notset", wire_value_of(0)));

        let mut v = wire_value_of(1);
        v.bool_value = true;
        out.push(("v_bool", v));

        let mut v = wire_value_of(2);
        v.integer_value = -42;
        out.push(("v_int", v));

        let mut v = wire_value_of(3);
        v.double_value = 2.5;
        out.push(("v_double", v));

        let mut v = wire_value_of(4);
        v.string_value = wire_string(&bound_string());
        out.push(("v_string_bound", v));

        let mut v = wire_value_of(5);
        for b in 0..8u8 {
            v.byte_array_value.push(b).expect("fits");
        }
        out.push(("v_bytes", v));

        let mut v = wire_value_of(6);
        for b in 0..8 {
            v.bool_array_value.push(b % 2 == 0).expect("fits");
        }
        out.push(("v_bools", v));

        let mut v = wire_value_of(7);
        for i in 0..8i64 {
            v.integer_array_value.push(i * -3).expect("fits");
        }
        out.push(("v_ints", v));

        let mut v = wire_value_of(8);
        for i in 0..8 {
            v.double_array_value.push(f64::from(i) / 4.0).expect("fits");
        }
        out.push(("v_doubles", v));

        let mut v = wire_value_of(9);
        for i in 0..4 {
            let mut s = heapless::String::<256>::new();
            let _ = s.push_str("word");
            let _ = s.push((b'0' + i) as char);
            v.string_array_value.push(s).expect("fits");
        }
        out.push(("v_strings", v));

        // issue 0323 — 40 integers fit the wire message (64) but not the
        // store (MAX_ARRAY_LEN, 32 by default): rejected, never truncated.
        let mut v = wire_value_of(7);
        for i in 0..40i64 {
            v.integer_array_value.push(i).expect("fits the wire");
        }
        out.push(("v_oversize", v));

        // An unrecognised type code is an error, not a silent NOT_SET.
        out.push(("v_unknown_type", wire_value_of(99)));

        out
    }

    /// The store the read-only services are exercised against: one parameter
    /// of every type, a dotted name, and a string at the capacity bound.
    #[inline(never)]
    fn populated_server() -> ParameterServer<'static> {
        let mut server = leaked_server();
        for (name, value) in every_wire_value() {
            // The two rejection shapes have no stored form; skip them here.
            if let Ok(internal) = from_rcl_value(&value) {
                assert!(
                    server.declare(name, internal),
                    "test store must hold {name}"
                );
            }
        }
        assert!(
            server.declare("robot.speed", InternalValue::Double(1.5)),
            "dotted name declares"
        );
        assert!(
            server.declare("robot.mode", InternalValue::from_string("fast").unwrap()),
            "dotted name declares"
        );
        server
    }

    /// A store whose parameters carry descriptors — both range kinds, the
    /// read-only flag, dynamic typing — plus one with no descriptor at all.
    #[inline(never)]
    fn described_server() -> ParameterServer<'static> {
        let mut server = leaked_server();
        // Every declare is asserted: since issue 1150 a default off its own
        // step lattice is REFUSED at declare, and a fixture that silently
        // lost an entry would make every oracle comparison below vacuous.
        assert!(
            server.declare_with_descriptor(
                "speed",
                InternalValue::Double(1.0),
                Some(
                    InternalDescriptor::new("speed", InternalType::Double)
                        .expect("name fits")
                        .with_description("wheel speed")
                        .with_float_range(0.0, 10.0, 0.5),
                ),
            )
        );
        assert!(
            server.declare_with_descriptor(
                "count",
                InternalValue::Integer(4),
                Some(
                    InternalDescriptor::new("count", InternalType::Integer)
                        .expect("name fits")
                        .with_integer_range(0, 100, 2)
                        .with_read_only(true),
                ),
            )
        );
        assert!(
            server.declare_with_descriptor(
                "mode",
                InternalValue::from_string("idle").unwrap(),
                Some(
                    InternalDescriptor::new("mode", InternalType::String)
                        .expect("name fits")
                        .with_dynamic_typing(true),
                ),
            )
        );
        // No descriptor: DescribeParameters must synthesise a minimal one.
        assert!(server.declare("bare", InternalValue::Bool(true)));
        server
    }

    /// An empty store that ALLOWS undeclared parameters — the shape the
    /// "everything declares" oracle comparisons need since issue 1151 made
    /// the default refuse them.
    #[inline(never)]
    fn permissive_server() -> ParameterServer<'static> {
        let mut server = leaked_server();
        server.set_allow_undeclared(true);
        server
    }

    /// Every name in the populated store, an unknown one, and the dotted ones.
    const PROBE_NAMES: &[&str] = &[
        "v_notset",
        "v_bool",
        "v_int",
        "v_double",
        "v_string_bound",
        "v_bytes",
        "v_bools",
        "v_ints",
        "v_doubles",
        "v_strings",
        "robot.speed",
        "robot.mode",
        "no_such_parameter",
    ];

    macro_rules! names_request {
        ($ty:ty, $names:expr) => {{
            #[inline(never)]
            fn build(names: &[&str]) -> Box<$ty> {
                let mut req = Box::new(<$ty>::default());
                for name in names {
                    req.names.push(wire_string(name)).expect("64 names fit");
                }
                req
            }
            build($names)
        }};
    }

    macro_rules! push_param {
        ($req:expr, $name:expr, $value:expr) => {{
            let mut p = Parameter::default();
            p.name.push_str($name).expect("name fits the wire message");
            p.value = $value;
            $req.parameters.push(p).expect("64 parameters fit");
        }};
    }

    /// Read the streamed `GetParameters` reply back with the GENERATED
    /// `ParameterValue::deserialize`, one element at a time, and compare each
    /// against the oracle's.
    ///
    /// This is the round trip the other five do with `decode_boxed`, taken
    /// element-wise because `GetParametersResponse` is 1,176,072 bytes and the
    /// generated `deserialize` needs two of them live inside its own frame —
    /// more than a libtest thread's 2 MiB. Element-wise costs nothing in
    /// coverage: the sequence length is checked, every element is decoded by
    /// the generated impl, and the reply must end exactly where the last one
    /// does.
    #[inline(never)]
    fn assert_get_reply_matches(streamed: &[u8], oracle: &GetParametersResponse, names: &[&str]) {
        let mut reader = CdrReader::new_with_header(streamed).expect("reply header");
        let count = reader.read_sequence_len().expect("reply sequence length");
        assert_eq!(
            count,
            oracle.values.len(),
            "value count (names = {names:?})"
        );
        for (i, expected) in oracle.values.iter().enumerate() {
            let got = ParameterValue::deserialize(&mut reader)
                .expect("streamed element is a generated ParameterValue");
            assert_eq!(&got, expected, "value {i} (names = {names:?})");
        }
        assert_eq!(
            reader.remaining(),
            0,
            "streamed reply has trailing bytes (names = {names:?})"
        );
    }

    #[inline(never)]
    fn oracle_get_parameters(
        server: &ParameterServer,
        request: &GetParametersRequest,
    ) -> Box<GetParametersResponse> {
        handle_get_parameters(server, request)
    }

    #[test]
    fn get_parameters_streams_like_the_oracle() {
        let server = populated_server();
        // Empty name list, then every stored type plus a name that does not exist.
        for names in [&[][..], PROBE_NAMES] {
            let request = names_request!(GetParametersRequest, names);
            let streamed = run_streaming(&encode(&*request), |reader, writer| {
                stream_get_parameters(&server, reader, writer)
            });
            // `GetParametersResponse` is 1.17 MB, so the oracle's reply and
            // the round-tripped one are never alive in the same FRAME: each
            // lives behind an `#[inline(never)]` helper that pops before the
            // next runs, and only the boxes meet.
            let oracle = oracle_get_parameters(&server, &request);
            assert_eq!(
                streamed,
                encode(&*oracle),
                "streamed GetParameters bytes drifted from the generated Serialize \
                 (names = {names:?})"
            );
            assert_get_reply_matches(&streamed, &oracle, names);
        }
    }

    #[test]
    fn get_parameter_types_streams_like_the_oracle() {
        let server = populated_server();
        for names in [&[][..], PROBE_NAMES] {
            let request = names_request!(GetParameterTypesRequest, names);
            let streamed = run_streaming(&encode(&*request), |reader, writer| {
                stream_get_parameter_types(&server, reader, writer)
            });
            let oracle = handle_get_parameter_types(&server, &request);
            assert_eq!(
                streamed,
                encode(&*oracle),
                "streamed GetParameterTypes bytes drifted"
            );
            let round: Box<GetParameterTypesResponse> = decode_boxed(&streamed);
            assert_eq!(*round, *oracle);
        }
    }

    #[test]
    fn describe_parameters_streams_like_the_oracle() {
        let server = described_server();
        for names in [
            &[][..],
            &["speed", "count", "mode", "bare", "no_such_parameter"][..],
        ] {
            let request = names_request!(DescribeParametersRequest, names);
            let streamed = run_streaming(&encode(&*request), |reader, writer| {
                stream_describe_parameters(&server, reader, writer)
            });
            let oracle = handle_describe_parameters(&server, &request);
            assert_eq!(
                streamed,
                encode(&*oracle),
                "streamed DescribeParameters bytes drifted (names = {names:?})"
            );
            let round: Box<DescribeParametersResponse> = decode_boxed(&streamed);
            assert_eq!(*round, *oracle);
        }
    }

    #[test]
    fn list_parameters_streams_like_the_oracle() {
        let server = populated_server();
        #[inline(never)]
        fn list_request(prefixes: &[&str], depth: u64) -> Box<ListParametersRequest> {
            let mut req = Box::new(ListParametersRequest::default());
            for p in prefixes {
                req.prefixes.push(wire_string(p)).expect("64 prefixes fit");
            }
            req.depth = depth;
            req
        }

        // No filter; a prefix that matches the dotted names; a prefix that
        // matches nothing; depth 0 (unlimited) against a depth that excludes
        // the dotted names.
        for (prefixes, depth) in [
            (&[][..], 0u64),
            (&["robot."][..], 0),
            (&["nothing_matches"][..], 0),
            (&[][..], 1),
            (&["robot."][..], 1),
        ] {
            let request = list_request(prefixes, depth);
            let streamed = run_streaming(&encode(&*request), |reader, writer| {
                stream_list_parameters(&server, reader, writer)
            });
            let oracle = handle_list_parameters(&server, &request);
            assert_eq!(
                streamed,
                encode(&*oracle),
                "streamed ListParameters bytes drifted (prefixes = {prefixes:?}, depth = {depth})"
            );
            let round: Box<ListParametersResponse> = decode_boxed(&streamed);
            assert_eq!(*round, *oracle);
        }
    }

    #[test]
    fn set_parameters_streams_like_the_oracle() {
        #[inline(never)]
        fn request_of(
            values: AllocVec<(&'static str, ParameterValue)>,
        ) -> Box<SetParametersRequest> {
            let mut req = Box::new(SetParametersRequest::default());
            for (name, value) in values {
                push_param!(req, name, value);
            }
            req
        }

        // Empty batch, then one parameter of every type — including the two
        // issue-0323 rejections and a name too long for the STORE (which the
        // wire message accepts), so both success and every failure reason
        // reach the comparison.
        // Every `SetParameterResult` the reply can carry must reach the
        // comparison, or a wrong reason string is invisible: `count` is
        // read-only, `speed` carries a 0..10 float range and a Double type, a
        // name longer than the STORE's `MAX_PARAM_NAME_LEN` cannot be
        // declared, and the two issue-0323 shapes are rejected before the
        // store is reached.
        #[inline(never)]
        fn hard_cases() -> AllocVec<(&'static str, ParameterValue)> {
            let mut out = AllocVec::new();
            out.push(("count", wire_value_of(1))); // read-only
            let mut wrong_type = wire_value_of(1);
            wrong_type.bool_value = true;
            out.push(("speed", wrong_type)); // type mismatch
            let mut out_of_range = wire_value_of(3);
            out_of_range.double_value = 99.0;
            out.push(("speed", out_of_range)); // out of range
            let mut in_range = wire_value_of(3);
            in_range.double_value = 4.0;
            out.push(("speed", in_range)); // success on an EXISTING parameter
            let mut off_step = wire_value_of(3);
            off_step.double_value = 4.2;
            out.push(("speed", off_step)); // issue 1150: inside 0..10, off the 0.5 step
            out.push(("max_speeed", wire_value_of(3))); // issue 1151: undeclared
            let oversize = every_wire_value()
                .into_iter()
                .find(|(name, _)| *name == "v_oversize")
                .expect("every_wire_value carries the oversize shape")
                .1;
            out.push(("v_oversize", oversize));
            out.push(("v_unknown_type", wire_value_of(99)));
            out
        }

        for values in [AllocVec::new(), every_wire_value(), hard_cases()] {
            let mut request = request_of(values);
            let too_long: alloc::string::String =
                core::iter::repeat_n('n', MAX_PARAM_NAME_LEN + 1).collect();
            if !request.parameters.is_empty() {
                push_param!(request, &too_long, wire_value_of(1));
            }

            // Once against an empty PERMISSIVE store (everything declares),
            // once against an empty default one (issue 1151: everything is
            // refused as undeclared), and once against one with descriptors
            // (read-only, ranges, fixed types).
            for build in [
                (permissive_server as fn() -> ParameterServer<'static>),
                leaked_server,
                described_server,
            ] {
                let mut streaming_server = build();
                let mut oracle_server = build();

                let streamed = run_streaming(&encode(&*request), |reader, writer| {
                    stream_set_parameters(&mut streaming_server, reader, writer)
                });
                let oracle = handle_set_parameters(&mut oracle_server, &request);

                assert_eq!(
                    streamed,
                    encode(&*oracle),
                    "streamed SetParameters bytes drifted"
                );
                let round: Box<SetParametersResponse> = decode_boxed(&streamed);
                assert_eq!(*round, *oracle);
                assert_eq!(
                    store_digest(&streaming_server),
                    store_digest(&oracle_server),
                    "the two paths left the store in different states"
                );
            }
        }
    }

    #[test]
    fn set_parameters_atomically_streams_like_the_oracle() {
        #[inline(never)]
        fn request_of(
            values: AllocVec<(&'static str, ParameterValue)>,
        ) -> Box<SetParametersAtomicallyRequest> {
            let mut req = Box::new(SetParametersAtomicallyRequest::default());
            for (name, value) in values {
                push_param!(req, name, value);
            }
            req
        }

        // A batch that must apply whole, a batch containing an issue-0323
        // rejection that must apply NOTHING, a batch blocked by a read-only
        // descriptor, one blocked by a type mismatch, and the empty batch.
        let mut all_good = AllocVec::new();
        for (name, value) in every_wire_value() {
            if name != "v_oversize" && name != "v_unknown_type" {
                all_good.push((name, value));
            }
        }
        let mut read_only_batch = AllocVec::new();
        let mut int_value = wire_value_of(2);
        int_value.integer_value = 9;
        read_only_batch.push(("count", int_value));
        let mut mismatch_batch = AllocVec::new();
        mismatch_batch.push(("speed", wire_value_of(1)));

        for values in [
            AllocVec::new(),
            all_good,
            every_wire_value(),
            read_only_batch,
            mismatch_batch,
        ] {
            let request = request_of(values);
            let mut streaming_server = described_server();
            let mut oracle_server = described_server();

            let streamed = run_streaming(&encode(&*request), |reader, writer| {
                stream_set_parameters_atomically(&mut streaming_server, reader, writer)
            });
            let oracle = handle_set_parameters_atomically(&mut oracle_server, &request);

            assert_eq!(
                streamed,
                encode(&*oracle),
                "streamed SetParametersAtomically bytes drifted"
            );
            let round: Box<SetParametersAtomicallyResponse> = decode_boxed(&streamed);
            assert_eq!(*round, *oracle);
            assert_eq!(
                store_digest(&streaming_server),
                store_digest(&oracle_server),
                "atomic apply left the two stores in different states"
            );
        }
    }

    /// Issue 1151 — `ros2 param set /node max_speeed 5.0` on a node that
    /// declared `max_speed` used to answer "successful" and create
    /// `max_speeed`. Now it is refused with rclrs's reason, and the store is
    /// untouched. Asserted on the LIVE streaming path, decoded with the
    /// generated message type so the reason string is checked on the wire.
    #[test]
    fn set_parameters_refuses_an_undeclared_name_with_a_reason() {
        let mut server = leaked_server();
        assert!(server.declare("max_speed", InternalValue::Double(1.0)));

        let mut request = Box::new(SetParametersRequest::default());
        let mut typo = wire_value_of(3);
        typo.double_value = 5.0;
        push_param!(request, "max_speeed", typo);

        let streamed = run_streaming(&encode(&*request), |reader, writer| {
            stream_set_parameters(&mut server, reader, writer)
        });
        let reply: Box<SetParametersResponse> = decode_boxed(&streamed);
        assert_eq!(reply.results.len(), 1);
        assert!(!reply.results[0].successful);
        assert_eq!(reply.results[0].reason.as_str(), UNDECLARED_REASON);

        assert!(!server.has("max_speeed"), "the typo must not be created");
        assert_eq!(server.len(), 1);
        assert_eq!(server.get_double("max_speed"), Some(1.0));
    }

    /// Issue 1151 — the opt-in. With `allow_undeclared` on, the same request
    /// declares the name, which is upstream's `allow_undeclared_parameters`.
    #[test]
    fn set_parameters_declares_an_unknown_name_under_allow_undeclared() {
        let mut server = permissive_server();

        let mut request = Box::new(SetParametersRequest::default());
        let mut v = wire_value_of(3);
        v.double_value = 5.0;
        push_param!(request, "fresh", v);

        let streamed = run_streaming(&encode(&*request), |reader, writer| {
            stream_set_parameters(&mut server, reader, writer)
        });
        let reply: Box<SetParametersResponse> = decode_boxed(&streamed);
        assert!(reply.results[0].successful, "{}", reply.results[0].reason);
        assert_eq!(reply.results[0].reason.as_str(), "");
        assert_eq!(server.get_double("fresh"), Some(5.0));
    }

    /// Issue 1150 on the wire — a value inside min..max but off the step is
    /// refused as out of range by the streaming `SetParameters`.
    #[test]
    fn set_parameters_refuses_a_value_off_the_step() {
        let mut server = described_server();
        let mut request = Box::new(SetParametersRequest::default());
        let mut off_step = wire_value_of(3);
        off_step.double_value = 4.2; // 0..10 step 0.5
        push_param!(request, "speed", off_step);

        let streamed = run_streaming(&encode(&*request), |reader, writer| {
            stream_set_parameters(&mut server, reader, writer)
        });
        let reply: Box<SetParametersResponse> = decode_boxed(&streamed);
        assert!(!reply.results[0].successful);
        assert_eq!(reply.results[0].reason.as_str(), "Value out of range");
        assert_eq!(server.get_double("speed"), Some(1.0));
    }

    /// Issue 1151, atomic variant — one undeclared name in the batch rejects
    /// the WHOLE batch: the declared sibling keeps its old value.
    #[test]
    fn set_parameters_atomically_rejects_the_batch_on_an_undeclared_name() {
        let mut server = leaked_server();
        assert!(server.declare("max_speed", InternalValue::Double(1.0)));

        let mut request = Box::new(SetParametersAtomicallyRequest::default());
        let mut good = wire_value_of(3);
        good.double_value = 2.0;
        push_param!(request, "max_speed", good);
        let mut typo = wire_value_of(3);
        typo.double_value = 5.0;
        push_param!(request, "max_speeed", typo);

        let streamed = run_streaming(&encode(&*request), |reader, writer| {
            stream_set_parameters_atomically(&mut server, reader, writer)
        });
        let reply: Box<SetParametersAtomicallyResponse> = decode_boxed(&streamed);
        assert!(!reply.result.successful);
        assert!(!reply.result.reason.is_empty());

        assert_eq!(
            server.get_double("max_speed"),
            Some(1.0),
            "atomic: the good half must not have applied"
        );
        assert!(!server.has("max_speeed"));
        assert_eq!(server.len(), 1);

        // Same batch, permissive store: applies whole.
        let mut server = permissive_server();
        assert!(server.declare("max_speed", InternalValue::Double(1.0)));
        let streamed = run_streaming(&encode(&*request), |reader, writer| {
            stream_set_parameters_atomically(&mut server, reader, writer)
        });
        let reply: Box<SetParametersAtomicallyResponse> = decode_boxed(&streamed);
        assert!(reply.result.successful, "{}", reply.result.reason);
        assert_eq!(server.get_double("max_speed"), Some(2.0));
        assert_eq!(server.get_double("max_speeed"), Some(5.0));
    }

    /// The streaming readers refuse a sequence or string the generated
    /// `Deserialize` could not have held, so the two paths agree on what is a
    /// malformed request as well as on what is a valid one. If a regenerated
    /// message changes a capacity, this fails rather than the wire silently
    /// diverging.
    #[test]
    fn wire_capacities_match_the_generated_messages() {
        let request = Box::new(GetParametersRequest::default());
        assert_eq!(request.names.capacity(), WIRE_SEQ_CAP);
        assert_eq!(wire_string("").capacity(), WIRE_STRING_CAP);

        let value = Box::new(ParameterValue::default());
        assert_eq!(value.string_value.capacity(), WIRE_STRING_CAP);
        assert_eq!(value.byte_array_value.capacity(), WIRE_SEQ_CAP);
        assert_eq!(value.bool_array_value.capacity(), WIRE_SEQ_CAP);
        assert_eq!(value.integer_array_value.capacity(), WIRE_SEQ_CAP);
        assert_eq!(value.double_array_value.capacity(), WIRE_SEQ_CAP);
        assert_eq!(value.string_array_value.capacity(), WIRE_SEQ_CAP);

        let descriptor = Box::new(ParameterDescriptor::default());
        assert_eq!(descriptor.name.capacity(), WIRE_STRING_CAP);
        assert_eq!(descriptor.description.capacity(), WIRE_STRING_CAP);
        assert_eq!(
            descriptor.additional_constraints.capacity(),
            WIRE_STRING_CAP
        );
        assert_eq!(descriptor.floating_point_range.capacity(), 1);
        assert_eq!(descriptor.integer_range.capacity(), 1);

        let listed = Box::new(ListParametersResponse::default());
        assert_eq!(listed.result.names.capacity(), WIRE_SEQ_CAP);
        assert_eq!(listed.result.prefixes.capacity(), WIRE_SEQ_CAP);
        assert_eq!(MAX_PARAMS_PER_REQUEST, WIRE_SEQ_CAP);
    }

    /// phase-382 W1' measured, in one line.
    ///
    /// The dominant local of every streaming handler is one internal
    /// `ParameterValue` — the value `read_parameter_value` builds for the
    /// parameter in flight. It replaces a by-value `SetParametersRequest` on
    /// the calling task's stack. This is a DOMINANT-LOCAL bound, not a full
    /// stack profile.
    #[test]
    fn the_streaming_peak_is_one_internal_value_not_the_whole_request() {
        let internal = core::mem::size_of::<InternalValue>();
        let request = core::mem::size_of::<SetParametersRequest>();
        let reply = core::mem::size_of::<GetParametersResponse>();
        assert!(
            request / internal >= 100,
            "the phase doc claims two orders of magnitude: request {request} / internal {internal}"
        );
        assert!(
            reply / internal >= 100,
            "reply {reply} / internal {internal}"
        );
    }
}
