//! Parameter types for ROS 2 compatible parameter handling
//!
//! This module provides types for representing ROS 2 parameters including
//! scalar values, arrays, and parameter descriptors.

use heapless::{String, Vec};

// phase-359 W8 — `alloc`, not `std`: `ToString`, `String` and `Vec` all live in
// `alloc`, so gating them on `std` withheld the `ParameterVariant` impls below
// from `no_std + alloc` targets that can use them.
#[cfg(feature = "alloc")]
use alloc::string::ToString;

pub use crate::config::*;

/// ROS 2 parameter types
///
/// These match the parameter types defined in rcl_interfaces/msg/ParameterType
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum ParameterType {
    /// Parameter value not set
    #[default]
    NotSet = 0,
    /// Boolean parameter
    Bool = 1,
    /// 64-bit signed integer parameter
    Integer = 2,
    /// 64-bit floating point parameter
    Double = 3,
    /// String parameter
    String = 4,
    /// Byte array parameter
    ByteArray = 5,
    /// Boolean array parameter
    BoolArray = 6,
    /// Integer array parameter
    IntegerArray = 7,
    /// Double array parameter
    DoubleArray = 8,
    /// String array parameter
    StringArray = 9,
}

/// Parameter value container
///
/// Holds a parameter value of any supported type.
/// Note: This enum is intentionally large to support all parameter types
/// without heap allocation in embedded systems.
#[derive(Debug, Clone, Default)]
#[allow(clippy::large_enum_variant)]
pub enum ParameterValue {
    /// Value not set
    #[default]
    NotSet,
    /// Boolean value
    Bool(bool),
    /// 64-bit signed integer value
    Integer(i64),
    /// 64-bit floating point value
    Double(f64),
    /// String value
    String(String<MAX_STRING_VALUE_LEN>),
    /// Byte array value
    ByteArray(Vec<u8, MAX_BYTE_ARRAY_LEN>),
    /// Boolean array value
    BoolArray(Vec<bool, MAX_ARRAY_LEN>),
    /// Integer array value
    IntegerArray(Vec<i64, MAX_ARRAY_LEN>),
    /// Double array value
    DoubleArray(Vec<f64, MAX_ARRAY_LEN>),
    /// String array value (array of fixed-size strings)
    StringArray(Vec<String<MAX_STRING_VALUE_LEN>, MAX_ARRAY_LEN>),
}

impl ParameterValue {
    /// Get the parameter type for this value
    pub fn param_type(&self) -> ParameterType {
        match self {
            Self::NotSet => ParameterType::NotSet,
            Self::Bool(_) => ParameterType::Bool,
            Self::Integer(_) => ParameterType::Integer,
            Self::Double(_) => ParameterType::Double,
            Self::String(_) => ParameterType::String,
            Self::ByteArray(_) => ParameterType::ByteArray,
            Self::BoolArray(_) => ParameterType::BoolArray,
            Self::IntegerArray(_) => ParameterType::IntegerArray,
            Self::DoubleArray(_) => ParameterType::DoubleArray,
            Self::StringArray(_) => ParameterType::StringArray,
        }
    }

    /// Check if the value is set
    pub fn is_set(&self) -> bool {
        !matches!(self, Self::NotSet)
    }

    /// Try to get the value as a bool
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Self::Bool(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get the value as an integer
    pub fn as_integer(&self) -> Option<i64> {
        match self {
            Self::Integer(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get the value as a double
    pub fn as_double(&self) -> Option<f64> {
        match self {
            Self::Double(v) => Some(*v),
            _ => None,
        }
    }

    /// Try to get the value as a string slice
    pub fn as_string(&self) -> Option<&str> {
        match self {
            Self::String(v) => Some(v.as_str()),
            _ => None,
        }
    }

    /// Try to get the value as a byte array slice
    pub fn as_byte_array(&self) -> Option<&[u8]> {
        match self {
            Self::ByteArray(v) => Some(v.as_slice()),
            _ => None,
        }
    }

    /// Try to get the value as a bool array slice
    pub fn as_bool_array(&self) -> Option<&[bool]> {
        match self {
            Self::BoolArray(v) => Some(v.as_slice()),
            _ => None,
        }
    }

    /// Try to get the value as an integer array slice
    pub fn as_integer_array(&self) -> Option<&[i64]> {
        match self {
            Self::IntegerArray(v) => Some(v.as_slice()),
            _ => None,
        }
    }

    /// Try to get the value as a double array slice
    pub fn as_double_array(&self) -> Option<&[f64]> {
        match self {
            Self::DoubleArray(v) => Some(v.as_slice()),
            _ => None,
        }
    }

    /// Create a bool value
    pub fn from_bool(value: bool) -> Self {
        Self::Bool(value)
    }

    /// Create an integer value
    pub fn from_integer(value: i64) -> Self {
        Self::Integer(value)
    }

    /// Create a double value
    pub fn from_double(value: f64) -> Self {
        Self::Double(value)
    }

    /// Create a string value from a str slice
    pub fn from_string(value: &str) -> Option<Self> {
        let mut s = String::new();
        s.push_str(value).ok()?;
        Some(Self::String(s))
    }
}

/// Floating point range constraints for parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct FloatingPointRange {
    /// Minimum allowed value (inclusive)
    pub min: f64,
    /// Maximum allowed value (inclusive)
    pub max: f64,
    /// Step size for value changes (0 = any step allowed)
    pub step: f64,
}

impl FloatingPointRange {
    /// Create a new floating point range
    pub const fn new(min: f64, max: f64, step: f64) -> Self {
        Self { min, max, step }
    }

    /// A range is well-formed when `min <= max` and `step >= 0` (0 = no step).
    ///
    /// Upstream refuses to DECLARE with an ill-formed range
    /// (`DeclarationError::InvalidRange`); a negative step here would also go
    /// on the wire as a huge `u64` in `DescribeParameters` (issue 1150).
    pub fn is_valid(&self) -> bool {
        self.min <= self.max && self.step >= 0.0 && !self.step.is_nan()
    }

    /// Check if a value is within this range.
    ///
    /// Issue 1150: `step` is ENFORCED, mirroring rclrs `range.rs`. A value is
    /// in range iff `min <= v <= max` and, when `step != 0`, `v` lies on the
    /// lattice `min + k*step` within floating tolerance. Either endpoint is
    /// always accepted, exactly as upstream (the upper bound need not be on
    /// the lattice). The tolerance is upstream's: 100 ULP, relative to the
    /// magnitude of the two operands.
    pub fn contains(&self, value: f64) -> bool {
        if are_close(value, self.min) || are_close(value, self.max) {
            return true;
        }
        if !(value >= self.min && value <= self.max) {
            return false;
        }
        if self.step == 0.0 {
            return true;
        }
        let nearest = round_to_nearest((value - self.min) / self.step) * self.step + self.min;
        are_close(nearest, value)
    }
}

/// rclrs `ParameterRange<f64>::are_close`: equal within 100 ULP of the
/// operands' magnitude. One spelling, shared by the endpoint and lattice
/// checks so they cannot drift apart.
#[inline]
fn are_close(a: f64, b: f64) -> bool {
    const ULP_TOL: f64 = 100.0;
    (a - b).abs() <= f64::EPSILON * (a + b).abs() * ULP_TOL
}

/// Round half away from zero without `std` (`f64::round` is not in `core`
/// on every toolchain this crate targets). Saturates through the `i64` cast
/// for a quotient beyond `2^63`, where the lattice is not representable
/// anyway.
#[inline]
fn round_to_nearest(q: f64) -> f64 {
    if !q.is_finite() {
        return q;
    }
    let shifted = if q < 0.0 { q - 0.5 } else { q + 0.5 };
    (shifted as i64) as f64
}

/// Integer range constraints for parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct IntegerRange {
    /// Minimum allowed value (inclusive)
    pub min: i64,
    /// Maximum allowed value (inclusive)
    pub max: i64,
    /// Step size for value changes (0 = any step allowed)
    pub step: i64,
}

impl IntegerRange {
    /// Create a new integer range
    pub const fn new(min: i64, max: i64, step: i64) -> Self {
        Self { min, max, step }
    }

    /// A range is well-formed when `min <= max` and `step >= 0` (0 = no step).
    /// See [`FloatingPointRange::is_valid`].
    pub fn is_valid(&self) -> bool {
        self.min <= self.max && self.step >= 0
    }

    /// Check if a value is within this range.
    ///
    /// Issue 1150: `step` is ENFORCED, mirroring rclrs `range.rs`: in range
    /// iff `min <= v <= max` and, when `step != 0`, `(v - min)` is a multiple
    /// of `step`. The upper bound is always accepted even when off-lattice,
    /// exactly as upstream. Arithmetic is checked so an extreme `min` cannot
    /// overflow into a panic.
    pub fn contains(&self, value: i64) -> bool {
        if !(value >= self.min && value <= self.max) {
            return false;
        }
        if value == self.max || self.step == 0 {
            return true;
        }
        match value
            .checked_sub(self.min)
            .and_then(|d| d.checked_rem(self.step))
        {
            Some(rem) => rem == 0,
            // `value - min` does not fit an `i64`, so it is not `k * step`
            // for any representable `k` the caller could have meant.
            None => false,
        }
    }
}

/// Range constraints for a parameter
#[derive(Debug, Clone, Copy, Default)]
pub enum ParameterRange {
    /// No range constraints
    #[default]
    None,
    /// Floating point range
    FloatingPoint(FloatingPointRange),
    /// Integer range
    Integer(IntegerRange),
}

impl ParameterRange {
    /// Well-formed: `None`, or an arm whose `min <= max` and `step >= 0`.
    pub fn is_valid(&self) -> bool {
        match self {
            Self::None => true,
            Self::FloatingPoint(r) => r.is_valid(),
            Self::Integer(r) => r.is_valid(),
        }
    }

    /// THE range predicate. Every set — `ParameterServer::set`, the six
    /// service handlers, the typed API — and every declare-time default check
    /// reaches this one function, so an integer and a floating-point value
    /// cannot be judged by two different rules (issue 1150 class).
    ///
    /// A value whose type does not match the arm (a string against an
    /// integer range, a `NotSet`) is not the range's business and passes; the
    /// type check is a separate, earlier verdict.
    pub fn contains(&self, value: &ParameterValue) -> bool {
        match (self, value) {
            (Self::None, _) => true,
            (Self::Integer(range), ParameterValue::Integer(v)) => range.contains(*v),
            (Self::FloatingPoint(range), ParameterValue::Double(v)) => range.contains(*v),
            _ => true,
        }
    }
}

/// Parameter descriptor containing metadata
///
/// Describes a parameter including its type, constraints, and documentation.
#[derive(Debug, Clone)]
pub struct ParameterDescriptor {
    /// Parameter name
    pub name: String<MAX_PARAM_NAME_LEN>,
    /// Parameter type
    pub param_type: ParameterType,
    /// Human-readable description
    pub description: String<MAX_STRING_VALUE_LEN>,
    /// Whether the parameter is read-only
    pub read_only: bool,
    /// Whether the parameter type can change dynamically
    pub dynamic_typing: bool,
    /// Range constraints
    pub range: ParameterRange,
}

impl ParameterDescriptor {
    /// Create a new parameter descriptor
    pub fn new(name: &str, param_type: ParameterType) -> Option<Self> {
        let mut n = String::new();
        n.push_str(name).ok()?;
        Some(Self {
            name: n,
            param_type,
            description: String::new(),
            read_only: false,
            dynamic_typing: false,
            range: ParameterRange::None,
        })
    }

    /// Set the description
    pub fn with_description(mut self, desc: &str) -> Self {
        self.description.clear();
        let _ = self.description.push_str(desc);
        self
    }

    /// Set read-only flag
    pub fn with_read_only(mut self, read_only: bool) -> Self {
        self.read_only = read_only;
        self
    }

    /// Set dynamic typing flag
    pub fn with_dynamic_typing(mut self, dynamic: bool) -> Self {
        self.dynamic_typing = dynamic;
        self
    }

    /// Set integer range constraints
    pub fn with_integer_range(mut self, min: i64, max: i64, step: i64) -> Self {
        self.range = ParameterRange::Integer(IntegerRange::new(min, max, step));
        self
    }

    /// Set floating point range constraints
    pub fn with_float_range(mut self, min: f64, max: f64, step: f64) -> Self {
        self.range = ParameterRange::FloatingPoint(FloatingPointRange::new(min, max, step));
        self
    }

    /// Check if a value satisfies the range constraints — min, max AND step.
    ///
    /// Forwards to [`ParameterRange::contains`], the single predicate.
    pub fn validate_range(&self, value: &ParameterValue) -> bool {
        self.range.contains(value)
    }
}

/// A named parameter with value and optional descriptor
#[derive(Debug, Clone)]
pub struct Parameter {
    /// Parameter name
    pub name: String<MAX_PARAM_NAME_LEN>,
    /// Parameter value
    pub value: ParameterValue,
}

impl Parameter {
    /// Create a new parameter with a value
    pub fn new(name: &str, value: ParameterValue) -> Option<Self> {
        let mut n = String::new();
        n.push_str(name).ok()?;
        Some(Self { name: n, value })
    }

    /// Create a new boolean parameter
    pub fn bool(name: &str, value: bool) -> Option<Self> {
        Self::new(name, ParameterValue::Bool(value))
    }

    /// Create a new integer parameter
    pub fn integer(name: &str, value: i64) -> Option<Self> {
        Self::new(name, ParameterValue::Integer(value))
    }

    /// Create a new double parameter
    pub fn double(name: &str, value: f64) -> Option<Self> {
        Self::new(name, ParameterValue::Double(value))
    }

    /// Create a new string parameter
    pub fn string(name: &str, value: &str) -> Option<Self> {
        Self::new(name, ParameterValue::from_string(value)?)
    }

    /// Get the parameter type
    pub fn param_type(&self) -> ParameterType {
        self.value.param_type()
    }

    /// Get the parameter name
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Result of setting a parameter
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetParameterResult {
    /// Parameter was set successfully
    Success,
    /// Parameter is read-only
    ReadOnly,
    /// Parameter type mismatch (and dynamic typing disabled)
    TypeMismatch,
    /// Value is outside allowed range
    OutOfRange,
    /// Parameter not found
    NotFound,
    /// Storage is full
    StorageFull,
    /// A set on a name that was never declared, and the server does not
    /// allow undeclared parameters (issue 1151; rclrs `parameter.rs`).
    ///
    /// Distinct from [`NotFound`](Self::NotFound): that is what a DIRECT
    /// `ParameterServer::set` says about a missing name; this is what the
    /// wire-facing [`ParameterServer::apply`](crate::ParameterServer::apply)
    /// says when the only remaining option — declaring it — is switched off.
    Undeclared,
    /// A declaration whose range is ill-formed (`min > max` or `step < 0`),
    /// or whose default does not satisfy its own range (issue 1150; rclrs
    /// `DeclarationError::InvalidRange` / `InitialValueOutOfRange`).
    InvalidRange,
}

impl SetParameterResult {
    /// Check if the result indicates success
    pub fn is_success(&self) -> bool {
        matches!(self, Self::Success)
    }
}

/// A value does not fit the compile-time parameter capacity (issue 0323).
///
/// Returned by [`ParameterVariant::try_to_parameter_value`]. A named type
/// rather than `()` so the failure reads at the call site and clippy's
/// `result_unit_err` stays satisfied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityExceeded;

/// Trait for types that can be used as typed parameters
///
/// This trait provides conversions between Rust types and ParameterValue,
/// enabling type-safe parameter access.
pub trait ParameterVariant: Clone {
    /// Convert this type to a ParameterValue
    fn to_parameter_value(&self) -> ParameterValue;

    /// Fallible conversion — `Err(())` when `self` does not fit the
    /// compile-time capacity (issue 0323).
    ///
    /// [`Self::to_parameter_value`] cannot report that: the hosted `std`
    /// impls used `unwrap_or_default()`, so an over-long `String` became
    /// `ParameterValue::NotSet` (a TYPE change) and an oversized `Vec`
    /// became an EMPTY array, both indistinguishable from a caller who
    /// meant it. Callers at the declare/set boundary should prefer this.
    ///
    /// Defaults to infallible for impls whose values cannot overflow (the
    /// scalars); the `std` collection impls override it.
    fn try_to_parameter_value(&self) -> Result<ParameterValue, CapacityExceeded> {
        Ok(self.to_parameter_value())
    }

    /// Try to extract this type from a ParameterValue
    fn from_parameter_value(value: &ParameterValue) -> Option<Self>;

    /// Get the expected parameter type
    fn parameter_type() -> ParameterType;
}

// Implement ParameterVariant for basic types

impl ParameterVariant for bool {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::Bool(*self)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_bool()
    }

    fn parameter_type() -> ParameterType {
        ParameterType::Bool
    }
}

impl ParameterVariant for i64 {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::Integer(*self)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_integer()
    }

    fn parameter_type() -> ParameterType {
        ParameterType::Integer
    }
}

impl ParameterVariant for f64 {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::Double(*self)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_double()
    }

    fn parameter_type() -> ParameterType {
        ParameterType::Double
    }
}

// `ParameterVariant` for `heapless::String` — the fixed-capacity parameter type.
//
// phase-359 W10 — this was `#[cfg(not(feature = "std"))]`, which said a hosted
// build may not use a fixed-capacity string parameter. Nothing enforced that:
// the `alloc` impl below is on a DIFFERENT type, so an `alloc`-without-`std`
// build has always had both, and only the hosted flavour was singled out. A
// consumer that wants bounded parameter storage on a host — the usual reason
// being that the same node also builds for a target — was told no by a gate
// that was expressing a preference, not a constraint.
impl ParameterVariant for String<MAX_STRING_VALUE_LEN> {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::String(self.clone())
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        if let ParameterValue::String(s) = value {
            Some(s.clone())
        } else {
            None
        }
    }

    fn parameter_type() -> ParameterType {
        ParameterType::String
    }
}

// Implement ParameterVariant for std::string::String (std)
#[cfg(feature = "alloc")]
impl ParameterVariant for alloc::string::String {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::from_string(self.as_str()).unwrap_or_default()
    }

    fn try_to_parameter_value(&self) -> Result<ParameterValue, CapacityExceeded> {
        ParameterValue::from_string(self.as_str()).ok_or(CapacityExceeded)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_string().map(|s| s.to_string())
    }

    fn parameter_type() -> ParameterType {
        ParameterType::String
    }
}

// Implement ParameterVariant for std::vec::Vec<i64> (std)
#[cfg(feature = "alloc")]
impl ParameterVariant for alloc::vec::Vec<i64> {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::IntegerArray(Vec::from_slice(self.as_slice()).unwrap_or_default())
    }

    fn try_to_parameter_value(&self) -> Result<ParameterValue, CapacityExceeded> {
        Vec::from_slice(self.as_slice())
            .map(ParameterValue::IntegerArray)
            .map_err(|_| CapacityExceeded)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_integer_array().map(|v| v.to_vec())
    }

    fn parameter_type() -> ParameterType {
        ParameterType::IntegerArray
    }
}

// Implement ParameterVariant for std::vec::Vec<f64> (std)
#[cfg(feature = "alloc")]
impl ParameterVariant for alloc::vec::Vec<f64> {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::DoubleArray(Vec::from_slice(self.as_slice()).unwrap_or_default())
    }

    fn try_to_parameter_value(&self) -> Result<ParameterValue, CapacityExceeded> {
        Vec::from_slice(self.as_slice())
            .map(ParameterValue::DoubleArray)
            .map_err(|_| CapacityExceeded)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_double_array().map(|v| v.to_vec())
    }

    fn parameter_type() -> ParameterType {
        ParameterType::DoubleArray
    }
}

// Implement ParameterVariant for std::vec::Vec<bool> (std)
#[cfg(feature = "alloc")]
impl ParameterVariant for alloc::vec::Vec<bool> {
    fn to_parameter_value(&self) -> ParameterValue {
        ParameterValue::BoolArray(Vec::from_slice(self.as_slice()).unwrap_or_default())
    }

    fn try_to_parameter_value(&self) -> Result<ParameterValue, CapacityExceeded> {
        Vec::from_slice(self.as_slice())
            .map(ParameterValue::BoolArray)
            .map_err(|_| CapacityExceeded)
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        value.as_bool_array().map(|v| v.to_vec())
    }

    fn parameter_type() -> ParameterType {
        ParameterType::BoolArray
    }
}

// Implement ParameterVariant for std::vec::Vec<std::string::String> (std)
#[cfg(feature = "alloc")]
impl ParameterVariant for alloc::vec::Vec<alloc::string::String> {
    fn to_parameter_value(&self) -> ParameterValue {
        let mut vec = Vec::new();
        for s in self {
            let mut h_string = String::new();
            if h_string.push_str(s.as_str()).is_ok() {
                let _ = vec.push(h_string);
            }
        }
        ParameterValue::StringArray(vec)
    }

    fn try_to_parameter_value(&self) -> Result<ParameterValue, CapacityExceeded> {
        // The infallible form above SKIPS any element that does not fit,
        // so a 3-element vec could arrive as 2 with no signal (issue 0323).
        let mut vec = Vec::new();
        for s in self {
            let mut h_string = String::new();
            h_string
                .push_str(s.as_str())
                .map_err(|_| CapacityExceeded)?;
            vec.push(h_string).map_err(|_| CapacityExceeded)?;
        }
        Ok(ParameterValue::StringArray(vec))
    }

    fn from_parameter_value(value: &ParameterValue) -> Option<Self> {
        if let ParameterValue::StringArray(v) = value {
            Some(v.iter().map(|s| s.as_str().to_string()).collect())
        } else {
            None
        }
    }

    fn parameter_type() -> ParameterType {
        ParameterType::StringArray
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── issue 1150: step is ENFORCED, min/max/step as rclrs `range.rs` ──

    #[test]
    fn integer_step_accepts_lattice_points_and_rejects_the_rest() {
        let r = IntegerRange::new(10, 20, 5);
        assert!(r.contains(10));
        assert!(r.contains(15));
        assert!(r.contains(20));
        // Inside min..max but off the lattice: this is the case the old
        // `min <= v <= max` accepted and upstream rejects.
        assert!(!r.contains(11));
        assert!(!r.contains(14));
        assert!(!r.contains(19));
        // Bounds still apply.
        assert!(!r.contains(5));
        assert!(!r.contains(25));
    }

    #[test]
    fn integer_upper_bound_is_accepted_even_off_lattice() {
        // rclrs: `if upper == value { return true }` before the step test.
        let r = IntegerRange::new(0, 7, 3);
        assert!(r.contains(6));
        assert!(!r.contains(5));
        assert!(r.contains(7));
    }

    #[test]
    fn integer_step_zero_means_unconstrained() {
        let r = IntegerRange::new(0, 100, 0);
        for v in [0, 1, 37, 99, 100] {
            assert!(r.contains(v), "{v}");
        }
        assert!(!r.contains(101));
    }

    #[test]
    fn integer_step_does_not_overflow_at_the_extremes() {
        let r = IntegerRange::new(i64::MIN, i64::MAX, 2);
        // `v - min` overflows for every positive v; that must be a verdict,
        // not a panic.
        let _ = r.contains(1);
        let _ = r.contains(i64::MAX - 1);
        assert!(r.contains(i64::MIN));
        assert!(r.contains(i64::MAX));
    }

    #[test]
    fn float_step_accepts_lattice_points_within_tolerance() {
        let r = FloatingPointRange::new(0.0, 1.0, 0.1);
        // 0.3 is not exactly representable and 0.1 * 3 != 0.3 in binary;
        // upstream's 100-ULP tolerance is what makes this pass.
        for v in [0.0, 0.1, 0.3, 0.7, 1.0] {
            assert!(r.contains(v), "{v}");
        }
        assert!(r.contains(0.1 + 0.2));
        // Off the lattice by more than the tolerance.
        assert!(!r.contains(0.05));
        assert!(!r.contains(0.35));
        assert!(!r.contains(0.999));
        // Bounds.
        assert!(!r.contains(-0.1));
        assert!(!r.contains(1.1));
    }

    #[test]
    fn float_endpoints_are_accepted_even_off_lattice() {
        let r = FloatingPointRange::new(0.0, 1.0, 0.3);
        assert!(r.contains(0.9));
        assert!(!r.contains(0.95));
        assert!(r.contains(1.0));
    }

    #[test]
    fn float_step_zero_means_unconstrained() {
        let r = FloatingPointRange::new(0.0, 10.0, 0.0);
        for v in [0.0, 0.001, 3.25, 9.999, 10.0] {
            assert!(r.contains(v), "{v}");
        }
        assert!(!r.contains(10.001));
    }

    #[test]
    fn float_step_with_a_nonzero_min_is_relative_to_min() {
        let r = FloatingPointRange::new(0.5, 2.5, 0.5);
        assert!(r.contains(1.5));
        assert!(!r.contains(1.25));
        assert!(r.contains(0.5));
    }

    #[test]
    fn range_validity_is_min_le_max_and_step_ge_zero() {
        assert!(IntegerRange::new(0, 10, 0).is_valid());
        assert!(IntegerRange::new(0, 10, 3).is_valid());
        assert!(IntegerRange::new(5, 5, 1).is_valid());
        assert!(!IntegerRange::new(10, 0, 1).is_valid());
        assert!(!IntegerRange::new(0, 10, -1).is_valid());
        assert!(FloatingPointRange::new(0.0, 1.0, 0.0).is_valid());
        assert!(!FloatingPointRange::new(1.0, 0.0, 0.1).is_valid());
        assert!(!FloatingPointRange::new(0.0, 1.0, -0.1).is_valid());
        assert!(!FloatingPointRange::new(0.0, 1.0, f64::NAN).is_valid());
        assert!(ParameterRange::None.is_valid());
    }

    #[test]
    fn descriptor_validate_range_enforces_step_on_both_arms() {
        let int_desc = ParameterDescriptor::new("i", ParameterType::Integer)
            .unwrap()
            .with_integer_range(0, 100, 10);
        assert!(int_desc.validate_range(&ParameterValue::Integer(30)));
        assert!(!int_desc.validate_range(&ParameterValue::Integer(31)));
        // A value of the other type is not the range's business.
        assert!(int_desc.validate_range(&ParameterValue::Double(31.0)));

        let f_desc = ParameterDescriptor::new("f", ParameterType::Double)
            .unwrap()
            .with_float_range(0.0, 10.0, 0.5);
        assert!(f_desc.validate_range(&ParameterValue::Double(2.5)));
        assert!(!f_desc.validate_range(&ParameterValue::Double(2.7)));
        assert!(f_desc.validate_range(&ParameterValue::Integer(3)));
    }

    #[test]
    #[allow(clippy::approx_constant)]
    fn test_parameter_value_types() {
        let bool_val = ParameterValue::Bool(true);
        assert_eq!(bool_val.param_type(), ParameterType::Bool);
        assert_eq!(bool_val.as_bool(), Some(true));

        let int_val = ParameterValue::Integer(42);
        assert_eq!(int_val.param_type(), ParameterType::Integer);
        assert_eq!(int_val.as_integer(), Some(42));

        let double_val = ParameterValue::Double(3.14);
        assert_eq!(double_val.param_type(), ParameterType::Double);
        assert_eq!(double_val.as_double(), Some(3.14));
    }

    #[test]
    fn test_parameter_value_string() {
        let string_val = ParameterValue::from_string("hello").unwrap();
        assert_eq!(string_val.param_type(), ParameterType::String);
        assert_eq!(string_val.as_string(), Some("hello"));
    }

    #[test]
    fn test_parameter_creation() {
        let param = Parameter::bool("my_param", true).unwrap();
        assert_eq!(param.name(), "my_param");
        assert_eq!(param.param_type(), ParameterType::Bool);
        assert_eq!(param.value.as_bool(), Some(true));
    }

    #[test]
    fn test_parameter_descriptor() {
        let desc = ParameterDescriptor::new("speed", ParameterType::Double)
            .unwrap()
            .with_description("Maximum speed in m/s")
            .with_float_range(0.0, 10.0, 0.1);

        assert_eq!(desc.name.as_str(), "speed");
        assert_eq!(desc.param_type, ParameterType::Double);
        assert!(!desc.read_only);

        // Test range validation
        assert!(desc.validate_range(&ParameterValue::Double(5.0)));
        assert!(!desc.validate_range(&ParameterValue::Double(15.0)));
    }

    #[test]
    fn test_integer_range() {
        let range = IntegerRange::new(0, 100, 1);
        assert!(range.contains(50));
        assert!(range.contains(0));
        assert!(range.contains(100));
        assert!(!range.contains(-1));
        assert!(!range.contains(101));
    }

    #[test]
    fn test_set_result() {
        assert!(SetParameterResult::Success.is_success());
        assert!(!SetParameterResult::ReadOnly.is_success());
    }
}

// =============================================================================
// Ghost model validation
// =============================================================================

#[cfg(test)]
mod ghost_checks {
    use super::*;
    use nros_ghost_types::ParameterValueGhost;

    /// Structural check: exhaustive match maps all 10 variants.
    /// If a variant is added or removed, this fails to compile.
    fn ghost_from_value(v: &ParameterValue) -> ParameterValueGhost {
        match v {
            ParameterValue::NotSet => ParameterValueGhost::NotSet,
            ParameterValue::Bool(b) => ParameterValueGhost::Bool(*b),
            ParameterValue::Integer(i) => ParameterValueGhost::Integer(*i),
            ParameterValue::Double(_) => ParameterValueGhost::Double,
            ParameterValue::String(_) => ParameterValueGhost::String,
            ParameterValue::ByteArray(_) => ParameterValueGhost::ByteArray,
            ParameterValue::BoolArray(_) => ParameterValueGhost::BoolArray,
            ParameterValue::IntegerArray(_) => ParameterValueGhost::IntegerArray,
            ParameterValue::DoubleArray(_) => ParameterValueGhost::DoubleArray,
            ParameterValue::StringArray(_) => ParameterValueGhost::StringArray,
        }
    }

    #[test]
    fn ghost_variant_coverage() {
        // Exhaustive match compiles — all 10 variants exist in both types
        let _ = ghost_from_value(&ParameterValue::NotSet);
        let _ = ghost_from_value(&ParameterValue::Bool(true));
        let _ = ghost_from_value(&ParameterValue::Integer(0));
        let _ = ghost_from_value(&ParameterValue::Double(0.0));
        let _ = ghost_from_value(&ParameterValue::String(String::new()));
        let _ = ghost_from_value(&ParameterValue::ByteArray(Vec::new()));
        let _ = ghost_from_value(&ParameterValue::BoolArray(Vec::new()));
        let _ = ghost_from_value(&ParameterValue::IntegerArray(Vec::new()));
        let _ = ghost_from_value(&ParameterValue::DoubleArray(Vec::new()));
        let _ = ghost_from_value(&ParameterValue::StringArray(Vec::new()));
    }

    #[test]
    fn ghost_bool_preserves() {
        let ghost = ghost_from_value(&ParameterValue::Bool(true));
        assert!(matches!(ghost, ParameterValueGhost::Bool(true)));
    }

    #[test]
    fn ghost_integer_preserves() {
        let ghost = ghost_from_value(&ParameterValue::Integer(42));
        assert!(matches!(ghost, ParameterValueGhost::Integer(42)));
    }
}

// =============================================================================
// Kani bounded model checking proofs
// =============================================================================

#[cfg(kani)]
mod verification {
    use super::*;

    // ---- ParameterValue type conversions ----

    #[kani::proof]
    fn parameter_i64_roundtrip() {
        let val: i64 = kani::any();
        let pv = ParameterValue::from_integer(val);
        assert_eq!(pv.as_integer(), Some(val));
        assert_eq!(pv.param_type(), ParameterType::Integer);
        assert!(pv.is_set());
    }

    #[kani::proof]
    fn parameter_bool_roundtrip() {
        let val: bool = kani::any();
        let pv = ParameterValue::from_bool(val);
        assert_eq!(pv.as_bool(), Some(val));
        assert_eq!(pv.param_type(), ParameterType::Bool);
        assert!(pv.is_set());
    }

    #[kani::proof]
    fn parameter_double_roundtrip() {
        let val: f64 = kani::any();
        let pv = ParameterValue::from_double(val);
        let result = pv.as_double();
        assert!(result.is_some());
        assert_eq!(result.unwrap().to_bits(), val.to_bits());
        assert_eq!(pv.param_type(), ParameterType::Double);
    }

    #[kani::proof]
    fn parameter_not_set_default() {
        let pv = ParameterValue::default();
        assert!(!pv.is_set());
        assert_eq!(pv.param_type(), ParameterType::NotSet);
        assert!(pv.as_bool().is_none());
        assert!(pv.as_integer().is_none());
        assert!(pv.as_double().is_none());
        assert!(pv.as_string().is_none());
    }

    // ---- Type mismatch returns None ----

    #[kani::proof]
    fn parameter_type_mismatch_bool() {
        let val: i64 = kani::any();
        let pv = ParameterValue::from_integer(val);
        assert!(pv.as_bool().is_none());
        assert!(pv.as_double().is_none());
        assert!(pv.as_string().is_none());
    }

    #[kani::proof]
    fn parameter_type_mismatch_integer() {
        let val: bool = kani::any();
        let pv = ParameterValue::from_bool(val);
        assert!(pv.as_integer().is_none());
        assert!(pv.as_double().is_none());
        assert!(pv.as_string().is_none());
    }

    // ---- IntegerRange ----

    #[kani::proof]
    fn integer_range_contains_bounds() {
        let min: i64 = kani::any();
        let max: i64 = kani::any();
        // Constrain to bounded range to avoid subtraction overflow
        kani::assume(min >= -1_000_000 && min <= 1_000_000);
        kani::assume(max >= -1_000_000 && max <= 1_000_000);
        kani::assume(min <= max);
        let range = IntegerRange::new(min, max, 1);
        // Endpoints must be contained
        assert!(range.contains(min));
        assert!(range.contains(max));
    }

    #[kani::proof]
    fn integer_range_outside_bounds() {
        let min: i64 = kani::any();
        let max: i64 = kani::any();
        kani::assume(min <= max);
        kani::assume(min > i64::MIN); // So min-1 doesn't overflow
        kani::assume(max < i64::MAX); // So max+1 doesn't overflow
        let range = IntegerRange::new(min, max, 1);
        assert!(!range.contains(min - 1));
        assert!(!range.contains(max + 1));
    }

    // ---- FloatingPointRange ----

    #[kani::proof]
    fn float_range_contains_bounds() {
        let min: f64 = kani::any();
        let max: f64 = kani::any();
        kani::assume(!min.is_nan() && !max.is_nan());
        kani::assume(min <= max);
        let range = FloatingPointRange::new(min, max, 0.0);
        assert!(range.contains(min));
        assert!(range.contains(max));
    }

    // ---- SetParameterResult ----

    #[kani::proof]
    fn set_result_success_only() {
        // Only Success should return true from is_success()
        assert!(SetParameterResult::Success.is_success());
        assert!(!SetParameterResult::ReadOnly.is_success());
        assert!(!SetParameterResult::TypeMismatch.is_success());
        assert!(!SetParameterResult::OutOfRange.is_success());
        assert!(!SetParameterResult::NotFound.is_success());
        assert!(!SetParameterResult::StorageFull.is_success());
    }
}

// The heap is what these need — the over-long values they build are a `Vec` and
// a `String`, which `alloc` has.
#[cfg(all(test, feature = "alloc"))]
mod issue_0323_tests {
    use super::*;

    /// issue 0323 — an over-long hosted `String` used to become
    /// `ParameterValue::NotSet` via `unwrap_or_default()`: a TYPE change the
    /// caller could not distinguish from deliberately clearing the parameter.
    #[test]
    fn oversize_string_is_rejected_not_silently_notset() {
        let long = "x".repeat(MAX_STRING_VALUE_LEN + 1);
        assert!(
            matches!(long.to_parameter_value(), ParameterValue::NotSet),
            "documenting the legacy infallible behaviour"
        );
        assert!(
            long.try_to_parameter_value().is_err(),
            "the fallible path must reject it"
        );
    }

    /// The array case: `unwrap_or_default()` yielded an EMPTY array.
    #[test]
    fn oversize_integer_array_is_rejected_not_silently_empty() {
        let big: alloc::vec::Vec<i64> = (0..(MAX_ARRAY_LEN as i64 + 1)).collect();
        match big.to_parameter_value() {
            ParameterValue::IntegerArray(v) => {
                assert!(v.is_empty(), "documenting the legacy empty-array behaviour")
            }
            other => panic!("unexpected {other:?}"),
        }
        assert!(big.try_to_parameter_value().is_err());
    }

    /// `Vec<String>` silently SKIPPED elements that did not fit.
    #[test]
    fn oversize_string_array_element_is_rejected_not_skipped() {
        let big: alloc::vec::Vec<alloc::string::String> =
            (0..(MAX_ARRAY_LEN + 1)).map(|i| i.to_string()).collect();
        assert!(big.try_to_parameter_value().is_err());
    }
}
