//! Parameter types for ROS 2 compatible parameter handling
//!
//! This module provides types for representing ROS 2 parameters including
//! scalar values, arrays, and parameter descriptors.

use heapless::{String, Vec};

/// Maximum length for parameter names
pub const MAX_PARAM_NAME_LEN: usize = 64;

/// Maximum length for parameter string values
pub const MAX_STRING_VALUE_LEN: usize = 256;

/// Maximum length for array parameters
pub const MAX_ARRAY_LEN: usize = 32;

/// Maximum length for byte array parameters
pub const MAX_BYTE_ARRAY_LEN: usize = 256;

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

    /// Check if a value is within this range
    pub fn contains(&self, value: f64) -> bool {
        value >= self.min && value <= self.max
    }
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

    /// Check if a value is within this range
    pub fn contains(&self, value: i64) -> bool {
        value >= self.min && value <= self.max
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

    /// Check if a value satisfies the range constraints
    pub fn validate_range(&self, value: &ParameterValue) -> bool {
        match (&self.range, value) {
            (ParameterRange::None, _) => true,
            (ParameterRange::Integer(range), ParameterValue::Integer(v)) => range.contains(*v),
            (ParameterRange::FloatingPoint(range), ParameterValue::Double(v)) => range.contains(*v),
            _ => true,
        }
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
}

impl SetParameterResult {
    /// Check if the result indicates success
    pub fn is_success(&self) -> bool {
        matches!(self, Self::Success)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
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
