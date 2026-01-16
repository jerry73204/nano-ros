//! Parameter types

/// Parameter type enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterType {
    NotSet,
    Bool,
    Integer,
    Double,
    String,
    ByteArray,
    BoolArray,
    IntegerArray,
    DoubleArray,
    StringArray,
}

/// Parameter value container
pub struct ParameterValue {
    pub param_type: ParameterType,
}

/// A named parameter
pub struct Parameter {
    pub param_type: ParameterType,
}
