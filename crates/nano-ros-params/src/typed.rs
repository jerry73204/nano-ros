//! Typed parameter API
//!
//! This module provides a fluent builder pattern for declaring typed parameters
//! in a ROS 2 node, aligning with the rclrs API.

use crate::{
    ParameterDescriptor, ParameterRange, ParameterServer, ParameterType, SetParameterResult,
};
use heapless::String;

/// Trait for types that can be used as typed parameters
pub use crate::ParameterVariant;

/// Error type for typed parameter operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterError {
    /// Parameter already declared with a different type
    TypeMismatch,
    /// Value is outside allowed range
    OutOfRange,
    /// Parameter is read-only
    ReadOnly,
    /// Parameter not found
    NotFound,
    /// Internal storage is full
    StorageFull,
    /// String conversion failed
    StringConversion,
    /// Invalid range for type
    InvalidRange,
}

impl From<SetParameterResult> for ParameterError {
    fn from(result: SetParameterResult) -> Self {
        match result {
            SetParameterResult::TypeMismatch => ParameterError::TypeMismatch,
            SetParameterResult::OutOfRange => ParameterError::OutOfRange,
            SetParameterResult::ReadOnly => ParameterError::ReadOnly,
            SetParameterResult::NotFound => ParameterError::NotFound,
            SetParameterResult::StorageFull => ParameterError::StorageFull,
            _ => panic!("Unexpected SetParameterResult"), // Should not happen with valid results
        }
    }
}

/// Builder for declaring a typed parameter
pub struct ParameterBuilder<'a, T: ParameterVariant> {
    /// Reference to the parameter server
    server: &'a mut ParameterServer,
    /// Parameter name
    name: &'a str,
    /// Default value
    default: Option<T>,
    /// Human-readable description
    description: Option<&'a str>,
    /// Range constraints
    range: Option<ParameterRange>,
    /// Whether the parameter is read-only
    read_only: bool,
    /// Phantom data to hold the type parameter
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, T: ParameterVariant> ParameterBuilder<'a, T> {
    /// Create a new parameter builder
    pub(crate) fn new(server: &'a mut ParameterServer, name: &'a str) -> Self {
        Self {
            server,
            name,
            default: None,
            description: None,
            range: None,
            read_only: false,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Set a default value for the parameter
    pub fn default(mut self, value: T) -> Self {
        self.default = Some(value);
        self
    }

    /// Set a human-readable description for the parameter
    pub fn description(mut self, desc: &'a str) -> Self {
        self.description = Some(desc);
        self
    }

    /// Set integer range constraints for the parameter
    pub fn integer_range(mut self, min: i64, max: i64, step: i64) -> Result<Self, ParameterError> {
        if T::parameter_type() != ParameterType::Integer {
            return Err(ParameterError::InvalidRange);
        }
        self.range = Some(ParameterRange::Integer(crate::IntegerRange::new(
            min, max, step,
        )));
        Ok(self)
    }

    /// Set floating point range constraints for the parameter
    pub fn float_range(mut self, min: f64, max: f64, step: f64) -> Result<Self, ParameterError> {
        if T::parameter_type() != ParameterType::Double {
            return Err(ParameterError::InvalidRange);
        }
        self.range = Some(ParameterRange::FloatingPoint(
            crate::FloatingPointRange::new(min, max, step),
        ));
        Ok(self)
    }

    /// Mark the parameter as read-only
    pub fn read_only(mut self) -> Self {
        self.read_only = true;
        self
    }

    /// Declare a mandatory parameter
    ///
    /// If no default value is provided, it must be set externally before use.
    pub fn mandatory(self) -> Result<MandatoryParameter<'a, T>, ParameterError> {
        let mut descriptor = ParameterDescriptor::new(self.name, T::parameter_type())
            .ok_or(ParameterError::StorageFull)?;
        descriptor.description.clear();
        if let Some(desc) = self.description {
            descriptor
                .description
                .push_str(desc)
                .map_err(|_| ParameterError::StringConversion)?;
        }
        descriptor.read_only = self.read_only;
        descriptor.range = self.range.unwrap_or_default();

        let default_value = self.default.map(|v| v.to_parameter_value());

        self.server
            .declare_parameter(descriptor, default_value.as_ref())?;

        Ok(MandatoryParameter::new(self.server, self.name))
    }

    /// Declare an optional parameter
    pub fn optional(self) -> Result<OptionalParameter<'a, T>, ParameterError> {
        let mut descriptor = ParameterDescriptor::new(self.name, T::parameter_type())
            .ok_or(ParameterError::StorageFull)?;
        descriptor.description.clear();
        if let Some(desc) = self.description {
            descriptor
                .description
                .push_str(desc)
                .map_err(|_| ParameterError::StringConversion)?;
        }
        descriptor.read_only = self.read_only;
        descriptor.range = self.range.unwrap_or_default();

        let default_value = self.default.map(|v| v.to_parameter_value());

        self.server
            .declare_parameter(descriptor, default_value.as_ref())?;

        Ok(OptionalParameter::new(self.server, self.name))
    }
}

/// A parameter that must always have a value
pub struct MandatoryParameter<'a, T: ParameterVariant> {
    server: &'a mut ParameterServer,
    name: String<{ crate::MAX_PARAM_NAME_LEN }>,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, T: ParameterVariant> MandatoryParameter<'a, T> {
    pub(crate) fn new(server: &'a mut ParameterServer, name: &'a str) -> Self {
        let mut n = String::new();
        n.push_str(name).unwrap();
        Self {
            server,
            name: n,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Get the current value of the parameter
    pub fn get(&self) -> T {
        self.server
            .get_parameter_value(self.name.as_str())
            .and_then(|val| T::from_parameter_value(&val))
            .expect("Mandatory parameter must have a value")
    }

    /// Set the value of the parameter
    pub fn set(&mut self, value: T) -> Result<(), ParameterError> {
        let result = self
            .server
            .set_parameter_value(self.name.as_str(), value.to_parameter_value());
        if result == SetParameterResult::Success {
            Ok(())
        } else {
            Err(ParameterError::from(result))
        }
    }
}

/// A parameter that may or may not have a value
pub struct OptionalParameter<'a, T: ParameterVariant> {
    server: &'a mut ParameterServer,
    name: String<{ crate::MAX_PARAM_NAME_LEN }>,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, T: ParameterVariant> OptionalParameter<'a, T> {
    pub(crate) fn new(server: &'a mut ParameterServer, name: &'a str) -> Self {
        let mut n = String::new();
        n.push_str(name).unwrap();
        Self {
            server,
            name: n,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Get the current value of the parameter, if set
    pub fn get(&self) -> Option<T> {
        self.server
            .get_parameter_value(self.name.as_str())
            .and_then(|val| T::from_parameter_value(&val))
    }

    /// Set the value of the parameter
    pub fn set(&mut self, value: Option<T>) -> Result<(), ParameterError> {
        let param_value = value.map(|v| v.to_parameter_value());
        let result = self
            .server
            .set_parameter_value(self.name.as_str(), param_value.unwrap_or_default());
        if result == SetParameterResult::Success {
            Ok(())
        } else {
            Err(ParameterError::from(result))
        }
    }
}

/// A parameter whose value cannot be changed after declaration
pub struct ReadOnlyParameter<'a, T: ParameterVariant> {
    server: &'a mut ParameterServer,
    name: String<{ crate::MAX_PARAM_NAME_LEN }>,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, T: ParameterVariant> ReadOnlyParameter<'a, T> {
    pub(crate) fn new(server: &'a mut ParameterServer, name: &'a str) -> Self {
        let mut n = String::new();
        n.push_str(name).unwrap();
        Self {
            server,
            name: n,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Get the current value of the parameter
    pub fn get(&self) -> T {
        self.server
            .get_parameter_value(self.name.as_str())
            .and_then(|val| T::from_parameter_value(&val))
            .expect("Read-only parameter must have a value")
    }
}

/// Provides access to undeclared parameters in a ParameterServer
///
/// This struct is returned by `Node::use_undeclared_parameters()` and allows
/// for dynamic retrieval of parameter values by name without explicit declaration.
pub struct UndeclaredParameters<'a> {
    server: &'a mut ParameterServer,
}

impl<'a> UndeclaredParameters<'a> {
    pub(crate) fn new(server: &'a mut ParameterServer) -> Self {
        Self { server }
    }

    /// Try to get the value of an undeclared boolean parameter
    pub fn get_bool(&self, name: &str) -> Option<bool> {
        self.server.get_bool(name)
    }

    /// Try to get the value of an undeclared integer parameter
    pub fn get_integer(&self, name: &str) -> Option<i64> {
        self.server.get_integer(name)
    }

    /// Try to get the value of an undeclared double parameter
    pub fn get_double(&self, name: &str) -> Option<f64> {
        self.server.get_double(name)
    }

    /// Try to get the value of an undeclared string parameter
    pub fn get_string(&self, name: &str) -> Option<&str> {
        self.server.get_string(name)
    }
}
