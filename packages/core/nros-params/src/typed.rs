//! Typed parameter API
//!
//! This module provides a fluent builder pattern for declaring typed parameters
//! in a ROS 2 node, aligning with the rclrs API.

#[cfg(test)]
use crate::ParameterStorage;
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
            SetParameterResult::InvalidRange => ParameterError::InvalidRange,
            // The typed API never auto-declares, so an unknown name is
            // simply not there.
            SetParameterResult::Undeclared => ParameterError::NotFound,
            SetParameterResult::Success => {
                panic!("SetParameterResult::Success is not an error")
            }
        }
    }
}

/// Trait for types that can be converted to a parameter range from `RangeInclusive`
pub trait RangeConvertible: ParameterVariant + Clone {
    /// Convert a `RangeInclusive<Self>` to a `ParameterRange`
    fn to_parameter_range(
        range: core::ops::RangeInclusive<Self>,
    ) -> Result<ParameterRange, ParameterError>;
}

impl RangeConvertible for i64 {
    fn to_parameter_range(
        range: core::ops::RangeInclusive<Self>,
    ) -> Result<ParameterRange, ParameterError> {
        Ok(ParameterRange::Integer(crate::IntegerRange::new(
            *range.start(),
            *range.end(),
            1, // Default step of 1
        )))
    }
}

impl RangeConvertible for f64 {
    fn to_parameter_range(
        range: core::ops::RangeInclusive<Self>,
    ) -> Result<ParameterRange, ParameterError> {
        Ok(ParameterRange::FloatingPoint(
            crate::FloatingPointRange::new(
                *range.start(),
                *range.end(),
                0.0, // No step constraint (any value in range is valid)
            ),
        ))
    }
}

/// Builder for declaring a typed parameter
pub struct ParameterBuilder<'a, 's, T: ParameterVariant> {
    /// Borrow of the parameter server. phase-382 W2' — the server itself
    /// borrows its table, so every handle onto it carries BOTH lifetimes:
    /// `'a` is this borrow, `'s` is the caller-owned storage underneath.
    server: &'a mut ParameterServer<'s>,
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

impl<'a, 's, T: ParameterVariant> ParameterBuilder<'a, 's, T> {
    /// Create a new parameter builder
    pub fn new(server: &'a mut ParameterServer<'s>, name: &'a str) -> Self {
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
        let range = crate::IntegerRange::new(min, max, step);
        // Issue 1150 — `min > max` or a negative step is refused here, where
        // the caller can see it, not silently stored and published.
        if !range.is_valid() {
            return Err(ParameterError::InvalidRange);
        }
        self.range = Some(ParameterRange::Integer(range));
        Ok(self)
    }

    /// Set floating point range constraints for the parameter
    pub fn float_range(mut self, min: f64, max: f64, step: f64) -> Result<Self, ParameterError> {
        if T::parameter_type() != ParameterType::Double {
            return Err(ParameterError::InvalidRange);
        }
        let range = crate::FloatingPointRange::new(min, max, step);
        if !range.is_valid() {
            return Err(ParameterError::InvalidRange);
        }
        self.range = Some(ParameterRange::FloatingPoint(range));
        Ok(self)
    }

    /// Set range constraints using an inclusive range
    ///
    /// This is a convenience method that works with `RangeInclusive`:
    /// - For `i64` parameters: `range(0..=100)` sets an integer range with step 1
    /// - For `f64` parameters: `range(0.0..=1.0)` sets a floating point range with step 0.0
    ///
    /// For more control (e.g., custom step), use `integer_range()` or `float_range()`.
    pub fn range(mut self, range: core::ops::RangeInclusive<T>) -> Result<Self, ParameterError>
    where
        T: RangeConvertible,
    {
        self.range = Some(T::to_parameter_range(range)?);
        Ok(self)
    }

    /// Declare a read-only parameter
    ///
    /// Read-only parameters cannot be changed after declaration.
    /// A default value must be provided.
    pub fn read_only(self) -> Result<ReadOnlyParameter<'a, 's, T>, ParameterError> {
        // Must have a default value for read-only parameters
        let default_value = self
            .default
            .as_ref()
            .ok_or(ParameterError::NotFound)?
            .clone();

        let mut descriptor = ParameterDescriptor::new(self.name, T::parameter_type())
            .ok_or(ParameterError::StorageFull)?;
        descriptor.description.clear();
        if let Some(desc) = self.description {
            descriptor
                .description
                .push_str(desc)
                .map_err(|_| ParameterError::StringConversion)?;
        }
        descriptor.read_only = true;
        descriptor.range = self.range.unwrap_or_default();

        let param_value = default_value.to_parameter_value();

        self.server
            .declare_parameter(descriptor, Some(&param_value))?;

        Ok(ReadOnlyParameter::new(self.server, self.name))
    }

    /// Declare a mandatory parameter
    ///
    /// If no default value is provided, it must be set externally before use.
    pub fn mandatory(self) -> Result<MandatoryParameter<'a, 's, T>, ParameterError> {
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
    pub fn optional(self) -> Result<OptionalParameter<'a, 's, T>, ParameterError> {
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
pub struct MandatoryParameter<'a, 's, T: ParameterVariant> {
    server: &'a mut ParameterServer<'s>,
    name: String<{ crate::MAX_PARAM_NAME_LEN }>,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, 's, T: ParameterVariant> MandatoryParameter<'a, 's, T> {
    pub(crate) fn new(server: &'a mut ParameterServer<'s>, name: &'a str) -> Self {
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
        // issue 0323 — reject an over-capacity value here rather than letting
        // `to_parameter_value`'s `unwrap_or_default()` turn it into `NotSet`
        // (a TYPE change) or an EMPTY array and then reporting success.
        let converted = value
            .try_to_parameter_value()
            .map_err(|_| ParameterError::StringConversion)?;
        let result = self
            .server
            .set_parameter_value(self.name.as_str(), converted);
        if result == SetParameterResult::Success {
            Ok(())
        } else {
            Err(ParameterError::from(result))
        }
    }
}

/// A parameter that may or may not have a value
pub struct OptionalParameter<'a, 's, T: ParameterVariant> {
    server: &'a mut ParameterServer<'s>,
    name: String<{ crate::MAX_PARAM_NAME_LEN }>,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, 's, T: ParameterVariant> OptionalParameter<'a, 's, T> {
    pub(crate) fn new(server: &'a mut ParameterServer<'s>, name: &'a str) -> Self {
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
pub struct ReadOnlyParameter<'a, 's, T: ParameterVariant> {
    server: &'a mut ParameterServer<'s>,
    name: String<{ crate::MAX_PARAM_NAME_LEN }>,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, 's, T: ParameterVariant> ReadOnlyParameter<'a, 's, T> {
    pub(crate) fn new(server: &'a mut ParameterServer<'s>, name: &'a str) -> Self {
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
pub struct UndeclaredParameters<'a, 's> {
    server: &'a mut ParameterServer<'s>,
}

impl<'a, 's> UndeclaredParameters<'a, 's> {
    pub fn new(server: &'a mut ParameterServer<'s>) -> Self {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mandatory_parameter_with_default() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let param = ParameterBuilder::<i64>::new(&mut server, "test_param")
            .default(42)
            .description("A test parameter")
            .mandatory()
            .expect("Failed to declare parameter");

        assert_eq!(param.get(), 42);
    }

    #[test]
    fn test_mandatory_parameter_set() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let mut param = ParameterBuilder::<i64>::new(&mut server, "test_param")
            .default(0)
            .mandatory()
            .expect("Failed to declare parameter");

        param.set(100).expect("Failed to set parameter");
        assert_eq!(param.get(), 100);
    }

    #[test]
    fn test_optional_parameter_none() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let param = ParameterBuilder::<i64>::new(&mut server, "test_param")
            .optional()
            .expect("Failed to declare parameter");

        assert_eq!(param.get(), None);
    }

    #[test]
    fn test_optional_parameter_with_default() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let param = ParameterBuilder::<i64>::new(&mut server, "test_param")
            .default(42)
            .optional()
            .expect("Failed to declare parameter");

        assert_eq!(param.get(), Some(42));
    }

    #[test]
    fn test_optional_parameter_set() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let mut param = ParameterBuilder::<i64>::new(&mut server, "test_param")
            .optional()
            .expect("Failed to declare parameter");

        param.set(Some(100)).expect("Failed to set parameter");
        assert_eq!(param.get(), Some(100));
    }

    #[test]
    fn test_read_only_parameter() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let param = ParameterBuilder::<i64>::new(&mut server, "readonly_param")
            .default(42)
            .description("A read-only parameter")
            .read_only()
            .expect("Failed to declare parameter");

        assert_eq!(param.get(), 42);
    }

    #[test]
    fn test_read_only_parameter_requires_default() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let result = ParameterBuilder::<i64>::new(&mut server, "readonly_param").read_only();

        assert_eq!(result.err(), Some(ParameterError::NotFound));
    }

    #[test]
    fn test_integer_range_constraint() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let mut param = ParameterBuilder::<i64>::new(&mut server, "ranged_param")
            .default(50)
            .integer_range(0, 100, 1)
            .expect("Failed to set range")
            .mandatory()
            .expect("Failed to declare parameter");

        // Valid value within range
        param.set(75).expect("Failed to set valid value");
        assert_eq!(param.get(), 75);
    }

    #[test]
    fn test_float_range_constraint() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let mut param = ParameterBuilder::<f64>::new(&mut server, "float_param")
            .default(0.5)
            .float_range(0.0, 1.0, 0.0)
            .expect("Failed to set range")
            .mandatory()
            .expect("Failed to declare parameter");

        // Valid value within range
        param.set(0.75).expect("Failed to set valid value");
        assert_eq!(param.get(), 0.75);
    }

    #[test]
    fn test_range_convenience_integer() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let param = ParameterBuilder::<i64>::new(&mut server, "ranged_param")
            .default(50)
            .range(0..=100)
            .expect("Failed to set range")
            .mandatory()
            .expect("Failed to declare parameter");

        assert_eq!(param.get(), 50);
    }

    #[test]
    fn test_range_convenience_float() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let param = ParameterBuilder::<f64>::new(&mut server, "float_param")
            .default(0.5)
            .range(0.0..=1.0)
            .expect("Failed to set range")
            .mandatory()
            .expect("Failed to declare parameter");

        assert_eq!(param.get(), 0.5);
    }

    #[test]
    fn test_parameter_description() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let _param = ParameterBuilder::<i64>::new(&mut server, "described_param")
            .default(42)
            .description("This is a test description")
            .mandatory()
            .expect("Failed to declare parameter");

        // Verify description was set in the server
        let desc = server.get_descriptor("described_param");
        assert!(desc.is_some());
        assert_eq!(
            desc.unwrap().description.as_str(),
            "This is a test description"
        );
    }

    #[test]
    fn test_bool_parameter() {
        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());
        let mut param = ParameterBuilder::<bool>::new(&mut server, "bool_param")
            .default(false)
            .mandatory()
            .expect("Failed to declare parameter");

        assert!(!param.get());
        param.set(true).expect("Failed to set parameter");
        assert!(param.get());
    }

    #[test]
    fn test_undeclared_parameters() {
        use crate::ParameterValue;

        let mut storage: ParameterStorage = ParameterStorage::new();
        let mut server = ParameterServer::new_in(storage.as_table());

        // Set some values using set_or_declare (simulating external parameter loading)
        server.set_or_declare("flag", ParameterValue::Bool(true));
        server.set_or_declare("count", ParameterValue::Integer(42));
        server.set_or_declare("ratio", ParameterValue::Double(0.5));

        let undeclared = UndeclaredParameters::new(&mut server);

        assert_eq!(undeclared.get_bool("flag"), Some(true));
        assert_eq!(undeclared.get_integer("count"), Some(42));
        assert_eq!(undeclared.get_double("ratio"), Some(0.5));
        assert_eq!(undeclared.get_string("nonexistent"), None);
    }
}
