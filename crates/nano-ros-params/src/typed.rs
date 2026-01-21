//! Typed parameter API for type-safe parameter access
//!
//! This module provides a fluent API for declaring and using typed parameters
//! that match the rclrs 0.6.0 patterns.

use crate::server::ParameterServer;
use crate::types::{
    FloatingPointRange, IntegerRange, ParameterDescriptor, ParameterRange, ParameterValue,
    ParameterVariant, SetParameterResult,
};

/// Error type for parameter operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterError {
    /// Parameter not found
    NotFound,
    /// Parameter type mismatch
    TypeMismatch,
    /// Parameter is read-only
    ReadOnly,
    /// Value out of allowed range
    OutOfRange,
    /// Parameter storage is full
    StorageFull,
    /// Invalid parameter name
    InvalidName,
}

impl From<SetParameterResult> for ParameterError {
    fn from(result: SetParameterResult) -> Self {
        match result {
            SetParameterResult::Success => unreachable!("Cannot convert Success to error"),
            SetParameterResult::ReadOnly => Self::ReadOnly,
            SetParameterResult::TypeMismatch => Self::TypeMismatch,
            SetParameterResult::OutOfRange => Self::OutOfRange,
            SetParameterResult::NotFound => Self::NotFound,
            SetParameterResult::StorageFull => Self::StorageFull,
        }
    }
}

/// Builder for declaring typed parameters
///
/// # Example
///
/// ```ignore
/// let speed: MandatoryParameter<f64> = node
///     .declare_parameter("speed")
///     .default(5.0)
///     .range(0.0, 10.0, 0.1)
///     .description("Maximum speed in m/s")
///     .mandatory()?;
/// ```
pub struct ParameterBuilder<'a, T: ParameterVariant> {
    server: &'a mut ParameterServer,
    name: &'a str,
    default: Option<T>,
    description: Option<&'a str>,
    range: Option<ParameterRange>,
    read_only: bool,
}

impl<'a, T: ParameterVariant> ParameterBuilder<'a, T> {
    /// Create a new parameter builder
    ///
    /// This is typically called via `Node::declare_parameter()` rather than directly.
    pub fn new(server: &'a mut ParameterServer, name: &'a str) -> Self {
        Self {
            server,
            name,
            default: None,
            description: None,
            range: None,
            read_only: false,
        }
    }

    /// Set the default value for this parameter
    ///
    /// If the parameter already exists with a different value, the existing
    /// value is preserved. Otherwise, the default is used.
    pub fn default(mut self, value: T) -> Self {
        self.default = Some(value);
        self
    }

    /// Set a human-readable description
    pub fn description(mut self, desc: &'a str) -> Self {
        self.description = Some(desc);
        self
    }

    /// Set an integer range constraint (only valid for i64 parameters)
    pub fn integer_range(mut self, min: i64, max: i64, step: i64) -> Self {
        self.range = Some(ParameterRange::Integer(IntegerRange::new(min, max, step)));
        self
    }

    /// Set a floating point range constraint (only valid for f64 parameters)
    pub fn range(mut self, min: f64, max: f64, step: f64) -> Self {
        self.range = Some(ParameterRange::FloatingPoint(FloatingPointRange::new(
            min, max, step,
        )));
        self
    }

    /// Make this parameter read-only
    ///
    /// Read-only parameters cannot be modified after declaration.
    pub fn read_only(mut self) -> Self {
        self.read_only = true;
        self
    }

    /// Declare as a mandatory parameter
    ///
    /// Mandatory parameters must have a value. If no default is provided
    /// and the parameter doesn't already exist, this returns an error.
    pub fn mandatory(self) -> Result<MandatoryParameter<'a, T>, ParameterError> {
        // Check if parameter already exists
        let existing = self.server.get(self.name);

        let value = if let Some(existing_value) = existing {
            // Use existing value if type matches
            T::from_parameter_value(existing_value).ok_or(ParameterError::TypeMismatch)?
        } else if let Some(default) = self.default {
            // Use default value
            default
        } else {
            // No value and no default - mandatory parameter requires a value
            return Err(ParameterError::NotFound);
        };

        // Create descriptor if needed
        if existing.is_none() {
            let mut desc = ParameterDescriptor::new(self.name, T::parameter_type())
                .ok_or(ParameterError::InvalidName)?;

            if let Some(description) = self.description {
                desc = desc.with_description(description);
            }

            if let Some(range) = self.range {
                desc.range = range;
            }

            desc = desc.with_read_only(self.read_only);

            // Declare with descriptor
            self.server
                .declare_with_descriptor(self.name, value.to_parameter_value(), Some(desc));
        }

        Ok(MandatoryParameter {
            server: self.server,
            name: self.name,
            _phantom: core::marker::PhantomData,
        })
    }

    /// Declare as an optional parameter
    ///
    /// Optional parameters may or may not have a value.
    pub fn optional(self) -> Result<OptionalParameter<'a, T>, ParameterError> {
        // Check if parameter already exists
        let existing = self.server.get(self.name);

        // If exists, validate type
        if let Some(existing_value) = existing {
            T::from_parameter_value(existing_value).ok_or(ParameterError::TypeMismatch)?;
        } else if let Some(default) = self.default {
            // Declare with default
            let mut desc = ParameterDescriptor::new(self.name, T::parameter_type())
                .ok_or(ParameterError::InvalidName)?;

            if let Some(description) = self.description {
                desc = desc.with_description(description);
            }

            if let Some(range) = self.range {
                desc.range = range;
            }

            desc = desc.with_read_only(self.read_only);

            self.server.declare_with_descriptor(
                self.name,
                default.to_parameter_value(),
                Some(desc),
            );
        } else {
            // Declare without value (optional)
            let mut desc = ParameterDescriptor::new(self.name, T::parameter_type())
                .ok_or(ParameterError::InvalidName)?;

            if let Some(description) = self.description {
                desc = desc.with_description(description);
            }

            if let Some(range) = self.range {
                desc.range = range;
            }

            desc = desc.with_read_only(self.read_only);

            self.server
                .declare_with_descriptor(self.name, ParameterValue::NotSet, Some(desc));
        }

        Ok(OptionalParameter {
            server: self.server,
            name: self.name,
            _phantom: core::marker::PhantomData,
        })
    }
}

/// A mandatory parameter that always has a value
///
/// # Example
///
/// ```ignore
/// let speed: MandatoryParameter<f64> = node
///     .declare_parameter("speed")
///     .default(5.0)
///     .mandatory()?;
///
/// let current_speed = speed.get();
/// speed.set(7.0)?;
/// ```
pub struct MandatoryParameter<'a, T: ParameterVariant> {
    server: *mut ParameterServer,
    name: &'a str,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, T: ParameterVariant> MandatoryParameter<'a, T> {
    /// Get the current parameter value
    ///
    /// # Panics
    ///
    /// Panics if the parameter no longer exists or has wrong type
    /// (should not happen if used correctly)
    pub fn get(&self) -> T {
        unsafe {
            let server = &*self.server;
            let value = server.get(self.name).expect("Mandatory parameter missing");
            T::from_parameter_value(value).expect("Type mismatch")
        }
    }

    /// Set the parameter value
    ///
    /// Returns an error if the parameter is read-only or the value
    /// is outside the allowed range.
    pub fn set(&self, value: T) -> Result<(), ParameterError> {
        unsafe {
            let server = &mut *self.server;
            let result = server.set(self.name, value.to_parameter_value());
            if result.is_success() {
                Ok(())
            } else {
                Err(result.into())
            }
        }
    }

    /// Get the parameter name
    pub fn name(&self) -> &str {
        self.name
    }
}

/// An optional parameter that may or may not have a value
///
/// # Example
///
/// ```ignore
/// let timeout: OptionalParameter<f64> = node
///     .declare_parameter("timeout")
///     .optional()?;
///
/// if let Some(value) = timeout.get() {
///     println!("Timeout: {}", value);
/// }
///
/// timeout.set(Some(10.0))?;
/// ```
pub struct OptionalParameter<'a, T: ParameterVariant> {
    server: *mut ParameterServer,
    name: &'a str,
    _phantom: core::marker::PhantomData<T>,
}

impl<'a, T: ParameterVariant> OptionalParameter<'a, T> {
    /// Get the current parameter value, if set
    pub fn get(&self) -> Option<T> {
        unsafe {
            let server = &*self.server;
            let value = server.get(self.name)?;
            T::from_parameter_value(value)
        }
    }

    /// Set the parameter value
    ///
    /// Pass `None` to unset the parameter.
    pub fn set(&self, value: Option<T>) -> Result<(), ParameterError> {
        unsafe {
            let server = &mut *self.server;

            if let Some(v) = value {
                // Setting a value - use normal set
                let result = server.set(self.name, v.to_parameter_value());
                if result.is_success() {
                    Ok(())
                } else {
                    Err(result.into())
                }
            } else {
                // Unsetting - use unset() to bypass type check
                let result = server.unset(self.name);
                if result.is_success() {
                    Ok(())
                } else {
                    Err(result.into())
                }
            }
        }
    }

    /// Get the parameter name
    pub fn name(&self) -> &str {
        self.name
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mandatory_parameter() {
        let mut server = ParameterServer::new();

        let speed: MandatoryParameter<f64> = ParameterBuilder::new(&mut server, "speed")
            .default(5.0)
            .mandatory()
            .unwrap();

        assert_eq!(speed.get(), 5.0);

        speed.set(7.0).unwrap();
        assert_eq!(speed.get(), 7.0);
    }

    #[test]
    fn test_optional_parameter() {
        let mut server = ParameterServer::new();

        let timeout: OptionalParameter<f64> = ParameterBuilder::new(&mut server, "timeout")
            .optional()
            .unwrap();

        assert_eq!(timeout.get(), None);

        timeout.set(Some(10.0)).unwrap();
        assert_eq!(timeout.get(), Some(10.0));

        timeout.set(None).unwrap();
        assert_eq!(timeout.get(), None);
    }

    #[test]
    fn test_range_validation() {
        let mut server = ParameterServer::new();

        let speed: MandatoryParameter<f64> = ParameterBuilder::new(&mut server, "speed")
            .default(5.0)
            .range(0.0, 10.0, 0.1)
            .mandatory()
            .unwrap();

        assert_eq!(speed.get(), 5.0);

        // Valid value
        assert!(speed.set(7.0).is_ok());

        // Out of range (should fail)
        assert_eq!(speed.set(15.0), Err(ParameterError::OutOfRange));
    }

    #[test]
    fn test_read_only() {
        let mut server = ParameterServer::new();

        let speed: MandatoryParameter<f64> = ParameterBuilder::new(&mut server, "speed")
            .default(5.0)
            .read_only()
            .mandatory()
            .unwrap();

        assert_eq!(speed.get(), 5.0);

        // Should fail - read-only
        assert_eq!(speed.set(7.0), Err(ParameterError::ReadOnly));
    }
}
