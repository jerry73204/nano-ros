//! Parameter server implementation
//!
//! Provides a static storage parameter server for embedded systems.
//! Parameters are stored in a fixed-size array with compile-time capacity.

use crate::types::{
    Parameter, ParameterDescriptor, ParameterType, ParameterValue, SetParameterResult,
    MAX_PARAM_NAME_LEN,
};
use heapless::String;

/// Maximum number of parameters the server can store
pub const MAX_PARAMETERS: usize = 32;

/// Entry in the parameter storage
#[derive(Debug, Clone)]
struct ParameterEntry {
    /// The parameter (name + value)
    param: Parameter,
    /// Optional descriptor with constraints
    descriptor: Option<ParameterDescriptor>,
}

/// Parameter server with static storage
///
/// Stores parameters in a fixed-size array. The capacity is determined
/// at compile time via the `MAX_PARAMETERS` constant.
///
/// # Example
///
/// ```
/// use nano_ros_params::{ParameterServer, ParameterValue};
///
/// let mut server = ParameterServer::new();
///
/// // Declare a parameter with initial value
/// server.declare("max_speed", ParameterValue::Double(1.0));
///
/// // Get parameter value
/// if let Some(value) = server.get("max_speed") {
///     println!("max_speed = {:?}", value.as_double());
/// }
///
/// // Set parameter value
/// server.set("max_speed", ParameterValue::Double(2.0));
/// ```
pub struct ParameterServer {
    /// Parameter storage
    entries: [Option<ParameterEntry>; MAX_PARAMETERS],
    /// Number of parameters currently stored
    count: usize,
}

impl Default for ParameterServer {
    fn default() -> Self {
        Self::new()
    }
}

impl ParameterServer {
    /// Create a new empty parameter server
    #[allow(clippy::large_stack_arrays)] // Intentional: static allocation for embedded use
    pub const fn new() -> Self {
        // Initialize with None values
        Self {
            entries: [const { None }; MAX_PARAMETERS],
            count: 0,
        }
    }

    /// Get the number of parameters stored
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if the server has no parameters
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Check if the server is at capacity
    pub fn is_full(&self) -> bool {
        self.count >= MAX_PARAMETERS
    }

    /// Find the index of a parameter by name
    fn find_index(&self, name: &str) -> Option<usize> {
        self.entries.iter().position(|entry| {
            entry
                .as_ref()
                .map(|e| e.param.name.as_str() == name)
                .unwrap_or(false)
        })
    }

    /// Find an empty slot
    fn find_empty_slot(&self) -> Option<usize> {
        self.entries.iter().position(|entry| entry.is_none())
    }

    /// Declare a new parameter with a value
    ///
    /// If the parameter already exists, this does nothing and returns false.
    /// Returns true if the parameter was declared successfully.
    pub fn declare(&mut self, name: &str, value: ParameterValue) -> bool {
        self.declare_with_descriptor(name, value, None)
    }

    /// Declare a new parameter with value and descriptor
    ///
    /// The descriptor provides metadata like description, constraints, and read-only flag.
    pub fn declare_with_descriptor(
        &mut self,
        name: &str,
        value: ParameterValue,
        descriptor: Option<ParameterDescriptor>,
    ) -> bool {
        // Check if already exists
        if self.find_index(name).is_some() {
            return false;
        }

        // Find an empty slot
        let slot = match self.find_empty_slot() {
            Some(idx) => idx,
            None => return false,
        };

        // Create the parameter
        let param = match Parameter::new(name, value) {
            Some(p) => p,
            None => return false,
        };

        self.entries[slot] = Some(ParameterEntry { param, descriptor });
        self.count += 1;
        true
    }

    /// Get a parameter value by name
    pub fn get(&self, name: &str) -> Option<&ParameterValue> {
        self.find_index(name)
            .and_then(|idx| self.entries[idx].as_ref())
            .map(|entry| &entry.param.value)
    }

    /// Get a parameter by name
    pub fn get_parameter(&self, name: &str) -> Option<&Parameter> {
        self.find_index(name)
            .and_then(|idx| self.entries[idx].as_ref())
            .map(|entry| &entry.param)
    }

    /// Get a parameter descriptor by name
    pub fn get_descriptor(&self, name: &str) -> Option<&ParameterDescriptor> {
        self.find_index(name)
            .and_then(|idx| self.entries[idx].as_ref())
            .and_then(|entry| entry.descriptor.as_ref())
    }

    /// Set a parameter value
    ///
    /// Returns the result of the set operation.
    pub fn set(&mut self, name: &str, value: ParameterValue) -> SetParameterResult {
        let idx = match self.find_index(name) {
            Some(idx) => idx,
            None => return SetParameterResult::NotFound,
        };

        let entry = match self.entries[idx].as_mut() {
            Some(e) => e,
            None => return SetParameterResult::NotFound,
        };

        // Check if read-only
        if let Some(ref desc) = entry.descriptor {
            if desc.read_only {
                return SetParameterResult::ReadOnly;
            }

            // Check type compatibility
            if !desc.dynamic_typing && desc.param_type != value.param_type() {
                return SetParameterResult::TypeMismatch;
            }

            // Check range constraints
            if !desc.validate_range(&value) {
                return SetParameterResult::OutOfRange;
            }
        }

        entry.param.value = value;
        SetParameterResult::Success
    }

    /// Unset an optional parameter (set to NotSet)
    ///
    /// This bypasses the type check to allow optional parameters to be unset.
    /// Returns an error if the parameter is read-only.
    pub fn unset(&mut self, name: &str) -> SetParameterResult {
        let idx = match self.find_index(name) {
            Some(idx) => idx,
            None => return SetParameterResult::NotFound,
        };

        let entry = match self.entries[idx].as_mut() {
            Some(e) => e,
            None => return SetParameterResult::NotFound,
        };

        // Check if read-only
        if let Some(ref desc) = entry.descriptor {
            if desc.read_only {
                return SetParameterResult::ReadOnly;
            }
        }

        entry.param.value = ParameterValue::NotSet;
        SetParameterResult::Success
    }

    /// Set or declare a parameter
    ///
    /// If the parameter exists, sets its value. Otherwise, declares it.
    pub fn set_or_declare(&mut self, name: &str, value: ParameterValue) -> SetParameterResult {
        if self.find_index(name).is_some() {
            self.set(name, value)
        } else if self.declare(name, value) {
            SetParameterResult::Success
        } else {
            SetParameterResult::StorageFull
        }
    }

    /// Check if a parameter exists
    pub fn has(&self, name: &str) -> bool {
        self.find_index(name).is_some()
    }

    /// Remove a parameter
    ///
    /// Returns true if the parameter was removed.
    pub fn remove(&mut self, name: &str) -> bool {
        if let Some(idx) = self.find_index(name) {
            self.entries[idx] = None;
            self.count -= 1;
            true
        } else {
            false
        }
    }

    /// Get the type of a parameter
    pub fn get_type(&self, name: &str) -> Option<ParameterType> {
        self.get(name).map(|v| v.param_type())
    }

    /// Iterate over all parameters
    pub fn iter(&self) -> impl Iterator<Item = &Parameter> {
        self.entries
            .iter()
            .filter_map(|entry| entry.as_ref().map(|e| &e.param))
    }

    /// List all parameter names
    pub fn list_names(&self) -> impl Iterator<Item = &str> {
        self.iter().map(|p| p.name.as_str())
    }

    /// List parameter names with a given prefix
    pub fn list_with_prefix<'a>(&'a self, prefix: &'a str) -> impl Iterator<Item = &'a str> {
        self.list_names()
            .filter(move |name| name.starts_with(prefix))
    }

    /// Get a bool parameter value
    pub fn get_bool(&self, name: &str) -> Option<bool> {
        self.get(name).and_then(|v| v.as_bool())
    }

    /// Get an integer parameter value
    pub fn get_integer(&self, name: &str) -> Option<i64> {
        self.get(name).and_then(|v| v.as_integer())
    }

    /// Get a double parameter value
    pub fn get_double(&self, name: &str) -> Option<f64> {
        self.get(name).and_then(|v| v.as_double())
    }

    /// Get a string parameter value
    pub fn get_string(&self, name: &str) -> Option<&str> {
        self.get(name).and_then(|v| v.as_string())
    }

    /// Set a bool parameter value
    pub fn set_bool(&mut self, name: &str, value: bool) -> SetParameterResult {
        self.set(name, ParameterValue::Bool(value))
    }

    /// Set an integer parameter value
    pub fn set_integer(&mut self, name: &str, value: i64) -> SetParameterResult {
        self.set(name, ParameterValue::Integer(value))
    }

    /// Set a double parameter value
    pub fn set_double(&mut self, name: &str, value: f64) -> SetParameterResult {
        self.set(name, ParameterValue::Double(value))
    }

    /// Set a string parameter value
    pub fn set_string(&mut self, name: &str, value: &str) -> SetParameterResult {
        match ParameterValue::from_string(value) {
            Some(v) => self.set(name, v),
            None => SetParameterResult::StorageFull, // String too long
        }
    }

    /// Declare a parameter with a descriptor and optional initial value.
    ///
    /// This is a more comprehensive declaration method used by the typed parameter API.
    ///
    /// # Arguments
    /// - `descriptor`: The metadata for the parameter.
    /// - `initial_value`: An optional initial value for the parameter. If `None`,
    ///   the parameter will be initialized to `ParameterValue::NotSet`.
    pub fn declare_parameter(
        &mut self,
        descriptor: ParameterDescriptor,
        initial_value: Option<&ParameterValue>,
    ) -> Result<(), SetParameterResult> {
        let name = descriptor.name.as_str();

        // Check if already exists
        if self.find_index(name).is_some() {
            return Err(SetParameterResult::TypeMismatch); // Indicates already declared
        }

        // Find an empty slot
        let slot = match self.find_empty_slot() {
            Some(idx) => idx,
            None => return Err(SetParameterResult::StorageFull),
        };

        let param_value = initial_value.cloned().unwrap_or_default();

        let param = Parameter::new(name, param_value).ok_or(SetParameterResult::StorageFull)?; // name too long

        self.entries[slot] = Some(ParameterEntry {
            param,
            descriptor: Some(descriptor),
        });
        self.count += 1;
        Ok(())
    }

    /// Get the current value of a parameter.
    ///
    /// Returns `Some(ParameterValue)` if the parameter exists, `None` otherwise.
    /// The `ParameterValue` is cloned to avoid lifetime issues with `&mut self`.
    pub fn get_parameter_value(&self, name: &str) -> Option<ParameterValue> {
        self.get(name).cloned()
    }

    /// Set the value of a parameter.
    ///
    /// Returns `SetParameterResult::Success` on success, or an error if the parameter
    /// is read-only, type mismatches, or is out of range.
    pub fn set_parameter_value(&mut self, name: &str, value: ParameterValue) -> SetParameterResult {
        self.set(name, value)
    }
}

/// Builder for declaring parameters with a fluent API
pub struct LegacyParameterBuilder<'a> {
    server: &'a mut ParameterServer,
    name: String<MAX_PARAM_NAME_LEN>,
    value: ParameterValue,
    descriptor: Option<ParameterDescriptor>,
}

impl<'a> LegacyParameterBuilder<'a> {
    /// Create a new parameter builder
    pub fn new(server: &'a mut ParameterServer, name: &str, value: ParameterValue) -> Option<Self> {
        let mut n = String::new();
        n.push_str(name).ok()?;
        Some(Self {
            server,
            name: n,
            value,
            descriptor: None,
        })
    }

    /// Set the parameter description
    pub fn description(mut self, desc: &str) -> Self {
        let param_type = self.value.param_type();
        if self.descriptor.is_none() {
            self.descriptor = ParameterDescriptor::new(self.name.as_str(), param_type);
        }
        if let Some(ref mut d) = self.descriptor {
            d.description.clear();
            let _ = d.description.push_str(desc);
        }
        self
    }

    /// Set the parameter as read-only
    pub fn read_only(mut self) -> Self {
        let param_type = self.value.param_type();
        if self.descriptor.is_none() {
            self.descriptor = ParameterDescriptor::new(self.name.as_str(), param_type);
        }
        if let Some(ref mut d) = self.descriptor {
            d.read_only = true;
        }
        self
    }

    /// Set integer range constraints
    pub fn integer_range(mut self, min: i64, max: i64, step: i64) -> Self {
        let param_type = self.value.param_type();
        if self.descriptor.is_none() {
            self.descriptor = ParameterDescriptor::new(self.name.as_str(), param_type);
        }
        if let Some(ref mut d) = self.descriptor {
            d.range = crate::types::ParameterRange::Integer(crate::types::IntegerRange::new(
                min, max, step,
            ));
        }
        self
    }

    /// Set float range constraints
    pub fn float_range(mut self, min: f64, max: f64, step: f64) -> Self {
        let param_type = self.value.param_type();
        if self.descriptor.is_none() {
            self.descriptor = ParameterDescriptor::new(self.name.as_str(), param_type);
        }
        if let Some(ref mut d) = self.descriptor {
            d.range = crate::types::ParameterRange::FloatingPoint(
                crate::types::FloatingPointRange::new(min, max, step),
            );
        }
        self
    }

    /// Declare the parameter
    pub fn declare(self) -> bool {
        self.server
            .declare_with_descriptor(self.name.as_str(), self.value, self.descriptor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_server() {
        let server = ParameterServer::new();
        assert_eq!(server.len(), 0);
        assert!(server.is_empty());
    }

    #[test]
    fn test_declare_and_get() {
        let mut server = ParameterServer::new();

        assert!(server.declare("my_bool", ParameterValue::Bool(true)));
        assert!(server.declare("my_int", ParameterValue::Integer(42)));
        assert!(server.declare("my_double", ParameterValue::Double(3.14)));

        assert_eq!(server.len(), 3);
        assert!(!server.is_empty());

        assert_eq!(server.get_bool("my_bool"), Some(true));
        assert_eq!(server.get_integer("my_int"), Some(42));
        assert_eq!(server.get_double("my_double"), Some(3.14));
    }

    #[test]
    fn test_set_parameter() {
        let mut server = ParameterServer::new();
        server.declare("count", ParameterValue::Integer(0));

        assert_eq!(server.set_integer("count", 10), SetParameterResult::Success);
        assert_eq!(server.get_integer("count"), Some(10));
    }

    #[test]
    fn test_set_nonexistent() {
        let mut server = ParameterServer::new();
        assert_eq!(
            server.set("nonexistent", ParameterValue::Integer(1)),
            SetParameterResult::NotFound
        );
    }

    #[test]
    fn test_read_only_parameter() {
        let mut server = ParameterServer::new();

        let desc = ParameterDescriptor::new("version", ParameterType::String)
            .unwrap()
            .with_read_only(true);

        server.declare_with_descriptor(
            "version",
            ParameterValue::from_string("1.0.0").unwrap(),
            Some(desc),
        );

        assert_eq!(
            server.set_string("version", "2.0.0"),
            SetParameterResult::ReadOnly
        );
        assert_eq!(server.get_string("version"), Some("1.0.0"));
    }

    #[test]
    fn test_range_constraints() {
        let mut server = ParameterServer::new();

        let desc = ParameterDescriptor::new("speed", ParameterType::Double)
            .unwrap()
            .with_float_range(0.0, 10.0, 0.0);

        server.declare_with_descriptor("speed", ParameterValue::Double(5.0), Some(desc));

        // Valid value
        assert_eq!(server.set_double("speed", 8.0), SetParameterResult::Success);

        // Out of range
        assert_eq!(
            server.set_double("speed", 15.0),
            SetParameterResult::OutOfRange
        );
        assert_eq!(server.get_double("speed"), Some(8.0)); // Unchanged
    }

    #[test]
    fn test_remove_parameter() {
        let mut server = ParameterServer::new();
        server.declare("temp", ParameterValue::Double(25.0));

        assert!(server.has("temp"));
        assert!(server.remove("temp"));
        assert!(!server.has("temp"));
        assert!(!server.remove("temp")); // Already removed
    }

    #[test]
    fn test_list_parameters() {
        let mut server = ParameterServer::new();
        server.declare("robot.speed", ParameterValue::Double(1.0));
        server.declare("robot.name", ParameterValue::from_string("bot1").unwrap());
        server.declare("sensor.range", ParameterValue::Double(10.0));

        let robot_params: heapless::Vec<&str, 8> = server.list_with_prefix("robot.").collect();
        assert_eq!(robot_params.len(), 2);
    }

    #[test]
    fn test_set_or_declare() {
        let mut server = ParameterServer::new();

        // First call declares
        assert_eq!(
            server.set_or_declare("new_param", ParameterValue::Integer(1)),
            SetParameterResult::Success
        );
        assert_eq!(server.get_integer("new_param"), Some(1));

        // Second call sets
        assert_eq!(
            server.set_or_declare("new_param", ParameterValue::Integer(2)),
            SetParameterResult::Success
        );
        assert_eq!(server.get_integer("new_param"), Some(2));
    }

    #[test]
    fn test_duplicate_declare() {
        let mut server = ParameterServer::new();
        assert!(server.declare("param", ParameterValue::Integer(1)));
        assert!(!server.declare("param", ParameterValue::Integer(2))); // Already exists
        assert_eq!(server.get_integer("param"), Some(1)); // Unchanged
    }
}
