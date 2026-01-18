//! ROS 2 Service types
//!
//! Services provide synchronous request/response communication.
//! A service client sends a request and waits for a response from a service server.

use crate::types::RosService;
use crate::Error;

/// Service server handle
///
/// Receives requests and sends responses for a ROS 2 service.
pub struct ServiceServer<S: RosService> {
    /// Service name (e.g., "/add_two_ints")
    pub name: &'static str,
    /// Marker for service type
    _marker: core::marker::PhantomData<S>,
}

impl<S: RosService> ServiceServer<S> {
    /// Create a new service server handle
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            _marker: core::marker::PhantomData,
        }
    }

    /// Get the service name
    pub fn name(&self) -> &str {
        self.name
    }

    /// Get the service type name
    pub fn service_type(&self) -> &'static str {
        S::SERVICE_NAME
    }

    /// Get the service type hash
    pub fn service_hash(&self) -> &'static str {
        S::SERVICE_HASH
    }
}

/// Service client handle
///
/// Sends requests and receives responses for a ROS 2 service.
pub struct ServiceClient<S: RosService> {
    /// Service name (e.g., "/add_two_ints")
    pub name: &'static str,
    /// Marker for service type
    _marker: core::marker::PhantomData<S>,
}

impl<S: RosService> ServiceClient<S> {
    /// Create a new service client handle
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            _marker: core::marker::PhantomData,
        }
    }

    /// Get the service name
    pub fn name(&self) -> &str {
        self.name
    }

    /// Get the service type name
    pub fn service_type(&self) -> &'static str {
        S::SERVICE_NAME
    }

    /// Get the service type hash
    pub fn service_hash(&self) -> &'static str {
        S::SERVICE_HASH
    }
}

/// Service request context
///
/// Passed to service handlers with request data and means to send a reply.
pub struct ServiceRequest<'a, S: RosService> {
    /// The deserialized request message
    pub request: S::Request,
    /// Raw request data (CDR encoded)
    pub raw_data: &'a [u8],
}

/// Result type for service calls
pub type ServiceResult<T> = Result<T, Error>;

/// Callback type for service handlers
///
/// Takes a request and returns a response.
pub type ServiceCallback<S> = fn(&<S as RosService>::Request) -> <S as RosService>::Reply;

#[cfg(test)]
mod tests {
    use super::*;

    // Mock service for testing
    struct MockService;

    #[derive(Debug, Clone)]
    struct MockRequest {
        pub a: i32,
        pub b: i32,
    }

    #[derive(Debug, Clone)]
    struct MockReply {
        pub sum: i32,
    }

    // Implement minimal traits for testing
    impl nano_ros_serdes::Serialize for MockRequest {
        fn serialize(
            &self,
            _writer: &mut nano_ros_serdes::CdrWriter,
        ) -> Result<(), nano_ros_serdes::SerError> {
            Ok(())
        }
    }

    impl nano_ros_serdes::Deserialize for MockRequest {
        fn deserialize(
            _reader: &mut nano_ros_serdes::CdrReader,
        ) -> Result<Self, nano_ros_serdes::DeserError> {
            Ok(MockRequest { a: 0, b: 0 })
        }
    }

    impl crate::RosMessage for MockRequest {
        const TYPE_NAME: &'static str = "test_msgs::srv::dds_::AddTwoInts_Request_";
        const TYPE_HASH: &'static str =
            "0000000000000000000000000000000000000000000000000000000000000000";
    }

    impl nano_ros_serdes::Serialize for MockReply {
        fn serialize(
            &self,
            _writer: &mut nano_ros_serdes::CdrWriter,
        ) -> Result<(), nano_ros_serdes::SerError> {
            Ok(())
        }
    }

    impl nano_ros_serdes::Deserialize for MockReply {
        fn deserialize(
            _reader: &mut nano_ros_serdes::CdrReader,
        ) -> Result<Self, nano_ros_serdes::DeserError> {
            Ok(MockReply { sum: 0 })
        }
    }

    impl crate::RosMessage for MockReply {
        const TYPE_NAME: &'static str = "test_msgs::srv::dds_::AddTwoInts_Reply_";
        const TYPE_HASH: &'static str =
            "0000000000000000000000000000000000000000000000000000000000000000";
    }

    impl RosService for MockService {
        type Request = MockRequest;
        type Reply = MockReply;
        const SERVICE_NAME: &'static str = "test_msgs::srv::dds_::AddTwoInts_";
        const SERVICE_HASH: &'static str =
            "0000000000000000000000000000000000000000000000000000000000000000";
    }

    #[test]
    fn test_service_server_creation() {
        let server = ServiceServer::<MockService>::new("/add_two_ints");
        assert_eq!(server.name(), "/add_two_ints");
        assert_eq!(server.service_type(), "test_msgs::srv::dds_::AddTwoInts_");
    }

    #[test]
    fn test_service_client_creation() {
        let client = ServiceClient::<MockService>::new("/add_two_ints");
        assert_eq!(client.name(), "/add_two_ints");
        assert_eq!(client.service_type(), "test_msgs::srv::dds_::AddTwoInts_");
    }
}
