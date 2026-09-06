// emit:ok
// Idiomatic Rust layer - user-friendly types
// Package: fingerprint-corpus
// Service: Probe

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// Request message
pub mod request {
    #[cfg(feature = "serde")]
    use super::{Deserialize, Serialize};

    #[allow(non_camel_case_types)]
    #[derive(Debug, Clone, PartialEq)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct Probe_Request {
        
        pub items: std::vec::Vec<i64>,
        
        pub note: std::string::String,
        
    }

    impl Probe_Request {
        
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Default for Probe_Request {
        fn default() -> Self {
            // Leverage FFI message's C init function to get correct default values
            <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ProbeRequest::default())
        }
    }

    // Conversion from FFI layer
    impl From<crate::srv::rmw::ProbeRequest> for Probe_Request {
        #[allow(unused_variables)]
        fn from(rmw: crate::srv::rmw::ProbeRequest) -> Self {
            Self {
                
                
                // Sequence<primitive> → Vec<primitive> (use rosidl_runtime_rs 0.5 API)
                items: rmw.items.as_slice().to_vec(),
                
                
                
                // rosidl_runtime_rs::String → String (use rosidl_runtime_rs 0.5 API)
                note: rmw.note.to_string(),
                
                
            }
        }
    }

    // Conversion to FFI layer
    impl From<Probe_Request> for crate::srv::rmw::ProbeRequest {
        #[allow(unused_variables)]
        fn from(idiomatic: Probe_Request) -> Self {
            Self {
                
                
                // Vec<primitive> → Sequence<primitive> (use rosidl_runtime_rs 0.5 API)
                items: crate::rosidl_runtime_rs::Sequence::from(idiomatic.items.clone()),
                
                
                
                // String → rosidl_runtime_rs::String (use rosidl_runtime_rs 0.5 API)
                note: crate::rosidl_runtime_rs::String::from(idiomatic.note.as_str()),
                
                
            }
        }
    }

    // Message trait implementation for rosidl_runtime_rs
    impl crate::rosidl_runtime_rs::Message for Probe_Request {
        type RmwMsg = crate::srv::rmw::ProbeRequest;

        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            // Convert from idiomatic to RMW format
            std::borrow::Cow::Owned(msg_cow.into_owned().into())
        }

        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            // Convert from RMW to idiomatic format
            msg.into()
        }
    }
}

// Response message
pub mod response {
    #[cfg(feature = "serde")]
    use super::{Deserialize, Serialize};

    #[allow(non_camel_case_types)]
    #[derive(Debug, Clone, PartialEq)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct Probe_Response {
        
        pub sum: i64,
        
        pub lines: std::vec::Vec<std::string::String>,
        
    }

    impl Probe_Response {
        
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Default for Probe_Response {
        fn default() -> Self {
            // Leverage FFI message's C init function to get correct default values
            <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::ProbeResponse::default())
        }
    }

    // Conversion from FFI layer
    impl From<crate::srv::rmw::ProbeResponse> for Probe_Response {
        #[allow(unused_variables)]
        fn from(rmw: crate::srv::rmw::ProbeResponse) -> Self {
            Self {
                
                
                // Primitives are Copy, just copy the value
                sum: rmw.sum,
                
                
                
                // Sequence<rosidl_runtime_rs::String> → Vec<String>
                lines: rmw.lines.as_slice().iter().map(|item| item.to_string()).collect(),
                
                
            }
        }
    }

    // Conversion to FFI layer
    impl From<Probe_Response> for crate::srv::rmw::ProbeResponse {
        #[allow(unused_variables)]
        fn from(idiomatic: Probe_Response) -> Self {
            Self {
                
                
                // Primitives are Copy, just copy the value
                sum: idiomatic.sum,
                
                
                
                // Vec<String> → Sequence<rosidl_runtime_rs::String>
                lines: crate::rosidl_runtime_rs::Sequence::from(
                    idiomatic.lines.iter().map(|item| crate::rosidl_runtime_rs::String::from(item.as_str())).collect::<Vec<_>>()
                ),
                
                
            }
        }
    }

    // Message trait implementation for rosidl_runtime_rs
    impl crate::rosidl_runtime_rs::Message for Probe_Response {
        type RmwMsg = crate::srv::rmw::ProbeResponse;

        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            // Convert from idiomatic to RMW format
            std::borrow::Cow::Owned(msg_cow.into_owned().into())
        }

        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            // Convert from RMW to idiomatic format
            msg.into()
        }
    }
}

// Re-export for convenience
pub use request::Probe_Request;
pub use response::Probe_Response;

// Service type support
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__srv__Probe() -> *const std::ffi::c_void;
}

// Service struct (zero-sized type)
pub struct Probe;

impl crate::rosidl_runtime_rs::Service for Probe {
    type Request = Probe_Request;
    type Response = Probe_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__srv__Probe() }
    }
}