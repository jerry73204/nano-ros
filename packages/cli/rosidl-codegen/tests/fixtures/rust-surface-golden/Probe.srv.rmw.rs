// emit:ok
// RMW (ROS Middleware) layer - C-compatible FFI types
// Package: fingerprint-corpus
// Service: Probe

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// Request constants


// Response constants


// FFI bindings to C libraries for Request
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__srv__Probe_Request() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__srv__Probe_Request__init(msg: *mut ProbeRequest) -> bool;
    fn fingerprint-corpus__srv__Probe_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ProbeRequest>, size: usize) -> bool;
    fn fingerprint-corpus__srv__Probe_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ProbeRequest>);
    fn fingerprint-corpus__srv__Probe_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ProbeRequest>, out_seq: *mut rosidl_runtime_rs::Sequence<ProbeRequest>) -> bool;
}

// Request message type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeRequest {
    
    pub items: crate::rosidl_runtime_rs::Sequence<i64>,
    
    pub note: crate::rosidl_runtime_rs::String,
    
}

impl ProbeRequest {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for ProbeRequest {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__srv__Probe_Request__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__srv__Probe_Request__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for ProbeRequest {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__srv__Probe_Request__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__srv__Probe_Request__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__srv__Probe_Request__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for ProbeRequest {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for ProbeRequest where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/srv/Probe_Request";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__srv__Probe_Request() }
    }
}

// FFI bindings to C libraries for Response
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__srv__Probe_Response() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__srv__Probe_Response__init(msg: *mut ProbeResponse) -> bool;
    fn fingerprint-corpus__srv__Probe_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ProbeResponse>, size: usize) -> bool;
    fn fingerprint-corpus__srv__Probe_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ProbeResponse>);
    fn fingerprint-corpus__srv__Probe_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ProbeResponse>, out_seq: *mut rosidl_runtime_rs::Sequence<ProbeResponse>) -> bool;
}

// Response message type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeResponse {
    
    pub sum: i64,
    
    pub lines: crate::rosidl_runtime_rs::Sequence<crate::rosidl_runtime_rs::String>,
    
}

impl ProbeResponse {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for ProbeResponse {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__srv__Probe_Response__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__srv__Probe_Response__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for ProbeResponse {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__srv__Probe_Response__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__srv__Probe_Response__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__srv__Probe_Response__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for ProbeResponse {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for ProbeResponse where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/srv/Probe_Response";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__srv__Probe_Response() }
    }
}

// Service type support
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__srv__Probe() -> *const std::ffi::c_void;
}

// Service struct (zero-sized type)
pub struct Probe;

impl rosidl_runtime_rs::Service for Probe {
    type Request = ProbeRequest;
    type Response = ProbeResponse;

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__srv__Probe() }
    }
}