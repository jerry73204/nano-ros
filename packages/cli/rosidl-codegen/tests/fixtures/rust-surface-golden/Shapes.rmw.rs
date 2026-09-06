// emit:ok
// RMW (ROS Middleware) layer - C-compatible FFI types
// Package: fingerprint-corpus
// Message: Shapes

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// FFI bindings to C libraries
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__msg__Shapes() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__msg__Shapes__init(msg: *mut Shapes) -> bool;
    fn fingerprint-corpus__msg__Shapes__Sequence__init(seq: *mut crate::rosidl_runtime_rs::Sequence<Shapes>, size: usize) -> bool;
    fn fingerprint-corpus__msg__Shapes__Sequence__fini(seq: *mut crate::rosidl_runtime_rs::Sequence<Shapes>);
    fn fingerprint-corpus__msg__Shapes__Sequence__copy(in_seq: &crate::rosidl_runtime_rs::Sequence<Shapes>, out_seq: *mut crate::rosidl_runtime_rs::Sequence<Shapes>) -> bool;
}

// RMW types are C-compatible FFI types
// Serde support is available via the "serde" feature flag
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Shapes {
    
    pub flag: bool,
    
    pub i8_v: i8,
    
    pub u8_v: u8,
    
    pub i16_v: i16,
    
    pub u16_v: u16,
    
    pub i32_v: i32,
    
    pub u32_v: u32,
    
    pub i64_v: i64,
    
    pub u64_v: u64,
    
    pub f32_v: f32,
    
    pub f64_v: f64,
    
    pub text: crate::rosidl_runtime_rs::String,
    
    pub seq_prim: crate::rosidl_runtime_rs::Sequence<i64>,
    
    pub seq_string: crate::rosidl_runtime_rs::Sequence<crate::rosidl_runtime_rs::String>,
    
    pub arr_fixed: [f64; 3],
    
    pub seq_bounded: crate::rosidl_runtime_rs::BoundedSequence<i32, 4>,
    
    pub str_bounded: crate::rosidl_runtime_rs::BoundedString<8>,
    
}

impl Shapes {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Shapes {
    fn default() -> Self {
        unsafe {
            // SAFETY: Zeroing a message structure is valid for all ROS message types
            let mut msg = std::mem::zeroed();
            // SAFETY: The init function is safe to call on a zeroed message
            if !fingerprint-corpus__msg__Shapes__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__msg__Shapes__init() failed");
            }
            msg
        }
    }
}

// Trait implementations for ROS runtime

impl crate::rosidl_runtime_rs::SequenceAlloc for Shapes {
    fn sequence_init(seq: &mut crate::rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        // SAFETY: The pointer is guaranteed to be valid since it comes from a mutable reference
        unsafe { fingerprint-corpus__msg__Shapes__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut crate::rosidl_runtime_rs::Sequence<Self>) {
        // SAFETY: The pointer is guaranteed to be valid since it comes from a mutable reference
        unsafe { fingerprint-corpus__msg__Shapes__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &crate::rosidl_runtime_rs::Sequence<Self>, out_seq: &mut crate::rosidl_runtime_rs::Sequence<Self>) -> bool {
        // SAFETY: Both pointers are guaranteed to be valid since they come from references
        unsafe { fingerprint-corpus__msg__Shapes__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl crate::rosidl_runtime_rs::Message for Shapes {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        // Identity conversion: RMW message is already in RMW format
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        // Identity conversion: RMW message is already in RMW format
        msg
    }
}

impl crate::rosidl_runtime_rs::RmwMessage for Shapes where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/msg/Shapes";

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__msg__Shapes() }
    }
}