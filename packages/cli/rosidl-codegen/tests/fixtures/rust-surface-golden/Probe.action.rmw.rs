// emit:ok
// RMW (ROS Middleware) layer - C-compatible FFI types
// Package: fingerprint-corpus
// Action: Probe

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// Goal constants
pub mod goal_constants {
    
}

// Result constants
pub mod result_constants {
    
}

// Feedback constants
pub mod feedback_constants {
    
}

// FFI bindings to C libraries for Goal
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_Goal() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_Goal__init(msg: *mut ProbeGoal) -> bool;
    fn fingerprint-corpus__action__Probe_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ProbeGoal>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ProbeGoal>);
    fn fingerprint-corpus__action__Probe_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ProbeGoal>, out_seq: *mut rosidl_runtime_rs::Sequence<ProbeGoal>) -> bool;
}

// Goal message type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeGoal {
    
    pub waypoints: crate::rosidl_runtime_rs::Sequence<i64>,
    
    pub name: crate::rosidl_runtime_rs::String,
    
}

impl ProbeGoal {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for ProbeGoal {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_Goal__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_Goal__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for ProbeGoal {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_Goal__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_Goal__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for ProbeGoal {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for ProbeGoal where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_Goal";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_Goal() }
    }
}

// FFI bindings to C libraries for Result
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_Result() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_Result__init(msg: *mut ProbeResult) -> bool;
    fn fingerprint-corpus__action__Probe_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ProbeResult>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ProbeResult>);
    fn fingerprint-corpus__action__Probe_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ProbeResult>, out_seq: *mut rosidl_runtime_rs::Sequence<ProbeResult>) -> bool;
}

// Result message type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeResult {
    
    pub total: i64,
    
    pub report: crate::rosidl_runtime_rs::String,
    
}

impl ProbeResult {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for ProbeResult {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_Result__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_Result__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for ProbeResult {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_Result__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_Result__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_Result__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for ProbeResult {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for ProbeResult where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_Result";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_Result() }
    }
}

// FFI bindings to C libraries for Feedback
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_Feedback__init(msg: *mut ProbeFeedback) -> bool;
    fn fingerprint-corpus__action__Probe_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ProbeFeedback>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ProbeFeedback>);
    fn fingerprint-corpus__action__Probe_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ProbeFeedback>, out_seq: *mut rosidl_runtime_rs::Sequence<ProbeFeedback>) -> bool;
}

// Feedback message type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeFeedback {
    
    pub done: i64,
    
    pub stage: crate::rosidl_runtime_rs::String,
    
}

impl ProbeFeedback {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for ProbeFeedback {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_Feedback__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_Feedback__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for ProbeFeedback {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_Feedback__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_Feedback__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for ProbeFeedback {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for ProbeFeedback where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_Feedback";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_Feedback() }
    }
}

// FFI bindings to C libraries for FeedbackMessage
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_FeedbackMessage__init(msg: *mut ProbeFeedbackMessage) -> bool;
    fn fingerprint-corpus__action__Probe_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ProbeFeedbackMessage>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ProbeFeedbackMessage>);
    fn fingerprint-corpus__action__Probe_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ProbeFeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<ProbeFeedbackMessage>) -> bool;
}

// FeedbackMessage type (wraps feedback with goal UUID)
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeFeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub feedback: ProbeFeedback,
}

impl ProbeFeedbackMessage {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for ProbeFeedbackMessage {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_FeedbackMessage__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_FeedbackMessage__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for ProbeFeedbackMessage {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_FeedbackMessage__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_FeedbackMessage__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for ProbeFeedbackMessage {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for ProbeFeedbackMessage where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_FeedbackMessage";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_FeedbackMessage() }
    }
}

// FFI bindings to C libraries for SendGoal_Request
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_SendGoal_Request__init(msg: *mut Probe_SendGoal_Request) -> bool;
    fn fingerprint-corpus__action__Probe_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Probe_SendGoal_Request>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Probe_SendGoal_Request>);
    fn fingerprint-corpus__action__Probe_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Probe_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Probe_SendGoal_Request>) -> bool;
}

// SendGoal_Request type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Probe_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
    pub goal: ProbeGoal,
}

impl Probe_SendGoal_Request {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Probe_SendGoal_Request {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_SendGoal_Request__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_SendGoal_Request__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for Probe_SendGoal_Request {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_SendGoal_Request__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_SendGoal_Request__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for Probe_SendGoal_Request {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for Probe_SendGoal_Request where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_SendGoal_Request";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_SendGoal_Request() }
    }
}

// FFI bindings to C libraries for SendGoal_Response
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_SendGoal_Response__init(msg: *mut Probe_SendGoal_Response) -> bool;
    fn fingerprint-corpus__action__Probe_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Probe_SendGoal_Response>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Probe_SendGoal_Response>);
    fn fingerprint-corpus__action__Probe_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Probe_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Probe_SendGoal_Response>) -> bool;
}

// SendGoal_Response type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Probe_SendGoal_Response {
    pub accepted: bool,
    pub stamp: builtin_interfaces::msg::rmw::Time,
}

impl Probe_SendGoal_Response {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Probe_SendGoal_Response {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_SendGoal_Response__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_SendGoal_Response__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for Probe_SendGoal_Response {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_SendGoal_Response__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_SendGoal_Response__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for Probe_SendGoal_Response {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for Probe_SendGoal_Response where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_SendGoal_Response";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_SendGoal_Response() }
    }
}

// SendGoal service type support
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__action__Probe_SendGoal() -> *const std::ffi::c_void;
}

// SendGoal service struct
#[allow(non_camel_case_types)]
pub struct Probe_SendGoal;

impl rosidl_runtime_rs::Service for Probe_SendGoal {
    type Request = Probe_SendGoal_Request;
    type Response = Probe_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__action__Probe_SendGoal() }
    }
}

// FFI bindings to C libraries for GetResult_Request
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_GetResult_Request__init(msg: *mut Probe_GetResult_Request) -> bool;
    fn fingerprint-corpus__action__Probe_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Probe_GetResult_Request>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Probe_GetResult_Request>);
    fn fingerprint-corpus__action__Probe_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Probe_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Probe_GetResult_Request>) -> bool;
}

// GetResult_Request type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Probe_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,
}

impl Probe_GetResult_Request {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Probe_GetResult_Request {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_GetResult_Request__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_GetResult_Request__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for Probe_GetResult_Request {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_GetResult_Request__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_GetResult_Request__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for Probe_GetResult_Request {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for Probe_GetResult_Request where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_GetResult_Request";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_GetResult_Request() }
    }
}

// FFI bindings to C libraries for GetResult_Response
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "fingerprint-corpus__rosidl_generator_c")]
#[allow(improper_ctypes)]
extern "C" {
    fn fingerprint-corpus__action__Probe_GetResult_Response__init(msg: *mut Probe_GetResult_Response) -> bool;
    fn fingerprint-corpus__action__Probe_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Probe_GetResult_Response>, size: usize) -> bool;
    fn fingerprint-corpus__action__Probe_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Probe_GetResult_Response>);
    fn fingerprint-corpus__action__Probe_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Probe_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Probe_GetResult_Response>) -> bool;
}

// GetResult_Response type
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Probe_GetResult_Response {
    pub status: i8,
    pub result: ProbeResult,
}

impl Probe_GetResult_Response {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Probe_GetResult_Response {
    fn default() -> Self {
        unsafe {
            let mut msg = std::mem::zeroed();
            if !fingerprint-corpus__action__Probe_GetResult_Response__init(&mut msg as *mut _) {
                panic!("Call to fingerprint-corpus__action__Probe_GetResult_Response__init() failed");
            }
            msg
        }
    }
}

impl rosidl_runtime_rs::SequenceAlloc for Probe_GetResult_Response {
    fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
        unsafe { fingerprint-corpus__action__Probe_GetResult_Response__Sequence__init(seq as *mut _, size) }
    }

    fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
        unsafe { fingerprint-corpus__action__Probe_GetResult_Response__Sequence__fini(seq as *mut _) }
    }

    fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
        unsafe { fingerprint-corpus__action__Probe_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
    }
}

impl rosidl_runtime_rs::Message for Probe_GetResult_Response {
    type RmwMsg = Self;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        msg_cow
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        msg
    }
}

impl rosidl_runtime_rs::RmwMessage for Probe_GetResult_Response where Self: Sized {
    const TYPE_NAME: &'static str = "fingerprint-corpus/action/Probe_GetResult_Response";

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_message_type_support_handle__fingerprint-corpus__action__Probe_GetResult_Response() }
    }
}

// GetResult service type support
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__action__Probe_GetResult() -> *const std::ffi::c_void;
}

// GetResult service struct
#[allow(non_camel_case_types)]
pub struct Probe_GetResult;

impl rosidl_runtime_rs::Service for Probe_GetResult {
    type Request = Probe_GetResult_Request;
    type Response = Probe_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__fingerprint-corpus__action__Probe_GetResult() }
    }
}

// Action type support
#[link(name = "fingerprint-corpus__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__fingerprint-corpus__action__Probe() -> *const std::ffi::c_void;
}

// Action struct (zero-sized type)
pub struct Probe;

impl rosidl_runtime_rs::Action for Probe {
    type Goal = ProbeGoal;
    type Result = ProbeResult;
    type Feedback = ProbeFeedback;
    type FeedbackMessage = ProbeFeedbackMessage;
    type SendGoalService = Probe_SendGoal;
    type GetResultService = Probe_GetResult;
    type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

    fn get_type_support() -> *const std::ffi::c_void {
        unsafe { rosidl_typesupport_c__get_action_type_support_handle__fingerprint-corpus__action__Probe() }
    }

    fn create_goal_request(goal_id: &[u8; 16], goal: ProbeGoal) -> Probe_SendGoal_Request {
        Probe_SendGoal_Request {
            goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
            goal,
        }
    }

    fn split_goal_request(request: Probe_SendGoal_Request) -> ([u8; 16], ProbeGoal) {
        (request.goal_id.uuid, request.goal)
    }

    fn create_goal_response(accepted: bool, stamp: (i32, u32)) -> Probe_SendGoal_Response {
        Probe_SendGoal_Response {
            accepted,
            stamp: builtin_interfaces::msg::rmw::Time { sec: stamp.0, nanosec: stamp.1 },
        }
    }

    fn get_goal_response_accepted(response: &Probe_SendGoal_Response) -> bool {
        response.accepted
    }

    fn get_goal_response_stamp(response: &Probe_SendGoal_Response) -> (i32, u32) {
        (response.stamp.sec, response.stamp.nanosec)
    }

    fn create_feedback_message(goal_id: &[u8; 16], feedback: ProbeFeedback) -> ProbeFeedbackMessage {
        ProbeFeedbackMessage {
            goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
            feedback,
        }
    }

    fn split_feedback_message(feedback: ProbeFeedbackMessage) -> ([u8; 16], ProbeFeedback) {
        (feedback.goal_id.uuid, feedback.feedback)
    }

    fn create_result_request(goal_id: &[u8; 16]) -> Probe_GetResult_Request {
        Probe_GetResult_Request {
            goal_id: unique_identifier_msgs::msg::rmw::UUID { uuid: *goal_id },
        }
    }

    fn get_result_request_uuid(request: &Probe_GetResult_Request) -> &[u8; 16] {
        &request.goal_id.uuid
    }

    fn create_result_response(status: i8, result: ProbeResult) -> Probe_GetResult_Response {
        Probe_GetResult_Response {
            status,
            result,
        }
    }

    fn split_result_response(response: Probe_GetResult_Response) -> (i8, ProbeResult) {
        (response.status, response.result)
    }
}