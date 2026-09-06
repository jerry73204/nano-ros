// emit:ok
// Idiomatic Rust layer - user-friendly types
// Package: fingerprint-corpus
// Action: Probe

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// Goal message
pub mod goal {
    #[cfg(feature = "serde")]
    use super::{Deserialize, Serialize};

    #[derive(Debug, Clone, PartialEq)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct ProbeGoal {
        
        pub waypoints: std::vec::Vec<i64>,
        
        pub name: std::string::String,
        
    }

    impl ProbeGoal {
        
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Default for ProbeGoal {
        fn default() -> Self {
            // Leverage FFI message's C init function to get correct default values
            <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ProbeGoal::default())
        }
    }

    // Conversion from FFI layer
    impl From<crate::action::rmw::ProbeGoal> for ProbeGoal {
        #[allow(unused_variables)]
        fn from(rmw: crate::action::rmw::ProbeGoal) -> Self {
            Self {
                
                
                waypoints: rmw.waypoints.as_slice().to_vec(),
                
                
                
                name: rmw.name.to_string(),
                
                
            }
        }
    }

    // Conversion to FFI layer
    impl From<ProbeGoal> for crate::action::rmw::ProbeGoal {
        #[allow(unused_variables)]
        fn from(idiomatic: ProbeGoal) -> Self {
            Self {
                
                
                waypoints: crate::rosidl_runtime_rs::Sequence::from(idiomatic.waypoints.clone()),
                
                
                
                name: crate::rosidl_runtime_rs::String::from(idiomatic.name.as_str()),
                
                
            }
        }
    }

    // Message trait implementation for rosidl_runtime_rs
    impl crate::rosidl_runtime_rs::Message for ProbeGoal {
        type RmwMsg = crate::action::rmw::ProbeGoal;

        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            std::borrow::Cow::Owned(msg_cow.into_owned().into())
        }

        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg.into()
        }
    }
}

// Result message
pub mod result {
    #[cfg(feature = "serde")]
    use super::{Deserialize, Serialize};

    #[derive(Debug, Clone, PartialEq)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct ProbeResult {
        
        pub total: i64,
        
        pub report: std::string::String,
        
    }

    impl ProbeResult {
        
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Default for ProbeResult {
        fn default() -> Self {
            <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ProbeResult::default())
        }
    }

    impl From<crate::action::rmw::ProbeResult> for ProbeResult {
        #[allow(unused_variables)]
        fn from(rmw: crate::action::rmw::ProbeResult) -> Self {
            Self {
                
                
                total: rmw.total,
                
                
                
                report: rmw.report.to_string(),
                
                
            }
        }
    }

    impl From<ProbeResult> for crate::action::rmw::ProbeResult {
        #[allow(unused_variables)]
        fn from(idiomatic: ProbeResult) -> Self {
            Self {
                
                
                total: idiomatic.total,
                
                
                
                report: crate::rosidl_runtime_rs::String::from(idiomatic.report.as_str()),
                
                
            }
        }
    }

    impl crate::rosidl_runtime_rs::Message for ProbeResult {
        type RmwMsg = crate::action::rmw::ProbeResult;

        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            std::borrow::Cow::Owned(msg_cow.into_owned().into())
        }

        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg.into()
        }
    }
}

// Feedback message
pub mod feedback {
    #[cfg(feature = "serde")]
    use super::{Deserialize, Serialize};

    #[derive(Debug, Clone, PartialEq)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct ProbeFeedback {
        
        pub done: i64,
        
        pub stage: std::string::String,
        
    }

    impl ProbeFeedback {
        
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Default for ProbeFeedback {
        fn default() -> Self {
            <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::action::rmw::ProbeFeedback::default())
        }
    }

    impl From<crate::action::rmw::ProbeFeedback> for ProbeFeedback {
        #[allow(unused_variables)]
        fn from(rmw: crate::action::rmw::ProbeFeedback) -> Self {
            Self {
                
                
                done: rmw.done,
                
                
                
                stage: rmw.stage.to_string(),
                
                
            }
        }
    }

    impl From<ProbeFeedback> for crate::action::rmw::ProbeFeedback {
        #[allow(unused_variables)]
        fn from(idiomatic: ProbeFeedback) -> Self {
            Self {
                
                
                done: idiomatic.done,
                
                
                
                stage: crate::rosidl_runtime_rs::String::from(idiomatic.stage.as_str()),
                
                
            }
        }
    }

    impl crate::rosidl_runtime_rs::Message for ProbeFeedback {
        type RmwMsg = crate::action::rmw::ProbeFeedback;

        fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
            std::borrow::Cow::Owned(msg_cow.into_owned().into())
        }

        fn from_rmw_message(msg: Self::RmwMsg) -> Self {
            msg.into()
        }
    }
}

// Re-export for convenience
pub use goal::ProbeGoal;
pub use result::ProbeResult;
pub use feedback::ProbeFeedback;

// Action struct (zero-sized type)
pub struct Probe;

impl crate::rosidl_runtime_rs::Action for Probe {
    type Goal = ProbeGoal;
    type Result = ProbeResult;
    type Feedback = ProbeFeedback;
    type FeedbackMessage = crate::action::rmw::ProbeFeedbackMessage;
    type SendGoalService = crate::action::rmw::Probe_SendGoal;
    type GetResultService = crate::action::rmw::Probe_GetResult;
    type CancelGoalService = action_msgs::srv::rmw::CancelGoal;

    fn get_type_support() -> *const std::ffi::c_void {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::get_type_support()
    }

    fn create_goal_request(
        goal_id: &[u8; 16],
        goal: crate::action::rmw::ProbeGoal,
    ) -> crate::action::rmw::Probe_SendGoal_Request {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::create_goal_request(goal_id, goal)
    }

    fn split_goal_request(
        request: crate::action::rmw::Probe_SendGoal_Request,
    ) -> ([u8; 16], crate::action::rmw::ProbeGoal) {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::split_goal_request(request)
    }

    fn create_goal_response(
        accepted: bool,
        stamp: (i32, u32),
    ) -> crate::action::rmw::Probe_SendGoal_Response {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::create_goal_response(accepted, stamp)
    }

    fn get_goal_response_accepted(response: &crate::action::rmw::Probe_SendGoal_Response) -> bool {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::get_goal_response_accepted(response)
    }

    fn get_goal_response_stamp(response: &crate::action::rmw::Probe_SendGoal_Response) -> (i32, u32) {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::get_goal_response_stamp(response)
    }

    fn create_feedback_message(
        goal_id: &[u8; 16],
        feedback: crate::action::rmw::ProbeFeedback,
    ) -> crate::action::rmw::ProbeFeedbackMessage {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::create_feedback_message(goal_id, feedback)
    }

    fn split_feedback_message(
        feedback: crate::action::rmw::ProbeFeedbackMessage,
    ) -> ([u8; 16], crate::action::rmw::ProbeFeedback) {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::split_feedback_message(feedback)
    }

    fn create_result_request(goal_id: &[u8; 16]) -> crate::action::rmw::Probe_GetResult_Request {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::create_result_request(goal_id)
    }

    fn get_result_request_uuid(request: &crate::action::rmw::Probe_GetResult_Request) -> &[u8; 16] {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::get_result_request_uuid(request)
    }

    fn create_result_response(
        status: i8,
        result: crate::action::rmw::ProbeResult,
    ) -> crate::action::rmw::Probe_GetResult_Response {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::create_result_response(status, result)
    }

    fn split_result_response(
        response: crate::action::rmw::Probe_GetResult_Response,
    ) -> (i8, crate::action::rmw::ProbeResult) {
        <crate::action::rmw::Probe as crate::rosidl_runtime_rs::Action>::split_result_response(response)
    }
}