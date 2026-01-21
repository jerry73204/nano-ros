//! ROS 2 Action types
//!
//! Actions provide asynchronous goal-based communication with feedback.
//! An action client sends a goal to an action server, which executes the goal
//! and provides feedback during execution and a result upon completion.
//!
//! ## Action Communication Pattern
//!
//! Actions use 5 underlying communication channels:
//! - `send_goal` service: Submit a new goal
//! - `cancel_goal` service: Request cancellation
//! - `get_result` service: Retrieve final result
//! - `feedback` topic: Progress updates during execution
//! - `status` topic: Goal state transitions
//!
//! ## Example
//!
//! ```ignore
//! use nano_ros_core::{RosAction, GoalStatus};
//!
//! // Define an action type
//! struct Fibonacci;
//!
//! impl RosAction for Fibonacci {
//!     type Goal = FibonacciGoal;
//!     type Result = FibonacciResult;
//!     type Feedback = FibonacciFeedback;
//!
//!     const ACTION_NAME: &'static str = "example_interfaces::action::dds_::Fibonacci_";
//!     const ACTION_HASH: &'static str = "...";
//! }
//! ```

use crate::types::RosMessage;
use core::fmt;

/// Trait for ROS 2 action types
///
/// This trait defines the associated types and metadata for a ROS 2 action.
/// Actions consist of three message types:
/// - `Goal`: Sent by client to initiate the action
/// - `Result`: Returned by server when action completes
/// - `Feedback`: Sent by server during execution to report progress
pub trait RosAction: Sized {
    /// Goal message sent by client to initiate the action
    type Goal: RosMessage;

    /// Result message returned by server upon completion
    type Result: RosMessage;

    /// Feedback message sent by server during execution
    type Feedback: RosMessage;

    /// Action type name (e.g., "example_interfaces::action::dds_::Fibonacci_")
    const ACTION_NAME: &'static str;

    /// Type hash for discovery (RIHS format)
    const ACTION_HASH: &'static str;
}

/// Goal status states
///
/// These states match `action_msgs/msg/GoalStatus` from ROS 2.
/// A goal progresses through these states during its lifecycle.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
#[repr(i8)]
pub enum GoalStatus {
    /// Status has not been set
    #[default]
    Unknown = 0,
    /// Goal has been accepted and is awaiting execution
    Accepted = 1,
    /// Goal is currently being executed
    Executing = 2,
    /// Goal is in the process of being canceled
    Canceling = 3,
    /// Goal completed successfully
    Succeeded = 4,
    /// Goal was canceled before completion
    Canceled = 5,
    /// Goal was aborted due to an error
    Aborted = 6,
}

impl GoalStatus {
    /// Check if the goal is in a terminal state (completed, canceled, or aborted)
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            GoalStatus::Succeeded | GoalStatus::Canceled | GoalStatus::Aborted
        )
    }

    /// Check if the goal is still active (accepted, executing, or canceling)
    pub fn is_active(&self) -> bool {
        matches!(
            self,
            GoalStatus::Accepted | GoalStatus::Executing | GoalStatus::Canceling
        )
    }

    /// Convert from i8 value
    pub fn from_i8(value: i8) -> Option<Self> {
        match value {
            0 => Some(GoalStatus::Unknown),
            1 => Some(GoalStatus::Accepted),
            2 => Some(GoalStatus::Executing),
            3 => Some(GoalStatus::Canceling),
            4 => Some(GoalStatus::Succeeded),
            5 => Some(GoalStatus::Canceled),
            6 => Some(GoalStatus::Aborted),
            _ => None,
        }
    }
}

impl fmt::Display for GoalStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GoalStatus::Unknown => write!(f, "UNKNOWN"),
            GoalStatus::Accepted => write!(f, "ACCEPTED"),
            GoalStatus::Executing => write!(f, "EXECUTING"),
            GoalStatus::Canceling => write!(f, "CANCELING"),
            GoalStatus::Succeeded => write!(f, "SUCCEEDED"),
            GoalStatus::Canceled => write!(f, "CANCELED"),
            GoalStatus::Aborted => write!(f, "ABORTED"),
        }
    }
}

/// Unique identifier for a goal
///
/// This is a 128-bit UUID matching `unique_identifier_msgs/msg/UUID`.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct GoalId {
    /// UUID bytes in standard format
    pub uuid: [u8; 16],
}

impl GoalId {
    /// Create a new GoalId from UUID bytes
    pub const fn new(uuid: [u8; 16]) -> Self {
        Self { uuid }
    }

    /// Create a zero/null GoalId
    pub const fn zero() -> Self {
        Self { uuid: [0; 16] }
    }

    /// Check if this is a zero/null GoalId
    pub fn is_zero(&self) -> bool {
        self.uuid == [0; 16]
    }

    /// Create a GoalId from a simple counter (for testing/embedded use)
    ///
    /// This creates a deterministic UUID-like identifier from a counter value.
    /// Not a true UUID, but useful for embedded systems without random number generators.
    pub fn from_counter(counter: u64) -> Self {
        let mut uuid = [0u8; 16];
        // Put counter in last 8 bytes (big-endian)
        uuid[8..16].copy_from_slice(&counter.to_be_bytes());
        // Set version 4 (random) variant bits for compatibility
        uuid[6] = (uuid[6] & 0x0f) | 0x40; // Version 4
        uuid[8] = (uuid[8] & 0x3f) | 0x80; // Variant 1
        Self { uuid }
    }
}

impl Default for GoalId {
    fn default() -> Self {
        Self::zero()
    }
}

impl fmt::Debug for GoalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Format as UUID string: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
        write!(
            f,
            "GoalId({:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x})",
            self.uuid[0], self.uuid[1], self.uuid[2], self.uuid[3],
            self.uuid[4], self.uuid[5],
            self.uuid[6], self.uuid[7],
            self.uuid[8], self.uuid[9],
            self.uuid[10], self.uuid[11], self.uuid[12], self.uuid[13], self.uuid[14], self.uuid[15]
        )
    }
}

impl fmt::Display for GoalId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.uuid[0], self.uuid[1], self.uuid[2], self.uuid[3],
            self.uuid[4], self.uuid[5],
            self.uuid[6], self.uuid[7],
            self.uuid[8], self.uuid[9],
            self.uuid[10], self.uuid[11], self.uuid[12], self.uuid[13], self.uuid[14], self.uuid[15]
        )
    }
}

/// Cancel goal response codes
///
/// These codes match `action_msgs/srv/CancelGoal` response codes.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(i8)]
pub enum CancelResponse {
    /// No error, goal(s) will be canceled
    #[default]
    Ok = 0,
    /// Goal was rejected (not cancelable or doesn't exist)
    Rejected = 1,
    /// Unknown goal ID
    UnknownGoal = 2,
    /// Goal is already in a terminal state
    GoalTerminated = 3,
}

impl CancelResponse {
    /// Convert from i8 value
    pub fn from_i8(value: i8) -> Option<Self> {
        match value {
            0 => Some(CancelResponse::Ok),
            1 => Some(CancelResponse::Rejected),
            2 => Some(CancelResponse::UnknownGoal),
            3 => Some(CancelResponse::GoalTerminated),
            _ => None,
        }
    }
}

/// Goal accept/reject response codes
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(i8)]
pub enum GoalResponse {
    /// Goal was rejected
    #[default]
    Reject = 0,
    /// Goal was accepted and will be executed
    AcceptAndExecute = 1,
    /// Goal was accepted and is deferred
    AcceptAndDefer = 2,
}

impl GoalResponse {
    /// Convert from i8 value
    pub fn from_i8(value: i8) -> Option<Self> {
        match value {
            0 => Some(GoalResponse::Reject),
            1 => Some(GoalResponse::AcceptAndExecute),
            2 => Some(GoalResponse::AcceptAndDefer),
            _ => None,
        }
    }

    /// Check if the goal was accepted
    pub fn is_accepted(&self) -> bool {
        matches!(
            self,
            GoalResponse::AcceptAndExecute | GoalResponse::AcceptAndDefer
        )
    }
}

/// Action server handle (type-level marker)
///
/// This is a lightweight handle for tracking an action server.
/// The actual implementation is in `nano-ros-node`.
pub struct ActionServer<A: RosAction> {
    /// Action name (e.g., "/fibonacci")
    pub name: &'static str,
    /// Marker for action type
    _marker: core::marker::PhantomData<A>,
}

impl<A: RosAction> ActionServer<A> {
    /// Create a new action server handle
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            _marker: core::marker::PhantomData,
        }
    }

    /// Get the action name
    pub fn name(&self) -> &str {
        self.name
    }

    /// Get the action type name
    pub fn action_type(&self) -> &'static str {
        A::ACTION_NAME
    }

    /// Get the action type hash
    pub fn action_hash(&self) -> &'static str {
        A::ACTION_HASH
    }
}

/// Action client handle (type-level marker)
///
/// This is a lightweight handle for tracking an action client.
/// The actual implementation is in `nano-ros-node`.
pub struct ActionClient<A: RosAction> {
    /// Action name (e.g., "/fibonacci")
    pub name: &'static str,
    /// Marker for action type
    _marker: core::marker::PhantomData<A>,
}

impl<A: RosAction> ActionClient<A> {
    /// Create a new action client handle
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            _marker: core::marker::PhantomData,
        }
    }

    /// Get the action name
    pub fn name(&self) -> &str {
        self.name
    }

    /// Get the action type name
    pub fn action_type(&self) -> &'static str {
        A::ACTION_NAME
    }

    /// Get the action type hash
    pub fn action_hash(&self) -> &'static str {
        A::ACTION_HASH
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::format;

    #[test]
    fn test_goal_status_is_terminal() {
        assert!(!GoalStatus::Unknown.is_terminal());
        assert!(!GoalStatus::Accepted.is_terminal());
        assert!(!GoalStatus::Executing.is_terminal());
        assert!(!GoalStatus::Canceling.is_terminal());
        assert!(GoalStatus::Succeeded.is_terminal());
        assert!(GoalStatus::Canceled.is_terminal());
        assert!(GoalStatus::Aborted.is_terminal());
    }

    #[test]
    fn test_goal_status_is_active() {
        assert!(!GoalStatus::Unknown.is_active());
        assert!(GoalStatus::Accepted.is_active());
        assert!(GoalStatus::Executing.is_active());
        assert!(GoalStatus::Canceling.is_active());
        assert!(!GoalStatus::Succeeded.is_active());
        assert!(!GoalStatus::Canceled.is_active());
        assert!(!GoalStatus::Aborted.is_active());
    }

    #[test]
    fn test_goal_status_from_i8() {
        assert_eq!(GoalStatus::from_i8(0), Some(GoalStatus::Unknown));
        assert_eq!(GoalStatus::from_i8(1), Some(GoalStatus::Accepted));
        assert_eq!(GoalStatus::from_i8(2), Some(GoalStatus::Executing));
        assert_eq!(GoalStatus::from_i8(3), Some(GoalStatus::Canceling));
        assert_eq!(GoalStatus::from_i8(4), Some(GoalStatus::Succeeded));
        assert_eq!(GoalStatus::from_i8(5), Some(GoalStatus::Canceled));
        assert_eq!(GoalStatus::from_i8(6), Some(GoalStatus::Aborted));
        assert_eq!(GoalStatus::from_i8(7), None);
        assert_eq!(GoalStatus::from_i8(-1), None);
    }

    #[test]
    fn test_goal_id_zero() {
        let id = GoalId::zero();
        assert!(id.is_zero());
        assert_eq!(id.uuid, [0; 16]);
    }

    #[test]
    fn test_goal_id_from_counter() {
        let id1 = GoalId::from_counter(1);
        let id2 = GoalId::from_counter(2);

        assert!(!id1.is_zero());
        assert!(!id2.is_zero());
        assert_ne!(id1, id2);

        // Same counter should produce same ID
        let id1_again = GoalId::from_counter(1);
        assert_eq!(id1, id1_again);
    }

    #[test]
    fn test_goal_id_display() {
        let id = GoalId::new([
            0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
            0x77, 0x88,
        ]);
        let s = format!("{}", id);
        assert_eq!(s, "12345678-9abc-def0-1122-334455667788");
    }

    #[test]
    fn test_cancel_response_from_i8() {
        assert_eq!(CancelResponse::from_i8(0), Some(CancelResponse::Ok));
        assert_eq!(CancelResponse::from_i8(1), Some(CancelResponse::Rejected));
        assert_eq!(
            CancelResponse::from_i8(2),
            Some(CancelResponse::UnknownGoal)
        );
        assert_eq!(
            CancelResponse::from_i8(3),
            Some(CancelResponse::GoalTerminated)
        );
        assert_eq!(CancelResponse::from_i8(4), None);
    }

    #[test]
    fn test_goal_response_is_accepted() {
        assert!(!GoalResponse::Reject.is_accepted());
        assert!(GoalResponse::AcceptAndExecute.is_accepted());
        assert!(GoalResponse::AcceptAndDefer.is_accepted());
    }
}
