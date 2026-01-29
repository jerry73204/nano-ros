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
use nano_ros_serdes::{CdrReader, CdrWriter, DeserError, Deserialize, SerError, Serialize};

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

impl Serialize for GoalStatus {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i8(*self as i8)
    }
}

impl Deserialize for GoalStatus {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let value = reader.read_i8()?;
        GoalStatus::from_i8(value).ok_or(DeserError::InvalidData)
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

impl Serialize for GoalId {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        for byte in &self.uuid {
            writer.write_u8(*byte)?;
        }
        Ok(())
    }
}

impl Deserialize for GoalId {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let mut uuid = [0u8; 16];
        for byte in &mut uuid {
            *byte = reader.read_u8()?;
        }
        Ok(Self { uuid })
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

/// Information about a goal
///
/// This matches `action_msgs/msg/GoalInfo` from ROS 2.
/// Contains the goal ID and timestamp when the goal was accepted.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GoalInfo {
    /// Unique identifier for the goal
    pub goal_id: GoalId,
    /// Timestamp when the goal was accepted (nanoseconds since epoch)
    pub stamp_sec: i32,
    /// Nanosecond part of timestamp
    pub stamp_nanosec: u32,
}

impl GoalInfo {
    /// Create a new GoalInfo with the given ID and timestamp
    pub const fn new(goal_id: GoalId, stamp_sec: i32, stamp_nanosec: u32) -> Self {
        Self {
            goal_id,
            stamp_sec,
            stamp_nanosec,
        }
    }

    /// Create a GoalInfo with zero timestamp
    pub const fn with_id(goal_id: GoalId) -> Self {
        Self {
            goal_id,
            stamp_sec: 0,
            stamp_nanosec: 0,
        }
    }
}

impl Serialize for GoalInfo {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        self.goal_id.serialize(writer)?;
        writer.write_i32(self.stamp_sec)?;
        writer.write_u32(self.stamp_nanosec)?;
        Ok(())
    }
}

impl Deserialize for GoalInfo {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            goal_id: GoalId::deserialize(reader)?,
            stamp_sec: reader.read_i32()?,
            stamp_nanosec: reader.read_u32()?,
        })
    }
}

/// Goal status with associated goal info
///
/// This matches `action_msgs/msg/GoalStatus` from ROS 2.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GoalStatusStamped {
    /// Goal information (ID and timestamp)
    pub goal_info: GoalInfo,
    /// Current status of the goal
    pub status: GoalStatus,
}

impl GoalStatusStamped {
    /// Create a new GoalStatusStamped
    pub const fn new(goal_info: GoalInfo, status: GoalStatus) -> Self {
        Self { goal_info, status }
    }
}

impl Serialize for GoalStatusStamped {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        self.goal_info.serialize(writer)?;
        writer.write_i8(self.status as i8)?;
        Ok(())
    }
}

impl Deserialize for GoalStatusStamped {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let goal_info = GoalInfo::deserialize(reader)?;
        let status_val = reader.read_i8()?;
        let status = GoalStatus::from_i8(status_val).unwrap_or(GoalStatus::Unknown);
        Ok(Self { goal_info, status })
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

impl Serialize for CancelResponse {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i8(*self as i8)
    }
}

impl Deserialize for CancelResponse {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        let value = reader.read_i8()?;
        CancelResponse::from_i8(value).ok_or(DeserError::InvalidData)
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

    #[test]
    fn test_goal_info_new() {
        let goal_id = GoalId::from_counter(42);
        let info = GoalInfo::new(goal_id, 123, 456);
        assert_eq!(info.goal_id, goal_id);
        assert_eq!(info.stamp_sec, 123);
        assert_eq!(info.stamp_nanosec, 456);
    }

    #[test]
    fn test_goal_info_with_id() {
        let goal_id = GoalId::from_counter(42);
        let info = GoalInfo::with_id(goal_id);
        assert_eq!(info.goal_id, goal_id);
        assert_eq!(info.stamp_sec, 0);
        assert_eq!(info.stamp_nanosec, 0);
    }

    #[test]
    fn test_goal_status_stamped() {
        let goal_id = GoalId::from_counter(1);
        let info = GoalInfo::with_id(goal_id);
        let stamped = GoalStatusStamped::new(info, GoalStatus::Executing);
        assert_eq!(stamped.goal_info.goal_id, goal_id);
        assert_eq!(stamped.status, GoalStatus::Executing);
    }

    #[test]
    fn test_goal_id_serialization() {
        use nano_ros_serdes::{CdrReader, CdrWriter};

        let original = GoalId::new([
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
            0x0f, 0x10,
        ]);

        // Serialize
        let mut buf = [0u8; 32];
        let mut writer = CdrWriter::new(&mut buf);
        original.serialize(&mut writer).unwrap();
        let len = writer.position();
        assert_eq!(len, 16); // UUID is 16 bytes

        // Deserialize
        let mut reader = CdrReader::new(&buf[..len]);
        let deserialized = GoalId::deserialize(&mut reader).unwrap();

        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_goal_status_serialization() {
        use nano_ros_serdes::{CdrReader, CdrWriter};

        for status in [
            GoalStatus::Unknown,
            GoalStatus::Accepted,
            GoalStatus::Executing,
            GoalStatus::Canceling,
            GoalStatus::Succeeded,
            GoalStatus::Canceled,
            GoalStatus::Aborted,
        ] {
            let mut buf = [0u8; 8];
            let len = {
                let mut writer = CdrWriter::new(&mut buf);
                status.serialize(&mut writer).unwrap();
                writer.position()
            };

            let mut reader = CdrReader::new(&buf[..len]);
            let deserialized = GoalStatus::deserialize(&mut reader).unwrap();

            assert_eq!(status, deserialized);
        }
    }

    #[test]
    fn test_goal_info_serialization() {
        use nano_ros_serdes::{CdrReader, CdrWriter};

        let goal_id = GoalId::from_counter(42);
        let original = GoalInfo::new(goal_id, 1234567890, 123456789);

        // Serialize
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new(&mut buf);
        original.serialize(&mut writer).unwrap();
        let len = writer.position();
        // UUID (16) + i32 (4) + u32 (4) = 24 bytes
        assert_eq!(len, 24);

        // Deserialize
        let mut reader = CdrReader::new(&buf[..len]);
        let deserialized = GoalInfo::deserialize(&mut reader).unwrap();

        assert_eq!(original.goal_id, deserialized.goal_id);
        assert_eq!(original.stamp_sec, deserialized.stamp_sec);
        assert_eq!(original.stamp_nanosec, deserialized.stamp_nanosec);
    }

    #[test]
    fn test_goal_status_stamped_serialization() {
        use nano_ros_serdes::{CdrReader, CdrWriter};

        let goal_id = GoalId::from_counter(99);
        let goal_info = GoalInfo::new(goal_id, 987654321, 111222333);
        let original = GoalStatusStamped::new(goal_info, GoalStatus::Succeeded);

        // Serialize
        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new(&mut buf);
        original.serialize(&mut writer).unwrap();
        let len = writer.position();
        // GoalInfo (24) + i8 (1) = 25 bytes
        assert_eq!(len, 25);

        // Deserialize
        let mut reader = CdrReader::new(&buf[..len]);
        let deserialized = GoalStatusStamped::deserialize(&mut reader).unwrap();

        assert_eq!(original.goal_info.goal_id, deserialized.goal_info.goal_id);
        assert_eq!(
            original.goal_info.stamp_sec,
            deserialized.goal_info.stamp_sec
        );
        assert_eq!(
            original.goal_info.stamp_nanosec,
            deserialized.goal_info.stamp_nanosec
        );
        assert_eq!(original.status, deserialized.status);
    }
}
