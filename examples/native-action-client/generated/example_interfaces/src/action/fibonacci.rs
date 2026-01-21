// nano-ros action type - pure Rust, no_std compatible
// Package: example_interfaces
// Action: Fibonacci

use nano_ros_core::{RosMessage, RosAction, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

// ============================================================================
// Goal Message
// ============================================================================

/// Fibonacci goal message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FibonacciGoal {
    pub order: i32,
}

impl Serialize for FibonacciGoal {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.order)?;
        Ok(())
    }
}

impl Deserialize for FibonacciGoal {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            order: reader.read_i32()?,
        })
    }
}

impl RosMessage for FibonacciGoal {
    const TYPE_NAME: &'static str = "example_interfaces::action::dds_::Fibonacci_Goal_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}

// ============================================================================
// Result Message
// ============================================================================

/// Fibonacci result message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FibonacciResult {
    pub sequence: heapless::Vec<i32, 64>,
}

impl Serialize for FibonacciResult {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u32(self.sequence.len() as u32)?;
        for item in &self.sequence {
            writer.write_i32(*item)?;
        }
        Ok(())
    }
}

impl Deserialize for FibonacciResult {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sequence: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    vec.push(reader.read_i32()?).map_err(|_| DeserError::CapacityExceeded)?;
                }
                vec
            },
        })
    }
}

impl RosMessage for FibonacciResult {
    const TYPE_NAME: &'static str = "example_interfaces::action::dds_::Fibonacci_Result_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}

// ============================================================================
// Feedback Message
// ============================================================================

/// Fibonacci feedback message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FibonacciFeedback {
    pub sequence: heapless::Vec<i32, 64>,
}

impl Serialize for FibonacciFeedback {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_u32(self.sequence.len() as u32)?;
        for item in &self.sequence {
            writer.write_i32(*item)?;
        }
        Ok(())
    }
}

impl Deserialize for FibonacciFeedback {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sequence: {
                let len = reader.read_u32()? as usize;
                let mut vec = heapless::Vec::new();
                for _ in 0..len {
                    vec.push(reader.read_i32()?).map_err(|_| DeserError::CapacityExceeded)?;
                }
                vec
            },
        })
    }
}

impl RosMessage for FibonacciFeedback {
    const TYPE_NAME: &'static str = "example_interfaces::action::dds_::Fibonacci_Feedback_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}

// ============================================================================
// Action Definition
// ============================================================================

/// Fibonacci action definition
pub struct Fibonacci;

impl RosAction for Fibonacci {
    type Goal = FibonacciGoal;
    type Result = FibonacciResult;
    type Feedback = FibonacciFeedback;

    const ACTION_NAME: &'static str = "example_interfaces::action::dds_::Fibonacci_";
    const ACTION_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}