// nano-ros service type - pure Rust, no_std compatible
// Package: example_interfaces
// Service: AddTwoInts

use nano_ros_core::{RosMessage, RosService, Serialize, Deserialize};
use nano_ros_serdes::{CdrReader, CdrWriter, SerError, DeserError};

/// AddTwoInts request message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AddTwoIntsRequest {
    pub a: i64,
    pub b: i64,
}

impl Serialize for AddTwoIntsRequest {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i64(self.a)?;
        writer.write_i64(self.b)?;
        Ok(())
    }
}

impl Deserialize for AddTwoIntsRequest {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            a: reader.read_i64()?,
            b: reader.read_i64()?,
        })
    }
}

impl RosMessage for AddTwoIntsRequest {
    const TYPE_NAME: &'static str = "example_interfaces::srv::dds_::AddTwoInts_Request_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}

/// AddTwoInts response message
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AddTwoIntsResponse {
    pub sum: i64,
}

impl Serialize for AddTwoIntsResponse {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i64(self.sum)?;
        Ok(())
    }
}

impl Deserialize for AddTwoIntsResponse {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sum: reader.read_i64()?,
        })
    }
}

impl RosMessage for AddTwoIntsResponse {
    const TYPE_NAME: &'static str = "example_interfaces::srv::dds_::AddTwoInts_Response_";
    const TYPE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}

/// AddTwoInts service definition
pub struct AddTwoInts;

impl RosService for AddTwoInts {
    type Request = AddTwoIntsRequest;
    type Reply = AddTwoIntsResponse;

    const SERVICE_NAME: &'static str = "example_interfaces::srv::dds_::AddTwoInts_";
    const SERVICE_HASH: &'static str = "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
}