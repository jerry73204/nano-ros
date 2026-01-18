//! example_interfaces - Example service and message types
//!
//! Provides simple service types for testing and examples.

use nano_ros_core::{RosMessage, RosService};
use nano_ros_serdes::{CdrReader, CdrWriter, DeserError, Deserialize, SerError, Serialize};

/// Request message for AddTwoInts service
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
    const TYPE_HASH: &'static str =
        "RIHS01_0000000000000000000000000000000000000000000000000000000000000001";
}

/// Response message for AddTwoInts service
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
    const TYPE_HASH: &'static str =
        "RIHS01_0000000000000000000000000000000000000000000000000000000000000001";
}

/// AddTwoInts service - adds two integers and returns the sum
pub struct AddTwoInts;

impl RosService for AddTwoInts {
    type Request = AddTwoIntsRequest;
    type Reply = AddTwoIntsResponse;

    const SERVICE_NAME: &'static str = "example_interfaces::srv::dds_::AddTwoInts_";
    const SERVICE_HASH: &'static str =
        "RIHS01_0000000000000000000000000000000000000000000000000000000000000001";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_two_ints_request_serialization() {
        let request = AddTwoIntsRequest { a: 10, b: 20 };

        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new(&mut buf);
        request.serialize(&mut writer).unwrap();
        let len = writer.position();

        let mut reader = CdrReader::new(&buf[..len]);
        let decoded = AddTwoIntsRequest::deserialize(&mut reader).unwrap();

        assert_eq!(request, decoded);
    }

    #[test]
    fn test_add_two_ints_response_serialization() {
        let response = AddTwoIntsResponse { sum: 30 };

        let mut buf = [0u8; 64];
        let mut writer = CdrWriter::new(&mut buf);
        response.serialize(&mut writer).unwrap();
        let len = writer.position();

        let mut reader = CdrReader::new(&buf[..len]);
        let decoded = AddTwoIntsResponse::deserialize(&mut reader).unwrap();

        assert_eq!(response, decoded);
    }
}
