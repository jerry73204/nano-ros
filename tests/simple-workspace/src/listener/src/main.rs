//! nano-ros listener executable
//!
//! Demonstrates message deserialization using generated std_msgs bindings.

use nano_ros_core::{Deserialize, RosMessage};
use nano_ros_serdes::CdrReader;
use std_msgs::msg::Int32;

fn main() {
    println!("nano-ros listener starting...");
    println!("Message type: {}", Int32::TYPE_NAME);

    // Simulate receiving serialized messages
    for i in 0..5 {
        // Simulated CDR-encoded Int32 message
        let value: i32 = i * 10;
        let mut buffer = [0u8; 64];

        // CDR header (little-endian)
        buffer[0..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
        // i32 value
        buffer[4..8].copy_from_slice(&value.to_le_bytes());

        // Deserialize
        let mut reader = CdrReader::new_with_header(&buffer).unwrap();
        let msg = Int32::deserialize(&mut reader).unwrap();

        println!("Received: {}", msg.data);

        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    println!("Done!");
}
