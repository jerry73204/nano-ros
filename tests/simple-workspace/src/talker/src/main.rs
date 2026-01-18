//! nano-ros talker executable
//!
//! Demonstrates message serialization using generated std_msgs bindings.

use nano_ros_core::{RosMessage, Serialize};
use nano_ros_serdes::CdrWriter;
use std_msgs::msg::Int32;

fn main() {
    println!("nano-ros talker starting...");
    println!("Message type: {}", Int32::TYPE_NAME);

    let mut count: i32 = 0;

    loop {
        // Create message
        let msg = Int32 { data: count };

        // Serialize to buffer
        let mut buffer = [0u8; 64];
        let mut writer = CdrWriter::new_with_header(&mut buffer).unwrap();
        msg.serialize(&mut writer).unwrap();

        println!(
            "Publishing: {} (serialized {} bytes)",
            count,
            writer.position()
        );

        count += 1;

        // Sleep for 1 second
        std::thread::sleep(std::time::Duration::from_secs(1));

        if count > 10 {
            println!("Done!");
            break;
        }
    }
}
