//! Custom Message Example
//!
//! Demonstrates how to define and use custom message types with nano-ros.
//!
//! This example shows two approaches:
//! 1. Manually defining message types (for learning/prototyping)
//! 2. Using generated types from `cargo nano-ros generate` (for production)
//!
//! # Running
//!
//! ```bash
//! # Without zenoh (serialization test only):
//! cargo run
//!
//! # With zenoh (full pub/sub):
//! zenohd --listen tcp/127.0.0.1:7447  # In another terminal
//! cargo run --features zenoh
//! ```
//!
//! # Creating Real Custom Messages
//!
//! For production use, create a ROS 2 interface package:
//!
//! 1. Create a ROS 2 package with your .msg files
//! 2. Build with `colcon build`
//! 3. Add to package.xml: `<depend>your_custom_msgs</depend>`
//! 4. Run `cargo nano-ros generate`
//!
//! The .msg files in this example's `msg/` directory show the format.

use heapless::String as HString;
#[cfg(feature = "zenoh")]
use log::info;
use nano_ros::{CdrReader, CdrWriter, Deserialize, RosMessage, Serialize};
use nano_ros_serdes::{DeserError, SerError};

// =============================================================================
// Custom Message Type Definitions
// =============================================================================
//
// These demonstrate how to manually define message types.
// In production, use `cargo nano-ros generate` to create these automatically.

/// Custom sensor reading message
///
/// Corresponds to msg/SensorReading.msg:
/// ```text
/// int32 sensor_id
/// float32 temperature
/// float32 humidity
/// uint64 timestamp
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SensorReading {
    pub sensor_id: i32,
    pub temperature: f32,
    pub humidity: f32,
    pub timestamp: u64,
}

impl Serialize for SensorReading {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_i32(self.sensor_id)?;
        writer.write_f32(self.temperature)?;
        writer.write_f32(self.humidity)?;
        writer.write_u64(self.timestamp)?;
        Ok(())
    }
}

impl Deserialize for SensorReading {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            sensor_id: reader.read_i32()?,
            temperature: reader.read_f32()?,
            humidity: reader.read_f32()?,
            timestamp: reader.read_u64()?,
        })
    }
}

impl RosMessage for SensorReading {
    // Type name follows ROS 2 DDS naming convention
    const TYPE_NAME: &'static str = "custom_msgs::msg::dds_::SensorReading_";
    // For custom types without ROS 2, use placeholder hash
    const TYPE_HASH: &'static str = "TypeHashNotSupported";
}

/// Custom status message with string field
///
/// Corresponds to msg/Status.msg:
/// ```text
/// bool active
/// string message
/// int32 error_code
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Status {
    pub active: bool,
    pub message: HString<256>,
    pub error_code: i32,
}

impl Serialize for Status {
    fn serialize(&self, writer: &mut CdrWriter) -> Result<(), SerError> {
        writer.write_bool(self.active)?;
        writer.write_string(&self.message)?;
        writer.write_i32(self.error_code)?;
        Ok(())
    }
}

impl Deserialize for Status {
    fn deserialize(reader: &mut CdrReader) -> Result<Self, DeserError> {
        Ok(Self {
            active: reader.read_bool()?,
            message: {
                let s = reader.read_string()?;
                HString::try_from(s).unwrap_or_default()
            },
            error_code: reader.read_i32()?,
        })
    }
}

impl RosMessage for Status {
    const TYPE_NAME: &'static str = "custom_msgs::msg::dds_::Status_";
    const TYPE_HASH: &'static str = "TypeHashNotSupported";
}

// =============================================================================
// Test Helpers
// =============================================================================

/// Test serialization roundtrip for a message type
fn test_roundtrip<T: RosMessage + Serialize + Deserialize + PartialEq + std::fmt::Debug>(
    original: &T,
    name: &str,
) -> bool {
    // Buffer for serialization
    let mut buf = [0u8; 1024];

    // Serialize with CDR encapsulation header
    let mut writer = CdrWriter::new_with_header(&mut buf).expect("create writer");
    original.serialize(&mut writer).expect("serialize");
    let serialized_size = writer.position();

    // Deserialize
    let mut reader = CdrReader::new_with_header(&buf[..serialized_size]).expect("create reader");
    let deserialized = T::deserialize(&mut reader).expect("deserialize");

    let success = *original == deserialized;
    println!(
        "  {}: {} bytes, roundtrip {}",
        name,
        serialized_size,
        if success { "OK" } else { "FAILED" }
    );
    success
}

// =============================================================================
// Main
// =============================================================================

fn main() {
    env_logger::init();

    println!("nano-ros Custom Message Example (Rust)");
    println!("======================================");
    println!();

    // =========================================================================
    // Test custom message serialization
    // =========================================================================
    println!("Testing custom message serialization:");

    // Test SensorReading
    let sensor = SensorReading {
        sensor_id: 42,
        temperature: 23.5,
        humidity: 65.0,
        timestamp: 1234567890123,
    };
    assert!(test_roundtrip(&sensor, "SensorReading"));

    // Test Status
    let status = Status {
        active: true,
        message: HString::try_from("System operational").unwrap(),
        error_code: 0,
    };
    assert!(test_roundtrip(&status, "Status"));

    // Test generated std_msgs::Int32 for comparison
    let int_msg = std_msgs::msg::Int32 { data: 12345 };
    assert!(test_roundtrip(&int_msg, "std_msgs::Int32"));

    println!();
    println!("All serialization tests passed!");
    println!();

    // =========================================================================
    // Pub/Sub with custom messages (requires zenoh feature)
    // =========================================================================
    #[cfg(feature = "zenoh")]
    {
        use nano_ros::prelude::*;
        use std::sync::atomic::{AtomicU64, Ordering};
        use std::time::Duration;

        static MSG_COUNT: AtomicU64 = AtomicU64::new(0);

        println!("Testing pub/sub with custom messages:");

        let context = match Context::from_env() {
            Ok(ctx) => ctx,
            Err(e) => {
                eprintln!("  Failed to create context: {:?}", e);
                eprintln!("  Make sure zenohd is running: zenohd --listen tcp/127.0.0.1:7447");
                return;
            }
        };
        info!("Context created, domain_id: {}", context.domain_id());

        let node = context
            .create_node("custom_msg_node")
            .expect("create node");
        info!("Node created: {}", node.fully_qualified_name());

        // Create publisher for SensorReading
        let publisher = node
            .create_publisher::<SensorReading>("/sensor_data")
            .expect("create publisher");
        info!("Publisher created for: /sensor_data");

        // Create subscriber for SensorReading
        let _subscriber = node
            .create_subscriber::<SensorReading, _>("/sensor_data", move |msg: SensorReading| {
                MSG_COUNT.fetch_add(1, Ordering::SeqCst);
                println!(
                    "  Received: sensor_id={}, temp={:.1}, humidity={:.1}",
                    msg.sensor_id, msg.temperature, msg.humidity
                );
            })
            .expect("create subscriber");
        info!("Subscriber created for: /sensor_data");

        println!();
        println!("Publishing sensor readings...");

        for i in 0..3 {
            let reading = SensorReading {
                sensor_id: i + 1,
                temperature: 20.0 + (i as f32) * 0.5,
                humidity: 50.0 + (i as f32) * 5.0,
                timestamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
            };

            publisher.publish(&reading).expect("publish");
            println!(
                "  Published: sensor_id={}, temp={:.1}, humidity={:.1}",
                reading.sensor_id, reading.temperature, reading.humidity
            );

            // Give time for message to be received
            std::thread::sleep(Duration::from_millis(100));
        }

        // Wait a bit more for any remaining messages
        std::thread::sleep(Duration::from_millis(500));

        let received = MSG_COUNT.load(Ordering::SeqCst);
        println!();
        println!("Received {} messages", received);
    }

    #[cfg(not(feature = "zenoh"))]
    {
        println!("Pub/sub test skipped (compile with --features zenoh)");
    }

    println!();
    println!("Custom message example completed successfully!");
}
