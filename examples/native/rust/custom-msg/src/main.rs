//! Custom Message Example
//!
//! Demonstrates how to define and use custom message types with nros.
//!
//! This example shows two approaches:
//! 1. Manually defining message types (for learning/prototyping)
//! 2. Using generated types from `nros generate-rust` (for production)
//!
//! # Running
//!
//! ```bash
//! # Start zenoh router first:
//! ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
//!
//! # Run the example:
//! cargo run
//! ```
//!
//! # Creating Real Custom Messages
//!
//! For production use, create a ROS 2 interface package:
//!
//! 1. Create a ROS 2 package with your .msg files
//! 2. Build with `colcon build`
//! 3. Add to package.xml: `<depend>your_custom_msgs</depend>`
//! 4. Run `nros generate-rust`
//!
//! The .msg files in this example's `msg/` directory show the format.

use heapless::String as HString;
use nros::{CdrReader, CdrWriter, DeserError, Deserialize, RosMessage, SerError, Serialize};
use nros_log::{Logger, log_info};

// Phase 88.16.B — diagnostics route through `nros-log`.
static LOGGER: Logger = Logger::new("custom-msg");

extern crate nros_platform_cffi as _;

// =============================================================================
// Custom Message Type Definitions
// =============================================================================
//
// These demonstrate how to manually define message types.
// In production, use `nros generate-rust` to create these automatically.

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
    // Register the RMW backend the build linked (idempotent; must run before
    // the executor opens). RMW selection is build/config, never source.
    nros_board_linux::register_linked_rmw();

    nros_log::register_logger(&LOGGER);
    nros_log::init(nros_platform_cffi::log::default_sinks());

    println!("nros Custom Message Example (Rust)");
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
    // Pub/Sub with custom messages (requires zenoh router)
    // =========================================================================
    {
        use nros::prelude::*;
        use std::time::Duration;

        println!("Testing pub/sub with custom messages:");

        let config = ExecutorConfig::from_env().node_name("custom_msg_node");
        // Phase 115.L.5 — install zenoh-pico C-vtable backend.

        let mut executor: Executor = match Executor::open(&config) {
            Ok(e) => e,
            Err(e) => {
                println!("Skipping pub/sub (no zenoh router): {:?}", e);
                println!();
                println!("Custom message example completed successfully!");
                return;
            }
        };
        log_info!(&LOGGER, "Session created");

        // Create publisher
        let nid = executor
            .node_builder("custom_msg_node")
            .build()
            .expect("Failed to build node");
        let publisher = executor
            .node_mut(nid)
            .create_publisher::<SensorReading>("/sensor_data")
            .expect("Failed to create publisher");
        log_info!(&LOGGER, "Publisher created for: /sensor_data");

        // Register subscription callback
        executor
            .node_mut(nid)
            .create_subscription::<SensorReading, _>("/sensor_data", move |msg: &SensorReading| {
                println!(
                    "  Received: sensor_id={}, temp={:.1}, humidity={:.1}",
                    msg.sensor_id, msg.temperature, msg.humidity
                );
            })
            .expect("Failed to add subscription");
        log_info!(&LOGGER, "Subscriber created for: /sensor_data");

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

            executor.spin_once(core::time::Duration::from_millis(100));
            std::thread::sleep(Duration::from_millis(100));
        }

        // Process remaining callbacks
        executor.spin_once(core::time::Duration::from_millis(500));
    }

    println!();
    println!("Custom message example completed successfully!");
}
