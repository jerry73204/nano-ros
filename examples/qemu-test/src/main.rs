//! QEMU test binary for nano-ros on Cortex-M3
//!
//! This test verifies that nano-ros works correctly on embedded targets.
//! Run with: `just qemu-test`

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_semihosting as _;

use nano_ros_core::RosMessage;
use nano_ros_serdes::{CdrReader, CdrWriter, Deserialize, Serialize};
use std_msgs::msg::{Float64, Int32};
use builtin_interfaces::msg::Time;

// Import Node API
use nano_ros_node::{Node, NodeConfig};

/// Test primitive type serialization
fn test_int32_roundtrip() -> bool {
    let mut buf = [0u8; 16];
    let msg = Int32 { data: -42 };

    let mut writer = CdrWriter::new(&mut buf);
    if msg.serialize(&mut writer).is_err() {
        return false;
    }

    let mut reader = CdrReader::new(&buf);
    match Int32::deserialize(&mut reader) {
        Ok(result) => result.data == -42,
        Err(_) => false,
    }
}

/// Test floating point serialization
fn test_float64_roundtrip() -> bool {
    let mut buf = [0u8; 16];
    let test_value = 1.23456789; // Arbitrary test value
    let msg = Float64 { data: test_value };

    let mut writer = CdrWriter::new(&mut buf);
    if msg.serialize(&mut writer).is_err() {
        return false;
    }

    let mut reader = CdrReader::new(&buf);
    match Float64::deserialize(&mut reader) {
        Ok(result) => (result.data - test_value).abs() < 1e-10,
        Err(_) => false,
    }
}

/// Test Time type serialization
fn test_time_roundtrip() -> bool {
    let mut buf = [0u8; 16];
    let time = Time { sec: 1234, nanosec: 567890 };

    let mut writer = CdrWriter::new(&mut buf);
    if time.serialize(&mut writer).is_err() {
        return false;
    }

    let mut reader = CdrReader::new(&buf);
    match Time::deserialize(&mut reader) {
        Ok(result) => result.sec == 1234 && result.nanosec == 567890,
        Err(_) => false,
    }
}

/// Test type metadata
fn test_type_names() -> bool {
    let name_ok = Int32::TYPE_NAME == "std_msgs::msg::dds_::Int32_";
    let hash_ok = !Int32::TYPE_HASH.is_empty();
    name_ok && hash_ok
}

/// Test CDR encapsulation header
fn test_cdr_header() -> bool {
    let mut buf = [0u8; 16];

    // Create writer to write header, then let it go out of scope
    if CdrWriter::new_with_header(&mut buf).is_err() {
        return false;
    }

    // Verify CDR little-endian header
    buf[0] == 0x00 && buf[1] == 0x01 && buf[2] == 0x00 && buf[3] == 0x00
}

/// Test Node API
fn test_node_creation() -> bool {
    let config = NodeConfig::new("qemu_node", "/test");
    let node = Node::<4, 4>::new(config);

    node.name() == "qemu_node" && node.namespace() == "/test"
}

/// Test publisher creation via Node
fn test_node_publisher() -> bool {
    let mut node = Node::<4, 4>::default();
    let result = node.create_publisher::<Int32>("/counter");
    result.is_ok() && node.publisher_count() == 1
}

/// Test subscriber creation via Node
fn test_node_subscriber() -> bool {
    let mut node = Node::<4, 4>::default();
    let result = node.create_subscriber::<Int32>("/counter");
    result.is_ok() && node.subscriber_count() == 1
}

/// Test message serialization via Node
fn test_node_serialize() -> bool {
    let mut node = Node::<4, 4>::default();
    let pub_handle = match node.create_publisher::<Int32>("/test") {
        Ok(h) => h,
        Err(_) => return false,
    };

    let msg = Int32 { data: 123 };
    match node.serialize_message(&pub_handle, &msg) {
        Ok(bytes) => {
            // Should have CDR header (4 bytes) + i32 (4 bytes)
            bytes.len() == 8
        }
        Err(_) => false,
    }
}

#[entry]
fn main() -> ! {
    hprintln!("");
    hprintln!("========================================");
    hprintln!("  nano-ros QEMU Test Suite (Cortex-M3)");
    hprintln!("========================================");
    hprintln!("");

    let mut passed = 0;
    let mut failed = 0;

    macro_rules! run_test {
        ($name:expr, $test:expr) => {
            if $test {
                hprintln!("[PASS] {}", $name);
                passed += 1;
            } else {
                hprintln!("[FAIL] {}", $name);
                failed += 1;
            }
        };
    }

    hprintln!("--- Serialization Tests ---");
    run_test!("Int32 roundtrip", test_int32_roundtrip());
    run_test!("Float64 roundtrip", test_float64_roundtrip());
    run_test!("Time roundtrip", test_time_roundtrip());
    run_test!("Type names", test_type_names());
    run_test!("CDR header", test_cdr_header());

    hprintln!("");
    hprintln!("--- Node API Tests ---");
    run_test!("Node creation", test_node_creation());
    run_test!("Node publisher", test_node_publisher());
    run_test!("Node subscriber", test_node_subscriber());
    run_test!("Node serialize", test_node_serialize());

    hprintln!("");
    hprintln!("----------------------------------------");
    hprintln!("  Results: {} passed, {} failed", passed, failed);
    hprintln!("----------------------------------------");
    hprintln!("");

    if failed == 0 {
        hprintln!("All tests passed!");
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);
    } else {
        hprintln!("Some tests failed!");
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_FAILURE);
    }

    loop {
        cortex_m::asm::wfi();
    }
}
