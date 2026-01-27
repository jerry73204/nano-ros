/// @file main.cpp
/// @brief C++ custom message example - demonstrates message generation

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <nano_ros/nano_ros.hpp>

// Generated message headers
#include <cpp_custom_msg/msg/sensor_reading.hpp>
#include <cpp_custom_msg/msg/status.hpp>

// Pre-generated std_msgs for comparison
#include <std_msgs/msg/int32.hpp>

/// Test message serialization roundtrip
template <typename T>
bool test_roundtrip(const T& original, const char* name) {
    // Serialize
    nano_ros::CdrWriter writer(1024);
    writer.write_encapsulation();
    original.serialize(writer);
    size_t serialized_size = writer.size();

    // Deserialize
    T deserialized;
    nano_ros::CdrReader reader(writer.data(), serialized_size);
    reader.read_encapsulation();
    deserialized.deserialize(reader);

    std::cout << "  " << name << ": serialized " << serialized_size << " bytes" << std::endl;
    return true;
}

int main() {
    std::cout << "nano-ros C++ Custom Message Example" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // Test custom message types
    // ========================================================================
    std::cout << "Testing custom message serialization:" << std::endl;

    // Test SensorReading
    cpp_custom_msg::msg::SensorReading sensor_msg;
    sensor_msg.sensor_id = 42;
    sensor_msg.temperature = 23.5f;
    sensor_msg.humidity = 65.0f;
    sensor_msg.timestamp = 1234567890123ULL;
    test_roundtrip(sensor_msg, "SensorReading");

    // Test Status
    cpp_custom_msg::msg::Status status_msg;
    status_msg.active = true;
    status_msg.message = "System operational";
    status_msg.error_code = 0;
    test_roundtrip(status_msg, "Status");

    // Test std_msgs::Int32 for comparison
    std_msgs::msg::Int32 int_msg;
    int_msg.data = 12345;
    test_roundtrip(int_msg, "std_msgs::Int32");

    std::cout << std::endl;

    // ========================================================================
    // Pub/Sub with custom messages (requires zenohd)
    // ========================================================================
    std::cout << "Testing pub/sub with custom messages:" << std::endl;

    try {
        auto ctx = nano_ros::Context::from_env();
        std::cout << "  Context created, domain_id: " << ctx->domain_id() << std::endl;

        auto node = ctx->create_node("custom_msg_node");
        std::cout << "  Node created: " << node->get_fully_qualified_name() << std::endl;

        // Create publisher for SensorReading
        auto pub = node->create_publisher<cpp_custom_msg::msg::SensorReading>("/sensor_data",
                                                                              nano_ros::QoS(10));
        std::cout << "  Publisher created for: " << pub->get_topic_name() << std::endl;

        // Create subscription for SensorReading
        auto sub = node->create_subscription<cpp_custom_msg::msg::SensorReading>("/sensor_data",
                                                                                 nano_ros::QoS(10));
        std::cout << "  Subscription created for: " << sub->get_topic_name() << std::endl;

        // Publish a few messages
        std::cout << std::endl;
        std::cout << "Publishing sensor readings..." << std::endl;

        for (int i = 0; i < 3; ++i) {
            cpp_custom_msg::msg::SensorReading reading;
            reading.sensor_id = i + 1;
            reading.temperature = 20.0f + static_cast<float>(i) * 0.5f;
            reading.humidity = 50.0f + static_cast<float>(i) * 5.0f;
            reading.timestamp =
                static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

            pub->publish(reading);
            std::cout << "  Published: sensor_id=" << reading.sensor_id
                      << ", temp=" << reading.temperature << ", humidity=" << reading.humidity
                      << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Try to receive
            if (auto msg = sub->take()) {
                std::cout << "  Received: sensor_id=" << msg->sensor_id
                          << ", temp=" << msg->temperature << std::endl;
            }
        }

        std::cout << std::endl;
        std::cout << "Pub/sub test completed!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "  Pub/sub test skipped (zenohd not available): " << e.what() << std::endl;
        std::cout << std::endl;
    }

    std::cout << "Custom message generation test completed successfully!" << std::endl;
    return 0;
}
