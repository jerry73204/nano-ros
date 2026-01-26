/// @file hello.cpp
/// @brief Simple example demonstrating nano-ros C++ API

#include "nano_ros/nano_ros.hpp"

#include <iostream>

int main() {
    // Print version info
    auto version = nano_ros::get_version();
    std::cout << "nano-ros version: " << version.to_string() << std::endl;
    std::cout << "Zenoh support: " << (nano_ros::has_zenoh_support() ? "yes" : "no") << std::endl;

    // Demonstrate Duration and Time
    std::cout << "\n=== Duration and Time ===" << std::endl;
    auto d1 = nano_ros::Duration::from_seconds(1.5);
    auto d2 = nano_ros::Duration::from_milliseconds(500);
    std::cout << "Duration 1: " << d1.seconds() << "s" << std::endl;
    std::cout << "Duration 2: " << d2.seconds() << "s" << std::endl;
    std::cout << "Sum: " << (d1 + d2).seconds() << "s" << std::endl;

    // Demonstrate Clock
    std::cout << "\n=== Clock ===" << std::endl;
    auto system_clock = nano_ros::Clock::create_system();
    auto steady_clock = nano_ros::Clock::create_steady();
    std::cout << "System clock now: " << system_clock->now().seconds() << "s" << std::endl;
    std::cout << "Steady clock now: " << steady_clock->now().seconds() << "s" << std::endl;

    // Demonstrate QoS
    std::cout << "\n=== QoS ===" << std::endl;
    auto qos = nano_ros::QoS(10).reliable().transient_local();
    std::cout << "QoS depth: " << qos.get_depth() << std::endl;
    std::cout << "QoS reliable: "
              << (qos.get_reliability() == nano_ros::ReliabilityPolicy::Reliable ? "yes" : "no")
              << std::endl;

    // Demonstrate NodeOptions
    std::cout << "\n=== NodeOptions ===" << std::endl;
    auto options = nano_ros::NodeOptions("my_node").namespace_("/robot");
    std::cout << "Node name: " << options.get_node_name() << std::endl;
    std::cout << "Namespace: " << options.get_namespace() << std::endl;

    try {
        // Create context from environment
        std::cout << "\n=== Context and Node ===" << std::endl;
        std::cout << "Creating context..." << std::endl;
        auto ctx = nano_ros::Context::from_env();
        std::cout << "Context created, domain_id: " << ctx->domain_id() << std::endl;

        // Create a node using NodeOptions
        std::cout << "Creating node with options..." << std::endl;
        auto node = ctx->create_node(options);

        std::cout << "Node created:" << std::endl;
        std::cout << "  Name: " << node->get_name() << std::endl;
        std::cout << "  Namespace: " << node->get_namespace() << std::endl;
        std::cout << "  Fully qualified: " << node->get_fully_qualified_name() << std::endl;
        std::cout << "  Node time: " << node->now().seconds() << "s" << std::endl;

        // Demonstrate Publisher and Subscription (raw bytes)
        std::cout << "\n=== Publisher and Subscription ===" << std::endl;

        // Create a raw publisher
        auto publisher = node->create_publisher("/chatter", nano_ros::QoS(10).best_effort());
        std::cout << "Publisher created for topic: " << publisher->get_topic_name() << std::endl;

        // Create a raw subscription
        auto subscription = node->create_subscription("/chatter", nano_ros::QoS(10).best_effort());
        std::cout << "Subscription created for topic: " << subscription->get_topic_name()
                  << std::endl;

        // Publish raw CDR data (Int32 with value 42)
        // CDR format: 4-byte header + 4-byte little-endian int32
        std::vector<uint8_t> cdr_data = {
            0x00, 0x01, 0x00, 0x00,  // CDR header (little-endian)
            0x2A, 0x00, 0x00, 0x00   // int32 value 42 (little-endian)
        };
        std::cout << "Publishing raw CDR data (Int32 = 42)" << std::endl;
        publisher->publish_raw(cdr_data);

        std::cout << "\nSuccess!" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
