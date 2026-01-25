/// @file hello.cpp
/// @brief Simple example demonstrating nano-ros C++ API

#include <iostream>
#include "nano_ros/nano_ros.hpp"

int main() {
    // Print version info
    auto version = nano_ros::get_version();
    std::cout << "nano-ros version: " << version.to_string() << std::endl;
    std::cout << "Zenoh support: " << (nano_ros::has_zenoh_support() ? "yes" : "no") << std::endl;

    try {
        // Create context from environment
        std::cout << "\nCreating context..." << std::endl;
        auto ctx = nano_ros::Context::from_env();
        std::cout << "Context created, domain_id: " << ctx->domain_id() << std::endl;

        // Create a node
        std::cout << "\nCreating node..." << std::endl;
        auto node = ctx->create_node("hello_node", "/demo");

        std::cout << "Node created:" << std::endl;
        std::cout << "  Name: " << node->get_name() << std::endl;
        std::cout << "  Namespace: " << node->get_namespace() << std::endl;
        std::cout << "  Fully qualified: " << node->get_fully_qualified_name() << std::endl;

        std::cout << "\nSuccess!" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
