/// @file main.cpp
/// @brief C++ listener example - subscribes to std_msgs::Int32 messages

#include <chrono>
#include <iostream>
#include <thread>

#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>

int main() {
    std::cout << "nano-ros C++ Listener" << std::endl;
    std::cout << "=====================" << std::endl;

    try {
        // Create context from environment
        auto ctx = nano_ros::Context::from_env();
        std::cout << "Context created, domain_id: " << ctx->domain_id() << std::endl;

        // Create node
        auto node = ctx->create_node("cpp_listener");
        std::cout << "Node created: " << node->get_fully_qualified_name() << std::endl;

        // Create typed subscription for std_msgs::Int32
        auto subscription =
            node->create_subscription<std_msgs::msg::Int32>("/chatter", nano_ros::QoS(10));
        std::cout << "Subscription created for topic: " << subscription->get_topic_name()
                  << std::endl;

        std::cout << "\nWaiting for messages..." << std::endl;

        // Poll for messages in a loop
        while (true) {
            if (auto msg = subscription->take()) {
                std::cout << "Received: " << msg->data << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
