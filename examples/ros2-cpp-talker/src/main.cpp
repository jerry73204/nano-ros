/// @file main.cpp
/// @brief C++ talker example - publishes std_msgs::Int32 messages

#include <chrono>
#include <iostream>
#include <thread>

#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>

int main() {
    std::cout << "nano-ros C++ Talker" << std::endl;
    std::cout << "===================" << std::endl;

    try {
        // Create context from environment
        auto ctx = nano_ros::Context::from_env();
        std::cout << "Context created, domain_id: " << ctx->domain_id() << std::endl;

        // Create node
        auto node = ctx->create_node("cpp_talker");
        std::cout << "Node created: " << node->get_fully_qualified_name() << std::endl;

        // Create typed publisher for std_msgs::Int32
        auto publisher =
            node->create_publisher<std_msgs::msg::Int32>("/chatter", nano_ros::QoS(10));
        std::cout << "Publisher created for topic: " << publisher->get_topic_name() << std::endl;

        // Publish messages in a loop
        std_msgs::msg::Int32 msg;
        msg.data = 0;

        std::cout << "\nPublishing messages..." << std::endl;

        while (true) {
            msg.data++;
            std::cout << "Publishing: " << msg.data << std::endl;
            publisher->publish(msg);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
