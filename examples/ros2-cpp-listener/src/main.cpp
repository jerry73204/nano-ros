/// @file main.cpp
/// @brief C++ listener example - demonstrates executor usage with polling subscriptions

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>

// Global executor pointer for signal handler
std::atomic<nano_ros::SingleThreadedExecutor*> g_executor{nullptr};

void signal_handler(int /* signum */) {
    auto* exec = g_executor.load();
    if (exec) {
        exec->cancel();
    }
}

int main() {
    std::cout << "nano-ros C++ Listener (with Executor)" << std::endl;
    std::cout << "======================================" << std::endl;

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

        // Create executor and add node
        nano_ros::SingleThreadedExecutor executor;
        executor.add_node(node);
        std::cout << "Executor created with " << executor.node_count() << " node(s)" << std::endl;

        // Set up signal handler for graceful shutdown
        g_executor.store(&executor);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        std::cout << "\nWaiting for messages (Ctrl+C to exit)..." << std::endl;

        // Poll for messages in a thread while executor spins
        std::thread poller([&subscription, &executor]() {
            while (executor.is_spinning()) {
                if (auto msg = subscription->take()) {
                    std::cout << "Received: " << msg->data << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        // Spin the executor (blocks until cancel() is called)
        executor.spin();

        std::cout << "\nShutting down..." << std::endl;
        poller.join();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Goodbye!" << std::endl;
    return 0;
}
