/// @file main.cpp
/// @brief C++ service server example for nano-ros
///
/// This example creates a service server that handles AddTwoInts requests.
/// Run the zenoh router first, then this server, then the client.

#include <chrono>
#include <iostream>
#include <thread>

#include <cpp_service_server/srv/add_two_ints.hpp>
#include <nano_ros/nano_ros.hpp>

using AddTwoInts = cpp_service_server::srv::AddTwoInts;

int main() {
    std::cout << "Starting AddTwoInts service server..." << std::endl;

    // Create context from environment variables
    auto ctx = nano_ros::Context::from_env();

    // Create node
    auto node = ctx->create_node("add_two_ints_server");

    // Create service server
    auto server = node->create_service<AddTwoInts>("/add_two_ints");
    std::cout << "Service server created: " << server->get_service_name() << std::endl;

    // Main loop - poll for requests
    while (true) {
        auto request = server->try_recv_request();
        if (request) {
            std::cout << "Received request: a=" << request->a << ", b=" << request->b << std::endl;

            // Compute response
            AddTwoInts::Response response;
            response.sum = request->a + request->b;

            std::cout << "Sending response: sum=" << response.sum << std::endl;
            server->send_reply(response);
        }

        // Sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
