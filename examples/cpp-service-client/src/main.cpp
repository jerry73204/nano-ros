/// @file main.cpp
/// @brief C++ service client example for nano-ros
///
/// This example creates a service client that calls AddTwoInts service.
/// Run the zenoh router first, then the service server, then this client.
///
/// @note SERVICE CLIENT IS NOT YET SUPPORTED
/// The underlying transport (zenoh-pico shim) does not yet implement service
/// client support. This example compiles but will fail at runtime with
/// "ServiceClientCreationFailed". Service SERVER is fully supported.
/// This limitation applies to both Rust and C++ bindings.

#include <iostream>

#include <cpp_service_client/srv/add_two_ints.hpp>
#include <nano_ros/nano_ros.hpp>

using AddTwoInts = cpp_service_client::srv::AddTwoInts;

int main() {
    std::cout << "Starting AddTwoInts service client..." << std::endl;

    // Create context from environment variables
    auto ctx = nano_ros::Context::from_env();

    // Create node
    auto node = ctx->create_node("add_two_ints_client");

    // Create service client
    auto client = node->create_client<AddTwoInts>("/add_two_ints");
    std::cout << "Service client created: " << client->get_service_name() << std::endl;

    // Make a few test calls
    for (int i = 0; i < 5; ++i) {
        AddTwoInts::Request request;
        request.a = i;
        request.b = i * 2;

        std::cout << "Calling service: a=" << request.a << ", b=" << request.b << std::endl;

        try {
            auto response = client->call(request);
            std::cout << "Response: sum=" << response.sum << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Service call failed: " << e.what() << std::endl;
        }
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
