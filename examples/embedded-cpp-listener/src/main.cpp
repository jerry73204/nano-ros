/// @file main.cpp
/// @brief Embedded C++ listener example for nano-ros
///
/// This example demonstrates the embedded-compatible subscriber API that works
/// without std library (no heap allocation in C++, no exceptions).
///
/// Build for desktop testing:
///   mkdir build && cd build
///   cmake .. && make
///   ./cpp_embedded_listener
///
/// Build for embedded:
///   mkdir build && cd build
///   cmake .. -DBUILD_FOR_EMBEDDED=ON
///   make

#if defined(NANO_ROS_EMBEDDED)
// Embedded mode: use embedded headers
#include <nano_ros/embedded/node.hpp>
#include <std_msgs/msg/int32.hpp>  // Generated with --embedded flag
#else
// Desktop mode: use standard headers
#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#endif

#if defined(NANO_ROS_EMBEDDED)

// ============================================================================
// Embedded implementation (no std library)
// ============================================================================

// Static storage for context and node
alignas(nano_ros::EmbeddedContext) static uint8_t g_ctx_storage[sizeof(nano_ros::EmbeddedContext)];
alignas(nano_ros::EmbeddedNode) static uint8_t g_node_storage[sizeof(nano_ros::EmbeddedNode)];

// Buffer for deserialization
static uint8_t g_sub_buffer[256];

// Platform-specific functions (implement for your platform)
extern "C" {
void platform_sleep_ms(uint32_t ms);
uint32_t platform_get_time_ms();
void platform_print(const char* msg);
void platform_print_int(const char* prefix, int32_t value);
}

int main() {
    platform_print("nano-ros Embedded C++ Listener Example\n");

    // Initialize context
    auto err = nano_ros::EmbeddedContext::init(g_ctx_storage, sizeof(g_ctx_storage),
                                                "tcp/127.0.0.1:7447");
    if (err != nano_ros::ErrorCode::Ok) {
        platform_print("Failed to initialize context\n");
        return 1;
    }

    auto* ctx = reinterpret_cast<nano_ros::EmbeddedContext*>(g_ctx_storage);
    platform_print("Context initialized\n");

    // Create node
    err = nano_ros::EmbeddedNode::create(g_node_storage, sizeof(g_node_storage), ctx,
                                         "embedded_listener", "");
    if (err != nano_ros::ErrorCode::Ok) {
        platform_print("Failed to create node\n");
        ctx->close();
        return 1;
    }

    auto* node = reinterpret_cast<nano_ros::EmbeddedNode*>(g_node_storage);
    platform_print("Node created\n");

    // Create subscriber
    nano_ros::QoS qos(10);
    auto sub_handle = node->create_subscriber("/chatter", "std_msgs/msg/Int32", qos);
    if (sub_handle < 0) {
        platform_print("Failed to create subscriber\n");
        ctx->close();
        return 1;
    }

    platform_print("Subscriber created, waiting for messages...\n");

    // Create typed subscriber wrapper
    nano_ros::EmbeddedTypedSubscriber<std_msgs::msg::Int32, 256> sub(node, sub_handle, g_sub_buffer);

    // Receive loop
    while (true) {
        // Poll transport
        ctx->poll(10);

        // Try to receive message
        std_msgs::msg::Int32 msg;
        if (sub.try_receive(msg)) {
            platform_print_int("Received: ", msg.data);
        }

        // Small sleep to avoid busy-waiting
        platform_sleep_ms(10);
    }

    ctx->close();
    return 0;
}

// Default platform implementations for desktop testing
// (These would be replaced by actual platform implementations on embedded)
#if !defined(PLATFORM_ZEPHYR) && !defined(PLATFORM_NUTTX)
#include <stdio.h>
#include <unistd.h>

void platform_sleep_ms(uint32_t ms) {
    usleep(ms * 1000);
}

uint32_t platform_get_time_ms() {
    // Simple implementation - not accurate
    static uint32_t time = 0;
    return time++;
}

void platform_print(const char* msg) {
    printf("%s", msg);
    fflush(stdout);
}

void platform_print_int(const char* prefix, int32_t value) {
    printf("%s%d\n", prefix, value);
    fflush(stdout);
}
#endif

#else

// ============================================================================
// Desktop implementation (with std library)
// ============================================================================

int main() {
    std::cout << "nano-ros C++ Listener Example (Desktop Mode)\n";

    // Initialize context
    auto ctx = nano_ros::Context::from_env();
    if (!ctx) {
        std::cerr << "Failed to create context\n";
        return 1;
    }
    std::cout << "Context initialized on domain " << ctx->domain_id() << "\n";

    // Create node
    auto node = ctx->create_node("cpp_listener", "");
    std::cout << "Node '" << node->get_name() << "' created\n";

    // Create subscriber with callback
    auto sub = node->create_subscription<std_msgs::msg::Int32>(
        "/chatter",
        nano_ros::QoS(10),
        [](const std_msgs::msg::Int32& msg) {
            std::cout << "Received: " << msg.data << "\n";
        });
    std::cout << "Subscriber on '/chatter' created, waiting for messages...\n";

    // Spin loop - process callbacks
    while (true) {
        sub->spin_once();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}

#endif  // NANO_ROS_EMBEDDED
