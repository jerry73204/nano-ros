/// @file main.cpp
/// @brief C++ talker example - publishes std_msgs/String "Hello World: N" at 1 Hz

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// Route NROS_TRY_RET through fprintf (we have stdio).
#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for std_msgs/msg/String
#include "std_msgs.hpp"

// ----------------------------------------------------------------------------
// Application state
// ----------------------------------------------------------------------------

struct TalkerContext {
    rclcpp::Publisher<std_msgs::msg::String>* publisher;
    int count;
};

static volatile sig_atomic_t g_running = 1;

// ----------------------------------------------------------------------------
// Signal handler for graceful shutdown
// ----------------------------------------------------------------------------

static void signal_handler(int signum) {
    (void)signum;
    g_running = 0;
}

// ----------------------------------------------------------------------------
// Timer callback - publish a message
// ----------------------------------------------------------------------------

static void timer_callback(void* context) {
    TalkerContext* ctx = static_cast<TalkerContext*>(context);

    // Pre-increment so the first payload is "Hello World: 1", matching the
    // official ROS 2 demo talker (demo_nodes_cpp `talker.cpp`).
    ctx->count++;
    char payload[64];
    snprintf(payload, sizeof(payload), "Hello World: %d", ctx->count);
    std_msgs::msg::String msg;
    msg.data = payload;

    rclcpp::Result ret = ctx->publisher->publish(msg);
    if (ret.ok()) {
        printf("Publishing: '%s'\n", msg.data.c_str());
    } else {
        fprintf(stderr, "Publish failed: %d\n", ret.raw());
    }
}

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int nros_app_main(int argc, char** argv) {
    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, nullptr, _IOLBF, 0);
#endif
    (void)argc;
    (void)argv;

    printf("nros C++ Talker\n");
    printf("===================\n");

    // Phase 212.M.2 — `nros::init()` (no-arg) pulls locator + domain_id
    // from `$NROS_LOCATOR` / `$ROS_DOMAIN_ID` at runtime. Falling back
    // defaults match the prior hand-rolled env reads.
    NROS_TRY_RET(nros::init(), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "talker"), 1);
    printf("Node created: %s\n", node.get_name());

    rclcpp::Publisher<std_msgs::msg::String> pub;
    NROS_TRY_RET(node.create_publisher(pub, "/chatter"), 1);

    TalkerContext ctx;
    ctx.publisher = &pub;
    ctx.count = 0;

    nros::Timer timer;
    NROS_TRY_RET(node.create_wall_timer(timer, 1000, timer_callback, &ctx), 1);

    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\nPublishing messages (Ctrl+C to exit)...\n\n");

    // Spin
    while (g_running && rclcpp::ok()) {
        nros::spin_once(100);
    }

    // Cleanup
    printf("\nShutting down...\n");
    rclcpp::shutdown();

    printf("Goodbye!\n");
    return 0;
}

NROS_APP_MAIN_REGISTER()
