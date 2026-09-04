/// @file main.cpp
/// @brief C++ listener example - subscribes to std_msgs/String (manual-poll)

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for std_msgs/msg/String
#include "std_msgs.hpp"

// ----------------------------------------------------------------------------
// Application state
// ----------------------------------------------------------------------------

static volatile sig_atomic_t g_running = 1;

// ----------------------------------------------------------------------------
// Signal handler for graceful shutdown
// ----------------------------------------------------------------------------

static void signal_handler(int signum) {
    (void)signum;
    g_running = 0;
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

    printf("nros C++ Listener\n");
    printf("===================\n");

    // Phase 212.M.2 — `nros::init()` (no-arg) pulls locator + domain_id
    // from `$NROS_LOCATOR` / `$ROS_DOMAIN_ID` at runtime.
    NROS_TRY_RET(nros::init(), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "listener"), 1);
    printf("Node created: %s\n", node.get_name());

    rclcpp::Subscription<std_msgs::msg::String> sub;
    NROS_TRY_RET(node.create_subscription(sub, "/chatter"), 1);
    // phase-342 — the READINESS marker the test matrix waits on
    // (`nros_tests::output::LISTENER_READY_MARKER`). The rust and C listeners
    // print it too; one demo should not spell its own readiness three ways.
    printf("Subscriber created for topic: %s\n", "/chatter");

    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\nWaiting for messages (Ctrl+C to exit)...\n\n");

    int message_count = 0;

    // Alternative: use Stream::wait_next for blocking reception
    // std_msgs::msg::String msg;
    // sub.stream().wait_next(nros::global_handle(), 1000, msg);

    // Spin + poll loop
    while (g_running && rclcpp::ok()) {
        nros::spin_once(100);

        std_msgs::msg::String msg;
        while (sub.take(msg)) {
            message_count++;
            printf("I heard: [%s]\n", msg.data.c_str());
        }
    }

    // Cleanup
    printf("\nShutting down...\n");
    printf("Total messages received: %d\n", message_count);
    rclcpp::shutdown();

    printf("Goodbye!\n");
    return 0;
}

NROS_APP_MAIN_REGISTER()
