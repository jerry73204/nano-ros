/// @file main.cpp
/// @brief C++ service server example - handles AddTwoInts requests (manual-poll)

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for example_interfaces/srv/AddTwoInts
#include "example_interfaces.hpp"

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
    printf("nros C++ Service Server (AddTwoInts)\n");
    printf("=====================================\n");

    // Launch-aware init. Env overlay is the active source today
    // (`$NROS_LOCATOR` / `$ROS_DOMAIN_ID`).
    NROS_TRY_RET(nros::init_with_launch_auto(argc, argv), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "add_two_ints_server"), 1);
    printf("Node created: %s\n", node.get_name());

    rclcpp::Service<example_interfaces::srv::AddTwoInts> srv;
    NROS_TRY_RET(node.create_service(srv, "/add_two_ints"), 1);
    rclcpp::Result ret;

    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\nWaiting for service requests (Ctrl+C to exit)...\n\n");

    int request_count = 0;

    // Spin + poll loop
    while (g_running && rclcpp::ok()) {
        nros::spin_once(100);

        example_interfaces::srv::AddTwoInts::Request req;
        int64_t seq_id = 0;
        while (srv.take_request(req, seq_id)) {
            request_count++;

            example_interfaces::srv::AddTwoInts::Response resp;
            resp.sum = req.a + req.b;

            printf("Incoming request\na: %lld b: %lld\n", static_cast<long long>(req.a),
                   static_cast<long long>(req.b));

            ret = srv.send_response(seq_id, resp);
            if (!ret.ok()) {
                fprintf(stderr, "Failed to send reply: %d\n", ret.raw());
            }
        }
    }

    // Cleanup
    printf("\nShutting down...\n");
    printf("Total requests handled: %d\n", request_count);
    rclcpp::shutdown();

    printf("Goodbye!\n");
    return 0;
}

NROS_APP_MAIN_REGISTER()
