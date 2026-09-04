/// @file main.cpp
/// @brief C++ action server example - Fibonacci (callback-based)

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for example_interfaces/action/Fibonacci
#include "example_interfaces.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

// ----------------------------------------------------------------------------
// Application state
// ----------------------------------------------------------------------------

static volatile sig_atomic_t g_running = 1;

/// State that the goal callback needs to reach — held on the stack of
/// `main` and handed to the ActionServer via the
/// `set_goal_callback_with_ctx` overload. The older `set_goal_callback`
/// path requires a stateless function pointer and forces file-scope
/// globals; `_with_ctx` lets us pass a `void*` through each invocation
/// so the callback reaches the server and counter without any globals.
struct ServerState {
    rclcpp_action::Server<Fibonacci>* srv;
    int goal_count;
};

// ----------------------------------------------------------------------------
// Signal handler for graceful shutdown
// ----------------------------------------------------------------------------

static void signal_handler(int signum) {
    (void)signum;
    g_running = 0;
}

// ----------------------------------------------------------------------------
// Goal callback — runs the Fibonacci computation inline, publishing
// feedback and completing the goal before returning. Reaches the
// ActionServer + counter through the `void* ctx` parameter.
// ----------------------------------------------------------------------------

static nros::GoalResponse on_goal(const uint8_t uuid[16], const Fibonacci::Goal& goal, void* ctx) {
    auto* state = static_cast<ServerState*>(ctx);
    printf("Received goal request with order %d\n", goal.order);
    if (goal.order < 0 || goal.order >= 64) {
        printf("Goal rejected: order out of range\n");
        return nros::GoalResponse::Reject;
    }

    state->goal_count++;
    printf("Executing goal\n");

    int32_t a = 0;
    int32_t b = 1;
    Fibonacci::Result result;

    // issue 0453 — `i <= goal.order`, so an order-N goal yields N+1 elements.
    // That is the ROS 2 `action_tutorials` convention, which the Rust and C
    // servers already follow (`0..=order` / `i <= order`); this loop was the
    // one `i < order` and produced 10 elements for order 10 while its siblings
    // produced 11. One convention across the three languages is what lets a
    // single expected sequence assert delivery for EVERY cell.
    for (int32_t i = 0; i <= goal.order && i < 64; i++) {
        result.sequence.push_back(a);

        // Publish feedback periodically
        if (i > 0 && (i % 3 == 0 || i == goal.order)) {
            Fibonacci::Feedback fb;
            for (uint32_t k = 0; k < result.sequence.length(); k++) {
                fb.sequence.push_back(result.sequence[k]);
            }
            state->srv->publish_feedback(uuid, fb);
            printf("Publish feedback\n");
        }

        int32_t next = a + b;
        a = b;
        b = next;
    }

    if (state->srv->complete_goal(uuid, result).ok()) {
        printf("Goal succeeded\n");
    }
    return nros::GoalResponse::AcceptAndExecute;
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
    printf("nros C++ Action Server (Fibonacci)\n");
    printf("===================================\n");

    // Launch-aware init. Env overlay active today.
    NROS_TRY_RET(nros::init_with_launch_auto(argc, argv), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "fibonacci_action_server"), 1);
    printf("Node created: %s\n", node.get_name());

    rclcpp_action::Server<Fibonacci> srv;
    NROS_TRY_RET(node.create_action_server(srv, "/fibonacci"), 1);

    // Register the goal callback with a ServerState context — no globals
    // needed.
    ServerState state{&srv, 0};
    srv.set_goal_callback_with_ctx(on_goal, &state);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\nWaiting for action goals (Ctrl+C to exit)...\n\n");

    while (g_running && rclcpp::ok()) {
        nros::spin_once(100);
    }

    printf("\nShutting down...\n");
    printf("Total goals handled: %d\n", state.goal_count);
    rclcpp::shutdown();

    printf("Goodbye!\n");
    return 0;
}

NROS_APP_MAIN_REGISTER()
