/// @file main.cpp
/// @brief C++ action client example - Fibonacci (blocking)

#include <stdio.h>
// <stdlib.h> (not <cstdlib>): newlib on the embedded cross toolchains does
// not inject strtoll/getenv into namespace std — the global C spellings are
// the portable ones (this source builds native AND on the RTOS boards).
#include <stdlib.h>

#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for example_interfaces/action/Fibonacci
#include "example_interfaces.hpp"

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int nros_app_main(int argc, char** argv) {
    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, nullptr, _IOLBF, 0);
#endif
    printf("nros C++ Action Client (Fibonacci)\n");
    printf("===================================\n");

    // Launch-aware init. Env overlay active today.
    NROS_TRY_RET(nros::init_with_launch_auto(argc, argv), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "fibonacci_action_client"), 1);
    printf("Node created: %s\n", node.get_name());

    rclcpp_action::Client<example_interfaces::action::Fibonacci> client;
    NROS_TRY_RET(node.create_action_client(client, "/fibonacci"), 1);
    rclcpp::Result ret;

    // Default order=10; override via NROS_TEST_GOAL_ORDER for tests that
    // want to exercise server-side rejection (order >= 64) or other edges.
    int32_t order = 10;
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
    // Host-only: env override. Embedded (freestanding C++) has no environment
    // and newlib's freestanding <stdlib.h> declares no getenv/atoi.
    if (const char* ord = getenv("NROS_TEST_GOAL_ORDER")) {
        order = atoi(ord);
    }
#endif
    // phase-338 W8 — wait for the action server, then send ONCE.
    //
    // Issue 0153 / #188 root cause: on zenoh the server's readiness gossips
    // ahead of its send-goal queryable ROUTE, and a query fired in that window
    // matches no queryable and can only time out (-2) — a zenoh get is
    // evaluated against the queryables visible at fire time, so retrying the
    // same query is the only recourse once it has been fired.
    //
    // The NuttX copy used to retry send_goal three times with a ~1 s spin
    // between attempts. `wait_for_action_server` is the primitive that was
    // approximating: it probes the send-goal queryable's liveliness and
    // RE-probes until the budget expires, so a server appearing mid-wait is
    // still seen. Mirrors `rclcpp_action::Client::wait_for_action_server`.
    ret = client.wait_for_action_server(10000);
    if (!ret.ok()) {
        fprintf(stderr, "Action server did not appear within 10s (ret=%d)\n", ret.raw());
        rclcpp::shutdown();
        return 2;
    }

    printf("\nSending goal\n");

    example_interfaces::action::Fibonacci::Goal goal;
    goal.order = order;

    uint8_t goal_id[16];
    ret = client.send_goal(goal, goal_id);
    if (!ret.ok()) {
        // Issue 0868 — say WHICH of the three outcomes this was.
        //
        // This used to print "Goal was rejected by server" for every non-OK
        // code, so a TIMEOUT — the server never answered, and may never have
        // received the goal at all — was reported as a decision the server
        // made. `on_goal` does have a real reject path (order out of range),
        // so the message sent readers to the wrong file; it cost exactly that
        // once, with the real fault in the transport.
        //
        // The three became distinguishable in the same issue: before it,
        // `send_goal` returned ERROR for both "the server said no" and "the
        // goal never left", so no message here could have been accurate.
        if (ret.code() == nros::ErrorCode::Rejected) {
            fprintf(stderr, "Goal was rejected by server (order=%d)\n", order);
        } else if (ret.code() == nros::ErrorCode::Timeout) {
            fprintf(stderr,
                    "No goal response from server (order=%d, ret=%d) — the goal may never "
                    "have been received\n",
                    order, ret.raw());
        } else {
            fprintf(stderr, "Failed to send goal (order=%d, ret=%d)\n", order, ret.raw());
        }
        rclcpp::shutdown();
        return 2;
    }

    printf("Goal accepted by server, waiting for result\n");

    // Poll for feedback while waiting — drain via the Stream<T> API,
    // which aligns the feedback receive surface with
    // Subscription<M>::stream() / Promise<T>::wait(). `try_recv_feedback`
    // below is still supported for callers that want the bool-convertible
    // helper.
    auto& feedback = client.feedback_stream();
    for (int i = 0; i < 20; i++) {
        nros::spin_once(100);

        example_interfaces::action::Fibonacci::Feedback fb;
        while (feedback.try_next(fb).ok()) {
            printf("Next number in sequence received: [");
            for (uint32_t k = 0; k < fb.sequence.length(); k++) {
                if (k > 0) printf(", ");
                printf("%d", fb.sequence[k]);
            }
            printf("]\n");
        }
    }

    // Get result (blocking)
    example_interfaces::action::Fibonacci::Result result;
    ret = client.get_result(goal_id, result);
    if (ret.ok()) {
        printf("Result received: [");
        for (uint32_t i = 0; i < result.sequence.length(); i++) {
            if (i > 0) printf(", ");
            printf("%d", result.sequence[i]);
        }
        printf("]\n");
    } else {
        fprintf(stderr, "Failed to get result: %d\n", ret.raw());
        rclcpp::shutdown();
        return 1;
    }

    // Cleanup
    printf("\nShutting down...\n");
    rclcpp::shutdown();

    printf("Goodbye!\n");
    return 0;
}

NROS_APP_MAIN_REGISTER()
