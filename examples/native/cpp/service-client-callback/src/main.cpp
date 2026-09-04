/// @file main.cpp
/// @brief C++ service client example — **callback** variant.
///
/// Mirrors the Future-based `service-client` example, but receives each reply
/// through a typed `void(const Response&)` handler dispatched by `spin_once`
/// (the rclcpp `async_send_request(req, cb)` analogue). Created via the
/// callback overload `node.create_client(client, name, handler)`; requests go
/// out non-blocking with `client.async_send_request(req)`.

#include <stdio.h>
// <stdlib.h> (not <cstdlib>): newlib on the embedded cross toolchains does
// not inject strtoll/getenv into namespace std — the global C spellings are
// the portable ones (this source builds native AND on the RTOS boards).
#include <stdlib.h>

#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for example_interfaces/srv/AddTwoInts
#include "example_interfaces.hpp"

// ----------------------------------------------------------------------------
// Reply state shared with the callback. The callback overload requires a
// non-capturing `void(const Response&)`, so the handler talks to file scope.
// ----------------------------------------------------------------------------

namespace {
int g_reply_count = 0; // bumped each time the callback fires

void on_response(const example_interfaces::srv::AddTwoInts::Response& resp) {
    g_reply_count++;
    printf("Result of add_two_ints: %lld\n", static_cast<long long>(resp.sum));
}
} // namespace

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int nros_app_main(int argc, char** argv) {
    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, nullptr, _IOLBF, 0);
#endif
    printf("nros C++ Service Client (AddTwoInts, callback)\n");
    printf("===============================================\n");

    // Launch-aware init. Env overlay
    // (`$NROS_LOCATOR` / `$ROS_DOMAIN_ID`) active today.
    NROS_TRY_RET(nros::init_with_launch_auto(argc, argv), 1);

    // Operands from the first two positional args (default: 2 3).
    int64_t a = 2;
    int64_t b = 3;
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
    // Host-only: positional-arg override. Embedded (freestanding C++) has no
    // argv and newlib's freestanding <stdlib.h> declares no strtoll.
    if (argc >= 3) {
        char* end_a = nullptr;
        char* end_b = nullptr;
        long long parsed_a = strtoll(argv[1], &end_a, 10);
        long long parsed_b = strtoll(argv[2], &end_b, 10);
        if (end_a && *end_a == '\0' && end_b && *end_b == '\0') {
            a = static_cast<int64_t>(parsed_a);
            b = static_cast<int64_t>(parsed_b);
        }
    }
#endif

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "add_two_ints_client_cb"), 1);
    printf("Node created: %s\n", node.get_name());

    // Callback-style client. The arena dispatches `on_response` at
    // `spin_once`; sends are non-blocking via `async_send_request`.
    rclcpp::Client<example_interfaces::srv::AddTwoInts> client;
    NROS_TRY_RET(node.create_client(client, "/add_two_ints", &on_response), 1);
    printf("Callback service client created for: /add_two_ints\n");

    // Let discovery settle (the callback client has no Future to gate on).
    for (int i = 0; i < 20; i++) {
        nros::spin_once(50);
    }

    example_interfaces::srv::AddTwoInts::Request req;
    req.a = a;
    req.b = b;

    int exit_code = 0;

    rclcpp::Result send = client.async_send_request(req);
    if (!send.ok()) {
        fprintf(stderr, "Async send failed with error %d\n", send.raw());
        exit_code = 1;
    } else {
        // Spin until the reply callback fires (or a 5 s budget elapses).
        int waited_ms = 0;
        while (g_reply_count == 0 && waited_ms < 5000) {
            nros::spin_once(50);
            waited_ms += 50;
        }

        if (g_reply_count == 0) {
            fprintf(stderr, "Timeout waiting for callback\n");
            exit_code = 1;
        }
    }

    printf("\nShutting down...\n");
    rclcpp::shutdown();

    printf("Goodbye!\n");
    return exit_code;
}

NROS_APP_MAIN_REGISTER()
