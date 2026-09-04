/// @file main.cpp
/// @brief C++ E2E-safety listener — polls `Subscription::take_validated` and
///        prints the per-message integrity status (CRC + sequence gap/dup).
///
/// Issue 0073 / phase-259 W3 — the C++ analog of the C safety-listener. Pairs
/// with the safety talker (the Rust example built `--features safety-e2e`):
/// receives `std_msgs/Int32` on `/chatter` over zenohd and validates the CRC the
/// publisher attached. Built with `NANO_ROS_SAFETY_E2E=ON` (set in CMakeLists) so
/// the zenoh backend carries the CRC path. The C++ ABI calls the same
/// `RmwSubscriber::take_validated` the C path does.

#include <cstdint>
#include <stdio.h>
#include <stdlib.h>

#define NROS_TRY_LOG(file, line, expr, ret)                                                        \
    fprintf(stderr, "[nros] %s:%d %s -> %d\n", (file), (line), (expr), (int)(ret))

#include <nros/app_main.h>
#include <nros/nros.hpp>

// Generated C++ bindings for std_msgs/msg/Int32
#include "std_msgs.hpp"

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, nullptr, _IOLBF, 0);
#endif

    printf("nros C++ Safety Listener\n");

    // Phase 212.M.2 — `nros::init()` pulls locator + domain_id from
    // `$NROS_LOCATOR` / `$ROS_DOMAIN_ID` at runtime.
    NROS_TRY_RET(nros::init(), 1);

    nros::Node node;
    NROS_TRY_RET(nros::create_node(node, "cpp_safety_listener"), 1);

    // Poll-mode subscription — the validated receive path is poll-only.
    rclcpp::Subscription<std_msgs::msg::Int32> sub;
    NROS_TRY_RET(node.create_subscription(sub, "/chatter"), 1);
    printf("Waiting for Int32 messages on /chatter...\n");

    int count = 0;
    while (rclcpp::ok()) {
        nros::spin_once(100);

        std_msgs::msg::Int32 msg;
        nros_cpp_integrity_status_t status;
        // Result is true while a sample was received+deserialized (TryAgain → false).
        while (sub.take_validated(msg, status)) {
            count++;
            const char* crc = status.crc_valid == 1 ? "ok" : status.crc_valid == 0 ? "FAIL" : "n-a";
            printf("[%d] Received: data=%d [SAFETY] INTEGRITY gap=%lld dup=%d crc=%s\n", count,
                   msg.data, (long long)status.gap, (int)status.duplicate, crc);
        }
    }

    rclcpp::shutdown();
    return 0;
}

NROS_APP_MAIN_REGISTER()
