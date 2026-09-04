/// @file main.c
/// @brief C E2E-safety listener — polls `nros_subscription_take_validated`
///        and prints the per-message integrity status (CRC + sequence gap/dup).
///
/// Issue 0073 — the C analog of the Rust declarative-safety-listener. Pairs with
/// the safety talker (the imperative Rust example built `--features safety-e2e`):
/// receives `std_msgs/Int32` on `/chatter` over zenohd and validates the CRC
/// attached by the publisher. Built with `NANO_ROS_SAFETY_E2E=ON` (set below) so
/// the zenoh backend carries the CRC path.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nros/app_main.h>
#include <nros/check.h>
#include <nros/init.h>
#include <nros/node.h>
#include <nros/subscription.h>

#include "std_msgs.h"

static struct {
    nros_support_t support;
    nros_node_t node;
    nros_subscription_t subscription;
} app;

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    const char* locator = getenv("NROS_LOCATOR");
    if (!locator) {
        locator = NROS_ENTRY_LOCATOR;
    }
    const char* domain_str = getenv("ROS_DOMAIN_ID");
    uint8_t domain_id = domain_str ? (uint8_t)atoi(domain_str) : 0;

    printf("nros C Safety Listener\n");
    memset(&app, 0, sizeof(app));
    NROS_CHECK_RET(nros_support_init(&app.support, locator, domain_id), 1);
    NROS_CHECK_RET(rclc_node_init_default(&app.node, "c_safety_listener", "/", &app.support), 1);

    // Polling-mode subscription — the validated receive path is poll-only.
    NROS_CHECK_RET(nros_subscription_init_polling(&app.subscription, &app.node,
                                                  std_msgs_msg_int32_get_type_support(),
                                                  "/chatter"),
                   1);
    printf("Waiting for Int32 messages on /chatter...\n");

    uint8_t buf[256];
    int count = 0;
    for (;;) {
        nros_integrity_status_t status;
        int32_t n = nros_subscription_take_validated(&app.subscription, buf, sizeof(buf), &status);
        if (n > 0) {
            std_msgs_msg_int32 msg;
            std_msgs_msg_int32_init(&msg);
            if (std_msgs_msg_int32_deserialize(&msg, buf, (size_t)n) == 0) {
                count++;
                const char* crc = status.crc_valid == 1   ? "ok"
                                  : status.crc_valid == 0 ? "FAIL"
                                                          : "n-a";
                printf("[%d] Received: data=%d [SAFETY] INTEGRITY gap=%lld dup=%d crc=%s\n", count,
                       msg.data, (long long)status.gap, (int)status.duplicate, crc);
            }
        } else if (n == 0) {
            usleep(5000);
        } else {
            fprintf(stderr, "recv error: %d\n", n);
            usleep(5000);
        }
    }

    nros_subscription_fini(&app.subscription);
    rcl_node_fini(&app.node);
    rclc_support_fini(&app.support);
    return 0;
}

NROS_APP_MAIN_REGISTER()
