/// @file main.c
/// @brief C listener example - subscribes to std_msgs/String messages

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

// nros modular includes (rclc-style)
#include <nros/app_main.h>
#include <nros/check.h>
#include <nros/executor.h>
#include <nros/init.h>
#include <nros/node.h>
#include <nros/subscription.h>

// Generated message bindings
#include "std_msgs.h"

// ----------------------------------------------------------------------------
// Application state
// ----------------------------------------------------------------------------

typedef struct {
    int message_count;
} listener_context_t;

// Static allocation — all nros structs live in .bss, not on the stack
static struct {
    nros_support_t support;
    nros_node_t node;
    listener_context_t listener_ctx;
    nros_subscription_t subscription;
    nros_executor_t executor;
} app;

static volatile sig_atomic_t g_running = 1;
static nros_executor_t* g_executor = NULL;

// ----------------------------------------------------------------------------
// Signal handler for graceful shutdown
// ----------------------------------------------------------------------------

static void signal_handler(int signum) {
    (void)signum;
    g_running = 0;
    if (g_executor) {
        nros_executor_cancel(g_executor);
    }
}

// ----------------------------------------------------------------------------
// Subscription callback - process received message
// ----------------------------------------------------------------------------

static void subscription_callback(const uint8_t* data, size_t len, void* context) {
    listener_context_t* ctx = (listener_context_t*)context;

    std_msgs_msg_string msg;
    std_msgs_msg_string_init(&msg);

    if (std_msgs_msg_string_deserialize(&msg, data, len) == 0) {
        ctx->message_count++;
        printf("I heard: [%s]\n", msg.data);
    } else {
        fprintf(stderr, "Failed to deserialize message (len=%zu)\n", len);
    }
}

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    // Line-buffer stdout so each printf flushes on its newline. When stdout is
    // a pipe (e.g. a test harness capturing output) glibc defaults to 4 KiB
    // block buffering, so "Received: N" lines would sit unflushed for a long
    // time and an observer waiting on them sees nothing. Line buffering makes
    // the output appear live, matching interactive (tty) behaviour.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    printf("nros C Listener\n");
    printf("===================\n");

    // Get configuration from environment
    const char* locator = getenv("NROS_LOCATOR");
    if (!locator) {
        locator = NROS_ENTRY_LOCATOR;
    }

    const char* domain_str = getenv("ROS_DOMAIN_ID");
    uint8_t domain_id = (uint8_t)NROS_ENTRY_DOMAIN_ID;
    if (domain_str) {
        domain_id = (uint8_t)atoi(domain_str);
    }

    printf("Locator: %s\n", locator);
    printf("Domain ID: %d\n", domain_id);

    // Zero-initialize all static state (avoids return-by-value temporaries on stack)
    memset(&app, 0, sizeof(app));

    // Initialize support context
    NROS_CHECK_RET(nros_support_init(&app.support, locator, domain_id), 1);
    printf("Support initialized\n");
    NROS_CHECK_RET(rclc_node_init_default(&app.node, "listener", "/", &app.support), 1);
    printf("Node created: %s\n", rcl_node_get_name(&app.node));

    // Create application context
    app.listener_ctx = (listener_context_t){
        .message_count = 0,
    };

    NROS_CHECK_RET(rclc_subscription_init_default(&app.subscription, &app.node,
                                                  std_msgs_msg_string_get_type_support(),
                                                  "/chatter"),
                   1);
    printf("Subscriber created for topic: %s\n",
           rcl_subscription_get_topic_name(&app.subscription));

    NROS_CHECK_RET(nros_executor_init(&app.executor, &app.support, 4), 1);
    g_executor = &app.executor;
    /* phase-417 stage 6 — rclc arity: the byte callback is supplied at
     * REGISTRATION, not at `*_init`. */
    NROS_CHECK_RET(nros_executor_add_subscription_raw(&app.executor, &app.subscription,
                                                      subscription_callback, &app.listener_ctx,
                                                      NROS_EXECUTOR_ON_NEW_DATA),
                   1);
    printf("Executor created with %d handle(s)\n", nros_executor_get_handle_count(&app.executor));

    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\nWaiting for messages (Ctrl+C to exit)...\n\n");

    // Spin with 100ms period
    nros_ret_t ret = rclc_executor_spin_period(&app.executor, 100000000ULL);
    if (ret != NROS_RET_OK && g_running) {
        fprintf(stderr, "Executor spin failed: %d\n", ret);
    }

    // Cleanup
    printf("\nShutting down...\n");
    printf("Total messages received: %d\n", app.listener_ctx.message_count);
    rclc_executor_fini(&app.executor);
    nros_subscription_fini(&app.subscription);
    rcl_node_fini(&app.node);
    rclc_support_fini(&app.support);

    printf("Goodbye!\n");
    return 0;
}

NROS_APP_MAIN_REGISTER()
