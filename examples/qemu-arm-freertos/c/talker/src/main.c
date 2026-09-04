/// @file main.c
/// @brief C talker example - publishes std_msgs/String "Hello World: N" using a timer

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
#include <nros/publisher.h>
#include <nros/timer.h>

// Generated message bindings
#include "std_msgs.h"

// ----------------------------------------------------------------------------
// Application state
// ----------------------------------------------------------------------------

typedef struct {
    nros_publisher_t* publisher;
    std_msgs_msg_string message;
    int count;
} talker_context_t;

// Static allocation — all nros structs live in .bss, not on the stack
static struct {
    nros_support_t support;
    nros_node_t node;
    nros_publisher_t publisher;
    talker_context_t talker_ctx;
    nros_timer_t timer;
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
// Timer callback - publish a message
// ----------------------------------------------------------------------------

static void timer_callback(struct nros_timer_t* timer, void* context) {
    (void)timer;
    talker_context_t* ctx = (talker_context_t*)context;

    // Pre-increment so the first payload is "Hello World: 1", matching the
    // official ROS 2 demo talker (demo_nodes_cpp `talker.cpp`).
    ctx->count++;
    snprintf(ctx->message.data, sizeof(ctx->message.data), "Hello World: %d", ctx->count);
    NROS_SOFTCHECK(std_msgs_msg_string_publish(ctx->publisher, &ctx->message));
    printf("Publishing: '%s'\n", ctx->message.data);
}

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline.
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

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

    // Zero-initialize all static state (avoids return-by-value temporaries on stack)
    memset(&app, 0, sizeof(app));

    // Initialize support context
    NROS_CHECK_RET(nros_support_init(&app.support, locator, domain_id), 1);
    NROS_CHECK_RET(rclc_node_init_default(&app.node, "talker", "/", &app.support), 1);

    NROS_CHECK_RET(rclc_publisher_init_default(&app.publisher, &app.node,
                                               std_msgs_msg_string_get_type_support(), "/chatter"),
                   1);

    // Create application context
    app.talker_ctx = (talker_context_t){
        .publisher = &app.publisher,
        .message = {0},
        .count = 0,
    };
    std_msgs_msg_string_init(&app.talker_ctx.message);

    NROS_CHECK_RET(
        nros_timer_init(&app.timer, &app.support, 1000000000ULL, timer_callback, &app.talker_ctx),
        1);
    NROS_CHECK_RET(nros_executor_init(&app.executor, &app.support, 4), 1);
    g_executor = &app.executor;
    NROS_CHECK_RET(rclc_executor_add_timer(&app.executor, &app.timer), 1);

    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Spin with 100ms period
    nros_ret_t ret = rclc_executor_spin_period(&app.executor, 100000000ULL);
    if (ret != NROS_RET_OK && g_running) {
        fprintf(stderr, "Executor spin failed: %d\n", ret);
    }

    // Cleanup
    rclc_executor_fini(&app.executor);
    rcl_timer_fini(&app.timer);
    nros_publisher_fini(&app.publisher);
    rcl_node_fini(&app.node);
    rclc_support_fini(&app.support);

    return 0;
}

NROS_APP_MAIN_REGISTER()
