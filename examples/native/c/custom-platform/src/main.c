/**
 * @file main.c
 * @brief Bare-metal platform demo for nros-c
 *
 * This example demonstrates:
 * 1. How to use the platform abstraction layer
 * 2. Guard conditions for cross-thread/interrupt signaling
 * 3. Static allocation patterns (no malloc)
 * 4. Timer callbacks
 * 5. Executor-based event loop
 *
 * On a real embedded system, this would run in a main loop or RTOS task.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

// NROS_PLATFORM_POSIX is propagated by the NanoRos::NanoRos CMake target.
// On real bare-metal: link a board crate that forwards the
// platform-bare-metal feature instead.

// nros modular includes
#include <nros/app_main.h>
#include <nros/init.h>
#include <nros/log.h>
#include <nros/node.h>
#include <nros/publisher.h>
#include <nros/timer.h>
#include <nros/executor.h>
#include <nros/guard_condition.h>
#include <nros/clock.h>

// ============================================================================
// Message Definition (statically allocated)
// ============================================================================

typedef struct {
    int32_t data;
} std_msgs_Int32;

static const nros_message_type_t std_msgs_Int32_type = {
    .type_name = "std_msgs::msg::dds_::Int32_",
    .type_hash = "RIHS01_5bf22a2e7c2c8a4ca3f55054648f6d8c7c77cc0ae5695a1ff1df0b7ef8df1f09",
    .serialized_size_max = 8,
};

// Serialize message (all buffers statically sized)
static int32_t std_msgs_Int32_serialize(const std_msgs_Int32* msg, uint8_t* buffer,
                                        size_t buffer_size) {
    if (buffer_size < 8) return -1;
    buffer[0] = 0x00;
    buffer[1] = 0x01;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = (uint8_t)(msg->data & 0xFF);
    buffer[5] = (uint8_t)((msg->data >> 8) & 0xFF);
    buffer[6] = (uint8_t)((msg->data >> 16) & 0xFF);
    buffer[7] = (uint8_t)((msg->data >> 24) & 0xFF);
    return 8;
}

// ============================================================================
// Static Allocation - All memory is allocated at compile time
// ============================================================================

// Application state (would be in .bss section on embedded)
static struct {
    // nros resources
    nros_support_t support;
    nros_node_t node;
    nros_publisher_t publisher;
    nros_timer_t timer;
    nros_executor_t executor;
    nros_guard_condition_t shutdown_guard;

    // Application data
    std_msgs_Int32 message;
    int count;
    volatile bool running;
} app;

// Serialization buffer (statically allocated)
static uint8_t g_serialize_buffer[64];

// Phase 88.16.B — set after `rclc_node_init_default`; used by post-init
// diagnostics + the timer callback. NULL before init = `NROS_LOG_*`
// silently drops. Moved here from after `main` (Phase 212.M native/c
// sweep) so the C compiler sees the declaration before the callback's
// use site.
static nros_logger_t g_logger = NULL;

// ============================================================================
// Callbacks
// ============================================================================

/**
 * Timer callback - called periodically by the executor.
 *
 * On a real embedded system, this is where you would:
 * - Read sensors
 * - Publish telemetry
 * - Update control loops
 */
static void timer_callback(struct nros_timer_t* timer, void* context) {
    (void)timer;
    (void)context;

    app.count++;
    app.message.data = app.count;

    int32_t len =
        std_msgs_Int32_serialize(&app.message, g_serialize_buffer, sizeof(g_serialize_buffer));
    if (len > 0) {
        nros_ret_t ret = nros_publish_raw(&app.publisher, g_serialize_buffer, (size_t)len);
        if (ret == NROS_RET_OK) {
            NROS_LOG_INFO(g_logger, "Published: %d", app.message.data);
        }
    }
}

/**
 * Guard condition callback - triggered when shutdown is requested.
 *
 * Guard conditions are useful for:
 * - Signaling shutdown from an interrupt handler
 * - Waking up the executor from another thread
 * - Coordinating between tasks in an RTOS
 */
static void shutdown_callback(void* context) {
    (void)context;
    printf("[Guard] Shutdown signal received!\n");
    app.running = false;
    (void)nros_executor_cancel(&app.executor);
}

// ============================================================================
// Signal Handler (simulates external interrupt on embedded)
// ============================================================================

static void signal_handler(int signum) {
    (void)signum;
    printf("\n[Signal] SIGINT received, triggering guard condition...\n");

    // This is how you would signal from an interrupt handler:
    // The guard condition trigger is thread-safe and can be called from any context
    (void)rcl_trigger_guard_condition(&app.shutdown_guard);
}

// ============================================================================
// Platform Time Demo
// ============================================================================

static void demo_platform_time(void) {
    printf("\n=== Platform Time Demo ===\n");

    // Get time using platform abstraction
    nros_clock_t clock = nros_clock_get_zero_initialized();
    (void)nros_clock_init(&clock, NROS_CLOCK_STEADY_TIME);

    nros_time_t t1, t2;
    (void)nros_clock_get_now(&clock, &t1);

    // Sleep for 100ms using platform sleep
    printf("Sleeping for 100ms...\n");
    // The portable way is the canonical platform ABI: nros_platform_sleep_ms(100)
    // (see src/platform_impl.c). Use nanosleep here to keep this demo TU free of
    // the platform header.
    struct timespec ts = {0, 100000000}; // 100ms
    nanosleep(&ts, NULL);

    (void)nros_clock_get_now(&clock, &t2);

    int64_t elapsed_ns = nros_time_to_nanoseconds(&t2) - nros_time_to_nanoseconds(&t1);
    printf("Elapsed time: %.3f ms\n", (double)elapsed_ns / 1000000.0);

    (void)rcl_clock_fini(&clock);
}

// ============================================================================
// Guard Condition Demo
// ============================================================================

static void demo_guard_condition(void) {
    printf("\n=== Guard Condition Demo ===\n");

    // Initialize guard condition with callback
    app.shutdown_guard = rcl_get_zero_initialized_guard_condition();
    nros_ret_t ret = nros_guard_condition_init(&app.shutdown_guard, &app.support);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to init guard condition: %d\n", ret);
        return;
    }

    ret = nros_guard_condition_set_callback(&app.shutdown_guard, shutdown_callback, NULL);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to set guard condition callback: %d\n", ret);
        return;
    }

    // Check initial state
    printf("Guard condition initialized\n");
    printf("  Is valid: %s\n", nros_guard_condition_is_valid(&app.shutdown_guard) ? "yes" : "no");
    printf("  Is triggered: %s\n",
           nros_guard_condition_is_triggered(&app.shutdown_guard) ? "yes" : "no");

    // Demonstrate trigger/clear cycle
    printf("Triggering guard condition...\n");
    (void)rcl_trigger_guard_condition(&app.shutdown_guard);
    printf("  Is triggered: %s\n",
           nros_guard_condition_is_triggered(&app.shutdown_guard) ? "yes" : "no");

    printf("Clearing guard condition...\n");
    (void)nros_guard_condition_clear(&app.shutdown_guard);
    printf("  Is triggered: %s\n",
           nros_guard_condition_is_triggered(&app.shutdown_guard) ? "yes" : "no");

    // Add to executor - callback will be invoked when triggered
    ret = nros_executor_add_guard_condition(&app.executor, &app.shutdown_guard);
    if (ret == NROS_RET_OK) {
        printf("Guard condition added to executor\n");
    }
}

// ============================================================================
// Main
// ============================================================================

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline (Phase 177.34).
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    printf("=========================================\n");
    printf("nros-c Bare-Metal Platform Demo\n");
    printf("=========================================\n");
    printf("\n");
    printf("This demo shows:\n");
    printf("  - Platform abstraction layer usage\n");
    printf("  - Guard conditions for async signaling\n");
    printf("  - Static allocation (no malloc)\n");
    printf("  - Timer-based publishing\n");
    printf("\n");

    // Initialize static application state
    memset(&app, 0, sizeof(app));
    app.running = true;

    // Demo platform time functions
    demo_platform_time();

    // Get configuration
    const char* locator = getenv("NROS_LOCATOR");
    if (!locator) locator = NROS_ENTRY_LOCATOR;

    const char* domain_str = getenv("ROS_DOMAIN_ID");
    uint8_t domain_id = domain_str ? (uint8_t)atoi(domain_str) : 0;

    printf("\n=== Initialization ===\n");
    printf("Locator: %s\n", locator);
    printf("Domain ID: %d\n", domain_id);

    // Initialize support
    app.support = nros_support_get_zero_initialized();
    nros_ret_t ret = nros_support_init(&app.support, locator, domain_id);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to init support: %d\n", ret);
        return 1;
    }
    printf("Support initialized\n");

    // Initialize node
    app.node = rcl_get_zero_initialized_node();
    ret = rclc_node_init_default(&app.node, "baremetal_demo", "/", &app.support);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to init node: %d\n", ret);
        goto cleanup_support;
    }
    printf("Node created: %s\n", rcl_node_get_name(&app.node));

    // Fetch the node's logger so NROS_LOG_* in the timer callback emits.
    // Without this g_logger stays NULL and every log call silently drops
    // (the timer still fires and publishing still succeeds — the messages
    // just never print). See the logging example's `nros_node_get_logger`.
    g_logger = nros_node_get_logger(&app.node);

    // Initialize publisher
    app.publisher = rcl_get_zero_initialized_publisher();
    ret = rclc_publisher_init_default(&app.publisher, &app.node, &std_msgs_Int32_type,
                                      "/baremetal_demo/counter");
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to init publisher: %d\n", ret);
        goto cleanup_node;
    }
    printf("Publisher created: %s\n", rcl_publisher_get_topic_name(&app.publisher));

    // Initialize timer (500ms period)
    app.timer = rcl_get_zero_initialized_timer();
    ret = nros_timer_init(&app.timer, &app.support, 500000000ULL, timer_callback, NULL);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to init timer: %d\n", ret);
        goto cleanup_publisher;
    }
    printf("Timer created (500ms period)\n");

    // Initialize executor
    app.executor = rclc_executor_get_zero_initialized_executor();
    ret = nros_executor_init(&app.executor, &app.support, 4);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to init executor: %d\n", ret);
        goto cleanup_timer;
    }

    // Add timer to executor
    ret = rclc_executor_add_timer(&app.executor, &app.timer);
    if (ret != NROS_RET_OK) {
        fprintf(stderr, "Failed to add timer: %d\n", ret);
        goto cleanup_executor;
    }
    printf("Executor initialized with %d handles\n", nros_executor_get_handle_count(&app.executor));

    // Demo and setup guard condition
    demo_guard_condition();

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\n=== Running ===\n");
    printf("Publishing every 500ms (Ctrl+C to exit)...\n\n");

    // Main loop - on embedded this would be the main while(1) loop
    // The executor handles all callbacks including:
    // - Timer callbacks (periodic publishing)
    // - Guard condition callbacks (shutdown signal)
    ret = rclc_executor_spin_period(&app.executor, 50000000ULL); // 50ms spin period

    // Cleanup (in reverse order of initialization)
    printf("\n=== Cleanup ===\n");

    (void)rcl_guard_condition_fini(&app.shutdown_guard);
    printf("Guard condition finalized\n");

cleanup_executor:
    (void)rclc_executor_fini(&app.executor);
    printf("Executor finalized\n");

cleanup_timer:
    (void)rcl_timer_fini(&app.timer);
    printf("Timer finalized\n");

cleanup_publisher:
    (void)nros_publisher_fini(&app.publisher);
    printf("Publisher finalized\n");

cleanup_node:
    (void)rcl_node_fini(&app.node);
    printf("Node finalized\n");

cleanup_support:
    (void)rclc_support_fini(&app.support);
    printf("Support finalized\n");

    printf("\n");
    printf("Demo complete. Memory usage summary:\n");
    printf("  Static app struct: %zu bytes\n", sizeof(app));
    printf("  Serialize buffer:  %zu bytes\n", sizeof(g_serialize_buffer));
    printf("  Total static:      %zu bytes\n", sizeof(app) + sizeof(g_serialize_buffer));
    printf("\n");

    return 0;
}

NROS_APP_MAIN_REGISTER()
