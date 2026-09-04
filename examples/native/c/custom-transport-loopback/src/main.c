/// @file main.c
/// @brief Phase 115.F — C-side custom-transport loopback demo.
///
/// Installs a ring-buffer `nros_transport_ops_t` via
/// `nros_set_custom_transport`, opens an nros session over a
/// `custom://loopback` locator, and exercises the four
/// callbacks: `open` once at session bring-up, `write` on every
/// outgoing wire frame, `read` on every incoming poll, `close`
/// at teardown.
///
/// Pass / fail criteria:
///  1. `nros_set_custom_transport(&ops)` returns `NROS_RET_OK`.
///  2. `open_count >= 1` after `nros_support_init` (session
///     bring-up invoked the transport's open callback).
///  3. `write_count >= 1` after publishing a frame.
///  4. `read_count >= 1` after the executor's spin tick.
///  5. `close_count >= 1` after `rclc_support_fini` /
///     `nros_set_custom_transport(NULL)`.

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <nros/nros.h>

#include "std_msgs.h"

/* --------------------------------------------------------------
 * Loopback ring buffer + bookkeeping
 * -------------------------------------------------------------- */

#define LOOPBACK_RING_CAPACITY 4096

typedef struct {
    uint8_t buf[LOOPBACK_RING_CAPACITY];
    size_t head;
    size_t tail;
    size_t len;
    pthread_mutex_t mu;
    pthread_cond_t data_ready;
    atomic_int open_count;
    atomic_int close_count;
    atomic_int write_count;
    atomic_int read_count;
} loopback_state_t;

static loopback_state_t g_loop = {
    .head = 0,
    .tail = 0,
    .len = 0,
    .mu = PTHREAD_MUTEX_INITIALIZER,
    .data_ready = PTHREAD_COND_INITIALIZER,
};

static nros_ret_t loopback_open(void* user_data, const void* params) {
    (void)user_data;
    (void)params;
    atomic_fetch_add(&g_loop.open_count, 1);
    return NROS_RET_OK;
}

static void loopback_close(void* user_data) {
    (void)user_data;
    atomic_fetch_add(&g_loop.close_count, 1);
}

static nros_ret_t loopback_write(void* user_data, const uint8_t* buf, size_t len) {
    (void)user_data;
    if (buf == NULL || len == 0) {
        return NROS_RET_OK;
    }
    pthread_mutex_lock(&g_loop.mu);
    /* Drop bytes that don't fit — keeps the loopback bounded and
     * mirrors what a real lossy transport would do. */
    size_t to_write = len;
    if (to_write > LOOPBACK_RING_CAPACITY - g_loop.len) {
        to_write = LOOPBACK_RING_CAPACITY - g_loop.len;
    }
    for (size_t i = 0; i < to_write; i++) {
        g_loop.buf[g_loop.tail] = buf[i];
        g_loop.tail = (g_loop.tail + 1) % LOOPBACK_RING_CAPACITY;
    }
    g_loop.len += to_write;
    atomic_fetch_add(&g_loop.write_count, 1);
    pthread_cond_broadcast(&g_loop.data_ready);
    pthread_mutex_unlock(&g_loop.mu);
    return NROS_RET_OK;
}

static int32_t loopback_read(void* user_data, uint8_t* buf, size_t len, uint32_t timeout_ms) {
    (void)user_data;
    if (buf == NULL || len == 0) {
        return 0;
    }
    atomic_fetch_add(&g_loop.read_count, 1);

    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += (time_t)(timeout_ms / 1000);
    deadline.tv_nsec += (long)((timeout_ms % 1000) * 1000000L);
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec += 1;
        deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&g_loop.mu);
    while (g_loop.len == 0) {
        int rc = pthread_cond_timedwait(&g_loop.data_ready, &g_loop.mu, &deadline);
        if (rc != 0) {
            pthread_mutex_unlock(&g_loop.mu);
            return 0; /* timeout — caller treats as "no data this tick" */
        }
    }
    size_t to_read = len;
    if (to_read > g_loop.len) {
        to_read = g_loop.len;
    }
    for (size_t i = 0; i < to_read; i++) {
        buf[i] = g_loop.buf[g_loop.head];
        g_loop.head = (g_loop.head + 1) % LOOPBACK_RING_CAPACITY;
    }
    g_loop.len -= to_read;
    pthread_mutex_unlock(&g_loop.mu);
    return (int32_t)to_read;
}

/* --------------------------------------------------------------
 * Application state (static .bss — no malloc)
 * -------------------------------------------------------------- */

typedef struct {
    nros_publisher_t* publisher;
    std_msgs_msg_int32 message;
    int32_t tick;
} demo_ctx_t;

static struct {
    nros_clock_t clock;
    nros_parameter_t param_storage[4];
    nros_parameter_server_t params;
    nros_support_t support;
    nros_node_t node;
    nros_publisher_t publisher;
    nros_timer_t timer;
    nros_executor_t executor;
    demo_ctx_t demo_ctx;
} app;

static volatile sig_atomic_t g_running = 1;
static nros_executor_t* g_executor = NULL;

static void signal_handler(int signum) {
    (void)signum;
    g_running = 0;
    if (g_executor != NULL) {
        nros_executor_cancel(g_executor);
    }
}

static void timer_callback(nros_timer_t* timer, void* arg) {
    (void)timer;
    demo_ctx_t* ctx = (demo_ctx_t*)arg;
    ctx->message.data = ctx->tick++;
    // Phase 212.M native/c sweep — replace the never-defined
    // `nros_publisher_publish` symbol with the canonical typed
    // helper `std_msgs_msg_int32_publish` (same shape talker uses).
    (void)std_msgs_msg_int32_publish(ctx->publisher, &ctx->message);
}

/* --------------------------------------------------------------
 * main (via nros_app_main — see <nros/app_main.h>)
 * -------------------------------------------------------------- */

int nros_app_main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    // Line-buffer stdout: glibc full-buffers non-tty stdout, so when piped to
    // a test harness each line must flush on its newline (Phase 177.34).
#ifdef _IOLBF /* absent on the bare-metal riscv64-threadx libc */
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    memset(&app, 0, sizeof(app));

    /* 1. Install the custom transport BEFORE session bring-up. */
    const nros_transport_ops_t ops = {
        .abi_version = NROS_TRANSPORT_OPS_ABI_VERSION_V1,
        ._reserved = 0,
        .user_data = NULL,
        .open = loopback_open,
        .close = loopback_close,
        .write = loopback_write,
        .read = loopback_read,
    };
    nros_ret_t set_rc = nros_set_custom_transport(&ops);
    if (set_rc != NROS_RET_OK) {
        fprintf(stderr, "nros_set_custom_transport failed: %d\n", set_rc);
        return 1;
    }
    fprintf(stdout, "loopback: transport installed\n");

    /* 2. Spin up the nros stack pointed at the `custom://` locator.
     *    Active RMW backend (zenoh-pico) routes every wire frame
     *    through our ops above. */
    (void)nros_clock_init(&app.clock, NROS_CLOCK_SYSTEM_TIME);
    (void)nros_parameter_server_init(&app.params, app.param_storage, 4);
    NROS_CHECK_RET(nros_support_init(&app.support, "custom://loopback", 0), 1);
    NROS_CHECK_RET(rclc_node_init_default(&app.node, "loopback_demo", "/", &app.support), 1);

    NROS_CHECK_RET(rclc_publisher_init_default(&app.publisher, &app.node,
                                               std_msgs_msg_int32_get_type_support(),
                                               "/loopback_chatter"),
                   1);

    app.demo_ctx.publisher = &app.publisher;
    app.demo_ctx.tick = 0;
    std_msgs_msg_int32_init(&app.demo_ctx.message);

    NROS_CHECK_RET(nros_timer_init(&app.timer, &app.support, 500ULL * 1000 * 1000 /* 500ms */,
                                   timer_callback, &app.demo_ctx),
                   1);
    NROS_CHECK_RET(nros_executor_init(&app.executor, &app.support, 4), 1);
    NROS_CHECK_RET(rclc_executor_add_timer(&app.executor, &app.timer), 1);
    g_executor = &app.executor;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    fprintf(stdout, "loopback: spinning for ~3 seconds (Ctrl-C to stop sooner)\n");
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec += 3;
    while (g_running) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline.tv_sec ||
            (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec)) {
            break;
        }
        (void)rclc_executor_spin_some(&app.executor, 100ULL * 1000 * 1000 /* 100ms */);
    }

    /* 3. Teardown — exercises the transport's close callback. */
    (void)rcl_timer_fini(&app.timer);
    (void)nros_publisher_fini(&app.publisher);
    (void)rcl_node_fini(&app.node);
    (void)rclc_executor_fini(&app.executor);
    (void)rclc_support_fini(&app.support);

    /* Drop the registered transport explicitly so `close` fires
     * even on backends that don't tear it down at session-end. */
    (void)nros_set_custom_transport(NULL);

    /* 4. Report bookkeeping. */
    int oc = atomic_load(&g_loop.open_count);
    int wc = atomic_load(&g_loop.write_count);
    int rc = atomic_load(&g_loop.read_count);
    int cc = atomic_load(&g_loop.close_count);
    fprintf(stdout, "loopback callback counts:\n");
    fprintf(stdout, "  open:  %d\n", oc);
    fprintf(stdout, "  write: %d\n", wc);
    fprintf(stdout, "  read:  %d\n", rc);
    fprintf(stdout, "  close: %d\n", cc);

    int fail = 0;
    if (oc < 1) {
        fprintf(stderr, "FAIL: open callback never fired\n");
        fail = 1;
    }
    if (wc < 1) {
        fprintf(stderr, "FAIL: write callback never fired\n");
        fail = 1;
    }
    if (rc < 1) {
        fprintf(stderr, "FAIL: read callback never fired\n");
        fail = 1;
    }
    if (cc < 1) {
        fprintf(stderr, "FAIL: close callback never fired\n");
        fail = 1;
    }

    return fail;
}

// Phase 212.M native/c sweep — emit the POSIX `int main(int argc,
// char** argv)` shim that forwards into `nros_app_main` (see
// <nros/app_main.h>). Pre-212 example shipped a bare `main()` here;
// the canonical shape uses the macro so the entry point is uniform
// across the example matrix.
NROS_APP_MAIN_REGISTER()
