/**
 * zenoh-pico ThreadX Platform Type Definitions
 *
 * Defines types required by zenoh-pico's platform abstraction for
 * Eclipse ThreadX RTOS. Works with both the Linux simulation port
 * and embedded targets (RISC-V, ARM).
 *
 * Included via ZENOH_GENERIC + ZENOH_THREADX defines pointing through
 * zenoh_generic_platform.h to this file.
 */

#ifndef ZENOH_PICO_SYSTEM_THREADX_TYPES_H
#define ZENOH_PICO_SYSTEM_THREADX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "zenoh-pico/config.h"
#include "tx_api.h"

/* Issue 0626 — `nros_platform_task_attr_t` for the `z_task_attr_t` typedef
 * below. The generic alias header already includes this; both must, or the two
 * views of the type cannot agree. */
#include <nros/platform.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Threading Types
// ============================================================================

#if Z_FEATURE_MULTI_THREAD == 1

#ifndef Z_TASK_STACK_SIZE
#define Z_TASK_STACK_SIZE 8192
#endif
/* Issue 1131 — this sizes `_z_task_t::threadx_stack`, the actual stack every
 * zenoh-pico task runs on, and a thread with no stack is not a smaller thread.
 * `tx_thread_create` refuses anything under TX_MINIMUM_STACK with
 * TX_SIZE_ERROR, so 0 buys nothing an image could use; a build that wants no
 * zenoh tasks at all sets `Z_FEATURE_MULTI_THREAD=0`, which removes this whole
 * block. Below the `#ifndef`, never above it: an undefined identifier reads as
 * 0 in `#if`, so a guard above its own default fires on every build (1167). */
#if Z_TASK_STACK_SIZE < 1
#error "Z_TASK_STACK_SIZE must be >= 1: it sizes a C array (issue 1015)"
#endif

#ifndef Z_TASK_PRIORITY
#define Z_TASK_PRIORITY 14
#endif

#ifndef Z_TASK_PREEMPT_THRESHOLD
#define Z_TASK_PREEMPT_THRESHOLD 14
#endif

#ifndef Z_TASK_TIME_SLICE
/* TX_NO_TIME_SLICE (= 0). Per-tick time-slicing across same-priority
 * threads on rv64 ThreadX appeared to interfere with the timer chain
 * for sleeping threads. Match app_thread (also TX_NO_TIME_SLICE). */
#define Z_TASK_TIME_SLICE 0
#endif

typedef struct {
    TX_THREAD threadx_thread;
    uint8_t threadx_stack[Z_TASK_STACK_SIZE];
    void *(*_fun)(void *);   /* Real entry function (full pointer width) */
    void  *_arg;             /* Real argument (full pointer width) */
    /* Phase 77.21: replaces the `tx_thread_sleep(1)` polling loop in
     * `_z_task_join`. Trampoline sets bit 0 after `_fun` returns; join
     * waits on it via `tx_event_flags_get(..., TX_WAIT_FOREVER)`. */
    TX_EVENT_FLAGS_GROUP done_flags;
} _z_task_t;

/* Issue 0626 — was `void *` and documented "Not used", because `_z_task_init`
 * below discarded it and every zenoh task took the compile-time
 * `Z_TASK_PRIORITY`. It now carries the platform ABI's attributes, so a caller
 * can state a per-task priority.
 *
 * MUST match `nros_zenoh_generic_platform.h`'s typedef: a ThreadX build sees
 * this header only inside `task.c` (which does a TU-local
 * `#undef NROS_PLATFORM_ALIASES` to reach the concrete TX_THREAD-flavoured
 * `_z_task_t`) and the generic one everywhere else. The shim ALLOCATES the
 * attr and `task.c` DEREFERENCES it, so a disagreement here is a silent
 * type confusion across that seam — issue 0135's shape. */
typedef nros_platform_task_attr_t z_task_attr_t;

typedef TX_MUTEX _z_mutex_rec_t;
typedef TX_MUTEX _z_mutex_t;

typedef struct {
    TX_MUTEX mutex;
    TX_SEMAPHORE sem;
    UINT waiters;
} _z_condvar_t;

#endif  // Z_FEATURE_MULTI_THREAD == 1

// ============================================================================
// Clock and Time Types
// ============================================================================

/**
 * Monotonic clock type.
 * Uses a timespec-compatible struct backed by tx_time_get().
 */
typedef struct {
    long tv_sec;
    long tv_nsec;
} z_clock_t;

/**
 * System time type.
 * ThreadX tick count.
 */
typedef ULONG z_time_t;

// ============================================================================
// Network Types (BSD sockets via NetX Duo nxd_bsd.h)
// ============================================================================

/**
 * Socket handle for NetX Duo BSD socket layer.
 * Uses standard BSD file descriptor returned by socket().
 */
typedef struct {
    int _fd;  // BSD socket file descriptor (-1 = invalid)
} _z_sys_net_socket_t;

/**
 * Network endpoint.
 * Stores IPv4 address and port for TCP/UDP connections.
 * Converted to sockaddr_in in the network transport layer.
 */
typedef struct {
    uint32_t _addr;   // IPv4 address in network byte order
    uint16_t _port;   // Port in network byte order
} _z_sys_net_endpoint_t;

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_THREADX_TYPES_H */
