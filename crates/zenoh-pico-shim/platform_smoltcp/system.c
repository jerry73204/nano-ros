/**
 * zenoh-pico smoltcp Platform - System Functions
 *
 * Implements the z_* system functions for bare-metal smoltcp operation:
 * - Memory management (via embedded allocator)
 * - Random number generation (via hardware RNG or PRNG)
 * - Clock and time functions (via DWT/monotonic timer)
 * - Sleep functions (busy-wait with smoltcp polling)
 * - Threading stubs (single-threaded operation)
 *
 * All functions call Rust FFI for actual implementation.
 */

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/utils/result.h"
#include <string.h>
#include <stdint.h>

// ============================================================================
// External Rust FFI Functions
// ============================================================================

// Memory management (from embedded allocator)
extern void *smoltcp_alloc(size_t size);
extern void *smoltcp_realloc(void *ptr, size_t size);
extern void smoltcp_free(void *ptr);

// Random number generation
extern uint32_t smoltcp_random_u32(void);

// Clock functions (milliseconds since boot)
extern uint64_t smoltcp_clock_now_ms(void);

// Polling (to avoid missing packets during sleep)
extern int smoltcp_poll(void);

// ============================================================================
// Memory Management
// ============================================================================

void *z_malloc(size_t size) {
    return smoltcp_alloc(size);
}

void *z_realloc(void *ptr, size_t size) {
    return smoltcp_realloc(ptr, size);
}

void z_free(void *ptr) {
    smoltcp_free(ptr);
}

// ============================================================================
// Random Number Generation
// ============================================================================

uint8_t z_random_u8(void) {
    return (uint8_t)(smoltcp_random_u32() & 0xFF);
}

uint16_t z_random_u16(void) {
    return (uint16_t)(smoltcp_random_u32() & 0xFFFF);
}

uint32_t z_random_u32(void) {
    return smoltcp_random_u32();
}

uint64_t z_random_u64(void) {
    uint64_t high = smoltcp_random_u32();
    uint64_t low = smoltcp_random_u32();
    return (high << 32) | low;
}

void z_random_fill(void *buf, size_t len) {
    uint8_t *ptr = (uint8_t *)buf;
    size_t remaining = len;

    // Fill in 4-byte chunks
    while (remaining >= 4) {
        uint32_t r = smoltcp_random_u32();
        memcpy(ptr, &r, 4);
        ptr += 4;
        remaining -= 4;
    }

    // Fill remaining bytes
    if (remaining > 0) {
        uint32_t r = smoltcp_random_u32();
        memcpy(ptr, &r, remaining);
    }
}

// ============================================================================
// Sleep Functions
// ============================================================================

z_result_t z_sleep_us(size_t time_us) {
    // Convert to milliseconds, rounding up
    size_t time_ms = (time_us + 999) / 1000;
    return z_sleep_ms(time_ms);
}

z_result_t z_sleep_ms(size_t time_ms) {
    // Busy-wait while polling smoltcp to avoid missing packets
    uint64_t start = smoltcp_clock_now_ms();
    while (smoltcp_clock_now_ms() - start < time_ms) {
        smoltcp_poll();
    }
    return _Z_RES_OK;
}

z_result_t z_sleep_s(size_t time_s) {
    return z_sleep_ms(time_s * 1000);
}

// ============================================================================
// Clock Functions (Monotonic)
// ============================================================================

z_clock_t z_clock_now(void) {
    return smoltcp_clock_now_ms();
}

unsigned long z_clock_elapsed_us(z_clock_t *time) {
    uint64_t now = smoltcp_clock_now_ms();
    uint64_t elapsed_ms = now - *time;
    return (unsigned long)(elapsed_ms * 1000);
}

unsigned long z_clock_elapsed_ms(z_clock_t *time) {
    uint64_t now = smoltcp_clock_now_ms();
    return (unsigned long)(now - *time);
}

unsigned long z_clock_elapsed_s(z_clock_t *time) {
    uint64_t now = smoltcp_clock_now_ms();
    uint64_t elapsed_ms = now - *time;
    return (unsigned long)(elapsed_ms / 1000);
}

void z_clock_advance_us(z_clock_t *clock, unsigned long duration) {
    // Add microseconds (convert to ms)
    *clock += (duration + 999) / 1000;
}

void z_clock_advance_ms(z_clock_t *clock, unsigned long duration) {
    *clock += duration;
}

void z_clock_advance_s(z_clock_t *clock, unsigned long duration) {
    *clock += duration * 1000;
}

// ============================================================================
// Time Functions (System Time)
// ============================================================================

z_time_t z_time_now(void) {
    // For embedded without RTC, use monotonic clock
    return smoltcp_clock_now_ms();
}

const char *z_time_now_as_str(char *const buf, unsigned long buflen) {
    // Simplified - just return milliseconds as string
    if (buf != NULL && buflen > 0) {
        uint64_t now = smoltcp_clock_now_ms();
        // Simple itoa for embedded
        char *ptr = buf + buflen - 1;
        *ptr = '\0';
        do {
            if (ptr == buf) break;
            ptr--;
            *ptr = '0' + (now % 10);
            now /= 10;
        } while (now > 0);
        return ptr;
    }
    return "";
}

unsigned long z_time_elapsed_us(z_time_t *time) {
    uint64_t now = smoltcp_clock_now_ms();
    uint64_t elapsed_ms = now - *time;
    return (unsigned long)(elapsed_ms * 1000);
}

unsigned long z_time_elapsed_ms(z_time_t *time) {
    uint64_t now = smoltcp_clock_now_ms();
    return (unsigned long)(now - *time);
}

unsigned long z_time_elapsed_s(z_time_t *time) {
    uint64_t now = smoltcp_clock_now_ms();
    uint64_t elapsed_ms = now - *time;
    return (unsigned long)(elapsed_ms / 1000);
}

z_result_t _z_get_time_since_epoch(_z_time_since_epoch *t) {
    // For embedded without RTC, use monotonic time
    uint64_t now_ms = smoltcp_clock_now_ms();
    t->secs = (uint32_t)(now_ms / 1000);
    t->nanos = (uint32_t)((now_ms % 1000) * 1000000);
    return _Z_RES_OK;
}

// ============================================================================
// Threading Stubs (Z_FEATURE_MULTI_THREAD == 0)
// ============================================================================

#if Z_FEATURE_MULTI_THREAD == 0

// These are compiled when multi-threading is disabled
// The platform.h header provides dummy types

z_result_t _z_task_init(_z_task_t *task, z_task_attr_t *attr, void *(*fun)(void *), void *arg) {
    (void)task;
    (void)attr;
    (void)fun;
    (void)arg;
    return _Z_ERR_GENERIC;
}

z_result_t _z_task_join(_z_task_t *task) {
    (void)task;
    return _Z_RES_OK;
}

z_result_t _z_task_detach(_z_task_t *task) {
    (void)task;
    return _Z_RES_OK;
}

z_result_t _z_task_cancel(_z_task_t *task) {
    (void)task;
    return _Z_RES_OK;
}

void _z_task_exit(void) {
    // No-op for single-threaded
}

void _z_task_free(_z_task_t **task) {
    (void)task;
}

z_result_t _z_mutex_init(_z_mutex_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_drop(_z_mutex_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_lock(_z_mutex_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_try_lock(_z_mutex_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_unlock(_z_mutex_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_rec_init(_z_mutex_rec_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_rec_drop(_z_mutex_rec_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_rec_lock(_z_mutex_rec_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_rec_try_lock(_z_mutex_rec_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_mutex_rec_unlock(_z_mutex_rec_t *m) {
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_condvar_init(_z_condvar_t *cv) {
    (void)cv;
    return _Z_RES_OK;
}

z_result_t _z_condvar_drop(_z_condvar_t *cv) {
    (void)cv;
    return _Z_RES_OK;
}

z_result_t _z_condvar_signal(_z_condvar_t *cv) {
    (void)cv;
    return _Z_RES_OK;
}

z_result_t _z_condvar_signal_all(_z_condvar_t *cv) {
    (void)cv;
    return _Z_RES_OK;
}

z_result_t _z_condvar_wait(_z_condvar_t *cv, _z_mutex_t *m) {
    (void)cv;
    (void)m;
    return _Z_RES_OK;
}

z_result_t _z_condvar_wait_until(_z_condvar_t *cv, _z_mutex_t *m, const z_clock_t *abstime) {
    (void)cv;
    (void)m;
    (void)abstime;
    return _Z_RES_OK;
}

#endif  // Z_FEATURE_MULTI_THREAD == 0

// ============================================================================
// Socket Helper Functions
// ============================================================================

z_result_t _z_socket_set_non_blocking(const _z_sys_net_socket_t *sock) {
    // smoltcp sockets are inherently non-blocking
    (void)sock;
    return _Z_RES_OK;
}

z_result_t _z_socket_accept(const _z_sys_net_socket_t *sock_in, _z_sys_net_socket_t *sock_out) {
    // Not implemented for client-mode connections
    (void)sock_in;
    (void)sock_out;
    return _Z_ERR_GENERIC;
}

void _z_socket_close(_z_sys_net_socket_t *sock) {
    // Forward to TCP close
    if (sock != NULL && sock->_handle >= 0) {
        extern void _z_close_tcp(_z_sys_net_socket_t *sock);
        _z_close_tcp(sock);
    }
}

z_result_t _z_socket_wait_event(void *peers, _z_mutex_rec_t *mutex) {
    // For single-threaded polling, just poll the network
    (void)peers;
    (void)mutex;
    smoltcp_poll();
    return _Z_RES_OK;
}
