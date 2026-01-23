/**
 * zenoh-pico-shim Zephyr Backend
 *
 * This backend uses Zephyr RTOS APIs for threading and time.
 *
 * Execution model: Threaded (uses zenoh-pico background tasks)
 */

#include "zenoh_shim_platform.h"

#ifdef __ZEPHYR__

#include <zenoh-pico.h>
#include <zephyr/kernel.h>

// ============================================================================
// Execution Model
// ============================================================================

bool zenoh_platform_uses_polling(void) {
    return false;  // Zephyr uses threads
}

// ============================================================================
// Background Tasks
// ============================================================================

int zenoh_platform_start_read_task(void *session) {
    z_loaned_session_t *s = (z_loaned_session_t *)session;
    return zp_start_read_task(z_session_loan_mut(s), NULL);
}

int zenoh_platform_start_lease_task(void *session) {
    z_loaned_session_t *s = (z_loaned_session_t *)session;
    return zp_start_lease_task(z_session_loan_mut(s), NULL);
}

void zenoh_platform_stop_tasks(void *session) {
    z_loaned_session_t *s = (z_loaned_session_t *)session;
    zp_stop_lease_task(z_session_loan_mut(s));
    zp_stop_read_task(z_session_loan_mut(s));
}

// ============================================================================
// Polling (no-op for threaded backend)
// ============================================================================

int zenoh_platform_poll(void *session, uint32_t timeout_ms) {
    (void)session;
    // For threaded backends, polling is handled by background tasks
    // Just sleep briefly to yield
    if (timeout_ms > 0) {
        k_msleep(timeout_ms);
    }
    return 0;
}

// ============================================================================
// Time Functions
// ============================================================================

uint64_t zenoh_platform_time_ms(void) {
    return k_uptime_get();
}

void zenoh_platform_sleep_ms(uint32_t ms) {
    k_msleep(ms);
}

// ============================================================================
// Platform Lifecycle
// ============================================================================

int zenoh_platform_init(void) {
    // No special initialization needed for Zephyr
    return 0;
}

void zenoh_platform_cleanup(void) {
    // No special cleanup needed for Zephyr
}

#else
// Stub implementation when not building for Zephyr
// This allows the file to be compiled but will error at link time if selected

bool zenoh_platform_uses_polling(void) { return false; }
int zenoh_platform_start_read_task(void *session) { (void)session; return -1; }
int zenoh_platform_start_lease_task(void *session) { (void)session; return -1; }
void zenoh_platform_stop_tasks(void *session) { (void)session; }
int zenoh_platform_poll(void *session, uint32_t timeout_ms) { (void)session; (void)timeout_ms; return -1; }
uint64_t zenoh_platform_time_ms(void) { return 0; }
void zenoh_platform_sleep_ms(uint32_t ms) { (void)ms; }
int zenoh_platform_init(void) { return -1; }
void zenoh_platform_cleanup(void) { }

#endif // __ZEPHYR__
