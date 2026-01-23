/**
 * zenoh-pico-shim POSIX Backend
 *
 * This backend uses POSIX APIs for threading and time.
 * It enables desktop testing of the shim on Linux/macOS.
 *
 * Execution model: Threaded (uses zenoh-pico background tasks)
 */

#include "zenoh_shim_platform.h"
#include <zenoh-pico.h>
#include <sys/time.h>
#include <unistd.h>

// ============================================================================
// Execution Model
// ============================================================================

bool zenoh_platform_uses_polling(void) {
    return false;  // POSIX uses threads
}

// ============================================================================
// Background Tasks
// ============================================================================

int zenoh_platform_start_read_task(void *session) {
    // Session is passed as z_loaned_session_t* from the core shim
    z_loaned_session_t *s = (z_loaned_session_t *)session;
    return zp_start_read_task(s, NULL);
}

int zenoh_platform_start_lease_task(void *session) {
    z_loaned_session_t *s = (z_loaned_session_t *)session;
    return zp_start_lease_task(s, NULL);
}

void zenoh_platform_stop_tasks(void *session) {
    z_loaned_session_t *s = (z_loaned_session_t *)session;
    zp_stop_lease_task(s);
    zp_stop_read_task(s);
}

// ============================================================================
// Polling (no-op for threaded backend)
// ============================================================================

int zenoh_platform_poll(void *session, uint32_t timeout_ms) {
    (void)session;
    (void)timeout_ms;
    // For threaded backends, polling is handled by background tasks
    // Just sleep briefly to yield
    if (timeout_ms > 0) {
        usleep(timeout_ms * 1000);
    }
    return 0;
}

// ============================================================================
// Time Functions
// ============================================================================

uint64_t zenoh_platform_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void zenoh_platform_sleep_ms(uint32_t ms) {
    usleep(ms * 1000);
}

// ============================================================================
// Platform Lifecycle
// ============================================================================

int zenoh_platform_init(void) {
    // No special initialization needed for POSIX
    return 0;
}

void zenoh_platform_cleanup(void) {
    // No special cleanup needed for POSIX
}
