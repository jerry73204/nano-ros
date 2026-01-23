/**
 * zenoh-pico-shim Platform Backend Interface
 *
 * This header defines the interface that platform backends must implement.
 * Each backend (Zephyr, POSIX, smoltcp) provides its own implementation.
 *
 * Backends handle:
 * - Background task management (threaded backends)
 * - Polling (polling backends)
 * - Time functions
 * - Platform initialization
 */

#ifndef ZENOH_SHIM_PLATFORM_H
#define ZENOH_SHIM_PLATFORM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declare zenoh session type (opaque pointer)
struct z_loaned_session_t;

// ============================================================================
// Execution Model
// ============================================================================

/**
 * Check if this backend uses polling or background threads
 *
 * @return true if polling required (smoltcp), false if threaded (Zephyr, POSIX)
 */
bool zenoh_platform_uses_polling(void);

// ============================================================================
// Background Tasks (for threaded backends)
// ============================================================================

/**
 * Start the zenoh read task
 *
 * For threaded backends, this starts a background thread that reads
 * incoming data. For polling backends, this is a no-op.
 *
 * @param session  Pointer to zenoh session
 * @return 0 on success, negative on error
 */
int zenoh_platform_start_read_task(void *session);

/**
 * Start the zenoh lease task
 *
 * For threaded backends, this starts a background thread that maintains
 * the session lease. For polling backends, this is a no-op.
 *
 * @param session  Pointer to zenoh session
 * @return 0 on success, negative on error
 */
int zenoh_platform_start_lease_task(void *session);

/**
 * Stop all background tasks
 *
 * @param session  Pointer to zenoh session
 */
void zenoh_platform_stop_tasks(void *session);

// ============================================================================
// Polling (for polling backends)
// ============================================================================

/**
 * Poll for network I/O and process zenoh protocol
 *
 * For polling backends (smoltcp), this must:
 * 1. Poll the network stack for incoming packets
 * 2. Process zenoh protocol (read task equivalent)
 * 3. Handle lease maintenance (lease task equivalent)
 *
 * For threaded backends, this is a no-op (returns 0).
 *
 * @param session     Pointer to zenoh session
 * @param timeout_ms  Maximum time to wait (0 = non-blocking)
 * @return Number of events processed, or negative on error
 */
int zenoh_platform_poll(void *session, uint32_t timeout_ms);

// ============================================================================
// Time Functions
// ============================================================================

/**
 * Get current time in milliseconds since some epoch
 *
 * @return Current time in milliseconds
 */
uint64_t zenoh_platform_time_ms(void);

/**
 * Sleep for specified duration
 *
 * For polling backends, this should continue polling during the sleep
 * to avoid missing network events.
 *
 * @param ms  Duration to sleep in milliseconds
 */
void zenoh_platform_sleep_ms(uint32_t ms);

// ============================================================================
// Platform Lifecycle
// ============================================================================

/**
 * Initialize platform-specific resources
 *
 * Called before any other platform functions. For smoltcp backend,
 * this initializes the network stack. For others, may be a no-op.
 *
 * @return 0 on success, negative on error
 */
int zenoh_platform_init(void);

/**
 * Cleanup platform-specific resources
 *
 * Called during shutdown after all zenoh resources are closed.
 */
void zenoh_platform_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // ZENOH_SHIM_PLATFORM_H
