/**
 * zenoh-pico-shim smoltcp Backend (Stub)
 *
 * This backend uses smoltcp for network I/O on bare-metal systems.
 * It requires polling rather than background threads.
 *
 * Execution model: Polling (must call zenoh_shim_poll/spin_once regularly)
 *
 * NOTE: This is a stub implementation. Full implementation is in Phase 8.4.
 * The smoltcp FFI functions are implemented in platform_smoltcp/mod.rs
 */

#include "zenoh_shim_platform.h"

#ifdef ZENOH_SHIM_SMOLTCP

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// External FFI functions (implemented in Rust platform_smoltcp/mod.rs)
// ============================================================================

// These functions will be implemented in Phase 8.4
extern int smoltcp_init(void);
extern int smoltcp_poll(void);
extern uint64_t smoltcp_time_ms(void);
extern void smoltcp_sleep_ms(uint32_t ms);
extern void smoltcp_cleanup(void);

// ============================================================================
// Execution Model
// ============================================================================

bool zenoh_platform_uses_polling(void) {
    return true;  // smoltcp requires polling
}

// ============================================================================
// Background Tasks (not used for polling backend)
// ============================================================================

int zenoh_platform_start_read_task(void *session) {
    (void)session;
    // No background tasks for polling backend
    return 0;
}

int zenoh_platform_start_lease_task(void *session) {
    (void)session;
    // No background tasks for polling backend
    return 0;
}

void zenoh_platform_stop_tasks(void *session) {
    (void)session;
    // No background tasks to stop
}

// ============================================================================
// Polling
// ============================================================================

int zenoh_platform_poll(void *session, uint32_t timeout_ms) {
    (void)session;

    // Poll the network stack
    int events = smoltcp_poll();

    // If no events and timeout requested, wait a bit and try again
    if (events == 0 && timeout_ms > 0) {
        smoltcp_sleep_ms(timeout_ms < 10 ? timeout_ms : 10);
        events = smoltcp_poll();
    }

    return events;
}

// ============================================================================
// Time Functions
// ============================================================================

uint64_t zenoh_platform_time_ms(void) {
    return smoltcp_time_ms();
}

void zenoh_platform_sleep_ms(uint32_t ms) {
    // For smoltcp, we should poll while sleeping to avoid missing packets
    uint64_t start = smoltcp_time_ms();
    while (smoltcp_time_ms() - start < ms) {
        smoltcp_poll();
    }
}

// ============================================================================
// Platform Lifecycle
// ============================================================================

int zenoh_platform_init(void) {
    return smoltcp_init();
}

void zenoh_platform_cleanup(void) {
    smoltcp_cleanup();
}

#else
// Stub implementation when smoltcp feature not enabled

bool zenoh_platform_uses_polling(void) { return true; }
int zenoh_platform_start_read_task(void *session) { (void)session; return 0; }
int zenoh_platform_start_lease_task(void *session) { (void)session; return 0; }
void zenoh_platform_stop_tasks(void *session) { (void)session; }
int zenoh_platform_poll(void *session, uint32_t timeout_ms) { (void)session; (void)timeout_ms; return -1; }
uint64_t zenoh_platform_time_ms(void) { return 0; }
void zenoh_platform_sleep_ms(uint32_t ms) { (void)ms; }
int zenoh_platform_init(void) { return -1; }
void zenoh_platform_cleanup(void) { }

#endif // ZENOH_SHIM_SMOLTCP
