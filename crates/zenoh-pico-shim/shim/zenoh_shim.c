/**
 * zenoh-pico-shim Core Implementation
 *
 * This file implements the zenoh shim API using zenoh-pico.
 * Platform-specific behavior is delegated to the backend interface.
 */

#include "zenoh_shim.h"
#include "zenoh_shim_platform.h"
#include <zenoh-pico.h>
#include <string.h>

// ============================================================================
// Internal Data Structures
// ============================================================================

// Subscriber entry with callback
typedef struct {
    z_owned_subscriber_t subscriber;
    ShimCallback callback;
    void *ctx;
    bool active;
} subscriber_entry_t;

// Publisher entry
typedef struct {
    z_owned_publisher_t publisher;
    bool active;
} publisher_entry_t;

// Static storage for zenoh objects
static z_owned_config_t g_config;
static z_owned_session_t g_session;
static bool g_session_open = false;
static bool g_initialized = false;

static publisher_entry_t g_publishers[ZENOH_SHIM_MAX_PUBLISHERS];
static subscriber_entry_t g_subscribers[ZENOH_SHIM_MAX_SUBSCRIBERS];

// ============================================================================
// Internal Helper Functions
// ============================================================================

/**
 * Internal callback that receives zenoh samples and forwards to user callback
 */
static void shim_sample_handler(z_loaned_sample_t *sample, void *arg) {
    int idx = (int)(intptr_t)arg;
    if (idx < 0 || idx >= ZENOH_SHIM_MAX_SUBSCRIBERS) {
        return;
    }

    subscriber_entry_t *entry = &g_subscribers[idx];
    if (!entry->active || entry->callback == NULL) {
        return;
    }

    // Get payload
    const z_loaned_bytes_t *payload = z_sample_payload(sample);

    // Copy bytes to slice
    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) == 0) {
        const uint8_t *data = z_slice_data(z_slice_loan(&slice));
        size_t len = z_slice_len(z_slice_loan(&slice));
        entry->callback(data, len, entry->ctx);
        z_slice_drop(z_slice_move(&slice));
    }
}

// ============================================================================
// Session Lifecycle Implementation
// ============================================================================

int zenoh_shim_init(const char *locator) {
    // Initialize storage
    memset(g_publishers, 0, sizeof(g_publishers));
    memset(g_subscribers, 0, sizeof(g_subscribers));
    g_session_open = false;

    // Initialize platform
    int ret = zenoh_platform_init();
    if (ret < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    // Initialize zenoh config
    z_config_default(&g_config);

    if (zp_config_insert(z_config_loan_mut(&g_config), Z_CONFIG_MODE_KEY, "client") < 0) {
        return ZENOH_SHIM_ERR_CONFIG;
    }

    if (locator != NULL) {
        if (zp_config_insert(z_config_loan_mut(&g_config), Z_CONFIG_CONNECT_KEY, locator) < 0) {
            return ZENOH_SHIM_ERR_CONFIG;
        }
    }

    g_initialized = true;
    return ZENOH_SHIM_OK;
}

int zenoh_shim_open(void) {
    if (!g_initialized) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    if (z_open(&g_session, z_config_move(&g_config), NULL) < 0) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // For threaded backends, start background tasks
    if (!zenoh_platform_uses_polling()) {
        int ret = zenoh_platform_start_read_task(z_session_loan_mut(&g_session));
        if (ret < 0) {
            z_close(z_session_loan_mut(&g_session), NULL);
            return ZENOH_SHIM_ERR_TASK;
        }

        ret = zenoh_platform_start_lease_task(z_session_loan_mut(&g_session));
        if (ret < 0) {
            zenoh_platform_stop_tasks(z_session_loan_mut(&g_session));
            z_close(z_session_loan_mut(&g_session), NULL);
            return ZENOH_SHIM_ERR_TASK;
        }
    }

    g_session_open = true;
    return ZENOH_SHIM_OK;
}

int zenoh_shim_is_open(void) {
    return g_session_open ? 1 : 0;
}

void zenoh_shim_close(void) {
    // Clean up publishers
    for (int i = 0; i < ZENOH_SHIM_MAX_PUBLISHERS; i++) {
        if (g_publishers[i].active) {
            z_undeclare_publisher(z_publisher_move(&g_publishers[i].publisher));
            g_publishers[i].active = false;
        }
    }

    // Clean up subscribers
    for (int i = 0; i < ZENOH_SHIM_MAX_SUBSCRIBERS; i++) {
        if (g_subscribers[i].active) {
            z_undeclare_subscriber(z_subscriber_move(&g_subscribers[i].subscriber));
            g_subscribers[i].active = false;
            g_subscribers[i].callback = NULL;
            g_subscribers[i].ctx = NULL;
        }
    }

    // Close session
    if (g_session_open) {
        if (!zenoh_platform_uses_polling()) {
            zenoh_platform_stop_tasks(z_session_loan_mut(&g_session));
        }
        z_close(z_session_loan_mut(&g_session), NULL);
        g_session_open = false;
    }

    // Cleanup platform
    zenoh_platform_cleanup();
    g_initialized = false;
}

// ============================================================================
// Publisher Implementation
// ============================================================================

int zenoh_shim_declare_publisher(const char *keyexpr) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZENOH_SHIM_MAX_PUBLISHERS; i++) {
        if (!g_publishers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZENOH_SHIM_ERR_FULL;
    }

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return ZENOH_SHIM_ERR_KEYEXPR;
    }

    if (z_declare_publisher(z_session_loan(&g_session), &g_publishers[idx].publisher,
                            z_view_keyexpr_loan(&ke), NULL) < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    g_publishers[idx].active = true;
    return idx;
}

int zenoh_shim_publish(int handle, const uint8_t *data, size_t len) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_PUBLISHERS || !g_publishers[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        return ZENOH_SHIM_ERR_PUBLISH;
    }

    if (z_publisher_put(z_publisher_loan(&g_publishers[handle].publisher),
                        z_bytes_move(&payload), NULL) < 0) {
        return ZENOH_SHIM_ERR_PUBLISH;
    }

    return ZENOH_SHIM_OK;
}

int zenoh_shim_undeclare_publisher(int handle) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_PUBLISHERS || !g_publishers[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_undeclare_publisher(z_publisher_move(&g_publishers[handle].publisher));
    g_publishers[handle].active = false;
    return ZENOH_SHIM_OK;
}

// ============================================================================
// Subscriber Implementation
// ============================================================================

int zenoh_shim_declare_subscriber(const char *keyexpr,
                                   ShimCallback callback,
                                   void *ctx) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZENOH_SHIM_MAX_SUBSCRIBERS; i++) {
        if (!g_subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZENOH_SHIM_ERR_FULL;
    }

    g_subscribers[idx].callback = callback;
    g_subscribers[idx].ctx = ctx;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        g_subscribers[idx].callback = NULL;
        g_subscribers[idx].ctx = NULL;
        return ZENOH_SHIM_ERR_KEYEXPR;
    }

    // Create closure for callback, passing index as context
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, shim_sample_handler, NULL, (void *)(intptr_t)idx);

    if (z_declare_subscriber(z_session_loan(&g_session), &g_subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL) < 0) {
        g_subscribers[idx].callback = NULL;
        g_subscribers[idx].ctx = NULL;
        return ZENOH_SHIM_ERR_GENERIC;
    }

    g_subscribers[idx].active = true;
    return idx;
}

int zenoh_shim_undeclare_subscriber(int handle) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_SUBSCRIBERS || !g_subscribers[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_undeclare_subscriber(z_subscriber_move(&g_subscribers[handle].subscriber));
    g_subscribers[handle].active = false;
    g_subscribers[handle].callback = NULL;
    g_subscribers[handle].ctx = NULL;
    return ZENOH_SHIM_OK;
}

// ============================================================================
// Polling Implementation
// ============================================================================

int zenoh_shim_poll(uint32_t timeout_ms) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    return zenoh_platform_poll(z_session_loan_mut(&g_session), timeout_ms);
}

int zenoh_shim_spin_once(uint32_t timeout_ms) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // For threaded backends, just a brief poll is enough
    // For polling backends, this handles network I/O and protocol maintenance
    return zenoh_platform_poll(z_session_loan_mut(&g_session), timeout_ms);
}

bool zenoh_shim_uses_polling(void) {
    return zenoh_platform_uses_polling();
}
