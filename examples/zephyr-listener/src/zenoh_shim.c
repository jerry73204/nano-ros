/**
 * zenoh-pico shim for Rust FFI
 *
 * This file provides wrapper functions that handle zenoh-pico types
 * with correct memory management, avoiding struct size mismatches in Rust FFI.
 *
 * Supports multiple publishers and subscribers with static storage.
 */

#include <zenoh-pico.h>
#include <string.h>

// Configuration
#define MAX_PUBLISHERS 4
#define MAX_SUBSCRIBERS 4

// Callback function type for Rust
typedef void (*rust_sample_callback_t)(const uint8_t *data, size_t len, void *ctx);

// Subscriber entry with callback
typedef struct {
    z_owned_subscriber_t subscriber;
    rust_sample_callback_t callback;
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

static publisher_entry_t g_publishers[MAX_PUBLISHERS];
static subscriber_entry_t g_subscribers[MAX_SUBSCRIBERS];

/**
 * Internal callback that receives zenoh samples and forwards to Rust
 */
static void sample_handler(z_loaned_sample_t *sample, void *arg) {
    int idx = (int)(intptr_t)arg;
    if (idx < 0 || idx >= MAX_SUBSCRIBERS) {
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

/**
 * Initialize zenoh config with client mode and connect locator
 */
int zenoh_init_config(const char *locator) {
    // Initialize storage
    memset(g_publishers, 0, sizeof(g_publishers));
    memset(g_subscribers, 0, sizeof(g_subscribers));

    z_config_default(&g_config);

    if (zp_config_insert(z_config_loan_mut(&g_config), Z_CONFIG_MODE_KEY, "client") < 0) {
        return -1;
    }

    if (locator != NULL) {
        if (zp_config_insert(z_config_loan_mut(&g_config), Z_CONFIG_CONNECT_KEY, locator) < 0) {
            return -2;
        }
    }

    return 0;
}

/**
 * Open zenoh session and start background tasks
 */
int zenoh_open_session(void) {
    if (z_open(&g_session, z_config_move(&g_config), NULL) < 0) {
        return -1;
    }

    if (zp_start_read_task(z_session_loan_mut(&g_session), NULL) < 0) {
        z_close(z_session_loan_mut(&g_session), NULL);
        return -2;
    }

    if (zp_start_lease_task(z_session_loan_mut(&g_session), NULL) < 0) {
        zp_stop_read_task(z_session_loan_mut(&g_session));
        z_close(z_session_loan_mut(&g_session), NULL);
        return -3;
    }

    g_session_open = true;
    return 0;
}

/**
 * Check if session is open
 */
int zenoh_is_session_open(void) {
    return g_session_open ? 1 : 0;
}

/**
 * Declare a publisher for the given key expression
 * Returns publisher handle (>= 0) on success, negative on error
 */
int zenoh_declare_publisher(const char *keyexpr) {
    // Find free slot
    int idx = -1;
    for (int i = 0; i < MAX_PUBLISHERS; i++) {
        if (!g_publishers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return -1; // No free slots
    }

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return -2;
    }

    if (z_declare_publisher(z_session_loan(&g_session), &g_publishers[idx].publisher,
                            z_view_keyexpr_loan(&ke), NULL) < 0) {
        return -3;
    }

    g_publishers[idx].active = true;
    return idx;
}

/**
 * Publish data using publisher handle
 */
int zenoh_publish(int handle, const uint8_t *data, size_t len) {
    if (handle < 0 || handle >= MAX_PUBLISHERS || !g_publishers[handle].active) {
        return -1;
    }

    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        return -2;
    }

    if (z_publisher_put(z_publisher_loan(&g_publishers[handle].publisher),
                        z_bytes_move(&payload), NULL) < 0) {
        return -3;
    }

    return 0;
}

/**
 * Undeclare a publisher
 */
int zenoh_undeclare_publisher(int handle) {
    if (handle < 0 || handle >= MAX_PUBLISHERS || !g_publishers[handle].active) {
        return -1;
    }

    z_undeclare_publisher(z_publisher_move(&g_publishers[handle].publisher));
    g_publishers[handle].active = false;
    return 0;
}

/**
 * Declare a subscriber for the given key expression
 * Returns subscriber handle (>= 0) on success, negative on error
 */
int zenoh_declare_subscriber(const char *keyexpr, rust_sample_callback_t callback, void *ctx) {
    // Find free slot
    int idx = -1;
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!g_subscribers[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return -1; // No free slots
    }

    g_subscribers[idx].callback = callback;
    g_subscribers[idx].ctx = ctx;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return -2;
    }

    // Create closure for callback, passing index as context
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, (void *)(intptr_t)idx);

    if (z_declare_subscriber(z_session_loan(&g_session), &g_subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL) < 0) {
        g_subscribers[idx].callback = NULL;
        g_subscribers[idx].ctx = NULL;
        return -3;
    }

    g_subscribers[idx].active = true;
    return idx;
}

/**
 * Undeclare a subscriber
 */
int zenoh_undeclare_subscriber(int handle) {
    if (handle < 0 || handle >= MAX_SUBSCRIBERS || !g_subscribers[handle].active) {
        return -1;
    }

    z_undeclare_subscriber(z_subscriber_move(&g_subscribers[handle].subscriber));
    g_subscribers[handle].active = false;
    g_subscribers[handle].callback = NULL;
    g_subscribers[handle].ctx = NULL;
    return 0;
}

/**
 * Close the session and clean up all resources
 */
void zenoh_close(void) {
    // Clean up publishers
    for (int i = 0; i < MAX_PUBLISHERS; i++) {
        if (g_publishers[i].active) {
            z_undeclare_publisher(z_publisher_move(&g_publishers[i].publisher));
            g_publishers[i].active = false;
        }
    }

    // Clean up subscribers
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (g_subscribers[i].active) {
            z_undeclare_subscriber(z_subscriber_move(&g_subscribers[i].subscriber));
            g_subscribers[i].active = false;
            g_subscribers[i].callback = NULL;
            g_subscribers[i].ctx = NULL;
        }
    }

    // Close session
    if (g_session_open) {
        zp_stop_lease_task(z_session_loan_mut(&g_session));
        zp_stop_read_task(z_session_loan_mut(&g_session));
        z_close(z_session_loan_mut(&g_session), NULL);
        g_session_open = false;
    }
}
