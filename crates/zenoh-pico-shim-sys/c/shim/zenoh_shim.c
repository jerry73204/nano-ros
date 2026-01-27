/**
 * zenoh-pico-shim Core Implementation
 *
 * This file implements the zenoh shim API using zenoh-pico.
 * Platform-specific behavior is handled by zenoh-pico's platform layer.
 *
 * The shim provides a simplified C API that hides zenoh-pico's complex
 * ownership types from Rust FFI (avoiding struct size mismatch issues).
 */

#include "zenoh_shim.h"
#include <zenoh-pico.h>
#include <string.h>

#ifdef ZENOH_ZEPHYR
#include <zephyr/kernel.h>  // For printk
#else
#define printk(...)  // No-op on non-Zephyr platforms
#endif

// ============================================================================
// Platform-Specific Declarations
// ============================================================================

#ifdef ZENOH_SHIM_SMOLTCP
// External Rust FFI functions for smoltcp platform initialization
extern int32_t smoltcp_init(void);
extern void smoltcp_cleanup(void);
#endif

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

// Liveliness token entry
typedef struct {
    z_owned_liveliness_token_t token;
    bool active;
} liveliness_entry_t;

// Queryable entry for service servers
typedef struct {
    z_owned_queryable_t queryable;
    ShimQueryCallback callback;
    void *ctx;
    bool active;
} queryable_entry_t;

// Maximum number of concurrent liveliness tokens
#define ZENOH_SHIM_MAX_LIVELINESS 16

// Maximum number of concurrent queryables
#define ZENOH_SHIM_MAX_QUERYABLES 8

// Static storage for zenoh objects
static z_owned_config_t g_config;
static z_owned_session_t g_session;
static bool g_session_open = false;
static bool g_initialized = false;

static publisher_entry_t g_publishers[ZENOH_SHIM_MAX_PUBLISHERS];
static subscriber_entry_t g_subscribers[ZENOH_SHIM_MAX_SUBSCRIBERS];
static liveliness_entry_t g_liveliness[ZENOH_SHIM_MAX_LIVELINESS];
static queryable_entry_t g_queryables[ZENOH_SHIM_MAX_QUERYABLES];

// Storage for current query (cloned from callback for later reply)
static z_owned_query_t g_stored_query;
static bool g_stored_query_valid = false;
static int g_stored_query_idx = -1;

// Storage for blocking z_get reply
#define ZENOH_SHIM_GET_REPLY_BUF_SIZE 4096
static uint8_t g_get_reply_buf[ZENOH_SHIM_GET_REPLY_BUF_SIZE];
static size_t g_get_reply_len = 0;
static bool g_get_reply_received = false;
static bool g_get_reply_done = false;  // True when query is complete (no more replies)

// ============================================================================
// Internal Helper Functions
// ============================================================================

/**
 * Internal callback for queryable that receives queries
 */
static void shim_query_handler(z_loaned_query_t *query, void *arg) {
    int idx = (int)(intptr_t)arg;
    if (idx < 0 || idx >= ZENOH_SHIM_MAX_QUERYABLES) {
        return;
    }

    queryable_entry_t *entry = &g_queryables[idx];
    if (!entry->active || entry->callback == NULL) {
        return;
    }

    // Get keyexpr
    const z_loaned_keyexpr_t *keyexpr = z_query_keyexpr(query);
    z_view_string_t keyexpr_view;
    z_keyexpr_as_view_string(keyexpr, &keyexpr_view);
    const char *keyexpr_str = z_string_data(z_view_string_loan(&keyexpr_view));
    size_t keyexpr_len = z_string_len(z_view_string_loan(&keyexpr_view));

    // Get payload
    const z_loaned_bytes_t *payload_bytes = z_query_payload(query);
    const uint8_t *payload_data = NULL;
    size_t payload_len = 0;

    z_owned_slice_t payload_slice;
    if (payload_bytes != NULL && z_bytes_len(payload_bytes) > 0) {
        if (z_bytes_to_slice(payload_bytes, &payload_slice) == 0) {
            payload_data = z_slice_data(z_slice_loan(&payload_slice));
            payload_len = z_slice_len(z_slice_loan(&payload_slice));
        }
    }

    // Drop any previously stored query
    if (g_stored_query_valid) {
        z_query_drop(z_query_move(&g_stored_query));
        g_stored_query_valid = false;
    }

    // Clone the query for later reply (after callback returns)
    if (z_query_clone(&g_stored_query, query) == 0) {
        g_stored_query_valid = true;
        g_stored_query_idx = idx;
    }

    // Call user callback
    entry->callback(keyexpr_str, keyexpr_len, payload_data, payload_len, entry->ctx);

    // Clean up slice
    if (payload_data != NULL) {
        z_slice_drop(z_slice_move(&payload_slice));
    }
}

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

int32_t zenoh_shim_init(const char *locator) {
    // Initialize storage
    memset(g_publishers, 0, sizeof(g_publishers));
    memset(g_subscribers, 0, sizeof(g_subscribers));
    memset(g_liveliness, 0, sizeof(g_liveliness));
    memset(g_queryables, 0, sizeof(g_queryables));
    g_stored_query_valid = false;
    g_stored_query_idx = -1;
    z_internal_query_null(&g_stored_query);
    g_session_open = false;

#ifdef ZENOH_SHIM_SMOLTCP
    // Initialize smoltcp platform
    int ret = smoltcp_init();
    if (ret < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }
#endif

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

int32_t zenoh_shim_open(void) {
    if (!g_initialized) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    if (z_open(&g_session, z_config_move(&g_config), NULL) < 0) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // Start background tasks
    // For single-threaded platforms (Z_FEATURE_MULTI_THREAD=0), these are no-ops
    // For threaded platforms, they start background read/lease threads
    if (zp_start_read_task(z_session_loan_mut(&g_session), NULL) < 0) {
        z_close(z_session_loan_mut(&g_session), NULL);
        return ZENOH_SHIM_ERR_TASK;
    }

    if (zp_start_lease_task(z_session_loan_mut(&g_session), NULL) < 0) {
        zp_stop_read_task(z_session_loan_mut(&g_session));
        z_close(z_session_loan_mut(&g_session), NULL);
        return ZENOH_SHIM_ERR_TASK;
    }

    g_session_open = true;
    return ZENOH_SHIM_OK;
}

int32_t zenoh_shim_is_open(void) {
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

    // Clean up liveliness tokens
    for (int i = 0; i < ZENOH_SHIM_MAX_LIVELINESS; i++) {
        if (g_liveliness[i].active) {
            z_liveliness_undeclare_token(z_liveliness_token_move(&g_liveliness[i].token));
            g_liveliness[i].active = false;
        }
    }

    // Clean up queryables
    for (int i = 0; i < ZENOH_SHIM_MAX_QUERYABLES; i++) {
        if (g_queryables[i].active) {
            z_undeclare_queryable(z_queryable_move(&g_queryables[i].queryable));
            g_queryables[i].active = false;
            g_queryables[i].callback = NULL;
            g_queryables[i].ctx = NULL;
        }
    }

    // Close session
    if (g_session_open) {
        // Stop background tasks (no-op for single-threaded platforms)
        zp_stop_read_task(z_session_loan_mut(&g_session));
        zp_stop_lease_task(z_session_loan_mut(&g_session));
        z_close(z_session_loan_mut(&g_session), NULL);
        g_session_open = false;
    }

#ifdef ZENOH_SHIM_SMOLTCP
    // Cleanup smoltcp platform
    smoltcp_cleanup();
#endif

    g_initialized = false;
}

// ============================================================================
// Publisher Implementation
// ============================================================================

int32_t zenoh_shim_declare_publisher(const char *keyexpr) {
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
    int ke_ret = z_view_keyexpr_from_str(&ke, keyexpr);
    if (ke_ret < 0) {
        printk("zenoh_shim: z_view_keyexpr_from_str failed: %d for '%s'\n", ke_ret, keyexpr);
        return ZENOH_SHIM_ERR_KEYEXPR;
    }

    int pub_ret = z_declare_publisher(z_session_loan(&g_session), &g_publishers[idx].publisher,
                            z_view_keyexpr_loan(&ke), NULL);
    if (pub_ret < 0) {
        printk("zenoh_shim: z_declare_publisher failed: %d for '%s'\n", pub_ret, keyexpr);
        return ZENOH_SHIM_ERR_GENERIC;
    }

    g_publishers[idx].active = true;
    return idx;
}

int32_t zenoh_shim_publish(int32_t handle, const uint8_t *data, size_t len) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_PUBLISHERS || !g_publishers[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_owned_bytes_t payload;
    int bytes_ret = z_bytes_copy_from_buf(&payload, data, len);
    if (bytes_ret < 0) {
        printk("zenoh_shim: z_bytes_copy_from_buf failed: %d\n", bytes_ret);
        return ZENOH_SHIM_ERR_PUBLISH;
    }

    int put_ret = z_publisher_put(z_publisher_loan(&g_publishers[handle].publisher),
                        z_bytes_move(&payload), NULL);
    if (put_ret < 0) {
        printk("zenoh_shim: z_publisher_put failed: %d\n", put_ret);
        return ZENOH_SHIM_ERR_PUBLISH;
    }

    return ZENOH_SHIM_OK;
}

int32_t zenoh_shim_undeclare_publisher(int32_t handle) {
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

int32_t zenoh_shim_declare_subscriber(const char *keyexpr,
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

    int sub_ret = z_declare_subscriber(z_session_loan(&g_session), &g_subscribers[idx].subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL);
    if (sub_ret < 0) {
        printk("zenoh_shim: z_declare_subscriber failed: %d for '%s'\n", sub_ret, keyexpr);
        g_subscribers[idx].callback = NULL;
        g_subscribers[idx].ctx = NULL;
        return ZENOH_SHIM_ERR_GENERIC;
    }

    g_subscribers[idx].active = true;
    return idx;
}

int32_t zenoh_shim_undeclare_subscriber(int32_t handle) {
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

int32_t zenoh_shim_poll(uint32_t timeout_ms) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    (void)timeout_ms;  // Timeout handled by socket layer

    // For single-threaded platforms, zp_read() processes incoming data
    // and invokes subscriber callbacks. The underlying socket operations
    // (in network.c) poll the network stack via smoltcp_poll().
    return zp_read(z_session_loan_mut(&g_session), NULL);
}

int32_t zenoh_shim_spin_once(uint32_t timeout_ms) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    (void)timeout_ms;  // Timeout handled by socket layer

    // Combined poll and keepalive operation
    // zp_read handles incoming data, zp_send_keep_alive handles lease maintenance
    int ret = zp_read(z_session_loan_mut(&g_session), NULL);
    if (ret < 0) {
        return ret;
    }

    // Send keepalive if needed
    zp_send_keep_alive(z_session_loan_mut(&g_session), NULL);

    return ret;
}

bool zenoh_shim_uses_polling(void) {
    // Returns true if multi-threading is disabled
#if Z_FEATURE_MULTI_THREAD == 0
    return true;
#else
    return false;
#endif
}

// ============================================================================
// ZenohId Implementation
// ============================================================================

int32_t zenoh_shim_get_zid(uint8_t *zid_out) {
    if (!g_session_open || zid_out == NULL) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    z_id_t zid = z_info_zid(z_session_loan(&g_session));
    memcpy(zid_out, zid.id, 16);
    return ZENOH_SHIM_OK;
}

// ============================================================================
// Liveliness Implementation
// ============================================================================

int32_t zenoh_shim_declare_liveliness(const char *keyexpr) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZENOH_SHIM_MAX_LIVELINESS; i++) {
        if (!g_liveliness[i].active) {
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

    if (z_liveliness_declare_token(z_session_loan(&g_session),
                                    &g_liveliness[idx].token,
                                    z_view_keyexpr_loan(&ke), NULL) < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    g_liveliness[idx].active = true;
    return idx;
}

int32_t zenoh_shim_undeclare_liveliness(int32_t handle) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_LIVELINESS || !g_liveliness[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_liveliness_undeclare_token(z_liveliness_token_move(&g_liveliness[handle].token));
    g_liveliness[handle].active = false;
    return ZENOH_SHIM_OK;
}

// ============================================================================
// Publish with Attachment Implementation
// ============================================================================

int32_t zenoh_shim_publish_with_attachment(int32_t handle,
                                            const uint8_t *data, size_t len,
                                            const uint8_t *attachment, size_t attachment_len) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_PUBLISHERS || !g_publishers[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    // Create payload
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        return ZENOH_SHIM_ERR_PUBLISH;
    }

    // Create put options with attachment
    z_publisher_put_options_t options;
    z_publisher_put_options_default(&options);

    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len) < 0) {
            z_bytes_drop(z_bytes_move(&payload));
            return ZENOH_SHIM_ERR_PUBLISH;
        }
        options.attachment = z_bytes_move(&attachment_bytes);
    }

    if (z_publisher_put(z_publisher_loan(&g_publishers[handle].publisher),
                        z_bytes_move(&payload), &options) < 0) {
        return ZENOH_SHIM_ERR_PUBLISH;
    }

    return ZENOH_SHIM_OK;
}

// ============================================================================
// Queryable Implementation (for ROS 2 Services)
// ============================================================================

int32_t zenoh_shim_declare_queryable(const char *keyexpr,
                                      ShimQueryCallback callback,
                                      void *ctx) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // Find free slot
    int idx = -1;
    for (int i = 0; i < ZENOH_SHIM_MAX_QUERYABLES; i++) {
        if (!g_queryables[i].active) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        return ZENOH_SHIM_ERR_FULL;
    }

    g_queryables[idx].callback = callback;
    g_queryables[idx].ctx = ctx;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        g_queryables[idx].callback = NULL;
        g_queryables[idx].ctx = NULL;
        return ZENOH_SHIM_ERR_KEYEXPR;
    }

    // Create closure for callback
    z_owned_closure_query_t closure;
    z_closure_query(&closure, shim_query_handler, NULL, (void *)(intptr_t)idx);

    if (z_declare_queryable(z_session_loan(&g_session), &g_queryables[idx].queryable,
                            z_view_keyexpr_loan(&ke), z_closure_query_move(&closure), NULL) < 0) {
        g_queryables[idx].callback = NULL;
        g_queryables[idx].ctx = NULL;
        return ZENOH_SHIM_ERR_GENERIC;
    }

    g_queryables[idx].active = true;
    return idx;
}

int32_t zenoh_shim_undeclare_queryable(int32_t handle) {
    if (handle < 0 || handle >= ZENOH_SHIM_MAX_QUERYABLES || !g_queryables[handle].active) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_undeclare_queryable(z_queryable_move(&g_queryables[handle].queryable));
    g_queryables[handle].active = false;
    g_queryables[handle].callback = NULL;
    g_queryables[handle].ctx = NULL;
    return ZENOH_SHIM_OK;
}

// ============================================================================
// Service Client Implementation (z_get for ROS 2 service calls)
// ============================================================================

/**
 * Internal callback for z_get reply handling
 */
static void shim_get_reply_handler(z_loaned_reply_t *reply, void *ctx) {
    (void)ctx;

    // Only process successful replies
    if (!z_reply_is_ok(reply)) {
        return;
    }

    // Skip if we already have a reply (only take first)
    if (g_get_reply_received) {
        return;
    }

    const z_loaned_sample_t *sample = z_reply_ok(reply);
    const z_loaned_bytes_t *payload = z_sample_payload(sample);

    // Copy payload to reply buffer
    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) == 0) {
        const uint8_t *data = z_slice_data(z_slice_loan(&slice));
        size_t len = z_slice_len(z_slice_loan(&slice));

        if (len <= ZENOH_SHIM_GET_REPLY_BUF_SIZE) {
            memcpy(g_get_reply_buf, data, len);
            g_get_reply_len = len;
            g_get_reply_received = true;
        }
        z_slice_drop(z_slice_move(&slice));
    }
}

/**
 * Internal callback for z_get completion (dropper)
 */
static void shim_get_reply_dropper(void *ctx) {
    (void)ctx;
    g_get_reply_done = true;
}

int32_t zenoh_shim_get(const char *keyexpr,
                       const uint8_t *payload, size_t payload_len,
                       uint8_t *reply_buf, size_t reply_buf_size,
                       uint32_t timeout_ms) {
    if (!g_session_open) {
        return ZENOH_SHIM_ERR_SESSION;
    }

    // Reset reply state
    g_get_reply_len = 0;
    g_get_reply_received = false;
    g_get_reply_done = false;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return ZENOH_SHIM_ERR_KEYEXPR;
    }

    // Set up get options
    z_get_options_t opts;
    z_get_options_default(&opts);
    opts.timeout_ms = timeout_ms;

    // Set payload if provided
    z_owned_bytes_t payload_bytes;
    if (payload != NULL && payload_len > 0) {
        if (z_bytes_copy_from_buf(&payload_bytes, payload, payload_len) < 0) {
            return ZENOH_SHIM_ERR_GENERIC;
        }
        opts.payload = z_bytes_move(&payload_bytes);
    }

    // Create closure for reply handling
    z_owned_closure_reply_t callback;
    z_closure(&callback, shim_get_reply_handler, shim_get_reply_dropper, NULL);

    // Send the query
    if (z_get(z_session_loan(&g_session), z_view_keyexpr_loan(&ke), "",
              z_move(callback), &opts) < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    // For multi-threaded platforms, wait for reply via background threads
    // For single-threaded platforms, poll until reply or timeout
#if Z_FEATURE_MULTI_THREAD == 0
    // Single-threaded: poll until reply received or timeout
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;  // 10ms polling interval

    while (!g_get_reply_done && elapsed < timeout_ms) {
        zp_read(z_session_loan_mut(&g_session), NULL);
        zp_send_keep_alive(z_session_loan_mut(&g_session), NULL);

        if (g_get_reply_received) {
            break;
        }

        // Simple delay (platform-specific sleep would be better)
        // For now, just continue polling - the zenoh timeout handles the actual timing
        elapsed += poll_interval;
    }
#else
    // Multi-threaded: wait for completion with timeout
    // Background threads will invoke callbacks
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;

    while (!g_get_reply_done && elapsed < timeout_ms) {
        // On threaded platforms, the read/lease tasks handle the session
        // We just need to wait
        z_sleep_ms(poll_interval);
        elapsed += poll_interval;
    }
#endif

    // Check if we got a reply
    if (!g_get_reply_received) {
        return -9;  // ZENOH_SHIM_ERR_TIMEOUT (defined in Rust FFI)
    }

    // Copy reply to output buffer
    if (g_get_reply_len > reply_buf_size) {
        return ZENOH_SHIM_ERR_FULL;  // Buffer too small
    }

    memcpy(reply_buf, g_get_reply_buf, g_get_reply_len);
    return (int32_t)g_get_reply_len;
}

// ============================================================================
// Query Reply Implementation (for service servers)
// ============================================================================

int32_t zenoh_shim_query_reply(const char *keyexpr,
                                const uint8_t *data, size_t len,
                                const uint8_t *attachment, size_t attachment_len) {
    if (!g_stored_query_valid) {
        return ZENOH_SHIM_ERR_INVALID;
    }

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return ZENOH_SHIM_ERR_KEYEXPR;
    }

    // Create payload
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    // Create reply options with attachment
    z_query_reply_options_t options;
    z_query_reply_options_default(&options);

    z_owned_bytes_t attachment_bytes;
    if (attachment != NULL && attachment_len > 0) {
        if (z_bytes_copy_from_buf(&attachment_bytes, attachment, attachment_len) < 0) {
            z_bytes_drop(z_bytes_move(&payload));
            return ZENOH_SHIM_ERR_GENERIC;
        }
        options.attachment = z_bytes_move(&attachment_bytes);
    }

    // Reply using the stored (cloned) query
    if (z_query_reply(z_query_loan(&g_stored_query), z_view_keyexpr_loan(&ke),
                      z_bytes_move(&payload), &options) < 0) {
        return ZENOH_SHIM_ERR_GENERIC;
    }

    // Drop the stored query after reply
    z_query_drop(z_query_move(&g_stored_query));
    g_stored_query_valid = false;
    g_stored_query_idx = -1;

    return ZENOH_SHIM_OK;
}
