/**
 * zenoh-pico shim for Rust FFI (subscriber)
 *
 * This file provides wrapper functions that handle zenoh-pico types
 * with correct memory management, avoiding struct size mismatches in Rust FFI.
 */

#include <zenoh-pico.h>
#include <string.h>

// Callback function type for Rust
typedef void (*rust_sample_callback_t)(const uint8_t *data, size_t len, void *ctx);

// Static storage for zenoh objects (single subscriber use case)
static z_owned_config_t g_config;
static z_owned_session_t g_session;
static z_owned_subscriber_t g_subscriber;

// Callback storage
static rust_sample_callback_t g_callback = NULL;
static void *g_callback_ctx = NULL;

/**
 * Internal callback that receives zenoh samples and forwards to Rust
 */
static void sample_handler(z_loaned_sample_t *sample, void *arg) {
    (void)arg;

    if (g_callback == NULL) {
        return;
    }

    // Get payload
    const z_loaned_bytes_t *payload = z_sample_payload(sample);

    // Copy bytes to slice
    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) == 0) {
        const uint8_t *data = z_slice_data(z_slice_loan(&slice));
        size_t len = z_slice_len(z_slice_loan(&slice));
        g_callback(data, len, g_callback_ctx);
        z_slice_drop(z_slice_move(&slice));
    }
}

/**
 * Initialize zenoh config with client mode and connect locator
 */
int zenoh_init_config(const char *locator) {
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

    return 0;
}

/**
 * Declare a subscriber for the given key expression
 */
int zenoh_declare_subscriber(const char *keyexpr, rust_sample_callback_t callback, void *ctx) {
    g_callback = callback;
    g_callback_ctx = ctx;

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return -1;
    }

    // Create closure for callback
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, sample_handler, NULL, NULL);

    if (z_declare_subscriber(z_session_loan(&g_session), &g_subscriber,
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&closure), NULL) < 0) {
        return -2;
    }

    return 0;
}

/**
 * Close the session and clean up
 */
void zenoh_close(void) {
    z_undeclare_subscriber(z_subscriber_move(&g_subscriber));
    zp_stop_lease_task(z_session_loan_mut(&g_session));
    zp_stop_read_task(z_session_loan_mut(&g_session));
    z_close(z_session_loan_mut(&g_session), NULL);
    g_callback = NULL;
    g_callback_ctx = NULL;
}
