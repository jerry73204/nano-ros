/**
 * zenoh-pico shim for Rust FFI
 *
 * This file provides wrapper functions that handle zenoh-pico types
 * with correct memory management, avoiding struct size mismatches in Rust FFI.
 */

#include <zenoh-pico.h>
#include <string.h>

// Static storage for zenoh objects (single publisher use case)
static z_owned_config_t g_config;
static z_owned_session_t g_session;
static z_owned_publisher_t g_publisher;

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
 * Declare a publisher for the given key expression
 */
int zenoh_declare_publisher(const char *keyexpr) {
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        return -1;
    }

    if (z_declare_publisher(z_session_loan(&g_session), &g_publisher,
                            z_view_keyexpr_loan(&ke), NULL) < 0) {
        return -2;
    }

    return 0;
}

/**
 * Publish data to the declared publisher
 */
int zenoh_publish(const uint8_t *data, size_t len) {
    z_owned_bytes_t payload;
    if (z_bytes_copy_from_buf(&payload, data, len) < 0) {
        return -1;
    }

    if (z_publisher_put(z_publisher_loan(&g_publisher), z_bytes_move(&payload), NULL) < 0) {
        return -2;
    }

    return 0;
}

/**
 * Close the session and clean up
 */
void zenoh_close(void) {
    z_undeclare_publisher(z_publisher_move(&g_publisher));
    zp_stop_lease_task(z_session_loan_mut(&g_session));
    zp_stop_read_task(z_session_loan_mut(&g_session));
    z_close(z_session_loan_mut(&g_session), NULL);
}
