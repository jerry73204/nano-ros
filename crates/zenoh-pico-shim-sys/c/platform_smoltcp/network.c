/**
 * zenoh-pico smoltcp Platform - Network Functions
 *
 * Implements the TCP socket operations for zenoh-pico using smoltcp.
 * All socket operations are delegated to Rust FFI functions.
 *
 * Key differences from BSD sockets:
 * - Sockets are polled, not blocking
 * - Read/write operations return immediately with available data
 * - Blocking behavior is simulated by polling in a loop
 */

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"
#include "zenoh-pico/system/link/tcp.h"
#include "zenoh-pico/utils/result.h"
#include <string.h>
#include <stdint.h>

#if Z_FEATURE_LINK_TCP == 1

// ============================================================================
// External Rust FFI Functions
// ============================================================================

// Clock function for timeout handling
extern uint64_t smoltcp_clock_now_ms(void);

// Network polling
extern int smoltcp_poll(void);

// Socket operations
extern int smoltcp_socket_open(void);
extern int smoltcp_socket_connect(int handle, const uint8_t *ip, uint16_t port);
extern int smoltcp_socket_is_connected(int handle);
extern int smoltcp_socket_close(int handle);
extern int smoltcp_socket_recv(int handle, uint8_t *buf, size_t len);
extern int smoltcp_socket_send(int handle, const uint8_t *buf, size_t len);
extern int smoltcp_socket_can_recv(int handle);
extern int smoltcp_socket_can_send(int handle);

// ============================================================================
// Configuration
// ============================================================================

// Default socket timeout in milliseconds
#ifndef SMOLTCP_SOCKET_TIMEOUT_MS
#define SMOLTCP_SOCKET_TIMEOUT_MS 10000
#endif

// Connect timeout in milliseconds
#ifndef SMOLTCP_CONNECT_TIMEOUT_MS
#define SMOLTCP_CONNECT_TIMEOUT_MS 30000
#endif

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Parse an IP address string into bytes.
 * Supports IPv4 dotted decimal notation (e.g., "192.168.1.10").
 */
static int parse_ip_address(const char *s, uint8_t *out) {
    int octets[4] = {0, 0, 0, 0};
    int octet_idx = 0;
    int value = 0;
    bool has_digit = false;

    for (const char *p = s; *p != '\0'; p++) {
        if (*p >= '0' && *p <= '9') {
            value = value * 10 + (*p - '0');
            has_digit = true;
            if (value > 255) {
                return -1;  // Invalid octet value
            }
        } else if (*p == '.') {
            if (!has_digit || octet_idx >= 3) {
                return -1;  // Invalid format
            }
            octets[octet_idx++] = value;
            value = 0;
            has_digit = false;
        } else {
            return -1;  // Invalid character
        }
    }

    // Store last octet
    if (!has_digit || octet_idx != 3) {
        return -1;  // Incomplete address
    }
    octets[octet_idx] = value;

    out[0] = (uint8_t)octets[0];
    out[1] = (uint8_t)octets[1];
    out[2] = (uint8_t)octets[2];
    out[3] = (uint8_t)octets[3];

    return 0;
}

/**
 * Parse a port string into a uint16_t.
 */
static int parse_port(const char *s, uint16_t *out) {
    unsigned int value = 0;
    bool has_digit = false;

    for (const char *p = s; *p != '\0'; p++) {
        if (*p >= '0' && *p <= '9') {
            value = value * 10 + (*p - '0');
            has_digit = true;
            if (value > 65535) {
                return -1;  // Invalid port
            }
        } else {
            return -1;  // Invalid character
        }
    }

    if (!has_digit) {
        return -1;  // Empty string
    }

    *out = (uint16_t)value;
    return 0;
}

// ============================================================================
// Endpoint Functions
// ============================================================================

z_result_t _z_create_endpoint_tcp(_z_sys_net_endpoint_t *ep, const char *s_address, const char *s_port) {
    if (ep == NULL || s_address == NULL || s_port == NULL) {
        return _Z_ERR_GENERIC;
    }

    // Parse IP address
    if (parse_ip_address(s_address, ep->_ip) < 0) {
        return _Z_ERR_GENERIC;
    }

    // Parse port
    if (parse_port(s_port, &ep->_port) < 0) {
        return _Z_ERR_GENERIC;
    }

    return _Z_RES_OK;
}

void _z_free_endpoint_tcp(_z_sys_net_endpoint_t *ep) {
    // No dynamic allocation, nothing to free
    (void)ep;
}

// ============================================================================
// Socket Lifecycle Functions
// ============================================================================

z_result_t _z_open_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep, uint32_t tout) {
    if (sock == NULL) {
        return _Z_ERR_GENERIC;
    }

    // Initialize socket state
    sock->_handle = -1;
    sock->_connected = false;

    // Allocate a socket from smoltcp
    int handle = smoltcp_socket_open();
    if (handle < 0) {
        return _Z_ERR_GENERIC;
    }
    sock->_handle = (int8_t)handle;

    // Initiate connection
    if (smoltcp_socket_connect(handle, rep._ip, rep._port) < 0) {
        smoltcp_socket_close(handle);
        sock->_handle = -1;
        return _Z_ERR_GENERIC;
    }

    // Wait for connection with timeout
    uint32_t timeout_ms = (tout > 0) ? tout : SMOLTCP_CONNECT_TIMEOUT_MS;
    uint64_t start = smoltcp_clock_now_ms();

    while (!sock->_connected) {
        // Poll the network stack
        smoltcp_poll();

        // Check connection status
        if (smoltcp_socket_is_connected(handle) > 0) {
            sock->_connected = true;
            return _Z_RES_OK;
        }

        // Check timeout
        if (smoltcp_clock_now_ms() - start > timeout_ms) {
            smoltcp_socket_close(handle);
            sock->_handle = -1;
            return _Z_ERR_TRANSPORT_TX_FAILED;
        }
    }

    return _Z_RES_OK;
}

z_result_t _z_listen_tcp(_z_sys_net_socket_t *sock, const _z_sys_net_endpoint_t rep) {
    // Server-side listening not implemented for client-mode operation
    (void)sock;
    (void)rep;
    return _Z_ERR_GENERIC;
}

void _z_close_tcp(_z_sys_net_socket_t *sock) {
    if (sock != NULL && sock->_handle >= 0) {
        smoltcp_socket_close(sock->_handle);
        sock->_handle = -1;
        sock->_connected = false;
    }
}

// ============================================================================
// Socket I/O Functions
// ============================================================================

size_t _z_read_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    if (sock._handle < 0 || ptr == NULL || len == 0) {
        return SIZE_MAX;
    }

    uint64_t start = smoltcp_clock_now_ms();

    while (1) {
        // Poll the network stack
        smoltcp_poll();

        // Try to receive data
        if (smoltcp_socket_can_recv(sock._handle) > 0) {
            int received = smoltcp_socket_recv(sock._handle, ptr, len);
            if (received > 0) {
                return (size_t)received;
            }
        }

        // Check for connection closed or error
        if (smoltcp_socket_is_connected(sock._handle) <= 0) {
            return SIZE_MAX;
        }

        // Check timeout
        if (smoltcp_clock_now_ms() - start > SMOLTCP_SOCKET_TIMEOUT_MS) {
            return SIZE_MAX;  // Timeout
        }
    }
}

size_t _z_read_exact_tcp(const _z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    if (sock._handle < 0 || ptr == NULL) {
        return SIZE_MAX;
    }

    if (len == 0) {
        return 0;
    }

    size_t total_read = 0;
    uint64_t start = smoltcp_clock_now_ms();

    while (total_read < len) {
        // Poll the network stack
        smoltcp_poll();

        // Try to receive data
        if (smoltcp_socket_can_recv(sock._handle) > 0) {
            int received = smoltcp_socket_recv(sock._handle, ptr + total_read, len - total_read);
            if (received > 0) {
                total_read += (size_t)received;
                // Reset timeout on progress
                start = smoltcp_clock_now_ms();
            }
        }

        // Check for connection closed or error
        if (smoltcp_socket_is_connected(sock._handle) <= 0) {
            return SIZE_MAX;
        }

        // Check timeout
        if (smoltcp_clock_now_ms() - start > SMOLTCP_SOCKET_TIMEOUT_MS) {
            return SIZE_MAX;  // Timeout
        }
    }

    return total_read;
}

size_t _z_send_tcp(const _z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    if (sock._handle < 0 || ptr == NULL) {
        return SIZE_MAX;
    }

    if (len == 0) {
        return 0;
    }

    size_t total_sent = 0;
    uint64_t start = smoltcp_clock_now_ms();

    while (total_sent < len) {
        // Poll the network stack
        smoltcp_poll();

        // Try to send data
        if (smoltcp_socket_can_send(sock._handle) > 0) {
            int sent = smoltcp_socket_send(sock._handle, ptr + total_sent, len - total_sent);
            if (sent > 0) {
                total_sent += (size_t)sent;
                // Reset timeout on progress
                start = smoltcp_clock_now_ms();
            }
        }

        // Check for connection closed or error
        if (smoltcp_socket_is_connected(sock._handle) <= 0) {
            return SIZE_MAX;
        }

        // Check timeout
        if (smoltcp_clock_now_ms() - start > SMOLTCP_SOCKET_TIMEOUT_MS) {
            return SIZE_MAX;  // Timeout
        }
    }

    return total_sent;
}

#endif  // Z_FEATURE_LINK_TCP == 1
