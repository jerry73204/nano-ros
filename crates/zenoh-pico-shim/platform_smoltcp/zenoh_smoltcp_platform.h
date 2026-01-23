/**
 * zenoh-pico smoltcp Platform Type Definitions
 *
 * This header defines the types required by zenoh-pico's platform abstraction
 * layer for use with smoltcp on bare-metal systems.
 *
 * Included via ZENOH_GENERIC define pointing to this file.
 */

#ifndef ZENOH_PICO_SYSTEM_SMOLTCP_TYPES_H
#define ZENOH_PICO_SYSTEM_SMOLTCP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "zenoh-pico/config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Threading Types (stubs when Z_FEATURE_MULTI_THREAD == 0)
// ============================================================================

#if Z_FEATURE_MULTI_THREAD == 1
// smoltcp platform is single-threaded, but provide types if needed for compilation
typedef void *_z_task_t;
typedef void *z_task_attr_t;
typedef void *_z_mutex_t;
typedef void *_z_mutex_rec_t;
typedef void *_z_condvar_t;
#endif  // Z_FEATURE_MULTI_THREAD == 1

// ============================================================================
// Clock and Time Types
// ============================================================================

/**
 * Monotonic clock type.
 * Stores milliseconds since system start (from DWT or RTIC monotonic).
 */
typedef uint64_t z_clock_t;

/**
 * System time type.
 * For embedded systems without RTC, this is the same as z_clock_t.
 * Stores milliseconds since some epoch.
 */
typedef uint64_t z_time_t;

// ============================================================================
// Network Types
// ============================================================================

/**
 * Socket handle.
 * Index into the smoltcp SocketSet plus connection state.
 */
typedef struct {
    int8_t _handle;     // Socket handle (-1 = invalid)
    bool _connected;    // Connection state
} _z_sys_net_socket_t;

/**
 * Network endpoint (address + port).
 * Stores IPv4 address and port for TCP connections.
 */
typedef struct {
    uint8_t _ip[4];     // IPv4 address bytes
    uint16_t _port;     // Port number
} _z_sys_net_endpoint_t;

#ifdef __cplusplus
}
#endif

#endif /* ZENOH_PICO_SYSTEM_SMOLTCP_TYPES_H */
