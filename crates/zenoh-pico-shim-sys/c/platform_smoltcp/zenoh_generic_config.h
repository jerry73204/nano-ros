/**
 * zenoh_generic_config.h - zenoh-pico Configuration for smoltcp Platform
 *
 * This header defines the compile-time configuration for zenoh-pico
 * when building for the smoltcp bare-metal platform.
 *
 * Key differences from default configuration:
 * - Z_FEATURE_MULTI_THREAD = 0 (single-threaded operation)
 * - Only TCP link enabled (no UDP multicast, serial, etc.)
 * - Reduced buffer sizes for embedded memory constraints
 */

#ifndef ZENOH_GENERIC_CONFIG_H
#define ZENOH_GENERIC_CONFIG_H

// ============================================================================
// Buffer Sizes (optimized for embedded)
// ============================================================================

#define Z_FRAG_MAX_SIZE 2048
#define Z_BATCH_UNICAST_SIZE 1024
#define Z_BATCH_MULTICAST_SIZE 1024
#define Z_CONFIG_SOCKET_TIMEOUT 100
#define Z_TRANSPORT_LEASE 10000
#define Z_TRANSPORT_LEASE_EXPIRE_FACTOR 3
#define ZP_PERIODIC_SCHEDULER_MAX_TASKS 8

// ============================================================================
// Core Features
// ============================================================================

// Single-threaded operation (no background tasks)
#define Z_FEATURE_MULTI_THREAD 0

// Basic pub/sub functionality
#define Z_FEATURE_PUBLICATION 1
#define Z_FEATURE_ADVANCED_PUBLICATION 0
#define Z_FEATURE_SUBSCRIPTION 1
#define Z_FEATURE_ADVANCED_SUBSCRIPTION 0

// Query/queryable (for services)
#define Z_FEATURE_QUERY 1
#define Z_FEATURE_QUERYABLE 1

// Liveliness (for discovery)
#define Z_FEATURE_LIVELINESS 1

// Interest (for subscription matching)
#define Z_FEATURE_INTEREST 1

// ============================================================================
// Transport Features
// ============================================================================

// Only TCP transport enabled
#define Z_FEATURE_LINK_TCP 1
#define Z_FEATURE_LINK_UDP_MULTICAST 0
#define Z_FEATURE_LINK_UDP_UNICAST 0
#define Z_FEATURE_LINK_BLUETOOTH 0
#define Z_FEATURE_LINK_WS 0
#define Z_FEATURE_LINK_SERIAL 0
#define Z_FEATURE_LINK_SERIAL_USB 0
#define Z_FEATURE_LINK_TLS 0
#define Z_FEATURE_RAWETH_TRANSPORT 0

// Transport modes
#define Z_FEATURE_UNICAST_TRANSPORT 1
#define Z_FEATURE_MULTICAST_TRANSPORT 0

// Disable scouting (connect directly to router)
#define Z_FEATURE_SCOUTING 0
#define Z_FEATURE_SCOUTING_UDP 0

// ============================================================================
// Protocol Features
// ============================================================================

// Fragmentation support
#define Z_FEATURE_FRAGMENTATION 1

// Encoding values
#define Z_FEATURE_ENCODING_VALUES 1

// TCP_NODELAY for lower latency
#define Z_FEATURE_TCP_NODELAY 1

// Local subscriber (same-session pub/sub)
#define Z_FEATURE_LOCAL_SUBSCRIBER 0
#define Z_FEATURE_LOCAL_QUERYABLE 0

// Session validation
#define Z_FEATURE_SESSION_CHECK 1

// Batching for efficiency
#define Z_FEATURE_BATCHING 1
#define Z_FEATURE_BATCH_TX_MUTEX 0
#define Z_FEATURE_BATCH_PEER_MUTEX 0

// Matching notifications
#define Z_FEATURE_MATCHING 0

// RX cache
#define Z_FEATURE_RX_CACHE 0

// Peer features (for client mode only)
#define Z_FEATURE_UNICAST_PEER 0
#define Z_FEATURE_AUTO_RECONNECT 1

// Multicast declarations
#define Z_FEATURE_MULTICAST_DECLARATIONS 0

// Periodic tasks (disabled - using manual polling)
#define Z_FEATURE_PERIODIC_TASKS 0

// Unstable API
/* #undef Z_FEATURE_UNSTABLE_API */

#endif /* ZENOH_GENERIC_CONFIG_H */
