/**
 * @file client.h
 * @ingroup grp_service
 * @brief Service client API.
 *
 * Create service clients with nros_client_init() and call services
 * with nros_client_call() (blocking).
 *
 * The non-blocking paths take the reply: nros_client_take_response()
 * after nros_client_send_request_async(), and
 * nros_client_take_response_raw() on an L1 polling client created with
 * nros_client_init_polling().
 *
 * All four are BYTES. For a typed reply — a deserialised response struct
 * rather than a CDR buffer — the generated per-service header emits
 * `<Srv>_client_send_request()`, `<Srv>_client_take_response()`,
 * `<Srv>_client_call()` and the callback trio
 * `<Srv>_client_handler_t` / `<Srv>_client_handler_init()` /
 * `<Srv>_client_set_response_callback()` (phase-417 W5.e). They are
 * `static inline` forwarders onto the entry points named above — the same
 * transport, the same timeout, one CDR implementation — so nothing here is
 * superseded, and the raw path stays the one to reach for when the payload
 * is not a generated type.
 *
 * The typed send/take helpers take a CALLER-SUPPLIED scratch buffer. That is
 * deliberate: the service pack emits no `_MAX_SERIALIZED_SIZE` constants yet
 * (the message pack does, issue 0896), so a `static inline` with a hidden
 * fixed-size array would reintroduce the silent cliff that work removed. An
 * explicit buffer has no cliff, and no allocator on the delivery path.
 */

#ifndef NROS_CLIENT_H
#define NROS_CLIENT_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/client.h>` continues to compile. */
#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings — phase-379 W6 decision 1 (2026-09-03)
 *
 * `try_recv` -> `take` and `reply` -> `response`, the pair rcl / rclcpp /
 * rclrs use (`rcl_take_response`, `rclcpp::Client::take_response`) and the
 * pair our own RMW vtable already used one layer down. See
 * `nros/subscription.h` for the full note on the shape and the ABI cost
 * (none: these are `static inline`, so `nros_client_take_response*` stay
 * the only exported symbols).
 *
 * Define NROS_NO_DEPRECATED_CLIENT_ALIASES to compile without any of it.
 *
 * These are scheduled for removal; migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_CLIENT_ALIASES

NROS_DEPRECATED_MSG("nros_client_try_recv_reply_raw() is deprecated; use "
                    "nros_client_take_response_raw()")
static inline int32_t nros_client_try_recv_reply_raw(struct nros_client_t* client, uint8_t* buf,
                                                     size_t buf_len) {
    return nros_client_take_response_raw(client, buf, buf_len);
}

NROS_DEPRECATED_MSG("nros_client_try_recv_response() is deprecated; use "
                    "nros_client_take_response()")
static inline nros_ret_t nros_client_try_recv_response(struct nros_client_t* client,
                                                       uint8_t* response_data,
                                                       size_t response_capacity,
                                                       size_t* response_len) {
    return nros_client_take_response(client, response_data, response_capacity, response_len);
}

#endif /* NROS_NO_DEPRECATED_CLIENT_ALIASES */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings -- phase-417 stage 6 (RFC-0087, 2026-09-04)
 *
 * RFC-0087 settled that **the C API takes rcl's spellings**: the goal is
 * drop-in replacement and a ported file's line is rcl's line. These names
 * are the pre-stage-6 `nros_` spellings, kept one release as
 * `NROS_DEPRECATED_MSG` `static inline` forwarders so an out-of-tree C node
 * that still writes them keeps compiling and gets a diagnostic naming its
 * replacement.
 *
 * An inline definition in a header has no external linkage, so every
 * translation unit may define it and none exports it: the rcl/rclc name is
 * the ONLY exported symbol. This is a SOURCE compatibility promise, not a
 * binary one -- an object file built against the pre-rename library must be
 * recompiled.
 *
 * `nros_ret_t`'s VALUES are unchanged. RFC-0087 records this as the one place
 * where taking rcl's spelling must not mean taking rcl's numbering; the
 * mapping lives in `<nros/rcl_compat.h>` and is the one part of that header
 * which does not dissolve.
 *
 * Define NROS_NO_DEPRECATED_CLIENT_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_CLIENT_ALIASES

NROS_DEPRECATED_MSG(
    "nros_client_get_zero_initialized() is deprecated; use rcl_get_zero_initialized_client()")
static inline struct nros_client_t nros_client_get_zero_initialized(void) {
    return rcl_get_zero_initialized_client();
}

NROS_DEPRECATED_MSG(
    "nros_client_get_default_options() is deprecated; use rcl_client_get_default_options()")
static inline struct nros_client_options_t nros_client_get_default_options(void) {
    return rcl_client_get_default_options();
}

NROS_DEPRECATED_MSG(
    "nros_client_get_service_name() is deprecated; use rcl_client_get_service_name()")
static inline const char* nros_client_get_service_name(const struct nros_client_t* client) {
    return rcl_client_get_service_name(client);
}

NROS_DEPRECATED_MSG("nros_client_is_valid() is deprecated; use rcl_client_is_valid()")
static inline bool nros_client_is_valid(const struct nros_client_t* client) {
    return rcl_client_is_valid(client);
}

NROS_DEPRECATED_MSG("nros_client_init() is deprecated; use rclc_client_init_default()")
static inline nros_ret_t nros_client_init(struct nros_client_t* client,
                                          const struct nros_node_t* node,
                                          const struct nros_service_type_t* type_info,
                                          const char* service_name) {
    return rclc_client_init_default(client, node, type_info, service_name);
}

#endif /* NROS_NO_DEPRECATED_CLIENT_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_CLIENT_H */
