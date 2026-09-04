/**
 * @file service.h
 * @ingroup grp_service
 * @brief Service server API.
 *
 * Create service servers with nros_service_init().  For executor-driven
 * dispatch — the usual shape — register a `nros_service_callback_t` at init
 * time and let the executor deliver requests and send responses for you.
 *
 * For manual polling, create the server with nros_service_init_polling(),
 * take requests with nros_service_take_request_raw(), and send responses
 * with nros_service_send_response_raw().
 *
 * For a TYPED handler — a deserialised request in, a typed response out,
 * with no hand-written CDR — the generated per-service header emits
 * `<Srv>_service_handler_t`, `<Srv>_service_handler_init()` and
 * `<Srv>_service_init()`; see `packs/c/service.h.jinja` (phase-417 W5.e).
 * Those are `static inline` glue over the same entry points documented
 * here, so the two paths cannot diverge.
 *
 * (The `c:take_request` question in the phase-379 parity ledger is
 * ANSWERED: `nros_service_take_request()` is now a deprecated forwarder
 * onto `nros_service_take_request_raw()` — see below.)
 */

#ifndef NROS_SERVICE_H
#define NROS_SERVICE_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/service.h>` continues to compile. */
#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings — phase-379 W5 (2026-08-27)
 *
 * `send_reply` -> `send_response`. rcl, rclcpp and rclrs all say
 * `send_response`; our C said BOTH, with `nros_service_send_response` sitting
 * beside `nros_service_send_reply_raw`. The live spelling is
 * `nros_service_send_response_raw()` — `_raw` because that is the family
 * convention for the byte-buffer entry points (`nros_publish_raw`,
 * `nros_client_send_request_raw`), not because a non-raw twin exists.
 *
 * Both old spellings survive here as `NROS_DEPRECATED_MSG` `static inline`
 * forwarders, the shape the `nros_param_*` family established in
 * `nros/parameter.h`: an inline definition in a header has no external
 * linkage, so every translation unit may define it and none of them export
 * it. That makes `nros_service_send_response_raw` the only exported symbol
 * and these a SOURCE compatibility promise, not a binary one — an object file
 * built against the pre-rename library must be recompiled.
 *
 * `nros_service_send_response()` is the odd one: it was never an alias, it
 * was a permanent `NROS_RET_NOT_INIT` stub from before the polling API
 * existed, so no caller can ever have had it succeed. Forwarding it is
 * therefore strictly an improvement on what it did — and it keeps the
 * un-suffixed name from reading like a second, working entry point.
 *
 * Define NROS_NO_DEPRECATED_SERVICE_ALIASES to compile without any of it —
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * These are scheduled for removal; migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_SERVICE_ALIASES

NROS_DEPRECATED_MSG("nros_service_send_reply_raw() is deprecated; use "
                    "nros_service_send_response_raw()")
static inline nros_ret_t nros_service_send_reply_raw(struct nros_service_t* service,
                                                     int64_t sequence_number, const uint8_t* data,
                                                     size_t len) {
    return nros_service_send_response_raw(service, sequence_number, data, len);
}

/* phase-379 W6 decision 1 (2026-09-03): `try_recv` -> `take`. rcl
 * (`rcl_take_request`), rclcpp (`Service::take_request`) and our own RMW
 * vtable (`take_request`) all spell the non-blocking consuming receive that
 * way; only the user-facing layer said `try_recv`. Same `static inline`
 * shape, same source-not-binary promise -- see `nros/subscription.h`. */

NROS_DEPRECATED_MSG("nros_service_try_recv_request_raw() is deprecated; use "
                    "nros_service_take_request_raw()")
static inline int32_t nros_service_try_recv_request_raw(struct nros_service_t* service,
                                                        uint8_t* buf, size_t buf_len,
                                                        int64_t* sequence_number) {
    return nros_service_take_request_raw(service, buf, buf_len, sequence_number);
}

/* phase-417 W5.e (2026-09-04): `nros_service_take_request()` was a
 * PERMANENT `NROS_RET_NOT_INIT` stub — `validate_not_null!` then an
 * unconditional error return, with no code path that could ever succeed and
 * a doc comment that said so ("Currently not supported"). It read as
 * coverage: it is the name rcl uses (`rcl_take_request`) and the name a
 * porting user reaches for first, and it sat one screen from the working
 * `_raw` twin.
 *
 * It is not deleted, because deleting an exported symbol breaks a consumer
 * who did nothing wrong. It takes the shape this file already established
 * for exactly this case one entry down — `nros_service_send_response()` was
 * the same kind of stub and became a forwarder — so the un-suffixed name
 * now WORKS rather than merely existing, and forwarding is strictly an
 * improvement on what it did.
 *
 * The signatures differ, so this adapts rather than aliases: `_raw` returns
 * a byte count (`>= 0`, `0` = nothing pending) where this returns an
 * `nros_ret_t` plus an out-parameter. `NROS_RET_OK` with `*request_len == 0`
 * means "no request pending", which is what the `_raw` `0` means.
 */

NROS_DEPRECATED_MSG("nros_service_take_request() is deprecated; use "
                    "nros_service_take_request_raw(), whose return value "
                    "distinguishes 'no request' from 'error'")
static inline nros_ret_t nros_service_take_request(struct nros_service_t* service,
                                                   uint8_t* request_data, size_t request_capacity,
                                                   size_t* request_len, int64_t* sequence_number) {
    int32_t n;
    if (request_len == NULL) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    *request_len = 0;
    n = nros_service_take_request_raw(service, request_data, request_capacity, sequence_number);
    if (n < 0) {
        return (nros_ret_t)n;
    }
    *request_len = (size_t)n;
    return NROS_RET_OK;
}

NROS_DEPRECATED_MSG("nros_service_send_response() is deprecated; use "
                    "nros_service_send_response_raw()")
static inline nros_ret_t nros_service_send_response(struct nros_service_t* service,
                                                    int64_t sequence_number,
                                                    const uint8_t* response_data,
                                                    size_t response_len) {
    return nros_service_send_response_raw(service, sequence_number, response_data, response_len);
}

#endif /* NROS_NO_DEPRECATED_SERVICE_ALIASES */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings -- phase-417 stage 6 (RFC-0089, 2026-09-04)
 *
 * RFC-0089 settled that **the C API takes rcl's spellings**: the goal is
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
 * `nros_ret_t`'s VALUES are unchanged. RFC-0089 records this as the one place
 * where taking rcl's spelling must not mean taking rcl's numbering; the
 * mapping lives in `<nros/rcl_compat.h>` and is the one part of that header
 * which does not dissolve.
 *
 * Define NROS_NO_DEPRECATED_SERVICE_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_SERVICE_ALIASES

NROS_DEPRECATED_MSG(
    "nros_service_get_zero_initialized() is deprecated; use rcl_get_zero_initialized_service()")
static inline struct nros_service_t nros_service_get_zero_initialized(void) {
    return rcl_get_zero_initialized_service();
}

NROS_DEPRECATED_MSG(
    "nros_service_get_default_options() is deprecated; use rcl_service_get_default_options()")
static inline struct nros_service_options_t nros_service_get_default_options(void) {
    return rcl_service_get_default_options();
}

NROS_DEPRECATED_MSG(
    "nros_service_get_service_name() is deprecated; use rcl_service_get_service_name()")
static inline const char* nros_service_get_service_name(const struct nros_service_t* service) {
    return rcl_service_get_service_name(service);
}

NROS_DEPRECATED_MSG("nros_service_is_valid() is deprecated; use rcl_service_is_valid()")
static inline bool nros_service_is_valid(const struct nros_service_t* service) {
    return rcl_service_is_valid(service);
}

/* The ARITY move, the service side of `nros_subscription_init`'s. The handler
 * and its context are supplied at REGISTRATION now, by
 * `nros_executor_add_service_raw` -- the call rclc calls
 * `rclc_executor_add_service`. Forwards to `nros_service_init_with_qos`, which
 * still carries them, so the old behaviour is preserved exactly. */
NROS_DEPRECATED_MSG("nros_service_init() is deprecated; use rclc_service_init_default() and "
                    "supply the handler at registration with nros_executor_add_service_raw()")
static inline nros_ret_t nros_service_init(struct nros_service_t* service,
                                           const struct nros_node_t* node,
                                           const struct nros_service_type_t* type_info,
                                           const char* service_name,
                                           nros_service_callback_t callback, void* context) {
    return nros_service_init_with_qos(service, node, type_info, service_name, callback, context,
                                      NULL);
}

#endif /* NROS_NO_DEPRECATED_SERVICE_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_SERVICE_H */
