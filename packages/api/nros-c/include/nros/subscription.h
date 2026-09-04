/**
 * @file subscription.h
 * @ingroup grp_pubsub
 * @brief Topic subscription API.
 *
 * Create subscriptions with nros_subscription_init() and receive
 * deserialised messages via a user-provided callback.
 *
 * For manual polling, create the subscription with
 * nros_subscription_init_polling() and drain it with
 * nros_subscription_take_serialized() — or
 * nros_subscription_take_sequence() for a batch and
 * nros_subscription_take_validated() for the E2E-safety variant.
 */

#ifndef NROS_SUBSCRIPTION_H
#define NROS_SUBSCRIPTION_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/subscription.h>` continues to compile. */
#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings — phase-379 W6 decision 1 (2026-09-03)
 *
 * `try_recv` -> `take`. rcl (`rcl_take`), rclcpp
 * (`Subscription::take`) and our OWN RMW vtable (`take`, `take_request`,
 * `take_response`, `take_sequence`) all spell the non-blocking consuming
 * receive that way; only the user-facing layer said `try_recv`, which is
 * Rust-channel vocabulary that reads as a different contract to a ROS 2
 * user. Both forms are non-blocking and both report emptiness without
 * failing, so no platform constraint asked for the other word.
 *
 * `_raw` -> `_serialized` for the pre-CDR byte form, because that is
 * ROS 2's word for it (`rcl_take_serialized_message`).
 *
 * The forwarders are `NROS_DEPRECATED_MSG` `static inline`, the shape the
 * `nros_param_*` family established in `nros/parameter.h`: an inline
 * definition in a header has no external linkage, so every translation
 * unit may define it and none of them export it. The `take_*` name stays
 * the ONLY exported symbol — this is a SOURCE compatibility promise, not
 * a binary one, and an object file built against the pre-rename library
 * must be recompiled.
 *
 * Define NROS_NO_DEPRECATED_SUBSCRIPTION_ALIASES to compile without any
 * of it — for a consumer whose build is `-Werror` and who wants the old
 * names to be a hard error rather than a warning.
 *
 * These are scheduled for removal; migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_SUBSCRIPTION_ALIASES

NROS_DEPRECATED_MSG("nros_subscription_try_recv_raw() is deprecated; use "
                    "nros_subscription_take_serialized()")
static inline int32_t nros_subscription_try_recv_raw(struct nros_subscription_t* subscription,
                                                     uint8_t* buf, size_t buf_len) {
    return nros_subscription_take_serialized(subscription, buf, buf_len);
}

NROS_DEPRECATED_MSG("nros_subscription_try_recv_sequence() is deprecated; use "
                    "nros_subscription_take_sequence()")
static inline int32_t nros_subscription_try_recv_sequence(struct nros_subscription_t* subscription,
                                                          uint8_t* buf, size_t per_msg_cap,
                                                          size_t max_msgs, size_t* out_lens) {
    return nros_subscription_take_sequence(subscription, buf, per_msg_cap, max_msgs, out_lens);
}

NROS_DEPRECATED_MSG("nros_subscription_try_recv_validated() is deprecated; use "
                    "nros_subscription_take_validated()")
static inline int32_t
nros_subscription_try_recv_validated(struct nros_subscription_t* subscription, uint8_t* buf,
                                     size_t buf_len, struct nros_integrity_status_t* out_status) {
    return nros_subscription_take_validated(subscription, buf, buf_len, out_status);
}

#endif /* NROS_NO_DEPRECATED_SUBSCRIPTION_ALIASES */

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
 * Define NROS_NO_DEPRECATED_SUBSCRIPTION_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_SUBSCRIPTION_ALIASES

NROS_DEPRECATED_MSG("nros_subscription_get_zero_initialized() is deprecated; use "
                    "rcl_get_zero_initialized_subscription()")
static inline struct nros_subscription_t nros_subscription_get_zero_initialized(void) {
    return rcl_get_zero_initialized_subscription();
}

NROS_DEPRECATED_MSG("nros_subscription_get_default_options() is deprecated; use "
                    "rcl_subscription_get_default_options()")
static inline struct nros_subscription_options_t nros_subscription_get_default_options(void) {
    return rcl_subscription_get_default_options();
}

NROS_DEPRECATED_MSG(
    "nros_subscription_get_topic_name() is deprecated; use rcl_subscription_get_topic_name()")
static inline const char*
nros_subscription_get_topic_name(const struct nros_subscription_t* subscription) {
    return rcl_subscription_get_topic_name(subscription);
}

NROS_DEPRECATED_MSG("nros_subscription_is_valid() is deprecated; use rcl_subscription_is_valid()")
static inline bool nros_subscription_is_valid(const struct nros_subscription_t* subscription) {
    return rcl_subscription_is_valid(subscription);
}

/* The ARITY move. `rclc_subscription_init_default` takes rclc's four
 * arguments; the byte callback and its context are supplied at REGISTRATION,
 * by `nros_executor_add_subscription_raw` (raw path) or
 * `nros_executor_add_subscription_typed` (typed path, rclc's
 * `rclc_executor_add_subscription_with_context` shape). RFC-0087 records that
 * nothing mandated the old binding site: RFC-0041 governs the DISPATCH MODEL,
 * and `executor-owns-no-entity-storage` -- cited by name in ten ledger rows --
 * is defined nowhere in `docs/design/`.
 *
 * This forwarder does NOT lose the callback: it delegates to
 * `nros_subscription_init_with_qos`, which still carries it, so the old
 * behaviour is preserved exactly. */
NROS_DEPRECATED_MSG("nros_subscription_init() is deprecated; use "
                    "rclc_subscription_init_default() and supply the callback at registration "
                    "with nros_executor_add_subscription_raw() or "
                    "nros_executor_add_subscription_typed()")
static inline nros_ret_t
nros_subscription_init(struct nros_subscription_t* subscription, const struct nros_node_t* node,
                       const struct nros_message_type_t* type_info, const char* topic_name,
                       nros_subscription_callback_t callback, void* context) {
    return nros_subscription_init_with_qos(subscription, node, type_info, topic_name, callback,
                                           context, NULL);
}

#endif /* NROS_NO_DEPRECATED_SUBSCRIPTION_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_SUBSCRIPTION_H */
