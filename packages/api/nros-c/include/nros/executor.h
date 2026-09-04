/**
 * @file executor.h
 * @ingroup grp_executor
 * @brief Callback executor (polling) API.
 *
 * The executor drives middleware I/O and dispatches ready callbacks for
 * subscriptions, timers, services, guard conditions, and action servers.
 */

#ifndef NROS_EXECUTOR_H
#define NROS_EXECUTOR_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/executor.h>` continues to compile. */
#include "nros/types.h"
#include "nros/nros_generated.h"

/*
 * Issue 0338 — the entity-registration family is spelled `nros_executor_add_*`,
 * matching rclc (`rclc_executor_add_subscription` / `_add_timer` /
 * `_add_client` / ...). It used to be `nros_executor_register_*` for every kind
 * EXCEPT `add_client`, which kept the rclc spelling — so the family was not even
 * internally consistent and a C user could not guess the verb.
 *
 * These macros keep the old spelling compiling for one release. They are macros
 * rather than exported symbols on purpose: no new ABI surface, and a
 * recompile is all a consumer needs. Code compiled against the old SYMBOLS must
 * be rebuilt.
 *
 * phase-417 stage 6 — `nros_executor_add_timer` is now itself an
 * `NROS_DEPRECATED_MSG` forwarder onto `rclc_executor_add_timer`, so
 * `nros_executor_register_timer` resolves through TWO deprecations and the
 * user is told about both spellings in one diagnostic chain. That is
 * deliberate: pointing the 0338 macro straight at the rclc name would leave
 * the oldest spelling of all warning about nothing.
 */
#ifndef NROS_NO_DEPRECATED_EXECUTOR_REGISTER_ALIASES
#define nros_executor_register_subscription nros_executor_add_subscription
#define nros_executor_register_subscription_raw_with_info                                          \
    nros_executor_add_subscription_raw_with_info
#define nros_executor_register_subscription_in_group nros_executor_add_subscription_in_group
#define nros_executor_register_timer nros_executor_add_timer
#define nros_executor_register_timer_in_group nros_executor_add_timer_in_group
#define nros_executor_register_service nros_executor_add_service
#define nros_executor_register_guard_condition nros_executor_add_guard_condition
#define nros_executor_register_action_server nros_executor_add_action_server
#define nros_executor_register_action_client nros_executor_add_action_client
#define nros_executor_register_time_triggered_dispatcher nros_executor_add_time_triggered_dispatcher
#endif /* NROS_NO_DEPRECATED_EXECUTOR_REGISTER_ALIASES */

/* ===================================================================
 * phase-417 W4.c — `stop` -> `cancel`.
 *
 * Our own three languages gave three answers to "stop spinning without
 * tearing down the session": C had `nros_executor_stop`, Rust had
 * `Executor::halt`, and C++ had nothing at all. `cancel` is ROS 2's word
 * (`rclcpp::Executor::cancel`, `rclpy.executors.Executor.cancel`) and is
 * now the one all three carry, alongside `is_spinning` — the observable
 * that says when a cancel has been ACTED on, which the C API modelled
 * (`nros_executor_state_t`) but never exported.
 *
 * The forwarder is `NROS_DEPRECATED_MSG` `static inline`, the shape
 * `nros/parameter.h` and `nros/subscription.h` established: an inline
 * definition has no external linkage, so every translation unit may
 * define it and none export it. `nros_executor_cancel` is the ONLY
 * exported symbol — a SOURCE compatibility promise, not a binary one, so
 * an object built against the pre-rename library must be recompiled.
 *
 * Define NROS_NO_DEPRECATED_EXECUTOR_STOP_ALIAS to compile without it,
 * for a consumer whose build is `-Werror` and who wants the old name to
 * be a hard error rather than a warning.
 *
 * Scheduled for removal; migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_EXECUTOR_STOP_ALIAS

/* No `extern "C"` wrapper: a `static inline` has no external linkage, so it has
 * no name to mangle. */
NROS_DEPRECATED_MSG("nros_executor_stop() is deprecated; use nros_executor_cancel() "
                    "(and nros_executor_is_spinning() to observe when it takes effect)")
static inline nros_ret_t nros_executor_stop(struct nros_executor_t* executor) {
    return nros_executor_cancel(executor);
}

#endif /* NROS_NO_DEPRECATED_EXECUTOR_STOP_ALIAS */

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
 * Define NROS_NO_DEPRECATED_EXECUTOR_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_EXECUTOR_ALIASES

NROS_DEPRECATED_MSG("nros_executor_get_zero_initialized() is deprecated; use "
                    "rclc_executor_get_zero_initialized_executor()")
static inline struct nros_executor_t nros_executor_get_zero_initialized(void) {
    return rclc_executor_get_zero_initialized_executor();
}

NROS_DEPRECATED_MSG("nros_executor_fini() is deprecated; use rclc_executor_fini()")
static inline nros_ret_t nros_executor_fini(struct nros_executor_t* executor) {
    return rclc_executor_fini(executor);
}

NROS_DEPRECATED_MSG("nros_executor_add_timer() is deprecated; use rclc_executor_add_timer()")
static inline nros_ret_t nros_executor_add_timer(struct nros_executor_t* executor,
                                                 struct nros_timer_t* timer) {
    return rclc_executor_add_timer(executor, timer);
}

NROS_DEPRECATED_MSG("nros_executor_set_timeout() is deprecated; use rclc_executor_set_timeout()")
static inline nros_ret_t nros_executor_set_timeout(struct nros_executor_t* executor,
                                                   uint64_t timeout_ns) {
    return rclc_executor_set_timeout(executor, timeout_ns);
}

NROS_DEPRECATED_MSG(
    "nros_executor_set_semantics() is deprecated; use rclc_executor_set_semantics()")
static inline nros_ret_t nros_executor_set_semantics(struct nros_executor_t* executor,
                                                     enum nros_executor_semantics_t semantics) {
    return rclc_executor_set_semantics(executor, semantics);
}

NROS_DEPRECATED_MSG("nros_executor_set_trigger() is deprecated; use rclc_executor_set_trigger()")
static inline nros_ret_t nros_executor_set_trigger(struct nros_executor_t* executor,
                                                   nros_executor_trigger_t trigger, void* context) {
    return rclc_executor_set_trigger(executor, trigger, context);
}

NROS_DEPRECATED_MSG("nros_executor_trigger_all() is deprecated; use rclc_executor_trigger_all()")
static inline bool nros_executor_trigger_all(const bool* ready, size_t count, void* context) {
    return rclc_executor_trigger_all(ready, count, context);
}

NROS_DEPRECATED_MSG("nros_executor_trigger_any() is deprecated; use rclc_executor_trigger_any()")
static inline bool nros_executor_trigger_any(const bool* ready, size_t count, void* context) {
    return rclc_executor_trigger_any(ready, count, context);
}

NROS_DEPRECATED_MSG("nros_executor_trigger_one() is deprecated; use rclc_executor_trigger_one()")
static inline bool nros_executor_trigger_one(const bool* ready, size_t count, void* context) {
    return rclc_executor_trigger_one(ready, count, context);
}

NROS_DEPRECATED_MSG(
    "nros_executor_trigger_always() is deprecated; use rclc_executor_trigger_always()")
static inline bool nros_executor_trigger_always(const bool* ready, size_t count, void* context) {
    return rclc_executor_trigger_always(ready, count, context);
}

NROS_DEPRECATED_MSG("nros_executor_spin() is deprecated; use rclc_executor_spin()")
static inline nros_ret_t nros_executor_spin(struct nros_executor_t* executor) {
    return rclc_executor_spin(executor);
}

NROS_DEPRECATED_MSG("nros_executor_spin_some() is deprecated; use rclc_executor_spin_some()")
static inline nros_ret_t nros_executor_spin_some(struct nros_executor_t* executor,
                                                 uint64_t timeout_ns) {
    return rclc_executor_spin_some(executor, timeout_ns);
}

NROS_DEPRECATED_MSG("nros_executor_spin_period() is deprecated; use rclc_executor_spin_period()")
static inline nros_ret_t nros_executor_spin_period(struct nros_executor_t* executor,
                                                   uint64_t period_ns) {
    return rclc_executor_spin_period(executor, period_ns);
}

NROS_DEPRECATED_MSG(
    "nros_executor_spin_one_period() is deprecated; use rclc_executor_spin_one_period()")
static inline nros_ret_t nros_executor_spin_one_period(struct nros_executor_t* executor,
                                                       uint64_t period_ns) {
    return rclc_executor_spin_one_period(executor, period_ns);
}

#endif /* NROS_NO_DEPRECATED_EXECUTOR_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_EXECUTOR_H */
