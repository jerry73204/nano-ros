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

#endif /* NROS_EXECUTOR_H */
