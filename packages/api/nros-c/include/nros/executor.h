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
 * phase-417 stage 6 step B — the `nros_executor_add_timer` forwarder this
 * macro used to route through is DELETED, so `nros_executor_register_timer`
 * now names `rclc_executor_add_timer` directly. It is the only member of the
 * family whose target moved: every other `nros_executor_add_*` here is a live
 * exported symbol, not a stage-6 rename. Note what the retirement costs this
 * one macro — it used to warn on the way through, and now resolves silently
 * until the 0338 aliases are themselves retired.
 */
#ifndef NROS_NO_DEPRECATED_EXECUTOR_REGISTER_ALIASES
#define nros_executor_register_subscription nros_executor_add_subscription
#define nros_executor_register_subscription_raw_with_info                                          \
    nros_executor_add_subscription_raw_with_info
#define nros_executor_register_subscription_in_group nros_executor_add_subscription_in_group
#define nros_executor_register_timer rclc_executor_add_timer
#define nros_executor_register_timer_in_group nros_executor_add_timer_in_group
#define nros_executor_register_service nros_executor_add_service
#define nros_executor_register_guard_condition nros_executor_add_guard_condition
#define nros_executor_register_action_server nros_executor_add_action_server
#define nros_executor_register_action_client nros_executor_add_action_client
#define nros_executor_register_time_triggered_dispatcher nros_executor_add_time_triggered_dispatcher
#endif /* NROS_NO_DEPRECATED_EXECUTOR_REGISTER_ALIASES */

#endif /* NROS_EXECUTOR_H */
