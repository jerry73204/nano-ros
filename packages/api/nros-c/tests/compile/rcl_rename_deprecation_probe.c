/*
 * phase-417 stage 6 — the NEGATIVE half of `rcl_compat_aliases.c`.
 *
 * That file proves the rcl/rclc spellings COMPILE. This one proves the old
 * `nros_` spellings still WARN. A deprecation nobody is told about is just an
 * alias, and the whole reason stage 6 kept the old names as
 * `NROS_DEPRECATED_MSG` `static inline` forwarders was to get the diagnostic —
 * so "the attribute reaches callers" is the thing worth pinning, not an
 * implementation detail.
 *
 * `-Werror=deprecated-declarations` is set INSIDE the TU rather than left to
 * the lane's flags, for the reason `rcl_compat_aliases.c` gives one file over:
 * a check that only works when the caller remembers to pass a flag is one
 * copy-paste away from being decorative. It is a normal, valid TU otherwise;
 * only that pragma turns the warning into an error. Written as an expected
 * failure because a clean compile is exactly what a silently-dropped attribute
 * looks like.
 *
 * One call per HEADER, not per symbol: the forwarders are generated as one
 * block per header from one template, so a dropped attribute is a whole-header
 * event. The reorder case (`nros_node_init`) is first because it is the one
 * whose silent survival would be worst — see
 * `rcl_node_init_reorder_probe.c` for the other half of that guard.
 */

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic error "-Wdeprecated-declarations"
#endif

#include "nros/nros.h"

nros_ret_t nros_rcl_rename_deprecation_probe(struct nros_node_t* node,
                                             const struct nros_support_t* support,
                                             struct nros_executor_t* executor,
                                             struct nros_timer_t* timer,
                                             struct nros_clock_t* clock);
nros_ret_t nros_rcl_rename_deprecation_probe(struct nros_node_t* node,
                                             const struct nros_support_t* support,
                                             struct nros_executor_t* executor,
                                             struct nros_timer_t* timer,
                                             struct nros_clock_t* clock) {
    /* node.h — the REORDERED one. */
    nros_ret_t ret = nros_node_init(node, support, "probe", "/");
    /* node.h — a plain rename. */
    (void)nros_node_is_valid(node);
    /* publisher.h */
    (void)nros_publisher_get_zero_initialized();
    /* subscription.h */
    (void)nros_subscription_get_zero_initialized();
    /* client.h */
    (void)nros_client_get_zero_initialized();
    /* service.h */
    (void)nros_service_get_zero_initialized();
    /* timer.h */
    (void)nros_timer_reset(timer);
    /* clock.h */
    (void)nros_clock_time_started(clock);
    /* guard_condition.h */
    (void)nros_guard_condition_get_zero_initialized();
    /* executor.h */
    (void)nros_executor_spin_some(executor, 1000);
    /* init.h */
    (void)nros_support_fini((struct nros_support_t*)support);
    /* action.h */
    (void)nros_action_client_get_zero_initialized();
    return ret;
}
