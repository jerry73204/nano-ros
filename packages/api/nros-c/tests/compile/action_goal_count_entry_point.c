/*
 * The merged active-goal count entry point, pinned by SIGNATURE.
 *
 * Phase 379 W6 decision 3 collapsed a tier-disjoint PAIR into ONE function in
 * rcl's shape:
 *
 *     nros_ret_t nros_action_server_get_active_goal_count(server, size_t *out);
 *
 * Each old spelling folded "did the query work" into "what is the answer" —
 * the callback-tier `size_t nros_action_server_get_active_goal_count(const
 * server)` answered 0 for every error, and the polling-tier `int32_t
 * nros_action_server_active_goal_count_raw(server)` doubled a negative return
 * as the error code. The `_raw` one survived a release as an
 * `NROS_DEPRECATED_MSG` `static inline` forwarder keeping its `int32_t`
 * tri-state; phase-417 stage 6 step B retired it.
 *
 * The callback-tier spelling never had a forwarder and could not have one: it
 * IS the merged identifier with a different signature, and C has one
 * declaration per identifier. So pinning the merged signature below is what
 * asserts the old `size_t` form is gone — the assertion this file carried
 * before step B and still carries after it.
 *
 * Compile-only (no main): taking a function POINTER forces a real lookup and a
 * real signature match.
 */

#include "nros/action.h"

/* The merged entry point, in rcl's two-channel shape. */
static nros_ret_t (*const k_get_active_goal_count)(struct nros_action_server_t*, size_t*) =
    nros_action_server_get_active_goal_count;

/* Reference it so no compiler prunes the lookup this file exists to force. */
const void* nros_action_goal_count_entry_point_anchors[] = {
    (const void*)&k_get_active_goal_count,
};
