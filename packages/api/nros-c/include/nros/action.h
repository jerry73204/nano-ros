/**
 * @file action.h
 * @ingroup grp_action
 * @brief Action server and client API.
 *
 * Actions provide long-running goal-oriented communication with
 * feedback and cancellation support.
 */

#ifndef NROS_ACTION_H
#define NROS_ACTION_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/action.h>` continues to compile. */
#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * Deprecated compatibility alias (phase 379 W6, decision 3)
 *
 * The active-goal count shipped TWICE, tier-disjoint, each spelling
 * collapsing "did the query work" into "what is the answer":
 *
 *   size_t  nros_action_server_get_active_goal_count(const server)
 *              -- callback tier (state INITIALIZED, executor arena);
 *                 answered 0 for every error, so "error" and "no goals"
 *                 were the same value.
 *   int32_t nros_action_server_active_goal_count_raw(server)
 *              -- polling tier (state POLLING, inline core); negative
 *                 return doubled as the error code.
 *
 * Both are replaced by one function in rcl's shape -- return code for the
 * call, out-param for the answer, exactly as
 * `rcl_action_server_get_goal_handles(server, ***handles, size_t
 * *num_goals) -> rcl_ret_t` reports its count:
 *
 *   nros_ret_t nros_action_server_get_active_goal_count(server, size_t *out);
 *
 * which branches on `server->state` internally to serve both tiers.
 *
 * ONLY THE `_raw` SPELLING GETS A FORWARDER. The callback-tier spelling
 * IS the new name -- `nros_action_server_get_active_goal_count` -- with a
 * new signature, and C has one declaration per identifier, so there is no
 * way to keep the old `size_t`-returning form alive beside it. A caller of
 * the old form gets a hard compile error ("too few arguments", or a
 * `size_t` initialised from `nros_ret_t`), which is the right failure for a
 * signature change: the alternative would be a silent semantic swap. There
 * were no callers anywhere in this tree when the merge landed.
 *
 * `static inline` (not a second NROS_PUBLIC declaration) is what keeps the
 * forwarder from becoming a duplicate-symbol problem: an inline definition
 * in a header has no external linkage. That also makes this a SOURCE
 * compatibility promise, not a binary one -- an object file built against
 * the pre-merge library still refers to a
 * `nros_action_server_active_goal_count_raw` symbol that no longer exists,
 * and must be recompiled.
 *
 * Define NROS_NO_DEPRECATED_ACTION_ALIASES to compile without it -- for a
 * consumer whose build is `-Werror` and who wants the old name to be a hard
 * error rather than a warning.
 *
 * This is scheduled for removal; migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_ACTION_ALIASES

/* Preserves the OLD tri-state `int32_t` convention exactly:
 *   >= 0                        the count
 *   NROS_RET_INVALID_ARGUMENT   NULL, or not a polling-tier server
 *   NROS_RET_NOT_INIT           built without `rmw-cffi`
 *
 * The tier gate is re-applied HERE, before delegating, because the merged
 * function deliberately answers a callback-tier server too -- and this name
 * promised the polling tier only. Without the gate a `_raw` caller holding
 * an INITIALIZED server would start getting a count where it used to get
 * NROS_RET_INVALID_ARGUMENT.
 *
 * One documented difference, in a build WITHOUT `rmw-cffi` only: the old
 * function returned NROS_RET_NOT_INIT for any non-NULL server regardless of
 * state, where this returns NROS_RET_INVALID_ARGUMENT for a non-POLLING
 * one. The old behaviour disagreed with itself across the feature flag (the
 * `rmw-cffi` build returned NROS_RET_INVALID_ARGUMENT there); the forwarder
 * keeps the meaningful half.
 *
 * The width guard has no reachable failure -- the arena and the inline core
 * both hold a fixed-capacity goal array far below INT32_MAX -- but the old
 * body's bare `as i32` would have wrapped a large count into a value a
 * caller reads as an error code, so the conversion is made explicit rather
 * than inherited.
 */
NROS_DEPRECATED_MSG("nros_action_server_active_goal_count_raw() is deprecated; use "
                    "nros_action_server_get_active_goal_count(server, &count), which "
                    "returns nros_ret_t and serves both tiers")
static inline int32_t
nros_action_server_active_goal_count_raw(struct nros_action_server_t* server) {
    size_t count = 0;
    nros_ret_t ret;

    if (server == NULL || server->state != NROS_ACTION_SERVER_STATE_POLLING) {
        return (int32_t)NROS_RET_INVALID_ARGUMENT;
    }
    ret = nros_action_server_get_active_goal_count(server, &count);
    if (ret != NROS_RET_OK) {
        return (int32_t)ret;
    }
    if (count > (size_t)INT32_MAX) {
        return (int32_t)NROS_RET_ERROR;
    }
    return (int32_t)count;
}

#endif /* NROS_NO_DEPRECATED_ACTION_ALIASES */

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
 * Define NROS_NO_DEPRECATED_ACTION_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_ACTION_ALIASES

NROS_DEPRECATED_MSG("nros_action_client_get_zero_initialized() is deprecated; use "
                    "rcl_action_get_zero_initialized_client()")
static inline struct nros_action_client_t nros_action_client_get_zero_initialized(void) {
    return rcl_action_get_zero_initialized_client();
}

NROS_DEPRECATED_MSG("nros_action_server_get_zero_initialized() is deprecated; use "
                    "rcl_action_get_zero_initialized_server()")
static inline struct nros_action_server_t nros_action_server_get_zero_initialized(void) {
    return rcl_action_get_zero_initialized_server();
}

NROS_DEPRECATED_MSG("nros_action_client_get_default_options() is deprecated; use "
                    "rcl_action_client_get_default_options()")
static inline struct nros_action_client_options_t nros_action_client_get_default_options(void) {
    return rcl_action_client_get_default_options();
}

NROS_DEPRECATED_MSG("nros_action_server_get_default_options() is deprecated; use "
                    "rcl_action_server_get_default_options()")
static inline struct nros_action_server_options_t nros_action_server_get_default_options(void) {
    return rcl_action_server_get_default_options();
}

#endif /* NROS_NO_DEPRECATED_ACTION_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_ACTION_H */
