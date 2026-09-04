/*
 * Phase-417 stage 6 — the C entry points ARE rcl's spellings now.
 *
 * This file began at W5.b as a probe over `<nros/rcl_compat.h>`'s twelve
 * `static inline` forwarders. Stage 6 renamed the entry points themselves, so
 * the forwarders became IDENTITY and were deleted; every name below now
 * resolves to the exported symbol directly, and the header contributes only
 * the `RCL_RET_*` mapping and the handle typedefs.
 *
 * What the file tests did not change, which is the point — a ported rcl/rclc
 * file writes exactly these lines, and it must compile either way.
 *
 * This TU is written the way a PORTED rcl/rclc file is written: it names rcl's
 * types, rcl's zero-initialisers, rcl's guard-condition verb and rcl's return
 * codes, and never spells `nros_` except at the one seam where it calls an
 * entry point that has no rcl counterpart. If the alias layer is right, this
 * compiles; if an alias drifted from the function it forwards to, it does not.
 *
 * It pins four things, in increasing order of what they would cost to lose:
 *
 *   1. every alias RESOLVES with the signature the header documents. Taking a
 *      function POINTER forces a real lookup and a real signature match, the
 *      way `receive_verb_aliases.c` and `executor_verb_aliases.c` do — a macro
 *      that quietly reordered arguments would still pass a call-shaped test.
 *
 *   2. `if (ret == RCL_RET_TIMEOUT)` is true exactly when a REAL nano-ros call
 *      timed out. `nros_executor_ping` is the call: its own header documents
 *      `NROS_RET_TIMEOUT` as the no-reply-within-budget code
 *      (`nros/nros_generated.h:4292`). This is the property RFC-0087 says must
 *      survive the spelling change, and the only one a porting user can
 *      observe. (`ping` has no rcl name — micro-ROS spells it
 *      `rmw_uros_ping_agent` — so it is called under ours.)
 *
 *   3. the mapping is INJECTIVE. A `switch` over the mapped codes is the proof:
 *      C makes a duplicate case label a hard error, so two rcl codes collapsing
 *      onto one nano-ros value fails HERE. That matters because a ported file's
 *      dispatch is usually a switch, and the collapse would otherwise show up
 *      as one arm silently shadowing another.
 *
 *   4. the VALUES stayed ours. RFC-0087 forbids renumbering `nros_ret_t` to
 *      rcl's numbers — a stored return code crosses the C, C++ and Rust FFI
 *      seams, so renumbering flips all three at once with nothing to read. The
 *      header asserts this too; asserted again from the CONSUMER side because
 *      that is where the damage lands.
 *
 * Compile-only (no main): the assertion is that these names resolve with these
 * types and these values.
 */

/*
 * Passing the wrong pointer type is a WARNING in C, not an error, so a
 * forwarder whose BODY mis-wires its arguments compiles cleanly under plain
 * `-std=c11` -- and under `-Wall -Wextra` too. That was measured, not assumed:
 * mutating `rclc_node_init_default` to forward positionally (the way a macro
 * alias would, dropping the reorder onto `nros_node_init`) left this TU at
 * rc=0 with only `note: expected 'const struct nros_support_t *' but argument
 * is of type 'const char *'` to show for it.
 *
 * Since the reorder is the whole reason that alias is a `static inline` and
 * not a macro, the check that guards it cannot be advisory. Promoted here
 * rather than in the lane's flags so the guard travels with the probe: a check
 * that only works when the caller remembers to pass `-Werror=...` is one
 * copy-paste away from being decorative.
 *
 * Scoped to this TU. The header imposes nothing on consumers.
 */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic error "-Wincompatible-pointer-types"
#endif

#include "nros/rcl_compat.h"

/* ── 1. Every alias resolves, with the documented signature ─────────────── */

static rcl_node_t (*const k_zero_node)(void) = rcl_get_zero_initialized_node;
static rcl_publisher_t (*const k_zero_pub)(void) = rcl_get_zero_initialized_publisher;
static rcl_subscription_t (*const k_zero_sub)(void) = rcl_get_zero_initialized_subscription;
static rcl_client_t (*const k_zero_cli)(void) = rcl_get_zero_initialized_client;
static rcl_service_t (*const k_zero_srv)(void) = rcl_get_zero_initialized_service;
static rcl_timer_t (*const k_zero_timer)(void) = rcl_get_zero_initialized_timer;
static rcl_guard_condition_t (*const k_zero_guard)(void) = rcl_get_zero_initialized_guard_condition;
static rclc_executor_t (*const k_zero_exec)(void) = rclc_executor_get_zero_initialized_executor;
static rcl_ret_t (*const k_trigger)(rcl_guard_condition_t*) = rcl_trigger_guard_condition;

/*
 * The rclc `_init_default` constructors. Pointer-taking matters most for
 * `rclc_node_init_default`: it is the one entry point in the surface that is a
 * pure PERMUTATION of its upstream counterpart, and under the `#pragma` above
 * a reverted reorder is a hard ERROR here rather than the warning C would
 * otherwise emit. `rclc_timer_init_default` remains deliberately absent -- see
 * section 4 of the header for both signatures and the callback-contract
 * reason.
 */
static rcl_ret_t (*const k_pub_init_default)(rcl_publisher_t*, const rcl_node_t*,
                                             const struct nros_message_type_t*,
                                             const char*) = rclc_publisher_init_default;
static rcl_ret_t (*const k_cli_init_default)(rcl_client_t*, const rcl_node_t*,
                                             const struct nros_service_type_t*,
                                             const char*) = rclc_client_init_default;
static rcl_ret_t (*const k_node_init_default)(rcl_node_t*, const char*, const char*,
                                              const rclc_support_t*) = rclc_node_init_default;

/*
 * The two that stage 6 moved from REFUSED to native. Their arity used to be 6
 * against rclc's 4, because ours carried `callback` and `context`; RFC-0087
 * withdrew the reason (RFC-0041 governs the DISPATCH MODEL, not a binding
 * site, and `executor-owns-no-entity-storage` is defined nowhere), so the
 * callback moved to registration where rclc puts it. Pinning the pointer here
 * is what makes a silent re-growth of the argument list fail.
 */
static rcl_ret_t (*const k_sub_init_default)(rcl_subscription_t*, const rcl_node_t*,
                                             const struct nros_message_type_t*,
                                             const char*) = rclc_subscription_init_default;
static rcl_ret_t (*const k_srv_init_default)(rcl_service_t*, const rcl_node_t*,
                                             const struct nros_service_type_t*,
                                             const char*) = rclc_service_init_default;

/*
 * ...and the registration entry points that now carry the callback. These keep
 * the `nros_` prefix deliberately: rclc's `rclc_executor_add_subscription` and
 * `rclc_executor_add_service` deliver a DESERIALIZED message into caller-owned
 * storage, so a byte-oriented entry point must not wear their names
 * (RFC-0087's compile-or-conform rule).
 */
static nros_ret_t (*const k_add_sub_raw)(
    struct nros_executor_t*, struct nros_subscription_t*, nros_subscription_callback_t, void*,
    enum nros_executor_handle_invocation_t) = nros_executor_add_subscription_raw;
static nros_ret_t (*const k_add_srv_raw)(struct nros_executor_t*, struct nros_service_t*,
                                         nros_service_callback_t,
                                         void*) = nros_executor_add_service_raw;

/* ── 2. A ported file's own shape: rcl types, rcl zero-init, rcl codes ─── */

/*
 * The line a ported rcl file writes. `rcl_publisher_t pub =
 * rcl_get_zero_initialized_publisher();` is unusable without BOTH halves of
 * the alias -- the type and the free function -- so this statement is the
 * whole idiom under test, not a spot check of either.
 */
rcl_ret_t nros_rcl_compat_ported_shape(struct nros_executor_t* executor, rclc_support_t* support_in,
                                       const struct nros_message_type_t* msg_type,
                                       const struct nros_service_type_t* srv_type);
rcl_ret_t nros_rcl_compat_ported_shape(struct nros_executor_t* executor, rclc_support_t* support_in,
                                       const struct nros_message_type_t* msg_type,
                                       const struct nros_service_type_t* srv_type) {
    rclc_support_t support = *support_in;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_client_t client = rcl_get_zero_initialized_client();
    rcl_service_t service = rcl_get_zero_initialized_service();
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rcl_guard_condition_t guard = rcl_get_zero_initialized_guard_condition();
    rclc_executor_t zero_executor = rclc_executor_get_zero_initialized_executor();
    rcl_ret_t ret;

    (void)node;
    (void)publisher;
    (void)timer;
    (void)zero_executor;

    /* rcl's free-function verb over our method-style implementation. */
    ret = rcl_trigger_guard_condition(&guard);
    if (ret != RCL_RET_OK) {
        return ret;
    }

    /*
     * rclc's constructors, called in RCLC's argument order -- which for
     * `rclc_node_init_default` is NOT ours. Writing the call the ported way
     * round is the point: if the forwarder ever stopped reordering, `name`
     * would land in the support slot and this stops compiling.
     */
    ret = rclc_node_init_default(&node, "ported_node", "/", &support);
    if (ret != RCL_RET_OK) {
        return ret;
    }
    ret = rclc_publisher_init_default(&publisher, &node, msg_type, "chatter");
    if (ret != RCL_RET_OK) {
        return ret;
    }
    ret = rclc_client_init_default(&client, &node, srv_type, "add_two_ints");
    if (ret != RCL_RET_OK) {
        return ret;
    }
    /* rclc's four-argument subscription and service constructors: no callback
     * here, exactly as upstream. */
    ret = rclc_subscription_init_default(&subscription, &node, msg_type, "chatter");
    if (ret != RCL_RET_OK) {
        return ret;
    }
    ret = rclc_service_init_default(&service, &node, srv_type, "add_two_ints");
    if (ret != RCL_RET_OK) {
        return ret;
    }

    /*
     * THE assertion of this file, in the form a ported program writes it.
     * `nros_executor_ping` documents NROS_RET_TIMEOUT for "no reply within
     * budget"; the ported line tests RCL_RET_TIMEOUT and must be true for
     * exactly that outcome. The mapping is what makes the two the same
     * comparison -- NOT a renumbering of what ping returns.
     */
    ret = nros_executor_ping(executor, 100);
    if (ret == RCL_RET_TIMEOUT) {
        return RCL_RET_TIMEOUT;
    }
    if (ret == RCL_RET_UNSUPPORTED) {
        return RCL_RET_UNSUPPORTED;
    }
    if (ret == RCL_RET_NOT_INIT) {
        return RCL_RET_NOT_INIT;
    }
    if (ret == RCL_RET_INVALID_ARGUMENT) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    return ret == RCL_RET_OK ? RCL_RET_OK : RCL_RET_ERROR;
}

/* ── 3. The mapping is injective: a duplicate case label is a hard error ── */

/*
 * The compiler is the checker here. If two RCL_RET_* constants ever mapped
 * onto the same nano-ros value, this switch would fail with "duplicate case
 * value" and name both labels. Written as a switch precisely because that is
 * also the shape a ported dispatch takes: a collapse would otherwise appear as
 * one arm silently shadowing another at run time.
 */
int nros_rcl_compat_code_is_distinct(rcl_ret_t ret);
int nros_rcl_compat_code_is_distinct(rcl_ret_t ret) {
    switch (ret) {
    case RCL_RET_OK:
        return 0;
    case RCL_RET_ERROR:
        return 1;
    case RCL_RET_TIMEOUT:
        return 2;
    case RCL_RET_UNSUPPORTED:
        return 3;
    case RCL_RET_INVALID_ARGUMENT:
        return 4;
    case RCL_RET_NOT_INIT:
        return 5;
    default:
        return -1;
    }
}

/* rcl re-exports the rmw codes under its own names, so the two spellings must
 * name the same value here exactly as they do upstream (`rcl/types.h:26-36`).
 * Checked with the RMW_ spellings a ported file may equally have written. */
_Static_assert(RCL_RET_OK == RMW_RET_OK, "RCL_RET_OK must be RMW_RET_OK, as upstream");
_Static_assert(RCL_RET_ERROR == RMW_RET_ERROR, "RCL_RET_ERROR must be RMW_RET_ERROR, as upstream");
_Static_assert(RCL_RET_TIMEOUT == RMW_RET_TIMEOUT,
               "RCL_RET_TIMEOUT must be RMW_RET_TIMEOUT, as upstream");
_Static_assert(RCL_RET_INVALID_ARGUMENT == RMW_RET_INVALID_ARGUMENT,
               "RCL_RET_INVALID_ARGUMENT must be RMW_RET_INVALID_ARGUMENT, as upstream");

/* ── 4. Values are OURS, mapped -- not rcl's, renumbered ────────────────── */

/*
 * Asserted from the consumer side as well as inside the header, because this
 * is the mutation RFC-0087 singles out: renumbering `nros_ret_t` to rcl's
 * values would keep every one of the aliases above compiling while flipping
 * the meaning of every return code already stored across the C, C++ and Rust
 * FFI seams. The header would still be self-consistent; only a check that
 * names the OLD values catches it.
 */
_Static_assert(RCL_RET_TIMEOUT == NROS_RET_TIMEOUT,
               "RCL_RET_TIMEOUT must MAP to the code a nano-ros timeout returns");
_Static_assert(RCL_RET_TIMEOUT == -2, "a nano-ros timeout is -2; rcl's own RCL_RET_TIMEOUT is 2");
_Static_assert(RCL_RET_ERROR == -1, "a nano-ros error is -1; rcl's own RCL_RET_ERROR is 1");
_Static_assert(RCL_RET_INVALID_ARGUMENT == -3,
               "ours is -3; rcl's own RCL_RET_INVALID_ARGUMENT is 11");
_Static_assert(RCL_RET_NOT_INIT == -7, "ours is -7; rcl's own RCL_RET_NOT_INIT is 101");
_Static_assert(RCL_RET_UNSUPPORTED == -16, "ours is -16; rcl's own RCL_RET_UNSUPPORTED is 3");
_Static_assert(RCL_RET_OK == 0, "OK is the ONE code where ours and rcl's values agree");

/* Silence "defined but not used" without needing a main(). */
const void* nros_rcl_compat_alias_probe(void);
const void* nros_rcl_compat_alias_probe(void) {
    (void)k_zero_node;
    (void)k_zero_pub;
    (void)k_zero_sub;
    (void)k_zero_cli;
    (void)k_zero_srv;
    (void)k_zero_timer;
    (void)k_zero_guard;
    (void)k_zero_exec;
    (void)k_trigger;
    (void)k_pub_init_default;
    (void)k_cli_init_default;
    (void)k_node_init_default;
    (void)k_sub_init_default;
    (void)k_srv_init_default;
    (void)k_add_sub_raw;
    (void)k_add_srv_raw;
    return (const void*)k_trigger;
}
