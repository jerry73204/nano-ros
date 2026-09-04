/**
 * @file rcl_compat.h
 * @brief What is LEFT of the rcl compat layer after phase-417 stage 6.
 *
 * RFC-0089 settled on 2026-09-04 that **the C API takes rcl's spellings**,
 * because the goal is drop-in replacement and a ported file's line is rcl's
 * line. The migration it prescribes is ALIAS first, REPLACE later. Stage 5 was
 * the alias step and this header held twelve `static inline` forwarders;
 * **stage 6 renamed the entry points themselves, so all twelve became IDENTITY
 * and were deleted** — see section 3 for the list and for where each one lives
 * now.
 *
 * That is RFC-0089's "dissolves by construction": a shim exists to bridge two
 * spellings, and when there is one spelling it has nothing to bridge. The
 * deletion is not a judgement call either — an identity `static inline` beside
 * the non-static declaration in `<nros/nros_generated.h>` is a hard
 * `error: static declaration of '...' follows non-static declaration`.
 *
 * ## What does NOT dissolve, and why that is fine
 *
 * TWO things, and neither is a spelling:
 *
 * 1. **The `RCL_RET_*` value MAPPING (section 1).** RFC-0089 forbids
 *    renumbering `nros_ret_t` to rcl's values — that would silently flip the
 *    meaning of every stored return code across three FFI seams. So
 *    `RCL_RET_TIMEOUT` does not go away; it becomes the NAME of our constant,
 *    with our value. A ported `if (ret == RCL_RET_TIMEOUT)` is correct; a
 *    program that hardcodes `if (ret == 2)` is not, and never was.
 * 2. **The handle TYPEDEFS (section 2).** Our structs are still
 *    `struct nros_node_t` and friends, so `rcl_node_t` is still an alias
 *    rather than the name itself. These retire with the TYPE rename, which is
 *    a separate step from the entry-point rename this file records.
 *
 *     #include <nros/rcl_compat.h>   // opt-in; nothing force-includes it
 *
 * ## Scope
 *
 * Freestanding C: no allocator, no libc beyond what `<nros/types.h>` already
 * uses. This header adds no symbol to the link — every remaining item is a
 * typedef or a macro.
 */

#ifndef NROS_RCL_COMPAT_H
#define NROS_RCL_COMPAT_H

/* ── Collision guard ──────────────────────────────────────────────────────
 * The real rcl and this header define the same names with DIFFERENT VALUES
 * (see the mapping table below), so a translation unit holding both would
 * silently take whichever won the include race. Refuse instead of redefining.
 *
 * One-way only: C cannot detect an `<rcl/types.h>` included AFTER this file.
 * If you are linking real rcl, you do not want this header at all.
 *
 * The whole body sits in the `#else` arm so that a collision produces exactly
 * ONE diagnostic. Letting it fall through would bury the message under a
 * cascade of "RCL_RET_OK redefined" warnings, which is the shape of an error
 * a reader skims past. */
#if defined(RCL_RET_OK) || defined(RCL__TYPES_H_)
#error "<nros/rcl_compat.h> collides with the real <rcl/types.h>: both define \
RCL_RET_* and the VALUES differ (nano-ros ERROR/TIMEOUT/INVALID_ARGUMENT/NOT_INIT \
are -1/-2/-3/-7, rcl's are 1/2/11/101). Include one. If you are building against \
a real ROS 2 rcl, drop this header."
#elif defined(RMW_RET_OK) || defined(RMW__RET_TYPES_H_)
#error "<nros/rcl_compat.h> collides with the real <rmw/ret_types.h>: both define \
RMW_RET_* and the VALUES differ. Include one."
#else

#include "nros/types.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * 1. Return codes — the SPELLING is adopted, the VALUES are NOT
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * `nros_ret_t`'s own doc comment claims it is "Compatible with rcl_ret_t for
 * familiarity" (`nros/nros_generated.h:840`). **It is not, and that sentence
 * is wrong.** Only OK agrees:
 *
 *     code                ours   rcl   rcl's source
 *     OK                     0     0   RMW_RET_OK
 *     ERROR                 -1     1   RMW_RET_ERROR
 *     TIMEOUT               -2     2   RMW_RET_TIMEOUT
 *     UNSUPPORTED          -16     3   RMW_RET_UNSUPPORTED
 *     INVALID_ARGUMENT      -3    11   RMW_RET_INVALID_ARGUMENT
 *     NOT_INIT              -7   101   RCL_RET_NOT_INIT
 *
 * (read from `/opt/ros/humble/include/rmw/rmw/ret_types.h:29-38` and
 * `rcl/rcl/types.h:26-41`.)
 *
 * So this header MAPS the constants onto ours. It does NOT renumber
 * `nros_ret_t` — RFC-0089 records this as the one place where taking rcl's
 * spelling must not mean taking rcl's values, because a stored return code
 * crosses three FFI seams (C, C++, the Rust bindings) and renumbering would
 * flip the meaning of every one of them silently. A ported
 * `if (ret == RCL_RET_TIMEOUT)` is true exactly when ours returned a timeout,
 * which is the property that matters; the integer behind it is not.
 *
 * ── Which constants exist here, and why the rest do not ───────────────────
 *
 * Only codes a nano-ros entry point actually returns are defined. That is not
 * laziness: **a constant we define but never return is a branch that compiles
 * and is never taken**, which is precisely the silent difference the
 * compile-or-conform rule forbids. Leaving it undefined makes the ported line
 * fail to compile, which is the loudness the rule requires.
 *
 * Not defined, and the reason:
 *
 * - `RCL_RET_BAD_ALLOC` — rcl means "an allocation failed, and freeing may
 *   let you retry". The nearest nano-ros code is `NROS_RET_FULL`, which means
 *   "a compile-time arena is full; rebuild it larger". Same shape, different
 *   REMEDY, so a ported handler would do the wrong thing while compiling
 *   cleanly. Refused rather than approximated.
 * - `RCL_RET_SUBSCRIPTION_TAKE_FAILED` / `_CLIENT_TAKE_FAILED` /
 *   `_SERVICE_TAKE_FAILED` — our take entry points report "nothing available"
 *   as a byte COUNT (`int32_t`), not as an `nros_ret_t`; there is no code to
 *   map onto.
 * - `RCL_RET_ALREADY_INIT`, `RCL_RET_ALREADY_SHUTDOWN`,
 *   `RCL_RET_*_INVALID`, `RCL_RET_TIMER_CANCELED`, `RCL_RET_WAIT_SET_*`,
 *   `RCL_RET_MISMATCHED_RMW_ID`, the argument-parsing 1XXX block — no
 *   nano-ros entry point returns a distinguishable equivalent. Two of these
 *   had two equally plausible nano-ros targets each, which is itself the
 *   signal not to guess.
 */

/** rcl's return type. Upstream this is `rmw_ret_t`, an `int32_t`; ours is
 *  `nros_ret_t`, an `int`. The width guard is asserted below. */
typedef nros_ret_t rcl_ret_t;
/** Upstream `rcl_ret_t` IS `rmw_ret_t`, and ported rcl code mixes the two
 *  freely, so the alias exists here too. Nothing else of rmw is provided —
 *  this is the rcl compat header, not an rmw one. */
typedef nros_ret_t rmw_ret_t;

/** Success. The one code where ours and rcl's VALUES agree (both 0). */
#define RMW_RET_OK NROS_RET_OK
/** Generic error. rcl's value is 1; ours is -1. Mapped, not renumbered. */
#define RMW_RET_ERROR NROS_RET_ERROR
/** The operation exceeded its timeout. rcl's value is 2; ours is -2. */
#define RMW_RET_TIMEOUT NROS_RET_TIMEOUT
/** The operation is not supported by the active backend. rcl's is 3. */
#define RMW_RET_UNSUPPORTED NROS_RET_UNSUPPORTED
/** An argument to the function was invalid. rcl's value is 11; ours is -3. */
#define RMW_RET_INVALID_ARGUMENT NROS_RET_INVALID_ARGUMENT

/* rcl re-exports the rmw codes under its own names (`rcl/types.h:26-36`), so
 * `RCL_RET_ERROR == RMW_RET_ERROR` holds here exactly as it does upstream. */
#define RCL_RET_OK RMW_RET_OK
#define RCL_RET_ERROR RMW_RET_ERROR
#define RCL_RET_TIMEOUT RMW_RET_TIMEOUT
#define RCL_RET_UNSUPPORTED RMW_RET_UNSUPPORTED
#define RCL_RET_INVALID_ARGUMENT RMW_RET_INVALID_ARGUMENT
/** `rcl_init()` has not been called. rcl's value is 101; ours is -7. rcl-only
 *  (there is no `RMW_RET_NOT_INIT`), so it is spelled only as `RCL_RET_*`. */
#define RCL_RET_NOT_INIT NROS_RET_NOT_INIT

/* ── Drift guards ─────────────────────────────────────────────────────────
 * Two mutations would break the mapping silently, and each has its own
 * assertion:
 *
 *   (a) someone renumbers `nros_ret_t` — the first block pins every mapped
 *       code to the literal it has today, so a moved value fails HERE rather
 *       than at a consumer that stored the old one. Pinning six exact
 *       literals also makes the mapping provably INJECTIVE, so a ported
 *       `switch` over these labels keeps compiling (the probe proves that
 *       half, where a duplicate case label is a hard error).
 *
 *   (b) someone "fixes" the incompatibility by taking rcl's VALUES as well as
 *       its spelling — the second block asserts we did not. This is the
 *       mutation RFC-0089 names as forbidden, and without it the header would
 *       still compile after the change that breaks three FFI seams.
 */
#if defined(__cplusplus) && __cplusplus >= 201103L
#define NROS_RCL_COMPAT_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define NROS_RCL_COMPAT_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#define NROS_RCL_COMPAT_ASSERT(cond, msg)
#endif

/* (a) ours have not moved. */
NROS_RCL_COMPAT_ASSERT(NROS_RET_OK == 0, "NROS_RET_OK moved; rcl_compat.h's mapping is stale");
NROS_RCL_COMPAT_ASSERT(NROS_RET_ERROR == -1,
                       "NROS_RET_ERROR moved; rcl_compat.h's mapping is stale");
NROS_RCL_COMPAT_ASSERT(NROS_RET_TIMEOUT == -2,
                       "NROS_RET_TIMEOUT moved; rcl_compat.h's mapping is stale");
NROS_RCL_COMPAT_ASSERT(NROS_RET_INVALID_ARGUMENT == -3,
                       "NROS_RET_INVALID_ARGUMENT moved; rcl_compat.h's mapping is stale");
NROS_RCL_COMPAT_ASSERT(NROS_RET_NOT_INIT == -7,
                       "NROS_RET_NOT_INIT moved; rcl_compat.h's mapping is stale");
NROS_RCL_COMPAT_ASSERT(NROS_RET_UNSUPPORTED == -16,
                       "NROS_RET_UNSUPPORTED moved; rcl_compat.h's mapping is stale");

/* (b) we took rcl's SPELLINGS and not its VALUES. RFC-0089: renumbering
 *     nros_ret_t would silently flip every stored return code across the C,
 *     C++ and Rust FFI seams. OK is the sole code where the two agree. */
NROS_RCL_COMPAT_ASSERT(RCL_RET_OK == 0, "RCL_RET_OK must be 0 -- the one value ours and rcl share");
NROS_RCL_COMPAT_ASSERT(RCL_RET_ERROR != 1,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0089 -- MAP, never "
                       "renumber (three FFI seams store these)");
NROS_RCL_COMPAT_ASSERT(RCL_RET_TIMEOUT != 2,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0089");
NROS_RCL_COMPAT_ASSERT(RCL_RET_UNSUPPORTED != 3,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0089");
NROS_RCL_COMPAT_ASSERT(RCL_RET_INVALID_ARGUMENT != 11,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0089");
NROS_RCL_COMPAT_ASSERT(RCL_RET_NOT_INIT != 101,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0089");

/* The type alias must be able to hold what upstream's `int32_t` holds. */
NROS_RCL_COMPAT_ASSERT(sizeof(rcl_ret_t) == 4, "rcl_ret_t must be 32-bit, as upstream's rmw_ret_t");

/* ═══════════════════════════════════════════════════════════════════════════
 * 2. Handle types
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * A ported line is `rcl_publisher_t pub = rcl_get_zero_initialized_publisher();`,
 * so the zero-init aliases below are unusable without these.
 *
 * The alias is on the NAME and the LIFECYCLE idiom (caller owns the struct by
 * value, zero-initialises it, passes it to `_init`), which is the same in both
 * APIs. The LAYOUT is nano-ros's: a ported file that reaches into `.impl` or
 * any other upstream member fails to compile, which is correct — RFC-0035 puts
 * the entity internals behind the API on purpose.
 *
 * C has no portable way to deprecate a typedef (see `nros/visibility.h`), so
 * these carry a comment and no attribute, as the repo's convention requires.
 */
typedef struct nros_node_t rcl_node_t;
typedef struct nros_publisher_t rcl_publisher_t;
typedef struct nros_subscription_t rcl_subscription_t;
typedef struct nros_client_t rcl_client_t;
typedef struct nros_service_t rcl_service_t;
typedef struct nros_timer_t rcl_timer_t;
typedef struct nros_guard_condition_t rcl_guard_condition_t;
/** rclc's executor. Spelled `rclc_`, not `rcl_`: the executor is rclc's, and
 *  rcl has no executor type at all. */
typedef struct nros_executor_t rclc_executor_t;
/** rclc's init/context bundle, built once and passed to node and timer init.
 *  Same role and same opacity as ours; `rclc_` for the same reason. */
typedef struct nros_support_t rclc_support_t;

/*
 * NOT aliased: `rosidl_message_type_support_t` / `rosidl_service_type_support_t`.
 *
 * The handle typedefs above are honest because those types are OPAQUE on both
 * sides -- no member is part of the contract, so aliasing the name claims only
 * the lifecycle idiom, and a ported file reaching into `.impl` fails to
 * compile (RFC-0035 puts entity internals behind the API on purpose).
 *
 * A rosidl typesupport is the opposite: its MEMBERS are the contract. Upstream
 * carries `typesupport_identifier`, `data` and the `func` dispatcher that
 * resolves a nested typesupport; ours is a flat descriptor of `type_name`,
 * `type_hash` and `serialized_size_max` (`nros/nros_generated.h:2547`). Same
 * ROLE, different substance, so the name is not taken. The `init` forwarders
 * below therefore spell this parameter `const struct nros_message_type_t *`,
 * which costs a ported call site nothing: the argument it passes comes from
 * OUR codegen either way, because `ROSIDL_GET_MSG_TYPE_SUPPORT` is absent and
 * a ported line that tries to produce one fails to compile.
 */

/* ═══════════════════════════════════════════════════════════════════════════
 * 3. What used to be here: TWELVE forwarders, now IDENTITY and therefore gone
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * phase-417 stage 6 (2026-09-04) renamed the C entry points themselves, so
 * every function forwarder this header used to carry now has the SAME name on
 * both sides. An identity `static inline` is not a shim, it is infinite
 * recursion waiting for `-O0`, and the compiler says so first: a
 * `static inline rcl_get_zero_initialized_node` beside the non-static
 * declaration in `<nros/nros_generated.h>` is
 * `error: static declaration of 'rcl_get_zero_initialized_node' follows
 * non-static declaration`. They are deleted rather than argued away — RFC-0089
 * calls this "dissolving by construction", and it is the property that keeps a
 * compat layer from rotting into permanent debt.
 *
 * The twelve, and where each one now lives natively:
 *
 *   rcl_get_zero_initialized_node                  nros_generated.h (node.rs)
 *   rcl_get_zero_initialized_publisher             nros_generated.h (publisher.rs)
 *   rcl_get_zero_initialized_subscription          nros_generated.h (subscription.rs)
 *   rcl_get_zero_initialized_client                nros_generated.h (service.rs)
 *   rcl_get_zero_initialized_service               nros_generated.h (service.rs)
 *   rcl_get_zero_initialized_timer                 nros_generated.h (timer.rs)
 *   rcl_get_zero_initialized_guard_condition       nros_generated.h (guard_condition.rs)
 *   rclc_executor_get_zero_initialized_executor    nros_generated.h (executor.rs)
 *   rcl_trigger_guard_condition                    nros_generated.h (guard_condition.rs)
 *   rclc_publisher_init_default                    nros_generated.h (publisher.rs)
 *   rclc_client_init_default                       nros_generated.h (service.rs)
 *   rclc_node_init_default                         nros_generated.h (node.rs)  <-- REORDERED
 *
 * `rclc_node_init_default` is the one that moved arguments, and the rename and
 * the reorder landed TOGETHER on purpose. RFC-0089's corrected hazard note:
 *
 *     C   : warning: passing argument 1 of 'f' from incompatible pointer type
 *           ...even under -Wall -Wextra.
 *     C++ : error: cannot convert 'support_t*' to 'const char*'
 *
 * So a C reorder is silent-by-default for exactly the callers it must not be
 * silent for — out-of-tree consumers, who do not build with our flags. A
 * rename makes the stale call fail on the IDENTIFIER, which C does diagnose
 * fatally. The old `nros_node_init` survives as an `NROS_DEPRECATED_MSG`
 * `static inline` in `<nros/node.h>` that names each parameter and forwards in
 * the old order; it is deliberately not a macro, because a macro would forward
 * positionally and silently build a node with its name in the support slot.
 *
 * TWO of rclc's six `_init_default` constructors were refused here as recently
 * as stage 5 and are now native, because the reason was withdrawn rather than
 * worked around:
 *
 *   rclc_subscription_init_default(sub, node, type_support, topic_name)
 *   rclc_service_init_default(service, node, type_support, service_name)
 *
 * Ours used to carry `callback` and `context` in two extra positions, and the
 * ledger attributed that to RFC-0041 and to a rule called
 * `executor-owns-no-entity-storage`. RFC-0089 checked both: RFC-0041's
 * normative content is the DISPATCH MODEL and says nothing about a binding
 * site, and `executor-owns-no-entity-storage` has zero occurrences in
 * `docs/design/` — it is a phrase the ledger invented and then cited ten
 * times. The one real constraint (`c:timer_exchange_callback`) forbids
 * SWAPPING a callback on a live entity, not supplying one at registration.
 *
 * So the callback moved to where rclc puts it:
 *   - typed path: `nros_executor_add_subscription_typed(exec, sub, msg,
 *     deserialize, cb, ctx, invocation)`, which the generated per-type macro
 *     collapses to rclc's six arguments in rclc's order;
 *   - byte path: `nros_executor_add_subscription_raw(exec, sub, cb, ctx,
 *     invocation)` and `nros_executor_add_service_raw(exec, service, cb, ctx)`.
 *     These keep the `nros_` prefix on purpose: rclc's `add_subscription` and
 *     `add_service` deliver DESERIALIZED messages into caller-owned storage,
 *     so a byte-oriented entry point must not wear their names.
 *
 * ── The typesupport parameter still keeps OUR type ────────────────────────
 *
 * `rclc_publisher_init_default` and friends spell their third parameter
 * `const struct nros_message_type_t *` (or `nros_service_type_t`), not
 * `rosidl_message_type_support_t`. Settled 2026-09-04 and unchanged by the
 * rename: a rosidl typesupport's MEMBERS are its contract — the
 * `typesupport_identifier`, the `data`, and the `func` dispatcher that
 * resolves the implementation at runtime — against our flat `type_name` /
 * `type_hash` / `serialized_size_max`. Adopting the name would claim a
 * dispatcher we do not have, and the "compiles and differs" that follows is
 * exactly what RFC-0089 forbids. It costs a ported call site nothing, because
 * the argument comes from OUR codegen either way: `ROSIDL_GET_MSG_TYPE_SUPPORT`
 * does not exist here, so a ported line that tries to produce one fails to
 * compile.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * 4. Still refused: `rclc_timer_init_default`
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The one `_init_default` that did NOT converge, and the reason is the
 * CALLBACK CONTRACT rather than the arity:
 *
 *   rclc: rcl_ret_t rclc_timer_init_default(
 *             rcl_timer_t *timer, rclc_support_t *support,
 *             const uint64_t timeout_ns, const rcl_timer_callback_t callback)
 *   ours: nros_ret_t nros_timer_init(
 *             struct nros_timer_t *timer, const struct nros_support_t *support,
 *             uint64_t period_ns, nros_timer_callback_t callback,
 *             void *context)
 *
 *   rcl_timer_callback_t   = void (*)(rcl_timer_t *, int64_t last_call_time)
 *   nros_timer_callback_t  = void (*)(struct nros_timer_t *, void *context)
 *
 * Both are two-parameter and both lead with the timer; the SECOND parameter
 * carries entirely different information. Upstream hands the callback the time
 * since its last call, we hand it the user's context pointer. A ported timer
 * callback is therefore a different FUNCTION, not a differently-registered
 * one, so dropping our trailing `void *context` to reach rclc's arity would
 * buy a matching argument COUNT over a mismatched contract — which is the
 * "compile and differ" the rule exists to prevent. It happens to fail to
 * compile on assignment (`int64_t` against `void *`), but the refusal does not
 * rest on that: a target where the two are assignable would make the mistake
 * silent.
 *
 * `nros_timer_init` therefore keeps its name, its arity and its `context`, and
 * is NOT deprecated. Closing this needs a decision about what our timer
 * callback receives, not a rename.
 *
 * ── Also still refused, and why, so the absence is a record ───────────────
 *
 * * `rcl_*_fini` for publisher / subscription / client / service / action —
 *   upstream takes `(entity, node)`; ours take `(entity)`. Arity, not
 *   spelling. (`rcl_node_fini`, `rcl_timer_fini`, `rcl_clock_fini`,
 *   `rcl_guard_condition_fini`, `rclc_executor_fini` and `rclc_support_fini`
 *   ARE arity 1 upstream and were adopted.)
 * * `rcl_node_get_domain_id` — upstream writes `size_t *`, ours writes
 *   `uint32_t *`. In C that mismatch is a WARNING, so a ported
 *   `size_t id; rcl_node_get_domain_id(&node, &id);` would compile and leave
 *   half of `id` uninitialised. The out-parameter type is authored at the
 *   CALL SITE, which is what separates this from the typesupport case above.
 * * `rcl_timer_get_period` / `rcl_timer_get_time_until_next_call` — upstream
 *   returns the value through an `int64_t *` out-parameter; ours return it,
 *   and `nros_timer_get_time_until_next_call` additionally takes the current
 *   time IN. Different data flow, not a different word.
 * * `rcl_timer_get_time_since_last_call` — same shape, but `uint64_t *`
 *   against upstream's `int64_t *`; caller-authored out-parameter again.
 * * `rcl_clock_get_now` — upstream writes an `int64_t` nanosecond count, ours
 *   writes a `struct nros_time_t`.
 * * `rcl_clock_init`, `rcl_guard_condition_init`, `rcl_node_resolve_name`,
 *   `rcl_node_get_fully_qualified_name`, `rclc_support_init`,
 *   `rclc_executor_init`, the `rcl_action_*` constructors, the
 *   `rclc_lifecycle_*` family and `rclc_parameter_server_fini` — every one
 *   differs in ARITY, and several take an `rcl_allocator_t` we do not have.
 * * `rcl_take_request` / `rcl_take_response` — upstream hands back a
 *   DESERIALIZED message plus an `rmw_request_id_t` out-struct
 *   (`rcl/service.h:279`, `rcl/client.h:301`); ours are the CDR-byte drains
 *   `nros_service_take_request_raw(service, buf, buf_len, int64_t *seq)` and
 *   `nros_client_take_response(client, buf, buf_len, size_t *out_len)`.
 *   Different arity, different argument order, different data. This is
 *   `PollingSubscription::take()`'s class exactly — a plausible name over an
 *   opposite data contract — and it is the reason RFC-0089 exists.
 * * `RCL_RET_BAD_ALLOC`, `RCL_RET_ALREADY_INIT`, `RCL_RET_*_INVALID`,
 *   `RCL_RET_*_TAKE_FAILED`, `RCL_RET_TIMER_CANCELED`, the wait-set codes —
 *   see the note above the constant block.
 */

/* ═══════════════════════════════════════════════════════════════════════════
 * 6. NOT aliased: the executor's entity registration
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * `rclc_executor_add_subscription{,_with_context}` and
 * `rclc_executor_add_service{,_with_context}` are absent here, and the reason
 * is a missing entry point rather than a naming decision:
 *
 *   rclc: rcl_ret_t rclc_executor_add_subscription(
 *             rclc_executor_t *executor, rcl_subscription_t *subscription,
 *             void *msg, rclc_subscription_callback_t callback,
 *             rclc_executor_handle_invocation_t invocation)
 *   rclc: rcl_ret_t rclc_executor_add_subscription_with_context(
 *             rclc_executor_t *executor, rcl_subscription_t *subscription,
 *             void *msg, rclc_subscription_callback_with_context_t callback,
 *             void *context, rclc_executor_handle_invocation_t invocation)
 *   ours: nros_ret_t nros_executor_add_subscription(
 *             struct nros_executor_t *executor,
 *             struct nros_subscription_t *subscription,
 *             enum nros_executor_handle_invocation_t invocation)
 *                                                      (`nros_generated.h:4612`)
 *
 * Two things are missing, not one. The `void *msg` slot is caller-owned
 * storage for a DESERIALIZED message, and the callback shape follows from it:
 *
 *   rclc_subscription_callback_t              = void (*)(const void *msg)
 *   rclc_subscription_callback_with_context_t = void (*)(const void *msg, void *context)
 *   nros_subscription_callback_t              = void (*)(const uint8_t *data,
 *                                                        size_t len, void *context)
 *                                                      (`nros_generated.h:1774`)
 *
 * Ours delivers CDR BYTES; rclc delivers a typed message into storage the
 * caller supplied. That is RFC-0089's stage-5 item — "typed C subscription
 * delivery needs an `add_subscription` variant that carries caller-owned
 * message storage through the FFI" — and it is Rust-side work, not a wrapper's.
 *
 * So the honest move is to name the shape a faithful alias needs and wait for
 * it, rather than alias onto the byte-oriented entry point. What is needed:
 *
 *   nros_ret_t nros_executor_add_subscription_typed(
 *       struct nros_executor_t *executor,
 *       struct nros_subscription_t *subscription,
 *       void *msg,                                 // caller-owned storage
 *       nros_subscription_typed_callback_t callback,  // void(*)(const void *, void *)
 *       void *context,
 *       enum nros_executor_handle_invocation_t invocation);
 *
 * Given that, `..._with_context` is a direct six-argument alias in this file.
 * The CONTEXT-LESS `rclc_executor_add_subscription` still would not be, and
 * that is worth saying now: adapting a `void (*)(const void *)` to a
 * `void (*)(const void *, void *)` needs a trampoline that REMEMBERS the
 * original pointer, and state in the wrapper is RFC-0019/0020's violation, not
 * an ergonomic. It needs its own FFI slot or it stays refused.
 *
 * The service pair is the same story one entity over: rclc's carries
 * `void *request_msg` and `void *response_msg` and a
 * `void (*)(const void *, void *, void *)` callback, against our
 * `nros_executor_add_service(executor, service)` (`nros_generated.h:4700`)
 * over a `bool (*)(const uint8_t *, size_t, uint8_t *, size_t, size_t *,
 * void *)` bound at creation (`nros_generated.h:2159`) — note ours also
 * RETURNS a value where rclc's returns void.
 *
 * Deliberately not aliased onto anything today. Two arenas for one concept is
 * the drift RFC-0019 exists to prevent.
 */

#endif /* collision guard */

#endif /* NROS_RCL_COMPAT_H */
