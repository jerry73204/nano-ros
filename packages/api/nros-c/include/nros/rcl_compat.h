/**
 * @file rcl_compat.h
 * @brief OPT-IN rcl / rclc spellings for ported ROS 2 C code (RFC-0087).
 *
 * RFC-0087 settled on 2026-09-04 that **the C API takes rcl's spellings**,
 * because the goal is drop-in replacement and a ported file's line is rcl's
 * line. The migration it prescribes is ALIAS first, REPLACE later, and this
 * header is the alias step: our own surface keeps `nros_` until stage 6, and a
 * ported `.c` file meets the names it already writes by including this header.
 *
 *     #include <nros/rcl_compat.h>   // opt-in; nothing force-includes it
 *
 * Nothing here changes the meaning of an existing symbol. Every entry point is
 * a `static inline` forwarder onto the `nros_` function that already
 * implements the contract, so an alias whose argument list drifted from the
 * function it forwards to fails to COMPILE rather than at a call site
 * (RFC-0019/0020: ergonomics may live in the wrapper, behaviour may not — a
 * forwarder emits no logic of its own).
 *
 * ## What is deliberately NOT here
 *
 * RFC-0087's rule is *never compile and differ*. A name whose contract we
 * cannot keep must fail to compile, so it is ABSENT here rather than aliased:
 *
 * - `rcl_take_request` / `rcl_take_response` — upstream hands back a
 *   DESERIALIZED message plus an `rmw_request_id_t` out-struct
 *   (`rcl/service.h:279`, `rcl/client.h:301`); ours are the CDR-byte drains
 *   `nros_service_take_request_raw(service, buf, buf_len, int64_t *seq)` and
 *   `nros_client_take_response(client, buf, buf_len, size_t *out_len)`.
 *   Different arity, different argument order, different data. This is
 *   `PollingSubscription::take()`'s class exactly — a plausible name over an
 *   opposite data contract — and it is the reason RFC-0087 exists.
 *   (`nros_service_take_request` is additionally a permanent
 *   `NROS_RET_NOT_INIT` stub; see `nros/service.h:15`.)
 * - the `rclc_*_init_default` family — three of the six differ in ARITY by
 *   construction — see the refusals recorded beside the three that ARE aliased
 *   in section 5.
 * - `rclc_executor_add_subscription_with_context` /
 *   `..._add_service_with_context` — rclc's variants carry caller-owned
 *   message STORAGE (`void *msg`, `void *request_msg` / `void *response_msg`)
 *   plus a callback receiving a DESERIALIZED message; ours are arity 3 and 2,
 *   over a callback receiving CDR BYTES. The CAPABILITY matches (which is what
 *   the ledger's `disposition: adopt` records); the SIGNATURE does not, and
 *   only a signature can carry a name. Section 6 states the entry point shape
 *   a faithful alias needs.
 * - `RCL_RET_BAD_ALLOC`, `RCL_RET_ALREADY_INIT`, `RCL_RET_*_INVALID`,
 *   `RCL_RET_*_TAKE_FAILED`, `RCL_RET_TIMER_CANCELED`, the wait-set codes —
 *   see the note above the constant block.
 *
 * ## Scope
 *
 * Freestanding C: no allocator, no libc beyond what `<nros/types.h>` already
 * uses. This header adds no symbol to the link — every forwarder is
 * `static inline` and every constant is a macro.
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
 * `nros_ret_t` — RFC-0087 records this as the one place where taking rcl's
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
 *       mutation RFC-0087 names as forbidden, and without it the header would
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

/* (b) we took rcl's SPELLINGS and not its VALUES. RFC-0087: renumbering
 *     nros_ret_t would silently flip every stored return code across the C,
 *     C++ and Rust FFI seams. OK is the sole code where the two agree. */
NROS_RCL_COMPAT_ASSERT(RCL_RET_OK == 0, "RCL_RET_OK must be 0 -- the one value ours and rcl share");
NROS_RCL_COMPAT_ASSERT(RCL_RET_ERROR != 1,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0087 -- MAP, never "
                       "renumber (three FFI seams store these)");
NROS_RCL_COMPAT_ASSERT(RCL_RET_TIMEOUT != 2,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0087");
NROS_RCL_COMPAT_ASSERT(RCL_RET_UNSUPPORTED != 3,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0087");
NROS_RCL_COMPAT_ASSERT(RCL_RET_INVALID_ARGUMENT != 11,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0087");
NROS_RCL_COMPAT_ASSERT(RCL_RET_NOT_INIT != 101,
                       "nros_ret_t was renumbered to rcl's values; see RFC-0087");

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
 * 3. The zero-initialiser family — rcl's free-function spelling
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * rcl spells it `rcl_get_zero_initialized_<thing>(void)` returning the struct
 * BY VALUE; ours is the method-style `nros_<thing>_get_zero_initialized(void)`,
 * also nullary and also by value. Identical shape, identical contract — the
 * ledger records the difference as spelling alone at every site
 * (`docs/reference/api-parity-ledger/pubsub.json` `c:get_zero_initialized_publisher`,
 * `node.json` `c:get_zero_initialized_node`, `timer.json` `c:get_zero_initialized_timer`,
 * `exec.json` `c:get_zero_initialized_guard_condition` and
 * `c:executor_get_zero_initialized_executor`, `service.json`
 * `c:get_zero_initialized_client` / `c:get_zero_initialized_service`).
 *
 * The service/client pair reads `refuse-loud` in `service.json` where the
 * identical class reads `adopt` in four other shards. RFC-0087 identifies that
 * split as "never agent disagreement but an unmade decision surfacing five
 * times", and settles it: all of them are `adopt`. Aliased here on the
 * settlement, having re-read each signature — RFC-0087 also warns that
 * `disposition` records an intent and "a later pass must verify against the
 * header, not against the field".
 *
 * Only the eight types nano-ros actually has are here. rcl also ships
 * `rcl_get_zero_initialized_{context,init_options,arguments,log_levels,event,
 * wait_set,subscription_content_filter_options}` and the seven
 * `rcl_action_get_zero_initialized_*` forms; we have none of those TYPES, so
 * the names stay ABSENT and a ported line naming one fails to compile.
 */

static inline rcl_node_t rcl_get_zero_initialized_node(void) {
    return nros_node_get_zero_initialized();
}

static inline rcl_publisher_t rcl_get_zero_initialized_publisher(void) {
    return nros_publisher_get_zero_initialized();
}

static inline rcl_subscription_t rcl_get_zero_initialized_subscription(void) {
    return nros_subscription_get_zero_initialized();
}

static inline rcl_client_t rcl_get_zero_initialized_client(void) {
    return nros_client_get_zero_initialized();
}

static inline rcl_service_t rcl_get_zero_initialized_service(void) {
    return nros_service_get_zero_initialized();
}

static inline rcl_timer_t rcl_get_zero_initialized_timer(void) {
    return nros_timer_get_zero_initialized();
}

static inline rcl_guard_condition_t rcl_get_zero_initialized_guard_condition(void) {
    return nros_guard_condition_get_zero_initialized();
}

/** rclc's stuttering spelling — the type name appears twice. Ours is the
 *  method-style `nros_executor_get_zero_initialized`. */
static inline rclc_executor_t rclc_executor_get_zero_initialized_executor(void) {
    return nros_executor_get_zero_initialized();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 4. Verbs whose word already matches and only the ORDER differs
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * rcl's free-function spelling of the guard-condition trigger.
 *
 * Upstream: `rcl_ret_t rcl_trigger_guard_condition(rcl_guard_condition_t *)`
 * (`rcl/guard_condition.h:207`). Ours:
 * `nros_ret_t nros_guard_condition_trigger(struct nros_guard_condition_t *)`
 * (`nros/nros_generated.h:5025`). Same arity, same argument, same return
 * kind — the WORD already matches and only our `nros_<entity>_<verb>` order
 * differs, which is what `docs/reference/api-parity-ledger/exec.json`
 * `c:trigger_guard_condition` records (`disposition: adopt`).
 *
 * Thread-safe, as upstream's is.
 */
static inline rcl_ret_t rcl_trigger_guard_condition(rcl_guard_condition_t* guard_condition) {
    return nros_guard_condition_trigger(guard_condition);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 5. rclc's `_init_default` convenience constructors
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * rclc ships six. THREE are aliased here and THREE are refused, and the split
 * is not a judgement call — it falls out of comparing the two signatures. Both
 * halves were read: upstream from the recorded surface
 * `docs/reference/api-surface/rclc.json` (which stores full parameter types
 * for all 95 `rclc_*` and 489 `rcl_*` functions, at the record's top level as
 * `params`), ours from `nros/nros_generated.h`.
 *
 * The pattern in the three refusals is one thing, stated once: nano-ros binds
 * a callback and its `void *context` to the ENTITY at creation (RFC-0041/0043)
 * where rclc hands them to the executor later. So our `_init` takes arguments
 * rclc's `_init_default` does not, and there is no value a forwarder could
 * invent for them. Each is written out below rather than summarised, because
 * "the arities differ" is the kind of claim that gets asserted from memory.
 */

/**
 * rclc's default-QoS publisher constructor.
 *
 *   rclc: rcl_ret_t rclc_publisher_init_default(
 *             rcl_publisher_t *publisher, const rcl_node_t *node,
 *             const rosidl_message_type_support_t *type_support,
 *             const char *topic_name)
 *   ours: nros_ret_t nros_publisher_init(
 *             struct nros_publisher_t *publisher, const struct nros_node_t *node,
 *             const struct nros_message_type_t *type_info,
 *             const char *topic_name)                  (`nros_generated.h:5441`)
 *
 * Same arity, same order, same const-ness, no callback on either side.
 * `nros_publisher_init` takes no QoS and IS the default form
 * (`nros_publisher_init_with_qos` is the explicit one), which is exactly what
 * `pubsub.json c:publisher_init_default` records. Aliased.
 */
static inline rcl_ret_t rclc_publisher_init_default(rcl_publisher_t* publisher,
                                                    const rcl_node_t* node,
                                                    const struct nros_message_type_t* type_support,
                                                    const char* topic_name) {
    return nros_publisher_init(publisher, node, type_support, topic_name);
}

/**
 * rclc's default-QoS service-client constructor.
 *
 *   rclc: rcl_ret_t rclc_client_init_default(
 *             rcl_client_t *client, const rcl_node_t *node,
 *             const rosidl_service_type_support_t *type_support,
 *             const char *service_name)
 *   ours: nros_ret_t nros_client_init(
 *             struct nros_client_t *client, const struct nros_node_t *node,
 *             const struct nros_service_type_t *type_info,
 *             const char *service_name)                (`nros_generated.h:5988`)
 *
 * Same arity, same order. A client has no callback in either API — it is the
 * SERVICE side that does — so the RFC-0041/0043 divergence does not reach
 * here. Aliased.
 *
 * Note this row reads `refuse-loud` in `service.json` where the identical
 * class reads `adopt` in four other shards; RFC-0087 settles that split as
 * `adopt`. Verified against the signature, not taken from the field.
 */
static inline rcl_ret_t rclc_client_init_default(rcl_client_t* client, const rcl_node_t* node,
                                                 const struct nros_service_type_t* type_support,
                                                 const char* service_name) {
    return nros_client_init(client, node, type_support, service_name);
}

/**
 * rclc's default-options node constructor. **The arguments are REORDERED.**
 *
 *   rclc: rcl_ret_t rclc_node_init_default(
 *             rcl_node_t *node, const char *name, const char *namespace_,
 *             rclc_support_t *support)
 *   ours: nros_ret_t nros_node_init(
 *             struct nros_node_t *node, const struct nros_support_t *support,
 *             const char *name, const char *namespace_)
 *                                                      (`nros_generated.h:5290`)
 *
 * Same arity, same four things, different ORDER: rclc puts `support` last,
 * ours puts it second. That is precisely why this is a `static inline` naming
 * each parameter and not a macro — a macro alias would forward the arguments
 * positionally and silently build a node with its name and namespace swapped
 * into the support slot. Here the compiler checks each one, and
 * `const char *` against `const struct nros_support_t *` cannot be confused.
 *
 * Ours takes no node options at all, so it already IS the default form
 * (`node.json c:node_init_default`). A ported file wanting non-default options
 * reaches `nros_node_init_ex`, which is `rcl_node_init_with_options`'s
 * counterpart (`c:node_init_with_options`).
 */
static inline rcl_ret_t rclc_node_init_default(rcl_node_t* node, const char* name,
                                               const char* namespace_, rclc_support_t* support) {
    return nros_node_init(node, support, name, namespace_);
}

/*
 * ── The three refused, with both signatures ──────────────────────────────
 *
 * `rclc_subscription_init_default` — arity 4 against our 6.
 *
 *   rclc: rcl_ret_t rclc_subscription_init_default(
 *             rcl_subscription_t *subscription, rcl_node_t *node,
 *             const rosidl_message_type_support_t *type_support,
 *             const char *topic_name)
 *   ours: nros_ret_t nros_subscription_init(
 *             struct nros_subscription_t *subscription,
 *             const struct nros_node_t *node,
 *             const struct nros_message_type_t *type_info,
 *             const char *topic_name,
 *             nros_subscription_callback_t callback,
 *             void *context)                           (`nros_generated.h:6309`)
 *
 * The callback and its context are ours to require and rclc's to defer to
 * `rclc_executor_add_subscription`. A forwarder has nothing to pass for them.
 *
 * `rclc_service_init_default` — arity 4 against our 6, the same way.
 *
 *   rclc: rcl_ret_t rclc_service_init_default(
 *             rcl_service_t *service, const rcl_node_t *node,
 *             const rosidl_service_type_support_t *type_support,
 *             const char *service_name)
 *   ours: nros_ret_t nros_service_init(
 *             struct nros_service_t *service, const struct nros_node_t *node,
 *             const struct nros_service_type_t *type_info,
 *             const char *service_name,
 *             nros_service_callback_t callback,
 *             void *context)                           (`nros_generated.h:5758`)
 *
 * `rclc_timer_init_default` — arity 4 against our 5, AND the callback contract
 * differs, which is the sharper of the two reasons.
 *
 *   rclc: rcl_ret_t rclc_timer_init_default(
 *             rcl_timer_t *timer, rclc_support_t *support,
 *             const uint64_t timeout_ns, const rcl_timer_callback_t callback)
 *   ours: nros_ret_t nros_timer_init(
 *             struct nros_timer_t *timer, const struct nros_support_t *support,
 *             uint64_t period_ns, nros_timer_callback_t callback,
 *             void *context)                           (`nros_generated.h:6725`)
 *
 * Note the callbacks, because the trailing `void *context` is the LESS
 * interesting difference:
 *
 *   rcl_timer_callback_t   = void (*)(rcl_timer_t *, int64_t last_call_time)
 *   nros_timer_callback_t  = void (*)(struct nros_timer_t *, void *context)
 *                                                      (`nros_generated.h:2104`)
 *
 * Both are two-parameter, both lead with the timer, and the SECOND parameter
 * carries entirely different information: upstream hands the callback the time
 * since its last call, we hand it the user's context pointer. A ported timer
 * callback is therefore a different function, not a differently-registered
 * one. It happens to fail to compile on assignment (`int64_t` against
 * `void *`), which is the loudness the rule wants — but the refusal is not
 * resting on that, because a target where the two are assignable would make
 * the mistake silent.
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
 * caller supplied. That is RFC-0087's stage-5 item — "typed C subscription
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
