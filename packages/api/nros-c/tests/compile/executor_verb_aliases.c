/*
 * Issue 0338 — the executor's entity-registration family is spelled
 * `nros_executor_add_*`, matching rclc. It used to be `register_*` for every
 * kind EXCEPT `add_client`, so the family was not internally consistent and a C
 * user could not guess the verb.
 *
 * This TU pins BOTH halves of that change:
 *   1. the new `add_*` spelling exists for every kind, and
 *   2. the deprecated `register_*` aliases still compile, so a consumer gets one
 *      release to migrate.
 *
 * Compile-only (no main): the assertion is that these names resolve with these
 * signatures. Taking a function pointer forces a real lookup — a macro alias
 * that pointed at a nonexistent symbol would fail here.
 */

#include "nros/executor.h"

/* 1. The rclc-spelled verbs exist. */
static nros_ret_t (*const kAddTimer)(struct nros_executor_t*,
                                     struct nros_timer_t*) = rclc_executor_add_timer;
static nros_ret_t (*const kAddService)(struct nros_executor_t*,
                                       struct nros_service_t*) = nros_executor_add_service;
static nros_ret_t (*const kAddClient)(struct nros_executor_t*,
                                      struct nros_client_t*) = nros_executor_add_client;
static nros_ret_t (*const kAddGuard)(struct nros_executor_t*, struct nros_guard_condition_t*) =
    nros_executor_add_guard_condition;

/* 2. The deprecated aliases still resolve, to the SAME functions. */
static nros_ret_t (*const kOldTimer)(struct nros_executor_t*,
                                     struct nros_timer_t*) = nros_executor_register_timer;
/* `nros_executor_register_timer` is the one member of this family whose target
 * moved. Phase-417 stage 6 made `nros_executor_add_timer` a deprecated
 * forwarder onto `rclc_executor_add_timer`, so for one release the macro
 * resolved through TWO deprecations; step B retired that forwarder and the
 * macro now names `rclc_executor_add_timer` directly. It therefore no longer
 * warns on the way through — the 0338 aliases have their own retirement, and
 * until it lands this spelling is silent. Every other target below is a live
 * exported symbol. The assertion here is that the name still RESOLVES with
 * this signature. */
static nros_ret_t (*const kOldService)(struct nros_executor_t*,
                                       struct nros_service_t*) = nros_executor_register_service;
static nros_ret_t (*const kOldGuard)(struct nros_executor_t*, struct nros_guard_condition_t*) =
    nros_executor_register_guard_condition;

/* Silence "defined but not used" without needing a main(). */
const void* nros_executor_verb_alias_probe(void);
const void* nros_executor_verb_alias_probe(void) {
    (void)kAddTimer;
    (void)kAddService;
    (void)kAddClient;
    (void)kAddGuard;
    (void)kOldTimer;
    (void)kOldService;
    (void)kOldGuard;
    /* The alias must name the same entity as the new spelling. */
    return (kOldTimer == kAddTimer && kOldService == kAddService && kOldGuard == kAddGuard)
               ? (const void*)kAddTimer
               : (const void*)0;
}
