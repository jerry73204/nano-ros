/**
 * @file guard_condition.h
 * @ingroup grp_executor
 * @brief Manual wake-up trigger API.
 *
 * Guard conditions provide a mechanism for signalling an executor from
 * outside the middleware (e.g., from an ISR or another thread).
 */

#ifndef NROS_GUARD_CONDITION_H
#define NROS_GUARD_CONDITION_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/guard_condition.h>` continues to compile. */
#include "nros/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 * DEPRECATED spellings -- phase-417 stage 6 (RFC-0089, 2026-09-04)
 *
 * RFC-0089 settled that **the C API takes rcl's spellings**: the goal is
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
 * `nros_ret_t`'s VALUES are unchanged. RFC-0089 records this as the one place
 * where taking rcl's spelling must not mean taking rcl's numbering; the
 * mapping lives in `<nros/rcl_compat.h>` and is the one part of that header
 * which does not dissolve.
 *
 * Define NROS_NO_DEPRECATED_GUARD_CONDITION_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_GUARD_CONDITION_ALIASES

NROS_DEPRECATED_MSG("nros_guard_condition_get_zero_initialized() is deprecated; use "
                    "rcl_get_zero_initialized_guard_condition()")
static inline struct nros_guard_condition_t nros_guard_condition_get_zero_initialized(void) {
    return rcl_get_zero_initialized_guard_condition();
}

NROS_DEPRECATED_MSG("nros_guard_condition_fini() is deprecated; use rcl_guard_condition_fini()")
static inline nros_ret_t nros_guard_condition_fini(struct nros_guard_condition_t* guard) {
    return rcl_guard_condition_fini(guard);
}

NROS_DEPRECATED_MSG(
    "nros_guard_condition_trigger() is deprecated; use rcl_trigger_guard_condition()")
static inline nros_ret_t nros_guard_condition_trigger(struct nros_guard_condition_t* guard) {
    return rcl_trigger_guard_condition(guard);
}

#endif /* NROS_NO_DEPRECATED_GUARD_CONDITION_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_GUARD_CONDITION_H */
