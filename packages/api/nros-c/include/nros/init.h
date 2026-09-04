/**
 * @file init.h
 * @ingroup grp_init
 * @brief Library initialisation and support context.
 *
 * The support context (`nros_support_t`) is the entry point for all
 * nros operations.  It manages the middleware session (zenoh-pico) and
 * must be initialised before any nodes, publishers, or subscriptions
 * are created.
 *
 * Typical usage:
 * @code
 * nros_support_t support = nros_support_get_zero_initialized();
 * nros_support_init(&support, NULL, 0);
 * // ... create nodes, publishers, etc.
 * nros_support_fini(&support);
 * @endcode
 */

#ifndef NROS_INIT_H
#define NROS_INIT_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/init.h>` continues to compile. */
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
 * Define NROS_NO_DEPRECATED_INIT_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_INIT_ALIASES

NROS_DEPRECATED_MSG("nros_support_fini() is deprecated; use rclc_support_fini()")
static inline nros_ret_t nros_support_fini(struct nros_support_t* support) {
    return rclc_support_fini(support);
}

#endif /* NROS_NO_DEPRECATED_INIT_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_INIT_H */
