/**
 * @file clock.h
 * @ingroup grp_clock
 * @brief Clock and time utilities.
 *
 * Provides clocks for reading wall-clock or monotonic time, and
 * arithmetic helpers for `nros_time_t` and `nros_duration_t`.
 */

#ifndef NROS_CLOCK_H
#define NROS_CLOCK_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/clock.h>` continues to compile. */
#include "nros/types.h"

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
 * Define NROS_NO_DEPRECATED_CLOCK_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_CLOCK_ALIASES

NROS_DEPRECATED_MSG("nros_clock_fini() is deprecated; use rcl_clock_fini()")
static inline nros_ret_t nros_clock_fini(struct nros_clock_t* clock) {
    return rcl_clock_fini(clock);
}

NROS_DEPRECATED_MSG("nros_clock_time_started() is deprecated; use rcl_clock_time_started()")
static inline bool nros_clock_time_started(const struct nros_clock_t* clock) {
    return rcl_clock_time_started(clock);
}

NROS_DEPRECATED_MSG(
    "nros_enable_ros_time_override() is deprecated; use rcl_enable_ros_time_override()")
static inline nros_ret_t nros_enable_ros_time_override(const struct nros_clock_t* clock) {
    return rcl_enable_ros_time_override(clock);
}

NROS_DEPRECATED_MSG(
    "nros_disable_ros_time_override() is deprecated; use rcl_disable_ros_time_override()")
static inline nros_ret_t nros_disable_ros_time_override(const struct nros_clock_t* clock) {
    return rcl_disable_ros_time_override(clock);
}

NROS_DEPRECATED_MSG(
    "nros_is_enabled_ros_time_override() is deprecated; use rcl_is_enabled_ros_time_override()")
static inline nros_ret_t nros_is_enabled_ros_time_override(const struct nros_clock_t* clock,
                                                           bool* is_enabled) {
    return rcl_is_enabled_ros_time_override(clock, is_enabled);
}

NROS_DEPRECATED_MSG("nros_set_ros_time_override() is deprecated; use rcl_set_ros_time_override()")
static inline nros_ret_t nros_set_ros_time_override(const struct nros_clock_t* clock,
                                                    int64_t nanoseconds) {
    return rcl_set_ros_time_override(clock, nanoseconds);
}

#endif /* NROS_NO_DEPRECATED_CLOCK_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_CLOCK_H */
