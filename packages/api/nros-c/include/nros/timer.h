/**
 * @file timer.h
 * @ingroup grp_executor
 * @brief Periodic timer API.
 *
 * Create timers with nros_timer_init() and register them with an
 * executor via nros_executor_add_timer().
 */

#ifndef NROS_TIMER_H
#define NROS_TIMER_H

/* Type and function definitions live in <nros/nros_generated.h>.
 * This per-module header is kept as a thin shim so existing code that
 * does `#include <nros/timer.h>` continues to compile. */
#include "nros/types.h"

/**
 * @brief Compute the duration between two time points: `finish - start`.
 *
 * rcl's `rcl_difference_times(const rcl_time_point_t *start,
 * const rcl_time_point_t *finish, rcl_duration_t *delta)` — **Time - Time ->
 * Duration**. Ledger row `c:difference_times` (phase-379 W5, group B) records
 * why this was a gap rather than a rename: `nros_time_sub` is Time - Duration
 * -> Time, a different operation with different operands and a different
 * return type, so there was never a rename to make. C++ ships the operation as
 * `Time::operator-(const Time &) -> Duration` and Rust as
 * `impl Sub<Time> for Time`; only C had no way to get a duration out of two
 * timestamps.
 *
 * A `static inline` rather than an exported symbol because there is no state
 * behind it: this is arithmetic on two values the caller already holds.
 * Nothing is computed here that is not already an entry point — the
 * decode is `nros_time_to_nanoseconds` and the encode is
 * `nros_duration_from_nanoseconds`, which is the ONE place the
 * `builtin_interfaces` `{sec, nanosec}` split (floor division, non-negative
 * remainder, issue 0799) is implemented. Open-coding the split here is what
 * the C++ `Duration::to_msg` used to do, and it got negatives wrong.
 *
 * ### Overflow, and how this differs from rcl
 *
 * The nanosecond subtraction **cannot** overflow. `nros_time_t` is
 * `{int32_t sec, uint32_t nanosec}`, so a decoded time point is bounded by
 * roughly ±2.15e18 ns and any difference of two of them by ±4.3e18 ns —
 * inside `int64_t`'s ±9.22e18. rcl has no such bound: `rcl_time_point_t`
 * carries a bare `int64_t` nanosecond count, and `rcl_difference_times`
 * subtracts two of them with plain signed arithmetic (`rcl/src/rcl/time.c`,
 * `rcl_difference_times`), which is undefined behaviour at the extremes of
 * the type. Our narrower time encoding removes the case rather than checking
 * for it.
 *
 * What *can* exceed its type is the RESULT once re-encoded: a duration of
 * ±4.3e9 seconds does not fit `nros_duration_t`'s `int32_t sec`.
 * `nros_duration_from_nanoseconds` SATURATES there — it clamps to the
 * nearest representable duration rather than wrapping, which keeps the
 * encoding monotone (see its own note on `split_nanoseconds`). It is the only
 * behaviour available to a function with no error channel, and it is the same
 * choice every other `_time_` entry point already makes.
 *
 * ### Divergence from rcl, deliberate
 *
 * rcl returns `RCL_RET_ERROR` when the two points carry different
 * `clock_type`s. Ours cannot: ledger row `c:time_point_t` records that
 * `nros_time_t` is the value alone and the clock kind is a property of the
 * clock it was read from, not of every timestamp — which halves a timestamp
 * on the wire and in a message. Differencing two times read from different
 * clocks is therefore the caller's obligation here, and the header says so
 * rather than the function pretending to check.
 *
 * @param[in]  start  The time point for the start of the duration.
 * @param[in]  finish The time point for the end of the duration.
 * @param[out] delta  `finish - start`. Negative when `finish` precedes
 *                    `start`, encoded the way `builtin_interfaces/msg/Duration`
 *                    requires (`{sec: -2, nanosec: 3e8}` is -1.7 s).
 * @return `NROS_RET_OK`, or `NROS_RET_INVALID_ARGUMENT` if any argument is
 *         NULL.
 */
static inline nros_ret_t nros_difference_times(const struct nros_time_t* start,
                                               const struct nros_time_t* finish,
                                               struct nros_duration_t* delta) {
    if (start == NULL || finish == NULL || delta == NULL) {
        return NROS_RET_INVALID_ARGUMENT;
    }
    /* Both decodes go through the exported entry point; see the note above on
     * why neither the decode nor the encode is spelled out here. */
    *delta = nros_duration_from_nanoseconds(nros_time_to_nanoseconds(finish) -
                                            nros_time_to_nanoseconds(start));
    return NROS_RET_OK;
}

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
 * Define NROS_NO_DEPRECATED_TIMER_ALIASES to compile without any of it --
 * for a consumer whose build is `-Werror` and who wants the old names to be a
 * hard error rather than a warning.
 *
 * Scheduled for removal as ONE batch (stage 6 step B); migrate.
 * =================================================================== */

#ifndef NROS_NO_DEPRECATED_TIMER_ALIASES

NROS_DEPRECATED_MSG(
    "nros_timer_get_zero_initialized() is deprecated; use rcl_get_zero_initialized_timer()")
static inline struct nros_timer_t nros_timer_get_zero_initialized(void) {
    return rcl_get_zero_initialized_timer();
}

NROS_DEPRECATED_MSG("nros_timer_fini() is deprecated; use rcl_timer_fini()")
static inline nros_ret_t nros_timer_fini(struct nros_timer_t* timer) {
    return rcl_timer_fini(timer);
}

NROS_DEPRECATED_MSG("nros_timer_cancel() is deprecated; use rcl_timer_cancel()")
static inline nros_ret_t nros_timer_cancel(struct nros_timer_t* timer) {
    return rcl_timer_cancel(timer);
}

NROS_DEPRECATED_MSG("nros_timer_reset() is deprecated; use rcl_timer_reset()")
static inline nros_ret_t nros_timer_reset(struct nros_timer_t* timer) {
    return rcl_timer_reset(timer);
}

NROS_DEPRECATED_MSG("nros_timer_is_canceled() is deprecated; use rcl_timer_is_canceled()")
static inline nros_ret_t nros_timer_is_canceled(const struct nros_timer_t* timer,
                                                bool* is_canceled) {
    return rcl_timer_is_canceled(timer, is_canceled);
}

NROS_DEPRECATED_MSG("nros_timer_is_ready() is deprecated; use rcl_timer_is_ready()")
static inline nros_ret_t nros_timer_is_ready(const struct nros_timer_t* timer, bool* is_ready) {
    return rcl_timer_is_ready(timer, is_ready);
}

#endif /* NROS_NO_DEPRECATED_TIMER_ALIASES */

#ifdef __cplusplus
}
#endif

#endif /* NROS_TIMER_H */
