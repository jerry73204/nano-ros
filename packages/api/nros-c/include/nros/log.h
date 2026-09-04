/**
 * @file log.h
 * @ingroup grp_log
 * @brief Phase 88.12 — ROS 2 style leveled logging surface for the C API.
 *
 * Mirrors the Rust [`nros_log`] facade. Each `nros_node_t` exposes a
 * `&'static nros_log::Logger` via `nros_node_get_logger(node)`; the
 * `NROS_LOG_*` macros render the message, attach severity + line
 * info, and dispatch through the same per-platform sink chain as
 * Rust call sites (see Phase 88.5 onwards).
 *
 * Usage:
 * @code
 * nros_logger_t logger = nros_node_get_logger(&node);
 * NROS_LOG_INFO(logger, "started; domain=%u", domain_id);
 * NROS_LOG_WARN(logger, "queue depth %u exceeds soft limit", depth);
 * @endcode
 *
 * Per-platform delivery (POSIX stderr / Zephyr LOG / ESP-IDF
 * esp_log_write / NuttX syslog / FreeRTOS+ThreadX+bare-metal board
 * fn-ptr) is invisible at the C API layer — the dispatcher routes
 * through `nros_platform_log_write` (declared in
 * `<nros/platform.h>`).
 */

#ifndef NROS_LOG_H
#define NROS_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Severity levels — match `nros_log::Severity::as_u8()` discriminant.
 * Lower value = more verbose.
 */
typedef enum nros_log_severity_t {
    NROS_LOG_SEVERITY_TRACE = 0,
    NROS_LOG_SEVERITY_DEBUG = 1,
    NROS_LOG_SEVERITY_INFO = 2,
    NROS_LOG_SEVERITY_WARN = 3,
    NROS_LOG_SEVERITY_ERROR = 4,
    NROS_LOG_SEVERITY_FATAL = 5,
} nros_log_severity_t;

/**
 * Opaque handle to a `&'static nros_log::Logger`. Obtain via
 * `nros_node_get_logger(...)`. NEVER free.
 */
typedef const void* nros_logger_t;

/**
 * Low-level emit. Renders `message` (already-formatted UTF-8 text;
 * NOT null-terminated by contract — pass an explicit length) at
 * `severity` through the dispatcher.
 *
 * Most users should use the `NROS_LOG_*` printf-style macros below
 * rather than calling this directly.
 *
 * @param logger    Logger handle from `nros_node_get_logger`. NULL =
 *                  silently drops (kept total to simplify call sites).
 * @param severity  One of `nros_log_severity_t`.
 * @param message   UTF-8 text; not required to be null-terminated.
 * @param message_len  Length of `message` in bytes.
 */
void nros_log_emit(nros_logger_t logger, nros_log_severity_t severity, const char* message,
                   size_t message_len);

/**
 * Internal helper used by the macros. Formats `fmt + args` into a
 * stack buffer, then calls `nros_log_emit`. Buffer size = 256 bytes;
 * overflow is truncated + appended `...`.
 */
void nros_log_emit_fmt(nros_logger_t logger, nros_log_severity_t severity, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * phase-417 W4.d — emit WITH the call site.
 *
 * `nros_log_emit` above hardcoded `file = "<nros-c>"`, `line = 0` and
 * `timestamp_ns = 0` into every record, contradicting this file's own header
 * comment ("attach severity + line info"). Every C and C++ record in the tree
 * therefore reached the sinks stripped of the two fields the sinks render.
 *
 * @param file  Must have STATIC storage duration — `__FILE__`. Borrowed for the
 *              life of the program, never copied. NULL = unknown.
 * @param line  `__LINE__`.
 *
 * The timestamp is NOT a parameter: it comes from `nros_log`'s own clock, so a
 * C record is stamped by the same clock at the same resolution as a Rust one.
 * See `nros_log_timestamp_available()`.
 */
void nros_log_emit_at(nros_logger_t logger, nros_log_severity_t severity, const char* message,
                      size_t message_len, const char* file, uint32_t line);

/** Stack buffer the located printf-style entry point below renders into. */
#define NROS_LOG_FMT_BUFFER_SIZE 256

/**
 * phase-417 W4.d — printf-style emit carrying `__FILE__` / `__LINE__`.
 *
 * `static inline` rather than a sibling of `nros_log_emit_fmt` in
 * `c-stubs/log_fmt.c` because the variadic entry point cannot be written in
 * Rust (`c_variadic` is unstable) and this header is the C surface's own file.
 * The cost is one copy of ~30 instructions per translation unit that logs; the
 * de-duplication — moving this beside `nros_log_emit_fmt` and having that one
 * forward with `file = NULL` — is a follow-up in `c-stubs/log_fmt.c`.
 *
 * `nros_log_emit_fmt` is UNCHANGED and still exported: it is the ABI existing
 * consumers link, and it simply carries no call site.
 */
static inline void nros_log_emit_fmt_at(nros_logger_t logger, nros_log_severity_t severity,
                                        const char* file, uint32_t line, const char* fmt, ...)
    __attribute__((format(printf, 5, 6)));

static inline void nros_log_emit_fmt_at(nros_logger_t logger, nros_log_severity_t severity,
                                        const char* file, uint32_t line, const char* fmt, ...) {
    if (logger == NULL || fmt == NULL) {
        return;
    }
    char buf[NROS_LOG_FMT_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (written <= 0) {
        return;
    }
    size_t n = (size_t)written;
    if (n >= sizeof(buf)) {
        /* `vsnprintf` returns the would-have-written length on overflow but
         * writes at most `sizeof(buf) - 1` bytes. Append "..." inside the
         * buffer to mark truncation — same convention as `nros_log_emit_fmt`. */
        n = sizeof(buf) - 1;
        buf[n - 3] = '.';
        buf[n - 2] = '.';
        buf[n - 1] = '.';
    }
    nros_log_emit_at(logger, severity, buf, n, file, line);
}

/* ---- Convenience macros ---- */
/* The macros stage the printf args into a stack buffer inside
 * `nros_log_emit_fmt_at`, which attaches `__FILE__` / `__LINE__`.
 * Below-ceiling filtering happens on the Rust side via the per-logger
 * threshold (`nros_logger_set_level`); no compile-time gating on the C surface
 * (use `if (...)` guards if you need it). */

#define NROS_LOG_TRACE(logger, ...)                                                                \
    nros_log_emit_fmt_at((logger), NROS_LOG_SEVERITY_TRACE, __FILE__, (uint32_t)__LINE__,          \
                         __VA_ARGS__)
#define NROS_LOG_DEBUG(logger, ...)                                                                \
    nros_log_emit_fmt_at((logger), NROS_LOG_SEVERITY_DEBUG, __FILE__, (uint32_t)__LINE__,          \
                         __VA_ARGS__)
#define NROS_LOG_INFO(logger, ...)                                                                 \
    nros_log_emit_fmt_at((logger), NROS_LOG_SEVERITY_INFO, __FILE__, (uint32_t)__LINE__,           \
                         __VA_ARGS__)
#define NROS_LOG_WARN(logger, ...)                                                                 \
    nros_log_emit_fmt_at((logger), NROS_LOG_SEVERITY_WARN, __FILE__, (uint32_t)__LINE__,           \
                         __VA_ARGS__)
#define NROS_LOG_ERROR(logger, ...)                                                                \
    nros_log_emit_fmt_at((logger), NROS_LOG_SEVERITY_ERROR, __FILE__, (uint32_t)__LINE__,          \
                         __VA_ARGS__)
#define NROS_LOG_FATAL(logger, ...)                                                                \
    nros_log_emit_fmt_at((logger), NROS_LOG_SEVERITY_FATAL, __FILE__, (uint32_t)__LINE__,          \
                         __VA_ARGS__)

/**
 * Phase 88.16.H — explicit installation of the default sink list.
 *
 * Cross-language `no_std` builds (FreeRTOS / NuttX / ThreadX C and
 * C++ examples) must call this once after the board crate's
 * platform-log writer is registered (typically right after
 * `nros_node_init` returns). Pins `nros_log::init` as a linker root
 * so `--gc-sections` keeps it; without this, the lazy guard inside
 * `nros_log_emit` can be unreachable and every record silently
 * drops.
 *
 * Idempotent. Hosted POSIX consumers can skip this — the
 * `.init_array` ctors already wire the dispatcher.
 */
void nros_log_init(void);

/**
 * Phase 88.16.H — opaque handle to the catch-all `nros` logger.
 *
 * Lets C callers emit through `NROS_LOG_*` without standing up a
 * full `nros_node_t` (useful for boot diagnostics, panic hooks,
 * smoke fixtures). The returned handle is `'static`. Never free.
 *
 * @return `nros_logger_t` for the default ("nros") logger.
 */
nros_logger_t nros_log_default_logger(void);

/* -------------------------------------------------------------------------
 * phase-417 W4.d — named loggers, per-logger levels, sinks, throttle.
 *
 * Every entry point below is a thin forwarder onto `nros_log` (RFC-0019: the
 * Rust API is the implementation, C delegates). No level table, no throttle
 * arithmetic and no sink list lives on this side.
 * ------------------------------------------------------------------------- */

/**
 * Look up a logger by name, CREATING one if the name is new.
 *
 * `rclcpp::get_logger("x")`'s shape. Until phase-417 the C surface had only
 * `nros_log_default_logger()`, so every C node in an image shared one logger
 * and therefore one threshold.
 *
 * Total — always returns a usable handle, `'static`, never freed. A NULL or
 * empty name, a name longer than 48 bytes, or an exhausted logger arena answers
 * the CATCH-ALL logger and says so at WARN through that logger. Check the log
 * before calling `nros_logger_set_level` on a handle you did not verify: the
 * catch-all's threshold is shared with every other unnamed logger.
 *
 * Arena size is `nros-log`'s `dynamic-loggers-<N>` feature (default 16).
 *
 * @param name  NUL-terminated UTF-8. Copied; need not outlive the call.
 */
nros_logger_t nros_log_get_logger(const char* name);

/**
 * Copy `logger`'s name into `buf` as a NUL-terminated string.
 *
 * @return The name length in bytes EXCLUDING the NUL, whether or not it fit —
 *         `snprintf`'s convention, so one call can size a buffer. 0 for a NULL
 *         logger. Writes nothing when `buf` is NULL or `buf_len` is 0.
 *
 * A copy rather than a borrowed pointer because the Rust side holds a
 * length-delimited `&str`: its pointer is not NUL-terminated, and handing it
 * out would hand out a string every C function reads past the end of.
 */
size_t nros_logger_get_name(nros_logger_t logger, char* buf, size_t buf_len);

/**
 * Set `logger`'s runtime severity threshold. Records below it are dropped
 * before any sink sees them.
 *
 * @return false for a NULL handle, true otherwise.
 */
bool nros_logger_set_level(nros_logger_t logger, nros_log_severity_t severity);

/**
 * `logger`'s runtime severity threshold.
 *
 * A NULL handle answers `NROS_LOG_SEVERITY_FATAL` — the quiet end — so a
 * dropped handle cannot read as "trace everything".
 */
nros_log_severity_t nros_logger_get_level(nros_logger_t logger);

/**
 * Whether a record at `severity` would pass `logger`'s threshold.
 *
 * The `NROS_LOG_*_THROTTLE` macros call this so the severity test runs BEFORE
 * the throttle window: a record the level filters must not consume the window.
 * NULL handle answers false.
 */
bool nros_logger_is_enabled(nros_logger_t logger, nros_log_severity_t severity);

/**
 * A C log sink — see `nros_log_add_sink`.
 *
 * Flat arguments rather than a `const nros_log_record_t *` on purpose: a struct
 * crossing this FFI would be hand-mirrored (cbindgen skips the logging module),
 * and a mirror that misses an appended field passes a SHORTER struct than the
 * reader reads — issue 0160's class, which has bitten three times. With flat
 * arguments an appended field changes this TYPE, so every sink fails to compile
 * instead of reading stack garbage.
 *
 * `logger_name`, `message` and `file` are pointer + length and are NOT
 * NUL-terminated. They are valid only for the duration of the call.
 *
 * `timestamp_ns` is 0 when no monotonic clock is linked — see
 * `nros_log_timestamp_available()`.
 */
typedef void (*nros_log_sink_fn)(void* user_data, nros_log_severity_t severity,
                                 const char* logger_name, size_t logger_name_len,
                                 const char* message, size_t message_len, const char* file,
                                 size_t file_len, uint32_t line, uint64_t timestamp_ns);

/**
 * Install a C callback as a log sink.
 *
 * Until phase-417 the C surface could not install a sink at all: `nros_log_init`
 * takes no arguments and hardcodes the platform default. This APPENDS, so a
 * consumer teeing records to its own transport does not discard whatever the
 * board installed.
 *
 * Records raised before any sink existed are replayed into the first one
 * installed, so a sink registered at boot still sees the boot story.
 *
 * @param sink       The callback. NULL installs nothing and returns false.
 * @param user_data  Passed back untouched, never dereferenced here. Must
 *                   outlive the program: a sink cannot be removed (the read
 *                   path is lock-free).
 * @return false when `sink` is NULL or the registry is full. CHECK IT — a sink
 *         that was silently not installed is indistinguishable from one that
 *         never fires.
 */
bool nros_log_add_sink(nros_log_sink_fn sink, void* user_data);

/**
 * Whether records carry a real `timestamp_ns` rather than a constant 0.
 *
 * False means `nros-log` was built without its `platform-clock` feature, so
 * nothing derived from the timestamp — the throttle windows above all — has a
 * time base. See `nros_log_throttle_admit`.
 */
bool nros_log_timestamp_available(void);

/**
 * Decide whether a throttled call site may emit now, and arm its window.
 *
 * The `NROS_LOG_*_THROTTLE` macros call this; a hand-written call site can too.
 *
 * @param last_ns      Caller-owned storage — a function-scope
 *                     `static uint64_t x = 0;`, one per call site. Caller-owned
 *                     rather than a handle into a pool because a pool needs a
 *                     bound, and a throttled call site is not a resource anyone
 *                     should have to budget.
 * @param interval_ms  Minimum gap between emitted records at this site.
 * @return true when the record should be emitted. The FIRST record at a site
 *         always is (as in `rclcpp`). NULL `last_ns` returns true: a broken
 *         caller gets every record, never silence.
 *
 * The rule itself is `nros_log::throttle_admits` — the same function the Rust
 * `nros_*_throttle!` macros use. Only the storage differs.
 *
 * Without a monotonic clock every timestamp is 0, the window can never elapse,
 * and every record is emitted; the first such call reports that at WARN rather
 * than degrading quietly.
 */
bool nros_log_throttle_admit(uint64_t* last_ns, uint64_t interval_ms);

/* ---- Throttled convenience macros ---- */
/* One `static uint64_t` per call site, which is what makes the window
 * per-SITE and not per-logger — the same shape as `rclcpp`'s
 * `RCLCPP_INFO_THROTTLE`. Statements, not expressions (they declare storage),
 * so they need their own `;` and cannot be used inside a comma expression.
 *
 * Severity is tested BEFORE the window: a record the logger's level already
 * filters must not consume the window, or a raised-then-lowered log level
 * loses its first record for a whole interval. */

#define NROS_LOG_THROTTLE_AT(logger, severity, interval_ms, ...)                                   \
    do {                                                                                           \
        static uint64_t _nros_log_throttle_last = 0;                                               \
        if (nros_logger_is_enabled((logger), (severity)) &&                                        \
            nros_log_throttle_admit(&_nros_log_throttle_last, (uint64_t)(interval_ms))) {          \
            nros_log_emit_fmt_at((logger), (severity), __FILE__, (uint32_t)__LINE__, __VA_ARGS__); \
        }                                                                                          \
    } while (0)

#define NROS_LOG_TRACE_THROTTLE(logger, interval_ms, ...)                                          \
    NROS_LOG_THROTTLE_AT((logger), NROS_LOG_SEVERITY_TRACE, (interval_ms), __VA_ARGS__)
#define NROS_LOG_DEBUG_THROTTLE(logger, interval_ms, ...)                                          \
    NROS_LOG_THROTTLE_AT((logger), NROS_LOG_SEVERITY_DEBUG, (interval_ms), __VA_ARGS__)
#define NROS_LOG_INFO_THROTTLE(logger, interval_ms, ...)                                           \
    NROS_LOG_THROTTLE_AT((logger), NROS_LOG_SEVERITY_INFO, (interval_ms), __VA_ARGS__)
#define NROS_LOG_WARN_THROTTLE(logger, interval_ms, ...)                                           \
    NROS_LOG_THROTTLE_AT((logger), NROS_LOG_SEVERITY_WARN, (interval_ms), __VA_ARGS__)
#define NROS_LOG_ERROR_THROTTLE(logger, interval_ms, ...)                                          \
    NROS_LOG_THROTTLE_AT((logger), NROS_LOG_SEVERITY_ERROR, (interval_ms), __VA_ARGS__)
#define NROS_LOG_FATAL_THROTTLE(logger, interval_ms, ...)                                          \
    NROS_LOG_THROTTLE_AT((logger), NROS_LOG_SEVERITY_FATAL, (interval_ms), __VA_ARGS__)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NROS_LOG_H */
