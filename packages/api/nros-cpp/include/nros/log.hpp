// nros-cpp: lightweight logging macros
// Freestanding C++ — no STL, opt-in to stdio via NROS_CPP_STD or
// hosted-build detection.

/**
 * @file log.hpp
 * @ingroup grp_misc
 * @brief Phase 123.B.7 — `NROS_INFO` / `NROS_WARN` / `NROS_ERROR` /
 *        `NROS_DEBUG` printf-style log macros.
 *
 * Routes through a single configurable sink. By default, on hosted
 * builds (`__STDC_HOSTED__` or `NROS_CPP_STD` defined) the sink
 * writes to `stderr` with a `[level] file:line — fmt…` prefix.
 * Embedded builds without stdio fall through to a no-op so the
 * macros compile away.
 *
 * Override the sink with `#define NROS_LOG_SINK(level, file, line, fmt, ...)`
 * before including this header (or via `-DNROS_LOG_SINK=…`) to
 * route through `defmt`, semihosting, Zephyr's `LOG_INF`, etc.
 *
 * The macros take a `printf`-style format string + variadic
 * arguments. They evaluate `fmt` and the variadics exactly once.
 */

#ifndef NROS_CPP_LOG_HPP
#define NROS_CPP_LOG_HPP

#ifndef NROS_LOG_SINK
#if defined(NROS_CPP_STD) || (__STDC_HOSTED__ + 0)
#include <cstdio>
#define NROS_LOG_SINK(level, file, line, ...)                                                      \
    do {                                                                                           \
        ::std::fprintf(stderr, "[" level "] %s:%d ", (file), (line));                              \
        ::std::fprintf(stderr, __VA_ARGS__);                                                       \
        ::std::fputc('\n', stderr);                                                                \
    } while (0)
#else
#define NROS_LOG_SINK(level, file, line, ...) ((void)(level), (void)(file), (void)(line))
#endif
#endif

/// Print an INFO-level log line.
#define NROS_INFO(...) NROS_LOG_SINK("INFO", __FILE__, __LINE__, __VA_ARGS__)
/// Print a WARN-level log line.
#define NROS_WARN(...) NROS_LOG_SINK("WARN", __FILE__, __LINE__, __VA_ARGS__)
/// Print an ERROR-level log line.
#define NROS_ERROR(...) NROS_LOG_SINK("ERROR", __FILE__, __LINE__, __VA_ARGS__)
/// Print a DEBUG-level log line. Compiled out when `NDEBUG` is set.
#ifdef NDEBUG
#define NROS_DEBUG(...) ((void)0)
#else
#define NROS_DEBUG(...) NROS_LOG_SINK("DEBUG", __FILE__, __LINE__, __VA_ARGS__)
#endif

/* ---- Phase 88.12 — node-/logger-keyed surface ----
 *
 * The macros above are legacy (Phase 123.B.7) — file:line-prefixed
 * stderr printf with no per-logger routing. The macros below carry
 * a Logger handle through to the post-Phase-88 dispatcher
 * (`nros_log_emit_fmt` → per-platform sinks, see
 * `<nros/platform.h>` for the ABI).
 *
 * Obtain the handle from a Node via `node.get_logger()`:
 *
 * ```cpp
 * nros::Node node;
 * NROS_TRY(nros::create_node(node, "my_node"));
 * auto logger = node.get_logger();
 * NROS_LOG_INFO(logger, "started; domain=%u", 42);
 * ```
 *
 * Below-threshold filtering happens runtime-side via the
 * `nros_log::Logger`'s `set_level`; compile-time filtering is via
 * `nros-log/max-level-*` Cargo features (compiled into the nros-c
 * staticlib that ships `nros_log_emit_fmt`). */

/* `<nros/log.h>` already defines the six Phase-88 macros
 * (`NROS_LOG_TRACE` … `NROS_LOG_FATAL`) — include it here so C++
 * call sites pick up the same definitions without re-emitting
 * them (would trigger -Wmacro-redefined on identical-but-redeclared
 * macros). */
#include <nros/log.h>

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0087 stage 6, step A)
// ============================================================================
//
// These names used to live in `nros/rclcpp_compat.hpp`, a separate source-compat
// shim a ported file had to be force-included with. RFC-0087 §"Naming: replace,
// with alias as the migration step" makes the ROS 2 spelling a FIRST-CLASS name
// declared by the API headers themselves, at which point the shim has nothing
// left to bridge and dissolves by construction. `nros::` is untouched: both
// spellings work, and deprecating one is step B.
//
// This header carries the REFUSAL VOCABULARY as well as the logging surface,
// and the two belong together for a reason RFC-0087 states: a REFUSE-LOUD name
// EMITS NO CODE — it is a diagnostic, and this is nano-ros's diagnostic header.
// It is also the only header the umbrella pulls before everything else while
// including nothing of ours, so `qos.hpp`, `options.hpp`, `service.hpp` and the
// umbrella can all reach one definition of `refuse` without a cycle and without
// a second spelling of "dependent false".

// `<string>` for `rclcpp::get_logger(const std::string&)` and `<sstream>` for
// the `_STREAM` family. Gated: this header is reachable from a `no_std` C++
// build with a minimal libcpp, where neither exists. `__has_include`, not
// `__STDC_HOSTED__` — issue 0112, rationale in `publisher.hpp`. When they are
// absent the names below simply do not exist, which is what a freestanding
// target should see; the un-gated half (the refusal vocabulary, `Logger`, the
// printf-style macros) still reaches it.
#if defined(NROS_CPP_STD)
#include <string>
#define NROS_CPP_HAS_STD_STRING 1
#elif defined(__has_include)
#if __has_include(<string>)
#include <string>
#define NROS_CPP_HAS_STD_STRING 1
#endif
#endif

#if defined(NROS_CPP_STD)
#include <sstream>
#define NROS_CPP_HAS_STD_SSTREAM 1
#elif defined(__has_include)
#if __has_include(<sstream>)
#include <sstream>
#define NROS_CPP_HAS_STD_SSTREAM 1
#endif
#endif

namespace rclcpp {

// --- REFUSE-LOUD infrastructure (RFC-0087 stage 3) ---------------------------
//
// RFC-0087's rule:
//
//   An upstream name may be adopted only if its observable contract is the
//   same, or strictly weaker in a documented, non-inverting way. A contract
//   that inverts, or silently drops data or configuration, must fail to
//   COMPILE. Never compile and differ.
//
// A refusal is per-CONCEPT, not per-symbol — the ten inert `NodeOptions`
// accessors share ONE message. The name EXISTS rather than being absent,
// because `no member named 'use_intra_process_comms'` is honest and teaches
// nothing, while a diagnostic that names the constraint AND the nano-ros
// alternative is the migration, delivered at the point of failure.
//
// Why a dependent `static_assert` and not `= delete`: these headers are parsed
// as C++14 (`just check cpp` compiles them and their probes with `-std=c++14`),
// where a deleted function carries NO message — `= delete("reason")` is C++26.
// A `static_assert` inside a template body fires on INSTANTIATION, so the name
// stays declarable and only USING it fails, with the full text attached.
namespace detail {

/// Dependent `false`. `static_assert(refuse<T>::value, …)` in a template body
/// is ill-formed only once that template is instantiated — which is exactly
/// "the name exists, calling it fails".
///
/// ONE definition, reached by every refusal in the C++ API: `qos.hpp`'s
/// `SystemDefaultsQoS`, `options.hpp`'s `NodeOptions` accessors, the
/// `throttle_is_refused` below, and the shared_ptr service/client callback
/// overloads on `rclcpp::Node` in `nros.hpp`.
template <typename T> struct refuse {
    static const bool value = false;
};

} // namespace detail

} // namespace rclcpp

// The diagnostics are macros so one literal backs every site that shares a
// concept (C++14 `static_assert` takes a string LITERAL, not a constexpr
// variable, so a `constexpr const char*` cannot be used here). They live
// together because they are one vocabulary; each is USED in the header that
// declares the name it refuses.
//
// Every one of them contains the marker `REFUSED by nano-ros`, which is what
// `just check cpp`'s expected-failure lane greps for: an expected-failure
// compile cannot tell "the refusal fired" from "the file is not there", so the
// exit code alone proves nothing.

#define NROS_RCLCPP_REFUSE_NODE_OPTIONS                                                            \
    "rclcpp::NodeOptions' option setters and getters are REFUSED by nano-ros "                     \
    "(RFC-0087, phase-417 W3.f). Each one used to store its argument in a private field that "     \
    "NOTHING read and return *this, so the idiomatic chained call compiled and configured "        \
    "nothing -- a silent drop of configuration, which the compile-or-conform rule requires to "    \
    "fail to compile instead. nano-ros resolves parameters and remaps in the LAUNCHER "            \
    "(`nros launch` / play_launch, RFC-0060) and projects them into the process environment "      \
    "before exec; it has no runtime ComponentManager, no intra-process transport, no topic "       \
    "statistics collector and no /rosout topic, so there is nothing for these knobs to switch. "   \
    "Use: node->declare_parameter<T>(name, default) for parameter overrides; the launch file "     \
    "for remaps; and drop the option chain. `rclcpp::NodeOptions{}` itself still constructs, so "  \
    "the `Node(name, options)` constructor shape a composable node needs keeps compiling."

#define NROS_RCLCPP_REFUSE_INIT_ARGV                                                               \
    "rclcpp::init(argc, argv) was given --ros-args, which nano-ros cannot honour "                 \
    "(RFC-0087, phase-417 W3.b). Proceeding would DISCARD it, so `-r chatter:=/other` would "      \
    "silently become a wrong-topic bug at runtime -- the 'compiles and differs' the rule "         \
    "forbids. Nothing in this process parses --ros-args yet: nros::init_with_launch_auto(argc, "   \
    "argv) discards them too (node.hpp:1025-1027), and honouring them is remap resolution -- "     \
    "RFC-0020 violation class 4 -- so the parser belongs beside nros::resolve_name, not in this "  \
    "header. Today remaps and parameter overrides come from the LAUNCHER, which projects them "    \
    "into the environment before exec. Call the zero-argument rclcpp::init(), or "                 \
    "nros::init_with_launch_auto(0, nullptr, \"my_session\") for the launch-aware entry point."

#define NROS_RCLCPP_REFUSE_SYSTEM_DEFAULTS_QOS                                                     \
    "rclcpp::SystemDefaultsQoS is REFUSED by nano-ros (RFC-0087 W3.f, issue 0829). Upstream's "    \
    "rmw_qos_profile_system_default names NO concrete policy: every field is a sentinel meaning "  \
    "'let the RMW decide', and the two reference RMWs resolve the depth sentinel differently "     \
    "(rmw_cyclonedds_cpp -> KEEP_LAST 1, rmw_zenoh_cpp -> 42). nros::QoS has no sentinel, "        \
    "deliberately: the backend is linked at build time, so there is no middleware to defer to. "   \
    "Any value this could return would be a concrete profile wearing the name of an absent one, "  \
    "and it used to return QoS(10) -- which is rmw_qos_profile_DEFAULT, a different upstream "     \
    "profile. Name the policy you want: rclcpp::QoS(10) for the ROS default, "                     \
    "rclcpp::SensorDataQoS(), rclcpp::ServicesQoS(), or nros::QoS().best_effort().keep_last(1)."

#define NROS_RCLCPP_REFUSE_THROTTLE                                                                \
    "RCLCPP_*_THROTTLE is REFUSED by nano-ros (RFC-0087 W3.a, issue 1019). It expanded to the "    \
    "plain RCLCPP_* macro with `clock` and the period left UNEVALUATED, so a 1 Hz throttle "       \
    "logged at loop rate and a side-effecting clock expression was dropped entirely. There is no " \
    "throttle on the C or C++ logging path; nros-log has one Rust-side and re-exporting it is "    \
    "phase-417 W4.d, so a throttle written here would be a second implementation of behaviour "    \
    "Rust already owns (RFC-0019). Rate-limit at the call site, or use the un-throttled "          \
    "RCLCPP_INFO / RCLCPP_WARN / RCLCPP_ERROR."

#define NROS_RCLCPP_REFUSE_SHARED_PTR_SERVICE_CALLBACK                                             \
    "the shared_ptr service-callback shape is REFUSED by nano-ros (RFC-0087, phase-417 W2.c). "    \
    "rclcpp's create_service/create_client callback takes std::shared_ptr<Request> and "           \
    "std::shared_ptr<Response> (plus a request header), which needs a per-request heap "           \
    "allocation on the delivery path. nano-ros has no allocator there (RFC-0022) and hands the "   \
    "request and response BY REFERENCE into caller-owned storage instead, so adopting that "       \
    "signature would mean a second delivery path. Change the handler to "                          \
    "void(const S::Request&, S::Response&) for a service, or void(const S::Response&) for a "      \
    "client -- a plain function pointer or a capture-less lambda -- or take the poll-style "       \
    "overload create_service<S>(name, qos) / create_client<S>(name, qos) and drain it from your "  \
    "spin loop."

namespace rclcpp {

// --- Logger surface ----------------------------------------------------------
//
// `rclcpp::Logger` in upstream is a pull-through to the rcl logger. Here it is
// a name-only sentinel; the log macros below dispatch through NROS_*, which
// already carry the file/line. The logger NAME is lost (nros has no per-logger
// dispatch yet). Documented; a follow-up can teach nros::log a tag.

class Logger {
  public:
    explicit Logger(const char* name = "") : name_(name) {}
    const char* get_name() const { return name_; }

  private:
    const char* name_;
};

inline Logger get_logger(const char* name) {
    return Logger(name);
}

#ifdef NROS_CPP_HAS_STD_STRING
/// `std::string`-keyed overload. Present only where `<string>` is — a
/// freestanding target has no `std::string` to take.
inline Logger get_logger(const std::string& name) {
    return Logger(name.c_str());
}
#endif

namespace detail {

/// **REFUSED** — the target of every `RCLCPP_*_THROTTLE` macro. Variadic so
/// the macro can forward `logger`, `clock`, the period and the whole format
/// pack, which means the arguments are still parsed and type-checked; only the
/// `static_assert` stops the build, with the migration attached.
template <typename Logger, typename Clock, typename Period, typename... Rest>
void throttle_is_refused(Logger&&, Clock&&, Period&&, Rest&&...) {
    static_assert(refuse<Logger>::value, NROS_RCLCPP_REFUSE_THROTTLE);
}

} // namespace detail

} // namespace rclcpp

// --- Log macros --------------------------------------------------------------
//
// Same call shape as rclcpp.
//
// ADOPT-BOUNDED on ONE point, shared by the whole family: the logger argument
// is evaluated and then DISCARDED. `rclcpp::Logger` here is a name-only
// sentinel and the sink is keyed on `__FILE__` / `__LINE__`, so two loggers in
// one file are indistinguishable in the output and a per-logger level cannot be
// set. Named loggers exist Rust-side; re-exporting them is phase-417 W4.d.
// Nothing is lost that the call site did not already have — the message and its
// arguments all reach the sink.
//
// `_STREAM` no longer discards its message (issue 1019). It used to expand to
// `RCLCPP_INFO(logger, "%s", "")` with `args` NEVER REFERENCED, which is the
// worst outcome available: the call compiled, the level was right, the file and
// line were right, and the text was gone. It now formats through
// `std::ostringstream` and hands the result to the same sink — a string
// conversion that copies and calls through, which RFC-0087 §"Who implements an
// adopted name" allows in the wrapper. Since the move into this header
// `<sstream>` is GATED rather than unconditional, so the `_STREAM` family is
// declared only where the standard library that backs it exists.
//
// `_THROTTLE` is REFUSE-LOUD. See `NROS_RCLCPP_REFUSE_THROTTLE`.

// NROS_INFO is a do-while(0) block; the comma-operator wrapper around it was
// invalid C++. Use a do-while wrapper so RCLCPP_INFO is a single statement.
//
// The whole family sits behind `#ifndef RCLCPP_INFO` so a translation unit that
// somehow also has real rclcpp keeps rclcpp's own definitions.
#ifndef RCLCPP_INFO
#define RCLCPP_INFO(logger, ...)                                                                   \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_INFO(__VA_ARGS__);                                                                    \
    } while (0)
#define RCLCPP_WARN(logger, ...)                                                                   \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_WARN(__VA_ARGS__);                                                                    \
    } while (0)
#define RCLCPP_ERROR(logger, ...)                                                                  \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_ERROR(__VA_ARGS__);                                                                   \
    } while (0)
#define RCLCPP_DEBUG(logger, ...)                                                                  \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_DEBUG(__VA_ARGS__);                                                                   \
    } while (0)
#define RCLCPP_FATAL(logger, ...)                                                                  \
    do {                                                                                           \
        (void)(logger);                                                                            \
        NROS_ERROR(__VA_ARGS__);                                                                   \
    } while (0)

// REFUSED. The arguments are still forwarded so they are parsed and
// type-checked — a refusal should not also hide a typo in the format pack.
#define RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, ...)                                       \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, period_ms, ...)                                        \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, period_ms, ...)                                        \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, ...)                                       \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#define RCLCPP_FATAL_THROTTLE(logger, clock, period_ms, ...)                                       \
    ::rclcpp::detail::throttle_is_refused((logger), (clock), (period_ms), __VA_ARGS__)
#endif // RCLCPP_INFO

// The stream family, carrying its message. `NROS_RCLCPP_STREAM_` builds the
// text once and forwards it as a single `%s` argument, so a `%` inside the
// user's text can never be read as a conversion.
#if defined(NROS_CPP_HAS_STD_SSTREAM) && !defined(RCLCPP_INFO_STREAM)
#define NROS_RCLCPP_STREAM_(macro, logger, ...)                                                    \
    do {                                                                                           \
        ::std::ostringstream nros_rclcpp_stream_;                                                  \
        nros_rclcpp_stream_ << __VA_ARGS__;                                                        \
        macro(logger, "%s", nros_rclcpp_stream_.str().c_str());                                    \
    } while (0)

#define RCLCPP_DEBUG_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_DEBUG, logger, __VA_ARGS__)
#define RCLCPP_INFO_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_INFO, logger, __VA_ARGS__)
#define RCLCPP_WARN_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_WARN, logger, __VA_ARGS__)
#define RCLCPP_ERROR_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_ERROR, logger, __VA_ARGS__)
#define RCLCPP_FATAL_STREAM(logger, ...) NROS_RCLCPP_STREAM_(RCLCPP_FATAL, logger, __VA_ARGS__)
#endif // NROS_CPP_HAS_STD_SSTREAM && !RCLCPP_INFO_STREAM

#endif // NROS_CPP_LOG_HPP
