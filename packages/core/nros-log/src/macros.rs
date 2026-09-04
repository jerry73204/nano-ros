//! Phase 88.2 — `nros_*!` macros.
//!
//! Each macro formats its arguments into a stack-resident
//! [`crate::FormatBuffer`] (capacity picked at compile time by the
//! `buffer-size-<N>` feature), wraps the result in a [`crate::Record`],
//! and hands it to the [`crate::Logger`]'s dispatcher.
//!
//! Compile-time ceiling: macros below
//! [`crate::compile_time_ceiling`](super::severity_enabled_at_compile_time)
//! expand to `()` — the format call is dead-code-eliminated.

/// Internal macro emitting one log record at `$severity`.
///
/// Use the named helpers ([`nros_trace!`], [`nros_debug!`], etc.) — they
/// gate on the compile-time ceiling before evaluating this body.
#[doc(hidden)]
#[macro_export]
macro_rules! __nros_log_emit {
    ($logger:expr, $severity:expr, $($arg:tt)+) => {{
        // Cheap runtime threshold check first to short-circuit
        // disabled call sites before formatting.
        let __logger: &$crate::Logger = $logger;
        let __sev = $severity;
        if __logger.is_enabled(__sev) {
            use ::core::fmt::Write as _;
            let mut __buf = $crate::FormatBuffer::new();
            // Ignoring `write!` result — `FormatBuffer::write_str`
            // never returns `Err`; overflow is signalled via the
            // `truncated()` accessor (not the `Result`).
            let _ = ::core::write!(__buf, $($arg)+);
            let __record = $crate::Record {
                severity:     __sev,
                logger_name:  __logger.name(),
                message:      __buf.as_str(),
                file:         ::core::file!(),
                line:         ::core::line!(),
                timestamp_ns: $crate::__timestamp_ns(),
            };
            __logger.dispatch(&__record);
        }
    }};
}

/// Emit at [`crate::Severity::Trace`].
///
/// Disabled at compile time unless the `max-level-trace` feature is
/// the active ceiling (default).
#[macro_export]
macro_rules! nros_trace {
    ($logger:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Trace) {
            $crate::__nros_log_emit!($logger, $crate::Severity::Trace, $($arg)+);
        }
    };
}

/// Emit at [`crate::Severity::Debug`].
#[macro_export]
macro_rules! nros_debug {
    ($logger:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Debug) {
            $crate::__nros_log_emit!($logger, $crate::Severity::Debug, $($arg)+);
        }
    };
}

/// Emit at [`crate::Severity::Info`].
#[macro_export]
macro_rules! nros_info {
    ($logger:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Info) {
            $crate::__nros_log_emit!($logger, $crate::Severity::Info, $($arg)+);
        }
    };
}

/// Emit at [`crate::Severity::Warn`].
#[macro_export]
macro_rules! nros_warn {
    ($logger:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Warn) {
            $crate::__nros_log_emit!($logger, $crate::Severity::Warn, $($arg)+);
        }
    };
}

/// Emit at [`crate::Severity::Error`].
#[macro_export]
macro_rules! nros_error {
    ($logger:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Error) {
            $crate::__nros_log_emit!($logger, $crate::Severity::Error, $($arg)+);
        }
    };
}

/// Emit at [`crate::Severity::Fatal`].
#[macro_export]
macro_rules! nros_fatal {
    ($logger:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Fatal) {
            $crate::__nros_log_emit!($logger, $crate::Severity::Fatal, $($arg)+);
        }
    };
}

// -----------------------------------------------------------------------------
// phase-417 W4.d — throttled emission.
//
// There WAS a throttle in this tree before: `nros_core::logger::Logger::
// *_throttle`. It sat on a DIFFERENT `Logger` — the one that forwards to the
// `log` crate — so it never reached `nros_platform_log_write`, and the
// `Logger` the FFI hands to C and C++ had no throttle at all. Two types, the
// capability on the unused one. The rule itself now lives in
// [`crate::throttle`], which both this macro family and the C shim call.
//
// Two spellings, and the difference is WHERE THE TIME COMES FROM:
//
// * `nros_*_throttle!(logger, interval_ms, ...)` reads the platform clock.
// * `nros_*_throttle_at!(logger, now_ns, interval_ms, ...)` takes it, which is
//   `rclcpp`'s shape (`RCLCPP_INFO_THROTTLE(logger, clock, ...)` names the
//   clock) and the only form available with no platform port.
//
// The clock-reading form REFUSES TO COMPILE without the `platform-clock`
// feature, rather than compiling into a throttle with no time base. Without a
// clock `__timestamp_ns()` is a constant `0`, and a window measured against a
// constant does not rate-limit — it admits every record, forever, while
// reading at the call site exactly like a working throttle. RFC-0087: never
// compile and differ.
// -----------------------------------------------------------------------------

/// Resolves the timestamp the `nros_*_throttle!` family measures against.
///
/// `platform-clock` on: the platform's monotonic clock. Off: a compile error
/// naming the feature — see the family's module note for why this is not a
/// silent degradation.
#[cfg(feature = "platform-clock")]
#[doc(hidden)]
#[macro_export]
macro_rules! __nros_throttle_now {
    () => {
        $crate::__timestamp_ns()
    };
}

/// See the `platform-clock` arm above.
#[cfg(not(feature = "platform-clock"))]
#[doc(hidden)]
#[macro_export]
macro_rules! __nros_throttle_now {
    () => {
        ::core::compile_error!(
            "nros_*_throttle! needs a monotonic clock: enable the `platform-clock` feature on \
             nros-log, or use nros_*_throttle_at!(logger, now_ns, interval_ms, ...) and pass the \
             time yourself. Without a clock `__timestamp_ns()` is a constant 0, so the window \
             would admit every record while looking exactly like a working throttle."
        )
    };
}

/// Internal macro emitting one throttled record at `$severity`.
///
/// `$state` is a `&'static ThrottleState`; the public macros declare it as a
/// `static` at the call site, which is what makes the window per-SITE rather
/// than per-logger.
#[doc(hidden)]
#[macro_export]
macro_rules! __nros_log_emit_throttled {
    ($logger:expr, $severity:expr, $state:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {{
        let __logger: &$crate::Logger = $logger;
        let __sev = $severity;
        let __now: u64 = $now_ns;
        let __interval = $crate::interval_ms_to_ns($interval_ms as u64);
        // Severity first, window second — see `Logger::is_enabled_throttled`.
        // Also short-circuits the format call, like the unthrottled macros.
        if __logger.is_enabled_throttled(__sev, $state, __now, __interval) {
            use ::core::fmt::Write as _;
            let mut __buf = $crate::FormatBuffer::new();
            let _ = ::core::write!(__buf, $($arg)+);
            let __record = $crate::Record {
                severity:     __sev,
                logger_name:  __logger.name(),
                message:      __buf.as_str(),
                file:         ::core::file!(),
                line:         ::core::line!(),
                // The timestamp the window was decided against, not a second
                // reading — two clock reads for one record is how a record can
                // be stamped before the window it opened.
                timestamp_ns: __now,
            };
            __logger.dispatch(&__record);
        }
    }};
}

/// Declares the per-call-site window and forwards. Internal.
#[doc(hidden)]
#[macro_export]
macro_rules! __nros_throttle_site {
    ($logger:expr, $severity:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {{
        static __NROS_THROTTLE_SITE: $crate::ThrottleState = $crate::ThrottleState::new();
        $crate::__nros_log_emit_throttled!(
            $logger, $severity, &__NROS_THROTTLE_SITE, $now_ns, $interval_ms, $($arg)+
        );
    }};
}

/// Emit at [`crate::Severity::Trace`], at most once per `interval_ms` at this
/// call site.
///
/// Reads the platform clock; needs the `platform-clock` feature (see
/// [`nros_trace_throttle_at!`] for the form that takes the time).
#[macro_export]
macro_rules! nros_trace_throttle {
    ($logger:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Trace) {
            $crate::__nros_throttle_site!(
                $logger,
                $crate::Severity::Trace,
                $crate::__nros_throttle_now!(),
                $interval_ms,
                $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Trace`], at most once per `interval_ms` at this
/// call site, measured against the `now_ns` the CALLER supplies.
///
/// `rclcpp`'s shape — `RCLCPP_TRACE_THROTTLE(logger, clock, …)` names its clock
/// too — and the only throttled form available with no platform clock.
#[macro_export]
macro_rules! nros_trace_throttle_at {
    ($logger:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Trace) {
            $crate::__nros_throttle_site!(
                $logger, $crate::Severity::Trace, $now_ns, $interval_ms, $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Debug`], at most once per `interval_ms` at this
/// call site.
///
/// Reads the platform clock; needs the `platform-clock` feature (see
/// [`nros_debug_throttle_at!`] for the form that takes the time).
#[macro_export]
macro_rules! nros_debug_throttle {
    ($logger:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Debug) {
            $crate::__nros_throttle_site!(
                $logger,
                $crate::Severity::Debug,
                $crate::__nros_throttle_now!(),
                $interval_ms,
                $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Debug`], at most once per `interval_ms` at this
/// call site, measured against the `now_ns` the CALLER supplies.
///
/// `rclcpp`'s shape — `RCLCPP_DEBUG_THROTTLE(logger, clock, …)` names its clock
/// too — and the only throttled form available with no platform clock.
#[macro_export]
macro_rules! nros_debug_throttle_at {
    ($logger:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Debug) {
            $crate::__nros_throttle_site!(
                $logger, $crate::Severity::Debug, $now_ns, $interval_ms, $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Info`], at most once per `interval_ms` at this
/// call site.
///
/// Reads the platform clock; needs the `platform-clock` feature (see
/// [`nros_info_throttle_at!`] for the form that takes the time).
#[macro_export]
macro_rules! nros_info_throttle {
    ($logger:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Info) {
            $crate::__nros_throttle_site!(
                $logger,
                $crate::Severity::Info,
                $crate::__nros_throttle_now!(),
                $interval_ms,
                $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Info`], at most once per `interval_ms` at this
/// call site, measured against the `now_ns` the CALLER supplies.
///
/// `rclcpp`'s shape — `RCLCPP_INFO_THROTTLE(logger, clock, …)` names its clock
/// too — and the only throttled form available with no platform clock.
#[macro_export]
macro_rules! nros_info_throttle_at {
    ($logger:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Info) {
            $crate::__nros_throttle_site!(
                $logger, $crate::Severity::Info, $now_ns, $interval_ms, $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Warn`], at most once per `interval_ms` at this
/// call site.
///
/// Reads the platform clock; needs the `platform-clock` feature (see
/// [`nros_warn_throttle_at!`] for the form that takes the time).
#[macro_export]
macro_rules! nros_warn_throttle {
    ($logger:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Warn) {
            $crate::__nros_throttle_site!(
                $logger,
                $crate::Severity::Warn,
                $crate::__nros_throttle_now!(),
                $interval_ms,
                $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Warn`], at most once per `interval_ms` at this
/// call site, measured against the `now_ns` the CALLER supplies.
///
/// `rclcpp`'s shape — `RCLCPP_WARN_THROTTLE(logger, clock, …)` names its clock
/// too — and the only throttled form available with no platform clock.
#[macro_export]
macro_rules! nros_warn_throttle_at {
    ($logger:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Warn) {
            $crate::__nros_throttle_site!(
                $logger, $crate::Severity::Warn, $now_ns, $interval_ms, $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Error`], at most once per `interval_ms` at this
/// call site.
///
/// Reads the platform clock; needs the `platform-clock` feature (see
/// [`nros_error_throttle_at!`] for the form that takes the time).
#[macro_export]
macro_rules! nros_error_throttle {
    ($logger:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Error) {
            $crate::__nros_throttle_site!(
                $logger,
                $crate::Severity::Error,
                $crate::__nros_throttle_now!(),
                $interval_ms,
                $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Error`], at most once per `interval_ms` at this
/// call site, measured against the `now_ns` the CALLER supplies.
///
/// `rclcpp`'s shape — `RCLCPP_ERROR_THROTTLE(logger, clock, …)` names its clock
/// too — and the only throttled form available with no platform clock.
#[macro_export]
macro_rules! nros_error_throttle_at {
    ($logger:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Error) {
            $crate::__nros_throttle_site!(
                $logger, $crate::Severity::Error, $now_ns, $interval_ms, $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Fatal`], at most once per `interval_ms` at this
/// call site.
///
/// Reads the platform clock; needs the `platform-clock` feature (see
/// [`nros_fatal_throttle_at!`] for the form that takes the time).
#[macro_export]
macro_rules! nros_fatal_throttle {
    ($logger:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Fatal) {
            $crate::__nros_throttle_site!(
                $logger,
                $crate::Severity::Fatal,
                $crate::__nros_throttle_now!(),
                $interval_ms,
                $($arg)+
            );
        }
    };
}

/// Emit at [`crate::Severity::Fatal`], at most once per `interval_ms` at this
/// call site, measured against the `now_ns` the CALLER supplies.
///
/// `rclcpp`'s shape — `RCLCPP_FATAL_THROTTLE(logger, clock, …)` names its clock
/// too — and the only throttled form available with no platform clock.
#[macro_export]
macro_rules! nros_fatal_throttle_at {
    ($logger:expr, $now_ns:expr, $interval_ms:expr, $($arg:tt)+) => {
        if $crate::severity_enabled_at_compile_time($crate::Severity::Fatal) {
            $crate::__nros_throttle_site!(
                $logger, $crate::Severity::Fatal, $now_ns, $interval_ms, $($arg)+
            );
        }
    };
}
