//! Phase 88 — `PlatformSink` → `nros-platform-posix` round-trip.
//!
//! Links `nros-platform-cffi[posix-c-port]` (which compiles
//! `nros-platform-posix/src/platform.c` via `cc`) so
//! `nros_platform_log_write` resolves at test-link time. Verifies
//! the facade reaches the C impl without panicking; we don't try
//! to capture stderr here (that would be a `gag`-style hack on
//! every host). Stderr capture lands with the broader
//! `nros-tests/tests/logging.rs` work (88.14).

// issue 0710 — the platform sink lives HERE now, with the ABI it speaks, so
// this is a normal import rather than a `extern crate … as _` touch to make a
// C build run. The dependency edge and the link requirement are the same edge.
use nros_platform_cffi::log::default_sinks;

use nros_log::{Logger, Severity, init};

static TEST_LOGGER: Logger = Logger::new("posix_dispatch_test");

#[test]
fn platform_sink_round_trips_through_posix_c_impl() {
    // Publish the logger so the facade's intern table sees it.
    let l = nros_log::register_logger(&TEST_LOGGER);
    assert_eq!(l.name(), TEST_LOGGER.name());

    // Install the default sink list (PlatformSink only).
    init(default_sinks());

    // Drop the threshold so all severities fire.
    TEST_LOGGER.set_level(Severity::Trace);

    // Each macro should complete without panicking; the C impl
    // writes the rendered line to stderr.
    nros_log::nros_trace!(&TEST_LOGGER, "trace payload {} / {}", 1, "two");
    nros_log::log_debug!(&TEST_LOGGER, "debug payload");
    nros_log::log_info!(&TEST_LOGGER, "info  payload — {}", "string");
    nros_log::log_warn!(&TEST_LOGGER, "warn  payload");
    nros_log::log_error!(&TEST_LOGGER, "error payload");
    nros_log::log_fatal!(&TEST_LOGGER, "fatal payload");
    nros_log::flush();
}

#[test]
fn runtime_threshold_silences_below() {
    init(default_sinks());
    let l = nros_log::register_logger({
        static QUIET: Logger = Logger::with_level("quiet_test", Severity::Warn);
        &QUIET
    });
    assert_eq!(l.name(), "quiet_test");
    // Trace/Debug/Info dropped; Warn/Error/Fatal delivered. No
    // panics, no allocator pressure beyond the stack buffer.
    nros_log::nros_trace!(l, "should be filtered");
    nros_log::log_info!(l, "should be filtered");
    nros_log::log_warn!(l, "should appear");
    nros_log::log_error!(l, "should appear");
}
