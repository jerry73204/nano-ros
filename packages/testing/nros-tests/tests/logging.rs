//! Phase 88.14 — broader integration coverage for the `nros-log` facade.
//!
//! Verifies:
//! - Compile-time ceiling helper agrees with the active `max-level-*`
//!   feature set (default = trace).
//! - Per-`Logger` runtime threshold suppresses records below the bar
//!   without affecting other loggers in the same process.
//! - Sink fan-out: every sink installed via [`nros_log::init`] receives
//!   each dispatched record exactly once.
//!
//! The throttle/once macros from the original 88.14 acceptance list
//! are not implemented in nros-log v1 (see
//! `docs/roadmap/phase-88-nros-log.md` design notes — they were
//! deferred along with `/rosout`). When those macros land, extend this
//! file with the corresponding coverage; no other touch points need
//! changing.
//!
//! RTOS-specific impl verification (asserting the captured QEMU UART
//! output contains the expected line) is best-effort and not
//! attempted here — the per-platform smoke is the
//! `posix_dispatch.rs` integration test inside `nros-log`'s own
//! `tests/` directory plus the platform-impl unit coverage. This
//! suite stays host-only so it can run as part of the default
//! `just test` matrix without spinning up QEMU.

use std::sync::Mutex;

use nros_log::{
    LogSink, Logger, Record, Severity, init, log_debug, log_error, log_fatal, log_info, log_warn,
    nros_trace, register_logger, severity_enabled_at_compile_time,
};

/// Process-wide serialization for tests that mutate the global
/// `SINKS_PTR` or a logger threshold. The sink list is a single
/// `AtomicPtr` shared across all integration-test threads.
static SERIALIZE: Mutex<()> = Mutex::new(());

/// One captured `Record` in owned form.
#[derive(Clone, Debug)]
struct CapturedRecord {
    severity: Severity,
    logger_name: String,
    message: String,
}

/// Mutex-guarded buffer a `CapturingSink` writes into. Each test
/// drains the buffer at start so it sees only its own records.
static SINK_A_BUF: Mutex<Vec<CapturedRecord>> = Mutex::new(Vec::new());
static SINK_B_BUF: Mutex<Vec<CapturedRecord>> = Mutex::new(Vec::new());

/// Sink that pushes each record into the given buffer.
struct CapturingSink {
    buf: &'static Mutex<Vec<CapturedRecord>>,
}

impl LogSink for CapturingSink {
    fn log(&self, record: &Record<'_>) {
        // unwrap: panics here only on a poisoned mutex, which would
        // already invalidate the test.
        self.buf.lock().unwrap().push(CapturedRecord {
            severity: record.severity,
            logger_name: record.logger_name.to_string(),
            message: record.message.to_string(),
        });
    }
}

static SINK_A: CapturingSink = CapturingSink { buf: &SINK_A_BUF };
static SINK_B: CapturingSink = CapturingSink { buf: &SINK_B_BUF };

static FANOUT_SINKS: &[&dyn LogSink] = &[&SINK_A, &SINK_B];
static SINGLE_SINKS: &[&dyn LogSink] = &[&SINK_A];

fn drain(buf: &Mutex<Vec<CapturedRecord>>) -> Vec<CapturedRecord> {
    std::mem::take(&mut *buf.lock().unwrap())
}

// ---------------------------------------------------------------------------
// Compile-time ceiling
// ---------------------------------------------------------------------------

/// Sanity check on the const ceiling helper. Default features include
/// `max-level-trace`, so every severity must pass — otherwise the
/// `nros_*!` macros would dead-code-eliminate themselves before
/// reaching the runtime check.
#[test]
fn compile_time_ceiling_lets_every_severity_through_by_default() {
    assert!(severity_enabled_at_compile_time(Severity::Trace));
    assert!(severity_enabled_at_compile_time(Severity::Debug));
    assert!(severity_enabled_at_compile_time(Severity::Info));
    assert!(severity_enabled_at_compile_time(Severity::Warn));
    assert!(severity_enabled_at_compile_time(Severity::Error));
    assert!(severity_enabled_at_compile_time(Severity::Fatal));
}

// ---------------------------------------------------------------------------
// Runtime threshold
// ---------------------------------------------------------------------------

/// `Logger::set_level(Warn)` suppresses `Trace`/`Debug`/`Info` calls
/// on that logger; a sibling logger left at the default keeps
/// emitting `Info`.
#[test]
fn per_logger_runtime_threshold_filters_below() {
    let _g = SERIALIZE.lock().unwrap();
    drain(&SINK_A_BUF);
    drain(&SINK_B_BUF);
    init(SINGLE_SINKS);

    static QUIET: Logger = Logger::new("phase88.runtime.quiet");
    static LOUD: Logger = Logger::new("phase88.runtime.loud");
    let quiet = register_logger(&QUIET);
    let loud = register_logger(&LOUD);
    quiet.set_level(Severity::Warn);
    loud.set_level(Severity::Info);

    nros_trace!(quiet, "drop-trace");
    log_debug!(quiet, "drop-debug");
    log_info!(quiet, "drop-info");
    log_warn!(quiet, "keep-warn");
    log_error!(quiet, "keep-error");
    log_fatal!(quiet, "keep-fatal");

    log_info!(loud, "loud-info");
    log_warn!(loud, "loud-warn");

    let records = drain(&SINK_A_BUF);
    let quiet_records: Vec<_> = records
        .iter()
        .filter(|r| r.logger_name == "phase88.runtime.quiet")
        .collect();
    let loud_records: Vec<_> = records
        .iter()
        .filter(|r| r.logger_name == "phase88.runtime.loud")
        .collect();

    assert_eq!(
        quiet_records.len(),
        3,
        "quiet logger should drop trace/debug/info; got {:?}",
        quiet_records
    );
    assert!(quiet_records.iter().all(|r| matches!(
        r.severity,
        Severity::Warn | Severity::Error | Severity::Fatal
    )));
    assert!(quiet_records.iter().any(|r| r.message == "keep-warn"));

    assert_eq!(loud_records.len(), 2, "loud logger should emit info + warn");
    assert!(loud_records.iter().any(|r| r.message == "loud-info"));
    assert!(loud_records.iter().any(|r| r.message == "loud-warn"));
}

// ---------------------------------------------------------------------------
// Sink fan-out
// ---------------------------------------------------------------------------

/// Every sink installed via `init` must receive every dispatched
/// record. This guards against a regression where the dispatcher
/// short-circuits after the first sink.
#[test]
fn every_sink_receives_each_record_in_order() {
    let _g = SERIALIZE.lock().unwrap();
    drain(&SINK_A_BUF);
    drain(&SINK_B_BUF);
    init(FANOUT_SINKS);

    static FANOUT_LOGGER: Logger = Logger::new("phase88.fanout");
    let l = register_logger(&FANOUT_LOGGER);
    l.set_level(Severity::Trace);

    log_info!(l, "first");
    log_warn!(l, "second {}", 2);
    log_error!(l, "third");

    let a = drain(&SINK_A_BUF);
    let b = drain(&SINK_B_BUF);

    let messages_a: Vec<_> = a
        .iter()
        .filter(|r| r.logger_name == "phase88.fanout")
        .map(|r| (r.severity, r.message.clone()))
        .collect();
    let messages_b: Vec<_> = b
        .iter()
        .filter(|r| r.logger_name == "phase88.fanout")
        .map(|r| (r.severity, r.message.clone()))
        .collect();

    assert_eq!(
        messages_a,
        vec![
            (Severity::Info, "first".to_string()),
            (Severity::Warn, "second 2".to_string()),
            (Severity::Error, "third".to_string()),
        ],
        "sink A missed or reordered records",
    );
    assert_eq!(
        messages_a, messages_b,
        "sink B saw a different sequence than sink A — fan-out is broken",
    );
}

/// A logger whose threshold drops a record must NOT reach any sink.
/// Independent of `every_sink_receives_each_record_in_order` because
/// that test left the threshold at `Trace`.
#[test]
fn dropped_records_reach_no_sink() {
    let _g = SERIALIZE.lock().unwrap();
    drain(&SINK_A_BUF);
    drain(&SINK_B_BUF);
    init(FANOUT_SINKS);

    static DROP_LOGGER: Logger = Logger::new("phase88.drop");
    let l = register_logger(&DROP_LOGGER);
    l.set_level(Severity::Error);

    log_info!(l, "should-be-dropped");
    log_warn!(l, "should-also-be-dropped");
    log_error!(l, "kept");

    for buf in [&SINK_A_BUF, &SINK_B_BUF] {
        let records: Vec<_> = drain(buf)
            .into_iter()
            .filter(|r| r.logger_name == "phase88.drop")
            .collect();
        assert_eq!(records.len(), 1, "expected exactly the Error record");
        assert_eq!(records[0].severity, Severity::Error);
        assert_eq!(records[0].message, "kept");
    }
}
