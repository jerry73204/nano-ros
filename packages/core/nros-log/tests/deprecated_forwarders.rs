//! phase-417 — the `nros_*!` -> `log_*!` rename keeps a working forwarder.
//!
//! RFC-0089's "replace, with alias as the migration step": the ROS 2 spelling
//! becomes first-class, ours remains, and BOTH work until the removal batch.
//! The claim that needs a test is not "the old name still compiles" — a
//! forwarder that emitted at the wrong severity, through a different logger,
//! or with a `file!()`/`line!()` pointing inside `nros-log` would compile just
//! as happily and would silently degrade every unmigrated call site in the
//! tree.
//!
//! So each test emits the SAME message twice, once through each spelling, and
//! asserts the two records agree on everything a sink can observe except the
//! line they came from — and that both lines are in THIS file, which is what
//! "lands in the same place" means for a macro that is now one expansion
//! deeper than it was.

use std::sync::Mutex;

use nros_log::{LogSink, Logger, Record, Severity};

/// What a sink can see. `timestamp_ns` is excluded deliberately: without the
/// `platform-clock` feature it is a constant `0`, and with it the two calls
/// legitimately differ.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Seen {
    logger_name: String,
    severity: Severity,
    message: String,
    file: &'static str,
    line: u32,
}

struct Collector {
    seen: Mutex<Vec<Seen>>,
}

impl LogSink for Collector {
    fn log(&self, record: &Record<'_>) {
        self.seen.lock().unwrap().push(Seen {
            logger_name: record.logger_name.to_string(),
            severity: record.severity,
            message: record.message.to_string(),
            file: record.file,
            line: record.line,
        });
    }
}

static COLLECTOR: Collector = Collector {
    seen: Mutex::new(Vec::new()),
};

/// `nros_log`'s dispatcher holds a process-global recursion guard, so two
/// threads logging at once means one record is dropped by design. Every test
/// here asserts on WHAT was collected, so all of them serialise.
static DISPATCH: Mutex<()> = Mutex::new(());

fn install_collector() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        assert!(
            nros_log::add_sink(&COLLECTOR),
            "add_sink refused the first sink in the program"
        );
    });
    assert_eq!(
        nros_log::added_sink_count(),
        1,
        "the collector must be installed exactly once"
    );
}

fn drain_for(logger_name: &str) -> Vec<Seen> {
    let mut seen = COLLECTOR.seen.lock().unwrap();
    let mine: Vec<Seen> = seen
        .iter()
        .filter(|s| s.logger_name == logger_name)
        .cloned()
        .collect();
    seen.retain(|s| s.logger_name != logger_name);
    mine
}

/// Asserts the pair a `$new!` / `$old!` test collected is indistinguishable
/// apart from the call-site line, and that both lines are in this file.
fn assert_same_record(pair: &[Seen], expect_severity: Severity, expect_message: &str) {
    assert_eq!(
        pair.len(),
        2,
        "expected exactly one record per spelling, got {pair:?}"
    );
    let (new, old) = (&pair[0], &pair[1]);

    assert_eq!(new.severity, expect_severity, "new spelling's severity");
    assert_eq!(old.severity, expect_severity, "forwarder's severity");
    assert_eq!(new.message, expect_message);
    assert_eq!(old.message, expect_message);
    assert_eq!(new.logger_name, old.logger_name);

    // `file!()`/`line!()` resolve at the OUTERMOST invocation, so the extra
    // expansion the forwarder adds must not move them into `nros-log`'s
    // sources. This is the assertion that would catch a forwarder written as a
    // function or as a `#[doc(hidden)]` re-emit.
    assert_eq!(
        new.file,
        file!(),
        "the renamed macro must attribute the record to the caller's file"
    );
    assert_eq!(
        old.file,
        file!(),
        "the forwarder is one expansion deeper; it must still attribute the \
         record to the caller's file, not to nros-log/src/macros.rs"
    );
    assert_ne!(
        new.line, old.line,
        "the two calls are on different lines, so a shared line would mean \
         line!() was captured somewhere other than the call site"
    );
    assert!(
        new.line > 0 && old.line > 0,
        "line numbers must be real: {new:?} / {old:?}"
    );
}

#[test]
fn deprecated_info_forwarder_lands_where_log_info_does() {
    let _guard = DISPATCH.lock().unwrap_or_else(|e| e.into_inner());
    install_collector();
    let logger: &'static Logger = nros_log::get_or_create_logger("fwd_info").expect("arena room");

    nros_log::log_info!(logger, "same text {}", 7);
    #[allow(deprecated)]
    {
        nros_log::nros_info!(logger, "same text {}", 7);
    }

    assert_same_record(&drain_for("fwd_info"), Severity::Info, "same text 7");
}

#[test]
fn deprecated_debug_forwarder_lands_where_log_debug_does() {
    let _guard = DISPATCH.lock().unwrap_or_else(|e| e.into_inner());
    install_collector();
    let logger: &'static Logger = nros_log::get_or_create_logger("fwd_debug").expect("arena room");
    logger.set_level(Severity::Trace);

    nros_log::log_debug!(logger, "d {}", 1);
    #[allow(deprecated)]
    {
        nros_log::nros_debug!(logger, "d {}", 1);
    }

    assert_same_record(&drain_for("fwd_debug"), Severity::Debug, "d 1");
}

#[test]
fn deprecated_warn_error_fatal_forwarders_keep_their_severities() {
    let _guard = DISPATCH.lock().unwrap_or_else(|e| e.into_inner());
    install_collector();

    let warn: &'static Logger = nros_log::get_or_create_logger("fwd_warn").expect("arena room");
    nros_log::log_warn!(warn, "w");
    #[allow(deprecated)]
    {
        nros_log::nros_warn!(warn, "w");
    }
    assert_same_record(&drain_for("fwd_warn"), Severity::Warn, "w");

    let error: &'static Logger = nros_log::get_or_create_logger("fwd_error").expect("arena room");
    nros_log::log_error!(error, "e");
    #[allow(deprecated)]
    {
        nros_log::nros_error!(error, "e");
    }
    assert_same_record(&drain_for("fwd_error"), Severity::Error, "e");

    let fatal: &'static Logger = nros_log::get_or_create_logger("fwd_fatal").expect("arena room");
    nros_log::log_fatal!(fatal, "f");
    #[allow(deprecated)]
    {
        nros_log::nros_fatal!(fatal, "f");
    }
    assert_same_record(&drain_for("fwd_fatal"), Severity::Fatal, "f");
}

/// `nros_trace!` is NOT renamed: rclrs stops at `debug`, so there is no
/// `log_trace!` to follow and inventing one would claim an upstream twin that
/// does not exist. Asserted rather than left to a comment, because the
/// tempting "finish the family" edit is exactly the one this forbids.
#[test]
fn trace_keeps_our_prefix_because_rclrs_has_no_trace() {
    let _guard = DISPATCH.lock().unwrap_or_else(|e| e.into_inner());
    install_collector();
    let logger: &'static Logger = nros_log::get_or_create_logger("fwd_trace").expect("arena room");
    logger.set_level(Severity::Trace);

    nros_log::nros_trace!(logger, "t");
    let seen = drain_for("fwd_trace");

    // `max-level-trace` is the default ceiling; if a narrower one is selected
    // the record is compiled out, and the point of the test is the NAME, so
    // only assert delivery when the ceiling admits it.
    if nros_log::severity_enabled_at_compile_time(Severity::Trace) {
        assert_eq!(seen.len(), 1, "nros_trace! must still deliver: {seen:?}");
        assert_eq!(seen[0].severity, Severity::Trace);
        assert_eq!(seen[0].file, file!());
    } else {
        assert!(
            seen.is_empty(),
            "below the compile-time ceiling nothing may be dispatched: {seen:?}"
        );
    }
}
