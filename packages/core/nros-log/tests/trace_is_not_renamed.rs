//! phase-417 W-B5 — `nros_trace!` keeps our prefix, and that is deliberate.
//!
//! RFC-0089's rule is "a name takes rclrs's spelling exactly when rclrs HAS
//! that name". rclrs's severity macros stop at `debug`, so there is no
//! `log_trace!` to follow, and inventing one would claim an upstream twin that
//! does not exist. TRACE is ours (ledger `rust:nros_trace`, verdict
//! `extension`), so it keeps a name that says so.
//!
//! This test outlived the removal of the `nros_debug!` .. `nros_fatal!`
//! forwarders on purpose. Those five had rclrs twins and were retired with the
//! `deprecate-legacy-names` feature; `nros_trace!` did not and was not. The
//! assertion is written down rather than left to a comment because the
//! tempting "finish the family" edit — renaming this one too, or deleting it
//! alongside the forwarders it merely resembles — is exactly the one it
//! forbids.

use std::sync::Mutex;

use nros_log::{LogSink, Logger, Record, Severity};

/// What a sink can see. `timestamp_ns` is excluded deliberately: without the
/// `platform-clock` feature it is a constant `0`.
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

/// `nros_trace!` is NOT renamed: rclrs stops at `debug`, so there is no
/// `log_trace!` to follow. Asserts the macro still exists under that name AND
/// still delivers a TRACE record attributed to this call site.
#[test]
fn trace_keeps_our_prefix_because_rclrs_has_no_trace() {
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
        assert_eq!(seen[0].message, "t");
    } else {
        assert!(
            seen.is_empty(),
            "below the compile-time ceiling nothing may be dispatched: {seen:?}"
        );
    }
}
