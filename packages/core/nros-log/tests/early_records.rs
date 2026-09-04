//! Issue 0710 follow-up — a record raised BEFORE `init` reaches the sinks the
//! board eventually installs.
//!
//! Its own test binary, deliberately: `init` publishes process-global state, so
//! "before any sink was installed" is only observable in a process where
//! nothing else has installed one. Sharing `posix_dispatch.rs`'s binary would
//! make this test's result depend on which test ran first.
//!
//! It also links NO platform port, and that is half the point. Before this
//! change dispatch installed `PlatformSink` on first use, so
//! `nros_platform_log_write` was reachable from every binary that logged and
//! this file would not link. The facade stays a facade; the ABI belongs to
//! whoever owns it.

use std::sync::Mutex;

use nros_log::{LogSink, Logger, Record, Severity, init};

struct Capture {
    lines: Mutex<Vec<String>>,
}

impl LogSink for Capture {
    fn log(&self, record: &Record<'_>) {
        self.lines
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(format!("{}|{}", record.logger_name, record.message));
    }
}

static CAPTURE: Capture = Capture {
    lines: Mutex::new(Vec::new()),
};
static SINKS: [&dyn LogSink; 1] = [&CAPTURE];
static EARLY_LOGGER: Logger = Logger::new("early_test");

#[test]
fn records_raised_before_init_reach_the_sinks_installed_later() {
    nros_log::register_logger(&EARLY_LOGGER);
    EARLY_LOGGER.set_level(Severity::Trace);

    // BEFORE any `init`. Historically dropped (pre-0708), then delivered to a
    // sink dispatch guessed (0710); now held.
    nros_log::log_info!(&EARLY_LOGGER, "raised before init");
    nros_log::log_warn!(&EARLY_LOGGER, "also before init");

    assert!(
        CAPTURE.lines.lock().unwrap().is_empty(),
        "nothing may reach a sink that has not been installed"
    );

    init(&SINKS);

    let lines = CAPTURE.lines.lock().unwrap_or_else(|e| e.into_inner());
    let held: Vec<&String> = lines
        .iter()
        .filter(|l| l.starts_with("early_test|"))
        .collect();
    assert_eq!(
        held,
        vec![
            &"early_test|raised before init".to_string(),
            &"early_test|also before init".to_string()
        ],
        "both early records must arrive, in the order they were raised: {lines:?}"
    );
    drop(lines);

    // And the ring is spent: a later record goes straight through, once.
    nros_log::log_info!(&EARLY_LOGGER, "after init");
    let lines = CAPTURE.lines.lock().unwrap_or_else(|e| e.into_inner());
    assert_eq!(
        lines.iter().filter(|l| l.ends_with("|after init")).count(),
        1
    );
}
