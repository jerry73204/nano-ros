//! phase-417 W4.d — the three capabilities C and C++ had none of, asserted on
//! the Rust side that implements them (RFC-0019: the wrappers delegate here).
//!
//! Every test observes DELIVERY through a real [`LogSink`] rather than
//! inspecting the logger's fields, because "the level was stored" and "the
//! record was filtered" are different claims and only the second one is the
//! capability.
//!
//! ## Why the lock
//!
//! `nros_log`'s dispatcher holds a process-global recursion guard, so two
//! threads logging at once means one record is dropped by design. Tests that
//! assert on WHAT was collected therefore serialise; the ones that only assert
//! on logger identity do not need to.

use std::sync::Mutex;

use nros_log::{LogSink, Logger, Record, Severity};

// -----------------------------------------------------------------------------
// A sink that remembers, installed once through the phase-417 `add_sink`.
// -----------------------------------------------------------------------------

struct Collector {
    lines: Mutex<Vec<(String, Severity, String, u32)>>,
}

impl LogSink for Collector {
    fn log(&self, record: &Record<'_>) {
        self.lines.lock().unwrap().push((
            record.logger_name.to_string(),
            record.severity,
            record.message.to_string(),
            record.line,
        ));
    }
}

static COLLECTOR: Collector = Collector {
    lines: Mutex::new(Vec::new()),
};

/// Serialises the tests that assert on collected output; see the module note.
static DISPATCH: Mutex<()> = Mutex::new(());

/// Idempotent: `add_sink` appends, so calling it once per test would install
/// four copies and every assertion would count double.
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

fn drain_for(logger_name: &str) -> Vec<(Severity, String, u32)> {
    let mut lines = COLLECTOR.lines.lock().unwrap();
    let mine = lines
        .iter()
        .filter(|(name, ..)| name == logger_name)
        .map(|(_, sev, msg, line)| (*sev, msg.clone(), *line))
        .collect();
    lines.retain(|(name, ..)| name != logger_name);
    mine
}

// -----------------------------------------------------------------------------
// 1. Named logger lookup CREATES, and creates something distinct.
// -----------------------------------------------------------------------------

#[test]
fn a_created_logger_is_distinct_from_every_other_and_from_the_default() {
    let alpha = nros_log::get_or_create_logger("w4d_alpha").expect("arena has room");
    let beta = nros_log::get_or_create_logger("w4d_beta").expect("arena has room");

    assert_eq!(alpha.name(), "w4d_alpha");
    assert_eq!(beta.name(), "w4d_beta");
    assert!(
        !core::ptr::eq(alpha, beta),
        "two names must not resolve to one logger"
    );
    assert!(
        !core::ptr::eq(alpha, &nros_log::DEFAULT_LOGGER),
        "a created logger must not be the catch-all — that aliasing is the \
         whole defect this closes: `set_level(get_logger(\"nav\"), Debug)` \
         would otherwise move the threshold of every unregistered name"
    );

    // Idempotent: the second call is a LOOKUP, not a second slot.
    let again = nros_log::get_or_create_logger("w4d_alpha").expect("already interned");
    assert!(core::ptr::eq(alpha, again));
    // ...and plain `get_logger` finds it too, so the two entry points agree.
    assert!(core::ptr::eq(alpha, nros_log::get_logger("w4d_alpha")));

    // The arena accounted for exactly the two it created.
    assert!(
        nros_log::dynamic_loggers_in_use() >= 2,
        "creation must consume arena slots"
    );
    let (used, total) = nros_log::dynamic_logger_name_arena();
    assert!(used >= "w4d_alpha".len() + "w4d_beta".len());
    assert!(used <= total);
}

#[test]
fn creation_refuses_rather_than_returning_a_logger_under_the_wrong_name() {
    assert!(
        nros_log::get_or_create_logger("").is_none(),
        "an empty name must not resolve to anything"
    );
    let too_long = "x".repeat(nros_log::MAX_LOGGER_NAME_LEN + 1);
    assert!(
        nros_log::get_or_create_logger(&too_long).is_none(),
        "a name over MAX_LOGGER_NAME_LEN must be refused, not truncated onto \
         some other logger's name"
    );
}

// -----------------------------------------------------------------------------
// 2. A per-logger level actually FILTERS.
// -----------------------------------------------------------------------------

#[test]
fn a_per_logger_level_filters_delivery_and_does_not_leak_to_other_loggers() {
    let _guard = DISPATCH.lock().unwrap();
    install_collector();

    let quiet = nros_log::get_or_create_logger("w4d_quiet").expect("arena has room");
    let loud = nros_log::get_or_create_logger("w4d_loud").expect("arena has room");
    quiet.set_level(Severity::Warn);
    loud.set_level(Severity::Trace);

    assert_eq!(quiet.level(), Severity::Warn);
    assert_eq!(loud.level(), Severity::Trace);
    assert_eq!(
        nros_log::DEFAULT_LOGGER.level(),
        Severity::Info,
        "setting a level on a created logger must not move the catch-all's"
    );

    nros_log::log_info!(quiet, "filtered by level");
    nros_log::log_warn!(quiet, "admitted by level");
    nros_log::log_debug!(loud, "admitted on the loud logger");

    let quiet_lines = drain_for("w4d_quiet");
    assert_eq!(
        quiet_lines.len(),
        1,
        "expected exactly the Warn record, got {quiet_lines:?}"
    );
    assert_eq!(quiet_lines[0].0, Severity::Warn);
    assert_eq!(quiet_lines[0].1, "admitted by level");

    let loud_lines = drain_for("w4d_loud");
    assert_eq!(
        loud_lines.len(),
        1,
        "the quiet logger's threshold must not reach the loud one, got {loud_lines:?}"
    );
    assert_eq!(loud_lines[0].0, Severity::Debug);

    // Raise the threshold back and the previously-admitted severity stops.
    quiet.set_level(Severity::Error);
    nros_log::log_warn!(quiet, "now filtered");
    assert!(
        drain_for("w4d_quiet").is_empty(),
        "set_level must take effect for records raised after it"
    );
}

// -----------------------------------------------------------------------------
// 3. The throttle suppresses inside the window and admits after it.
// -----------------------------------------------------------------------------

const MS: u64 = 1_000_000;

/// One throttled call SITE. The `static ThrottleState` the macro declares lives
/// in this function, so every call goes through the same window — which is what
/// "per call site" means, and is why the test cannot just write the macro five
/// times in a row (that would be five sites, each seeing a first record).
fn throttled_tick(logger: &'static Logger, now_ns: u64) {
    nros_log::nros_info_throttle_at!(logger, now_ns, 100, "t={}", now_ns / MS);
}

/// A SECOND site with the same logger and the same interval.
fn throttled_tick_other_site(logger: &'static Logger, now_ns: u64) {
    nros_log::nros_info_throttle_at!(logger, now_ns, 100, "other t={}", now_ns / MS);
}

/// A third site, used to prove a level-filtered record does not arm the window.
fn throttled_tick_third_site(logger: &'static Logger, now_ns: u64) {
    nros_log::nros_info_throttle_at!(logger, now_ns, 100, "third t={}", now_ns / MS);
}

#[test]
fn a_throttled_site_suppresses_within_the_window_and_admits_after_it() {
    let _guard = DISPATCH.lock().unwrap();
    install_collector();

    let logger = nros_log::get_or_create_logger("w4d_throttle").expect("arena has room");
    logger.set_level(Severity::Trace);

    // `_at` rather than the clock-reading form: this must assert the RULE, and
    // a test that waits on a real clock asserts the host's scheduler instead.
    // The clock-reading spelling shares this exact code path — it only chooses
    // where `now_ns` comes from.
    throttled_tick(logger, 10 * MS); // first record: always emits
    throttled_tick(logger, 50 * MS); // 40 ms in: suppressed
    throttled_tick(logger, 109 * MS); // 1 ms short of the window: suppressed
    throttled_tick(logger, 110 * MS); // window elapsed: emits
    throttled_tick(logger, 150 * MS); // 40 ms after THAT record: suppressed

    let lines = drain_for("w4d_throttle");
    let messages: Vec<&str> = lines.iter().map(|(_, m, _)| m.as_str()).collect();
    assert_eq!(
        messages,
        vec!["t=10", "t=110"],
        "the first record always emits, the window is measured from the record \
         that EMITTED, and every record inside it is dropped"
    );

    // Two sites, two windows: a throttle is per call site, not per logger.
    throttled_tick_other_site(logger, 151 * MS);
    let other = drain_for("w4d_throttle");
    assert_eq!(
        other.iter().map(|(_, m, _)| m.as_str()).collect::<Vec<_>>(),
        vec!["other t=151"],
        "a second call site has its own window even though the first one is \
         mid-suppression"
    );

    // The severity test runs BEFORE the window, so a record the level filters
    // must not consume it.
    logger.set_level(Severity::Error);
    throttled_tick_third_site(logger, 1_000 * MS);
    assert!(
        drain_for("w4d_throttle").is_empty(),
        "an Info record on an Error-threshold logger must not be delivered"
    );
    logger.set_level(Severity::Trace);
    throttled_tick_third_site(logger, 1_001 * MS);
    let after = drain_for("w4d_throttle");
    assert_eq!(
        after.iter().map(|(_, m, _)| m.as_str()).collect::<Vec<_>>(),
        vec!["third t=1001"],
        "the level-filtered record must not have armed this site's window — if \
         it had, this one would be 1 ms into a 100 ms window and silent"
    );
}

// -----------------------------------------------------------------------------
// 4. The record carries its call site.
// -----------------------------------------------------------------------------

#[test]
fn a_record_carries_the_line_it_was_raised_on() {
    let _guard = DISPATCH.lock().unwrap();
    install_collector();

    let logger = nros_log::get_or_create_logger("w4d_site").expect("arena has room");
    let expected = line!() + 1;
    nros_log::log_error!(logger, "with a call site");

    let lines = drain_for("w4d_site");
    assert_eq!(lines.len(), 1);
    assert_eq!(
        lines[0].2, expected,
        "Record::line must be the macro's call site"
    );
}

/// A `Logger` declared the OLD way still works — the pool is additive.
static STATIC_LOGGER: Logger = Logger::with_level("w4d_static", Severity::Warn);

#[test]
fn a_statically_registered_logger_still_wins_over_creation() {
    let published = nros_log::register_logger(&STATIC_LOGGER);
    assert!(core::ptr::eq(published, &STATIC_LOGGER));
    let looked_up = nros_log::get_or_create_logger("w4d_static").expect("already registered");
    assert!(
        core::ptr::eq(looked_up, &STATIC_LOGGER),
        "creation must never shadow a logger the program registered itself"
    );
    assert_eq!(looked_up.level(), Severity::Warn);
}
