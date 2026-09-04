//! phase-417 W4.d — `add_sink`'s BOUND, in its own binary.
//!
//! Separate from `named_loggers_levels_and_throttle.rs` on purpose: this test
//! fills the process-global registry to `MAX_ADDED_SINKS`, and the sink list is
//! append-only with no removal (by construction — the read path is lock-free).
//! Sharing a binary with the delivery tests would leave them asserting against
//! whatever slot count ran first.

use nros_log::{LogSink, Record};

#[test]
fn add_sink_refuses_past_its_bound_rather_than_dropping_silently() {
    struct Null;
    impl LogSink for Null {
        fn log(&self, _record: &Record<'_>) {}
    }
    static NULLS: [Null; nros_log::MAX_ADDED_SINKS] = [const { Null }; nros_log::MAX_ADDED_SINKS];

    let mut installed = 0;
    for null in &NULLS {
        if nros_log::add_sink(null) {
            installed += 1;
        }
    }
    assert_eq!(
        installed,
        nros_log::MAX_ADDED_SINKS,
        "add_sink must fill exactly MAX_ADDED_SINKS slots and then refuse"
    );
    assert_eq!(nros_log::added_sink_count(), nros_log::MAX_ADDED_SINKS);
    assert!(
        !nros_log::add_sink(&NULLS[0]),
        "past the bound, add_sink must return false rather than pretend"
    );
}
