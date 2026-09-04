//! Phase 88 — minimal nros-log demo (Rust).
//!
//! Wires the `PlatformSink` (the default sink that forwards to
//! `nros_platform_log_write`), publishes records at every severity,
//! then drops the runtime threshold to filter the same logger at
//! Warn so the bottom three severities go quiet.
//!
//! Run:
//!
//! ```bash
//! cargo run -p native-rs-logging
//! ```
//!
//! Expected stderr:
//!
//! ```text
//! [TRACE] demo: round 1: hello with trace=true
//! [DEBUG] demo: round 1: hello with debug=true
//! [INFO] demo: round 1: hello with info=true
//! [WARN] demo: round 1: hello with warn=true
//! [ERROR] demo: round 1: hello with error=true
//! [FATAL] demo: round 1: hello with fatal=true
//! -- threshold raised to Warn --
//! [WARN] demo: round 2: hello with warn=true
//! [ERROR] demo: round 2: hello with error=true
//! [FATAL] demo: round 2: hello with fatal=true
//! ```

use nros_log::{Logger, Severity, log_debug, log_error, log_fatal, log_info, log_warn, nros_trace};

// Pre-register a Logger so `get_logger("demo")` / `Node::logger()`
// resolve to the same instance.
static LOGGER: Logger = Logger::new("demo");

// Force the nros-platform-cffi crate's `posix-c-port` C build to
// stay in the link graph. Without an explicit symbol reference,
// rustc elides the rlib + the `nros_platform_log_write` C symbol
// goes missing at the binary link step.
extern crate nros_platform_cffi as _;

fn main() {
    // Publish the logger into the bounded intern table.
    nros_log::register_logger(&LOGGER);

    // Install the default sink list (PlatformSink → POSIX stderr).
    nros_log::init(nros_platform_cffi::log::default_sinks());

    // Drop the threshold so every severity fires in round 1.
    LOGGER.set_level(Severity::Trace);

    for round in 1..=2 {
        if round == 2 {
            eprintln!("-- threshold raised to Warn --");
            LOGGER.set_level(Severity::Warn);
        }
        nros_trace!(&LOGGER, "round {}: hello with trace={}", round, true);
        log_debug!(&LOGGER, "round {}: hello with debug={}", round, true);
        log_info!(&LOGGER, "round {}: hello with info={}", round, true);
        log_warn!(&LOGGER, "round {}: hello with warn={}", round, true);
        log_error!(&LOGGER, "round {}: hello with error={}", round, true);
        log_fatal!(&LOGGER, "round {}: hello with fatal={}", round, true);
    }

    nros_log::flush();
}
