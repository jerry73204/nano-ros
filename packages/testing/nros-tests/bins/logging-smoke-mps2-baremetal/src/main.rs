//! Phase 88.15.a — bare-metal MPS2-AN385 nros-log smoke fixture.
//!
//! Emits exactly one record per [`Severity`], drives them through
//! `nros-log` → `PlatformSink` → `nros_platform_log_write` →
//! `nros-platform-mps2-an385`'s semihosting writer, then exits with
//! `EXIT_SUCCESS` so the host harness can assert on the captured
//! semihosting output.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use nros_log::{
    Logger, Severity, log_debug, log_error, log_fatal, log_info, nros_trace, log_warn,
    register_logger,
};
use panic_semihosting as _;

// Force-link the per-platform PlatformLog impl so its
// `nros_platform_log_write` symbol resolves at link time. Without this
// the optimizer can drop the `nros-platform-mps2-an385` rlib and the
// PlatformSink call hits a NULL slot.
extern crate nros_platform_mps2_an385 as _;

static LOGGER: Logger = Logger::new("smoke");

#[entry]
fn main() -> ! {
    // issue 0710 (amended) — dispatch no longer installs a platform sink on
    // first use: records raised before `init` are HELD (`nros_log::early`) and
    // replayed into whatever sinks arrive, so delivery stays pluggable and the
    // platform ABI is a dependency of the binary, not a side effect of logging.
    // This image takes its own `#[entry]` and never reaches board code, so no
    // funnel publishes for it — it is the funnel, and installs the platform
    // sink list itself.
    nros_platform_cffi::log::init_default();
    let logger = register_logger(&LOGGER);
    // Drop the threshold so every severity macro emits.
    logger.set_level(Severity::Trace);

    nros_trace!(logger, "trace payload");
    log_debug!(logger, "debug payload");
    log_info!(logger, "info payload");
    log_warn!(logger, "warn payload");
    log_error!(logger, "error payload");
    log_fatal!(logger, "fatal payload");

    nros_log::flush();
    debug::exit(debug::EXIT_SUCCESS);
    loop {}
}
