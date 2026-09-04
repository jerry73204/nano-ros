//! Phase 88.15.f — ESP32-C3 QEMU nros-log smoke fixture.
//!
//! Boots ESP32-C3 + OpenETH via the board crate's `run()`, which
//! registers an `esp_println`-backed writer against the
//! `nros-platform-esp32-qemu` log fn-ptr slot (Phase 88.15.f
//! groundwork). The closure drives every Severity through the
//! `nros-log` facade and then loops forever — QEMU is killed by
//! the harness once the captured output contains every expected
//! line.

#![no_std]
#![no_main]

use esp_backtrace as _;
use nros_board_esp32_qemu::{Config, entry, run_bare};
use nros_log::{Logger, Severity, log_debug, log_error, log_fatal, log_info, nros_trace, log_warn, register_logger};

nros_board_esp32_qemu::esp_bootloader_esp_idf::esp_app_desc!();

static LOGGER: Logger = Logger::new("smoke");

// Network config lives in a sibling `config.toml`, compile-baked here
// (RFC-0004: config in a file, not hardcoded in code). `from_toml` applies the
// build-time `NROS_DOMAIN_ID` override for per-fixture domain isolation.
const CONFIG: &str = include_str!("../config.toml");

#[entry]
fn main() -> ! {
    run_bare(Config::from_toml(CONFIG), || {
        register_logger(&LOGGER);
        // issues 0708/0710 — deliberately NO `init(sinks::default())`: the board's
        // `run_bare` funnel publishes the sink list. Relying on it is what makes this
        // an assertion about the BOARD rather than about the platform ABI, which is
        // the only check a spelling-based gate cannot fool.
        let logger = &LOGGER;
        logger.set_level(Severity::Trace);

        nros_trace!(logger, "trace payload");
        log_debug!(logger, "debug payload");
        log_info!(logger, "info payload");
        log_warn!(logger, "warn payload");
        log_error!(logger, "error payload");
        log_fatal!(logger, "fatal payload");
        nros_log::flush();

        Ok::<(), &'static str>(())
    })
}
