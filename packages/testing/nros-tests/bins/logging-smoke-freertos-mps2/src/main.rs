//! Phase 88.15.b — FreeRTOS + MPS2-AN385 nros-log smoke fixture.
//!
//! Boots FreeRTOS via the board crate's `run()`, lets the board
//! crate register its semihosting log writer (Phase 88.11), then
//! emits one record per [`nros_log::Severity`] from the app task
//! and exits via semihosting `EXIT_SUCCESS`. The companion harness
//! (`packages/testing/nros-tests/tests/logging_smoke.rs`) drains
//! the QEMU semihosting stderr and asserts every `[<LEVEL>] smoke: …`
//! line appears.

#![no_std]
#![no_main]

use nros_board_mps2_an385_freertos::{Config, Mps2An385};
use nros_log::{Logger, Severity, log_debug, log_error, log_fatal, log_info, nros_trace, log_warn, register_logger};
use panic_semihosting as _;

// Link `nros-platform` so its FreeRTOS C symbols + `global-allocator`
// adapter end up in the binary even though we do not name them.
extern crate nros_platform as _;

static LOGGER: Logger = Logger::new("smoke");

// Network config lives in a sibling `config.toml`, compile-baked here
// (RFC-0004: config in a file, not hardcoded in code). `from_toml` applies the
// build-time `NROS_DOMAIN_ID` override for per-fixture domain isolation.
const CONFIG: &str = include_str!("../config.toml");

#[unsafe(no_mangle)]
extern "C" fn main() -> ! {
    // Phase 313 W-freertos (#0243) — no-session `run_bare` (scheduler + boot
    // bringup + UART writer, no `Executor::open`). The closure exits via
    // semihosting itself, so the family Ok/exit arms are never reached.
    let _ = Mps2An385::run_bare(Config::from_toml(CONFIG), |_config| {
        register_logger(&LOGGER);
        // issue 0708 — deliberately NO `init(sinks::default())` here.
        //
        // A fixture that publishes its own sink list proves the PLATFORM half
        // (`nros_platform_log_write` exists, issue 0420's question) and nothing about
        // whether this BOARD publishes one. Six boards did not, and library records —
        // `log_error!` raised inside a crate whose author cannot know what the board
        // did — were dropped. Relying on the board is what makes this an assertion
        // about the board.
        //
        // If this image ever emits nothing again, do not add an `init` here: that
        // hides the defect the assertion exists to catch. The boot funnel is where it
        // belongs, and `check-board-log-sink` names the funnel that is missing it.
        let logger = &LOGGER;
        logger.set_level(Severity::Trace);

        nros_trace!(logger, "trace payload");
        log_debug!(logger, "debug payload");
        log_info!(logger, "info payload");
        log_warn!(logger, "warn payload");
        log_error!(logger, "error payload");
        log_fatal!(logger, "fatal payload");
        nros_log::flush();

        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS);

        #[allow(unreachable_code)]
        Ok::<(), &'static str>(())
    });
    // `run_bare` diverges internally (scheduler start → exit_success/failure);
    // the closure's semihosting exit fires first for this fixture.
    unreachable!()
}
