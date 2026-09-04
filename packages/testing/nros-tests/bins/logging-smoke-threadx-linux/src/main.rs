//! ThreadX Linux nros-log smoke fixture.
//!
//! Boots via the board crate so `run()` registers the ThreadX log
//! writer, then emits every severity through `nros-log`.

use nros_board_threadx_linux::ThreadxLinux;
use nros_log::{
    Logger, Severity, init, log_debug, log_error, log_fatal, log_info, nros_trace, log_warn,
    register_logger, sinks,
};

static LOGGER: Logger = Logger::new("smoke");

fn main() {
    // Phase 313 W0 (#0243) — boot via the new-family NO-SESSION `run_bare`
    // (was the legacy `run(Config, closure)`). A logging fixture opens no ROS
    // session, so `run_bare` boots the kernel + log writer and runs this closure
    // directly — `BoardEntry::run` would abort on `Executor::open` with no router.
    let _ = ThreadxLinux::run_bare(|| {
        register_logger(&LOGGER);
        // issue 0708 — deliberately NO `init(sinks::default())` here.
        //
        // This fixture used to publish its own sink list, which made it prove
        // that the platform's `nros_platform_log_write` works (issue 0420's
        // question) and nothing about whether a BOARD publishes one. It did
        // not, and library records — `log_error!` raised inside a crate whose
        // author cannot know what the board did — were dropped on this whole
        // family. Relying on the board is what makes this an assertion about
        // the board.
        //
        // If this fixture ever emits nothing again, do not add an `init` here:
        // that hides the defect it exists to catch. The boot funnel is where it
        // belongs, and `check-board-log-sink` says which one is missing it.
        LOGGER.set_level(Severity::Trace);

        nros_trace!(&LOGGER, "trace payload");
        log_debug!(&LOGGER, "debug payload");
        log_info!(&LOGGER, "info payload");
        log_warn!(&LOGGER, "warn payload");
        log_error!(&LOGGER, "error payload");
        log_fatal!(&LOGGER, "fatal payload");
        nros_log::flush();

        Ok::<(), &'static str>(())
    });
}
