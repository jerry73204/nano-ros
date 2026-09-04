//! Phase 88.15.d — ThreadX RISC-V QEMU nros-log smoke fixture.
//!
//! Boots ThreadX via the board crate's `run()` so the UART writer
//! gets wired into `nros-platform-threadx`'s log fn-ptr slot
//! (Phase 88.11), then drives every severity through `nros-log`
//! from the app thread and exits via the QEMU `test-finisher` MMIO
//! device.

#![no_std]
#![no_main]

use nros_board_threadx_qemu_riscv64::{Config, ThreadxQemuRiscv64};
use nros_log::{Logger, Severity, log_debug, log_error, log_fatal, log_info, nros_trace, log_warn, register_logger};

static LOGGER: Logger = Logger::new("smoke");

// phase-366 W5.c/W5.d — this image declares its own ending, because the board
// stopped doing it for us. Written out rather than `nros::panic_to_platform!()`
// only because this bin does not dep the `nros` facade; the body is identical
// and forwards to the board's `nros_platform_panic` (UART, then exit QEMU),
// which is what makes a panicking fixture report a failure instead of hanging
// until the harness times out.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write as _;

    struct Buf {
        bytes: [u8; 192],
        used: usize,
    }
    impl core::fmt::Write for Buf {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let room = self.bytes.len() - self.used;
            let n = s.len().min(room);
            self.bytes[self.used..self.used + n].copy_from_slice(&s.as_bytes()[..n]);
            self.used += n;
            Ok(())
        }
    }

    let mut buf = Buf {
        bytes: [0u8; 192],
        used: 0,
    };
    let _ = write!(buf, "{info}");

    unsafe extern "C" {
        fn nros_platform_panic(msg: *const u8, len: usize) -> !;
    }
    // SAFETY: `buf.bytes[..used]` is initialised and outlives the diverging call.
    unsafe { nros_platform_panic(buf.bytes.as_ptr(), buf.used) }
}

// Network config lives in a sibling `config.toml`, compile-baked here
// (RFC-0004: config in a file, not hardcoded in code). `from_toml` applies the
// build-time `NROS_DOMAIN_ID` override for per-fixture domain isolation.
const CONFIG: &str = include_str!("../config.toml");

#[unsafe(no_mangle)]
extern "C" fn main() -> ! {
    // Phase 313 W-threadx (#0243) — no-session `run_bare` (kernel + UART writer,
    // no `Executor::open`). Ok → the family driver prints the completion banner
    // and exits via the QEMU test-finisher (`exit_success`).
    let _ = ThreadxQemuRiscv64::run_bare(Config::from_toml(CONFIG), || {
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

        Ok::<(), &'static str>(())
    });
    // `run_bare` diverges internally (kernel entry → exit_success/failure).
    unreachable!()
}

// Panic handler ships with the board crate — no local definition.
