//! Phase 88.15.c — NuttX QEMU ARM nros-log smoke fixture.
//!
//! Boots NuttX through the board crate's `nsh_main` override
//! (`nros-board-nuttx-qemu/src/entry.rs`: NuttX init → `nsh_main` →
//! `nsh_initialize()` → Rust `main`), then drives every severity through
//! `nros-log`. The NuttX C platform path routes records through syslog so
//! the QEMU harness can assert the captured UART output.
//!
//! #127 — this bin previously carried its OWN `nsh_main` + a build.rs copy
//! of the NuttX image link. Both are gone: the board dep supplies the init
//! entrypoint and (via `nros_board_common::nuttx_image_link`) the
//! propagating link directives; `.cargo/config.toml` carries the static
//! link args.

// phase-359 W7 — the NuttX family is `no_std`; the board crate supplies this
// image's `#[global_allocator]`, and `nros::main!` is not used here, so this bin
// spells out the `extern "C" fn main` that `nsh_main` calls (libstd's
// `lang_start` used to supply that symbol).
//
// phase-366 W5.c/W5.d — the `#[panic_handler]` is no longer the board's. This
// image declares it below, forwarding to the board's `nros_platform_panic`
// (`nros: PANIC <msg>`, then exit(1) — the status the harness expects). Written
// out rather than `nros::panic_to_platform!()` because this bin does not dep the
// `nros` facade.
#![no_std]
#![no_main]

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

// Link-anchor the board crate: its `entry.rs` `nsh_main` (the NuttX
// `CONFIG_INIT_ENTRYPOINT`) and its build.rs's propagating image-link
// directives are the whole point of the dependency.
use nros_board_nuttx_qemu as _;
use nros_log::{Logger, Severity, log_debug, log_error, log_fatal, log_info, nros_trace, log_warn, register_logger};

static LOGGER: Logger = Logger::new("smoke");

/// The image entry NuttX's `nsh_main` calls; signature matches the
/// `extern "C" fn main(argc, argv)` that the board's `entry.rs` declares.
#[unsafe(no_mangle)]
pub extern "C" fn main(_argc: i32, _argv: *const *const core::ffi::c_char) -> i32 {
    register_logger(&LOGGER);
    // issue 0710 — deliberately NO `init(sinks::default())`: the board's
    // `nsh_main` funnel publishes the sink list. Relying on it is what makes
    // this an assertion about the BOARD rather than about the platform ABI.
    LOGGER.set_level(Severity::Trace);

    nros_trace!(&LOGGER, "trace payload");
    log_debug!(&LOGGER, "debug payload");
    log_info!(&LOGGER, "info payload");
    log_warn!(&LOGGER, "warn payload");
    log_error!(&LOGGER, "error payload");
    log_fatal!(&LOGGER, "fatal payload");
    nros_log::flush();
    0
}
