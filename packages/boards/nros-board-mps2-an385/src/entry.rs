//! Phase 244.D1 enabler — `nros_platform::BoardEntry` for the pure
//! bare-metal (no-RTOS) MPS2-AN385 board.
//!
//! Mirrors the FreeRTOS family driver's `BoardEntry` shim
//! (`nros-board-freertos/src/entry.rs`) but for direct bare-metal Cortex-M
//! execution: there is no kernel task to spawn and no scheduler to start, so
//! the boot scaffold runs inline on the reset thread —
//! `init_hardware` (clock + ethernet/serial bring-up) → open the `Executor`
//! → wrap it in an `ExecutorNodeRuntime` + `RuntimeCtx` → hand it to the
//! codegen-emitted `setup` closure (the launch-resolved `register(...)`
//! calls) → spin forever. The reset entry itself is emitted by `nros::main!()`
//! (`#[cortex_m_rt::entry]`); this file owns only the post-reset boot body.
//!
//! The linked RMW backend is registered by the macro
//! (`__register_linked_rmw()` before `BoardEntry::run_with_deploy`), so this
//! board stays RMW-agnostic — it never names a concrete backend.

use nros::{BootConfig, Executor, ExecutorConfig, node_runtime::ExecutorNodeRuntime};
use nros_platform::{
    BakedBootConfig, BoardEntry, BoardExit, BoardInit, BoardPrint, DeployOverlay,
    NodeDispatchRuntime, RuntimeCtx,
};

use crate::{Config, init_hardware, node::Mps2An385};

// Additive impls of the new `nros_platform::board` trait set (parameterless
// `init_hardware`) that `BoardEntry: Board` requires. The legacy
// `nros_board_common::{BoardInit,BoardPrint,BoardExit}` impls in `node.rs` stay
// for the `run(Config, closure)` path; these mirror their bodies. Real
// hardware init runs in `boot()` via `crate::init_hardware(&cfg)`, so the
// parameterless trait method is a no-op.
impl BoardInit for Mps2An385 {
    fn init_hardware() {}
}

impl BoardPrint for Mps2An385 {
    fn println(args: core::fmt::Arguments<'_>) {
        use core::fmt::Write;
        if let Ok(mut stdout) = crate::cortex_m_semihosting::hio::hstdout() {
            let _ = writeln!(stdout, "{args}");
        }
    }
}

impl BoardExit for Mps2An385 {
    fn exit_success() -> ! {
        crate::exit_success()
    }

    fn exit_failure() -> ! {
        crate::exit_failure()
    }
}

/// The board's boot [`Config`] before any deploy overlay. Ethernet is the
/// default link; a board built `serial`-only (no `ethernet`) boots the UART
/// link (`Config::serial_default`, locator `serial/UART_0#…`) — phase-244.D1
/// serial deploys. Ethernet wins when both features are on (its `Config` has
/// the full field set).
#[cfg(feature = "ethernet")]
fn base_config() -> Config {
    Config::default()
}
#[cfg(all(feature = "serial", not(feature = "ethernet")))]
fn base_config() -> Config {
    Config::serial_default()
}

/// Build the board boot [`Config`] from the per-link base default, overlaying
/// any `[package.metadata.nros.deploy.<board>]` fields the Entry pkg supplied
/// (issue #48 cause 1). `None` fields keep the board default. The ip/gateway/
/// netmask overlay is ethernet-only (the serial `Config` has no IP fields); the
/// locator + domain overlay applies to both links.
fn config_with_overlay(deploy: &DeployOverlay) -> Config {
    let mut cfg = base_config();
    if let Some(locator) = deploy.locator {
        cfg.zenoh_locator = locator;
    }
    #[cfg(feature = "ethernet")]
    {
        if let Some(ip) = deploy.ip {
            cfg.ip = ip;
        }
        if let Some(gateway) = deploy.gateway {
            cfg.gateway = gateway;
        }
        if let Some(netmask) = deploy.netmask {
            // phase-337 W6.a — was a local popcount `mask_to_prefix`, one of
            // two copies (the other in the folded RTIC crate). One spelling.
            cfg.prefix = nros_board_common::prefix_from_netmask(netmask);
        }
    }
    if let Some(domain_id) = deploy.domain_id {
        cfg.domain_id = domain_id;
    }
    cfg
}

/// Shared boot body: hardware/network bring-up → executor → runtime → user
/// setup → spin. Never returns on the happy path (the firmware loops for its
/// lifetime); a setup `Err` propagates out so the macro can route to
/// `exit_failure`.
///
/// `boot_config` — the baked `.nros_boot_config` static from `nros::main!()`,
/// supplied by `run_with_deploy` (issue #98 / RFC-0045). `None` when called
/// from the no-deploy `run` path (keeps historical `"nros_app"` default).
fn boot<F, E>(cfg: Config, boot_config: Option<&'static BakedBootConfig>, setup: F) -> Result<(), E>
where
    F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
    E: core::fmt::Debug,
{
    // Clock + ethernet/serial bring-up. Must precede any executor / socket op.
    init_hardware(&cfg);

    // Phase 244.D1 — install the agnostic `nros_log` dispatcher so declarative
    // nodes can `log_info!` (the mps2-an385 semihosting `PlatformLog` already
    // ships; this only wires the dispatcher to the default sinks). Replaces the
    // per-example `nros_log::init(...)` that used to live in each talker's boot
    // closure. Nodes still `register_logger(&LOGGER)` in their `register()`.
    nros_platform_cffi::log::init_default();

    // phase-338 W7 — bridge the `log` facade too, so BOTH work here.
    //
    // This board was the last one whose node bodies had to use `log_info!`:
    // every other platform bridges `log`, so a body written against `log`
    // compiled here and printed nothing. That made the logging facade a board
    // property leaking into user source, which is the defect class this phase
    // exists to remove.
    //
    // Note the direction. W7 was drafted the other way round — add `nros_log`
    // to the boards that lack it — on the premise that `log` needs `std`.
    // qemu-esp32-baremetal disproves that: it bridges `log` on a `no_std`
    // target through `esp_println`. So `log` is the user-facing facade
    // everywhere and `nros_log` stays the platform/ABI layer (it is what
    // `nros_platform_log_write`, and therefore the C API, is built on).
    crate::log_bridge::install_semihosting_log_bridge();

    // Phase 248 C5a (#60 T4) — the board owns RMW selection: register the linked
    // zenoh backend into the CFFI vtable here, before `Executor::open`.
    // Bare-metal (`target_os = "none"`) is linkme-blind + runs no `.init_array`,
    // so the auto-register section is a no-op; this explicit, idempotent call is
    // the registration path (mirrors `crate::rtic::init_with_config`).
    // Gated on the board's own `rmw-zenoh` feature so DDS-/XRCE-only builds drop it.
    #[cfg(feature = "rmw-zenoh")]
    if let Err(err) = nros_rmw_zenoh::register() {
        Mps2An385::println(format_args!(""));
        Mps2An385::println(format_args!("zenoh RMW register failed: {err:?}"));
        Mps2An385::exit_failure();
    }

    // Issue #98 / RFC-0045 — node name from the baked `.nros_boot_config` (a
    // launch that names the node overrides the board default); locator/domain
    // unchanged from the board config (NOT env vars — bare-metal libc has no
    // host `getenv` trampoline on QEMU).
    let baked = boot_config.map(BootConfig::from_baked).unwrap_or_default();
    let exec_cfg = ExecutorConfig::resolve(BootConfig {
        node_name: baked.node_name.or(Some("nros_app")),
        locator: Some(cfg.zenoh_locator),
        domain_id: Some(cfg.domain_id),
        namespace: None,
        // Issue 1050 defect (3) — the baked rung reaches the resolver. This
        // board has no environment to read, so before it existed an image here
        // could not name its backend at all.
        rmw: baked.rmw,
    });
    let executor = match Executor::open(&exec_cfg) {
        Ok(executor) => executor,
        Err(err) => {
            Mps2An385::println(format_args!(""));
            Mps2An385::println(format_args!("Executor::open failed: {err:?}"));
            Mps2An385::exit_failure();
        }
    };

    let mut runtime_inner = ExecutorNodeRuntime::from_executor(executor);
    let mut runtime = RuntimeCtx::with_runtime(&mut runtime_inner);

    setup(&mut runtime)?;

    Mps2An385::println(format_args!(""));
    Mps2An385::println(format_args!(
        "Application setup complete — entering spin loop."
    ));
    loop {
        if let Err(err) = NodeDispatchRuntime::spin_once(&mut runtime_inner, 10) {
            Mps2An385::println(format_args!(""));
            Mps2An385::println(format_args!("spin_once error: {err:?}"));
            Mps2An385::exit_failure();
        }
    }
}

impl BoardEntry for Mps2An385 {
    fn run<F, E>(setup: F) -> Result<(), E>
    where
        F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
        E: core::fmt::Debug,
    {
        boot(base_config(), None, setup)
    }

    fn run_with_deploy<F, E>(deploy: &DeployOverlay, setup: F) -> Result<(), E>
    where
        F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
        E: core::fmt::Debug,
    {
        boot(config_with_overlay(deploy), deploy.boot_config, setup)
    }

    /// Phase-244.D1 — install the XRCE-over-UART custom transport when the
    /// deploy overlay requests `transport = "xrce"`. `nros::main!()` calls this
    /// immediately before `__register_linked_rmw()`, so the vtable is in place
    /// before the XRCE backend registers (the ordering `set_custom_transport_ops`
    /// requires). Wraps the board's shared CMSDK UART0 (`framing = true` selects
    /// XRCE HDLC framing for the byte-stream link). No-op without the
    /// `xrce-transport` feature or for any other `transport` value.
    fn setup_transport(deploy: &DeployOverlay) {
        #[cfg(feature = "xrce-transport")]
        if deploy.transport == Some("xrce") {
            let ops = crate::xrce_transport::xrce_transport_ops();
            // SAFETY: `ops`' fn pointers are static; XRCE's custom-transport
            // contract (no concurrent read/write, no ISR invocation) is met by
            // the single-threaded bare-metal executor.
            if unsafe { nros_rmw_xrce_cffi::set_custom_transport_ops(&ops, true) }.is_err() {
                Mps2An385::println(format_args!("XRCE custom transport install failed"));
                Mps2An385::exit_failure();
            }
            // #189 — register the XRCE backend explicitly. Bare-metal runs no
            // `.init_array`, so the linkme auto-register in nros-rmw-xrce-cffi
            // never fires (#163 class), and `__register_linked_rmw()` is a
            // Phase-249 no-op: without this call NO backend is registered and
            // `Executor::open` fails before a single byte reaches the UART.
            // Mirrors the explicit `nros_rmw_zenoh::register()` in `boot()`.
            if let Err(err) = nros_rmw_xrce_cffi::register() {
                Mps2An385::println(format_args!("XRCE RMW register failed: {err:?}"));
                Mps2An385::exit_failure();
            }
        }
        let _ = deploy;
    }
}
