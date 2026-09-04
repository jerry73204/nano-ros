//! Phase 212.N.2 — `BoardEntry::run` shim for the FreeRTOS family.
//!
//! Adds an additive entry point built on the 212.N.1 trait set in
//! `nros_platform::board` (`BoardInit` parameterless, `BoardPrint`,
//! `BoardExit`, `RuntimeCtx`). Mirrors the legacy
//! [`crate::run`] body — kernel-spawn shape: allocate the app task,
//! hand it the user closure, call `vTaskStartScheduler()`, never
//! return — but threads the new `RuntimeCtx` through the user setup
//! callback instead of an opaque `&Config`.
//!
//! ## Why a free fn (not a blanket `impl BoardEntry`)
//!
//! The new [`nros_platform::BoardEntry`] trait is
//!
//! ```ignore
//! fn run<F, E>(setup: F) -> Result<(), E>
//!     where F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
//!           E: core::fmt::Debug;
//! ```
//!
//! FreeRTOS bring-up needs a board [`Config`] (MAC / IP / netmask /
//! gateway / task priorities + stack sizes) — that lives outside
//! `RuntimeCtx` (codegen overlay knobs, not hardware config). The
//! per-board crate (`nros-board-mps2-an385-freertos`, …) owns the
//! `Config` source (TOML / `Config::default()`); it implements
//! `BoardEntry` directly and delegates here:
//!
//! ```ignore
//! impl BoardEntry for MyBoard {
//!     fn run<F, E>(setup: F) -> Result<(), E>
//!     where
//!         F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
//!         E: core::fmt::Debug,
//!     {
//!         let cfg = Config::default();
//!         nros_board_freertos::run_entry::<MyBoard, F, E>(cfg, setup)
//!     }
//! }
//! ```
//!
//! 212.N.3 wires that into `nros-board-mps2-an385-freertos`; this
//! file just provides the family-side helper. The legacy
//! [`crate::run`] coexists during the 212.N transition.

use core::{
    ffi::c_void,
    sync::atomic::{AtomicUsize, Ordering},
};

use nros_platform::{BakedBootConfig, BoardExit, BoardInit, BoardPrint, RuntimeCtx, TierSpec};

use crate::{
    Config,
    error::{Error, Result as FrResult},
};

unsafe extern "C" {
    fn nros_trace_scheduler_started();
    fn nros_trace_trigger_and_dump();

    fn nros_freertos_init_network(
        mac: *const u8,
        ip: *const u8,
        netmask: *const u8,
        gw: *const u8,
    ) -> i32;

    fn nros_freertos_poll_network();
    fn nros_freertos_start_scheduler();

    fn nros_freertos_create_task(
        entry: unsafe extern "C" fn(*mut c_void),
        name: *const u8,
        stack_words: u32,
        arg: *mut c_void,
        priority: u32,
    ) -> i32;

    fn nros_freertos_get_netif_state() -> i32;

    fn nros_freertos_set_current_task_priority(priority: u32);
}

/// Network polling task stack size in words (1 KB = 256 words).
const POLL_TASK_STACK: u32 = 256;

// #191 — `log`-crate sink for the entry runtime. The Node components
// (`freertos_rs_talker` etc.) emit their e2e markers (`Publishing:` /
// `I heard:`) via `log::info!`; the freertos board installed NO `log::Log`
// backend, so every record was silently dropped — images built, booted, and
// delivered while the harness (which counts the marker lines) reported
// "messages received: 0". Mirrors `nros-board-threadx`'s `install_uart_logger`
// (same fn-pointer indirection so the sink stays generic over `BoardPrint`).
static LOG_PRINT_FN: AtomicUsize = AtomicUsize::new(0);

struct UartLogger;

impl log::Log for UartLogger {
    fn enabled(&self, _: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        let p = LOG_PRINT_FN.load(Ordering::Relaxed);
        if p != 0 {
            // SAFETY: `p` is only ever set by `install_uart_logger` to a valid
            // `fn(core::fmt::Arguments)` cast to `usize`; 0 means unset (checked).
            let f: fn(core::fmt::Arguments<'_>) = unsafe { core::mem::transmute(p) };
            // Body verbatim, `[LEVEL]` in front — same contract as the
            // linux/nuttx/mps2/threadx sinks and `nros_log`'s (issue 0309).
            // `format_args!` is built in the call: its temporaries do not
            // outlive the enclosing expression, so it cannot be `let`-bound.
            f(format_args!("[{}] {}", record.level(), record.args()));
        }
    }

    fn flush(&self) {}
}

static UART_LOGGER: UartLogger = UartLogger;

/// Install the `log` sink, routing records through `B::println`. Idempotent:
/// re-arms the print fn each call and ignores a repeated `set_logger` (the
/// second returns `Err`). Call once per boot before the user setup runs.
fn install_uart_logger<B: BoardPrint>() {
    fn print_via_board<B: BoardPrint>(args: core::fmt::Arguments<'_>) {
        B::println(args);
    }
    // Cast through a fn pointer then a raw pointer — `fn_item as usize` directly
    // trips the `fn_to_numeric_cast` lint (`-D warnings`).
    let f: fn(core::fmt::Arguments<'_>) = print_via_board::<B>;
    LOG_PRINT_FN.store(f as *const () as usize, Ordering::Relaxed);
    let _ = log::set_logger(&UART_LOGGER);
    log::set_max_level(log::LevelFilter::Trace);
}

struct AppContext<F> {
    config: Config,
    /// Issue #98 / RFC-0045 — baked `.nros_boot_config` for node-name resolution.
    boot_config: Option<&'static BakedBootConfig>,
    closure: F,
}

static mut POLL_INTERVAL_MS: u32 = 5;

/// FreeRTOS task entry for the application closure (212.N flavour —
/// hands the closure a `&mut RuntimeCtx<'_>` instead of `&Config`).
///
/// # Safety
/// `arg` must point to a valid `AppContext<F>` allocated on the
/// FreeRTOS heap by `run_entry()`, surviving until the scheduler
/// exits.
unsafe extern "C" fn app_task_entry_runtime<B, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    F: FnOnce(&mut RuntimeCtx<'_>) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &mut *(arg as *mut AppContext<F>) };

    // Phase 228.E.2 — the boot bringup (network + RNG + poll task + zenoh task
    // config + netif wait) is shared with the per-tier app task.
    unsafe { freertos_boot_bringup::<B>(&ctx.config) };

    // #191 — wire `log::info!` (the Node components' marker lines) to the
    // board console BEFORE the user setup runs.
    install_uart_logger::<B>();

    // FnOnce — `core::ptr::read` because this task entry is only
    // called once by FreeRTOS.
    let closure = unsafe { core::ptr::read(&ctx.closure) };

    // Phase 212.N.7 step-3.5 — open the executor + wrap it in an
    // `ExecutorNodeRuntime` so the codegen-emitted
    // `run_plan(runtime)` body can register components against a
    // live RMW session. Locator + domain_id come from `Config` (the
    // FreeRTOS overlay's TOML / default), NOT env vars — embedded
    // libc `getenv` has no host trampoline on QEMU. After the
    // closure returns Ok, the app task drops into a spin loop; the
    // scheduler never lets `main` return so the loop runs for the
    // firmware lifetime.
    // Issue #98 / RFC-0045 — node name from the baked `.nros_boot_config`; locator
    // + domain unchanged from the board Config (NOT env vars — embedded getenv has
    // no host trampoline on QEMU).
    let baked = ctx
        .boot_config
        .map(::nros::BootConfig::from_baked)
        .unwrap_or_default();
    let exec_cfg = ::nros::ExecutorConfig::resolve(::nros::BootConfig {
        node_name: baked.node_name.or(Some("nros_app")),
        locator: Some(ctx.config.base.zenoh_locator),
        domain_id: Some(ctx.config.base.domain_id),
        namespace: None,
        // Issue 1050 defect (3) — the baked rung reaches the resolver. This
        // board has no environment to read, so before it existed an image here
        // could not name its backend at all.
        rmw: baked.rmw,
    });
    let executor = match ::nros::Executor::open(&exec_cfg) {
        Ok(e) => e,
        Err(err) => {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!(""));
            B::println(format_args!("Executor::open failed: {:?}", err));
            B::exit_failure();
        }
    };
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
    let mut runtime = RuntimeCtx::with_runtime(&mut crt);

    match closure(&mut runtime) {
        Ok(()) => {
            B::println(format_args!(""));
            B::println(format_args!(
                "Application setup complete — entering spin loop."
            ));
            // Embedded spin: the FreeRTOS scheduler never returns from
            // this task, so we loop forever. `spin_once` errors trip
            // the trace dump + exit_failure (a working bring-up never
            // gets here).
            loop {
                if let Err(err) = ::nros_platform::NodeDispatchRuntime::spin_once(&mut crt, 10) {
                    unsafe {
                        nros_trace_trigger_and_dump();
                    }
                    B::println(format_args!(""));
                    B::println(format_args!("spin_once error: {:?}", err));
                    B::exit_failure();
                }
            }
        }
        Err(e) => {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!(""));
            B::println(format_args!("Application error: {:?}", e));
            B::exit_failure();
        }
    }
}

unsafe extern "C" fn poll_task_entry(_arg: *mut c_void) {
    unsafe extern "C" {
        fn vTaskDelay(ticks: u32);
    }
    let interval = unsafe { POLL_INTERVAL_MS };
    loop {
        unsafe {
            nros_freertos_poll_network();
            vTaskDelay(interval);
        }
    }
}

// =============================================================================
// Phase 228.E.2 — per-tier multi-task entry (RFC-0032 §5, §8.2)
// =============================================================================

/// Shared boot bringup: network init + RNG seed + poll task + zenoh task config
/// + netif wait. Extracted from `app_task_entry_runtime` so the per-tier app
/// task reuses the exact same sequence.
///
/// # Safety
/// Runs inside a FreeRTOS task, pre-`Executor::open`. `config` must be valid.
unsafe fn freertos_boot_bringup<B>(config: &Config)
where
    B: BoardPrint + BoardExit,
{
    // phase-370 W4 (issue 0733) — run the static constructors FIRST.
    //
    // This flat bare-metal image has no crt0, so nothing walks `.init_array`
    // unless the board does. The Cyclone message-descriptor registration TUs
    // are `__attribute__((constructor))`, so without this every descriptor
    // lookup misses and publisher/subscriber create returns a bare `-1` naming
    // nothing. The C/C++ lane's `freertos_c_entry.c` does the same at the same
    // point; the walk itself is idempotent, and its bounds come from the shared
    // linker script's `.init_array` KEEP block.
    unsafe extern "C" {
        fn nros_board_freertos_run_init_array();
    }
    unsafe { nros_board_freertos_run_init_array() };

    if let Err(e) = init_network(config) {
        B::println(format_args!("Error initializing network: {:?}", e));
        B::exit_failure();
    }
    B::println(format_args!("Network ready."));
    B::println(format_args!(""));

    // Seed the platform RNG so distinct sessions get distinct xorshift output.
    {
        let ip = &config.base.ip;
        let mac = &config.base.mac;
        let mut seed = ((ip[0] as u32) << 24)
            | ((ip[1] as u32) << 16)
            | ((ip[2] as u32) << 8)
            | (ip[3] as u32);
        seed = seed.wrapping_mul(2654435761);
        seed ^= ((mac[4] as u32) << 8) | (mac[5] as u32);
        if seed == 0 {
            seed = 1;
        }
        unsafe extern "C" {
            fn nros_platform_freertos_seed_rng(value: u32);
        }
        unsafe { nros_platform_freertos_seed_rng(seed) };
    }

    let poll_pri = config.poll_priority as u32; // issue 0623 — already raw

    #[cfg(feature = "rmw-zenoh")]
    {
        let read_pri = config.zenoh_read_priority as u32; // issue 0623 — already raw
        let lease_pri = config.zenoh_lease_priority as u32; // issue 0623 — already raw
        unsafe extern "C" {
            fn zpico_set_task_config(
                read_priority: u32,
                read_stack_bytes: u32,
                lease_priority: u32,
                lease_stack_bytes: u32,
            );
        }
        unsafe {
            zpico_set_task_config(
                read_pri,
                config.zenoh_read_stack_bytes,
                lease_pri,
                config.zenoh_lease_stack_bytes,
            );
        }

        // Phase 248 C5a (#60 T4) — the board owns RMW selection. Register the
        // linked zenoh backend into the CFFI vtable here, before any
        // `Executor::open` (`app_task_entry_runtime` + `app_task_entry_tiers`
        // both bring up through this fn). FreeRTOS is `target_os = "none"`
        // (linkme is a no-op + the flat image runs no `.init_array`), so without
        // this explicit, idempotent call `resolve_backend` finds no transport and
        // `Executor::open` fails with `Transport(ConnectionFailed)`. Replaces the
        // prior reliance on `nros::__register_linked_rmw()` via `nros/rmw-zenoh`.
        if let Err(err) = ::nros_rmw_zenoh::register() {
            B::println(format_args!(
                "nros: zenoh RMW backend register failed: {:?}",
                err
            ));
        }
    }

    unsafe {
        nros_trace_scheduler_started();
    }

    unsafe {
        POLL_INTERVAL_MS = config.poll_interval_ms;
    }
    let ret = unsafe {
        nros_freertos_create_task(
            poll_task_entry,
            b"net_poll\0".as_ptr(),
            POLL_TASK_STACK,
            core::ptr::null_mut(),
            poll_pri,
        )
    };
    if ret != 0 {
        B::println(format_args!("Error creating network poll task"));
        B::exit_failure();
    }

    // Brief delay so the poll task flushes stale RX + the TAP settles.
    unsafe {
        unsafe extern "C" {
            fn vTaskDelay(ticks: u32);
        }
        vTaskDelay(2000);
    }

    let netif_state = unsafe { nros_freertos_get_netif_state() };
    if netif_state & 0xF != 0xF {
        B::println(format_args!(
            "WARNING: lwIP netif not ready (default={} up={} link={} ip={})",
            netif_state & 1 != 0,
            netif_state & 2 != 0,
            netif_state & 4 != 0,
            netif_state & 8 != 0,
        ));
    }
}

// Task-context heap goes through the canonical platform ABI (RFC-0034 /
// phase-230 1e). On FreeRTOS boards `nros_platform_alloc` wraps `pvPortMalloc`
// (heap_4) — same heap, single funnel.
unsafe extern "C" {
    fn nros_platform_alloc(size: usize) -> *mut c_void;
    fn nros_platform_dealloc(ptr: *mut c_void);
}

/// Heap context for the boot (per-tier) app task.
struct AppContextTiers<F> {
    config: Config,
    /// Issue #98 / RFC-0045 — baked `.nros_boot_config` for node-name resolution.
    boot_config: Option<&'static BakedBootConfig>,
    tiers: &'static [TierSpec<'static>],
    setup: F,
}

/// Heap context handed to each spawned (non-boot) tier task.
struct TierTaskCtx<F> {
    session: ::nros::SessionHandle,
    tier: TierSpec<'static>,
    /// Tiers still to spawn AFTER this one — the chained-spawn tail
    /// (issue #144). This tier spawns `rest[0]` (carrying `rest[1..]`) only
    /// after its own setup returns, so no two setups overlap.
    rest: &'static [TierSpec<'static>],
    /// Fallback stack (words) for a spawned tier whose `stack_bytes == 0`,
    /// threaded down the chain since the tier tasks no longer see `Config`.
    app_stack_default_words: u32,
    setup: F,
}

/// issue #144 — chained tier spawn for FreeRTOS. `remaining.split_first()` is
/// the next tier to bring up and the tail it carries; empty → `Ok`. Spawns
/// exactly ONE FreeRTOS task for `remaining[0]`, handing it `remaining[1..]`
/// as its own `rest` so the chain continues once its setup completes.
/// Serializing spawns behind each setup guarantees no two `setup()` (entity
/// declare) calls run concurrently on the shared zenoh-pico session — the
/// interest-handshake race that silently closes a losing publisher's write
/// filter. On alloc/spawn failure returns `Err(())`; the caller decides
/// whether that is fatal (boot) or merely logged (a spawned tier).
///
/// `app_stack_default_words` supplies the stack for a tier whose
/// `stack_bytes == 0` (the tier tasks no longer capture `Config`).
fn spawn_next_tier<B, F, E>(
    session: ::nros::SessionHandle,
    remaining: &'static [TierSpec<'static>],
    app_stack_default_words: u32,
    setup: F,
) -> core::result::Result<(), ()>
where
    B: BoardPrint + BoardExit,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    let Some((tier, rest)) = remaining.split_first() else {
        return Ok(());
    };
    let tier_ctx = TierTaskCtx::<F> {
        session,
        tier: *tier,
        rest,
        app_stack_default_words,
        setup,
    };
    let size = core::mem::size_of::<TierTaskCtx<F>>();
    let ptr = unsafe { nros_platform_alloc(size) as *mut TierTaskCtx<F> };
    if ptr.is_null() {
        B::println(format_args!("nros: tier `{}` ctx alloc failed", tier.name));
        return Err(());
    }
    unsafe { core::ptr::write(ptr, tier_ctx) };
    // Raw per-RTOS priority (the author wrote the FreeRTOS value directly).
    let prio = tier.priority.clamp(0, u32::MAX as i64) as u32;
    let stack_words = if tier.stack_bytes == 0 {
        app_stack_default_words
    } else {
        (tier.stack_bytes / 4) as u32
    };
    let ret = unsafe {
        nros_freertos_create_task(
            tier_task_entry::<B, F, E>,
            b"nros_tier\0".as_ptr(),
            stack_words,
            ptr as *mut c_void,
            prio,
        )
    };
    if ret != 0 {
        B::println(format_args!("nros: failed to spawn tier `{}`", tier.name));
        unsafe { nros_platform_dealloc(ptr as *mut c_void) };
        return Err(());
    }
    Ok(())
}

/// Spawned tier task: open an `Executor` over the shared session, install this
/// tier's `active_groups` filter, register (the off-tier callbacks are gated
/// out), spawn the NEXT tier once its own setup returns (issue #144 chained
/// spawn — a downstream spawn failure is logged, not fatal), then spin forever
/// at the tier's period.
///
/// # Safety
/// `arg` is an `nros_platform_alloc`-allocated `TierTaskCtx<F>` from
/// `app_task_entry_tiers`; this task consumes + frees it.
unsafe extern "C" fn tier_task_entry<B, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    let ctx = unsafe { core::ptr::read(arg as *mut TierTaskCtx<F>) };
    unsafe { nros_platform_dealloc(arg) };

    // SAFETY: the boot task owns the session for the firmware lifetime (its spin
    // loop never returns), so the handle stays valid.
    let executor = unsafe { ::nros::Executor::open_with_session_handle(ctx.session) };
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
    crt.executor_mut().set_active_groups(ctx.tier.groups);
    // W5.4 — shared tier→SchedContext lowering (Sporadic / EDF / TT).
    crt.apply_tier_sched_policy(
        ctx.tier.class,
        ctx.tier.period_us,
        ctx.tier.budget_us,
        ctx.tier.deadline_us,
        ctx.tier.deadline_policy,
    );
    {
        let mut runtime = RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = (ctx.setup)(&mut runtime) {
            // The chain is serialized (issue #144): this tier spawns the next
            // only after its own setup returns Ok, so a failure here HALTS the
            // chain — the downstream tiers (`ctx.rest`) will not start. This
            // path then aborts the firmware (pre-#144 `exit_failure` behavior),
            // so the halt is loud, not silent.
            B::println(format_args!(
                "nros: tier `{}` setup failed: {:?} — {} downstream tier(s) will NOT start",
                ctx.tier.name,
                e,
                ctx.rest.len()
            ));
            B::exit_failure();
        }
    }
    // issue #144 — this tier's setup is done, so bringing up the next tier can
    // no longer race our declares: spawn `rest[0]` (carrying `rest[1..]`). Mint
    // a fresh handle off this tier's executor (same as the boot path —
    // `ctx.session` was consumed opening the executor above). A failed
    // DOWNSTREAM spawn must NOT stop this tier spinning its own work, so warn +
    // continue (do NOT exit_failure).
    let next_session = crt.executor_mut().session_handle();
    if spawn_next_tier::<B, F, E>(
        next_session,
        ctx.rest,
        ctx.app_stack_default_words,
        ctx.setup,
    )
    .is_err()
    {
        B::println(format_args!(
            "nros: tier `{}` failed to spawn next tier; continuing",
            ctx.tier.name
        ));
    }
    let period_ms = (ctx.tier.spin_period_us / 1000).max(1) as u32;
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired, so under
    // sustained traffic this loop would otherwise never block, and a task that
    // never blocks never lets a lower-priority tier run. Costs nothing while
    // the spins do block.
    let mut gap = ::nros_platform::TierSpinGap::new(ctx.tier.spin_period_us);
    loop {
        let iter = gap.mark();
        if let Err(err) = ::nros_platform::NodeDispatchRuntime::spin_once(&mut crt, period_ms) {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!(
                "nros: tier `{}` spin error: {:?}",
                ctx.tier.name, err
            ));
            B::exit_failure();
        }
        gap.after_spin(iter);
    }
}

/// Boot app task for the per-tier model: bring up the network, open the one
/// session, run the boot tier's setup, then CHAIN-spawn the remaining tiers —
/// boot spawns tiers[1] (carrying tiers[2..]); each tier spawns the next only
/// after its own setup returns (issue #144) — and finally run the
/// highest-priority tier (tiers[0]) on this task. Chaining keeps setup order
/// total so no two tiers' entity declares race the shared session's interest
/// handshake; the boot tier's declares run before any spawn.
///
/// # Safety
/// `arg` is an `nros_platform_alloc`-allocated `AppContextTiers<F>` from
/// `run_tiers_entry`, surviving until the scheduler exits.
unsafe extern "C" fn app_task_entry_tiers<B, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &mut *(arg as *mut AppContextTiers<F>) };
    unsafe { freertos_boot_bringup::<B>(&ctx.config) };

    // #191 — same `log` sink wiring as the single-task entry above.
    install_uart_logger::<B>();

    if ctx.tiers.is_empty() {
        B::println(format_args!("nros: run_tiers called with no tiers"));
        B::exit_failure();
    }

    // The boot task was created at the generic `app_priority`, but from here
    // on it IS tiers[0] (the highest tier: session open, tier setup, then the
    // tier spin loop, forever). Spawned tiers are born at their declared
    // priority (see `spawn_next_tier`); without this the boot tier silently
    // ran at the app default (normalized 12, FreeRTOS 3) regardless of what
    // the system declared, so "high tier" priority ablations did not change
    // the boot tier at all. Assume the declared priority now, before the
    // session opens. Same raw-units conversion as `spawn_next_tier`; the C
    // helper clamps to configMAX_PRIORITIES - 1.
    //
    // issue 0636 — and it is the LEAST urgent tier, not `tiers[0]`.
    //
    // `resolve_tiers` orders by raw number DESCENDING without inverting per
    // kernel, and FreeRTOS is bigger-number-wins, so `tiers[0]` was the MOST
    // urgent tier here. An owner that outranks its peers and then spins starves
    // them: FreeRTOS runs the highest-priority READY task, and a spin loop does
    // not reliably stop being ready. `17666723d` fixed exactly this on the Rust
    // NuttX/Linux arms and left `freertos` behind; this is that half.
    //
    // The least urgent tier is the LAST element of a descending table, which
    // also keeps the remaining tiers CONTIGUOUS — required, because the chain
    // hands each tier a `rest` SLICE and skipping an interior index would
    // change that protocol. Hence not `nros_platform::boot_tier_index`, which
    // the non-chaining arms call: same rule, different mechanics.
    //
    // CHECKED, not assumed: a table that is not non-increasing means the
    // emitter's contract changed, and the alternative failure is silent
    // starvation seconds after boot on one platform.
    let boot_index = if ctx
        .tiers
        .windows(2)
        .any(|w| w[1].priority > w[0].priority)
    {
        B::println(format_args!(
            "nros: tier table is not sorted highest-priority-first — \
             boot tier falls back to index 0 (issue 0636)"
        ));
        0
    } else {
        ctx.tiers.len() - 1
    };
    let boot_prio = ctx.tiers[boot_index].priority.clamp(0, u32::MAX as i64) as u32;
    unsafe { nros_freertos_set_current_task_priority(boot_prio) };

    // Open the one session on the boot task, then move it into its final
    // location (`crt`) BEFORE handing out `SessionHandle`s — the handle aliases
    // `crt`'s owned session, and `crt` never moves again (the boot spin loop
    // below never returns), so the spawned tasks' pointers stay valid.
    // Issue #98 / RFC-0045 — node name from the baked `.nros_boot_config`.
    let baked = ctx
        .boot_config
        .map(::nros::BootConfig::from_baked)
        .unwrap_or_default();
    let exec_cfg = ::nros::ExecutorConfig::resolve(::nros::BootConfig {
        node_name: baked.node_name.or(Some("nros_app")),
        locator: Some(ctx.config.base.zenoh_locator),
        domain_id: Some(ctx.config.base.domain_id),
        namespace: None,
        // Issue 1050 defect (3) — the baked rung reaches the resolver. This
        // board has no environment to read, so before it existed an image here
        // could not name its backend at all.
        rmw: baked.rmw,
    });
    let boot_exec = match ::nros::Executor::open(&exec_cfg) {
        Ok(e) => e,
        Err(err) => {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!("Executor::open failed: {:?}", err));
            B::exit_failure();
        }
    };
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(boot_exec);

    // issue #144 — boot-tier setup FIRST, tier spawn after (previously this
    // spawned ALL of tiers[1..] BEFORE boot setup, so it had the boot↔tier
    // race too). Entity declares carry an interest handshake, and concurrent
    // declares from two threads race it — the losing publisher's write filter
    // stays closed and every put is silently dropped. Running boot's declares
    // before ANY spawn, then CHAINING the remaining spawns (boot spawns
    // tiers[1] only; each tier spawns the next after its own setup returns),
    // makes setup order total (boot, t1, t2, …) so no two declares overlap.
    // Spins still overlap the next tier's setup, which is SAFE — a spin
    // exchanges keepalives/data, not declares.
    let boot_tier = ctx.tiers[boot_index];
    crt.executor_mut().set_active_groups(boot_tier.groups);
    {
        let mut runtime = RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = (ctx.setup)(&mut runtime) {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!("Application error: {:?}", e));
            B::exit_failure();
        }
    }

    // Kick off the chain with everything except the boot tier, which runs on
    // this task. Both arrangements leave a contiguous run (issue 0636): the
    // boot tier is the last element, or index 0 on the unsorted fallback.
    // A boot-side spawn failure is fatal (exit_failure) — unlike a downstream
    // tier's.
    let rest = if boot_index == 0 {
        &ctx.tiers[1..]
    } else {
        &ctx.tiers[..ctx.tiers.len() - 1]
    };
    let app_stack_default_words = ctx.config.app_stack_bytes / 4;
    if spawn_next_tier::<B, F, E>(
        crt.executor_mut().session_handle(),
        rest,
        app_stack_default_words,
        ctx.setup,
    )
    .is_err()
    {
        B::exit_failure();
    }

    B::println(format_args!(""));
    B::println(format_args!(
        "Multi-tier setup complete — entering boot-tier spin loop."
    ));
    let period_ms = (boot_tier.spin_period_us / 1000).max(1) as u32;
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired, so under
    // sustained traffic this loop would otherwise never block, and a task that
    // never blocks never lets a lower-priority tier run. Costs nothing while
    // the spins do block.
    let mut gap = ::nros_platform::TierSpinGap::new(boot_tier.spin_period_us);
    loop {
        let iter = gap.mark();
        if let Err(err) = ::nros_platform::NodeDispatchRuntime::spin_once(&mut crt, period_ms) {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!("spin_once error: {:?}", err));
            B::exit_failure();
        }
        gap.after_spin(iter);
    }
}

/// Phase 228.E.2 — per-tier FreeRTOS entry. The `nros::main!()` macro emits
/// `<Board>::run_tiers(TIERS, run_plan)`; the board ZST routes here. Mirrors
/// [`run_entry`] but runs one FreeRTOS task per priority tier over one shared
/// session, the non-boot tiers CHAIN-spawned so their setups serialize
/// (issue #144; RFC-0032 §5; MT=1 is the default on FreeRTOS, §5.0). `tiers` are the
/// macro-baked `&'static [TierSpec]`; `setup` is the register-only `run_plan`
/// (invoked once per tier, hence `Fn + Copy`).
///
/// `boot_config` — the baked `.nros_boot_config` static, supplied by the
/// per-board `run_tiers` (issue #98 / RFC-0045). `None` keeps `"nros_app"`.
pub fn run_tiers_entry<B, F, E>(
    config: Config,
    boot_config: Option<&'static BakedBootConfig>,
    tiers: &'static [TierSpec<'static>],
    setup: F,
) -> core::result::Result<(), E>
where
    B: BoardInit + BoardPrint + BoardExit,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    B::println(format_args!(""));
    B::println(format_args!("========================================"));
    B::println(format_args!("  nros FreeRTOS Platform (multi-tier)"));
    B::println(format_args!("========================================"));
    B::println(format_args!(""));

    B::init_hardware();
    // Wire the default nros_log sink (platform console) at the boot funnel,
    // as the linux/zephyr run_tiers do — idempotent; without it Node-pkg
    // `log_info!` output is silently dropped on multi-tier entries.
    ::nros_platform_cffi::log::init_default();

    report_tiers_above_transport::<B>(&config, tiers);

    let app_pri = config.app_priority as u32; // issue 0623 — already raw
    let app_stack_words = config.app_stack_bytes / 4;

    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<AppContextTiers<F>>();
        let ptr = nros_platform_alloc(size) as *mut AppContextTiers<F>;
        assert!(!ptr.is_null(), "Failed to allocate AppContextTiers");
        core::ptr::write(
            ptr,
            AppContextTiers {
                config,
                boot_config,
                tiers,
                setup,
            },
        );
        ptr
    };

    let ret = unsafe {
        nros_freertos_create_task(
            app_task_entry_tiers::<B, F, E>,
            b"nros_app\0".as_ptr(),
            app_stack_words,
            ctx_ptr as *mut c_void,
            app_pri,
        )
    };
    if ret != 0 {
        B::println(format_args!("Error creating application task"));
        B::exit_failure();
    }

    unsafe {
        nros_freertos_start_scheduler();
    }

    B::exit_failure()
}

/// Report tiers that will PREEMPT the transport band, naming both numbers in
/// FreeRTOS units (issue 0623).
///
/// Tier and transport priorities now share ONE vocabulary — raw FreeRTOS — so
/// this compares two numbers written in the same units (issue 0623). It used to
/// have to explain a mapping: a tier priority was authored RAW while the
/// transport knobs were on a normalised 0–31 band that the board mapped down
/// (`zenoh_read_priority = 16` became FreeRTOS **4**), so an author who wrote 5
/// against a transport that read "16" got a tier ABOVE the transport band,
/// having reasonably concluded the opposite.
///
/// That is not a hypothetical. `nano-ros-rt-eval` ran tiers at 5/4/2 for
/// exactly this reason; the starved RX drain dropped frames, every publisher
/// stalled on lwIP retransmission timeouts, and the island froze for 1–3 s at a
/// time. Its `system.toml` now runs 3/2/1 with a comment explaining the mapping
/// — knowledge that lived in one consumer's config file and nowhere in nano-ros.
///
/// Deliberately a REPORT, not an error. A tier that must preempt transport is a
/// legitimate design (hard-RT control that would rather lose packets than a
/// deadline); what is not legitimate is choosing it by accident. Both effective
/// values are printed so the comparison needs no arithmetic from the reader.
fn report_tiers_above_transport<B: BoardPrint>(
    config: &Config,
    tiers: &'static [TierSpec<'static>],
) {
    // The band's FLOOR: the lowest-priority transport task is the first to be
    // starved, so it decides whether transport makes progress at all.
    let read = config.zenoh_read_priority as u32;
    let lease = config.zenoh_lease_priority as u32;
    let poll = config.poll_priority as u32;
    let floor = read.min(lease).min(poll);

    let mut offenders = 0usize;
    for tier in tiers {
        let prio = tier.priority.clamp(0, u32::MAX as i64) as u32;
        if prio >= floor {
            if offenders == 0 {
                B::println(format_args!(
                    "nros: tier priority meets the transport band (FreeRTOS units):"
                ));
                B::println(format_args!(
                    "  transport: zenoh_read {read}, zenoh_lease {lease}, net_poll {poll} (floor {floor})"
                ));
            }
            offenders += 1;
            B::println(format_args!(
                "  tier `{}` at {prio} >= {floor} — this tier PREEMPTS transport I/O",
                tier.name
            ));
        }
    }
    if offenders > 0 {
        B::println(format_args!(
            "  Intended? then nothing to do. Both numbers above are RAW FreeRTOS \
             units, the same ones a `[tiers.<name>.freertos] priority` is written in. \
             If NOT intended: starving the RX drain stalls publishers on lwIP \
             retransmission, which costs far more than the deadline it was meant to \
             protect (issues 0506, 0623)."
        ));
    }
}

fn init_network(config: &Config) -> FrResult<()> {
    let ret = unsafe {
        nros_freertos_init_network(
            config.base.mac.as_ptr(),
            config.base.ip.as_ptr(),
            config.base.netmask.as_ptr(),
            config.base.gateway.as_ptr(),
        )
    };
    if ret != 0 {
        return Err(Error::NetworkInit);
    }
    Ok(())
}

/// Phase 212.N.2 — family-driver entry point for FreeRTOS boards.
///
/// Mirrors the legacy [`crate::run`] body — allocates an app task on
/// the FreeRTOS heap, hands it the user closure, calls
/// `vTaskStartScheduler()`, never returns — but routes through the
/// 212.N.1 `nros_platform::board` trait set + [`RuntimeCtx`].
///
/// Per-board crates (e.g. `nros-board-mps2-an385-freertos`) wire
/// this into their `impl BoardEntry for Self::run` body in 212.N.3:
///
/// ```ignore
/// impl nros_platform::board::BoardEntry for MyBoard {
///     fn run<F, E>(setup: F) -> Result<(), E>
///     where
///         F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
///         E: core::fmt::Debug,
///     {
///         let cfg = Config::default();
///         nros_board_freertos::run_entry::<MyBoard, F, E>(cfg, None, setup)
///     }
/// }
/// ```
///
/// # Type parameters
///
/// - `B: BoardInit + BoardPrint + BoardExit` — per-board glue
///   pulled from `nros_platform::board` (212.N.1 surface).
/// - `F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>` — user
///   closure receiving the runtime context.
/// - `E: core::fmt::Debug` — closure error type.
///
/// # Return
///
/// The signature is `Result<(), E>` to satisfy the
/// [`nros_platform::BoardEntry::run`] trait contract, but in
/// practice the kernel-spawn flow never returns to the caller —
/// either the scheduler runs forever and the app task drives
/// `exit_success` / `exit_failure`, or scheduler startup itself
/// fails and we `exit_failure` defensively. The `Ok(())` arm exists
/// only so the function signature lines up with the trait; it is
/// unreachable in a working build.
///
/// `boot_config` — the baked `.nros_boot_config` static, passed from the
/// per-board `run_with_deploy` (issue #98 / RFC-0045). `None` keeps the
/// historical `"nros_app"` node-name default.
pub fn run_entry<B, F, E>(
    config: Config,
    boot_config: Option<&'static BakedBootConfig>,
    setup: F,
) -> core::result::Result<(), E>
where
    B: BoardInit + BoardPrint + BoardExit,
    F: FnOnce(&mut RuntimeCtx<'_>) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    // A record raised inside a LIBRARY (the zenoh session-pool diagnostic of
    // issue 0589, for one) is dropped until a sink list exists, and its author
    // cannot know whether the board published one. Idempotent.
    ::nros_platform_cffi::log::init_default();
    B::println(format_args!(""));
    B::println(format_args!("========================================"));
    B::println(format_args!("  nros FreeRTOS Platform"));
    B::println(format_args!("========================================"));
    B::println(format_args!(""));

    B::println(format_args!("Initializing LAN9118 + lwIP..."));
    B::println(format_args!(
        "  MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        config.base.mac[0],
        config.base.mac[1],
        config.base.mac[2],
        config.base.mac[3],
        config.base.mac[4],
        config.base.mac[5],
    ));
    B::println(format_args!(
        "  IP:  {}.{}.{}.{}",
        config.base.ip[0], config.base.ip[1], config.base.ip[2], config.base.ip[3],
    ));

    // Per-board pre-scheduler init. New 212.N.1 `BoardInit::init_hardware`
    // is parameterless — board crates read any needed config off their
    // own `pub const` / `pub static` rather than a passed-in arg.
    B::init_hardware();

    let app_pri = config.app_priority as u32; // issue 0623 — already raw
    let app_stack_words = config.app_stack_bytes / 4;

    // Heap-allocate the app context. Pre-scheduler MSP stack is
    // reclaimed by FreeRTOS when `vPortStartFirstTask()` resets MSP
    // to `_estack`, so locals would be clobbered by the next
    // exception that stacks on MSP. (Same rationale as legacy `run`.)
    // `nros_platform_alloc` is declared at module scope (heap_4 funnel).
    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<AppContext<F>>();
        let ptr = nros_platform_alloc(size) as *mut AppContext<F>;
        assert!(!ptr.is_null(), "Failed to allocate AppContext");
        core::ptr::write(
            ptr,
            AppContext {
                config,
                boot_config,
                closure: setup,
            },
        );
        ptr
    };

    let ret = unsafe {
        nros_freertos_create_task(
            app_task_entry_runtime::<B, F, E>,
            b"nros_app\0".as_ptr(),
            app_stack_words,
            ctx_ptr as *mut c_void,
            app_pri,
        )
    };
    if ret != 0 {
        B::println(format_args!("Error creating application task"));
        B::exit_failure();
    }

    unsafe {
        nros_freertos_start_scheduler();
    }

    // Unreachable — scheduler never returns. `exit_failure()`
    // diverges (`-> !`), so this satisfies the `Result<(), E>`
    // signature without an explicit `Ok` arm.
    B::exit_failure()
}

/// Phase 313 W-freertos (#0243) — lightweight NO-SESSION FreeRTOS entry for
/// logging / init-only fixtures AND no-session apps that manage their own
/// executor (e.g. the `wake-latency-cortex-m3` bench, which opens its own
/// `Executor::open`). Boots the scheduler + shared bringup (network, RNG, poll
/// task, netif wait) then runs `setup(&Config)` WITHOUT the board opening an
/// `Executor` session (unlike [`run_entry`], which would fail
/// `Transport(ConnectionFailed)` with no router). The `&Config` mirrors the
/// legacy `node::run(Config, closure)` this replaces (and the direct-exec
/// `run_bare`), so config-reading apps keep working; logging fixtures that don't
/// need it take `|_cfg|`.
pub fn run_bare<B, F, E>(config: Config, setup: F) -> core::result::Result<(), E>
where
    B: BoardInit + BoardPrint + BoardExit,
    F: FnOnce(&Config) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    // A record raised inside a LIBRARY (the zenoh session-pool diagnostic of
    // issue 0589, for one) is dropped until a sink list exists, and its author
    // cannot know whether the board published one. Idempotent.
    ::nros_platform_cffi::log::init_default();
    B::println(format_args!(""));
    B::println(format_args!("========================================"));
    B::println(format_args!("  nros FreeRTOS Platform (bare)"));
    B::println(format_args!("========================================"));
    B::println(format_args!(""));

    B::init_hardware();

    let app_pri = config.app_priority as u32; // issue 0623 — already raw
    let app_stack_words = config.app_stack_bytes / 4;

    // Heap-allocate the app context (same MSP-reclaim rationale as `run_entry`).
    // `boot_config` is unused on the no-session path.
    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<AppContext<F>>();
        let ptr = nros_platform_alloc(size) as *mut AppContext<F>;
        assert!(!ptr.is_null(), "Failed to allocate AppContext");
        core::ptr::write(
            ptr,
            AppContext {
                config,
                boot_config: None,
                closure: setup,
            },
        );
        ptr
    };

    let ret = unsafe {
        nros_freertos_create_task(
            app_task_entry_bare::<B, F, E>,
            b"nros_app\0".as_ptr(),
            app_stack_words,
            ctx_ptr as *mut c_void,
            app_pri,
        )
    };
    if ret != 0 {
        B::println(format_args!("Error creating application task"));
        B::exit_failure();
    }

    unsafe {
        nros_freertos_start_scheduler();
    }

    B::exit_failure()
}

/// FreeRTOS task entry for the NO-SESSION [`run_bare`] path — runs the shared
/// boot bringup then the nullary closure with NO `Executor::open`. The `Ok` arm
/// prints "Application completed successfully." (the completion banner logging
/// fixtures grep for) before `exit_success`.
///
/// # Safety
/// `arg` must point to a valid `AppContext<F>` allocated on the FreeRTOS heap by
/// [`run_bare`], surviving until the scheduler exits.
unsafe extern "C" fn app_task_entry_bare<B, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    F: FnOnce(&Config) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &mut *(arg as *mut AppContext<F>) };

    unsafe { freertos_boot_bringup::<B>(&ctx.config) };
    install_uart_logger::<B>();

    // FnOnce — `core::ptr::read` because this task entry runs once.
    let closure = unsafe { core::ptr::read(&ctx.closure) };

    match closure(&ctx.config) {
        Ok(()) => {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!(""));
            B::println(format_args!("Application completed successfully."));
            B::exit_success();
        }
        Err(e) => {
            unsafe {
                nros_trace_trigger_and_dump();
            }
            B::println(format_args!(""));
            B::println(format_args!("Application error: {:?}", e));
            B::exit_failure();
        }
    }
}
