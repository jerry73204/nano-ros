//! Phase 212.N.2 — `BoardEntry::run` shim for the ThreadX family.
//!
//! Adds an additive entry point built on the 212.N.1 trait set in
//! `nros_platform::board` (`BoardInit` parameterless, `BoardPrint`,
//! `BoardExit`, `RuntimeCtx`). Mirrors the legacy [`crate::run`]
//! body — kernel-spawn shape: stash the user closure into static
//! storage, push the network config + app callback through the C
//! glue (`nros_threadx_set_config` + `nros_threadx_set_app_callback`),
//! call `tx_kernel_enter()`, never return — but threads the new
//! `RuntimeCtx` through the user setup callback instead of an opaque
//! `&Config`.
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
//! ThreadX bring-up needs a board `Config` (MAC / IP / netmask /
//! gateway / interface) — that lives outside `RuntimeCtx` (codegen
//! overlay knobs, not hardware config). The per-board crate
//! (`nros-board-threadx-linux`, `nros-board-threadx-qemu-riscv64`)
//! owns the `Config` source (TOML / `Config::default()`); it
//! implements `BoardEntry` directly and delegates here:
//!
//! ```ignore
//! impl BoardEntry for MyBoard {
//!     fn run<F, E>(setup: F) -> Result<(), E>
//!     where
//!         F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
//!         E: core::fmt::Debug,
//!     {
//!         let cfg = Config::default();
//!         nros_board_threadx::run_entry::<MyBoard, Config, F, E>(cfg, setup)
//!     }
//! }
//! ```
//!
//! 212.N.3 wires that into `nros-board-threadx-linux` +
//! `nros-board-threadx-qemu-riscv64`; this file just provides the
//! family-side helper. The legacy [`crate::run`] coexists during the
//! 212.N transition.

use core::{
    ffi::c_void,
    sync::atomic::{AtomicUsize, Ordering},
};

use nros_board_common::ThreadxConfig;
use nros_platform::{BakedBootConfig, BoardExit, BoardInit, BoardPrint, RuntimeCtx, TierSpec};

// #131 — no_std `log` sink routing to the board's UART. The concrete print path
// depends on the board type `B`, which a `static` logger can't name, so the
// logger reads a function pointer set (once, monomorphised for `B`) by
// `install_uart_logger::<B>()`. `log::info!` from the examples then reaches the
// console; without a registered logger the `log` facade silently drops records
// (bare-metal has no default stdout), so the harness never sees `Publishing:` /
// `I heard:`. Mirrors the nuttx board's stdout logger (which can use `std`).
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
            // The examples bake the full human line into the message, so the
            // BODY goes through verbatim; the `[LEVEL]` prefix rides in front
            // of it, matching the linux/nuttx/mps2 sinks and `nros_log`'s own
            // (issue 0309 made the logging proof assert the tag on the marker
            // line, so a sink that drops it renders every `log::` record as
            // the bare-`printf` bypass that proof exists to catch).
            //
            // `format_args!` can't be bound to a `let` — its temporaries live
            // only for the enclosing expression — so it is built in the call.
            f(format_args!("[{}] {}", record.level(), record.args()));
        }
    }

    fn flush(&self) {}
}

static UART_LOGGER: UartLogger = UartLogger;

/// Install the UART `log` sink, routing records through `B::println`. Idempotent:
/// re-arms the print fn each call and ignores a repeated `set_logger` (the second
/// returns `Err`). Safe to call once per boot before the spin loop.
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

unsafe extern "C" {
    fn nros_threadx_set_config(
        ip: *const u8,
        netmask: *const u8,
        gateway: *const u8,
        mac: *const u8,
        interface_name: *const u8,
    );

    fn nros_threadx_set_app_callback(entry: unsafe extern "C" fn(*mut c_void), arg: *mut c_void);

    /// Phase 297 W2/W4 (RFC-0053) — the shared ThreadX thread-creation backend
    /// (`nros_board_common`'s `threadx_hooks.c`). Mirrors the FreeRTOS
    /// `nros_freertos_create_task` shape; the single spawn path `run_tiers`
    /// (W4) and any C/C++ entry both call. `entry`/`arg` are POINTER-WIDTH
    /// (`void(*)(void*)` / `void*`), NOT ThreadX ULONG: the x86_64 linux port
    /// defines ULONG as `unsigned int` (32-bit), so a pointer through the
    /// thread input truncates and SEGVs (found at first W5 boot). The shim
    /// parks {entry, arg} in a slot table and passes only the slot index
    /// through the ULONG input to a C trampoline. `stack_bytes` (0 ⇒ the
    /// overlay's app-stack size) is `tx_byte_allocate`d from the shared byte
    /// pool (same as the boot app thread). `preempt_threshold < 0` ⇒
    /// `= priority` (no threshold); `>= 0` is the native `non_preempt_scope`
    /// value. Returns 0 on success, -1 on failure.
    /// `time_slice_us` (#0266): 0 ⇒ `TX_NO_TIME_SLICE` (FIFO-until-block, the
    /// ThreadX default); nonzero requests round-robin among same-priority tiers
    /// (converted µs→ticks in the C shim where the tick constant lives).
    fn nros_threadx_create_task(
        name: *const u8,
        entry: unsafe extern "C" fn(*mut c_void),
        arg: *mut c_void,
        stack_bytes: core::ffi::c_ulong,
        priority: core::ffi::c_uint,
        preempt_threshold: core::ffi::c_int,
        time_slice_us: core::ffi::c_ulong,
    ) -> core::ffi::c_int;

    /// Re-prioritize the CALLING thread to a tier's sched values (W5). The
    /// boot tier runs on the app thread, created at the fixed
    /// `nros_board_app_priority()` — without this, a boot tier whose declared
    /// priority is numerically above a spawned tier's silently inverts the
    /// tiers (the W5 e2e caught exactly that: app@4 starved `high`@5).
    /// `preempt_threshold < 0` ⇒ `= priority`. Returns 0 on success.
    fn nros_threadx_set_current_priority(
        priority: core::ffi::c_uint,
        preempt_threshold: core::ffi::c_int,
    ) -> core::ffi::c_int;

    /// phase-296 W5.13 — self-apply the CALLING thread's SMP core pin (the
    /// placement dim) via `tx_thread_smp_core_exclude`. `core_plus1` is the
    /// `core + 1` encoding (0 = unpinned). Returns 1 when the kernel accepted
    /// the pin, 0 otherwise (no `core` / no TX_THREAD_SMP / rejection — the
    /// Rust caller prints the accept marker or the loud fallback).
    fn nros_threadx_apply_current_core_exclude(core_plus1: core::ffi::c_uint) -> core::ffi::c_int;

    /// phase-296 #0266 — self-apply the CALLING (boot) thread's round-robin time
    /// slice via `tx_thread_time_slice_change`. `time_slice_us` 0 ⇒ no slice.
    /// Returns 1 when a slice was set, 0 otherwise (unrequested / rejection).
    fn nros_threadx_apply_current_time_slice(time_slice_us: core::ffi::c_ulong)
    -> core::ffi::c_int;

    /// Allocate `bytes` from the shared ThreadX byte pool (W4 — per-tier heap
    /// context). Mirrors the FreeRTOS `nros_platform_alloc`. NULL on failure.
    fn nros_threadx_alloc(bytes: core::ffi::c_ulong) -> *mut c_void;

    /// Free a block from [`nros_threadx_alloc`].
    fn nros_threadx_free(ptr: *mut c_void);

    #[link_name = "_tx_initialize_kernel_enter"]
    fn tx_kernel_enter();

    #[link_name = "_tx_thread_sleep"]
    fn tx_thread_sleep(ticks: u32);
}

// ============================================================================
// Phase 297 W4 (RFC-0053) — ThreadX multi-tier `run_tiers`.
//
// One `Executor` per tier over ONE shared RMW session, mirroring the FreeRTOS
// `run_tiers_entry`. The boot tier (`tiers[0]` — the numerically-largest =
// LOWEST ThreadX priority; the sort does not invert per RTOS direction,
// issue 0251) runs on the
// `tx_application_define` app thread; each remaining tier is spawned via the
// W2 `nros_threadx_create_task` shim (byte-pool stack). Spawns are CHAINED —
// a tier spawns the next only after its own `setup` returns — so no two
// tiers' entity declares race the shared zenoh-pico session's interest
// handshake (issue #144). Each spawned tier calls `apply_tier_sched_policy`
// (the common backend, W1) on its own executor.
// ============================================================================

/// Boot context for the multi-tier path, written into `CTX_STORAGE` by
/// [`run_tiers_entry`] and read once on the app thread.
struct TiersContext<C, F> {
    config: C,
    boot_config: Option<&'static BakedBootConfig>,
    tiers: &'static [TierSpec<'static>],
    setup: F,
}

/// Byte-pool context handed to each spawned (non-boot) tier thread. Allocated
/// with [`nros_threadx_alloc`], read + freed by [`tier_task_entry`].
struct TierTaskCtx<F> {
    session: ::nros::SessionHandle,
    tier: TierSpec<'static>,
    /// Tiers still to spawn AFTER this one — the chained-spawn tail (#144).
    rest: &'static [TierSpec<'static>],
    setup: F,
}

/// Lower a `TierSpec` onto the executor runtime: the active-groups filter + the
/// common-backend `apply_tier_sched_policy` (W1). Shared by the boot tier and
/// every spawned tier so the lowering never diverges.
fn apply_tier(crt: &mut ::nros::node_runtime::ExecutorNodeRuntime, tier: &TierSpec<'static>) {
    crt.executor_mut().set_active_groups(tier.groups);
    crt.apply_tier_sched_policy(
        tier.class,
        tier.period_us,
        tier.budget_us,
        tier.deadline_us,
        tier.deadline_policy,
    );
}

/// phase-296 W5.13 — the placement dim: self-pin the CALLING tier thread to its
/// declared `core` via `tx_thread_smp_core_exclude` (no-op when the tier
/// declares no core). Never silently drops: on a non-SMP ThreadX build the
/// shim returns 0 and the loud fallback note fires. Called from BOTH the boot
/// and every spawned tier so the placement lowering never diverges.
fn apply_tier_core_exclude<B: BoardPrint>(tier: &TierSpec<'static>) {
    let Some(core) = tier.core else {
        return;
    };
    let core_plus1 = core.saturating_add(1) as core::ffi::c_uint;
    // SAFETY: FFI to the board-common ThreadX shim; no aliasing, no allocation.
    let accepted = unsafe { nros_threadx_apply_current_core_exclude(core_plus1) };
    if accepted == 1 {
        // Literal mirrors `nros_tests::output::THREADX_CORE_PIN_MARKER`.
        B::println(format_args!(
            "nros: core pin tier=`{}` cpu={}",
            tier.name, core
        ));
    } else {
        // Literal mirrors `nros_tests::output::THREADX_CORE_PIN_FALLBACK_MARKER`.
        B::println(format_args!(
            "nros: core pin FAILED tier=`{}` cpu={} — ThreadX build lacks TX_THREAD_SMP, \
             tier runs unpinned",
            tier.name, core
        ));
    }
}

/// issue #144 — chained tier spawn. Spawns exactly ONE ThreadX thread for
/// `remaining[0]`, handing it `remaining[1..]` as its own `rest` so the chain
/// continues once that tier's setup completes. Empty `remaining` → `Ok`.
/// `Err(())` on ctx-alloc or spawn failure (the caller decides fatality).
fn spawn_next_tier<B, C, F, E>(
    session: ::nros::SessionHandle,
    remaining: &'static [TierSpec<'static>],
    setup: F,
) -> core::result::Result<(), ()>
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    let Some((tier, rest)) = remaining.split_first() else {
        return Ok(());
    };
    let size = core::mem::size_of::<TierTaskCtx<F>>();
    let ptr = unsafe { nros_threadx_alloc(size as core::ffi::c_ulong) as *mut TierTaskCtx<F> };
    if ptr.is_null() {
        B::println(format_args!("nros: tier `{}` ctx alloc failed", tier.name));
        return Err(());
    }
    unsafe {
        core::ptr::write(
            ptr,
            TierTaskCtx::<F> {
                session,
                tier: *tier,
                rest,
                setup,
            },
        );
    }
    // Raw per-RTOS priority; `preempt_threshold` < 0 sentinel ⇒ = priority.
    let prio = tier.priority.clamp(0, u32::MAX as i64) as core::ffi::c_uint;
    let pt: core::ffi::c_int = match tier.preempt_threshold {
        Some(v) => v.clamp(0, u32::MAX as i64) as core::ffi::c_int,
        None => -1,
    };
    let stack_bytes = tier.stack_bytes as core::ffi::c_ulong;
    // #0266 — round-robin time slice (0 ⇒ TX_NO_TIME_SLICE = FIFO).
    let time_slice_us = tier.time_slice_us.unwrap_or(0) as core::ffi::c_ulong;
    let rc = unsafe {
        nros_threadx_create_task(
            b"nros_tier\0".as_ptr(),
            tier_task_entry::<B, C, F, E>,
            ptr as *mut c_void,
            stack_bytes,
            prio,
            pt,
            time_slice_us,
        )
    };
    if rc != 0 {
        B::println(format_args!("nros: failed to spawn tier `{}`", tier.name));
        unsafe { nros_threadx_free(ptr as *mut c_void) };
        return Err(());
    }
    if tier.preempt_threshold.is_some() {
        // phase-296 W5.10 — non_preempt_scope trace marker: tx_thread_create
        // applied the threshold at creation (rc == 0). Literal mirrors
        // `nros_tests::output::THREADX_PREEMPT_MARKER`.
        B::println(format_args!(
            "nros: preempt threshold set tier=`{}` {}",
            tier.name, pt
        ));
    }
    if let Some(ts) = tier.time_slice_us {
        // phase-296 #0266 — time-slice trace marker: tx_thread_create applied
        // the round-robin slice at creation (rc == 0; ThreadX honors a
        // per-thread slice unconditionally). Literal mirrors
        // `nros_tests::output::THREADX_TIME_SLICE_MARKER`.
        B::println(format_args!(
            "nros: time slice set tier=`{}` {}us",
            tier.name, ts
        ));
    }
    Ok(())
}

/// Spawned-tier thread body, reached through the shim's slot-table trampoline
/// (pointer-width `input` — NEVER ThreadX ULONG, which is 32-bit on the
/// x86_64 linux port): `input` is
/// the [`TierTaskCtx`] pointer (from [`spawn_next_tier`]). Opens an `Executor`
/// over the SHARED session, applies this tier's groups + sched policy,
/// registers (off-tier callbacks gated out), chain-spawns the next tier once
/// its own setup returns (#144), then spins forever at the tier's period.
///
/// # Safety
/// `input` must be a `nros_threadx_alloc`-allocated `TierTaskCtx<F>` from
/// [`spawn_next_tier`]; this thread consumes + frees it.
unsafe extern "C" fn tier_task_entry<B, C, F, E>(input: *mut c_void)
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    let arg = input as *mut TierTaskCtx<F>;
    let ctx = unsafe { core::ptr::read(arg) };
    unsafe { nros_threadx_free(arg as *mut c_void) };
    B::println(format_args!("nros: tier `{}` thread up", ctx.tier.name));

    // SAFETY: the boot thread owns the session for the firmware lifetime (its
    // spin loop never returns), so the handle stays valid.
    let executor = unsafe { ::nros::Executor::open_with_session_handle(ctx.session) };
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
    apply_tier(&mut crt, &ctx.tier);
    apply_tier_core_exclude::<B>(&ctx.tier);
    {
        let mut runtime = RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = (ctx.setup)(&mut runtime) {
            B::println(format_args!(
                "nros: tier `{}` setup failed: {:?} — {} downstream tier(s) will NOT start",
                ctx.tier.name,
                e,
                ctx.rest.len()
            ));
            B::exit_failure();
        }
    }
    // #144 — setup done, so bringing up the next tier can no longer race our
    // declares. Mint a fresh handle off this executor (the boot handle was
    // consumed above). A DOWNSTREAM spawn failure must not stop this tier
    // spinning, so warn + continue.
    let next_session = crt.executor_mut().session_handle();
    if spawn_next_tier::<B, C, F, E>(next_session, ctx.rest, ctx.setup).is_err() {
        B::println(format_args!(
            "nros: tier `{}` failed to spawn next tier; continuing",
            ctx.tier.name
        ));
    }
    let period_ms = (ctx.tier.spin_period_us / 1000).max(1) as u32;
    B::println(format_args!(
        "nros: tier `{}` setup complete — spinning at {} ms",
        ctx.tier.name, period_ms
    ));
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired, so under
    // sustained traffic this loop would otherwise never block, and a thread
    // that never blocks never lets a lower-priority tier run. Costs nothing
    // while the spins do block.
    let mut gap = ::nros_platform::TierSpinGap::new(ctx.tier.spin_period_us);
    loop {
        let iter = gap.mark();
        if let Err(err) = ::nros_platform::NodeDispatchRuntime::spin_once(&mut crt, period_ms) {
            B::println(format_args!(
                "nros: tier `{}` spin error: {:?}",
                ctx.tier.name, err
            ));
            B::exit_failure();
        }
        gap.after_spin(iter);
    }
}

/// App-thread body for the multi-tier path (mirrors [`run_app_thread`] but per
/// tier). Boots the one session, runs `tiers[0]` (boot tier) setup, CHAIN-spawns
/// `tiers[1..]`, then spins the boot tier forever. Diverges.
///
/// # Safety
/// `arg` must point at a valid `TiersContext<C, F>` in `CTX_STORAGE` from
/// [`run_tiers_entry`], living until the kernel terminates.
unsafe extern "C" fn app_task_entry_tiers<B, C, F, E>(arg: *mut c_void) -> !
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &*(arg as *const TiersContext<C, F>) };
    let config = unsafe { core::ptr::read(&ctx.config) };
    let boot_config = ctx.boot_config;
    let tiers = ctx.tiers;
    let setup = ctx.setup;

    if tiers.is_empty() {
        B::println(format_args!("nros: run_tiers called with no tiers"));
        B::exit_failure();
    }

    // Issue 0484 — the 2 s "network stabilisation" sleep that used to be here is
    // GONE. See `run_app_thread` below for the measurement.

    let baked = boot_config
        .map(::nros::BootConfig::from_baked)
        .unwrap_or_default();
    let exec_cfg = ::nros::ExecutorConfig::resolve(::nros::BootConfig {
        node_name: baked.node_name.or(Some("nros_app")),
        locator: Some(config.locator()),
        domain_id: Some(config.domain_id()),
        namespace: None,
        // Issue 1050 defect (3) — the baked rung reaches the resolver. This
        // board has no environment to read, so before it existed an image here
        // could not name its backend at all.
        rmw: baked.rmw,
    });

    // #131 — register the linked zenoh CFFI backend before `Executor::open`.
    #[cfg(feature = "rmw-zenoh")]
    if let Err(err) = ::nros_rmw_zenoh::register() {
        B::println(format_args!(
            "nros: zenoh RMW backend register failed: {:?}",
            err
        ));
    }

    let executor = match ::nros::Executor::open(&exec_cfg) {
        Ok(e) => e,
        Err(err) => {
            B::println(format_args!("Executor::open failed: {:?}", err));
            B::exit_failure();
        }
    };
    install_uart_logger::<B>();
    B::println(format_args!("nros entry ready"));

    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);

    // Boot tier = tiers[0]. NOTE `resolve_tiers` sorts descending by RAW
    // priority number without per-RTOS inversion, so on ThreadX (lower =
    // higher) tiers[0] is the numerically-largest = LOWEST-priority tier.
    // Its declares still run FIRST, before any spawn (#144), so no
    // concurrent declare can race the handshake.
    let boot_tier = &tiers[0];
    // W5 — adopt the boot tier's ThreadX priority: the app thread was
    // created at the fixed `nros_board_app_priority()`; keeping that would
    // let the boot tier outrank every spawned tier regardless of the
    // declared numbers (the e2e's app@4 starved the spawned high@5 to zero
    // publishes). Non-fatal on error: the tiers still run, mis-ranked.
    let boot_prio = boot_tier.priority.clamp(0, u32::MAX as i64) as core::ffi::c_uint;
    let boot_pt: core::ffi::c_int = match boot_tier.preempt_threshold {
        Some(v) => v.clamp(0, u32::MAX as i64) as core::ffi::c_int,
        None => -1,
    };
    if unsafe { nros_threadx_set_current_priority(boot_prio, boot_pt) } != 0 {
        B::println(format_args!(
            "nros: boot tier `{}` priority change failed; tiers may be mis-ranked",
            boot_tier.name
        ));
    } else if boot_tier.preempt_threshold.is_some() {
        // phase-296 W5.10 — the non_preempt_scope dim, trace-confirmed: the
        // kernel accepted the preemption threshold (printed ONLY on success,
        // the W5.5 marker discipline). Literal mirrors
        // `nros_tests::output::THREADX_PREEMPT_MARKER`.
        B::println(format_args!(
            "nros: preempt threshold set tier=`{}` {}",
            boot_tier.name, boot_pt
        ));
    }
    apply_tier(&mut crt, boot_tier);
    apply_tier_core_exclude::<B>(boot_tier);
    // phase-296 #0266 — the boot tier self-applies its round-robin time slice
    // (the boot thread was created with TX_NO_TIME_SLICE; change it in place).
    if let Some(ts) = boot_tier.time_slice_us {
        if unsafe { nros_threadx_apply_current_time_slice(ts as core::ffi::c_ulong) } == 1 {
            // Literal mirrors `nros_tests::output::THREADX_TIME_SLICE_MARKER`.
            B::println(format_args!(
                "nros: time slice set tier=`{}` {}us",
                boot_tier.name, ts
            ));
        }
    }
    {
        let mut runtime = RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = setup(&mut runtime) {
            B::println(format_args!("Application error (boot tier): {:?}", e));
            B::exit_failure();
        }
    }

    // Chain-spawn the remaining tiers off this executor's session.
    if spawn_next_tier::<B, C, F, E>(crt.executor_mut().session_handle(), &tiers[1..], setup)
        .is_err()
    {
        B::exit_failure();
    }

    B::println(format_args!(
        "Multi-tier setup complete — entering boot-tier spin loop."
    ));
    let period_ms = (boot_tier.spin_period_us / 1000).max(1) as u32;
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired, so under
    // sustained traffic this loop would otherwise never block, and a thread
    // that never blocks never lets a lower-priority tier run. Costs nothing
    // while the spins do block.
    let mut gap = ::nros_platform::TierSpinGap::new(boot_tier.spin_period_us);
    loop {
        let iter = gap.mark();
        if let Err(err) = ::nros_platform::NodeDispatchRuntime::spin_once(&mut crt, period_ms) {
            B::println(format_args!("spin_once error: {:?}", err));
            B::exit_failure();
        }
        gap.after_spin(iter);
    }
}

/// Phase 297 W4 — multi-tier entry for the ThreadX family. The `nros::main!`
/// macro emits `<Board>::run_tiers(&overlay, TIERS, setup)` whenever a system
/// declares more than the synthesized single `default` tier; the per-board ZST
/// routes here. Mirrors [`run_entry`] (build the boot context into
/// `CTX_STORAGE`, push the network config + app callback through the C glue,
/// `tx_kernel_enter()`) but runs one `Executor` per tier over one shared
/// session (see [`app_task_entry_tiers`]).
///
/// `tiers` is the macro-baked `&'static [TierSpec]`; `setup` is the
/// register-only closure (invoked once per tier — `Fn + Copy`).
pub fn run_tiers_entry<B, C, F, E>(
    config: C,
    boot_config: Option<&'static BakedBootConfig>,
    tiers: &'static [TierSpec<'static>],
    setup: F,
) -> core::result::Result<(), E>
where
    B: BoardInit + BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    //
    // The board also installs a `log`-crate logger below, and that is a
    // DIFFERENT facade: an `log_error!` raised inside a LIBRARY (the zenoh
    // session-pool diagnostic of issue 0589, for one) dispatches through
    // nros_log, which drops every record until a sink list is published.
    // Measured before the fix: the threadx-linux logging fixture with its own
    // `init` removed booted fully and emitted 0 of 6 records.
    //
    // At the FUNNEL, not next to `install_uart_logger`/`install_stdout_logger`:
    // those live in inner helpers, and a first attempt that patched them left
    // `run_bare` — the funnel the fixture actually boots through — still
    // silent. Idempotent, so nesting funnels may each call it.
    ::nros_platform_cffi::log::init_default();
    B::println(format_args!(""));
    B::println(format_args!("========================================"));
    B::println(format_args!("  nros ThreadX Platform (multi-tier)"));
    B::println(format_args!("========================================"));
    B::init_hardware();

    // Boot context into the shared static storage (single-threaded here —
    // pre-kernel-enter). Reuses `CTX_STORAGE` (run_entry vs run_tiers are
    // mutually exclusive per image).
    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<TiersContext<C, F>>();
        let align = core::mem::align_of::<TiersContext<C, F>>();
        assert!(
            size <= CTX_STORAGE_SIZE,
            "TiersContext too large for CTX_STORAGE — bump CTX_STORAGE_SIZE"
        );
        let storage_ptr = core::ptr::addr_of_mut!(CTX_STORAGE) as *mut u8;
        let addr = storage_ptr as usize;
        let aligned = (addr + align - 1) & !(align - 1);
        let offset = aligned - addr;
        assert!(
            offset + size <= CTX_STORAGE_SIZE,
            "TiersContext alignment + size exceeds CTX_STORAGE"
        );
        let ptr = storage_ptr.add(offset) as *mut TiersContext<C, F>;
        core::ptr::write(
            ptr,
            TiersContext {
                config,
                boot_config,
                tiers,
                setup,
            },
        );
        ptr
    };

    let iface_ptr: *const u8 = unsafe {
        let cfg = &(*ctx_ptr).config;
        match cfg.interface() {
            Some(iface) => {
                let buf_ptr = core::ptr::addr_of_mut!(IFACE_BUF) as *mut u8;
                let bytes = iface.as_bytes();
                let n = bytes.len().min(IFACE_BUF_SIZE - 1);
                core::ptr::copy_nonoverlapping(bytes.as_ptr(), buf_ptr, n);
                *buf_ptr.add(n) = 0;
                buf_ptr as *const u8
            }
            None => core::ptr::null(),
        }
    };

    unsafe {
        let cfg = &(*ctx_ptr).config;
        nros_threadx_set_config(
            cfg.ip().as_ptr(),
            cfg.netmask().as_ptr(),
            cfg.gateway().as_ptr(),
            cfg.mac().as_ptr(),
            iface_ptr,
        );
        // The multi-tier app callback runs `app_task_entry_tiers` (which
        // diverges); the `void(*)(void*)` app-callback shape is preserved.
        nros_threadx_set_app_callback(
            app_task_entry_tiers_trampoline::<B, C, F, E>,
            ctx_ptr as *mut c_void,
        );
        tx_kernel_enter();
    }

    B::exit_failure()
}

/// `nros_threadx_set_app_callback` wants `extern "C" fn(*mut c_void)` (a
/// non-diverging signature); [`app_task_entry_tiers`] diverges (`-> !`). This
/// thin trampoline bridges the two.
///
/// # Safety
/// Same contract as [`app_task_entry_tiers`].
unsafe extern "C" fn app_task_entry_tiers_trampoline<B, C, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: Fn(&mut RuntimeCtx<'_>) -> core::result::Result<(), E> + Copy,
    E: core::fmt::Debug,
{
    unsafe { app_task_entry_tiers::<B, C, F, E>(arg) }
}

/// Wrapper passed through the ThreadX thread `void *` arg.
struct AppContext<C, F> {
    config: C,
    /// Issue #98 / RFC-0045 — baked `.nros_boot_config` for node-name resolution.
    boot_config: Option<&'static BakedBootConfig>,
    closure: F,
}

/// Static storage for the 212.N.2 path's `AppContext`. Distinct from
/// the legacy `node::run`'s `CTX_STORAGE` so both entry points can
/// coexist during the 212.N migration; per-board overlays only ever
/// link one path at a time, but keeping the statics separate avoids a
/// type / generic-parameter clash if a future overlay accidentally
/// pulls both. Sized for typical closure captures (Executor handle +
/// a handful of node handles); asserted at runtime in `run_entry()`
/// so overflow is caught loudly instead of corrupting adjacent
/// memory.
// Phase 214.H.1 — both constants live in the crate-level `sizes` module
// (`lib.rs`); see there for the rationale + bump procedure.
use crate::sizes::{CTX_STORAGE_SIZE, IFACE_BUF_SIZE};

static mut CTX_STORAGE: [u8; CTX_STORAGE_SIZE] = [0u8; CTX_STORAGE_SIZE];

/// Static interface-name buffer for the C FFI's `interface_name`
/// argument. Linux overlay copies its `Config::interface` here +
/// appends NUL; bare-metal overlays leave it empty and pass `NULL`.
static mut IFACE_BUF: [u8; IFACE_BUF_SIZE] = [0u8; IFACE_BUF_SIZE];

/// ThreadX task entry for the application closure (212.N flavour —
/// hands the closure a `&mut RuntimeCtx<'_>` instead of `&Config`).
///
/// # Safety
/// `arg` must point to a valid `AppContext<C, F>` written into
/// `CTX_STORAGE` by `run_entry()`, surviving until the ThreadX
/// kernel terminates.
unsafe extern "C" fn app_task_entry_runtime<B, C, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: FnOnce(&mut RuntimeCtx<'_>) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &*(arg as *const AppContext<C, F>) };

    // FnOnce / by-value config — `core::ptr::read` because this task
    // entry runs once and `run_app_thread` consumes both.
    let closure = unsafe { core::ptr::read(&ctx.closure) };
    let config = unsafe { core::ptr::read(&ctx.config) };
    let boot_config = ctx.boot_config;

    run_app_thread::<B, C, F, E>(config, boot_config, closure)
}

/// Phase 245 — the post-kernel app-thread body, factored out of
/// [`app_task_entry_runtime`] so the **bare-metal CycloneDDS path** can reuse it.
///
/// The cargo/zenoh path enters via [`run_entry`] (which calls `tx_kernel_enter`
/// and registers [`app_task_entry_runtime`] as the app callback). The
/// CMake/CycloneDDS firmware instead has a **C** `startup.c::main` that calls
/// `tx_kernel_enter` itself and dispatches to a Rust `app_main` — so by the time
/// `app_main` runs, **the kernel is already entered**. `app_main` must therefore
/// NOT call [`run_entry`] (double kernel-enter); it calls this directly.
///
/// Body: network-stabilisation sleep, open the executor (locator/domain from the
/// board `ThreadxConfig` — CycloneDDS ignores the locator, no router), wrap it in
/// an `ExecutorNodeRuntime`, run the user `setup` (the `nros::node!()`-emitted
/// `register`), then spin for the firmware lifetime. Diverges (`-> !`): the
/// ThreadX scheduler never lets the app thread exit.
///
/// `boot_config` — the baked `.nros_boot_config` static (issue #98 / RFC-0045).
/// Pass `None` for the bare-metal CycloneDDS path (no macro, no baked config).
pub fn run_app_thread<B, C, F, E>(
    config: C,
    boot_config: Option<&'static BakedBootConfig>,
    setup: F,
) -> !
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: FnOnce(&mut RuntimeCtx<'_>) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    //
    // The board also installs a `log`-crate logger below, and that is a
    // DIFFERENT facade: an `log_error!` raised inside a LIBRARY (the zenoh
    // session-pool diagnostic of issue 0589, for one) dispatches through
    // nros_log, which drops every record until a sink list is published.
    // Measured before the fix: the threadx-linux logging fixture with its own
    // `init` removed booted fully and emitted 0 of 6 records.
    //
    // At the FUNNEL, not next to `install_uart_logger`/`install_stdout_logger`:
    // those live in inner helpers, and a first attempt that patched them left
    // `run_bare` — the funnel the fixture actually boots through — still
    // silent. Idempotent, so nesting funnels may each call it.
    ::nros_platform_cffi::log::init_default();
    // Issue #214 — echo the effective identity/domain so a two-node QEMU
    // pair failure is diagnosable from the console (the `run_entry` path
    // prints an equivalent banner; this path had none).
    {
        let mac = config.mac();
        let ip = config.ip();
        B::println(format_args!(
            "[app] MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}  IP {}.{}.{}.{}  domain {}",
            mac[0],
            mac[1],
            mac[2],
            mac[3],
            mac[4],
            mac[5],
            ip[0],
            ip[1],
            ip[2],
            ip[3],
            config.domain_id()
        ));
    }

    // Issue 0484 — a `tx_thread_sleep(200)` "network stabilisation delay" used to
    // sit here. At TX_TIMER_TICKS_PER_SECOND = 100 that is exactly 2 s, and it
    // was inherited ("matching the legacy per-overlay wait in
    // `node::app_task_entry`") rather than derived from a measurement.
    //
    // It was costing every RUST ThreadX image 2 s that the C and C++ images
    // never paid, because their `main` calls the nros-c API directly and never
    // passes through this entry. Same board, same platform, same RMW — and the
    // C/C++ API is a thin wrapper over this same Rust API, so the asymmetry was
    // the bug, not just the duration.
    //
    // Measured on qemu-riscv64-threadx, CycloneDDS, listener-ready then delivery:
    //
    //     cell    before          after
    //     c       1.35 s          1.40 s
    //     cpp     1.45 s          1.35 s
    //     rust    5.31 s          1.40 s      <- was 2.11 s just to subscribe
    //
    // The condition it was approximating is already met before this code runs:
    // the board prints `[virtio] enable: link UP` at 0.07 s and the app thread
    // starts at 0.08 s. There is nothing left to stabilise for.
    //
    // If a board ever does need to wait here, wait for the LINK, not for a
    // duration — a fixed delay does not just cost its time, it hides the
    // distribution underneath it (phase-342 W8).

    // Issue #98 / RFC-0045 — node name from the baked `.nros_boot_config`
    // (a launch that names the node overrides the board default); locator +
    // domain_id come from the per-board `ThreadxConfig` (NOT env vars —
    // embedded libc `getenv` has no host trampoline).
    let baked = boot_config
        .map(::nros::BootConfig::from_baked)
        .unwrap_or_default();
    let exec_cfg = ::nros::ExecutorConfig::resolve(::nros::BootConfig {
        node_name: baked.node_name.or(Some("nros_app")),
        locator: Some(config.locator()),
        domain_id: Some(config.domain_id()),
        namespace: None,
        // Issue 1050 defect (3) — the baked rung reaches the resolver. This
        // board has no environment to read, so before it existed an image here
        // could not name its backend at all.
        rmw: baked.rmw,
    });

    // #131 — register the linked zenoh CFFI backend before `Executor::open`.
    // ThreadX is `target_os = "none"`: the `nros_rmw_register_backend!`
    // `.init_array` ctor is a no-op and the flat image runs no static ctors, so
    // without this explicit, idempotent call `resolve_backend` finds NoBackend
    // and `Executor::open` returns `Transport(ConnectionFailed)` before any wire
    // I/O (empty pcap — the observed threadx-riscv64 rust-lane failure). Mirrors
    // the nuttx / freertos / mps2 boot paths (the sanctioned embedded path per
    // nros/src/lib.rs: "embedded boards perform the explicit <backend>::register()
    // in their boot path"). Gated by the overlay-forwarded `rmw-zenoh` feature →
    // cyclonedds builds omit it.
    #[cfg(feature = "rmw-zenoh")]
    if let Err(err) = ::nros_rmw_zenoh::register() {
        B::println(format_args!(
            "nros: zenoh RMW backend register failed: {:?}",
            err
        ));
    }

    let executor = match ::nros::Executor::open(&exec_cfg) {
        Ok(e) => e,
        Err(err) => {
            B::println(format_args!(""));
            B::println(format_args!("Executor::open failed: {:?}", err));
            B::exit_failure();
        }
    };
    // #131 — install the UART `log` sink so the examples' `log::info!` lines
    // (`Publishing: '...'` / `I heard: [...]`) reach the console, then emit the
    // cross-RTOS readiness marker the e2e harness gates on (mirrors nuttx's
    // `run_entry`). Both come AFTER `Executor::open` so the marker means "session
    // up". `install_uart_logger` is idempotent (a second `set_logger` is a no-op).
    install_uart_logger::<B>();
    B::println(format_args!("nros entry ready"));

    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
    let mut runtime = RuntimeCtx::with_runtime(&mut crt);

    match setup(&mut runtime) {
        Ok(()) => {
            B::println(format_args!(""));
            B::println(format_args!(
                "Application setup complete — entering spin loop."
            ));
            // Embedded spin: the ThreadX scheduler never returns from
            // this thread, so we loop forever. `spin_once` errors trip
            // exit_failure (a working bring-up never gets here).
            loop {
                if let Err(err) = ::nros_platform::NodeDispatchRuntime::spin_once(&mut crt, 10) {
                    B::println(format_args!(""));
                    B::println(format_args!("spin_once error: {:?}", err));
                    B::exit_failure();
                }
            }
        }
        Err(e) => {
            B::println(format_args!(""));
            B::println(format_args!("Application error: {:?}", e));
            B::exit_failure();
        }
    }
}

/// Phase 212.N.2 — family-driver entry point for ThreadX boards.
///
/// Mirrors the legacy [`crate::run`] body — stashes the user closure
/// into static storage, registers the network config + app callback
/// through the unified ThreadX C glue, calls `tx_kernel_enter()`,
/// never returns — but routes through the 212.N.1
/// `nros_platform::board` trait set + [`RuntimeCtx`].
///
/// Per-board crates (e.g. `nros-board-threadx-linux`,
/// `nros-board-threadx-qemu-riscv64`) wire this into their
/// `impl BoardEntry for Self::run` body in 212.N.3:
///
/// ```ignore
/// impl nros_platform::board::BoardEntry for MyBoard {
///     fn run<F, E>(setup: F) -> Result<(), E>
///     where
///         F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
///         E: core::fmt::Debug,
///     {
///         let cfg = Config::default();
///         nros_board_threadx::run_entry::<MyBoard, Config, F, E>(cfg, None, setup)
///     }
/// }
/// ```
///
/// # Type parameters
///
/// - `B: BoardInit + BoardPrint + BoardExit` — per-board glue pulled
///   from `nros_platform::board` (212.N.1 surface).
/// - `C: ThreadxConfig` — board's config type, exposing
///   `mac/ip/netmask/gateway/interface()` accessors. Stays as a
///   separate generic (rather than folded onto `B::Config`) so the
///   per-board overlay can keep its existing concrete `Config`
///   struct unchanged during the 212.N migration.
/// - `F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>` — user closure
///   receiving the runtime context.
/// - `E: core::fmt::Debug` — closure error type.
///
/// # Return
///
/// The signature is `Result<(), E>` to satisfy the
/// [`nros_platform::BoardEntry::run`] trait contract, but in practice
/// the kernel-spawn flow never returns to the caller — either
/// `tx_kernel_enter()` runs forever and the app thread drives
/// `exit_success` / `exit_failure`, or kernel entry itself returns
/// (e.g. on the Linux ThreadX-sim port after a clean shutdown) and we
/// `exit_failure` defensively. The `Ok(())` arm exists only so the
/// function signature lines up with the trait; it is unreachable in
/// a working build.
///
/// `boot_config` — the baked `.nros_boot_config` static, passed from
/// the per-board `run_with_deploy` (issue #98 / RFC-0045). `None`
/// keeps the historical `"nros_app"` node-name default.
pub fn run_entry<B, C, F, E>(
    config: C,
    boot_config: Option<&'static BakedBootConfig>,
    setup: F,
) -> core::result::Result<(), E>
where
    B: BoardInit + BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: FnOnce(&mut RuntimeCtx<'_>) -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    //
    // The board also installs a `log`-crate logger below, and that is a
    // DIFFERENT facade: an `log_error!` raised inside a LIBRARY (the zenoh
    // session-pool diagnostic of issue 0589, for one) dispatches through
    // nros_log, which drops every record until a sink list is published.
    // Measured before the fix: the threadx-linux logging fixture with its own
    // `init` removed booted fully and emitted 0 of 6 records.
    //
    // At the FUNNEL, not next to `install_uart_logger`/`install_stdout_logger`:
    // those live in inner helpers, and a first attempt that patched them left
    // `run_bare` — the funnel the fixture actually boots through — still
    // silent. Idempotent, so nesting funnels may each call it.
    ::nros_platform_cffi::log::init_default();
    B::println(format_args!(""));
    B::println(format_args!("========================================"));
    B::println(format_args!("  nros ThreadX Platform"));
    B::println(format_args!("========================================"));
    B::println(format_args!(""));

    let mac = config.mac();
    let ip = config.ip();
    B::println(format_args!("Initializing ThreadX + NetX..."));
    B::println(format_args!(
        "  MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    ));
    B::println(format_args!(
        "  IP:  {}.{}.{}.{}",
        ip[0], ip[1], ip[2], ip[3]
    ));
    if let Some(iface) = config.interface() {
        B::println(format_args!("  IF:  {}", iface));
    }

    // Per-board pre-kernel init. New 212.N.1 `BoardInit::init_hardware`
    // is parameterless — board crates read any needed config off
    // their own `pub const` / `pub static` rather than a passed-in
    // arg.
    B::init_hardware();

    // Static-storage placement of AppContext. Closure size is
    // bounded by CTX_STORAGE_SIZE (8 KB) — asserted so overflow is
    // caught loudly instead of corrupting adjacent memory.
    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<AppContext<C, F>>();
        let align = core::mem::align_of::<AppContext<C, F>>();
        assert!(
            size <= CTX_STORAGE_SIZE,
            "AppContext too large for CTX_STORAGE — bump CTX_STORAGE_SIZE"
        );
        let storage_ptr = core::ptr::addr_of_mut!(CTX_STORAGE) as *mut u8;
        let addr = storage_ptr as usize;
        let aligned = (addr + align - 1) & !(align - 1);
        let offset = aligned - addr;
        assert!(
            offset + size <= CTX_STORAGE_SIZE,
            "AppContext alignment + size exceeds CTX_STORAGE"
        );
        let ptr = storage_ptr.add(offset) as *mut AppContext<C, F>;
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

    // Materialise interface name into the static buffer (with NUL
    // terminator) or pass NULL for bare-metal overlays.
    let iface_ptr: *const u8 = unsafe {
        let cfg = &(*ctx_ptr).config;
        match cfg.interface() {
            Some(iface) => {
                let buf_ptr = core::ptr::addr_of_mut!(IFACE_BUF) as *mut u8;
                let bytes = iface.as_bytes();
                let n = bytes.len().min(IFACE_BUF_SIZE - 1);
                core::ptr::copy_nonoverlapping(bytes.as_ptr(), buf_ptr, n);
                *buf_ptr.add(n) = 0;
                buf_ptr as *const u8
            }
            None => core::ptr::null(),
        }
    };

    unsafe {
        let cfg = &(*ctx_ptr).config;
        nros_threadx_set_config(
            cfg.ip().as_ptr(),
            cfg.netmask().as_ptr(),
            cfg.gateway().as_ptr(),
            cfg.mac().as_ptr(),
            iface_ptr,
        );
        nros_threadx_set_app_callback(app_task_entry_runtime::<B, C, F, E>, ctx_ptr as *mut c_void);

        // Enter the ThreadX kernel — does not return on a working
        // bring-up.
        tx_kernel_enter();
    }

    // Unreachable — kernel enter diverges on production paths.
    // `exit_failure()` is `-> !`, so this satisfies the
    // `Result<(), E>` signature without an explicit `Ok` arm.
    B::exit_failure()
}

/// Phase 313 W0 (#0243) — LIGHTWEIGHT, NO-SESSION variant of [`run_entry`].
///
/// Boots the ThreadX kernel + NetX config (same pre-kernel path as `run_entry`),
/// then runs `setup` on the app thread WITHOUT opening an `Executor` session. For
/// logging / init-only fixtures (`logging-smoke-*`, `*-board-bringup`) that must
/// exercise the board boot + platform log writer but do NOT open a ROS session —
/// where `run_entry`'s `Executor::open` would fail `Transport(ConnectionFailed)`
/// with no router and abort before `setup` ever runs.
///
/// The closure takes NO `RuntimeCtx` (there is no session to back it); it returns
/// `Ok`→`exit_success`, `Err`→`exit_failure`. This is the new-family replacement
/// for the no-session role the legacy `run<B>(config, closure)` also served.
pub fn run_bare<B, C, F, E>(config: C, setup: F) -> core::result::Result<(), E>
where
    B: BoardInit + BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: FnOnce() -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel.
    //
    // The board also installs a `log`-crate logger below, and that is a
    // DIFFERENT facade: an `log_error!` raised inside a LIBRARY (the zenoh
    // session-pool diagnostic of issue 0589, for one) dispatches through
    // nros_log, which drops every record until a sink list is published.
    // Measured before the fix: the threadx-linux logging fixture with its own
    // `init` removed booted fully and emitted 0 of 6 records.
    //
    // At the FUNNEL, not next to `install_uart_logger`/`install_stdout_logger`:
    // those live in inner helpers, and a first attempt that patched them left
    // `run_bare` — the funnel the fixture actually boots through — still
    // silent. Idempotent, so nesting funnels may each call it.
    ::nros_platform_cffi::log::init_default();
    B::println(format_args!(""));
    B::println(format_args!("========================================"));
    B::println(format_args!("  nros ThreadX Platform (bare)"));
    B::println(format_args!("========================================"));
    B::println(format_args!(""));

    B::init_hardware();

    let ctx_ptr = unsafe {
        let size = core::mem::size_of::<AppContext<C, F>>();
        let align = core::mem::align_of::<AppContext<C, F>>();
        assert!(
            size <= CTX_STORAGE_SIZE,
            "AppContext too large for CTX_STORAGE — bump CTX_STORAGE_SIZE"
        );
        let storage_ptr = core::ptr::addr_of_mut!(CTX_STORAGE) as *mut u8;
        let addr = storage_ptr as usize;
        let aligned = (addr + align - 1) & !(align - 1);
        let offset = aligned - addr;
        assert!(
            offset + size <= CTX_STORAGE_SIZE,
            "AppContext alignment + size exceeds CTX_STORAGE"
        );
        let ptr = storage_ptr.add(offset) as *mut AppContext<C, F>;
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

    let iface_ptr: *const u8 = unsafe {
        let cfg = &(*ctx_ptr).config;
        match cfg.interface() {
            Some(iface) => {
                let buf_ptr = core::ptr::addr_of_mut!(IFACE_BUF) as *mut u8;
                let bytes = iface.as_bytes();
                let n = bytes.len().min(IFACE_BUF_SIZE - 1);
                core::ptr::copy_nonoverlapping(bytes.as_ptr(), buf_ptr, n);
                *buf_ptr.add(n) = 0;
                buf_ptr as *const u8
            }
            None => core::ptr::null(),
        }
    };

    unsafe {
        let cfg = &(*ctx_ptr).config;
        nros_threadx_set_config(
            cfg.ip().as_ptr(),
            cfg.netmask().as_ptr(),
            cfg.gateway().as_ptr(),
            cfg.mac().as_ptr(),
            iface_ptr,
        );
        nros_threadx_set_app_callback(app_task_entry_bare::<B, C, F, E>, ctx_ptr as *mut c_void);
        tx_kernel_enter();
    }

    B::exit_failure()
}

/// ThreadX task entry for the NO-SESSION [`run_bare`] path — runs the closure
/// directly (no `Executor`, no `RuntimeCtx`), then exits.
///
/// # Safety
/// `arg` must point to a valid `AppContext<C, F>` written into `CTX_STORAGE` by
/// [`run_bare`], surviving until the ThreadX kernel terminates.
unsafe extern "C" fn app_task_entry_bare<B, C, F, E>(arg: *mut c_void)
where
    B: BoardPrint + BoardExit,
    C: ThreadxConfig,
    F: FnOnce() -> core::result::Result<(), E>,
    E: core::fmt::Debug,
{
    let ctx = unsafe { &*(arg as *const AppContext<C, F>) };
    // FnOnce / by-value — this task entry runs once.
    let closure = unsafe { core::ptr::read(&ctx.closure) };
    // Mirror the legacy `node::app_task_entry` completion banner so fixtures
    // that grep for "Application completed successfully." keep passing.
    match closure() {
        Ok(()) => {
            B::println(format_args!(""));
            B::println(format_args!("Application completed successfully."));
            B::println(format_args!(""));
            B::println(format_args!("========================================"));
            B::println(format_args!("  Done"));
            B::println(format_args!("========================================"));
            B::exit_success();
        }
        Err(e) => {
            B::println(format_args!(""));
            B::println(format_args!("Application error: {:?}", e));
            B::exit_failure();
        }
    }
}
