//! issue #128 (half 2) / RFC-0015 Model 1 — per-tier multi-task entry for
//! Zephyr: one `k_thread` per priority tier over ONE shared zenoh session.
//!
//! Mirrors `nros_board_freertos::run_tiers_entry` minus the network +
//! scheduler bring-up (Zephyr owns boot; `rust_main` — the caller — is
//! already a running post-init thread): the caller thread opens the boot
//! `Executor`, runs the boot tier's setup, then CHAIN-spawns the remaining
//! tiers (issue #144) through the module's `nros_zephyr_tier_task_create`
//! shim (`k_thread_create` on a static pool, RAW Zephyr priority — negatives
//! = cooperative, exactly the `[tiers.<name>.zephyr].priority` value), and
//! then runs `tiers[0]` itself. NOTE (issue 0251): `resolve_tiers` sorts
//! descending by RAW number without inverting per RTOS direction, and Zephyr
//! is lower-number-wins — so `tiers[0]`, the boot tier, is the
//! numerically-largest = LOWEST-priority tier. Each spawned tier task opens
//! an `Executor` over the shared session (`SessionHandle`), installs its
//! `active_groups` filter, runs the SAME register-only setup closure (the
//! groups gate what actually registers), spawns the NEXT tier once its own
//! setup returns, and spins forever at the tier's declared period. Chaining
//! spawns behind each setup keeps setup order total so no two entity-declare
//! bursts race the shared session's interest handshake.
//!
//! The `nros::main!` `Framework::Zephyr` arm emits
//! `ZephyrBoard::run_tiers(&config, TIERS, closure)` for multi-tier systems
//! (single-tier keeps the plain register+spin scaffold).

extern crate alloc;

use alloc::boxed::Box;
use core::ffi::c_void;

use nros_platform::{NodeDispatchRuntime, RuntimeCtx, RuntimeError, TierSpec};

use crate::ZephyrBoard;

unsafe extern "C" {
    /// `zephyr/nros_platform_zephyr_shims.c` — `k_thread_create` on a static
    /// tier pool at the RAW Zephyr priority. Returns 0 on success, -1 when
    /// the pool (`NROS_ZEPHYR_MAX_TIERS`) is exhausted.
    /// issue 0655 — `core_plus1` (0 = unpinned) makes the shim create the
    /// thread SUSPENDED and pin it before starting, the only window in which
    /// Zephyr accepts a cpu mask. The kernel's own return code comes back in
    /// `pin_rc` so the marker text stays here, in lockstep with
    /// `nros_tests::output::ZEPHYR_CORE_PIN_*`.
    fn nros_zephyr_tier_task_create(
        entry: unsafe extern "C" fn(*mut c_void) -> *mut c_void,
        arg: *mut c_void,
        priority: i32,
        name: *const core::ffi::c_char,
        stack_bytes: usize,
        core_plus1: u32,
        pin_rc: *mut i32,
    ) -> i32;
    /// Adopt a raw Zephyr priority on the CALLING thread — the boot thread
    /// runs `tiers[0]` itself, so it must take that tier's declared priority
    /// (`k_thread_priority_set(k_current_get(), …)`).
    fn nros_zephyr_set_current_priority(priority: i32);
    /// phase-296 W5.5 — apply an earliest-deadline (µs) on the CALLING
    /// thread via `k_thread_deadline_set`. Returns 1 when the kernel
    /// actually applied it (EDF present), 0 when it was a no-op (image
    /// lacks `CONFIG_SCHED_DEADLINE`).
    fn nros_zephyr_set_current_deadline(deadline_us: u32) -> i32;
    /// Phase 110.D shim — pin the CALLING thread to a CPU
    /// (`k_thread_cpu_pin`). Returns 0 on success, `-ENOSYS` when the image
    /// lacks `CONFIG_SCHED_CPU_MASK_PIN_ONLY`, else the kernel error.
    fn nros_zephyr_thread_cpu_pin(cpu: i32) -> i32;
    /// issue 0260 — the CPU the CALLING thread is observed on, or
    /// `NROS_ZEPHYR_CPU_UNKNOWN` when the image cannot say (no `CONFIG_SMP`;
    /// the posix arch does not provide `arch_proc_id` at all).
    fn nros_zephyr_current_cpu() -> u32;
}

/// Mirrors `NROS_ZEPHYR_CPU_UNKNOWN` in `nros_platform_zephyr_shims.c`.
const CPU_UNKNOWN: u32 = u32::MAX;

/// Apply this tier's kernel EDF deadline on the CALLING thread, when the
/// tier is real-time and carries a deadline. Gated by the `zephyr-edf`
/// feature; off ⇒ the executor's cooperative `SchedContext` deadline
/// monitor is the sole enforcement (an honest Backfill). The marker is
/// logged ONLY when the shim reports the deadline was actually applied
/// (kernel has `CONFIG_SCHED_DEADLINE`) — else a kernel-less image could
/// log the marker while nothing was applied. The `::log::info!` literal
/// MUST match `nros_tests::output::ZEPHYR_EDF_DEADLINE_MARKER`.
#[cfg(feature = "zephyr-edf")]
fn apply_tier_deadline(tier: &TierSpec<'_>) {
    if tier.class == Some("real_time") {
        if let Some(us) = tier.deadline_us {
            let us = us.min(u32::MAX as u64) as u32;
            let applied = unsafe { nros_zephyr_set_current_deadline(us) };
            if applied != 0 {
                ::log::info!("nros: EDF deadline set tier=`{}` {}us", tier.name, us);
            }
        }
    }
}

#[cfg(not(feature = "zephyr-edf"))]
#[inline]
fn apply_tier_deadline(_tier: &TierSpec<'_>) {}

/// The BOOT tier's placement attempt (issue 0655).
///
/// phase-296 W5.5 added this as a general "pin the calling thread" consumer,
/// used by both the boot tier and every spawned tier. It could never work:
/// Zephyr's `cpu_mask_mod` accepts a mask only on a thread that is prevented
/// from running, and `k_current_get()` never is. Spawned tiers now pin in the
/// create→start window inside `nros_zephyr_tier_task_create`; this path remains
/// only for the boot tier, which the kernel started before `run_tiers` ever saw
/// it and which therefore has no such window.
///
/// So a `core` declared on the boot tier CANNOT be honored on Zephyr. That is a
/// real limitation of the API, not a missing feature here, and it is reported
/// as one — RFC-0052's contract is that a dropped declaration is loud, and a
/// limitation stated plainly is more useful than a retry that cannot succeed.
fn apply_boot_tier_core_pin(tier: &TierSpec<'_>) {
    if let Some(cpu) = tier.core {
        let rc = unsafe { nros_zephyr_thread_cpu_pin(cpu.min(i32::MAX as u32) as i32) };
        if rc == 0 {
            // Not reachable on any Zephyr this pins against, but the accept arm
            // stays: if a future kernel permits a running-thread mask, this is
            // the honest marker for it rather than a stale failure note.
            ::log::info!("nros: core pin tier=`{}` cpu={}", tier.name, cpu);
        } else {
            ::log::warn!(
                "nros: core pin FAILED tier=`{}` cpu={} rc={} — this is the BOOT tier, which \
                 Zephyr started before nros ran, and a cpu mask is only settable before a \
                 thread starts (issue 0655). Declare the `core` on a SPAWNED tier, which is \
                 pinned in its create->start window. -88/ENOSYS instead means the image lacks \
                 CONFIG_SCHED_CPU_MASK. Tier runs unpinned",
                tier.name,
                cpu,
                rc
            );
        }
    }
}

/// Leaked per-tier context, consumed by [`tier_task_entry`].
struct TierTaskCtx<F> {
    session: ::nros::SessionHandle,
    tier: TierSpec<'static>,
    /// Tiers still to spawn AFTER this one — the chained-spawn tail
    /// (issue #144). This tier spawns `rest[0]` (carrying `rest[1..]`)
    /// only after its OWN setup returns, so no two setups overlap.
    rest: &'static [TierSpec<'static>],
    setup: F,
}

/// issue #144 — chained tier spawn. `remaining.split_first()` is the next
/// tier to bring up and the tail it must carry; empty → nothing left, `Ok`.
/// Spawns exactly ONE `k_thread` for `remaining[0]`, handing it `remaining[1..]`
/// as its own `rest` so the spawn chain continues once its setup completes.
/// Serializing spawns behind each setup guarantees no two `setup()` (entity
/// declare) calls ever run concurrently on the shared zenoh-pico session — the
/// interest-handshake race that silently closes a losing publisher's write
/// filter under `Z_FEATURE_MULTI_THREAD=0`.
fn spawn_next_tier<F>(
    session: ::nros::SessionHandle,
    remaining: &'static [TierSpec<'static>],
    setup: F,
) -> Result<(), RuntimeError>
where
    F: Fn(&mut RuntimeCtx<'_>) -> Result<(), RuntimeError> + Copy + 'static,
{
    let Some((tier, rest)) = remaining.split_first() else {
        return Ok(());
    };
    let ctx = Box::new(TierTaskCtx::<F> {
        session,
        tier: *tier,
        rest,
        setup,
    });
    let prio = tier.priority.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
    // Hold the raw pointer so a failed create can reclaim it — the task never
    // runs, so `tier_task_entry`'s `Box::from_raw` never fires; without this
    // the `TierTaskCtx` heap block leaks for the firmware lifetime.
    let raw = Box::into_raw(ctx);
    // issue 0655 — the pin happens INSIDE the shim, between `k_thread_create`
    // and `k_thread_start`. `core_plus1` is 0 for an unpinned tier (which
    // keeps the old K_NO_WAIT path verbatim); `pin_rc` receives the kernel's
    // return code so the marker below can tell accept from fallback.
    let core_plus1 = tier.core.map(|c| c.saturating_add(1)).unwrap_or(0);
    let mut pin_rc: i32 = 0;
    let rc = unsafe {
        nros_zephyr_tier_task_create(
            tier_task_entry::<F>,
            raw as *mut c_void,
            prio,
            c"nros_tier".as_ptr(),
            // phase-302 W2 (issue 0262) — the declared stack rides to the
            // shim, which prints LOUD when it exceeds the fixed pool slot.
            tier.stack_bytes,
            core_plus1,
            &mut pin_rc,
        )
    };
    if rc != 0 {
        // SAFETY: the create failed, so ownership of `raw` was not transferred
        // to a task; reclaim + drop it here.
        drop(unsafe { Box::from_raw(raw) });
        ::log::error!(
            "nros: failed to spawn tier `{}` (pool exhausted? NROS_ZEPHYR_MAX_TIERS)",
            tier.name
        );
        return Err(RuntimeError::Spin);
    }
    // issue 0655 — report the placement dim for this SPAWNED tier. The thread
    // is already running by now, but the pin was applied before it started, so
    // this reports a real kernel verdict rather than the -EINVAL a self-pin
    // could only ever produce.
    if let Some(cpu) = tier.core {
        report_core_pin(tier.name, cpu, pin_rc);
    }
    Ok(())
}

/// The placement dim's accept/fallback marker, shared by the spawned-tier and
/// boot-tier paths (issue 0655 — one spelling, so the two cannot drift).
///
/// The literal prefixes MIRROR `nros_tests::output::ZEPHYR_CORE_PIN_MARKER` and
/// `ZEPHYR_CORE_PIN_FALLBACK_MARKER`, and the C arm's `zephyr_apply_core_pin`
/// printk — keep all three in lockstep.
fn report_core_pin(name: &str, cpu: u32, rc: i32) {
    if rc == 0 {
        ::log::info!("nros: core pin tier=`{}` cpu={}", name, cpu);
    } else {
        ::log::warn!(
            "nros: core pin FAILED tier=`{}` cpu={} rc={} (-22/EINVAL: the thread was already \
             RUNNING — Zephyr accepts a cpu mask only before start, issue 0655; -88/ENOSYS: \
             image lacks CONFIG_SCHED_CPU_MASK) — tier runs unpinned",
            name,
            cpu,
            rc
        );
    }
}

/// Spawned tier task: open an `Executor` over the shared session, install
/// this tier's `active_groups` filter, register (off-tier callbacks are
/// gated out), then spin forever at the tier's period. Never returns; on
/// setup/spin failure it logs and parks (the firmware equivalent of the
/// FreeRTOS `exit_failure`, which Zephyr application threads don't have).
unsafe extern "C" fn tier_task_entry<F>(arg: *mut c_void) -> *mut c_void
where
    F: Fn(&mut RuntimeCtx<'_>) -> Result<(), RuntimeError> + Copy + 'static,
{
    let ctx = unsafe { Box::from_raw(arg as *mut TierTaskCtx<F>) };
    // SAFETY: the boot thread owns the session for the firmware lifetime
    // (its spin loop never returns), so the handle stays valid.
    let executor = unsafe { ::nros::Executor::open_with_session_handle(ctx.session) };
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
    crt.executor_mut().set_active_groups(ctx.tier.groups);
    // W5.4 — lower this tier's class/budget/period/deadline onto the executor's
    // default SchedContext (Sporadic / EDF / TT), shared with every board.
    crt.apply_tier_sched_policy(
        ctx.tier.class,
        ctx.tier.period_us,
        ctx.tier.budget_us,
        ctx.tier.deadline_us,
        ctx.tier.deadline_policy,
    );
    apply_tier_deadline(&ctx.tier);
    // issue 0260 — report the CPU this tier is OBSERVED on, from the tier's own
    // thread, which is the only place that can answer.
    //
    // Distinct from the pin markers above, and deliberately so: those report
    // that the kernel ACCEPTED `k_thread_cpu_pin`, which on a uniprocessor
    // image is true and uninformative — a pin to cpu 0 is accepted and cpu 0 is
    // the only CPU there is. Until something reports where the tier ACTUALLY
    // ran, an SMP fixture would assert exactly what native_sim already asserts.
    //
    // Silent when the image cannot answer, rather than printing a fabricated
    // `cpu=0`.
    {
        let observed = unsafe { nros_zephyr_current_cpu() };
        if observed != CPU_UNKNOWN {
            ::log::info!(
                "nros: core pin observed tier=`{}` running_on={}",
                ctx.tier.name,
                observed
            );
        }
    }

    // issue 0655 — NO core pin here. This runs on the tier's own thread, which
    // is started, and Zephyr refuses a cpu mask on a started thread. The pin
    // was applied inside `nros_zephyr_tier_task_create`, before the start;
    // repeating it here would only ever log a spurious -EINVAL over a
    // successful pin.
    {
        let mut runtime = RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = (ctx.setup)(&mut runtime) {
            // The chain is serialized (issue #144): this tier spawns the next
            // only AFTER its own setup returns Ok, so a setup failure here HALTS
            // the chain — `ctx.rest` (this tier's downstream tiers) will not
            // start. That is intentional (a tier whose baked config can't
            // declare its entities means a degraded deploy anyway), but say so
            // loudly rather than leaving the downstream tiers silently absent.
            ::log::error!(
                "nros: tier `{}` setup failed: {:?} — {} downstream tier(s) will NOT start",
                ctx.tier.name,
                e,
                ctx.rest.len()
            );
            loop {
                crate::zephyr_msleep(1000);
            }
        }
    }
    // issue #144 — this tier's setup is done, so it is now safe to bring up the
    // next tier: spawn `rest[0]` (carrying `rest[1..]`). Mint a fresh handle off
    // this tier's executor (same as the boot path — `ctx.session` was consumed
    // opening the executor above). A failed DOWNSTREAM spawn must NOT stop this
    // tier spinning its own work, so log + continue.
    let next_session = crt.executor_mut().session_handle();
    if let Err(e) = spawn_next_tier(next_session, ctx.rest, ctx.setup) {
        ::log::error!(
            "nros: tier `{}` failed to spawn next tier: {:?}",
            ctx.tier.name,
            e
        );
    }
    let period_ms = ((ctx.tier.spin_period_us / 1000).max(1)) as u32;
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired, so under
    // sustained traffic this loop would otherwise never block, and a thread
    // that never blocks never lets a lower-priority tier run. Costs nothing
    // while the spins do block.
    let mut gap = ::nros_platform::TierSpinGap::new(ctx.tier.spin_period_us);
    loop {
        let iter = gap.mark();
        if let Err(err) = NodeDispatchRuntime::spin_once(&mut crt, period_ms) {
            ::log::error!("nros: tier `{}` spin error: {:?}", ctx.tier.name, err);
            loop {
                crate::zephyr_msleep(1000);
            }
        }
        gap.after_spin(iter);
    }
}

impl ZephyrBoard {
    /// issue #128 (half 2) — per-tier multi-task entry. The caller (the
    /// `nros::main!` Zephyr arm's `rust_main` context) has already gated on
    /// the network, registered the RMW backend, and built `config` from the
    /// west-baked locator; this opens the ONE session, spawns `tiers[1..]`
    /// as `k_thread`s at their raw Zephyr priorities, and runs `tiers[0]`
    /// (the numerically-largest = LOWEST Zephyr priority — issue 0251, the
    /// sort does not invert per RTOS direction) on the caller thread —
    /// never returns on success.
    pub fn run_tiers<F>(
        config: &::nros::ExecutorConfig,
        tiers: &'static [TierSpec<'static>],
        setup: F,
    ) -> Result<(), RuntimeError>
    where
        F: Fn(&mut RuntimeCtx<'_>) -> Result<(), RuntimeError> + Copy + 'static,
    {
        // Wire the default nros_log sink (platform console) at the boot
        // funnel, as nros-board-linux does in its `run`/`run_tiers` —
        // idempotent, and without it Node-pkg `log_info!` output is
        // silently dropped on Zephyr multi-tier entries.
        ::nros_platform_cffi::log::init_default();

        // issue 0758 W4 — acquire the wall-clock epoch before ANY executor
        // opens, mirroring the C arm (`zephyr_run_tiers.c`) call at the same
        // point. Until this runs `nros_platform_epoch_us()` answers 0, which
        // nano-ros defines as "no wall clock"; after it, stamps are absolute and
        // a validating peer accepts them.
        //
        // The shim decides whether there is anything to do — this arm CANNOT,
        // because `CONFIG_NROS_SNTP_EPOCH` is a Kconfig symbol and Kconfig knobs
        // reach the Zephyr C lane but not the cargo one (issue 0460). Calling
        // unconditionally into C is what keeps the two tier arms from drifting;
        // an image without the knob gets an empty function.
        unsafe {
            unsafe extern "C" {
                fn nros_zephyr_epoch_acquire_configured();
            }
            nros_zephyr_epoch_acquire_configured();
        }

        if tiers.is_empty() {
            ::log::error!("nros: run_tiers called with no tiers");
            return Err(RuntimeError::Spin);
        }

        let boot_exec = match ::nros::Executor::open(config) {
            Ok(e) => e,
            Err(err) => {
                ::log::error!("nros: zephyr tiers — executor open failed: {:?}", err);
                return Err(RuntimeError::Spin);
            }
        };
        let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(boot_exec);

        // Boot-tier setup FIRST, tier spawn after: entity declares carry an
        // interest handshake (the zenoh-pico write filter opens only when the
        // router's current-subscriber reply lands), and concurrent declares
        // from two threads race that handshake — the losing publisher's
        // filter stays closed and every put is silently dropped. issue #144 —
        // serializing boot's declares before ANY spawn removes the boot↔tier
        // race, and CHAINING the remaining spawns (boot spawns tiers[1] only;
        // each tier spawns the next after its own setup returns) removes the
        // tier↔tier race too: setup order is total (boot, t1, t2, …), no two
        // declares ever overlap. Spins still overlap the next tier's setup,
        // which is SAFE — a spin exchanges keepalives/data, not declares (only
        // declare-vs-declare races the interest handshake).
        let boot_tier = &tiers[0];
        crt.executor_mut().set_active_groups(boot_tier.groups);
        crt.apply_tier_sched_policy(
            boot_tier.class,
            boot_tier.period_us,
            boot_tier.budget_us,
            boot_tier.deadline_us,
            boot_tier.deadline_policy,
        );
        {
            let mut runtime = RuntimeCtx::with_runtime(&mut crt);
            setup(&mut runtime)?;
        }

        // Kick off the chain: spawn tiers[1] carrying tiers[2..] as its tail;
        // tiers[0] runs on this (boot) thread. A boot-side spawn failure is
        // fatal (takes the error/exit path) — unlike a downstream tier's.
        spawn_next_tier(crt.executor_mut().session_handle(), &tiers[1..], setup)?;

        // The boot thread runs tiers[0] itself — adopt its declared raw
        // priority (the spawned tiers already got theirs at k_thread_create;
        // without this, tiers[0] would inherit the main-thread default and
        // the declared tier QoS would not hold for it).
        unsafe {
            nros_zephyr_set_current_priority(
                boot_tier.priority.clamp(i32::MIN as i64, i32::MAX as i64) as i32,
            );
        }
        apply_tier_deadline(boot_tier);
        apply_boot_tier_core_pin(boot_tier);
        ::log::info!(
            "nros: zephyr multi-tier entry up ({} tiers, boot tier `{}`)",
            tiers.len(),
            boot_tier.name
        );
        let period_ms = ((boot_tier.spin_period_us / 1000).max(1)) as u32;
        // issue 0636 option 3 — see the per-tier loop above; the boot tier owns
        // the shared session, so it is the one whose spin was most likely to
        // free-run under load.
        let mut gap = ::nros_platform::TierSpinGap::new(boot_tier.spin_period_us);
        loop {
            let iter = gap.mark();
            if let Err(err) = NodeDispatchRuntime::spin_once(&mut crt, period_ms) {
                ::log::error!("nros: boot tier `{}` spin error: {:?}", boot_tier.name, err);
                return Err(RuntimeError::Spin);
            }
            gap.after_spin(iter);
        }
    }
}
