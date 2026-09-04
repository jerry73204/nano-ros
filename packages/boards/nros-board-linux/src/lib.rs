//! The Linux host board — Phase 212.N.2 family driver + phase-337 W8.a.
//!
//! **Naming (RFC-0064 R3).** The PLATFORM stays `posix`: `nros-platform-posix`
//! is a genuine portability seam written to POSIX, and the platform layer names
//! software-stack facts (RFC-0049's duty rule). The BOARD is `linux`, because
//! the board layer names what we actually claim to support and a tier-1 promise
//! means "`just ci` exercises it" — which only Linux does (every CI job is
//! `ubuntu-*`, and `nros-platform-posix/src/timer.c` calls `timer_create` with
//! no fallback). `macos` / `freebsd` can later join as their own boards on this
//! unchanged platform; that is the one-source-many-targets model applied to
//! hosted OSes, and its prerequisite is a `timer_create` fallback.
//!
//! phase-337 W8.a merged `nros-board-native` (a 226-line delegating shim) and
//! `nros-board-posix` (the family driver) into this one crate. `board_path_for`
//! already mapped BOTH deploy keys to the same ZST, so the shim was named by no
//! generated entry — ceremony, not a behaviour boundary.
//!
//! Implements the `Board` trait family from `nros_platform` (the
//! traits live in `packages/platform/nros-platform/src/board/` and are
//! re-exported at the `nros_platform` crate root) for the hosted
//! LINUX target. Not "POSIX (Linux, macOS, BSD)", which this said and
//! which the crate cannot honour: `apply_tier_affinity` below calls
//! `sched_setaffinity` with `cpu_set_t` / `CPU_SET`, none of which
//! libc defines for apple, and nothing here is `cfg(target_os)`-gated.
//! The PLATFORM beneath it (`nros-platform-posix`) is POSIX-clean —
//! see the Naming note above. This is the simplest of the
//! family driver crates:
//!
//! - `init_hardware` is a no-op — libstd's runtime already brought up
//!   the heap, stdio, signal handlers and threading by the time
//!   `fn main` reaches us.
//! - `println` writes to `STDOUT_FILENO` via libstd's `Stdout` (which
//!   ultimately calls `write(2)` — matches the contract documented in
//!   `nros_platform::BoardPrint`).
//! - `exit_success` / `exit_failure` call `std::process::exit`.
//! - There is no device bring-up step — POSIX sockets are open as
//!   soon as `init_hardware` returns. This used to be phrased as the
//!   absence of a `TransportBringup` / `NetworkWait` impl; phase-206
//!   W4 removed those traits (issue 1067), so the absence is now
//!   universal and this board's point is simply that it needs no
//!   bring-up at all.
//!
//! ## `BoardEntry::run` body
//!
//! The body sequences the lifecycle the trait surface documents:
//!
//! ```text
//! init_hardware()            // no-op
//! ↓
//! build RuntimeCtx           // empty by default; Phase 212.N.4
//!                            // codegen will populate from env / CLI
//! ↓
//! setup(&mut ctx)            // user closure — typically
//!                            // codegen-emitted `run_plan(runtime)`
//!                            // which owns nros::init + Executor::open
//!                            // + Executor::spin internally
//! ↓
//! exit_success() or          // -> !
//! exit_failure()
//! ```
//!
//! Note that the **executor lifecycle is deliberately owned by the
//! `setup` callback** rather than by `run` itself. Every existing
//! POSIX Entry pkg `main.rs` (see `examples/native/rust/talker`) opens
//! its own `Executor`, registers timers / nodes, and calls
//! `spin_blocking` from inside what becomes the `setup` closure once
//! Phase 212.N.4 codegen lands. `run` would have nothing portable to
//! say about which `Executor` instance to spin or how, so it stays out
//! of that decision. The seam is documented under "Open seams" below.

#![forbid(unsafe_op_in_unsafe_fn)]

// Phase 248 C5a (#60 T4) — the BOARD is the RMW selection point. Under its
// own `rmw-zenoh` feature it force-links the zenoh backend rlib so the
// backend's `RMW_INIT_ENTRIES` self-register section survives stable-Rust
// rlib pruning and reaches the final binary, WITHOUT relying on the `nros`
// umbrella's `rmw-zenoh` feature. Mirrors the `__FORCE_LINK_ZENOH` static in
// `nros/src/lib.rs` (referencing `register` keeps both the symbol and its
// linker section alive — strictly stronger than the prior `extern crate _`,
// which only kept the rlib). On a host (linkme-aware + `.init_array`) the
// section auto-registers; the static guarantees it is not pruned first.
// Cycle-free: the backend crate does not depend on this board crate. Inert
// unless `rmw-zenoh` selects the backend.
#[cfg(feature = "rmw-zenoh")]
#[doc(hidden)]
#[used]
pub static __FORCE_LINK_ZENOH: fn() -> Result<(), nros_rmw_zenoh::RegisterError> =
    nros_rmw_zenoh::register;

use std::io::Write as _;

// `nros_platform::board` is `mod board;` (private); the Board trait
// family is re-exported at the crate root.
use nros_platform::{BoardEntry, BoardExit, BoardInit, BoardPrint, RuntimeCtx, TierSpec};

/// `Send` wrapper for the shared raw session pointer so it can cross the
/// `std::thread::scope` boundary. The pointed-to RMW session type is
/// `pub(crate)` in `nros-node` (unnameable here), so the wrapper is
/// generic over `T` and never names it — `T` is inferred from
/// [`nros::Executor::session_ptr`]. Sharing the pointer is sound under
/// the per-tier contract: the boot executor owns the one session, the
/// RMW backend serializes concurrent access through its own locks, and
/// `thread::scope` guarantees no spawned tier outlives the owner.
struct SharedSession<T>(*mut T);
// Hand-written Copy/Clone: `#[derive]` would add a spurious `T: Copy`
// bound (the session type isn't `Copy`), but a raw pointer always is.
impl<T> Clone for SharedSession<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Copy for SharedSession<T> {}
// SAFETY: the per-tier model shares one RMW session across tier tasks by
// design; concurrent access is serialized inside the backend.
unsafe impl<T> Send for SharedSession<T> {}

/// POSIX family driver ZST. Plug into an Entry pkg `main.rs` via:
///
/// ```ignore
/// use nros_board_linux::LinuxBoard;
/// use nros_platform::board::BoardEntry;
///
/// fn main() {
///     let _ = <LinuxBoard as BoardEntry>::run(|runtime| {
///         // codegen-emitted (Phase 212.N.4)
///         run_plan(runtime)
///     });
/// }
/// ```
pub struct LinuxBoard;

impl BoardInit for LinuxBoard {
    /// POSIX needs no hardware init: libstd's runtime already
    /// initialized the heap, stdio, signal handlers and threading
    /// before `fn main` ran. Kept as a documented no-op so the
    /// lifecycle in [`BoardEntry::run`] is uniform across families.
    #[inline]
    fn init_hardware() {}
}

impl BoardPrint for LinuxBoard {
    fn println(args: core::fmt::Arguments<'_>) {
        // Write to a stdout lock so concurrent threads don't
        // interleave a single line. `libc::write(STDOUT_FILENO, …)`
        // would also satisfy the trait, but libstd's `Stdout` already
        // bottoms out in `write(2)` and adds line-buffered locking
        // that we'd otherwise have to rebuild. If the write fails
        // (closed stdout, broken pipe) we deliberately swallow the
        // error — a board-print failure shouldn't tear down the boot.
        let mut out = std::io::stdout().lock();
        let _ = writeln!(out, "{}", args);
    }
}

impl BoardExit for LinuxBoard {
    fn exit_success() -> ! {
        std::process::exit(0)
    }

    fn exit_failure() -> ! {
        std::process::exit(1)
    }
}

/// Phase 249 P3.5 — the board owns the RMW registration on EVERY OS (hosted
/// included), not just `target_os="none"`. Calling the linked backend's
/// `register()` explicitly before the executor opens is the one universal
/// trigger; it replaces the hosted reliance on the linkme `.init_array` walk
/// (retired in P4) and lets the weak `nros_app_register_backends` default die.
/// Gated on the board's own `rmw-<x>` feature; the call also force-links the
/// backend (strictly stronger than the `#[used] __FORCE_LINK_*` static). Inert
/// when no backend feature is selected.
///
/// Phase 277 W2.b — public so a plain-API example (no `BoardEntry` /
/// `nros::main!` framework, just `LinuxBoard` + `Executor::open` directly) can
/// trigger registration itself before opening the executor. Idempotent — safe
/// to call more than once; re-registering an already-registered backend is a
/// no-op. Apps that boot through `nros::main!()` or any `BoardEntry::run*` /
/// `run_tiers` entry point never need to call this themselves — every boot path
/// below funnels through `boot_hosted` / `run_tiers`, which call it.
#[inline]
pub fn register_linked_rmw() {
    #[cfg(feature = "rmw-zenoh")]
    {
        let _ = nros_rmw_zenoh::register();
    }
    #[cfg(feature = "rmw-xrce")]
    {
        let _ = nros_rmw_xrce_cffi::register();
    }
    #[cfg(feature = "rmw-cyclonedds")]
    {
        let _ = nros_rmw_cyclonedds_sys::register();
    }
}

impl BoardEntry for LinuxBoard {
    /// Drive the boot → setup → exit flow. POSIX has no transport
    /// bringup or network-wait step:
    ///
    /// 1. [`BoardInit::init_hardware`] (no-op).
    /// 2. Open the live [`nros::Executor`] from the env-derived
    ///    [`nros::ExecutorConfig`] (`ROS_DOMAIN_ID`, `NROS_LOCATOR`,
    ///    `NROS_SESSION_MODE`) and wrap it in an
    ///    [`nros::node_runtime::ExecutorNodeRuntime`] —
    ///    Phase 212.N.7 step-3.5. The codegen-emitted
    ///    `run_plan(runtime)` body now talks to a real executor.
    /// 3. Build a [`RuntimeCtx`] backed by that runtime.
    /// 4. Invoke `setup(&mut ctx)`.
    /// 5. Log the result via [`BoardPrint::println`].
    /// 6. Diverge into [`BoardExit::exit_success`] or
    ///    [`BoardExit::exit_failure`].
    ///
    /// Native (POSIX) does **not** enter an infinite spin loop after
    /// `setup` returns — POSIX-shaped applications drive their own
    /// spinning inside `setup` (e.g. a codegen `run_plan` that calls
    /// `Executor::spin_blocking`, or an Entry pkg main that simply
    /// exits when the closure finishes). The contract mirrors the
    /// hosted nuttx carve-out and matches the existing
    /// `nros-board-posix` doc comment ("the executor open + spin
    /// happens *inside* `setup`"). The change here is that the open
    /// step is now done **for** the closure rather than by it — the
    /// `setup` body receives a live runtime sink through
    /// `RuntimeCtx::runtime` and dispatches Node pkg `register`
    /// calls into it.
    fn run<F, E>(setup: F) -> Result<(), E>
    where
        F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
        E: core::fmt::Debug,
    {
        Self::run_with_deploy(&nros_platform::DeployOverlay::default(), setup)
    }

    /// Issue #98 — name the primary session (the ROS graph node name) from the
    /// launch file's single `<node>` when `deploy.node_name` is set. Locator/IP
    /// stay env-driven (issue #48); this override exists only to thread the node
    /// name onto the boot `ExecutorConfig`. `run` forwards here with a default
    /// (empty) overlay, so this is the single hosted boot body.
    fn run_with_deploy<F, E>(deploy: &nros_platform::DeployOverlay, setup: F) -> Result<(), E>
    where
        F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
        E: core::fmt::Debug,
    {
        // Default sizing (the build-time `MAX_CBS`/`ARENA_SIZE`).
        Self::boot_hosted(deploy, None, setup)
    }

    /// phase-271 (issue #110) — hosted boot sized to the entry's own declared
    /// topology (`[package.metadata.nros.entry] max_callbacks` /
    /// `max_sched_contexts`), so a fat native entry (>default `MAX_CBS`
    /// callbacks) fits WITHOUT a workspace-global `NROS_EXECUTOR_MAX_CBS`.
    fn run_with_deploy_sized<F, E>(
        deploy: &nros_platform::DeployOverlay,
        max_cbs: usize,
        max_sched_contexts: usize,
        setup: F,
    ) -> Result<(), E>
    where
        F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
        E: core::fmt::Debug,
    {
        Self::boot_hosted(deploy, Some((max_cbs, max_sched_contexts)), setup)
    }
}

impl LinuxBoard {
    /// phase-271 (issue #98 + #110) — the single hosted boot body shared by
    /// [`BoardEntry::run_with_deploy`] (default sizing, `sizing = None`) and
    /// [`BoardEntry::run_with_deploy_sized`] (`sizing = Some((max_cbs,
    /// max_sched_contexts))` — a `0` sched-context count means "use the build
    /// default"). The ONLY difference between the two paths is whether the
    /// executor opens via `Executor::open` or `Executor::open_sized`.
    fn boot_hosted<F, E>(
        deploy: &nros_platform::DeployOverlay,
        sizing: Option<(usize, usize)>,
        setup: F,
    ) -> Result<(), E>
    where
        F: FnOnce(&mut RuntimeCtx<'_>) -> Result<(), E>,
        E: core::fmt::Debug,
    {
        // Phase 249 P3.5 / phase-337 W8.a — register the linked backend before
        // the executor opens. Was the deleted `nros-board-native` shim's job;
        // it now sits on the ONE hosted boot funnel instead of on four
        // forwarding methods.
        register_linked_rmw();
        <Self as BoardInit>::init_hardware();
        // Phase 264 W3 — wire the default log sink (host → stdout/stderr) so a Node
        // pkg's `log_info!` produces output without per-app `nros_log::init`.
        // Idempotent (swaps the sink list atomically).
        ::nros_platform_cffi::log::init_default();
        // phase-338 W3 — and the `log` facade's host sink, for the same reason.
        // Four of the five scheduled-platform node bodies log via `log::info!`
        // (their boards bridge it: threadx's `install_uart_logger`, nuttx's
        // stdout logger); this board only ever inited `nros_log`, so such a body
        // ran on native and printed nothing at all. Bridging both here lets ONE
        // node body work on every board. W7 unifies the two stacks.
        install_stdout_log_bridge();

        // Phase 212.N.7 step-3.5 — open the executor + wrap it in an
        // `ExecutorNodeRuntime` so the codegen-emitted `run_plan(runtime)` body
        // can register components against a live RMW session. Env-derived config
        // picks up `ROS_DOMAIN_ID` / `NROS_LOCATOR` / `NROS_SESSION_MODE` at
        // runtime — the host-side carve-out from the embedded compile-time
        // domain-id contract documented in CLAUDE.md.
        //
        // If executor open fails (no RMW backend linked, or the configured
        // router/peer is unreachable), we fall back to
        // [`nros_platform::NullNodeRuntime`] so the setup closure still runs. The
        // fall-back errors loud on any `register_dispatch_slot_dyn` call.
        // Issue #98 — route the single-node launch overlay through the W1
        // resolver (RFC-0045 precedence model A): env > baked > compiled default.
        // Only `node_name` is mapped from the overlay; locator/domain/namespace
        // stay `None` so env keeps authority over them (issue #48 preserved).
        // issue 0687 — the ONE `hosted_env = true` in the tree, now spelled as
        // what it is: this board runs on a host, so it resolves through the
        // edge that reads the environment. Every other board passed `false`.
        let exec_cfg = ::nros::env::resolve_hosted(::nros::BootConfig {
            node_name: deploy.node_name,
            ..Default::default()
        });
        // phase-271 — open at the entry's declared sizing when supplied.
        let opened = match sizing {
            None => ::nros::Executor::open(&exec_cfg),
            Some((cbs, sc)) => {
                let sc = if sc == 0 {
                    ::nros::ExecutorSizing::DEFAULT.sc
                } else {
                    sc
                };
                ::nros::Executor::open_sized(
                    &exec_cfg,
                    ::nros::ExecutorSizing {
                        cbs,
                        sc,
                        arena: ::nros::arena_size_for(cbs),
                        // phase-409 — the entry declares callbacks, not Nodes
                        // (`[package.metadata.nros.entry] max_callbacks`), so the
                        // Node-scaled tables keep the build-time default.
                        ..::nros::ExecutorSizing::DEFAULT
                    },
                )
            }
        };
        let mut crt_real: Option<::nros::node_runtime::ExecutorNodeRuntime> = match opened {
            Ok(e) => Some(::nros::node_runtime::ExecutorNodeRuntime::from_executor(e)),
            Err(err) => {
                <Self as BoardPrint>::println(format_args!(
                    "nros: Executor::open failed ({err:?}); proceeding with NullNodeRuntime — \
                     `run_plan` register calls will fail loud."
                ));
                None
            }
        };
        let mut crt_null = ::nros_platform::NullNodeRuntime;
        let result = match crt_real.as_mut() {
            Some(crt) => {
                let mut runtime = RuntimeCtx::with_runtime(crt);
                setup(&mut runtime)
            }
            None => {
                let mut runtime = RuntimeCtx::with_runtime(&mut crt_null);
                setup(&mut runtime)
            }
        };
        match result {
            Ok(()) => {
                <Self as BoardPrint>::println(format_args!("nros: application complete"));
                <Self as BoardExit>::exit_success();
            }
            Err(e) => {
                <Self as BoardPrint>::println(format_args!("nros: application error: {e:?}"));
                <Self as BoardExit>::exit_failure();
            }
        }
    }

    /// Phase 228.E — per-tier multi-task entry. Opens the one RMW
    /// session, then runs one `Executor` per [`TierSpec`] over that
    /// shared session: the LEAST urgent tier runs on the boot task (chosen by
    /// `boot_tier_index` — issue 0636, because an owner that outranks its peers
    /// and then spins starves them); the rest are spawned as `std::thread`s. Each tier sets its
    /// `active_groups` filter, runs `setup` (register-only — only this
    /// tier's callbacks take), then spins forever.
    ///
    /// `setup` is `Fn` (not `FnOnce`) — it is invoked once per tier
    /// executor — and `Sync`, since spawned tiers share `&setup`. It
    /// must register entities only; the spin loop is owned here so the
    /// board can install the group filter first. (The single-tier
    /// [`BoardEntry::run`] path, where `setup` owns the spin, is
    /// unchanged.)
    ///
    /// Native preemption uses the default scheduler; the normalized
    /// [`TierSpec::priority`] is advisory here (strict ordering needs
    /// `SCHED_FIFO` + privileges). The FreeRTOS port maps it to real
    /// task priorities (RFC-0016). Blocks forever (server semantics);
    /// returns only if a tier `setup` fails before the spin loop.
    pub fn run_tiers<F, E>(
        _deploy: &nros_platform::DeployOverlay,
        tiers: &[TierSpec<'_>],
        setup: F,
    ) -> Result<(), E>
    where
        F: Fn(&mut RuntimeCtx<'_>) -> Result<(), E> + Sync,
        E: core::fmt::Debug,
    {
        // Issue #48 — hosted boards take their locator from `from_env()`, so the
        // deploy overlay is ignored here (kept for signature parity with the
        // firmware boards' `run_tiers`).
        // phase-337 W8.a — the second boot funnel, so it registers too.
        register_linked_rmw();
        <Self as BoardInit>::init_hardware();
        // Phase 264 W3 — default log sink at boot (see `run`).
        ::nros_platform_cffi::log::init_default();
        // Same rationale as `run` (phase-338 W3): bridge the `log` facade
        // too, or a node body using `log::info!` prints nothing on the
        // multi-tier native entry while working on every RTOS board.
        install_stdout_log_bridge();

        if tiers.is_empty() {
            <Self as BoardPrint>::println(format_args!(
                "nros: run_tiers called with no tiers — nothing to run"
            ));
            <Self as BoardExit>::exit_failure();
        }

        // RFC-0079 / issue 0765 — place the transport ABOVE every tier, BEFORE
        // the session opens. The read/lease tasks are created during open and
        // `zpico_set_task_config` is a process-wide default consulted at that
        // moment; after open it changes nothing.
        //
        // Necessary here in a way it is not on the RTOSes. Once tier priorities
        // became real (issue 0765) the tiers run SCHED_FIFO while the read and
        // lease tasks stay on SCHED_OTHER, and a SCHED_FIFO thread outranks
        // every SCHED_OTHER thread unconditionally — so the app preempted the
        // link it publishes over, always, with no arrangement in which it did
        // not. That is not one of the two legitimate orderings issue 0623
        // describes; it is the absence of a choice.
        //
        // DERIVED, not a literal: the right value depends on the tier table
        // this entry was generated with, and a constant that stops being above
        // the tiers when someone edits `system.toml` is worse than no constant.
        // Best-effort like every other priority on this port — without
        // `CAP_SYS_NICE` the attribute is ignored at spawn and the tiers'
        // own refusal line has already said why.
        // `feature = "rmw-zenoh"` as well as `unix`: the symbol below lives in
        // zpico-sys's C shim, which is only in the graph when the zenoh backend
        // is selected. Guarded on `unix` alone, a board built for another RMW
        // referenced a symbol nothing linked — `undefined symbol:
        // zpico_set_task_config`, at LINK time in the consumer, naming neither
        // this crate nor the missing backend.
        //
        // Same family as issue 0919 one layer up: a component assuming zenoh is
        // present because it usually is. The priority plan below is about
        // TRANSPORT tasks, and a build with no zenoh transport has none to
        // place — so skipping it is the correct behaviour, not a degradation.
        #[cfg(all(unix, feature = "rmw-zenoh"))]
        {
            // RFC-0079 — the RESERVED band's floor, not `max_tier + 1`.
            //
            // Deriving it from the tiers was circular: a reserved band defined
            // by the tiers it is meant to sit above moves whenever they do, and
            // cannot be checked statically because nothing knows it until the
            // entry runs. Every other port states a fixed band (FreeRTOS 4,
            // NuttX 100, ThreadX 14) with the pool below it; POSIX now does
            // too — `[board.priority_plan]` in packages/boards/linux, reserved
            // 90..99, pool 1..89, cross-checked against this constant by
            // `check-tier-priority-plan` so the two cannot drift.
            const TRANSPORT_BAND_FLOOR: u32 = 90;
            let max_tier = tiers.iter().map(|t| t.priority).max().unwrap_or(0);
            if max_tier > 0 {
                let transport = TRANSPORT_BAND_FLOOR;
                unsafe extern "C" {
                    fn zpico_set_task_config(
                        read_priority: u32,
                        read_stack_bytes: u32,
                        lease_priority: u32,
                        lease_stack_bytes: u32,
                    );
                }
                // 0 stack bytes = "leave the port's default alone" — this
                // states a PRIORITY, not a stack size.
                unsafe { zpico_set_task_config(transport, 0, transport, 0) };
                <Self as BoardPrint>::println(format_args!(
                    "nros: transport tasks at SCHED_FIFO {} (reserved band floor; most \
                     urgent tier is {})",
                    transport, max_tier
                ));
            }
        }

        // Open the one session on the boot task; it owns the session for
        // the program's life (the boot tier's spin loop never returns).
        // issue 0687 — `from_env` is an extension trait now (the environment is
        // read at `nros`'s edge, not in the core), so it needs to be in scope.
        use ::nros::ExecutorConfigEnvExt as _;
        let exec_cfg = ::nros::ExecutorConfig::from_env();
        let boot_exec = match ::nros::Executor::open(&exec_cfg) {
            Ok(e) => e,
            Err(err) => {
                <Self as BoardPrint>::println(format_args!(
                    "nros: Executor::open failed ({err:?}); multi-tier entry needs a live \
                     session — aborting."
                ));
                <Self as BoardExit>::exit_failure();
            }
        };
        let mut boot_crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(boot_exec);
        let shared = SharedSession(boot_crt.executor_mut().session_ptr());

        // phase-302 W2 (issue 0262) — posix tier priorities/pins are ADVISORY
        // (no sched_setscheduler/affinity consumer; strict ordering needs
        // SCHED_FIFO + privileges — phase-162 territory). Say so ONCE, loudly,
        // when any tier declares them, instead of silently dropping the knobs.
        // issue 0765 — priority is no longer advisory here; `core` still is.
        //
        // This note used to say BOTH were "advisory (not applied natively)",
        // which was true and is no longer: each tier now self-applies its
        // declared SCHED_FIFO priority at entry (`apply_tier_priority`) and
        // says so, or says why not. Leaving the old wording would have been
        // worse than the original silence — a reader would believe a
        // declaration was inert while it was being honoured.
        //
        // …and then the `core` half went stale in exactly that way. phase-296
        // W5.13 landed `apply_tier_affinity`, called for EVERY tier — spawned
        // (`run_one_tier`) and boot — which pins via `sched_setaffinity` and
        // prints `core pin tier=…` on success or `core pin FAILED …` on
        // refusal. So `core` has not been advisory since; the note went on
        // saying it was, which is the failure the paragraph above describes,
        // one field over. There is now nothing to announce up front: both
        // dims report their own outcome per tier, which is strictly better
        // than a blanket note that can rot.
        //
        // Kept as a `linux`, not `posix`, statement — the pin is
        // `sched_setaffinity`, absent from libc's apple module.
        let setup = &setup;
        // Issue 0447 — held across each tier's `setup` so entity declaration on
        // the one shared session is serialized (see `run_one_tier`).
        let setup_lock = std::sync::Mutex::new(());
        let setup_lock = &setup_lock;
        // issue 0636 — the boot tier is CHOSEN, not `tiers[0]`. POSIX runs
        // bigger-is-more-urgent, and `resolve_tiers` orders by raw number
        // descending without inverting per kernel, so `tiers[0]` was the MOST
        // urgent tier here. An owner that outranks its peers and then spins
        // starves them on any single-CPU deployment (measured on NuttX, which
        // runs this same POSIX shape); `boot_tier_index` picks the tier that
        // outranks nothing. Where every tier declares the same priority — or
        // none — this is still index 0.
        let boot_index = ::nros_platform::boot_tier_index(
            tiers,
            ::nros_platform::PriorityDirection::BiggerIsMoreUrgent,
        );
        std::thread::scope(|scope| {
            // Spawn every tier except the one the boot task runs; each borrows
            // the shared session pointer and `&setup` from the enclosing scope.
            for (spawn_index, tier) in tiers.iter().enumerate() {
                if spawn_index == boot_index {
                    continue;
                }
                let mut builder =
                    std::thread::Builder::new().name(format!("nros-tier-{}", tier.name));
                // phase-302 W2 (issue 0262) — honor a declared per-tier stack
                // (previously silently ignored; std default otherwise).
                if tier.stack_bytes > 0 {
                    builder = builder.stack_size(tier.stack_bytes);
                }
                let spawn = builder.spawn_scoped(scope, move || {
                    // Re-bind the whole wrapper so the closure captures the
                    // `Send` `SharedSession`, not the bare `*mut` field
                    // (edition-2021 disjoint capture would grab the field).
                    let shared = shared;
                    // SAFETY: `shared.0` aliases the boot executor's
                    // session, kept alive for this scope by `thread::scope`.
                    let exec = unsafe { ::nros::Executor::open_with_session(shared.0) };
                    run_one_tier::<Self, F, E>(exec, tier, setup, setup_lock);
                });
                if let Err(e) = spawn {
                    <Self as BoardPrint>::println(format_args!(
                        "nros: failed to spawn tier `{}`: {e}",
                        tier.name
                    ));
                }
            }
            // Reached once the session is open + every non-boot tier task is
            // spawned; the boot tier then registers + spins below. Unique line
            // (the single-tier path never prints it) so an E2E can confirm the
            // emitted binary entered the per-tier run with a live session.
            <Self as BoardPrint>::println(format_args!(
                "nros: multi-tier run — {} tier(s) over one session",
                tiers.len()
            ));
            // Boot tier runs on this task, reusing the owning executor.
            run_boot_tier::<Self, F, E>(&mut boot_crt, &tiers[boot_index], setup, setup_lock);
        });

        // Unreachable: the boot tier's spin loop never returns.
        Ok(())
    }
}

/// RFC-0052 / phase-296 W3a — lower a tier's RTOS-agnostic scheduling
/// policy onto the tier executor's DEFAULT scheduling context. One
/// Executor per tier means "the tier's policy" == "this executor's
/// default SC"; per-group/per-handle bindings still take precedence.
///
/// v1 lowers the sporadic-budget shape (`class = "real_time"` with
/// `budget_us` + `period_us` → `SchedClass::Sporadic`) and `best_effort`;
/// `time_triggered` window registration is W3b (needs the major-frame
/// dispatcher wired at this altitude too). A tier with no class/budget
/// leaves the default Fifo SC untouched — byte-identical pre-W3 behavior.
fn apply_tier_sched(crt: &mut ::nros::node_runtime::ExecutorNodeRuntime, tier: &TierSpec<'_>) {
    // W3a lowering now lives portably on `ExecutorNodeRuntime` (W5.4) so every
    // board shares it; posix just forwards its tier fields.
    crt.apply_tier_sched_policy(
        tier.class,
        tier.period_us,
        tier.budget_us,
        tier.deadline_us,
        tier.deadline_policy,
    );
}

/// phase-296 W5.13 — the placement dim's LINUX realization: pin the CALLING
/// tier thread to its declared `core` via `sched_setaffinity` (no-op when the
/// tier declares no core). Unlike the RTOS SMP arms (zephyr/nuttx/freertos/
/// threadx), a Linux host is genuinely multi-core and the call needs no
/// privilege, so this is the FIRST RUNTIME ACCEPT-arm proof of the core-pin
/// consumer (issue #260). Never silently drops: a rejected pin (bad cpu id)
/// falls back LOUDLY and the tier runs unpinned.
///
/// **Linux, not POSIX** — this doc said "POSIX" and the distinction is the
/// whole reason the board is named `linux`. `sched_setaffinity`, `cpu_set_t`
/// and `CPU_SET` are absent from libc's apple module, and nothing here is
/// `cfg(target_os)`-gated, so this crate does not build on macOS. Gating it to
/// a loud no-op off Linux is what would make the board's reach `posix`; until
/// someone needs that, the accurate word is the one in the crate name.
fn apply_tier_affinity<B: BoardPrint>(tier: &TierSpec<'_>) {
    let Some(core) = tier.core else {
        return;
    };
    // SAFETY: libc FFI — a zeroed `cpu_set_t` with a single CPU bit set, applied
    // to the calling thread (pid 0). No aliasing, no allocation.
    let accepted = unsafe {
        let mut set: libc::cpu_set_t = core::mem::zeroed();
        libc::CPU_SET(core as usize, &mut set);
        libc::sched_setaffinity(0, core::mem::size_of::<libc::cpu_set_t>(), &set) == 0
    };
    if accepted {
        // Literal mirrors `nros_tests::output::LINUX_CORE_PIN_MARKER`.
        B::println(format_args!(
            "nros: core pin tier=`{}` cpu={}",
            tier.name, core
        ));
    } else {
        // Literal mirrors `nros_tests::output::LINUX_CORE_PIN_FALLBACK_MARKER`.
        B::println(format_args!(
            "nros: core pin FAILED tier=`{}` cpu={} — sched_setaffinity failed, \
             tier runs unpinned",
            tier.name, core
        ));
    }
}

// NOTE: this text describes a different function and is preserved verbatim,
// only demoted from `///` to `//`. A doc comment cannot attach to an `extern`
// block, so under the lane's `-D warnings` it was a hard `unused_doc_comments`
// error; whoever owns issue 0765 should say what it meant to say.
// Register + spin one tier on a freshly-opened borrowed-session
// executor (spawned-tier path).
// RFC-0079 / issue 0765 — see `apply_tier_priority` below.
#[cfg(unix)]
unsafe extern "C" {
    fn nros_posix_apply_current_priority(name: *const core::ffi::c_char, priority: u32) -> i32;
}

/// Adopt this tier's declared SCHED_FIFO priority on the CALLING thread, and
/// report the outcome.
///
/// The native board spawns tiers with `std::thread::scope`, which never reaches
/// `nros-platform-posix`'s `task_init` — so the attribute path that gives NuttX
/// its priorities cannot reach a native tier, and every `[tiers.*.posix]
/// priority` in the tree was inert with nothing saying so. This is the same
/// self-apply-at-tier-entry shape NuttX and FreeRTOS already use: one
/// implementation per port, one marker, and a refusal that is REPORTED.
///
/// Not fatal when refused. An unprivileged host is the normal case for `just
/// ci`, and trading a silent degradation for a stopped test suite would be a
/// bad bargain; the operator gets a line naming `setcap cap_sys_nice+ep`
/// instead.
#[cfg(unix)]
fn apply_tier_priority(tier: &TierSpec<'_>) {
    if tier.priority <= 0 {
        return;
    }
    let prio = tier.priority.clamp(0, i64::from(u32::MAX)) as u32;
    // The tier name reaches the C side for the marker; a NUL-bearing name is
    // simply reported as `?` rather than failing the tier.
    match std::ffi::CString::new(tier.name) {
        Ok(c) => unsafe {
            nros_posix_apply_current_priority(c.as_ptr(), prio);
        },
        Err(_) => unsafe {
            nros_posix_apply_current_priority(core::ptr::null(), prio);
        },
    }
}

#[cfg(not(unix))]
fn apply_tier_priority(_tier: &TierSpec<'_>) {}

fn run_one_tier<B, F, E>(
    exec: ::nros::Executor<'static>,
    tier: &TierSpec<'_>,
    setup: &F,
    setup_lock: &std::sync::Mutex<()>,
) where
    B: BoardPrint,
    F: Fn(&mut RuntimeCtx<'_>) -> Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0765 — the tier's OWN priority, before it does any work, so the
    // declaration holds for the setup it is about to run and not just for the
    // spin loop afterwards.
    apply_tier_priority(tier);
    let mut crt = ::nros::node_runtime::ExecutorNodeRuntime::from_executor(exec);
    crt.executor_mut().set_active_groups(tier.groups);
    apply_tier_sched(&mut crt, tier);
    apply_tier_affinity::<B>(tier);
    {
        // Issue 0447 — SERIALIZE registration across tiers. Every tier executor
        // borrows the ONE shared session and declares its entities on it, and the
        // boot tier runs its own `setup` concurrently with these spawned ones with
        // no synchronization. Interleaved declaration bound topics
        // non-deterministically: the same binary would deliver correctly, or land
        // the 10 ms tier's samples on the 100 ms tier's topic, or deliver nothing.
        // Registration happens once per boot and off the hot path, so the mutex
        // costs nothing that matters; the spin loops stay fully concurrent.
        let _guard = setup_lock.lock().unwrap_or_else(|e| e.into_inner());
        let mut ctx = RuntimeCtx::with_runtime(&mut crt);
        if let Err(e) = setup(&mut ctx) {
            B::println(format_args!(
                "nros: tier `{}` setup failed: {e:?} — tier task exiting",
                tier.name
            ));
            return;
        }
    }
    spin_forever::<B>(&mut crt, tier);
}

/// Register + spin the boot tier on the session-owning executor.
fn run_boot_tier<B, F, E>(
    crt: &mut ::nros::node_runtime::ExecutorNodeRuntime,
    tier: &TierSpec<'_>,
    setup: &F,
    setup_lock: &std::sync::Mutex<()>,
) where
    B: BoardPrint,
    F: Fn(&mut RuntimeCtx<'_>) -> Result<(), E>,
    E: core::fmt::Debug,
{
    // issue 0765 — the BOOT tier adopts its declared priority too.
    //
    // It runs on the calling thread rather than a spawned one, so it is the
    // tier most easily forgotten — and forgetting it is not hypothetical: the
    // same omission was a real defect on FreeRTOS (the boot task adopted
    // nothing at all) and cost issue 0636 a measurement round on NuttX. A tier
    // is a tier whichever thread carries it.
    apply_tier_priority(tier);
    crt.executor_mut().set_active_groups(tier.groups);
    apply_tier_sched(crt, tier);
    apply_tier_affinity::<B>(tier);
    {
        // Issue 0447 — see `run_one_tier`: the boot tier's registration races the
        // spawned tiers' on the shared session unless they take the same lock.
        let _guard = setup_lock.lock().unwrap_or_else(|e| e.into_inner());
        let mut ctx = RuntimeCtx::with_runtime(crt);
        if let Err(e) = setup(&mut ctx) {
            B::println(format_args!(
                "nros: boot tier `{}` setup failed: {e:?}",
                tier.name
            ));
            return;
        }
    }
    spin_forever::<B>(crt, tier);
}

/// Drive a tier executor's `spin_once` at its declared period, forever.
fn spin_forever<B: BoardPrint>(
    crt: &mut ::nros::node_runtime::ExecutorNodeRuntime,
    tier: &TierSpec<'_>,
) {
    let period = std::time::Duration::from_micros(tier.spin_period_us.max(1));
    // issue 0636 option 3 — every iteration reaches a scheduling point. The
    // executor's own wait is SKIPPED whenever a wake already fired, so under
    // sustained traffic this loop would otherwise never block. On a host under
    // SCHED_OTHER the kernel preempts anyway; under the SCHED_FIFO this board
    // asks for when it can get it, a thread that never blocks never lets a
    // lower-priority tier run. Costs nothing while the spins do block.
    let mut gap = ::nros_platform::TierSpinGap::new(tier.spin_period_us);
    loop {
        let iter = gap.mark();
        if let Err(e) = crt.spin_once(period) {
            B::println(format_args!("nros: tier `{}` spin error: {e:?}", tier.name));
        }
        gap.after_spin(iter);
    }
}

/// phase-338 W3 — route the `log` facade to stdout on hosted targets.
///
/// Mirrors `nros-board-nuttx::install_stdout_logger`. `set_logger` may only
/// succeed once per process, and the `is_ok()` guard makes this a NO-OP when
/// the application installed its own (the imperative examples call
/// `env_logger::init()` in `main` before the board ever boots) — so this adds a
/// sink where there was none and never fights one that exists.
fn install_stdout_log_bridge() {
    use std::{io::Write as _, sync::Once};

    struct StdoutLogger;
    impl log::Log for StdoutLogger {
        fn enabled(&self, _: &log::Metadata<'_>) -> bool {
            true
        }
        fn log(&self, record: &log::Record<'_>) {
            // The examples bake the full human line into the message
            // (`Publishing: '...'` / `I heard: [...]`), so the message is
            // emitted verbatim — the e2e harness greps those markers with
            // `contains`, so the level prefix below leaves every one intact.
            //
            // The prefix is not decoration. Issue 0309 made the workspace
            // logging proof require the level tag specifically so that a line
            // carrying the marker but no tag reads as "this record bypassed the
            // logging facade". When this bridge landed (phase-338 W3) it became
            // the Rust lane's `log::Log`, and printing `record.args()` bare made
            // every Rust record look exactly like the bypass the proof was
            // written to catch — `workspace_features_e2e` rust/logging went to
            // 0 tagged lines while c/cpp/mixed stayed green, because those go
            // through the C API to `nros_platform_log_write`, which prefixes.
            //
            // Labels match `severity_label_log` in nros-platform-posix so the
            // two lanes render a record the same way.
            let level = match record.level() {
                log::Level::Trace => "TRACE",
                log::Level::Debug => "DEBUG",
                log::Level::Info => "INFO",
                log::Level::Warn => "WARN",
                log::Level::Error => "ERROR",
            };
            let mut out = std::io::stdout();
            let _ = writeln!(out, "[{level}] {}", record.args());
            let _ = out.flush();
        }
        fn flush(&self) {
            let _ = std::io::stdout().flush();
        }
    }
    static LOGGER: StdoutLogger = StdoutLogger;
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        if log::set_logger(&LOGGER).is_ok() {
            log::set_max_level(log::LevelFilter::Trace);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_hardware_is_noop() {
        // Smoke: calling `init_hardware` from a unit test must not
        // panic or affect global state.
        <LinuxBoard as BoardInit>::init_hardware();
    }

    #[test]
    fn println_writes_without_panicking() {
        <LinuxBoard as BoardPrint>::println(format_args!("nros-board-linux: hello from unit test"));
    }

    // Note: `BoardEntry::run` itself can't be unit-tested directly
    // because both exit branches diverge (`-> !`) via
    // `std::process::exit`, which would kill the test process. The
    // doc comment on `BoardEntry::run` explicitly preserves the
    // `-> Result` shape on the *callback* path so production boards
    // can still wrap the trait in a non-diverging test harness; that
    // harness lives outside this crate.
}
