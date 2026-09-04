//! RTIC board-entry surface for QEMU MPS2-AN385 — the `rtic` feature.
//!
//! phase-337 W6.a folded `nros-board-rtic-mps2-an385` in here. That crate was
//! never independent: it depended on this one and called its `init_hardware` /
//! `exit_success` / `exit_failure` / `enable_wfi_idle`, so all it added was the
//! framework-owned entry surface — the deferred-callback SPSC queue, the
//! [`RticBoardEntry`] impl, and the CMSDK TIMER0 tick. What it *also* carried
//! was a second board `Config` builder and a second `mask_to_prefix`, which
//! drifted from this crate's (see [`crate::Config::qemu_slirp`]).
//!
//! The [`RticMps2An385`] ZST stays a distinct type from [`crate::Mps2An385`]:
//! they are two entry shapes on one board (RTIC owns its own reset vector and
//! dispatch; direct-exec spins inline), and `board_path_for("rtic-mps2-an385")`
//! names this one.

use core::{
    fmt::Arguments,
    sync::atomic::{AtomicBool, Ordering},
};
// Only the synthetic-callback E2E path uses this.
#[cfg(feature = "e2e-synthetic-callback")]
use core::mem::MaybeUninit;

use heapless::spsc::{Consumer, Producer, Queue};
#[cfg(feature = "e2e-synthetic-callback")]
use nros::PublisherResolver;
use nros_platform::{
    BoardExit, BoardInit, BoardPrint, DeployOverlay, DispatchStrategy, NodeDispatchRuntime,
    RticBoardEntry, SignaledCallback,
};

use crate::Config;

pub const QUEUE_CAPACITY: usize = 32;

#[repr(transparent)]
pub struct SignaledCallbackEnvelope(SignaledCallback<'static>);

unsafe impl Send for SignaledCallbackEnvelope {}

impl SignaledCallbackEnvelope {
    pub fn into_inner(self) -> SignaledCallback<'static> {
        self.0
    }
}

static mut CALLBACK_QUEUE: Queue<SignaledCallbackEnvelope, QUEUE_CAPACITY> = Queue::new();
static DISPATCH_QUEUE_CLAIMED: AtomicBool = AtomicBool::new(false);

pub fn take_dispatch_queue() -> Option<(
    Producer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>,
    Consumer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>,
)> {
    if DISPATCH_QUEUE_CLAIMED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return None;
    }

    // SAFETY: the claim flag grants unique access to the private static queue.
    let queue: &'static mut Queue<SignaledCallbackEnvelope, QUEUE_CAPACITY> =
        unsafe { &mut *core::ptr::addr_of_mut!(CALLBACK_QUEUE) };
    Some(queue.split())
}

static mut DISPATCH_CONSUMER_SLOT: Option<
    Consumer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>,
> = None;
static DISPATCH_CONSUMER_STASHED: AtomicBool = AtomicBool::new(false);

fn stash_dispatch_consumer(consumer: Consumer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>) {
    // SAFETY: called once during RTIC init before tasks spawn.
    unsafe {
        let slot = core::ptr::addr_of_mut!(DISPATCH_CONSUMER_SLOT);
        (*slot) = Some(consumer);
    }
    DISPATCH_CONSUMER_STASHED.store(true, Ordering::Release);
}

pub fn take_dispatch_consumer()
-> Option<Consumer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>> {
    if !DISPATCH_CONSUMER_STASHED.swap(false, Ordering::AcqRel) {
        return None;
    }
    // SAFETY: the swap grants unique access to the slot.
    unsafe {
        let slot = core::ptr::addr_of_mut!(DISPATCH_CONSUMER_SLOT);
        (*slot).take()
    }
}

pub struct RticRuntime {
    producer: Option<Producer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>>,
}

impl RticRuntime {
    pub const fn new() -> Self {
        Self { producer: None }
    }

    pub const fn with_producer(
        producer: Producer<'static, SignaledCallbackEnvelope, QUEUE_CAPACITY>,
    ) -> Self {
        Self {
            producer: Some(producer),
        }
    }
}

impl Default for RticRuntime {
    fn default() -> Self {
        Self::new()
    }
}

impl NodeDispatchRuntime for RticRuntime {
    fn spin_once(&mut self, _timeout_ms: u32) -> Result<(), ()> {
        Err(())
    }

    fn signal_callback(&mut self, cb: SignaledCallback<'_>) {
        let envelope = SignaledCallbackEnvelope(unsafe {
            core::mem::transmute::<SignaledCallback<'_>, SignaledCallback<'static>>(cb)
        });
        if let Some(producer) = self.producer.as_mut() {
            let _ = producer.enqueue(envelope);
        }
    }

    fn dispatch_strategy(&self) -> DispatchStrategy {
        DispatchStrategy::Deferred
    }
}

pub struct RticMps2An385;

impl BoardInit for RticMps2An385 {
    fn init_hardware() {}
}

impl BoardPrint for RticMps2An385 {
    fn println(args: Arguments<'_>) {
        cortex_m_semihosting::hprintln!("{}", args);
    }
}

impl BoardExit for RticMps2An385 {
    fn exit_success() -> ! {
        crate::exit_success()
    }

    fn exit_failure() -> ! {
        crate::exit_failure()
    }
}

/// Phase 244.D1 — overlay a `[package.metadata.nros.deploy.rtic-mps2-an385]`
/// block onto [`Config::qemu_slirp`], so each RTIC Entry pkg can pin its own
/// ip / locator / gateway (required when the talker-rtic + listener-rtic
/// pub/sub pair share this board on one QEMU network). `None` fields keep the
/// baked default.
///
/// phase-337 W6.a — the netmask→prefix step was a hand-rolled popcount here
/// AND in [`crate::entry`]; both now call
/// [`nros_board_common::prefix_from_netmask`].
fn config_with_overlay(deploy: &DeployOverlay) -> Config {
    let mut config = Config::qemu_slirp();
    if let Some(locator) = deploy.locator {
        config.zenoh_locator = locator;
    }
    if let Some(ip) = deploy.ip {
        config.ip = ip;
    }
    if let Some(gateway) = deploy.gateway {
        config.gateway = gateway;
    }
    if let Some(netmask) = deploy.netmask {
        config.prefix = nros_board_common::prefix_from_netmask(netmask);
    }
    if let Some(domain_id) = deploy.domain_id {
        config.domain_id = domain_id;
    }
    config
}

/// #178 — hardware-ready deferred-open carrier. Holds the `'static` transport
/// parameters so [`RticMps2An385::open_executor`] can open the executor from the
/// RTIC run task (interrupts live) instead of `#[init]` (interrupts masked,
/// which would deadlock the zenoh TCP handshake).
pub struct RticBoot {
    locator: &'static str,
    domain_id: u32,
    node_name: &'static str,
}

/// Shared RTIC `#[init]` body: bring up the board from `config`, register the
/// linked RMW backend, split the dispatch SPSC, and build the dispatch runtime.
/// #178 — does NOT open the executor (that blocking connect is deferred to
/// [`RticMps2An385::open_executor`]); returns the `(RticBoot, RticRuntime)` pair.
///
/// `deploy` — the `[package.metadata.nros.deploy.<board>]` overlay; `None` on
/// the no-deploy code path.  Issue #98 / RFC-0045 — node name comes from
/// `deploy.boot_config` (the baked `.nros_boot_config`), falling back to the
/// board-historical default `"nros-rtic-mps2"`.
fn init_with_config(config: Config, deploy: Option<&DeployOverlay>) -> (RticBoot, RticRuntime) {
    crate::init_hardware(&config);

    // Phase 289 (#191 class) — install the agnostic `nros_log` dispatcher so
    // the Node pkgs' `log_info!` marker lines (`Publishing:` / `I heard:`)
    // reach the semihosting console. The direct-exec entry does this in its
    // `BoardEntry` boot (`crate::entry`); the RTIC entry never did — nodes
    // registered their `Logger` against an uninitialized dispatcher and every
    // record was silently dropped.
    nros_platform_cffi::log::init_default();

    // phase-338 W7 — bridge `log` on the RTIC path too, not just `entry::boot`.
    // Without this an RTIC node body written against `log::info!` compiles and
    // prints NOTHING, which is the silent-failure mode W7.b exists to avoid:
    // the facade must not depend on which of the board's two boot paths ran.
    crate::log_bridge::install_semihosting_log_bridge();

    // Phase 289 (#178 layer 3) — arm CMSDK TIMER0 as the periodic tick that
    // wakes the `wfi` idle-yield installed by `on_interrupts_live`. The
    // proc-macro emits a `#[task(binds = TIMER0, priority = 2)]` handler
    // (RTIC wires the real vector; NVIC unmask is RTIC's job), which calls
    // `on_tick` to acknowledge. Arming here is pure register config — no
    // interrupt is DELIVERED until `#[init]` returns and RTIC unmasks.
    arm_tick_timer();

    // Phase 248 C1 (#60 T4) — gated behind the optional `rmw-zenoh`
    // feature so the board can build DDS-/XRCE-only; another `nros-rmw-*`
    // crate then registers the linked backend. (No network I/O — a
    // section-free registry insert — so it stays in `#[init]`.)
    #[cfg(feature = "rmw-zenoh")]
    match nros_rmw_zenoh::register() {
        Ok(()) => {}
        Err(_) => {
            crate::exit_failure();
        }
    }

    // Issue #98 / RFC-0045 — node name from the baked `.nros_boot_config`
    // when a deploy overlay is present; fall back to `option_env!("NROS_NODE_NAME")`
    // (compile-time override, mirrors `nros-board-embassy-stm32f4::init_hardware`)
    // and finally to the board-historical default so undeployed firmware keeps
    // its prior identity.
    let node_name = deploy
        .and_then(|d| d.boot_config)
        .map(::nros::BootConfig::from_baked)
        .and_then(|b| b.node_name)
        .or(option_env!("NROS_NODE_NAME"))
        .unwrap_or("nros-rtic-mps2");

    let (producer, consumer) = take_dispatch_queue()
        .expect("RticMps2An385::init_hardware: dispatch queue already claimed");
    stash_dispatch_consumer(consumer);
    // `mut` is only needed by the synthetic-callback enqueue below.
    #[cfg_attr(not(feature = "e2e-synthetic-callback"), allow(unused_mut))]
    let mut runtime = RticRuntime::with_producer(producer);

    #[cfg(feature = "e2e-synthetic-callback")]
    enqueue_e2e_callback(&mut runtime);

    // `config.zenoh_locator` is a `&'static str` (option_env / baked literal);
    // node_name likewise — so the carrier is `'static`, no borrow of `config`.
    let boot = RticBoot {
        locator: config.zenoh_locator,
        domain_id: config.domain_id,
        node_name,
    };
    (boot, runtime)
}

// ---- Phase 289 — CMSDK TIMER0 tick (raw MMIO; the PAC is IRQ-names-only) ----
//
// CMSDK APB timer 0 on MPS2-AN385 (QEMU `hw/timer/cmsdk-apb-timer.c`,
// base per `hw/arm/mps2.c`): CTRL 0x0 (bit0 EN, bit3 IRQEN), VALUE 0x4,
// RELOAD 0x8, INTSTATUS/INTCLEAR 0xC (write 1 to clear). Clocked at the
// 25 MHz system clock → RELOAD 25 000 = 1 ms tick. 1 ms bounds every
// `wfi` wait in the connect/poll busy-waits without measurable IRQ load.
const CMSDK_TIMER0_BASE: usize = 0x4000_0000;
const TIMER_CTRL: usize = 0x0;
const TIMER_RELOAD: usize = 0x8;
const TIMER_INTCLEAR: usize = 0xC;
const TIMER_CTRL_EN: u32 = 1 << 0;
const TIMER_CTRL_IRQEN: u32 = 1 << 3;
/// 25 MHz sysclk / 1 kHz tick.
const TICK_RELOAD: u32 = 25_000;

fn arm_tick_timer() {
    // SAFETY: CMSDK TIMER0 MMIO, single-core, called once during `#[init]`
    // before any task runs; same raw-MMIO discipline as the UART/LAN9118
    // bring-up in `crate::node`.
    unsafe {
        core::ptr::write_volatile((CMSDK_TIMER0_BASE + TIMER_RELOAD) as *mut u32, TICK_RELOAD);
        core::ptr::write_volatile(
            (CMSDK_TIMER0_BASE + TIMER_CTRL) as *mut u32,
            TIMER_CTRL_EN | TIMER_CTRL_IRQEN,
        );
    }
}

fn clear_tick_irq() {
    // SAFETY: write-1-to-clear INTCLEAR; safe from the tick handler.
    unsafe {
        core::ptr::write_volatile((CMSDK_TIMER0_BASE + TIMER_INTCLEAR) as *mut u32, 1);
    }
}

impl RticBoardEntry for RticMps2An385 {
    type Pac = mps2_an385_pac::Peripherals;
    type Core = cortex_m::Peripherals;
    type Executor = ::nros::Executor<'static>;
    type Runtime = RticRuntime;
    type Boot = RticBoot;

    const DISPATCHERS: &'static [&'static str] = &["UARTRX0", "UARTTX0"];

    fn init_hardware(_device: Self::Pac, _core: Self::Core) -> (Self::Boot, Self::Runtime) {
        init_with_config(Config::qemu_slirp(), None)
    }

    fn init_hardware_with_deploy(
        _device: Self::Pac,
        _core: Self::Core,
        deploy: &DeployOverlay,
    ) -> (Self::Boot, Self::Runtime) {
        init_with_config(config_with_overlay(deploy), Some(deploy))
    }

    /// #178 — the blocking executor open, called from the RTIC run task.
    fn open_executor(boot: Self::Boot) -> Self::Executor {
        let exec_config = ::nros::ExecutorConfig::new(boot.locator)
            .domain_id(boot.domain_id)
            .node_name(boot.node_name);
        match ::nros::Executor::open(&exec_config) {
            Ok(e) => e,
            Err(_) => crate::exit_failure(),
        }
    }

    /// Phase 289 — acknowledge TIMER0. Un-cleared = IRQ storm that starves
    /// the priority-1 `__nros_run` task.
    fn on_tick() {
        clear_tick_irq();
    }

    /// Phase 289 (#178 layer 2) — install `wfi` on both busy-wait sites
    /// (`sleep_ms` + `nros_smoltcp::do_poll`) now that TIMER0 is armed and
    /// interrupts are unmasked. Without this, the zenoh connect spin races
    /// QEMU `-icount` virtual time ahead of wall-clock and host-timed slirp
    /// never delivers the handshake packets (#178 layer 2).
    fn on_interrupts_live() {
        #[cfg(feature = "ethernet")]
        crate::enable_wfi_idle();
    }
}

#[cfg(feature = "e2e-synthetic-callback")]
struct NoopResolver;

#[cfg(feature = "e2e-synthetic-callback")]
impl PublisherResolver for NoopResolver {
    fn publish_raw(&self, _entity_id: &str, _data: &[u8]) -> ::nros::NodeResult<()> {
        Ok(())
    }
}

#[cfg(feature = "e2e-synthetic-callback")]
static NOOP_RESOLVER: NoopResolver = NoopResolver;

#[cfg(feature = "e2e-synthetic-callback")]
static mut E2E_CTX: MaybeUninit<::nros::CallbackCtx<'static>> = MaybeUninit::uninit();

#[cfg(feature = "e2e-synthetic-callback")]
fn enqueue_e2e_callback(runtime: &mut RticRuntime) {
    // SAFETY: initialized once during RTIC init before tasks spawn; the storage
    // is static and lives for the firmware lifetime.
    let ctx: &'static mut ::nros::CallbackCtx<'static> = unsafe {
        let slot = core::ptr::addr_of_mut!(E2E_CTX);
        (*slot).write(::nros::CallbackCtx::new(&[], &NOOP_RESOLVER));
        (&mut *slot).assume_init_mut()
    };
    runtime.signal_callback(SignaledCallback {
        cb_id: "__nros_e2e",
        ctx_ptr: ctx as *mut ::nros::CallbackCtx<'static> as *mut core::ffi::c_void,
    });
}

pub mod prelude {
    pub use crate::{
        entry,
        rtic::{
            RticMps2An385, RticRuntime, SignaledCallbackEnvelope, take_dispatch_consumer,
            take_dispatch_queue,
        },
    };
}
