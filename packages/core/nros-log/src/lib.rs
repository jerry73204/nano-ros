//! Phase 88 — portable leveled-logging facade for nano-ros.
//!
//! See [`docs/roadmap/archived/phase-88-nros-log.md`](../../../docs/roadmap/archived/phase-88-nros-log.md)
//! for the design and acceptance criteria.
//!
//! ## Layering
//!
//! - This crate carries only the portable types + dispatcher +
//!   macros + `PlatformSink`. No backend code.
//! - Per-platform log delivery is the responsibility of each
//!   `nros-platform-<rtos>` crate, exposing
//!   `nros_platform_log_write` / `nros_platform_log_flush` via the
//!   `nros_platform_*` ABI (header at
//!   `packages/platform/nros-platform-api/include/nros/platform.h`).
//! - `PlatformSink` is the bridge: a single `LogSink` impl that
//!   forwards to the ABI. Apps that want fan-out (e.g.
//!   `Platform + /rosout`) compose a `&'static [&dyn LogSink]`
//!   manually and pass it to [`init`].
//!
//! ## Quick start
//!
//! ```ignore
//! use nros_log::{Logger, Severity};
//! use nros_log::{log_info, log_warn};
//!
//! static LOGGER: Logger = Logger::new("my_node");
//!
//! fn main() {
//!     nros_log::register_logger(&LOGGER);
//!     nros_log::init(nros_log::sinks::default());
//!     log_info!(&LOGGER, "started; domain = {}", 42);
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

#[cfg(feature = "alloc")]
extern crate alloc;

// Phase 88.16.E — portable-atomic polyfill for CAS-less targets
// (RISC-V `imc`, etc.). Feature unification: a consuming bare-metal
// crate enables `unsafe-assume-single-core` / `critical-section` on
// its own `portable-atomic` dep; native CAS targets get the
// passthrough.
use portable_atomic::{AtomicPtr, AtomicU8, AtomicUsize, Ordering};

pub mod early;
#[cfg(feature = "log-compat")]
pub mod log_compat;
pub mod macros;
pub mod pool;
pub mod sinks;
pub mod throttle;

mod buffer;

pub use buffer::{FormatBuffer, format_buffer_capacity};
pub use pool::{
    MAX_LOGGER_NAME_LEN, dynamic_logger_capacity, dynamic_logger_name_arena, dynamic_loggers_in_use,
};
pub use throttle::{ThrottleState, interval_ms_to_ns, throttle_admits};

/// REP-2012 severity levels, mirroring `rcutils_log_severity_t`.
///
/// The integer representation is stable and part of the ABI for
/// `nros_platform_log_write`. Lower value = more verbose.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Severity {
    /// Per-instruction granularity. Off unless `max-level-trace` is
    /// the active ceiling.
    Trace = 0,
    /// Diagnostic information useful while developing.
    Debug = 1,
    /// Normal operation events worth surfacing once.
    Info = 2,
    /// Unexpected but recoverable conditions.
    Warn = 3,
    /// Errors the caller should surface; the system continues.
    Error = 4,
    /// Unrecoverable — the system is about to abort.
    Fatal = 5,
}

impl Severity {
    /// Short uppercase label suitable for log-line rendering.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Trace => "TRACE",
            Self::Debug => "DEBUG",
            Self::Info => "INFO",
            Self::Warn => "WARN",
            Self::Error => "ERROR",
            Self::Fatal => "FATAL",
        }
    }

    /// Stable `u8` discriminant for cross-ABI use.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Reconstruct a [`Severity`] from its `u8` discriminant.
///
/// Returns `None` for `> 5`.
#[must_use]
pub const fn severity_from_u8(value: u8) -> Option<Severity> {
    match value {
        0 => Some(Severity::Trace),
        1 => Some(Severity::Debug),
        2 => Some(Severity::Info),
        3 => Some(Severity::Warn),
        4 => Some(Severity::Error),
        5 => Some(Severity::Fatal),
        _ => None,
    }
}

/// Compile-time ceiling check used by the `nros_*!` macros.
///
/// Returns `true` iff `severity` is allowed under the configured
/// `max-level-*` feature.
#[must_use]
pub const fn severity_enabled_at_compile_time(severity: Severity) -> bool {
    if cfg!(feature = "max-level-off") {
        return false;
    }
    let ceiling = compile_time_ceiling();
    (severity as u8) >= (ceiling as u8)
}

const fn compile_time_ceiling() -> Severity {
    if cfg!(feature = "max-level-trace") {
        Severity::Trace
    } else if cfg!(feature = "max-level-debug") {
        Severity::Debug
    } else if cfg!(feature = "max-level-info") {
        Severity::Info
    } else if cfg!(feature = "max-level-warn") {
        Severity::Warn
    } else if cfg!(feature = "max-level-error") {
        Severity::Error
    } else {
        // No ceiling feature = treat as `max-level-trace`.
        Severity::Trace
    }
}

/// One log entry, handed to each [`LogSink`].
///
/// `message` is already formatted — sinks must NOT re-format.
#[derive(Debug)]
pub struct Record<'a> {
    /// Severity of the record.
    pub severity: Severity,
    /// Name of the originating [`Logger`].
    pub logger_name: &'a str,
    /// Formatted message text (no trailing newline).
    pub message: &'a str,
    /// File the macro invocation came from (`core::file!()`).
    pub file: &'static str,
    /// Line within `file` (`core::line!()`).
    pub line: u32,
    /// Monotonic timestamp in nanoseconds. `0` if unavailable.
    pub timestamp_ns: u64,
}

/// Backend a log record is delivered to.
///
/// Implementations must be `Sync` so the dispatcher can hold them
/// in `&'static [&dyn LogSink]`. ISR-safety is per-impl — see the
/// table in `docs/roadmap/archived/phase-88-nros-log.md`.
pub trait LogSink: Sync {
    /// Render `record`. Called only when the record's severity passes
    /// both the compile-time ceiling AND the [`Logger`]'s runtime
    /// threshold.
    fn log(&self, record: &Record<'_>);

    /// Optional flush hook (default no-op).
    fn flush(&self) {}
}

/// A named logger with a runtime severity threshold.
///
/// Threshold defaults to [`Severity::Info`]. Use [`register_logger`]
/// to publish a `'static Logger` so multiple call sites with the
/// same name share the same threshold.
pub struct Logger {
    name: &'static str,
    level: AtomicU8,
}

impl Logger {
    /// `const`-construct with the default threshold ([`Severity::Info`]).
    #[must_use]
    pub const fn new(name: &'static str) -> Self {
        Self {
            name,
            level: AtomicU8::new(Severity::Info as u8),
        }
    }

    /// `const`-construct with an explicit threshold.
    #[must_use]
    pub const fn with_level(name: &'static str, level: Severity) -> Self {
        Self {
            name,
            level: AtomicU8::new(level as u8),
        }
    }

    /// Logger name (used as `Record::logger_name`).
    #[must_use]
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Current runtime threshold.
    #[must_use]
    pub fn level(&self) -> Severity {
        severity_from_u8(self.level.load(Ordering::Relaxed)).unwrap_or(Severity::Info)
    }

    /// Update the runtime threshold.
    pub fn set_level(&self, level: Severity) {
        self.level.store(level as u8, Ordering::Relaxed);
    }

    /// Whether a record at `severity` would be emitted by this
    /// logger AT RUNTIME.
    #[must_use]
    pub fn is_enabled(&self, severity: Severity) -> bool {
        (severity as u8) >= self.level.load(Ordering::Relaxed)
    }

    /// Whether a record at `severity` would be emitted, given both this
    /// logger's runtime threshold AND a per-call-site throttle window.
    ///
    /// phase-417 W4.d. The order is load-bearing and is the same order
    /// `rclcpp` uses: the SEVERITY test runs first, so a record the level
    /// already filters does not consume the throttle window. Reversing them
    /// makes a raised-then-lowered log level lose its first record for a whole
    /// interval, which is the shape of bug nobody reports because it looks
    /// like the throttle working.
    ///
    /// Consumes the window when it returns `true`, so call it exactly once per
    /// candidate record — the `nros_*_throttle!` macros do.
    ///
    /// `now_ns` is the caller's monotonic clock; the macros pass
    /// [`__timestamp_ns`]. Taking it as an argument rather than reading the
    /// clock here is what lets the whole rule be tested without a platform
    /// port, and is the same shape `rclcpp::RCLCPP_*_THROTTLE` uses when it
    /// asks for a `Clock`.
    #[must_use]
    pub fn is_enabled_throttled(
        &self,
        severity: Severity,
        state: &ThrottleState,
        now_ns: u64,
        interval_ns: u64,
    ) -> bool {
        self.is_enabled(severity) && state.should_log(now_ns, interval_ns)
    }

    /// Hand `record` to every registered sink, after the runtime
    /// threshold check.
    ///
    /// Macros call this; user code should not.
    pub fn dispatch(&self, record: &Record<'_>) {
        if !self.is_enabled(record.severity) {
            return;
        }
        dispatch_to_sinks(record);
    }
}

// -----------------------------------------------------------------------------
// Static intern table for `get_logger("name")`. Bounded; no alloc.
// -----------------------------------------------------------------------------

/// Maximum number of named loggers that can be registered via
/// [`register_logger`]. Beyond this, [`get_logger`] returns
/// [`DEFAULT_LOGGER`].
pub const MAX_LOGGERS: usize = 32;

/// Catch-all logger returned when the requested name is not
/// registered (or the intern table is full).
pub static DEFAULT_LOGGER: Logger = Logger::new("nros");

mod intern {
    use super::{AtomicPtr, Logger, MAX_LOGGERS, Ordering};

    pub(super) struct InternTable {
        slots: [AtomicPtr<Logger>; MAX_LOGGERS],
    }

    impl InternTable {
        pub(super) const fn new() -> Self {
            // `AtomicPtr::new` is `const` on both `core::sync::atomic`
            // and `portable_atomic`, so we can initialise the array
            // by repeating the call rather than naming a `const` —
            // which clippy flags as interior-mutable.
            #[allow(clippy::declare_interior_mutable_const)]
            const NULL: AtomicPtr<Logger> = AtomicPtr::new(core::ptr::null_mut());
            Self {
                slots: [NULL; MAX_LOGGERS],
            }
        }

        pub(super) fn lookup(&self, name: &str) -> Option<&'static Logger> {
            for slot in &self.slots {
                let ptr = slot.load(Ordering::Acquire);
                if ptr.is_null() {
                    return None;
                }
                // SAFETY: pointer published via Release after the
                // owner constructed a `'static Logger`. The Acquire
                // load synchronizes.
                let logger: &'static Logger = unsafe { &*ptr };
                if logger.name() == name {
                    return Some(logger);
                }
            }
            None
        }

        pub(super) fn insert(&self, logger: &'static Logger) -> Option<&'static Logger> {
            if let Some(existing) = self.lookup(logger.name()) {
                return Some(existing);
            }
            let ptr = logger as *const _ as *mut Logger;
            for slot in &self.slots {
                if slot
                    .compare_exchange(
                        core::ptr::null_mut(),
                        ptr,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok()
                {
                    return Some(logger);
                }
                let existing_ptr = slot.load(Ordering::Acquire);
                if !existing_ptr.is_null() {
                    // SAFETY: same publication invariant as `lookup`.
                    let existing: &'static Logger = unsafe { &*existing_ptr };
                    if existing.name() == logger.name() {
                        return Some(existing);
                    }
                }
            }
            None
        }
    }
}

static INTERN: intern::InternTable = intern::InternTable::new();

/// Publish `logger` under its name so subsequent `get_logger`
/// calls with that name return THIS reference.
///
/// On name collision returns the pre-existing entry (the input
/// `logger` is NOT inserted). On a full table returns
/// [`DEFAULT_LOGGER`].
pub fn register_logger(logger: &'static Logger) -> &'static Logger {
    INTERN.insert(logger).unwrap_or(&DEFAULT_LOGGER)
}

/// Look up a registered logger by name. Returns [`DEFAULT_LOGGER`]
/// if none is registered (call [`register_logger`] for a `'static
/// Logger` to publish one).
///
/// Total — every call returns a usable handle the macros can
/// dispatch through.
#[must_use]
pub fn get_logger(name: &str) -> &'static Logger {
    INTERN.lookup(name).unwrap_or(&DEFAULT_LOGGER)
}

/// Look up a logger by name, CREATING one if the name is new.
///
/// phase-417 W4.d. This is the shape `rclcpp::get_logger` / `rcutils`' logger
/// lookup have, and the one a wrapper needs: [`get_logger`] answers
/// [`DEFAULT_LOGGER`] for an unregistered name, so a C or C++ caller doing
/// `set_level(get_logger("nav"), Debug)` over the lookup-only form would have
/// moved the threshold of the catch-all logger that EVERY other unregistered
/// name also resolves to. Same call, same types, different effect — precisely
/// the compile-and-differ RFC-0089 forbids.
///
/// Storage is the bounded static arena in [`pool`], sized by the
/// `dynamic-loggers-<N>` feature. Returns `None` — never a logger under the
/// wrong name — when:
///
/// * `name` is empty or longer than [`MAX_LOGGER_NAME_LEN`],
/// * the name arena or the logger arena is full,
/// * the [`MAX_LOGGERS`] intern table is full.
///
/// Callers decide what an exhausted arena means for them; `nros-c` reports it
/// and falls back to [`DEFAULT_LOGGER`] loudly rather than silently.
///
/// Loggers created this way are never destroyed — the arena has no free list,
/// which is what makes the returned `&'static Logger` honest on `no_std`.
#[must_use]
pub fn get_or_create_logger(name: &str) -> Option<&'static Logger> {
    if let Some(existing) = INTERN.lookup(name) {
        return Some(existing);
    }
    let interned = pool::intern_name(name)?;
    let placed = pool::place(Logger::new(interned))?;
    // Not `register_logger`: that answers `DEFAULT_LOGGER` on a full table,
    // which would hand back a logger under the wrong name — the exact aliasing
    // this function exists to avoid. On a lost race `insert` returns the
    // winner and our slot is spent, which `pool::dynamic_loggers_in_use`
    // reports.
    INTERN.insert(placed)
}

// -----------------------------------------------------------------------------
// Sink list. Set once at `init`; read every dispatch.
// -----------------------------------------------------------------------------

static SINKS_PTR: AtomicPtr<&'static [&'static dyn LogSink]> =
    AtomicPtr::new(core::ptr::null_mut());

// issue 0710 — `init_default()` was here. It named a default this crate can no
// longer name: the platform sink moved to `nros_platform_cffi::log`, where the
// ABI it speaks is a dependency rather than a feature. Call
// `nros_platform_cffi::log::init_default()`, or `init()` with your own sinks.

/// Install the global sink list.
///
/// MUST be called at app startup BEFORE any record-emitting macro
/// runs (otherwise the dispatch is a no-op — records are silently
/// dropped). Calling `init` more than once swaps the list
/// atomically; the previous pointer is leaked (intentional: the
/// read path is lock-free so we can't safely free).
///
/// The sinks themselves must outlive the program (`'static`).
pub fn init(sinks: &'static [&'static dyn LogSink]) {
    // Indirect through a small `'static` cell so the read path
    // dereferences a fat-pointer-sized slot rather than reading
    // a wide pointer atomically.
    #[cfg(feature = "alloc")]
    {
        let boxed: alloc::boxed::Box<&'static [&'static dyn LogSink]> =
            alloc::boxed::Box::new(sinks);
        let ptr = alloc::boxed::Box::into_raw(boxed);
        SINKS_PTR.store(ptr, Ordering::Release);
    }
    #[cfg(not(feature = "alloc"))]
    {
        static CELL: SinkSlot = SinkSlot::new();
        CELL.store(sinks);
        SINKS_PTR.store(CELL.as_ptr(), Ordering::Release);
    }
    // AFTER publishing, so a record raised during the drain reaches `sinks`
    // directly rather than joining a ring nobody will drain again.
    early::drain(sinks);
    let lost = early::overflowed();
    if lost > 0 {
        // Reported through the sinks just installed, because the alternative is
        // a silent hole exactly where the boot story is (`nros-log` cannot know
        // what those records said, but it does know how many there were).
        let logger = Logger::new("nros_log");
        crate::log_warn!(
            &logger,
            "{lost} record(s) raised before `init` did not fit the early ring \
             (see `nros_log::early`; raise `early-records-<N>`)"
        );
    }
}

#[cfg(not(feature = "alloc"))]
struct SinkSlot {
    inner: core::cell::UnsafeCell<Option<&'static [&'static dyn LogSink]>>,
}

#[cfg(not(feature = "alloc"))]
// SAFETY: only written from `init`, which the user contracts to call
// once at startup before any concurrent reader exists.
unsafe impl Sync for SinkSlot {}

#[cfg(not(feature = "alloc"))]
impl SinkSlot {
    const fn new() -> Self {
        Self {
            inner: core::cell::UnsafeCell::new(None),
        }
    }
    fn store(&self, sinks: &'static [&'static dyn LogSink]) {
        // SAFETY: see Sync note above.
        unsafe {
            *self.inner.get() = Some(sinks);
        }
    }
    fn as_ptr(&self) -> *mut &'static [&'static dyn LogSink] {
        self.inner.get().cast()
    }
}

// -----------------------------------------------------------------------------
// Appendable sink registry (phase-417 W4.d).
//
// `init` REPLACES the list, which is right for the board — it names, once, the
// whole delivery it chose. It is the wrong verb for a consumer: C and C++ had
// no way to install a sink at all (`nros_log_init(void)` takes no arguments
// and hardcodes the platform default), and a library that wants to tee records
// to /rosout cannot call `init` without discarding whatever the board
// installed.
//
// So `add_sink` APPENDS, and dispatch walks both lists. The two are kept
// separate rather than merged into one growable list because they have
// different owners and different lifetimes-of-decision: `init` may be called
// again and swap the board's choice wholesale, and doing so must not silently
// unregister a consumer's sink.
// -----------------------------------------------------------------------------

/// How many sinks [`add_sink`] can append on top of the [`init`] list.
///
/// Small on purpose: the appendable list is for consumers that TEE (a /rosout
/// bridge, a test collector), not for composing a delivery — a board composing
/// its own delivery passes the whole slice to [`init`].
pub const MAX_ADDED_SINKS: usize = 4;

struct AddedSinks {
    slots: core::cell::UnsafeCell<[Option<&'static dyn LogSink>; MAX_ADDED_SINKS]>,
    /// Number of slots fully written. `Release` on publish, `Acquire` on read.
    len: AtomicUsize,
}

// SAFETY: writes happen only under `ADD_LOCK` and only to the slot at index
// `len`, which no reader looks at until the `Release` store to `len` that
// follows the write is observed by an `Acquire` load. A slot is written once
// and never rewritten.
unsafe impl Sync for AddedSinks {}

static ADDED: AddedSinks = AddedSinks {
    slots: core::cell::UnsafeCell::new([None; MAX_ADDED_SINKS]),
    len: AtomicUsize::new(0),
};

/// Serialises `add_sink` against itself.
///
/// A spin lock, held for two stores with no call inside it, is admissible here
/// where the in-order-publish alternative is not: that one has a thread
/// waiting on ANOTHER thread's progress, which on a single-core RTOS with
/// priorities is a deadlock rather than a delay. Do not call `add_sink` from
/// an ISR.
static ADD_LOCK: AtomicBool = AtomicBool::new(false);

/// Append `sink` to the sinks every record is delivered to.
///
/// Returns `false` when [`MAX_ADDED_SINKS`] are already registered — the sink
/// is NOT installed, and the caller is expected to say so rather than carry on
/// as if it were.
///
/// Records raised before any sink existed are replayed into `sink` here, the
/// same way [`init`] replays them, so the first sink to arrive still sees the
/// boot story regardless of which of the two calls installed it.
pub fn add_sink(sink: &'static dyn LogSink) -> bool {
    while ADD_LOCK
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Acquire)
        .is_err()
    {
        core::hint::spin_loop();
    }
    let idx = ADDED.len.load(Ordering::Relaxed);
    let installed = if idx < MAX_ADDED_SINKS {
        // SAFETY: `ADD_LOCK` makes this the only writer, `idx` is beyond what
        // any reader may look at until the `Release` store below publishes it,
        // and the slot is never written again.
        unsafe {
            ADDED
                .slots
                .get()
                .cast::<Option<&'static dyn LogSink>>()
                .add(idx)
                .write(Some(sink));
        }
        ADDED.len.store(idx + 1, Ordering::Release);
        true
    } else {
        false
    };
    ADD_LOCK.store(false, Ordering::Release);

    if installed && SINKS_PTR.load(Ordering::Acquire).is_null() && idx == 0 {
        // First delivery target in the program, and `init` has not run: the
        // held records are ours to replay. AFTER publishing, for the reason
        // `init` states — a record raised during the drain reaches the sink
        // directly rather than joining a ring nobody will drain again.
        early::drain_with(&mut |record| sink.log(record));
    }
    installed
}

/// How many sinks [`add_sink`] has installed.
#[must_use]
pub fn added_sink_count() -> usize {
    ADDED.len.load(Ordering::Acquire)
}

/// Hand `record` to every sink installed by [`add_sink`]. Returns how many
/// saw it.
fn deliver_to_added(record: &Record<'_>) -> usize {
    let len = ADDED.len.load(Ordering::Acquire);
    let base = ADDED.slots.get().cast::<Option<&'static dyn LogSink>>();
    for idx in 0..len {
        // SAFETY: `idx < len`, and the `Acquire` load above pairs with the
        // `Release` store in `add_sink` that published slot `idx`. Slots below
        // `len` are immutable for the rest of the program.
        if let Some(sink) = unsafe { base.add(idx).read() } {
            sink.log(record);
        }
    }
    len
}

/// Whether [`__timestamp_ns`] can answer with a real clock.
///
/// `false` means every `Record::timestamp_ns` is `0` and anything DERIVED from
/// the timestamp — the throttle windows above all of them — has no time base.
/// Exposed so a wrapper can say so out loud instead of degrading quietly:
/// without this, a throttled C call site looks identical whether it is
/// rate-limiting or not.
#[must_use]
pub const fn timestamp_available() -> bool {
    cfg!(feature = "platform-clock")
}

/// Current monotonic time for `Record::timestamp_ns` (issue #503).
///
/// With the `platform-clock` feature this reads
/// `nros_platform_clock_ns` — the universal per-platform export the
/// executor's timer accounting already links — scaled to nanoseconds.
/// Without the feature it returns `0` ("unavailable"), the historical
/// behavior, and imposes no link-time requirement.
///
/// Public because the emission macros expand it in user crates; not
/// part of the supported API surface.
#[doc(hidden)]
#[must_use]
pub fn __timestamp_ns() -> u64 {
    #[cfg(feature = "platform-clock")]
    {
        unsafe extern "C" {
            fn nros_platform_clock_ns() -> u64;
        }
        // SAFETY: bare query of the platform's monotonic us counter;
        // the symbol comes from whichever `nros-platform-<rtos>` port
        // linked the binary (the contract `PlatformSink ->
        // nros_platform_log_write` already relies on).
        unsafe { nros_platform_clock_ns() }
    }
    #[cfg(not(feature = "platform-clock"))]
    {
        0
    }
}

fn dispatch_to_sinks(record: &Record<'_>) {
    if recursion_guard_check_and_set() {
        return;
    }
    // phase-417 W4.d — the appendable list is walked whether or not `init`
    // ran, and a record with at least one destination is NOT held.
    let added = deliver_to_added(record);
    let ptr = SINKS_PTR.load(Ordering::Acquire);
    if ptr.is_null() {
        if added > 0 {
            recursion_guard_clear();
            return;
        }
        // Nothing installed yet — HOLD the record; `init` replays it into
        // whatever sinks the board chooses. See `early` for why this crate
        // does not reach for the platform sink itself: doing that (issue 0710)
        // put `nros_platform_log_write` on a path every binary executes, which
        // turned a pluggable delivery into a link-time requirement that no
        // Cargo feature can undo under workspace feature unification.
        //
        // It is also a better answer to issue 0708 than installing a default
        // was: the early records land in the sink the board picked, not in the
        // one dispatch guessed before the board had spoken.
        early::hold(record);
        recursion_guard_clear();
        return;
    }
    {
        // SAFETY: `init` published a valid `'static` slice reference.
        let sinks: &'static [&'static dyn LogSink] = unsafe { *ptr };
        for sink in sinks {
            sink.log(record);
        }
    }
    recursion_guard_clear();
}

/// Flush every registered sink, appended ones included.
pub fn flush() {
    let len = ADDED.len.load(Ordering::Acquire);
    let base = ADDED.slots.get().cast::<Option<&'static dyn LogSink>>();
    for idx in 0..len {
        // SAFETY: same publication invariant as `deliver_to_added`.
        if let Some(sink) = unsafe { base.add(idx).read() } {
            sink.flush();
        }
    }
    let ptr = SINKS_PTR.load(Ordering::Acquire);
    if ptr.is_null() {
        return;
    }
    // SAFETY: same invariant as `dispatch_to_sinks`.
    let sinks: &'static [&'static dyn LogSink] = unsafe { *ptr };
    for sink in sinks {
        sink.flush();
    }
}

// -----------------------------------------------------------------------------
// Recursion guard — process-global single AtomicBool.
//
// Granularity is intentionally coarse (process-wide, not per-thread).
// The guard exists to break a sink that triggers log() during write
// — not to serialize concurrent loggers across threads. A thread
// re-entering through its own sink loses its other in-flight
// sinks for that call; a different thread logging concurrently is
// also short-circuited momentarily. This is acceptable: the alt is
// per-thread storage which doesn't exist uniformly across our
// `no_std` targets (`thread_local!` requires `std`).
// -----------------------------------------------------------------------------

use portable_atomic::AtomicBool;
static RECURSION_GUARD: AtomicBool = AtomicBool::new(false);

fn recursion_guard_check_and_set() -> bool {
    RECURSION_GUARD
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Acquire)
        .is_err()
}

fn recursion_guard_clear() {
    RECURSION_GUARD.store(false, Ordering::Release);
}

// -----------------------------------------------------------------------------
// Tests (host-only).
// -----------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn severity_round_trips_through_u8() {
        for s in [
            Severity::Trace,
            Severity::Debug,
            Severity::Info,
            Severity::Warn,
            Severity::Error,
            Severity::Fatal,
        ] {
            assert_eq!(severity_from_u8(s.as_u8()), Some(s));
        }
        assert_eq!(severity_from_u8(99), None);
    }

    #[test]
    fn logger_runtime_threshold_filters_below() {
        let logger = Logger::with_level("test_thresh", Severity::Warn);
        assert!(!logger.is_enabled(Severity::Info));
        assert!(logger.is_enabled(Severity::Warn));
        assert!(logger.is_enabled(Severity::Error));
        logger.set_level(Severity::Debug);
        assert!(logger.is_enabled(Severity::Info));
    }

    #[test]
    fn unregistered_get_logger_returns_default() {
        let l = get_logger("definitely-not-registered-99");
        assert_eq!(l.name(), DEFAULT_LOGGER.name());
    }

    #[test]
    fn registered_logger_round_trips_through_intern_table() {
        static LOGGER: Logger = Logger::new("test_intern_round_trip");
        let published = register_logger(&LOGGER);
        assert_eq!(published.name(), LOGGER.name());
        let looked_up = get_logger("test_intern_round_trip");
        assert!(core::ptr::eq(published, looked_up));
    }

    #[test]
    fn compile_time_ceiling_matches_enabled_feature() {
        let expected = if cfg!(feature = "max-level-off") {
            None
        } else if cfg!(feature = "max-level-trace") {
            Some(Severity::Trace)
        } else if cfg!(feature = "max-level-debug") {
            Some(Severity::Debug)
        } else if cfg!(feature = "max-level-info") {
            Some(Severity::Info)
        } else if cfg!(feature = "max-level-warn") {
            Some(Severity::Warn)
        } else if cfg!(feature = "max-level-error") {
            Some(Severity::Error)
        } else {
            // No ceiling feature = treat as `max-level-trace`.
            Some(Severity::Trace)
        };

        for severity in [
            Severity::Trace,
            Severity::Debug,
            Severity::Info,
            Severity::Warn,
            Severity::Error,
            Severity::Fatal,
        ] {
            let enabled = expected.is_some_and(|ceiling| severity >= ceiling);
            assert_eq!(severity_enabled_at_compile_time(severity), enabled);
        }
    }
}
