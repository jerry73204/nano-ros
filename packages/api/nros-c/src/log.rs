//! Phase 88.12 — C-API surface for the `nros_log` facade.
//!
//! Mirrors `<nros/log.h>`. The `nros_log_emit` /
//! `nros_log_emit_fmt` symbols dispatch through the same per-platform
//! sink chain the Rust call sites use (Phase 88.5 onwards).
//!
//! cbindgen is told to skip every item in this module — the
//! hand-written `<nros/log.h>` is authoritative for the C ABI
//! (cbindgen would re-emit the enum + functions under their
//! mangled names, colliding with the hand-written header).

use core::ffi::{c_char, c_void};

/// C severity mirror of `nros_log::Severity`. Discriminants match
/// `Severity::as_u8()`.
///
/// cbindgen:ignore
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum nros_log_severity_t {
    NROS_LOG_SEVERITY_TRACE = 0,
    NROS_LOG_SEVERITY_DEBUG = 1,
    NROS_LOG_SEVERITY_INFO = 2,
    NROS_LOG_SEVERITY_WARN = 3,
    NROS_LOG_SEVERITY_ERROR = 4,
    NROS_LOG_SEVERITY_FATAL = 5,
}

impl nros_log_severity_t {
    fn to_facade(self) -> nros_log::Severity {
        match self {
            Self::NROS_LOG_SEVERITY_TRACE => nros_log::Severity::Trace,
            Self::NROS_LOG_SEVERITY_DEBUG => nros_log::Severity::Debug,
            Self::NROS_LOG_SEVERITY_INFO => nros_log::Severity::Info,
            Self::NROS_LOG_SEVERITY_WARN => nros_log::Severity::Warn,
            Self::NROS_LOG_SEVERITY_ERROR => nros_log::Severity::Error,
            Self::NROS_LOG_SEVERITY_FATAL => nros_log::Severity::Fatal,
        }
    }
}

/// Low-level emit. `message` is UTF-8 text + explicit length; the
/// dispatcher hands it to whichever sink list was registered via
/// `nros_log::init`.
///
/// `logger` is the opaque handle from `nros_node_get_logger(...)`;
/// passing NULL drops the record silently.
///
/// Carries no call site — the record lands with `file`/`line` unknown. Kept at
/// this signature because it is the C ABI every existing consumer (and
/// `nros_log_emit_fmt`) already links; new call sites want
/// [`nros_log_emit_at`], which the `NROS_LOG_*` macros use.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_log_emit(
    logger: *const c_void,
    severity: nros_log_severity_t,
    message: *const c_char,
    message_len: usize,
) {
    nros_log_emit_at(logger, severity, message, message_len, core::ptr::null(), 0);
}

/// phase-417 W4.d — emit WITH the call site.
///
/// The record `nros_log_emit` built was hardcoded: `file: "<nros-c>", line: 0,
/// timestamp_ns: 0`, while `<nros/log.h>`'s own header comment promised the
/// macros "attach severity + line info". Every C and C++ record in the tree
/// therefore reached the sinks stripped of the two fields the sinks render.
///
/// `file` must be a pointer with STATIC storage duration — `__FILE__`, which is
/// what the macros pass. It is borrowed for the life of the program (a
/// [`nros_log::Record`] holds `&'static str`), never copied. NULL means
/// "unknown" and lands as `<nros-c>`, which is what the un-located
/// `nros_log_emit` above passes.
///
/// `timestamp_ns` is not a parameter: it comes from `nros_log`'s own
/// `__timestamp_ns()`, so C records are stamped by the same clock, at the same
/// resolution, under the same `platform-clock` feature as Rust ones. A second
/// C-side clock read would be a second answer to "when".
///
/// # Safety
/// * `logger` is NULL or a handle from `nros_log_get_logger` /
///   `nros_log_default_logger` / `nros_node_get_logger`.
/// * `message` is readable for `message_len` bytes.
/// * `file` is NULL or a NUL-terminated string with static storage duration.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_log_emit_at(
    logger: *const c_void,
    severity: nros_log_severity_t,
    message: *const c_char,
    message_len: usize,
    file: *const c_char,
    line: u32,
) {
    if logger.is_null() {
        return;
    }
    // Lazy-install the default sink list on first emit so C/C++
    // call sites work without an explicit `nros_log_init` step from
    // the user. Rust callers that want a custom sink list can still
    // call `nros_log::init(...)` before any record fires (the
    // install is idempotent — replacing the pointer is fine).
    ensure_default_sinks();
    let logger: &'static nros_log::Logger = &*(logger as *const nros_log::Logger);
    let sev = severity.to_facade();
    if !logger.is_enabled(sev) {
        return;
    }
    let msg_bytes: &[u8] = if message.is_null() || message_len == 0 {
        &[]
    } else {
        core::slice::from_raw_parts(message.cast::<u8>(), message_len)
    };
    let msg_str = core::str::from_utf8(msg_bytes).unwrap_or("<invalid utf-8>");
    let record = nros_log::Record {
        severity: sev,
        logger_name: logger.name(),
        message: msg_str,
        file: static_str(file).unwrap_or("<nros-c>"),
        line,
        timestamp_ns: nros_log::__timestamp_ns(),
    };
    logger.dispatch(&record);
}

/// Borrow a caller-owned NUL-terminated C string as a `&'static str`.
///
/// The `'static` is the CALLER's promise, stated in every doc comment that
/// reaches this: the only pointers we accept here are `__FILE__` and logger
/// names, and a record holds `&'static str` because a sink may outlive the
/// frame that raised it. `None` for NULL or non-UTF-8, never a silent
/// substitution.
///
/// # Safety
/// `ptr` is NULL or a NUL-terminated string that lives for the program.
unsafe fn static_str(ptr: *const c_char) -> Option<&'static str> {
    if ptr.is_null() {
        return None;
    }
    core::ffi::CStr::from_ptr(ptr).to_str().ok()
}

/// Borrow a caller-owned NUL-terminated C string for the duration of THIS call.
///
/// Separate from [`static_str`] on purpose: a logger NAME is looked up and
/// copied into `nros_log`'s arena, so it needs no lifetime beyond the call, and
/// giving it one would be a promise nobody made.
///
/// # Safety
/// `ptr` is NULL or a NUL-terminated string valid for this call.
unsafe fn borrowed_str<'a>(ptr: *const c_char) -> Option<&'a str> {
    if ptr.is_null() {
        return None;
    }
    core::ffi::CStr::from_ptr(ptr).to_str().ok()
}

use portable_atomic::{AtomicBool, Ordering};
static DEFAULT_SINKS_INSTALLED: AtomicBool = AtomicBool::new(false);

fn ensure_default_sinks() {
    if DEFAULT_SINKS_INSTALLED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        nros_log::init(nros_platform_cffi::log::default_sinks());
    }
}

/// Phase 88.16.H — explicit C-side install of the default sink list.
///
/// Pins `nros_log::init` as a linker root for cross-language consumers
/// (FreeRTOS / NuttX / ThreadX C and C++ examples). Without this,
/// `--gc-sections` can drop `nros_log::init` because it is only
/// reachable from the lazy `ensure_default_sinks` guard inside Rust,
/// and the C example never resolves a symbol that drags it in.
///
/// Idempotent: re-calling replaces the sink-list pointer with the
/// same default. Safe to call from any task / thread once the
/// platform-log fn-ptr slot is registered by the board crate.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_log_init() {
    DEFAULT_SINKS_INSTALLED.store(true, Ordering::Release);
    nros_log::init(nros_platform_cffi::log::default_sinks());
}

/// Phase 88.16.H — opaque handle to `nros_log::DEFAULT_LOGGER`.
///
/// Lets C callers emit records without standing up a full `Node`
/// (useful for boot-time diagnostics, panic hooks, smoke fixtures).
/// The returned pointer is `'static`. Never free.
#[unsafe(no_mangle)]
pub extern "C" fn nros_log_default_logger() -> *const c_void {
    (&raw const nros_log::DEFAULT_LOGGER).cast()
}

// -----------------------------------------------------------------------------
// phase-417 W4.d — named loggers, per-logger levels, sinks, throttle.
//
// RFC-0019: the Rust API is the implementation, C delegates. Everything below
// is a pointer/UTF-8 conversion around a `nros_log` call — no level table, no
// throttle arithmetic, no sink list lives here. The one thing that IS here is
// STORAGE (the `CSink` slots), because a C function pointer plus a `void *` is
// not a Rust type and `nros_log::add_sink` takes `&'static dyn LogSink`.
// -----------------------------------------------------------------------------

/// Look up a logger by name, CREATING one if the name is new.
///
/// `rclcpp::get_logger` / `rcutils`' shape. Before this the C surface had only
/// `nros_log_default_logger()`, so every C node shared one logger and one
/// threshold.
///
/// Total: always returns a usable handle. NULL/empty `name`, a name longer
/// than `nros_log::MAX_LOGGER_NAME_LEN`, or an exhausted arena answers the
/// catch-all logger AND reports it at WARN — the alternative is handing back
/// the catch-all silently, which makes `nros_logger_set_level` on the result
/// move the threshold of every other unnamed logger in the image.
///
/// The handle is `'static`. Never free it.
///
/// # Safety
/// `name` is NULL or a NUL-terminated UTF-8 string valid for this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_log_get_logger(name: *const c_char) -> *const c_void {
    let Some(name) = borrowed_str(name).filter(|n| !n.is_empty()) else {
        return nros_log_default_logger();
    };
    match nros_log::get_or_create_logger(name) {
        Some(logger) => (logger as *const nros_log::Logger).cast(),
        None => {
            let (used, total) = nros_log::dynamic_logger_name_arena();
            nros_log::nros_warn!(
                &nros_log::DEFAULT_LOGGER,
                "nros_log_get_logger(\"{name}\"): no logger created — {} of {} slots and {used} \
                 of {total} name bytes are spent, or the name is over {} bytes. Returning the \
                 catch-all logger, whose threshold is SHARED: raise `dynamic-loggers-<N>` on \
                 nros-log rather than calling nros_logger_set_level on this handle.",
                nros_log::dynamic_loggers_in_use(),
                nros_log::dynamic_logger_capacity(),
                nros_log::MAX_LOGGER_NAME_LEN,
            );
            nros_log_default_logger()
        }
    }
}

/// Copy the logger's name into `buf` as a NUL-terminated string.
///
/// Returns the name's length in bytes EXCLUDING the NUL, whether or not it
/// fit — `snprintf`'s convention, so a caller can size a buffer from one call.
/// Writes nothing when `buf` is NULL or `buf_len` is 0. Returns 0 for a NULL
/// logger.
///
/// A copy rather than a borrowed pointer because `nros_log::Logger` holds a
/// Rust `&str`, which is length-delimited and not NUL-terminated; handing out
/// its pointer would hand out a string every C function reads past the end of.
///
/// # Safety
/// `logger` is NULL or a handle from this module; `buf` is NULL or writable for
/// `buf_len` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_logger_get_name(
    logger: *const c_void,
    buf: *mut c_char,
    buf_len: usize,
) -> usize {
    let Some(logger) = logger_ref(logger) else {
        return 0;
    };
    let name = logger.name();
    if !buf.is_null() && buf_len > 0 {
        let copy = core::cmp::min(name.len(), buf_len - 1);
        core::ptr::copy_nonoverlapping(name.as_ptr(), buf.cast::<u8>(), copy);
        buf.add(copy).write(0);
    }
    name.len()
}

/// Set this logger's runtime severity threshold. Records below it are dropped
/// before any sink sees them.
///
/// Returns `false` for a NULL handle, `true` otherwise.
///
/// # Safety
/// `logger` is NULL or a handle from this module.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_logger_set_level(
    logger: *const c_void,
    severity: nros_log_severity_t,
) -> bool {
    match logger_ref(logger) {
        Some(logger) => {
            logger.set_level(severity.to_facade());
            true
        }
        None => false,
    }
}

/// This logger's runtime severity threshold.
///
/// A NULL handle answers `NROS_LOG_SEVERITY_FATAL` — the value at which almost
/// nothing is emitted — so a dropped handle reads as "quiet", not as "trace
/// everything".
///
/// # Safety
/// `logger` is NULL or a handle from this module.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_logger_get_level(logger: *const c_void) -> nros_log_severity_t {
    match logger_ref(logger) {
        Some(logger) => nros_log_severity_t::from_facade(logger.level()),
        None => nros_log_severity_t::NROS_LOG_SEVERITY_FATAL,
    }
}

/// Whether a record at `severity` would pass this logger's threshold.
///
/// The `NROS_LOG_*_THROTTLE` macros call this so the severity test runs BEFORE
/// the throttle window — a record the level filters must not consume the
/// window. NULL handle answers `false`.
///
/// # Safety
/// `logger` is NULL or a handle from this module.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_logger_is_enabled(
    logger: *const c_void,
    severity: nros_log_severity_t,
) -> bool {
    logger_ref(logger).is_some_and(|logger| logger.is_enabled(severity.to_facade()))
}

/// # Safety
/// `logger` is NULL or a handle from this module.
unsafe fn logger_ref(logger: *const c_void) -> Option<&'static nros_log::Logger> {
    if logger.is_null() {
        None
    } else {
        Some(&*(logger as *const nros_log::Logger))
    }
}

impl nros_log_severity_t {
    fn from_facade(severity: nros_log::Severity) -> Self {
        match severity {
            nros_log::Severity::Trace => Self::NROS_LOG_SEVERITY_TRACE,
            nros_log::Severity::Debug => Self::NROS_LOG_SEVERITY_DEBUG,
            nros_log::Severity::Info => Self::NROS_LOG_SEVERITY_INFO,
            nros_log::Severity::Warn => Self::NROS_LOG_SEVERITY_WARN,
            nros_log::Severity::Error => Self::NROS_LOG_SEVERITY_ERROR,
            nros_log::Severity::Fatal => Self::NROS_LOG_SEVERITY_FATAL,
        }
    }
}

// -----------------------------------------------------------------------------
// C sinks.
// -----------------------------------------------------------------------------

/// A C log sink. See `nros_log_add_sink`.
///
/// Deliberately NOT a `const nros_log_record_t *`: a struct crossing this FFI
/// would be a hand-mirrored one (cbindgen skips this module), and a mirror that
/// misses an appended field passes a SHORTER struct than the reader reads —
/// issue 0160's class, which has bitten three times. With flat arguments an
/// appended field changes the function-pointer TYPE, so every sink fails to
/// compile instead.
///
/// Strings are pointer + length and are NOT NUL-terminated.
///
/// cbindgen:ignore
pub type nros_log_sink_fn = unsafe extern "C" fn(
    user_data: *mut c_void,
    severity: nros_log_severity_t,
    logger_name: *const c_char,
    logger_name_len: usize,
    message: *const c_char,
    message_len: usize,
    file: *const c_char,
    file_len: usize,
    line: u32,
    timestamp_ns: u64,
);

/// One registered C sink. Storage only; the list, the ordering and the
/// dispatch are `nros_log`'s.
struct CSink {
    func: portable_atomic::AtomicPtr<()>,
    user: portable_atomic::AtomicPtr<c_void>,
}

impl CSink {
    const fn new() -> Self {
        Self {
            func: portable_atomic::AtomicPtr::new(core::ptr::null_mut()),
            user: portable_atomic::AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

impl nros_log::LogSink for CSink {
    fn log(&self, record: &nros_log::Record<'_>) {
        let raw = self.func.load(Ordering::Acquire);
        if raw.is_null() {
            return;
        }
        // SAFETY: `raw` is non-null only after `nros_log_add_sink` stored a
        // `nros_log_sink_fn` there, and the slot is never rewritten.
        let func: nros_log_sink_fn = unsafe { core::mem::transmute(raw) };
        let user = self.user.load(Ordering::Acquire);
        // SAFETY: the callback is the caller's, invoked with the pointers and
        // lengths its type declares; all four slices outlive the call.
        unsafe {
            func(
                user,
                nros_log_severity_t::from_facade(record.severity),
                record.logger_name.as_ptr().cast::<c_char>(),
                record.logger_name.len(),
                record.message.as_ptr().cast::<c_char>(),
                record.message.len(),
                record.file.as_ptr().cast::<c_char>(),
                record.file.len(),
                record.line,
                record.timestamp_ns,
            );
        }
    }
}

/// One slot per `nros_log::MAX_ADDED_SINKS`, so the C surface can fill the
/// registry and no further — the bound is `nros_log`'s, not a second one.
#[allow(clippy::declare_interior_mutable_const)]
const EMPTY_C_SINK: CSink = CSink::new();
static C_SINKS: [CSink; nros_log::MAX_ADDED_SINKS] = [EMPTY_C_SINK; nros_log::MAX_ADDED_SINKS];

/// Install a C callback as a log sink.
///
/// Before this the C surface could not install a sink at all: `nros_log_init()`
/// takes no arguments and hardcodes the platform default. `add_sink` APPENDS,
/// so a consumer teeing records to its own transport does not discard whatever
/// the board installed.
///
/// Returns `false` — and installs nothing — when `sink` is NULL or all
/// `nros_log::MAX_ADDED_SINKS` slots are taken. Check it; a sink that silently
/// was not installed is indistinguishable from one that never fires.
///
/// Records raised before the first sink existed are replayed into it, so a
/// sink installed at boot still sees the boot story.
///
/// `user_data` is passed back untouched and is never dereferenced here; it must
/// outlive the program, because a sink cannot be removed (the read path is
/// lock-free).
///
/// # Safety
/// `sink` must be callable for the life of the program, and `user_data` valid
/// for the same.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_log_add_sink(
    sink: Option<nros_log_sink_fn>,
    user_data: *mut c_void,
) -> bool {
    let Some(sink) = sink else {
        return false;
    };
    let raw = sink as *mut ();
    for slot in &C_SINKS {
        if slot
            .func
            .compare_exchange(
                core::ptr::null_mut(),
                raw,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
        {
            continue;
        }
        slot.user.store(user_data, Ordering::Release);
        // Reachable only from here on: `nros_log` owns the list.
        if nros_log::add_sink(slot) {
            return true;
        }
        // The facade refused (its own bound). Release the slot so a later call
        // is not charged for it.
        slot.func.store(core::ptr::null_mut(), Ordering::Release);
        return false;
    }
    false
}

// -----------------------------------------------------------------------------
// Throttle.
// -----------------------------------------------------------------------------

/// Decide whether a throttled call site may emit now, and arm its window.
///
/// `last_ns` is CALLER-OWNED storage — a `static uint64_t` the
/// `NROS_LOG_*_THROTTLE` macros declare, zero-initialised. Caller-owned rather
/// than a handle into a Rust-side pool because a pool needs a bound, and a
/// throttled call site is not a resource anyone should have to budget.
///
/// The RULE is `nros_log::throttle_admits` — the same function
/// `nros_log::ThrottleState` and the Rust `nros_*_throttle!` macros use. Only
/// the storage differs.
///
/// Returns `true` when the record should be emitted. NULL `last_ns` returns
/// `true`: a broken caller gets every record, never silence.
///
/// # Safety
/// `last_ns` is NULL or points to a `u64` that lives as long as the call site
/// (a function-scope `static`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_log_throttle_admit(last_ns: *mut u64, interval_ms: u64) -> bool {
    if last_ns.is_null() {
        return true;
    }
    warn_once_if_no_clock();
    let now = nros_log::__timestamp_ns();
    let last = last_ns.read();
    if nros_log::throttle_admits(last, now, nros_log::interval_ms_to_ns(interval_ms)) {
        // `max(1)`: `0` is the "never emitted" sentinel the facade uses, so a
        // clock that legitimately reads 0 must not re-arm the window forever.
        last_ns.write(if now == 0 { 1 } else { now });
        true
    } else {
        false
    }
}

static NO_CLOCK_REPORTED: AtomicBool = AtomicBool::new(false);

/// Say it once, out loud, rather than degrading quietly.
///
/// Without `nros-log/platform-clock` every timestamp is a constant `0`, so
/// `throttle_admits` sees an elapsed time that never grows and admits every
/// record. That is the safe direction to fail — a throttle that silences is
/// worse than one that does not throttle — but it is still not what the call
/// site says, and RFC-0089 puts the burden on us to say so.
fn warn_once_if_no_clock() {
    if nros_log::timestamp_available() {
        return;
    }
    if NO_CLOCK_REPORTED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        nros_log::nros_warn!(
            &nros_log::DEFAULT_LOGGER,
            "NROS_LOG_*_THROTTLE has no monotonic clock (nros-log's `platform-clock` feature is \
             off), so every timestamp is 0 and the window can never elapse: throttled sites will \
             emit EVERY record. Enable `nros-log/platform-clock` on the nros-c dependency."
        );
    }
}

/// Whether `Record::timestamp_ns` is a real clock reading rather than `0`.
///
/// Exposed so a C consumer can decide what a zero timestamp means instead of
/// guessing — and so the throttle's degradation above is inspectable.
#[unsafe(no_mangle)]
pub extern "C" fn nros_log_timestamp_available() -> bool {
    nros_log::timestamp_available()
}

// `nros_log_emit_fmt` is implemented in C
// (`packages/api/nros-c/c-stubs/log_fmt.c`) because the Rust
// `c_variadic` feature is still unstable on stable. The C shim
// vsnprintfs the format args + forwards to `nros_log_emit` above.
unsafe extern "C" {
    pub fn nros_log_emit_fmt(
        logger: *const c_void,
        severity: nros_log_severity_t,
        fmt: *const c_char,
        ...
    );
}

// Force the Rust symbol to land even when the linker greedily prunes.
#[used]
static _ANCHOR: unsafe extern "C" fn(*const c_void, nros_log_severity_t, *const c_char, usize) =
    nros_log_emit;

// Same for the phase-417 W4.d entry points. The `NROS_LOG_*` macros reach
// `nros_log_emit_at` and `nros_log_throttle_admit` from C, and
// `nros_log_get_logger` is the entry a C node starts from — none of them is
// referenced from Rust, so `--gc-sections` on a cross-language image is free to
// drop them. That is the Phase 88.16.H failure mode one family over.
#[used]
static _ANCHOR_EMIT_AT: unsafe extern "C" fn(
    *const c_void,
    nros_log_severity_t,
    *const c_char,
    usize,
    *const c_char,
    u32,
) = nros_log_emit_at;
#[used]
static _ANCHOR_GET_LOGGER: unsafe extern "C" fn(*const c_char) -> *const c_void =
    nros_log_get_logger;
#[used]
static _ANCHOR_THROTTLE: unsafe extern "C" fn(*mut u64, u64) -> bool = nros_log_throttle_admit;
