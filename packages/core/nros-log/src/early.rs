//! Records raised before any sink was installed.
//!
//! ## The problem this exists for
//!
//! [`crate::init`] publishes the sink list. A record raised before that call
//! has nowhere to go, and for most of this crate's life it was constructed,
//! dispatched and DROPPED — silently, and invisibly to its author, who cannot
//! know what the board did before reaching their code.
//!
//! Issue 0708 answered by requiring every board boot funnel to call
//! `init_default()`, gated on funnels spelled `pub fn run*`. That is a SEARCH
//! for boot paths and it kept losing: NuttX's funnel is `pub extern "C" fn
//! nsh_main`, and three board crates did not link `nros-log` in the
//! configuration holding the funnel at all.
//!
//! Issue 0710 answered by having dispatch install the platform sink itself.
//! That removed the search — but it put `nros_platform_log_write` on a path
//! EVERY binary executes, which turned a pluggable delivery into a LINK-TIME
//! requirement. Seven test targets in `nros-rmw-cffi` and most of `nros-tests`
//! stopped linking under `check-workspace-features`, and no Cargo feature can
//! fix that: `nros-platform-cffi` and `nros-rmw-bridge` enable
//! `nros-node/rmw-cffi` unconditionally, so feature unification turns any
//! forwarded gate back ON for every member of a workspace build. A feature is a
//! property of the BUILD; what the question needs is a property of the BINARY.
//!
//! ## What this does instead
//!
//! Hold the records. A record raised with no sinks installed is copied into a
//! bounded static ring here; [`crate::init`] drains it into whatever sinks the
//! board actually chose. Nothing is dropped, no board can forget, and this
//! crate touches no platform symbol to do it — the facade stays a facade.
//!
//! It is also STRICTLY better than installing a default sink was: the early
//! records land in the sink the board picked, rather than in whichever one
//! dispatch guessed before the board had spoken.
//!
//! ## Cost, and how to decline it
//!
//! `EARLY_DEPTH * (format buffer + name + header)` of static RAM, all of it in
//! `.bss`. The depth is chosen by the `early-records-<N>` feature family the
//! same way `buffer-size-<N>` picks the format buffer, and for the same
//! reason — a 64 KB MCU and a Linux host do not want the same number. `0`
//! declines the buffer entirely and restores the pre-0708 behaviour of
//! dropping, with the count below still kept so the loss is at least
//! reportable.
//!
//! Overflow is counted, never silently absorbed: [`overflowed`] returns how
//! many records did not fit, and `init` reports it through the freshly
//! installed sinks.

use core::cell::UnsafeCell;

use portable_atomic::{AtomicUsize, Ordering};

use crate::{LogSink, Record, Severity, buffer::format_buffer_capacity};

/// Records held before `init`. See the module docs for the trade.
#[must_use]
pub const fn early_depth() -> usize {
    if cfg!(feature = "early-records-0") {
        0
    } else if cfg!(feature = "early-records-16") {
        16
    } else if cfg!(feature = "early-records-8") {
        8
    } else {
        4
    }
}

const DEPTH: usize = early_depth();
const MSG_CAP: usize = format_buffer_capacity();
/// A logger name is an identifier, not prose — `nros-node`'s longest is 20.
const NAME_CAP: usize = 48;

struct Pending {
    severity: Severity,
    logger_name: heapless::String<NAME_CAP>,
    message: heapless::String<MSG_CAP>,
    file: &'static str,
    line: u32,
    timestamp_ns: u64,
}

impl Pending {
    const fn new() -> Self {
        Self {
            severity: Severity::Info,
            logger_name: heapless::String::new(),
            message: heapless::String::new(),
            file: "",
            line: 0,
            timestamp_ns: 0,
        }
    }
}

struct Slot {
    /// Written by exactly one claimant; see `CLAIMED`.
    cell: UnsafeCell<Pending>,
    /// Publishes `cell` to the drain. `Release` here, `Acquire` there.
    ready: AtomicUsize,
}

// SAFETY: `cell` is written only by the thread that won a distinct index from
// `CLAIMED`'s `fetch_add` (each index is handed out once), and read only after
// that thread's `Release` store to `ready` is observed by an `Acquire` load.
unsafe impl Sync for Slot {}

impl Slot {
    const fn new() -> Self {
        Self {
            cell: UnsafeCell::new(Pending::new()),
            ready: AtomicUsize::new(0),
        }
    }
}

#[allow(clippy::declare_interior_mutable_const)]
const EMPTY_SLOT: Slot = Slot::new();
static SLOTS: [Slot; DEPTH] = [EMPTY_SLOT; DEPTH];

/// Total records offered while no sinks were installed. Indices `>= DEPTH`
/// did not fit; the count of those is [`overflowed`].
static CLAIMED: AtomicUsize = AtomicUsize::new(0);

/// How many early records did not fit and were lost.
#[must_use]
pub fn overflowed() -> usize {
    CLAIMED.load(Ordering::Relaxed).saturating_sub(DEPTH)
}

/// Hold `record` until a sink list is installed.
///
/// Returns `false` when it did not fit — the caller has nothing further to do,
/// but [`overflowed`] will report it.
pub(crate) fn hold(record: &Record<'_>) -> bool {
    let idx = CLAIMED.fetch_add(1, Ordering::AcqRel);
    if idx >= DEPTH {
        return false;
    }
    let slot = &SLOTS[idx];
    // SAFETY: `idx` was handed out exactly once by the `fetch_add` above, so
    // this thread is the only writer of `slot.cell`, and no reader may touch it
    // until the `Release` store below.
    let pending = unsafe { &mut *slot.cell.get() };
    pending.severity = record.severity;
    // Truncating rather than refusing: a clipped early record is worth more
    // than none, and the alternative is deciding at boot that a long logger
    // name loses the whole line.
    let _ = pending
        .logger_name
        .push_str(clip(record.logger_name, NAME_CAP));
    let _ = pending.message.push_str(clip(record.message, MSG_CAP));
    pending.file = record.file;
    pending.line = record.line;
    pending.timestamp_ns = record.timestamp_ns;
    slot.ready.store(1, Ordering::Release);
    true
}

/// Longest prefix of `s` that fits `cap` bytes without splitting a character.
fn clip(s: &str, cap: usize) -> &str {
    if s.len() <= cap {
        return s;
    }
    let mut end = cap;
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    &s[..end]
}

/// Replay everything held, in the order it was raised, into `sinks`.
///
/// Called by [`crate::init`] AFTER the sink list is published, so a record
/// raised concurrently with the drain reaches the sinks directly rather than
/// the ring. That can interleave a live record with a replayed one; ordering
/// among the replayed records themselves is preserved, which is the property
/// worth having.
pub(crate) fn drain(sinks: &'static [&'static dyn LogSink]) {
    drain_with(&mut |record| {
        for sink in sinks {
            sink.log(record);
        }
    });
}

/// The half of [`drain`] that does not name a sink LIST.
///
/// phase-417 W4.d added [`crate::add_sink`], so "where a held record goes" is
/// no longer answered by one `&'static [&dyn LogSink]`. The replay walk stayed
/// one function and grew a callback rather than being copied next to the new
/// registry — a second copy is a second answer to "in what order, and what
/// happens to a slot whose writer has not published yet".
pub(crate) fn drain_with(deliver: &mut dyn FnMut(&Record<'_>)) {
    let claimed = CLAIMED.load(Ordering::Acquire);
    let held = if claimed > DEPTH { DEPTH } else { claimed };
    for slot in SLOTS.iter().take(held) {
        if slot.ready.swap(0, Ordering::AcqRel) == 0 {
            // Either already drained by a concurrent `init`, or its writer has
            // not published yet. Neither is worth spinning at boot for.
            continue;
        }
        // SAFETY: the `Acquire` half of the swap above pairs with the writer's
        // `Release` store, and the swap makes this the only reader.
        let pending = unsafe { &*slot.cell.get() };
        let record = Record {
            severity: pending.severity,
            logger_name: pending.logger_name.as_str(),
            message: pending.message.as_str(),
            file: pending.file,
            line: pending.line,
            timestamp_ns: pending.timestamp_ns,
        };
        deliver(&record);
    }
    CLAIMED.store(0, Ordering::Release);
}
