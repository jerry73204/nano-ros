//! RFC-0052 / phase-296 W3b.4/.5 — on-target contract monitors.
//!
//! The baked shape mirrors Phase 211.H's `qos_overrides`: codegen emits a
//! `&'static [MonitorSpec]` table (plus one `static PubMonitorCell` per
//! contracted publisher) from the SystemModel's contract layer; the entry
//! installs it on the executor before entity creation. An uncontracted
//! image bakes an empty table — every path below dead-code-eliminates.
//!
//! Publish counting is an atomic bump on the publisher handle (no clock,
//! no lock on the hot path); the rate check runs on spin ticks over a
//! ~[`RATE_CHECK_INTERVAL_US`] window and pushes violations into a small
//! ring the entry glue drains into the `nros-diagnostics` reporter.
//!
//! W3b.5 adds three more rules on the same drain:
//! - `max-age-runtime` — subscriber take-age (`epoch_now - header.stamp`
//!   peeked from the raw CDR buffer at [`RosMessage::STAMP_OFFSET`],
//!   recorded into a [`SubMonitorCell`] on the take path).
//! - `max-latency-runtime` — node-path (take → publish) latency: the
//!   dispatch elapsed time is attributed to every monitored publisher
//!   whose counter advanced during that dispatch (an upper bound on
//!   take → publish, measured on the executor's monotonic clock).
//! - `deadline-miss-runtime` — a dispatched callback ran past its bound
//!   SchedContext's `deadline_us`; what ELSE happens is the tier's
//!   [`DeadlineAction`](super::sched_context::DeadlineAction).

use core::sync::atomic::Ordering;
// portable-atomic: RMW ops (fetch_add/fetch_max/swap) exist even on
// riscv32imc / Cortex-M0+ that lack native CAS (same choice as
// `SporadicState` / `AtomicSporadicState` in sched_context.rs).
use portable_atomic::AtomicU32;

/// One contracted publisher's counters. Baked as a `static` by codegen
/// (or declared by the fixture); the publisher handle bumps `count` on
/// every publish, the executor reads deltas on spin ticks.
#[derive(Debug, Default)]
pub struct PubMonitorCell {
    pub count: AtomicU32,
    /// W3b.5 — max observed take→publish latency (µs) in the current
    /// check window. Written by the dispatch loop (fetch_max), drained
    /// (swap 0) by the latency check.
    pub max_latency_us: AtomicU32,
    /// Age of the stamp this publisher last put ON THE WIRE, in
    /// microseconds: `epoch_now - outgoing header.stamp`.
    ///
    /// Distinct from `max_latency_us`, which times this node's own
    /// take→publish work. This says how old the DATA is that the node just
    /// published, which is the quantity a chain is made of.
    ///
    /// It exists to answer a question `max-age-runtime` cannot. That rule
    /// measures `epoch_now - stamp` on the TAKE path, so if every node in a
    /// chain propagates the original stamp -- the usual ROS convention, each
    /// node copying its input's stamp to its output -- the age at the final
    /// consumer already IS the end-to-end latency. If any node re-stamps
    /// with `now`, the clock silently resets and the same number becomes
    /// single-hop age instead. Same units, same magnitude, no warning.
    ///
    /// A publish age near zero on a node that consumes input is the
    /// signature of re-stamping. Recording it here is what lets a chain's
    /// provenance be checked at all, rather than assumed.
    ///
    /// `0` = never observed, matching the other cells: the type has no
    /// `STAMP_OFFSET`, or no epoch source is installed.
    pub last_publish_stamp_age_us: AtomicU32,
}

impl PubMonitorCell {
    pub const fn new() -> Self {
        Self {
            count: AtomicU32::new(0),
            max_latency_us: AtomicU32::new(0),
            last_publish_stamp_age_us: AtomicU32::new(0),
        }
    }
}

/// One contracted subscriber's take-age accumulator (W3b.5). The take
/// path records `epoch_now - header.stamp` per message (fetch_max); the
/// age check drains it (swap 0) per window.
#[derive(Debug, Default)]
pub struct SubMonitorCell {
    /// Max observed take-age (ms) in the current check window.
    pub max_age_ms: AtomicU32,
}

impl SubMonitorCell {
    pub const fn new() -> Self {
        Self {
            max_age_ms: AtomicU32::new(0),
        }
    }

    /// Take-path hook: record one message's age. `stamp_us` is the
    /// peeked `header.stamp` as µs since the UNIX epoch, `epoch_now_us`
    /// the receive-side wall clock. A stamp from the future clamps to 0.
    pub fn observe(&self, stamp_us: u64, epoch_now_us: u64) {
        let age_ms = (epoch_now_us.saturating_sub(stamp_us) / 1_000).min(u32::MAX as u64) as u32;
        self.max_age_ms.fetch_max(age_ms, Ordering::Relaxed);
    }
}

/// Peek `Time { i32 sec; u32 nanosec }` little-endian at `offset` in a
/// raw CDR receive buffer (encapsulation header included) and return µs
/// since the UNIX epoch. `None` when the buffer is too short or the
/// stamp is pre-epoch/zero (unstamped messages never fire age monitors).
/// Record the age of the stamp a publisher just put on the wire.
///
/// Called from the publish path with the encoded CDR still in hand, using the
/// same `STAMP_OFFSET` peek the take path uses. A no-op when the type carries
/// no stamp, when no epoch source is installed, or when the publisher is
/// uncontracted -- the same three ways `observe_age` folds away.
///
/// Stores rather than accumulates: this is "how old was the last thing
/// published", a state, not a window maximum. A chain check wants the current
/// value, and a max would be pinned forever by one stale message at startup.
#[inline]
pub fn observe_publish_stamp(cell: &PubMonitorCell, raw: &[u8], offset: usize, now_us: u64) {
    if let Some(stamp_us) = peek_stamp_us(raw, offset) {
        let age = now_us.saturating_sub(stamp_us).min(u32::MAX as u64) as u32;
        cell.last_publish_stamp_age_us.store(age, Ordering::Relaxed);
    }
}

pub fn peek_stamp_us(raw: &[u8], offset: usize) -> Option<u64> {
    let sec_b = raw.get(offset..offset + 4)?;
    let nsec_b = raw.get(offset + 4..offset + 8)?;
    let sec = i32::from_le_bytes([sec_b[0], sec_b[1], sec_b[2], sec_b[3]]);
    let nsec = u32::from_le_bytes([nsec_b[0], nsec_b[1], nsec_b[2], nsec_b[3]]);
    if sec <= 0 {
        return None;
    }
    Some(sec as u64 * 1_000_000 + nsec as u64 / 1_000)
}

/// One monitored publisher endpoint.
#[derive(Debug, Clone, Copy)]
pub struct MonitorSpec {
    /// Topic name EXACTLY as the node passes it to `create_publisher`
    /// (the SystemModel's wiring carries the same resolved name).
    pub topic: &'static str,
    /// Endpoint ref for violation reports (`<node FQN>/<endpoint>` — the
    /// SystemModel contract key).
    pub fqn: &'static str,
    /// Declared publisher guarantee, in milli-Hz (fixed point: Hz × 1000).
    /// 0 = no rate contract on this endpoint.
    pub min_rate_hz_milli: u32,
    /// W3b.5 — node-path budget (ms) for paths whose OUTPUT is this
    /// endpoint (`contracts.node_paths[..].max_latency_ms`). 0 = no
    /// latency contract.
    pub max_latency_ms: u32,
    /// The endpoint's counter cell.
    pub cell: &'static PubMonitorCell,
}

/// One monitored subscriber endpoint (W3b.5 age contracts). Separate
/// table from [`MonitorSpec`] — sub contracts key different endpoints
/// and need no publish counter.
#[derive(Debug, Clone, Copy)]
pub struct AgeMonitorSpec {
    /// Topic name EXACTLY as the node passes it to `create_subscription`.
    pub topic: &'static str,
    /// Endpoint ref for violation reports (the SystemModel contract key).
    pub fqn: &'static str,
    /// Declared max take-age (ms). 0 = no age contract.
    pub max_age_ms: u32,
    /// The endpoint's age accumulator.
    pub cell: &'static SubMonitorCell,
}

/// Rate-check window (µs). Matches play_launch's ~5 s time-based trigger
/// so both runtimes converge on comparable cadence.
pub const RATE_CHECK_INTERVAL_US: u64 = 5_000_000;

/// Max monitored endpoints per executor (const table, no_std).
pub const MAX_MONITORS: usize = 8;
/// Violation ring depth.
pub const MAX_VIOLATIONS: usize = 8;

/// A detected contract violation, in the play_launch rule-id vocabulary.
#[derive(Debug, Clone)]
pub struct Violation {
    /// `"rate-hierarchy-runtime"` | `"max-age-runtime"` |
    /// `"max-latency-runtime"` | `"deadline-miss-runtime"` |
    /// `"timer-overrun-runtime"` | `"release-jitter-runtime"` |
    /// `"stack-headroom-runtime"`.
    pub rule: &'static str,
    /// Violating endpoint ref (from the spec's `fqn`; the SC name for
    /// deadline misses).
    pub fqn: &'static str,
    /// Measured value. Unit is per-rule: milli-Hz for the rate rule, ms
    /// for age/latency, µs for deadline misses, dropped activations for
    /// the timer-overrun rule.
    pub measured: u32,
    /// Declared bound, same unit as `measured`.
    pub declared: u32,
}

/// Per-spec accounting state (parallel to the spec table).
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct MonitorState {
    /// Window opened (a plain bool, not a 0-sentinel on the timestamp —
    /// `now_us == 0` is a legitimate first sample on freshly-started
    /// monotonic clocks).
    pub(crate) opened: bool,
    pub(crate) window_start_us: u64,
    pub(crate) count_at_window_start: u32,
    /// Suppress duplicate reports: only re-report after a clean window.
    pub(crate) violated_last_window: bool,
    /// W3b.5 — separate dedup for the latency rule on the same spec row.
    pub(crate) latency_violated_last_window: bool,
}

/// Pure rate check over one window boundary. Returns `Some(violation)`
/// when the window elapsed AND the measured rate is below the declared
/// minimum (and we didn't already report last window).
///
/// Extracted from the executor so the math is unit-testable without a
/// session: publish counting is injected via the cell, time via `now_us`.
pub(crate) fn check_rate(
    spec: &MonitorSpec,
    state: &mut MonitorState,
    now_us: u64,
) -> Option<Violation> {
    if spec.min_rate_hz_milli == 0 {
        return None;
    }
    let count = spec.cell.count.load(Ordering::Relaxed);
    if !state.opened {
        // First observation: open the window, no verdict yet.
        state.opened = true;
        state.window_start_us = now_us;
        state.count_at_window_start = count;
        return None;
    }
    let window_us = now_us.saturating_sub(state.window_start_us);
    if window_us < RATE_CHECK_INTERVAL_US {
        return None;
    }
    let published = count.wrapping_sub(state.count_at_window_start) as u64;
    // milli-Hz = published * 1e3 / window_s = published * 1e9 / window_us
    let measured_milli_hz =
        (published.saturating_mul(1_000_000_000) / window_us.max(1)).min(u32::MAX as u64) as u32;

    // Roll the window.
    state.window_start_us = now_us;
    state.count_at_window_start = count;

    if measured_milli_hz < spec.min_rate_hz_milli {
        if state.violated_last_window {
            return None; // still violated — already reported
        }
        state.violated_last_window = true;
        Some(Violation {
            rule: "rate-hierarchy-runtime",
            fqn: spec.fqn,
            measured: measured_milli_hz,
            declared: spec.min_rate_hz_milli,
        })
    } else {
        state.violated_last_window = false;
        None
    }
}

/// Pure latency check: drains the spec cell's window-max take→publish
/// latency and fires when it exceeds the declared node-path budget.
/// Same report-once-until-recovery semantics as the rate rule; runs on
/// every monitor tick (the cell accumulates between ticks, so no window
/// bookkeeping is needed — draining IS the window roll).
pub(crate) fn check_latency(spec: &MonitorSpec, state: &mut MonitorState) -> Option<Violation> {
    if spec.max_latency_ms == 0 {
        return None;
    }
    let max_us = spec.cell.max_latency_us.swap(0, Ordering::Relaxed);
    let max_ms = max_us / 1_000;
    if max_ms > spec.max_latency_ms {
        if state.latency_violated_last_window {
            return None;
        }
        state.latency_violated_last_window = true;
        Some(Violation {
            rule: "max-latency-runtime",
            fqn: spec.fqn,
            measured: max_ms,
            declared: spec.max_latency_ms,
        })
    } else {
        // A quiet window (no dispatch attributed) also counts as clean —
        // recovery resets the dedup like the rate rule's clean window.
        state.latency_violated_last_window = false;
        None
    }
}

/// Per-age-spec dedup state.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct AgeState {
    pub(crate) violated_last_window: bool,
}

/// Pure age check: drains the sub cell's window-max take-age and fires
/// when it exceeds the declared bound. Report-once-until-recovery.
pub(crate) fn check_age(spec: &AgeMonitorSpec, state: &mut AgeState) -> Option<Violation> {
    if spec.max_age_ms == 0 {
        return None;
    }
    let max_ms = spec.cell.max_age_ms.swap(0, Ordering::Relaxed);
    if max_ms > spec.max_age_ms {
        if state.violated_last_window {
            return None;
        }
        state.violated_last_window = true;
        Some(Violation {
            rule: "max-age-runtime",
            fqn: spec.fqn,
            measured: max_ms,
            declared: spec.max_age_ms,
        })
    } else {
        state.violated_last_window = false;
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static CELL: PubMonitorCell = PubMonitorCell::new();

    fn spec(min_milli: u32) -> MonitorSpec {
        MonitorSpec {
            topic: "/chatter",
            fqn: "/demo/talker/chatter",
            min_rate_hz_milli: min_milli,
            max_latency_ms: 0,
            cell: &CELL,
        }
    }

    #[test]
    fn slow_publisher_fires_once_until_recovery() {
        CELL.count.store(0, Ordering::Relaxed);
        let s = spec(100_000); // 100 Hz declared
        let mut st = MonitorState::default();

        // t=0: opens window.
        assert!(check_rate(&s, &mut st, 0).is_none());
        // 5 publishes in 5 s = 1 Hz — violation.
        CELL.count.store(5, Ordering::Relaxed);
        let v = check_rate(&s, &mut st, RATE_CHECK_INTERVAL_US).expect("fires");
        assert_eq!(v.rule, "rate-hierarchy-runtime");
        assert_eq!(v.fqn, "/demo/talker/chatter");
        assert_eq!(v.measured, 1_000);
        assert_eq!(v.declared, 100_000);
        // Still slow next window — suppressed (no re-report spam).
        CELL.count.store(10, Ordering::Relaxed);
        assert!(check_rate(&s, &mut st, 2 * RATE_CHECK_INTERVAL_US).is_none());
        // Recovers (500 publishes in 5 s = 100 Hz) — clean window resets.
        CELL.count.store(510, Ordering::Relaxed);
        assert!(check_rate(&s, &mut st, 3 * RATE_CHECK_INTERVAL_US).is_none());
        // Degrades again — fires again.
        CELL.count.store(511, Ordering::Relaxed);
        assert!(check_rate(&s, &mut st, 4 * RATE_CHECK_INTERVAL_US).is_some());
    }

    #[test]
    fn compliant_and_uncontracted_stay_silent() {
        static C2: PubMonitorCell = PubMonitorCell::new();
        let s = MonitorSpec {
            topic: "/t",
            fqn: "/n/t",
            min_rate_hz_milli: 500, // 0.5 Hz
            max_latency_ms: 0,
            cell: &C2,
        };
        let mut st = MonitorState::default();
        assert!(check_rate(&s, &mut st, 0).is_none());
        C2.count.store(5, Ordering::Relaxed); // 1 Hz measured ≥ 0.5 Hz declared
        assert!(check_rate(&s, &mut st, RATE_CHECK_INTERVAL_US).is_none());

        // min_rate 0 = uncontracted: never fires, no state.
        let s0 = MonitorSpec {
            topic: "/t",
            fqn: "/n/t",
            min_rate_hz_milli: 0,
            max_latency_ms: 0,
            cell: &C2,
        };
        let mut st0 = MonitorState::default();
        assert!(check_rate(&s0, &mut st0, 10 * RATE_CHECK_INTERVAL_US).is_none());
    }

    #[test]
    fn stale_take_fires_age_once_until_recovery() {
        static SC: SubMonitorCell = SubMonitorCell::new();
        let s = AgeMonitorSpec {
            topic: "/scan",
            fqn: "/perc/detector/scan",
            max_age_ms: 100,
            cell: &SC,
        };
        let mut st = AgeState::default();

        // Fresh message: stamped 5 ms ago — silent.
        SC.observe(1_000_000_000, 1_000_005_000);
        assert!(check_age(&s, &mut st).is_none());
        // Stale: 250 ms old — fires with the measured age.
        SC.observe(1_000_000_000, 1_000_250_000);
        let v = check_age(&s, &mut st).expect("fires");
        assert_eq!(v.rule, "max-age-runtime");
        assert_eq!(v.fqn, "/perc/detector/scan");
        assert_eq!(v.measured, 250);
        assert_eq!(v.declared, 100);
        // Still stale next window — suppressed.
        SC.observe(1_000_000_000, 1_000_300_000);
        assert!(check_age(&s, &mut st).is_none());
        // Recovers — clean window resets; stale again refires.
        SC.observe(1_000_000_000, 1_000_010_000);
        assert!(check_age(&s, &mut st).is_none());
        SC.observe(1_000_000_000, 1_000_999_000);
        assert!(check_age(&s, &mut st).is_some());
    }

    #[test]
    fn peek_stamp_reads_le_time_and_rejects_unstamped() {
        // Encapsulation header (4B) + sec=100 nsec=5000 at offset 4.
        let mut raw = [0u8; 12];
        raw[4..8].copy_from_slice(&100i32.to_le_bytes());
        raw[8..12].copy_from_slice(&5_000u32.to_le_bytes());
        assert_eq!(peek_stamp_us(&raw, 4), Some(100_000_005));
        // Zero / negative sec = unstamped: no age sample.
        assert_eq!(peek_stamp_us(&[0u8; 12], 4), None);
        // Short buffer: no panic, no sample.
        assert_eq!(peek_stamp_us(&raw[..8], 4), None);
    }

    #[test]
    fn slow_path_fires_latency_once_until_recovery() {
        static C3: PubMonitorCell = PubMonitorCell::new();
        let s = MonitorSpec {
            topic: "/cmd",
            fqn: "/ctrl/control/cmd",
            min_rate_hz_milli: 0,
            max_latency_ms: 10,
            cell: &C3,
        };
        let mut st = MonitorState::default();
        // 4 ms dispatch — within budget.
        C3.max_latency_us.store(4_000, Ordering::Relaxed);
        assert!(check_latency(&s, &mut st).is_none());
        assert_eq!(C3.max_latency_us.load(Ordering::Relaxed), 0, "drained");
        // 25 ms dispatch — fires.
        C3.max_latency_us.store(25_000, Ordering::Relaxed);
        let v = check_latency(&s, &mut st).expect("fires");
        assert_eq!(v.rule, "max-latency-runtime");
        assert_eq!(v.measured, 25);
        assert_eq!(v.declared, 10);
        // Still slow — suppressed; recovery resets.
        C3.max_latency_us.store(30_000, Ordering::Relaxed);
        assert!(check_latency(&s, &mut st).is_none());
        C3.max_latency_us.store(1_000, Ordering::Relaxed);
        assert!(check_latency(&s, &mut st).is_none());
        C3.max_latency_us.store(30_000, Ordering::Relaxed);
        assert!(check_latency(&s, &mut st).is_some());
    }
}

/// Issue #514 — emit one violation to the log.
///
/// A log line is deliberately the floor rather than a `/diagnostics`
/// publication: it needs no publisher, no topic wiring, and no contract
/// on the reporting path itself, so it works on a bare RTOS image and
/// during boot. Publishing the same verdicts as `DiagnosticArray`
/// belongs on top of this, not instead of it.
///
/// Free function rather than an `Executor` method because every call
/// site sits inside a loop that already borrows the executor's spec
/// tables.
pub(crate) fn log_violation(v: &Violation) {
    nros_log::log_warn!(
        nros_log::get_logger("nros"),
        "contract violation: {} {} measured={} declared={}",
        v.rule,
        v.fqn,
        v.measured,
        v.declared
    );
}

/// Issue #505 — periodic activations a timer dropped because its
/// executor was blocked past the period boundary.
///
/// This rule exists because `check_rate` cannot see an isolated stall:
/// it samples publish counts over a ~5 s window, so 20 missed
/// activations of a 100 Hz loop are a 0.4% rate deficit — under any
/// sane declared minimum, silence. (And under
/// [`TimerOverrunPolicy::CatchUp`](super::arena::TimerOverrunPolicy)
/// the replayed activations refill the window entirely, so the rate
/// rule reports a HEALTHY loop while the tier is stalling.) The
/// overrun counter is exact, needs no window, and does not depend on
/// clock resolution.
///
/// `overruns` is the timer's monotonic saturating counter;
/// `last_reported` is the value at the previous check, so the verdict
/// is on the delta. Returns a violation when more than `tolerated`
/// activations were dropped since the last check.
pub(crate) fn check_timer_overrun(
    overruns: u32,
    last_reported: &mut u32,
    tolerated: u32,
) -> Option<Violation> {
    let dropped = overruns.saturating_sub(*last_reported);
    *last_reported = overruns;
    if dropped <= tolerated {
        return None;
    }
    Some(Violation {
        rule: "timer-overrun-runtime",
        // Timer entries carry no name at this altitude; same stand-in
        // as `deadline-miss-runtime`.
        fqn: "timer",
        measured: dropped,
        declared: tolerated,
    })
}

/// Issue #515 — report a spin wake that arrived so late the cadence it
/// claims cannot have been met.
///
/// Like `check_timer_overrun` and unlike the rate/age/latency rules, this
/// needs no baked spec table. The bound is the spin period ITSELF: the
/// caller passes it to `spin_once` as the pacing quantum, it is what
/// `system.toml` declares as `spin_period_us`, and a wake later than a full
/// period past its predecessor means an activation's worth of cadence was
/// lost. That is a contract failure for any declared period, so there is
/// nothing further to declare.
///
/// The tolerance is one whole period rather than zero, and deliberately so.
/// Sub-period lateness is ordinary scheduling noise -- on the measured FVP
/// lane the executor is late on a large fraction of wakes while still
/// holding its rate -- and a rule that fired on each one would report a
/// healthy system as broken thousands of times a second. What is NOT
/// ordinary is being a full period late, because by then the wake that
/// should have happened in between never did.
///
/// `max_jitter_us` is the executor's high-water since the last check and
/// `last_reported` the value at the previous one, so the verdict is on the
/// delta -- the same shape as the overrun counter, and for the same reason:
/// a maximum that has not moved is not a new fault.
pub(crate) fn check_release_jitter(
    max_jitter_us: u64,
    last_reported: &mut u64,
    period_us: u64,
) -> Option<Violation> {
    if period_us == 0 || max_jitter_us <= *last_reported {
        return None;
    }
    *last_reported = max_jitter_us;
    if max_jitter_us < period_us {
        return None;
    }
    Some(Violation {
        rule: "release-jitter-runtime",
        // Same stand-in as the timer and deadline rules: the spin loop is
        // not an endpoint and carries no fqn at this altitude.
        fqn: "spin",
        measured: max_jitter_us.min(u32::MAX as u64) as u32,
        declared: period_us.min(u32::MAX as u64) as u32,
    })
}

/// Report a spin thread whose stack has come closer to its end than the
/// declared minimum.
///
/// Unlike every other rule here the bound CANNOT be derived from something
/// already declared, and that is worth stating rather than papering over.
/// `check_timer_overrun` and `check_release_jitter` both judge against a
/// period the caller already passes in; there is no equivalent for a stack.
/// The executor never sees `stack_bytes` -- it lives in the spawn attr and
/// goes no further -- and the total is not portably queryable either:
/// FreeRTOS exposes the high-water mark and not the size it was taken
/// against, so even a percentage cannot be computed. A minimum headroom is
/// therefore a real declaration, and `min_bytes == 0` means the caller has
/// not made one, which disables the rule.
///
/// Reports on the WORST case, not on each crossing: `worst_reported` holds
/// the lowest headroom already reported, so a stack hovering just under the
/// bound says so once and then only when it gets worse. The same delta
/// discipline as the overrun and jitter rules, inverted because for headroom
/// smaller is worse.
pub(crate) fn check_stack_headroom(
    unused_bytes: usize,
    min_bytes: usize,
    worst_reported: &mut usize,
) -> Option<Violation> {
    if min_bytes == 0 || unused_bytes >= min_bytes {
        return None;
    }
    // `usize::MAX` is the "nothing reported yet" sentinel: any real headroom
    // is below it, so the first breach always reports.
    if *worst_reported != usize::MAX && unused_bytes >= *worst_reported {
        return None;
    }
    *worst_reported = unused_bytes;
    Some(Violation {
        rule: "stack-headroom-runtime",
        // The spin thread is not an endpoint; same stand-in as the timer,
        // deadline and jitter rules.
        fqn: "stack",
        measured: unused_bytes.min(u32::MAX as usize) as u32,
        declared: min_bytes.min(u32::MAX as usize) as u32,
    })
}

#[cfg(test)]
mod stack_headroom_rule_tests {
    use super::*;

    /// No declared minimum means no claim to breach.
    #[test]
    fn a_zero_minimum_disables_the_rule() {
        let mut worst = usize::MAX;
        assert!(check_stack_headroom(8, 0, &mut worst).is_none());
    }

    #[test]
    fn headroom_at_the_bound_is_not_a_breach() {
        let mut worst = usize::MAX;
        assert!(check_stack_headroom(1024, 1024, &mut worst).is_none());
        assert!(check_stack_headroom(2048, 1024, &mut worst).is_none());
    }

    #[test]
    fn reports_the_first_breach_with_both_numbers() {
        let mut worst = usize::MAX;
        let v = check_stack_headroom(512, 1024, &mut worst).expect("under the bound reports");
        assert_eq!(v.rule, "stack-headroom-runtime");
        assert_eq!(v.measured, 512);
        assert_eq!(v.declared, 1024);
    }

    /// Smaller is worse for headroom, so the delta runs the other way.
    #[test]
    fn only_a_new_low_is_a_new_fault() {
        let mut worst = usize::MAX;
        assert!(check_stack_headroom(512, 1024, &mut worst).is_some());
        assert!(check_stack_headroom(512, 1024, &mut worst).is_none());
        assert!(check_stack_headroom(600, 1024, &mut worst).is_none());
        let v = check_stack_headroom(100, 1024, &mut worst).expect("a new low reports");
        assert_eq!(v.measured, 100);
    }
}

#[cfg(test)]
mod publish_stamp_tests {
    use super::*;

    /// CDR: 4-byte encapsulation header, then `Time { i32 sec; u32 nanosec }`
    /// little-endian, so `sec` sits at byte 4 — the layout `STAMP_OFFSET`
    /// encodes.
    fn cdr_with_stamp(sec: i32, nanosec: u32) -> [u8; 12] {
        let mut b = [0u8; 12];
        b[4..8].copy_from_slice(&sec.to_le_bytes());
        b[8..12].copy_from_slice(&nanosec.to_le_bytes());
        b
    }

    #[test]
    fn records_the_age_of_what_was_published() {
        let cell = PubMonitorCell::new();
        let raw = cdr_with_stamp(10, 0); // stamped at 10_000_000 us
        observe_publish_stamp(&cell, &raw, 4, 10_500_000);
        assert_eq!(
            cell.last_publish_stamp_age_us.load(Ordering::Relaxed),
            500_000,
            "published data was half a second old"
        );
    }

    /// The signature of a node that RE-STAMPED: it publishes data whose
    /// stamp is now, so downstream age is single-hop, not end-to-end.
    #[test]
    fn a_restamping_node_shows_near_zero_age() {
        let cell = PubMonitorCell::new();
        let raw = cdr_with_stamp(10, 0);
        observe_publish_stamp(&cell, &raw, 4, 10_000_000);
        assert_eq!(cell.last_publish_stamp_age_us.load(Ordering::Relaxed), 0);
    }

    /// A state, not a window maximum: one stale message at startup must not
    /// pin the value for the life of the process.
    #[test]
    fn the_latest_publish_replaces_the_previous() {
        let cell = PubMonitorCell::new();
        observe_publish_stamp(&cell, &cdr_with_stamp(10, 0), 4, 12_000_000);
        assert_eq!(
            cell.last_publish_stamp_age_us.load(Ordering::Relaxed),
            2_000_000
        );
        observe_publish_stamp(&cell, &cdr_with_stamp(20, 0), 4, 20_100_000);
        assert_eq!(
            cell.last_publish_stamp_age_us.load(Ordering::Relaxed),
            100_000,
            "a fresh publish replaces the old age rather than maxing with it"
        );
    }

    /// An unset stamp is not an age of `now`. `peek_stamp_us` rejects
    /// `sec <= 0`, so a zeroed header records nothing at all.
    #[test]
    fn an_unstamped_message_records_nothing() {
        let cell = PubMonitorCell::new();
        observe_publish_stamp(&cell, &cdr_with_stamp(0, 0), 4, 5_000_000);
        assert_eq!(
            cell.last_publish_stamp_age_us.load(Ordering::Relaxed),
            0,
            "no stamp means no observation, not an enormous age"
        );
    }
}

#[cfg(test)]
mod release_jitter_rule_tests {
    use super::*;

    /// Sub-period lateness is noise, not a violation -- otherwise a
    /// healthy-but-jittery loop reports thousands of faults a second.
    #[test]
    fn tolerates_lateness_within_one_period() {
        let mut last = 0;
        assert!(check_release_jitter(4_000, &mut last, 5_000).is_none());
        assert_eq!(last, 4_000, "still recorded, so the next delta is honest");
    }

    /// A full period late means the wake that belonged in between never
    /// happened.
    #[test]
    fn reports_a_wake_a_whole_period_late() {
        let mut last = 0;
        let v = check_release_jitter(5_000, &mut last, 5_000).expect("one period late reports");
        assert_eq!(v.rule, "release-jitter-runtime");
        assert_eq!(v.measured, 5_000);
        assert_eq!(v.declared, 5_000);
    }

    /// The verdict is on the DELTA. A high-water that has not moved is the
    /// same fault already reported, not a new one.
    #[test]
    fn an_unchanged_maximum_is_not_a_new_fault() {
        let mut last = 0;
        assert!(check_release_jitter(9_000, &mut last, 5_000).is_some());
        assert!(check_release_jitter(9_000, &mut last, 5_000).is_none());
        assert!(check_release_jitter(12_000, &mut last, 5_000).is_some());
    }

    /// No declared period means no cadence to be late for.
    #[test]
    fn a_zero_period_declares_nothing() {
        let mut last = 0;
        assert!(check_release_jitter(1_000_000, &mut last, 0).is_none());
    }
}

#[cfg(test)]
mod timer_overrun_rule_tests {
    use super::*;

    #[test]
    fn reports_the_delta_not_the_total() {
        let mut last = 0;
        let v = check_timer_overrun(19, &mut last, 0).expect("first drop reports");
        assert_eq!(v.rule, "timer-overrun-runtime");
        assert_eq!(v.measured, 19);
        // Same total on the next check is not a new fault.
        assert!(check_timer_overrun(19, &mut last, 0).is_none());
        // Only the newly dropped activations are reported.
        assert_eq!(check_timer_overrun(25, &mut last, 0).unwrap().measured, 6);
    }

    #[test]
    fn a_clean_timer_never_reports() {
        let mut last = 0;
        for _ in 0..10 {
            assert!(check_timer_overrun(0, &mut last, 0).is_none());
        }
    }

    #[test]
    fn tolerance_suppresses_small_drops() {
        let mut last = 0;
        assert!(check_timer_overrun(2, &mut last, 2).is_none());
        assert_eq!(check_timer_overrun(6, &mut last, 2).unwrap().measured, 4);
    }
}
