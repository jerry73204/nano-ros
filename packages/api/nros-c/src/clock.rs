//! Clock API for nros C API.
//!
//! Provides time sources for ROS 2 compatible timing operations.

use core::ffi::c_int;

use crate::error::*;

// ============================================================================
// Time Types
// ============================================================================

/// Time representation compatible with builtin_interfaces/msg/Time.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct nros_time_t {
    /// Seconds component
    pub sec: i32,
    /// Nanoseconds component (0 to 999,999,999)
    pub nanosec: u32,
}

/// Duration representation compatible with builtin_interfaces/msg/Duration.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct nros_duration_t {
    /// Seconds component (can be negative)
    pub sec: i32,
    /// Nanoseconds component (0 to 999,999,999)
    pub nanosec: u32,
}

// ============================================================================
// Clock Types
// ============================================================================

/// Clock type enumeration.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_clock_type_t {
    /// Uninitialized clock
    NROS_CLOCK_UNINITIALIZED = 0,
    /// ROS time - follows /clock topic if available, otherwise system time
    NROS_CLOCK_ROS_TIME = 1,
    /// System time - wall clock time from the operating system
    NROS_CLOCK_SYSTEM_TIME = 2,
    /// Steady time - monotonic clock, not affected by system time changes
    NROS_CLOCK_STEADY_TIME = 3,
}

/// Clock state enumeration.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum nros_clock_state_t {
    /// Not initialized
    NROS_CLOCK_STATE_UNINITIALIZED = 0,
    /// Initialized and ready
    NROS_CLOCK_STATE_READY = 1,
    /// Shutdown
    NROS_CLOCK_STATE_SHUTDOWN = 2,
}

/// Clock structure.
#[repr(C)]
pub struct nros_clock_t {
    /// Clock type
    pub r#type: nros_clock_type_t,
    /// Current state
    pub state: nros_clock_state_t,
    /// Internal: steady clock epoch (nanoseconds since process start)
    pub _steady_epoch_ns: u64,
}

impl Default for nros_clock_t {
    fn default() -> Self {
        Self {
            r#type: nros_clock_type_t::NROS_CLOCK_UNINITIALIZED,
            state: nros_clock_state_t::NROS_CLOCK_STATE_UNINITIALIZED,
            _steady_epoch_ns: 0,
        }
    }
}

// ============================================================================
// Platform-specific time functions
// ============================================================================

use crate::platform;

/// Nanoseconds per second constant
const NANOS_PER_SEC: u64 = 1_000_000_000;

/// Get system time in nanoseconds since Unix epoch.
fn get_system_time_ns() -> i64 {
    platform::get_system_time_ns()
}

/// Get steady (monotonic) time in nanoseconds.
fn get_steady_time_ns() -> u64 {
    platform::get_time_ns()
}

/// What a `NROS_CLOCK_ROS_TIME` clock reads (issue 0789).
///
/// The ROS time override when one is active — a simulator's or a bag player's
/// `/clock`, installed through `rcl_set_ros_time_override` — and otherwise the
/// system clock, which is what `NROS_CLOCK_ROS_TIME` has always read here and
/// what rcl's ROS-time clock falls back to when the override is disabled.
///
/// The override itself is `nros_core::Clock`'s, not a second copy: the C
/// switches below drive the SAME global a Rust `Clock::ros_time()` reads, so an
/// image whose C nodes and Rust nodes share a process cannot end up with two
/// answers to "what time is it". (Rust's `RosTime` fallback is the steady
/// counter rather than the system clock — that difference predates this and is
/// the two languages' fallback, not their override.)
fn get_ros_time_ns() -> i64 {
    match nros_core::Clock::get_ros_time_override() {
        Some(t) => t.to_nanos(),
        None => get_system_time_ns(),
    }
}

// ============================================================================
// Clock Functions
// ============================================================================

/// Get a zero-initialized clock.
#[unsafe(no_mangle)]
pub extern "C" fn nros_clock_get_zero_initialized() -> nros_clock_t {
    nros_clock_t::default()
}

/// Initialize a clock.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_clock_init(
    clock: *mut nros_clock_t,
    clock_type: nros_clock_type_t,
) -> nros_ret_t {
    if clock.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    // Validate clock type
    match clock_type {
        nros_clock_type_t::NROS_CLOCK_ROS_TIME
        | nros_clock_type_t::NROS_CLOCK_SYSTEM_TIME
        | nros_clock_type_t::NROS_CLOCK_STEADY_TIME => {}
        _ => return NROS_RET_INVALID_ARGUMENT,
    }

    let clock = &mut *clock;

    // Check if already initialized
    if clock.state != nros_clock_state_t::NROS_CLOCK_STATE_UNINITIALIZED {
        return NROS_RET_ALREADY_EXISTS;
    }

    clock.r#type = clock_type;
    clock.state = nros_clock_state_t::NROS_CLOCK_STATE_READY;

    // For steady time, record the epoch
    if clock_type == nros_clock_type_t::NROS_CLOCK_STEADY_TIME {
        clock._steady_epoch_ns = get_steady_time_ns();
    }

    NROS_RET_OK
}

/// Get the current time from a clock.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_clock_get_now(
    clock: *const nros_clock_t,
    time_out: *mut nros_time_t,
) -> nros_ret_t {
    if clock.is_null() || time_out.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let clock = &*clock;

    if clock.state != nros_clock_state_t::NROS_CLOCK_STATE_READY {
        return NROS_RET_NOT_INIT;
    }

    let nanos = match clock.r#type {
        nros_clock_type_t::NROS_CLOCK_ROS_TIME => get_ros_time_ns(),
        nros_clock_type_t::NROS_CLOCK_SYSTEM_TIME => get_system_time_ns(),
        nros_clock_type_t::NROS_CLOCK_STEADY_TIME => {
            // Steady time relative to epoch
            get_steady_time_ns() as i64
        }
        _ => return NROS_RET_ERROR,
    };

    *time_out = nros_time_from_nanoseconds(nanos);
    NROS_RET_OK
}

/// Get the current time from a clock as nanoseconds.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_clock_get_now_ns(
    clock: *const nros_clock_t,
    nanoseconds: *mut i64,
) -> nros_ret_t {
    if clock.is_null() || nanoseconds.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let clock = &*clock;

    if clock.state != nros_clock_state_t::NROS_CLOCK_STATE_READY {
        return NROS_RET_NOT_INIT;
    }

    let nanos = match clock.r#type {
        nros_clock_type_t::NROS_CLOCK_ROS_TIME => get_ros_time_ns(),
        nros_clock_type_t::NROS_CLOCK_SYSTEM_TIME => get_system_time_ns(),
        nros_clock_type_t::NROS_CLOCK_STEADY_TIME => get_steady_time_ns() as i64,
        _ => return NROS_RET_ERROR,
    };

    *nanoseconds = nanos;
    NROS_RET_OK
}

/// Check if a clock is valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_clock_is_valid(clock: *const nros_clock_t) -> bool {
    if clock.is_null() {
        return false;
    }

    let clock = &*clock;
    clock.state == nros_clock_state_t::NROS_CLOCK_STATE_READY
}

/// Get the clock type.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_clock_get_type(clock: *const nros_clock_t) -> nros_clock_type_t {
    if clock.is_null() {
        return nros_clock_type_t::NROS_CLOCK_UNINITIALIZED;
    }

    (*clock).r#type
}

/// Finalize a clock.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_fini(clock: *mut nros_clock_t) -> nros_ret_t {
    if clock.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }

    let clock = &mut *clock;

    if clock.state == nros_clock_state_t::NROS_CLOCK_STATE_UNINITIALIZED {
        return NROS_RET_NOT_INIT;
    }

    clock.state = nros_clock_state_t::NROS_CLOCK_STATE_SHUTDOWN;
    clock.r#type = nros_clock_type_t::NROS_CLOCK_UNINITIALIZED;
    clock._steady_epoch_ns = 0;

    NROS_RET_OK
}

// ============================================================================
// ROS time — driving a C image from a simulator's `/clock` (issue 0789)
// ============================================================================
//
// These are the C face of `nros_core::Clock`'s override switches
// (`set_ros_time_override`, `clear_ros_time_override`,
// `is_ros_time_override_active`, `get_ros_time_override`) and they drive that
// crate's global, not a copy of it — one model per image, whichever language
// installs the time. Names and signatures are rcl's
// (`rcl_enable_ros_time_override` …), so a ported node keeps its spelling.
//
// # The one model difference from rcl, stated once
//
// rcl holds two pieces of state: an ENABLED flag and a VALUE, so setting a
// value while disabled stores it without taking effect. Ours holds one — a
// negative sentinel means "no override" — because that is the model Rust has
// had since the clock shipped and a second one in C is exactly the split issue
// 0789 is about. Consequences, both documented at their entry points:
//
//   * `rcl_set_ros_time_override` also ENABLES. There is no representable
//     state where a value is stored but inert.
//   * `rcl_set_ros_time_override` REJECTS a negative time, which is the
//     sentinel's cost. ROS time is nanoseconds since the epoch, so it is
//     non-negative for any clock a simulator or a bag publishes.
//
// The override is process-global rather than per-clock (rcl's lives in the
// clock's storage). With no allocator a clock cannot own a separately-shared
// time source — the same constraint the Rust ledger rows record for
// `Clock::set_ros_time_override` — and an image has one simulation clock.

/// Shared precondition for every ROS-time switch: a live clock, of ROS type.
///
/// `NROS_RET_NOT_ALLOWED` for the wrong clock TYPE names why the call was
/// refused; rcl returns its generic error there. A steady or system clock is
/// not a thing `/clock` can drive, and silently accepting the call would leave
/// the caller believing the image is simulated when it is reading the wall.
unsafe fn ros_time_clock_ok(clock: *const nros_clock_t) -> nros_ret_t {
    if clock.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let clock = &*clock;
    if clock.state != nros_clock_state_t::NROS_CLOCK_STATE_READY {
        return NROS_RET_NOT_INIT;
    }
    if clock.r#type != nros_clock_type_t::NROS_CLOCK_ROS_TIME {
        return NROS_RET_NOT_ALLOWED;
    }
    NROS_RET_OK
}

/// Enable the ROS time override on a `NROS_CLOCK_ROS_TIME` clock.
///
/// Mirrors `rcl_enable_ros_time_override`. If no override is active yet the
/// clock starts at time 0 — the state rcl reports as "not started"
/// (`rcl_clock_time_started`), i.e. enabled but waiting for the first
/// `/clock` sample. An already-active override keeps its value.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_enable_ros_time_override(clock: *const nros_clock_t) -> nros_ret_t {
    let ret = ros_time_clock_ok(clock);
    if ret != NROS_RET_OK {
        return ret;
    }

    if !nros_core::Clock::is_ros_time_override_active() {
        nros_core::Clock::set_ros_time_override(0);
    }
    NROS_RET_OK
}

/// Disable the ROS time override; the clock reads the system clock again.
///
/// Mirrors `rcl_disable_ros_time_override`, and `nros_core::Clock::
/// clear_ros_time_override` underneath. The stored time does not survive: with
/// one piece of state there is nowhere for an inert value to live, so
/// re-enabling starts from 0 again.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_disable_ros_time_override(clock: *const nros_clock_t) -> nros_ret_t {
    let ret = ros_time_clock_ok(clock);
    if ret != NROS_RET_OK {
        return ret;
    }

    nros_core::Clock::clear_ros_time_override();
    NROS_RET_OK
}

/// Whether the ROS time override is in effect.
///
/// Mirrors `rcl_is_enabled_ros_time_override`. Writes `is_enabled` only on
/// `NROS_RET_OK`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_is_enabled_ros_time_override(
    clock: *const nros_clock_t,
    is_enabled: *mut bool,
) -> nros_ret_t {
    if is_enabled.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let ret = ros_time_clock_ok(clock);
    if ret != NROS_RET_OK {
        return ret;
    }

    *is_enabled = nros_core::Clock::is_ros_time_override_active();
    NROS_RET_OK
}

/// Set the time a `NROS_CLOCK_ROS_TIME` clock reads, in nanoseconds since the
/// epoch — the entry point a `/clock` subscriber or a bag player calls.
///
/// Mirrors `rcl_set_ros_time_override`, with the one model difference stated at
/// the top of this section: setting a value also ENABLES the override, and a
/// NEGATIVE `nanoseconds` is `NROS_RET_INVALID_ARGUMENT` because a negative
/// value is the "no override" sentinel.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_set_ros_time_override(
    clock: *const nros_clock_t,
    nanoseconds: i64,
) -> nros_ret_t {
    if nanoseconds < 0 {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let ret = ros_time_clock_ok(clock);
    if ret != NROS_RET_OK {
        return ret;
    }

    nros_core::Clock::set_ros_time_override(nanoseconds);
    NROS_RET_OK
}

/// Read the ROS time override back, in nanoseconds since the epoch.
///
/// The C face of `nros_core::Clock::get_ros_time_override`, whose `Option` this
/// splits into a status and an out-parameter: `NROS_RET_OK` and a value when an
/// override is in effect, `NROS_RET_NOT_FOUND` and an untouched out-parameter
/// when none is. rcl has no equivalent — a caller there reads the clock — but
/// dropping it would leave the C mirror of the Rust surface one entry point
/// short, and "is a simulator driving me, and at what time" is one question.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_get_ros_time_override(
    clock: *const nros_clock_t,
    nanoseconds: *mut i64,
) -> nros_ret_t {
    if nanoseconds.is_null() {
        return NROS_RET_INVALID_ARGUMENT;
    }
    let ret = ros_time_clock_ok(clock);
    if ret != NROS_RET_OK {
        return ret;
    }

    match nros_core::Clock::get_ros_time_override() {
        Some(t) => {
            *nanoseconds = t.to_nanos();
            NROS_RET_OK
        }
        None => NROS_RET_NOT_FOUND,
    }
}

/// Whether the clock has produced a time yet.
///
/// Mirrors `rcl_clock_time_started`, including its implementation: read the
/// clock and report whether the value is non-zero. For a ROS-time clock that is
/// the "enabled, but no `/clock` sample has arrived" state — the one
/// `rcl_enable_ros_time_override` leaves behind — and for a system or steady
/// clock it is true as soon as the clock is valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_clock_time_started(clock: *const nros_clock_t) -> bool {
    let mut nanoseconds: i64 = 0;
    if nros_clock_get_now_ns(clock, &mut nanoseconds) != NROS_RET_OK {
        return false;
    }
    nanoseconds != 0
}

// ============================================================================
// Time Utility Functions
// ============================================================================

/// Split a signed nanosecond count into the `(sec, nanosec)` pair
/// `builtin_interfaces` carries (issue 0799).
///
/// `Time.msg` states the convention in its own comments: *"The nanoseconds
/// component, valid in the range [0, 1e9), to be added to the seconds
/// component. The time -1.7 seconds is represented as {sec: -2, nanosec:
/// 3e8}"*. That is FLOOR division with a NON-NEGATIVE remainder, which is
/// exactly `div_euclid` / `rem_euclid` — and exactly what truncation plus
/// `unsigned_abs` (what this used to be) is not: `-0.5 s` came out as
/// `{sec: 0, nanosec: 5e8}`, which decodes as `+0.5 s`.
///
/// # Out-of-range input SATURATES, it does not wrap
///
/// `sec` is `int32` on the wire, so the encodable range is roughly ±68 years of
/// nanoseconds while the input is an `i64` reaching ±292. The old `as i32`
/// truncated the high bits, so one nanosecond past the end came back as a time
/// 136 years in the other direction — a silent sign flip, the same failure this
/// function was fixed for. Saturating keeps the encoding MONOTONE (an
/// unrepresentable instant clamps to the nearest representable one and still
/// compares in the right direction) and is the only choice available: the
/// signature returns a value with no error channel, and a `panic!` in a
/// `no_std` C entry point aborts the image over a timestamp.
fn split_nanoseconds(nanoseconds: i64) -> (i32, u32) {
    let sec = nanoseconds.div_euclid(NANOS_PER_SEC as i64);
    if sec > i32::MAX as i64 {
        return (i32::MAX, NANOS_PER_SEC as u32 - 1);
    }
    if sec < i32::MIN as i64 {
        return (i32::MIN, 0);
    }
    // `rem_euclid` with a positive divisor is guaranteed in `[0, 1e9)`, which
    // is the range `Time.msg` requires of `nanosec`.
    (
        sec as i32,
        nanoseconds.rem_euclid(NANOS_PER_SEC as i64) as u32,
    )
}

/// Convert nanoseconds to a nros_time_t structure.
#[unsafe(no_mangle)]
pub extern "C" fn nros_time_from_nanoseconds(nanoseconds: i64) -> nros_time_t {
    let (sec, nanosec) = split_nanoseconds(nanoseconds);
    nros_time_t { sec, nanosec }
}

/// Convert nanoseconds to a nros_duration_t structure.
///
/// The duration half of `nros_time_from_nanoseconds`, and the same encoding —
/// `builtin_interfaces/msg/Duration` has the identical `{sec, nanosec}` shape
/// and the identical `[0, 1e9)` rule on `nanosec`. Issue 0799: a negative span
/// is ordinary ("how far ahead of the deadline are we" is negative half the
/// time), and until this existed the C++ `Duration::to_msg` open-coded the
/// split rather than delegate to a `_time_` entry point that got negatives
/// wrong.
#[unsafe(no_mangle)]
pub extern "C" fn nros_duration_from_nanoseconds(nanoseconds: i64) -> nros_duration_t {
    let (sec, nanosec) = split_nanoseconds(nanoseconds);
    nros_duration_t { sec, nanosec }
}

/// Convert a nros_time_t structure to nanoseconds.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_time_to_nanoseconds(time: *const nros_time_t) -> i64 {
    if time.is_null() {
        return 0;
    }

    let time = &*time;
    (time.sec as i64) * (NANOS_PER_SEC as i64) + (time.nanosec as i64)
}

/// Add a duration to a time.
#[unsafe(no_mangle)]
pub extern "C" fn nros_time_add(time: nros_time_t, duration: nros_duration_t) -> nros_time_t {
    let time_ns = (time.sec as i64) * (NANOS_PER_SEC as i64) + (time.nanosec as i64);
    let duration_ns = (duration.sec as i64) * (NANOS_PER_SEC as i64) + (duration.nanosec as i64);

    nros_time_from_nanoseconds(time_ns + duration_ns)
}

/// Subtract a duration from a time.
#[unsafe(no_mangle)]
pub extern "C" fn nros_time_sub(time: nros_time_t, duration: nros_duration_t) -> nros_time_t {
    let time_ns = (time.sec as i64) * (NANOS_PER_SEC as i64) + (time.nanosec as i64);
    let duration_ns = (duration.sec as i64) * (NANOS_PER_SEC as i64) + (duration.nanosec as i64);

    nros_time_from_nanoseconds(time_ns - duration_ns)
}

/// Compare two times.
#[unsafe(no_mangle)]
pub extern "C" fn nros_time_compare(a: nros_time_t, b: nros_time_t) -> c_int {
    if a.sec < b.sec {
        -1
    } else if a.sec > b.sec {
        1
    } else if a.nanosec < b.nanosec {
        -1
    } else if a.nanosec > b.nanosec {
        1
    } else {
        0
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const NS: i64 = NANOS_PER_SEC as i64;

    /// The wire decoder, which is the correct inverse of the correct encoding —
    /// so a round trip through it EXPOSES an encoder error rather than
    /// cancelling it (issue 0799).
    fn decode(t: nros_time_t) -> i64 {
        unsafe { nros_time_to_nanoseconds(&t) }
    }

    /// The largest and smallest nanosecond counts an `int32` seconds field can
    /// hold. Beyond these `split_nanoseconds` saturates by design.
    const MAX_ENCODABLE_NS: i64 = (i32::MAX as i64) * NS + (NS - 1);
    const MIN_ENCODABLE_NS: i64 = (i32::MIN as i64) * NS;

    /// `builtin_interfaces/msg/Time.msg` states the convention in its own
    /// comments; these are the two values it names, plus their mirror images.
    #[test]
    fn encodes_the_convention_time_msg_documents() {
        // "The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}"
        assert_eq!(
            nros_time_from_nanoseconds(-1_700_000_000),
            nros_time_t {
                sec: -2,
                nanosec: 300_000_000
            }
        );
        // The sub-second case that used to come back with the sign gone.
        assert_eq!(
            nros_time_from_nanoseconds(-500_000_000),
            nros_time_t {
                sec: -1,
                nanosec: 500_000_000
            }
        );
        assert_eq!(
            nros_time_from_nanoseconds(1_700_000_000),
            nros_time_t {
                sec: 1,
                nanosec: 700_000_000
            }
        );
        // Exactly on a second boundary: no borrow, `nanosec` is zero on both
        // sides of zero.
        assert_eq!(
            nros_time_from_nanoseconds(-2 * NS),
            nros_time_t {
                sec: -2,
                nanosec: 0
            }
        );
        assert_eq!(nros_time_from_nanoseconds(0), nros_time_t::default());
    }

    /// The deliverable of issue 0799: encode/decode is the identity over
    /// NEGATIVE, zero, sub-second and multi-second values, and every encoding
    /// obeys `nanosec ∈ [0, 1e9)`.
    ///
    /// The old encoder (truncate toward zero, `unsigned_abs` remainder) fails
    /// this on the first negative sub-second value — verified by reverting it
    /// and re-running, which is exactly the check the positives-only tests
    /// could not make.
    #[test]
    fn round_trips_over_negative_zero_subsecond_and_multisecond() {
        // Hand-picked coordinates first: the boundaries a generator hits only
        // by luck.
        let edges: [i64; 24] = [
            0,
            1,
            -1,
            999_999_999,
            -999_999_999,
            NS,
            -NS,
            NS + 1,
            -NS - 1,
            NS - 1,
            -(NS - 1),
            -500_000_000,
            -1_700_000_000,
            500_000_000,
            1_700_000_000,
            -86_400 * NS,
            86_400 * NS,
            -86_400 * NS - 1,
            123 * NS + 456,
            -123 * NS - 456,
            MAX_ENCODABLE_NS,
            MIN_ENCODABLE_NS,
            MAX_ENCODABLE_NS - 1,
            MIN_ENCODABLE_NS + 1,
        ];

        // Then a sweep. A deterministic LCG rather than a random source: a
        // property test that cannot be re-run on the value that failed is a
        // report nobody can act on. The magnitude cycles through nanoseconds,
        // milliseconds, seconds and days so sub-second and multi-second values
        // are both dense in the sample, and each is tried with both signs.
        let mut state: u64 = 0x2545_F491_4F6C_DD1D;
        let scales = [1i64, 1_000, 1_000_000, NS, 3_600 * NS];
        let generated = (0..20_000).map(|i| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let scale = scales[i % scales.len()];
            let magnitude = ((state >> 11) as i64).rem_euclid(MAX_ENCODABLE_NS / scale) * scale;
            if i % 2 == 0 { magnitude } else { -magnitude }
        });

        for ns in edges.into_iter().chain(generated) {
            let t = nros_time_from_nanoseconds(ns);

            assert!(
                t.nanosec < NS as u32,
                "nanosec out of the [0, 1e9) range Time.msg requires: {ns} -> {t:?}"
            );
            assert_eq!(decode(t), ns, "round trip lost {ns}: encoded as {t:?}");
            // Floor, not truncation: the seconds field never rounds toward zero.
            assert_eq!(
                t.sec as i64,
                ns.div_euclid(NS),
                "seconds did not floor for {ns}"
            );
            // A negative instant must decode negative. Stated separately from
            // the round trip because THIS is the user-visible symptom: -0.5 s
            // used to come back as +0.5 s.
            assert_eq!(
                decode(t).signum(),
                ns.signum(),
                "sign lost encoding {ns} as {t:?}"
            );

            // The duration encoding is the same encoding.
            let d = nros_duration_from_nanoseconds(ns);
            assert_eq!(
                (d.sec, d.nanosec),
                (t.sec, t.nanosec),
                "duration != time for {ns}"
            );
        }
    }

    /// Past ±68 years of nanoseconds an `int32` seconds field cannot hold the
    /// value. The old `as i32` wrapped, turning an overflow into a time in the
    /// opposite direction; this clamps to the nearest representable instant, so
    /// the encoding stays monotone and the sign survives.
    #[test]
    fn out_of_range_saturates_rather_than_wrapping() {
        let past_max = nros_time_from_nanoseconds(MAX_ENCODABLE_NS + 1);
        assert_eq!(
            past_max,
            nros_time_t {
                sec: i32::MAX,
                nanosec: 999_999_999
            }
        );
        assert_eq!(nros_time_from_nanoseconds(i64::MAX), past_max);

        let past_min = nros_time_from_nanoseconds(MIN_ENCODABLE_NS - 1);
        assert_eq!(
            past_min,
            nros_time_t {
                sec: i32::MIN,
                nanosec: 0
            }
        );
        assert_eq!(nros_time_from_nanoseconds(i64::MIN), past_min);

        // Monotone across the clamp, and still signed the right way — the two
        // properties wrapping destroyed.
        assert!(decode(past_max) > 0);
        assert!(decode(past_min) < 0);
        assert!(decode(past_max) > decode(nros_time_from_nanoseconds(MAX_ENCODABLE_NS - NS)));
        assert!(decode(past_min) < decode(nros_time_from_nanoseconds(MIN_ENCODABLE_NS + NS)));
    }

    /// `nros_time_add` / `nros_time_sub` route their `i64` result through the
    /// encoder, so a result landing in (−1 s, 0) is exactly the case that used
    /// to come back positive.
    #[test]
    fn arithmetic_across_zero_keeps_its_sign() {
        let quarter_second = nros_duration_t {
            sec: 0,
            nanosec: 250_000_000,
        };

        // 0.1 s − 0.25 s = −0.15 s, which lands inside the broken window.
        let t = nros_time_t {
            sec: 0,
            nanosec: 100_000_000,
        };
        let diff = nros_time_sub(t, quarter_second);
        assert_eq!(decode(diff), -150_000_000);
        assert_eq!(
            diff,
            nros_time_t {
                sec: -1,
                nanosec: 850_000_000
            }
        );

        // Adding it back is the identity.
        assert_eq!(nros_time_add(diff, quarter_second), t);

        // And ordering survives: an earlier time compares less, which is what
        // a receiver reasoning about latency depends on.
        assert_eq!(nros_time_compare(diff, t), -1);
    }

    /// Issue 0789 — the ROS-time switches C did not have, and the clock read
    /// they drive. One test rather than five: the override is process-global
    /// (as it is in Rust), so separate `#[test]`s would race under nextest's
    /// thread pool.
    #[test]
    fn ros_time_override_drives_a_ros_clock() {
        let mut ros = nros_clock_get_zero_initialized();
        let mut system = nros_clock_get_zero_initialized();
        unsafe {
            assert_eq!(
                nros_clock_init(&mut ros, nros_clock_type_t::NROS_CLOCK_ROS_TIME),
                NROS_RET_OK
            );
            assert_eq!(
                nros_clock_init(&mut system, nros_clock_type_t::NROS_CLOCK_SYSTEM_TIME),
                NROS_RET_OK
            );

            // Start from a known state — another crate in the image may have
            // installed one.
            assert_eq!(rcl_disable_ros_time_override(&ros), NROS_RET_OK);

            let mut enabled = true;
            assert_eq!(
                rcl_is_enabled_ros_time_override(&ros, &mut enabled),
                NROS_RET_OK
            );
            assert!(!enabled);

            let mut nanoseconds: i64 = 42;
            assert_eq!(
                nros_get_ros_time_override(&ros, &mut nanoseconds),
                NROS_RET_NOT_FOUND
            );
            assert_eq!(nanoseconds, 42, "out-parameter written with no override");

            // With no override the ROS clock reads the wall, as it always has.
            assert_eq!(nros_clock_get_now_ns(&ros, &mut nanoseconds), NROS_RET_OK);
            assert!(nanoseconds > 0);
            assert!(rcl_clock_time_started(&ros));

            // Enabled but no `/clock` sample yet: time 0, and NOT started —
            // the state `rcl_clock_time_started` exists to report.
            assert_eq!(rcl_enable_ros_time_override(&ros), NROS_RET_OK);
            assert_eq!(
                rcl_is_enabled_ros_time_override(&ros, &mut enabled),
                NROS_RET_OK
            );
            assert!(enabled);
            assert_eq!(nros_clock_get_now_ns(&ros, &mut nanoseconds), NROS_RET_OK);
            assert_eq!(nanoseconds, 0);
            assert!(!rcl_clock_time_started(&ros));

            // A `/clock` sample arrives.
            let simulated = 1_234_500_000_000i64;
            assert_eq!(rcl_set_ros_time_override(&ros, simulated), NROS_RET_OK);
            assert_eq!(nros_clock_get_now_ns(&ros, &mut nanoseconds), NROS_RET_OK);
            assert_eq!(nanoseconds, simulated);
            assert!(rcl_clock_time_started(&ros));

            let mut read_back: i64 = 0;
            assert_eq!(
                nros_get_ros_time_override(&ros, &mut read_back),
                NROS_RET_OK
            );
            assert_eq!(read_back, simulated);

            let mut stamp = nros_time_t::default();
            assert_eq!(nros_clock_get_now(&ros, &mut stamp), NROS_RET_OK);
            assert_eq!(
                stamp,
                nros_time_t {
                    sec: 1234,
                    nanosec: 500_000_000
                }
            );

            // Time does not move while the simulator holds it.
            assert_eq!(nros_clock_get_now_ns(&ros, &mut nanoseconds), NROS_RET_OK);
            assert_eq!(nanoseconds, simulated);

            // A negative override is the "no override" sentinel, so it is
            // refused rather than silently clearing the clock.
            assert_eq!(
                rcl_set_ros_time_override(&ros, -1),
                NROS_RET_INVALID_ARGUMENT
            );
            assert_eq!(
                nros_get_ros_time_override(&ros, &mut read_back),
                NROS_RET_OK
            );
            assert_eq!(read_back, simulated);

            // A system clock is not something `/clock` can drive, and the
            // system clock keeps reading the wall while ROS time is held.
            assert_eq!(rcl_enable_ros_time_override(&system), NROS_RET_NOT_ALLOWED);
            assert_eq!(
                rcl_is_enabled_ros_time_override(&system, &mut enabled),
                NROS_RET_NOT_ALLOWED
            );
            assert_eq!(
                nros_clock_get_now_ns(&system, &mut nanoseconds),
                NROS_RET_OK
            );
            assert!(nanoseconds > simulated);

            // Null clock, null out-parameter, and an uninitialised clock.
            assert_eq!(
                rcl_enable_ros_time_override(core::ptr::null()),
                NROS_RET_INVALID_ARGUMENT
            );
            assert_eq!(
                rcl_is_enabled_ros_time_override(&ros, core::ptr::null_mut()),
                NROS_RET_INVALID_ARGUMENT
            );
            let fresh = nros_clock_get_zero_initialized();
            assert_eq!(rcl_enable_ros_time_override(&fresh), NROS_RET_NOT_INIT);
            assert!(!rcl_clock_time_started(&fresh));

            // Released: the ROS clock reads the wall again.
            assert_eq!(rcl_disable_ros_time_override(&ros), NROS_RET_OK);
            assert_eq!(
                rcl_is_enabled_ros_time_override(&ros, &mut enabled),
                NROS_RET_OK
            );
            assert!(!enabled);
            assert_eq!(nros_clock_get_now_ns(&ros, &mut nanoseconds), NROS_RET_OK);
            assert!(nanoseconds > simulated);
        }
    }
}
